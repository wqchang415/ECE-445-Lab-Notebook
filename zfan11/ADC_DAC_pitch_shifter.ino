#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>
#include <vector>
#include <complex>

// Define ADC and DAC pins
#define ADC_PIN 34   // ADC input pin (GPIO34)
#define DAC_CH1 25   // DAC output pin (GPIO25)
#define TRIG_PIN 16  // HC-SR04 trigger pin (GPIO 16)
#define ECHO_PIN 17  // HC-SR04 echo pin (GPIO 17)

// Buffer configuration
#define IO_BUFFER_SIZE 256
#define RING_BUFFER_SIZE 4096
#define SAMPLING_RATE 22050
#define FFT_SIZE 1024
#define HOP 256 // FFT_SIZE / 4
#define FRAMES_PER_BUFFER 256

volatile float global_semitone_shift = 0.0f;

// Double buffer implementation
volatile int inputBuffer_0[IO_BUFFER_SIZE];
volatile int input_head_0 = 0;
volatile int input_tail_0 = 0;
volatile int outputBuffer_0[IO_BUFFER_SIZE];
volatile int output_head_0 = 0;
volatile int output_tail_0 = 0;
volatile int inputBuffer_1[IO_BUFFER_SIZE];
volatile int input_head_1 = 0;
volatile int input_tail_1 = 0;
volatile int outputBuffer_1[IO_BUFFER_SIZE];
volatile int output_head_1 = 0;
volatile int output_tail_1 = 0;
volatile bool input_buffer_selector = false;
volatile bool output_buffer_selector = false;

volatile bool input_block_read_ready = false;
volatile bool output_block_write_ready = false;

// Ring buffer implementation
volatile int ringBuffer[RING_BUFFER_SIZE];
volatile int ringBufferOut[RING_BUFFER_SIZE];
volatile int ring_head = 0;
volatile int ring_tail = 0;

volatile long int stream_counter = 0;
volatile int sensor_counter = 0;
volatile long duration;
volatile float distance = 10;

ArduinoFFT<float> FFT = ArduinoFFT<float>();
float frame_r[FFT_SIZE] = {0};
float frame_i[FFT_SIZE] = {0};
float stft_frame_r[FFT_SIZE/2 + 1] = {0};
float stft_frame_i[FFT_SIZE/2 + 1] = {0};
float ifft_frame_r[FFT_SIZE] = {0};
float ifft_frame_i[FFT_SIZE] = {0};
float last_phase[FFT_SIZE/2 + 1] = {0};
float phase_acc[FFT_SIZE/2 + 1] = {0};

enum SensorState { TRIGGER_OFF, WAITING_ECHO, LOADING };
SensorState state = TRIGGER_OFF;
volatile unsigned long triggerStart = 0;

// Push data to input buffer
void inputBufferPush(int value) {
  if (input_buffer_selector == false) {
    inputBuffer_0[input_head_0] = value;
    input_head_0 = (input_head_0 + 1) % IO_BUFFER_SIZE;
    if (input_head_0 == 0) {
      input_buffer_selector = true;
      input_block_read_ready = true;
      Serial.printf("read successfully_0\n");
    }
  } else {
    inputBuffer_1[input_head_1] = value;
    input_head_1 = (input_head_1 + 1) % IO_BUFFER_SIZE;
    if (input_head_1 == 0) {
      input_buffer_selector = false;
      input_block_read_ready = true;
      Serial.printf("read successfully_1\n");
    }
  }
}

// Pop data from input buffer
int inputBufferPop(bool selector) {
  int value = 0;
  if (selector == false) {
    value = inputBuffer_0[input_tail_0];
    input_tail_0 = (input_tail_0 + 1) % IO_BUFFER_SIZE;
  } else {
    value = inputBuffer_1[input_tail_1];
    input_tail_1 = (input_tail_1 + 1) % IO_BUFFER_SIZE;
  }
  return value;
}

// Push data to output buffer
void outputBufferPush(int value, bool selector) {
  if (selector == false) {
    outputBuffer_0[output_head_0] = value;
    output_head_0 = (output_head_0 + 1) % IO_BUFFER_SIZE;
  } else {
    outputBuffer_1[output_head_1] = value;
    output_head_1 = (output_head_1 + 1) % IO_BUFFER_SIZE;
  }
}

// Pop data from output buffer
int outputBufferPop() {
  int value;
  if (output_buffer_selector == false) {
    value = outputBuffer_0[output_tail_0];
    output_tail_0 = (output_tail_0 + 1) % IO_BUFFER_SIZE;
    if (output_tail_0 == 0) {
      output_buffer_selector = true;
    }
  } else {
    value = outputBuffer_1[output_tail_1];
    output_tail_1 = (output_tail_1 + 1) % IO_BUFFER_SIZE;
    if (output_tail_1 == 0) {
      output_buffer_selector = false;
    }
  }
  return value;
}

// Push to ring buffer
void ringBufferPush(int value) {
  ringBuffer[ring_head] = value;
  ringBufferOut[ring_head] = 0;
  ring_head = (ring_head + 1) % RING_BUFFER_SIZE;
}

// Pop from ring buffer
int ringBufferPop() {
  int value = ringBufferOut[ring_tail];
  ring_tail = (ring_tail + 1) % RING_BUFFER_SIZE;
  return value;
}

// Window functions
float hanningWindow[FFT_SIZE];
float synthesisWindow[FFT_SIZE];

// Generate Hanning window
void generateHanningWindow() {
  for (int i = 0; i < FFT_SIZE; i++) {
    hanningWindow[i] = 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1)));
  }
  Serial.printf("Generate hanning window successfull\n");
}

// Generate synthesis window
void generateSynthesisWindow() {
  for (int i = 0; i < FFT_SIZE; i++) {
    synthesisWindow[i] = (2.0f / 3.0f) * hanningWindow[i];
  }
  Serial.printf("Generate synthesis window successfull\n");
}

// ADC-DAC task: Read ADC and write to DAC at 22050Hz
void adcDacTask(void * parameter) {
  analogReadResolution(8);
  const unsigned long samplePeriod = 1000000UL / SAMPLING_RATE;
  unsigned long nextSampleTime = micros();

  while(1) {
    while (micros() < nextSampleTime) {
      int x = 0;
    }
    nextSampleTime += samplePeriod;

    int adcValue = analogRead(ADC_PIN);
    inputBufferPush(adcValue - 48);
    int value = outputBufferPop();
    dacWrite(DAC_CH1, value + 128);
  }
}

// Pitch shifting processing task
void pitchShiftTask(void * parameter) {
  bool read_selector = false;
  bool write_selector = false;
  int value_to_push_to_ring_buffer = 0;
  int value_to_pop_from_ring_buffer = 0;

  generateHanningWindow();
  generateSynthesisWindow();

  while(1) {
    while(input_block_read_ready){
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      duration = pulseIn(ECHO_PIN, HIGH, 2900);
      distance = duration * 0.0344 / 2;
      Serial.println(distance);
      
      if (distance > 20) distance = 20;
      if (distance > 1 && distance <= 10) { 
        global_semitone_shift = map(distance, 0, 10, -1.5f, 0.0f);
      } else if (distance > 10 || distance <= 1) {
        global_semitone_shift = 0.0f;
      }

      input_block_read_ready = false;
      read_selector = input_buffer_selector;
      if (read_selector == true) {
        for (int i = 0; i < IO_BUFFER_SIZE; i++) {
          value_to_push_to_ring_buffer = inputBufferPop(false);
          ringBufferPush(value_to_push_to_ring_buffer);
        }
      } else {
        for (int i = 0; i < IO_BUFFER_SIZE; i++) {
          value_to_push_to_ring_buffer = inputBufferPop(true);
          ringBufferPush(value_to_push_to_ring_buffer);
        }
      }
      stream_counter = stream_counter + 1;
      
      if (stream_counter >= 4) {
        for (int i = 0; i < FFT_SIZE; i++) {
          frame_r[i] = 0;
          frame_i[i] = 0;
          ifft_frame_r[i] = 0;
          ifft_frame_i[i] = 0;
        }
        for (int i = 0; i < FFT_SIZE/2 + 1; i++) {
          stft_frame_r[i] = 0;
          stft_frame_i[i] = 0;
        }
        
        float pitch_factor = pow(2.0f, -global_semitone_shift / 12.0f);
        float r = 1.0f / pitch_factor;
        
        // STFT processing
        for (int i = 0; i < FFT_SIZE; i++) {
          frame_r[i] = (ringBuffer[(((ring_head - FFT_SIZE + i) % RING_BUFFER_SIZE) + RING_BUFFER_SIZE) % RING_BUFFER_SIZE]);
          frame_r[i] *= hanningWindow[i];
        }
        
        FFT.dcRemoval(frame_r, FFT_SIZE);
        FFT.compute(frame_r, frame_i, FFT_SIZE, FFT_FORWARD);
        
        // Phase vocoder processing
        for (int k = 0; k <= FFT_SIZE/2; ++k) {
            float mag = sqrt((frame_r[k] * frame_r[k]) + (frame_i[k] * frame_i[k]));
            float phase = atan2(frame_i[k], frame_r[k]);
            
            // Calculate phase difference
            float delta_phase = phase - last_phase[k];
            last_phase[k] = phase;
            
            // Remove expected phase
            float expected = 2 * M_PI * HOP * k / FFT_SIZE;
            delta_phase -= expected;
            delta_phase = fmod(delta_phase + M_PI, 2*M_PI) - M_PI;

            // Accumulate phase with scaling
            phase_acc[k] += expected * r + delta_phase;
            phase_acc[k] = fmod(phase_acc[k] + M_PI, 2*M_PI) - M_PI;
            
            stft_frame_r[k] = mag * cos(phase_acc[k]);
            stft_frame_i[k] = mag * sin(phase_acc[k]);
        }

        // IFFT processing
        for (int k = 0; k <= FFT_SIZE/2; ++k) {
            ifft_frame_r[k] = stft_frame_r[k];
            ifft_frame_i[k] = stft_frame_i[k];
        }
        for (int k = FFT_SIZE/2+1; k < FFT_SIZE; ++k) {
            ifft_frame_r[k] = stft_frame_r[FFT_SIZE - k];
            ifft_frame_i[k] = - stft_frame_i[FFT_SIZE - k];
        }
        FFT.compute(ifft_frame_r, ifft_frame_i, FFT_SIZE, FFT_REVERSE);

        // Overlap-add
        for (int i = 0; i < FFT_SIZE; ++i) {
            ringBufferOut[(((ring_head - FFT_SIZE + i) % RING_BUFFER_SIZE) + RING_BUFFER_SIZE) % RING_BUFFER_SIZE] += std::round(ifft_frame_r[i] * synthesisWindow[i]);
        }

        // Output processing
        if (1) {
          write_selector = output_buffer_selector;
          if(write_selector == true) {
            Serial.printf("write successfully_0\n");
            for (int i = 0; i < IO_BUFFER_SIZE; i++) {
              value_to_pop_from_ring_buffer = ringBufferPop();
              outputBufferPush(value_to_pop_from_ring_buffer, false);
            }
          } else {
            Serial.printf("write successfully_1\n");
            for (int i = 0; i < IO_BUFFER_SIZE; i++) {
              value_to_pop_from_ring_buffer = ringBufferPop();
              outputBufferPush(value_to_pop_from_ring_buffer, true);
            }
          }
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("5 seconds elapsed!");
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  disableCore0WDT();
  disableCore1WDT();

  xTaskCreatePinnedToCore(
    adcDacTask,
    "ADC DAC Task",
    32768,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    pitchShiftTask,
    "Pitch Shift Task",
    98304,
    NULL,
    3,
    NULL,
    1
  );
}

void loop() {
  // Main loop remains empty
}