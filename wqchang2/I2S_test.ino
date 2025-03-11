#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_SAMPLE_RATE     (1000)
#define I2S_SAMPLE_BITS     I2S_BITS_PER_SAMPLE_16BIT  // Correct bits per sample
#define PIN_LRC             25
#define PIN_BCLK            26
#define PIN_DOUT            22

int count = 0;

void setup() 
{
  // Start the serial communication for printing
  Serial.begin(115200);

  // Set up I2S configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),  // Cast to i2s_mode_t type
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_SAMPLE_BITS,                   // Use defined I2S_SAMPLE_BITS
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64
  };

  // Set up I2S pins
  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_BCLK,
    .ws_io_num = PIN_LRC,
    .data_out_num = PIN_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Initialize I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

int16_t GenerateSineWave()
{
    double rad = 2 * M_PI * 1000 * count++ / I2S_SAMPLE_RATE;
    int16_t sineVal = 32767 * sin(rad);

    // Print sine wave value to serial monitor
    Serial.print("Sine value: ");
    Serial.println(sineVal);

    return sineVal;
}

void loop()
{
  int16_t sample = GenerateSineWave();
  
  // Send two samples for stereo output
  size_t bytes_written;
  i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
}
