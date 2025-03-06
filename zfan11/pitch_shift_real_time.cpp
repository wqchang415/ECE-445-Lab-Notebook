#include <iostream>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <complex>
#include <portaudio.h>
#include <limits>

using namespace std;

// Audio settings
const int SAMPLE_RATE = 44100;           // Standard sample rate
const int FRAMES_PER_BUFFER = 256;       // How many samples per buffer
const int FFT_SIZE = 1024;               // Size for Fourier transforms
const int HOP = FFT_SIZE / 4;            // Overlap between frames
const double INITIAL_DELAY_SECONDS = 0.1; // Initial buffer delay

// Global variables for pitch control
atomic<float> global_semitone_shift(0.0f); // How much to shift pitch (in # of semitones)
atomic<bool> processingRunning{true};     // Flag to stop processing

// Structure for audio device info
struct CallbackData {
    int inputChannel;     // Input channel to use
    int inputChannels;    // Total input channels
    int outputChannels;   // Total output channels
};

// Thread-safe ring buffer for audio data
class RingBuffer {
public:
    RingBuffer(size_t capacity) : buffer(capacity), capacity(capacity) {}
    
    // Write data to buffer
    bool write(const float* data, size_t samples) {
        if (samples > capacity - write_count.load()) return false;
        for (size_t i = 0; i < samples; ++i)
            buffer[(write_idx + i) % capacity] = data[i];
        write_idx = (write_idx + samples) % capacity;
        write_count.fetch_add(samples);
        return true;
    }
    
    // Read data from buffer
    bool read(float* data, size_t samples) {
        if (samples > write_count.load()) return false;
        for (size_t i = 0; i < samples; ++i)
            data[i] = buffer[(read_idx + i) % capacity];
        read_idx = (read_idx + samples) % capacity;
        write_count.fetch_sub(samples);
        return true;
    }

private:
    vector<float> buffer;       // Storage for samples
    size_t write_idx = 0;       // Where to write next
    size_t read_idx = 0;        // Where to read next
    atomic<size_t> write_count{0}; // How many samples in buffer
    size_t capacity;            // Max buffer size
};

// Create buffers for audio processing, input and output
RingBuffer inputBuffer(FFT_SIZE * 4);
RingBuffer outputBuffer(FFT_SIZE * 4);

// FFT
void fft(vector<complex<float>>& a) {
    size_t n = a.size();
    if (n <= 1) return; // Base case
    
    // Split into even and odd
    vector<complex<float>> even(n/2), odd(n/2);
    for (size_t i = 0; i < n/2; i++) {
        even[i] = a[2*i];
        odd[i] = a[2*i+1];
    }
    
    // Recurse
    fft(even);
    fft(odd);
    
    // Combine results
    for (size_t k = 0; k < n/2; k++) {
        complex<float> t = polar(1.0f, -2 * float(M_PI) * k / n) * odd[k];
        a[k] = even[k] + t;
        a[k+n/2] = even[k] - t;
    }
}

// Inverse FFT
void ifft(vector<complex<float>>& a) {
    // Conjugate to reverse FFT
    for (auto& x : a) x = conj(x);
    fft(a);
    // Conjugate again and scale
    for (auto& x : a) x = conj(x) / float(a.size());
}

// Create Hann window
vector<float> hann_window(int N) {
    vector<float> win(N);
    for (int i = 0; i < N; i++)
        win[i] = 0.5f * (1 - cos(2 * float(M_PI) * i / (N - 1)));
    return win;
}

// Phase vocoder for pitch shifting
class PitchShifter {
public:
    PitchShifter() : fft_size(FFT_SIZE), hop(HOP),
          win(hann_window(fft_size)),           // Analysis window
          synthesis_win(hann_window(fft_size)), // Synthesis window
          last_phase(fft_size/2+1, 0.0f),       // Lsat phase tracking
          phase_acc(fft_size/2+1, 0.0f) {       // Phase accumulation
        // Adjust synthesis window
        for (auto& w : synthesis_win) w *= 2.0f/3.0f;
    }

    void process(const float* input, size_t n, vector<float>& output) {
        // Get current pitch shift amount
        float current_shift = global_semitone_shift.load();
        float pitch_factor = pow(2.0f, -current_shift / 12.0f);
        float r = 1.0f / pitch_factor;

        // Add new input to buffer
        input_buffer.insert(input_buffer.end(), input, input + n);

        // Process while enough data
        while (input_buffer.size() >= fft_size) {
            // Step 1: Get frame and apply Hann
            vector<float> frame(fft_size);
            copy(input_buffer.begin(), input_buffer.begin() + fft_size, frame.begin());
            for (int i = 0; i < fft_size; ++i)
                frame[i] *= win[i];

            // Step 2: Compute FFT
            vector<complex<float>> fft_frame(fft_size);
            for (int i = 0; i < fft_size; ++i)
                fft_frame[i] = complex<float>(frame[i], 0.0f);
            fft(fft_frame);

            // Step 3: Phase manipulation
            vector<complex<float>> stft_frame(fft_size/2 + 1);
            for (int k = 0; k <= fft_size/2; ++k) {
                float mag = abs(fft_frame[k]);
                float phase = arg(fft_frame[k]);
                
                // Calculate phase difference
                float delta_phase = phase - last_phase[k];
                last_phase[k] = phase;

                // Remove expected phase
                float expected = 2 * M_PI * hop * k / fft_size;
                delta_phase -= expected;
                delta_phase = fmod(delta_phase + M_PI, 2*M_PI) - M_PI;

                // Accumulate phase with scaling
                phase_acc[k] += expected * r + delta_phase;
                stft_frame[k] = polar(mag, phase_acc[k]);
            }

            // Step 4: Inverse FFT
            vector<complex<float>> ifft_frame(fft_size);
            for (int k = 0; k <= fft_size/2; ++k)
                ifft_frame[k] = stft_frame[k];
            for (int k = fft_size/2+1; k < fft_size; ++k)
                ifft_frame[k] = conj(stft_frame[fft_size - k]);

            ifft(ifft_frame);

            // Step 5: Apply synthesis window
            vector<float> out_frame(fft_size);
            for (int i = 0; i < fft_size; ++i)
                out_frame[i] = ifft_frame[i].real() * synthesis_win[i];

            // Step 6: Overlap-add to output buffer
            for (int i = 0; i < fft_size; ++i) {
                if (i < output_buffer.size())
                    output_buffer[i] += out_frame[i];
                else
                    output_buffer.push_back(out_frame[i]);
            }

            // Move input buffer forward
            input_buffer.erase(input_buffer.begin(), input_buffer.begin() + hop);

            // Handle output hopping
            int output_hop = max(1, static_cast<int>(hop * r));
            size_t available = output_buffer.size() / output_hop * output_hop;
            if (available > 0) {
                output.insert(output.end(), output_buffer.begin(), output_buffer.begin() + output_hop);
                output_buffer.erase(output_buffer.begin(), output_buffer.begin() + output_hop);
                output_buffer.resize(output_buffer.size() + output_hop, 0.0f);
            }
        }
    }

private:
    const int fft_size, hop;    // FFT size and hop
    vector<float> input_buffer; // Input samples
    vector<float> output_buffer; // Processed samples
    vector<float> win, synthesis_win; // Window functions
    vector<float> last_phase;   // Previous phase values
    vector<float> phase_acc;    // Accumulated phase
};

// Called when input audio is available
static int inputCallback(const void* inputData, void*,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo*,
                         PaStreamCallbackFlags,
                         void* userData) {
    const CallbackData* data = static_cast<CallbackData*>(userData);
    const float* in = static_cast<const float*>(inputData);
    
    // Convert to mono from selected channel
    vector<float> mono(framesPerBuffer);
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        mono[i] = in[i * data->inputChannels + data->inputChannel];
    
    // Write to input buffer
    inputBuffer.write(mono.data(), framesPerBuffer);
    return paContinue;
}

// Called when output is needed
static int outputCallback(const void*, void* outputData,
                          unsigned long framesPerBuffer,
                          const PaStreamCallbackTimeInfo*,
                          PaStreamCallbackFlags,
                          void* userData) {
    const CallbackData* data = static_cast<CallbackData*>(userData);
    float* out = static_cast<float*>(outputData);
    
    // Read from output buffer
    vector<float> mono(framesPerBuffer);
    bool success = outputBuffer.read(mono.data(), framesPerBuffer);
    if (!success) // If no data, output silence
        memset(out, 0, framesPerBuffer * data->outputChannels * sizeof(float));
    
    // Copy to all output channels
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        for (int c = 0; c < data->outputChannels; ++c)
            out[i * data->outputChannels + c] = mono[i];
    
    return paContinue;
}

// Main processing thread
void processingThreadFunc() {
    PitchShifter shifter;
    vector<float> inputBlock(FRAMES_PER_BUFFER);
    vector<float> outputBlock;

    while (processingRunning) {
        if (inputBuffer.read(inputBlock.data(), FRAMES_PER_BUFFER)) {
            outputBlock.clear();
            shifter.process(inputBlock.data(), FRAMES_PER_BUFFER, outputBlock);
            
            if (!outputBlock.empty()) {
                // Simple resampling to match buffer size
                vector<float> resampled(FRAMES_PER_BUFFER);
                float ratio = outputBlock.size() / (float)FRAMES_PER_BUFFER;
                for (int i = 0; i < FRAMES_PER_BUFFER; ++i) {
                    float pos = i * ratio;
                    int idx = min(int(pos), (int)outputBlock.size()-1);
                    float frac = pos - idx;
                    resampled[i] = outputBlock[idx] * (1-frac) + 
                                  (idx+1 < outputBlock.size() ? outputBlock[idx+1] * frac : 0);
                }
                outputBuffer.write(resampled.data(), FRAMES_PER_BUFFER);
            }
        }
        // Prevent CPU overuse
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

int main() {
    // Initialize PortAudio
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        cerr << "PortAudio init failed: " << Pa_GetErrorText(err) << endl;
        return 1;
    }

    // List input devices
    vector<pair<int, const PaDeviceInfo*>> inputDevices;
    int numDevices = Pa_GetDeviceCount();
    
    cout << "=== Input Devices ===\n";
    for (int i = 0; i < numDevices; i++) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info->maxInputChannels > 0) {
            cout << "[" << inputDevices.size() << "] "
                 << info->name << " (Channels: " 
                 << info->maxInputChannels << ")\n";
            inputDevices.emplace_back(i, info);
        }
    }

    if (inputDevices.empty()) {
        cerr << "No input devices!\n";
        Pa_Terminate();
        return 1;
    }

    // Get user input for device selection
    int deviceChoice = -1;
    cout << "\nChoose input device (0-" << inputDevices.size()-1 << "): ";
    cin >> deviceChoice;
    
    if (deviceChoice < 0 || deviceChoice >= inputDevices.size()) {
        cerr << "Bad device choice\n";
        Pa_Terminate();
        return 1;
    }

    // Setup selected device
    const auto& [inputDeviceId, inputDeviceInfo] = inputDevices[deviceChoice];
    cout << "Using: " << inputDeviceInfo->name 
         << " (Channels: " << inputDeviceInfo->maxInputChannels << ")\n";

    // Select audio channel
    int selectedChannel = -1;
    cout << "Pick channel (0-" << inputDeviceInfo->maxInputChannels-1 << "): ";
    cin >> selectedChannel;
    
    if (selectedChannel < 0 || selectedChannel >= inputDeviceInfo->maxInputChannels) {
        cerr << "Bad channel\n";
        Pa_Terminate();
        return 1;
    }

    // Get initial pitch shift
    float initial_shift = 0.0f;
    cout << "Start shift (semitones, +up/-down): ";
    cin >> initial_shift;
    global_semitone_shift.store(initial_shift);
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    // Setup output device
    const int outputDeviceId = Pa_GetDefaultOutputDevice();
    const PaDeviceInfo* outputDeviceInfo = Pa_GetDeviceInfo(outputDeviceId);

    // Create callback data
    CallbackData callbackData{
        selectedChannel,
        inputDeviceInfo->maxInputChannels,
        outputDeviceInfo->maxOutputChannels
    };

    // Configure input stream
    PaStreamParameters inputParams;
    inputParams.device = inputDeviceId;
    inputParams.channelCount = inputDeviceInfo->maxInputChannels;
    inputParams.sampleFormat = paFloat32;
    inputParams.suggestedLatency = inputDeviceInfo->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    // Configure output stream
    PaStreamParameters outputParams;
    outputParams.device = outputDeviceId;
    outputParams.channelCount = outputDeviceInfo->maxOutputChannels;
    outputParams.sampleFormat = paFloat32;
    outputParams.suggestedLatency = outputDeviceInfo->defaultLowOutputLatency;
    outputParams.hostApiSpecificStreamInfo = nullptr;

    // Open audio streams
    PaStream* inputStream;
    if ((err = Pa_OpenStream(&inputStream,
                            &inputParams,
                            nullptr,
                            SAMPLE_RATE,
                            FRAMES_PER_BUFFER,
                            paNoFlag,
                            inputCallback,
                            &callbackData)) != paNoError) {
        cerr << "Input stream error: " << Pa_GetErrorText(err) << endl;
        Pa_Terminate();
        return 1;
    }

    PaStream* outputStream;
    if ((err = Pa_OpenStream(&outputStream,
                            nullptr,
                            &outputParams,
                            SAMPLE_RATE,
                            FRAMES_PER_BUFFER,
                            paNoFlag,
                            outputCallback,
                            &callbackData)) != paNoError) {
        cerr << "Output stream error: " << Pa_GetErrorText(err) << endl;
        Pa_CloseStream(inputStream);
        Pa_Terminate();
        return 1;
    }

    // Start audio streams
    if ((err = Pa_StartStream(inputStream)) != paNoError) {
        cerr << "Can't start input: " << Pa_GetErrorText(err) << endl;
        Pa_CloseStream(inputStream);
        Pa_CloseStream(outputStream);
        Pa_Terminate();
        return 1;
    }

    if ((err = Pa_StartStream(outputStream)) != paNoError) {
        cerr << "Can't start output: " << Pa_GetErrorText(err) << endl;
        Pa_StopStream(inputStream);
        Pa_CloseStream(inputStream);
        Pa_CloseStream(outputStream);
        Pa_Terminate();
        return 1;
    }

    // Start processing thread
    thread processingThread(processingThreadFunc);

    // User input handling
    atomic<bool> inputRunning{true};
    thread inputThread([&inputRunning](){
        string line;
        while (inputRunning) {
            cout << "Enter new semitone shift (Enter to quit): ";
            getline(cin, line);
            if (line.empty()) {
                inputRunning = false;
                processingRunning = false;
                break;
            }
            try {
                float new_shift = stof(line);
                global_semitone_shift.store(new_shift);
                cout << "New shift: " << new_shift << endl;
            } catch (const exception& e) {
                cout << "Bad input, try number\n";
            }
        }
    });

    cout << "\nRunning... Enter shifts or press Enter to exit\n";

    // Wait for threads
    processingThread.join();
    inputThread.join();
    
    // Cleanup
    Pa_StopStream(inputStream);
    Pa_StopStream(outputStream);
    Pa_CloseStream(inputStream);
    Pa_CloseStream(outputStream);
    Pa_Terminate();

    return 0;
}