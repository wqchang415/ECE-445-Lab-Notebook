#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <sndfile.h>
#include <algorithm>
#include <cstring>
#include <complex>

// Define audio data container type
using AudioData = std::vector<float>;

AudioData pitch_shift(const AudioData& input, 
                     double semitone_shift, 
                     const SF_INFO& info);

// -------------------------------------------------------------------
// Below are helper functions required for the phase vocoder

// Recursive FFT (in-place), requires n to be a power of 2
void fft(std::vector<std::complex<float>>& a) {
    size_t n = a.size();
    if(n <= 1)
        return;
    std::vector<std::complex<float>> even(n/2), odd(n/2);
    for (size_t i = 0; i < n/2; i++) {
        even[i] = a[2*i];
        odd[i] = a[2*i+1];
    }
    fft(even);
    fft(odd);
    for (size_t k = 0; k < n/2; k++) {
        std::complex<float> t = std::polar(1.0f, -2 * float(M_PI) * k / n) * odd[k];
        a[k] = even[k] + t;
        a[k+n/2] = even[k] - t;
    }
}

// Inverse FFT: implemented using FFT
void ifft(std::vector<std::complex<float>>& a) {
    for (auto & x : a)
        x = std::conj(x);
    fft(a);
    for (auto & x : a)
        x = std::conj(x) / float(a.size());
}

// Generate Hann window
std::vector<float> hann_window(int N) {
    std::vector<float> win(N);
    for (int i = 0; i < N; i++) {
        win[i] = 0.5f * (1 - std::cos(2 * float(M_PI) * i / (N - 1)));
    }
    return win;
}

// Compute Short-Time Fourier Transform (STFT)
// Input signal x, FFT size fft_size, using Hann window, hop size = fft_size/4
std::vector<std::vector<std::complex<float>>> stft(const std::vector<float>& x, int fft_size) {
    int hop = fft_size / 4;
    int num_frames = (x.size() - fft_size) / hop + 1;
    int num_bins = fft_size / 2 + 1;
    // Dimensions of X: num_bins x num_frames
    std::vector<std::vector<std::complex<float>>> X(num_bins, std::vector<std::complex<float>>(num_frames));
    std::vector<float> win = hann_window(fft_size);
    
    for (int frame = 0; frame < num_frames; frame++) {
        int start = frame * hop;
        std::vector<std::complex<float>> frame_data(fft_size);
        for (int i = 0; i < fft_size; i++) {
            frame_data[i] = std::complex<float>( x[start + i] * win[i], 0.0f );
        }
        fft(frame_data);
        // Save the first num_bins frequency bins (discarding the conjugate symmetric part)
        for (int k = 0; k < num_bins; k++) {
            X[k][frame] = frame_data[k];
        }
    }
    return X;
}

// Inverse STFT, using overlap-add to reconstruct the signal.
// The synthesis window is a Hann window multiplied by 2/3 (according to MATLAB code)
std::vector<float> istft(const std::vector<std::vector<std::complex<float>>>& X, int fft_size, int hop) {
    int num_bins = X.size(); // fft_size/2+1
    int num_frames = X[0].size();
    int output_length = fft_size + (num_frames - 1) * hop;
    std::vector<float> y(output_length, 0.0f);
    std::vector<float> win = hann_window(fft_size);
    // As in MATLAB code, synthesis window is multiplied by 2/3
    for (int i = 0; i < fft_size; i++) {
        win[i] *= (2.0f/3.0f);
    }
    
    for (int frame = 0; frame < num_frames; frame++) {
        std::vector<std::complex<float>> fft_frame(fft_size);
        // Reconstruct full FFT symmetric spectrum
        for (int k = 0; k < num_bins; k++) {
            fft_frame[k] = X[k][frame];
        }
        for (int k = num_bins; k < fft_size; k++) {
            fft_frame[k] = std::conj(fft_frame[fft_size - k]);
        }
        ifft(fft_frame);
        int start = frame * hop;
        for (int i = 0; i < fft_size; i++) {
            y[start + i] += fft_frame[i].real() * win[i];
        }
    }
    return y;
}

// Implement phase vocoder resampling
// X: STFT matrix (dimensions: num_bins x num_frames)
// t: new time coordinate vector (real, can be fractional; index corresponds to columns of X)
// hop and fft_size are used to compute the expected phase increments
std::vector<std::vector<std::complex<float>>> pvsample(
    const std::vector<std::vector<std::complex<float>>>& X,
    const std::vector<float>& t,
    int hop,
    int fft_size)
{
    int num_bins = X.size(); // fft_size/2+1
    int num_frames = X[0].size();
    std::vector<std::vector<std::complex<float>>> Y(num_bins, std::vector<std::complex<float>>(t.size()));
    float N = fft_size;
    // Compute expected phase increment for each frequency bin:
    // for k >= 1, dphi = 2π * hop * k / fft_size
    std::vector<float> dphi(num_bins, 0.0f);
    dphi[0] = 0.0f;
    for (int k = 1; k < num_bins; k++) {
        dphi[k] = 2 * float(M_PI) * hop * k / N;
    }
    // Initialize phase accumulator with the phase of each bin from the first frame of X
    std::vector<float> phase(num_bins, 0.0f);
    for (int k = 0; k < num_bins; k++) {
        phase[k] = std::arg(X[k][0]);
    }
    // For each new time position
    for (size_t col = 0; col < t.size(); col++) {
        float ti = t[col];
        int idx = static_cast<int>(std::floor(ti));
        float tf = ti - idx;
        int idx2 = (idx + 1 < num_frames) ? idx + 1 : idx;
        for (int k = 0; k < num_bins; k++) {
            // Magnitude linear interpolation
            float mag1 = std::abs(X[k][idx]);
            float mag2 = std::abs(X[k][idx2]);
            float mag = (1 - tf) * mag1 + tf * mag2;
            // Calculate phase difference, and subtract expected phase increment
            float phase1 = std::arg(X[k][idx]);
            float phase2 = std::arg(X[k][idx2]);
            float dphase = phase2 - phase1 - dphi[k];
            // Adjust dphase to be in the range -pi to pi
            while (dphase < -float(M_PI)) dphase += 2 * float(M_PI);
            while (dphase >  float(M_PI)) dphase -= 2 * float(M_PI);
            // Accumulate phase
            phase[k] += dphi[k] + dphase;
            // Reconstruct bin value
            Y[k][col] = std::polar(mag, phase[k]);
        }
    }
    return Y;
}

// pvoc: calls stft -> pvsample -> istft to perform time-scale modification
std::vector<float> pvoc(const std::vector<float>& input, float r, int fft_size) {
    int hop = fft_size / 4;
    auto X = stft(input, fft_size);
    int num_frames = X[0].size();
    // Construct new time coordinate vector from 0 to (num_frames-2) with step size r
    std::vector<float> t;
    for (float ti = 0; ti <= num_frames - 2; ti += r)
        t.push_back(ti);
    auto X2 = pvsample(X, t, hop, fft_size);
    auto y = istft(X2, fft_size, hop);
    return y;
}

// Simple linear interpolation resampling function
std::vector<float> resample(const std::vector<float>& input, float ratio) {
    size_t new_length = static_cast<size_t>(std::round(input.size() * ratio));
    std::vector<float> output(new_length);
    for (size_t i = 0; i < new_length; i++) {
        float orig_index = i / ratio;
        size_t idx = static_cast<size_t>(std::floor(orig_index));
        float frac = orig_index - idx;
        float sample = 0.0f;
        if (idx + 1 < input.size()) {
            sample = (1 - frac) * input[idx] + frac * input[idx + 1];
        } else {
            sample = input[idx];
        }
        output[i] = sample;
    }
    return output;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <input.wav> <output.wav> <semitone_number>\n";
        return 1;
    }

    try {
        const char* input_file = argv[1];
        const char* output_file = argv[2];
        double semitone = std::stod(argv[3]);

        // Open input file
        SF_INFO in_info;
        std::memset(&in_info, 0, sizeof(in_info));
        SNDFILE* in_sndfile = sf_open(input_file, SFM_READ, &in_info);
        if (!in_sndfile) {
            throw std::runtime_error(sf_strerror(nullptr));
        }

        // Validate audio format
        if (sf_format_check(&in_info) == 0) {
            sf_close(in_sndfile);
            throw std::runtime_error("Unsupported audio format");
        }

        // Read audio data
        AudioData input_data(in_info.frames * in_info.channels);
        sf_count_t frames_read = sf_readf_float(in_sndfile, input_data.data(), in_info.frames);
        sf_close(in_sndfile);

        if (frames_read != in_info.frames) {
            throw std::runtime_error("Failed to read complete audio data");
        }

        // Process pitch shifting
        AudioData output_data = pitch_shift(input_data, semitone, in_info);

        // Prepare output format
        SF_INFO out_info;
        std::memset(&out_info, 0, sizeof(out_info));
        out_info.samplerate = in_info.samplerate;
        out_info.channels = in_info.channels;
        out_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;  // 16-bit PCM format

        // Open output file
        SNDFILE* out_sndfile = sf_open(output_file, SFM_WRITE, &out_info);
        if (!out_sndfile) {
            throw std::runtime_error(sf_strerror(nullptr));
        }

        // Write audio data
        const sf_count_t output_frames = output_data.size() / out_info.channels;
        sf_count_t frames_written = sf_writef_float(out_sndfile, output_data.data(), output_frames);
        sf_close(out_sndfile);

        if (frames_written != output_frames) {
            throw std::runtime_error("Incomplete audio data write");
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

// -------------------------------------------------------------------
// Pitch_shift function:
// For each channel, first apply phase vocoder to time-stretch/compress the signal (ratio r = 1/pitch_factor),
// then resample back to the original length, achieving pitch shift upward or downward.
AudioData pitch_shift(const AudioData& input,
    double semitone_shift,
    const SF_INFO& info) {
    AudioData output = input; // Final result (multichannel interleaved)
    int channels = info.channels;
    size_t total_samples = input.size();
    size_t frames = total_samples / channels;
    // Compute pitch scaling factor: for example, a positive semitone_shift means pitch shift upward,
    // corresponding to frequency multiplication factor: pitch_factor = 2^(semitone_shift/12)
    float pitch_factor = std::pow(2.0f, semitone_shift / 12.0f);
    // Time-scale modification factor: r = 1 / pitch_factor
    float r = 1.0f / pitch_factor;
    int fft_size = 1024; // For example, 1024-point FFT (about 60ms window length @16kHz)

    // Process each channel separately
    for (int ch = 0; ch < channels; ch++) {
        std::vector<float> channel_data(frames);
        // Deinterleave
        for (size_t i = 0; i < frames; i++) {
            channel_data[i] = input[i * channels + ch];
        }
        // First apply phase vocoder for time stretching; note: if pitch_factor > 1 (pitch shifted upward), then r < 1,
        // in which case the pvoc output signal duration is about original length / r (i.e., lengthened)
        auto pvoc_output = pvoc(channel_data, r, fft_size);
        // Then, resample to compress the output back to the original duration, with resampling ratio equal to r
        auto resampled = resample(pvoc_output, r);
        // Adjust length to match the original frame count (truncate if too long, zero-pad if too short)
        if (resampled.size() > frames)
            resampled.resize(frames);
        else if (resampled.size() < frames)
            resampled.resize(frames, 0.0f);
        // Reinterleave back into the output
        for (size_t i = 0; i < frames; i++) {
            output[i * channels + ch] = resampled[i];
        }
    }
    return output;
}

