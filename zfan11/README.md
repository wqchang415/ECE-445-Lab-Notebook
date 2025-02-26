# Pitch Shift

This is a simple C++ program that implements pitch shifting using a phase vocoder algorithm. The program reads an input WAV file, applies pitch shifting based on a semitone value, and writes the processed audio to an output WAV file.

## Usage

Run the program with:

./pitch_shift input.wav output.wav [semitone number]

- input.wav: Path to the input WAV file.
- output.wav: Path for the output WAV file.
- [semitone number]: The number of semitones to shift the pitch. A positive value shifts the pitch upward, while a negative value shifts it downward. You can enter floating-point numbers.

## Dependencies

This program requires the following library:
- libsndfile (http://www.mega-nerd.com/libsndfile/) for reading and writing WAV files.

Additionally, a C++ compiler with C++17 support (or later) is required.

## Compilation

1. Install libsndfile:

   On Ubuntu or Debian-based systems, install libsndfile using:
   sudo apt-get install libsndfile1-dev

   On macOS, you can install it using Homebrew:
   brew install libsndfile

2. Compile the Code:

   Use a C++ compiler such as g++ to compile the program. For example:
   g++ -std=c++11 -o pitch_shift pitch_shift.cpp -lsndfile

   Adjust the command as necessary based on your compiler and system setup.

## Running the Program

After compiling, run the program as described in the usage section. For example, to shift the pitch up by 3 semitones:
./pitch_shift input.wav output.wav 3

## Notes

- Ensure the input WAV file is in a supported format (e.g., 16-bit PCM).
- The ultimate goal of this program is to implement a real-time pitch shifter for the ECE445 course's Digital Guitar Pitch Shifter. Further development will be carried out afterward.

Happy Pitch Shifting!