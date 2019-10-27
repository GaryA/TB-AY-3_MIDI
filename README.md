# TB-AY-3_MIDI
MIDI/Arduino Controlled 8-Bit Sound Generator (AY-3-8910)

This is re-factored code for Erik Oostveen's TB-AY-3 project: https://www.instructables.com/id/MIDIArduino-Controlled-8-Bit-Sound-Generator-AY-3-/
It requires the following Arduino libraries to be installed:
Encoder library: https://github.com/PaulStoffregen/Encoder
Button library: https://github.com/tigoe/Button
Button library: https://github.com/tigoe/Button

The code is written for Arduino Nano, it will not run as-is on Arduino Uno as it uses pin A6 which is not available on the 28-pin DIP ATMEGA328.
