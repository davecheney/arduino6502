# Arduino6502

This is my Retrochallenge 2017/04 entry, a software only 6502 microprocessor system emulator for Arduino platforms.

[Read more](https://dave.cheney.net/2017/04/05/retrochallenge-201704-6502-on-arduino)

# Installation

Assuming you have compatible hardware (see below). Installation and setup instructions are

1. Check out this repository and open `arduino6502.ino` in the Arduino IDE.
2. Upload the sketch to your Arduino.

That's it. Read on for usage instructions.

# Usage

This simulator runs over the Arduino serial port at 9600 baud. You will need to use a terminal emulator to access the simulator.

## Linux instructions

I recommend `screen(1)` as the terminal emulator, sample usage:

  screen /dev/tty.ACM0 9600

To exit the screen, press `Ctrl-a Z`.

## OS X instructions

I recommend `screen(1)` as the terminal emulator, sample usage:

  screen /dev/tty.usbmodem1421 9600

To exit the screen, press `Ctrl-a Z`.

_note_: the exact device name may be different on your system.

## Windows instructions

TODO: Tera Term or PuTTY I guess. Anyone want to send a PR for Windows users.

# Supported hardware

Currently _only_ the Arduino Mega 2560 is supported. Support for Arduino Uno boards is planned, 

# Speed

On 8 bit Atmel platforms the performance is roughly 1/4 of the speed of a real 1Mhz 6502 system.

# License

This project is licensed under a BSD "simplified" 2 clause licence.

Portions of this project (`common.hpp`, `cpu.hpp`, `cpu.cpp`) are copywrite Andrea Orru, see [LICENSE.LaiNES](LICENSE.LaiNES).

This project contains ROM images from Vince Briel's [Replica 1](http://www.brielcomputers.com/wordpress/?cat=17) project, including Applesoft lite BASIC and the original Woz monitor.
