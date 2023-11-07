# Sega P-ras 3 software tools

This repository contains software for controlling the three serial devices found on
the Sega P-ras 3 arcade machine.

- SEGA 837-15396/610-0955 NFC reader/writer.
- Futaba GP1232A02A vacuum fluorescent display (VFD) (Sega part no. 200-6275).
- Sega LED control board.

The python script can be run as an executable or imported and used as a module.
The executalbe only provides a subset of the features of the module. The module
requires PySerial.

There is also a compiled program written in C that runs on Linux/Windows/MacOS.
It provides the same features as the python executable.
