# ledian: KiCad projects

This directory contains KiCad projects for PCBs that Ledian uses.
Projects were created with KiCad 7 with no dependencies from external libraries.

## ledian-2.0-small

Prototype of Ledian 2.0, small panel with just 40 LEDs used to create a proof of concept.

## ledian-support-board

Support board for the microcontroller ESP32-C6. The boards mounts the developing board for ESP32-C6 made by Waveshare.
The board has also
 - 3 ADCs to read all the analog signals from temperature/current sensors of PCBs and power supplies
 - a digital temperature/humidity/pressure sensor
 - power good input signals from power supplies
 - voltage level shifter 3.3V -> 5V for LEDs data signals
