# ledian: microcontroller

This directory contains the project for the microcontroller, an ESP32-C6.

## Building

Enter in the espressif sdk environment and then:

```
idf.py build
idf.py flash
stty -F /dev/ttyACM0 115200
screen /dev/ttyACM0 115200
```
