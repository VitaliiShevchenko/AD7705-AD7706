# Arduino AVR Boards connect with AD7705/AD7706 chip

[![Check Arduino status](https://github.com/arduino/ArduinoCore-avr/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/arduino/ArduinoCore-avr/actions/workflows/check-arduino.yml)
[![Compile Examples status](https://github.com/arduino/ArduinoCore-avr/actions/workflows/compile-platform-examples.yml/badge.svg)](https://github.com/arduino/ArduinoCore-avr/actions/workflows/compile-platform-examples.yml)

This software(driver) are using to read the analogue signals through the 16-bit ADC with two full-duplex or three pseudo differential channels. 

## Usage

1. Select the folder where your project files are located `cd your_project`.
2. Create a new folder `$ mkdir libraries`.
3. Go to the library `$ cd libraries` .
4. Upload the files to this folder `$ git clone https://github.com/VitaliiShevchenko/AD7705-AD7706`.
5. After successful upload a folder  name `AD7705-AD7706` was created .
6. In the Arduino IDE tab on the `Sketch`, select `Include Library` and select our `AD7705-AD7706` library under `library contributed`.
7. In your files, use the `#include <ModuleAD7705.h>` directive.

### More detailed information about using any function in this project by the request

