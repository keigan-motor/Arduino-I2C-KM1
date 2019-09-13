# Arduino-I2C-KM1
KeiganMotor KM-1 control library using I2C communication
NOTE) You can use this library also with ESP32 micro-controller series.

# Documents web site
- https://document.keigan-motor.com

## Description
This library allows an Arduino/Genuino board to control KeiganMotor as slave KM-1 using I2C communication.
ESP32 or ESP8266 (by Espressif) is also available.

## Physical Connection
Please refer to the following page.
- Japanese: https://document.keigan-motor.com/software_dev/ports_on_wire

Communication speed should be 100kHz or 400kHz.

***NOTE***
Please refer to the following about pull-up resister.
We recommend 

### Arduino/Genuino
If control is unstable, please add external pullup resisters from 1kOhm to 10kOhm between (SDA and Vdd) and ) (SDL and Vdd) and add the following lines in setup() function to disable internal pullup resisters..
```arduino
pinMode(SDA, INPUT); // For Arduino Uno, SDA = 4
pinMode(SCL, INPUT);  // For Arduino Uno, SCL = 5
```
You may not need to add pullup resisters  because Arduino Library "Wire" enables internal pullup automatically, but we recommend to use external pull-up especially for 400kHz communication.

### ESP32 or ESP8266
Please add external pullup resisters from 1kOhm to 10kOhm as follows. (INPUT_PULLUP is not recommended.)
- between SDA and 5V(or 3.3V) // Default SDA = 21
- between SCL and 5V(or 3.3V) // Default SCL = 22

### M5 Stack
M5 Stack do not need external pullup resisters because it already has external pullup resisters.

## Basic
Including the library and initialization are required to control KeiganMotor.
### Include library
```arduino
#include "KM1_I2C.h"

```
### Initialize with I2C slave address
#### The default I2C address of KM-1 is "0x20" 
```arduino
KeiganMotor motor(0x20);
```
***NOTE***
"0xA0" will cause the same result because I2C address is available only for 7bit.

### Start I2C communication
```arduino
motor.begin();
```
***NOTE***
This line is not needed from version 1.1.0 because I2C communication is available when initializing KeiganMotor.

## Examples
### See "examples" folder
### Run
```arduino
motor.enable();
motor.speedRpm(10);
motor.runForward();
delay(5000);
motor.runReverse();
delay(5000);
motor.stop();
```

### Change I2C Address
```arduino
motor.i2cSlaveAddress(0xB0);
motor.saveAllRegisters();
delay(2000);
motor.reboot();
```

## Requirement

- Arduino UNO / Mega / Mega 2560
- KeiganMotor KM-1 series

## Installation

    $ git clone https://github.com/keigan-motor/Arduino-I2C-KM1

## Zip

    https://github.com/keigan-motor/Arduino-I2C-KM1/archive/master.zip

## Author

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## License

[MIT](http://b4b4r07.mit-license.org)
