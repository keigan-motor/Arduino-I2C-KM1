# Arduino-I2C-KM1
This library allows an Arduino/Genuino board to control KeiganMotor as slave KM-1 using I2C communication.
ESP32 or ESP8266 (by Espressif) is also available.

# Documents web site
- https://document.keigan-motor.com

## Requirement

- Arduino UNO / Mega / Mega 2560 or ESP8266 / ESP32 series
- KeiganMotor KM-1 series

## Installation

    $ git clone https://github.com/keigan-motor/Arduino-I2C-KM1

You can also install it from library manager. Keyword is "keigan".

## Zip

    https://github.com/keigan-motor/Arduino-I2C-KM1/archive/master.zip
    
## Physical Connection
Please refer to the following page.
- Japanese: https://document.keigan-motor.com/software_dev/ports_on_wire

Communication speed should be 100kHz or 400kHz.

***NOTE***
Please refer to the following about pull-up resister.


#### Pullup resisters
Please add external pullup resisters from 1kOhm to 10kOhm as follows. (INPUT_PULLUP is not recommended.)
- between SDA and 3.3V // Default SDA = 21
- between SCL and 3.3V // Default SCL = 22

***NOTE***
KeiganMotor uses 3.3V for SDA and SCL lines.

##### M5 Stack and Raspberry Pi
They do not need external pullup resisters because it already has external pullup resisters.


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

***NOTE***
Please use delay(ms) instead of motor.wait(ms).
The latter can cause restart of KeiganMotor if sending continuously.


### Change I2C Address
```arduino
motor.i2cSlaveAddress(0xB0);
motor.saveAllRegisters();
delay(2000);
motor.reboot();
```

## Author

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## License

[MIT](http://b4b4r07.mit-license.org)
