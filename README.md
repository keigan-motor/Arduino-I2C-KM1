@mainpage
# Arduino-I2C-KM1
This library allows an Arduino/Genuino board to control KeiganMotor as slave KM-1 using I2C communication.
ESP32 or ESP8266 (by Espressif) is also available.

## Github
- https://github.com/keigan-motor/Arduino-I2C-KM1

## Documentation
- https://docs.keigan-motor.com

## Requirement
- Arduino UNO / Mega / Mega 2560, ESP8266 / ESP32 series and M5Stack
- KeiganMotor KM-1 series
 - Firmware version more than 2.36 required

## Installation

    $ git clone https://github.com/keigan-motor/Arduino-I2C-KM1

You can also install it from library manager. Keyword is "keigan".

### Zip

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
KeiganMotor uses 3.0V for SDA and SCL lines.
If you use microcontroller its high voltage level is 5V like Arduino UNO,
Please use a logic level converter to avoid communication error such as the following. 
- http://akizukidenshi.com/catalog/g/gM-05452/


##### M5Stack and Raspberry Pi
You can connect KeiganMotor to them directly.
They do not need external pullup resisters because it already has external pullup resisters,
and the high signal level is 3.3V.


## Basic
Including the library and initialization are required to control KeiganMotor.

### Include library
```arduino
#include "KM1_I2C.h"
```
### Initialize with I2C slave address
#### The default I2C address of KM-1 is "0x20" 
This will start I2C communication automatically.
```arduino
KeiganMotor motor(0x20); // It includes wire.begin();
```
***NOTE***
"0xA0" will cause the same result because I2C address is available only for 7bit.

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

## Examples
### See "examples" folder

|Example  |Desc.  |
|---|---|
|Scan.ino  |Scan all I2C devices and identify KeiganMotor|
|ChangeI2CAddress.ino  |Change I2C slave address  |
|MoveTo.ino  |Move to absolute position. (Position control)|
|MoveBy.ino  |Move by distance (move to relative position) (Position control)|
|ReadMotorMeasurement.ino  |Read Motor Measurement (position, velocity and torque)|
|Dual.ino  |Run two KeiganMotors at the same time|
|Reset.ino  |Reset and save all the registers|
|MotionControl.ino  |Cosine wave position control using timer|




## Author

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## License

[MIT](http://b4b4r07.mit-license.org)
