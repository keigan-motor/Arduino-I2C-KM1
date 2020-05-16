@mainpage
# Arduino-I2C-KM1
This library allows an Arduino/Genuino board to control KeiganMotor as slave KM-1 using I2C communication.
ESP32 or ESP8266 (by Espressif) is also available.

- [日本語版の説明はこちら](readme_ja.md)

## Github
- https://github.com/keigan-motor/Arduino-I2C-KM1

## Documentation
- https://docs.keigan-motor.com/apiDocument/Arduino-I2C-KM1/
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

Harness option to use header pin from KeiganMotor I2C port
- https://keiganmotor.myshopify.com/products/i2c


***NOTE***
Please refer to the following about pull-up resister.


### Pullup resisters
Please add external pullup resisters from 1kOhm to 10kOhm as follows. (INPUT_PULLUP is not recommended but may work well especially for Arduino UNO.)
- between SDA and 3.3V // Default SDA = 21
- between SCL and 3.3V // Default SCL = 22

***NOTE***
KeiganMotor uses 3.0V for SDA and SCL lines.
If you use microcontroller its high voltage level is 5V like Arduino UNO,
Please use a logic level converter to protect KeiganMotor and avoid communication error such as the following. 
- https://www.switch-science.com/catalog/1523/

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/levelshifter.jpg?raw=true" width="400">

|Level shifter |Arduino UNO|KeiganMotor_I2C|
|---|---|---|
|HV|5V     |-  |
|HV1 |SDA(21)|-  |
|HV2 |SCL(22)|-  |
|LV|3.3V   |-  |
|LV1 |-      |SDA|
|LV2 |-      |SCL|
|GND  |GND    |GND| 


In case you use PCA9306 module http://akizukidenshi.com/catalog/g/gM-05452/ , You need to cut the lines of Jamper J4 and J5 to disable the pull-up on the board because the drive power of KeiganMotor is not enough and may fail to receive data from KeiganMotor. Just in case, please add external pull-ups more than 4kohm, while it may work without external pull-ups.

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/pca9306_top.jpg?raw=true" width="400">
<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/pca9306_bottom.jpg?raw=true" width="400">

The connection is like this. Connect VREF1 side as KeiganMotor (See PCA9305 datasheet).
|PCA9306 |Arduino UNO|KeiganMotor_I2C|
|---|---|---|
|VREF1|3.3V     |-|
|SDA1 |-|SDA|
|SCL1 |-|SCL|
|VREF2|5V   |-  |
|SDA2|SDA(21)|-|
|SCL2|SCL(22)|SCL|
|GND  |GND    |GND| 


##### M5Stack
You can connect KeiganMotor to it directly.
No need external pullup resisters because it already has external pullup resisters,
and the logic signal level is 3.3V.
（Normal ESP32 and ESP8266 need external pullups)

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/ver2/img/M5Stack_connection.jpg?raw=true" width="400">

|M5Stack |KeiganMotor_I2C|
|---|---|
|SDA(21)|SDA|
|SCL(22)|SCL|
|GND|GND|

***NOTE***
No need to connect 5V pin of KeiganMotor I2C port.

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
motor.i2cSlaveAddress(0x30);
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
|Torque.ino  |Max Torque and position control example|
|ReadMotorMeasurement.ino  |Read Motor Measurement (position, velocity and torque)|
|Dual.ino  |Run two KeiganMotors at the same time|
|Reset.ino  |Reset and save all the registers|
|Wave.ino  |Cosine wave position control using timer|
|PID.ino  |Read the PID parameters|

## History
- ver 2.0.2 Fix a bug to fail to compile when using ESP32 or M5Stack
- ver.2.0.1 Modified explanation of I2C physical connection
- ver.2.0.0 major update

## Author

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## License

[MIT](http://b4b4r07.mit-license.org)
