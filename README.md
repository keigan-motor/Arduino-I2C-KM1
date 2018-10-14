# Arduino-I2C-KM1
KeiganMotor KM-1 control library using I2C communication

## Description
This library allows an Arduino/Genuino board to control KeiganMotor KM-1 using I2C communication.

## Basic
### Include library
```arduino
#include "KM1_I2C.h"

```
### Initialize with I2C slave address
#### The default I2C address of KM-1 is "0xA0"
```arduino
KeiganMotor motor(0xA0);
```
### Start I2C
```arduino
motor.begin();
```

## Examples
### See "examples" folder
### Run
```arduino
motor.enable();
motor.speedRpm(10);
motor.runForward();
```

## Requirement

- Arduino UNO / Mega / Mega 2560
- KeiganMotor KM-1 series

## Installation

    $ git clone https://github.com/keigan-motor/Arduino-I2C-KM1
   
***NOTE***

You should add pullup resisters as small as possible between SDA and Vdd and between SDL and Vdd.

## Author

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## License

[MIT](http://b4b4r07.mit-license.org)