# Arduino-I2C-KM1
KeiganMotor KM-1 control library using I2C communication

# Documents web site
 (https://document.keigan-motor.com)

## Description
This library allows an Arduino/Genuino board to control KeiganMotor KM-1 using I2C communication.

## Connection
Please refer to the following page.
- Japanese: https://document.keigan-motor.com/software_dev/ports_on_wire
- English: https://en.document.keigan-motor.com/development-environment/comm_on_wire.html

***NOTE***
You should add pullup resisters as small as possible between SDA and Vdd and between SDL and Vdd.

## Basic
Including the library and initialization are required to control KeiganMotor.
### Include library
```arduino
#include "KM1_I2C.h"

```
### Initialize with I2C slave address
#### The default I2C address of KM-1 is "0xA0"
```arduino
KeiganMotor motor(0xA0);
```
### Start I2C communication
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
motor.wait(5000); // Wait Queue
motor.runReverse();
motor.wait(5000);
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