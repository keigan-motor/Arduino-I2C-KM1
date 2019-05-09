#include "KM1_I2C.h"

// NOTE) Default I2C Slave Address is 0xA0 (KeiganMotor KM-1 Series)
KeiganMotor motor(0xA0);

void setup() {
  
  motor.i2cSlaveAddress(0xB0); // Change I2C Address from 0xA0 to 0xB0
  motor.saveAllRegisters();
  delay(2000);
  motor.reboot();
    
}

void loop() {
  // put your main code here, to run repeatedly:

}
