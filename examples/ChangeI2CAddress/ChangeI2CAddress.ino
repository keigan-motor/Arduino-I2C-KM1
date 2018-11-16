#include "KM1_I2C.h"

KeiganMotor motor(0xA0);

void setup() {
  
  motor.begin();
  
  motor.i2cSlaveAddress(0xB0);
  motor.saveAllRegisters();
  delay(2000);
  motor.reboot();
    
}

void loop() {
  // put your main code here, to run repeatedly:

}
