#include "KM1_I2C.h"

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0xA0)



KeiganMotor motor(0xA0); // Start I2C communication

void setup() {

  motor.led(1, 255, 0, 0); // Set LED color solid:red
  motor.enable(); // Enable Motor Action
  motor.acc(100);
  motor.dec(100);
  motor.speedRpm(100);
 
  for (int i = 0; i<12; i++){
    motor.moveByDistanceDegree(30);
    delay(2000);
  }

  motor.led(1, 0, 255, 0); // Set LED color solid:green

}

void loop() {
  

}