#include "KM1_I2C.h"

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0xA0)



KeiganMotor motor(0xA0); 

void setup() {
  
  motor.begin(); // Start I2C communication
  motor.led(1, 255, 0, 0); // Set LED color solid:red
  motor.enable(); // Enable Motor Action

  for (int i = 0; i++; i<36){
    motor.speed(5*(i+1));
    motor.moveByDistanceDegree(10);
    motor.wait(1000);
  }

  motor.led(1, 0, 255, 0); // Set LED color solid:green

}

void loop() {

}
