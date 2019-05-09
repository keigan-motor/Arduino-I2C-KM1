#include "KM1_I2C.h"

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0xA0)

KeiganMotor m1(0xA0); 
KeiganMotor m2(0xB0); 

void setup() {

  m1.led(1, 255, 0, 0); // Set LED color solid:red
  m2.led(1, 0, 255, 0); // Set LED color solid:green
  
  m1.enable(); // Enable Motor Action
  m2.enable();
  m1.speedRpm(10);
  m2.speedRpm(10);


}

void loop() {

  m1.moveByDistanceDegree(30);
  m2.moveByDistanceDegree(-30);
    
  delay(3000);

  m1.moveByDistanceDegree(-30);
  m2.moveByDistanceDegree(30);
  
  delay(3000);

}
