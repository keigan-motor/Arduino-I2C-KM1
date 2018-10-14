#include "KM1_I2C.h"

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0xA0)



KeiganMotor motor(0xA0); 

void setup() {
  
  motor.begin(); // Start I2C communication
  motor.led(1, 255, 0, 0); // Set LED color solid:red
  motor.speedRpm(10); // Set speed
  motor.presetPosition(0); // Set the current position as zero
  motor.enable(); // Enable Motor Action
  motor.moveToPositionDegree(90); // Move to position 90 degree
  motor.wait(3000); // Wait for 3000 msec until next command
  motor.led(1, 0, 255, 0); // Set LED color solid:green
  motor.moveToPositionDegree(0); // Move to position 0 
  
}

void loop() {

}
