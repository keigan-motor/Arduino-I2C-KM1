#include "KM1_I2C.h"

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0x20)



KeiganMotor motor(0x20); // Start I2C communication

void setup() {

  motor.led(1, 255, 0, 0); // Set LED color solid:red
  motor.enable(); // Enable Motor Action
  motor.speedRpm(10); // Set Speed
  motor.runForward(); // Run forward
  motor.wait(5000); // Wait for 5000 msec until next command
  motor.led(1, 0, 255, 0); // Set LED color solid:green
  motor.runReverse(); // Run reverse
  motor.wait(5000); // Wait for 5000 msec until next command
  motor.led(1, 0, 0, 255); // Set LED color solid:blue
  motor.stop(); // Stop
  
}

void loop() {

}
