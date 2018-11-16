
#include "KM1_I2C.h"


#define PI (3.141592653589793)

// Robot Car
#define WHEEL_D (120) // The diameter of the wheels = 120mm
#define RAD_1MM (2 * PI / 120) // radian by 1 millimeter

// KeiganMotor KM-1
// Initialization with the following address.
// NOTE) You should change the I2C address of the right wheel's KeiganMotor previously.
//       See the example "ChangeI2CAddress".
KeiganMotor motor_l(0xA0); // Left wheel 0xA0
KeiganMotor motor_r(0xB0); // Right wheel 0xB0



// Velocity control to give each velocity to both of KeiganMotors. The unit is rpm (rotation per minutes).
// The arguments are signed float.
void run(float rpm_left, float rpm_right) {
  motor_l.runAtVelocityRpm(rpm_left);  // The left wheel moves forward when it rotates by counter clockwise.
  motor_r.runAtVelocityRpm(rpm_right); // The right wheel moves forward when it rotates by clockwise.
}

// Set preset speed to both of KeiganMotors The .unit is rpm (rotation per minutes).
// NOTE) Move commands don't work without this function.
// The arguments are unsigned float.
void speedRpm(float rpm_left, float rpm_right) {
  motor_l.speedRpm(rpm_left);
  motor_r.speedRpm(rpm_right);
}

// Relative position control to give each distance to the wheels. The unit is mm (millimeters).
void move(float d_left, float d_right) {
  motor_l.moveByDistance(d_left * RAD_1MM);  // The left wheel moves forward when it rotates by counter clockwise.
  motor_r.moveByDistance(-d_right * RAD_1MM); // // The left wheel moves forward when it rotates by clockwise.
}


// Disable action
void disable() {
  motor_l.disable();
  motor_r.disable();
}

// Enable action
void enable() {
  motor_l.enable();
  motor_r.enable();
}

// Give LED color to both of KeiganMotors.
void led(uint8_t r_left, uint8_t g_left, uint8_t b_left, uint8_t r_right, uint8_t g_right, uint8_t b_right) {
  motor_l.led(1, r_left, g_left, b_left); // The 1st argument is given by enum (0:OFF, 1:ON_SOLID, 3:ON_FLASH).
  motor_r.led(1, r_right, g_right, b_right);
}


void setup() {

  // Start I2C communication.
  motor_r.begin();
  motor_l.begin();

  // Disable KeiganMotors.
  disable();

  // Set preset speed (rpm).
  motor_r.speedRpm(10);
  motor_l.speedRpm(10);

  // Check LED
  for (int i = 0; i < 5; i++) {
    led(0, 0, 0, 0, 0, 0);
    delay(250);
    led(0, 0, 255, 255, 0, 0);
    delay(250);
  }         
  
  enable();
  led(255, 255, 0, 0, 255, 255);

  // Action: remove comment out and customize to do your action

  // Velocity control
  // Turn right
  run(10, -10);

  // Turn left
  // run(-10, 10);

  // Run forward
  // run(10, 10);

  // Run backward
  // run (-10, -10);

  // Position control
  // speedRpm(10, 10); // preset speed is required to do position control
  
  // Turn right
  // move(100, -100);

  // Turn left
  // move(-100, 100);

  // Run forward
  // move(100, 100);

  // Run backward
  // move(-100, 100);
  
  delay(10000); // wait for 10 seconds
 
  disable(); // deenergize motor


}

void loop() {

}
