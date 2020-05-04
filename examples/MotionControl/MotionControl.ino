/**
   @file MotionControl.ino
   @brief Cosine wave position control example using timer.
   @note Default I2C Slave Address is 0x20 (KeiganMotor KM-1 Series)
   @date 2020/5/1
   @author Takashi Tokuda (Keigan Inc.)
*/

#include <KM1_I2C.h>
#include <TypeUtility.h>
#include <math.h>
#include <MsTimer2.h> /** Please install MsTimer2 library*/

static int addr = 0x20;
KeiganMotor m(addr);

static bool isWorking = false; /** true when motion control is working */
static int read_error_cnt = 0; // read motor measurement error cuont
static int move_error_cnt = 0; // move command error count
#define AMP_DEGREE (30) // Amplitude of cosine wave [degree]

void measurement() {
  
  if (!isWorking) return;

  if (m.readMotorMeasurement()) {
    Serial.print("[");
    Serial.print(addr, HEX);
    Serial.println("] ");
    Serial.print("Pos [deg]: ");
    Serial.println(m.degree);
    Serial.print("Vel [rpm]: ");
    Serial.println(m.rpm);
    Serial.print("Trq [N*m]: ");
    Serial.println(m.torque);
    Serial.print("read error: ");
    Serial.println(read_error_cnt);
    Serial.print("move error: ");
    Serial.println(move_error_cnt);
    Serial.print("seconds: ");
    Serial.println(millis() / 1000);
    Serial.println();

  } else {
    Serial.println("Error!!");
    read_error_cnt ++;
    Serial.print("error code: ");
    Serial.println(m.getError().code);
    Serial.print("read error: ");
    Serial.println(read_error_cnt);

  }
}

void timerHandler() {

  static int count = 0;
  static int i = 0;
  static int stamp = millis();

  count ++;

  interrupts(); // Necessary to allow interrupts in this handler

  bool success = m.moveToPositionDegree(AMP_DEGREE * (cos(i * 2 * M_PI / 100)));
  measurement();
  //  if (!success) {
  //    move_error_cnt++;
  //    Serial.print("move error: ");
  //    Serial.println(m.getError().code);
  //  }

  Serial.print("success: ");
  Serial.println(success);
  i ++;

  Serial.println(millis() - stamp); // Time stamp

  if (count > 1000) {
    Serial.println("hello");
    MsTimer2::stop();
    isWorking = false;
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Read Motor Measurement");

  m.response(false);
  m.startMotorMeasurement();
  m.enable();
  m.speedRpm(1000);
  m.presetPosition(0);

  m.curveType(0); // Turn off trapezoidal curve.
  m.moveToPosition(0);

  delay(3000);

  // Start timer
  MsTimer2::set(5, timerHandler);
  MsTimer2::start();

  isWorking = true;

}



void loop() {



}
