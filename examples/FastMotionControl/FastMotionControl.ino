/**
   @file ReadMotorMeasurement.ino
   @brief Move by distance (move to relative position) example. (Position control).
   @note Default I2C Slave Address is 0x20 (KeiganMotor KM-1 Series)
   @date 2020/5/1
   @author Takashi Tokuda (Keigan Inc.)
*/

#include <KM1_I2C.h>
#include <TypeUtility.h>
#include <math.h>
//#include <TimerOne.h> // Timer Library 
#include <MsTimer2.h>

static int addr = 0x20;
KeiganMotor m(addr);

static int read_error_cnt = 0; // read motor measurement error
#define AMP_DEGREE (30)

void timerHandler() {

  Serial.println("a");
  
  static int count = 0;
  static int i = 0;
  static int stamp = millis();
  
  count ++;
  interrupts(); 
  //m.led(1, 255, 0, 0);
  m.moveToPositionDegree(AMP_DEGREE * (cos(i * 2 * M_PI / 100)), false); // Send command without response request (false as the 2nd argument)
  i ++;
  Serial.println(millis() - stamp);
  
  if(count > 1000){
    Serial.println("hello");
    MsTimer2::stop();
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Read Motor Measurement");

  m.enable();
  m.speedRpm(1000);
  m.presetPosition(0);

  m.curveType(0); // Turn off trapezoidal curve.
  m.moveToPosition(0);

  delay(3000);

//  Timer1.initialize(50000); //マイクロ秒単位で設定
//  Timer1.attachInterrupt(timerHandler);
  
  //Timer1.start();
    //1ms毎にtimerFire関数を呼び出す
  MsTimer2::set(10, timerHandler);
 
  //タイマー開始
  MsTimer2::start();

}


void loop() {

return;
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
    Serial.print("seconds: ");
    Serial.println(millis() / 1000);
    Serial.println();
  } else {
    Serial.println("Error!!");
    Serial.print("read error: ");
    Serial.println(read_error_cnt);    
  }

}
