#include <KM1_I2C.h>
#include <TypeUtility.h>

// This example send "Read Motor Measurement Command" to KeiganMotor (I2C Address: 0x20)
// and show Position, Velocity and Torque on Serial Monitor.

// #1 Connect default I2C port to KeiganMotor KM-1 
// #2 Initialize KeiganMotor with I2C slave address (default: 0x20)

KeiganMotor m(0x20); 

void setup() {

  Serial.begin(115200);
  Serial.println("Started");
}

void handleReceivedData(uint8_t *data, uint8_t len) {
    
  uint8_t address = data[0];
  uint8_t cmd = data[1];
  
  if (cmd == 0xB4 && len == 16) {
    // Motor Measurement
    float position = float_big_decode(&data[2]);
    float velocity = float_big_decode(&data[6]);
    float torque = float_big_decode(&data[10]);

    Serial.print("[");
    Serial.print(address, HEX);
    Serial.print("] ");
    Serial.print("Pos, Vel, Trq = ");
    Serial.print(position);
    Serial.print(", ");
    Serial.print(velocity);
    Serial.print(", ");
    Serial.println(torque);

  } 
  
}


void loop() {
  
  m.readMotorMeasurement();

  Wire.requestFrom(0x20, 16); // (Address, bytes number)
  
  uint8_t len = 0;
  uint8_t receivedData[255];

  while (Wire.available()) { // slave may send less than requested
    receivedData[len] = Wire.read();
    len ++;
  }
  
  handleReceivedData(receivedData, len);

  delay(100); 
  
}
