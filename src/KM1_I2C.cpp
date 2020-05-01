#include "KM1_I2C.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

#define KM_I2C_TX_MAX_LEN 20
#define KM_I2C_RX_MAX_LEN 30

// Wait requestFrom after sending data
#define I2C_WRITE_DELAY_MS 10
#define I2C_READ_DELAY_MS 5

// I2C Data Buffer
static uint8_t tx_buf[KM_I2C_TX_MAX_LEN];
static uint8_t rx_buf[KM_I2C_RX_MAX_LEN];



/*
 **@brief Write Data to KeiganMotor

        [0]          [1][2]           [3]...[n-3]		 [n-2][n-1]
   uint8_t command	uint16_t id		uint8_t *value		 uint16_t CRC
        0~0xFF	     0～0xFFFF      depends on command    0～0xFFFF
*/

/*
 **@brief Read response from KeiganMotor
 
          [0]               [0]              [1][2]         	[3]		        [4]...[n-3]		    [n-2][n-1]
 uint8_t i2c_address  uint8_t tx_type	   uint16_t id		uint8_t command 	uint8_t *value		 uint16_t CRC
       0x01~0x7E        0x00~0xE0x40	     0～0xFFFF         0~0xFF   		depends on command   0～0xFFFF
*/



static bool crc16_ready = false;

KeiganMotor::KeiganMotor(uint8_t address, int clock = 1000000)
{
  init(address, clock);
  begin();
  if (!crc16_ready)
  {
    crc16_init();
  }
}

void KeiganMotor::init(uint8_t address, int clock = 1000000)
{
  _address = address;
  _commandID = 0;
  Wire.setClock(clock);
}

void KeiganMotor::begin()
{
  Wire.begin();
}

uint16_t KeiganMotor::appendID(uint8_t *data)
{
  uint16_big_encode(_commandID, data);
  uint16_t cmd = _commandID;

  if (_commandID == 0xFFFF)
  {
    _commandID = 0;
  }
  else
  {
    _commandID++;
  }
  return cmd;
}

error_t KeiganMotor::getError(void)
{
  return error;
}

bool KeiganMotor::writeFloat(uint8_t cmd, float val, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(val, data);
  return write(cmd, data, sizeof(float), response);
}

bool KeiganMotor::write(uint8_t command, uint8_t *value, uint8_t value_len, bool response = true)
{
  uint8_t len = value_len + 5;
  tx_buf[0] = command;
  uint16_t id = appendID(&tx_buf[1]);

  if (value_len > 0)
  {
    memcpy(&tx_buf[3], value, value_len);
  }

  append_crc16(tx_buf, len - 2);
  Wire.beginTransmission(_address);
  Wire.write(tx_buf, len);
  Wire.endTransmission();

  if(!response) return true;
  
  //print_hexdump(tx_buf, len);
  Serial.println("not reach");
  
  delay(I2C_WRITE_DELAY_MS);

  Wire.requestFrom((int)_address, RECV_DATA_ERROR_LEN);

  uint8_t cnt = 0;
  uint8_t rx_buf[RECV_DATA_ERROR_LEN];

  while (Wire.available())
  { // slave may send less than requested

    rx_buf[cnt] = Wire.read();
    cnt++;
  }

  if (rx_buf[1] == RECV_DATA_ERROR && cnt == RECV_DATA_ERROR_LEN)
  {
    error.cmd = rx_buf[4];
    error.id = uint16_big_decode(&rx_buf[2]);
    error.code = uint32_big_decode(&rx_buf[5]);
    error.info = uint32_big_decode(&rx_buf[9]);

    if (id != error.id)
    {
      error.code = ERROR_CODE_ID_NOT_MATCHED;
    }

    if (crc16(rx_buf, cnt) == 0)
    {
      if (error.code == 0)
        return true; // return true in this case
    }
    else
    {
      error.code = ERROR_CODE_CRC_INVALID;
    }
  }
  else
  {
    error.code = ERROR_CODE_INVALID_DATA;
  }

  return false;
}

bool KeiganMotor::readRegister(uint8_t reg, uint8_t *value, uint8_t value_len)
{
  tx_buf[0] = CMD_REG_READ_REGISTER;
  appendID(&tx_buf[1]);
  tx_buf[3] = reg;

  Wire.beginTransmission(_address);
  Wire.write(tx_buf, CMD_REG_READ_REGISTER_LEN);
  Wire.endTransmission(true);

  delay(I2C_READ_DELAY_MS);

  uint8_t len = value_len + 7;

  Wire.requestFrom((int)_address, (int)len);

  uint8_t cnt = 0;

  while (Wire.available())
  { // slave may send less than requested
    rx_buf[cnt] = Wire.read();
    cnt++;
  }
  // Serial.println(cnt);

  if (crc16(rx_buf, cnt) == 0)
  {
    if (rx_buf[1] == RECV_DATA_READ && cnt == len)
    {
      memcpy(value, &rx_buf[5], value_len);
      return true;
    }
    else
    {
      error.code = ERROR_CODE_INVALID_DATA;
    }
  } else {
    error.code = ERROR_CODE_CRC_INVALID;
  }
  memset(value, 0, value_len);
  return false;

}

status_t KeiganMotor::readStatus(void)
{
  uint8_t value[4];
  memset(value, 0, sizeof(value));

  status.isValid = readRegister(CMD_READ_STATUS, value, 4);

  if(status.isValid){
    status.isCheckSumEnabled = value[0] & 0x80;
    status.isIMUMeasNotifyEnabled = value[0] & 0x08;
    status.isMotorMeasNotifyEnabled = value[0] & 0x04;
    status.isQueueRunning = value[0] & 0x02;
    status.flashState = value[1];
    status.motorControlMode = value[2];

  } 
  return status;
}

bool KeiganMotor::readMotorMeasurement(void)
{
  tx_buf[0] = CMD_READ_MOTOR_MEASUREMENT;
  appendID(&tx_buf[1]);
  uint8_t len = CMD_READ_MOTOR_MEASUREMENT_LEN;

  append_crc16(tx_buf, len - 2);
  Wire.beginTransmission(_address);
  Wire.write(tx_buf, len);
  Wire.endTransmission(true);

  delay(I2C_READ_DELAY_MS);

  Wire.requestFrom((int)_address, max(RECV_DATA_MOTOR_MEAS_LEN, RECV_DATA_ERROR_LEN));

  uint8_t cnt = 0;

  while (Wire.available())
  {
    rx_buf[cnt] = Wire.read();
    cnt++;
  }

  measurement = {false, 0, 0, 0};

  if (crc16(rx_buf, cnt) == 0)
  {
    if (rx_buf[1] == RECV_DATA_MOTOR_MEAS && cnt == RECV_DATA_MOTOR_MEAS_LEN)
    {
      measurement.isValid = true;
      measurement.position = float_big_decode(&rx_buf[2]);
      measurement.velocity = float_big_decode(&rx_buf[6]);
      measurement.torque = float_big_decode(&rx_buf[10]);

      position = measurement.position;
      velocity = measurement.velocity;
      torque = measurement.torque;
      degree = RADIANS_TO_DEGREES(position);
      rpm = RADPERSEC_TO_RPM(velocity);

      return true;
    }
    else if (rx_buf[1] == RECV_DATA_ERROR && cnt == RECV_DATA_ERROR_LEN)
    {
      error.cmd = rx_buf[4];
      error.id = uint16_big_decode(&rx_buf[2]);
      error.code = uint32_big_decode(&rx_buf[5]);
      error.info = uint32_big_decode(&rx_buf[9]);
    }
    else
    {
      error.code = ERROR_CODE_INVALID_DATA;
    }
  }
  else
  {
    error.code = ERROR_CODE_CRC_INVALID;
  }

  measurement.isValid = false;
  return false;
}

bool KeiganMotor::readIMUMeasurement(void)
{
  tx_buf[0] = CMD_READ_IMU_MEASUREMENT;
  appendID(&tx_buf[1]);
  uint8_t len = CMD_READ_IMU_MEASUREMENT_LEN;
  append_crc16(tx_buf, len - 2);

  Wire.beginTransmission(_address);
  Wire.write(tx_buf, len);
  Wire.endTransmission(true);

  delay(I2C_READ_DELAY_MS);

  Wire.requestFrom((int)_address, RECV_DATA_IMU_MEAS_LEN);

  uint8_t cnt = 0;

  while (Wire.available())
  {
    rx_buf[cnt] = Wire.read();
    cnt++;
  }

  imu = {false, 0, 0, 0, 0, 0, 0, 0}; // response from KeiganMotor

  if (crc16(rx_buf, cnt) == 0)
  {
    if (rx_buf[1] == RECV_DATA_IMU_MEAS && cnt == RECV_DATA_IMU_MEAS_LEN)
    {
      imu.isValid = true;
      imu.accelX = int16_big_decode(&rx_buf[2]);
      imu.accelY = int16_big_decode(&rx_buf[4]);
      imu.accelZ = int16_big_decode(&rx_buf[6]);
      imu.temp = int16_big_decode(&rx_buf[8]);
      imu.gyroX = int16_big_decode(&rx_buf[10]);
      imu.gyroY = int16_big_decode(&rx_buf[12]);
      imu.gyroZ = int16_big_decode(&rx_buf[14]);

      position = measurement.position;
      velocity = measurement.velocity;
      torque = measurement.torque;
      degree = RADIANS_TO_DEGREES(position);
      rpm = RADPERSEC_TO_RPM(velocity);

      return true;
    }
    else if (rx_buf[1] == RECV_DATA_ERROR && cnt == RECV_DATA_ERROR_LEN)
    {
      error.cmd = rx_buf[4];
      error.id = uint16_big_decode(&rx_buf[2]);
      error.code = uint32_big_decode(&rx_buf[5]);
      error.info = uint32_big_decode(&rx_buf[9]);
    }
    else
    {
      error.code = ERROR_CODE_INVALID_DATA;
    }
  }
  else
  {
    error.code = ERROR_CODE_CRC_INVALID;
  }
  imu.isValid = false;
  return false;
}

float KeiganMotor::readFloat(uint8_t reg)
{
  uint8_t value[sizeof(float)];
  if(readRegister(reg, value, sizeof(float))){
    return float_big_decode(value);
  } else {
    return DUMMY_DATA_INDICATE_ERROR;
  }
}

uint8_t KeiganMotor::readByte(uint8_t reg)
{
  uint8_t value;
  if(readRegister(reg, &value, sizeof(uint8_t))){
    return value;
  }
  return DUMMY_DATA_INDICATE_ERROR;
}

uint32_t KeiganMotor::readUint32(uint8_t reg)
{
  uint8_t value[sizeof(uint32_t)];
  if(readRegister(reg, value, sizeof(uint32_t))){
    return uint32_big_decode(value);
  }
  return DUMMY_DATA_INDICATE_ERROR;
}

// Set motion control curve type 0:None, 1:Trapezoid
bool KeiganMotor::curveType(CurveType type, bool response = true)
{
  uint8_t data[] = {type};
  return write(CMD_REG_CURVE_TYPE, data, sizeof(data), response);
}

// Set acceleration [rad/s^2]
bool KeiganMotor::acc(float value, bool response = true)
{
  return writeFloat(CMD_REG_ACC, value, response);
}

// Set deceleration [rad/s^2]
bool KeiganMotor::dec(float value, bool response = true)
{
  return writeFloat(CMD_REG_DEC, value, response);
}

// Set max torque [N*m]
bool KeiganMotor::maxTorque(float value, bool response = true)
{
  return writeFloat(CMD_REG_MAX_TORQUE, value, response);
}

// Enable CheckSum
bool KeiganMotor::enableCheckSum(bool isEnabled, bool response = true)
{
  uint8_t data[] = {isEnabled};
  return write(CMD_OTHERS_ENABLE_CHECK_SUM, data, sizeof(data), response);
}

// PID Parameters
bool KeiganMotor::qCurrentP(float value, bool response = true)
{
  return writeFloat(CMD_REG_Q_CURRENT_P, value, response);
}

bool KeiganMotor::qCurrentI(float value, bool response = true)
{
  return writeFloat(CMD_REG_Q_CURRENT_I, value, response);
}

bool KeiganMotor::qCurrentD(float value, bool response = true)
{
  return writeFloat(CMD_REG_Q_CURRENT_D, value, response);
}

bool KeiganMotor::speedP(float value, bool response = true)
{
  return writeFloat(CMD_REG_SPEED_P, value, response);
}

bool KeiganMotor::speedI(float value, bool response = true)
{
  return writeFloat(CMD_REG_SPEED_I, value, response);
}

bool KeiganMotor::speedD(float value, bool response = true)
{
  return writeFloat(CMD_REG_SPEED_D, value, response);
}

bool KeiganMotor::positionP(float value, bool response = true)
{
  return writeFloat(CMD_REG_POSITION_P, value, response);
}

bool KeiganMotor::positionI(float value, bool response = true)
{
  return writeFloat(CMD_REG_POSITION_I, value, response);
}

bool KeiganMotor::positionD(float value, bool response = true)
{
  return writeFloat(CMD_REG_POSITION_D, value, response);
}

bool KeiganMotor::positionIDThreshold(float value, bool response = true)
{
  return writeFloat(CMD_REG_POS_ID_THRESHOLD, value, response);
}

bool KeiganMotor::resetPID(bool response = true)
{
  return write(CMD_REG_RESET_PID, NULL, 0, response);
}

// Set interface port by bit flag
// If set to 0, the interface port is ignored.
//
//  bit7   | bit6   | bit5 | bit4 | bit3     | bit2     | bit1    | bit0
//  button | UART2  | -    | I2C  | UART(USB)| microbit | Linkage | BLE
//
// NOTE) microbit is deprecated
//       The priority when sending notification is as follows.
//       BLE > Linkage > microbit > UART(USB) > UART2
// So you must set flag 0 that you don't want to receive data by notification.
// Any read command can be replyed via received port.
// If you want to receive notification via I2C port, set flag as "0x90". (Button and I2C are enabled.)

bool KeiganMotor::interface(uint8_t flag, bool response = true)
{
  uint8_t data[] = {flag};
  return write(CMD_REG_INTERFACE, data, sizeof(data), response);
}

bool KeiganMotor::limitCurrent(float value, bool response = true)
{
  return writeFloat(CMD_REG_LIMIT_CURRENT, value, response);
}

bool KeiganMotor::safeRun(bool isEnabled, uint32_t timeout, SafeRunOption op, bool response = true)
{
  uint8_t buf[] = {isEnabled, 0, 0, 0, 0, (uint8_t)op};
  uint32_big_encode(timeout, &buf[1]);
  return write(CMD_REG_SAFE_RUN, buf, sizeof(buf), response);
}

// Set I2C Slave address
// NOTE) Need to saveAllRegisters() and reboot() to reflect change.
bool KeiganMotor::i2cSlaveAddress(uint8_t address, bool response = true)
{
  uint8_t data[] = {address};
  return write(CMD_REG_I2C_SLAVE_ADDR, data, sizeof(data), response);
}

bool KeiganMotor::saveAllRegisters(bool response = true)
{
  return write(CMD_REG_SAVE_ALL_REGISTERS, NULL, 0, response);
}


bool KeiganMotor::resetRegister(uint8_t reg, bool response = true)
{
  uint8_t data[] = {reg};
  return write(CMD_REG_RESET_REGISTER, data, sizeof(data), response);  
}

bool KeiganMotor::resetAllRegisters(bool response = true)
{
  return write(CMD_REG_RESET_ALL_REGISTERS, NULL, 0, response);
}

bool KeiganMotor::readDeviceName(char *name)
{
  memset(name, 0, 14);
  readRegister(CMD_READ_DEVICE_NAME, name, 14);
  if(strncmp(name,"KM-",3)==0) return true;
  return false;
}

float KeiganMotor::readMaxTorque(void)
{
  return readFloat(CMD_REG_MAX_TORQUE);
}


bool KeiganMotor::enable(bool response = true)
{
  return write(CMD_DRIVER_ENABLE, NULL, 0, response);
}

bool KeiganMotor::disable(bool response = true)
{
  return write(CMD_DRIVER_DISABLE, NULL, 0, response);
}

bool KeiganMotor::speed(float value, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(value, data);
  return write(CMD_ACT_SPEED, data, sizeof(float), response);
}

bool KeiganMotor::speedRpm(float rpm, bool response = true)
{
  return speed(RPM_TO_RADPERSEC(rpm), response);
}

bool KeiganMotor::runAtVelocity(float value, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(value, data);
  return write(CMD_ACT_RUN_AT_VELOCITY, data, sizeof(float), response);
}

bool KeiganMotor::runAtVelocityRpm(float rpm, bool response = true)
{
  return runAtVelocity(RPM_TO_RADPERSEC(rpm), response);
}

bool KeiganMotor::presetPosition(float position, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(position, data);
  return write(CMD_ACT_PRESET_POSITION, data, sizeof(float), response);
}

bool KeiganMotor::presetPositionDegree(float degree, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(degree, data);
  return presetPosition(DEGREES_TO_RADIANS(degree), response);
}


bool KeiganMotor::runForward(bool response = true)
{
  return write(CMD_ACT_RUN_FORWARD, NULL, 0, response);
}

bool KeiganMotor::runReverse(bool response = true)
{
  return write(CMD_ACT_RUN_REVERSE, NULL, 0, response);
}

bool KeiganMotor::moveToPosition(float position, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(position, data);
  return write(CMD_ACT_MOVE_TO_POS, data, sizeof(float), response);
}

bool KeiganMotor::moveToPositionDegree(float degree, bool response = true)
{
  return moveToPosition(DEGREES_TO_RADIANS(degree), response);
}

bool KeiganMotor::moveByDistance(float distance, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  float_big_encode(distance, data);
  return write(CMD_ACT_MOVE_BY_DIST, data, sizeof(float), response);
}

bool KeiganMotor::moveByDistanceDegree(float degree, bool response = true)
{
  return moveByDistance(DEGREES_TO_RADIANS(degree), response);
}

bool KeiganMotor::stop(bool response = true)
{
  return write(CMD_ACT_STOP, NULL, 0, response);
}

bool KeiganMotor::free(bool response = true)
{
  return write(CMD_ACT_FREE, NULL, 0, response);
}

bool KeiganMotor::wait(uint32_t time, bool response = true)
{
  uint8_t data[sizeof(float)] = {0};
  uint32_big_encode(time, data);
  return write(CMD_QUE_WAIT, data, sizeof(float), response);
}

bool KeiganMotor::led(LedState state, uint8_t r, uint8_t g, uint8_t b, bool response = true)
{
  uint8_t data[4] = {(uint8_t)state, r, g, b};
  return write(CMD_LED_SET_LED, data, sizeof(data), response);
}

bool KeiganMotor::reboot(bool response = true)
{
  return write(CMD_OTHERS_REBOOT, NULL, 0, response);
}
