#include "KM1_I2C.h"
#include "TypeUtility.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

static bool crc16_ready = false;

// #define _ESP32_HAL_I2C_H_

KeiganMotor::KeiganMotor(uint8_t address)
{
    init(address);
    Wire.begin();
    if(!crc16_ready){
        crc16_init();
    }
}

void KeiganMotor::init(uint8_t address)
{
    _address = address;
    _commandID = 0;

}

void KeiganMotor::begin()
{
    Wire.begin();
}


/*
 **@brief Data frame to send to KeiganMotor
 * 
 *      [0]          [1][2]           [3]...[n-3]		 [n-2][n-1]	
 * uint8_t command	uint16_t id		uint8_t *value		 uint16_t CRC		
 *      0~0xFF	     0～0xFFFF      depends on command    0～0xFFFF   
 */

/*
 **@brief Read response from KeiganMotor 
 * 
 *      [0]          [1][2]         	[3]		          [4]...[n-3]		 [n-2][n-1]	
 * uint8_t tx_type	uint16_t id		uint8_t command 	uint8_t *value		 uint16_t CRC		
 *      0x40	     0～0xFFFF         0~0xFF   		depends on command   0～0xFFFF   
 */


void KeiganMotor::appendID(uint8_t *data)
{
    uint16_big_encode(_commandID, data);
    if(_commandID == 0xFFFF){
        _commandID = 0;
    } else {
        _commandID++;
    }
}


void KeiganMotor::write(uint8_t command, uint8_t *value, uint8_t value_len)
{
    uint8_t len = value_len + 5;
    uint8_t data[50] = {0};
    data[0] = command;
    appendID(&data[1]);
    if(value_len > 0){
        memcpy(&data[3], value, value_len);
    }
    // TODO CheckSum
    append_crc16(data, len-2);
    Wire.beginTransmission(_address);
    Wire.write(data, len);
    Wire.endTransmission();
    // delay(1);
}

void KeiganMotor::readRegister(uint8_t reg, uint8_t *value, uint8_t value_len)
{
    // TODO
    uint8_t data[50] = {0};
    data[0] = CMD_REG_READ_REGISTER;
    appendID(&data[1]);
    data[3] = reg;
    
    Wire.beginTransmission(_address);
    Wire.write(data, CMD_REG_READ_REGISTER_LEN);
    Wire.endTransmission();

    uint8_t len = value_len + 6;

    Wire.requestFrom(_address, len);

    uint8_t cnt = 0;
    while (Wire.available()) { // slave may send less than requested
    
        value[cnt] = Wire.read();
        cnt ++;
    }
    
}

void KeiganMotor::readMotorMeasurement(void)
{
    write(CMD_READ_MOTOR_MEASUREMENT, NULL, 0);
}

// void KeiganMotor::readMotorMeasurement2(void)
// {
//     uint8_t val[20];

//     readRegisters(CMD_READ_MOTOR_MEASUREMENT, val, 12);


//     uint8_t address = data[0];
//     uint8_t cmd = data[1];
  
//   if (cmd == 0xB4 && len == 16) {
    
//     // Motor Measurement
//     float position = float_big_decode(&data[2]);
//     float velocity = float_big_decode(&data[6]);
//     float torque = float_big_decode(&data[10]);
// }


float KeiganMotor::readFloat(uint8_t reg)
{
    uint8_t value[sizeof(float)]; 
    readRegister(reg, value, sizeof(float));
}


// Set motion control curve type 0:None, 1:Trapezoid
void KeiganMotor::curveType(uint8_t curveType)
{
    uint8_t data[] = {curveType};
    write(CMD_REG_CURVE_TYPE, data, sizeof(data));
}


// Set acceleration [rad/s^2]
void KeiganMotor::acc(float value)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(value, data);
    write(CMD_REG_ACC, data, sizeof(float));
}

// Set deceleration [rad/s^2]
void KeiganMotor::dec(float value)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(value, data);
    write(CMD_REG_DEC, data, sizeof(float));
}

// Set max torque [N*m]
void KeiganMotor::maxTorque(float value)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(value, data);
    write(CMD_REG_MAX_TORQUE, data, sizeof(float));
}

// Enable CheckSum
void KeiganMotor::enableCheckSum(bool isEnabled)
{
    uint8_t data[] = {isEnabled};
    write(CMD_OTHERS_ENABLE_CHECK_SUM, data, sizeof(data));
}


// Set interface port by bit flag
// If set to 0, the interface port is ignored.
//
// bit7   | bit6 | bit5 | bit4 | bit3 | bit2     | bit1    | bit0
// button | -    | -    | I2C  | USB  | microbit | Linkage | BLE
//
// NOTE) The priority when sending notification is as follows.
//       BLE > Linkage > microbit > USB > I2C 
// So you must set flag 0 that you don't want to receive data by notification. 
// Any read command can be replyed via received port.
// If you want to receive notification via I2C port, set flag as "0x90". (Button and I2C are enabled.)

void KeiganMotor::interface(uint8_t flag)
{
    uint8_t data[] = {flag};
    write(CMD_REG_INTERFACE, data, sizeof(data));
}


// Set I2C Slave address
// NOTE) Need to saveAllRegisters() and reboot() to reflect change.
void KeiganMotor::i2cSlaveAddress(uint8_t address)
{
    uint8_t data[] = {address};
    write(CMD_REG_I2C_SLAVE_ADDR, data, sizeof(data));
}

void KeiganMotor::saveAllRegisters()
{
    write(CMD_REG_SAVE_ALL_REGISTERS, NULL, 0);
}

void KeiganMotor::resetAllRegisters()
{
    write(CMD_REG_RESET_ALL_REGISTERS, NULL, 0);   
}

void KeiganMotor::enable()
{
    write(CMD_DRIVER_ENABLE, NULL, 0);
}

void KeiganMotor::disable()
{
    write(CMD_DRIVER_DISABLE, NULL, 0);
}


void KeiganMotor::speed(float value)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(value, data);
    write(CMD_ACT_SPEED, data, sizeof(float));
}


void KeiganMotor::speedRpm(float rpm)
{
    speed(RPM_TO_RADPERSEC(rpm));
}   


void KeiganMotor::runAtVelocity(float value)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(value, data);
    write(CMD_ACT_RUN_AT_VELOCITY, data, sizeof(float));
}

void KeiganMotor::runAtVelocityRpm(float rpm)
{
    runAtVelocity(RPM_TO_RADPERSEC(rpm));
}

void KeiganMotor::presetPosition(float position)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(position, data);
    write(CMD_ACT_PRESET_POSITION, data, sizeof(float));
}

void KeiganMotor::runForward()
{
    write(CMD_ACT_RUN_FORWARD, NULL, 0);
}

void KeiganMotor::runReverse()
{
    write(CMD_ACT_RUN_REVERSE, NULL, 0);
}

void KeiganMotor::moveToPosition(float position)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(position, data);
    write(CMD_ACT_MOVE_TO_POS, data, sizeof(float));
}

void KeiganMotor::moveToPositionDegree(float degree)
{
    moveToPosition(DEGREES_TO_RADIANS(degree));
}

void KeiganMotor::moveByDistance(float distance)
{
    uint8_t data[sizeof(float)] = {0};
    float_big_encode(distance, data);
    write(CMD_ACT_MOVE_BY_DIST, data, sizeof(float));
}

void KeiganMotor::moveByDistanceDegree(float degree)
{
    moveByDistance(DEGREES_TO_RADIANS(degree));
}

void KeiganMotor::stop()
{
    write(CMD_ACT_STOP, NULL, 0);

}
void KeiganMotor::free()
{
    write(CMD_ACT_FREE, NULL, 0);
}

void KeiganMotor::wait(uint32_t time)
{
    uint8_t data[sizeof(float)] = {0};
    uint32_big_encode(time, data);
    write(CMD_QUE_WAIT, data, sizeof(float));
}

void KeiganMotor::led(uint8_t state, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t data[4] = {(uint8_t)state, r, g, b};
    write(CMD_LED_SET_LED, data, sizeof(data));
}

void KeiganMotor::reboot()
{
    write(CMD_OTHERS_REBOOT, NULL, 0);
}
