#ifndef km1_i2c_h
#define km1_i2c_h

#include <Wire.h>
#include "CRC16.h"

#define LIBRARY_VERSION 1.3.0


#define PI (3.14159265358979f) // π: def. of PI
#define RPM_TO_RADPERSEC(rpm) ((float)(rpm) * 2 * (PI) / 60)  
#define RPM_TO_RADPERMILLIS(rpm) ((float)(rpm) * 2 * (PI) / 60 / 1000)  
#define RADPERSEC_TO_RPM(rps) ((float)(rps) * 360 / (2 * (PI)))
#define RADPERMILLIS_TO_RPM(rpms) ((float)(rpms) * 1000 * 360 / (2 * (PI)))
#define DEGREES_TO_RADIANS(deg) ((float)(deg) * (PI) / 180)
#define RADIANS_TO_DEGREES(rad) ((float)(rad) * 180 / (PI))

// Read Registers
#define CMD_REG_MAX_SPEED 0x02 
#define CMD_REG_MAX_SPEED_LEN 9

#define CMD_REG_MIN_SPEED 0x03 
#define CMD_REG_MIN_SPEED_LEN 9

#define CMD_REG_CURVE_TYPE 0x05 
#define CMD_REG_CURVE_TYPE_LEN 6

#define CMD_REG_ACC 0x07 
#define CMD_REG_ACC_LEN 9

#define CMD_REG_DEC 0x08 
#define CMD_REG_DEC_LEN 9

#define CMD_REG_JERK 0x0C
#define CMD_REG_JERK_LEN 11

#define CMD_REG_MAX_TORQUE 0x0E
#define CMD_REG_MAX_TORQUE_LEN 9

// #define CMD_REG_GEAR_RATIO 0x13
// #define CMD_REG_GEAR_RATIO_LEN 9

#define CMD_REG_TEACH_INTERVAL 0x16
#define CMD_REG_TEACH_INTERVAL_LEN 9

#define CMD_REG_PLAYBACK_INTERVAL 0x17
#define CMD_REG_PLAYBACK_INTERVAL_LEN 9

#define CMD_REG_Q_CURRENT_P 0x18
#define CMD_REG_Q_CURRENT_P_LEN 9

#define CMD_REG_Q_CURRENT_I 0x19
#define CMD_REG_Q_CURRENT_I_LEN 9

#define CMD_REG_Q_CURRENT_D 0x1A
#define CMD_REG_Q_CURRENT_D_LEN 9

#define CMD_REG_SPEED_P 0x1B 
#define CMD_REG_SPEED_P_LEN 9

#define CMD_REG_SPEED_I 0x1C 
#define CMD_REG_SPEED_I_LEN 9

#define CMD_REG_SPEED_D 0x1D 
#define CMD_REG_SPEED_D_LEN 9

#define CMD_REG_POSITION_P 0x1E 
#define CMD_REG_POSITION_P_LEN 9

#define CMD_REG_RESET_PID 0x22
#define CMD_REG_RESET_PID_LEN 5

#define CMD_REG_LIMIT_CURRENT 0x25 
#define CMD_REG_LIMIT_CURRENT_LEN 9

#define CMD_REG_MOTOR_MEAS_INTVL 0x2C 
#define CMD_REG_MOTOR_MEAS_INTVL_LEN 6

#define CMD_REG_MOTOR_MEAS_BY_DEFAULT 0x2D 
#define CMD_REG_MOTOR_MEAS_BY_DEFAULT_LEN 6

#define CMD_REG_INTERFACE 0x2E
#define CMD_REG_INTERFACE_LEN 6

#define CMD_REG_RESPONSE 0x2F 
#define CMD_REG_RESPONSE_LEN 6

#define CMD_REG_OWN_COLOR 0x3A 
#define CMD_REG_OWN_COLOR_LEN 8

#define CMD_REG_IMU_MEAS_INTVL 0x3C
#define CMD_REG_IMU_MEAS_INTVL_LEN 6

#define CMD_REG_IMU_MEAS_BY_DEFAULT 0x3D
#define CMD_REG_IMU_MEAS_BY_DEFAULT_LEN 6

#define CMD_REG_READ_REGISTER 0x40
#define CMD_REG_READ_REGISTER_LEN 6

#define CMD_REG_SAVE_ALL_REGISTERS 0x41 
#define CMD_REG_SAVE_ALL_REGISTERS_LEN 5

#define CMD_READ_DEVICE_NAME 0x46 
#define CMD_READ_DEVICE_NAME_LEN 5

#define CMD_READ_DEVICE_INFO 0x47 
#define CMD_READ_DEVICE_INFO_LEN 5

#define CMD_REG_RESET_REGISTER 0x4E 
#define CMD_REG_RESET_REGISTER_LEN 6

#define CMD_REG_RESET_ALL_REGISTERS 0x4F 
#define CMD_REG_RESET_ALL_REGISTERS_LEN 5

// Read Status
#define CMD_READ_STATUS 0x9A 
#define CMD_READ_STATUS_LEN 5

// I2C
#define CMD_REG_I2C_SLAVE_ADDR 0xC0 
#define CMD_REG_I2C_SLAVE_ADDR_LEN 6


// Motor Disable and Enable
#define CMD_DRIVER_DISABLE 0x50
#define CMD_DRIVER_DISABLE_LEN 5

#define CMD_DRIVER_ENABLE 0x51 
#define CMD_DRIVER_ENABLE_LEN 5

// Motor Motion Control
#define CMD_ACT_SPEED 0x58 
#define CMD_ACT_SPEED_LEN 9

#define CMD_ACT_PRESET_POSITION 0x5A 
#define CMD_ACT_PRESET_POSITION_LEN 9

#define CMD_READ_POSITION_OFFSET 0x5B 
#define CMD_READ_POSITION_OFFSET_LEN 5 

#define CMD_ACT_RUN_FORWARD 0x60
#define CMD_ACT_RUN_FORWARD_LEN 5

#define CMD_ACT_RUN_REVERSE 0x61
#define CMD_ACT_RUN_REVERSE_LEN 5

#define CMD_ACT_RUN_AT_VELOCITY 0x62
#define CMD_ACT_RUN_AT_VELOCITY_LEN 9

#define CMD_ACT_MOVE_TO_POS_AT_SPEED 0x65
#define CMD_ACT_MOVE_TO_POS_AT_SPEED_LEN 13

#define CMD_ACT_MOVE_TO_POS 0x66
#define CMD_ACT_MOVE_TO_POS_LEN 9

#define CMD_ACT_MOVE_BY_DIST_AT_SPEED 0x67 
#define CMD_ACT_MOVE_BY_DIST_AT_SPEED_LEN 13

#define CMD_ACT_MOVE_BY_DIST 0x68 
#define CMD_ACT_MOVE_BY_DIST_LEN 9

#define CMD_ACT_FREE 0x6C
#define CMD_ACT_FREE_LEN 5

#define CMD_ACT_STOP 0x6D 
#define CMD_ACT_STOP_LEN 5

#define CMD_ACT_BRAKE 0x6E 
#define CMD_ACT_BRAKE_LEN 5

#define CMD_ACT_HOLD_TORQUE 0x72
#define CMD_ACT_HOLD_TORQUE_LEN 9

#define CMD_ACT_DO_TASKSET 0x81
#define CMD_ACT_DO_TASKSET_LEN 11

#define CMD_ACT_STOP_DO_TASKSET 0x82 
#define CMD_ACT_STOP_DO_TASKSET_LEN 5

#define CMD_ACT_START_PLAYBACK_MOTION_WO_PREP 0x85 
#define CMD_ACT_START_PLAYBACK_MOTION_WO_PREP_LEN 12

#define CMD_ACT_PREPARE_PLAYBACK_MOTION 0x86 
#define CMD_ACT_PREPARE_PLAYBACK_MOTION_LEN 12

#define CMD_ACT_START_PLAYBACK_MOTION 0x87 
#define CMD_ACT_START_PLAYBACK_MOTION_LEN 5

#define CMD_ACT_STOP_PLAYBACK_MOTION 0x88 
#define CMD_ACT_STOP_PLAYBACK_MOTION_LEN 5

// Queue
#define CMD_QUE_PAUSE 0x90 
#define CMD_QUE_PAUSE_LEN 5

#define CMD_QUE_RESUME 0x91
#define CMD_QUE_RESUME_LEN 5

#define CMD_QUE_WAIT 0x92 
#define CMD_QUE_WAIT_LEN 9

#define CMD_QUE_ERASE_TASK 0x94
#define CMD_QUE_ERASE_TASK_LEN 5

#define CMD_QUE_RESET 0x95
#define CMD_QUE_RESET_LEN 5

// Recording Taskset
#define CMD_T_START_RECORD_TASKSET 0xA0
#define CMD_T_START_RECORD_TASKSET_LEN 7

#define CMD_T_STOP_RECORD_TASKSET 0xA2
#define CMD_T_STOP_RECORD_TASKSET_LEN 5

#define CMD_T_ERASE_TASKSET 0xA3
#define CMD_T_ERASE_TASKSET_LEN 7

#define CMD_T_ERASE_ALL_TASKSETS 0xA4
#define CMD_T_ERASE_ALL_TASKSETS_LEN 5

#define CMD_T_TASKSET_NAME 0xA5
#define CMD_T_TASKSET_NAME_LEN 20

#define CMD_READ_TASKSET_INFO 0xA6 
#define CMD_READ_TASKSET_INFO_LEN 7

#define CMD_READ_TASKSET_USAGE 0xA7
#define CMD_READ_TASKSET_USAGE_LEN 5

// Teaching Motion
#define CMD_DT_START_TEACH_MOTION_WO_PREP 0xA9 
#define CMD_DT_START_TEACH_MOTION_WO_PREP_LEN 11

#define CMD_DT_PREPARE_TEACH_MOTION 0xAA
#define CMD_DT_PREPARE_TEACH_MOTION_LEN 11

#define CMD_DT_START_TEACH_MOTION 0xAB
#define CMD_DT_START_TEACH_MOTION_LEN 5

#define CMD_DT_STOP_TEACH_MOTION 0xAC
#define CMD_DT_STOP_TEACH_MOTION_LEN 5

#define CMD_DT_ERASE_MOTION 0xAD
#define CMD_DT_ERASE_MOTION_LEN 7

#define CMD_DT_ERASE_ALL_MOTION 0xAE
#define CMD_DT_ERASE_ALL_MOTION_LEN 5

#define CMD_DT_MOTION_NAME 0xAF
#define CMD_DT_MOTION_NAME_LEN 20

#define CMD_READ_MOTION_INFO 0xB0
#define CMD_READ_MOTION_INFO_LEN 7

#define CMD_READ_MOTION_USAGE 0xB1
#define CMD_READ_MOTION_USAGE_LEN 5

// Measurement Read
#define CMD_READ_MOTOR_MEASUREMENT 0xB4
#define CMD_READ_MOTOR_MEASUREMENT_LEN 5

#define CMD_READ_IMU_MEASUREMENT 0xB5
#define CMD_READ_IMU_MEASUREMENT_LEN 5

// LED
#define CMD_LED_SET_LED 0xE0
#define CMD_LED_SET_LED_LEN 9

// Measurement Notification
#define CMD_MOTOR_START_MEASUREMENT 0xE6
#define CMD_MOTOR_START_MEASUREMENT_LEN 5

#define CMD_MOTOR_STOP_MEASUREMENT 0xE7
#define CMD_MOTOR_STOP_MEASUREMENT_LEN 5

#define CMD_IMU_START_MEASUREMENT 0xEA
#define CMD_IMU_START_MEASUREMENT_LEN 5

#define CMD_IMU_STOP_MEASUREMENT 0xEB
#define CMD_IMU_STOP_MEASUREMENT_LEN 5

// Others
#define CMD_OTHERS_REBOOT 0xF0
#define CMD_OTHERS_REBOOT_LEN 5

#define CMD_OTHERS_ENABLE_CHECK_SUM 0xF3
#define CMD_OTHERS_ENABLE_CHECK_SUM_LEN 6

#define CMD_OTHERS_SERIAL_NUMBER 0xFA
#define CMD_OTHERS_SERIAL_NUMBER_LEN 6

#define CMD_OTHERS_ADJUST_PHASE_DIFF 0xFB
#define CMD_OTHERS_ADJUST_PHASE_DIFF_LEN 14

#define CMD_OTHERS_DFU 0xFD
#define CMD_OTHERS_DFU_LEN 5

#define CMD_TASKSET_END_FLAG 0xFF
#define CMD_TASKSET_END_FLAG_LEN 1


class KeiganMotor {

    // byte address;
    // float position;
    // float velocity;
    // float torque;

    public:
        KeiganMotor(uint8_t address);

        void init(uint8_t address);
        void begin();

        float getPosition();
        float getVelocity();
        float getTorque();

        // Write / Read
        void write(uint8_t command, uint8_t *value, uint8_t value_len);
        void readRegister(uint8_t reg, uint8_t *value, uint8_t value_len);

        // Registers
        enum curveType {
            CURVETYPE_NONE = 0,
            CURVETYPE_TRAPEZOID = 1
        };
        void maxSpeed(float value); //TODO // Set max speed [rad/s]
        void minSpeed(float value); //TODO // Set min speed [rad/s]
        void curveType(uint8_t curveType); // Set motion control curve type 0:None, 1:Trapezoid
        void acc(float value); // Set acceleration [rad/s^2]
        void dec(float value); // Set deceleration [rad/s^2]
        void maxTorque(float value); // Set max torque
        void enableCheckSum(bool isEnabled); // Enable CheckSum (CRC16)
        void teachingInterval(uint32_t interval_ms); // Set teaching interval (position sampling interval) [msec]
        void playbackInterval(uint32_t interval_ms); // Set playback interval (position playback interval) [msec]

            // PID Parameters //TODO 
            void qCurrentP(float p);
            void qCurrentI(float i);
            void qCurrentD(float d);
            void speedP(float p);
            void speedI(float i);
            void speedD(float d);
            void positionP(float p);
            void resetPID();

        void motorMeasurementInterval(uint8_t interval);//TODO 
        void motorMeasurementByDefault(bool isEnabled);//TODO 
        void interface(uint8_t flag); 
        void response(uint8_t flag);//TODO 
        void iMUMeasurementInterval(uint8_t interval);//TODO 
        void iMUMeasurementByDefault(bool isEnabled);//TODO 

        void readRegister(uint8_t reg);
        void readMotorMeasurement(void);
        void saveAllRegisters();

        void readDeviceName(char *name);//TODO 
        void readDeviceInfo(uint8_t type,  char *str);//TODO 

        void resetRegister(uint8_t reg); //TODO
        void resetAllRegisters();

        void i2cSlaveAddress(uint8_t address); // I2C Slave address. Need to saveAllRegisters() and reboot() to reflect.

        // Motion Control
        void disable(); // Disable Motor Action
        void enable(); // Enable Motor Action and energize 
        // Preset
        void speed(float value); // Set motor speed [rad/s] 
        void speedRpm(float rpm); // Set motor speed [rpm]
        void presetPosition(float position); // Preset position: presetPosition(0) is set the current position as "Zero" 
        // Speed Control
        void runAtVelocity(float value); // Run motor at velocity [rad/s]
        void runAtVelocityRpm(float rpm); // Run motor at velobity [rpm]
        void runForward(); // Run motor forward (Direction Counter Clock Wise)
        void runReverse(); // Run motor reverse (Direction Clock Wise)
        // Position Control
        void moveToPosition(float position); // Move to absolute position [radians]
        void moveToPositionDegree(float degree); // Move to absolute position [degree]
        void moveByDistance(float distance); // Move by distance [radians]
        void moveByDistanceDegree(float degree); // Move by distance [degree]
        // Stop
        void stop(); // Stop and hold torque
        void free(); // De-energize motor

        // Queue
        void wait(uint32_t time); // Wait for execution of next commands for time [msec].

        // LED
        enum led_state{
            LED_STATE_OFF = 0,
            LED_STATE_ON_SOLID = 1,
            LED_STATE_ON_FLASH = 2
        };

        void led(uint8_t state, uint8_t r, uint8_t g, uint8_t b); // Set LED state
      
        void enableMotorMeasurement(); // TODO
        void disableMotorMeasurement(); // TODO

        // Others
        void reboot();

    private:
        void appendID(uint8_t *data);
        float readFloat(uint8_t reg);
        uint8_t _address; // I2C address
        uint16_t _commandID; // commandID
        
};

#endif // km1_i2c_h