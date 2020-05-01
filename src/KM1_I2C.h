/**
   @file KM1_I2C.h
   @brief        KeiganMotor library using I2C communication
   @brief        KeiganMotor を I2C 経由で制御するためのライブラリ
   @details       This library uses Arduino "Wire" library.
   @version      2.00
   @date         2020/5/1
   @author       Takashi Tokuda (Keigan Inc.)
   @par          History              
                 
*/

#ifndef km1_i2c_h
#define km1_i2c_h

#include <Wire.h>
#include "CRC16.h"
#include "command_list.h"
#include "TypeUtility.h"

#define LIBRARY_VERSION 1.5.0

/**
   @typedef error_t
   @brief Error or success information of KeiganMotor
*/
typedef struct
{
  bool isValid; /**< true when the received data is valid (without error) */
  uint16_t id; /**< identifier to specify command */
  uint8_t cmd; /**< command number */
  uint8_t code; /**< error code */
  uint32_t info; /**< error detail information */
} error_t;

/**
   @typedef motor_meas_t
   @brief Motor measurement data of KeiganMotor
   @details Get by using readMotorMeasurement()
*/
typedef struct
{
  bool isValid;
  float position;
  float velocity;
  float torque;
} motor_meas_t;

/**
   @typedef imu_meas_t
   @brief IMU measurement data of KeiganMotor
   @details Get by using readIMUMeasurement()
*/
typedef struct
{
  bool isValid;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temp;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
} imu_meas_t;

/**
   @typedef status_t
   @brief Status of KeiganMotor
*/
typedef struct
{
  bool isValid; /**< true when the received status is valid (without error)*/
  bool isEnabled; /**< motor is enabled action */
  bool isQueueRunning; /**< motor queue is running (not paused) */
  bool isMotorMeasNotifyEnabled; /**< motor measurement notification enabled (always false when using I2C) */
  bool isIMUMeasNotifyEnabled; /**< IMU measurement notification enabled (always false when using I2C) */
  bool isCheckSumEnabled; /**< true when KeiganMotor validates CRC16(Checksum) */
  uint8_t flashState;
  uint8_t motorControlMode;

} status_t;


/**
   @brief Safe run option
   @details Motor action in case KeiganMotor cannot receive next action command
*/
enum SafeRunOption
{
  SAFE_RUN_TIMEOUT_FREE = 0, /**< free 非励磁状態 */
  SAFE_RUN_TIMEOUT_DISABLE = 1, /**< disable 動作不許可状態 */
  SAFE_RUN_TIMEOUT_STOP = 2, /**< run_at_velocity(0)  速度制御ゼロ */
  SAFE_RUN_TIMEOUT_POS_FIX = 3 /**< fix position at the point その場で位置制御*/
};

/**
   @enum LedState
   @brief Led State of KeiganMotor
*/
enum LedState
{
  LED_STATE_OFF = 0, /**< off 消灯 */
  LED_STATE_ON_SOLID = 1, /**< on 点灯 */
  LED_STATE_ON_FLASH = 2 /**< on flashing 点滅 */
};

/**
   @enum CurveType
   @brief Curve type for motion control
   @remark Set value 0 when sending run or move command continously in short period.
*/
enum CurveType
{
  CURVETYPE_NONE = 0, /**< No curve (Cylic) */
  CURVETYPE_TRAPEZOID = 1 /**< Trapezoidal curve */
};


/**

   KeiganMotor Class <br>
   @brief Class to handle KeiganMotor via I2C Communication
   @details It is able to control motor, change settings and read measurement data
   KeiganMotor を I2C 経由で扱うためのクラス
   モーターの制御や設定、測定値の取得を行う
*/
class KeiganMotor
{

  public:
    /**
       KeiganMotorコンストラクタ.
       @brief Initialize by specifying I2C Address
       @param address I2C address (7bit: 0x01~0x7E)
       @
    */
    KeiganMotor(uint8_t address, int clock = 1000000);
    void init(uint8_t address, int clock = 1000000);
    /** @brief start I2C communication via Wire library */
    void begin();


    /**
      @brief Status of KeiganMotor
       KeiganMotor のステータス
    */
    status_t status; // Status

    /** @brief Motor measurement data*/
    motor_meas_t measurement;

    /** @brief motor angle position [radian]*/
    float position;
    /** @brief motor angle position [degree]*/
    float degree;

    /** @brief motor angle velocity [radian/second]*/
    float velocity;
    /** @brief motor angle velocity [rotation/minute] (rpm)*/
    float rpm;
    /** @brief motor torque [N*m] */
    float torque;

    /** @brief IMU measurement data */
    imu_meas_t imu; // IMU Measurement

    /** @brief Error or success information*/
    error_t error;

    /** @brief returns error or success information
        @return error

    */
    error_t getError();

    /**
       @brief Write command function
       @param[in] command      command
       @param[in] *value       value pointer to write
       @param[in] value_len    value length
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @ref                    Use getError().code to get error code.
    */
    bool write(uint8_t command, uint8_t *value, uint8_t value_len, bool response = true); //!< Write command function.

    /**
       @brief Read register function
       @param[in] reg          register
       @param[out] *value      value pointer to read
       @param[in] value_len    value length
       @retval true            received data successfully
       @retval false           got an error
       @ref                    Use getError().code to get error code.
    */
    bool readRegister(uint8_t reg, uint8_t *value, uint8_t value_len);

    /**
       @brief Read status function
       @param[in] reg          register
       @param[out] *value      value pointer to read
       @param[in] value_len    value length
       @return status          Status of KeiganMotor (see status_t)
    */
    status_t readStatus(void);


    /**
       @brief Set max speed function to limit absolute value of velocity
       @param[in] value        float speed to set [radian/second] 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool maxSpeed(float value, bool response = true);          //TODO // Set max speed [rad/s]
    
    /**
       @brief Set min speed function. It is used in case of "prepare playback motion"
       @param[in] value        float speed to set [radian/second] 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool minSpeed(float value, bool response = true);          //TODO // Set min speed [rad/s]

    /**
       @brief Set curve type function for motion control.
       @param[in] type         CurveType to set  
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool curveType(CurveType type, bool response = true);   // Set motion control curve type 0:None, 1:Trapezoid

    /**
       @brief Set acceleration function. It is ignored when curve type is 0(CURVETYPE_NONE).
       @param[in] value        float acceleration to set [radian/second^2] 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool acc(float value, bool response = true);               // Set acceleration [rad/s^2]

    /**
       @brief Set deceleration function. It is ignored when curve type is 0(CURVETYPE_NONE).
       @param[in] value        float deceleration to set [radian/second^2] 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool dec(float value, bool response = true);               // Set deceleration [rad/s^2]

    /**
       @brief Set Max torque function to limit current not to be above the torque.
       @param[in] value        float max torque to set [N*m] 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */       
    bool maxTorque(float value, bool response = true);         // Set max torque

    /**
       @brief Set checksum validadtion enabled.
       @details KeiganMotor will validate received data after receiving this command.
       @param[in] isEnabled    true if make Checksum validation enabled 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */       
    bool enableCheckSum(bool isEnabled, bool response = true); // Enable CheckSum (CRC16)

    /**
       @brief Set Q-axis current PID controller gain P.
       @param[in] value    Q-axis current PID controller gain P (proportional)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool qCurrentP(float value, bool response = true);
    
     /**
       @brief Set Q-axis current PID controller gain I.
       @param[in] value    Q-axis current PID controller gain I (integral)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */   
    
    bool qCurrentI(float value, bool response = true);
    /**
       @brief Set Q-axis current PID controller gain D.
       @param[in] value    Q-axis current PID controller gain D (differential)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool qCurrentD(float value, bool response = true);
    
    /**
       @brief Set Speed PID controller gain P.
       @param[in] value    Speed PID controller gain P (proportional)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool speedP(float value, bool response = true);

    /**
       @brief Set Speed PID controller gain I.
       @param[in] value    Speed PID controller gain I (integral)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */        
    bool speedI(float value, bool response = true);

    /**
       @brief Set Speed PID controller gain D.
       @param[in] value    Speed PID controller gain D (differential).
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */        
    bool speedD(float value, bool response = true);

    /**
       @brief Set Position PID controller gain P 
       @param[in] value    Position PID controller gain P (proportional)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */            
    bool positionP(float value, bool response = true);

    /**
       @brief Set Position PID controller gain I
       @param[in] value    Position PID controller gain I (integral)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Position integral and differential term are ignored out of positionIDThreshold range. (See positionIDThreshold function)
    */         
    bool positionI(float value, bool response = true);

    /**
       @brief Set Position PID controller gain D
       @param[in] value    Position PID controller gain D (differential).
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Position integral and differential term are ignored out of positionIDThreshold range. (See positionIDThreshold function)
    */           
    bool positionD(float value, bool response = true);

    /**
       @brief Set Position PID controller gain D
       @param[in] value    Position PID controller gain D (differential).
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Position integral and differential term are ignored out of positionIDThreshold range. (See positionIDThreshold function)
    */           
    bool positionIDThreshold(float value, bool response = true);

    /**
       @brief Reset all the PID parameters to default
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() is required to save in flash.
    */      
    bool resetPID(bool response = true);

    /**
       @brief Set available interface
       @todo
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */     
    bool interface(uint8_t flag, bool response = true);

    /**
       @brief Limit current
       @param[in] value        Max current to limit. 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Make this value big enough when you want to draw maximum energy from power source.
    */   
    bool limitCurrent(float value, bool response = true);

    /**
       @brief Set safe run mode to stop motor automatically when it cannot receive next command within a certain period (timeout).
       @param[in] isEnabled    true if you want KeiganMotor stop automatically 
       @param[in] timeout      timeout [millisecond]
       @param[in] op           SafeRunOption (stop behaviour in case of timeout)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */   
    bool safeRun(bool isEnabled, uint32_t timeout, SafeRunOption op, bool response = true);

    /**
       @brief Read motor measurement data
       @details Position, velocity and torque values are available after sending this command.
       @code 
           if(m.readMotorMeasurement()){
               Serial.print(degree: );
               Serial.println(m.degree);
           } else {
               Serial.println("error");
           }
       @endcode
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */     
    bool readMotorMeasurement(void);

    /**
     * @todo
     * @remark You need to enable IMU measurement
     */
    bool readIMUMeasurement(void);

    /**
       @brief Save all the registers KeiganMotor retains.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark The register values are saved by this command permanently in the flash. 
    */          
    bool saveAllRegisters(bool response = true);

    /**
       @brief Reset a register value to default.
       @param[in] reg          Register (See command_list.h)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark The register values are saved by this command permanently in the flash. 
    */      
    bool resetRegister(uint8_t reg, bool response = true);

    /**
       @brief Reset all the registers to default.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() is required to save in flash.
    */        
    bool resetAllRegisters(bool response = true);
    

    /**
     * @brief Read the device name 
     * @details The name is like "KM-1U AI09#3R9"
     * @todo
     */
    float readMaxTorque(void);  

    /**
     * @brief Read the KeiganMotor device name 
     * @details The name is like "KM-1U AI09#3R9"
     * @todo
     */
    bool readDeviceName(char *name);              

    /**
     * @brief Read the device information 
     * @todo
     */    
    bool readDeviceInfo(uint8_t type, char *str); 

    /**
       @brief Set I2C slave address.
       @param[in] address      I2C address (0x00~0x7F)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() and reboot() is required to reflact the address value.
       @code
           if(m.i2cSlaveAddress(0x30)){
               m.saveAllRegisters();
               delay(3000);
               m.reboot();
           }
       @codeend
    */      
    bool i2cSlaveAddress(uint8_t address, bool response = true); 

    /**
       @brief Disable motor action.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */   
    bool disable(bool response = true); // Disable Motor Action

    /**
       @brief Enable motor action.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */  
    bool enable(bool response = true); 

    /**
       @brief Set angle speed [radian/second]  
       @param[in] value        speed [radian/second]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool speed(float value, bool response = true);            

    /**
       @brief Set angle speed [rotation/minute] (rpm)  
       @param[in] value        speed [radian/second]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */        
    bool speedRpm(float rpm, bool response = true);            // Set motor speed [rpm]

    /**
       @brief Set the current position as a certain position [radian]
       @details presetPosition(0) makes the current position as "Zero" point. 
       @param[in] value        preset position [radian]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */       
    bool presetPosition(float value, bool response = true); 

    /**
       @brief Set the current position as a certain position [degree]
       @details presetPositionDegree(30) makes the current position as "30 degree". 
       @param[in] value        preset position [degree]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */           
    bool presetPositionDegree(float degree, bool response = true); 


    /** @name Velocity Control
     *
     */
    /* @{ */
    /**
       @brief Run at a certain velocity [radian/second]
       @details This function combines speed and runForward or runReverse function.
       @param[in] value        velocity [radian/second]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */         
    bool runAtVelocity(float value, bool response = true);  // Run motor at velocity [rad/s]

    /**
       @brief Run at a certain velocity [rotation/minute] (rpm)
       @details Plus direction is counter clockwise when viewed from the rotation axis.
       @details This function combines speed and runForward or runReverse function.
       @param[in] value        velocity [rotation/minute] (rpm)
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool runAtVelocityRpm(float rpm, bool response = true); // Run motor at velobity [rpm]

    /**
       @brief Run forward
       @details It uses preset speed value by function speed()
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */       
    bool runForward(bool response = true);                // Run motor forward (Direction Counter Clock Wise)

    /**
       @brief Run Reverse
       @details It uses preset speed value by function speed()
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */    
    bool runReverse(bool response = true);                // Run motor reverse (Direction Clock Wise)

    /**
       @brief Stop
       @details Keep velocity 0 by velocity control
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool stop(bool response = true); // Stop and hold torque
    
    /* @} */

    
    /** @name Position Control
     *
     */
    /* @{ */
   
    /**
       @brief Move to an absolute position [radian]
       @details It uses preset speed value by function speed()
       @param[in] position     position [radian]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */        
    bool moveToPosition(float position, bool response = true);    

    /**
       @brief Move to an absolute position [degree]
       @details It uses preset speed value by function speed()
       @param[in] position     position [degree]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */       
    bool moveToPositionDegree(float degree, bool response = true); // Move to absolute position [degree]
    
    /**
       @brief Move by a distance (move to an relative position) [radian]
       @details It uses preset speed value by function speed()
       @param[in] distance     distance [radian]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool moveByDistance(float distance, bool response = true);     // Move by distance [radians]
    
    /**
       @brief Move by a distance (move to an relative position) [degree]
       @details It uses preset speed value by function speed()
       @param[in] distance     distance [degree]
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool moveByDistanceDegree(float degree, bool response = true); // Move by distance [degree]
    /* @} */
    
    /**
       @brief De-energize motor (make motor free)
       @details It keeps viscosity a little bit. You should use disable() to turn off rotational resistance completely.
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool free(bool response = true); 

    /**
       @brief  Wait for execution of next commands for time [msec].
       @details KeiganMotor has a FIFO queue inside, and this function make its execution paused for the time. 
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */     
    bool wait(uint32_t time, bool response = true); 

    /**
       @brief Set LED lit
       @param[in] state        LedState
       @param[in] r            Red brightness
       @param[in] g            Green brightness       
       @param[in] b            Blue brightness
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool led(LedState state, uint8_t r, uint8_t g, uint8_t b, bool response = true); 

    /**
       @brief Reboot KeiganMotor
       @param[in] response     true if you require response after sending command, default:true
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */      
    bool reboot(bool response = true);

  private:
    uint16_t appendID(uint8_t *data);
    /**
       @brief Write float command function
       @param[in] command      command
       @param[in] value        float value to write
       @param[in] response     true if you require response after sending command
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
    bool writeFloat(uint8_t cmd, float val, bool response = true);
    float readFloat(uint8_t reg);
    uint8_t readByte(uint8_t reg);
    uint32_t readUint32(uint8_t reg);
    uint8_t _address;    // I2C address
    uint16_t _commandID; // commandID
};

#endif // km1_i2c_h
