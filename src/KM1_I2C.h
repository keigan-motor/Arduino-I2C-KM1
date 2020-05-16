/**
   @file         KM1_I2C.h
   @brief        KeiganMotor library using I2C communication
   @brief        KeiganMotor を I2C 経由で制御するためのライブラリ
   @details      This library uses Arduino "Wire" library.
   @version      2.0.0
   @date         2020/5/1
   @author       Takashi Tokuda (Keigan Inc.)
   @par          History              
                 
*/

#ifndef km1_i2c_h
#define km1_i2c_h

#include <Wire.h>
#include "CRC16.h"
#include "Definitions.h"
#include "TypeUtility.h"

#define LIBRARY_VERSION 2.0.0

/**
   error_t
   @brief Error or success information received from KeiganMotor
*/
typedef struct
{
   bool isValid;  /**< true when the received data is valid (without error) */
   uint16_t id;   /**< identifier to specify command */
   uint8_t cmd;   /**< command number */
   uint8_t code;  /**< error code */
   uint32_t info; /**< error detail information */
} error_t;

/**
   motor_meas_t
   @brief Motor measurement data of KeiganMotor
   @details Call KeiganMotor::readMotorMeasurement() to get
*/
typedef struct
{
   bool isValid;   /**< true when the received data is valid (without error) */
   float position; /**< position [radian] */
   float velocity; /**< velocity [radian/second] */
   float torque;   /**< torque [N*m] */
} motor_meas_t;

/**
   imu_meas_t
   @brief IMU measurement data of KeiganMotor
   @details Call KeiganMotor::readIMUMeasurement() to get @n
   You can convert the raw value to the actual value as follows. @n
    - Acceleration
      - value [G] = raw_value * 2 / 32,767
    - Temperature
      - value [℃] = raw_value / 333.87 + 21.00
    - Gyroscope (Angular velocity)
      - value [degree/second] = raw_value * 250 / 32,767
      - value [radians/second] = raw_value * 0.00013316211    
   @note You need to call KeiganMotor::startIMUMeasurement() in advance to enable IMU.   
*/
typedef struct
{
   bool isValid;   /**< true when the received data is valid (without error) */
   int16_t accelX; /**< Acceleration of x-axis */
   int16_t accelY; /**< Acceleration of y-axis */
   int16_t accelZ; /**< Acceleration of z-axis */
   int16_t temp;   /**< Temperature */
   int16_t gyroX;  /**< Angular velocity around x-axis */
   int16_t gyroY;  /**< Angular velocity around y-axis */
   int16_t gyroZ;  /**< Angular velocity around z-axis */
} imu_meas_t;

/**
   status_t
   @brief Status of KeiganMotor
   @details flash state is as follows. 
   @see
   uint8_t flashState:
      - #FLASH_STATE_READY 0: ready, not busy (準備OK、ビジーでない、アイドル状態)
      - #FLASH_STATE_TEACHING_PREPARE 1: preparing for teaching motion (ティーチング準備中)
      - #FLASH_STATE_TEACHING_DOING 2: teaching motion (ティーチング実行中)
      - #FLASH_STATE_PLAYBACK_PREPARE 3: preparing for playback motion (プレイバック準備中)
      - #FLASH_STATE_PLAYBACK_DOING 4: executing playback motion (プレイバック実行中)
      - #FLASH_STATE_PLAYBACK_PAUSING 5: pausing playback motion (プレイバック一時停止中)
      - #FLASH_STATE_TASKSET_RECORDING 6: recording taskset (タスクセット記録中)
      - #FLASH_STATE_TASKSET_DOING 7: doing taskset (タスクセット実行中)
      - #FLASH_STATE_TASKSET_PAUSING 8: pausing taskset (タスクセット一時停止)
      - #FLASH_STATE_IMU 20: using IMU (shared resource with the flash) (IMU使用中, フラッシュと共通リソース)　@n
   @see
   uint8_t motorControlMode:
      - #MOTOR_CONTROL_MODE_NONE = 0: idle mode (アイドル状態、非動作中)
      - #MOTOR_CONTROL_MODE_VELOCITY = 1: Velocity Control（速度制御）
      - #MOTOR_CONTROL_MODE_POSITION = 2: Position Control（位置制御）
      - #MOTOR_CONTROL_MODE_TORQUE = 3: Torque Control（トルク制御）
   
*/
typedef struct
{
   bool isValid;                  /**< true when the received status is valid (without error)*/
   bool isEnabled;                /**< motor is enabled action */
   bool isQueuePaused;            /**< motor queue is running (not paused) */
   bool isMotorMeasNotifyEnabled; /**< motor measurement notification enabled (always false when using I2C) */
   bool isIMUMeasNotifyEnabled;   /**< IMU measurement notification enabled (always false when using I2C) */
   bool isCheckSumEnabled;        /**< true when KeiganMotor validates CRC16(Checksum) */
   uint8_t flashState;            /**< flash state. See above.*/
   uint8_t motorControlMode;      /**< motor control mode. See above */

} status_t;


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
       @param[in] address I2C slave address (7bit: 0x01~0x7E)
       @param[in] clock (It can be up to 4000000. default:1000000)
    */
   KeiganMotor(uint8_t address, int clock = 1000000);
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



   /**
       @brief Write command function
       @param[in] command      command
       @param[in] *value       value pointer to write
       @param[in] value_len    value length
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @note                    Use getError().code to get error code.
    */
   bool write(uint8_t command, uint8_t *value, uint8_t value_len, bool response = false);

   /** 
     * @brief Get the error that master retains. @n
     * getError() is designed to use after getting response of write command.
     * @return the latest error that Arduino retains (see error_t)
     * @code
     *    bool result = m.runFoward(true);
     *    if(result != 0){
     *      error_t error = m.getError();
     *      Serial.print("error code is ");
     *      Serial.println(error.code);
     *    }
     * @endcode 
     * @note The difference between getError() and readError() function is that @n
     * getError() just returns the latest error data  retained by master (Arduino), while readError() @n
     * send a command to request the latest error data that KeiganMotor retains. @n
     * 
    */
   error_t getError();

   /** @name Write register */
   /* @{ */
   /**
       @brief Set checksum validadtion enabled.
       @details KeiganMotor will validate received data after receiving this command.
       @param[in] isEnabled    true if make Checksum validation enabled 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool enableCheckSum(bool isEnabled, bool response = false); // Enable CheckSum (CRC16)
   /**
       @brief Set available interface
       @details You can enable or disable the data interfaces(physical 3 buttons, I2C, USB, BLE and so on). @n
        The motor chooses the output interface of motor measurement values and IMU values as @n
        (High priority)BLE > UART1(USB) > I2C(Low priority) by default. @n
        If you want to force it to send measurement values through USB, @n
        you need to set bit0(BLE) to OFF(0) and bit3(USB) to ON(1).
        For example, if you call set_interface(0b10001000), @n 
        Physical 3 buttons: enabled, I2C: disabled, USB: enabled and BLE: disabled. @n
        To save this setting to the flash memory, ensure you call saveAllRegisters(). @n
       @param[in] flag         uint8_t value consists of the following bit flag.
        - bit0: #INTERFACE_BIT_BLE: Bluetooth Low Energy (Wireless)  
        - bit1: #INTERFACE_BIT_LINKAGE: Linkage (Wireless)
        - bit2: -
        - bit3: #INTERFACE_BIT_UART1: UART1 (USB)
        - bit4: #INTERFACE_BIT_I2C: I2C (Wired)
        - bit5: #INTERFACE_BIT_DIGITAL_IO: DigitalIO (Wired) 
        - bit6: #INTERFACE_BIT_UART2: UART2 (Wired)
        - bit7: #INTERFACE_BIT_BUTTON: Physical buttons

       @param[in] response     true if you get response after sending command, default: false
       @code
         // Enable I2C and disable both of BLE and USB
         // Notification will be sent via I2C.
         uint8_t flag = 0;
         flag |= INTERFACE_BIT_I2C; // Enable I2C
         flag |= INTERFACE_BIT_BUTTON; // Enable buttons
         m.interface(flag);
         m.saveAllRegisters();
         m.reboot();
       @endcode
       @note The following interfaces share a common resource, so you cannot set 1 to all of them.
       - BLE and Linkage
       - UART1(USB) and UART2
       - I2C, DigitalIO and UART2 
       
     */
   bool interface(uint8_t flag, bool response = false);

   /**      
       @brief Limit current
       @param[in] value        Max current to limit. 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Make this value big enough when you want to draw maximum energy from power source.
    */
   bool limitCurrent(float value, bool response = false);

   /**      
       @brief Set write response enabled or disabled
       @details It response is enabled, KeiganMotor will send write response after receiving write command. @n
       @param[in] isEnabled    true if write response is enabled (default: true)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark 
    */
   bool writeResponse(bool isEnabled, bool response = false);

   /**
       @brief Set safe run mode to stop motor automatically when it cannot receive next command within a certain period (timeout).
       @param[in] isEnabled    true if you want KeiganMotor stop automatically 
       @param[in] timeout      timeout [millisecond]
       @param[in] op           SafeRunOption (stop behaviour in case of timeout)
       @see #SAFE_RUN_TIMEOUT_FREE = 0 @n
            #SAFE_RUN_TIMEOUT_DISABLE = 1 @n
            #SAFE_RUN_TIMEOUT_STOP = 2 @n
            #SAFE_RUN_TIMEOUT_POS_FIX = 3 @n
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool safeRun(bool isEnabled, uint32_t timeout, uint8_t op, bool response = false);

   /**
       @brief Set motor measurement interval
       @param[in] interval     the following integer value
                               - 0: INTERVAL_MS_2MS     =    2 [ms]
                               - 1: INTERVAL_MS_5MS     =    5 [ms]
                               - 2: INTERVAL_MS_10MS    =   10 [ms]
                               - 3: INTERVAL_MS_20MS    =   20 [ms]
                               - 4: INTERVAL_MS_50MS    =   50 [ms]
                               - 5: INTERVAL_MS_100MS   =  100 [ms]
                               - 6: INTERVAL_MS_200MS   =  200 [ms]
                               - 7: INTERVAL_MS_500MS   =  500 [ms]
                               - 8: INTERVAL_MS_1000MS  = 1000 [ms]
                               - 9: INTERVAL_MS_2000MS  = 2000 [ms]
                               - 10:INTERVAL_MS_5000MS  = 5000 [ms]
                               - 11:INTERVAL_MS_10000MS =10000 [ms]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @note The default value is 100ms. The value less than 100ms will be ignored only in case of BLE (Bluetooth) @n
       See interface function

    */
   bool motorMeasurementInterval(uint8_t interval, bool response = false);

   /**
       @brief Save all the registers KeiganMotor retains.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark The register values are saved by this command permanently in the flash. 
    */
   bool saveAllRegisters(bool response = false);

   /**
       @brief Reset a register value to default.
       @param[in] reg          Register (See command_list.h)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark The register values are saved by this command permanently in the flash. 
    */
   bool resetRegister(uint8_t reg, bool response = false);

   /**
       @brief Reset all the registers to default.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() is required to save in flash.
    */
   bool resetAllRegisters(bool response = false);
   /* @} */


   /** @name Write register (PID Controller) */
   /* @{ */
   /**
       @brief Set Q-axis current PID controller gain P.
       @param[in] value    Q-axis current PID controller gain P (proportional)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool qCurrentP(float value, bool response = false);

   /**
       @brief Set Q-axis current PID controller gain I.
       @param[in] value    Q-axis current PID controller gain I (integral)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   

   bool qCurrentI(float value, bool response = false);
   /**
       @brief Set Q-axis current PID controller gain D.
       @param[in] value    Q-axis current PID controller gain D (differential)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool qCurrentD(float value, bool response = false);

   /**
       @brief Set Speed PID controller gain P.
       @param[in] value    Speed PID controller gain P (proportional)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool speedP(float value, bool response = false);

   /**
       @brief Set Speed PID controller gain I.
       @param[in] value    Speed PID controller gain I (integral)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool speedI(float value, bool response = false);

   /**
       @brief Set Speed PID controller gain D.
       @param[in] value    Speed PID controller gain D (differential).
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool speedD(float value, bool response = false);

   /**
       @brief Set Position PID controller gain P 
       @param[in] value    Position PID controller gain P (proportional)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool positionP(float value, bool response = false);

   /**
       @brief Set Position PID controller gain I
       @param[in] value    Position PID controller gain I (integral)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Position integral and differential term are ignored out of positionIDThreshold range. (See positionIDThreshold function)
    */
   bool positionI(float value, bool response = false);

   /**
       @brief Set Position PID controller gain D
       @param[in] value    Position PID controller gain D (differential).
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark Position integral and differential term are ignored out of positionIDThreshold range. (See positionIDThreshold function)
    */
   bool positionD(float value, bool response = false);

   /**
       @brief Set the threshold [radian] to determine Position PID control available range
       @details Position PID control is available within the range determined by this threshold, @n
       while only Position P control is available out of the range. @n
       The conditions are as follows. @n
       @code
       if(|currentPosition - targetPosition| < threshold) // do PID control.
       else {// do P control}
       @endcode 
       @note Position integral and differential gains are ignored out of the range.If you want to always do PID control, @n
       set the this value large enough.     
       @param[in] value        the threshold to determine the available range of PID control. [radian]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool positionIDThreshold(float value, bool response = false);

   /**
       @brief Set the threshold [degree] to determine Position PID control available range
       @details See KeiganMotor::positionIDThreshold.   
       @param[in] value        The threshold determines the available area of PID control. [degree]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool positionIDThresholdDegree(float value, bool response = false);


   /**
       @brief Reset all the PID parameters to default
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() is required to save in flash.
    */
   bool resetPID(bool response = false);
   /* @} */


   /** @name Read data*/
   /* @{ */
   /**
       @brief Read register function
       @param[in] reg          register
       @param[out] *value      value pointer to read
       @param[in] value_len    value length
       @retval true            received data successfully
       @retval false           got an error
       @note                    Use getError().code to get error code.
    */
   bool readRegister(uint8_t reg, uint8_t *value, uint8_t value_len);

   /**
       @brief Read status function
       @details You can see the current status of KeiganMotor. 
       @return status          Status of KeiganMotor (See status_t.)
    */
   status_t readStatus(void);

   /**
     * @brief Read the maximum speed
     * @return float max speed [radian/second]
     */
   float readMaxSpeed(void);   

   /**
     * @brief Read the minimum speed
     * @return float min speed [radian/second]
     */
   float readMinSpeed(void); 


   /**
     * @brief Read the curveType fo motion control. @n See #CURVE_TYPE_NONE and #CURVE_TYPE_TRAPEZOID.
     * @return curveType @n 
     *   - #CURVE_TYPE_NONE = 0: No curve. It's cyclic and step function.
         - #CURVE_TYPE_TRAPEZOID = 1: Trapezoidal velocity curve. 
     */
   uint8_t readCurveType(void);   

   /**
     * @brief Read the acceleration value
     * @return float acceleration [radian/second^2]
     */
   float readAcc(void);   

   /**
     * @brief Read the deceleration value
     * @return float deceleration [radian/second^2]
     */
   float readDec(void);   

   /**
     * @brief Read the resister value of max torque (limitation)
     * @return float max torque [N*m]
     * @details You can do torque control during velocity or position control 
     */
   float readMaxTorque(void);

   /**
       @brief Read motor measurement data
       @details Position, velocity and torque values are available from property after sending this command. @n
       In addition to it, you can get degree and rpm values as property of KeiganMotor. .
       @retval true            received data from KeiganMotor successfully
       @retval false           got an error
       @code 
           if(m.readMotorMeasurement()){
               Serial.print(degree: );
               Serial.println(m.degree);
           } else {
               Serial.println("error");
           }
       @endcode
    */
   bool readMotorMeasurement(void);

   /**
       @brief Read IMU measurement data
       @details IMU values are available after sending this command. See imu_t.
       @code 
           if(m.readIMUMeasurement()){
               Serial.print(gyro-x: );
               Serial.println(m.imu.gyroX);
           } else {
               Serial.println("error");
           }
       @endcode
       @retval true            received data from KeiganMotor successfully
       @retval false           got an error
     */
   bool readIMUMeasurement(void);

   /**
     * @brief Read the KeiganMotor device name 
     * @details The name is like "KM-1U AI09#3R9"
     * @note Please set char *name size more than 14 bytes.
     */
   bool readDeviceName(char *name);

   /**
     * @brief Read the device information 
     * @todo
     */
   bool readDeviceInfo(uint8_t type, char *str);

   /**
     * @brief Read the latest error
     * @details The response data is the same formart as error response of write command.
     * @return the latest error (see error_t)
     * @note The difference between getError() and this function is that @n
     * getError() just returns the retained error data by master (Arduino), while readError()
     * send a command to request the latest error data that KeiganMotor retains.
     */
   error_t readError(void);
   /* @} */

   /** @name Read PID controller parameters*/
   /* @{ */
   /**
     * @brief Read the resister value of Current PID controller P gain
     * @return float gain
     */
   float readQCurrentP(void);

   /**
     * @brief Read the resister value of Current PID controller I gain
     * @return float gain
     */
   float readQCurrentI(void);

   /**
     * @brief Read the resister value of Current PID controller D gain
     * @return float gain
     */
   float readQCurrentD(void);


   /**
     * @brief Read the resister value of Speed PID controller P gain
     * @return float gain
     */
   float readSpeedP(void);

   /**
     * @brief Read the resister value of Speed PID controller I gain
     * @return float gain
     */
   float readSpeedI(void);

   /**
     * @brief Read the resister value of Speed PID controller D gain
     * @return float gain
     */
   float readSpeedD(void);


   /**
     * @brief Read the resister value of Position PID controller P gain
     * @return float gain
     */
   float readPositionP(void);

   /**
     * @brief Read the resister value of Position PID controller I gain
     * @return float gain
     */
   float readPositionI(void);

   /**
     * @brief Read the resister value of Position PID controller D gain
     * @return float gain
     */
   float readPositionD(void);

   /**
     * @brief Read the threshold [radian] to determine Position PID control available range
     * @return float threshold [radian]
     */
   float readPositionIDThreshold(void);

   /**
     * @brief Read the threshold [degree] to determine Position PID control available range
     * @return float threshold [degree]
     */
   float readPositionIDThresholdDegree(void);

   /* @} */

   /** @name Enable or Disable motor action */
   /* @{ */
   /**
       @brief Disable motor action.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool disable(bool response = false); // Disable Motor Action

   /**
       @brief Enable motor action.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool enable(bool response = false);
   /* @} */

   /** @name Motor control Settings*/
   /* @{ */
   /**
       @brief Set max speed function to limit absolute value of velocity
       @param[in] value        float speed to set [radian/second] 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool maxSpeed(float value, bool response = false);

   /**
       @brief Set min speed. It is used in case of "prepare playback motion"
       @param[in] value        float speed to set [radian/second] 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool minSpeed(float value, bool response = false); //TODO // Set min speed [rad/s]

   /**
       @brief Set curve type for motion control. 
       @param[in] type         CurveType to set @n 
     *      - #CURVE_TYPE_NONE = 0: No curve. It's cyclic and step function.
            - #CURVE_TYPE_TRAPEZOID = 1: Trapezoidal velocity curve.  
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool curveType(uint8_t type, bool response = false); 

   /**
       @brief Set acceleration value. It is ignored when curve type is 0(CURVETYPE_NONE).
       @param[in] value        float acceleration to set [radian/second^2] 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool acc(float value, bool response = false); // Set acceleration [rad/s^2]

   /**
       @brief Set deceleration value. It is ignored when curve type is 0(CURVETYPE_NONE).
       @param[in] value        float deceleration to set [radian/second^2] 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool dec(float value, bool response = false); // Set deceleration [rad/s^2]

   /**
       @brief Set Max torque value to limit current not to be above the torque.
       @param[in] value        float max torque to set [N*m] 
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool maxTorque(float value, bool response = false); // Set max torque

   /**
       @brief Set anglular speed [radian/second]  
       @param[in] value        anglular speed [radian/second]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool speed(float value, bool response = false);

   /**
       @brief Set anglular speed [rotation/minute] (rpm)  
       @param[in] rpm          anglular speed [rotation/minute]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool speedRpm(float rpm, bool response = false); // Set motor speed [rpm]

   /**
       @brief Set the current position as a certain position [radian]
       @details presetPosition(0) makes the current position as "Zero" point. 
       @param[in] value        preset position [radian]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool presetPosition(float value, bool response = false);

   /**
       @brief Set the current position as a certain position [degree]
       @details presetPositionDegree(30) makes the current position as "30 degree". 
       @param[in] degree       preset position [degree]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool presetPositionDegree(float degree, bool response = false);


   /* @} */

   /** @name Motor action (Velocity Control)
     *
     */
   /* @{ */
   /**
       @brief Run at a certain velocity [radian/second]
       @details This function combines speed and runForward or runReverse function.
       @param[in] value        velocity [radian/second]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool runAtVelocity(float value, bool response = false); // Run motor at velocity [rad/s]

   /**
       @brief Run at a certain velocity [rotation/minute] (rpm)
       @details Plus direction is counter clockwise when viewed from the rotation axis.
       @details This function combines speed and runForward or runReverse function.
       @param[in] rpm          rpm [rotation/minute] (rpm)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool runAtVelocityRpm(float rpm, bool response = false); // Run motor at velobity [rpm]

   /**
       @brief Run forward
       @details It uses preset speed value by function speed()
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool runForward(bool response = false); // Run motor forward (Direction Counter Clock Wise)

   /**
       @brief Run Reverse
       @details It uses preset speed value by function speed()
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool runReverse(bool response = false); // Run motor reverse (Direction Clock Wise)

   /**
       @brief Stop
       @details Keep velocity 0 by velocity control
       @details Forward direction is counter clockwise when viewed from the rotation axis.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool stop(bool response = false); // Stop and hold torque

   /* @} */

   /** @name Motor action (Position Control) */
   /* @{ */

   /**
       @brief Move to an absolute position [radian]
       @details It uses preset speed value by function speed()
       @param[in] position     position [radian]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool moveToPosition(float position, bool response = false);

   /**
       @brief Move to an absolute position [degree]
       @details It uses preset speed value by function speed()
       @param[in] degree       degree [degree]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool moveToPositionDegree(float degree, bool response = false);

   /**
       @brief Move by a distance (move to an relative position) [radian]
       @details It uses preset speed value by function speed()
       @param[in] distance     distance [radian]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool moveByDistance(float distance, bool response = false);

   /**
       @brief Move by a distance (move to an relative position) [degree]
       @details It uses preset speed value by function speed()
       @param[in] degree     degree [degree]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool moveByDistanceDegree(float degree, bool response = false);

   /* @} */

   /** @name Motor action (others) */
   /* @{ */
   /**
       @brief De-energize motor (make motor free)
       @details It keeps viscosity a little bit. You should use disable() to turn off rotational resistance completely.
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool free(bool response = false);
   /* @} */

   /** @name Queue */
   /* @{ */
   /**
       @brief  Wait for execution of next commands for time [msec].
       @details KeiganMotor has a FIFO queue inside, and this function make its execution paused for the time. @n
       This function uses the internal timer of KeiganMotor.
       @param[in] time         time to stop the queue [milliseconds]
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool wait(uint32_t time, bool response = false);
   /* @} */


   /** @name Others */
   /* @{ */  
   /**
       @brief Set I2C slave address.
       @param[in] address      I2C address (0x00~0x7F)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
       @remark saveAllRegisters() and reboot() is required to reflact the address value.
       @code
           if(m.i2cSlaveAddress(0x30)){
               m.saveAllRegisters();
               delay(3000);
               m.reboot();
           }
       @endcode
    */
   bool i2cSlaveAddress(uint8_t address, bool response = false);
   
   /**
       @brief Reboot KeiganMotor
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool reboot(bool response = false);

   /**
       @brief Start motor measurement timer
       @details This function will start motor measurement notfication
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool startMotorMeasurement(bool response = false);

   /**
       @brief Stop motor measurement timer
       @details This function will stop motor measurement notfication
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool stopMotorMeasurement(bool response = false);

   /**
       @brief Start IMU measurement timer
       @details This function will start IMU measurement notfication
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool startIMUMeasurement(bool response = false);

   /**
       @brief Stop IMU measurement timer
       @details This function will stop IMU measurement notfication
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool stopIMUMeasurement(bool response = false);

   /* @} */  

   /** @name LED */
   /* @{ */  
   /**
       @brief Set LED lit
       @param[in] state        LedState @n
         - #LED_STATE_OFF = 0: off 消灯 
         - #LED_STATE_ON_SOLID = 1: on solid 点灯 
         - #LED_STATE_ON_FLASH = 2: on flashing 点滅
       @param[in] r            Red brightness (0-255)
       @param[in] g            Green brightness (0-255)       
       @param[in] b            Blue brightness (0-255)
       @param[in] response     true if you get response after sending command, default: false
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool led(uint8_t state, uint8_t r, uint8_t g, uint8_t b, bool response = false);
   /* @} */

   /** @name private */
   /* @{ */  
private:
   void init(uint8_t address, int clock = 1000000);
   uint16_t appendID(uint8_t *data);
   /**
       @brief Write float command function
       @param[in] command      command
       @param[in] value        float value to write
       @param[in] response     true if you require response after sending command
       @retval true            received response from KeiganMotor successfully
       @retval false           got an error
    */
   bool writeFloat(uint8_t cmd, float val, bool response = false);
   float readFloat(uint8_t reg);
   uint8_t readByte(uint8_t reg);
   uint32_t readUint32(uint8_t reg);
   uint8_t _address;    // I2C address
   uint16_t _commandID; // commandID
   /* @} */ 
};

#endif // km1_i2c_h
