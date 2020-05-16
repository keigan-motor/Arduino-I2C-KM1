/**
   @file         Definitions.h
   @brief        Defines for KeiganMotor  
   @date         2020/5/1
   @author       Takashi Tokuda (Keigan Inc.)                
*/

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

/** @name Curve Type
 * 
*/
/* @{ */
#define CURVE_TYPE_NONE 0 /**< No curve. It's cyclic and step function. */
#define CURVE_TYPE_TRAPEZOID 1 /**< Trapezoidal velocity curve.  */
/* @} */

/** @name Interface bit flag */
/* @{ */
#define INTERFACE_BIT_BLE        0x01 //  (0000 0001) Bluetooth Low Energy
#define INTERFACE_BIT_LINKAGE    0x02 //  (0000 0010) Linkage
#define INTERFACE_BIT_UART1      0x08 //  (0000 1000) UART1 (USB)
#define INTERFACE_BIT_I2C        0x10 //  (0001 0000) I2C
#define INTERFACE_BIT_DIGITAL_IO 0x20 //  (0010 0000) Digital IO
#define INTERFACE_BIT_UART2      0x40 //  (0100 0000) UART2
#define INTERFACE_BIT_BUTTON     0x80 //  (1000 0000) BUTTON
/* @} */

/** @name Motor Control Mode */
/* @{ */
#define MOTOR_CONTROL_MODE_NONE 0 /**< idle mode (No action)*/
#define MOTOR_CONTROL_MODE_VELOCITY 1 /**< Velocity Control */
#define MOTOR_CONTROL_MODE_POSITION 2 /**< Position Control */
#define MOTOR_CONTROL_MODE_TORQUE 3 /**< Torque Control */
/* @} */

/** @name Safe Run Setting Option */
/* @{ */
#define SAFE_RUN_TIMEOUT_FREE 0 /**< free @n 非励磁状態 */
#define SAFE_RUN_TIMEOUT_DISABLE 1 /**< disable 動作不許可状態 */
#define SAFE_RUN_TIMEOUT_STOP 2 /**< run_at_velocity(0)  速度制御ゼロ */
#define SAFE_RUN_TIMEOUT_POS_FIX 3 /**< fix position at the point その場で位置制御*/
/* @} */

/** @name Led State */
/* @{ */
#define LED_STATE_OFF 0 /**< off 消灯 */
#define LED_STATE_ON_SOLID 1 /**< on solid 点灯 */
#define LED_STATE_ON_FLASH 2 /**< on flashing 点滅 */
/* @} */

/** @name Flash State */
/* @{ */
#define FLASH_STATE_READY 0	/**< ready, not busy @n 準備OK、ビジーでない、アイドル状態  */
#define FLASH_STATE_TEACHING_PREPARE 1 /**< preparing for teaching motion @n ティーチング準備中 */
#define FLASH_STATE_TEACHING_DOING 2 /**< teaching motion @n ティーチング実行中 */
#define FLASH_STATE_PLAYBACK_PREPARE 3 /**< preparing for playback motion @n プレイバック準備中 */
#define FLASH_STATE_PLAYBACK_DOING 4 /**< executing playback motion @n プレイバック実行中 */
#define FLASH_STATE_PLAYBACK_PAUSING 5 /**< pausing playback motion @n プレイバック一時停止中 */
#define FLASH_STATE_TASKSET_RECORDING 6 /**< recording taskset @n タスクセット記録中 */
#define FLASH_STATE_TASKSET_DOING 7 /**< doing taskset @n タスクセット実行中 */
#define FLASH_STATE_TASKSET_PAUSING 8 /**< pausing taskset @n タスクセット一時停止中*/
#define FLASH_STATE_IMU 20 /**< using IMU (shared resource with the flash) @n  IMU使用中（フラッシュと共通リソース）*/
/* @} */

/** @name Command (Registers) */
/* @{ */
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

#define CMD_REG_POSITION_I 0x1F
#define CMD_REG_POSITION_I_LEN 9

#define CMD_REG_POSITION_D 0x20
#define CMD_REG_POSITION_D_LEN 9

#define CMD_REG_POS_ID_THRESHOLD 0x21
#define CMD_REG_POS_ID_THRESHOLD_LEN 9

#define CMD_REG_RESET_PID 0x22
#define CMD_REG_RESET_PID_LEN 5

#define CMD_REG_LIMIT_CURRENT 0x25
#define CMD_REG_LIMIT_CURRENT_LEN 9

#define CMD_REG_MOTOR_MEAS_INTVL 0x2C
#define CMD_REG_MOTOR_MEAS_INTVL_LEN 6

#define CMD_REG_MOTOR_MEAS_BY_DEFAULT 0x2D
#define CMD_REG_MOTOR_MEAS_BY_DEFAULT_LEN 6

#define CMD_REG_NOTIFY_POS_ARRIVAL 0x2B
#define CMD_REG_NOTIFY_POS_ARRIVAL_LEN 14

#define CMD_REG_INTERFACE 0x2E
#define CMD_REG_INTERFACE_LEN 6

#define CMD_REG_RESPONSE 0x30
#define CMD_REG_RESPONSE_LEN 6

#define CMD_REG_SAFE_RUN 0x31
#define CMD_REG_SAFE_RUN_LEN 11

#define CMD_REG_LIMIT_CURRENT 0x33
#define CMD_REG_LIMIT_CURRENT_LEN 9

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

#define CMD_REG_RESET_REGISTER 0x4E
#define CMD_REG_RESET_REGISTER_LEN 6

#define CMD_REG_RESET_ALL_REGISTERS 0x4F
#define CMD_REG_RESET_ALL_REGISTERS_LEN 5

#define CMD_REG_I2C_SLAVE_ADDR 0xC0
#define CMD_REG_I2C_SLAVE_ADDR_LEN 6

/* @} */

// @name Command (Motor Disable and Enable)
/* @{ */
#define CMD_DRIVER_DISABLE 0x50
#define CMD_DRIVER_DISABLE_LEN 5

#define CMD_DRIVER_ENABLE 0x51
#define CMD_DRIVER_ENABLE_LEN 5
/* @} */

/** @name Command (Motor Action) */
/* @{ */
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
/* @} */

/** @name Command (Command Queue) */
/* @{ */
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
/* @} */

/** @name Command (Taskset) */
/* @{ */
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
/* @} */

/** @name Command (Teaching and Playback Motion) */
/* @{ */
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

#define CMD_DT_READ_MOTION 0xB7
#define CMD_DT_READ_MOTION_LEN 7

#define CMD_DT_WRITE_MOTION_POSITION 0xB8
#define CMD_DT_WRITE_MOTION_POSITION_LEN 9
/* @} */

/** @name Command (Read) */
/* @{ */
#define CMD_READ_DEVICE_NAME 0x46
#define CMD_READ_DEVICE_NAME_LEN 5

#define CMD_READ_DEVICE_INFO 0x47
#define CMD_READ_DEVICE_INFO_LEN 5

#define CMD_READ_STATUS 0x9A
#define CMD_READ_STATUS_LEN 5

#define CMD_READ_MOTOR_MEASUREMENT 0xB4
#define CMD_READ_MOTOR_MEASUREMENT_LEN 5

#define CMD_READ_IMU_MEASUREMENT 0xB5
#define CMD_READ_IMU_MEASUREMENT_LEN 5

#define CMD_READ_ERROR 0xBE
#define CMD_READ_ERROR_LEN 5

/* @} */


/** @name Command (Button settings) */
/* @{ */ 
#define CMD_BUTTON_SETTING 0xBD
#define CMD_BUTTON_SETTING_LEN 6
/* @} */

/** @name Command (Read) */
/* @{ */
#define CMD_LED_SET_LED 0xE0
#define CMD_LED_SET_LED_LEN 9
/* @} */

/** @name Command (Measurement) */
/* @{ */
#define CMD_MOTOR_START_MEASUREMENT 0xE6
#define CMD_MOTOR_START_MEASUREMENT_LEN 5

#define CMD_MOTOR_STOP_MEASUREMENT 0xE7
#define CMD_MOTOR_STOP_MEASUREMENT_LEN 5

#define CMD_IMU_START_MEASUREMENT 0xEA
#define CMD_IMU_START_MEASUREMENT_LEN 5

#define CMD_IMU_STOP_MEASUREMENT 0xEB
#define CMD_IMU_STOP_MEASUREMENT_LEN 5
/* @} */

/** @name Command (Others) */
/* @{ */
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
/* @} */

/** @name Received Data Type and Error*/
/* @{ */
#define RECV_DATA_READ 0x40
// #define RECV_DATA_READ_LEN // depending on register

#define RECV_DATA_ERROR 0xBE
#define RECV_DATA_ERROR_LEN 15

#define RECV_DATA_MOTOR_MEAS 0xB4
#define RECV_DATA_MOTOR_MEAS_LEN 16

#define RECV_DATA_IMU_MEAS 0xB5
#define RECV_DATA_IMU_MEAS_LEN 18

#define ERROR_CODE_CRC_INVALID 200
#define ERROR_CODE_ID_NOT_MATCHED 201
#define ERROR_CODE_INVALID_DATA 202

#define DUMMY_DATA_INDICATE_ERROR 0xFF
/* @} */




#endif // DEFINITIONS_H
