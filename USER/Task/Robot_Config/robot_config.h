/**
  ******************************************************************************
  * @file    robot_config.h
  * @brief   功能模块与调试开关
  ******************************************************************************
  */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/******************************************
 * 1. 功能模块开关
 *****************************************/
#define ENABLE_IMU                      1   /* BMI088 + EKF */
#define ENABLE_INA226                   0   /* INA226 母线采样，需先在 CubeMX 启用 I2C1 */
#define ENABLE_SUPERCAP                 1   /* 超级电容，CAN2 */
#define ENABLE_SUPERCAP_BY_SW2          1   /* 1 = sw2 下档启用超电，其他档位请求停机 */
#define ENABLE_REFEREE                  1   /* 裁判系统，USART1 DMA 空闲中断 */
#define ENABLE_SPECIAL                  0   /* 气泵和泄压阀 */
#define ENABLE_TRANSMISSION             0   /* 上位机链路发送 */
#define ENABLE_RC_DBUS                  1   /* UART5 接收 DBUS */
#define ENABLE_RC_DBUS_RX_INVERT        0   /* 0：外部反相，1：MCU 内部反相 */
#define ENABLE_WIRELESS                 1   /* USART10 接收无线帧 */

/******************************************
 * 2. 诊断打印
 *****************************************/
#define ENABLE_DIAG_CMD                 1
#define ENABLE_DIAG_CHASSIS             1
#define ENABLE_DIAG_GIMBAL              1
#define ENABLE_DIAG_INS                 1
#define ENABLE_DIAG_MOTOR               1
#define ENABLE_DIAG_POWER               1
#define ENABLE_DIAG_REFEREE             1
#define ENABLE_DIAG_SPECIAL             1
#define ENABLE_DIAG_TRANSMISSION        1
#define ENABLE_DIAG_USART               1

/******************************************
 * 3. 台架与调试
 *****************************************/
#define ENABLE_CHASSIS_BENCH            0
#define CHASSIS_CMD_FROM_CMD_TASK       1   /* 1：指令来自 CmdTask，0：手动注入 */

#define GIMBAL_DEBUG_AXIS               1   /* 0：双轴，1：仅 yaw，2：仅 pitch */
#define ENABLE_GIMBAL_BENCH             1

#define ENABLE_MOTOR_BENCH              0
#define ENABLE_MOTOR_LOOPBACK           0
#define ENABLE_MOTOR_CAN_TX_PROBE       0   /* 2：TX 引脚静态显性 */
#define ENABLE_MOTOR_TX_DISABLE         0
#define ENABLE_MOTOR_BUS_ID_PROBE       0
#define ENABLE_MOTOR_MINIMAL_TX         0
#define ENABLE_MOTOR_CAN1_MONITOR       1
#define ENABLE_MOTOR_CAN_SCAN           1

#define ENABLE_CAN_MONITOR              0   /* 只听模式，不发送、不应答 ACK */
#define ENABLE_RC_RAMP                  0   /* 遥控速度斜坡；改成输出软启动后不需要指令斜坡 */

#endif /* ROBOT_CONFIG_H */
