/**
  ******************************************************************************
  * @file    robot.h
  * @brief   整车参数配置：CAN 分配、机械参数和控制参数
  ******************************************************************************
  */

#ifndef ROBOT_H
#define ROBOT_H

/* ------------------------------ CAN 总线分配 ------------------------------ */
/* CAN1：底盘 4 个麦轮
 * CAN2：yaw 电机、拨弹电机、特化机构电机
 * CAN3：pitch 电机、3 个摩擦轮电机 */
#define CAN_CHASSIS                 1U
#define CAN_YAW                     2U
#define CAN_TRIGGER                 2U
#define CAN_SPECIAL                 2U
#define CAN_PITCH                   3U
#define CAN_FRICTION                3U

/* -------------------------------- 底盘参数 -------------------------------- */
#define CHS_WHEEL_NUM               4U
#define CHS_WHEEL_PERIMETER         478.0f      /* mm */
#define CHS_LENGTH_A                238.0f      /* mm，底盘长的一半 */
#define CHS_LENGTH_B                233.0f      /* mm，底盘宽的一半 */
#define CHS_GEAR_RATIO              (1.0f / 19.0f)
#define CHS_WHEEL_TYPE              1           /* 0：45 度全向轮，1：麦轮 */

/* 数组下标与电机 ID 一一对应：ID1 左前、ID2 右前、ID3 右后、ID4 左后 */
#define CHS_IDX_FL                  0U          /* ID1 左前 */
#define CHS_IDX_FR                  1U          /* ID2 右前 */
#define CHS_IDX_RR                  2U          /* ID3 右后 */
#define CHS_IDX_RL                  3U          /* ID4 左后 */

#define CHS_WHEEL_ID_FL             1U
#define CHS_WHEEL_ID_FR             2U
#define CHS_WHEEL_ID_RR             3U
#define CHS_WHEEL_ID_RL             4U
#define CHS_WHEEL_ID                {CHS_WHEEL_ID_FL, CHS_WHEEL_ID_FR, \
                                     CHS_WHEEL_ID_RR, CHS_WHEEL_ID_RL}

/* 按 ID 顺序的方向修正：ID1、ID2、ID3、ID4
 * ID1/ID2 装反，输出和速度反馈同时取反 */
#define CHS_WHEEL_SIGN              {-1.0f, -1.0f, 1.0f, 1.0f}

/* 自转分量逐轮方向修正，顺序 ID1~ID4，标准麦轮全部为 +1
 * 实测 ID2、ID4 纯自转方向与期望相反，取 -1；只作用于 vw，不影响 vx/vy */
#define CHS_WHEEL_VW_SIGN           {+1.0f, -1.0f, +1.0f, -1.0f}

#define CHS_MAX_VX                  5000.0f     /* mm/s */
#define CHS_MAX_VY                  5000.0f     /* mm/s */
#define CHS_MAX_VW                  8.0f        /* rad/s */

/* 底盘速度环 */
#define CHS_SPEED_KP                3.0f
#define CHS_SPEED_KI                0.0f
#define CHS_SPEED_KD                0.1f
#define CHS_SPEED_MAX               16384.0f

/* 底盘台架测试 */
#define CHS_BENCH_WHEEL_RPM         800.0f
#define CHS_BENCH_STAGE_MS          3000U
#define CHS_BENCH_START_DELAY_MS    5000U

/* -------------------------------- 云台参数 -------------------------------- */
#define GIM_YAW_BUS                 CAN_YAW
#define GIM_YAW_ID                  1U
#define GIM_PITCH_BUS               CAN_PITCH
#define GIM_PITCH_ID                1U

#define GIM_USE_FIXED_CENTER        0
#define GIM_YAW_CENTER_ECD          0U
#define GIM_PITCH_CENTER_ECD        0U

#define GIM_YAW_SIGN                (+1.0f)
#define GIM_PITCH_SIGN              (-1.0f)

#define GIM_PITCH_MIN_DEG           (-35.0f)
#define GIM_PITCH_MAX_DEG           (25.0f)
#define GIM_YAW_TARGET_LIMIT        170.0f

#define GIM_HOME_TIME_MS            1500U
#define GIM_HOME_RATE_DPS           90.0f
#define GIM_HOME_TOL_DEG            0.5f
#define GIM_ECD_PER_DEG             (8192.0f / 360.0f)

/* yaw 串级参数 */
#define YAW_ANGLE_KP                8.0f
#define YAW_ANGLE_MAX               250.0f      /* 外环输出上限，°/s */
#define YAW_SPEED_KP                220.0f
#define YAW_SPEED_KI                12.0f
#define YAW_SPEED_ILIM              8000.0f
#define YAW_MAX_VOLT                20000.0f

/* pitch 串级参数 */
#define PITCH_ANGLE_KP              10.0f
#define PITCH_ANGLE_MAX             200.0f
#define PITCH_SPEED_KP              200.0f
#define PITCH_SPEED_KI              14.0f
#define PITCH_SPEED_ILIM            8000.0f
#define PITCH_MAX_VOLT              18000.0f
#define PITCH_GRAVITY_FF            0.0f

/* -------------------------------- 遥控参数 -------------------------------- */
#define CMD_PERIOD_MS               1U
#define CMD_RC_TIMEOUT_MS           500U

#define RC_MAX_VALUE                660.0f

#define CMD_RC_CH_VX                2           /* ch1 右侧左右 */
#define CMD_RC_CH_VY                1           /* ch2 右侧上下 */
#define CMD_RC_CH_VW                0           /* ch3 左侧左右 */
#define CMD_RC_SIGN_VX              (-1.0f)
#define CMD_RC_SIGN_VY              (+1.0f)     /* 右打杆 = 右移 */
#define CMD_RC_SIGN_VW              (+1.0f)     /* 左打杆 = 逆时针 */

#define CMD_RC_RATIO_X              0.5f
#define CMD_RC_RATIO_Y              0.8f
#define CMD_RC_RATIO_R              0.85f
#define CMD_MAX_VX                  5000.0f     /* mm/s */
#define CMD_MAX_VY                  5000.0f     /* mm/s */
#define CMD_MAX_VR                  8.0f        /* rad/s */
#define CMD_SPIN_VW                 3.0f        /* rad/s */
#define CMD_RC_ACC_XY               4.0f        /* mm/s per ms */
#define CMD_RC_ACC_W                0.008f      /* rad/s per ms */

/* -------------------------------- 功率参数 -------------------------------- */
#define POWER_PERIOD_MS             10U
#define POWER_LIMIT_W               80.0f
#define POWER_BUFFER_MAX_J          60.0f

/* -------------------------------- 姿态参数 -------------------------------- */
#define INS_PERIOD_MS               1U

/* -------------------------------- 裁判参数 -------------------------------- */
#define REFEREE_PERIOD_MS           10U

/* -------------------------------- 传输参数 -------------------------------- */
#define TRANS_PERIOD_MS             33U
#define TRANS_TIMEOUT_MS            200U

/* ------------------------------ 特殊机构参数 ------------------------------ */
#define SPECIAL_PERIOD_MS           20U
#define SPECIAL_PRECHARGE_MS        300U

/* -------------------------------- 发射参数 -------------------------------- */
#define SHOOT_PERIOD_MS             100U

/* -------------------------------- 串口参数 -------------------------------- */
#define USART_PERIOD_MS             1U
#define DBUS_INJECT_PERIOD_MS       10U

/* -------------------------------- 电机参数 -------------------------------- */
#define DJ_MOTOR_MAX_COUNT          12U
#define DJ_MOTOR_OFFLINE_MS         100U

#endif /* ROBOT_H */
