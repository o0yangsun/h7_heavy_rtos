/**
  ******************************************************************************
  * @file    robot.h
  * @brief   整车参数配置：CAN 分配、机械参数和控制参数
  ******************************************************************************
  */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>

/* ------------------------------ CAN 总线分配 ------------------------------ */
/* CAN1：底盘 4 个麦轮
 * CAN2：yaw 电机、拨弹电机、特化机构电机、超级电容
 * CAN3：pitch 电机、3 个摩擦轮电机 */
#define CAN_CHASSIS                 1U
#define CAN_YAW                     2U
#define CAN_TRIGGER                 2U
#define CAN_SPECIAL                 2U
#define CAN_SUPERCAP                2U
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

/* 单轮电机转速上限，正解与限幅都用它 */
#define CHS_MAX_WHEEL_RPM           9000.0f

/* 单轮配置：电机 ID、安装方向、运动学系数
 * sign   安装方向修正，同时作用于速度反馈和电流输出
 * kx/ky/kw  vx/vy/vw 对该轮转速的贡献，kw 已含逐轮自转修正 */
typedef struct
{
    uint8_t id;
    float   sign;
    float   kx;
    float   ky;
    float   kw;
} chs_wheel_cfg_t;

/* 顺序 ID1~ID4：左前、右前、右后、左后。
 * ID1/ID2 装反取 -1；ID2/ID4 纯自转方向与期望相反，kw 相应取 -1。
 * 改了任意一行都要重新确认前进、平移、自转三个方向。 */
#define CHS_WHEEL_CFG                                                       \
{                                                                           \
    {CHS_WHEEL_ID_FL, -1.0f, +1.0f, -1.0f, -1.0f},  /* ID1 左前 */          \
    {CHS_WHEEL_ID_FR, -1.0f, +1.0f, +1.0f, -1.0f},  /* ID2 右前 */          \
    {CHS_WHEEL_ID_RR, +1.0f, +1.0f, -1.0f, +1.0f},  /* ID3 右后 */          \
    {CHS_WHEEL_ID_RL, +1.0f, +1.0f, +1.0f, +1.0f},  /* ID4 左后 */          \
}

#define CHS_MAX_VX                  5000.0f     /* mm/s */
#define CHS_MAX_VY                  5000.0f     /* mm/s */
#define CHS_MAX_VW                  8.0f        /* rad/s */

/* 底盘速度环 */
#define CHS_SPEED_KP                3.0f
#define CHS_SPEED_KI                2.0f
/* 微分增益不宜过大：1 ms 周期下 Kd/dt = 1000*Kd，会把转速噪声放大成电流抖动 */
#define CHS_SPEED_KD                0.02f
#define CHS_SPEED_D_LPF_RC          0.005f      /* 微分一阶低通时间常数，秒 */
#define CHS_SPEED_MAX               16384.0f
/* 静摩擦补偿：有目标时输出的最小幅值，低于此值车轮启不动 */
#define CHS_SPEED_MIN_OUT           600.0f
/* 低于该转速视为堵转，才施加最小输出补偿 */
#define CHS_SPEED_STATIC_RPM        30.0f
/* 电刹模式下转速低于此值即认为静止，清积分避免噪声累积 */
#define CHS_LOCK_IDLE_RPM           3.0f

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
#define CMD_RC_ACC_XY               2.0f        /* mm/s per ms */
#define CMD_RC_ACC_W                0.005f      /* rad/s per ms */

/* 起步软启动：指令零延时，只限制起步阶段的电流上限。
 * 目标从 0 变为非 0 后 CHS_LAUNCH_MS 内，输出上限从 RATIO 线性放开到 100% */
#define CHS_LAUNCH_MS               120U
#define CHS_LAUNCH_RATIO            0.55f

/* 输出斜率限制：每 1ms 电流指令最大变化量（满量程 16384，400 约 41ms 爬满）。
 * 只压电流阶跃，不改变目标速度与稳态，起步冲击被摊平 */
#define CHS_OUT_SLEW                400.0f

/* 3508 功率模型（raw 单位，取自开源舵轮步兵参数）
 * P = Kt·I·ω + k1·ω² + k2·I² + C
 *   I 为 PID 输出原始值（±16384），ω 为转子转速 rpm */
#define CHS_PWR_KT_RAW              1.575e-06f      /* 机械功率系数，按本车高速点标定 */
#define CHS_PWR_K1                  1.453e-07f      /* 铁损/摩擦系数 */
#define CHS_PWR_K2                  1.23e-07f       /* 铜损系数 */
#define CHS_PWR_C                   1.4f            /* 单电机静态损耗 W，待机实测标定 */

/* -------------------------------- 功率参数 -------------------------------- */
#define POWER_PERIOD_MS             10U
#define POWER_LIMIT_W               47.0f       /* 主控硬上限，比较对象是电池侧 Pbat */
#define SUPERCAP_BUDGET_W           47.0f       /* 下发给超电板的电池侧功率预算 W，板端目标 = 45W */
#define POWER_BUFFER_MAX_J          60.0f
/* 功率一阶低通系数：超电上报的 Pbat+Pcap 在充放电切换时会跳变，
 * 10ms 周期下 alpha=0.1 约等于 100ms 时间常数 */
#define POWER_LPF_ALPHA             0.1f

/* -------------------------------- 姿态参数 -------------------------------- */
#define INS_PERIOD_MS               1U

/* IMU 安装矩阵：机体坐标 x 前、y 左、z 上，把芯片坐标旋转到机体坐标。
 * 默认按芯片与机体同向贴装取单位阵，换装或贴反时按实际方向改写。 */
#define INS_MOUNT_MATRIX                    \
{                                           \
    {+1.0f,  0.0f,  0.0f},                  \
    { 0.0f, +1.0f,  0.0f},                  \
    { 0.0f,  0.0f, +1.0f},                  \
}

/* EKF 参数，与 hero 工程一致 */
#define INS_EKF_Q1                  10.0f       /* 四元数过程噪声 */
#define INS_EKF_Q2                  0.001f      /* 零偏过程噪声 */
#define INS_EKF_R                   1000000.0f  /* 加速度量测噪声 */
#define INS_EKF_LAMBDA              1.0f        /* 渐消因子 */
#define INS_EKF_ACC_LPF             0.0f        /* 加速度低通时间常数，秒 */

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
