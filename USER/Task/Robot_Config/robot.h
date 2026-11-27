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
/* 云台反馈源：1 = 陀螺仪角度/角速度（锁头、抗底盘扰动），0 = 电机编码器。
 * 云台尚未安装 IMU，暂用编码器；装好后改回 1 即可恢复锁头。 */
#define GIM_USE_IMU_FEEDBACK        0

#define GIM_YAW_BUS                 CAN_YAW
#define GIM_YAW_ID                  3U          /* GM6020 直驱，反馈帧 0x207 */
#define GIM_PITCH_BUS               CAN_PITCH
#define GIM_PITCH_ID                5U          /* M3508 经 C620，反馈帧 0x205 */
/* M3508 直驱云台俯仰轴，无减速箱，转子即输出轴 */
#define GIM_PITCH_GEAR_RATIO        1.0f

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

/* pitch 速度环：M3508 直驱，转子转速闭环，输出电流 ±16384。
 * 拨轮直接给目标角速度，回中即停，不依赖角度反馈。
 * 直驱时 30°/s 仅对应 5 rpm，误差量级很小，增益需远高于带减速箱的场合。 */
#define PITCH_ANGLE_KP              10.0f       /* 速度模式下不使用，保留供以后扩展 */
#define PITCH_ANGLE_MAX             200.0f
#define PITCH_SPEED_KP              800.0f
#define PITCH_SPEED_KI              50.0f
#define PITCH_SPEED_ILIM            8000.0f
#define PITCH_MAX_OUT               16384.0f     /* 3508 电流量程 */
#define PITCH_GRAVITY_FF            0.0f

/* 拨轮控制 pitch 角速度：满偏对应的输出轴角速度。
 * DR16 拨轮实测输出 -660~660，与摇杆同量程。 */
#define PITCH_WHEEL_MAX_DPS         30.0f
#define PITCH_WHEEL_RAW_MAX         660.0f
#define PITCH_WHEEL_DEADBAND        60      /* 拨轮回中抖动可达数十，死区取大一些 */

/* 开环调试：拨轮满偏对应的电流占空，无反馈，空载转速由负载决定 */
#define PITCH_OPENLOOP_CURRENT      1000.0f

/* 开环限速：静摩擦被突破后负载骤降，小电流也能把电机推到数千 rpm。
 * 达到阈值的 DERATE 比例后开始线性降流，到阈值时降到 0。 */
#define PITCH_OPENLOOP_MAX_RPM      200.0f
#define PITCH_OPENLOOP_DERATE       0.6f

/* 回中阻尼刹车：拨轮回中时按转速反向出力，抵抗外力转动。
 * 增益远低于速度环，避免转速反馈量化噪声被放大成自激。 */
#define PITCH_BRAKE_KP              25.0f
#define PITCH_BRAKE_RPM_MIN         5.0f
#define PITCH_BRAKE_MAX             3000.0f

/* pitch 当前整定结果（M3508 直驱，无减速箱）：
 *   PITCH_OPENLOOP_CURRENT   1000  恒定电流，不随拨杆量变化
 *   PITCH_OPENLOOP_MAX_RPM   200   实测稳态转速，约 1200 °/s
 *   PITCH_OPENLOOP_DERATE    0.6   120 rpm 起线性降流，200 rpm 断流
 *   PITCH_WHEEL_DEADBAND     60    拨轮回中抖动可达数十，死区需大于它
 *   PITCH_BRAKE_KP           25    回中阻尼，手转时产生反向阻力
 *   GIM_PITCH_SIGN          -1.0   方向已上机确认
 * 开环无限速能力，突破静摩擦后会冲高，故必须保留降流段；
 * 实测转速在 170~240 rpm 间起伏，属降流环路的固有波动。
 * 若需真正稳速或定位，应改用速度/角度闭环，增益取 50~80 量级。 */

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

/* IMU 加热丝：TIM3_CH4 (PB1)，目标温度为 BMI088 的最佳工作点 */
#define IMU_HEAT_TARGET_C           40.0f       /* 目标温度 ℃ */
#define IMU_HEAT_HYST_C             0.5f        /* 滞环宽度 ℃ */
#define IMU_HEAT_KP                 20.0f       /* 误差 1℃ 对应 20% 占空比 */
#define IMU_HEAT_KI                 2.0f
#define IMU_HEAT_DUTY_MAX           60.0f       /* 占空比上限 %，限制发热功率 */
#define IMU_HEAT_DT_S               0.1f       /* 调用周期，秒 */
#define IMU_HEAT_VALID_MIN_C        5.0f        /* 低于此温度视为读数无效 */
#define IMU_HEAT_PERIOD_MS          100U        /* 温控更新周期 */

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

/* 摩擦轮：CAN3 上三个 M3508，ID1~ID3，按输出轴转速闭环。
 * 反馈为转子转速，除以减速比得到输出轴转速。 */
#define FRIC_NUM                    3U
#define FRIC_GEAR_RATIO             (3591.0f / 187.0f)

#define FRIC_ID_1                   1U
#define FRIC_ID_2                   2U
#define FRIC_ID_3                   3U

/* 安装方向修正：同时作用于转速反馈和电流输出。
 * 三个摩擦轮按左右对称安装，右轮与中轮反装取 -1。
 * 实际转向以台架观察为准，与出弹方向不符时改对应符号。 */
#define FRIC_DIR_1                  (-1.0f)
#define FRIC_DIR_2                  (+1.0f)
#define FRIC_DIR_3                  (+1.0f)

/* 转子 8600 rpm 对应输出轴约 448 rpm */
#define FRIC_MAX_OUT_RPM            450.0f

/* 速度环参数取自 hero_rtos 的实测整定值，反馈为转子 rpm */
#define FRIC_SPEED_KP               18.0f
#define FRIC_SPEED_KI               0.0f
#define FRIC_SPEED_KD               0.001f
#define FRIC_SPEED_ILIM             2000.0f
#define FRIC_SPEED_MAX              16384.0f
/* 有目标时的最小输出幅值，低于此值轮子启不动 */
#define FRIC_SPEED_MIN_OUT          500.0f
#define FRIC_SPEED_STATIC_RPM       20.0f

/* 台架：sw1 上拨后三个轮定速转动，用于确认转向和转速 */
#define FRIC_BENCH_OUT_RPM          300.0f
#define FRIC_BENCH_START_DELAY_MS   4000U
/* 启动后从 0 线性升到目标转速的时间，避免电流突变 */
#define FRIC_SPINUP_MS              8000U

/* 拨弹电机：CAN2 上的 M3508，ID1，反馈帧 0x201。
 * CAN2 上 yaw 为 GM6020 ID3（0x207），控制帧 0x1FF；拨弹走 0x200，互不冲突。
 * 参数取自 hero_rtos 实测整定值。 */
#define TRIGGER_BUS                 CAN_TRIGGER
#define TRIGGER_ID                  1U
#define TRIGGER_GEAR_RATIO          (3591.0f / 187.0f)
/* 拨弹盘转向修正：目标角与反馈同在物理域，符号只作用于最终电流输出。
 * 取 -1 表示正电流驱动拨弹盘反转，与原装方向相反时使用。 */
#define TRIGGER_FORWARD_SIGN        (-1.0f)

/* 单发拨弹盘的机械转角，换算到电机轴需乘减速比 */
#define TRIGGER_SHOT_DEG            120.0f

/* 角度环：输入电机轴累计角度，输出目标转速 */
#define TRIGGER_ANGLE_KP            2.0f
#define TRIGGER_ANGLE_MAX           10000.0f
/* 速度环：输入转子 rpm，输出电流 */
#define TRIGGER_SPEED_KP            5.0f
#define TRIGGER_SPEED_KI            3.0f
#define TRIGGER_SPEED_ILIM          1500.0f
#define TRIGGER_SPEED_MAX           16384.0f

/* 连发转速与反转到位的判定容差 */
#define TRIGGER_AUTO_RPM            2000.0f
#define TRIGGER_POS_TOL_DEG         30.0f

/* 目标与反馈角度偏差超过该值视为状态异常，重置目标止损。
 * 单发步长为 120°×19.2≈2304°，取 8000° 留出三倍余量。 */
#define TRIGGER_TARGET_LIMIT_DEG    8000.0f

/* 目标角允许领先实际角度的发数。连续推杆时目标会不断累加，
 * 限制领先量可避免电机长时间高速追击一个很远的目标。 */
#define TRIGGER_TARGET_LEAD_SHOTS   2.0f

/* ch4 摇杆触发阈值：上推单发、下推反转、中间保持。
 * REARM 是重新武装阈值，必须回落到该值以下才允许下一次单发，
 * 否则摇杆在 SINGLE 附近抖动会连续累加目标角。 */
#define TRIGGER_CH_SINGLE           400
#define TRIGGER_CH_REARM            200
#define TRIGGER_CH_REVERSE          (-400)
/* 两次单发的最小间隔，防止机械抖动引起的重复触发 */
#define TRIGGER_MIN_INTERVAL_MS     250U

/* -------------------------------- 串口参数 -------------------------------- */
#define USART_PERIOD_MS             1U
#define DBUS_INJECT_PERIOD_MS       10U

/* -------------------------------- 电机参数 -------------------------------- */
#define DJ_MOTOR_MAX_COUNT          12U
#define DJ_MOTOR_OFFLINE_MS         100U

#endif /* ROBOT_H */
