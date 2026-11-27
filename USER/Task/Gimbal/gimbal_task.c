/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @brief   云台任务实现，详见 gimbal_task.h
  ******************************************************************************
  */

#include "gimbal_task.h"
#include "dj_motor.h"
#include "pid.h"
#include "log_uart.h"
#include "cmd_task.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "ins_task.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

/* --------------------------------- 常量/宏 --------------------------------- */

#define AXIS_YAW                0U
#define AXIS_PITCH              1U
#define AXIS_COUNT              2U

#define GIM_RAD2DEG             57.2957795f

/* --------------------------------- 内部状态 -------------------------------- */

typedef struct
{
    dj_motor_object_t *motor;
    pid_obj_t         *pid_angle;
    pid_obj_t         *pid_speed;
    uint16_t           center_ecd;
    float              center_ins_deg;  /**< 回中时刻的陀螺角度（°） */
    bool               center_ready;
    bool               ins_locked;      /**< 已切到陀螺反馈 */
    volatile float     target_deg;      /**< 任务层写，1kHz 回调读 */
    volatile float     target_dps;
    volatile int16_t   out_volt;
    float              sign;            /**< 输出方向修正 */
    uint8_t            index;           /**< AXIS_YAW / AXIS_PITCH */
} gim_axis_t;

typedef struct
{
    uint8_t          bus;
    uint8_t          id;
    dj_motor_type_e  type;          /**< GM6020 电压驱动 / 3508 电流驱动 */
    float            gear;          /**< 转子到输出轴减速比，直驱填 1.0 */
    float            sign;
    float            angle_kp;
    float            angle_max;
    float            speed_kp;
    float            speed_ki;
    float            speed_ilim;
    float            max_out;       /**< 3508 为电流、GM6020 为电压 */
} gim_axis_cfg_t;

static const gim_axis_cfg_t s_axis_cfg[AXIS_COUNT] =
{
    {GIM_YAW_BUS,   GIM_YAW_ID,   DJ_MOTOR_GM6020, 1.0f,                 GIM_YAW_SIGN,
     YAW_ANGLE_KP,   YAW_ANGLE_MAX,   YAW_SPEED_KP,   YAW_SPEED_KI,   YAW_SPEED_ILIM,   YAW_MAX_VOLT},
    {GIM_PITCH_BUS, GIM_PITCH_ID, DJ_MOTOR_M3508,  GIM_PITCH_GEAR_RATIO, GIM_PITCH_SIGN,
     PITCH_ANGLE_KP, PITCH_ANGLE_MAX, PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_ILIM, PITCH_MAX_OUT},
};

static gim_axis_t           s_axis[AXIS_COUNT];
static volatile gimbal_mode_e s_mode = GIMBAL_MODE_RELAX;
static volatile bool        s_homed = false;
static gimbal_fdb_t         s_fdb;

/* ------------------------------- 内部函数声明 ------------------------------ */

static void  axis_pid_init(gim_axis_t *ax);
static float ecd_to_rel_deg(uint16_t ecd, uint16_t center);
static int16_t axis_control(gim_axis_t *ax, const dj_motor_measure_t *m);
static int16_t axis_ctrl_cb(const dj_motor_measure_t *m, void *ctx);
static void  gimbal_motor_register(void);
static bool  axis_active(uint8_t axis);

/* ---------------------------------- 接口 ---------------------------------- */

void gimbal_set_mode(gimbal_mode_e mode)
{
    s_mode = mode;
}

void gimbal_set_target(float yaw_deg, float pitch_deg)
{
    if (yaw_deg > GIM_YAW_TARGET_LIMIT)
    {
        yaw_deg = GIM_YAW_TARGET_LIMIT;
    }
    else if (yaw_deg < -GIM_YAW_TARGET_LIMIT)
    {
        yaw_deg = -GIM_YAW_TARGET_LIMIT;
    }

    if (pitch_deg > GIM_PITCH_MAX_DEG)
    {
        pitch_deg = GIM_PITCH_MAX_DEG;
    }
    else if (pitch_deg < GIM_PITCH_MIN_DEG)
    {
        pitch_deg = GIM_PITCH_MIN_DEG;
    }

    s_axis[AXIS_YAW].target_deg   = yaw_deg;
    s_axis[AXIS_PITCH].target_deg = pitch_deg;
}

void gimbal_set_speed(float yaw_dps, float pitch_dps)
{
    s_axis[AXIS_YAW].target_dps   = yaw_dps;
    s_axis[AXIS_PITCH].target_dps = pitch_dps;
}

gimbal_mode_e gimbal_get_mode(void)
{
    return s_mode;
}

void gimbal_get_fdb(gimbal_fdb_t *out)
{
    if (out != NULL)
    {
        /* 逐字段拷一份，避免上层读到半新半旧的组合 */
        s_fdb.mode            = s_mode;
        s_fdb.homed           = s_homed;
        s_fdb.yaw_target      = s_axis[AXIS_YAW].target_deg;
        s_fdb.pitch_target    = s_axis[AXIS_PITCH].target_deg;
        s_fdb.yaw_out         = s_axis[AXIS_YAW].out_volt;
        s_fdb.pitch_out       = s_axis[AXIS_PITCH].out_volt;
        s_fdb.online[0]       = (s_axis[AXIS_YAW].motor != NULL)
                                && s_axis[AXIS_YAW].motor->measure.online;
        s_fdb.online[1]       = (s_axis[AXIS_PITCH].motor != NULL)
                                && s_axis[AXIS_PITCH].motor->measure.online;
        *out = s_fdb;
    }
}

/* --------------------------------- 任务入口 -------------------------------- */

/**
 * @brief GimbalTask 入口
 * @note  1kHz，只负责状态机、目标生成和打印，控制量在 MotorTask 中计算。
 */
void StartGimbalTask(void const *argument)
{
    gimbal_mode_e last_mode = GIMBAL_MODE_RELAX;
    uint32_t      last_print = 0;
    uint32_t      sweep_t0 = 0;
    uint32_t      home_t0 = 0;
    uint8_t       i;

    (void)argument;

    for (i = 0; i < AXIS_COUNT; i++)
    {
        memset(&s_axis[i], 0, sizeof(s_axis[i]));
        s_axis[i].sign  = s_axis_cfg[i].sign;
        s_axis[i].index = i;
    }

    axis_pid_init(&s_axis[AXIS_YAW]);
    axis_pid_init(&s_axis[AXIS_PITCH]);
    gimbal_motor_register();

    if ((s_axis[AXIS_YAW].motor == NULL) || (s_axis[AXIS_PITCH].motor == NULL))
    {
        log_printf("[GIM] register FAILED (yaw=%p pit=%p)\r\n",
                   (void *)s_axis[AXIS_YAW].motor, (void *)s_axis[AXIS_PITCH].motor);
    }
    else
    {
        log_printf("[GIM] ready yaw=can%u/id%u pit=can%u/id%u dbg_axis=%d\r\n",
                   GIM_YAW_BUS, GIM_YAW_ID, GIM_PITCH_BUS, GIM_PITCH_ID,
                   (int)GIMBAL_DEBUG_AXIS);
    }

#if ENABLE_GIMBAL_BY_SW1
    /* 由 sw1 决定是否出力，上电保持松开 */
    s_mode  = GIMBAL_MODE_RELAX;
    s_homed = true;
#else
    /* 上电先回中，目标从当前角度开始 */
    s_mode = GIMBAL_MODE_HOME;
#endif

    for (;;)
    {
        uint32_t now = HAL_GetTick();

#if ENABLE_GIMBAL_BY_SW1 || ENABLE_PITCH_BY_WHEEL
        /* 两轴各自有开关，任一轴需要出力就进入角度控制，全部关闭才松开。
         * yaw 由 sw1 下档控制，靠陀螺反馈保持绝对方位；
         * pitch 由 sw2 非上档控制，目标角由拨轮增量调整。 */
        {
            uint8_t need = 0U;

#if ENABLE_GIMBAL_BY_SW1
            if (cmd_get_gimbal_request() != 0U)
            {
                need = 1U;
            }
#endif
#if ENABLE_PITCH_BY_WHEEL
            if (cmd_get_pitch_request() != 0U)
            {
                need = 1U;
            }
#endif

            if (need != 0U)
            {
                if (s_mode == GIMBAL_MODE_RELAX)
                {
                    s_mode = GIMBAL_MODE_ANGLE;
                    s_axis[AXIS_YAW].target_deg   = 0.0f;
                    s_axis[AXIS_PITCH].target_deg = 0.0f;
                }
            }
            else if (s_mode != GIMBAL_MODE_RELAX)
            {
                s_mode = GIMBAL_MODE_RELAX;
            }
        }
#endif

        /* 模式切换时清 PID，避免和 1kHz 回调竞争 */
        if (s_mode != last_mode)
        {
            for (i = 0; i < AXIS_COUNT; i++)
            {
                pid_clear(s_axis[i].pid_angle);
                pid_clear(s_axis[i].pid_speed);
            }

            if (s_mode == GIMBAL_MODE_HOME)
            {
                /* 以当前角度为起点斜坡转向 0° */
                s_homed = false;
                home_t0 = now;
                s_axis[AXIS_YAW].target_deg   = s_fdb.yaw_deg;
                s_axis[AXIS_PITCH].target_deg = s_fdb.pitch_deg;
            }
            last_mode = s_mode;
        }

        /* 目标生成 */
        switch (s_mode)
        {
            case GIMBAL_MODE_RELAX:
                dj_motor_relax(s_axis[AXIS_YAW].motor);
                dj_motor_relax(s_axis[AXIS_PITCH].motor);
                break;

            case GIMBAL_MODE_HOME:
            {
                float step = GIM_HOME_RATE_DPS * 0.001f;      /* 每 ms 最大变化 */
                float ty = s_axis[AXIS_YAW].target_deg;
                float tp = s_axis[AXIS_PITCH].target_deg;

                ty += (ty > step) ? -step : ((ty < -step) ? step : -ty);
                tp += (tp > step) ? -step : ((tp < -step) ? step : -tp);

                s_axis[AXIS_YAW].target_deg   = ty;
                s_axis[AXIS_PITCH].target_deg = tp;

                /* 两轴到位且有反馈才算回中完成，无反馈时靠超时退出 */
                if ((fabsf(ty) < GIM_HOME_TOL_DEG) && (fabsf(tp) < GIM_HOME_TOL_DEG))
                {
                    bool both_online = s_fdb.online[0] && s_fdb.online[1];
                    bool timeout     = (uint32_t)(now - home_t0) > (3U * GIM_HOME_TIME_MS);

                    if (both_online || timeout)
                    {
                        s_homed  = true;
                        s_mode   = GIMBAL_MODE_ANGLE;
                        sweep_t0 = now;
                        log_printf("[GIM] home done%s\r\n",
                                   both_online ? "" : " (no feedback, timeout)");
                    }
                }
                break;
            }

            case GIMBAL_MODE_ANGLE:
#if ENABLE_GIMBAL_BENCH
                if (s_homed)
                {
                    /* 0.1Hz / 0.07Hz 正弦摆动，幅度 ±20° / ±10° */
                    float t = (float)(now - sweep_t0) * 0.001f;

#if GIMBAL_DEBUG_AXIS == 1
                    gimbal_set_target(20.0f * sinf(6.2831853f * 0.10f * t), 0.0f);
#elif GIMBAL_DEBUG_AXIS == 2
                    gimbal_set_target(0.0f, 10.0f * sinf(6.2831853f * 0.07f * t));
#else
                    gimbal_set_target(20.0f * sinf(6.2831853f * 0.10f * t),
                                      10.0f * sinf(6.2831853f * 0.07f * t));
#endif
                }
#endif
#if ENABLE_PITCH_BY_WHEEL
                /* 拨轮直接给 pitch 目标角速度，回中即停 */
                if (cmd_get_pitch_request() != 0U)
                {
                    int16_t wheel = cmd_get_wheel();
                    float   dps   = 0.0f;

                    if ((wheel > PITCH_WHEEL_DEADBAND) || (wheel < -PITCH_WHEEL_DEADBAND))
                    {
                        dps = ((float)wheel / PITCH_WHEEL_RAW_MAX) * PITCH_WHEEL_MAX_DPS;
                    }

                    s_axis[AXIS_PITCH].target_dps = dps;
                }
                else
                {
                    s_axis[AXIS_PITCH].target_dps = 0.0f;
                }
#endif
                break;

            case GIMBAL_MODE_SPEED:
                break;

            default:
                break;
        }

        /* 使能/失能 */
        if (s_mode != GIMBAL_MODE_RELAX)
        {
            if (axis_active(AXIS_YAW))
            {
                dj_motor_enable(s_axis[AXIS_YAW].motor);
            }
            else
            {
                dj_motor_relax(s_axis[AXIS_YAW].motor);
            }

            if (axis_active(AXIS_PITCH))
            {
                dj_motor_enable(s_axis[AXIS_PITCH].motor);
            }
            else
            {
                dj_motor_relax(s_axis[AXIS_PITCH].motor);
            }
        }

        /* 反馈快照 */
        s_fdb.mode         = s_mode;
        s_fdb.homed        = s_homed;
        s_fdb.yaw_target   = s_axis[AXIS_YAW].target_deg;
        s_fdb.pitch_target = s_axis[AXIS_PITCH].target_deg;
        s_fdb.yaw_out      = s_axis[AXIS_YAW].out_volt;
        s_fdb.pitch_out    = s_axis[AXIS_PITCH].out_volt;
        s_fdb.online[0]    = (s_axis[AXIS_YAW].motor != NULL)
                             && s_axis[AXIS_YAW].motor->measure.online;
        s_fdb.online[1]    = (s_axis[AXIS_PITCH].motor != NULL)
                             && s_axis[AXIS_PITCH].motor->measure.online;

#if ENABLE_DIAG_GIMBAL
        if ((uint32_t)(now - last_print) >= 500U)
        {
            last_print = now;
            log_printf("[GIM] md=%d home=%d y=%.1f/%.1f p=%.1f/%.1f v=%d/%d on=%d%d\r\n",
                       (int)s_mode, (int)s_homed,
                       s_fdb.yaw_deg, s_fdb.yaw_target,
                       s_fdb.pitch_deg, s_fdb.pitch_target,
                       s_fdb.yaw_out, s_fdb.pitch_out,
                       (int)s_fdb.online[0], (int)s_fdb.online[1]);
        }
#endif

        osDelay(1);
    }
}

/* ------------------------------- 初始化与底层 ------------------------------ */

static void axis_pid_init(gim_axis_t *ax)
{
    const gim_axis_cfg_t *cfg = &s_axis_cfg[ax->index];
    pid_config_t angle_cfg;
    pid_config_t speed_cfg;

    angle_cfg = (pid_config_t)INIT_PID_CONFIG(cfg->angle_kp, 0.0f, 0.0f,
                                             cfg->angle_max, cfg->angle_max,
                                             PID_Derivative_On_Measurement);
    speed_cfg = (pid_config_t)INIT_PID_CONFIG(cfg->speed_kp, cfg->speed_ki, 0.0f,
                                             cfg->speed_ilim, cfg->max_out,
                                             PID_Integral_Limit
                                             | PID_Derivative_On_Measurement);

    ax->pid_angle = pid_register(&angle_cfg);
    ax->pid_speed = pid_register(&speed_cfg);
}

static void gimbal_motor_register(void)
{
    dj_motor_config_t cfg;

    for (uint8_t i = 0; i < AXIS_COUNT; i++)
    {
        memset(&cfg, 0, sizeof(cfg));
        cfg.type    = s_axis_cfg[i].type;
        cfg.bus     = s_axis_cfg[i].bus;
        cfg.id      = s_axis_cfg[i].id;
        cfg.control = axis_ctrl_cb;
        cfg.ctx     = &s_axis[i];
        s_axis[i].motor = dj_motor_register(&cfg);
    }
}

/**
 * @brief 编码器原始值转相对中位角度（°），折叠到 ±180
 */
static float ecd_to_rel_deg(uint16_t ecd, uint16_t center)
{
    int32_t d = (int32_t)ecd - (int32_t)center;

    if (d > 4096)
    {
        d -= 8192;
    }
    else if (d < -4096)
    {
        d += 8192;
    }
    return (float)d / GIM_ECD_PER_DEG;
}

/**
 * @brief 单轴串级控制
 * @note  角度环输出目标角速度，速度环输出 ±30000 电压。
 */
static int16_t axis_control(gim_axis_t *ax, const dj_motor_measure_t *m)
{
    uint8_t axis;
    float ang;
    float rpm;
    float ref_rpm;
    float out;
    int16_t send;

    if ((ax == NULL) || (m == NULL) || (ax->pid_angle == NULL) || (ax->pid_speed == NULL))
    {
        return 0;
    }

    axis = ax->index;

    /* 电机掉线：清状态并停止出力 */
    if (!m->online)
    {
        pid_clear(ax->pid_angle);
        pid_clear(ax->pid_speed);
        ax->out_volt = 0;
        return 0;
    }

    /* 第一帧有效反馈用于确定中位 */
    if (!ax->center_ready)
    {
#if GIM_USE_FIXED_CENTER
        ax->center_ecd = (axis == AXIS_YAW) ? GIM_YAW_CENTER_ECD : GIM_PITCH_CENTER_ECD;
#else
        ax->center_ecd = m->ecd;
#endif
#if GIM_USE_IMU_FEEDBACK
        {
            ins_attitude_t att;

            ins_get_attitude(&att);
            ax->center_ins_deg = ((axis == AXIS_YAW) ? att.yaw_total : att.pitch)
                               * GIM_RAD2DEG;
        }
#endif

        ax->center_ready = true;
        /* 中位确定前不输出 */
        ax->target_deg = 0.0f;
        return 0;
    }

    /* 单轴调试时另一轴不出力 */
    if (!axis_active(axis))
    {
        pid_clear(ax->pid_angle);
        pid_clear(ax->pid_speed);
        ax->out_volt = 0;
        return 0;
    }

#if GIM_USE_IMU_FEEDBACK
    {
        ins_attitude_t att;

        ins_get_attitude(&att);

        if (att.valid != 0U)
        {
            float cur_deg = ((axis == AXIS_YAW) ? att.yaw_total : att.pitch) * GIM_RAD2DEG;

            if (!ax->ins_locked)
            {
                /* 首次收敛：以当前位置为基准，避免角度跳变 */
                ax->center_ins_deg = cur_deg;
                ax->ins_locked     = true;
                ax->target_deg     = 0.0f;
                pid_clear(ax->pid_angle);
                pid_clear(ax->pid_speed);
            }

            ang = cur_deg - ax->center_ins_deg;
            rpm = ((axis == AXIS_YAW) ? att.gyro[2] : att.gyro[1])
                * GIM_RAD2DEG / RPM_2_ANGLE_PER_SEC;
        }
        else
        {
            /* IMU 未就绪时退回编码器，保证云台仍可回中 */
            ang = ecd_to_rel_deg(m->ecd, ax->center_ecd);
            rpm = m->speed_rpm_f;
        }
    }
#else
    ang = ecd_to_rel_deg(m->ecd, ax->center_ecd);
    rpm = m->speed_rpm_f;
#endif

    /* 统一到逻辑方向：sign 同时作用于反馈和输出，与底盘一致。
     * 只反输出会造成速度环正反馈，pitch 的 sign 为 -1 时必然发散。 */
    ang *= ax->sign;
    rpm *= ax->sign;

    if (axis == AXIS_YAW)
    {
        s_fdb.yaw_deg       = ang;
        s_fdb.yaw_speed_dps = rpm * RPM_2_ANGLE_PER_SEC;
    }
    else
    {
        s_fdb.pitch_deg       = ang;
        s_fdb.pitch_speed_dps = rpm * RPM_2_ANGLE_PER_SEC;
    }

    if (s_mode == GIMBAL_MODE_RELAX)
    {
        pid_clear(ax->pid_angle);
        pid_clear(ax->pid_speed);
        ax->out_volt = 0;
        return 0;
    }

#if ENABLE_PITCH_OPENLOOP
    if (axis == AXIS_PITCH)
    {
        /* 拨轮回中：按转速反向出力做阻尼刹车，增益低不会自激；
         * 拨轮偏离中位：开环给恒定电流，转向由拨轮方向决定。 */
        if (fabsf(ax->target_dps) < 0.5f)
        {
            if (fabsf(rpm) > PITCH_BRAKE_RPM_MIN)
            {
                out = -rpm * PITCH_BRAKE_KP;

                if (out > PITCH_BRAKE_MAX)
                {
                    out = PITCH_BRAKE_MAX;
                }
                else if (out < -PITCH_BRAKE_MAX)
                {
                    out = -PITCH_BRAKE_MAX;
                }
            }
            else
            {
                out = 0.0f;
            }
        }
        else
        {
            float abs_rpm;

            /* 拨轮偏离中位即按恒定电流输出，只有方向由拨轮决定，
             * 不再按拨杆量做比例调速。 */
            out = (ax->target_dps > 0.0f) ? PITCH_OPENLOOP_CURRENT
                                          : -PITCH_OPENLOOP_CURRENT;

            /* 接近转速上限时线性降流：达到阈值的 DERATE 比例后按比例减小，
             * 到阈值时降到 0。相比硬断流不会在阈值附近反复启停。 */
            abs_rpm = fabsf(rpm);

            if (abs_rpm > (PITCH_OPENLOOP_MAX_RPM * PITCH_OPENLOOP_DERATE))
            {
                float ratio = (PITCH_OPENLOOP_MAX_RPM - abs_rpm)
                            / (PITCH_OPENLOOP_MAX_RPM * (1.0f - PITCH_OPENLOOP_DERATE));

                if (ratio < 0.0f)
                {
                    ratio = 0.0f;
                }

                out *= ratio;
            }
        }

        send = (int16_t)(out * ax->sign);
        ax->out_volt = send;

        pid_clear(ax->pid_angle);
        pid_clear(ax->pid_speed);

        return send;
    }
#endif

    if ((s_mode == GIMBAL_MODE_SPEED) || (axis == AXIS_PITCH))
    {
        /* 回中静止时清积分，避免噪声累积成突加力矩 */
        if ((fabsf(ax->target_dps) < 0.5f) && (fabsf(rpm) < 2.0f))
        {
            pid_clear(ax->pid_speed);
            ax->out_volt = 0;
            return 0;
        }

        /* pitch 走纯速度控制：拨轮直接给输出轴角速度，不依赖角度反馈。
         * 目标换算到转子 rpm，减速比由轴配置决定，直驱为 1。 */
        ref_rpm = ax->target_dps / RPM_2_ANGLE_PER_SEC * s_axis_cfg[axis].gear;
        pid_clear(ax->pid_angle);
    }
    else
    {
        /* HOME / ANGLE：角度外环输出目标角速度 */
        float ref_dps = pid_calculate(ax->pid_angle, ang, ax->target_deg);

        ref_rpm = ref_dps / RPM_2_ANGLE_PER_SEC * s_axis_cfg[axis].gear;
    }

    out = pid_calculate(ax->pid_speed, rpm, ref_rpm);

    if (axis == AXIS_PITCH)
    {
        out += PITCH_GRAVITY_FF;
    }

    send = (int16_t)(out * ax->sign);
    ax->out_volt = send;

    return send;
}

static int16_t axis_ctrl_cb(const dj_motor_measure_t *m, void *ctx)
{
    return axis_control((gim_axis_t *)ctx, m);
}

/**
 * @brief 该轴是否允许出力
 */
static bool axis_active(uint8_t axis)
{
#if GIMBAL_DEBUG_AXIS == 1
    if (axis != AXIS_YAW) return false;
#elif GIMBAL_DEBUG_AXIS == 2
    if (axis != AXIS_PITCH) return false;
#endif

#if ENABLE_GIMBAL_BY_SW1
    if (axis == AXIS_YAW)
    {
        return (cmd_get_gimbal_request() != 0U);
    }
#endif

#if ENABLE_PITCH_BY_WHEEL
    if (axis == AXIS_PITCH)
    {
        return (cmd_get_pitch_request() != 0U);
    }
#endif

    (void)axis;
    return true;
}
