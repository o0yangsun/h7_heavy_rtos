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
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

/* --------------------------------- 常量/宏 --------------------------------- */

#define AXIS_YAW                0U
#define AXIS_PITCH              1U
#define AXIS_COUNT              2U

/* --------------------------------- 内部状态 -------------------------------- */

typedef struct
{
    dj_motor_object_t *motor;
    pid_obj_t         *pid_angle;
    pid_obj_t         *pid_speed;
    uint16_t           center_ecd;
    bool               center_ready;
    volatile float     target_deg;      /**< 任务层写，1kHz 回调读 */
    volatile float     target_dps;
    volatile int16_t   out_volt;
    float              sign;            /**< 输出方向修正 */
    uint8_t            index;           /**< AXIS_YAW / AXIS_PITCH */
} gim_axis_t;

typedef struct
{
    uint8_t bus;
    uint8_t id;
    float   sign;
    float   angle_kp;
    float   angle_max;
    float   speed_kp;
    float   speed_ki;
    float   speed_ilim;
    float   max_volt;
} gim_axis_cfg_t;

static const gim_axis_cfg_t s_axis_cfg[AXIS_COUNT] =
{
    {GIM_YAW_BUS,   GIM_YAW_ID,   GIM_YAW_SIGN,   YAW_ANGLE_KP,   YAW_ANGLE_MAX,
     YAW_SPEED_KP,   YAW_SPEED_KI,   YAW_SPEED_ILIM,   YAW_MAX_VOLT},
    {GIM_PITCH_BUS, GIM_PITCH_ID, GIM_PITCH_SIGN, PITCH_ANGLE_KP, PITCH_ANGLE_MAX,
     PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_ILIM, PITCH_MAX_VOLT},
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

    /* 上电先回中，目标从当前角度开始 */
    s_mode = GIMBAL_MODE_HOME;

    for (;;)
    {
        uint32_t now = HAL_GetTick();

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
                                             cfg->speed_ilim, cfg->max_volt,
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
        cfg.type    = DJ_MOTOR_GM6020;
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

    ang = ecd_to_rel_deg(m->ecd, ax->center_ecd);
    rpm = (float)m->speed_rpm;

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

    if (s_mode == GIMBAL_MODE_SPEED)
    {
        ref_rpm = ax->target_dps / RPM_2_ANGLE_PER_SEC;
        pid_clear(ax->pid_angle);        /* 速度模式不用外环 */
    }
    else
    {
        /* HOME / ANGLE：角度外环输出目标角速度 */
        float ref_dps = pid_calculate(ax->pid_angle, ang, ax->target_deg);

        ref_rpm = ref_dps / RPM_2_ANGLE_PER_SEC;
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
    return (axis == AXIS_YAW);
#elif GIMBAL_DEBUG_AXIS == 2
    return (axis == AXIS_PITCH);
#else
    (void)axis;
    return true;
#endif
}
