/**
  ******************************************************************************
  * @file    shoot_task.c
  * @brief   发射机构线程：三个摩擦轮转速闭环，详见 shoot_task.h
  ******************************************************************************
  */

#include "shoot_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "dj_motor.h"
#include "pid.h"
#include "cmd_task.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>
#include <math.h>

/* --------------------------------- 内部状态 -------------------------------- */

typedef struct
{
    dj_motor_object_t *motor;
    pid_obj_t         *pid_speed;
    float              dir;
    volatile float     ref_out_rpm;
    volatile float     act_out_rpm;
    volatile int16_t   out;
} fric_wheel_t;

typedef struct
{
    dj_motor_object_t      *motor;
    pid_obj_t              *pid_angle;
    pid_obj_t              *pid_speed;
    volatile float          target_angle;   /**< 电机轴累计角度目标 */
    volatile int16_t        out;
    volatile shoot_trigger_mode_e mode;
    uint8_t                 synced;         /**< 首次收到反馈后以当前角度为基准 */
} trigger_motor_t;

volatile shoot_diag_t g_shoot_diag;

static fric_wheel_t     s_fric[FRIC_NUM];
static volatile float   s_ref_out_rpm = 0.0f;
static volatile uint8_t s_enabled = 0U;
static task_stat_t      s_stat;

#if ENABLE_FRICTION && (ENABLE_FRICTION_BY_SW1 || ENABLE_FRICTION_BENCH)
static volatile uint8_t s_fric_started = 0U;
#endif

#if ENABLE_FRICTION && ENABLE_FRICTION_BY_SW1
static volatile uint32_t s_fric_start_ms = 0U;   /* 软启动起点 */
#endif

#if ENABLE_TRIGGER
static trigger_motor_t s_trigger;
static uint8_t         s_ch4_armed    = 1U;   /* 1 = 允许下一次单发 */
static uint32_t        s_last_shot_ms = 0U;
#endif

#if ENABLE_FRICTION
static const uint8_t s_fric_id[FRIC_NUM] = {FRIC_ID_1, FRIC_ID_2, FRIC_ID_3};
static const float   s_fric_dir[FRIC_NUM] = {FRIC_DIR_1, FRIC_DIR_2, FRIC_DIR_3};

/* ------------------------------- 内部函数声明 ------------------------------ */

static void    fric_init(void);
static int16_t fric_ctrl_cb(const dj_motor_measure_t *m, void *ctx);
static void    fric_set_ref(float out_rpm);
static void    fric_update_diag(void);
#endif

#if ENABLE_TRIGGER
static void    trigger_init(void);
static int16_t trigger_ctrl_cb(const dj_motor_measure_t *m, void *ctx);
static void    trigger_update_diag(void);
#endif

/* ---------------------------------- 接口 ---------------------------------- */

void shoot_set_friction_rpm(float out_rpm)
{
    if (out_rpm > FRIC_MAX_OUT_RPM)
    {
        out_rpm = FRIC_MAX_OUT_RPM;
    }
    else if (out_rpm < -FRIC_MAX_OUT_RPM)
    {
        out_rpm = -FRIC_MAX_OUT_RPM;
    }

    s_ref_out_rpm = out_rpm;
}

float shoot_get_friction_rpm(void)
{
    return s_ref_out_rpm;
}

void shoot_set_friction_enable(uint8_t enable)
{
    uint8_t i;

    s_enabled = (enable != 0U) ? 1U : 0U;

    for (i = 0; i < FRIC_NUM; i++)
    {
        if (s_fric[i].motor == NULL)
        {
            continue;
        }

        if (s_enabled != 0U)
        {
            dj_motor_enable(s_fric[i].motor);
        }
        else
        {
            dj_motor_relax(s_fric[i].motor);

            if (s_fric[i].pid_speed != NULL)
            {
                pid_clear(s_fric[i].pid_speed);
            }

            s_fric[i].out = 0;
        }
    }
}

void shoot_set_trigger_mode(shoot_trigger_mode_e mode)
{
#if ENABLE_TRIGGER
    if ((mode == TRIGGER_MODE_STOP) && (s_trigger.mode != TRIGGER_MODE_STOP))
    {
        /* 回到停止时以当前角度为目标，避免停在半路后回弹 */
        if (s_trigger.motor != NULL)
        {
            s_trigger.target_angle = s_trigger.motor->measure.total_angle
                                     * TRIGGER_FORWARD_SIGN;
        }

        if (s_trigger.pid_angle != NULL)
        {
            pid_clear(s_trigger.pid_angle);
        }

        if (s_trigger.pid_speed != NULL)
        {
            pid_clear(s_trigger.pid_speed);
        }
    }

    s_trigger.mode = mode;
#else
    (void)mode;
#endif
}

shoot_trigger_mode_e shoot_get_trigger_mode(void)
{
#if ENABLE_TRIGGER
    return s_trigger.mode;
#else
    return TRIGGER_MODE_STOP;
#endif
}

void shoot_trigger_once(void)
{
#if ENABLE_TRIGGER
    float delta;
    float cur_angle;
    float new_target;

    if ((s_trigger.motor == NULL) || (s_trigger.synced == 0U))
    {
        return;
    }

    /* 连发与反转期间不接受单发请求 */
    if ((s_trigger.mode == TRIGGER_MODE_AUTO) || (s_trigger.mode == TRIGGER_MODE_REVERSE))
    {
        return;
    }

    delta     = TRIGGER_SHOT_DEG * TRIGGER_GEAR_RATIO;
    cur_angle = s_trigger.motor->measure.total_angle * TRIGGER_FORWARD_SIGN;
    new_target = s_trigger.target_angle + delta;

    /* 目标最多领先实际角度两发。误触发或信号抖动时不再继续累加，
     * 避免目标跑远后角度环长时间饱和追击。 */
    if (fabsf(new_target - cur_angle) > (TRIGGER_TARGET_LEAD_SHOTS * delta))
    {
        return;
    }

    s_trigger.target_angle = new_target;
    s_trigger.mode         = TRIGGER_MODE_SINGLE;
#endif
}

#if ENABLE_FRICTION

/* --------------------------------- 内部实现 -------------------------------- */

/**
 * @brief 注册三个摩擦轮与各自的速度环
 * @note  在 ShootTask 启动时调用一次，须早于 MotorTask 首次发帧
 */
static void fric_init(void)
{
    pid_config_t      pcfg;
    dj_motor_config_t mcfg;
    uint8_t           i;

    for (i = 0; i < FRIC_NUM; i++)
    {
        memset(&pcfg, 0, sizeof(pcfg));
        pcfg.Kp            = FRIC_SPEED_KP;
        pcfg.Ki            = FRIC_SPEED_KI;
        pcfg.Kd            = FRIC_SPEED_KD;
        pcfg.MaxOut        = FRIC_SPEED_MAX;
        pcfg.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                           | PID_Derivative_On_Measurement;
        pcfg.IntegralLimit = FRIC_SPEED_ILIM;

        s_fric[i].pid_speed = pid_register(&pcfg);
        s_fric[i].dir       = s_fric_dir[i];

        memset(&mcfg, 0, sizeof(mcfg));
        mcfg.type    = DJ_MOTOR_M3508;
        mcfg.bus     = CAN_FRICTION;
        mcfg.id      = s_fric_id[i];
        mcfg.control = fric_ctrl_cb;
        mcfg.ctx     = &s_fric[i];

        s_fric[i].motor = dj_motor_register(&mcfg);
    }
}

/**
 * @brief 单轮速度环，在 MotorTask 的 1kHz 上下文里跑
 * @note  dir 同时作用于反馈和输出，避免只反输出造成正反馈
 */
static int16_t fric_ctrl_cb(const dj_motor_measure_t *m, void *ctx)
{
    fric_wheel_t *w = (fric_wheel_t *)ctx;
    float         rotor_rpm;
    float         ref_rotor_rpm;
    float         out;

    if ((w == NULL) || (m == NULL) || (w->pid_speed == NULL))
    {
        return 0;
    }

    if (!m->online)
    {
        pid_clear(w->pid_speed);
        w->act_out_rpm = 0.0f;
        w->out         = 0;
        return 0;
    }

    rotor_rpm     = m->speed_rpm_f * w->dir;
    w->act_out_rpm = rotor_rpm / FRIC_GEAR_RATIO;

    if (s_enabled == 0U)
    {
        pid_clear(w->pid_speed);
        w->out = 0;
        return 0;
    }

    ref_rotor_rpm = w->ref_out_rpm * FRIC_GEAR_RATIO;
    out           = pid_calculate(w->pid_speed, rotor_rpm, ref_rotor_rpm);

    /* 静摩擦补偿：静止且目标非零时补足最小输出，否则轮子启不动 */
    if (fabsf(w->act_out_rpm) < FRIC_SPEED_STATIC_RPM)
    {
        if (w->ref_out_rpm > 1.0f)
        {
            if (out < FRIC_SPEED_MIN_OUT)
            {
                out = FRIC_SPEED_MIN_OUT;
            }
        }
        else if (w->ref_out_rpm < -1.0f)
        {
            if (out > -FRIC_SPEED_MIN_OUT)
            {
                out = -FRIC_SPEED_MIN_OUT;
            }
        }
    }

    if (w->dir < 0.0f)
    {
        out = -out;
    }

    w->out = (int16_t)out;

    return w->out;
}

/** 下发目标转速到三个轮，输入为输出轴转速 */
static void fric_set_ref(float out_rpm)
{
    uint8_t i;

    for (i = 0; i < FRIC_NUM; i++)
    {
        s_fric[i].ref_out_rpm = out_rpm;
    }
}

/** 刷新诊断结构，100ms 调用一次 */
static void fric_update_diag(void)
{
    uint8_t i;
    uint8_t mask = 0U;

    for (i = 0; i < FRIC_NUM; i++)
    {
        g_shoot_diag.ref_out_rpm[i] = s_fric[i].ref_out_rpm;
        g_shoot_diag.act_out_rpm[i] = s_fric[i].act_out_rpm;
        g_shoot_diag.out[i]         = s_fric[i].out;

        if ((s_fric[i].motor != NULL) && s_fric[i].motor->measure.online)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    g_shoot_diag.online_mask = mask;
}

#endif /* ENABLE_FRICTION */

#if ENABLE_TRIGGER

/* --------------------------------- 拨弹实现 -------------------------------- */

/**
 * @brief 注册拨弹电机与角度、速度双环
 */
static void trigger_init(void)
{
    pid_config_t      angle_cfg;
    pid_config_t      speed_cfg;
    dj_motor_config_t mcfg;

    memset(&angle_cfg, 0, sizeof(angle_cfg));
    angle_cfg.Kp            = TRIGGER_ANGLE_KP;
    angle_cfg.MaxOut        = TRIGGER_ANGLE_MAX;
    angle_cfg.Improve       = PID_Integral_Limit;
    angle_cfg.IntegralLimit = 0.0f;

    memset(&speed_cfg, 0, sizeof(speed_cfg));
    speed_cfg.Kp            = TRIGGER_SPEED_KP;
    speed_cfg.Ki            = TRIGGER_SPEED_KI;
    speed_cfg.MaxOut        = TRIGGER_SPEED_MAX;
    speed_cfg.Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement;
    speed_cfg.IntegralLimit = TRIGGER_SPEED_ILIM;

    s_trigger.pid_angle = pid_register(&angle_cfg);
    s_trigger.pid_speed = pid_register(&speed_cfg);
    s_trigger.mode      = TRIGGER_MODE_STOP;
    s_trigger.synced    = 0U;

    memset(&mcfg, 0, sizeof(mcfg));
    mcfg.type    = DJ_MOTOR_M3508;
    mcfg.bus     = TRIGGER_BUS;
    mcfg.id      = TRIGGER_ID;
    mcfg.control = trigger_ctrl_cb;
    mcfg.ctx     = &s_trigger;

    s_trigger.motor = dj_motor_register(&mcfg);

    /* 注册后默认是松开状态，必须显式使能才会下发控制量 */
    if (s_trigger.motor != NULL)
    {
        dj_motor_enable(s_trigger.motor);
    }
}

/**
 * @brief 拨弹控制，在 MotorTask 的 1kHz 上下文里跑
 * @note  单发与停止走角度环定位，连发和反转直接给速度环
 */
static int16_t trigger_ctrl_cb(const dj_motor_measure_t *m, void *ctx)
{
    trigger_motor_t *t = (trigger_motor_t *)ctx;
    float            ang_meas;
    float            rpm_meas;
    float            ref_rpm;
    float            out;

    if ((t == NULL) || (m == NULL) || (t->pid_angle == NULL) || (t->pid_speed == NULL))
    {
        return 0;
    }

    if (!m->online)
    {
        pid_clear(t->pid_angle);
        pid_clear(t->pid_speed);
        t->out = 0;
        return 0;
    }

    /* 反馈转到逻辑方向。sign 若只作用于输出，负值时环路会变成正反馈，
     * 表现为目标越追越远、电机持续加速。 */
    ang_meas = m->total_angle * TRIGGER_FORWARD_SIGN;
    rpm_meas = m->speed_rpm_f * TRIGGER_FORWARD_SIGN;

    /* 首帧以当前角度为基准，避免目标从 0 起算造成猛转 */
    if (t->synced == 0U)
    {
        t->target_angle = ang_meas;
        t->synced       = 1U;
    }

    /* 目标与反馈偏离过远说明状态异常，重置目标止损 */
    if (fabsf(t->target_angle - ang_meas) > TRIGGER_TARGET_LIMIT_DEG)
    {
        t->target_angle = ang_meas;
        pid_clear(t->pid_angle);
        pid_clear(t->pid_speed);
    }

    switch (t->mode)
    {
        case TRIGGER_MODE_AUTO:
            ref_rpm = TRIGGER_AUTO_RPM;
            pid_clear(t->pid_angle);
            break;

        case TRIGGER_MODE_REVERSE:
            ref_rpm = -TRIGGER_AUTO_RPM;
            pid_clear(t->pid_angle);
            break;

        case TRIGGER_MODE_SINGLE:
        case TRIGGER_MODE_STOP:
        default:
            ref_rpm = pid_calculate(t->pid_angle, ang_meas, t->target_angle);
            break;
    }

    out = pid_calculate(t->pid_speed, rpm_meas, ref_rpm);

    t->out = (int16_t)(out * TRIGGER_FORWARD_SIGN);

    return t->out;
}

/** 刷新拨弹诊断，100ms 调用一次 */
static void trigger_update_diag(void)
{
    g_shoot_diag.trigger_angle  = (s_trigger.motor != NULL)
                                  ? s_trigger.motor->measure.total_angle : 0.0f;
    /* 目标角换算回物理域，便于与实测角度直接对照 */
    g_shoot_diag.trigger_target = s_trigger.target_angle * TRIGGER_FORWARD_SIGN;
    g_shoot_diag.trigger_out    = s_trigger.out;
    g_shoot_diag.trigger_mode   = (uint8_t)s_trigger.mode;
    g_shoot_diag.trigger_online = ((s_trigger.motor != NULL)
                                   && s_trigger.motor->measure.online) ? 1U : 0U;
}

#endif /* ENABLE_TRIGGER */

/* ---------------------------------- 线程 ---------------------------------- */

/**
 * @brief ShootTask 入口
 */
void StartShootTask(void const *argument)
{
#if ENABLE_FRICTION && ENABLE_FRICTION_BENCH && !ENABLE_FRICTION_BY_SW1
    uint32_t start_ms;
#endif

    (void)argument;

    memset((void *)&g_shoot_diag, 0, sizeof(g_shoot_diag));
    task_stat_init(&s_stat, SHOOT_PERIOD_MS);

#if ENABLE_FRICTION
    fric_init();

    log_printf("[SHT] friction can%u id%u/%u/%u sw1=%d\r\n",
               CAN_FRICTION, FRIC_ID_1, FRIC_ID_2, FRIC_ID_3, (int)ENABLE_FRICTION_BY_SW1);
#else
    log_printf("[SHT] friction disabled\r\n");
#endif

#if ENABLE_TRIGGER
    trigger_init();

    log_printf("[SHT] trigger can%u id%u\r\n", TRIGGER_BUS, TRIGGER_ID);
#endif

#if ENABLE_FRICTION && ENABLE_FRICTION_BENCH && !ENABLE_FRICTION_BY_SW1
    start_ms = HAL_GetTick();
#endif

    for (;;)
    {
        task_stat_begin(&s_stat, HAL_GetTick());

#if ENABLE_FRICTION
#if ENABLE_FRICTION_BY_SW1
        /* sw1 上拨启动，离开上档立即停止。启动后按 FRIC_SPINUP_MS 线性升速，
         * 目标缓慢爬升，速度环不会给出阶跃电流。 */
        if (cmd_get_friction_request() != 0U)
        {
            uint32_t elapsed;

            if (s_fric_started == 0U)
            {
                s_fric_started  = 1U;
                s_fric_start_ms = HAL_GetTick();
                shoot_set_friction_enable(1U);
            }

            elapsed = HAL_GetTick() - s_fric_start_ms;

            if (elapsed < FRIC_SPINUP_MS)
            {
                shoot_set_friction_rpm(FRIC_BENCH_OUT_RPM
                                       * (float)elapsed / (float)FRIC_SPINUP_MS);
            }
            else
            {
                shoot_set_friction_rpm(FRIC_BENCH_OUT_RPM);
            }
        }
        else if (s_fric_started != 0U)
        {
            s_fric_started = 0U;
            shoot_set_friction_rpm(0.0f);
            shoot_set_friction_enable(0U);
        }
#elif ENABLE_FRICTION_BENCH
        /* 台架：延时后三轮回定速转动，用于确认转向与转速 */
        if (((HAL_GetTick() - start_ms) >= FRIC_BENCH_START_DELAY_MS) && (s_fric_started == 0U))
        {
            s_fric_started = 1U;
            shoot_set_friction_rpm(FRIC_BENCH_OUT_RPM);
            shoot_set_friction_enable(1U);
        }
#endif
        fric_set_ref(s_ref_out_rpm);
        fric_update_diag();
#endif

#if ENABLE_TRIGGER
        /* ch4 上推一次前进一步，须回落到 REARM 以下才允许下一次；
         * 下推持续反转，回中保持当前位置 */
        {
            int16_t  ch4    = cmd_get_ch4();
            uint32_t now_ms = HAL_GetTick();

            if (ch4 > TRIGGER_CH_SINGLE)
            {
                if ((s_ch4_armed != 0U)
                    && ((now_ms - s_last_shot_ms) >= TRIGGER_MIN_INTERVAL_MS))
                {
                    shoot_trigger_once();
                    s_last_shot_ms = now_ms;
                }

                s_ch4_armed = 0U;
            }
            else if (ch4 < TRIGGER_CH_REARM)
            {
                s_ch4_armed = 1U;
            }

            if (ch4 < TRIGGER_CH_REVERSE)
            {
                if (s_trigger.mode != TRIGGER_MODE_REVERSE)
                {
                    shoot_set_trigger_mode(TRIGGER_MODE_REVERSE);
                }
            }
            else if (s_trigger.mode == TRIGGER_MODE_REVERSE)
            {
                shoot_set_trigger_mode(TRIGGER_MODE_STOP);
            }
        }

        trigger_update_diag();
#endif

        g_shoot_diag.loop_cnt = s_stat.loop_cnt;
        g_shoot_diag.enabled  = s_enabled;

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(SHOOT_PERIOD_MS);
    }
}
