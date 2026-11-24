/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   底盘任务实现，详见 chassis_task.h
  ******************************************************************************
  */

#include "chassis_task.h"
#include "dj_motor.h"
#include "pid.h"
#include "log_uart.h"
#include "cmd_task.h"
#include "power_task.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "cmsis_os.h"

#include <string.h>

/* --------------------------------- 内部状态 -------------------------------- */

typedef struct
{
    dj_motor_object_t *motor;
    pid_obj_t         *pid_speed;
    volatile float     ref_rpm;
    volatile float     act_rpm;
    volatile int16_t   out;
    float              sign;
    float              ref_prev;      /**< 上周期目标绝对值，用于检测从 0 起步 */
    uint32_t           launch_tick;   /**< 起步时刻 ms */
    uint8_t            launching;     /**< 1 = 软启动进行中 */
    float              out_last;      /**< 上周期输出，用于斜率限制 */
} chs_wheel_t;

static chs_wheel_t            s_wheel[CHS_WHEEL_NUM];
static volatile chassis_mode_e s_mode = CHASSIS_MODE_NORMAL;
static volatile float         s_cmd_vx = 0.0f;
static volatile float         s_cmd_vy = 0.0f;
static volatile float         s_cmd_vw = 0.0f;
static chassis_fdb_t          s_fdb;
static volatile uint8_t       s_bench_stage = 0U;

/* 四轮 ID、安装方向、运动学系数，顺序 ID1~ID4 */
static const chs_wheel_cfg_t  s_wheel_cfg[CHS_WHEEL_NUM] = CHS_WHEEL_CFG;

/* ------------------------------- 内部函数声明 ------------------------------ */

static void  chassis_motor_init(void);
static void  chassis_kin_selfcheck(void);
static void  chassis_update_fdb(void);
static void  mecanum_calc(float vx, float vy, float vw, float *out_rpm);
static void  mecanum_odom(const float *rpm, float *vx, float *vy, float *vw);
static int16_t wheel_ctrl_cb(const dj_motor_measure_t *m, void *ctx);

/* ---------------------------------- 接口 ---------------------------------- */

void chassis_set_cmd(float vx, float vy, float vw)
{
    s_cmd_vx = float_constrain(vx, -CHS_MAX_VX, CHS_MAX_VX);
    s_cmd_vy = float_constrain(vy, -CHS_MAX_VY, CHS_MAX_VY);
    s_cmd_vw = float_constrain(vw, -CHS_MAX_VW, CHS_MAX_VW);
}

void chassis_set_mode(chassis_mode_e mode)
{
    s_mode = mode;
}

static void chassis_update_fdb(void)
{
    uint8_t i;

    s_fdb.mode   = s_mode;
    s_fdb.vx_cmd = s_cmd_vx;
    s_fdb.vy_cmd = s_cmd_vy;
    s_fdb.vw_cmd = s_cmd_vw;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        s_fdb.rpm_ref[i] = s_wheel[i].ref_rpm;
        s_fdb.rpm_act[i] = s_wheel[i].act_rpm;
        s_fdb.out[i]     = s_wheel[i].out;
    }
}

/**
 * @brief 用四个底盘电机的输出与转速估算总功率
 * @return 估算功率 W，模型 P = Kt·I·ω + k1·ω² + k2·I² + C
 * @note  参数集中在 robot.h 的 CHS_PWR_*，取自 3508 经验值，可用超电实测对照标定
 */
float chassis_power_estimate(void)
{
    float   p = 0.0f;
    uint8_t i;

    for (i = 0U; i < CHS_WHEEL_NUM; i++)
    {
        dj_motor_object_t *m = s_wheel[i].motor;

        if ((m == NULL) || (m->measure.online == 0U))
        {
            continue;
        }

        {
            float w  = fabsf((float)m->measure.speed_rpm_f);
            float il = fabsf((float)s_wheel[i].out);

            p += CHS_PWR_KT_RAW * il * w
               + CHS_PWR_K1 * w * w
               + CHS_PWR_K2 * il * il
               + CHS_PWR_C;
        }
    }

    return p;
}

void chassis_get_fdb(chassis_fdb_t *out)
{
    if (out != NULL)
    {
        chassis_update_fdb();
        *out = s_fdb;
    }
}

/* --------------------------------- 任务入口 -------------------------------- */

/**
 * @brief ChassisTask 入口（覆盖 CubeMX 生成的 weak 版本）
 * @note  1kHz。速度环在 MotorTask 的 1kHz 回调里算，这里只做解算、模式、打印。
 */
void StartChassisTask(void const *argument)
{
    uint32_t last_print = 0;
    uint8_t  i;
#if ENABLE_CHASSIS_BENCH
    uint32_t bench_t0 = 0;
#endif

    (void)argument;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        memset(&s_wheel[i], 0, sizeof(s_wheel[i]));
        s_wheel[i].sign = s_wheel_cfg[i].sign;
    }

    chassis_motor_init();
    chassis_kin_selfcheck();

    {
        uint8_t ok = 1U;

        for (i = 0; i < CHS_WHEEL_NUM; i++)
        {
            if (s_wheel[i].motor == NULL)
            {
                ok = 0U;
            }
        }
        log_printf("[CHS] %s, can%u id1~4, bench=%d cmd_src=%s\r\n",
                   ok ? "motors ready" : "register FAILED",
                   CAN_CHASSIS, (int)ENABLE_CHASSIS_BENCH,
                   CHASSIS_CMD_FROM_CMD_TASK ? "CmdTask" : "manual");
    }

#if ENABLE_CHASSIS_BENCH
    bench_t0 = HAL_GetTick();
#endif

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        float vx = 0.0f;
        float vy = 0.0f;
        float vw = 0.0f;
        uint8_t bench_stage = 0U;
        bool  direct = false;

        s_bench_stage = 0U;

        /* 从 CmdTask 取指令；遥控掉线时锁死，主动失能时放开 */
#if CHASSIS_CMD_FROM_CMD_TASK
        {
            cmd_chassis_t c;

            cmd_get_chassis(&c);

            if (c.online == 0U)
            {
                /* 上电未接管：速度环目标 0 但保持出力，外力转不动轮子 */
                s_mode   = CHASSIS_MODE_STOP;
                s_cmd_vx = 0.0f;
                s_cmd_vy = 0.0f;
                s_cmd_vw = 0.0f;
            }
            else if (c.mode == CMD_MODE_RELAX)
            {
                /* 操作手主动失能：清零积分，轮子可以自由转动 */
                s_mode   = CHASSIS_MODE_RELAX;
                s_cmd_vx = 0.0f;
                s_cmd_vy = 0.0f;
                s_cmd_vw = 0.0f;
            }
            else
            {
                /* 功率限幅移到电流层（见 wheel_ctrl_cb），这里保留原始指令，
                 * 避免"目标速度被调低"带来的响应迟滞 */
                s_mode   = CHASSIS_MODE_NORMAL;
                s_cmd_vx = c.vx;
                s_cmd_vy = c.vy;
                s_cmd_vw = c.vw;
            }
        }
#endif

        /* 按模式生成速度指令 */
        switch (s_mode)
        {
            case CHASSIS_MODE_RELAX:
                for (i = 0; i < CHS_WHEEL_NUM; i++)
                {
                    s_wheel[i].ref_rpm = 0.0f;
                    dj_motor_relax(s_wheel[i].motor);
                }
                break;

            case CHASSIS_MODE_STOP:
                for (i = 0; i < CHS_WHEEL_NUM; i++)
                {
                    s_wheel[i].ref_rpm = 0.0f;
                    dj_motor_enable(s_wheel[i].motor);
                }
                break;

            case CHASSIS_MODE_NORMAL:
            default:
                vx = s_cmd_vx;
                vy = s_cmd_vy;
                vw = s_cmd_vw;

#if ENABLE_CHASSIS_BENCH
                /* 台架序列：等待电调就绪后按 1~7 阶段循环 */
                {
                    uint32_t elapsed = now - bench_t0;

                    if (elapsed < CHS_BENCH_START_DELAY_MS)
                    {
                        bench_stage = 0U;       /* 0 = 全停，等上电稳定 */
                    }
                    else
                    {
                        bench_stage = (uint8_t)(((elapsed - CHS_BENCH_START_DELAY_MS)
                                                 / CHS_BENCH_STAGE_MS) % 7U + 1U);
                    }
                }

                if ((bench_stage >= 1U) && (bench_stage <= 4U))
                {
                    /* 单轮阶跃，其余轮锁 0 */
                    direct = true;
                    for (i = 0; i < CHS_WHEEL_NUM; i++)
                    {
                        s_wheel[i].ref_rpm = (i == (bench_stage - 1U))
                                             ? CHS_BENCH_WHEEL_RPM : 0.0f;
                    }
                }
                else if (bench_stage == 5U)
                {
                    vx = 600.0f;  vy = 0.0f;    vw = 0.0f;
                }
                else if (bench_stage == 6U)
                {
                    vx = 0.0f;    vy = 600.0f;  vw = 0.0f;
                }
                else if (bench_stage == 7U)
                {
                    vx = 0.0f;    vy = 0.0f;    vw = 1.5f;
                }
                else
                {
                    vx = 0.0f;    vy = 0.0f;    vw = 0.0f;
                }
#endif

                for (i = 0; i < CHS_WHEEL_NUM; i++)
                {
                    dj_motor_enable(s_wheel[i].motor);
                }

                if (!direct)
                {
                    float rpm[CHS_WHEEL_NUM];

                    mecanum_calc(vx, vy, vw, rpm);
                    for (i = 0; i < CHS_WHEEL_NUM; i++)
                    {
                        s_wheel[i].ref_rpm = rpm[i];
                    }
                    s_fdb.vx_cmd = vx;
                    s_fdb.vy_cmd = vy;
                    s_fdb.vw_cmd = vw;
                }
                break;
        }

        s_bench_stage = bench_stage;

        /* 由轮速反算实际速度，并刷新在线掩码 */
        {
            float act[CHS_WHEEL_NUM];
            uint8_t mask = 0U;

            for (i = 0; i < CHS_WHEEL_NUM; i++)
            {
                act[i] = s_wheel[i].act_rpm;
                if ((s_wheel[i].motor != NULL) && s_wheel[i].motor->measure.online)
                {
                    mask |= (uint8_t)(1U << i);
                }
            }

            mecanum_odom(act, &s_fdb.vx_act, &s_fdb.vy_act, &s_fdb.vw_act);
            s_fdb.online_mask = mask;
        }

        chassis_update_fdb();

#if ENABLE_DIAG_CHASSIS
        if ((uint32_t)(now - last_print) >= 500U)
        {
            last_print = now;
            log_printf("[CHS] st=%u md=%d on=%uX vx=%d vy=%d vw=%.2f "
                       "ref=%d/%d/%d/%d act=%d/%d/%d/%d\r\n",
                       bench_stage, (int)s_mode, s_fdb.online_mask,
                       (int)s_fdb.vx_cmd, (int)s_fdb.vy_cmd, s_fdb.vw_cmd,
                       (int)s_wheel[0].ref_rpm, (int)s_wheel[1].ref_rpm,
                       (int)s_wheel[2].ref_rpm, (int)s_wheel[3].ref_rpm,
                       (int)s_wheel[0].act_rpm, (int)s_wheel[1].act_rpm,
                       (int)s_wheel[2].act_rpm, (int)s_wheel[3].act_rpm);
        }
#endif

        osDelay(1);
    }
}

/* ------------------------------- 初始化与解算 ------------------------------ */

static void chassis_motor_init(void)
{
    uint8_t i;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        /* 每个轮子独立一个 PID 实例 */
        pid_config_t cfg = (pid_config_t)INIT_PID_CONFIG(
            CHS_SPEED_KP, CHS_SPEED_KI, CHS_SPEED_KD,
            CHS_SPEED_MAX, CHS_SPEED_MAX,
            PID_Trapezoid_Intergral | PID_Integral_Limit
            | PID_Derivative_On_Measurement | PID_DerivativeFilter);

        cfg.Derivative_LPF_RC = CHS_SPEED_D_LPF_RC;

        dj_motor_config_t mcfg;

        s_wheel[i].pid_speed = pid_register(&cfg);

        memset(&mcfg, 0, sizeof(mcfg));
        mcfg.type    = DJ_MOTOR_M3508;
        mcfg.bus     = CAN_CHASSIS;
        mcfg.id      = s_wheel_cfg[i].id;
        mcfg.control = wheel_ctrl_cb;
        mcfg.ctx     = &s_wheel[i];
        s_wheel[i].motor = dj_motor_register(&mcfg);
    }
}

/**
 * @brief 底盘逆解
 * @param vx mm/s  vy mm/s  vw rad/s
 * @param out_rpm 4 个电机目标转速（rpm）
 *
 * 轮序：0 左前，1 右前，2 右后，3 左后
 * S = LENGTH_A + LENGTH_B，乘 60*19/周长 换算为 rpm。
 * 任一目标转速超过 CHS_MAX_WHEEL_RPM 时四轮等比缩放，保持合成方向不变。
 */
static void mecanum_calc(float vx, float vy, float vw, float *out_rpm)
{
    const float s = CHS_LENGTH_A + CHS_LENGTH_B;
    const float k = 60.0f / (CHS_WHEEL_PERIMETER * CHS_GEAR_RATIO);
    float peak = 0.0f;
    uint8_t i;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        float rpm = (vx * s_wheel_cfg[i].kx
                   + vy * s_wheel_cfg[i].ky
                   + vw * s * s_wheel_cfg[i].kw) * k;
        float mag = (rpm < 0.0f) ? -rpm : rpm;

        out_rpm[i] = rpm;

        if (mag > peak)
        {
            peak = mag;
        }
    }

    if (peak > CHS_MAX_WHEEL_RPM)
    {
        const float gain = CHS_MAX_WHEEL_RPM / peak;

        for (i = 0; i < CHS_WHEEL_NUM; i++)
        {
            out_rpm[i] *= gain;
        }
    }
}

/**
 * @brief 底盘正解（轮速 -> 底盘速度）
 */
static void mecanum_odom(const float *rpm, float *vx, float *vy, float *vw)
{
    const float s = CHS_LENGTH_A + CHS_LENGTH_B;
    const float k = (CHS_WHEEL_PERIMETER * CHS_GEAR_RATIO) / 60.0f;
    float sx = 0.0f;
    float sy = 0.0f;
    float sw = 0.0f;
    uint8_t i;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        sx += rpm[i] * s_wheel_cfg[i].kx;
        sy += rpm[i] * s_wheel_cfg[i].ky;
        sw += rpm[i] * s_wheel_cfg[i].kw;
    }

    *vx = sx * 0.25f * k;
    *vy = sy * 0.25f * k;
    *vw = sw * 0.25f * k / s;
}

/**
 * @brief 解算自检，上电时执行一次
 * @note  逆解->正解应还原输入；满杆输入必须被缩放到 CHS_MAX_WHEEL_RPM
 */
static void chassis_kin_selfcheck(void)
{
    const float vx = 500.0f;
    const float vy = 200.0f;
    const float vw = 0.5f;
    const float tol_v = 1.0f;
    const float tol_w = 0.01f;
    float rpm[CHS_WHEEL_NUM];
    float rpm_lim[CHS_WHEEL_NUM];
    float rx = 0.0f;
    float ry = 0.0f;
    float rw = 0.0f;
    float peak = 0.0f;
    float ex;
    float ey;
    float ew;
    uint8_t i;

    mecanum_calc(vx, vy, vw, rpm);
    mecanum_odom(rpm, &rx, &ry, &rw);

    ex = (rx > vx) ? (rx - vx) : (vx - rx);
    ey = (ry > vy) ? (ry - vy) : (vy - ry);
    ew = (rw > vw) ? (rw - vw) : (vw - rw);

    if ((ex > tol_v) || (ey > tol_v) || (ew > tol_w))
    {
        log_printf("[CHS] kin FAIL vx=%.1f/%.1f vy=%.1f/%.1f vw=%.3f/%.3f\r\n",
                   (double)rx, (double)vx, (double)ry, (double)vy,
                   (double)rw, (double)vw);
        return;
    }

    mecanum_calc(CHS_MAX_VX, 0.0f, CHS_MAX_VW, rpm_lim);

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        float mag = (rpm_lim[i] < 0.0f) ? -rpm_lim[i] : rpm_lim[i];

        if (mag > peak)
        {
            peak = mag;
        }
    }

    if ((peak > (CHS_MAX_WHEEL_RPM + 1.0f)) || (peak < (CHS_MAX_WHEEL_RPM - 1.0f)))
    {
        log_printf("[CHS] limit FAIL peak=%.0f max=%.0f\r\n",
                   (double)peak, (double)CHS_MAX_WHEEL_RPM);
    }
    else
    {
        log_printf("[CHS] kin ok, limit=%.0f rpm\r\n", (double)CHS_MAX_WHEEL_RPM);
    }
}

/**
 * @brief 单轮速度环，在 MotorTask 的 1kHz 上下文里跑
 */
static int16_t wheel_ctrl_cb(const dj_motor_measure_t *m, void *ctx)
{
    chs_wheel_t *w = (chs_wheel_t *)ctx;
    float out;

    if ((w == NULL) || (m == NULL) || (w->pid_speed == NULL))
    {
        return 0;
    }

    if (!m->online)
    {
        pid_clear(w->pid_speed);
        w->act_rpm = 0.0f;
        w->out     = 0;
        return 0;
    }

    /* sign 同时修正反馈和输出，避免只反输出导致速度环正反馈 */
    w->act_rpm = m->speed_rpm_f * w->sign;

    if (s_mode == CHASSIS_MODE_RELAX)
    {
        pid_clear(w->pid_speed);
        w->out = 0;
        return 0;
    }

    if (s_mode == CHASSIS_MODE_STOP)
    {
        /* 电刹：静止时清积分避免噪声累积出力，被外力推动时才抵抗 */
        if (fabsf(w->act_rpm) < CHS_LOCK_IDLE_RPM)
        {
            pid_clear(w->pid_speed);
            w->out = 0;
            return 0;
        }
    }

    /* 起步检测：目标从 0 变为非 0 时打开软启动窗口 */
    if ((fabsf(w->ref_rpm) > 1.0f) && (w->ref_prev <= 1.0f))
    {
        w->launch_tick = HAL_GetTick();
        w->launching   = 1U;
    }

    w->ref_prev = fabsf(w->ref_rpm);

    out = pid_calculate(w->pid_speed, w->act_rpm, w->ref_rpm);

    /* 静摩擦补偿：仅当输出与目标同向且幅值不足时补到最小有效值，
     * 反向输出说明已经超速，必须保留给 PID 去刹车 */
    if (w->ref_rpm > 0.5f)
    {
        if ((out >= 0.0f) && (out < CHS_SPEED_MIN_OUT)
            && (w->act_rpm < CHS_SPEED_STATIC_RPM))
        {
            out = CHS_SPEED_MIN_OUT;
        }
    }
    else if (w->ref_rpm < -0.5f)
    {
        if ((out <= 0.0f) && (out > -CHS_SPEED_MIN_OUT)
            && (w->act_rpm > -CHS_SPEED_STATIC_RPM))
        {
            out = -CHS_SPEED_MIN_OUT;
        }
    }

    /* 起步软启动：只压启动窗口内的输出上限，指令本身零延时 */
    if (w->launching != 0U)
    {
        uint32_t el = HAL_GetTick() - w->launch_tick;

        if (el < CHS_LAUNCH_MS)
        {
            float lim = CHS_SPEED_MAX * (CHS_LAUNCH_RATIO
                      + (1.0f - CHS_LAUNCH_RATIO) * (float)el / (float)CHS_LAUNCH_MS);

            if (out > lim)
            {
                out = lim;
            }
            else if (out < -lim)
            {
                out = -lim;
            }
        }
        else
        {
            w->launching = 0U;
        }
    }

    /* 功率限幅：直接压低电流指令，比缩速度指令响应更快、也更平滑。
     * 放在斜率限制之前，让限幅引起的变化同样被斜率摊平 */
    out *= power_get_scale();

    /* 输出斜率限制：把电流阶跃摊成斜坡，压平起步冲击而不拖慢目标响应 */
    {
        float delta = out - w->out_last;

        if (delta > CHS_OUT_SLEW)
        {
            out = w->out_last + CHS_OUT_SLEW;
        }
        else if (delta < -CHS_OUT_SLEW)
        {
            out = w->out_last - CHS_OUT_SLEW;
        }

        w->out_last = out;
    }

    w->out = (int16_t)(out * w->sign);

    return w->out;
}
