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
} chs_wheel_t;

static chs_wheel_t            s_wheel[CHS_WHEEL_NUM];
static volatile chassis_mode_e s_mode = CHASSIS_MODE_NORMAL;
static volatile float         s_cmd_vx = 0.0f;
static volatile float         s_cmd_vy = 0.0f;
static volatile float         s_cmd_vw = 0.0f;
static chassis_fdb_t          s_fdb;
static volatile uint8_t       s_bench_stage = 0U;
static const float            s_wheel_sign[CHS_WHEEL_NUM] = CHS_WHEEL_SIGN;
static const uint8_t          s_wheel_id[CHS_WHEEL_NUM]   = CHS_WHEEL_ID;
static const float            s_vw_sign[CHS_WHEEL_NUM]    = CHS_WHEEL_VW_SIGN;

/* 麦轮逆解系数，顺序 ID1~ID4：vx / vy / vw 对轮速的贡献 */
typedef struct
{
    float vx;
    float vy;
    float vw;
} chs_kin_t;

static const chs_kin_t s_kin[CHS_WHEEL_NUM] =
{
    {+1.0f, -1.0f, -1.0f},     /* ID1 左前 */
    {+1.0f, +1.0f, +1.0f},     /* ID2 右前 */
    {+1.0f, -1.0f, +1.0f},     /* ID3 右后 */
    {+1.0f, +1.0f, -1.0f},     /* ID4 左后 */
};

/* ------------------------------- 内部函数声明 ------------------------------ */

static void  chassis_motor_init(void);
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
        s_wheel[i].sign = s_wheel_sign[i];
    }

    chassis_motor_init();

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

        /* 从 CmdTask 取指令；失能或遥控掉线时进入 RELAX 并清零速度 */
#if CHASSIS_CMD_FROM_CMD_TASK
        {
            cmd_chassis_t c;
            float scale;

            cmd_get_chassis(&c);

            if ((c.online == 0U) || (c.mode == CMD_MODE_RELAX))
            {
                s_mode   = CHASSIS_MODE_RELAX;
                s_cmd_vx = 0.0f;
                s_cmd_vy = 0.0f;
                s_cmd_vw = 0.0f;
            }
            else
            {
                /* 功率限幅：INA226 没接时 power_get_scale() 恒为 1.0 */
                scale = power_get_scale();
                if (scale < 0.0f) { scale = 0.0f; }
                if (scale > 1.0f) { scale = 1.0f; }

                s_mode   = CHASSIS_MODE_NORMAL;
                s_cmd_vx = c.vx * scale;
                s_cmd_vy = c.vy * scale;
                s_cmd_vw = c.vw * scale;
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
            PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);

        dj_motor_config_t mcfg;

        s_wheel[i].pid_speed = pid_register(&cfg);

        memset(&mcfg, 0, sizeof(mcfg));
        mcfg.type    = DJ_MOTOR_M3508;
        mcfg.bus     = CAN_CHASSIS;
        mcfg.id      = s_wheel_id[i];
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
 */
static void mecanum_calc(float vx, float vy, float vw, float *out_rpm)
{
    const float s = CHS_LENGTH_A + CHS_LENGTH_B;
    const float k = 60.0f / (CHS_WHEEL_PERIMETER * CHS_GEAR_RATIO);
    uint8_t i;

    for (i = 0; i < CHS_WHEEL_NUM; i++)
    {
        out_rpm[i] = (vx * s_kin[i].vx
                    + vy * s_kin[i].vy
                    + vw * s * s_kin[i].vw * s_vw_sign[i]) * k;
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
        sx += rpm[i] * s_kin[i].vx;
        sy += rpm[i] * s_kin[i].vy;
        sw += rpm[i] * s_kin[i].vw * s_vw_sign[i];
    }

    *vx = sx * 0.25f * k;
    *vy = sy * 0.25f * k;
    *vw = sw * 0.25f * k / s;
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
    w->act_rpm = (float)m->speed_rpm * w->sign;

    if (s_mode == CHASSIS_MODE_RELAX)
    {
        pid_clear(w->pid_speed);
        w->out = 0;
        return 0;
    }

    out = pid_calculate(w->pid_speed, w->act_rpm, w->ref_rpm);

    w->out = (int16_t)(out * w->sign);

    return w->out;
}
