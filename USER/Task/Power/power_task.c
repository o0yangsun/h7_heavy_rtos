/**
  ******************************************************************************
  * @file    power_task.c
  * @brief   功率线程实现，详见 power_task.h
  *
  * 能量缓冲模型：
  *   buffer += (LIMIT - P) * dt，限制在 [0, MAX]
  *   buffer 耗尽时按比例压底盘功率，回充到 MAX 后恢复。
  ******************************************************************************
  */

#include "power_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "main.h"
#include "supercap_task.h"
#include "ina226.h"
#include "chassis_task.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile power_diag_t g_power_diag;

static task_stat_t s_stat;
static float       s_scale = 1.0f;
static float       s_power_lpf = 0.0f;
static float       s_power_peak = 0.0f;

/* ------------------------------- 内部函数 -------------------------------- */

/**
 * @brief 读一次母线电压和电流
 * @return 1 = 读成功，0 = 无数据
 * @note  优先 INA226 实测；没有 INA226 时用超电上报的底盘功率反算电流。
 */
static uint8_t power_read_bus(float *voltage, float *current)
{
    float v = 0.0f;
    float i = 0.0f;

    g_power_diag.src = 0U;

#if ENABLE_INA226
    if (ina226_read(&v, &i) != 0U)
    {
        g_power_diag.src = 1U;
        *voltage = v;
        *current = i;
        return 1U;
    }
#endif

#if ENABLE_SUPERCAP
    if (supercap_is_online() != 0U)
    {
        /* 限幅对象是电池侧功率 Pbat，这是裁判系统与板端闭环共同的考核量；
         * TotalPower 含超电放电，不能直接和预算比较 */
        float w = supercap_get_bat_power();

        v = SUPERCAP_NOMINAL_VOLTAGE;
        g_power_diag.src = 2U;
        *voltage = v;
        *current = w / v;
        return 1U;
    }
#endif

    *voltage = 0.0f;
    *current = 0.0f;
    return 0U;
}

/* --------------------------------- 接口 ---------------------------------- */

float power_get_scale(void)
{
    return s_scale;
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief PowerTask 入口
 */
void StartPowerTask(void const *argument)
{
    uint32_t last_print = 0U;

    (void)argument;

    memset((void *)&g_power_diag, 0, sizeof(g_power_diag));
    task_stat_init(&s_stat, POWER_PERIOD_MS);

    s_scale     = 1.0f;
    s_power_lpf = 0.0f;

#if ENABLE_INA226
    ina226_init();
#endif

#if ENABLE_SUPERCAP
    supercap_init();
#endif

    log_printf("[PWR] up: %uHz, ina226=%d supercap=%d limit=%.0fW\r\n",
               (unsigned)(1000U / POWER_PERIOD_MS),
               (int)ENABLE_INA226, (int)ENABLE_SUPERCAP, (double)POWER_LIMIT_W);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        float    v = 0.0f, i = 0.0f, p;

        task_stat_begin(&s_stat, now);

#if ENABLE_SUPERCAP
        /* 超电收发与在线判定，发送周期等于本线程周期 */
        supercap_update();
#endif

        /* 采样 */
        if (power_read_bus(&v, &i) == 0U)
        {
            g_power_diag.i2c_err++;
        }

        p = v * i;

        /* 峰值记录必须在低通之前，记录的是超电板实际看到的瞬时值 */
        if (p > s_power_peak)
        {
            s_power_peak = p;
        }

        /* 功率低通：超电上报值在充放电切换时跳变明显，直接进缓冲会让 scale 抖动 */
        if (g_power_diag.src != 0U)
        {
            s_power_lpf += POWER_LPF_ALPHA * (p - s_power_lpf);
            p = s_power_lpf;
        }
        else
        {
            s_power_lpf = 0.0f;
        }

        /* 直接限幅，不做能量缓冲。
         * 比赛按裁判系统的 60J 缓冲判罚，软件侧再预支一遍等于双重冒险，
         * 这里超限立即按比例压低，低于 90% 才恢复，中间区间保持以避免抖动 */
        if (g_power_diag.src == 0U)
        {
            s_scale = 1.0f;
        }
        else if (p > POWER_LIMIT_W)
        {
            s_scale = float_constrain(POWER_LIMIT_W / p, 0.2f, 1.0f);
            g_power_diag.over_cnt++;
        }
        else if (p < (POWER_LIMIT_W * 0.9f))
        {
            s_scale = 1.0f;
        }
        /* 90%~100% 区间保持上一周期的值，形成迟滞 */

        /* 诊断 */
        g_power_diag.loop_cnt   = s_stat.loop_cnt;
        g_power_diag.voltage_v  = v;
        g_power_diag.current_a  = i;
        g_power_diag.power_w    = p;
        g_power_diag.power_peak = s_power_peak;
        g_power_diag.buffer_j   = POWER_LIMIT_W - p;    /* 功率余量 W，负数表示超限 */
        g_power_diag.power_est  = chassis_power_estimate();
        g_power_diag.scale      = s_scale;
#if ENABLE_SUPERCAP
        g_power_diag.cap_online    = g_supercap_fdb.online;
        g_power_diag.cap_energy    = g_supercap_fdb.energy_pct;
        g_power_diag.cap_power_w   = supercap_get_bat_power();
        g_power_diag.cap_chassis_w = (uint16_t)(g_supercap_fdb.total_power_w10 / 100U);
#endif

#if ENABLE_DIAG_POWER
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            /* err 增长说明两路数据源都没有有效采样 */
            log_printf("[PWR] src=%u %.1fV %.2fA %.0fW buf=%.1f scale=%.2f err=%lu "
                       "cap=%u %u%% %dW\r\n",
                       (unsigned)g_power_diag.src,
                       (double)v, (double)i, (double)p, (double)(POWER_LIMIT_W - p),
                       (double)s_scale, g_power_diag.i2c_err,
                       (unsigned)g_power_diag.cap_online,
                       (unsigned)g_power_diag.cap_energy,
                       (int)g_power_diag.cap_power_w);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(POWER_PERIOD_MS);
    }
}
