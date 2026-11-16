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

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile power_diag_t g_power_diag;

static task_stat_t s_stat;
static float       s_scale = 1.0f;
static float       s_buffer = POWER_BUFFER_MAX_J * 0.5f;

/* ------------------------------- 内部函数 -------------------------------- */

/**
 * @brief 读一次母线电压和电流
 * @return 1 = 读成功
 * @note  TODO 接入 INA226 后在此读取实际值
 */
static uint8_t power_read_bus(float *voltage, float *current)
{
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
    uint32_t last_ms    = HAL_GetTick();

    (void)argument;

    memset((void *)&g_power_diag, 0, sizeof(g_power_diag));
    task_stat_init(&s_stat, POWER_PERIOD_MS);

    s_scale  = 1.0f;
    s_buffer = POWER_BUFFER_MAX_J * 0.5f;

    log_printf("[PWR] up: %uHz, ina226=%d limit=%.0fW\r\n",
               (unsigned)(1000U / POWER_PERIOD_MS),
               (int)ENABLE_INA226, (double)POWER_LIMIT_W);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        float    dt  = (float)(now - last_ms) / 1000.0f;
        float    v = 0.0f, i = 0.0f, p;

        task_stat_begin(&s_stat, now);
        last_ms = now;

        /* 采样 */
        if (power_read_bus(&v, &i) == 0U)
        {
            g_power_diag.i2c_err++;
        }

        p = v * i;

        /* 能量缓冲 */
        if (dt > 0.0f)
        {
            if (p > 0.0f)
            {
                s_buffer += (POWER_LIMIT_W - p) * dt;
                s_buffer = float_constrain(s_buffer, 0.0f, POWER_BUFFER_MAX_J);
            }

            if (s_buffer <= 0.0f)
            {
                /* 缓冲见底：按超出的比例压功率，最低压到 30% */
                float over = (p > POWER_LIMIT_W) ? (POWER_LIMIT_W / p) : 1.0f;
                s_scale = float_constrain(over, 0.3f, 1.0f);
                g_power_diag.over_cnt++;
            }
            else if (s_buffer >= (POWER_BUFFER_MAX_J * 0.9f))
            {
                s_scale = 1.0f;
            }
        }

        /* 诊断 */
        g_power_diag.loop_cnt  = s_stat.loop_cnt;
        g_power_diag.voltage_v = v;
        g_power_diag.current_a = i;
        g_power_diag.power_w   = p;
        g_power_diag.buffer_j  = s_buffer;
        g_power_diag.scale     = s_scale;

#if ENABLE_DIAG_POWER
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            /* INA226 未接入时 i2c_err 会持续增长 */
            log_printf("[PWR] %.1fV %.2fA %.0fW buf=%.1f scale=%.2f err=%lu\r\n",
                       (double)v, (double)i, (double)p, (double)s_buffer,
                       (double)s_scale, g_power_diag.i2c_err);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(POWER_PERIOD_MS);
    }
}
