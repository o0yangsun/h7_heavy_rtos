/**
  ******************************************************************************
  * @file    special_task.c
  * @brief   特殊机构线程实现，详见 special_task.h
  ******************************************************************************
  */

#include "special_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile special_diag_t g_special_diag;

static task_stat_t     s_stat;
static special_state_e s_state = SPECIAL_IDLE;
static uint32_t        s_state_t0;
static volatile uint8_t s_req_grip;

/* ------------------------------- 内部函数 -------------------------------- */

/**
 * @brief 写机构输出
 * @note  TODO 引脚未定，当前使用 PC5（气泵）和 PD10（泄压阀）作为占位
 */
static void special_hw_write(uint8_t pump, uint8_t valve)
{
#if ENABLE_SPECIAL
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,  pump  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, valve ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)pump;
    (void)valve;
#endif
}

static void special_enter(special_state_e st, uint32_t now)
{
    if (st != s_state)
    {
        s_state    = st;
        s_state_t0 = now;
        g_special_diag.switch_cnt++;
        g_special_diag.state = st;
    }
}

/* --------------------------------- 接口 ---------------------------------- */

void special_request(uint8_t grip)
{
    s_req_grip = grip;
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief SpecialTask 入口
 */
void StartSpecialTask(void const *argument)
{
    uint32_t last_print = 0U;

    (void)argument;

    memset((void *)&g_special_diag, 0, sizeof(g_special_diag));
    task_stat_init(&s_stat, SPECIAL_PERIOD_MS);

    s_state    = SPECIAL_IDLE;
    s_state_t0 = HAL_GetTick();
    s_req_grip = 0U;

    /* 上电先确保机构是关的 */
    special_hw_write(0U, 0U);

    log_printf("[SPC] up: %uHz, enable=%d\r\n",
               (unsigned)(1000U / SPECIAL_PERIOD_MS), (int)ENABLE_SPECIAL);

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);

        /* 机构状态机 */
        switch (s_state)
        {
            case SPECIAL_IDLE:
                special_hw_write(0U, 0U);
                if (s_req_grip != 0U)
                {
                    special_enter(SPECIAL_PUMP_PRE, now);
                }
                break;

            case SPECIAL_PUMP_PRE:
                special_hw_write(1U, 0U);               /* 开泵，阀关 */
                if ((now - s_state_t0) >= SPECIAL_PRECHARGE_MS)
                {
                    special_enter(SPECIAL_HOLD, now);
                }
                break;

            case SPECIAL_HOLD:
                special_hw_write(1U, 0U);
                if (s_req_grip == 0U)
                {
                    special_enter(SPECIAL_RELEASE, now);
                }
                break;

            case SPECIAL_RELEASE:
                special_hw_write(0U, 1U);               /* 泵停，阀开泄压 */
                if ((now - s_state_t0) >= SPECIAL_PRECHARGE_MS)
                {
                    special_enter(SPECIAL_IDLE, now);
                }
                break;

            default:
                special_enter(SPECIAL_IDLE, now);
                break;
        }

        g_special_diag.pump_on    = (uint8_t)((s_state == SPECIAL_PUMP_PRE) || (s_state == SPECIAL_HOLD));
        g_special_diag.valve_open = (uint8_t)(s_state == SPECIAL_RELEASE);
        g_special_diag.loop_cnt   = s_stat.loop_cnt;

#if ENABLE_DIAG_SPECIAL
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[SPC] state=%d pump=%u valve=%u req=%u\r\n",
                       (int)s_state, (unsigned)g_special_diag.pump_on,
                       (unsigned)g_special_diag.valve_open, (unsigned)s_req_grip);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(SPECIAL_PERIOD_MS);
    }
}
