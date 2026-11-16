/**
  ******************************************************************************
  * @file    shoot_task.c
  * @brief   发射机构线程（工程车为空壳），详见 shoot_task.h
  ******************************************************************************
  */

#include "shoot_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

volatile shoot_diag_t g_shoot_diag;

static task_stat_t s_stat;

/**
 * @brief ShootTask 入口
 */
void StartShootTask(void const *argument)
{
    (void)argument;

    memset((void *)&g_shoot_diag, 0, sizeof(g_shoot_diag));
    task_stat_init(&s_stat, SHOOT_PERIOD_MS);

    g_shoot_diag.enabled = 0U;

    log_printf("[SHT] engineer robot: no shooter, task idle\r\n");

    for (;;)
    {
        task_stat_begin(&s_stat, HAL_GetTick());

        /* TODO 变体为步兵/英雄时补充摩擦轮、拨弹和热量限制逻辑 */

        g_shoot_diag.loop_cnt = s_stat.loop_cnt;

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(SHOOT_PERIOD_MS);
    }
}
