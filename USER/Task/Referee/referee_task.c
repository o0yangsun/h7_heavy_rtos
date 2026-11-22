/**
  ******************************************************************************
  * @file    referee_task.c
  * @brief   裁判系统线程实现，详见 referee_task.h
  *
  *   协议解析在 referee_system 模块里，由 UsartTask 的 DMA 空闲回调驱动，
  *   本线程只负责周期汇总与打印，不参与收包。
  ******************************************************************************
  */

#include "referee_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "referee_system.h"
#include "cmsis_os.h"
#include "main.h"

static task_stat_t s_stat;

/**
 * @brief RefereeTask 入口
 */
void StartRefereeTask(void const *argument)
{
    uint32_t last_print = 0U;

    (void)argument;

    task_stat_init(&s_stat, REFEREE_PERIOD_MS);

#if ENABLE_REFEREE
    log_printf("[REF] up, USART1 115200, 8N1\r\n");
#else
    log_printf("[REF] disabled by ENABLE_REFEREE\r\n");
#endif

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);

#if ENABLE_REFEREE && ENABLE_DIAG_REFEREE
        if ((now - last_print) >= 1000U)
        {
            referee_fdb_t fdb;

            last_print = now;
            referee_get_fdb(&fdb);

            log_printf("[REF] on=%u fr=%lu crc=%lu len=%lu hp=%u/%u pwr=%uW buf=%uJ bar=%u/%u\r\n",
                       (unsigned)fdb.online,
                       fdb.frames, fdb.crc_err, fdb.len_err,
                       (unsigned)fdb.robot_status.current_hp,
                       (unsigned)fdb.robot_status.maximum_hp,
                       (unsigned)fdb.robot_status.chassis_power_limit,
                       (unsigned)fdb.power_heat.buffer_energy,
                       (unsigned)fdb.power_heat.shooter_17mm_1_heat,
                       (unsigned)fdb.robot_status.shooter_heat_limit);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(REFEREE_PERIOD_MS);
    }
}
