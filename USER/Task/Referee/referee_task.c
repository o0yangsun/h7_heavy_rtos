/**
  ******************************************************************************
  * @file    referee_task.c
  * @brief   裁判系统线程实现，详见 referee_task.h
  *
  * 帧格式：
  *   SOF(0xA5) | data_length(2, 小端) | seq(1) | CRC8(1) | cmd_id(2, 小端)
  *   | data(data_length-2 字节) | CRC16(2)
  * 其中 data_length 是「cmd_id + data」的长度，CRC8 校验前 4 字节。
  ******************************************************************************
  */

#include "referee_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "crc8.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile referee_diag_t g_referee_diag;

static task_stat_t s_stat;

static uint8_t s_buf[5U + REFEREE_MAX_DATA + 2U];
static uint8_t s_idx;
static uint16_t s_need;         /* 整帧字节数 */

/* ------------------------------- 内部函数 -------------------------------- */

/** 环形记录最近几条 cmd_id */
static void record_cmd(uint16_t cmd_id)
{
    g_referee_diag.last_cmd_id = cmd_id;
    g_referee_diag.last_cmd_id_list[g_referee_diag.list_idx & 0x07U] = cmd_id;
    g_referee_diag.list_idx++;
}

/** 完整帧分发，目前只解析最小字段 */
static void referee_dispatch(const uint8_t *frame, uint16_t cmd_id, const uint8_t *data, uint16_t len)
{
    (void)frame;

    g_referee_diag.frames++;
    g_referee_diag.last_ms = HAL_GetTick();
    record_cmd(cmd_id);

    switch (cmd_id)
    {
        case 0x0001U:   /* 比赛状态 */
            if (len >= 3U)
            {
                g_referee_diag.game_progress = (uint16_t)data[0];
            }
            break;

        case 0x0003U:   /* 机器人血量 */
            if (len >= 2U)
            {
                g_referee_diag.robot_hp = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
            }
            break;

        default:
            /* TODO 补充 0x0201 机器人状态、0x0202 功率热量、0x020C 伤害状态 */
            break;
    }
}

/* --------------------------------- 接口 ---------------------------------- */

void referee_feed(uint8_t b)
{
    if (s_idx == 0U)
    {
        if (b != REFEREE_SOF)
        {
            return;
        }
        s_buf[0] = b;
        s_idx    = 1U;
        return;
    }

    s_buf[s_idx++] = b;

    if (s_idx == 2U)
    {
        /* 低字节已到，先记住 */
        return;
    }

    if (s_idx == 3U)
    {
        /* data_length 两字节到齐：整帧 = 5(头) + (len - 2) + 2(CRC16) */
        uint16_t data_length = (uint16_t)(s_buf[1] | ((uint16_t)s_buf[2] << 8));

        if ((data_length < 2U) || (data_length > (REFEREE_MAX_DATA + 2U)))
        {
            g_referee_diag.len_err++;
            s_idx = 0U;
            return;
        }

        s_need = (uint16_t)(5U + (data_length - 2U) + 2U);
        return;
    }

    if ((s_need != 0U) && (s_idx >= s_need))
    {
        uint16_t data_length = (uint16_t)(s_buf[1] | ((uint16_t)s_buf[2] << 8));
        uint16_t cmd_id      = (uint16_t)(s_buf[5] | ((uint16_t)s_buf[6] << 8));

        if (crc8_calc(s_buf, 4U) == s_buf[4])
        {
            referee_dispatch(s_buf, cmd_id, &s_buf[7], (uint16_t)(data_length - 2U));
        }
        else
        {
            g_referee_diag.crc8_err++;
        }

        s_idx  = 0U;
        s_need = 0U;
    }
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief RefereeTask 入口
 */
void StartRefereeTask(void const *argument)
{
    uint32_t last_print = 0U;

    (void)argument;

    memset((void *)&g_referee_diag, 0, sizeof(g_referee_diag));
    task_stat_init(&s_stat, REFEREE_PERIOD_MS);

    s_idx  = 0U;
    s_need = 0U;

#if ENABLE_REFEREE
    /* TODO 串口分配后在此启动接收，并在 UART 回调中调用 referee_feed() */
    log_printf("[REF] enabled, waiting for uart hookup\r\n");
#else
    log_printf("[REF] up, uart NOT assigned (ENABLE_REFEREE=0)\r\n");
#endif

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);

        g_referee_diag.loop_cnt = s_stat.loop_cnt;

#if ENABLE_DIAG_REFEREE
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[REF] frames=%lu cmd=0x%04X hp=%u crc_err=%lu\r\n",
                       g_referee_diag.frames, (unsigned)g_referee_diag.last_cmd_id,
                       (unsigned)g_referee_diag.robot_hp, g_referee_diag.crc8_err);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(REFEREE_PERIOD_MS);
    }
}
