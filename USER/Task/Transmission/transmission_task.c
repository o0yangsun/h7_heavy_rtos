/**
  ******************************************************************************
  * @file    transmission_task.c
  * @brief   传输线程实现，详见 transmission_task.h
  ******************************************************************************
  */

#include "transmission_task.h"
#include "cmd_task.h"
#include "ins_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "crc8.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile trans_diag_t g_trans_diag;

static task_stat_t s_stat;

/* 接收拼帧状态 */
static uint8_t  s_buf[TRANS_OVERHEAD + TRANS_MAX_PAYLOAD];
static uint8_t  s_idx;
static uint8_t  s_payload_len;

/* ------------------------------- 内部函数 -------------------------------- */

static void trans_dispatch(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    (void)payload;

    g_trans_diag.last_cmd  = cmd;
    g_trans_diag.last_rx_ms = HAL_GetTick();

    switch (cmd)
    {
        case 0x01U:
            /* TODO 解析车端状态帧 */
            break;

        default:
            break;
    }
}

/* --------------------------------- 接口 ---------------------------------- */

void trans_rx_byte(uint8_t b)
{
    g_trans_diag.rx_byte++;

    if (s_idx == 0U)
    {
        if (b == TRANS_SOF0)
        {
            s_buf[0] = b;
            s_idx    = 1U;
        }
        return;
    }

    if (s_idx == 1U)
    {
        if (b == TRANS_SOF1)
        {
            s_buf[1] = b;
            s_idx    = 2U;
        }
        else
        {
            s_idx = (b == TRANS_SOF0) ? 1U : 0U;   /* 允许 AA AA 55 这种粘连 */
        }
        return;
    }

    if (s_idx == 3U)
    {
        /* 长度字节越界直接丢帧，否则后面的字节全被吃掉 */
        if (b > TRANS_MAX_PAYLOAD)
        {
            s_idx = 0U;
            return;
        }
        s_payload_len = b;
    }

    s_buf[s_idx++] = b;

    if (s_idx >= (uint8_t)(TRANS_OVERHEAD + s_payload_len))
    {
        uint8_t  cmd = s_buf[2];
        uint32_t n   = (uint32_t)TRANS_OVERHEAD + s_payload_len;

        if (crc8_calc(s_buf, n - 1U) == s_buf[n - 1U])
        {
            g_trans_diag.rx_frames++;
            trans_dispatch(cmd, &s_buf[4], s_payload_len);
        }
        else
        {
            g_trans_diag.crc_err++;
        }

        s_idx         = 0U;
        s_payload_len = 0U;
    }
}

uint8_t trans_send(uint8_t cmd, const void *data, uint8_t len)
{
    uint8_t frame[TRANS_OVERHEAD + TRANS_MAX_PAYLOAD];
    uint8_t n = 0U;

    if (len > TRANS_MAX_PAYLOAD)
    {
        return 0U;
    }

    frame[n++] = TRANS_SOF0;
    frame[n++] = TRANS_SOF1;
    frame[n++] = cmd;
    frame[n++] = len;

    if ((data != 0) && (len > 0U))
    {
        memcpy(&frame[n], data, len);
        n = (uint8_t)(n + len);
    }

    frame[n] = crc8_calc(frame, n);
    n++;

#if ENABLE_TRANSMISSION
    if (HAL_UART_Transmit(&huart10, frame, n, 20U) != HAL_OK)
    {
        return 0U;
    }
#else
    /* 未开启：只做组帧验证，不上总线 */
    (void)frame;
    (void)n;
    return 0U;
#endif

    g_trans_diag.tx_frames++;
    return 1U;
}

uint8_t trans_link_online(void)
{
    return ((HAL_GetTick() - g_trans_diag.last_rx_ms) < TRANS_TIMEOUT_MS) ? 1U : 0U;
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief TransmissionTask 入口
 */
void StartTransmissionTask(void const *argument)
{
    uint32_t last_print = 0U;
    uint32_t last_tx    = 0U;

    (void)argument;

    memset((void *)&g_trans_diag, 0, sizeof(g_trans_diag));
    task_stat_init(&s_stat, TRANS_PERIOD_MS);

    s_idx         = 0U;
    s_payload_len = 0U;

    log_printf("[TRN] up: %uHz, enable=%d (0=只组帧不发)\r\n",
               (unsigned)(1000U / TRANS_PERIOD_MS), (int)ENABLE_TRANSMISSION);

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);

        /* 30Hz 上行：打包整车状态 */
        if ((now - last_tx) >= TRANS_PERIOD_MS)
        {
            uint8_t payload[12];
            int16_t vx, vy, vw;

            last_tx = now;

            cmd_chassis_t cmd;
            cmd_get_chassis(&cmd);

            /* 单位换算：mm/s → cm/s（int16 足够），rad/s → mrad/s */
            vx = (int16_t)(cmd.vx / 10.0f);
            vy = (int16_t)(cmd.vy / 10.0f);
            vw = (int16_t)(cmd.vw * 1000.0f);

            payload[0] = (uint8_t)(vx & 0xFF);
            payload[1] = (uint8_t)((vx >> 8) & 0xFF);
            payload[2] = (uint8_t)(vy & 0xFF);
            payload[3] = (uint8_t)((vy >> 8) & 0xFF);
            payload[4] = (uint8_t)(vw & 0xFF);
            payload[5] = (uint8_t)((vw >> 8) & 0xFF);
            payload[6] = (uint8_t)cmd.mode;
            payload[7] = cmd.online;

            (void)trans_send(0x01U, payload, 8U);
        }

        /* 诊断 */
        g_trans_diag.loop_cnt = s_stat.loop_cnt;

#if ENABLE_DIAG_TRANSMISSION
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[TRN] tx=%lu rx=%lu crc_err=%lu link=%u\r\n",
                       g_trans_diag.tx_frames, g_trans_diag.rx_frames,
                       g_trans_diag.crc_err, (unsigned)trans_link_online());
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
