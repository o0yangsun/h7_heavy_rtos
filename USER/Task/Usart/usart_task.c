/**
  ******************************************************************************
  * @file    usart_task.c
  * @brief   DT7/DR16 DBUS 遥控解析和无线链路字节分发
  ******************************************************************************
  */

#include "usart_task.h"
#include "transmission_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "rc_dbus.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"

#include <string.h>

/* DBUS 一帧 18 字节，缓冲留两帧余量，避免 IDLE 回调前溢出 */
#define DBUS_RX_BUF_LEN         36U

/* ------------------------------- 全局变量 -------------------------------- */

volatile usart_diag_t g_usart_diag;
volatile dbus_inject_t g_dbus_inject;

static task_stat_t s_stat;
static uint8_t      s_dbus_buf[DBUS_RX_BUF_LEN];

#if ENABLE_WIRELESS
static uint8_t s_wl_rx_byte;
#endif

/* ------------------------------ DBUS 解析 -------------------------------- */

/**
 * @brief 按给定通道偏移拼一帧 DBUS，仅用于运行时注入
 */
static void dbus_build_frame(uint8_t *f, const int32_t *ch_off, const uint32_t *sw)
{
    uint16_t raw[4];
    uint8_t  i;

    for (i = 0; i < 4U; i++)
    {
        int32_t v = 1024 + ch_off[i];

        if (v < 0)
        {
            v = 0;
        }
        else if (v > 2047)
        {
            v = 2047;
        }
        raw[i] = (uint16_t)v;
    }

    memset(f, 0, RC_DBUS_FRAME_LEN);
    f[0] = (uint8_t)(raw[0] & 0xFFU);
    f[1] = (uint8_t)((raw[0] >> 8) | ((raw[1] & 0x1FU) << 3));
    f[2] = (uint8_t)(((raw[1] >> 5) & 0x3FU) | ((raw[2] & 0x03U) << 6));
    f[3] = (uint8_t)((raw[2] >> 2) & 0xFFU);
    f[4] = (uint8_t)(((raw[2] >> 10) & 0x01U) | ((raw[3] & 0x7FU) << 1));
    f[5] = (uint8_t)(((raw[3] >> 7) & 0x0FU)
                   | ((sw[0] & 0x03U) << 6)
                   | ((sw[1] & 0x03U) << 4));
}

/* ----------------------------- 串口中断回调 ------------------------------ */

/**
 * @brief UART 空闲事件回调，用于 DBUS 帧定界
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if ENABLE_RC_DBUS
    if (huart->Instance == UART5)
    {
        g_usart_diag.dbus_bytes += Size;

        if (Size == RC_DBUS_FRAME_LEN)
        {
            if (dbus_data_unpack(s_dbus_buf, Size) != 0U)
            {
                g_usart_diag.dbus_frames++;
                g_usart_diag.dbus_last_ms = HAL_GetTick();
            }
            else
            {
                g_usart_diag.dbus_err++;
            }
        }
        else
        {
            g_usart_diag.dbus_err++;
        }

        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart5, s_dbus_buf, DBUS_RX_BUF_LEN);
        return;
    }
#endif
}

/**
 * @brief 单字节接收完成回调，仅无线链路使用
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        return;
    }

#if ENABLE_WIRELESS
    if (huart->Instance == USART10)
    {
        g_usart_diag.wl_bytes++;
        trans_rx_byte(s_wl_rx_byte);
        (void)HAL_UART_Receive_IT(&huart10, &s_wl_rx_byte, 1U);
        return;
    }
#endif
}

/**
 * @brief 串口错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        return;
    }

    g_usart_diag.uart_err++;

    /* 先清 PE/NE/FE/ORE，否则中断会立刻再次触发 */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_OREF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

#if ENABLE_RC_DBUS
    if (huart->Instance == UART5)
    {
        huart->RxState = HAL_UART_STATE_READY;
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart5, s_dbus_buf, DBUS_RX_BUF_LEN);
        g_usart_diag.uart_rearm++;
        return;
    }
#endif

#if ENABLE_WIRELESS
    if (huart->Instance == USART10)
    {
        huart->RxState = HAL_UART_STATE_READY;
        (void)HAL_UART_Receive_IT(&huart10, &s_wl_rx_byte, 1U);
        g_usart_diag.uart_rearm++;
        return;
    }
#endif
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief UsartTask 入口
 */
void StartUsartTask(void const *argument)
{
    uint32_t last_print = 0U;
    uint32_t last_inject = 0U;

    (void)argument;

    memset((void *)&g_usart_diag, 0, sizeof(g_usart_diag));
    memset((void *)&g_dbus_inject, 0, sizeof(g_dbus_inject));
    dbus_data_init();
    g_dbus_inject.period_ms = DBUS_INJECT_PERIOD_MS;
    task_stat_init(&s_stat, USART_PERIOD_MS);

#if ENABLE_RC_DBUS
#if ENABLE_RC_DBUS_RX_INVERT
    /*
     * CR2.RXINV 只能在 UE=0 时写，HAL_UART_Init 之后 UE 已置位，
     * 因此先关闭 UE，修改 CR2 后再重新使能。
     */
    huart5.Instance->CR1 &= ~USART_CR1_UE;
    while ((huart5.Instance->CR1 & USART_CR1_UE) != 0U)
    {
    }
    huart5.Instance->CR2 |= USART_CR2_RXINV;
    huart5.Instance->CR1 |= USART_CR1_UE;
#endif

    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart5, s_dbus_buf, DBUS_RX_BUF_LEN);
#endif

#if ENABLE_WIRELESS
    (void)HAL_UART_Receive_IT(&huart10, &s_wl_rx_byte, 1U);
#endif

    log_printf("[USR] up: dbus(UART5)=%d wireless(USART10)=%d\r\n",
               (int)ENABLE_RC_DBUS, (int)ENABLE_WIRELESS);

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);

#if ENABLE_RC_DBUS
        /*
         * 接收看门狗：以 CR1.RXNEIE 为准，而不是 RxState。
         * RxState 可能停在 BUSY_RX 但 RXNEIE 已被清除。
         */
        if ((huart5.Instance->CR1 & USART_CR1_RXNEIE) == 0U)
        {
            huart5.RxState = HAL_UART_STATE_READY;
            (void)HAL_UARTEx_ReceiveToIdle_IT(&huart5, s_dbus_buf, DBUS_RX_BUF_LEN);
            g_usart_diag.uart_rearm++;
        }
#endif

#if ENABLE_WIRELESS
        if ((huart10.Instance->CR1 & USART_CR1_RXNEIE) == 0U)
        {
            huart10.RxState = HAL_UART_STATE_READY;
            (void)HAL_UART_Receive_IT(&huart10, &s_wl_rx_byte, 1U);
            g_usart_diag.uart_rearm++;
        }
#endif

        /* 运行时注入 */
        if (g_dbus_inject.enable != 0U)
        {
            uint32_t period = (g_dbus_inject.period_ms != 0U) ? g_dbus_inject.period_ms : DBUS_INJECT_PERIOD_MS;

            if ((now - last_inject) >= period)
            {
                int32_t  ch[4];
                uint32_t sw[2];
                uint8_t  frame[RC_DBUS_FRAME_LEN];

                last_inject = now;

                ch[0] = g_dbus_inject.ch1;
                ch[1] = g_dbus_inject.ch2;
                ch[2] = g_dbus_inject.ch3;
                ch[3] = g_dbus_inject.ch4;

                sw[0] = g_dbus_inject.sw1;
                sw[1] = g_dbus_inject.sw2;

                dbus_build_frame(frame, ch, sw);
                (void)dbus_data_unpack(frame, RC_DBUS_FRAME_LEN);
                g_dbus_inject.injected++;
            }
        }

        g_usart_diag.loop_cnt = s_stat.loop_cnt;

#if ENABLE_DIAG_USART
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[USR] dbus=%lu err=%lu bytes=%lu rearm=%lu wl=%lu ch=%u/%u/%u/%u s=%u/%u\r\n",
                       g_usart_diag.dbus_frames, g_usart_diag.dbus_err,
                       g_usart_diag.dbus_bytes, g_usart_diag.uart_rearm,
                       g_usart_diag.wl_bytes,
                       (unsigned)(dbus_data_fdb.ch1 + 1024),
                       (unsigned)(dbus_data_fdb.ch2 + 1024),
                       (unsigned)(dbus_data_fdb.ch3 + 1024),
                       (unsigned)(dbus_data_fdb.ch4 + 1024),
                       (unsigned)dbus_data_fdb.sw1,
                       (unsigned)dbus_data_fdb.sw2);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
