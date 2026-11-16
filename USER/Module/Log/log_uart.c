/**
  ******************************************************************************
  * @file    log_uart.c
  * @brief   printf 重定向到 UART7，详见 log_uart.h
  ******************************************************************************
  */

#include "log_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

/** 初始化标志：未初始化前丢弃字符 */
static volatile uint8_t s_log_ready = 0;

/** 多任务打印互斥锁，NULL 时不做互斥 */
static SemaphoreHandle_t s_log_mutex = NULL;

void log_uart_init(void)
{
    if (s_log_mutex == NULL)
    {
        s_log_mutex = xSemaphoreCreateMutex();
    }
    s_log_ready = 1;
}

/**
 * @brief newlib 单字符输出钩子
 */
int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;

    if (s_log_ready)
    {
        (void)HAL_UART_Transmit(&huart7, &byte, 1, 5);
    }
    return ch;
}

void log_printf(const char *fmt, ...)
{
    char buf[LOG_BUF_SIZE];
    va_list ap;
    int len;

    if ((fmt == NULL) || (s_log_ready == 0U))
    {
        return;
    }

    /* 先格式化到本地缓冲，再一次性输出，避免半行交错 */
    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0)
    {
        return;
    }
    if (len >= (int)sizeof(buf))
    {
        len = (int)sizeof(buf) - 1;
    }

    if (s_log_mutex != NULL)
    {
        /* 最多等 20ms，宁可丢本行也不阻塞控制任务 */
        if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        {
            return;
        }
    }

    (void)HAL_UART_Transmit(&huart7, (uint8_t *)buf, (uint16_t)len, 50);

    if (s_log_mutex != NULL)
    {
        (void)xSemaphoreGive(s_log_mutex);
    }
}
