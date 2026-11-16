/**
  ******************************************************************************
  * @file    log_uart.h
  * @brief   printf 重定向到 UART7（PE8，仅发送，115200）
  *
  * syscalls.c 的 _write() 调用 __io_putchar()，本模块提供其实现。
  * HAL_UART_Transmit 为阻塞发送，只应在低频路径调用。
  ******************************************************************************
  */

#ifndef __LOG_UART_H__
#define __LOG_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_BUF_SIZE            160U    /**< 单行格式化缓冲 */

/**
 * @brief 绑定 printf 到 UART7
 * @note  必须在 MX_UART7_Init() 之后调用（main 的 USER CODE 2 里）
 */
void log_uart_init(void);

/**
 * @brief 多任务安全的格式化输出（内部带互斥锁）
 * @note  多任务共用 newlib 时直接 printf 会串行，任务中统一用本接口。
 *        同样是阻塞实现，只放在低频路径。
 */
void log_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_UART_H__ */
