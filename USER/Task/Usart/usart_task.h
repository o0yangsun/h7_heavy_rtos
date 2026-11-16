/**
  ******************************************************************************
  * @file    usart_task.h
  * @brief   串口线程：DT7/DR16 DBUS 遥控解析和无线链路字节分发
  *
  *   UART5   100000 8E1   DBUS 遥控，18 字节帧由 IDLE 分隔
  *   USART10 115200       无线帧
  *   UART7   115200       日志输出
  *   USART1  921600       舵机总线
  ******************************************************************************
  */

#ifndef __USART_TASK_H__
#define __USART_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint32_t loop_cnt;
    uint32_t dbus_frames;   /**< 合法帧数 */
    uint32_t dbus_err;      /**< 长度或通道值非法次数 */
    uint32_t dbus_last_ms;  /**< 最近一帧时基 */
    uint32_t wl_bytes;      /**< USART10 收到的字节数 */
    uint32_t uart_err;      /**< HAL 错误回调次数 */
    uint32_t dbus_bytes;    /**< UART5 收到的原始字节数 */
    uint32_t uart_rearm;    /**< 接收重新武装次数 */
} usart_diag_t;

extern volatile usart_diag_t g_usart_diag;

/** DBUS 运行时注入，用于台架调试；通道值 -660~660，与解析后的 ch1~ch4 一一对应 */
typedef struct
{
    volatile uint32_t enable;
    volatile int32_t  ch1;      /**< 右侧水平 */
    volatile int32_t  ch2;      /**< 右侧竖直 */
    volatile int32_t  ch3;      /**< 左侧竖直 */
    volatile int32_t  ch4;      /**< 左侧水平 */
    volatile uint32_t sw1;
    volatile uint32_t sw2;
    volatile uint32_t period_ms;
    volatile uint32_t injected;
} dbus_inject_t;

extern volatile dbus_inject_t g_dbus_inject;

#ifdef __cplusplus
}
#endif

#endif /* __USART_TASK_H__ */
