/**
  ******************************************************************************
  * @file    dwt.h
  * @brief   DWT 周期计数器时基：微秒时间戳与周期 dt
  *
  *   CYCCNT 为 32 位，480 MHz 下约 8.9 s 回绕，差值运算在 1 kHz 周期内无影响。
  ******************************************************************************
  */

#ifndef DWT_H
#define DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 使能 CYCCNT，取时之前调用一次 */
void dwt_init(void);

/**
 * @brief 自 dwt_init 起的累计微秒数
 * @note  约 71 分钟回绕，只用于相对时间
 */
uint32_t dwt_get_us(void);

/**
 * @brief 距上次调用的时间间隔，单位秒
 * @note  非线程安全，同一时刻只能由一个任务调用
 */
float dwt_get_dt_s(void);

#ifdef __cplusplus
}
#endif

#endif /* DWT_H */
