/**
  ******************************************************************************
  * @file    bmi088_port.h
  * @brief   BMI088 硬件适配：SPI2 收发、片选与微秒延时
  ******************************************************************************
  */

#ifndef BMI088_PORT_H
#define BMI088_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** SPI 全双工单字节收发 */
uint8_t bmi088_port_spi_rw(uint8_t tx);

/**
 * @brief 片选控制
 * @param level 0 = 选中（CS 拉低），1 = 释放
 */
void bmi088_port_cs_accel(uint8_t level);
void bmi088_port_cs_gyro(uint8_t level);

/** 忙等延时，基于 DWT 周期计数器 */
void bmi088_port_delay_us(uint32_t us);

/** 毫秒延时，基于 HAL 时基，会阻塞当前任务 */
void bmi088_port_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_PORT_H */
