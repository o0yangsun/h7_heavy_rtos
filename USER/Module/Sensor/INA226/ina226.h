/**
  ******************************************************************************
  * @file    ina226.h
  * @brief   INA226 母线电压/电流采样
  *
  *   使用 HAL I2C，需先在 CubeMX 启用 I2C1 并接 SDA/SCL。
  *   ENABLE_INA226 = 0 时本模块只提供空实现，不占用 I2C。
  ******************************************************************************
  */

#ifndef INA226_H
#define INA226_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define INA226_ADDR                 0x40U
#define INA226_SHUNT_OHM            0.01f       /* 采样电阻 */
#define INA226_CURRENT_LSB_A        0.00005f    /* 50 uA / LSB */

/** 配置平均次数与转换时间，失败不阻塞启动 */
void ina226_init(void);

/**
 * @brief 读一次母线电压和电流
 * @param bus_v    母线电压 V
 * @param current_a 母线电流 A
 * @return 1 = 成功
 */
uint8_t ina226_read(float *bus_v, float *current_a);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
