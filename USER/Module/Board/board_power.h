/**
  ******************************************************************************
  * @file    board_power.h
  * @brief   DM-MC02 板级电源使能
  *
  *   PC15 = DC5_OUTPUT   对外 5V（CAN 收发器、PWM 排针供电）
  *   PC14 = DC24_0_OUT   可控 24V 输出 1
  *   PC13 = DC24_1_OUT   可控 24V 输出 2
  *
  * 对外 5V 不使能时 CAN 无法收发。
  ******************************************************************************
  */

#ifndef __BOARD_POWER_H__
#define __BOARD_POWER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使能对外 5V
 * @note  必须在 MX_GPIO_Init() 之后、CAN 通信之前调用。
 *        两路 24V 输出保持关闭。
 */
void board_power_init(void);

/** 单独控制两路 24V 输出（默认都是关） */
void board_power_set_24v(uint8_t ch0_on, uint8_t ch1_on);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_POWER_H__ */
