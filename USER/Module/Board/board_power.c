/**
  ******************************************************************************
  * @file    board_power.c
  * @brief   板级电源使能实现，详见 board_power.h
  ******************************************************************************
  */

#include "board_power.h"
#include "main.h"

void board_power_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 先定电平再切输出模式，避免上电瞬间 24V 输出出现毛刺 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);   /* 24V 输出关 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);                   /* 对外 5V 开 */

    gpio.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void board_power_set_24v(uint8_t ch0_on, uint8_t ch1_on)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, ch0_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ch1_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
