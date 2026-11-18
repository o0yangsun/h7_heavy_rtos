/**
  ******************************************************************************
  * @file    bmi088_port.c
  * @brief   BMI088 硬件适配实现，详见 bmi088_port.h
  ******************************************************************************
  */

#include "bmi088_port.h"
#include "main.h"
#include "spi.h"

static uint8_t s_dwt_ready;

static void port_dwt_ensure(void)
{
    if (s_dwt_ready == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        s_dwt_ready = 1U;
    }
}

uint8_t bmi088_port_spi_rw(uint8_t tx)
{
    uint8_t rx = 0U;

    (void)HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, 10U);

    return rx;
}

void bmi088_port_cs_accel(uint8_t level)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bmi088_port_cs_gyro(uint8_t level)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bmi088_port_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    port_dwt_ensure();

    start = DWT->CYCCNT;
    ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

void bmi088_port_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
