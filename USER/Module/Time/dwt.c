/**
  ******************************************************************************
  * @file    dwt.c
  * @brief   DWT 时基实现，详见 dwt.h
  ******************************************************************************
  */

#include "dwt.h"
#include "main.h"

static uint32_t s_last_cycles;
static uint64_t s_acc_cycles;
static uint32_t s_dt_cycles;

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_last_cycles = 0U;
    s_acc_cycles  = 0U;
    s_dt_cycles   = 0U;
}

uint32_t dwt_get_us(void)
{
    uint32_t now    = DWT->CYCCNT;
    uint32_t per_us = SystemCoreClock / 1000000U;

    s_acc_cycles += (uint32_t)(now - s_last_cycles);
    s_last_cycles = now;

    if (per_us == 0U)
    {
        return 0U;
    }

    return (uint32_t)(s_acc_cycles / per_us);
}

float dwt_get_dt_s(void)
{
    uint32_t now   = DWT->CYCCNT;
    uint32_t delta = now - s_dt_cycles;

    s_dt_cycles = now;

    if (SystemCoreClock == 0U)
    {
        return 0.0f;
    }

    return (float)delta / (float)SystemCoreClock;
}
