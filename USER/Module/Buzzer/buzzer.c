/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   蜂鸣器实现，详见 buzzer.h
  ******************************************************************************
  */

#include "buzzer.h"
#include "main.h"
#include "tim.h"

#include <string.h>

/* APB1 定时器时钟，TIM12 由此分频 */
#define BUZZER_TIMER_CLK        240000000U
#define BUZZER_MAX_NOTES        8U

typedef struct
{
    uint16_t freq_hz;
    uint16_t on_ms;
    uint16_t gap_ms;
} buzzer_note_t;

/* 上电提示音：Mavic Air 2 开机旋律，C5-D5-G5 上行三音 */
static const buzzer_note_t s_startup_seq[] =
{
    {523U, 250U, 30U},      /* C5 */
    {587U, 150U, 30U},      /* D5 */
    {784U, 400U, 0U},       /* G5 */
};

static buzzer_note_t s_note[BUZZER_MAX_NOTES];
static uint8_t  s_note_num;
static uint8_t  s_note_idx;
static uint8_t  s_phase;        /* 0 = 出声，1 = 间隔 */
static uint16_t s_elapsed_ms;
static uint8_t  s_playing;

/** 按目标频率重设预分频与自动重装，占空比固定 50% */
static void buzzer_set_freq(uint16_t freq_hz)
{
    uint32_t psc;
    uint32_t arr;

    if (freq_hz == 0U)
    {
        return;
    }

    psc = (BUZZER_TIMER_CLK / ((uint32_t)freq_hz * 65536U)) + 1U;
    arr = (BUZZER_TIMER_CLK / (psc * (uint32_t)freq_hz)) - 1U;

    if (arr > 65535U)
    {
        arr = 65535U;
    }

    __HAL_TIM_SET_PRESCALER(&htim12, psc - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, arr / 2U);
}

/** 启动当前音符 */
static void buzzer_start_note(void)
{
    buzzer_set_freq(s_note[s_note_idx].freq_hz);
    s_phase      = 0U;
    s_elapsed_ms = 0U;
}

void buzzer_init(void)
{
    s_playing = 0U;

    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
}

void buzzer_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    s_playing = 0U;
}

void buzzer_play_tone(uint16_t freq_hz, uint16_t on_ms, uint16_t gap_ms, uint8_t repeat)
{
    uint8_t i;

    if ((freq_hz == 0U) || (on_ms == 0U) || (repeat == 0U))
    {
        return;
    }

    if (repeat > BUZZER_MAX_NOTES)
    {
        repeat = BUZZER_MAX_NOTES;
    }

    for (i = 0U; i < repeat; i++)
    {
        s_note[i].freq_hz = freq_hz;
        s_note[i].on_ms   = on_ms;
        s_note[i].gap_ms  = gap_ms;
    }

    s_note_num   = repeat;
    s_note_idx   = 0U;
    s_playing    = 1U;
    buzzer_start_note();
}

void buzzer_play_startup(void)
{
    memcpy(s_note, s_startup_seq, sizeof(s_startup_seq));

    s_note_num = (uint8_t)(sizeof(s_startup_seq) / sizeof(s_startup_seq[0]));
    s_note_idx = 0U;
    s_playing  = 1U;
    buzzer_start_note();
}

void buzzer_update(void)
{
    if (s_playing == 0U)
    {
        return;
    }

    s_elapsed_ms++;

    if (s_phase == 0U)
    {
        if (s_elapsed_ms >= s_note[s_note_idx].on_ms)
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
            s_phase      = 1U;
            s_elapsed_ms = 0U;
        }
    }
    else
    {
        if (s_elapsed_ms >= s_note[s_note_idx].gap_ms)
        {
            s_note_idx++;

            if (s_note_idx >= s_note_num)
            {
                s_playing = 0U;
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
            }
            else
            {
                buzzer_start_note();
            }
        }
    }
}

uint8_t buzzer_is_busy(void)
{
    return s_playing;
}
