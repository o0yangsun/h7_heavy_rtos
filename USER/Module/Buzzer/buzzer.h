/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   蜂鸣器驱动：TIM12_CH2 (PB15) PWM 输出，支持非阻塞音效序列
  *
  *   无源蜂鸣器靠 PWM 频率定音调，有源蜂鸣器只需要占空比非零。
  *   update 由周期任务调用，建议 1 ms 一次。
  ******************************************************************************
  */

#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 使能 PWM 通道，上电后调用一次 */
void buzzer_init(void);

/** 立即静音 */
void buzzer_off(void);

/** 播放上电提示音（三短声），非阻塞 */
void buzzer_play_startup(void);

/** 播放指定次数的提示音，非阻塞 */
void buzzer_play_tone(uint16_t freq_hz, uint16_t on_ms, uint16_t gap_ms, uint8_t repeat);

/** 推进音效状态机，周期调用 */
void buzzer_update(void);

/** 1 = 正在播放 */
uint8_t buzzer_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
