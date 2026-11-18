/**
  ******************************************************************************
  * @file    ramp.h
  * @brief   线性斜坡发生器，用于速度/角度指令的平滑
  ******************************************************************************
  */

#ifndef RAMP_H
#define RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float step;     /**< 单周期最大变化量 */
    float y;        /**< 当前输出 */
} ramp_t;

/**
 * @brief 按变化率初始化
 * @param r         实例
 * @param rate_per_s 每秒最大变化量；<=0 表示直通
 * @param dt_s      调用周期，秒
 */
void ramp_init(ramp_t *r, float rate_per_s, float dt_s);

/** 逼近目标值，返回本周期输出 */
float ramp_update(ramp_t *r, float target);

/** 直接置位当前输出 */
void ramp_set(ramp_t *r, float value);

#ifdef __cplusplus
}
#endif

#endif /* RAMP_H */
