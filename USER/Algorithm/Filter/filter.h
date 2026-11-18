/**
  ******************************************************************************
  * @file    filter.h
  * @brief   一阶低通与二阶 IIR 滤波器，定点周期调用
  *
  *   一阶低通按时间常数初始化，系数在初始化时一次算好，运行期只做乘加。
  ******************************************************************************
  */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 一阶低通 y(k) = y(k-1) + a * (x - y(k-1)) */
typedef struct
{
    float alpha;    /**< 0~1，越大跟随越快 */
    float y;        /**< 上次输出 */
    uint8_t ready;  /**< 0 = 首次输入直接作为输出 */
} lpf1_t;

/**
 * @brief 按时间常数初始化一阶低通
 * @param f     滤波器实例
 * @param tau_s 时间常数，秒；<=0 表示直通
 * @param dt_s  调用周期，秒
 */
void lpf1_init(lpf1_t *f, float tau_s, float dt_s);

/** 输入一个样本，返回滤波结果 */
float lpf1_update(lpf1_t *f, float x);

/** 复位到未初始化状态 */
void lpf1_reset(lpf1_t *f);

/** 二阶 IIR，系数为标准直接型 */
typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} iir2_t;

/** 直接给定系数，a0 归一化为 1 */
void iir2_init(iir2_t *f, float b0, float b1, float b2, float a1, float a2);

float iir2_update(iir2_t *f, float x);

void iir2_reset(iir2_t *f);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
