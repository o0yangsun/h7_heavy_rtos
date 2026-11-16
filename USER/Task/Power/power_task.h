/**
  ******************************************************************************
  * @file    power_task.h
  * @brief   功率线程：采样母线电压电流，计算底盘功率限幅比例
  ******************************************************************************
  */

#ifndef __POWER_TASK_H__
#define __POWER_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint32_t loop_cnt;
    float    voltage_v;
    float    current_a;
    float    power_w;
    float    buffer_j;      /**< 能量缓冲余量（J） */
    float    scale;         /**< 功率比例 0~1 */
    uint32_t over_cnt;
    uint32_t i2c_err;
} power_diag_t;

extern volatile power_diag_t g_power_diag;

/**
 * @brief 取功率限幅比例，底盘每周期乘上去
 * @return 0.0 ~ 1.0
 */
float power_get_scale(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_TASK_H__ */
