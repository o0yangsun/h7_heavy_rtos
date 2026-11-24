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
    float    power_peak;    /**< 上电以来的功率峰值 W */
    float    power_est;     /**< 电机反馈估算功率 W，未标定 */
    float    buffer_j;      /**< 能量缓冲余量（J） */
    float    scale;         /**< 功率比例 0~1 */
    uint32_t over_cnt;
    uint32_t i2c_err;
    uint8_t  src;           /**< 0 = 无数据，1 = INA226，2 = 超电上报，3 = 电机模型兜底 */
    uint8_t  cap_online;    /**< 超电在线 */
    uint8_t  cap_energy;    /**< 超电剩余能量 % */
    float    bat_power_w;   /**< 电池侧功率 W（超电 0x220 上报） */
    float    total_power_w; /**< 总线总功率 W（Pbat + Pcap） */
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
