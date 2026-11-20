/**
  ******************************************************************************
  * @file    imu_heat.h
  * @brief   IMU 加热丝温度控制：TIM3_CH4 (PB1) PWM，PI + 滞环
  *
  *   目标温度见 IMU_HEAT_TARGET_C，温度低于目标时按比例输出占空比。
  ******************************************************************************
  */

#ifndef IMU_HEAT_H
#define IMU_HEAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 启动 PWM 通道，清状态，上电调用一次 */
void imu_heat_init(void);

/**
 * @brief 温度闭环更新
 * @param temp_c 当前 IMU 温度（℃），无效时传 0 关闭加热
 * @note  100 ms 左右调用一次，内部含 PI 与占空比限幅
 */
void imu_heat_update(float temp_c);

/** 0 = 关闭，1 = 使能；关闭时立即停 PWM */
void imu_heat_set_enable(uint8_t enable);

/** 当前输出占空比 0~100 % */
float imu_heat_get_duty(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_HEAT_H */
