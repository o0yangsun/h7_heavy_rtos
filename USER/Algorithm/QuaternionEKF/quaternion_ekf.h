/**
  ******************************************************************************
  * @file    quaternion_ekf.h
  * @brief   四元数姿态 EKF，含陀螺零偏估计与卡方检验
  *
  *   状态量 x[6] = [q0, q1, q2, q3, bx, by]，z 轴零偏不参与估计。
  *   量测量为归一化重力方向，接口单位 rad/s、m/s^2，输出欧拉角 rad。
  ******************************************************************************
  */

#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float   q[4];           /**< w,x,y,z */
    float   yaw;            /**< rad，范围 ±pi */
    float   pitch;          /**< rad */
    float   roll;           /**< rad */
    float   yaw_total;      /**< rad，跨 ±pi 累计 */
    float   gyro_bias[3];   /**< rad/s，bias[2] 恒为 0 */
    uint8_t converged;      /**< 1 = 卡方检验通过，姿态可信 */
} quat_ekf_out_t;

/**
 * @brief 初始化滤波器
 * @param q0~q3    初始四元数
 * @param Q1       四元数过程噪声，默认 10
 * @param Q2       零偏过程噪声，默认 0.001
 * @param R        加速度量测噪声，默认 1000000
 * @param lambda   渐消因子，取值 (0, 1]，默认 1
 * @param lpf      加速度一阶低通时间常数，秒，0 表示不平滑
 */
void quat_ekf_init(float q0, float q1, float q2, float q3,
                   float Q1, float Q2, float R, float lambda, float lpf);

/**
 * @brief 一次预测 + 量测更新
 * @param gyro  rad/s，未减零偏的原始角速度
 * @param accel m/s^2
 * @param dt    更新周期，秒，调用方需保证在 (0, 0.05] 内
 */
void quat_ekf_update(const float gyro[3], const float accel[3], float dt);

/** 取当前姿态估计 */
void quat_ekf_get(quat_ekf_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_EKF_H */
