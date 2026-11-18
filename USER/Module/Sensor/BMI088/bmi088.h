/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   BMI088 六轴驱动：初始化与数据读取
  *
  *   输出为芯片坐标系：角速度 rad/s，加速度 m/s^2，温度 ℃。
  *   安装方向修正由上层任务完成，驱动不感知机体坐标。
  ******************************************************************************
  */

#ifndef BMI088_H
#define BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float gyro[3];      /**< rad/s */
    float accel[3];     /**< m/s^2 */
    float temp_c;       /**< ℃ */
} bmi088_sample_t;

/** 初始化返回值，非 0 表示对应步骤失败 */
enum
{
    BMI088_OK                   = 0x00,
    BMI088_ERR_ACC_PWR_CTRL     = 0x01,
    BMI088_ERR_ACC_PWR_CONF     = 0x02,
    BMI088_ERR_ACC_CONF         = 0x03,
    BMI088_ERR_ACC_RANGE        = 0x04,
    BMI088_ERR_ACC_INT1_IO      = 0x05,
    BMI088_ERR_ACC_INT_MAP      = 0x06,
    BMI088_ERR_GYRO_RANGE       = 0x07,
    BMI088_ERR_GYRO_BANDWIDTH   = 0x08,
    BMI088_ERR_GYRO_LPM1        = 0x09,
    BMI088_ERR_GYRO_CTRL        = 0x0A,
    BMI088_ERR_GYRO_INT_CONF    = 0x0B,
    BMI088_ERR_GYRO_INT_MAP     = 0x0C,
    BMI088_ERR_NO_SENSOR        = 0xFF,
};

/**
 * @brief 复位并配置加速度计与陀螺仪
 * @return BMI088_OK 或错误码
 * @note  内部有 80 ms 阻塞延时，只在任务初始化阶段调用
 */
uint8_t bmi088_init(void);

/**
 * @brief 读一帧六轴数据
 * @param out 输出缓冲
 * @return 1 = 成功，0 = 参数为空或陀螺仪 ID 校验失败
 */
uint8_t bmi088_read(bmi088_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
