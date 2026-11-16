/**
  ******************************************************************************
  * @file    ins_task.h
  * @brief   姿态解算线程
  *
  * BMI088 驱动和 EKF 尚未移植，姿态输出保持为 0。
  * CubeMX 将 MX_USB_DEVICE_Init() 放在 InsTask 的弱定义中，
  * 本文件提供强定义，因此 StartInsTask 内需要自行调用一次。
  ******************************************************************************
  */

#ifndef __INS_TASK_H__
#define __INS_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float yaw;              /**< rad */
    float pitch;            /**< rad */
    float roll;             /**< rad */
    float yaw_total;        /**< rad，累计角度 */
    float gyro[3];          /**< rad/s */
    float accel[3];         /**< m/s^2 */
    uint8_t valid;          /**< 1 = 数据有效 */
} ins_attitude_t;

typedef struct
{
    uint32_t loop_cnt;
    uint32_t dt_ms;
    uint32_t imu_ok;        /**< 1 = IMU 数据有效 */
    uint32_t usb_inited;    /**< 1 = USB 已初始化 */
    uint32_t err_cnt;
} ins_diag_t;

extern volatile ins_attitude_t g_ins;
extern volatile ins_diag_t     g_ins_diag;

/** 取姿态快照 */
void ins_get_attitude(ins_attitude_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __INS_TASK_H__ */
