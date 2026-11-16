/**
  ******************************************************************************
  * @file    gimbal_task.h
  * @brief   云台任务：yaw(CAN2) / pitch(CAN3) 双 GM6020 直驱
  *
  * 控制结构：编码器角度环 -> 角速度环 -> 电压输出。
  ******************************************************************************
  */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** 云台工作模式 */
typedef enum
{
    GIMBAL_MODE_RELAX = 0,      /**< 断力 */
    GIMBAL_MODE_HOME,           /**< 回中，到位后自动切 ANGLE */
    GIMBAL_MODE_ANGLE,          /**< 角度控制 */
    GIMBAL_MODE_SPEED,          /**< 速度控制 */
} gimbal_mode_e;

/** 云台反馈 */
typedef struct
{
    gimbal_mode_e mode;
    bool     homed;
    bool     online[2];         /**< 0=yaw 1=pitch */
    float    yaw_deg;           /**< ° */
    float    pitch_deg;         /**< ° */
    float    yaw_target;        /**< ° */
    float    pitch_target;      /**< ° */
    float    yaw_speed_dps;     /**< °/s */
    float    pitch_speed_dps;   /**< °/s */
    int16_t  yaw_out;           /**< 电压输出 */
    int16_t  pitch_out;         /**< 电压输出 */
} gimbal_fdb_t;

/** 设置工作模式 */
void gimbal_set_mode(gimbal_mode_e mode);

/**
 * @brief 设置目标角度（相对中位，单位 °）
 * @note  pitch 自动限幅到机械行程内
 */
void gimbal_set_target(float yaw_deg, float pitch_deg);

/** 设置目标角速度（°/s） */
void gimbal_set_speed(float yaw_dps, float pitch_dps);

/** 读反馈快照 */
void gimbal_get_fdb(gimbal_fdb_t *out);

/** 当前模式 */
gimbal_mode_e gimbal_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
