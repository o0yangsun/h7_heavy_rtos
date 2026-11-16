/**
  ******************************************************************************
  * @file    motor_task.h
  * @brief   电机发送任务：1kHz 计算所有电机控制量并发送
  *
  * 业务电机由各任务自行注册，本任务只负责统一发帧和在线状态维护。
  ******************************************************************************
  */

#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 运行诊断信息 */
typedef struct
{
    uint8_t  motor_count;       /**< 已注册电机数 */
    uint16_t online_mask;       /**< 按注册顺序，bit0 为第 0 个 */
    uint8_t  reserved;
    uint32_t tick_ms;           /**< 最近一次更新的时基 */
    uint32_t tx_count;          /**< 累计发送帧数 */
    uint32_t tx_error;          /**< 累计发送失败帧数 */
    uint32_t rx_count[3];       /**< 三条总线各自的反馈帧数 */
    uint32_t busoff;            /**< bus-off 次数 */
    uint8_t  dead_mask;         /**< bit0/1/2 = CAN1/2/3 处于 bus-off 冷却期 */
    uint32_t rx_any[3];         /**< 各总线收到的任意标准帧数 */
    uint32_t last_id[3];        /**< 各总线最后收到的标准帧 ID */
} motor_diag_t;

extern volatile motor_diag_t g_motor_diag;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TASK_H__ */
