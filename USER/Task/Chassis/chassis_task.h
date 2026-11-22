/**
  ******************************************************************************
  * @file    chassis_task.h
  * @brief   底盘任务：麦轮解算 + 4 路 3508 速度环
  *
  * 输入：vx/vy 单位 mm/s，vw 单位 rad/s
  * 输出：电机目标转速 rpm（含 19:1 减速）
  ******************************************************************************
  */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /**< 断力，轮子自由 */
    CHASSIS_MODE_NORMAL,        /**< 跟随速度指令 */
    CHASSIS_MODE_STOP,          /**< 目标转速 0（电刹） */
} chassis_mode_e;

typedef struct
{
    chassis_mode_e mode;
    uint8_t  online_mask;       /**< bit0~3 = ID1~ID4 在线 */
    float    vx_cmd;            /**< mm/s */
    float    vy_cmd;            /**< mm/s */
    float    vw_cmd;            /**< rad/s */
    float    vx_act;            /**< mm/s */
    float    vy_act;            /**< mm/s */
    float    vw_act;            /**< rad/s */
    float    rpm_ref[4];        /**< 目标转速，顺序 ID1~ID4 */
    float    rpm_act[4];        /**< 实际转速，顺序 ID1~ID4 */
    int16_t  out[4];            /**< 电流输出，顺序 ID1~ID4 */
} chassis_fdb_t;

/**
 * @brief 下发行驶指令
 * @param vx mm/s（正=前进）
 * @param vy mm/s（正=左移）
 * @param vw rad/s（正=逆时针）
 */
void chassis_set_cmd(float vx, float vy, float vw);

/** 切换底盘模式 */
void chassis_set_mode(chassis_mode_e mode);

/** 读反馈快照 */
void chassis_get_fdb(chassis_fdb_t *out);

/** 用四个底盘电机的转矩电流与转速估算总功率 W，参数未标定，供功率前馈对比 */
float chassis_power_estimate(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TASK_H__ */
