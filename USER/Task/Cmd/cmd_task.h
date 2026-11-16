/**
  ******************************************************************************
  * @file    cmd_task.h
  * @brief   指令线程：把遥控器输入翻译成整车指令
  ******************************************************************************
  */

#ifndef __CMD_TASK_H__
#define __CMD_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    CMD_MODE_RELAX = 0,     /**< 失能 */
    CMD_MODE_RC,            /**< 遥控器正常行驶 */
    CMD_MODE_SPIN,          /**< 小陀螺 */
    CMD_MODE_PC,            /**< 上位机/键鼠控车 */
    CMD_MODE_AUTO           /**< 自动/视觉 */
} cmd_mode_e;

typedef enum
{
    CMD_ARM_SRC_USER = 0,   /**< 自定义控制器 */
    CMD_ARM_SRC_PC,         /**< 上位机 */
    CMD_ARM_SRC_MCU         /**< 板载自动流程 */
} cmd_arm_src_e;

typedef struct
{
    float vx;               /**< mm/s，正 = 前进 */
    float vy;               /**< mm/s，正 = 左移 */
    float vw;               /**< rad/s，正 = 逆时针 */
    cmd_mode_e mode;
    uint8_t online;         /**< 1 = 指令有效 */
} cmd_chassis_t;

typedef struct
{
    float x;                /**< mm */
    float y;                /**< mm */
    float z;                /**< mm */
    float pitch;            /**< rad */
    float roll;             /**< rad */
    float gripper;          /**< 0=闭合 1=张开 */
    cmd_arm_src_e src;
    uint8_t enabled;        /**< 1 = 机械臂使能 */
    uint8_t online;
} cmd_arm_t;

extern volatile cmd_chassis_t g_cmd_chassis;
extern volatile cmd_arm_t     g_cmd_arm;

typedef struct
{
    uint32_t loop_cnt;
    uint32_t rc_frames;     /**< 收到的遥控帧数 */
    uint32_t rc_lost;       /**< 掉线次数 */
    cmd_mode_e mode;
    uint32_t mode_change_cnt;
    uint8_t  sw1;           /**< 最近一次拨杆值 */
    uint8_t  sw2;
} cmd_diag_t;

extern volatile cmd_diag_t g_cmd_diag;

/** 键鼠输入，PC 控车预留 */
void cmd_set_keyboard(uint16_t key_code, int16_t mouse_x, int16_t mouse_y, int16_t mouse_z);

/** 下游读取接口，拷贝一份返回 */
void cmd_get_chassis(cmd_chassis_t *out);
void cmd_get_arm(cmd_arm_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_TASK_H__ */
