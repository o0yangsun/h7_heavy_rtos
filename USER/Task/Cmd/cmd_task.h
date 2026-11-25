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

typedef struct
{
    float vx;               /**< mm/s，正 = 前进 */
    float vy;               /**< mm/s，正 = 左移 */
    float vw;               /**< rad/s，正 = 逆时针 */
    cmd_mode_e mode;
    uint8_t online;         /**< 1 = 指令有效 */
} cmd_chassis_t;

extern volatile cmd_chassis_t g_cmd_chassis;

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

/**
 * @brief 超电使能请求
 * @return 1 = 请求超电工作（sw2 下档），0 = 请求停机
 * @note  ENABLE_SUPERCAP_BY_SW2 为 0 时恒返回 1
 */
uint8_t cmd_get_supercap_request(void);

/** 下游读取接口，拷贝一份返回 */
void cmd_get_chassis(cmd_chassis_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_TASK_H__ */
