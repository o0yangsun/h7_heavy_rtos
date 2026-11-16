/**
  ******************************************************************************
  * @file    referee_task.h
  * @brief   裁判系统线程：0xA5 拆包 + CRC8 校验 + cmd_id 分发
  ******************************************************************************
  */

#ifndef __REFEREE_TASK_H__
#define __REFEREE_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define REFEREE_SOF             0xA5U
#define REFEREE_MAX_DATA        128U

typedef struct
{
    uint32_t loop_cnt;
    uint32_t frames;            /**< 合法帧数 */
    uint32_t crc8_err;
    uint32_t len_err;
    uint32_t last_ms;
    uint16_t last_cmd_id;
    uint16_t game_progress;     /**< 0x0001 比赛阶段 */
    uint16_t robot_hp;          /**< 0x0003 血量 */
    uint16_t last_cmd_id_list[8];
    uint8_t  list_idx;
} referee_diag_t;

extern volatile referee_diag_t g_referee_diag;

/** 逐字节喂入，中断中调用 */
void referee_feed(uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_TASK_H__ */
