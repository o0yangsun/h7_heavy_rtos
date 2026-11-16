/**
  ******************************************************************************
  * @file    special_task.h
  * @brief   特殊机构线程：气泵和泄压阀的开关时序
  ******************************************************************************
  */

#ifndef __SPECIAL_TASK_H__
#define __SPECIAL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    SPECIAL_IDLE = 0,
    SPECIAL_PUMP_PRE,       /**< 气泵预充 */
    SPECIAL_HOLD,           /**< 保持吸附 */
    SPECIAL_RELEASE         /**< 泄压松开 */
} special_state_e;

typedef struct
{
    uint32_t loop_cnt;
    special_state_e state;
    uint8_t pump_on;
    uint8_t valve_open;
    uint32_t switch_cnt;
    uint32_t err_cnt;
} special_diag_t;

extern volatile special_diag_t g_special_diag;

/** 请求吸附或松开 */
void special_request(uint8_t grip);

#ifdef __cplusplus
}
#endif

#endif /* __SPECIAL_TASK_H__ */
