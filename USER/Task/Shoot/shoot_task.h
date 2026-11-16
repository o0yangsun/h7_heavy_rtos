/**
  ******************************************************************************
  * @file    shoot_task.h
  * @brief   发射机构线程
  *
  * 本车无发射机构，线程保留为空壳。
  ******************************************************************************
  */

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint32_t loop_cnt;
    uint8_t  enabled;
} shoot_diag_t;

extern volatile shoot_diag_t g_shoot_diag;

#ifdef __cplusplus
}
#endif

#endif /* __SHOOT_TASK_H__ */
