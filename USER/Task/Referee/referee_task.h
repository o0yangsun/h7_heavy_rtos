/**
  ******************************************************************************
  * @file    referee_task.h
  * @brief   裁判系统线程：周期汇总解析结果，供日志与上层取数
  ******************************************************************************
  */

#ifndef __REFEREE_TASK_H__
#define __REFEREE_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/** 线程入口，由 freertos.c 创建 */
void StartRefereeTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_TASK_H__ */
