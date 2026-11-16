/**
  ******************************************************************************
  * @file    task_common.h
  * @brief   线程周期统计工具（header-only）
  *
  * 用法：
  *     static task_stat_t s_stat;
  *     task_stat_init(&s_stat, 1U);
  *     for (;;) {
  *         task_stat_begin(&s_stat, HAL_GetTick());
  *         ...
  *         task_stat_end(&s_stat, HAL_GetTick());
  *         osDelay(1);
  *     }
  ******************************************************************************
  */

#ifndef __TASK_COMMON_H__
#define __TASK_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    volatile uint32_t loop_cnt;     /**< 累计循环次数 */
    volatile uint32_t period_ms;    /**< 期望周期（0 = 不统计超时） */
    volatile uint32_t dt_ms;        /**< 本圈耗时 */
    volatile uint32_t dt_max_ms;    /**< 历史最大耗时 */
    volatile uint32_t overrun_cnt;  /**< 单圈耗时超过期望周期的次数 */
    volatile uint32_t mark_ms;      /**< 本圈起点时基（内部用） */
} task_stat_t;

static inline void task_stat_init(task_stat_t *s, uint32_t period_ms)
{
    if (s == 0)
    {
        return;
    }
    s->loop_cnt    = 0U;
    s->period_ms   = period_ms;
    s->dt_ms       = 0U;
    s->dt_max_ms   = 0U;
    s->overrun_cnt = 0U;
    s->mark_ms     = 0U;
}

static inline void task_stat_begin(task_stat_t *s, uint32_t now_ms)
{
    if (s == 0)
    {
        return;
    }
    s->mark_ms = now_ms;
}

static inline void task_stat_end(task_stat_t *s, uint32_t now_ms)
{
    uint32_t dt;

    if (s == 0)
    {
        return;
    }

    dt = now_ms - s->mark_ms;
    s->dt_ms = dt;

    if (dt > s->dt_max_ms)
    {
        s->dt_max_ms = dt;
    }
    if ((s->period_ms != 0U) && (dt > s->period_ms))
    {
        s->overrun_cnt++;
    }

    s->loop_cnt++;
}

#ifdef __cplusplus
}
#endif

#endif /* __TASK_COMMON_H__ */
