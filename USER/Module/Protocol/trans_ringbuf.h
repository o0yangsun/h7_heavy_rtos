/**
  ******************************************************************************
  * @file    trans_ringbuf.h
  * @brief   单生产者/单消费者环形缓冲，容量必须是 2 的幂
  ******************************************************************************
  */

#ifndef __TRANS_RINGBUF_H__
#define __TRANS_RINGBUF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint8_t          *buf;
    uint16_t          size;
    uint16_t          mask;
    volatile uint16_t head;      /**< 生产者写位置 */
    volatile uint16_t tail;      /**< 消费者读位置 */
    volatile uint32_t overflow;  /**< 因缓冲满而丢弃的字节数 */
} trans_ringbuf_t;

static inline void trans_ringbuf_init(trans_ringbuf_t *r, uint8_t *buf, uint16_t size)
{
    if ((r == 0) || (buf == 0) || (size == 0U))
    {
        return;
    }

    r->buf      = buf;
    r->size     = size;
    r->mask     = (uint16_t)(size - 1U);
    r->head     = 0U;
    r->tail     = 0U;
    r->overflow = 0U;
}

static inline uint8_t trans_ringbuf_push(trans_ringbuf_t *r, uint8_t byte)
{
    uint16_t next;

    if (r == 0)
    {
        return 0U;
    }

    next = (uint16_t)((r->head + 1U) & r->mask);
    if (next == r->tail)
    {
        r->overflow++;
        return 0U;
    }

    r->buf[r->head] = byte;
    r->head         = next;
    return 1U;
}

static inline uint8_t trans_ringbuf_pop(trans_ringbuf_t *r, uint8_t *byte)
{
    if ((r == 0) || (byte == 0))
    {
        return 0U;
    }
    if (r->tail == r->head)
    {
        return 0U;
    }

    *byte   = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1U) & r->mask);
    return 1U;
}

static inline uint16_t trans_ringbuf_count(const trans_ringbuf_t *r)
{
    if (r == 0)
    {
        return 0U;
    }
    return (uint16_t)((r->head - r->tail) & r->mask);
}

#ifdef __cplusplus
}
#endif

#endif /* __TRANS_RINGBUF_H__ */
