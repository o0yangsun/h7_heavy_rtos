/**
  ******************************************************************************
  * @file    trans_frame.c
  * @brief   BCP 帧组包与流式解析
  ******************************************************************************
  */

#include "trans_frame.h"

#include <string.h>

static void trans_frame_drop(trans_frame_parser_t *p, uint16_t count)
{
    if (count >= p->len)
    {
        p->len = 0U;
        return;
    }

    memmove(p->buf, &p->buf[count], (size_t)(p->len - count));
    p->len = (uint16_t)(p->len - count);
}

void trans_frame_parser_init(trans_frame_parser_t *p)
{
    if (p == 0)
    {
        return;
    }

    memset(p, 0, sizeof(*p));
}

uint8_t trans_frame_parser_feed(trans_frame_parser_t *p, uint8_t byte, trans_frame_t *out)
{
    uint16_t i;
    uint16_t head;
    uint8_t  payload_len;
    uint16_t frame_len;
    uint8_t  sum;
    uint8_t  add;

    if ((p == 0) || (out == 0))
    {
        return 0U;
    }

    if (p->len >= TRANS_FRAME_PARSER_BUF)
    {
        trans_frame_drop(p, 1U);
        p->discarded_bytes++;
    }
    p->buf[p->len++] = byte;

    for (;;)
    {
        head = 0U;
        while ((head < p->len) && (p->buf[head] != TRANS_FRAME_HEAD))
        {
            head++;
        }

        if (head == p->len)
        {
            p->discarded_bytes += p->len;
            p->len = 0U;
            return 0U;
        }

        if (head > 0U)
        {
            trans_frame_drop(p, head);
            p->discarded_bytes += head;
        }

        if (p->len < 4U)
        {
            return 0U;
        }

        if (p->buf[1] != TRANS_FRAME_ADDR)
        {
            p->address_errors++;
            trans_frame_drop(p, 1U);
            p->discarded_bytes++;
            continue;
        }

        payload_len = p->buf[3];
        if (payload_len > TRANS_FRAME_MAX_PAYLOAD)
        {
            p->length_errors++;
            trans_frame_drop(p, 1U);
            p->discarded_bytes++;
            continue;
        }

        frame_len = (uint16_t)(4U + payload_len + 2U);
        if (p->len < frame_len)
        {
            return 0U;
        }

        sum = 0U;
        add = 0U;
        for (i = 0U; i < (uint16_t)(4U + payload_len); i++)
        {
            sum = (uint8_t)(sum + p->buf[i]);
            add = (uint8_t)(add + sum);
        }

        if ((p->buf[4U + payload_len] != sum) || (p->buf[5U + payload_len] != add))
        {
            p->checksum_errors++;
            trans_frame_drop(p, 1U);
            p->discarded_bytes++;
            continue;
        }

        out->daddr = p->buf[1];
        out->id    = p->buf[2];
        out->len   = payload_len;
        if (payload_len > 0U)
        {
            memcpy(out->data, &p->buf[4], (size_t)payload_len);
        }

        p->frames++;
        trans_frame_drop(p, frame_len);
        return 1U;
    }
}

uint16_t trans_frame_build(uint8_t id, const uint8_t *payload, uint8_t len,
                           uint8_t *out, uint16_t cap)
{
    uint16_t i;
    uint16_t total;
    uint8_t  sum = 0U;
    uint8_t  add = 0U;

    if ((out == 0) || (len > TRANS_FRAME_MAX_PAYLOAD))
    {
        return 0U;
    }

    total = (uint16_t)(TRANS_FRAME_OVERHEAD + len);
    if (cap < total)
    {
        return 0U;
    }

    out[0] = TRANS_FRAME_HEAD;
    out[1] = TRANS_FRAME_ADDR;
    out[2] = id;
    out[3] = len;

    if (len > 0U)
    {
        if (payload == 0)
        {
            return 0U;
        }
        memcpy(&out[4], payload, (size_t)len);
    }

    for (i = 0U; i < (uint16_t)(4U + len); i++)
    {
        sum = (uint8_t)(sum + out[i]);
        add = (uint8_t)(add + sum);
    }

    out[4U + len] = sum;
    out[5U + len] = add;

    return total;
}
