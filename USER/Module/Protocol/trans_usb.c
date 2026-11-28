/**
  ******************************************************************************
  * @file    trans_usb.c
  * @brief   小电脑链路 USB CDC 后端
  ******************************************************************************
  */

#include "trans_usb.h"
#include "trans_ringbuf.h"
#include "trans_frame.h"
#include "robot.h"

#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

static uint8_t s_ring_buf[TRANS_RX_RING_SIZE];
static uint8_t s_tx_buf[TRANS_FRAME_MAX_SIZE];

static trans_ringbuf_t s_ring =
{
    .buf      = s_ring_buf,
    .size     = TRANS_RX_RING_SIZE,
    .mask     = (uint16_t)(TRANS_RX_RING_SIZE - 1U),
    .head     = 0U,
    .tail     = 0U,
    .overflow = 0U,
};

static volatile uint32_t s_tx_busy;
static volatile uint32_t s_tx_error;

void trans_usb_rx_isr(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len == 0U))
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        (void)trans_ringbuf_push(&s_ring, buf[i]);
    }
}

uint16_t trans_usb_read(uint8_t *buf, uint16_t cap)
{
    uint16_t count = 0U;

    if ((buf == 0) || (cap == 0U))
    {
        return 0U;
    }

    while ((count < cap) && (trans_ringbuf_pop(&s_ring, &buf[count]) != 0U))
    {
        count++;
    }

    return count;
}

uint8_t trans_usb_ready(void)
{
    if (hUsbDeviceHS.pClassData == 0)
    {
        return 0U;
    }
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0U;
    }

    return 1U;
}

uint8_t trans_usb_tx(const uint8_t *frame, uint16_t len)
{
    const USBD_CDC_HandleTypeDef *hcdc;
    uint8_t result;

    if ((frame == 0) || (len == 0U) || (len > TRANS_FRAME_MAX_SIZE))
    {
        return 0U;
    }
    if (trans_usb_ready() == 0U)
    {
        s_tx_error++;
        return 0U;
    }

    hcdc = (const USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (hcdc->TxState != 0U)
    {
        s_tx_busy++;
        return 0U;
    }

    /* CDC 发送是异步的，驱动会保留指针直到 IN 传输完成，
     * 因此必须拷贝到静态缓冲，不能把调用者的栈空间交给 USB。 */
    memcpy(s_tx_buf, frame, len);

    result = CDC_Transmit_HS(s_tx_buf, len);
    if (result == USBD_OK)
    {
        return 1U;
    }
    if (result == USBD_BUSY)
    {
        s_tx_busy++;
    }
    else
    {
        s_tx_error++;
    }

    return 0U;
}

uint32_t trans_usb_overflow_count(void)
{
    return s_ring.overflow;
}

uint32_t trans_usb_tx_busy_count(void)
{
    return s_tx_busy;
}

uint32_t trans_usb_tx_error_count(void)
{
    return s_tx_error;
}
