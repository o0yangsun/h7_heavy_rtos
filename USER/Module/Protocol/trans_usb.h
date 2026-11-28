/**
  ******************************************************************************
  * @file    trans_usb.h
  * @brief   小电脑链路 USB CDC 后端
  ******************************************************************************
  */

#ifndef __TRANS_USB_H__
#define __TRANS_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** USB 中断中调用，只把字节写入环形缓冲 */
void trans_usb_rx_isr(const uint8_t *buf, uint16_t len);

/** 任务中读取环形缓冲，返回实际读出字节数 */
uint16_t trans_usb_read(uint8_t *buf, uint16_t cap);

/** 发送一帧；调用者保证 buffer 在发送完成前有效 */
uint8_t trans_usb_tx(const uint8_t *frame, uint16_t len);

/** USB 是否已完成枚举并配置 */
uint8_t trans_usb_ready(void);

uint32_t trans_usb_overflow_count(void);
uint32_t trans_usb_tx_busy_count(void);
uint32_t trans_usb_tx_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRANS_USB_H__ */
