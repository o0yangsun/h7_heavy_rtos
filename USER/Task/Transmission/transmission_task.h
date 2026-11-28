/**
  ******************************************************************************
  * @file    transmission_task.h
  * @brief   小电脑链路：BCP 帧收发、视觉指令快照与诊断
  ******************************************************************************
  */

#ifndef __TRANSMISSION_TASK_H__
#define __TRANSMISSION_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "trans_msg.h"

typedef struct
{
    uint32_t loop_cnt;
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t rx_0x12;
    uint32_t rx_0x22;
    uint32_t rx_0xf0;
    uint32_t unknown_id;
    uint32_t invalid_payload;
    uint32_t checksum_errors;
    uint32_t length_errors;
    uint32_t address_errors;
    uint32_t discarded_bytes;
    uint32_t ring_overflow;
    uint32_t tx_frames;
    uint32_t tx_0x23;
    uint32_t tx_0x24;
    uint32_t tx_busy;
    uint32_t tx_error;
    uint32_t last_rx_ms;
    uint8_t  last_id;
    uint8_t  last_len;
} trans_diag_t;

extern volatile trans_diag_t g_trans_diag;

/** 由 usart_task 逐字节喂入；ENABLE_WIRELESS=0 时不使用 */
void trans_rx_byte(uint8_t b);

/** 批量喂入接收字节，可在中断中调用 */
void trans_rx_push(const uint8_t *buf, uint16_t len);

/**
 * @brief 按 ID 发送一帧
 * @return 1 = 已交给 USB；0 = 参数错误/USB 未就绪/忙
 */
uint8_t trans_send_frame(uint8_t id, const void *data, uint8_t len);

/** 链路是否在线（收到任意合法帧） */
uint8_t trans_link_online(void);

/** 读视觉指令快照，valid 已按超时更新；本轮仅提供接口，控制层尚未接入 */
void trans_get_vision(trans_vision_cmd_t *out);

/** 读小电脑底盘指令快照；本轮仅提供接口，底盘尚未接入 */
void trans_get_pc_chassis(trans_pc_chassis_t *out);

/** 临时覆盖 0x23 上报的 gimbal_mode，范围 0~3；控制接入后改为按状态机映射 */
void trans_set_gimbal_mode(uint8_t mode);

void StartTransmissionTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSMISSION_TASK_H__ */
