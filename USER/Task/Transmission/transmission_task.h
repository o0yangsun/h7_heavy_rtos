/**
  ******************************************************************************
  * @file    transmission_task.h
  * @brief   上位机链路帧层：组帧、校验、超时和统计
  ******************************************************************************
  */

#ifndef __TRANSMISSION_TASK_H__
#define __TRANSMISSION_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRANS_SOF0              0xAAU
#define TRANS_SOF1              0x55U
#define TRANS_MAX_PAYLOAD       32U
#define TRANS_OVERHEAD          5U      /**< 帧头(2) + cmd(1) + len(1) + crc8(1) */

typedef struct
{
    uint32_t loop_cnt;
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t rx_byte;
    uint32_t crc_err;
    uint32_t last_rx_ms;
    uint16_t last_cmd;
} trans_diag_t;

extern volatile trans_diag_t g_trans_diag;

/** 由 usart_task 逐字节喂入 */
void trans_rx_byte(uint8_t b);

/**
 * @brief 主动发一帧
 * @param cmd 命令字
 * @param data 载荷，可为 NULL
 * @param len 载荷长度，<= TRANS_MAX_PAYLOAD
 * @return 1 = 已发出
 */
uint8_t trans_send(uint8_t cmd, const void *data, uint8_t len);

/** 链路是否在线 */
uint8_t trans_link_online(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSMISSION_TASK_H__ */
