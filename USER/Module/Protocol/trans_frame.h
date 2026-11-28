/**
  ******************************************************************************
  * @file    trans_frame.h
  * @brief   BCP 帧格式：0xFF + 目的地址 + ID + LEN + 载荷 + SC + AC
  *
  * 校验算法与小电脑侧 bcp_codec 一致：
  *   SC = 前 4+LEN 字节逐字节求和
  *   AC = 逐字节累加 SC
  ******************************************************************************
  */

#ifndef __TRANS_FRAME_H__
#define __TRANS_FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRANS_FRAME_HEAD            0xFFU
#define TRANS_FRAME_ADDR            0x05U
#define TRANS_FRAME_MAX_PAYLOAD     64U
#define TRANS_FRAME_OVERHEAD        6U      /**< 头(1)+地址(1)+ID(1)+LEN(1)+SC(1)+AC(1) */
#define TRANS_FRAME_MAX_SIZE        (TRANS_FRAME_OVERHEAD + TRANS_FRAME_MAX_PAYLOAD)
#define TRANS_FRAME_PARSER_BUF      128U

typedef struct
{
    uint8_t daddr;
    uint8_t id;
    uint8_t len;
    uint8_t data[TRANS_FRAME_MAX_PAYLOAD];
} trans_frame_t;

typedef struct
{
    uint8_t  buf[TRANS_FRAME_PARSER_BUF];
    uint16_t len;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t length_errors;
    uint32_t address_errors;
    uint32_t discarded_bytes;
} trans_frame_parser_t;

void trans_frame_parser_init(trans_frame_parser_t *p);

/**
 * @brief 喂入一个字节
 * @return 1 = 解析出一帧，out 有效；0 = 还需要更多字节
 */
uint8_t trans_frame_parser_feed(trans_frame_parser_t *p, uint8_t byte, trans_frame_t *out);

/**
 * @brief 组一帧
 * @return 帧总长度，0 表示参数错误或缓冲不足
 */
uint16_t trans_frame_build(uint8_t id, const uint8_t *payload, uint8_t len,
                           uint8_t *out, uint16_t cap);

#ifdef __cplusplus
}
#endif

#endif /* __TRANS_FRAME_H__ */
