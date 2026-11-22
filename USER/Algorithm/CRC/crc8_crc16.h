/**
  ******************************************************************************
  * @file    crc8_crc16.h
  * @brief   裁判系统协议用的 CRC8 / CRC16 查表校验
  *
  *   帧头 4 字节用 CRC8（初值 0xFF），整帧用 CRC16（初值 0xFFFF）。
  ******************************************************************************
  */

#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CRC8_INIT_VALUE         0xFFU
#define CRC16_INIT_VALUE        0xFFFFU

/** 计算 CRC8，init 通常传 CRC8_INIT_VALUE */
uint8_t crc8_get_check_sum(const uint8_t *data, uint32_t len, uint8_t init);

/** 计算 CRC16，init 通常传 CRC16_INIT_VALUE */
uint16_t crc16_get_check_sum(const uint8_t *data, uint32_t len, uint16_t init);

/**
 * @brief 校验带 CRC8 尾字节的帧头
 * @param data 帧头起始
 * @param len  帧头总长（含最后 1 字节 CRC8）
 * @return 1 = 通过
 */
uint8_t crc8_verify_check_sum(const uint8_t *data, uint32_t len);

/**
 * @brief 校验带 CRC16 尾两字节的整帧
 * @param data 帧起始
 * @param len  整帧长度（含最后 2 字节 CRC16，小端）
 * @return 1 = 通过
 */
uint8_t crc16_verify_check_sum(const uint8_t *data, uint32_t len);

/** 把 CRC8 写到 data[len-1] */
void crc8_append_check_sum(uint8_t *data, uint32_t len);

/** 把 CRC16 小端写到 data[len-2..len-1] */
void crc16_append_check_sum(uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* CRC8_CRC16_H */
