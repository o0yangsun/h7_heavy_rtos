/**
  ******************************************************************************
  * @file    crc8.h
  * @brief   CRC8 校验（多项式 0x31，初值 0xFF）
  ******************************************************************************
  */

#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

static inline uint8_t crc8_calc(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31U);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

#endif /* CRC8_H */
