/**
  ******************************************************************************
  * @file    rc_dbus.c
  * @brief   DT7/DR16 DBUS 遥控解析
  ******************************************************************************
  */

#include "rc_dbus.h"

#include <string.h>

volatile rc_dbus_obj_t dbus_data_fdb;
volatile uint32_t      dbus_heartbeat;

void dbus_data_init(void)
{
    memset((void *)&dbus_data_fdb, 0, sizeof(dbus_data_fdb));
    dbus_heartbeat = 0U;
}

uint8_t dbus_data_unpack(const uint8_t *data, uint16_t len)
{
    rc_dbus_obj_t rc;
    int16_t ch[4];
    uint8_t i;

    if ((data == 0) || (len != RC_DBUS_FRAME_LEN))
    {
        return 0U;
    }

    ch[0] = (int16_t)((int32_t)((data[0] | ((uint16_t)data[1] << 8)) & 0x07FFU) - 1024);
    ch[1] = (int16_t)((int32_t)(((data[1] >> 3) | ((uint16_t)data[2] << 5)) & 0x07FFU) - 1024);
    ch[2] = (int16_t)((int32_t)((((data[2] >> 6) | ((uint16_t)data[3] << 2)
                       | ((uint16_t)data[4] << 10)) & 0x07FFU)) - 1024);
    ch[3] = (int16_t)((int32_t)(((data[4] >> 1) | ((uint16_t)data[5] << 7)) & 0x07FFU) - 1024);

    for (i = 0; i < 4U; i++)
    {
        if ((ch[i] <= 5) && (ch[i] >= -5))
        {
            ch[i] = 0;
        }
        if ((ch[i] > RC_DBUS_MAX_VALUE) || (ch[i] < -RC_DBUS_MAX_VALUE))
        {
            return 0U;
        }
    }

    rc.ch1 = ch[0];
    rc.ch2 = ch[1];
    rc.ch3 = ch[2];
    rc.ch4 = ch[3];

    rc.sw1   = (uint8_t)(((data[5] >> 4) & 0x0CU) >> 2);
    rc.sw2   = (uint8_t)((data[5] >> 4) & 0x03U);
    rc.wheel = (int16_t)(((data[16] | ((uint16_t)data[17] << 8)) & 0x07FFU) - 1024);

    /* 拨轮实测输出 -660~660，超出按量程钳位，避免字节错位凑出野值 */
    if (rc.wheel > RC_DBUS_MAX_VALUE)
    {
        rc.wheel = RC_DBUS_MAX_VALUE;
    }
    else if (rc.wheel < -RC_DBUS_MAX_VALUE)
    {
        rc.wheel = -RC_DBUS_MAX_VALUE;
    }

    if ((rc.wheel <= 5) && (rc.wheel >= -5))
    {
        rc.wheel = 0;
    }

    dbus_data_fdb = rc;
    dbus_heartbeat++;

    return 1U;
}
