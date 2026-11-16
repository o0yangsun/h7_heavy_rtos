/**
  ******************************************************************************
  * @file    rc_dbus.h
  * @brief   DT7/DR16 DBUS 遥控解析
  ******************************************************************************
  */

#ifndef RC_DBUS_H
#define RC_DBUS_H

#include <stdint.h>

#define RC_DBUS_FRAME_LEN       18U
#define RC_DBUS_MAX_VALUE       660

/** DBUS 拨杆值 */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};

typedef struct
{
    int16_t ch1;        /**< ch0 右侧水平，-660 ~ 660 */
    int16_t ch2;        /**< ch1 右侧竖直 */
    int16_t ch3;        /**< ch2 左侧水平 */
    int16_t ch4;        /**< ch3 左侧竖直 */
    uint8_t sw1;        /**< 左拨杆 1/3/2 */
    uint8_t sw2;        /**< 右拨杆 */
    int16_t wheel;      /**< 左侧拨轮，-660 ~ 660 */
} rc_dbus_obj_t;

/** 最近一帧有效数据，中断中更新 */
extern volatile rc_dbus_obj_t dbus_data_fdb;

/** 每收到一帧有效数据加一 */
extern volatile uint32_t dbus_heartbeat;

/** 清零数据和心跳 */
void dbus_data_init(void);

/**
 * @brief 解析一帧 DBUS
 * @param data 18 字节帧
 * @param len 帧长
 * @return 1 = 数据有效
 */
uint8_t dbus_data_unpack(const uint8_t *data, uint16_t len);

#endif /* RC_DBUS_H */
