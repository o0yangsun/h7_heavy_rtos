/**
  ******************************************************************************
  * @file    trans_msg.h
  * @brief   小电脑链路消息编解码（0x12 / 0x22 / 0x23 / 0x24）
  ******************************************************************************
  */

#ifndef __TRANS_MSG_H__
#define __TRANS_MSG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "trans_frame.h"

#define TRANS_ID_CHASSIS_CMD        0x12U
#define TRANS_ID_VISION_CMD         0x22U
#define TRANS_ID_GIMBAL_FB          0x23U
#define TRANS_ID_SHOOT_STATUS       0x24U
#define TRANS_ID_HEARTBEAT          0xF0U

#define TRANS_VISION_PAYLOAD_LEN    17U
#define TRANS_GIMBAL_FB_PAYLOAD_LEN 25U
#define TRANS_SHOOT_PAYLOAD_LEN     7U
#define TRANS_CHASSIS_PAYLOAD_LEN   24U

typedef struct
{
    uint8_t  mode;              /**< 0 无目标, 1 有目标, 2 请求开火 */
    int32_t  yaw_mdeg;
    int32_t  yaw_vel_mdeg_s;
    int32_t  yaw_acc_mdeg_s2;
    int32_t  pitch_mdeg;
    float    yaw_rad;
    float    yaw_vel_rps;
    float    yaw_acc_rps2;
    float    pitch_rad;
    uint32_t tick_ms;
    uint32_t seq;
    uint8_t  valid;
} trans_vision_cmd_t;

typedef struct
{
    float    vx_mps;            /**< 已按小电脑侧约定还原符号 */
    float    vy_mps;
    float    vw_rps;
    uint32_t tick_ms;
    uint8_t  valid;
} trans_pc_chassis_t;

typedef struct
{
    float   continuous_deg;
    float   last_deg;
    uint8_t initialized;
} trans_yaw_unwrap_t;

/** 复位连续 yaw；下一次 update 以输入角作为新起点 */
void trans_yaw_unwrap_reset(trans_yaw_unwrap_t *u);

/** 把 ±180° 的 yaw 展开为连续角，跨圈/反向均可 */
float trans_yaw_unwrap_update(trans_yaw_unwrap_t *u, float yaw_deg);

/** 判断时间戳是否在 timeout 内，处理 32 位回绕 */
uint8_t trans_msg_is_fresh(uint32_t now_ms, uint32_t tick_ms, uint32_t timeout_ms);

/**
 * @brief 解析 0x22 视觉指令
 * @return 1 = 长度/模式合法，字段已写入
 */
uint8_t trans_msg_parse_vision(const trans_frame_t *f, trans_vision_cmd_t *out, uint32_t now_ms);

/**
 * @brief 解析 0x12 底盘指令
 * @note  小电脑发送顺序为 x, -y, z, wx, wy, -wz，缩放 10000
 * @return 1 = 长度合法
 */
uint8_t trans_msg_parse_pc_chassis(const trans_frame_t *f, trans_pc_chassis_t *out, uint32_t now_ms);

/**
 * @brief 组 0x23 云台反馈载荷
 * @return 载荷长度 25，0 表示参数错误
 */
uint8_t trans_msg_build_gimbal_fb(uint8_t *payload, uint8_t mode,
                                  float yaw_deg, float pitch_deg,
                                  float yaw_vel_dps, float yaw_acc_dps2);

/**
 * @brief 组 0x24 发射状态载荷
 * @return 载荷长度 7
 */
uint8_t trans_msg_build_shoot_status(uint8_t *payload, uint8_t can_fire,
                                     uint8_t reject_reason,
                                     uint32_t bullet_speed_mmps,
                                     uint8_t launcher_state);

#ifdef __cplusplus
}
#endif

#endif /* __TRANS_MSG_H__ */
