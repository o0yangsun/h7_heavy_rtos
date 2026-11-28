/**
  ******************************************************************************
  * @file    trans_msg.c
  * @brief   小电脑链路消息编解码
  ******************************************************************************
  */

#include "trans_msg.h"
#include "robot.h"

#include <math.h>
#include <string.h>

#define TRANS_MSG_MDEG_TO_RAD       1.7453292519943296e-5f
#define TRANS_MSG_YAW_LIMIT_DEG     1000000.0f

static int32_t trans_msg_get_i32(const uint8_t *p)
{
    uint32_t v;

    v = (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);

    return (int32_t)v;
}

static void trans_msg_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void trans_msg_put_i32(uint8_t *p, int32_t v)
{
    trans_msg_put_u32(p, (uint32_t)v);
}

static int32_t trans_msg_scale_deg(float value, float limit_deg)
{
    if (!isfinite(value))
    {
        return 0;
    }
    if (value > limit_deg)
    {
        value = limit_deg;
    }
    else if (value < -limit_deg)
    {
        value = -limit_deg;
    }

    return (int32_t)lroundf(value * 1000.0f);
}

uint8_t trans_msg_parse_vision(const trans_frame_t *f, trans_vision_cmd_t *out, uint32_t now_ms)
{
    uint8_t mode;

    if ((f == 0) || (out == 0))
    {
        return 0U;
    }
    if ((f->id != TRANS_ID_VISION_CMD) || (f->len != TRANS_VISION_PAYLOAD_LEN))
    {
        return 0U;
    }

    mode = f->data[0];
    if (mode > 2U)
    {
        return 0U;
    }

    out->mode            = mode;
    out->yaw_mdeg        = trans_msg_get_i32(&f->data[1]);
    out->yaw_vel_mdeg_s  = trans_msg_get_i32(&f->data[5]);
    out->yaw_acc_mdeg_s2 = trans_msg_get_i32(&f->data[9]);
    out->pitch_mdeg      = trans_msg_get_i32(&f->data[13]);
    out->yaw_rad         = (float)out->yaw_mdeg * TRANS_MSG_MDEG_TO_RAD;
    out->yaw_vel_rps     = (float)out->yaw_vel_mdeg_s * TRANS_MSG_MDEG_TO_RAD;
    out->yaw_acc_rps2    = (float)out->yaw_acc_mdeg_s2 * TRANS_MSG_MDEG_TO_RAD;
    out->pitch_rad       = (float)out->pitch_mdeg * TRANS_MSG_MDEG_TO_RAD;
    out->tick_ms         = now_ms;
    out->valid           = 1U;

    return 1U;
}

uint8_t trans_msg_parse_pc_chassis(const trans_frame_t *f, trans_pc_chassis_t *out, uint32_t now_ms)
{
    int32_t x;
    int32_t y;
    int32_t wz;

    if ((f == 0) || (out == 0))
    {
        return 0U;
    }
    if ((f->id != TRANS_ID_CHASSIS_CMD) || (f->len != TRANS_CHASSIS_PAYLOAD_LEN))
    {
        return 0U;
    }

    x  = trans_msg_get_i32(&f->data[0]);
    y  = trans_msg_get_i32(&f->data[4]);
    wz = trans_msg_get_i32(&f->data[20]);

    out->vx_mps = (float)x / 10000.0f;
    out->vy_mps = -(float)y / 10000.0f;
    out->vw_rps = -(float)wz / 10000.0f;
    out->tick_ms = now_ms;
    out->valid   = 1U;

    return 1U;
}

uint8_t trans_msg_build_gimbal_fb(uint8_t *payload, uint8_t mode,
                                  float yaw_deg, float pitch_deg,
                                  float yaw_vel_dps, float yaw_acc_dps2)
{
    if (payload == 0)
    {
        return 0U;
    }

    if (mode > 3U)
    {
        mode = 0U;
    }

    memset(payload, 0, TRANS_GIMBAL_FB_PAYLOAD_LEN);

    payload[0] = mode;
    trans_msg_put_i32(&payload[1], trans_msg_scale_deg(yaw_deg, TRANS_MSG_YAW_LIMIT_DEG));
    trans_msg_put_i32(&payload[5], trans_msg_scale_deg(pitch_deg, TRANS_PITCH_LIMIT_DEG));
    trans_msg_put_i32(&payload[17], trans_msg_scale_deg(yaw_vel_dps, TRANS_YAW_VEL_LIMIT_DPS));
    trans_msg_put_i32(&payload[21], trans_msg_scale_deg(yaw_acc_dps2, TRANS_YAW_ACC_LIMIT_DPS2));

    return TRANS_GIMBAL_FB_PAYLOAD_LEN;
}

uint8_t trans_msg_build_shoot_status(uint8_t *payload, uint8_t can_fire,
                                     uint8_t reject_reason,
                                     uint32_t bullet_speed_mmps,
                                     uint8_t launcher_state)
{
    if (payload == 0)
    {
        return 0U;
    }

    if (launcher_state > 3U)
    {
        launcher_state = 255U;
    }

    payload[0] = (can_fire != 0U) ? 1U : 0U;
    payload[1] = reject_reason;
    trans_msg_put_u32(&payload[2], bullet_speed_mmps);
    payload[6] = launcher_state;

    return TRANS_SHOOT_PAYLOAD_LEN;
}
