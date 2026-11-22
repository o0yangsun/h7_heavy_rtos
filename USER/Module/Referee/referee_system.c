/**
  ******************************************************************************
  * @file    referee_system.c
  * @brief   裁判系统协议解析实现，详见 referee_system.h
  ******************************************************************************
  */

#include "referee_system.h"
#include "crc8_crc16.h"
#include "main.h"

#include <string.h>

/* 编译期校验：结构体大小必须与协议数据段长度一致，改字段时会被这里挡住 */
_Static_assert(sizeof(ref_game_status_t)   == 11, "game_status should be 11 bytes");
_Static_assert(sizeof(ref_game_result_t)   == 1,  "game_result should be 1 byte");
_Static_assert(sizeof(ref_game_robot_hp_t) == 32, "robot_hp should be 32 bytes");
_Static_assert(sizeof(ref_event_data_t)    == 4,  "event_data should be 4 bytes");
_Static_assert(sizeof(ref_warning_t)       == 3,  "warning should be 3 bytes");
_Static_assert(sizeof(ref_dart_info_t)     == 3,  "dart_info should be 3 bytes");
_Static_assert(sizeof(ref_robot_status_t)  == 13, "robot_status should be 13 bytes");
_Static_assert(sizeof(ref_power_heat_t)    == 16, "power_heat should be 16 bytes");
_Static_assert(sizeof(ref_robot_pos_t)     == 12, "robot_pos should be 12 bytes");
_Static_assert(sizeof(ref_buff_t)          == 7,  "buff should be 7 bytes");
_Static_assert(sizeof(ref_hurt_t)          == 1,  "hurt should be 1 byte");
_Static_assert(sizeof(ref_shoot_data_t)    == 7,  "shoot should be 7 bytes");
_Static_assert(sizeof(ref_projectile_t)    == 6,  "projectile should be 6 bytes");
_Static_assert(sizeof(ref_rfid_t)          == 4,  "rfid should be 4 bytes");
_Static_assert(sizeof(ref_interaction_header_t) == 6, "interaction header should be 6 bytes");

/* data_length 是否包含 cmd_id（官方 2024 协议为 1，个别老固件为 0） */
#define REFEREE_LEN_INCLUDES_CMD    1U

#define REFEREE_OFFLINE_MS          1000U

typedef enum
{
    REF_STEP_SOF = 0,
    REF_STEP_LEN_LOW,
    REF_STEP_LEN_HIGH,
    REF_STEP_SEQ,
    REF_STEP_CRC8,
    REF_STEP_DATA,
} ref_step_e;

static struct
{
    uint8_t  frame[REFEREE_FRAME_MAX];
    uint16_t index;
    uint16_t data_length;
    uint8_t  step;
} s_unpack;

static referee_fdb_t s_fdb;

/* ------------------------------- 内部函数 -------------------------------- */

/** 整帧字节数（含帧头与 CRC16） */
static uint16_t ref_frame_total(uint16_t data_length)
{
#if REFEREE_LEN_INCLUDES_CMD
    return (uint16_t)(REFEREE_HEADER_SIZE + data_length + REFEREE_CRC16_SIZE);
#else
    return (uint16_t)(REFEREE_HEADER_SIZE + REFEREE_CMD_SIZE + data_length
                      + REFEREE_CRC16_SIZE);
#endif
}

/** 数据段有效载荷长度（不含 cmd_id） */
static uint16_t ref_payload_len(uint16_t data_length)
{
#if REFEREE_LEN_INCLUDES_CMD
    return (uint16_t)(data_length - REFEREE_CMD_SIZE);
#else
    return data_length;
#endif
}

static void ref_reset_frame(void)
{
    s_unpack.index       = 0U;
    s_unpack.data_length = 0U;
    s_unpack.step        = REF_STEP_SOF;
}

/**
 * @brief 校验失败后的重同步
 * @note  帧内已收到的字节里可能藏着下一帧的 SOF，直接清零会把它一起丢掉。
 *        这里在索引 1 之后找下一个 SOF，把尾巴挪到缓冲开头并恢复状态机。
 */
static void ref_resync(void)
{
    uint16_t i;

    for (i = 1U; i < s_unpack.index; i++)
    {
        if (s_unpack.frame[i] != REFEREE_SOF)
        {
            continue;
        }

        s_unpack.index = (uint16_t)(s_unpack.index - i);
        memmove(&s_unpack.frame[0], &s_unpack.frame[i], s_unpack.index);

        if (s_unpack.index == 1U)
        {
            s_unpack.step = REF_STEP_LEN_LOW;
        }
        else if (s_unpack.index == 2U)
        {
            s_unpack.data_length = s_unpack.frame[1];
            s_unpack.step = REF_STEP_LEN_HIGH;
        }
        else if (s_unpack.index == 3U)
        {
            s_unpack.data_length = (uint16_t)(s_unpack.frame[1]
                                 | ((uint16_t)s_unpack.frame[2] << 8));
            s_unpack.step = REF_STEP_SEQ;
        }
        else if (s_unpack.index == 4U)
        {
            s_unpack.data_length = (uint16_t)(s_unpack.frame[1]
                                 | ((uint16_t)s_unpack.frame[2] << 8));
            s_unpack.step = REF_STEP_CRC8;
        }
        else
        {
            ref_reset_frame();
        }

        return;
    }

    ref_reset_frame();
}

/**
 * @brief 命令分发，逐个校验长度后再落库
 * @note  只接受长度不小于结构体的帧，避免短帧读越界
 */
static void ref_dispatch(uint16_t cmd_id, const uint8_t *data, uint16_t len)
{
    s_fdb.frames++;
    s_fdb.last_rx_ms = HAL_GetTick();

    switch (cmd_id)
    {
        case REF_CMD_GAME_STATUS:
            if (len < sizeof(ref_game_status_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.game_status, data, sizeof(ref_game_status_t));
            break;

        case REF_CMD_GAME_RESULT:
            if (len < sizeof(ref_game_result_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.game_result, data, sizeof(ref_game_result_t));
            break;

        case REF_CMD_GAME_ROBOT_HP:
            if (len < sizeof(ref_game_robot_hp_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.robot_hp, data, sizeof(ref_game_robot_hp_t));
            break;

        case REF_CMD_FIELD_EVENTS:
            if (len < sizeof(ref_event_data_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.event, data, sizeof(ref_event_data_t));
            break;

        case REF_CMD_ROBOT_STATUS:
            if (len < sizeof(ref_robot_status_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.robot_status, data, sizeof(ref_robot_status_t));
            break;

        case REF_CMD_POWER_HEAT:
            if (len < sizeof(ref_power_heat_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.power_heat, data, sizeof(ref_power_heat_t));
            break;

        case REF_CMD_ROBOT_POS:
            if (len < sizeof(ref_robot_pos_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.robot_pos, data, sizeof(ref_robot_pos_t));
            break;

        case REF_CMD_BUFF:
            if (len < sizeof(ref_buff_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.buff, data, sizeof(ref_buff_t));
            break;

        case REF_CMD_ROBOT_HURT:
            if (len < sizeof(ref_hurt_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.hurt, data, sizeof(ref_hurt_t));
            break;

        case REF_CMD_SHOOT_DATA:
            if (len < sizeof(ref_shoot_data_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.shoot, data, sizeof(ref_shoot_data_t));
            break;

        case REF_CMD_PROJECTILE:
            if (len < sizeof(ref_projectile_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.projectile, data, sizeof(ref_projectile_t));
            break;

        case REF_CMD_RFID:
            if (len < sizeof(ref_rfid_t)) { s_fdb.len_err++; break; }
            memcpy(&s_fdb.rfid, data, sizeof(ref_rfid_t));
            break;

        default:
            /* 其余命令只计数，不落库 */
            break;
    }
}

/* -------------------------------- 对外接口 -------------------------------- */

void referee_system_init(void)
{
    memset(&s_unpack, 0, sizeof(s_unpack));
    memset(&s_fdb, 0, sizeof(s_fdb));
    s_unpack.step = REF_STEP_SOF;
}

void referee_data_unpack(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t total;
    uint16_t payload;
    uint16_t cmd_id;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        uint8_t b = data[i];

        switch (s_unpack.step)
        {
            case REF_STEP_SOF:
                if (b == REFEREE_SOF)
                {
                    s_unpack.frame[0] = b;
                    s_unpack.index    = 1U;
                    s_unpack.step     = REF_STEP_LEN_LOW;
                }
                break;

            case REF_STEP_LEN_LOW:
                s_unpack.frame[s_unpack.index++] = b;
                s_unpack.data_length = b;
                s_unpack.step = REF_STEP_LEN_HIGH;
                break;

            case REF_STEP_LEN_HIGH:
                s_unpack.frame[s_unpack.index++] = b;
                s_unpack.data_length |= (uint16_t)b << 8;

                if ((s_unpack.data_length < REFEREE_CMD_SIZE)
                    || (s_unpack.data_length > REFEREE_MAX_DATA)
                    || (ref_frame_total(s_unpack.data_length) > REFEREE_FRAME_MAX))
                {
                    s_fdb.len_err++;
                    ref_resync();
                }
                else
                {
                    s_unpack.step = REF_STEP_SEQ;
                }
                break;

            case REF_STEP_SEQ:
                s_unpack.frame[s_unpack.index++] = b;
                s_unpack.step = REF_STEP_CRC8;
                break;

            case REF_STEP_CRC8:
                s_unpack.frame[s_unpack.index++] = b;

                if (crc8_verify_check_sum(s_unpack.frame, REFEREE_HEADER_SIZE) == 0U)
                {
                    s_fdb.crc_err++;
                    ref_resync();
                }
                else
                {
                    s_unpack.step = REF_STEP_DATA;
                }
                break;

            case REF_STEP_DATA:
            default:
            {
                total = ref_frame_total(s_unpack.data_length);

                if (s_unpack.index < total)
                {
                    s_unpack.frame[s_unpack.index++] = b;
                }

                if (s_unpack.index >= total)
                {
                    if (crc16_verify_check_sum(s_unpack.frame, total) != 0U)
                    {
                        cmd_id  = (uint16_t)(s_unpack.frame[5]
                                          | ((uint16_t)s_unpack.frame[6] << 8));
                        payload = ref_payload_len(s_unpack.data_length);

                        if (payload > (uint16_t)(REFEREE_MAX_DATA - REFEREE_CMD_SIZE))
                        {
                            payload = (uint16_t)(REFEREE_MAX_DATA - REFEREE_CMD_SIZE);
                        }

                        ref_dispatch(cmd_id, &s_unpack.frame[7], payload);
                    }
                    else
                    {
                        s_fdb.crc_err++;
                    }

                    ref_reset_frame();
                }
                break;
            }
        }
    }
}

void referee_get_fdb(referee_fdb_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memcpy(out, &s_fdb, sizeof(referee_fdb_t));

    out->online = ((HAL_GetTick() - s_fdb.last_rx_ms) <= REFEREE_OFFLINE_MS) ? 1U : 0U;
}

uint8_t referee_is_online(void)
{
    return ((HAL_GetTick() - s_fdb.last_rx_ms) <= REFEREE_OFFLINE_MS) ? 1U : 0U;
}

uint16_t referee_get_chassis_power_limit(void)
{
    if (referee_is_online() == 0U)
    {
        return 0U;
    }

    return s_fdb.robot_status.chassis_power_limit;
}

uint16_t referee_get_buffer_energy(void)
{
    if (referee_is_online() == 0U)
    {
        return 0U;
    }

    return s_fdb.power_heat.buffer_energy;
}
