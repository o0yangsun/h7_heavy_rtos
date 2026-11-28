/**
  ******************************************************************************
  * @file    transmission_task.c
  * @brief   小电脑链路实现，详见 transmission_task.h
  ******************************************************************************
  */

#include "transmission_task.h"
#include "trans_frame.h"
#include "trans_usb.h"
#include "gimbal_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

volatile trans_diag_t g_trans_diag;

static task_stat_t          s_stat;
static trans_frame_parser_t s_parser;
static trans_vision_cmd_t   s_vision;
static trans_pc_chassis_t   s_pc_chassis;
static volatile uint8_t     s_gimbal_mode = TRANS_GIMBAL_MODE_REPORT;

static trans_yaw_unwrap_t s_yaw_unwrap;
static uint8_t            s_gimbal_was_online;
static float              s_pitch_deg;
static float              s_yaw_vel_last_dps;
static uint32_t           s_yaw_vel_last_ms;
static float              s_yaw_acc_lpf_dps2;

static float trans_update_yaw_acc(float yaw_vel_dps, uint32_t now_ms)
{
    float dt;
    float acc;

    if (s_yaw_vel_last_ms == 0U)
    {
        s_yaw_vel_last_ms  = now_ms;
        s_yaw_vel_last_dps = yaw_vel_dps;
        return 0.0f;
    }

    dt = (float)(now_ms - s_yaw_vel_last_ms) * 0.001f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }

    acc = (yaw_vel_dps - s_yaw_vel_last_dps) / dt;
    s_yaw_acc_lpf_dps2 += TRANS_YAW_ACC_LPF_ALPHA * (acc - s_yaw_acc_lpf_dps2);
    s_yaw_vel_last_dps  = yaw_vel_dps;
    s_yaw_vel_last_ms   = now_ms;

    return s_yaw_acc_lpf_dps2;
}

static void trans_dispatch(const trans_frame_t *f, uint32_t now_ms)
{
    g_trans_diag.last_id    = f->id;
    g_trans_diag.last_len   = f->len;
    g_trans_diag.last_rx_ms = now_ms;

    switch (f->id)
    {
        case TRANS_ID_VISION_CMD:
            if (trans_msg_parse_vision(f, &s_vision, now_ms) != 0U)
            {
                s_vision.seq++;
                g_trans_diag.rx_0x22++;
            }
            else
            {
                g_trans_diag.invalid_payload++;
            }
            break;

        case TRANS_ID_CHASSIS_CMD:
            if (trans_msg_parse_pc_chassis(f, &s_pc_chassis, now_ms) != 0U)
            {
                g_trans_diag.rx_0x12++;
            }
            else
            {
                g_trans_diag.invalid_payload++;
            }
            break;

        case TRANS_ID_HEARTBEAT:
            g_trans_diag.rx_0xf0++;
            break;

        default:
            g_trans_diag.unknown_id++;
            break;
    }
}

void trans_rx_push(const uint8_t *buf, uint16_t len)
{
    trans_usb_rx_isr(buf, len);
}

void trans_rx_byte(uint8_t b)
{
    trans_usb_rx_isr(&b, 1U);
}

uint8_t trans_send_frame(uint8_t id, const void *data, uint8_t len)
{
#if ENABLE_TRANSMISSION
    uint8_t  frame[TRANS_FRAME_MAX_SIZE];
    uint16_t n;

    if (len > TRANS_FRAME_MAX_PAYLOAD)
    {
        return 0U;
    }

    n = trans_frame_build(id, (const uint8_t *)data, len, frame, (uint16_t)sizeof(frame));
    if (n == 0U)
    {
        return 0U;
    }

    if (trans_usb_tx(frame, n) == 0U)
    {
        return 0U;
    }

    g_trans_diag.tx_frames++;
    return 1U;
#else
    (void)id;
    (void)data;
    (void)len;
    return 0U;
#endif
}

uint8_t trans_link_online(void)
{
    uint32_t last = g_trans_diag.last_rx_ms;

    if (last == 0U)
    {
        return 0U;
    }

    return trans_msg_is_fresh(HAL_GetTick(), last, TRANS_RX_TIMEOUT_MS);
}

void trans_get_vision(trans_vision_cmd_t *out)
{
    if (out == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_vision;
    taskEXIT_CRITICAL();

    if ((out->seq == 0U) || (out->valid == 0U) ||
        (trans_msg_is_fresh(HAL_GetTick(), out->tick_ms, TRANS_RX_TIMEOUT_MS) == 0U))
    {
        out->valid = 0U;
    }
}

void trans_get_pc_chassis(trans_pc_chassis_t *out)
{
    if (out == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_pc_chassis;
    taskEXIT_CRITICAL();

    if ((out->valid == 0U) ||
        (trans_msg_is_fresh(HAL_GetTick(), out->tick_ms, TRANS_RX_TIMEOUT_MS) == 0U))
    {
        out->valid = 0U;
    }
}

void trans_set_gimbal_mode(uint8_t mode)
{
    if (mode <= 3U)
    {
        s_gimbal_mode = mode;
    }
}

void StartTransmissionTask(void const *argument)
{
    static trans_frame_t frame;
    uint8_t  chunk[32];
    uint32_t last_tx_23;
    uint32_t last_tx_24;
    uint32_t last_print;

    (void)argument;

    memset((void *)&g_trans_diag, 0, sizeof(g_trans_diag));
    memset(&s_vision, 0, sizeof(s_vision));
    memset(&s_pc_chassis, 0, sizeof(s_pc_chassis));
    trans_frame_parser_init(&s_parser);

    trans_yaw_unwrap_reset(&s_yaw_unwrap);
    s_gimbal_was_online = 0U;
    s_pitch_deg         = 0.0f;
    s_yaw_vel_last_dps  = 0.0f;
    s_yaw_vel_last_ms   = 0U;
    s_yaw_acc_lpf_dps2  = 0.0f;

    task_stat_init(&s_stat, TRANS_TASK_PERIOD_MS);

    last_tx_23 = HAL_GetTick();
    last_tx_24 = last_tx_23;
    last_print = last_tx_23;

    log_printf("[TRN] up: USB BCP addr=0x%02X 23:%ums 24:%ums mode=%u\r\n",
               (unsigned)TRANS_FRAME_ADDR,
               (unsigned)TRANS_GIMBAL_FB_PERIOD_MS,
               (unsigned)TRANS_SHOOT_STATUS_PERIOD_MS,
               (unsigned)s_gimbal_mode);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint16_t drained = 0U;
        uint16_t n;
        uint16_t i;

        task_stat_begin(&s_stat, now);

#if TRANS_RX_ENABLE
        while (drained < TRANS_RX_DRAIN_MAX)
        {
            uint16_t want = (uint16_t)(TRANS_RX_DRAIN_MAX - drained);

            if (want > (uint16_t)sizeof(chunk))
            {
                want = (uint16_t)sizeof(chunk);
            }

            n = trans_usb_read(chunk, want);
            if (n == 0U)
            {
                break;
            }

            g_trans_diag.rx_bytes += n;
            drained = (uint16_t)(drained + n);

            for (i = 0U; i < n; i++)
            {
                if (trans_frame_parser_feed(&s_parser, chunk[i], &frame) != 0U)
                {
                    g_trans_diag.rx_frames++;
                    trans_dispatch(&frame, now);
                }
            }
        }
#endif

        g_trans_diag.checksum_errors = s_parser.checksum_errors;
        g_trans_diag.length_errors   = s_parser.length_errors;
        g_trans_diag.address_errors  = s_parser.address_errors;
        g_trans_diag.discarded_bytes = s_parser.discarded_bytes;
        g_trans_diag.ring_overflow   = trans_usb_overflow_count();
        g_trans_diag.tx_busy         = trans_usb_tx_busy_count();
        g_trans_diag.tx_error        = trans_usb_tx_error_count();
        g_trans_diag.loop_cnt        = s_stat.loop_cnt;

        if ((s_vision.seq != 0U) &&
            ((now - s_vision.tick_ms) >= TRANS_RX_TIMEOUT_MS))
        {
            s_vision.valid = 0U;
        }
        if ((s_pc_chassis.valid != 0U) &&
            ((now - s_pc_chassis.tick_ms) >= TRANS_RX_TIMEOUT_MS))
        {
            s_pc_chassis.valid = 0U;
        }

#if ENABLE_TRANSMISSION
        if (trans_usb_ready() != 0U)
        {
            if ((now - last_tx_23) >= TRANS_GIMBAL_FB_PERIOD_MS)
            {
                gimbal_fdb_t fdb;
                float yaw_deg;
                float pitch_deg;
                float yaw_vel;
                float yaw_acc;
                uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN];
                uint8_t plen;

                last_tx_23 = now;
                gimbal_get_fdb(&fdb);

                if (fdb.online[0] != false)
                {
                    if (s_gimbal_was_online == 0U)
                    {
                        trans_yaw_unwrap_reset(&s_yaw_unwrap);
                        s_gimbal_was_online = 1U;
                    }
                    yaw_deg = trans_yaw_unwrap_update(&s_yaw_unwrap, fdb.yaw_deg);
                }
                else
                {
                    s_gimbal_was_online = 0U;
                    yaw_deg = s_yaw_unwrap.continuous_deg;
                }

                if (fdb.online[1] != false)
                {
                    s_pitch_deg = fdb.pitch_deg;
                }
                pitch_deg = s_pitch_deg;

                yaw_vel = (fdb.online[0] != false) ? fdb.yaw_speed_dps : 0.0f;
                yaw_acc = trans_update_yaw_acc(yaw_vel, now);

                plen = trans_msg_build_gimbal_fb(payload, s_gimbal_mode,
                                                 yaw_deg, pitch_deg,
                                                 yaw_vel, yaw_acc);
                if (trans_send_frame(TRANS_ID_GIMBAL_FB, payload, plen) != 0U)
                {
                    g_trans_diag.tx_0x23++;
                }
            }

            if ((now - last_tx_24) >= TRANS_SHOOT_STATUS_PERIOD_MS)
            {
                uint8_t payload[TRANS_SHOOT_PAYLOAD_LEN];
                uint8_t plen;

                last_tx_24 = now;
                plen = trans_msg_build_shoot_status(payload, 0U, 0U, 0U, 255U);
                if (trans_send_frame(TRANS_ID_SHOOT_STATUS, payload, plen) != 0U)
                {
                    g_trans_diag.tx_0x24++;
                }
            }
        }
#endif

#if ENABLE_DIAG_TRANSMISSION
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[TRN] L%u rx%lu 12:%lu 22:%lu F0:%lu E%lu/%lu/%lu T%lu/%lu b%lu o%lu y%.0f p%.0f m%u v%u\r\n",
                       (unsigned)trans_link_online(),
                       (unsigned long)g_trans_diag.rx_frames,
                       (unsigned long)g_trans_diag.rx_0x12,
                       (unsigned long)g_trans_diag.rx_0x22,
                       (unsigned long)g_trans_diag.rx_0xf0,
                       (unsigned long)g_trans_diag.checksum_errors,
                       (unsigned long)g_trans_diag.length_errors,
                       (unsigned long)g_trans_diag.address_errors,
                       (unsigned long)g_trans_diag.tx_0x23,
                       (unsigned long)g_trans_diag.tx_0x24,
                       (unsigned long)g_trans_diag.tx_busy,
                       (unsigned long)g_trans_diag.ring_overflow,
                       s_yaw_unwrap.continuous_deg,
                       s_pitch_deg,
                       (unsigned)s_gimbal_mode,
                       (unsigned)s_vision.mode);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
