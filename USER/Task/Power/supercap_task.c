/**
  ******************************************************************************
  * @file    supercap_task.c
  * @brief   超级电容 CAN2 收发实现，详见 supercap_task.h
  ******************************************************************************
  */

#include "supercap_task.h"
#include "dj_motor.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "main.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile supercap_fdb_t g_supercap_fdb;

static supercap_cmd_t s_cmd;
static uint32_t       s_last_rx_tick;
static uint32_t       s_last_enable_tick;
static uint8_t        s_rx_seen;

/* ------------------------------- 内部函数 -------------------------------- */

/** 组一帧 0x221 并发出去 */
static uint8_t supercap_send_frame(uint8_t type, uint8_t byte1)
{
    uint8_t frame[8];

    frame[0] = type;
    frame[1] = byte1;
    frame[2] = (uint8_t)(s_cmd.budget_w & 0x00FFU);
    frame[3] = (uint8_t)((s_cmd.budget_w >> 8) & 0x00FFU);
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = 0U;
    frame[7] = 0U;

    return dj_motor_send_std(CAN_SUPERCAP, SUPERCAP_TX_ID, frame);
}

void supercap_can_rx(uint8_t bus, uint32_t std_id, const uint8_t *data)
{
    if ((bus != CAN_SUPERCAP) || (std_id != SUPERCAP_RX_ID) || (data == NULL))
    {
        return;
    }

    s_last_rx_tick = HAL_GetTick();
    s_rx_seen      = 1U;

    g_supercap_fdb.run_enable     = (data[0] != 0U) ? 1U : 0U;
    g_supercap_fdb.state_flags    = data[1];
    g_supercap_fdb.mode           = (uint8_t)(data[1] & SUPERCAP_FLAG_MODE_MASK);
    g_supercap_fdb.normal_running = ((data[1] & SUPERCAP_FLAG_NORMAL) != 0U) ? 1U : 0U;
    g_supercap_fdb.supply_ready   = ((data[1] & SUPERCAP_FLAG_SUPPLY_OK) != 0U) ? 1U : 0U;
    g_supercap_fdb.total_power_w10 = (uint16_t)((uint16_t)data[2]
                                    | ((uint16_t)data[3] << 8));
    g_supercap_fdb.energy_pct     = data[4];
    g_supercap_fdb.last_rx_ms     = s_last_rx_tick;
    g_supercap_fdb.rx_cnt++;
}

/* --------------------------------- 接口 ---------------------------------- */

void supercap_init(void)
{
    memset((void *)&g_supercap_fdb, 0, sizeof(g_supercap_fdb));

    s_cmd.robot_alive = 1U;
    s_cmd.budget_w    = (uint16_t)SUPERCAP_BUDGET_W;

    s_last_rx_tick     = 0U;
    s_last_enable_tick = 0U;
    s_rx_seen          = 0U;

    dj_motor_set_rx_hook(supercap_can_rx);

    log_printf("[CAP] up: can%u 0x%03X/0x%03X budget=%uW\r\n",
               (unsigned)CAN_SUPERCAP, (unsigned)SUPERCAP_TX_ID,
               (unsigned)SUPERCAP_RX_ID, (unsigned)s_cmd.budget_w);
}

void supercap_send_enable(void)
{
    if (supercap_send_frame(SUPERCAP_CMD_TYPE_ENABLE, 1U) != 0U)
    {
        g_supercap_fdb.tx_cnt++;
        s_last_enable_tick = HAL_GetTick();
    }
    else
    {
        g_supercap_fdb.err_cnt++;
    }
}

void supercap_update(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_rx_seen != 0U) && ((now - s_last_rx_tick) <= SUPERCAP_OFFLINE_MS))
    {
        g_supercap_fdb.online = 1U;
    }
    else
    {
        g_supercap_fdb.online = 0U;
    }

    /* 使能帧周期补发：板端重复请求不会重启自检，也能容忍掉帧 */
    if ((s_last_enable_tick == 0U)
        || ((now - s_last_enable_tick) >= SUPERCAP_ENABLE_RESEND_MS))
    {
        supercap_send_enable();
    }

    /* 控制帧：携带心跳与功率预算，板端 500 ms 收不到会回落默认预算 */
    if (supercap_send_frame(SUPERCAP_CMD_TYPE_CONTROL, s_cmd.robot_alive) != 0U)
    {
        g_supercap_fdb.tx_cnt++;
    }
    else
    {
        g_supercap_fdb.err_cnt++;
    }
}

void supercap_set_budget(uint16_t budget_w)
{
    s_cmd.budget_w = budget_w;
}

uint16_t supercap_get_budget(void)
{
    return s_cmd.budget_w;
}

uint8_t supercap_is_online(void)
{
    return g_supercap_fdb.online;
}

float supercap_get_total_power(void)
{
    if (g_supercap_fdb.online == 0U)
    {
        return 0.0f;
    }

    return (float)g_supercap_fdb.total_power_w10 * 0.01f;
}
