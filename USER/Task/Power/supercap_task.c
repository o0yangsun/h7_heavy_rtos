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
static uint8_t        s_rx_seen;

/* ------------------------------- 内部函数 -------------------------------- */

void supercap_can_rx(uint8_t bus, uint32_t std_id, const uint8_t *data)
{
    if ((bus != CAN_SUPERCAP) || (std_id != SUPERCAP_RX_ID) || (data == NULL))
    {
        return;
    }

    s_last_rx_tick = HAL_GetTick();
    s_rx_seen = 1U;

    g_supercap_fdb.ready           = (data[0] != 0U) ? 1U : 0U;
    g_supercap_fdb.charging        = (data[1] != 0U) ? 1U : 0U;
    g_supercap_fdb.energy_pct      = data[2];
    g_supercap_fdb.chassis_power_w = data[3];
    g_supercap_fdb.bat_voltage_v   = data[4];
    g_supercap_fdb.cap_power_w     = (int8_t)data[6];
    g_supercap_fdb.rx_cnt++;
}

/* --------------------------------- 接口 ---------------------------------- */

void supercap_init(void)
{
    memset((void *)&g_supercap_fdb, 0, sizeof(g_supercap_fdb));

    s_cmd.enable       = 1U;
    s_cmd.charge       = 0U;
    s_cmd.power_limit  = (uint8_t)POWER_LIMIT_W;
    s_cmd.charge_power = 0U;

    s_last_rx_tick = 0U;
    s_rx_seen      = 0U;

    dj_motor_set_rx_hook(supercap_can_rx);

    log_printf("[CAP] up: can%u id 0x%03X/0x%03X limit=%uW offline=%ums\r\n",
               CAN_SUPERCAP, (unsigned)SUPERCAP_TX_ID, (unsigned)SUPERCAP_RX_ID,
               (unsigned)s_cmd.power_limit, (unsigned)SUPERCAP_OFFLINE_MS);
}

void supercap_update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  frame[8];

    if ((s_rx_seen != 0U) && ((now - s_last_rx_tick) <= SUPERCAP_OFFLINE_MS))
    {
        g_supercap_fdb.online = 1U;
    }
    else
    {
        g_supercap_fdb.online = 0U;
    }

    frame[0] = s_cmd.enable;
    frame[1] = s_cmd.charge;
    frame[2] = s_cmd.power_limit;
    frame[3] = s_cmd.charge_power;
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = 0U;
    frame[7] = 0U;

    if (dj_motor_send_std(CAN_SUPERCAP, SUPERCAP_TX_ID, frame))
    {
        g_supercap_fdb.tx_cnt++;
    }
    else
    {
        g_supercap_fdb.err_cnt++;
    }
}

void supercap_set_cmd(const supercap_cmd_t *cmd)
{
    if (cmd != NULL)
    {
        s_cmd = *cmd;
    }
}

void supercap_get_cmd(supercap_cmd_t *out)
{
    if (out != NULL)
    {
        *out = s_cmd;
    }
}

uint8_t supercap_is_online(void)
{
    return g_supercap_fdb.online;
}

uint16_t supercap_get_chassis_power_w(void)
{
    if (g_supercap_fdb.online == 0U)
    {
        return 0U;
    }
    return g_supercap_fdb.chassis_power_w;
}
