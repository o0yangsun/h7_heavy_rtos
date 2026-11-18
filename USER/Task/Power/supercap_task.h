/**
  ******************************************************************************
  * @file    supercap_task.h
  * @brief   超级电容：CAN2 收发、在线判定与功率反馈
  *
  *   电控 -> 超电  0x221  8 字节：Enable, Charge, PowerLimit(W), ChargePower(W), 0...
  *   超电 -> 电控  0x220  8 字节：Ready, State, ChassisPower(W), VoltageBat(V),
  *                               BatPower, SuperCapPower(int8), 保留
  *   字段均为单字节，无跨字节拼接。
  ******************************************************************************
  */

#ifndef __SUPERCAP_TASK_H__
#define __SUPERCAP_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SUPERCAP_TX_ID              0x221U      /* 电控 -> 超电 */
#define SUPERCAP_RX_ID              0x220U      /* 超电 -> 电控 */
#define SUPERCAP_OFFLINE_MS         300U        /* 超时判定离线 */
#define SUPERCAP_PERIOD_MS          10U         /* 与 PowerTask 同周期 */

/** 下发控制量，字段直接对应发送帧 0~3 字节 */
typedef struct
{
    uint8_t enable;         /**< 1 = 使能输出 */
    uint8_t charge;         /**< 1 = 充电，0 = 放电 */
    uint8_t power_limit;    /**< 底盘功率上限 W */
    uint8_t charge_power;   /**< 充电功率 W */
} supercap_cmd_t;

typedef struct
{
    uint8_t  online;            /**< 1 = SUPERCAP_OFFLINE_MS 内收到过反馈 */
    uint8_t  ready;             /**< 超电就绪 */
    uint8_t  charging;          /**< 1 = 充电中 */
    uint8_t  energy_pct;        /**< 剩余能量 0~100 % */
    uint8_t  bat_voltage_v;     /**< 电池电压 V */
    int8_t   cap_power_w;       /**< 超电功率 W，有符号 */
    uint16_t chassis_power_w;   /**< 底盘功率 W */
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t err_cnt;
} supercap_fdb_t;

extern volatile supercap_fdb_t g_supercap_fdb;

/** 清零状态、装载默认指令、注册 CAN 原始帧钩子 */
void supercap_init(void);

/** 原始帧入口，由 dj_motor 的 RX 钩子调用；内部按总线和 ID 过滤 */
void supercap_can_rx(uint8_t bus, uint32_t std_id, const uint8_t *data);

/** 周期调用：更新在线状态并发送 0x221 */
void supercap_update(void);

void supercap_set_cmd(const supercap_cmd_t *cmd);
void supercap_get_cmd(supercap_cmd_t *out);

/** 1 = 在线 */
uint8_t supercap_is_online(void);

/** 底盘功率 W（超电上报值），离线时返回 0 */
uint16_t supercap_get_chassis_power_w(void);

#ifdef __cplusplus
}
#endif

#endif /* __SUPERCAP_TASK_H__ */
