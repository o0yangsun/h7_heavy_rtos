/**
  ******************************************************************************
  * @file    supercap_task.h
  * @brief   超级电容：CAN2 收发、在线判定与功率反馈
  *
  *   协议来自超电板固件（Supercap/Remaked_SuperCap/Core/Inc/fdcan.h）：
  *
  *   电控 -> 超电  0x221  8 字节
  *     [0] CmdType   0x01 使能帧 / 0x02 控制帧，其它值整帧丢弃
  *     [1] 使能帧 Enable(1) / 控制帧 RobotAlive(0|1)
  *     [2] 电池侧功率预算 budget_w 低字节
  *     [3] budget_w 高字节（小端 uint16，1 W/LSB）
  *     [4..7] 保留
  *
  *   超电 -> 电控  0x220  8 字节，2 ms 周期
  *     [0] RunEnable 回读（0/1）
  *     [1] StateFlags：bit0-3 状态码，bit4 正常运行，bit5 允许供能请求
  *     [2] TotalPower 低字节
  *     [3] TotalPower 高字节（小端 uint16，0.01 W/LSB，充电时压成 0）
  *     [4] EnergyPct 0~100 %
  *     [5..7] 保留
  *
  *   注意：板端使能只能置位不能清零，500 ms 收不到控制帧会回落到 60 W
  *   默认预算但不失能；0x220 不提供母线电压与电流。
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
#define SUPERCAP_OFFLINE_MS         600U        /* 板端 2ms 一帧，连续无帧判掉线 */
#define SUPERCAP_PERIOD_MS          10U         /* 与 PowerTask 同周期 */
#define SUPERCAP_ENABLE_RESEND_MS   1000U       /* 使能帧补发间隔，重复请求板端安全 */

/* 0x221 命令类型 */
#define SUPERCAP_CMD_TYPE_ENABLE    0x01U
#define SUPERCAP_CMD_TYPE_CONTROL   0x02U

/* 0x220 状态位 */
#define SUPERCAP_FLAG_MODE_MASK     0x0FU
#define SUPERCAP_FLAG_NORMAL        0x10U
#define SUPERCAP_FLAG_SUPPLY_OK     0x20U

/* 板端不提供母线电压，反算电流时用标称值，保证 p = v*i 与上报功率一致 */
#define SUPERCAP_NOMINAL_VOLTAGE    24.0f

/** 下发控制量 */
typedef struct
{
    uint8_t  robot_alive;       /**< 控制帧心跳，0/1 */
    uint16_t budget_w;          /**< 电池侧功率预算 W */
} supercap_cmd_t;

typedef struct
{
    uint8_t  online;            /**< 1 = SUPERCAP_OFFLINE_MS 内收到过反馈 */
    uint8_t  run_enable;        /**< 板端运行使能回读 */
    uint8_t  state_flags;       /**< 原始字节 1 */
    uint8_t  mode;              /**< 状态码 0~9 */
    uint8_t  normal_running;    /**< bit4 正常运行 */
    uint8_t  supply_ready;      /**< bit5 允许供能请求 */
    uint8_t  energy_pct;        /**< 储能百分比 0~100 */
    float    total_power_w;     /**< Pbat + Pcap，单位 W（总线负载） */
    float    bat_power_w;       /**< 电池侧功率 Pbat，单位 W */
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t err_cnt;
    uint32_t last_rx_ms;
} supercap_fdb_t;

extern volatile supercap_fdb_t g_supercap_fdb;

/** 清零状态、装载默认指令、注册 CAN 原始帧钩子 */
void supercap_init(void);

/** 原始帧入口，由 dj_motor 的 RX 钩子调用；内部按总线和 ID 过滤 */
void supercap_can_rx(uint8_t bus, uint32_t std_id, const uint8_t *data);

/** 周期调用：更新在线状态并发送控制帧（必要时补发使能帧） */
void supercap_update(void);

/** 请求使能：置位运行开关，板端收到 Enable=1 后保持工作 */
void supercap_send_enable(void);

/**
 * @brief 请求软下电：发 Enable=0，板端清运行开关并走正常停机流程
 * @note  停机后需要重新调用 supercap_send_enable()，板端会从自检开始
 */
void supercap_disable(void);

/** 1 = 主控当前期望超电工作，0 = 已请求软下电 */
uint8_t supercap_is_enabled(void);

void supercap_set_budget(uint16_t budget_w);
uint16_t supercap_get_budget(void);

/** 1 = 在线 */
uint8_t supercap_is_online(void);

/** 上报总功率 W（Pbat + Pcap），离线时返回 0 */
float supercap_get_total_power(void);

/** 电池侧功率 W（板端闭环的控制对象），离线时返回 0 */
float supercap_get_bat_power(void);

#ifdef __cplusplus
}
#endif

#endif /* __SUPERCAP_TASK_H__ */
