/**
  ******************************************************************************
  * @file    dj_motor.h
  * @brief   大疆电机驱动（M3508 / M2006 / GM6020），支持 FDCAN1/2/3
  *
  * 发送按总线 x 帧分 9 个槽位，每槽 8 字节，最多 4 个电机。
  *   3508/2006：0x200(ID1-4) / 0x1FF(ID5-8)，电流 ±16384
  *   GM6020   ：0x1FF(ID1-4) / 0x2FF(ID5-7)，电压 ±30000
  * 反馈按报文 ID 查表，不按电机 ID 推算。
  ******************************************************************************
  */

#ifndef __DJ_MOTOR_H__
#define __DJ_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include "robot.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------- 常量 ---------------------------------- */

/** 单条总线上的标准帧 ID 反查表大小：0x201 ~ 0x20B */
#define DJ_RX_MAP_SIZE          11U

/* --------------------------------- 类型定义 -------------------------------- */

/** 电机类型 */
typedef enum
{
    DJ_MOTOR_M3508 = 0,     /**< 3508 + C620，电流控制 */
    DJ_MOTOR_M2006,         /**< 2006 + C610，电流控制 */
    DJ_MOTOR_GM6020,        /**< GM6020 直驱云台电机，电压控制 */
} dj_motor_type_e;

/** 控制模式（仅用于日志/调试时区分，驱动本身不关心） */
typedef enum
{
    DJ_CTRL_STOP = 0,       /**< 输出强制为 0 */
    DJ_CTRL_RUN,            /**< 正常调用控制回调 */
} dj_ctrl_mode_e;

/** 单电机反馈数据 */
typedef struct
{
    uint16_t ecd;                   /**< 机械角 0~8191 */
    int16_t  speed_rpm;             /**< 转子转速 rpm（已一阶滤波） */
    int16_t  real_current;          /**< 3508: 实际电流值; 6020: 转矩电流 */
    uint8_t  temperature;           /**< 温度 ℃ */
    int16_t  last_ecd;              /**< 上一帧 ecd，用于多圈累加 */
    int32_t  total_round;           /**< 多圈计数 */
    float    angle_single_round;    /**< 0~360° */
    float    total_angle;           /**< 累计角度（°，含多圈） */
    uint32_t last_update_ms;        /**< 最近一帧的时间戳 */
    bool     online;                /**< 在线标志 */
} dj_motor_measure_t;

struct dj_motor_object;

/**
 * @brief 控制回调
 * @param measure 当前反馈（只读）
 * @param ctx     注册时传入的上下文指针
 * @return 3508/2006: -16384~16384 电流；GM6020: -30000~30000 电压
 * @note  在 MotorTask 的 1kHz 上下文中调用，不可阻塞
 */
typedef int16_t (*dj_motor_ctrl_fn)(const dj_motor_measure_t *measure, void *ctx);

/** 注册配置 */
typedef struct
{
    dj_motor_type_e  type;          /**< 电机类型 */
    uint8_t          bus;           /**< 总线号：1 / 2 / 3 */
    uint8_t          id;            /**< 电机 ID：3508/2006 为 1~8，6020 为 1~7 */
    dj_motor_ctrl_fn control;       /**< 控制回调，NULL 表示输出恒 0 */
    void            *ctx;           /**< 控制回调的上下文，可为 NULL */
} dj_motor_config_t;

/** 电机实例 */
typedef struct dj_motor_object
{
    dj_motor_type_e  type;
    uint8_t          bus;
    uint8_t          id;
    uint8_t          frame_slot;    /**< 发送槽位：0~8 = (bus-1)*3 + frame_idx */
    uint8_t          msg_index;     /**< 帧内槽位：0~3，对应 8 字节里的第几组 */
    uint8_t          rx_index;      /**< 反查表下标：std_id - 0x201 */
    bool             stopped;       /**< true 时该电机控制量强制写 0 */
    dj_motor_ctrl_fn control;
    void            *ctx;           /**< 控制回调上下文 */
    dj_motor_measure_t measure;
} dj_motor_object_t;

/* ---------------------------------- 接口 ---------------------------------- */

/**
 * @brief 初始化三条 FDCAN 总线：全局过滤器、启动、开 RX FIFO0 中断
 * @note  必须在 MX_FDCAN1/2/3_Init() 之后调用
 */
void dj_motor_can_init(void);

/**
 * @brief 注册一个电机实例
 * @param cfg 配置（注意 bus/id/type 组合不能与已注册实例冲突）
 * @return 实例指针；失败返回 NULL（重复 ID、参数越界、表满）
 */
dj_motor_object_t *dj_motor_register(const dj_motor_config_t *cfg);

/** 使能该电机（stopped = false） */
void dj_motor_enable(dj_motor_object_t *motor);

/** 松开该电机（输出强制 0，电机自由转动） */
void dj_motor_relax(dj_motor_object_t *motor);

/**
 * @brief 计算所有电机的控制量并按 9 个槽位发出
 * @note  由 MotorTask 以 1kHz 调用
 */
void dj_motor_send_all(void);

/**
 * @brief 更新所有电机的在线状态（超过 timeout 未收到反馈则 offline）
 * @param now_ms 当前毫秒时基（HAL_GetTick）
 */
void dj_motor_update_online(uint32_t now_ms, uint32_t timeout_ms);

/** 已注册的电机数量 */
uint8_t dj_motor_get_count(void);

/**
 * @brief 切换三条总线的内部回环模式
 * @param on true = 内部回环（帧不出引脚，自己应答自己）
 * @note  回环下 TX 引脚保持隐性。
 */
void dj_motor_set_loopback(bool on);

/** 按下标取实例（0 ~ count-1），用于遍历打印 */
dj_motor_object_t *dj_motor_get_by_index(uint8_t index);

/**
 * @brief 直接发一帧标准数据帧（8 字节），绕过槽位表
 * @param bus     1/2/3
 * @param std_id  标准帧 ID
 * @param data    8 字节数据
 * @return true = 成功入队
 * @note  仅用于自测和特殊帧，正常控制帧走 dj_motor_send_all()。
 */
bool dj_motor_send_std(uint8_t bus, uint32_t std_id, const uint8_t *data);

/**
 * @brief 反馈报文分发，由 HAL_FDCAN_RxFifo0Callback 调用
 * @param bus    1/2/3
 * @param std_id 标准帧 ID
 * @param data   8 字节数据
 */
void dj_motor_rx_dispatch(uint8_t bus, uint32_t std_id, const uint8_t *data);

/* ------------------------------ 统计量（台架验证用） ------------------------------ */

/** 累计成功入队的发送帧数 */
uint32_t dj_motor_get_tx_count(void);

/** 累计入队失败的发送帧数（总线上没节点或邮箱满时会涨） */
uint32_t dj_motor_get_tx_error(void);

/** 指定总线累计收到的反馈帧数（bus = 1/2/3） */
uint32_t dj_motor_get_rx_count(uint8_t bus);

/** 指定总线收到「任意标准帧」的累计数（不管 ID 是否匹配到电机），用于排查 ID 不符 */
uint32_t dj_motor_get_rx_any(uint8_t bus);

/** 指定总线最后收到的标准帧 ID（0 表示未收到） */
uint32_t dj_motor_get_last_id(uint8_t bus);

/** 累计 bus-off 次数（台架上总线没有节点回 ACK 时会持续增长，属正常现象） */
uint32_t dj_motor_get_busoff_count(void);

/**
 * @brief 当前处于 bus-off 冷却期的总线
 * @return bit0=CAN1 bit1=CAN2 bit2=CAN3，1 表示这条总线暂时不发
 * @note  总线上没接电机时它会一直是 1，接上电机后最多 500ms 自动恢复。
 */
uint8_t dj_motor_get_dead_mask(void);

#ifdef __cplusplus
}
#endif

#endif /* __DJ_MOTOR_H__ */
