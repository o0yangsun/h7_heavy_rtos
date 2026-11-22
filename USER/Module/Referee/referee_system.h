/**
  ******************************************************************************
  * @file    referee_system.h
  * @brief   RoboMaster 裁判系统协议解析：帧结构、数据结构与对外快照
  *
  *   帧格式：
  *     SOF(0xA5) | data_length(2, 小端) | seq(1) | CRC8(1)
  *     | cmd_id(2, 小端) | data(data_length-2) | CRC16(2, 小端)
  *   data_length 统计的是「cmd_id + data」，CRC8 校验前 4 字节，
  *   CRC16 校验从 SOF 到 data 末尾。
  ******************************************************************************
  */

#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------- 帧参数 --------------------------------- */

#define REFEREE_SOF                 0xA5U
#define REFEREE_HEADER_SIZE         5U
#define REFEREE_CMD_SIZE            2U
#define REFEREE_CRC16_SIZE          2U
#define REFEREE_MAX_DATA            127U
#define REFEREE_FRAME_MAX           (REFEREE_HEADER_SIZE + REFEREE_CMD_SIZE \
                                     + REFEREE_MAX_DATA + REFEREE_CRC16_SIZE)

/* -------------------------------- 命令码 ---------------------------------- */

typedef enum
{
    REF_CMD_GAME_STATUS          = 0x0001U,     /* 比赛状态，11 字节 */
    REF_CMD_GAME_RESULT          = 0x0002U,     /* 比赛结果，1 字节 */
    REF_CMD_GAME_ROBOT_HP        = 0x0003U,     /* 全场血量，32 字节 */
    REF_CMD_FIELD_EVENTS         = 0x0101U,     /* 场地事件，4 字节 */
    REF_CMD_REFEREE_WARNING      = 0x0104U,     /* 裁判警告，3 字节 */
    REF_CMD_DART_INFO            = 0x0105U,     /* 飞镖信息，3 字节 */
    REF_CMD_ROBOT_STATUS         = 0x0201U,     /* 机器人状态，13 字节 */
    REF_CMD_POWER_HEAT           = 0x0202U,     /* 功率与热量，16 字节 */
    REF_CMD_ROBOT_POS            = 0x0203U,     /* 机器人位置，16 字节 */
    REF_CMD_BUFF                 = 0x0204U,     /* 增益状态，7 字节 */
    REF_CMD_ROBOT_HURT           = 0x0206U,     /* 受伤状态，1 字节 */
    REF_CMD_SHOOT_DATA           = 0x0207U,     /* 射击数据，7 字节 */
    REF_CMD_PROJECTILE           = 0x0208U,     /* 剩余弹量，6 字节 */
    REF_CMD_RFID                 = 0x0209U,     /* RFID 状态，4 字节 */
    REF_CMD_ROBOT_LOCATION       = 0x020BU,     /* 地面机器人位置，40 字节 */
    REF_CMD_RADAR_MARK           = 0x020CU,     /* 雷达标记，1 字节 */
    REF_CMD_SENTRY_INFO          = 0x020DU,     /* 哨兵决策，6 字节 */
    REF_CMD_RADAR_INFO           = 0x020EU,     /* 雷达决策，1 字节 */
    REF_CMD_INTERACTION          = 0x0301U,     /* 交互数据（UI），127 字节 */
    REF_CMD_CUSTOM_ROBOT         = 0x0302U,     /* 自定义控制器 -> 机器人，30 字节 */
    REF_CMD_MAP_COMMAND          = 0x0303U,     /* 小地图指令，15 字节 */
    REF_CMD_KEYBOARD_MOUSE       = 0x0304U,     /* 键鼠遥控，12 字节 */
    REF_CMD_MAP_ROBOT            = 0x0305U,     /* 小地图雷达数据，24 字节 */
    REF_CMD_CUSTOM_CLIENT        = 0x0306U,     /* 自定义控制器 -> 选手端，8 字节 */
    REF_CMD_CUSTOM_INFO          = 0x0308U,     /* 小地图机器人数据，34 字节 */
} referee_cmd_e;

/* -------------------------------- 数据结构 -------------------------------- */

/** 0x0001 比赛状态，共 11 字节 */
typedef struct __attribute__((__packed__))
{
    uint8_t  game_type : 4;         /* bit0-3 比赛类型 */
    uint8_t  game_progress : 4;     /* bit4-7 当前阶段 */
    uint16_t stage_remain_time;     /* 当前阶段剩余秒数 */
    uint64_t sync_time_stamp;       /* Unix 时间戳 */
} ref_game_status_t;

/** 0x0002 比赛结果 */
typedef struct __attribute__((__packed__))
{
    uint8_t winner;                 /* 0 平局 1 红胜 2 蓝胜 */
} ref_game_result_t;

/** 0x0003 全场血量 */
typedef struct __attribute__((__packed__))
{
    uint16_t red_1_hp;
    uint16_t red_2_hp;
    uint16_t red_3_hp;
    uint16_t red_4_hp;
    uint16_t red_reserved;
    uint16_t red_7_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_hp;
    uint16_t blue_2_hp;
    uint16_t blue_3_hp;
    uint16_t blue_4_hp;
    uint16_t blue_reserved;
    uint16_t blue_7_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
} ref_game_robot_hp_t;

/** 0x0101 场地事件 */
typedef struct __attribute__((__packed__))
{
    uint32_t event_data;
} ref_event_data_t;

/** 0x0104 裁判警告 */
typedef struct __attribute__((__packed__))
{
    uint8_t level;                  /* 1 双方黄牌 2 黄牌 3 红牌 4 判负 */
    uint8_t offending_robot_id;
    uint8_t count;
} ref_warning_t;

/** 0x0105 飞镖信息 */
typedef struct __attribute__((__packed__))
{
    uint8_t  dart_remaining_time;
    uint16_t dart_info;
} ref_dart_info_t;

/** 0x0201 机器人状态与性能 */
typedef struct __attribute__((__packed__))
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_cooling_value;     /* 枪口每秒冷却 */
    uint16_t shooter_heat_limit;        /* 枪口热量上限 */
    uint16_t chassis_power_limit;       /* 底盘功率上限 W */
    uint8_t  power_gimbal_out : 1;
    uint8_t  power_chassis_out : 1;
    uint8_t  power_shooter_out : 1;
} ref_robot_status_t;

/** 0x0202 实时功率与热量 */
typedef struct __attribute__((__packed__))
{
    uint16_t reserved_voltage;      /* 旧协议：底盘输出电压 */
    uint16_t reserved_current;      /* 旧协议：底盘输出电流 */
    float    reserved_power;        /* 旧协议：底盘输出功率 */
    uint16_t buffer_energy;         /* 底盘缓冲能量 J */
    uint16_t shooter_17mm_1_heat;
    uint16_t shooter_17mm_2_heat;
    uint16_t shooter_42mm_heat;
} ref_power_heat_t;

/** 0x0203 机器人位置 */
typedef struct __attribute__((__packed__))
{
    float x;                        /* m */
    float y;                        /* m */
    float angle;                    /* 度，正北为 0 */
} ref_robot_pos_t;

/** 0x0204 增益状态 */
typedef struct __attribute__((__packed__))
{
    uint8_t  recovery_buff;
    uint8_t  cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
    uint8_t  remaining_energy;
} ref_buff_t;

/** 0x0206 受伤状态，共 1 字节 */
typedef struct __attribute__((__packed__))
{
    uint8_t armor_id : 4;           /* bit0-3 装甲 ID */
    uint8_t deduction_reason : 4;   /* bit4-7 扣血原因 */
} ref_hurt_t;

/** 0x0207 射击数据 */
typedef struct __attribute__((__packed__))
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
} ref_shoot_data_t;

/** 0x0208 剩余弹量 */
typedef struct __attribute__((__packed__))
{
    uint16_t projectile_17mm;
    uint16_t projectile_42mm;
    uint16_t remaining_gold_coin;
} ref_projectile_t;

/** 0x0209 RFID 状态 */
typedef struct __attribute__((__packed__))
{
    uint32_t rfid_status;
} ref_rfid_t;

/** 0x0301 交互数据（UI）头部 */
typedef struct __attribute__((__packed__))
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} ref_interaction_header_t;

/* -------------------------------- 对外快照 -------------------------------- */

typedef struct
{
    uint8_t  online;                /* 1 = 最近 1s 内收到过有效帧 */
    uint32_t last_rx_ms;
    uint32_t frames;                /* 累加有效帧数 */
    uint32_t crc_err;
    uint32_t len_err;

    ref_game_status_t   game_status;
    ref_game_result_t   game_result;
    ref_game_robot_hp_t robot_hp;
    ref_robot_status_t  robot_status;
    ref_power_heat_t    power_heat;
    ref_robot_pos_t     robot_pos;
    ref_buff_t          buff;
    ref_hurt_t          hurt;
    ref_shoot_data_t    shoot;
    ref_projectile_t    projectile;
    ref_rfid_t          rfid;
    ref_event_data_t    event;
} referee_fdb_t;

/* --------------------------------- 接口 ----------------------------------- */

/** 清状态，上电调用一次 */
void referee_system_init(void);

/**
 * @brief 喂入一段串口缓冲，内部逐字节同步
 * @param data 数据起始
 * @param len  字节数
 * @note  只能在单一上下文调用（串口回调或同一任务），不可重入
 */
void referee_data_unpack(const uint8_t *data, uint16_t len);

/** 拷贝一份完整快照 */
void referee_get_fdb(referee_fdb_t *out);

/** 1 = 裁判系统在线 */
uint8_t referee_is_online(void);

/** 底盘功率上限 W，离线时返回 0 */
uint16_t referee_get_chassis_power_limit(void);

/** 底盘缓冲能量 J，离线时返回 0 */
uint16_t referee_get_buffer_energy(void);

#ifdef __cplusplus
}
#endif

#endif /* REFEREE_SYSTEM_H */
