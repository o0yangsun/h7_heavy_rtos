/**
  ******************************************************************************
  * @file    dj_motor.c
  * @brief   大疆电机驱动实现，详见 dj_motor.h
  ******************************************************************************
  */

#include "dj_motor.h"
#include "robot_config.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* --------------------------------- 内部常量 -------------------------------- */

/** 发送帧在槽位内的排列：每个总线 3 帧 */
#define FRAME_IDX_1FF           0U      /* 0x1FF：6020 ID1-4 电压 / 3508 ID5-8 电流 */
#define FRAME_IDX_200           1U      /* 0x200：3508 ID1-4 电流 */
#define FRAME_IDX_2FF           2U      /* 0x2FF：6020 ID5-7 电压 */

#define FRAME_PER_BUS           3U
#define FRAME_TOTAL             (3U * FRAME_PER_BUS)   /* 9 个槽位 */

/** 输出限幅 */
#define DJ_CURRENT_LIMIT        16384   /* 3508/2006 电流 */
#define DJ_VOLTAGE_LIMIT        30000   /* 6020 电压 */

/** 反馈报文 ID 起点 */
#define DJ_RX_ID_BASE           0x200U  /* 3508/2006: 0x200 + id */
#define DJ_RX_ID_6020_BASE      0x204U  /* GM6020:    0x204 + id */

/* --------------------------------- 内部变量 -------------------------------- */

static dj_motor_object_t s_motor[DJ_MOTOR_MAX_COUNT];
static uint8_t           s_motor_count = 0;

/** 每个发送槽位是否有电机注册，无则不发该帧 */
static uint8_t s_frame_enabled[FRAME_TOTAL] = {0};
/** 每个发送槽位的 8 字节缓冲 */
static uint8_t s_frame_data[FRAME_TOTAL][8] = {{0}};
/** 槽位占用表：[槽位][帧内序号] = 电机实例，用于冲突检测 */
static dj_motor_object_t *s_slot_owner[FRAME_TOTAL][4] = {{0}};
/** 反馈反查表：[总线-1][std_id - 0x201] = 电机实例 */
static dj_motor_object_t *s_rx_map[3][DJ_RX_MAP_SIZE] = {{0}};

/** 统计量，供台架验证用 */
static volatile uint32_t s_tx_count = 0;        /* 成功入队的帧数 */
static volatile uint32_t s_tx_error = 0;        /* 入队失败的帧数 */
static volatile uint32_t s_rx_count[3] = {0};   /* 各总线收到的反馈帧数（已匹配到电机的） */

/* 各总线收到的任意标准帧计数和最后一个报文 ID，用于排查 ID 不符 */
static volatile uint32_t s_rx_any[3]  = {0};
static volatile uint32_t s_last_id[3] = {0};
static volatile uint32_t s_busoff_count = 0;    /* bus-off 次数 */

/* bus-off 处理：ISR 只置标志，任务中恢复，恢复后冷却 500ms */
static volatile uint8_t s_busoff_pending[3] = {0};

/** 非电机帧的转发钩子，由上层模块注册 */
static dj_motor_rx_hook_t s_rx_hook = NULL;

static bool             s_bus_dead[3]      = {false};
static uint32_t         s_bus_retry_ms[3]  = {0};

/** bus-off 之后的重试间隔（毫秒） */
#define DJ_BUS_RETRY_MS         500U
/* --------------------------------- 内部函数 -------------------------------- */

static FDCAN_HandleTypeDef *bus_to_handle(uint8_t bus)
{
    switch (bus)
    {
        case 1: return &hfdcan1;
        case 2: return &hfdcan2;
        case 3: return &hfdcan3;
        default: return NULL;
    }
}

static uint8_t handle_to_bus(const FDCAN_HandleTypeDef *h)
{
    if (h->Instance == FDCAN1) return 1;
    if (h->Instance == FDCAN2) return 2;
    if (h->Instance == FDCAN3) return 3;
    return 0;
}

static uint32_t frame_idx_to_stdid(uint8_t frame_idx)
{
    switch (frame_idx)
    {
        case FRAME_IDX_1FF: return 0x1FF;
        case FRAME_IDX_200: return 0x200;
        case FRAME_IDX_2FF: return 0x2FF;
        default:            return 0x200;
    }
}

static int16_t clamp_i16(int32_t v, int32_t limit)
{
    if (v >  limit) return (int16_t) limit;
    if (v < -limit) return (int16_t)-limit;
    return (int16_t)v;
}

/** 发送一帧标准数据帧，8 字节 */
static bool can_send_std(FDCAN_HandleTypeDef *h, uint32_t std_id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef tx = {0};

    tx.Identifier          = std_id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, (uint8_t *)data) == HAL_OK);
}

/**
 * @brief 强制使用经典帧并关闭自动重传
 * @note  CubeMX 生成的 fdcan.c 默认是 FD_BRS，重新生成时会被覆盖，
 *        因此在初始化后统一改掉。C620/C610 只支持经典帧。
 *        DAR=1 可避免总线上无人应答时 TX FIFO 被堵死。
 */
static void can_force_classic(FDCAN_HandleTypeDef *h)
{
    h->Instance->CCCR |= FDCAN_CCCR_INIT;
    while ((h->Instance->CCCR & FDCAN_CCCR_INIT) == 0U)
    {
        /* 等 INIT 生效 */
    }

    h->Instance->CCCR |= FDCAN_CCCR_CCE;
    h->Instance->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);   /* 纯经典帧 */
    h->Instance->CCCR |= FDCAN_CCCR_DAR;                          /* 不自动重传 */

    h->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    while ((h->Instance->CCCR & FDCAN_CCCR_INIT) != 0U)
    {
        /* 等退出 INIT */
    }
}

/** 配置一条总线：过滤器 + 全局过滤 + 启动 + 开接收中断 */
static void can_setup(FDCAN_HandleTypeDef *h)
{
    can_force_classic(h);

    /* 标准过滤器：掩码 0，全部接收 */
    if (h->Init.StdFiltersNbr > 0U)
    {
        FDCAN_FilterTypeDef filter = {0};

        filter.IdType       = FDCAN_STANDARD_ID;
        filter.FilterIndex  = 0;
        filter.FilterType   = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1    = 0x000;
        filter.FilterID2    = 0x000;
        (void)HAL_FDCAN_ConfigFilter(h, &filter);
    }

    /* 非匹配帧直接丢弃，避免落入未分配的 RX FIFO1 */
    (void)HAL_FDCAN_ConfigGlobalFilter(h,
                                       FDCAN_REJECT,
                                       FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE,
                                       FDCAN_REJECT_REMOTE);

    (void)HAL_FDCAN_Start(h);

#if ENABLE_CAN_MONITOR
    /* 只听模式：不发 ACK、不发错误帧、不参与仲裁 */
    h->Instance->CCCR |= FDCAN_CCCR_INIT;
    while ((h->Instance->CCCR & FDCAN_CCCR_INIT) == 0U) { }
    h->Instance->CCCR |= FDCAN_CCCR_CCE;
    h->Instance->CCCR |= FDCAN_CCCR_MON;
    h->Instance->CCCR &= ~FDCAN_CCCR_INIT;
#endif

    /* 打开接收和 bus-off 通知 */
    (void)HAL_FDCAN_ActivateNotification(h,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
                                         0);
}

/**
 * @brief 切换内部回环模式
 * @param on true = 内部回环（帧不出引脚，自己应答自己）
 * @note  回环模式下 TX 引脚保持隐性。
 */
void dj_motor_set_loopback(bool on)
{
    FDCAN_HandleTypeDef *bus[3] = {&hfdcan1, &hfdcan2, &hfdcan3};

    for (uint8_t i = 0; i < 3U; i++)
    {
        FDCAN_HandleTypeDef *h = bus[i];

        /* 进初始化模式并允许改配置 */
        h->Instance->CCCR |= FDCAN_CCCR_INIT;
        while ((h->Instance->CCCR & FDCAN_CCCR_INIT) == 0U)
        {
            /* 等 INRQ 生效 */
        }
        h->Instance->CCCR |= FDCAN_CCCR_CCE;

        if (on)
        {
            h->Instance->CCCR |= FDCAN_CCCR_TEST;
            h->Instance->TEST |= FDCAN_TEST_LBCK;
        }
        else
        {
            h->Instance->TEST &= ~FDCAN_TEST_LBCK;
            h->Instance->CCCR &= ~FDCAN_CCCR_TEST;
        }

        /* 退出初始化模式 */
        h->Instance->CCCR &= ~FDCAN_CCCR_INIT;
        while ((h->Instance->CCCR & FDCAN_CCCR_INIT) != 0U)
        {
            /* 等 INRQ 清除 */
        }
    }
}

/** 解析 8 字节反馈 */
static void decode_feedback(dj_motor_object_t *motor, const uint8_t *data)
{
    dj_motor_measure_t *ms = &motor->measure;
    int16_t delta;

    ms->last_ecd   = ms->ecd;
    ms->ecd        = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    ms->speed_rpm  = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    ms->real_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    ms->temperature  = data[6];

    /* 一阶平滑：编码器转速为 1 rpm 量化，微分项会把它放大成电流抖动 */
    ms->speed_rpm_f = (1.0f - DJ_SPEED_SMOOTH_COEF) * ms->speed_rpm_f
                    + DJ_SPEED_SMOOTH_COEF * (float)ms->speed_rpm;
    ms->real_current_f = (1.0f - DJ_CURRENT_SMOOTH_COEF) * ms->real_current_f
                       + DJ_CURRENT_SMOOTH_COEF * (float)ms->real_current;

    ms->angle_single_round = (float)ms->ecd * (360.0f / 8192.0f);

    /* 多圈累加：假设两次采样之间转过的角度小于 180° */
    delta = (int16_t)(ms->ecd - ms->last_ecd);
    if (delta > 4096)
    {
        ms->total_round--;
    }
    else if (delta < -4096)
    {
        ms->total_round++;
    }
    ms->total_angle = (float)ms->total_round * 360.0f + ms->angle_single_round;

    ms->last_update_ms = HAL_GetTick();
    ms->online         = true;
}

/* ---------------------------------- 接口 ---------------------------------- */

/**
 * @brief 设置实例在共享消息 RAM 中的起始偏移
 * @param offset_words 相对 SRAMCAN 的偏移，单位：字
 * @note  FDCAN1/2/3 共用 2560 字消息 RAM，CubeMX 生成的偏移全是 0，
 *        三套过滤器/FIFO/缓冲区会重叠，必须按实例错开。
 */
static void can_remap(FDCAN_HandleTypeDef *h, uint32_t offset_words)
{
    if (h->Init.MessageRAMOffset == offset_words)
    {
        return;
    }

    (void)HAL_FDCAN_DeInit(h);
    h->Init.MessageRAMOffset = offset_words;
    (void)HAL_FDCAN_Init(h);
}

void dj_motor_can_init(void)
{
    /* 单实例约占用 189 字，按 512 字间隔分区，留足余量 */
    can_remap(&hfdcan1, 0U);
    can_remap(&hfdcan2, 512U);
    can_remap(&hfdcan3, 1024U);

    can_setup(&hfdcan1);
    can_setup(&hfdcan2);
    can_setup(&hfdcan3);
}

bool dj_motor_send_std(uint8_t bus, uint32_t std_id, const uint8_t *data)
{
    FDCAN_HandleTypeDef *h = bus_to_handle(bus);

    if ((h == NULL) || (data == NULL))
    {
        return false;
    }
    return can_send_std(h, std_id, data);
}

dj_motor_object_t *dj_motor_register(const dj_motor_config_t *cfg)
{
    uint8_t frame_idx;
    uint8_t msg_index;
    uint32_t rx_id;
    uint8_t rx_index;
    uint8_t frame_slot;
    dj_motor_object_t *motor;

    if (cfg == NULL || cfg->bus < 1U || cfg->bus > 3U)
    {
        return NULL;
    }

    /* 按类型换算：发送槽位 / 帧内序号 / 反馈报文 ID */
    switch (cfg->type)
    {
        case DJ_MOTOR_M3508:
        case DJ_MOTOR_M2006:
            if (cfg->id < 1U || cfg->id > 8U)
            {
                return NULL;
            }
            if (cfg->id <= 4U)
            {
                frame_idx = FRAME_IDX_200;      /* ID1-4 → 0x200 */
                msg_index = (uint8_t)(cfg->id - 1U);
            }
            else
            {
                frame_idx = FRAME_IDX_1FF;      /* ID5-8 → 0x1FF */
                msg_index = (uint8_t)(cfg->id - 5U);
            }
            rx_id = DJ_RX_ID_BASE + cfg->id;
            break;

        case DJ_MOTOR_GM6020:
            if (cfg->id < 1U || cfg->id > 7U)
            {
                return NULL;
            }
            if (cfg->id <= 4U)
            {
                frame_idx = FRAME_IDX_1FF;      /* ID1-4 → 0x1FF */
                msg_index = (uint8_t)(cfg->id - 1U);
            }
            else
            {
                frame_idx = FRAME_IDX_2FF;      /* ID5-7 → 0x2FF */
                msg_index = (uint8_t)(cfg->id - 5U);
            }
            rx_id = DJ_RX_ID_6020_BASE + cfg->id;
            break;

        default:
            return NULL;
    }

    rx_index   = (uint8_t)(rx_id - 0x201U);
    frame_slot = (uint8_t)((cfg->bus - 1U) * FRAME_PER_BUS + frame_idx);

    if (rx_index >= DJ_RX_MAP_SIZE)
    {
        return NULL;
    }

    /* 冲突检测与插入做成临界区，避免多个任务并发注册时踩表 */
    taskENTER_CRITICAL();

    if (s_motor_count >= DJ_MOTOR_MAX_COUNT)
    {
        taskEXIT_CRITICAL();
        return NULL;
    }

    /* 同一槽位被占用：说明同总线上有两个电机算出同一个帧内序号 */
    if (s_slot_owner[frame_slot][msg_index] != NULL)
    {
        taskEXIT_CRITICAL();
        return NULL;
    }

    /* 反馈 ID 被占用：同总线上两个电机不能有相同的反馈报文 ID */
    if (s_rx_map[cfg->bus - 1U][rx_index] != NULL)
    {
        taskEXIT_CRITICAL();
        return NULL;
    }

    motor = &s_motor[s_motor_count];
    memset(motor, 0, sizeof(*motor));
    motor->type       = cfg->type;
    motor->bus        = cfg->bus;
    motor->id         = cfg->id;
    motor->frame_slot = frame_slot;
    motor->msg_index  = msg_index;
    motor->rx_index   = rx_index;
    motor->stopped    = true;               /* 默认松开，显式 enable 后才出力 */
    motor->control    = cfg->control;
    motor->ctx        = cfg->ctx;

    s_slot_owner[frame_slot][msg_index]      = motor;
    s_rx_map[cfg->bus - 1U][rx_index]        = motor;
    s_frame_enabled[frame_slot]              = 1U;
    s_motor_count++;

    taskEXIT_CRITICAL();

    return motor;
}

void dj_motor_enable(dj_motor_object_t *motor)
{
    if (motor != NULL)
    {
        motor->stopped = false;
    }
}

void dj_motor_relax(dj_motor_object_t *motor)
{
    if (motor != NULL)
    {
        motor->stopped = true;
    }
}

/**
 * @brief 从 bus-off 恢复总线
 * @note  只清 CCCR.INIT 可能卡在未同步状态，先用 Stop/Start 复位协议引擎，
 *        并丢弃已排队的发送请求。
 */
static void can_recover(FDCAN_HandleTypeDef *h)
{
    (void)HAL_FDCAN_AbortTxRequest(h, 0xFFU);   /* 丢掉发不出去的队列帧 */
    (void)HAL_FDCAN_Stop(h);                    /* 进 init，清 bus-off 状态与计数器 */
    (void)HAL_FDCAN_Start(h);                   /* 重新参与仲裁，等 129×11 个隐性位 */
}

/**
 * @brief 每次发送前维护总线健康状态
 * @return true = 这条总线当前可以发
 */
static bool bus_is_usable(uint8_t bus, uint32_t now_ms)
{
    uint8_t idx = (uint8_t)(bus - 1U);
    FDCAN_HandleTypeDef *h = bus_to_handle(bus);

    if (h == NULL)
    {
        return false;
    }

    /* bus-off 后恢复并冷却，避免无节点时反复重试 */
    if (s_busoff_pending[idx] != 0U)
    {
        s_busoff_pending[idx] = 0U;
        can_recover(h);
        s_bus_dead[idx]     = true;
        s_bus_retry_ms[idx] = now_ms + DJ_BUS_RETRY_MS;
        return false;
    }

    if (s_bus_dead[idx])
    {
        if ((int32_t)(now_ms - s_bus_retry_ms[idx]) < 0)
        {
            return false;
        }
        /* 冷却结束，放一帧试探 */
        s_bus_dead[idx] = false;
    }

    return true;
}

void dj_motor_send_all(void)
{
#if ENABLE_CAN_MONITOR
    return;                     /* 只听模式：一个帧都不发 */
#else
    uint8_t i;
    uint8_t slot;
    uint32_t now_ms = HAL_GetTick();

    /* 1. 先清零：没被任何电机占用的字节必须是 0，不能是上一轮的残留 */
    for (slot = 0; slot < FRAME_TOTAL; slot++)
    {
        if (s_frame_enabled[slot])
        {
            memset(s_frame_data[slot], 0, sizeof(s_frame_data[slot]));
        }
    }

    /* 2. 逐个电机算控制量并填进对应槽位 */
    for (i = 0; i < s_motor_count; i++)
    {
        dj_motor_object_t *motor = &s_motor[i];
        int32_t out = 0;

        if (!motor->stopped && motor->control != NULL)
        {
            out = motor->control(&motor->measure, motor->ctx);
        }

        if (motor->type == DJ_MOTOR_GM6020)
        {
            out = clamp_i16(out, DJ_VOLTAGE_LIMIT);
        }
        else
        {
            out = clamp_i16(out, DJ_CURRENT_LIMIT);
        }

        s_frame_data[motor->frame_slot][2U * motor->msg_index]      = (uint8_t)((out >> 8) & 0xFF);
        s_frame_data[motor->frame_slot][2U * motor->msg_index + 1U] = (uint8_t)(out & 0xFF);
    }

    /* 3. 按槽位发出去 */
    for (slot = 0; slot < FRAME_TOTAL; slot++)
    {
        uint8_t bus;
        uint8_t frame_idx;
        FDCAN_HandleTypeDef *h;

        if (!s_frame_enabled[slot])
        {
            continue;
        }

        bus       = (uint8_t)(slot / FRAME_PER_BUS + 1U);
        frame_idx = (uint8_t)(slot % FRAME_PER_BUS);
        h         = bus_to_handle(bus);

        if (h == NULL)
        {
            continue;
        }

        /* 总线处于 bus-off / 冷却期时整条总线跳过，计数器也不动，
         * 这样 err 只在真的"往能发的总线上发"的时候才涨 */
        if (!bus_is_usable(bus, now_ms))
        {
            continue;
        }

        if (can_send_std(h, frame_idx_to_stdid(frame_idx), s_frame_data[slot]))
        {
            s_tx_count++;
        }
        else
        {
            s_tx_error++;
        }
    }
#endif /* ENABLE_CAN_MONITOR */
}

uint8_t dj_motor_get_dead_mask(void)
{
    uint8_t mask = 0;

    for (uint8_t i = 0; i < 3U; i++)
    {
        if (s_bus_dead[i])
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

void dj_motor_update_online(uint32_t now_ms, uint32_t timeout_ms)
{
    uint8_t i;

    for (i = 0; i < s_motor_count; i++)
    {
        dj_motor_object_t *motor = &s_motor[i];

        if (motor->measure.online &&
            (uint32_t)(now_ms - motor->measure.last_update_ms) > timeout_ms)
        {
            motor->measure.online = false;
        }
    }
}

uint8_t dj_motor_get_count(void)
{
    return s_motor_count;
}

dj_motor_object_t *dj_motor_get_by_index(uint8_t index)
{
    if (index >= s_motor_count)
    {
        return NULL;
    }
    return &s_motor[index];
}

void dj_motor_rx_dispatch(uint8_t bus, uint32_t std_id, const uint8_t *data)
{
    uint32_t offset;
    dj_motor_object_t *motor;

    if (bus < 1U || bus > 3U || data == NULL || std_id < 0x201U)
    {
        return;
    }

    offset = std_id - 0x201U;
    if (offset >= DJ_RX_MAP_SIZE)
    {
        return;
    }

    motor = s_rx_map[bus - 1U][offset];
    if (motor == NULL)
    {
        return;     /* 未注册的电机 */
    }

    decode_feedback(motor, data);
    s_rx_count[bus - 1U]++;
}

void dj_motor_set_rx_hook(dj_motor_rx_hook_t hook)
{
    s_rx_hook = hook;
}

/* ------------------------------- 统计量接口 -------------------------------- */

uint32_t dj_motor_get_tx_count(void)
{
    return s_tx_count;
}

uint32_t dj_motor_get_tx_error(void)
{
    return s_tx_error;
}

uint32_t dj_motor_get_rx_count(uint8_t bus)
{
    if (bus < 1U || bus > 3U)
    {
        return 0;
    }
    return s_rx_count[bus - 1U];
}

uint32_t dj_motor_get_rx_any(uint8_t bus)
{
    if (bus < 1U || bus > 3U)
    {
        return 0;
    }
    return s_rx_any[bus - 1U];
}

uint32_t dj_motor_get_last_id(uint8_t bus)
{
    if (bus < 1U || bus > 3U)
    {
        return 0;
    }
    return s_last_id[bus - 1U];
}

uint32_t dj_motor_get_busoff_count(void)
{
    return s_busoff_count;
}

/* ------------------------------ HAL 回调实现 ------------------------------ */

/**
 * @brief FDCAN RX FIFO0 新报文回调
 * @note  HAL 里是 __weak 的，这里提供强定义
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t bus;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    bus = handle_to_bus(hfdcan);
    if (bus == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        if (header.IdType == FDCAN_STANDARD_ID)
        {
            s_last_id[bus - 1U] = header.Identifier;
            s_rx_any[bus - 1U]++;

            if (header.DataLength == FDCAN_DLC_BYTES_8)
            {
                if (s_rx_hook != NULL)
                {
                    s_rx_hook(bus, header.Identifier, data);
                }
                dj_motor_rx_dispatch(bus, header.Identifier, data);
            }
        }
    }
}

/**
 * @brief 总线错误回调
 * @note  FDCAN 没有自动 bus-off 恢复，这里只置标志，由任务完成恢复。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    uint8_t bus = handle_to_bus(hfdcan);

    if (((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U) || (bus == 0U))
    {
        return;
    }

    s_busoff_count++;

    /* 恢复动作放到任务中执行 */
    s_busoff_pending[bus - 1U] = 1U;
}
