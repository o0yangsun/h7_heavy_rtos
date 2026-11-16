/**
  ******************************************************************************
  * @file    motor_task.c
  * @brief   电机发送任务，1kHz 统一发帧
  ******************************************************************************
  */

#include "motor_task.h"
#include "dj_motor.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------ 全局变量 ------------------------------ */

volatile motor_diag_t g_motor_diag;

#if ENABLE_MOTOR_CAN1_MONITOR
static uint32_t s_can1_first_rx   = 0U;
static uint32_t s_can1_last_print = 0U;
#endif
static uint32_t s_map_dump_at = 0U;

/* ------------------------------ TX 探针 ------------------------------ */

#if ENABLE_MOTOR_CAN_TX_PROBE
static void can_tx_probe_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* FDCAN1_TX = PD1，FDCAN3_TX = PD13，FDCAN2_TX = PB6 */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/** 8ms 周期：7ms 显性（TX=低）+ 1ms 隐性（TX=高），万用表可直接读出平均值 */
static void can_tx_probe_step(void)
{
    static uint32_t n = 0U;
#if ENABLE_MOTOR_CAN_TX_PROBE == 2
    /* 静态显性：H 应稳定 ~3.5V、L 应稳定 ~1.5V，万用表直接读 */
    GPIO_PinState st = GPIO_PIN_RESET;
#else
    GPIO_PinState st = (((n % 8U) == 7U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_13, st);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, st);
    n++;
}
#endif

/**
 * CAN1 开机自检
 *   依次执行静默监听、内部回环、波特率扫描和总线探测，
 *   结果写入 g_can_selftest
 */
#if ENABLE_MOTOR_CAN_SCAN

typedef struct
{
    uint32_t stage;            /* 跑完 = 9 */
    uint32_t listen_frames;    /* 1Mbps 静默 500ms 收到的帧数 */
    uint32_t listen_id;        /* 最后一帧 ID */
    uint32_t lb_frames;        /* 内部回环 10 帧收回几帧 */
    uint32_t scan_baud[6];
    uint32_t scan_frames[6];
    uint32_t scan_tec[6];
    uint32_t scan_lec[6];
    uint32_t scan_ack[6];      /* 1 = 该速率下有人回 ACK */
    /* FDCAN 内核时钟，位定时依赖此值 */
    uint32_t fdcan_clk_hz;
    uint32_t sysclk_hz;
    uint32_t hclk_hz;
    uint32_t baud_calc;        /* 按 NBTP 反算的实际波特率 */
    /* 三总线探测：看电调/分析仪到底挂在哪个 FDCAN 实例上 */
    uint32_t bus_tec[3];
    uint32_t bus_lec[3];
    uint32_t bus_ack[3];
    uint32_t bus_frames[3];
} can_selftest_t;

volatile can_selftest_t g_can_selftest;

/** 运行中改波特率：Stop 后 CCE=1，NBTP 可直接写 */
static void can_apply_baud_h(FDCAN_HandleTypeDef *h, uint16_t psc, uint8_t tseg1, uint8_t tseg2, uint8_t sjw)
{
    (void)HAL_FDCAN_Stop(h);

    if (sjw > tseg2)
    {
        sjw = tseg2;
    }

    h->Instance->NBTP = ((uint32_t)(sjw   - 1U) << 25)
                           | ((uint32_t)(psc   - 1U) << 16)
                           | ((uint32_t)(tseg1 - 1U) <<  8)
                           |  (uint32_t)(tseg2 - 1U);

    (void)HAL_FDCAN_Start(h);
}

static void can_send_fixed_h(FDCAN_HandleTypeDef *h, uint32_t id, uint8_t tag)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8];

    for (uint32_t i = 0; i < 8U; i++)
    {
        d[i] = tag;
    }

    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, d);
}

static void can1_selftest_run(void)
{
    static const struct { uint16_t psc; uint8_t t1; uint8_t t2; uint32_t baud; } tbl[6] =
    {
        {  1U, 59U, 20U, 1000000U },   /* 80 MHz / 1 / 80 tq  */
        {  1U, 74U, 25U,  800000U },   /* 80 MHz / 1 / 100 tq */
        {  2U, 59U, 20U,  500000U },
        {  4U, 59U, 20U,  250000U },
        {  8U, 59U, 20U,  125000U },
        { 10U, 59U, 20U,  100000U },
    };
    uint32_t before;

    memset((void *)&g_can_selftest, 0, sizeof(g_can_selftest));

    /* 阶段 1：1 Mbps 静默监听，看总线上有没有活着的节点 */
    g_can_selftest.stage = 1U;
    before = dj_motor_get_rx_any(1U);
    osDelay(500U);
    g_can_selftest.listen_frames = dj_motor_get_rx_any(1U) - before;
    g_can_selftest.listen_id     = dj_motor_get_last_id(1U);

    /* 阶段 2：内部回环，验证发送/接收/中断/计数整条代码路径 */
    g_can_selftest.stage = 2U;
    dj_motor_set_loopback(true);
    before = dj_motor_get_rx_any(1U);
    for (uint32_t i = 0; i < 10U; i++)
    {
        can_send_fixed_h(&hfdcan1, 0x123U, 0x5AU);
        osDelay(2U);
    }
    osDelay(60U);
    g_can_selftest.lb_frames = dj_motor_get_rx_any(1U) - before;
    dj_motor_set_loopback(false);

    /* 阶段 3：波特率扫描，用 ACK 判定对端速率 */
    g_can_selftest.stage = 3U;
    for (uint32_t k = 0; k < 6U; k++)
    {
        can_apply_baud_h(&hfdcan1, tbl[k].psc, tbl[k].t1, tbl[k].t2, 20U);
        hfdcan1.Instance->ECR = 0U;
        before = dj_motor_get_rx_any(1U);

        osDelay(150U);                                     /* 先被动听 */
        g_can_selftest.scan_frames[k] = dj_motor_get_rx_any(1U) - before;

        hfdcan1.Instance->ECR = 0U;                        /* 再清一次，只看自己这两帧 */
        can_send_fixed_h(&hfdcan1, 0x123U, (uint8_t)(0xA0U + k));
        osDelay(5U);
        can_send_fixed_h(&hfdcan1, 0x123U, (uint8_t)(0xA0U + k));
        osDelay(20U);

        g_can_selftest.scan_baud[k] = tbl[k].baud;
        g_can_selftest.scan_tec[k]  = hfdcan1.Instance->ECR & 0xFFU;
        g_can_selftest.scan_lec[k]  = hfdcan1.Instance->PSR & 0x7U;
        g_can_selftest.scan_ack[k]  = (g_can_selftest.scan_tec[k] == 0U) ? 1U : 0U;
    }

    /* 收尾：恢复 1 Mbps 正常工作速率 */
    can_apply_baud_h(&hfdcan1, 1U, 59U, 20U, 20U);
    hfdcan1.Instance->ECR = 0U;

    /* 阶段 4：三条总线各发两帧，确认电调挂在哪个 FDCAN 实例上 */
    {
        FDCAN_HandleTypeDef *bus[3] = {&hfdcan1, &hfdcan2, &hfdcan3};

        for (uint32_t i = 0; i < 3U; i++)
        {
            can_apply_baud_h(bus[i], 1U, 59U, 20U, 20U);
            bus[i]->Instance->ECR = 0U;
            before = dj_motor_get_rx_any((uint8_t)(i + 1U));

            osDelay(100U);                                  /* 先听 */
            g_can_selftest.bus_frames[i] = dj_motor_get_rx_any((uint8_t)(i + 1U)) - before;

            bus[i]->Instance->ECR = 0U;                     /* 再发 */
            can_send_fixed_h(bus[i], 0x123U, (uint8_t)(0xB0U + i));
            osDelay(5U);
            can_send_fixed_h(bus[i], 0x123U, (uint8_t)(0xB0U + i));
            osDelay(20U);

            g_can_selftest.bus_tec[i]    = bus[i]->Instance->ECR & 0xFFU;
            g_can_selftest.bus_lec[i]    = bus[i]->Instance->PSR & 0x7U;
            g_can_selftest.bus_ack[i]    = (g_can_selftest.bus_tec[i] == 0U) ? 1U : 0U;
        }
    }

    /* 记录时钟配置并按 NBTP 反算实际波特率 */
    g_can_selftest.fdcan_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    g_can_selftest.sysclk_hz    = HAL_RCC_GetSysClockFreq();
    g_can_selftest.hclk_hz      = HAL_RCC_GetHCLKFreq();
    {
        uint32_t nbtp = hfdcan1.Instance->NBTP;
        uint32_t psc  = ((nbtp >> 16) & 0x1FFU) + 1U;
        uint32_t t1   = ((nbtp >>  8) & 0xFFU)  + 1U;
        uint32_t t2   = ( nbtp        & 0x7FU)  + 1U;
        g_can_selftest.baud_calc = g_can_selftest.fdcan_clk_hz / (psc * (1U + t1 + t2));
    }

    g_can_selftest.stage = 9U;
}

#endif /* ENABLE_MOTOR_CAN_SCAN */

/* ------------------------------ 最小发送测试 ------------------------------ */

#if ENABLE_MOTOR_MINIMAL_TX
static void minimal_tx_step(void)
{
    static uint32_t n = 0U;
    FDCAN_TxHeaderTypeDef tx = {0};
    static uint8_t data[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};

    /* 本路径绕过 dj_motor_send_all()，bus-off 恢复需自行处理 */
    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_Start(&hfdcan1);
    }

    n++;
    if ((n % 10U) != 0U)        /* 每 10ms 一帧 */
    {
        return;
    }

    tx.Identifier          = 0x123U;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
}
#endif

/* ------------------------------ 总线识别探针 ------------------------------ */

#if ENABLE_MOTOR_BUS_ID_PROBE
static void probe_send(FDCAN_HandleTypeDef *h, uint32_t id, uint8_t tag)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8];

    for (uint8_t i = 0; i < 8U; i++)
    {
        d[i] = tag;
    }

    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, d);
}

static void probe_recover(FDCAN_HandleTypeDef *h)
{
    if ((h->Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        (void)HAL_FDCAN_AbortTxRequest(h, 0xFFU);
        (void)HAL_FDCAN_Stop(h);
        (void)HAL_FDCAN_Start(h);
    }
}

/**
 * @brief 1kHz 识别帧：0x200 全 0（安全）+ 0x700 带本路标记
 */
static void bus_id_probe_step(void)
{
    static uint32_t n = 0U;

    n++;
    /* 每个周期都检查 bus-off，否则大部分时间会停在 INIT 状态 */
    if (1U)
    {
        probe_recover(&hfdcan1);
        probe_recover(&hfdcan2);
        probe_recover(&hfdcan3);
    }

    /* 0x200 全 0：标准控制帧，四路电流都为 0，电机不会动 */
    probe_send(&hfdcan1, 0x200U, 0x00U);
    probe_send(&hfdcan2, 0x200U, 0x00U);
    probe_send(&hfdcan3, 0x200U, 0x00U);

    /* 0x700 标记帧：0x7xx 不是任何 DJI 设备的 ID，电机不会理它 */
    probe_send(&hfdcan1, 0x700U, 0x11U);
    probe_send(&hfdcan2, 0x700U, 0x22U);
    probe_send(&hfdcan3, 0x700U, 0x33U);
}
#endif

/* ------------------------------ 台架验证 ------------------------------ */

#if ENABLE_MOTOR_BENCH

/** 三角波参数：amp 是幅值，phase 是相位偏移，让各电机波形错开 */
typedef struct
{
    int16_t  amp;
    uint16_t phase;
} bench_ctx_t;

static uint32_t s_bench_tick = 0;

static int16_t bench_control(const dj_motor_measure_t *measure, void *ctx)
{
    const bench_ctx_t *cfg = (const bench_ctx_t *)ctx;
    int32_t tri;

    (void)measure;

    /* 0..4000 的锯齿 → -2000..2000 → 按幅值缩放 */
    tri = (int32_t)((s_bench_tick * 2U + cfg->phase) % 4000U) - 2000;
    return (int16_t)(tri * cfg->amp / 2000);
}

typedef struct
{
    uint8_t         bus;
    uint8_t         id;
    dj_motor_type_e type;
    int16_t         amp;
    uint16_t        phase;
} bench_motor_cfg_t;

static const bench_motor_cfg_t s_bench_motor[] =
{
    /* CAN1：底盘四轮 */
    {CAN_CHASSIS,  1U, DJ_MOTOR_M3508,  2000,   200U},
    {CAN_CHASSIS,  2U, DJ_MOTOR_M3508,  2000,   400U},
    {CAN_CHASSIS,  3U, DJ_MOTOR_M3508,  2000,   600U},
    {CAN_CHASSIS,  4U, DJ_MOTOR_M3508,  2000,   800U},
    /* CAN2：yaw、拨弹、特化 */
    {CAN_YAW,      1U, DJ_MOTOR_GM6020, 20000,    0U},
    {CAN_TRIGGER,  1U, DJ_MOTOR_M3508,  1200,   900U},
    {CAN_SPECIAL,  2U, DJ_MOTOR_M3508,  1200,  1100U},
    {CAN_SPECIAL,  3U, DJ_MOTOR_M3508,  1200,  1300U},
    /* CAN3：pitch、摩擦轮 */
    {CAN_PITCH,    1U, DJ_MOTOR_GM6020, 20000,  400U},
    {CAN_FRICTION, 1U, DJ_MOTOR_M3508,  4000,   300U},
    {CAN_FRICTION, 2U, DJ_MOTOR_M3508,  4000,   600U},
    {CAN_FRICTION, 3U, DJ_MOTOR_M3508,  4000,   900U},
};

static void motor_bench_register(void)
{
    static bench_ctx_t ctx[DJ_MOTOR_MAX_COUNT];
    uint8_t count = (uint8_t)(sizeof(s_bench_motor) / sizeof(s_bench_motor[0]));

    for (uint8_t i = 0; i < count; i++)
    {
        dj_motor_config_t cfg;

        ctx[i].amp   = s_bench_motor[i].amp;
        ctx[i].phase = s_bench_motor[i].phase;

        memset(&cfg, 0, sizeof(cfg));
        cfg.type    = s_bench_motor[i].type;
        cfg.bus     = s_bench_motor[i].bus;
        cfg.id      = s_bench_motor[i].id;
        cfg.control = bench_control;
        cfg.ctx     = &ctx[i];
        (void)dj_motor_register(&cfg);
    }

    /* 台架模式下全部使能，否则输出恒 0 */
    for (uint8_t i = 0; i < dj_motor_get_count(); i++)
    {
        dj_motor_enable(dj_motor_get_by_index(i));
    }
}

#endif /* ENABLE_MOTOR_BENCH */

#if ENABLE_MOTOR_BENCH && ENABLE_MOTOR_LOOPBACK

/**
 * @brief 回环模式下发送伪反馈帧
 *
 * 按已注册电机的反馈 ID 发送已知数据，用于验证接收解析链路。
 * 仅在 ENABLE_MOTOR_LOOPBACK = 1 时编译。
 */
static void bench_inject_feedback(void)
{
    uint8_t count = dj_motor_get_count();

    for (uint8_t i = 0; i < count; i++)
    {
        dj_motor_object_t *m = dj_motor_get_by_index(i);
        uint8_t  d[8];
        uint16_t ecd = (uint16_t)(0x1000U + (uint16_t)i * 0x0100U);   /* 0x1000,0x1100,... */
        int16_t  spd = (int16_t)(100 + (int16_t)i * 10);              /* 100,110,...      */
        int16_t  cur = (int16_t)(-(200 + (int16_t)i * 5));            /* -200,-205,...    */
        uint8_t  tmp = (uint8_t)(30 + i);                             /* 30,31,...        */
        uint32_t base;

        d[0] = (uint8_t)(ecd >> 8);
        d[1] = (uint8_t)(ecd & 0xFFU);
        d[2] = (uint8_t)((uint16_t)spd >> 8);
        d[3] = (uint8_t)((uint16_t)spd & 0xFFU);
        d[4] = (uint8_t)((uint16_t)cur >> 8);
        d[5] = (uint8_t)((uint16_t)cur & 0xFFU);
        d[6] = tmp;
        d[7] = 0;

        base = (m->type == DJ_MOTOR_GM6020) ? 0x204U : 0x200U;
        (void)dj_motor_send_std(m->bus, base + m->id, d);
    }
}

#endif /* ENABLE_MOTOR_BENCH && ENABLE_MOTOR_LOOPBACK */

/* ------------------------------ 任务入口 ------------------------------ */

/**
 * @brief MotorTask 入口
 * @note  1kHz：发帧、维护在线状态、低频诊断
 */
void StartMotorTask(void const *argument)
{
    uint32_t last_print_ms = 0;

    (void)argument;

    /* 1. 配过滤器 / 启动三条总线 / 开 RX 中断 */
    dj_motor_can_init();

#if ENABLE_MOTOR_CAN_SCAN
    /* 1.5 开机自检：监听 / 内部回环 / 波特率扫描，结果看 g_can_selftest */
    can1_selftest_run();
#endif

#if ENABLE_MOTOR_CAN_TX_PROBE
    can_tx_probe_init();
#endif

#if ENABLE_MOTOR_LOOPBACK
    /* 台架无第二节点：切内部回环，自己应答自己 */
    dj_motor_set_loopback(true);
#endif

#if ENABLE_MOTOR_BENCH
    /* 2. 台架模式：注册假电机 */
    motor_bench_register();
#endif

    /* 1.5s 后打印一次注册表，注册冲突会表现为电机数量偏少 */
    s_map_dump_at = HAL_GetTick() + 1500U;

    for (;;)
    {
        /* 电机注册表一次性 dump */
        if ((s_map_dump_at != 0U) && ((int32_t)(HAL_GetTick() - s_map_dump_at) >= 0))
        {
            uint8_t n = dj_motor_get_count();
            uint8_t i;
            char    line[96];
            int     pos;

            s_map_dump_at = 0U;
            log_printf("[MAP] registered=%u\r\n", n);

            pos = snprintf(line, sizeof(line), "[MAP] ");
            for (i = 0; (i < n) && (i < 16U); i++)
            {
                dj_motor_object_t *m = dj_motor_get_by_index(i);

                pos += snprintf(&line[pos], sizeof(line) - (uint32_t)pos,
                                "can%u/id%u/type%d ", m->bus, m->id, (int)m->type);
                if ((pos > 60) || (i + 1U == n))
                {
                    log_printf("%s\r\n", line);
                    pos = snprintf(line, sizeof(line), "[MAP] ");
                }
            }
        }

#if ENABLE_MOTOR_BENCH
        s_bench_tick++;
#endif
#if ENABLE_MOTOR_MINIMAL_TX
        minimal_tx_step();
#elif ENABLE_MOTOR_BUS_ID_PROBE
        bus_id_probe_step();
#elif ENABLE_MOTOR_CAN_TX_PROBE
        can_tx_probe_step();
#elif ENABLE_MOTOR_TX_DISABLE
        /* 只收不发：什么都不发 */
#else
        /* 计算并发送所有电机的控制帧 */
        dj_motor_send_all();
#endif

#if ENABLE_MOTOR_BENCH && ENABLE_MOTOR_LOOPBACK
        /* 10Hz 回灌伪反馈帧：验证 RX FIFO → 中断 → 反查 → 解码 */
        if ((s_bench_tick % 100U) == 0U)
        {
            bench_inject_feedback();
        }
#endif

        /* 维护在线状态 */
        dj_motor_update_online(HAL_GetTick(), DJ_MOTOR_OFFLINE_MS);

        /* 刷新诊断信息，供 watch 窗口查看 */
        g_motor_diag.motor_count = dj_motor_get_count();
        g_motor_diag.tick_ms     = HAL_GetTick();
        g_motor_diag.tx_count    = dj_motor_get_tx_count();
        g_motor_diag.tx_error    = dj_motor_get_tx_error();
        g_motor_diag.rx_count[0] = dj_motor_get_rx_count(1);
        g_motor_diag.rx_count[1] = dj_motor_get_rx_count(2);
        g_motor_diag.rx_count[2] = dj_motor_get_rx_count(3);
        g_motor_diag.busoff     = dj_motor_get_busoff_count();
        g_motor_diag.dead_mask  = dj_motor_get_dead_mask();
        g_motor_diag.rx_any[0]  = dj_motor_get_rx_any(1);
        g_motor_diag.rx_any[1]  = dj_motor_get_rx_any(2);
        g_motor_diag.rx_any[2]  = dj_motor_get_rx_any(3);
        g_motor_diag.last_id[0] = dj_motor_get_last_id(1);
        g_motor_diag.last_id[1] = dj_motor_get_last_id(2);
        g_motor_diag.last_id[2] = dj_motor_get_last_id(3);

#if ENABLE_MOTOR_CAN1_MONITOR
        /* CAN1 监视：接上电调后，rx 应该立刻从 0 变成每秒约 1000（4 个 3508 各 1kHz） */
        if ((g_motor_diag.rx_any[0] != 0U) && (s_can1_first_rx == 0U))
        {
            s_can1_first_rx = 1U;
            log_printf("[CAN1] first frame! id=0x%03X  → 接收链路通\r\n",
                       g_motor_diag.last_id[0]);
        }
        {
            uint32_t now_ms2 = HAL_GetTick();
            if ((now_ms2 - s_can1_last_print) >= 1000U)
            {
                uint32_t ecr = hfdcan1.Instance->ECR;
                uint32_t psr = hfdcan1.Instance->PSR;

                s_can1_last_print = now_ms2;
                log_printf("[CAN1] tx=%lu err=%lu rx=%lu any=%lu id=0x%03X "
                           "TEC=%lu LEC=%lu dead=%u\r\n",
                           g_motor_diag.tx_count, g_motor_diag.tx_error,
                           g_motor_diag.rx_count[0], g_motor_diag.rx_any[0],
                           g_motor_diag.last_id[0],
                           ecr & 0xFFU, psr & 0x7U,
                           (unsigned)(g_motor_diag.dead_mask & 0x1U));
            }
        }
#endif

        {
            uint16_t mask = 0;
            uint8_t count = dj_motor_get_count();

            for (uint8_t i = 0; (i < count) && (i < 16U); i++)
            {
                if (dj_motor_get_by_index(i)->measure.online)
                {
                    mask |= (uint16_t)(1U << i);
                }
            }
            g_motor_diag.online_mask = mask;
        }

#if ENABLE_DIAG_MOTOR
        /* 1Hz 打印，调试期用；上车前把 ENABLE_DIAG_MOTOR 置 0 */
        if ((uint32_t)(HAL_GetTick() - last_print_ms) >= 1000U)
        {
            last_print_ms = HAL_GetTick();
            log_printf("[MTR] n=%u tx=%lu err=%lu bo=%lu dead=%u rx=%lu/%lu/%lu onl=0x%03X",
                   g_motor_diag.motor_count,
                   (unsigned long)g_motor_diag.tx_count,
                   (unsigned long)g_motor_diag.tx_error,
                   (unsigned long)g_motor_diag.busoff,
                   g_motor_diag.dead_mask,
                   (unsigned long)g_motor_diag.rx_count[0],
                   (unsigned long)g_motor_diag.rx_count[1],
                   (unsigned long)g_motor_diag.rx_count[2],
                   g_motor_diag.online_mask);

            /* 抽头打印 0 号电机的解码结果，用来核对字节序/符号位有没有解错 */
            if (g_motor_diag.motor_count > 0U)
            {
                dj_motor_object_t *m0 = dj_motor_get_by_index(0);

                log_printf(" m0=%u/%d/%u", m0->measure.ecd,
                       m0->measure.speed_rpm,
                       m0->measure.temperature);
            }
            log_printf(" any=%lu/%lu/%lu lid=%03lX/%03lX/%03lX\r\n",
                       (unsigned long)g_motor_diag.rx_any[0],
                       (unsigned long)g_motor_diag.rx_any[1],
                       (unsigned long)g_motor_diag.rx_any[2],
                       (unsigned long)g_motor_diag.last_id[0],
                       (unsigned long)g_motor_diag.last_id[1],
                       (unsigned long)g_motor_diag.last_id[2]);
        }
#endif

        osDelay(1);
    }
}
