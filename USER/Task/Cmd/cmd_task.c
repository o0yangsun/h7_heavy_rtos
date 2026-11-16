/**
  ******************************************************************************
  * @file    cmd_task.c
  * @brief   指令线程实现，详见 cmd_task.h
  ******************************************************************************
  */

#include "cmd_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "robot.h"
#include "robot_config.h"
#include "rc_dbus.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile cmd_chassis_t g_cmd_chassis;
volatile cmd_arm_t     g_cmd_arm;
volatile cmd_diag_t    g_cmd_diag;

static task_stat_t    s_stat;
static rc_dbus_obj_t  s_rc;             /* 最新一帧遥控输入（已居中） */
static uint32_t       s_rc_tick;        /* 最近一帧时基 */
static uint32_t       s_last_heartbeat; /* 已处理的遥控帧计数 */
static cmd_mode_e     s_mode = CMD_MODE_RELAX;

/* 斜坡当前值 */
static float s_vx, s_vy, s_vw;

static uint16_t s_key = 0U;         /* PC 键鼠数据 */
static int16_t  s_mx, s_my, s_mz;

/* ------------------------------- 内部函数 -------------------------------- */

static float ramp_to(float cur, float target, float step)
{
    float d = target - cur;

    if (d > step)  { return cur + step; }
    if (d < -step) { return cur - step; }
    return target;
}

/**
 * @brief 按配置下标取遥控通道
 */
static int16_t cmd_rc_get_ch(uint8_t ch)
{
    switch (ch)
    {
        case 0U: return s_rc.ch1;
        case 1U: return s_rc.ch2;
        case 2U: return s_rc.ch3;
        case 3U: return s_rc.ch4;
        default: return 0;
    }
}

/**
 * @brief 遥控通道到底盘速度（日本手）
 *
 *   vx = ch2 * 0.5  / 660 * 5000   (mm/s)
 *   vy = -ch1 * 0.8 / 660 * 5000   (mm/s)
 *   vw = -ch3 * 0.85 / 660 * 8     (rad/s)
 *   小陀螺模式下 vw 固定为 CMD_SPIN_VW
 */
static void cmd_rc_apply(void)
{
    float tgt_vx = 0.0f;
    float tgt_vy = 0.0f;
    float tgt_vw = 0.0f;

    if ((s_mode == CMD_MODE_RC) || (s_mode == CMD_MODE_SPIN))
    {
        tgt_vx = (float)cmd_rc_get_ch(CMD_RC_CH_VX) * CMD_RC_SIGN_VX
                 * CMD_RC_RATIO_X / RC_MAX_VALUE * CMD_MAX_VX;
        tgt_vy = (float)cmd_rc_get_ch(CMD_RC_CH_VY) * CMD_RC_SIGN_VY
                 * CMD_RC_RATIO_Y / RC_MAX_VALUE * CMD_MAX_VY;
    }

    if (s_mode == CMD_MODE_SPIN)
    {
        tgt_vw = CMD_SPIN_VW;               /* 固定自转速度 */
    }
    else if (s_mode == CMD_MODE_RC)
    {
        tgt_vw = (float)cmd_rc_get_ch(CMD_RC_CH_VW) * CMD_RC_SIGN_VW
                 * CMD_RC_RATIO_R / RC_MAX_VALUE * CMD_MAX_VR;
    }

    tgt_vx = float_constrain(tgt_vx, -CMD_MAX_VX, CMD_MAX_VX);
    tgt_vy = float_constrain(tgt_vy, -CMD_MAX_VY, CMD_MAX_VY);
    tgt_vw = float_constrain(tgt_vw, -CMD_MAX_VR, CMD_MAX_VR);

#if ENABLE_RC_RAMP
    s_vx = ramp_to(s_vx, tgt_vx, CMD_RC_ACC_XY);
    s_vy = ramp_to(s_vy, tgt_vy, CMD_RC_ACC_XY);
    s_vw = ramp_to(s_vw, tgt_vw, CMD_RC_ACC_W);
#else
    /* 直通，不做斜坡 */
    s_vx = tgt_vx;
    s_vy = tgt_vy;
    s_vw = tgt_vw;
#endif

    g_cmd_chassis.vx     = s_vx;
    g_cmd_chassis.vy     = s_vy;
    g_cmd_chassis.vw     = s_vw;
    g_cmd_chassis.mode   = s_mode;
    g_cmd_chassis.online = ((s_mode == CMD_MODE_RC) || (s_mode == CMD_MODE_SPIN)) ? 1U : 0U;
}

/**
 * @brief 拨杆到机械臂状态
 * @note  DT7 只有 sw1/sw2，机械臂使能固定由 sw1 上档控制
 */
static void cmd_arm_apply(uint8_t rc_online)
{
    g_cmd_arm.enabled = ((rc_online != 0U) && (s_rc.sw1 == RC_UP)) ? 1U : 0U;
    g_cmd_arm.online  = rc_online;
    g_cmd_arm.src     = CMD_ARM_SRC_USER;
    g_cmd_arm.gripper = 0.0f;
}

/**
 * @brief 底盘模式判定
 *
 * sw2 上档 = 失能，中/下档 = 开环行驶；sw1 下档 = 小陀螺。
 */
static cmd_mode_e cmd_mode_from_sw(uint8_t rc_online)
{
    if (rc_online == 0U)
    {
        return CMD_MODE_RELAX;
    }

    if (s_rc.sw2 == RC_UP)
    {
        return CMD_MODE_RELAX;
    }
    if (s_rc.sw1 == RC_DN)
    {
        return CMD_MODE_SPIN;
    }
    return CMD_MODE_RC;
}

/* --------------------------------- 接口 ---------------------------------- */

void cmd_set_keyboard(uint16_t key_code, int16_t mouse_x, int16_t mouse_y, int16_t mouse_z)
{
    s_key = key_code;
    s_mx  = mouse_x;
    s_my  = mouse_y;
    s_mz  = mouse_z;
}

void cmd_get_chassis(cmd_chassis_t *out)
{
    if (out != 0)
    {
        memcpy(out, (const void *)&g_cmd_chassis, sizeof(cmd_chassis_t));
    }
}

void cmd_get_arm(cmd_arm_t *out)
{
    if (out != 0)
    {
        memcpy(out, (const void *)&g_cmd_arm, sizeof(cmd_arm_t));
    }
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief CmdTask 入口
 */
void StartCmdTask(void const *argument)
{
    uint32_t last_print = 0U;
    uint8_t  last_online = 0U;

    (void)argument;

    memset(&s_rc, 0, sizeof(s_rc));
    memset((void *)&g_cmd_chassis, 0, sizeof(g_cmd_chassis));
    memset((void *)&g_cmd_arm, 0, sizeof(g_cmd_arm));
    memset((void *)&g_cmd_diag, 0, sizeof(g_cmd_diag));
    task_stat_init(&s_stat, CMD_PERIOD_MS);

    s_rc.sw1 = RC_UP;
    s_rc.sw2 = RC_UP;
    s_rc_tick = 0U;
    s_last_heartbeat = 0U;
    s_mode = CMD_MODE_RELAX;
    s_vx = s_vy = s_vw = 0.0f;

    log_printf("[CMD] up: sw2 UP=RELAX, sw1 DN=SPIN\r\n");

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t heartbeat;
        uint8_t  rc_online;
        cmd_mode_e new_mode;

        task_stat_begin(&s_stat, now);

        /* 有新帧时复制一份，临界区避免读到半帧 */
        heartbeat = dbus_heartbeat;
        if (heartbeat != s_last_heartbeat)
        {
            taskENTER_CRITICAL();
            s_rc = dbus_data_fdb;
            taskEXIT_CRITICAL();

            s_last_heartbeat = heartbeat;
            s_rc_tick = now;

            g_cmd_diag.rc_frames = heartbeat;
            g_cmd_diag.sw1 = s_rc.sw1;
            g_cmd_diag.sw2 = s_rc.sw2;
        }

        /* 遥控在线判定 */
        rc_online = ((uint32_t)(now - s_rc_tick) < CMD_RC_TIMEOUT_MS) ? 1U : 0U;

        if ((rc_online == 0U) && (last_online != 0U))
        {
            g_cmd_diag.rc_lost++;
        }
        last_online = rc_online;

        /* 档位状态机 */
        new_mode = cmd_mode_from_sw(rc_online);
        if (new_mode != s_mode)
        {
            s_mode = new_mode;
            g_cmd_diag.mode_change_cnt++;
            log_printf("[CMD] mode -> %d\r\n", (int)s_mode);
        }

        /* 生成指令 */
        cmd_rc_apply();
        cmd_arm_apply(rc_online);

        /* 诊断 */
        g_cmd_diag.loop_cnt = s_stat.loop_cnt;
        g_cmd_diag.mode     = s_mode;

#if ENABLE_DIAG_CMD
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[CMD] md=%d on=%u vx=%.0f vy=%.0f vw=%.2f sw=%u/%u rc=%lu\r\n",
                       (int)s_mode, (unsigned)rc_online,
                       (double)g_cmd_chassis.vx, (double)g_cmd_chassis.vy,
                       (double)g_cmd_chassis.vw,
                       (unsigned)g_cmd_diag.sw1, (unsigned)g_cmd_diag.sw2,
                       g_cmd_diag.rc_frames);
        }
#endif

        (void)s_key; (void)s_mx; (void)s_my; (void)s_mz;

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
