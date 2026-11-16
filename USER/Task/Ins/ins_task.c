/**
  ******************************************************************************
  * @file    ins_task.c
  * @brief   姿态解算线程实现，详见 ins_task.h
  ******************************************************************************
  */

#include "ins_task.h"
#include "task_common.h"
#include "log_uart.h"
#include "usb_device.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

/* ------------------------------- 全局变量 -------------------------------- */

volatile ins_attitude_t g_ins;
volatile ins_diag_t     g_ins_diag;

static task_stat_t s_stat;

/* --------------------------------- 接口 ---------------------------------- */

void ins_get_attitude(ins_attitude_t *out)
{
    if (out == 0)
    {
        return;
    }
    memcpy(out, (const void *)&g_ins, sizeof(ins_attitude_t));
}

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief InsTask 入口
 */
void StartInsTask(void const *argument)
{
    uint32_t last_print = 0U;

    (void)argument;

    /* CubeMX 将 USB 设备初始化放在本线程的弱定义中，必须保留 */
    MX_USB_DEVICE_Init();

    memset((void *)&g_ins, 0, sizeof(g_ins));
    memset((void *)&g_ins_diag, 0, sizeof(g_ins_diag));
    task_stat_init(&s_stat, INS_PERIOD_MS);

    g_ins_diag.usb_inited = 1U;
    g_ins_diag.imu_ok     = 0U;

#if ENABLE_IMU
    /* TODO 移植顺序：
     *   1. BMI088 初始化与采样（SPI + 片选 + 中断，加热丝 PB1 = TIM3_CH4）
     *   2. 四元数 EKF 更新
     *   3. 加速度低通与坐标系变换
     *   4. 温控 PID
     */
#else
    log_printf("[INS] 1kHz up, imu NOT ported (ENABLE_IMU=0)\r\n");
#endif

    for (;;)
    {
        task_stat_begin(&s_stat, HAL_GetTick());

        /* 本周期业务 */
#if ENABLE_IMU
        /* 1) 采样 */
        /* BMI088_Read(&BMI088); */
        /* 2) 零偏/温漂修正 + EKF */
        /* IMU_QuaternionEKF_Update(...); */
        /* 3) 姿态落库 */
        /* g_ins.yaw = ...; g_ins.valid = 1U; */
#else
        /* 未移植：姿态保持 0 */
        g_ins.valid = 0U;
#endif

        /* 诊断信息同步 */
        g_ins_diag.loop_cnt = s_stat.loop_cnt;
        g_ins_diag.dt_ms    = s_stat.dt_ms;

#if ENABLE_DIAG_INS
        {
            uint32_t now = HAL_GetTick();
            if ((now - last_print) >= 1000U)
            {
                last_print = now;
                log_printf("[INS] %luloop/s yaw=%.2f valid=%u\r\n",
                           s_stat.loop_cnt, (double)g_ins.yaw, (unsigned)g_ins.valid);
            }
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
