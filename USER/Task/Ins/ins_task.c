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

#include "buzzer.h"

#if ENABLE_IMU
#include "bmi088.h"
#include "imu_heat.h"
#include "dwt.h"
#include "quaternion_ekf.h"

#include <math.h>
#endif

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

/* ------------------------------ IMU 初始化 ------------------------------- */

#if ENABLE_IMU

static const float s_mount[3][3] = INS_MOUNT_MATRIX;

/**
 * @brief 用静止加速度估计初始 roll/pitch 并构造四元数
 * @note  采样 50 次，每次间隔 2 ms，只在初始化阶段调用
 */
static void ins_init_quaternion(float q[4])
{
    bmi088_sample_t s;
    float   ax = 0.0f;
    float   ay = 0.0f;
    float   az = 0.0f;
    float   n = 0.0f;
    float   roll;
    float   pitch;
    float   norm;
    uint8_t i;

    for (i = 0U; i < 50U; i++)
    {
        if (bmi088_read(&s) != 0U)
        {
            ax += s.accel[0];
            ay += s.accel[1];
            az += s.accel[2];
            n += 1.0f;
        }

        osDelay(2U);
    }

    norm = sqrtf(ax * ax + ay * ay + az * az);

    if ((n < 1.0f) || (norm < 1e-3f))
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    ax /= n;
    ay /= n;
    az /= n;

    roll  = atan2f(ay, az);
    pitch = asinf(-ax / norm);

    q[0] = cosf(roll * 0.5f) * cosf(pitch * 0.5f);
    q[1] = sinf(roll * 0.5f) * cosf(pitch * 0.5f);
    q[2] = cosf(roll * 0.5f) * sinf(pitch * 0.5f);
    q[3] = -sinf(roll * 0.5f) * sinf(pitch * 0.5f);
}

static void ins_imu_init(void)
{
    uint8_t st = 0xFFU;
    uint8_t retry;

    dwt_init();

    for (retry = 0U; retry < 3U; retry++)
    {
        st = bmi088_init();

        if (st == BMI088_OK)
        {
            break;
        }

        log_printf("[INS] bmi088 init 0x%02X\r\n", (unsigned)st);
        osDelay(50U);
    }

    if (st != BMI088_OK)
    {
        g_ins_diag.imu_ok = 0U;
        log_printf("[INS] bmi088 NOT ready\r\n");
        return;
    }

    {
        float q[4];

        ins_init_quaternion(q);
        quat_ekf_init(q[0], q[1], q[2], q[3],
                      INS_EKF_Q1, INS_EKF_Q2, INS_EKF_R,
                      INS_EKF_LAMBDA, INS_EKF_ACC_LPF);
    }

    imu_heat_init();

    g_ins_diag.imu_ok = 1U;
    log_printf("[INS] bmi088 ok\r\n");
}

#endif /* ENABLE_IMU */

/* ------------------------------- 线程入口 -------------------------------- */

/**
 * @brief InsTask 入口
 */
void StartInsTask(void const *argument)
{
    uint32_t last_print = 0U;
    uint32_t last_heat = 0U;
    float    imu_temp = 0.0f;

    (void)argument;

    /* CubeMX 将 USB 设备初始化放在本线程的弱定义中，必须保留 */
    MX_USB_DEVICE_Init();

    buzzer_init();
    buzzer_play_startup();

    memset((void *)&g_ins, 0, sizeof(g_ins));
    memset((void *)&g_ins_diag, 0, sizeof(g_ins_diag));
    task_stat_init(&s_stat, INS_PERIOD_MS);

    g_ins_diag.usb_inited = 1U;

#if ENABLE_IMU
    ins_imu_init();
#else
    log_printf("[INS] 1kHz up, imu disabled\r\n");
#endif

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        task_stat_begin(&s_stat, now);
        buzzer_update();

#if ENABLE_IMU
        if (g_ins_diag.imu_ok != 0U)
        {
            bmi088_sample_t raw;

            if (bmi088_read(&raw) != 0U)
            {
                float   gyro[3];
                float   accel[3];
                float   dt;
                uint8_t i;

                imu_temp = raw.temp_c;

                for (i = 0U; i < 3U; i++)
                {
                    gyro[i]  = s_mount[i][0] * raw.gyro[0]
                             + s_mount[i][1] * raw.gyro[1]
                             + s_mount[i][2] * raw.gyro[2];
                    accel[i] = s_mount[i][0] * raw.accel[0]
                             + s_mount[i][1] * raw.accel[1]
                             + s_mount[i][2] * raw.accel[2];
                }

                dt = dwt_get_dt_s();

                if (dt < 0.0005f)
                {
                    dt = 0.0005f;
                }
                else if (dt > 0.005f)
                {
                    dt = 0.005f;
                }

                quat_ekf_update(gyro, accel, dt);

                {
                    quat_ekf_out_t est;

                    quat_ekf_get(&est);

                    g_ins.roll      = est.roll;
                    g_ins.pitch     = est.pitch;
                    g_ins.yaw       = est.yaw;
                    g_ins.yaw_total = est.yaw_total;
                    g_ins.gyro[0]   = gyro[0];
                    g_ins.gyro[1]   = gyro[1];
                    g_ins.gyro[2]   = gyro[2];
                    g_ins.accel[0]  = accel[0];
                    g_ins.accel[1]  = accel[1];
                    g_ins.accel[2]  = accel[2];
                    g_ins.valid     = est.converged;
                }
            }
            else
            {
                g_ins_diag.err_cnt++;
            }
        }
#else
        g_ins.valid = 0U;
#endif

        g_ins_diag.loop_cnt = s_stat.loop_cnt;
        g_ins_diag.dt_ms    = s_stat.dt_ms;

#if ENABLE_DIAG_INS
        if ((now - last_print) >= 1000U)
        {
            last_print = now;
            log_printf("[INS] %luloop/s yaw=%.2f roll=%.2f pitch=%.2f ok=%u/%u\r\n",
                       s_stat.loop_cnt, (double)g_ins.yaw, (double)g_ins.roll,
                       (double)g_ins.pitch, (unsigned)g_ins_diag.imu_ok,
                       (unsigned)g_ins.valid);
        }
#endif

#if ENABLE_IMU
        if ((now - last_heat) >= IMU_HEAT_PERIOD_MS)
        {
            last_heat = now;
            imu_heat_update(imu_temp);
        }
#endif

        task_stat_end(&s_stat, HAL_GetTick());
        osDelay(1);
    }
}
