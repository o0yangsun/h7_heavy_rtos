/**
  ******************************************************************************
  * @file    imu_heat.c
  * @brief   IMU 加热丝温控实现，详见 imu_heat.h
  ******************************************************************************
  */

#include "imu_heat.h"
#include "main.h"
#include "tim.h"
#include "robot.h"

static uint8_t s_enable;
static float   s_integral;
static float   s_duty;

/** 占空比换算成比较值，h 的 ARR 为 65535 */
static void imu_heat_apply(float duty_pct)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = (uint32_t)(duty_pct * (float)arr / 100.0f);

    if (ccr > arr)
    {
        ccr = arr;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}

void imu_heat_init(void)
{
    s_enable   = 1U;
    s_integral = 0.0f;
    s_duty     = 0.0f;

    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    imu_heat_apply(0.0f);
}

void imu_heat_set_enable(uint8_t enable)
{
    s_enable = (enable != 0U) ? 1U : 0U;

    if (s_enable == 0U)
    {
        s_integral = 0.0f;
        s_duty     = 0.0f;
        imu_heat_apply(0.0f);
    }
}

void imu_heat_update(float temp_c)
{
    float err;

    if ((s_enable == 0U) || (temp_c < IMU_HEAT_VALID_MIN_C))
    {
        s_integral = 0.0f;
        s_duty     = 0.0f;
        imu_heat_apply(0.0f);
        return;
    }

    err = IMU_HEAT_TARGET_C - temp_c;

    if (err < -IMU_HEAT_HYST_C)
    {
        /* 超过目标温度：停止加热并清积分，防止冲温 */
        s_integral = 0.0f;
        s_duty     = 0.0f;
    }
    else if (err > IMU_HEAT_HYST_C)
    {
        s_integral += IMU_HEAT_KI * err * IMU_HEAT_DT_S;

        if (s_integral < 0.0f)
        {
            s_integral = 0.0f;
        }
        else if (s_integral > IMU_HEAT_DUTY_MAX)
        {
            s_integral = IMU_HEAT_DUTY_MAX;
        }

        s_duty = IMU_HEAT_KP * err + s_integral;
    }
    else
    {
        /* 滞环内维持当前输出，靠积分缓慢贴近目标 */
        s_duty = s_integral;
    }

    if (s_duty > IMU_HEAT_DUTY_MAX)
    {
        s_duty = IMU_HEAT_DUTY_MAX;
    }
    else if (s_duty < 0.0f)
    {
        s_duty = 0.0f;
    }

    imu_heat_apply(s_duty);
}

float imu_heat_get_duty(void)
{
    return s_duty;
}
