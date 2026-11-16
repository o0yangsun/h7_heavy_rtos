/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 实现，详见 pid.h
  ******************************************************************************
  */

#include "pid.h"
#include <string.h>

/* --------------------------------- 内部变量 -------------------------------- */

static pid_obj_t s_pid_pool[PID_NUM_MAX];
static uint8_t   s_pid_used = 0;

/* ------------------------------- 优化环节实现 ------------------------------ */

/** 梯形积分 */
static void f_trapezoid_integral(pid_obj_t *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) * 0.5f) * pid->dt;
}

/** 变速积分 */
static void f_changing_integration_rate(pid_obj_t *pid)
{
    if ((pid->Err * pid->Iout) > 0.0f)
    {
        if (pid_abs(pid->Err) <= pid->CoefB)
        {
            return;                                         /* 全速积分 */
        }
        if (pid_abs(pid->Err) <= (pid->CoefA + pid->CoefB))
        {
            pid->ITerm *= (pid->CoefA - pid_abs(pid->Err) + pid->CoefB) / pid->CoefA;
        }
        else
        {
            pid->ITerm = 0.0f;                              /* 误差太大，不积分 */
        }
    }
}

/** 积分限幅与抗积分饱和 */
static void f_integral_limit(pid_obj_t *pid)
{
    float temp_iout = pid->Iout + pid->ITerm;
    float temp_output = pid->Pout + pid->Iout + pid->Dout;

    if (pid_abs(temp_output) > pid->MaxOut)
    {
        if ((pid->Err * pid->Iout) > 0.0f)
        {
            pid->ITerm = 0.0f;                              /* 还在往饱和方向积，先停 */
        }
    }

    if (temp_iout > pid->IntegralLimit)
    {
        pid->ITerm = 0.0f;
        pid->Iout = pid->IntegralLimit;
    }
    else if (temp_iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0.0f;
        pid->Iout = -pid->IntegralLimit;
    }
}

/** 微分先行 */
static void f_derivative_on_measurement(pid_obj_t *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

/** 微分一阶低通 */
static void f_derivative_filter(pid_obj_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt)
              + pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

/** 输出一阶低通 */
static void f_output_filter(pid_obj_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt)
                + pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

/** 输出限幅 */
static void f_output_limit(pid_obj_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    else if (pid->Output < -pid->MaxOut)
    {
        pid->Output = -pid->MaxOut;
    }
}

/** 堵转检测 */
static void f_error_handle(pid_obj_t *pid)
{
    if ((pid_abs(pid->Output) < (pid->MaxOut * 0.001f)) || (pid_abs(pid->Ref) < 0.0001f))
    {
        return;
    }

    if ((pid_abs(pid->Ref - pid->Measure) / pid_abs(pid->Ref)) > 0.95f)
    {
        pid->ERRORHandler.error_count++;
    }
    else
    {
        pid->ERRORHandler.error_count = 0;
    }

    if (pid->ERRORHandler.error_count > 500U)
    {
        pid->ERRORHandler.error_type = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* --------------------------------- 外部接口 -------------------------------- */

pid_obj_t *pid_register(const pid_config_t *cfg)
{
    pid_obj_t *pid;

    if ((cfg == NULL) || (s_pid_used >= PID_NUM_MAX))
    {
        return NULL;
    }

    pid = &s_pid_pool[s_pid_used];
    s_pid_used++;

    memset(pid, 0, sizeof(*pid));

    pid->Kp       = cfg->Kp;
    pid->Ki       = cfg->Ki;
    pid->Kd       = cfg->Kd;
    pid->MaxOut   = cfg->MaxOut;
    pid->DeadBand = cfg->DeadBand;

    pid->Improve          = cfg->Improve;
    pid->IntegralLimit    = cfg->IntegralLimit;
    pid->CoefA            = cfg->CoefA;
    pid->CoefB            = cfg->CoefB;
    pid->Output_LPF_RC    = cfg->Output_LPF_RC;
    pid->Derivative_LPF_RC = cfg->Derivative_LPF_RC;

    pid->dt = PID_DEFAULT_DT;

    return pid;
}

float pid_calculate(pid_obj_t *pid, float measure, float ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if (pid->dt <= 0.0f)
    {
        pid->dt = PID_DEFAULT_DT;
    }

    if ((pid->Improve & PID_ErrorHandle) != 0U)
    {
        f_error_handle(pid);
    }

    pid->Measure = measure;
    pid->Ref     = ref;
    pid->Err     = pid->Ref - pid->Measure;

    if (pid_abs(pid->Err) > pid->DeadBand)
    {
        /* 位置式基础三项 */
        pid->Pout  = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        if ((pid->Improve & PID_Trapezoid_Intergral) != 0U)
        {
            f_trapezoid_integral(pid);
        }
        if ((pid->Improve & PID_ChangingIntegrationRate) != 0U)
        {
            f_changing_integration_rate(pid);
        }
        if ((pid->Improve & PID_Derivative_On_Measurement) != 0U)
        {
            f_derivative_on_measurement(pid);
        }
        if ((pid->Improve & PID_DerivativeFilter) != 0U)
        {
            f_derivative_filter(pid);
        }
        if ((pid->Improve & PID_Integral_Limit) != 0U)
        {
            f_integral_limit(pid);
        }

        pid->Iout  += pid->ITerm;
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        if ((pid->Improve & PID_OutputFilter) != 0U)
        {
            f_output_filter(pid);
        }

        f_output_limit(pid);
    }
    else
    {
        /* 死区内输出归零 */
        pid->Output = 0.0f;
        pid->ITerm  = 0.0f;
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output  = pid->Output;
    pid->Last_Dout    = pid->Dout;
    pid->Last_Err     = pid->Err;

    return pid->Output;
}

void pid_clear(pid_obj_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Measure      = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Ref          = 0.0f;
    pid->Err          = 0.0f;
    pid->Last_Err     = 0.0f;
    pid->Pout         = 0.0f;
    pid->Iout         = 0.0f;
    pid->Dout         = 0.0f;
    pid->ITerm        = 0.0f;
    pid->Output       = 0.0f;
    pid->Last_Output  = 0.0f;
    pid->Last_Dout    = 0.0f;

    pid->ERRORHandler.error_count = 0U;
    pid->ERRORHandler.error_type  = PID_ERROR_NONE;
}

void pid_set_dt(pid_obj_t *pid, float dt)
{
    if ((pid != NULL) && (dt > 0.0f))
    {
        pid->dt = dt;
    }
}

void pid_set_gains(pid_obj_t *pid, float kp, float ki, float kd)
{
    if (pid != NULL)
    {
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
    }
}

uint8_t pid_is_saturated(const pid_obj_t *pid)
{
    if (pid == NULL)
    {
        return 0U;
    }
    return (uint8_t)((pid_abs(pid->Output) >= (pid->MaxOut - 1e-3f)) ? 1U : 0U);
}
