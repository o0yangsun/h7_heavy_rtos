/**
  ******************************************************************************
  * @file    pid.h
  * @brief   位置式 PID，支持梯形积分、积分限幅、微分先行、输出滤波
  *
  * 实例由静态池分配，最多 PID_NUM_MAX 个。
  * 控制周期默认 PID_DEFAULT_DT，可用 pid_set_dt() 修改。
  ******************************************************************************
  */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

/** 最多能注册多少个 PID 实例 */
#define PID_NUM_MAX             24U

/** 默认控制周期（秒），MotorTask / GimbalTask 都是 1kHz */
#define PID_DEFAULT_DT          0.001f

#ifndef pid_abs
#define pid_abs(x)              (((x) > 0.0f) ? (x) : -(x))
#endif

/** PID 优化环节，按位或组合到 Improve 里 */
typedef enum
{
    PID_IMPROVE_NONE                = 0x00,
    PID_Integral_Limit              = 0x01,     /**< 积分限幅 */
    PID_Derivative_On_Measurement   = 0x02,     /**< 微分先行（只对反馈微分，避免设变跳变） */
    PID_Trapezoid_Intergral         = 0x04,     /**< 梯形积分 */
    PID_Proportional_On_Measurement = 0x08,     /**< 比例项只作用在反馈上（本版未实现） */
    PID_OutputFilter                = 0x10,     /**< 输出一阶低通 */
    PID_ChangingIntegrationRate     = 0x20,     /**< 变速积分（误差大时削弱积分） */
    PID_DerivativeFilter            = 0x40,     /**< 微分一阶低通 */
    PID_ErrorHandle                 = 0x80,     /**< 堵转检测 */
} pid_improve_e;

/** 堵转检测结果 */
typedef enum
{
    PID_ERROR_NONE                  = 0x00,
    PID_MOTOR_BLOCKED_ERROR         = 0x01,
} pid_error_e;

typedef struct
{
    uint32_t        error_count;
    pid_error_e     error_type;
} pid_error_handler_t;

/** PID 实例 */
typedef struct
{
    /* --- 配置 --- */
    float   Kp;
    float   Ki;
    float   Kd;
    float   MaxOut;                 /**< 输出限幅（绝对值） */
    float   DeadBand;               /**< 死区：误差绝对值小于它时输出 0 */
    uint8_t Improve;                /**< pid_improve_e 的按位或 */
    float   IntegralLimit;          /**< 积分项限幅 */
    float   CoefA;                  /**< 变速积分：A */
    float   CoefB;                  /**< 变速积分：B */
    float   Output_LPF_RC;          /**< 输出滤波时间常数（秒） */
    float   Derivative_LPF_RC;      /**< 微分滤波时间常数（秒） */

    /* --- 运行状态 --- */
    float   Measure;
    float   Last_Measure;
    float   Ref;
    float   Err;
    float   Last_Err;
    float   Pout;
    float   Iout;
    float   Dout;
    float   ITerm;
    float   Output;
    float   Last_Output;
    float   Last_Dout;
    float   dt;
    pid_error_handler_t ERRORHandler;
} pid_obj_t;

/** 注册配置 */
typedef struct
{
    float   Kp;
    float   Ki;
    float   Kd;
    float   MaxOut;
    float   DeadBand;
    uint8_t Improve;
    float   IntegralLimit;
    float   CoefA;
    float   CoefB;
    float   Output_LPF_RC;
    float   Derivative_LPF_RC;
} pid_config_t;

/** 只填常用项，其余字段为 0 */
#define INIT_PID_CONFIG(kp_val, ki_val, kd_val, ilimit_val, maxout_val, improve_val) \
    {                                                                                 \
        .Kp = (kp_val), .Ki = (ki_val), .Kd = (kd_val),                               \
        .MaxOut = (maxout_val), .DeadBand = 0.0f,                                     \
        .Improve = (uint8_t)(improve_val),                                            \
        .IntegralLimit = (ilimit_val),                                                \
        .CoefA = 0.0f, .CoefB = 0.0f,                                                 \
        .Output_LPF_RC = 0.0f, .Derivative_LPF_RC = 0.0f,                             \
    }

/**
 * @brief 注册一个 PID 实例
 * @param cfg 配置
 * @return 实例指针；静态池满时返回 NULL
 */
pid_obj_t *pid_register(const pid_config_t *cfg);

/**
 * @brief 计算一次 PID
 * @param pid     PID 实例
 * @param measure 反馈值
 * @param ref     目标值
 * @return 输出（已限幅）
 */
float pid_calculate(pid_obj_t *pid, float measure, float ref);

/** 清空历史状态（换模式/重新使能时调用） */
void pid_clear(pid_obj_t *pid);

/** 设置控制周期（秒），默认 PID_DEFAULT_DT */
void pid_set_dt(pid_obj_t *pid, float dt);

/** 在线改增益（调参用，不清状态） */
void pid_set_gains(pid_obj_t *pid, float kp, float ki, float kd);

/** 输出是否已饱和 */
uint8_t pid_is_saturated(const pid_obj_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
