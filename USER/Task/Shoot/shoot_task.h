/**
  ******************************************************************************
  * @file    shoot_task.h
  * @brief   发射机构线程：三个摩擦轮转速闭环与拨弹电机位置闭环
  ******************************************************************************
  */

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 摩擦轮数量 */
#define SHOOT_FRIC_NUM          3U

/** 拨弹工作模式 */
typedef enum
{
    TRIGGER_MODE_STOP = 0,      /**< 保持当前位置，出力托住 */
    TRIGGER_MODE_SINGLE,        /**< 单发：目标角度前进一步 */
    TRIGGER_MODE_AUTO,          /**< 连发：定速正转 */
    TRIGGER_MODE_REVERSE,       /**< 反转：定速倒转，用于清卡弹 */
} shoot_trigger_mode_e;

/** 运行诊断信息 */
typedef struct
{
    float    ref_out_rpm[SHOOT_FRIC_NUM];   /**< 摩擦轮目标输出轴转速 */
    float    act_out_rpm[SHOOT_FRIC_NUM];   /**< 摩擦轮实测输出轴转速 */
    int16_t  out[SHOOT_FRIC_NUM];           /**< 摩擦轮电流输出 */
    uint8_t  online_mask;                   /**< bit0~bit2 对应三个摩擦轮 */

    float    trigger_angle;                 /**< 拨弹电机轴累计角度 */
    float    trigger_target;                /**< 拨弹目标角度 */
    int16_t  trigger_out;                   /**< 拨弹电流输出 */
    uint8_t  trigger_mode;                  /**< shoot_trigger_mode_e */
    uint8_t  trigger_online;

    uint32_t loop_cnt;
    uint8_t  enabled;
} shoot_diag_t;

extern volatile shoot_diag_t g_shoot_diag;

/* --------------------------------- 摩擦轮 --------------------------------- */

/**
 * @brief 设置摩擦轮目标转速
 * @param out_rpm 输出轴转速，正为发射方向，内部限幅到 ±FRIC_MAX_OUT_RPM
 * @note  台架模式下由 ShootTask 周期覆盖，外部设置不会保留
 */
void shoot_set_friction_rpm(float out_rpm);

/** 当前摩擦轮目标输出轴转速 */
float shoot_get_friction_rpm(void);

/**
 * @brief 使能或松开摩擦轮
 * @param enable 0 松开（输出强制 0），非 0 允许出力
 * @note  初始状态为松开，须显式调用才会转动
 */
void shoot_set_friction_enable(uint8_t enable);

/* ---------------------------------- 拨弹 ---------------------------------- */

/**
 * @brief 切换拨弹工作模式
 * @param mode 目标模式
 * @note  STOP 会以当前角度为目标，避免切换瞬间跳变
 */
void shoot_set_trigger_mode(shoot_trigger_mode_e mode);

/** 当前拨弹模式 */
shoot_trigger_mode_e shoot_get_trigger_mode(void);

/**
 * @brief 前进一步拨弹
 * @note  仅在 STOP 模式下生效，目标角度累加一个单发转角
 */
void shoot_trigger_once(void);

#ifdef __cplusplus
}
#endif

#endif /* __SHOOT_TASK_H__ */
