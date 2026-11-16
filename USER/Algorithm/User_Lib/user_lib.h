/**
  ******************************************************************************
  * @file    user_lib.h
  * @brief   通用数学宏和限幅函数
  ******************************************************************************
  */

#ifndef USER_LIB_H
#define USER_LIB_H

#include <stdint.h>

#ifndef PI
#define PI                      3.1415926535f
#endif

#define RPM_2_ANGLE_PER_SEC     6.0f            /* rpm -> °/s */
#define RPM_2_RAD_PER_SEC       0.104719755f    /* rpm -> rad/s */

#define VAL_LIMIT(val, min, max)            \
    do                                      \
    {                                       \
        if ((val) <= (min))                 \
        {                                   \
            (val) = (min);                  \
        }                                   \
        else if ((val) >= (max))            \
        {                                   \
            (val) = (max);                  \
        }                                   \
    } while (0)

static inline float float_constrain(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

#endif /* USER_LIB_H */
