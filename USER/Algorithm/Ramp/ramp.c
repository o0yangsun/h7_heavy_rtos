/**
  ******************************************************************************
  * @file    ramp.c
  * @brief   斜坡发生器实现，详见 ramp.h
  ******************************************************************************
  */

#include "ramp.h"

#include <stddef.h>

void ramp_init(ramp_t *r, float rate_per_s, float dt_s)
{
    if (r == NULL)
    {
        return;
    }

    r->y = 0.0f;
    r->step = 0.0f;

    if ((rate_per_s <= 0.0f) || (dt_s <= 0.0f))
    {
        r->step = 0.0f;
        return;
    }

    r->step = rate_per_s * dt_s;
}

float ramp_update(ramp_t *r, float target)
{
    float d;

    if (r == NULL)
    {
        return target;
    }

    if (r->step <= 0.0f)
    {
        r->y = target;
        return r->y;
    }

    d = target - r->y;

    if (d > r->step)
    {
        r->y += r->step;
    }
    else if (d < -r->step)
    {
        r->y -= r->step;
    }
    else
    {
        r->y = target;
    }

    return r->y;
}

void ramp_set(ramp_t *r, float value)
{
    if (r != NULL)
    {
        r->y = value;
    }
}
