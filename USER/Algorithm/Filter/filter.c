/**
  ******************************************************************************
  * @file    filter.c
  * @brief   滤波器实现，详见 filter.h
  ******************************************************************************
  */

#include "filter.h"

#include <stddef.h>

void lpf1_init(lpf1_t *f, float tau_s, float dt_s)
{
    if (f == NULL)
    {
        return;
    }

    f->y = 0.0f;
    f->ready = 0U;

    if ((tau_s <= 0.0f) || (dt_s <= 0.0f))
    {
        f->alpha = 1.0f;
        return;
    }

    f->alpha = dt_s / (tau_s + dt_s);

    if (f->alpha > 1.0f)
    {
        f->alpha = 1.0f;
    }
}

float lpf1_update(lpf1_t *f, float x)
{
    if (f == NULL)
    {
        return x;
    }

    if (f->ready == 0U)
    {
        f->y = x;
        f->ready = 1U;
        return f->y;
    }

    f->y += f->alpha * (x - f->y);

    return f->y;
}

void lpf1_reset(lpf1_t *f)
{
    if (f == NULL)
    {
        return;
    }

    f->y = 0.0f;
    f->ready = 0U;
}

void iir2_init(iir2_t *f, float b0, float b1, float b2, float a1, float a2)
{
    if (f == NULL)
    {
        return;
    }

    f->b0 = b0;
    f->b1 = b1;
    f->b2 = b2;
    f->a1 = a1;
    f->a2 = a2;

    iir2_reset(f);
}

float iir2_update(iir2_t *f, float x)
{
    float y;

    if (f == NULL)
    {
        return x;
    }

    y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
      - f->a1 * f->y1 - f->a2 * f->y2;

    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

void iir2_reset(iir2_t *f)
{
    if (f == NULL)
    {
        return;
    }

    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}
