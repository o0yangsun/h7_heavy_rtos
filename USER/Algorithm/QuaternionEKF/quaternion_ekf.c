/**
  ******************************************************************************
  * @file    quaternion_ekf.c
  * @brief   四元数姿态 EKF 实现，详见 quaternion_ekf.h
  *
  *   算法来自 hero 工程的 QuaternionEKF（Wang Hongxi），矩阵维度固定为 6x6 与
  *   3x6，直接用一维数组展开，不再依赖通用矩阵库与 CMSIS-DSP。
  ******************************************************************************
  */

#include "quaternion_ekf.h"

#include <math.h>
#include <string.h>

#define EKF_N                   6
#define EKF_M                   3
#define EKF_CHI2_THRESHOLD      1e-8f
#define EKF_BIAS_LIMIT_PER_S    0.01f
#define EKF_BIAS_P_MAX          10000.0f
#define EKF_P_BIAS_INIT         100.0f
#define EKF_P_QUAT_INIT         100000.0f

static const float s_identity[EKF_N * EKF_N] =
{
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};

typedef struct
{
    float    x[EKF_N];
    float    P[EKF_N * EKF_N];
    float    accel_lpf[3];
    float    q1_noise;
    float    q2_noise;
    float    r_noise;
    float    lambda;
    float    lpf_coef;
    float    dt;
    uint8_t  stable;
    uint8_t  converged;
    uint32_t error_cnt;
    uint32_t update_cnt;
    float    yaw_last;
    int16_t  yaw_round;
    uint8_t  yaw_hold;
    uint32_t reset_cnt;
    uint8_t  fault;
    uint8_t  last_fault;
    float    dbg_gyro[3];
    float    dbg_accel[3];
    float    dbg_chi2;
    float    f_gyro[3];
    float    f_accel[3];
    float    f_x[EKF_N];
    float    f_chi2;
    float    f_p0;
    uint8_t  path;
} quat_ekf_t;

static quat_ekf_t s_ekf;
static float      s_P_init[EKF_N * EKF_N];

/* 数值保护阈值 */
#define EKF_BAD_LIMIT           1.0e6f
#define EKF_P_MIN               1.0e-6f
#define EKF_P_MAX               1.0e8f
#define EKF_BIAS_MAX            0.2f

/* ------------------------------- 内部工具 -------------------------------- */

static float inv_sqrt(float x)
{
    float   halfx = 0.5f * x;
    float   y = x;
    int32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    memcpy(&i, &y, sizeof(i));
    i = 0x5F375A86 - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/** out = a * b，均为 6x6 */
static void mat6_mul(const float *a, const float *b, float *out)
{
    float   t[EKF_N * EKF_N];
    uint8_t r;
    uint8_t c;
    uint8_t k;

    for (r = 0U; r < EKF_N; r++)
    {
        for (c = 0U; c < EKF_N; c++)
        {
            float s = 0.0f;

            for (k = 0U; k < EKF_N; k++)
            {
                s += a[r * EKF_N + k] * b[k * EKF_N + c];
            }

            t[r * EKF_N + c] = s;
        }
    }

    memcpy(out, t, sizeof(t));
}

/** 判断浮点数是否非法（NaN 或超出量程） */
static uint8_t is_bad(float v)
{
    if (v != v)
    {
        return 1U;
    }

    return ((v > EKF_BAD_LIMIT) || (v < -EKF_BAD_LIMIT)) ? 1U : 0U;
}

/** 仅判断 NaN 与溢出，用于量测协方差 S 这类量级远超状态量的矩阵 */
static uint8_t is_nan_or_inf(float v)
{
    if (v != v)
    {
        return 1U;
    }

    return ((v > 1.0e30f) || (v < -1.0e30f)) ? 1U : 0U;
}

/** 3x3 求逆，返回 0 表示近奇异或含非法值 */
static uint8_t mat3_inv(const float *in, float *out)
{
    float   det;
    float   inv;
    float   scale;
    float   t[9];
    uint8_t i;

    for (i = 0U; i < 9U; i++)
    {
        if (is_nan_or_inf(in[i]) != 0U)
        {
            return 0U;
        }
    }

    det = in[0] * (in[4] * in[8] - in[5] * in[7])
        - in[1] * (in[3] * in[8] - in[5] * in[6])
        + in[2] * (in[3] * in[7] - in[4] * in[6]);

    /* 用对角线量级做相对阈值，避免病态矩阵放大增益 */
    scale = fabsf(in[0]) + fabsf(in[4]) + fabsf(in[8]);

    if ((det != det) || (scale <= 0.0f)
        || (fabsf(det) < (1e-6f * scale * scale * scale)))
    {
        return 0U;
    }

    inv = 1.0f / det;

    t[0] =  (in[4] * in[8] - in[5] * in[7]) * inv;
    t[1] = -(in[1] * in[8] - in[2] * in[7]) * inv;
    t[2] =  (in[1] * in[5] - in[2] * in[4]) * inv;
    t[3] = -(in[3] * in[8] - in[5] * in[6]) * inv;
    t[4] =  (in[0] * in[8] - in[2] * in[6]) * inv;
    t[5] = -(in[0] * in[5] - in[2] * in[3]) * inv;
    t[6] =  (in[3] * in[7] - in[4] * in[6]) * inv;
    t[7] = -(in[0] * in[7] - in[1] * in[6]) * inv;
    t[8] =  (in[0] * in[4] - in[1] * in[3]) * inv;

    memcpy(out, t, sizeof(t));

    return 1U;
}

/** 6x6 左乘 6 维向量，结果写入 out */
static void mat6_mul_vec(const float *a, const float *v, float *out)
{
    float   t[EKF_N];
    uint8_t r;
    uint8_t k;

    for (r = 0U; r < EKF_N; r++)
    {
        float s = 0.0f;

        for (k = 0U; k < EKF_N; k++)
        {
            s += a[r * EKF_N + k] * v[k];
        }

        t[r] = s;
    }

    memcpy(out, t, sizeof(t));
}

/* -------------------------------- 对外接口 -------------------------------- */

void quat_ekf_init(float q0, float q1, float q2, float q3,
                   float Q1, float Q2, float R, float lambda, float lpf)
{
    uint8_t i;
    float   n;

    memset(&s_ekf, 0, sizeof(s_ekf));

    n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (n < 1e-6f)
    {
        s_ekf.x[0] = 1.0f;
    }
    else
    {
        s_ekf.x[0] = q0 / n;
        s_ekf.x[1] = q1 / n;
        s_ekf.x[2] = q2 / n;
        s_ekf.x[3] = q3 / n;
    }

    for (i = 0U; i < (EKF_N * EKF_N); i++)
    {
        s_P_init[i] = 0.1f;
    }

    s_P_init[0]  = EKF_P_QUAT_INIT;
    s_P_init[7]  = EKF_P_QUAT_INIT;
    s_P_init[14] = EKF_P_QUAT_INIT;
    s_P_init[21] = EKF_P_QUAT_INIT;
    s_P_init[28] = EKF_P_BIAS_INIT;
    s_P_init[35] = EKF_P_BIAS_INIT;

    memcpy(s_ekf.P, s_P_init, sizeof(s_P_init));

    s_ekf.q1_noise = Q1;
    s_ekf.q2_noise = Q2;
    s_ekf.r_noise  = R;
    s_ekf.lpf_coef = lpf;

    if ((lambda <= 0.0f) || (lambda > 1.0f))
    {
        s_ekf.lambda = 1.0f;
    }
    else
    {
        s_ekf.lambda = lambda;
    }
}

void quat_ekf_update(const float gyro[3], const float accel[3], float dt)
{
    float   F[EKF_N * EKF_N];
    float   Pn[EKF_N * EKF_N];
    float   H[EKF_M * EKF_N];
    float   HP[EKF_M * EKF_N];
    float   S[EKF_M * EKF_M];
    float   Sinv[EKF_M * EKF_M];
    float   K[EKF_N * EKF_M];
    float   KHP[EKF_N * EKF_N];
    float   xp[EKF_N];
    float   z[EKF_M];
    float   h[EKF_M];
    float   innov[EKF_M];
    float   gx;
    float   gy;
    float   gz;
    float   halfgxdt;
    float   halfgydt;
    float   halfgzdt;
    float   acc_inv_norm;
    float   acc_norm;
    float   gyro_norm;
    float   q0;
    float   q1;
    float   q2;
    float   q3;
    float   q_inv_norm;
    float   chi2;
    float   gain_scale = 1.0f;
    float   dt_bias_limit;
    uint8_t skip_p = 0U;
    uint8_t r;
    uint8_t c;
    uint8_t k;

    if ((gyro == NULL) || (accel == NULL) || (dt <= 0.0f) || (dt > 0.05f))
    {
        s_ekf.path = 1U;
        return;
    }

    s_ekf.dt = dt;
    s_ekf.dbg_gyro[0]  = gyro[0];
    s_ekf.dbg_gyro[1]  = gyro[1];
    s_ekf.dbg_gyro[2]  = gyro[2];
    s_ekf.dbg_accel[0] = accel[0];
    s_ekf.dbg_accel[1] = accel[1];
    s_ekf.dbg_accel[2] = accel[2];

    gx = gyro[0] - s_ekf.x[4];
    gy = gyro[1] - s_ekf.x[5];
    gz = gyro[2];

    /* 1. F 的单位阵与左上 4x4 块 */
    memcpy(F, s_identity, sizeof(F));

    halfgxdt = 0.5f * gx * dt;
    halfgydt = 0.5f * gy * dt;
    halfgzdt = 0.5f * gz * dt;

    F[1]  = -halfgxdt;
    F[2]  = -halfgydt;
    F[3]  = -halfgzdt;
    F[6]  = halfgxdt;
    F[8]  = halfgzdt;
    F[9]  = -halfgydt;
    F[12] = halfgydt;
    F[13] = -halfgzdt;
    F[15] = halfgxdt;
    F[18] = halfgzdt;
    F[19] = halfgydt;
    F[20] = -halfgxdt;

    /* 2. 加速度一阶低通 */
    if (s_ekf.update_cnt == 0U)
    {
        s_ekf.accel_lpf[0] = accel[0];
        s_ekf.accel_lpf[1] = accel[1];
        s_ekf.accel_lpf[2] = accel[2];
    }

    for (r = 0U; r < 3U; r++)
    {
        float den = dt + s_ekf.lpf_coef;

        if (den <= 0.0f)
        {
            s_ekf.accel_lpf[r] = accel[r];
        }
        else
        {
            s_ekf.accel_lpf[r] = s_ekf.accel_lpf[r] * s_ekf.lpf_coef / den
                               + accel[r] * dt / den;
        }
    }

    /* 3. 量测向量单位化 */
    acc_inv_norm = inv_sqrt(s_ekf.accel_lpf[0] * s_ekf.accel_lpf[0]
                          + s_ekf.accel_lpf[1] * s_ekf.accel_lpf[1]
                          + s_ekf.accel_lpf[2] * s_ekf.accel_lpf[2]);

    if (acc_inv_norm <= 0.0f)
    {
        s_ekf.path = 2U;
        return;
    }

    for (r = 0U; r < 3U; r++)
    {
        z[r] = s_ekf.accel_lpf[r] * acc_inv_norm;
    }

    /* 4. 静止判定，用于卡方检验的分支选择 */
    gyro_norm = 1.0f / inv_sqrt(gx * gx + gy * gy + gz * gz);
    acc_norm  = 1.0f / acc_inv_norm;

    s_ekf.stable = ((gyro_norm < 0.3f) && (fabsf(acc_norm - 9.8f) < 0.5f)) ? 1U : 0U;

    /* 5. 先验估计 x' = F x */
    mat6_mul_vec(F, s_ekf.x, xp);

    /* 6. F 右上 4x2 块线性化，四元数归一化，零偏方差渐消 */
    q0 = xp[0];
    q1 = xp[1];
    q2 = xp[2];
    q3 = xp[3];

    q_inv_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (q_inv_norm > 0.0f)
    {
        xp[0] = q0 * q_inv_norm;
        xp[1] = q1 * q_inv_norm;
        xp[2] = q2 * q_inv_norm;
        xp[3] = q3 * q_inv_norm;

        q0 = xp[0];
        q1 = xp[1];
        q2 = xp[2];
        q3 = xp[3];
    }

    F[4]  = q1 * dt * 0.5f;
    F[5]  = q2 * dt * 0.5f;
    F[10] = -q0 * dt * 0.5f;
    F[11] = q3 * dt * 0.5f;
    F[16] = -q3 * dt * 0.5f;
    F[17] = -q0 * dt * 0.5f;
    F[22] = q2 * dt * 0.5f;
    F[23] = -q1 * dt * 0.5f;

    s_ekf.P[28] /= s_ekf.lambda;
    s_ekf.P[35] /= s_ekf.lambda;

    if (s_ekf.P[28] > EKF_BIAS_P_MAX)
    {
        s_ekf.P[28] = EKF_BIAS_P_MAX;
    }
    if (s_ekf.P[35] > EKF_BIAS_P_MAX)
    {
        s_ekf.P[35] = EKF_BIAS_P_MAX;
    }

    /* 7. 预测协方差 P' = F P F^T + Q */
    {
        float FP[EKF_N * EKF_N];
        float FT[EKF_N * EKF_N];

        for (r = 0U; r < EKF_N; r++)
        {
            for (c = 0U; c < EKF_N; c++)
            {
                FT[r * EKF_N + c] = F[c * EKF_N + r];
            }
        }

        mat6_mul(F, s_ekf.P, FP);
        mat6_mul(FP, FT, Pn);
    }

    Pn[0]  += s_ekf.q1_noise * dt;
    Pn[7]  += s_ekf.q1_noise * dt;
    Pn[14] += s_ekf.q1_noise * dt;
    Pn[21] += s_ekf.q1_noise * dt;
    Pn[28] += s_ekf.q2_noise * dt;
    Pn[35] += s_ekf.q2_noise * dt;

    /* 8. 观测矩阵 H */
    memset(H, 0, sizeof(H));

    H[0]  = -2.0f * q2;
    H[1]  =  2.0f * q3;
    H[2]  = -2.0f * q0;
    H[3]  =  2.0f * q1;
    H[6]  =  2.0f * q1;
    H[7]  =  2.0f * q0;
    H[8]  =  2.0f * q3;
    H[9]  =  2.0f * q2;
    H[12] =  2.0f * q0;
    H[13] = -2.0f * q1;
    H[14] = -2.0f * q2;
    H[15] =  2.0f * q3;

    /* 9. S = H P' H^T + R */
    for (r = 0U; r < EKF_M; r++)
    {
        for (c = 0U; c < EKF_N; c++)
        {
            float s = 0.0f;

            for (k = 0U; k < EKF_N; k++)
            {
                s += H[r * EKF_N + k] * Pn[k * EKF_N + c];
            }

            HP[r * EKF_N + c] = s;
        }
    }

    for (r = 0U; r < EKF_M; r++)
    {
        for (c = 0U; c < EKF_M; c++)
        {
            float s = 0.0f;

            for (k = 0U; k < EKF_N; k++)
            {
                s += HP[r * EKF_N + k] * H[c * EKF_N + k];
            }

            S[r * EKF_M + c] = s;
        }

        S[r * EKF_M + r] += s_ekf.r_noise;
    }

    if (mat3_inv(S, Sinv) == 0U)
    {
        s_ekf.path = 3U;
        return;
    }

    /* 10. 新息与新息协方差，用于卡方检验 */
    h[0] = 2.0f * (q1 * q3 - q0 * q2);
    h[1] = 2.0f * (q0 * q1 + q2 * q3);
    h[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (r = 0U; r < EKF_M; r++)
    {
        innov[r] = z[r] - h[r];
    }

    chi2 = 0.0f;

    for (r = 0U; r < EKF_M; r++)
    {
        for (c = 0U; c < EKF_M; c++)
        {
            chi2 += innov[r] * Sinv[r * EKF_M + c] * innov[c];
        }
    }

    s_ekf.dbg_chi2 = chi2;

    if (chi2 < (0.5f * EKF_CHI2_THRESHOLD))
    {
        s_ekf.converged = 1U;
    }

    if ((chi2 > EKF_CHI2_THRESHOLD) && (s_ekf.converged != 0U))
    {
        if (s_ekf.stable != 0U)
        {
            s_ekf.error_cnt++;
        }
        else
        {
            s_ekf.error_cnt = 0U;
        }

        if (s_ekf.error_cnt > 50U)
        {
            /* 持续通不过检验，判定发散，下一轮重新收敛 */
            s_ekf.converged = 0U;
        }
        else
        {
            /* 本轮只预测，不融合 */
            memcpy(s_ekf.x, xp, sizeof(xp));
            memcpy(s_ekf.P, Pn, sizeof(Pn));
            s_ekf.path = 4U;
            s_ekf.update_cnt++;
            return;
        }
    }
    else
    {
        if ((chi2 > (0.1f * EKF_CHI2_THRESHOLD)) && (s_ekf.converged != 0U))
        {
            gain_scale = (EKF_CHI2_THRESHOLD - chi2) / (0.9f * EKF_CHI2_THRESHOLD);

            if (gain_scale < 0.0f)
            {
                gain_scale = 0.0f;
            }
        }
        else
        {
            gain_scale = 1.0f;
        }

        s_ekf.error_cnt = 0U;
        skip_p = 0U;
    }

    /* 11. K = P' H^T S^-1 */
    for (r = 0U; r < EKF_N; r++)
    {
        for (c = 0U; c < EKF_M; c++)
        {
            float s = 0.0f;

            for (k = 0U; k < EKF_M; k++)
            {
                s += HP[k * EKF_N + r] * Sinv[k * EKF_M + c];
            }

            K[r * EKF_M + c] = s * gain_scale;
        }
    }

    /* 零偏增益按观测方向余弦缩放，接近坐标轴时增益更大 */
    {
        float cosine[3];

        for (r = 0U; r < 3U; r++)
        {
            float a = fabsf(h[r]);

            /* h 理论上在 [-1,1]，浮点误差会让 acosf 返回 NaN */
            if (a > 1.0f)
            {
                a = 1.0f;
            }

            cosine[r] = acosf(a);
        }

        for (r = 4U; r < EKF_N; r++)
        {
            for (c = 0U; c < EKF_M; c++)
            {
                K[r * EKF_M + c] *= cosine[r - 4U] / 1.5707963f;
            }
        }
    }

    /* 12. x = x' + K(z - h(x')) */
    {
        float dx[EKF_N];
        float bias_step_max;

        for (r = 0U; r < EKF_N; r++)
        {
            float s = 0.0f;

            for (c = 0U; c < EKF_M; c++)
            {
                s += K[r * EKF_M + c] * innov[c];
            }

            dx[r] = s;
        }

        bias_step_max = EKF_BIAS_LIMIT_PER_S * dt;

        if (s_ekf.converged != 0U)
        {
            for (r = 4U; r < EKF_N; r++)
            {
                if (dx[r] > bias_step_max)
                {
                    dx[r] = bias_step_max;
                }
                else if (dx[r] < -bias_step_max)
                {
                    dx[r] = -bias_step_max;
                }
            }
        }

        /* yaw 不可观测，四元数第 3 分量不修正 */
        dx[3] = 0.0f;

        {
            uint8_t bad = 0U;

            for (r = 0U; r < EKF_N; r++)
            {
                if (is_bad(dx[r]) != 0U)
                {
                    bad = 1U;
                    break;
                }
            }

            if (bad != 0U)
            {
                /* 增益异常时丢弃本次修正，只保留预测 */
                memcpy(s_ekf.x, xp, sizeof(xp));
                memcpy(s_ekf.P, Pn, sizeof(Pn));
                s_ekf.path = 6U;
                s_ekf.update_cnt++;
                return;
            }
        }

        for (r = 0U; r < EKF_N; r++)
        {
            s_ekf.x[r] = xp[r] + dx[r];
        }
    }

    /* 13. P = P' - K H P' */
    if (skip_p == 0U)
    {
        for (r = 0U; r < EKF_N; r++)
        {
            for (c = 0U; c < EKF_N; c++)
            {
                float s = 0.0f;

                for (k = 0U; k < EKF_M; k++)
                {
                    s += K[r * EKF_M + k] * HP[k * EKF_N + c];
                }

                KHP[r * EKF_N + c] = s;
            }
        }

        for (r = 0U; r < (EKF_N * EKF_N); r++)
        {
            s_ekf.P[r] = Pn[r] - KHP[r];
        }
    }
    else
    {
        memcpy(s_ekf.P, Pn, sizeof(Pn));
    }

    /* 14. 协方差对称化与对角占优，抑制长时间运行的非对角漂移 */
    for (r = 0U; r < EKF_N; r++)
    {
        for (c = (uint8_t)(r + 1U); c < EKF_N; c++)
        {
            float v = 0.5f * (s_ekf.P[r * EKF_N + c] + s_ekf.P[c * EKF_N + r]);

            s_ekf.P[r * EKF_N + c] = v;
            s_ekf.P[c * EKF_N + r] = v;
        }
    }

    for (r = 0U; r < EKF_N; r++)
    {
        for (c = 0U; c < EKF_N; c++)
        {
            float lim;

            if (r == c)
            {
                continue;
            }

            lim = sqrtf(s_ekf.P[r * EKF_N + r] * s_ekf.P[c * EKF_N + c]);

            if (s_ekf.P[r * EKF_N + c] > lim)
            {
                s_ekf.P[r * EKF_N + c] = lim;
            }
            else if (s_ekf.P[r * EKF_N + c] < -lim)
            {
                s_ekf.P[r * EKF_N + c] = -lim;
            }
        }
    }

    /* 零偏物理上限，超过说明观测被污染 */
    for (r = 4U; r < EKF_N; r++)
    {
        if (s_ekf.x[r] > EKF_BIAS_MAX)
        {
            s_ekf.x[r] = EKF_BIAS_MAX;
        }
        else if (s_ekf.x[r] < -EKF_BIAS_MAX)
        {
            s_ekf.x[r] = -EKF_BIAS_MAX;
        }
    }

    /* 四元数保持单位长度，避免模长漂移影响欧拉角 */
    {
        float qn = inv_sqrt(s_ekf.x[0] * s_ekf.x[0] + s_ekf.x[1] * s_ekf.x[1]
                          + s_ekf.x[2] * s_ekf.x[2] + s_ekf.x[3] * s_ekf.x[3]);

        if (qn > 0.0f)
        {
            s_ekf.x[0] *= qn;
            s_ekf.x[1] *= qn;
            s_ekf.x[2] *= qn;
            s_ekf.x[3] *= qn;
        }
    }

    /* 15. 数值保护：协方差对角限幅，状态非法时回到初始值 */
    for (r = 0U; r < EKF_N; r++)
    {
        float d = s_ekf.P[r * EKF_N + r];

        if (d != d)
        {
            s_ekf.fault = 2U;
            break;
        }

        if (d < EKF_P_MIN)
        {
            s_ekf.P[r * EKF_N + r] = EKF_P_MIN;
        }
        else if (d > EKF_P_MAX)
        {
            s_ekf.P[r * EKF_N + r] = EKF_P_MAX;
        }
    }

    for (r = 0U; r < EKF_N; r++)
    {
        if (is_bad(s_ekf.x[r]) != 0U)
        {
            s_ekf.fault = 1U;
            break;
        }
    }

    if (s_ekf.fault != 0U)
    {
        memcpy(s_ekf.f_gyro, s_ekf.dbg_gyro, sizeof(s_ekf.f_gyro));
        memcpy(s_ekf.f_accel, s_ekf.dbg_accel, sizeof(s_ekf.f_accel));
        memcpy(s_ekf.f_x, s_ekf.x, sizeof(s_ekf.f_x));
        s_ekf.f_chi2    = s_ekf.dbg_chi2;
        s_ekf.f_p0      = s_ekf.P[0];
        s_ekf.last_fault = s_ekf.fault;

        memset(&s_ekf.x[0], 0, sizeof(s_ekf.x));
        s_ekf.x[0] = 1.0f;
        memcpy(s_ekf.P, s_P_init, sizeof(s_P_init));
        s_ekf.converged = 0U;
        s_ekf.error_cnt = 0U;
        s_ekf.reset_cnt++;
        s_ekf.fault = 0U;

        /* 重置后 yaw 会跳到 0，圈数累计同步归零且下一帧不判跳变 */
        s_ekf.yaw_round = 0;
        s_ekf.yaw_hold  = 1U;
    }

    s_ekf.path = 5U;
    s_ekf.update_cnt++;
}

void quat_ekf_get(quat_ekf_out_t *out)
{
    float   q0;
    float   q1;
    float   q2;
    float   q3;
    float   yaw;
    float   n;

    if (out == NULL)
    {
        return;
    }

    q0 = s_ekf.x[0];
    q1 = s_ekf.x[1];
    q2 = s_ekf.x[2];
    q3 = s_ekf.x[3];

    n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (n < 1e-6f)
    {
        out->q[0] = 1.0f;
        out->q[1] = 0.0f;
        out->q[2] = 0.0f;
        out->q[3] = 0.0f;
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
    else
    {
        out->q[0] = q0 / n;
        out->q[1] = q1 / n;
        out->q[2] = q2 / n;
        out->q[3] = q3 / n;
        q0 = out->q[0];
        q1 = out->q[1];
        q2 = out->q[2];
        q3 = out->q[3];
    }

    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                 2.0f * (q0 * q0 + q1 * q1) - 1.0f);

    {
        float sin_roll = -2.0f * (q1 * q3 - q0 * q2);

        if (sin_roll > 1.0f)
        {
            sin_roll = 1.0f;
        }
        else if (sin_roll < -1.0f)
        {
            sin_roll = -1.0f;
        }

        out->roll = asinf(sin_roll);
    }

    out->pitch = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        2.0f * (q0 * q0 + q3 * q3) - 1.0f);

    if (s_ekf.yaw_hold != 0U)
    {
        /* 刚重置，本次不做跨 ±pi 判定 */
        s_ekf.yaw_hold = 0U;
    }
    else if ((yaw - s_ekf.yaw_last) > 3.14159265f)
    {
        s_ekf.yaw_round--;
    }
    else if ((yaw - s_ekf.yaw_last) < -3.14159265f)
    {
        s_ekf.yaw_round++;
    }

    s_ekf.yaw_last = yaw;

    out->yaw       = yaw;
    out->yaw_total = 6.2831853f * (float)s_ekf.yaw_round + yaw;

    out->gyro_bias[0] = s_ekf.x[4];
    out->gyro_bias[1] = s_ekf.x[5];
    out->gyro_bias[2] = 0.0f;

    out->converged = s_ekf.converged;
}
