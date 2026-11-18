/**
  ******************************************************************************
  * @file    ina226.c
  * @brief   INA226 采样实现，详见 ina226.h
  ******************************************************************************
  */

#include "ina226.h"
#include "robot_config.h"

#define INA226_REG_CONFIG           0x00U
#define INA226_REG_SHUNT_VOLT       0x01U
#define INA226_REG_BUS_VOLT         0x02U
#define INA226_REG_POWER            0x03U
#define INA226_REG_CURRENT          0x04U
#define INA226_REG_CALIB            0x05U

#define INA226_CFG_AVG_64           0x0400U     /* 64 次平均 */
#define INA226_CFG_CT_1100US        0x0100U     /* 母线 1.1 ms */
#define INA226_CFG_MODE_CONT        0x0007U     /* 连续测量母线+分流 */

#if ENABLE_INA226

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

static uint16_t s_reg_cache;

static uint8_t ina226_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFFU);

    return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(INA226_ADDR << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, buf, 2U, 5U) == HAL_OK) ? 1U : 0U;
}

static uint8_t ina226_read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(INA226_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, 2U, 5U) != HAL_OK)
    {
        return 0U;
    }

    *val = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return 1U;
}

void ina226_init(void)
{
    /* CALIB = 0.00512 / (R_shunt * CURRENT_LSB) */
    uint32_t calib = (uint32_t)(5.12e-3f / (INA226_SHUNT_OHM * INA226_CURRENT_LSB_A));

    (void)ina226_write_reg(INA226_REG_CONFIG,
                           (uint16_t)(INA226_CFG_AVG_64 | INA226_CFG_CT_1100US
                                      | (INA226_CFG_CT_1100US >> 3) | INA226_CFG_MODE_CONT));
    (void)ina226_write_reg(INA226_REG_CALIB, (uint16_t)calib);
}

uint8_t ina226_read(float *bus_v, float *current_a)
{
    int16_t raw_v;
    int16_t raw_i;

    if ((bus_v == NULL) || (current_a == NULL))
    {
        return 0U;
    }

    if (ina226_read_reg(INA226_REG_BUS_VOLT, (uint16_t *)&s_reg_cache) == 0U)
    {
        return 0U;
    }
    raw_v = (int16_t)s_reg_cache;

    if (ina226_read_reg(INA226_REG_CURRENT, (uint16_t *)&s_reg_cache) == 0U)
    {
        return 0U;
    }
    raw_i = (int16_t)s_reg_cache;

    *bus_v     = (float)raw_v * 0.00125f;
    *current_a = (float)raw_i * INA226_CURRENT_LSB_A;

    return 1U;
}

#else

void ina226_init(void)
{
}

uint8_t ina226_read(float *bus_v, float *current_a)
{
    (void)bus_v;
    (void)current_a;
    return 0U;
}

#endif /* ENABLE_INA226 */
