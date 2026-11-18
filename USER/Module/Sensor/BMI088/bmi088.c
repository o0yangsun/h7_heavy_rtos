/**
  ******************************************************************************
  * @file    bmi088.c
  * @brief   BMI088 驱动实现，详见 bmi088.h
  *
  *   加速度计读时序比陀螺仪多一个占位字节：单寄存器读为"地址 + dummy + 数据"，
  *   突发读为"地址 + 地址 + 数据"，与 BMI088 数据手册的 SPI 时序一致。
  ******************************************************************************
  */

#include "bmi088.h"
#include "bmi088_reg.h"
#include "bmi088_port.h"

#include <stddef.h>

#define BMI088_COM_WAIT_US      150U
#define BMI088_RESET_DELAY_MS   80U
#define BMI088_CFG_NUM          6U

/* 加速度计配置：寄存器、目标值、失败码 */
static const uint8_t s_acc_cfg[BMI088_CFG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL,      BMI088_ACC_ENABLE_ON,   BMI088_ERR_ACC_PWR_CTRL},
    {BMI088_ACC_PWR_CONF,      BMI088_ACC_PWR_ACTIVE,  BMI088_ERR_ACC_PWR_CONF},
    {BMI088_ACC_CONF,          BMI088_ACC_CONF_VALUE,  BMI088_ERR_ACC_CONF},
    {BMI088_ACC_RANGE,         BMI088_ACC_RANGE_3G,    BMI088_ERR_ACC_RANGE},
    {BMI088_ACC_INT1_IO_CTRL,  BMI088_ACC_INT1_ENABLE, BMI088_ERR_ACC_INT1_IO},
    {BMI088_ACC_INT_MAP_DATA,  BMI088_ACC_INT1_DRDY,   BMI088_ERR_ACC_INT_MAP},
};

/* 陀螺仪配置：寄存器、目标值、失败码 */
static const uint8_t s_gyro_cfg[BMI088_CFG_NUM][3] =
{
    {BMI088_GYRO_RANGE,         BMI088_GYRO_RANGE_2000,    BMI088_ERR_GYRO_RANGE},
    {BMI088_GYRO_BANDWIDTH,     BMI088_GYRO_BANDWIDTH_VALUE, BMI088_ERR_GYRO_BANDWIDTH},
    {BMI088_GYRO_LPM1,          BMI088_GYRO_NORMAL_MODE,   BMI088_ERR_GYRO_LPM1},
    {BMI088_GYRO_CTRL,          BMI088_GYRO_DRDY_ON,       BMI088_ERR_GYRO_CTRL},
    {BMI088_GYRO_INT3_INT4_CONF, BMI088_GYRO_INT3_PP_LOW,  BMI088_ERR_GYRO_INT_CONF},
    {BMI088_GYRO_INT3_INT4_MAP, BMI088_GYRO_INT3_DRDY,     BMI088_ERR_GYRO_INT_MAP},
};

/* ------------------------------ 加速度计访问 ------------------------------ */

static void acc_write(uint8_t reg, uint8_t val)
{
    bmi088_port_cs_accel(0U);
    (void)bmi088_port_spi_rw(reg);
    (void)bmi088_port_spi_rw(val);
    bmi088_port_cs_accel(1U);
}

static uint8_t acc_read(uint8_t reg)
{
    uint8_t val;

    bmi088_port_cs_accel(0U);
    (void)bmi088_port_spi_rw(reg | 0x80U);
    (void)bmi088_port_spi_rw(0x55U);
    val = bmi088_port_spi_rw(0x55U);
    bmi088_port_cs_accel(1U);

    return val;
}

static void acc_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    bmi088_port_cs_accel(0U);
    (void)bmi088_port_spi_rw(reg | 0x80U);
    (void)bmi088_port_spi_rw(reg | 0x80U);

    for (i = 0U; i < len; i++)
    {
        buf[i] = bmi088_port_spi_rw(0x55U);
    }

    bmi088_port_cs_accel(1U);
}

/* -------------------------------- 陀螺仪访问 ------------------------------- */

static void gyro_write(uint8_t reg, uint8_t val)
{
    bmi088_port_cs_gyro(0U);
    (void)bmi088_port_spi_rw(reg);
    (void)bmi088_port_spi_rw(val);
    bmi088_port_cs_gyro(1U);
}

static void gyro_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    bmi088_port_cs_gyro(0U);
    (void)bmi088_port_spi_rw(reg | 0x80U);

    for (i = 0U; i < len; i++)
    {
        buf[i] = bmi088_port_spi_rw(0x55U);
    }

    bmi088_port_cs_gyro(1U);
}

/* -------------------------------- 初始化 --------------------------------- */

static uint8_t accel_init(void)
{
    uint8_t i;
    uint8_t id = 0U;

    (void)acc_read(BMI088_ACC_CHIP_ID);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);
    (void)acc_read(BMI088_ACC_CHIP_ID);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);

    acc_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bmi088_port_delay_ms(BMI088_RESET_DELAY_MS);

    (void)acc_read(BMI088_ACC_CHIP_ID);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);
    id = acc_read(BMI088_ACC_CHIP_ID);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);

    if (id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_ERR_NO_SENSOR;
    }

    for (i = 0U; i < BMI088_CFG_NUM; i++)
    {
        acc_write(s_acc_cfg[i][0], s_acc_cfg[i][1]);
        bmi088_port_delay_us(BMI088_COM_WAIT_US);

        if (acc_read(s_acc_cfg[i][0]) != s_acc_cfg[i][1])
        {
            return s_acc_cfg[i][2];
        }

        bmi088_port_delay_us(BMI088_COM_WAIT_US);
    }

    return BMI088_OK;
}

static uint8_t gyro_init(void)
{
    uint8_t i;
    uint8_t id = 0U;

    gyro_read_buf(BMI088_GYRO_CHIP_ID, &id, 1U);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);
    gyro_read_buf(BMI088_GYRO_CHIP_ID, &id, 1U);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);

    gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088_port_delay_ms(BMI088_RESET_DELAY_MS);

    gyro_read_buf(BMI088_GYRO_CHIP_ID, &id, 1U);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);
    gyro_read_buf(BMI088_GYRO_CHIP_ID, &id, 1U);
    bmi088_port_delay_us(BMI088_COM_WAIT_US);

    if (id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_ERR_NO_SENSOR;
    }

    for (i = 0U; i < BMI088_CFG_NUM; i++)
    {
        uint8_t rd = 0U;

        gyro_write(s_gyro_cfg[i][0], s_gyro_cfg[i][1]);
        bmi088_port_delay_us(BMI088_COM_WAIT_US);

        gyro_read_buf(s_gyro_cfg[i][0], &rd, 1U);

        if (rd != s_gyro_cfg[i][1])
        {
            return s_gyro_cfg[i][2];
        }

        bmi088_port_delay_us(BMI088_COM_WAIT_US);
    }

    return BMI088_OK;
}

/* --------------------------------- 接口 ---------------------------------- */

uint8_t bmi088_init(void)
{
    uint8_t st = accel_init();

    if (st != BMI088_OK)
    {
        return st;
    }

    return gyro_init();
}

uint8_t bmi088_read(bmi088_sample_t *out)
{
    uint8_t buf[8];
    int16_t raw;

    if (out == NULL)
    {
        return 0U;
    }

    acc_read_buf(BMI088_ACC_DATA, buf, 6U);
    raw = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    out->accel[0] = (float)raw * BMI088_ACC_SEN;
    raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    out->accel[1] = (float)raw * BMI088_ACC_SEN;
    raw = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    out->accel[2] = (float)raw * BMI088_ACC_SEN;

    gyro_read_buf(BMI088_GYRO_CHIP_ID, buf, 8U);

    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return 0U;
    }

    raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    out->gyro[0] = (float)raw * BMI088_GYRO_SEN;
    raw = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    out->gyro[1] = (float)raw * BMI088_GYRO_SEN;
    raw = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    out->gyro[2] = (float)raw * BMI088_GYRO_SEN;

    acc_read_buf(BMI088_ACC_TEMP_M, buf, 2U);
    raw = (int16_t)(((uint16_t)buf[0] << 3) | (buf[1] >> 5));

    if (raw > 1023)
    {
        raw = (int16_t)(raw - 2048);
    }

    out->temp_c = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return 1U;
}
