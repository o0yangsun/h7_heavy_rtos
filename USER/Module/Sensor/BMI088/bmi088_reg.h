/**
  ******************************************************************************
  * @file    bmi088_reg.h
  * @brief   BMI088 寄存器地址、配置值与量纲系数
  ******************************************************************************
  */

#ifndef BMI088_REG_H
#define BMI088_REG_H

/* -------------------------------- 加速度计 -------------------------------- */
#define BMI088_ACC_CHIP_ID          0x00U
#define BMI088_ACC_CHIP_ID_VALUE    0x1EU
#define BMI088_ACC_DATA             0x12U       /* X/Y/Z 各 2 字节，小端 */
#define BMI088_ACC_TEMP_M           0x22U
#define BMI088_ACC_TEMP_L           0x23U
#define BMI088_ACC_CONF             0x40U
#define BMI088_ACC_RANGE            0x41U
#define BMI088_ACC_INT1_IO_CTRL     0x53U
#define BMI088_ACC_INT_MAP_DATA     0x58U
#define BMI088_ACC_PWR_CONF         0x7CU
#define BMI088_ACC_PWR_CTRL         0x7DU
#define BMI088_ACC_SOFTRESET        0x7EU

#define BMI088_ACC_SOFTRESET_VALUE  0xB6U
#define BMI088_ACC_PWR_ACTIVE       0x00U
#define BMI088_ACC_ENABLE_ON        0x04U
/* bit7 必须置 1；BWP = normal(0x2<<4)；ODR = 800 Hz(0x0B) */
#define BMI088_ACC_CONF_VALUE       (0x80U | 0x20U | 0x0BU)
#define BMI088_ACC_RANGE_3G         0x00U
#define BMI088_ACC_INT1_ENABLE      (0x1U << 3)
#define BMI088_ACC_INT1_DRDY        (0x1U << 2)

/* --------------------------------- 陀螺仪 --------------------------------- */
#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_CHIP_ID_VALUE   0x0FU
#define BMI088_GYRO_DATA            0x02U       /* X/Y/Z 各 2 字节，小端 */
#define BMI088_GYRO_RANGE           0x0FU
#define BMI088_GYRO_BANDWIDTH       0x10U
#define BMI088_GYRO_LPM1            0x11U
#define BMI088_GYRO_SOFTRESET       0x14U
#define BMI088_GYRO_CTRL            0x15U
#define BMI088_GYRO_INT3_INT4_CONF  0x16U
#define BMI088_GYRO_INT3_INT4_MAP   0x18U

#define BMI088_GYRO_SOFTRESET_VALUE 0xB6U
#define BMI088_GYRO_RANGE_2000      0x00U
/* bit7 必须置 1；ODR 1000 Hz / 带宽 116 Hz */
#define BMI088_GYRO_BANDWIDTH_VALUE (0x80U | 0x02U)
#define BMI088_GYRO_NORMAL_MODE     0x00U
#define BMI088_GYRO_DRDY_ON         0x80U
#define BMI088_GYRO_INT3_PP_LOW     0x00U
#define BMI088_GYRO_INT3_DRDY       0x01U

/* -------------------------------- 量纲系数 -------------------------------- */
#define BMI088_ACC_SEN              0.0008974358974f      /* ±3 g，m/s^2/LSB */
#define BMI088_GYRO_SEN             0.0010652644360f      /* ±2000 dps，rad/s/LSB */
#define BMI088_TEMP_FACTOR          0.125f
#define BMI088_TEMP_OFFSET          23.0f

#endif /* BMI088_REG_H */
