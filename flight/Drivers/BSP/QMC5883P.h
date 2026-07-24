#ifndef __QMC5883P_H
#define __QMC5883P_H

#include "stm32f1xx_hal.h"

/* QMC5883P I2C 地址（固定） */
#define QMC5883P_ADDR   (0x2C << 1)

/* ========== 寄存器定义 ========== */
#define QMC5883P_CHIP_ID       0x00    /* 芯片 ID, 应返回 0x80 */
#define QMC5883P_DATA_X_LSB    0x01    /* X 轴数据 LSB */
#define QMC5883P_DATA_X_MSB    0x02    /* X 轴数据 MSB */
#define QMC5883P_DATA_Y_LSB    0x03    /* Y 轴数据 LSB */
#define QMC5883P_DATA_Y_MSB    0x04    /* Y 轴数据 MSB */
#define QMC5883P_DATA_Z_LSB    0x05    /* Z 轴数据 LSB */
#define QMC5883P_DATA_Z_MSB    0x06    /* Z 轴数据 MSB */
#define QMC5883P_STATUS        0x09    /* 状态: bit0=DRDY */
#define QMC5883P_CTRL1         0x0A    /* 控制寄存器1: 模式/ODR */
#define QMC5883P_CTRL2         0x0B    /* 控制寄存器2: 量程 */
#define QMC5883P_CTRL3         0x0D    /* 内部配置 */
#define QMC5883P_CTRL4         0x29    /* 内部配置 */

/* ---- STATUS (0x09) 位定义 ---- */
#define QMC5883P_STATUS_DRDY   (1 << 0)   /* 数据就绪 */

/* ---- CTRL1 (0x0A) ---- */
/* bit1:0 模式 */
#define QMC5883P_MODE_SUSPEND   0x00
#define QMC5883P_MODE_NORMAL    0x01
#define QMC5883P_MODE_SINGLE    0x02
#define QMC5883P_MODE_CONT      0x03    /* 连续模式 */

/* bit3:2 ODR（输出数据速率） */
#define QMC5883P_ODR_10HZ       0x00
#define QMC5883P_ODR_50HZ       0x04
#define QMC5883P_ODR_100HZ      0x08
#define QMC5883P_ODR_200HZ      0x0C

/* ---- CTRL2 (0x0B) ---- */
/* bit3:2 量程 */
#define QMC5883P_RANGE_30G      0x00    /* ±30G,  1000 LSB/G */
#define QMC5883P_RANGE_12G      0x04    /* ±12G,  2500 LSB/G */
#define QMC5883P_RANGE_8G       0x08    /* ±8G,   3750 LSB/G */
#define QMC5883P_RANGE_2G       0x0C    /* ±2G,  15000 LSB/G */

/* ========== 数据结构体 ========== */
typedef struct
{
    int16_t mx;     /* X 轴原始磁通量 */
    int16_t my;     /* Y 轴原始磁通量 */
    int16_t mz;     /* Z 轴原始磁通量 */

    float heading;  /* 航向角 (度), 0°~360° */
} QMC5883P_TypeDef;

extern QMC5883P_TypeDef qmc;
extern uint8_t g_qmc_ok;        /* 驱动是否初始化成功 */

/* ========== 函数声明 ========== */
uint8_t QMC5883P_Init(I2C_HandleTypeDef *hi2c);
void    QMC5883P_ReadData(I2C_HandleTypeDef *hi2c);
void    QMC5883P_ComputeHeading(void);

#endif /* __QMC5883P_H */
