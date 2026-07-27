#ifndef __SPA06_003_H
#define __SPA06_003_H

#include "stm32f1xx_hal.h"

/* SPA06-003 I2C 地址 (DPS310 兼容) */
/* CSB/SDO = VDD → 0x77, CSB/SDO = GND → 0x76 */
#define SPA06_ADDR          (0x77 << 1)

/* ========== 寄存器定义 ========== */
#define SPA06_PRS_B2        0x00    /* 气压数据 MSB */
#define SPA06_PRS_B1        0x01    /* 气压数据 MID */
#define SPA06_PRS_B0        0x02    /* 气压数据 LSB */
#define SPA06_TMP_B2        0x03    /* 温度数据 MSB */
#define SPA06_TMP_B1        0x04    /* 温度数据 MID */
#define SPA06_TMP_B0        0x05    /* 温度数据 LSB */
#define SPA06_PRS_CFG       0x06    /* 气压配置 */
#define SPA06_TMP_CFG       0x07    /* 温度配置 */
#define SPA06_MEAS_CFG      0x08    /* 测量配置/状态 */
#define SPA06_CFG_REG       0x09    /* 中断/FIFO/SPI 配置 */
#define SPA06_INT_STS       0x0A    /* 中断状态 */
#define SPA06_FIFO_STS      0x0B    /* FIFO 状态 */
#define SPA06_RESET         0x0C    /* 软复位 */
#define SPA06_PROD_ID       0x0D    /* 产品/版本 ID */
#define SPA06_COEF_BASE     0x10    /* 校准系数起始地址 (18 bytes) */
#define SPA06_TMP_COEF_SRCE 0x28    /* 温度校准源 */

/* ---- MEAS_CFG (0x08) 位定义 ---- */
#define SPA06_MEAS_PRS_EN       (1 << 0)   /* 使能气压测量 */
#define SPA06_MEAS_TEMP_EN      (1 << 1)   /* 使能温度测量 */
#define SPA06_MEAS_BG           (1 << 2)   /* 后台（连续）模式 */
#define SPA06_MEAS_PRS_RDY      (1 << 4)   /* 气压数据就绪 */
#define SPA06_MEAS_TMP_RDY      (1 << 5)   /* 温度数据就绪 */
#define SPA06_MEAS_SENSOR_RDY   (1 << 6)   /* 传感器初始化完成 */
#define SPA06_MEAS_COEF_RDY     (1 << 7)   /* 校准系数可用 */

/* ---- 工作模式 (MEAS_CFG[2:0], DPS310 标准) ---- */
#define SPA06_MODE_IDLE         0x00    /* 待机 */
#define SPA06_MODE_ONE_PRS      0x01    /* 单次气压测量 */
#define SPA06_MODE_ONE_TEMP     0x02    /* 单次温度测量 */
#define SPA06_MODE_CONT_PRS     0x04    /* 连续气压 */
#define SPA06_MODE_CONT_TEMP    0x05    /* 连续温度 */
#define SPA06_MODE_CONTINUOUS   0x07    /* 连续测量 (气压+温度) */

/* ---- 过采样率 ---- */
#define SPA06_OSR_1      0x00    /* 1倍   (3.6ms) */
#define SPA06_OSR_2      0x01    /* 2倍   (5.2ms) */
#define SPA06_OSR_4      0x02    /* 4倍   (8.4ms) */
#define SPA06_OSR_8      0x03    /* 8倍   (14.8ms) */
#define SPA06_OSR_16     0x04    /* 16倍  (27.6ms) */
#define SPA06_OSR_32     0x05    /* 32倍  (53.2ms) */
#define SPA06_OSR_64     0x06    /* 64倍  (104.4ms) */
#define SPA06_OSR_128    0x07    /* 128倍 (208.4ms) */

/* ---- 测量速率 ---- */
#define SPA06_RATE_1HZ    0x00
#define SPA06_RATE_2HZ    0x10
#define SPA06_RATE_4HZ    0x20
#define SPA06_RATE_8HZ    0x30
#define SPA06_RATE_16HZ   0x40
#define SPA06_RATE_32HZ   0x50
#define SPA06_RATE_64HZ   0x60
#define SPA06_RATE_128HZ  0x70

/* ---- 软复位命令 ---- */
#define SPA06_RESET_CMD   0x09

/* ========== 数据结构体 ========== */
typedef struct
{
    float pressure;         /* 气压 (hPa) - 完整补偿 */
    float pressure_linear;  /* 气压 (hPa) - 仅线性项 */
    float temperature;      /* 温度 (°C)  */
    float altitude;         /* 海拔 (m) */
} SPA06_TypeDef;

extern SPA06_TypeDef spa06;

/* ========== 函数声明 ========== */
uint8_t SPA06_Init(I2C_HandleTypeDef *hi2c);
void    SPA06_Update(I2C_HandleTypeDef *hi2c);

#endif /* __SPA06_003_H */
