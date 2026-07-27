#include "spa06.h"
#include <math.h>

SPA06_TypeDef spa06;

#define SPA06_KP  253952.0f    /* 气压缩放：固定16倍过采样 */
#define SPA06_KT  524288.0f    /* 温度缩放：固定1倍过采样  */

static int16_t c0, c1;
static int32_t c00, c10;
static int16_t c01, c11, c20, c21, c30;
static int16_t c31, c40;

static void wr(I2C_HandleTypeDef *h, uint8_t r, uint8_t d) {
    HAL_I2C_Mem_Write(h, SPA06_ADDR, r, I2C_MEMADD_SIZE_8BIT, &d, 1, 100);
}
static void rds(I2C_HandleTypeDef *h, uint8_t r, uint8_t *b, uint8_t n) {
    HAL_I2C_Mem_Read(h, SPA06_ADDR, r, I2C_MEMADD_SIZE_8BIT, b, n, 100);
}

static int16_t sgn12(uint16_t v) { return (v & 0x0800) ? (int16_t)(v | 0xF000) : (int16_t)v; }
static int32_t sgn20(uint32_t v) { return (v & 0x00080000) ? (int32_t)(v | 0xFFF00000) : (int32_t)v; }

uint8_t SPA06_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[21], id, i, meas;

    if (HAL_I2C_IsDeviceReady(hi2c, SPA06_ADDR, 3, 50) != HAL_OK)
        return 1;

    /* 软复位并等待 */
    wr(hi2c, SPA06_RESET, 0x09);
    HAL_Delay(50);
    for (i = 0; i < 50; i++) {
        HAL_Delay(10);
        rds(hi2c, SPA06_MEAS_CFG, &meas, 1);
        if (meas & SPA06_MEAS_COEF_RDY) break;
    }

    /* 读21字节系数 */
    rds(hi2c, SPA06_COEF_BASE, buf, 21);

    c0  = sgn12(((uint16_t)buf[0] << 4) | ((uint16_t)buf[1] >> 4));
    c1  = sgn12((((uint16_t)buf[1] & 0x0F) << 8) | (uint16_t)buf[2]);
    c00 = sgn20(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4));
    c10 = sgn20(((uint32_t)(buf[5] & 0x0F) << 16) | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7]);
    c01 = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
    c11 = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    c20 = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);
    c21 = (int16_t)(((uint16_t)buf[14] << 8) | (uint16_t)buf[15]);
    c30 = (int16_t)(((uint16_t)buf[16] << 8) | (uint16_t)buf[17]);
    c31 = sgn12(((uint16_t)buf[18] << 4) | ((uint16_t)buf[19] >> 4));
    c40 = sgn12((((uint16_t)buf[19] & 0x0F) << 8) | (uint16_t)buf[20]);

    rds(hi2c, SPA06_PROD_ID, &id, 1);

    /* 配置传感器: 气压16xOSR+32Hz, 温度1xOSR+16Hz, 压力右移, 连续模式 */
    wr(hi2c, SPA06_PRS_CFG, 0x54);
    wr(hi2c, SPA06_TMP_CFG, 0x50);
    wr(hi2c, SPA06_CFG_REG, 0x04);
    wr(hi2c, SPA06_MEAS_CFG, 0x07);
    HAL_Delay(60);

    return 0;
}

void SPA06_ReadData(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6];
    int32_t Praw, Traw;
    float Psc, Tsc, P2, P3, P4;

    /* 原子读取6字节 (0x00-0x05) */
    rds(hi2c, SPA06_PRS_B2, buf, 6);

    /* 气压在 0x00-0x02, 温度在 0x03-0x05 (DPS310 标准顺序) */
    Praw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
    Traw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | (int32_t)buf[5];
    if (Praw & 0x00800000) Praw |= 0xFF000000;
    if (Traw & 0x00800000) Traw |= 0xFF000000;

    Psc = (float)Praw / SPA06_KP;
    Tsc = (float)Traw / SPA06_KT;
    P2 = Psc * Psc;
    P3 = P2 * Psc;
    P4 = P3 * Psc;

    /* 气压补偿 (Pa → hPa) */
    spa06.pressure = ((float)c00
                    + (float)c10 * Psc + (float)c20 * P2
                    + (float)c30 * P3 + (float)c40 * P4
                    + Tsc * ((float)c01 + (float)c11 * Psc
                           + (float)c21 * P2 + (float)c31 * P3)) / 100.0f;

    /* 温度补偿 (°C) */
    spa06.temperature = (float)c0 * 0.5f + (float)c1 * Tsc;
}

void SPA06_ComputeAltitude(void)
{
    spa06.altitude = 44330.0f * (1.0f - powf(spa06.pressure / 1013.25f, 0.1903f));
}
