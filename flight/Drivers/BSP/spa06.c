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

/**
 * @brief 初始化 SPA06-003 气压计（DPS310 兼容）
 * @param  hi2c  I2C 句柄
 * @retval 0  成功
 * @retval 1  I2C 设备无响应
 * @retval 2  校准系数读取超时

 */
uint8_t SPA06_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[21],meas;
    uint8_t i;

    /* ---- 1. 检测 I2C 设备 ---- */
    if (HAL_I2C_IsDeviceReady(hi2c, SPA06_ADDR, 3, 50) != HAL_OK)
        return 1;

    /* ---- 2. 软复位，等待系数加载 ---- */
    wr(hi2c, SPA06_RESET, SPA06_RESET_CMD);
    HAL_Delay(50);

    for (i = 0; i < 50; i++) {
        HAL_Delay(10);
        rds(hi2c, SPA06_MEAS_CFG, &meas, 1);
        if (meas & SPA06_MEAS_COEF_RDY) break;
    }
    if (i >= 50) return 2;

    /* ---- 3. 读取校准系数（21 字节，DPS310 标准格式）---- */
    rds(hi2c, SPA06_COEF_BASE, buf, 21);

    /* 温度系数 (2个) */
    c0  = sgn12(((uint16_t)buf[0] << 4) | ((uint16_t)buf[1] >> 4));
    c1  = sgn12((((uint16_t)buf[1] & 0x0F) << 8) | (uint16_t)buf[2]);

    /* 气压系数 (5个) */
    c00 = sgn20(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4));
    c10 = sgn20(((uint32_t)(buf[5] & 0x0F) << 16) | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7]);
    c20 = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);
    c30 = (int16_t)(((uint16_t)buf[16] << 8) | (uint16_t)buf[17]);
    c40 = sgn12((((uint16_t)buf[19] & 0x0F) << 8) | (uint16_t)buf[20]);

    /* 温度-气压交叉系数 (4个) */
    c01 = (int16_t)(((uint16_t)buf[8]  << 8) | (uint16_t)buf[9]);
    c11 = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    c21 = (int16_t)(((uint16_t)buf[14] << 8) | (uint16_t)buf[15]);
    c31 = sgn12(((uint16_t)buf[18] << 4) | ((uint16_t)buf[19] >> 4));


    /* ---- 5. 配置并启动连续测量 ---- */
    /* 气压 16x 过采样 + 32Hz 速率 (单次转换 ~27.6ms) */
    wr(hi2c, SPA06_PRS_CFG, SPA06_OSR_16 | SPA06_RATE_32HZ);

    /* 温度  1x 过采样 + 32Hz 速率 (单次转换  ~3.6ms) */
    wr(hi2c, SPA06_TMP_CFG, SPA06_OSR_1  | SPA06_RATE_32HZ);

    /* 气压结果右移使能 (提高分辨率), 无 FIFO, 无中断 */
    wr(hi2c, SPA06_CFG_REG, 0x04);

    /* 启动：气压 + 温度 + 后台连续模式 */
    wr(hi2c, SPA06_MEAS_CFG, SPA06_MODE_CONTINUOUS);

    HAL_Delay(60);  /* 等第一帧测量完成 */
    return 0;
}

/**
 * @brief 读取 SPA06 传感器，一次完成温度、气压、海拔全部计算
 *
 * @param hi2c  I2C 句柄
 *
 * 输出（写入全局 spa06）：
 *   .temperature = 芯片温度 (°C)
 *   .pressure    = 绝对气压 (hPa)，出厂系数多项式补偿
 *   .altitude    = 绝对海拔 (m)，标准大气压公式，使用前需减去起飞基准得到相对高度
 */
void SPA06_Update(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6];
    int32_t Praw, Traw;
    float Psc, Tsc;

    /* ---- 1. 读气压+温度原始值（6字节原子读）---- */
    rds(hi2c, SPA06_PRS_B2, buf, 6);

    Praw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
    Traw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | (int32_t)buf[5];

    /* 24位 → 32位符号扩展 */
    if (Praw & 0x00800000) Praw |= 0xFF000000;
    if (Traw & 0x00800000) Traw |= 0xFF000000;

    Psc = (float)Praw / SPA06_KP;
    Tsc = (float)Traw / SPA06_KT;

    /* ---- 2. 温度补偿 (°C) ---- */
    spa06.temperature = (float)c0 * 0.5f + (float)c1 * Tsc;

    /* ---- 3. 气压补偿 (Pa → hPa) ---- */
    {
        float P2 = Psc * Psc;
        float P3 = P2 * Psc;
        float P4 = P3 * Psc;

        spa06.pressure = ((float)c00
                        + (float)c10 * Psc + (float)c20 * P2
                        + (float)c30 * P3 + (float)c40 * P4
                        + Tsc * ((float)c01 + (float)c11 * Psc
                               + (float)c21 * P2 + (float)c31 * P3)) / 100.0f;
    }

    /* ---- 4. 海拔转换 (标准大气压公式) ---- */
    spa06.altitude = 44330.0f * (1.0f - powf(spa06.pressure / 1013.25f, 0.1903f));
}
