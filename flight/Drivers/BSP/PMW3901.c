#include "PMW3901.h"
#include "spi.h"
#include <string.h>

/* ---- 内部 ---- */
static void CS_L(void)  { HAL_GPIO_WritePin(PMW3901_CSN_PORT, PMW3901_CSN_PIN, GPIO_PIN_RESET); }
static void CS_H(void)  { HAL_GPIO_WritePin(PMW3901_CSN_PORT, PMW3901_CSN_PIN, GPIO_PIN_SET); }
static uint8_t Spi(uint8_t d)
{
    uint8_t r = 0;
    HAL_SPI_TransmitReceive(PMW3901_SPI, &d, &r, 1, PMW3901_SPI_TIMEOUT_MS);
    return r;
}

/* ---- μs 延时 (72MHz, ~15次循环/μs) ---- */
static void DlyUs(uint32_t us)
{
    volatile uint32_t n = us * 15;
    while (n--) __NOP();
}

/* ---- 寄存器读写 ---- */
/* t_SRAD ≥ 35μs (数据手册 Table 5) */
static uint8_t Rd(uint8_t reg)
{
    uint8_t v;
    CS_L();
    Spi(reg & 0x7F);
    DlyUs(40);   /* ~40μs 满足 t_SRAD, 不消费传感器数据 */
    v = Spi(0x00);
    CS_H();
    return v;
}

static void Wr(uint8_t reg, uint8_t val)
{
    CS_L();
    Spi(reg | 0x80);
    Spi(val);
    CS_H();
    HAL_Delay(1);
}


static void InitSeq(void)
{
    /* Bank 0 */
    Wr(0x7F, 0x00);
    Wr(0x61, 0xAD);

    /* Bank 3 */
    Wr(0x7F, 0x03);
    Wr(0x40, 0x00);

    /* Bank 5 */
    Wr(0x7F, 0x05);
    Wr(0x41, 0xB3);
    Wr(0x43, 0xF1);
    Wr(0x45, 0x14);
    Wr(0x5B, 0x32);
    Wr(0x5F, 0x34);
    Wr(0x7B, 0x08);

    /* Bank 6 */
    Wr(0x7F, 0x06);
    Wr(0x44, 0x1B);
    Wr(0x40, 0xBF);
    Wr(0x4E, 0x3F);

    /* Bank 8 */
    Wr(0x7F, 0x08);
    Wr(0x65, 0x20);
    Wr(0x6A, 0x18);

    /* Bank 9 */
    Wr(0x7F, 0x09);
    Wr(0x4F, 0xAF);
    Wr(0x5F, 0x40);
    Wr(0x48, 0x80);
    Wr(0x49, 0x80);
    Wr(0x57, 0x77);
    Wr(0x60, 0x78);
    Wr(0x61, 0x78);
    Wr(0x62, 0x08);
    Wr(0x63, 0x50);

    /* Bank A */
    Wr(0x7F, 0x0A);
    Wr(0x45, 0x60);

    /* Bank 0 */
    Wr(0x7F, 0x00);
    Wr(0x4D, 0x11);
    Wr(0x55, 0x80);
    Wr(0x74, 0x1F);
    Wr(0x75, 0x1F);
    Wr(0x4A, 0x78);
    Wr(0x4B, 0x78);
    Wr(0x44, 0x08);
    Wr(0x45, 0x50);
    Wr(0x64, 0xFF);
    Wr(0x65, 0x1F);

    /* Bank 0x14 */
    Wr(0x7F, 0x14);
    Wr(0x65, 0x67);
    Wr(0x66, 0x08);
    Wr(0x63, 0x70);

    /* Bank 0x15 */
    Wr(0x7F, 0x15);
    Wr(0x48, 0x48);

    /* Bank 7 */
    Wr(0x7F, 0x07);
    Wr(0x41, 0x0D);
    Wr(0x43, 0x14);
    Wr(0x4B, 0x0E);
    Wr(0x45, 0x0F);
    Wr(0x44, 0x42);
    Wr(0x4C, 0x80);

    /* Bank 0x10 */
    Wr(0x7F, 0x10);
    Wr(0x5B, 0x02);

    /* Bank 7 */
    Wr(0x7F, 0x07);
    Wr(0x40, 0x41);
    Wr(0x70, 0x00);

    HAL_Delay(10);

    /* Bank 0 */
    Wr(0x7F, 0x00);
    Wr(0x32, 0x44);

    /* Bank 7 */
    Wr(0x7F, 0x07);
    Wr(0x40, 0x40);

    /* Bank 6 */
    Wr(0x7F, 0x06);
    Wr(0x62, 0xF0);
    Wr(0x63, 0x00);

    /* Bank 0x0D */
    Wr(0x7F, 0x0D);
    Wr(0x48, 0xC0);
    Wr(0x6F, 0xD5);

    /* Bank 0 */
    Wr(0x7F, 0x00);
    Wr(0x5B, 0xA0);
    Wr(0x4E, 0xA8);
    Wr(0x5A, 0x50);
    Wr(0x40, 0x80);

    /* Bank 0 最终 */
    Wr(0x7F, 0x00);
    Wr(0x5A, 0x10);
    Wr(0x54, 0x00);
}

/*============================================================================*/
HAL_StatusTypeDef PMW3901_Init(void)
{
    uint8_t id, i;

    CS_H();

    /* ---- 硬件复位 (RES 引脚) ---- */
    HAL_GPIO_WritePin(PMW3901_RES_PORT, PMW3901_RES_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(PMW3901_RES_PORT, PMW3901_RES_PIN, GPIO_PIN_SET);
    HAL_Delay(50);  /* 等待传感器上电稳定 */

    /* 重配 SPI2: Mode 3, ~1.125MHz */
    HAL_SPI_DeInit(PMW3901_SPI);
    PMW3901_SPI->Init.Mode              = SPI_MODE_MASTER;
    PMW3901_SPI->Init.Direction         = SPI_DIRECTION_2LINES;
    PMW3901_SPI->Init.DataSize          = SPI_DATASIZE_8BIT;
    PMW3901_SPI->Init.CLKPolarity       = SPI_POLARITY_HIGH;
    PMW3901_SPI->Init.CLKPhase          = SPI_PHASE_2EDGE;
    PMW3901_SPI->Init.NSS               = SPI_NSS_SOFT;
    PMW3901_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    PMW3901_SPI->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    PMW3901_SPI->Init.TIMode            = SPI_TIMODE_DISABLE;
    PMW3901_SPI->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    PMW3901_SPI->Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(PMW3901_SPI) != HAL_OK) return HAL_ERROR;

    /* CS 切换 唤醒 */
    CS_H(); HAL_Delay(2);
    CS_L(); HAL_Delay(2);
    CS_H(); HAL_Delay(2);

    /* 验证 Product_ID */
    id = Rd(PMW3901_REG_Product_ID);
    if (id != PMW3901_PRODUCT_ID_VAL)
    {
        for (i = 0; i < 3; i++) { HAL_Delay(10); id = Rd(0x00); if (id == 0x49) break; }
        if (id != 0x49) return HAL_ERROR;
    }

    /* Power-up reset */
    Wr(0x3A, 0x5A);
    HAL_Delay(60);

    /* 清残留 */
    Rd(0x02); Rd(0x03); Rd(0x04); Rd(0x05); Rd(0x06);
    HAL_Delay(1);

    /* 执行完整初始化 */
    InitSeq();

    /* 显式设置分辨率: 160 CPI (光流常用) */
    Wr(0x7F, 0x00);
    Wr(0x4C, PMW3901_RESOLUTION_160CPI);

    /* 唤醒 */
    Wr(0x3B, 0x01);  /* Shutdown 唤醒 */
    HAL_Delay(10);

    return HAL_OK;
}

/*============================================================================*/
HAL_StatusTypeDef PMW3901_ReadMotion(PMW3901_MotionData_t *pData)
{
    if (!pData) return HAL_ERROR;
    memset(pData, 0, sizeof(*pData));

    /* 前提: 调用者确保当前在 Bank 0. 此处不切 Bank 以节省 ~1ms */
    /* Motion_Burst (0x16) 突发读取 12 字节:
     * [0]Motion [1]Observation [2]DX_L [3]DX_H [4]DY_L [5]DY_H
     * [6]SQUAL [7]RawSum [8]MaxRaw [9]MinRaw [10]Sht_L [11]Sht_H
     *
     * SPI 全双工: 发送地址字节的同时 MISO 返回 1 字节 (丢弃),
     * 之后连续 12 个 dummy 字节读取传感器锁存的数据.
     */
    uint8_t buf[12];

    CS_L();
    Spi(0x16 & 0x7F);       /* 发送地址, 丢弃同时收到的字节 (非帧数据) */
    DlyUs(40);               /* t_SRAD ≥ 35μs */
    for (int i = 0; i < 12; i++)
        buf[i] = Spi(0x00);
    CS_H();
    DlyUs(50);               /* CS 释放后留恢复时间 */

    pData->motion      = buf[0];
    pData->observation = buf[1];
    pData->delta_x     = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    pData->delta_y     = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    pData->squal       = buf[6];
    pData->rawdata_sum = buf[7];
    pData->max_rawdata = buf[8];
    pData->min_rawdata = buf[9];
    /* 快门字节: Burst Byte10/11 某些模块高低字节与手册相反,
     * 使用 (Byte10 << 8) | Byte11 得到正确值          */
    pData->shutter     = ((uint16_t)buf[10] << 8) | buf[11];
    pData->is_motion   = (buf[0] & 0x80) != 0;
    pData->is_overflow = (buf[0] & 0x10) != 0;

    return HAL_OK;
}

/* ---- 其余未变 ---- */
bool PMW3901_MotionDetected(void)
{
    return HAL_GPIO_ReadPin(PMW3901_MOTION_PORT, PMW3901_MOTION_PIN) == GPIO_PIN_SET;
}

void PMW3901_SetResolution(uint8_t r)
{
    if (r > PMW3901_RESOLUTION_5120CPI) r = PMW3901_RESOLUTION_160CPI;
    /* 切到 Bank 0 */
    Wr(0x7F, 0x00);
    Wr(0x4C, r);
}

void PMW3901_SetShutdown(bool s)
{
    Wr(0x7F, 0x00);
    Wr(0x3B, s ? 0x00 : 0x01);
    if (!s) HAL_Delay(5);
}

/* 公开的寄存器读写 (始终切到 Bank 0) */
uint8_t PMW3901_ReadReg(uint8_t reg)
{
    Wr(0x7F, 0x00);       /* 确保在 Bank 0 */
    return Rd(reg);
}

void PMW3901_WriteReg(uint8_t reg, uint8_t val)
{
    Wr(0x7F, 0x00);       /* 确保在 Bank 0 */
    Wr(reg, val);
}
