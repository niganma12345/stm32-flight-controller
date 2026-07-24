/**
 * @file  vl53l1_platform.c
 * @brief VL53L1X 平台实现 — STM32 HAL I2C1 (PB6=SCL, PB7=SDA, 400kHz)
 */
#include "vl53l1_platform.h"
#include "i2c.h"
#include <string.h>

/* I2C1 句柄 (在 i2c.c 中定义) */
extern I2C_HandleTypeDef hi2c1;

/* 内部超时 ms */
#define I2C_TO  50

/*============================================================================*/
/* VL53L1_WriteMulti — 向 16-bit 寄存器写多个字节                                */
/*============================================================================*/
int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t buf[2 + count];
    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&buf[2], pdata, count);
    if (HAL_I2C_Master_Transmit(&hi2c1, dev, buf, (uint16_t)(2 + count), I2C_TO) != HAL_OK)
        return -1;
    return 0;
}

/*============================================================================*/
/* VL53L1_ReadMulti — 从 16-bit 寄存器读多个字节                                 */
/*============================================================================*/
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Read(&hi2c1, dev, index, I2C_MEMADD_SIZE_16BIT,
                          pdata, (uint16_t)count, I2C_TO) != HAL_OK)
        return -1;
    return 0;
}

/*============================================================================*/
/* 单字节写入                                                                    */
/*============================================================================*/
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

/*============================================================================*/
/* 双字节写入 (大端)                                                              */
/*============================================================================*/
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

/*============================================================================*/
/* 四字节写入 (大端)                                                              */
/*============================================================================*/
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)((data >> 16) & 0xFF);
    buf[2] = (uint8_t)((data >> 8) & 0xFF);
    buf[3] = (uint8_t)(data & 0xFF);
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

/*============================================================================*/
/* 单字节读取                                                                    */
/*============================================================================*/
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(dev, index, data, 1);
}

/*============================================================================*/
/* 双字节读取 (大端)                                                              */
/*============================================================================*/
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    int8_t ret = VL53L1_ReadMulti(dev, index, buf, 2);
    if (ret == 0)
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    return ret;
}

/*============================================================================*/
/* 四字节读取 (大端)                                                              */
/*============================================================================*/
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4];
    int8_t ret = VL53L1_ReadMulti(dev, index, buf, 4);
    if (ret == 0)
        *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
              | ((uint32_t)buf[2] << 8)  |  buf[3];
    return ret;
}

/*============================================================================*/
/* 毫秒延时                                                                      */
/*============================================================================*/
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    HAL_Delay((uint32_t)wait_ms);
    return 0;
}
