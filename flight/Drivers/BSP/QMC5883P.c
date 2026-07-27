#include "qmc5883p.h"
#include "com_debug.h"
#include <math.h>

QMC5883P_TypeDef qmc;
uint8_t g_qmc_ok = 0;    /* 驱动初始化成功标记 */

/* ========== 内部辅助函数 ========== */

/* 写单个寄存器 */
static void QMC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t dat)
{
    HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &dat, 1, 100);
}

/* 读多个寄存器 */
static void QMC_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* ========== 公开接口 ========== */

/**
  * @brief  QMC5883P 初始化
  * @param  hi2c: I2C 句柄指针
  * @retval 0=成功, 1=失败
  * @note   配置: 连续模式, 200Hz ODR, ±8G 量程
  */
uint8_t QMC5883P_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id;

    /* 确认设备是否存在 */
    if (HAL_I2C_IsDeviceReady(hi2c, QMC5883P_ADDR, 3, 50) != HAL_OK)
    {
        debug_printf("QMC5883P not found!\r\n");
        return 1;
    }

    /* 读取芯片 ID */
    QMC_ReadRegs(hi2c, QMC5883P_CHIP_ID, &chip_id, 1);

    /* 内部配置初始化 */
    QMC_WriteReg(hi2c, QMC5883P_CTRL3, 0x40);
    QMC_WriteReg(hi2c, QMC5883P_CTRL4, 0x06);

    /* 设置连续模式 + 200Hz ODR */
    QMC_WriteReg(hi2c, QMC5883P_CTRL1,
                 QMC5883P_MODE_CONT | QMC5883P_ODR_200HZ);

    /* 设置量程 ±8G */
    QMC_WriteReg(hi2c, QMC5883P_CTRL2, QMC5883P_RANGE_8G);

    /* 初始化 min/max 跟踪 */
    qmc.mx_min =  32767;
    qmc.mx_max = -32768;
    qmc.my_min =  32767;
    qmc.my_max = -32768;

    g_qmc_ok = 1;
    return 0;
}

/**
  * @brief  读取地磁原始数据
  * @param  hi2c: I2C 句柄指针
  *
  * 数据格式: 小端序 (LSB 在前)
  * ±8G 灵敏度: 3750 LSB/G
  */
void QMC5883P_ReadData(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6];

    /* 读取 6 字节: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
    QMC_ReadRegs(hi2c, QMC5883P_DATA_X_LSB, buf, 6);

    /* 组装为有符号 16-bit (小端序) */
    qmc.mx = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    qmc.my = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    qmc.mz = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);
}

/**
  * @brief  根据 X/Y 磁通量计算航向角（硬编码偏移校准）
  *         调用前需先调用 QMC5883P_ReadData()
  */
void QMC5883P_ComputeHeading(void)
{
    /* 实时 min/max 跟踪（校准采集用） */
    if (qmc.mx < qmc.mx_min) qmc.mx_min = qmc.mx;
    if (qmc.mx > qmc.mx_max) qmc.mx_max = qmc.mx;
    if (qmc.my < qmc.my_min) qmc.my_min = qmc.my;
    if (qmc.my > qmc.my_max) qmc.my_max = qmc.my;

    /* 硬编码偏移修正 + 航向计算 */
    float mx = (float)qmc.mx - (float)QMC_HARD_OFS_X;
    float my = (float)qmc.my - (float)QMC_HARD_OFS_Y;

    float heading = atan2f(my, mx) * 57.29578f;
    if (heading < 0.0f) heading += 360.0f;

    qmc.heading = heading;
}
