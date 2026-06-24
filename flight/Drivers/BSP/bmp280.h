#ifndef __BMP280_H__
#define __BMP280_H__

#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

// BMP280 I2C 地址 (SDO接GND=0x76, SDO接VDD=0x77)
#define BMP280_ADDR_WRITE 0xEC
#define BMP280_ADDR_READ  0xED

// BMP280 寄存器地址
#define BMP280_REG_CHIP_ID      0xD0  // 芯片ID寄存器（应读出0x58）
#define BMP280_REG_RESET        0xE0  // 软复位寄存器
#define BMP280_REG_STATUS       0xF3  // 状态寄存器
#define BMP280_REG_CTRL_MEAS    0xF4  // 测量控制寄存器
#define BMP280_REG_CONFIG       0xF5  // 配置寄存器
#define BMP280_REG_PRESS_MSB    0xF7  // 气压数据 MSB
#define BMP280_REG_PRESS_LSB    0xF8  // 气压数据 LSB
#define BMP280_REG_PRESS_XLSB   0xF9  // 气压数据 XLSB
#define BMP280_REG_TEMP_MSB     0xFA  // 温度数据 MSB
#define BMP280_REG_TEMP_LSB     0xFB  // 温度数据 LSB
#define BMP280_REG_TEMP_XLSB    0xFC  // 温度数据 XLSB

// BMP280 模式选择
#define BMP280_MODE_SLEEP   0x00  // 休眠模式
#define BMP280_MODE_FORCED  0x01  // 强制模式（单次测量后进入休眠）
#define BMP280_MODE_NORMAL  0x03  // 正常模式（持续测量）

// BMP280 过采样配置
#define BMP280_OVERSAMP_SKIP  0x00  // 跳过
#define BMP280_OVERSAMP_1X    0x01  // ×1
#define BMP280_OVERSAMP_2X    0x02  // ×2
#define BMP280_OVERSAMP_4X    0x03  // ×4
#define BMP280_OVERSAMP_8X    0x04  // ×8
#define BMP280_OVERSAMP_16X   0x05  // ×16

// BMP280 IIR 滤波器配置
#define BMP280_FILTER_OFF  0x00  // 关闭滤波器
#define BMP280_FILTER_2    0x01  // 系数2
#define BMP280_FILTER_4    0x02  // 系数4
#define BMP280_FILTER_8    0x03  // 系数8
#define BMP280_FILTER_16   0x04  // 系数16

// BMP280 待机时间配置（正常模式下的采样间隔）
#define BMP280_STANDBY_0_5MS   0x00
#define BMP280_STANDBY_62_5MS  0x01
#define BMP280_STANDBY_125MS   0x02
#define BMP280_STANDBY_250MS   0x03
#define BMP280_STANDBY_500MS   0x04
#define BMP280_STANDBY_1000MS  0x05
#define BMP280_STANDBY_2000MS  0x06
#define BMP280_STANDBY_4000MS  0x07

// BMP280 校准参数结构体（从芯片NVM读取）
typedef struct
{
    uint16_t dig_T1;    // 温度校准 T1
    int16_t  dig_T2;    // 温度校准 T2
    int16_t  dig_T3;    // 温度校准 T3
    uint16_t dig_P1;    // 气压校准 P1
    int16_t  dig_P2;    // 气压校准 P2
    int16_t  dig_P3;    // 气压校准 P3
    int16_t  dig_P4;    // 气压校准 P4
    int16_t  dig_P5;    // 气压校准 P5
    int16_t  dig_P6;    // 气压校准 P6
    int16_t  dig_P7;    // 气压校准 P7
    int16_t  dig_P8;    // 气压校准 P8
    int16_t  dig_P9;    // 气压校准 P9
} BMP280_Calib;

// 用于64位计算的有符号温度值
typedef int32_t BMP280_S32_t;

/**
 * @brief 初始化BMP280芯片
 *
 * @return uint8_t 0:初始化成功  1:初始化失败
 */
uint8_t BMP280_Init(void);

/**
 * @brief 读取BMP280原始温度值（ADC原始值）
 *
 * @return int32_t 原始温度值（含校准参数的补偿计算中间值）
 */
int32_t BMP280_Get_Temperature_Raw(void);

/**
 * @brief 读取BMP280原始气压值（ADC原始值）
 *
 * @return int32_t 原始气压值
 */
int32_t BMP280_Get_Pressure_Raw(void);

/**
 * @brief 读取并计算补偿后的温度值
 *
 * @return float 温度值（单位: ℃）
 */
float BMP280_Get_Temperature(void);

/**
 * @brief 读取并计算补偿后的气压值
 *
 * @return float 气压值（单位: Pa）
 */
float BMP280_Get_Pressure(void);

/**
 * @brief 读取并计算当前海拔高度
 *
 * 基于标准大气压公式，使用海平面气压作为基准
 * @param sea_level_pressure 海平面参考气压（单位: Pa），常用值 101325.0f
 * @return float 海拔高度（单位: m）
 */
float BMP280_Get_Altitude(float sea_level_pressure);

/**
 * @brief 设置海平面参考气压
 *
 * @param ref_pressure 参考气压值（单位: Pa）
 */
void BMP280_Set_SeaLevel_Pressure(float ref_pressure);

/**
 * @brief 获取当前海平面参考气压
 *
 * @return float 海平面参考气压（单位: Pa）
 */
float BMP280_Get_SeaLevel_Pressure(void);

/**
 * @brief 通过当前气压自动校准海平面气压
 *
 * 在地面调用此函数以获得准确的海平面参考气压
 * @param known_altitude 已知的当前海拔高度（单位: m），默认0
 * @return float 校准后的海平面气压（单位: Pa）
 */
float BMP280_Calibrate_SeaLevel(float known_altitude);

/**
 * @brief 软复位BMP280
 */
void BMP280_SoftReset(void);

#endif // __BMP280_H__
