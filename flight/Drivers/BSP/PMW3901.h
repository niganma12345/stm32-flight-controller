/**
 ******************************************************************************
 * @file    pmw3901.h
 * @brief   PMW3901 光流传感器 BSP 驱动头文件
 * @note    使用 SPI2 通信, MOTION(PA9) 中断输入, RES(PA15) 复位输出
 ******************************************************************************
 */

#ifndef __PMW3901_H__
#define __PMW3901_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* PMW3901 引脚定义 (SPI2 软件CS)                                              */
/*============================================================================*/
#define PMW3901_CSN_PIN         SPI2_CS_Pin
#define PMW3901_CSN_PORT        SPI2_CS_GPIO_Port
#define PMW3901_MOTION_PIN      MOTION_Pin
#define PMW3901_MOTION_PORT     MOTION_GPIO_Port
#define PMW3901_RES_PIN         RES_3901_Pin
#define PMW3901_RES_PORT        RES_3901_GPIO_Port

/* SPI2 句柄 (在 spi.c 中定义) */
#define PMW3901_SPI             (&hspi2)

/*============================================================================*/
/* PMW3901 寄存器映射                                                          */
/*============================================================================*/
#define PMW3901_REG_Product_ID          0x00    /* 产品ID (应为 0x49)           */
#define PMW3901_REG_Revision_ID         0x01    /* 版本ID                       */
#define PMW3901_REG_Motion              0x02    /* 运动检测状态                  */
#define PMW3901_REG_Delta_X_L           0x03    /* X轴位移低字节                 */
#define PMW3901_REG_Delta_X_H           0x04    /* X轴位移高字节                 */
#define PMW3901_REG_Delta_Y_L           0x05    /* Y轴位移低字节                 */
#define PMW3901_REG_Delta_Y_H           0x06    /* Y轴位移高字节                 */
#define PMW3901_REG_SQUAL               0x07    /* 表面质量 (Surface Quality)    */
#define PMW3901_REG_RawData_Sum         0x08    /* 原始数据总和                   */
#define PMW3901_REG_Max_RawData         0x09    /* 最大原始数据                   */
#define PMW3901_REG_Min_RawData         0x0A    /* 最小原始数据                   */
#define PMW3901_REG_Shutter_Lower       0x0B    /* 快门值低字节                   */
#define PMW3901_REG_Shutter_Upper       0x0C    /* 快门值高字节                   */
#define PMW3901_REG_Observation         0x15    /* 观测寄存器                     */
#define PMW3901_REG_Motion_Burst        0x16    /* 运动Burst读取                  */
#define PMW3901_REG_Power_Up_Reset      0x3A    /* 上电复位寄存器                 */
#define PMW3901_REG_Shutdown            0x3B    /* 关断寄存器                     */
#define PMW3901_REG_Inverse_Product_ID  0x3F    /* 反相产品ID (应为 0xB6)         */
#define PMW3901_REG_Motion_Burst_Alt    0x40    /* 运动Burst读取 (备用地址)        */
#define PMW3901_REG_Resolution          0x4C    /* 分辨率配置                     */
#define PMW3901_REG_Config2             0x4E    /* 配置寄存器2                    */

/*============================================================================*/
/* 寄存器位定义                                                                */
/*============================================================================*/
/* Motion 寄存器 (0x02) */
#define PMW3901_MOTION_MOT              0x80    /* 运动检测标志 (1=有运动)       */
#define PMW3901_MOTION_Ovf              0x10    /* 数据溢出标志                   */

/* Power_Up_Reset 寄存器 (0x3A) */
#define PMW3901_POWER_UP_RESET_CMD      0x5A    /* 强制复位命令                   */

/* Shutdown 寄存器 (0x3B) */
#define PMW3901_SHUTDOWN_ACTIVE         0x00    /* 进入关断模式                   */
#define PMW3901_SHUTDOWN_WAKEUP         0x01    /* 唤醒传感器                     */

/* Resolution 寄存器 (0x4C) */
#define PMW3901_RESOLUTION_40CPI        0x00    /* 40 counts per inch (默认)     */
#define PMW3901_RESOLUTION_80CPI        0x01
#define PMW3901_RESOLUTION_160CPI       0x02
#define PMW3901_RESOLUTION_320CPI       0x03
#define PMW3901_RESOLUTION_640CPI       0x04
#define PMW3901_RESOLUTION_1280CPI      0x05
#define PMW3901_RESOLUTION_2560CPI      0x06
#define PMW3901_RESOLUTION_5120CPI      0x07

/*============================================================================*/
/* 常量定义                                                                    */
/*============================================================================*/
#define PMW3901_PRODUCT_ID_VAL          0x49    /* 期望的产品ID值                */
#define PMW3901_INV_PRODUCT_ID_VAL      0xB6    /* 期望的反相产品ID值             */
#define PMW3901_BURST_READ_LEN          12      /* Motion_Burst 读取字节数        */
#define PMW3901_SPI_TIMEOUT_MS          100     /* SPI 超时时间(ms)               */

/*============================================================================*/
/* 运动数据结构体                                                              */
/*============================================================================*/
typedef struct {
    uint8_t  motion;        /* Motion 寄存器原始值                             */
    uint8_t  observation;   /* 观测值                                          */
    int16_t  delta_x;       /* X 轴位移 (有符号)                               */
    int16_t  delta_y;       /* Y 轴位移 (有符号)                               */
    uint8_t  squal;         /* 表面质量 (0~255, 越大越好)                       */
    uint8_t  rawdata_sum;   /* 原始数据总和                                    */
    uint8_t  max_rawdata;   /* 最大原始数据                                    */
    uint8_t  min_rawdata;   /* 最小原始数据                                    */
    uint16_t shutter;       /* 快门值 (积分时间)                                */
    bool     is_motion;     /* 是否有运动 (true = 运动)                         */
    bool     is_overflow;   /* 是否溢出 (true = 溢出)                           */
} PMW3901_MotionData_t;

/*============================================================================*/
/* 函数原型                                                                    */
/*============================================================================*/

/**
 * @brief  初始化 PMW3901 传感器
 * @note   配置 SPI2 (Mode 3), 软件 CS, 硬件复位, 验证 Product_ID
 * @retval HAL_OK=成功, 其他=失败
 */
HAL_StatusTypeDef PMW3901_Init(void);

/**
 * @brief  读取单次运动数据 (Burst Read)
 * @param  pData: 指向 MotionData 结构体的指针
 * @retval HAL_OK=成功, 其他=失败
 */
HAL_StatusTypeDef PMW3901_ReadMotion(PMW3901_MotionData_t *pData);

/**
 * @brief  读取指定寄存器
 * @param  reg: 寄存器地址
 * @retval 寄存器值
 */
uint8_t PMW3901_ReadReg(uint8_t reg);

/**
 * @brief  写入指定寄存器
 * @param  reg: 寄存器地址
 * @param  val: 要写入的值
 */
void PMW3901_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief  检测是否有运动发生 (轮询 MOTION 引脚)
 * @retval true=有运动, false=无运动
 */
bool PMW3901_MotionDetected(void);

/**
 * @brief  设置传感器分辨率
 * @param  resolution: 分辨率等级 (如 PMW3901_RESOLUTION_160CPI)
 */
void PMW3901_SetResolution(uint8_t resolution);

/**
 * @brief  进入/退出关断模式
 * @param  shutdown: true=关断, false=唤醒
 */
void PMW3901_SetShutdown(bool shutdown);


#ifdef __cplusplus
}
#endif

#endif /* __PMW3901_H__ */
