#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

// MPU6050 I2C地址 AD0=GND
#define MPU6050_ADDR     (0x68 << 1)

// 寄存器定义
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_TEMP_OUT_H     0x41

// 数据结构体
typedef struct
{
    int16_t ax, ay, az;    // 原始加速度
    int16_t gx, gy, gz;    // 原始陀螺仪
    int16_t temp;          // 原始温度

    int roll;              // 横滚角 *10，用整数显示
    int pitch;             // 俯仰角 *10
} MPU6050_TypeDef;

extern MPU6050_TypeDef mpu;

// 函数声明
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void    MPU6050_ReadData(I2C_HandleTypeDef *hi2c);
void    MPU6050_ComputeAngle(void);

#endif
