#include "mpu6050.h"
#include "math.h"

MPU6050_TypeDef mpu;

// 写寄存器
static void MPU_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t dat)
{
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &dat, 1, 100);
}

// 初始化
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    MPU_WriteReg(hi2c, MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(50);

    MPU_WriteReg(hi2c, MPU6050_SMPLRT_DIV, 0x07);
    MPU_WriteReg(hi2c, MPU6050_CONFIG, 0x06);
    MPU_WriteReg(hi2c, MPU6050_GYRO_CONFIG, 0x00);
    MPU_WriteReg(hi2c, MPU6050_ACCEL_CONFIG, 0x00);

    return 0;
}

// 读取原始数据
void MPU6050_ReadData(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buf, 14, 100);

    mpu.ax = (buf[0] << 8) | buf[1];
    mpu.ay = (buf[2] << 8) | buf[3];
    mpu.az = (buf[4] << 8) | buf[5];
    mpu.temp = (buf[6] << 8) | buf[7];
    mpu.gx = (buf[8] << 8) | buf[9];
    mpu.gy = (buf[10] << 8) | buf[11];
    mpu.gz = (buf[12] << 8) | buf[13];
}

// 计算姿态角，放大10倍用整数显示
void MPU6050_ComputeAngle(void)
{
    float ax = mpu.ax;
    float ay = mpu.ay;
    float az = mpu.az;

    float pitch = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / 3.14159f;
    float roll  = atan2(-ax, az) * 180.0f / 3.14159f;

    mpu.pitch = (int)(pitch * 10);
    mpu.roll  = (int)(roll  * 10);
}
