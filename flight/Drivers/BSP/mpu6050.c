#include "mpu6050.h"
#include <stdio.h>

/**
 * @brief 写寄存器
 *
 * @param reg 寄存器地址
 * @param data 寄存器的值
 */
void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    // HAL有固定的I2C读写函数
    // 1. 句柄(hi2c1) 2. 从设备地址(0x68) 3. 寄存器地址 reg 4. 寄存器地址的位数 5. 写入的数据地址 6. 写入的字节个数 7. 超时时间
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    // 1. 句柄(hi2c1) 2. 从设备地址(0x68) 3. 寄存器地址 reg 4. 寄存器地址的位数 5. 存放读取数据的地址 6. 读的字节个数 7. 超时时间
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

/**
 * @brief 零偏校准（仅需运行一次获取校准值）
 *
 *        将飞机水平放置后调用此函数，校准结果通过串口打印。
 *        拿到校准值后填入 mpu6050.h 中的 MPU6050_XXX_OFFSET 宏定义，
 *        之后无需再调用此函数。
 */
/**
 * @brief 初始化MPU6050芯片
 *
 */
void Int_MPU6050_Init(void)
{
    // 1. 重启芯片 重置所有寄存器的值 => 写电源管理寄存器1  => DEVICE_RESET
    Int_MPU6050_Write_Reg(0x6B, 0x80);
    uint8_t data = 0;
    // 重置完成之后 0x6B寄存器的值是0x40 表示当前为低功耗模式
    while (data != 0x40)
    {
        Int_MPU6050_Read_Reg(0x6B, &data);
    }
    // 唤醒MPU6050  进入到正常工作状态
    Int_MPU6050_Write_Reg(0x6B, 0x00);

    // 2. 选择合适的量程 => 在够用的范围内 选择的越小越好 => 精度高
    // 2.1 填写角速度量程为±2000°/s
    Int_MPU6050_Write_Reg(0x1B, 3 << 3);

    // 2.2 填写加速度量程为±2g
    Int_MPU6050_Write_Reg(0x1C, 0x00);

    // 3. 关闭中断使能  因为用不到中断
    Int_MPU6050_Write_Reg(0x38, 0x00);

    // 4. 用户配置寄存器 不使用FIFO队列  不使用扩展的I2C
    Int_MPU6050_Write_Reg(0x6A, 0x00);

    // 5. 设置采样频率 => 陀螺仪监控三轴加速度和三轴角速度 => 默认频率 1000HZ => 1ms读取一次
    // 基本逻辑 => 采样率必须大于后续数据的使用频率  否则失真 => 香农定理 采样率 >= 2倍使用频率
    // 设置采样分频为2 => 填写的值就是2-1
    Int_MPU6050_Write_Reg(0x19, 0x01);

    // 6. 设置低通滤波的值为184Hz 188Hz => 1
    Int_MPU6050_Write_Reg(0x1A, 1);

    // 7. 配置使用的系统时钟为添加PLL的
    Int_MPU6050_Write_Reg(0x6B, 0x01);

    // 8. 使能加速度传感器和角速度传感器
    Int_MPU6050_Write_Reg(0x6C, 0x00);

    // 9. 零偏校准已改为硬编码宏定义（见 mpu6050.h 顶部），无需在此调用
//     Int_MPU6050_calculate_offset();
}

/**
 * @brief 读取三轴角速度（使用硬编码零偏校准值）
 *
 * @param gyro
 */
void Int_MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    // 存储角速度的寄存器地址从0x43开始 高8位在前  XYZ的顺序
    uint8_t hight = 0;
    uint8_t low = 0;
    // X轴
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    gyro->gyro_x = (hight << 8 | low) - MPU6050_GYRO_X_OFFSET;
    // Y轴
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
    gyro->gyro_y = (hight << 8 | low) - MPU6050_GYRO_Y_OFFSET;
    // Z轴
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
    gyro->gyro_z = (hight << 8 | low) - MPU6050_GYRO_Z_OFFSET;
}

/**
 * @brief 读取三轴加速度（使用硬编码零偏校准值）
 *
 * @param acc
 */
void Int_MPU6050_Get_Acc(Accel_struct *acc)
{
    uint8_t hight = 0;
    uint8_t low = 0;
    // X轴
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    acc->accel_x = (hight << 8 | low) - MPU6050_ACC_X_OFFSET;
    // Y轴
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    acc->accel_y = (hight << 8 | low) - MPU6050_ACC_Y_OFFSET;
    // Z轴
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &hight);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    acc->accel_z = (hight << 8 | low) - MPU6050_ACC_Z_OFFSET;
}

/**
 * @brief 获取所有的六轴数据
 *
 * @param data
 */
void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    Int_MPU6050_Get_Gyro(&data->gyro);
    Int_MPU6050_Get_Acc(&data->accel);
}
