#include "mpu6050.h"

/* ---- 零偏校准值（Init 自动采样）---- */
int32_t acc_x_offset  = 0;
int32_t acc_y_offset  = 0;
int32_t acc_z_offset  = 0;
int32_t gyro_x_offset = 0;
int32_t gyro_y_offset = 0;
int32_t gyro_z_offset = 0;

void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

/**
 * @brief 零偏校准
 *
 * 等飞机静止 → 采样100次平均 → 填入6轴偏移量。
 * Init 末尾自动调用，无需外部介入。
 */
static void Int_MPU6050_calculate_offset(void)
{
    /* ---- 1. 等飞机静止：连续 100 次三轴加速度抖动 < 400 ADC ---- */
    Accel_struct cur = {0}, last = {0};
    uint8_t stable_cnt = 0;

    Int_MPU6050_Get_Acc(&last);
    while (stable_cnt < 100)
    {
        Int_MPU6050_Get_Acc(&cur);
        if (abs(cur.accel_x - last.accel_x) < 400 &&
            abs(cur.accel_y - last.accel_y) < 400 &&
            abs(cur.accel_z - last.accel_z) < 400)
            stable_cnt++;
        else
            stable_cnt = 0;
        last = cur;
        HAL_Delay(6);   /* 不用 vTaskDelay：I2C2 与 OLED 共享，防切换乱状态 */
    }

    /* ---- 2. 静止后采样 100 次取平均 ---- */
    Gyro_Accel_Struct raw = {0};
    int32_t acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        Int_MPU6050_Get_Data(&raw);
        acc_x_sum += (raw.accel.accel_x - 0);
        acc_y_sum += (raw.accel.accel_y - 0);
        acc_z_sum += (raw.accel.accel_z - 16384);
        gx_sum    += (raw.gyro.gyro_x - 0);
        gy_sum    += (raw.gyro.gyro_y - 0);
        gz_sum    += (raw.gyro.gyro_z - 0);
        HAL_Delay(6);
    }

    acc_x_offset  = acc_x_sum / 100;
    acc_y_offset  = acc_y_sum / 100;
    acc_z_offset  = acc_z_sum / 100;
    gyro_x_offset = gx_sum / 100;
    gyro_y_offset = gy_sum / 100;
    gyro_z_offset = gz_sum / 100;
}

void Int_MPU6050_Init(void)
{
    Int_MPU6050_Write_Reg(0x6B, 0x80);
    uint8_t data = 0;
    while (data != 0x40) Int_MPU6050_Read_Reg(0x6B, &data);
    Int_MPU6050_Write_Reg(0x6B, 0x00);

    Int_MPU6050_Write_Reg(0x1B, 3 << 3);     /* ±2000°/s */
    Int_MPU6050_Write_Reg(0x1C, 0x00);       /* ±2g      */
    Int_MPU6050_Write_Reg(0x38, 0x00);       /* 关中断   */
    Int_MPU6050_Write_Reg(0x6A, 0x00);       /* 关FIFO   */
    Int_MPU6050_Write_Reg(0x19, 0x01);       /* 分频2    */
    Int_MPU6050_Write_Reg(0x1A, 1);          /* LPF 184Hz */
    Int_MPU6050_Write_Reg(0x6B, 0x01);       /* PLL 时钟 */
    Int_MPU6050_Write_Reg(0x6C, 0x00);       /* 使能传感器 */

    Int_MPU6050_calculate_offset();
}

void Int_MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    uint8_t h = 0, l = 0;
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &l);
    gyro->gyro_x = (h << 8 | l) - gyro_x_offset;
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &l);
    gyro->gyro_y = (h << 8 | l) - gyro_y_offset;
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &l);
    gyro->gyro_z = (h << 8 | l) - gyro_z_offset;
}

void Int_MPU6050_Get_Acc(Accel_struct *acc)
{
    uint8_t h = 0, l = 0;
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &l);
    acc->accel_x = (h << 8 | l) - acc_x_offset;
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &l);
    acc->accel_y = (h << 8 | l) - acc_y_offset;
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &l);
    acc->accel_z = (h << 8 | l) - acc_z_offset;
}

void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    Int_MPU6050_Get_Gyro(&data->gyro);
    Int_MPU6050_Get_Acc(&data->accel);
}
