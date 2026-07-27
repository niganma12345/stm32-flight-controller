#include "mpu6050.h"
#include <stdio.h>

/* ===== 零偏校准值（Init 自动采样填充）===== */
static int16_t g_gx_ofs, g_gy_ofs, g_gz_ofs;
static int16_t g_ax_ofs, g_ay_ofs, g_az_ofs;

static void GetGyroRaw(int16_t *x, int16_t *y, int16_t *z);
static void GetAccRaw(int16_t *x, int16_t *y, int16_t *z);

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

    // 9. 自动零偏校准：静置采样 200 次取平均
    HAL_Delay(10);
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    for (int i = 0; i < 200; i++)
    {
        int16_t raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az;
        GetGyroRaw(&raw_gx, &raw_gy, &raw_gz);
        GetAccRaw(&raw_ax, &raw_ay, &raw_az);
        gx_sum += raw_gx; gy_sum += raw_gy; gz_sum += raw_gz;
        ax_sum += raw_ax; ay_sum += raw_ay; az_sum += raw_az;
        HAL_Delay(2);
    }

    g_gx_ofs = (int16_t)(gx_sum / 200);
    g_gy_ofs = (int16_t)(gy_sum / 200);
    g_gz_ofs = (int16_t)(gz_sum / 200);
    g_ax_ofs = (int16_t)(ax_sum / 200);
    g_ay_ofs = (int16_t)(ay_sum / 200);
    g_az_ofs = (int16_t)(az_sum / 200) - 16384;  /* Z 轴减去 1g */
}

/* ---- 内部：读原始 ADC 值（不含零偏）---- */
static void GetGyroRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t h, l;
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &l);
    *x = (int16_t)((h << 8) | l);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &l);
    *y = (int16_t)((h << 8) | l);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &l);
    *z = (int16_t)((h << 8) | l);
}

static void GetAccRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t h, l;
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &l);
    *x = (int16_t)((h << 8) | l);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &l);
    *y = (int16_t)((h << 8) | l);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &h); Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &l);
    *z = (int16_t)((h << 8) | l);
}

/* ---- 零偏修正后输出 ---- */
void Int_MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    int16_t x, y, z;
    GetGyroRaw(&x, &y, &z);
    gyro->gyro_x = x - g_gx_ofs;
    gyro->gyro_y = y - g_gy_ofs;
    gyro->gyro_z = z - g_gz_ofs;
}

/**
 * @brief 读取三轴加速度（使用硬编码零偏校准值）
 *
 * @param acc
 */
void Int_MPU6050_Get_Acc(Accel_struct *acc)
{
    int16_t x, y, z;
    GetAccRaw(&x, &y, &z);
    acc->accel_x = x - g_ax_ofs;
    acc->accel_y = y - g_ay_ofs;
    acc->accel_z = z - g_az_ofs;
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
