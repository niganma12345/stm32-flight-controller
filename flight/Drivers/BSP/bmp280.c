#include "bmp280.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

// BMP280 芯片ID期望值
#define BMP280_CHIP_ID 0x58

// 全局校准参数（初始化时从芯片读取一次）
static BMP280_Calib bmp280_calib;

// 温度精细值（用于气压补偿，从最近一次温度读取得到）
static int32_t t_fine = 0;

// 海平面参考气压（Pa），可运行时动态校准
static float sea_level_pressure = 101325.0f;

/**
 * @brief 写BMP280寄存器
 *
 * @param reg 寄存器地址
 * @param data 数据值
 */
static void BMP280_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR_WRITE, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

/**
 * @brief 读BMP280寄存器
 *
 * @param reg 寄存器地址
 * @param data 存放读取数据的地址指针
 */
static void BMP280_Read_Reg(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR_READ, reg,
                     I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

/**
 * @brief 读BMP280多个寄存器
 *
 * @param reg 起始寄存器地址
 * @param data 存放读取数据的数组指针
 * @param len 读取字节数
 */
static void BMP280_Read_Regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR_READ, reg,
                     I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}

/**
 * @brief 软复位BMP280
 */
void BMP280_SoftReset(void)
{
    BMP280_Write_Reg(BMP280_REG_RESET, 0xB6);
    vTaskDelay(10); // 等待复位完成
}

/**
 * @brief 从芯片读取校准参数
 */
static void BMP280_Read_Calib_Data(void)
{
    uint8_t calib_buf[24];

    // 读取温度校准数据（地址0x88~0x8D）
    BMP280_Read_Regs(0x88, calib_buf, 24);

    bmp280_calib.dig_T1 = (uint16_t)(calib_buf[0]  | (calib_buf[1] << 8));
    bmp280_calib.dig_T2 = (int16_t) (calib_buf[2]  | (calib_buf[3] << 8));
    bmp280_calib.dig_T3 = (int16_t) (calib_buf[4]  | (calib_buf[5] << 8));
    bmp280_calib.dig_P1 = (uint16_t)(calib_buf[6]  | (calib_buf[7] << 8));
    bmp280_calib.dig_P2 = (int16_t) (calib_buf[8]  | (calib_buf[9] << 8));
    bmp280_calib.dig_P3 = (int16_t) (calib_buf[10] | (calib_buf[11] << 8));
    bmp280_calib.dig_P4 = (int16_t) (calib_buf[12] | (calib_buf[13] << 8));
    bmp280_calib.dig_P5 = (int16_t) (calib_buf[14] | (calib_buf[15] << 8));
    bmp280_calib.dig_P6 = (int16_t) (calib_buf[16] | (calib_buf[17] << 8));
    bmp280_calib.dig_P7 = (int16_t) (calib_buf[18] | (calib_buf[19] << 8));
    bmp280_calib.dig_P8 = (int16_t) (calib_buf[20] | (calib_buf[21] << 8));
    bmp280_calib.dig_P9 = (int16_t) (calib_buf[22] | (calib_buf[23] << 8));
}

/**
 * @brief 初始化BMP280芯片
 *
 * 配置：正常模式，气压×4过采样，温度×1过采样，IIR滤波器系数8，待机时间62.5ms
 * @return uint8_t 0:初始化成功  1:初始化失败
 */
uint8_t BMP280_Init(void)
{
    uint8_t chip_id = 0;

    // 1. 软复位芯片
    BMP280_SoftReset();

    // 2. 读取芯片ID验证
    BMP280_Read_Reg(BMP280_REG_CHIP_ID, &chip_id);
    if (chip_id != BMP280_CHIP_ID)
    {
        return 1; // 芯片ID不匹配，可能没有焊接或损坏
    }

    // 3. 读取校准参数
    BMP280_Read_Calib_Data();

    // 4. 配置测量控制寄存器
    //    Bit[7:5] = osrs_t(过采样温度)    => ×1
    //    Bit[4:2] = osrs_p(过采样气压)    => ×4
    //    Bit[1:0] = mode(模式)            => 正常模式
    uint8_t ctrl_meas = (BMP280_OVERSAMP_1X << 5)
                      | (BMP280_OVERSAMP_4X << 2)
                      | BMP280_MODE_NORMAL;
    BMP280_Write_Reg(BMP280_REG_CTRL_MEAS, ctrl_meas);

    // 5. 配置寄存器
    //    Bit[7:5] = t_sb(待机时间)        => 62.5ms
    //    Bit[4:2] = filter(IIR滤波器)     => 系数8
    //    Bit[1:0] = spi3w_en(SPI使能)     => 0(关闭SPI3线)
    uint8_t config = (BMP280_STANDBY_62_5MS << 5)
                   | (BMP280_FILTER_8 << 2)
                   | 0x00;
    BMP280_Write_Reg(BMP280_REG_CONFIG, config);

    // 等待第一次测量完成
    vTaskDelay(20);

    return 0;
}

/**
 * @brief 读取BMP280原始温度值并执行补偿
 *
 * 返回补偿后的温度精细值 t_fine（存储在全局变量中供气压补偿使用）
 * @return int32_t 补偿后的温度值（单位: ℃ × 100）
 */
int32_t BMP280_Get_Temperature_Raw(void)
{
    uint8_t data[3];
    int32_t adc_T;

    // 读取温度ADC原始值（20位）
    BMP280_Read_Regs(BMP280_REG_TEMP_MSB, data, 3);
    adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    // BOSCH温度补偿公式
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1)))
            * ((int32_t)bmp280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))
              * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12)
            * ((int32_t)bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    T = (t_fine * 5 + 128) >> 8; // 温度值 ℃ × 100

    return T;
}

/**
 * @brief 读取BMP280原始气压值
 *
 * @return int32_t 原始气压值（ADC值）
 */
int32_t BMP280_Get_Pressure_Raw(void)
{
    uint8_t data[3];
    int32_t adc_P;

    // 读取气压ADC原始值（20位）
    BMP280_Read_Regs(BMP280_REG_PRESS_MSB, data, 3);
    adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    return adc_P;
}

/**
 * @brief 读取并计算补偿后的温度值
 *
 * @return float 温度值（单位: ℃）
 */
float BMP280_Get_Temperature(void)
{
    int32_t temp_raw = BMP280_Get_Temperature_Raw();
    return (float)temp_raw / 100.0f;
}

/**
 * @brief 读取并计算补偿后的气压值
 *
 * 必须先调用 BMP280_Get_Temperature_Raw() 更新 t_fine
 * @return float 气压值（单位: Pa）
 */
float BMP280_Get_Pressure(void)
{
    int32_t adc_P;
    int64_t var1, var2, p;
    uint8_t data[3];

    // 先更新温度（气压补偿依赖于 t_fine）
    BMP280_Get_Temperature_Raw();

    // 读取气压ADC原始值（20位）
    BMP280_Read_Regs(BMP280_REG_PRESS_MSB, data, 3);
    adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    // BOSCH气压补偿公式
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8)
         + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0.0f; // 避免除零
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);

    return (float)p / 256.0f; // 单位: Pa
}

/**
 * @brief 设置海平面参考气压
 *
 * 定高飞行前调用，以当前位置气压作为基准
 * @param ref_pressure 参考气压值（单位: Pa）
 */
void BMP280_Set_SeaLevel_Pressure(float ref_pressure)
{
    sea_level_pressure = ref_pressure;
}

/**
 * @brief 获取当前海平面参考气压
 *
 * @return float 海平面参考气压（单位: Pa）
 */
float BMP280_Get_SeaLevel_Pressure(void)
{
    return sea_level_pressure;
}

/**
 * @brief 读取并计算当前海拔高度
 *
 * 使用国际标准大气压（ISA）公式计算海拔
 * 高度 = 44330 * (1 - (P / P0)^(1/5.255))
 *
 * @param sea_level_pressure 海平面参考气压（单位: Pa）
 * @return float 海拔高度（单位: m）
 */
float BMP280_Get_Altitude(float sea_level_pressure)
{
    float pressure = BMP280_Get_Pressure();

    if (pressure <= 0.0f || sea_level_pressure <= 0.0f)
    {
        return 0.0f;
    }

    // ISA标准大气压海拔公式
    float altitude = 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.19029496f));

    return altitude;
}

/**
 * @brief 通过当前气压自动校准海平面气压
 *
 * 在地面调用此函数以获得准确的海平面参考气压
 * 如果你知道当前的实际海拔，可传入实际海拔值（例如手机GPS查询），传0表示当前=海平面
 * @param known_altitude 已知的当前海拔高度（单位: m），默认0
 * @return float 校准后的海平面气压（单位: Pa）
 */
float BMP280_Calibrate_SeaLevel(float known_altitude)
{
    // 先读5次取平均，减少噪声
    float pressure_sum = 0.0f;
    for (uint8_t i = 0; i < 5; i++)
    {
        pressure_sum += BMP280_Get_Pressure();
        vTaskDelay(10);
    }
    float avg_pressure = pressure_sum / 5.0f;

    // 根据ISA反推海平面气压
    // P0 = P / (1 - h/44330)^5.255
    float factor = 1.0f - known_altitude / 44330.0f;
    sea_level_pressure = avg_pressure / powf(factor, 5.255f);

    return sea_level_pressure;
}
