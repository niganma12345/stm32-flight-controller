/**
 * @file    joystick.c
 * @brief   摇杆驱动 —— ADC DMA 采集 + 数据处理
 *
 * ADC1 通过 DMA 循环采集 4 个摇杆通道（PA0~PA3），
 * Int_joystick_get 读取原始 ADC 值，
 * Joystick_DataProcess 将原始值转换为工程值 (-100 ~ +100)。
 */
#include "joystick.h"

/* 全局变量定义 */
uint16_t adc_buffer[4] = {0};   /* ADC DMA 缓冲区 */
Joystick_t g_Joystick;           /* 全局摇杆数据 */

/**
 * @brief  摇杆初始化，启动 ADC DMA 连续采集
 */
void joystick_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 4);
}

/**
 * @brief  从 DMA 缓冲区读取摇杆原始 ADC 值
 * @param  joystick 摇杆数据结构体指针
 */
void Int_joystick_get(Joystick_t *joystick)
{
    joystick->thr = adc_buffer[1];
    joystick->yaw = adc_buffer[0];
    joystick->pit = adc_buffer[3];
    joystick->rol = adc_buffer[2];
}

/**
 * @brief  摇杆 ADC 数据处理
 * @param  ADValue 原始 ADC 值，范围：0~4095
 * @return 处理后的摇杆值，范围：-100~+100
 * @note   中值 2048，死区对称 ±150，覆盖硬件中值偏移
 *
 *         死区示例：ADC ∈ [1898, 2198] → 输出 0
 *         满量程示例：ADC = 0           → 输出 -100
 *                     ADC = 4095        → 输出 +100
 */
int8_t Joystick_DataProcess(uint16_t ADValue)
{
    int16_t Value;

    /* 减去 ADC 中值（2048 = Vref/2） */
    Value = (int16_t)ADValue - 2048;

    /* 对称死区：±150 */
    if (Value > 150)
    {
        Value -= 150;
    }
    else if (Value < -150)
    {
        Value += 150;
    }
    else
    {
        Value = 0;
    }

    /* 缩放至 -100~+100（最大有效范围: 2048 - 150 = 1898） */
    if (Value != 0)
    {
        Value = Value * 100 / 1898;
    }

    /* 限幅 */
    if (Value >  100) Value =  100;
    if (Value < -100) Value = -100;

    return (int8_t)Value;
}
