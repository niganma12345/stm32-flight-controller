#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "adc.h"

/* 摇杆数据结构体 */
typedef struct
{
    uint16_t thr;        /* 油门 ADC 原始值 0~4095 */
    uint16_t yaw;        /* 偏航 ADC 原始值 0~4095 */
    uint16_t pit;        /* 俯仰 ADC 原始值 0~4095 */
    uint16_t rol;        /* 翻滚 ADC 原始值 0~4095 */
} Joystick_t;

/* 全局变量声明 */
extern uint16_t adc_buffer[4];  /* ADC DMA 缓冲，4 通道 */
extern Joystick_t g_Joystick;   /* 全局摇杆数据，供各任务读取 */

/* 函数声明 */
void joystick_init(void);
void Int_joystick_get(Joystick_t *joystick);
int8_t Joystick_DataProcess(uint16_t ADValue);

#endif /* __JOYSTICK_H */
