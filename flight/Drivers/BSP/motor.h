#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"
#include "com_debug.h"
typedef struct 
 {
    TIM_HandleTypeDef *tim; // 定时器句柄
    uint32_t channel; // 定时器通道
    int16_t speed;  // 当前速度（有符号，FAIL 减速可下至 0 以下由 Motor_SetSpeed 钳位）
} Motor;

void Motor_Init(Motor *motor);
void Motor_SetSpeed(Motor *motor);
#endif // __MOTOR_H
