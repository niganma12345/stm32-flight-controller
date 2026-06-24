#include "motor.h"

void Motor_SetSpeed(Motor *motor)
{
    /* 钳位到有效 PWM 范围，但不 return —— 必须设置 PWM */
    if (motor->speed < 0)   motor->speed = 0;
    if (motor->speed > 1000) motor->speed = 1000;
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, (uint32_t)motor->speed);
}


void Motor_Init(Motor *motor)
{
  HAL_TIM_PWM_Start(motor->tim, motor->channel);
}
