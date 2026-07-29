#include "led.h"

#include "App_task.h"

/* 外部全局变量 */
extern volatile Flight_State flight_state;

/* ---- LED0: 通讯状态指示 ---- */
/* 连接时常亮（低电平），连续断开 ~500ms 后才灭，防止信号抖动导致闪烁 */
static uint16_t led0_off_cnt = 0;

/* ---- LED1: 飞行状态指示 ---- */
/* LOCKED 慢闪(~1Hz)，解锁后常亮，FAIL 灭 */
static uint16_t led1_blink_cnt = 0;
static uint8_t  led1_blink_state = 0;  /* 0=灭, 1=亮 */

/**
 * @brief LED 灯控模块初始化
 *
 */
void Led_Init(void)
{
    /* 初始状态：关闭所有 LED */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);  /* 灭 */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  /* 灭 */

    led0_off_cnt = 0;
    led1_blink_cnt = 0;
    led1_blink_state = 0;
}

/**
 * @brief LED 状态处理（每6ms调用一次）
 *
 *        LED0: 通讯状态指示 — 连接时常亮，断开500ms后熄灭（防抖动）
 *        LED1: 飞行状态指示 — LOCKED慢闪(~1Hz)，解锁后常亮，故障灭
 */
void Led_Process(void)
{
    /* ---- LED0：通讯状态 ---- */
    if (xEventGroupGetBits(flight_evt_group) & EVT_REMOTE_CONNECTED)  /* 读事件组 */
    {
        led0_off_cnt = 0;
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);  /* 亮（低电平） */
    }
    else
    {
        if (++led0_off_cnt >= 83)  /* 83 * 6ms ≈ 500ms */
        {
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);  /* 灭 */
        }
    }

    /* ---- LED1：飞行状态 ---- */
    if (flight_state == LOCKED)
    {
        /* 慢闪：~500ms亮 ~500ms灭 (83 * 6ms ≈ 500ms) */
        if (++led1_blink_cnt >= 83)
        {
            led1_blink_cnt = 0;
            led1_blink_state = !led1_blink_state;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                led1_blink_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
    else if (flight_state == IDLE || flight_state == NORMAL || flight_state == FIX_HEIGHT)
    {
        led1_blink_cnt = 0;
        led1_blink_state = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);  /* 常亮 */
    }
    else  /* FAIL */
    {
        led1_blink_cnt = 0;
        led1_blink_state = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);    /* 灭 */
    }
}
