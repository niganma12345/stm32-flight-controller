#ifndef LED_H
#define LED_H

#include "main.h"
#include "Com_config.h"

/**
 * @brief LED 灯控模块初始化
 *
 */
void Led_Init(void);

/**
 * @brief LED 状态处理（每6ms调用一次）
 *
 *        LED0: 通讯状态指示 — 连接时常亮，断开500ms后熄灭（防抖动）
 *        LED1: 飞行状态指示 — LOCKED慢闪，解锁后常亮，故障灭
 */
void Led_Process(void);

#endif // LED_H
