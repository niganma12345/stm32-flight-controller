/**
 ******************************************************************************
 * @file    App_oled.h
 * @brief   OLED 显示任务封装 — FreeRTOS 队列驱动，多任务安全
 * @note    依赖底层 OLED.h，此层仅做队列调度，不改动底层
 ******************************************************************************
 */

#ifndef __APP_OLED_H
#define __APP_OLED_H

#include "OLED.h"

/*============================================================================*/
/* OLED 行号 (Y 坐标, 以 8x16 字体计，共 4 行)                                  */
/*============================================================================*/
#define OLED_ROW_0     0
#define OLED_ROW_1     16
#define OLED_ROW_2     32
#define OLED_ROW_3     48

/*============================================================================*/
/* 任务配置（供 App_task.c 使用）                                           */
/*============================================================================*/
#define APP_OLED_TASK_STACK     128
#define APP_OLED_TASK_PRIORITY  1

/*============================================================================*/
/* API                                                                         */
/*============================================================================*/

/**
 * @brief  创建 OLED 命令队列（调度器启动后调用一次）
 */
void App_OLED_Init(void);

/**
 * @brief  OLED 显示任务 — 由 App_task.c 的 start_task 创建
 */
void App_OLED_Task(void *pvParameters);

/**
 * @brief  发送文本到 OLED（非阻塞，队列满则丢弃，任何任务可调用）
 */
void App_OLED_Post(int16_t X, int16_t Y, uint8_t FontSize, const char *Text);

/**
 * @brief  发送格式化文本（printf 风格，非阻塞，任何任务可调用）
 */
void App_OLED_Postf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...);

#endif
