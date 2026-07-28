/**
 ******************************************************************************
 * @file    App_oled.h
 * @brief   OLED 显示任务封装
 * @note    多任务通过 Post / Postf 投递文本到队列，OLED 任务统一渲染刷屏。
 *          底层依赖 OLED.h，此层仅做队列调度和格式化，不改动底层驱动。
 ******************************************************************************
 */

#ifndef __APP_OLED_H
#define __APP_OLED_H

#include "OLED.h"

/* OLED 行号 (Y 坐标, 8x16 字体共 4 行) */
#define OLED_ROW_0     0
#define OLED_ROW_1     16
#define OLED_ROW_2     32
#define OLED_ROW_3     48

/* 任务配置（供 App_task.c 的 start_task 创建任务时使用） */
#define APP_OLED_TASK_STACK     128
#define APP_OLED_TASK_PRIORITY  1

/*
 * API
 */

/**
 * @brief  创建 OLED 命令队列（调度器启动后由 start_task 调用）
 */
void App_OLED_Init(void);

/**
 * @brief  OLED 显示任务 — 消费队列中的文本命令，清屏 → 渲染 → 刷屏
 */
void App_OLED_Task(void *pvParameters);

/**
 * @brief  投递文本到 OLED（非阻塞，队列满则丢弃，任意任务安全调用）
 * @param  X, Y      显示起始坐标
 * @param  FontSize  字体（OLED_8X16 或 OLED_6X8）
 * @param  Text      要显示的字符串（最大 31 字符）
 */
void App_OLED_Post(int16_t X, int16_t Y, uint8_t FontSize, const char *Text);

/**
 * @brief  投递格式化文本（printf 风格，内部格式化后再 Post）
 * @param  X, Y, FontSize  同 App_OLED_Post
 * @param  format          格式化字符串，后续可变参数为要填充的值
 */
void App_OLED_Postf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...);

#endif
