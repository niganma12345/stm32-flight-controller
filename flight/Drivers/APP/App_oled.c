/**
 ******************************************************************************
 * @file    App_oled.c
 * @brief   OLED 显示任务封装 — FreeRTOS 队列驱动
 * @note    核心思路：多任务通过 Post/Postf 投递文本命令到队列，
 *          OLED 任务按固定周期消费队列、渲染显存、推屏。
 *          完全隔离 FreeRTOS 任务逻辑与底层 I2C 驱动 (OLED.h)。
 ******************************************************************************
 */

#include "App_oled.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ---- 内部常量 ---- */

#define OLED_QUEUE_LEN      16     /* 命令队列深度（任意任务投递，超出丢弃） */
#define OLED_REFRESH_MS     200    /* 刷屏周期 (ms) */

/* ---- 文本命令（队列传输单元）---- */

typedef struct {
    int16_t X;                 /* 起始 X 坐标 */
    int16_t Y;                 /* 起始 Y 坐标 */
    uint8_t FontSize;          /* 字体：OLED_8X16 / OLED_6X8 */
    char    Text[32];          /* 显示文本（最大 31 字符 + '\0'） */
} OledCmd_t;

/* ---- 内部变量 ---- */

static QueueHandle_t g_queue = NULL;  /* 命令队列句柄 */

/*
 * 任务函数
 */

/**
 * @brief  OLED 显示任务
 *         周期清屏 → 消费队列中所有命令 → 刷屏 → 等待下一周期
 * @note   每帧先 Clear 再逐条 ShowString，最后 Update 一次性推屏，
 *         不会出现屏幕闪烁（Update 之前用户看不到中间状态）。
 */
void App_OLED_Task(void *pvParameters)
{
    OLED_Init();

    while (1)
    {
        OledCmd_t cmd;

        OLED_Clear();

        /* 消费队列中所有待显示命令（非阻塞，队列为空则画空白屏） */
        while (xQueueReceive(g_queue, &cmd, 0) == pdTRUE)
        {
            OLED_ShowString(cmd.X, cmd.Y, cmd.Text, cmd.FontSize);
        }

        OLED_Update();
        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_MS));
    }
}

/*
 * API
 */

/**
 * @brief  创建命令队列（调度器启动后由 start_task 调用一次）
 */
void App_OLED_Init(void)
{
    g_queue = xQueueCreate(OLED_QUEUE_LEN, sizeof(OledCmd_t));
}

/**
 * @brief  投递文本（非阻塞，调用方格式化用 Postf）
 */
void App_OLED_Post(int16_t X, int16_t Y, uint8_t FontSize, const char *Text)
{
    if (!g_queue) return;

    OledCmd_t cmd;
    cmd.X = X;
    cmd.Y = Y;
    cmd.FontSize = FontSize;
    strncpy(cmd.Text, Text, sizeof(cmd.Text) - 1);
    cmd.Text[sizeof(cmd.Text) - 1] = '\0';
    xQueueSend(g_queue, &cmd, 0);  /* 队列满则丢弃 */
}

/**
 * @brief  投递格式化文本
 *         - 内部 vsnprintf → 本地 64 字节缓冲区
 *         - 转调 App_OLED_Post 入队
 */
void App_OLED_Postf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...)
{
    char buf[64];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    App_OLED_Post(X, Y, FontSize, buf);
}
