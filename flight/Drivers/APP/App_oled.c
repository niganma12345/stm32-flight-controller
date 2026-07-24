/**
 ******************************************************************************
 * @file    App_oled.c
 * @brief   OLED 显示任务封装 — FreeRTOS 队列驱动
 * @note    其他任务通过 App_OLED_Post/App_OLED_Postf 发送文本，
 *          内部的 oled_task 每 200ms 消费队列、清屏、渲染、刷屏。
 *          完全隔离 FreeRTOS 逻辑与底层 OLED 驱动。
 ******************************************************************************
 */

#include "App_oled.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*============================================================================*/
/* 内部常量                                                                     */
/*============================================================================*/
#define OLED_QUEUE_LEN      32     /* 队列深度 */
#define OLED_REFRESH_MS     200     /* 刷新周期 */

/*============================================================================*/
/* 显示命令                                                                    */
/*============================================================================*/
typedef struct {
    int16_t X;
    int16_t Y;
    uint8_t FontSize;
    char    Text[32];
} OledCmd_t;

/*============================================================================*/
/* 内部静态变量                                                                 */
/*============================================================================*/
static QueueHandle_t g_queue = NULL;

/*============================================================================*/
/* 内部任务函数                                                                 */
/*============================================================================*/
void App_OLED_Task(void *pvParameters)
{
    OLED_Init();

    while (1)
    {
        OledCmd_t cmd;

        OLED_Clear();

        /* 消费队列中所有待显示命令 */
        while (xQueueReceive(g_queue, &cmd, 0) == pdTRUE)
        {
            OLED_ShowString(cmd.X, cmd.Y, cmd.Text, cmd.FontSize);
        }

        OLED_Update();
        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_MS));
    }
}

/*============================================================================*/
/* 队列创建（由 start_task 调用，调度器启动后）                                   */
/*============================================================================*/
void App_OLED_Init(void)
{
    g_queue = xQueueCreate(OLED_QUEUE_LEN, sizeof(OledCmd_t));
}

void App_OLED_Post(int16_t X, int16_t Y, uint8_t FontSize, const char *Text)
{
    if (!g_queue) return;
    OledCmd_t cmd;
    cmd.X = X;
    cmd.Y = Y;
    cmd.FontSize = FontSize;
    strncpy(cmd.Text, Text, sizeof(cmd.Text) - 1);
    cmd.Text[sizeof(cmd.Text) - 1] = '\0';
    xQueueSend(g_queue, &cmd, 0);
}

void App_OLED_Postf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...)
{
    char buf[64];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    App_OLED_Post(X, Y, FontSize, buf);
}
