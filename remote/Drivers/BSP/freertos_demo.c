#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"         /* 队列 API —— 用于任务间线程安全通信 */
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"
#include "oled.h"
#include "NRF24L01.h"
#include "key.h"
#include "mpu6050.h"
#include "joystick.h"
#include "FlowIcon.h"


/* ================================================================
 *  IPC 队列（替代全局变量，实现任务间线程安全通信）
 *
 *  joystickQueue  — 覆盖写队列，长度 1（总是保存最新值）
 *                   adc 任务写入 → task2 任务 Peek 读取
 *  keyEventQueue  — 普通队列，长度 8（缓冲按键事件防止丢失）
 *                   key 任务写入 → task2 任务 Receive 消费
 * ================================================================ */
static QueueHandle_t joystickQueue = NULL;
static QueueHandle_t keyEventQueue = NULL;

#define JOYSTICK_QUEUE_LEN   1u   /* 覆盖写，只保留最新摇杆值   */
#define KEYEVENT_QUEUE_LEN   8u   /* 缓冲 8 个事件，防止丢失    */


#define cycle_time 6

/* ---- start_task ---- */
#define START_TASK_STACK    128
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

/* ---- adc 任务：摇杆采集 ---- */
#define adc_STACK     128
#define adc_PRIORITY  4
TaskHandle_t adc_handle;
void adc(void *pvParameters);

/* ---- key 任务：按键扫描 ---- */
#define key_STACK     128
#define key_PRIORITY  4
TaskHandle_t key_handle;
void key(void *pvParameters);

/* ---- task2：NRF 收发 + OLED 显示 ---- */
#define TASK2_STACK     320
#define TASK2_PRIORITY  3
TaskHandle_t task2_handle;
void task2(void *pvParameters);


/**
 * @brief  FreeRTOS 启动入口
 */
void freertos_start(void)
{
    xTaskCreate((TaskFunction_t)start_task,
                (char *)"start_task",
                (configSTACK_DEPTH_TYPE)128,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&start_task_handle);

    vTaskStartScheduler();
}

/**
 * @brief  初始化任务：创建 IPC 队列和所有应用任务后自删除
 */
static void start_task(void *pvParameters)
{
    /* ---- 创建 IPC 队列（必须在子任务启动前完成） ---- */
    joystickQueue = xQueueCreate(JOYSTICK_QUEUE_LEN, sizeof(Joystick_t));
    keyEventQueue = xQueueCreate(KEYEVENT_QUEUE_LEN, sizeof(KeyEvent));
    configASSERT(joystickQueue != NULL);
    configASSERT(keyEventQueue != NULL);

    OLED_Init();
    key_init();

    /* 初始化 NRF24L01 并检查连接 */
    NRF24L01_Init();
    if (NRF24L01_ReadReg(NRF24L01_CONFIG) == 0xFF)
    {
        while (1) { HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET); }
    }

    /* LED 初始熄灭（高电平） */
    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

    /* 创建应用任务 */
    xTaskCreate((TaskFunction_t)adc,
                (char *)"adc",
                (configSTACK_DEPTH_TYPE)adc_STACK,
                (void *)NULL,
                (UBaseType_t)adc_PRIORITY,
                (TaskHandle_t *)&adc_handle);

    xTaskCreate((TaskFunction_t)key,
                (char *)"key",
                (configSTACK_DEPTH_TYPE)key_STACK,
                (void *)NULL,
                (UBaseType_t)key_PRIORITY,
                (TaskHandle_t *)&key_handle);

    xTaskCreate((TaskFunction_t)task2,
                (char *)"task2",
                (configSTACK_DEPTH_TYPE)TASK2_STACK,
                (void *)NULL,
                (UBaseType_t)TASK2_PRIORITY,
                (TaskHandle_t *)&task2_handle);

    vTaskDelete(NULL);
}


/* ================================================================
 *  adc 任务 —— 摇杆 ADC 采集（周期 6ms）
 *
 *  从 DMA 缓冲区读取摇杆原始值，通过覆盖写队列发送给 task2。
 *  队列长度 = 1，新值始终覆盖旧值，task2 永远读到最新摇杆数据。
 * ================================================================ */
void adc(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    joystick_init();

    while (1)
    {
        Joystick_t joystick;                     /* 局部变量，栈上分配 */
        Int_joystick_get(&joystick);
        xQueueOverwrite(joystickQueue, &joystick); /* 覆盖写入，无需等待 */

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}


/* ================================================================
 *  key 任务 —— 按键扫描（周期 6ms）
 *
 *  每周期调用 key_scan() 消抖 + 状态机处理，
 *  产生的事件通过队列发送给 task2。队列长度 = 8，缓冲区满时
 *  最旧事件被丢弃（xQueueSend 队列满返回 errQUEUE_FULL）。
 * ================================================================ */
void key(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        KeyEvent ev = key_scan();                /* 局部变量 */
        if (ev.event != KEY_EVENT_NONE)
        {
            /* 非阻塞发送：队列满则丢弃最旧事件，避免阻塞按键扫描 */
            xQueueSend(keyEventQueue, &ev, 0);
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}


/* ================================================================
 *  task2 —— NRF24L01 收发 + OLED 显示（周期 6ms）
 *
 *  1. 从队列消费按键事件，驱动 shutdown / fix_height 单次触发标志
 *  2. 从队列 Peek 最新摇杆值，转换后打包发送
 *  3. 接收飞机回传遥测数据，更新 OLED 显示
 * ================================================================ */
void task2(void *pvParameters)
{
    uint8_t TxFlag = 0;
    uint8_t RxFlag = 0;
    uint8_t tx_fail_cnt = 0;
    #define TX_FAIL_MAX  5

    int16_t thr = 0, yaw = 0, pit = 0, rol = 0;
    uint8_t shutdown_flag  = 0;
    uint8_t fix_height_flag = 0;
    uint32_t sum;
    uint8_t i;

    /* ---- 遥测数据（仅本任务访问 → 改为局部变量） ---- */
    float   altitude    = 0.0f;   /* 飞机回传高度 (m)    */
    uint8_t plane_state = 0;     /* 飞行状态             */
    float   voltage     = 0.0f;   /* 电池电压 (V)        */
    uint8_t flow_x      = 0;     /* 光流前后方向移动标志 */
    uint8_t flow_y      = 0;     /* 光流左右方向移动标志 */

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        /* ================================================================
         *  第 1 步：消费按键事件（从队列 Receive，消费即移除）
         *
         *  非阻塞读取（timeout = 0）：无事件时立即返回，不阻塞主循环。
         *  仅消费 1 个事件/周期（高优先操作，避免饥饿）。
         * ================================================================ */
        {
            KeyEvent ev;
            if (xQueueReceive(keyEventQueue, &ev, 0) == pdPASS)
            {
                if (ev.event == KEY_EVENT_SHORT)
                {
                    if (ev.key_id == KEY_ID_K1)
                        shutdown_flag = 1;
                    else if (ev.key_id == KEY_ID_K2)
                        fix_height_flag = 1;
                }
            }
        }

        /* ================================================================
         *  第 2 步：读取摇杆数据（从队列 Peek，不移除，始终获取最新值）
         *
         *  非阻塞 Peek（timeout = 0）：队列为空时跳过，沿用上一周期值。
         *  adc 任务以覆盖方式写入，Peek 保证读到最新一次采集结果。
         * ================================================================ */
        {
            Joystick_t joystick;
            if (xQueuePeek(joystickQueue, &joystick, 0) == pdPASS)
            {
                thr = (int16_t)((Joystick_DataProcess(joystick.thr) + 100) * 5);
                yaw = (int16_t)((Joystick_DataProcess(joystick.yaw) + 100) * 5);
                pit = (int16_t)((Joystick_DataProcess(joystick.pit) + 100) * 5);
                rol = (int16_t)((Joystick_DataProcess(joystick.rol) + 100) * 5);
            }
        }

        /* ---- 打包发送数据（协议与飞行端 App_receive_data 一致） ---- */
        NRF24L01_TxPacket[0] = 'n';
        NRF24L01_TxPacket[1] = 'g';
        NRF24L01_TxPacket[2] = 'm';
        NRF24L01_TxPacket[3]  = (uint8_t)(thr >> 8);
        NRF24L01_TxPacket[4]  = (uint8_t)(thr & 0xFF);
        NRF24L01_TxPacket[5]  = (uint8_t)(yaw >> 8);
        NRF24L01_TxPacket[6]  = (uint8_t)(yaw & 0xFF);
        NRF24L01_TxPacket[7]  = (uint8_t)(pit >> 8);
        NRF24L01_TxPacket[8]  = (uint8_t)(pit & 0xFF);
        NRF24L01_TxPacket[9]  = (uint8_t)(rol >> 8);
        NRF24L01_TxPacket[10] = (uint8_t)(rol & 0xFF);
        NRF24L01_TxPacket[11] = shutdown_flag;
        NRF24L01_TxPacket[12] = fix_height_flag;
        shutdown_flag  = 0;
        fix_height_flag = 0;

        /* ---- 校验和（Byte0~12 求和，4 字节 big-endian） ---- */
        sum = 0;
        for (i = 0; i < 13; i++)
        {
            sum += NRF24L01_TxPacket[i];
        }
        NRF24L01_TxPacket[13] = (uint8_t)(sum >> 24);
        NRF24L01_TxPacket[14] = (uint8_t)(sum >> 16);
        NRF24L01_TxPacket[15] = (uint8_t)(sum >> 8);
        NRF24L01_TxPacket[16] = (uint8_t)(sum & 0xFF);

        /* ---- NRF24L01 发送 ---- */
        TxFlag = NRF24L01_Send();

        if (TxFlag != 1)
        {
            tx_fail_cnt++;
            if (tx_fail_cnt >= TX_FAIL_MAX)
            {
                tx_fail_cnt = 0;
                NRF24L01_FlushTx();
                NRF24L01_FlushRx();
                NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
                NRF24L01_Rx();
            }
        }
        else
        {
            tx_fail_cnt = 0;
        }

        /* ---- NRF24L01 接收 ---- */
        RxFlag = NRF24L01_Receive();

        if (RxFlag == 1 && NRF24L01_RxPacket[0] == 'a'
            && NRF24L01_RxPacket[1] == 'l' && NRF24L01_RxPacket[2] == 't')
        {
            int16_t alt_cm = (int16_t)((NRF24L01_RxPacket[3] << 8) | NRF24L01_RxPacket[4]);
            altitude    = (float)alt_cm / 100.0f;
            plane_state = NRF24L01_RxPacket[5];
            uint16_t volt_dmv = (uint16_t)((NRF24L01_RxPacket[6] << 8) | NRF24L01_RxPacket[7]);
            voltage     = (float)volt_dmv / 10.0f;
            flow_x      = NRF24L01_RxPacket[8];
            flow_y      = NRF24L01_RxPacket[9];
        }

        /* ---- OLED 显示 ---- */
        OLED_Clear();
        OLED_Printf(0,  0, OLED_8X16, "TX:%s RX:%s",
                    TxFlag == 1 ? "OK" : TxFlag == 2 ? "MAXRT" : "FAIL",
                    RxFlag == 1 ? "OK" : RxFlag == 0 ? "-" : "ERR");
        OLED_Printf(96, 0, OLED_8X16, "%.1fV", (double)voltage);
        OLED_Printf(0, 16, OLED_8X16, "TH:%04d YW:%04d", (int)thr, (int)yaw);
        OLED_Printf(0, 32, OLED_8X16, "PI:%04d RO:%04d", (int)pit, (int)rol);

        {
            static const char *state_str[] = {
                "LOCKED", "IDLE", "NORMAL", "FIX_HEIGHT", "MANUAL", "FAIL"
            };
            const char *s = (plane_state <= 5) ? state_str[plane_state] : "?";
            OLED_Printf(80, 48, OLED_8X16, "%.1fm", (double)altitude);
            OLED_Printf(0,  56, OLED_6X8,  "%s", s);
        }

        {
            const uint8_t *flow_icon;
            if (flow_x && flow_y)
                flow_icon = FlowIcon_XY;
            else if (flow_x)
                flow_icon = FlowIcon_X;
            else if (flow_y)
                flow_icon = FlowIcon_Y;
            else
                flow_icon = FlowIcon_None;
            OLED_ShowImage(56, 47, 16, 16, flow_icon);
        }

        OLED_Update();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}
