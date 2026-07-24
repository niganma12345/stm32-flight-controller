#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "task.h"
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


/* 全局变量 —— 由各任务共享 */
KeyEvent g_KeyEvent;   /* 最新按键事件（key 任务写入，task2 读取显示） */
volatile float g_altitude = 0.0f;  /* 飞机回传的当前海拔高度（单位: m） */
volatile uint8_t g_plane_state = 0; /* 飞机回传的飞行状态 */
volatile float g_voltage = 0.0f;    /* 飞机回传的电池电压（单位: V） */
volatile uint8_t g_flow_x = 0;     /* 光流前后方向是否检测到移动 (0/1) */
volatile uint8_t g_flow_y = 0;     /* 光流左右方向是否检测到移动 (0/1) */

#define cycle_time 6  

#define START_TASK_STACK 128
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

#define adc_task 128
#define adc_PRIORITY 4
TaskHandle_t adc_handle;
void adc(void *pvParameters);

#define key_task 128
#define key_PRIORITY 4
TaskHandle_t key_handle;
void key(void *pvParameters);

#define TASK2_STACK 320
#define TASK2_PRIORITY 3
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
 * @brief  初始化任务：创建所有应用任务后自删除
 */
static void start_task(void *pvParameters)
{ 
	  OLED_Init();
    key_init();                  /* 初始化按键状态机 */

    /* 初始化 NRF24L01 并检查连接 */
    NRF24L01_Init();
    if (NRF24L01_ReadReg(NRF24L01_CONFIG) == 0xFF)
    {
        while (1){HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);};  // NRF24L01 未连接，停机
    }

    /* LED 初始熄灭（高电平） */
    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

    /* ADC 摇杆读取任务 */
    xTaskCreate((TaskFunction_t)adc,
                (char *)"adc",
                (configSTACK_DEPTH_TYPE)adc_task,
                (void *)NULL,
                (UBaseType_t)adc_PRIORITY,
                (TaskHandle_t *)&adc_handle);

    /* 按键扫描任务 */
    xTaskCreate((TaskFunction_t)key,
                (char *)"key",
                (configSTACK_DEPTH_TYPE)key_task,
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


/* ---- ADC 摇杆读取任务 (50ms 周期) ---- */

void adc(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 启动 ADC DMA 连续采集（4 通道循环扫描） */
    joystick_init();

    while (1)
    {
        /* 从 DMA 缓冲区读取摇杆原始 ADC 值 */
        Int_joystick_get(&g_Joystick);

        /* 50ms 采集周期 = 20Hz 更新率 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}

/* ---- 按键扫描任务（5ms 周期） ---- */

void key(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 初始化空事件 */
    g_KeyEvent.event        = KEY_EVENT_NONE;
    g_KeyEvent.key_id       = 0;
    g_KeyEvent.pressed_mask = 0;

    while (1)
    {
        KeyEvent ev = key_scan();       /* 消抖 + 状态机处理 */
        if (ev.event != KEY_EVENT_NONE) {
            g_KeyEvent = ev;           /* 更新全局事件，供其他任务读取 */
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}

/**
 * @description: NRF24L01 收发任务 —— 发送摇杆/按键数据到飞行器，同时接收回传遥测
 *     协议格式（17 字节）：
 *       帧头3B (ngm) + thr2B + yaw2B + pit2B + rol2B + shutdown1B + fix_height1B + 校验和4B
 *     连续发送失败达到阈值时触发软恢复，防止通讯锁死。
 * @param
 * @return
 */
void task2(void *pvParameters)
{

    uint8_t TxFlag = 0;
    uint8_t RxFlag = 0;
    uint8_t tx_fail_cnt = 0;     /* 连续发送失败计数 */
    #define TX_FAIL_MAX  5        /* 连续失败阈值 */
    int16_t thr, yaw, pit, rol;  /* 0~1000 范围 */
    uint8_t shutdown_flag = 0;   /* 单次触发：1=发送关机切换 */
    uint8_t fix_height_flag = 0; /* 单次触发：1=发送定高切换 */
    uint32_t sum;
    uint8_t i;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        /* ---- 处理按键事件（短按触发，单次生效） ---- */
        if (g_KeyEvent.event == KEY_EVENT_SHORT)
        {
            if (g_KeyEvent.key_id == KEY_ID_K1)
            {
                shutdown_flag = 1;
            }
            else if (g_KeyEvent.key_id == KEY_ID_K2)
            {
                fix_height_flag = 1;
            }
            g_KeyEvent.event = KEY_EVENT_NONE;  /* 消费事件，避免重复触发 */
        }

        /* ---- 1. 摇杆数据：ADC 原始 → -100~+100 → 0~1000 ---- */
        thr = (int16_t)((Joystick_DataProcess(g_Joystick.thr) + 100) * 5);
        yaw = (int16_t)((Joystick_DataProcess(g_Joystick.yaw) + 100) * 5);
        pit = (int16_t)((Joystick_DataProcess(g_Joystick.pit) + 100) * 5);
        rol = (int16_t)((Joystick_DataProcess(g_Joystick.rol) + 100) * 5);

        /* ---- 2. 打包发送数据（协议与飞行端 App_receive_data 一致） ---- */
        /* 帧头 */
        NRF24L01_TxPacket[0] = 'n';
        NRF24L01_TxPacket[1] = 'g';
        NRF24L01_TxPacket[2] = 'm';
        /* thr int16_t big-endian (0~1000) */
        NRF24L01_TxPacket[3] = (uint8_t)(thr >> 8);
        NRF24L01_TxPacket[4] = (uint8_t)(thr & 0xFF);
        /* yaw int16_t big-endian */
        NRF24L01_TxPacket[5] = (uint8_t)(yaw >> 8);
        NRF24L01_TxPacket[6] = (uint8_t)(yaw & 0xFF);
        /* pit int16_t big-endian */
        NRF24L01_TxPacket[7] = (uint8_t)(pit >> 8);
        NRF24L01_TxPacket[8] = (uint8_t)(pit & 0xFF);
        /* rol int16_t big-endian */
        NRF24L01_TxPacket[9]  = (uint8_t)(rol >> 8);
        NRF24L01_TxPacket[10] = (uint8_t)(rol & 0xFF);
        /* shutdown / fix_height 单次触发标志 */
        NRF24L01_TxPacket[11] = shutdown_flag;
        NRF24L01_TxPacket[12] = fix_height_flag;
        shutdown_flag  = 0;
        fix_height_flag = 0;

        /* ---- 3. 校验和（Byte0~12 求和，4 字节 big-endian） ---- */
        sum = 0;
        for (i = 0; i < 13; i++)
        {
            sum += NRF24L01_TxPacket[i];
        }
        NRF24L01_TxPacket[13] = (uint8_t)(sum >> 24);
        NRF24L01_TxPacket[14] = (uint8_t)(sum >> 16);
        NRF24L01_TxPacket[15] = (uint8_t)(sum >> 8);
        NRF24L01_TxPacket[16] = (uint8_t)(sum & 0xFF);

        /* ---- 4. NRF24L01 发送 ---- */
        TxFlag = NRF24L01_Send();

        /* ---- 发送失败处理 ---- */
        if (TxFlag != 1)
        {
            tx_fail_cnt++;
            if (tx_fail_cnt >= TX_FAIL_MAX)
            {
                /* 连续多次发送失败，软恢复 NRF24L01 */
                tx_fail_cnt = 0;
                NRF24L01_FlushTx();
                NRF24L01_FlushRx();
                NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
                NRF24L01_Rx();  /* 重新进入接收模式 */
            }
        }
        else
        {
            tx_fail_cnt = 0;  /* 发送成功，清零失败计数 */
        }

        /* ====== 接收部分 ====== */
        RxFlag = NRF24L01_Receive();

        /* 如果收到飞机回传的高度数据包，解析并更新 */
        if (RxFlag == 1 && NRF24L01_RxPacket[0] == 'a'
            && NRF24L01_RxPacket[1] == 'l' && NRF24L01_RxPacket[2] == 't')
        {
            int16_t alt_cm = (int16_t)((NRF24L01_RxPacket[3] << 8) | NRF24L01_RxPacket[4]);
            g_altitude = (float)alt_cm / 100.0f;
            g_plane_state = NRF24L01_RxPacket[5];
            uint16_t volt_dmv = (uint16_t)((NRF24L01_RxPacket[6] << 8) | NRF24L01_RxPacket[7]);
            g_voltage = (float)volt_dmv / 10.0f;
            g_flow_x = NRF24L01_RxPacket[8];   /* 光流前后移动标志 */
            g_flow_y = NRF24L01_RxPacket[9];   /* 光流左右移动标志 */
        }

        /* ====== OLED 显示 ====== */
        OLED_Clear();
        OLED_Printf(0,  0, OLED_8X16, "TX:%s RX:%s",
                    TxFlag == 1 ? "OK" : TxFlag == 2 ? "MAXRT" : "FAIL",
                    RxFlag == 1 ? "OK" : RxFlag == 0 ? "-" : "ERR");
        OLED_Printf(96, 0, OLED_8X16, "%.1fV", (double)g_voltage);
        OLED_Printf(0, 16, OLED_8X16, "TH:%04d YW:%04d", (int)thr, (int)yaw);
        OLED_Printf(0, 32, OLED_8X16, "PI:%04d RO:%04d", (int)pit, (int)rol);

        /* 飞行状态映射 */
        static const char *state_str[] = {"LOCKED", "IDLE", "NORMAL", "FIX_HEIGHT", "MANUAL", "FAIL"};
        const char *s = (g_plane_state <= 5) ? state_str[g_plane_state] : "?";
        OLED_Printf(80,  48, OLED_8X16, "%.1fm", (double)g_altitude);
        OLED_Printf(0,  56, OLED_6X8, "%s", s);

        /* 光流方向指示器（屏幕底部中央，16×16 图标） */
        {
            const uint8_t *flow_icon;
            if (g_flow_x && g_flow_y)
                flow_icon = FlowIcon_XY;
            else if (g_flow_x)
                flow_icon = FlowIcon_X;
            else if (g_flow_y)
                flow_icon = FlowIcon_Y;
            else
                flow_icon = FlowIcon_None;
            OLED_ShowImage(56, 47, 16, 16, flow_icon);
        }

        OLED_Update();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}





