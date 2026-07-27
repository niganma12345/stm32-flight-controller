#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "motor.h"
#include "NRF24L01.h"
#include "App_flight.h"
#include "App_receive_data.h"
#include "led.h"
#include "BlueSerial.h"
#include "App_oled.h"

extern Euler_struct euler_angle;



/* 通信超时保护：记录最后一次收到有效数据的时间戳 */
static volatile TickType_t g_last_rx_tick = 0;
#define COMM_TIMEOUT_MS  150   /* 超时阈值：150ms 无数据则自动停转 */

#define cycle_time 6
#define cycle_time1 1000
#define POWER_TASK_PERIOD 10000

// 飞行状态（volatile：多任务并发读写），上电默认锁定
volatile Flight_State flight_state = LOCKED;
// 遥控数据（volatile：nrf24l01_task 写入，flight_task 读取）
volatile Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};
// 遥控器连接状态（上电信任连接，收不到数据时自动断开）
volatile Remote_State remote_state = REMOTE_CONNECTED;
// 定高飞行的目标高度（volatile：多任务读写）
volatile float fix_height = 0.0f;
// BMP280 测量的当前海拔高度（单位: m）
volatile float g_spa06_altitude = 0.0f;
// 电池电压（单位: V），PB1 ADC读取
volatile float g_battery_voltage = 0.0f;
// 通信接收结果（供电源管理任务使用），0=收到数据，非0=未收到
volatile uint8_t g_rx_result = 1;


/**
 * @brief FreeRTOS Tick Hook - called from the SysTick ISR on every RTOS tick.
 *        We use this to keep the HAL timebase running.
 */
void vApplicationTickHook(void)
{
    HAL_IncTick();
}


#define START_TASK_STACK 128
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

//飞控任务
#define FLIGHT_TASK_STACK 512
#define FLIGHT_TASK_PRIORITY 2
TaskHandle_t flight_task_handle;
void flight_task(void *pvParameters);

//通讯任务
#define NRF24L01_TASK_STACK 256
#define NRF24L01_TASK_PRIORITY 3
TaskHandle_t nrf24l01_task_handle;
void nrf24l01_task(void *pvParameters);

//LED灯控任务
#define LED_TASK_STACK 128
#define LED_TASK_PRIORITY 1
TaskHandle_t led_task_handle;
void led_task(void *pvParameters);

//电源管理任务
#define POWER_MGMT_TASK_STACK 128
#define POWER_MGMT_TASK_PRIORITY 1
TaskHandle_t power_mgmt_task_handle;
void power_mgmt_task(void *pvParameters);

/**
 * @description:
 * @return {*}
 */
void freertos_start(void)
{

    xTaskCreate((TaskFunction_t)start_task,
                (char *)"start_task",
                (configSTACK_DEPTH_TYPE)START_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY,
                (TaskHandle_t *)&start_task_handle);


    vTaskStartScheduler();
}

/**
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void start_task(void *pvParameters)
{

    /* 初始化蓝牙串口 */
    BlueSerial_Init();

    xTaskCreate((TaskFunction_t)flight_task,
                (char *)"flight_task",
                (configSTACK_DEPTH_TYPE)FLIGHT_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)FLIGHT_TASK_PRIORITY,
                (TaskHandle_t *)&flight_task_handle);
    xTaskCreate((TaskFunction_t)nrf24l01_task,
                (char *)"nrf24l01_task",
                (configSTACK_DEPTH_TYPE)NRF24L01_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)NRF24L01_TASK_PRIORITY,
                (TaskHandle_t *)&nrf24l01_task_handle);
    xTaskCreate((TaskFunction_t)led_task,
                (char *)"led_task",
                (configSTACK_DEPTH_TYPE)LED_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)LED_TASK_PRIORITY,
                (TaskHandle_t *)&led_task_handle);
    xTaskCreate((TaskFunction_t)power_mgmt_task,
                (char *)"power_mgmt_task",
                (configSTACK_DEPTH_TYPE)POWER_MGMT_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)POWER_MGMT_TASK_PRIORITY,
                (TaskHandle_t *)&power_mgmt_task_handle);

    /* OLED: 先创建队列，再创建任务 */
    App_OLED_Init();
    xTaskCreate((TaskFunction_t)App_OLED_Task,
                (char *)"oled_task",
                (configSTACK_DEPTH_TYPE)APP_OLED_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)APP_OLED_TASK_PRIORITY,
                (TaskHandle_t *)NULL);

    vTaskDelete(NULL);

}

/**
 * @description: 飞行控制任务，采集传感器数据并解算姿态角
 * @param
 * @return
 */
void flight_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    App_flight_init();
    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        /* ---- 通信超时保护 ---- */
        if ((flight_state == NORMAL || flight_state == FIX_HEIGHT || flight_state == MANUAL) &&
            (now - g_last_rx_tick) > pdMS_TO_TICKS(COMM_TIMEOUT_MS))
        {
            flight_state = FAIL;
        }

        // 1. 姿态解算
        App_flight_get_euler_angle();

        /* 姿态显示 */
        App_OLED_Postf(0, OLED_ROW_0, OLED_8X16,
                       "P:%.1f R:%.1f", euler_angle.pitch, euler_angle.roll);

        // 2. 磁力计（读取 + 航向 + 硬铁校准）
        App_flight_process_mag();

        // 3. 光流/陀螺/高度传感器（内部 30ms 分频）
        App_flight_process_flow_sensors();

        // 4. PID 控制
        App_flight_pid_process();

        // 5. 定高 PID（每 24ms = 4 周期）
        {
            static uint8_t height_tick = 0;
            if (++height_tick >= 4)
            {
                height_tick = 0;
                App_flight_fix_height_pid_process();
            }
        }

        // 6. 电机输出
        App_flight_control_motor();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(cycle_time));
    }
}

/**
 * @description: NRF24L01 收发合并任务
 *     使用 App_receive_data 模块完成接收、校验、应答和飞行状态管理。
 *     NRF24L01_Send() 发送完成后自动切回接收模式，不会阻塞接收。
 *     收到有效数据时更新 g_last_rx_tick，供 flight_task 做超时判断。
 * @param
 * @return
 */
void nrf24l01_task(void *pvParameters)
{

    NRF24L01_Init();
    g_last_rx_tick = xTaskGetTickCount();  /* 初始时间戳 */

    /* BMP280 已在 App_flight_init 中初始化，此处仅标记可用 */
    uint8_t         spa06_ok_local = 1;
    static TickType_t last_alt_tick = 0;
    last_alt_tick = xTaskGetTickCount();

    while (1)
    {
        /* ---- 接收遥控数据、校验、应答（由 App_receive_data 模块完成） ---- */
        g_rx_result = App_receive_data();

        /* ---- 处理遥控器连接状态 ---- */
        App_process_connect_state(g_rx_result);

        /* ---- 处理飞行状态机（解锁/定高/故障） ---- */
        App_process_flight_state();
			
			 // 3. 处理关机命令
        if (remote_data.shutdown == 1)
        {
            // 使用freeRTOS直接任务通知 => 通知电源任务 => 执行关机
            xTaskNotifyGive(power_mgmt_task_handle);
        }


        /* 接收成功时更新最后收包时间戳 */
        if (g_rx_result == 0)
        {
            g_last_rx_tick = xTaskGetTickCount();

            /* ---- 每1秒读取BMP280高度和电池电压并回传 ---- */
            TickType_t now = xTaskGetTickCount();
            if (spa06_ok_local && (now - last_alt_tick) >= pdMS_TO_TICKS(200))
            {
                last_alt_tick = now;
                App_send_telemetry();
            }
        }


//        /* ---- 每 ~102ms 蓝牙输出：高度 + 光流真实位移（使用 flight_task 已处理的数据）---- */
//        {
//            static uint8_t  blue_tick = 0;
//            if (++blue_tick >= 17)   /* 17 × 6ms ≈ 102ms */
//            {
//                blue_tick = 0;

//                BlueSerial_Printf("[plot,%d,%d,%d]",
//                                  g_flow_height_mm,
//                                  (int)g_flow_data.vx,
//                                  (int)g_flow_data.vy);
//							BlueSerial_Printf(":%d,%d,%d\n",
//                                  g_flow_height_mm,
//                                  (int)g_flow_data.vx,
//                                  (int)g_flow_data.vy);
//            }
//        }

        vTaskDelay(6);

    }
}

/**
 * @description: LED 灯控任务
 *               每6ms调用 Led_Process() 更新 LED0/LED1 状态
 *               LED0: 通讯状态指示 — 连接时常亮，断开500ms后熄灭
 *               LED1: 飞行状态指示 — LOCKED慢闪，解锁后常亮，故障灭
 * @param {void} *pvParameters
 * @return {*}
 */
void led_task(void *pvParameters)
{
    Led_Init();

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        Led_Process();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(6));
    }
}

/**
 * @description: TP4336 电源管理任务
 *               每10秒给PB2一个下拉脉冲保持供电
 *               遥控K1按下后 → 双击脉冲关机
 * @param {void} *pvParameters
 * @return {*}
 */
void power_mgmt_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
			 uint32_t res = ulTaskNotifyTake(pdTRUE, POWER_TASK_PERIOD);
        if (res != 0)
        {
            // 两次短按关机(100ms低/200ms高/100ms低)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        }
        else
        {
            // 单次短按保持供电
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
					  vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(9000));
        }
    }
}
