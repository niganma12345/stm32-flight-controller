#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "motor.h"
#include "NRF24L01.h"
#include "App_flight.h"
#include "App_receive_data.h"
#include "led.h"
#include "PMW3901.h"
#include "vl53l1.h"
#include "BlueSerial.h"
#include "Int_VL53L1X.h"
#include "Com_flow.h"
#include "Com_height.h"
#include "App_oled.h"
#include "QMC5883P.h"
#include "QMC_MagCal.h"
#include <string.h>
#include <math.h>

extern Euler_struct euler_angle;
extern QMC_MagCal_t g_mag_cal;
extern uint8_t      g_mag_calibrated;
extern int16_t      g_mag_ofs_x;
extern int16_t      g_mag_ofs_y;



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
volatile float g_bmp280_altitude = 0.0f;
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
            flight_state = FAIL;  /* 通信丢失，自动进入故障减速模式 */
        }

        // 1. 获根据MPU6050测量的数据  姿态解算得到欧拉角
        App_flight_get_euler_angle();

        /* ---- 姿态显示到 OLED 第1行 ---- */
        App_OLED_Postf(0, OLED_ROW_0, OLED_8X16,
                       "P:%.1f R:%.1f", euler_angle.pitch, euler_angle.roll);

        /* ---- QMC5883P 每 6ms 读取 + 校准采集 ---- */
        QMC5883P_ReadData(&hi2c2);

        /* 计算航向（校准前裸数据，校准后用偏移修正后数据） */
        {
            float mx_f = (float)qmc.mx - (g_mag_calibrated ? (float)g_mag_ofs_x : 0.0f);
            float my_f = (float)qmc.my - (g_mag_calibrated ? (float)g_mag_ofs_y : 0.0f);
            float hd   = atan2f(my_f, mx_f) * 57.29578f;
            if (hd < 0.0f) hd += 360.0f;
            qmc.heading = hd;
        }

        /* 硬铁校准 (上电 5 秒内旋转飞机 360°，6ms 采样 ≈ 833 个样点) */
        {
            static TickType_t cal_start = 0;
            if (cal_start == 0) cal_start = xTaskGetTickCount();

            QMC_MagCal_Feed(&g_mag_cal, qmc.mx, qmc.my);

            if (!g_mag_calibrated)
            {
                TickType_t now   = xTaskGetTickCount();
                int32_t    left  = 5000 - (int32_t)((now - cal_start) * portTICK_PERIOD_MS);
                if (left <= 0)
                {
                    QMC_MagCal_Lock(&g_mag_cal);
                    g_mag_ofs_x     = g_mag_cal.mx_offset;
                    g_mag_ofs_y     = g_mag_cal.my_offset;
                    g_mag_calibrated = 1;
                }
                else
                {
                    /* 每 ~300ms 刷新一次校准界面，避免 6ms 频率刷屏黑掉 */
                    static uint8_t cal_tick = 0;
                    if (++cal_tick >= 17)   /* 17 × 6ms ≈ 100ms */
                    {
                        cal_tick = 0;
                        App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
                            "%ds X:%d Y:%d", left / 1000 + 1, qmc.mx, qmc.my);
                        App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
                            "X[%d,%d] Y[%d,%d]",
                            g_mag_cal.mx_min, g_mag_cal.mx_max,
                            g_mag_cal.my_min, g_mag_cal.my_max);
                    }
                }
            }
        }

        /* ---- 1.5 光流 + 陀螺仪累积 ---- */
        {
            static uint8_t  flow_tick = 0;
            static int32_t  gyro_sum_x = 0;  /* 30ms陀螺仪累积 (ADC counts) */
            static int32_t  gyro_sum_y = 0;

            /* 每个周期累积陀螺仪，用于旋转补偿 */
            {
                extern Gyro_Accel_Struct gyro_accel_data;
                gyro_sum_x += gyro_accel_data.gyro.gyro_x;
                gyro_sum_y += gyro_accel_data.gyro.gyro_y;
            }

            if (++flow_tick >= 5)
            {
                flow_tick = 0;

                /* 计算 30ms 陀螺仪均值，转为 °/s (MPU6050 ±2000°/s量程) */
                float gyro_x_dps = (float)gyro_sum_x / 5.0f * (2000.0f / 32768.0f);
                float gyro_y_dps = (float)gyro_sum_y / 5.0f * (2000.0f / 32768.0f);
                gyro_sum_x = 0;
                gyro_sum_y = 0;

                /* VL53L1X + SPA06 → 更新高度计算层（激光+气压计融合） */
                {
                    uint16_t laser_mm = Int_VL53L1X_GetDistance();
                    float    baro_m   = 0.0f;

                    if (g_spa06_ok)
                    {
                        SPA06_ReadData(&hi2c2);
                        SPA06_ComputeAltitude();

                        /* 前 10 次读数累积取平均作为起飞点海拔基准 */
                        static uint8_t  baro_cal_cnt = 0;
                        static float    baro_cal_sum = 0.0f;
                        if (baro_cal_cnt < 10)
                        {
                            baro_cal_sum += spa06.altitude;
                            baro_cal_cnt++;
                            if (baro_cal_cnt == 10)
                                g_baro_alt0 = baro_cal_sum / 10.0f;
                        }

                        baro_m = spa06.altitude - g_baro_alt0;

                        /* 异常值钳位: 相对高度不超过 ±200m */
                        if      (baro_m >  200.0f) baro_m = 0.0f;
                        else if (baro_m < -200.0f) baro_m = 0.0f;
                    }

                    Common_Height_Update(laser_mm, baro_m, 0.030f);
                    g_flow_height_mm  = Common_Height_GetFusedMM();

                    {
                        float h = Common_Height_GetFused();
                        App_OLED_Postf(0, OLED_ROW_1, OLED_6X8,
                            "B:%d L:%d F:%d",
                            (int)(baro_m * 100.0f),
                            (int)(laser_mm / 10),
                            (int)(h * 100.0f));
                    }
                }

                /* PMW3901 读取原始运动数据 */
                PMW3901_MotionData_t motion;
                PMW3901_ReadMotion(&motion);

                /* COM 层分步解算：映射 + 旋转补偿 + 高度 + 速度 */
                Com_Flow_MapAxis(&motion, &g_flow_data);
                Com_Flow_RemoveRotation(&g_flow_data, gyro_x_dps, gyro_y_dps, 0.030f);
                Com_Flow_ApplyHeightScale(&g_flow_data, g_flow_height_mm);
                Com_Flow_CalcVelocity(&g_flow_data, 30.0f);

                /* 机体加速度换算 (±2g 量程, 16384 LSB/g → m/s²) */
                {
                    extern Gyro_Accel_Struct gyro_accel_data;
                    float acc_x = gyro_accel_data.accel.accel_x * 9.81f / 16384.0f;
                    float acc_y = gyro_accel_data.accel.accel_y * 9.81f / 16384.0f;
                    float acc_z = gyro_accel_data.accel.accel_z * 9.81f / 16384.0f;

                    /* 水平速度互补滤波 — 融合加速度积分与光流速度 */
                    static FlowVelFusion_t g_flow_vel_fusion = {0};

                    /* 非飞行状态时清零融合状态，避免恢复时积分冲击 */
                    if (flight_state == LOCKED || flight_state == IDLE || flight_state == MANUAL)
                    {
                        g_flow_vel_fusion.vel_x      = 0.0f;
                        g_flow_vel_fusion.vel_y      = 0.0f;
                        g_flow_vel_fusion.acc_bias_x = 0.0f;
                        g_flow_vel_fusion.acc_bias_y = 0.0f;
                    }

                    Com_Flow_VelocityFusion(&g_flow_vel_fusion,
                        g_flow_data.vx, g_flow_data.vy,
                        acc_x, acc_y, acc_z,
                        euler_angle.pitch, euler_angle.roll,
                        0.030f);

                    /* 用融合速度替换原始光流速度（PID 层透明） */
                    g_flow_data.vx = g_flow_vel_fusion.vel_x;
                    g_flow_data.vy = g_flow_vel_fusion.vel_y;
                }

                /* OLED 第3行: 光流物理位移 */
                if (g_mag_calibrated)
                {
                    App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
                        "FB:%d LR:%d S:%d",
                        (int)g_flow_data.pos.x,
                        (int)g_flow_data.pos.y,
                        g_flow_data.squal);
                }

                /* OLED 第4行: 融合航向 vs 磁力计航向 */
                if (g_mag_calibrated)
                {
                    App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
                        "Y:%.0f M:%.0f", euler_angle.yaw, qmc.heading);
                }
            }
        }

        // 2. 根据当前的欧拉角 + 光流速度 进行PID计算控制
        App_flight_pid_process();

        // 3. 定高PID计算（每24ms执行一次 = 每4个周期）
        static uint8_t height_tick = 0;
        if (++height_tick >= 4)
        {
            height_tick = 0;
            App_flight_fix_height_pid_process();
        }

        // 4. 根据PID输出和飞行状态 控制电机
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
    uint8_t         bmp280_ok     = 1;
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
            if (bmp280_ok && (now - last_alt_tick) >= pdMS_TO_TICKS(200))
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
//                                  (int16_t)g_flow_data.pos.x,
//                                  (int16_t)g_flow_data.pos.y);
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
