#ifndef APP_FLIGHT_H
#define APP_FLIGHT_H

#include "math.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "Com_imu.h"
#include "Com_pid.h"
#include "Com_flow.h"
#include "Com_height.h"
#include "motor.h"
#include "mpu6050.h"
#include "bmp280.h"

/* ---- 全局光流数据（flight_task 写入，nrf24l01_task 蓝牙输出）---- */
extern Flow_Data_t g_flow_data;
extern uint16_t   g_flow_height_mm;

/* ---- SPA06 状态 (0=不可用, 1=可用) ---- */
extern uint8_t g_spa06_ok;
extern float   g_baro_alt0;

/**
 * @brief 飞控任务初始化 MPU6050初始化    启动电机
 */
void App_flight_init(void);

/**
 * @brief 根据陀螺仪测量的数据 计算出欧拉角
 */
void App_flight_get_euler_angle(void);

/**
 * @brief 根据欧拉角 + 光流速度 计算出PID的目标值
 *        自稳/定高模式下光流有效时自动叠加水平悬停锁定修正
 *        手动模式下不使用光流，纯角度自稳
 */
void App_flight_pid_process(void);

/**
 * @brief 根据PID的输出值 控制电机
 */
void App_flight_control_motor(void);

/**
 * @brief 定高模式高度PID计算（串级：位置外环 + 速度内环）
 */
void App_flight_fix_height_pid_process(void);

/**
 * @brief  水平零偏校准 — 平放飞机后调用，自动计算并补偿倾角零偏
 */
void App_flight_calibrate_level(void);

#endif // __APP_FLIGHT__
