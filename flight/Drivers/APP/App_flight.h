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
#include "spa06.h"

/* ---- 最大倾斜角限制（自稳模式安全保护）---- */
#define MAX_TILT_ANGLE  30.0f   /* 俯仰/横滚最大目标角度 ±30° */

/* ---- 全局光流数据（flight_task 写入，nrf24l01_task 蓝牙输出）---- */
extern Flow_Data_t g_flow_data;
extern uint16_t   g_flow_height_mm;

/* ---- SPA06 状态 (0=可用, 非0=Init返回值) ---- */
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

/**
 * @brief 磁力计处理 — QMC5883P读取 + 航向计算 + 硬铁校准
 *        每 6ms 调用一次
 */
void App_flight_process_mag(void);

/**
 * @brief 光流/陀螺/高度传感器处理
 *        每 6ms 调用，内部 30ms 分频：
 *          - 陀螺仪累积 → 30ms 均值（旋转补偿用）
 *          - VL53L1X + SPA06 高度融合
 *          - PMW3901 光流 → 旋转补偿 → 高度补偿 → 速度解算
 *          - 加速度 → 水平速度互补滤波
 *          - OLED 调试刷新
 */
void App_flight_process_flow_sensors(void);

#endif // __APP_FLIGHT__
