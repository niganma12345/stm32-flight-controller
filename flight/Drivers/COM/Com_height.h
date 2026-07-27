#ifndef __COMMON_HEIGHT_H
#define __COMMON_HEIGHT_H

#include "Com_config.h"
#include <stdint.h>

/* ============================================================================
 * Com_height — 高度计算层（激光+气压计融合 + 加速度互补垂直速度估计）
 * ============================================================================
 *
 * 与 Com_imu（姿态解算）同级的独立 COM 模块，对外提供融合高度和垂直速度。
 *
 * 使用方式：
 *   1. 初始化时调用 Common_Height_Init()
 *   2. 每次拿到新传感器数据时调用 Common_Height_Update(laser_mm, baro_m, dt_s)
 *   3. 随时通过 Common_Height_GetFused() / GetVelocity() 读取结果
 *
 * 融合策略：
 *   激光=0 + 气压<5m  → 地面锁定，高度=0（消除地面气压漂移）
 *   激光=0 + 气压>5m  → 纯气压计（高空激光超量程安全保护）
 *   激光有效 + 首次从地面唤醒 → 直接赋值为激光值（消除起飞跳变）
 *   < 1.2m → 激光+气压LPF融合（低空精确悬停）
 *   1.2~2m → 线性加权混合激光→气压计
 *   > 2m   → 纯气压计
 *
 * 垂直速度：
 *   高度微分（低频绝对参考） + Z轴加速度 normAccz（高频动态响应）互补融合
 * ============================================================================ */

/**
 * @brief 初始化高度计算模块
 */
void Common_Height_Init(void);

/**
 * @brief 输入最新传感器原始数据，内部更新融合高度和垂直速度
 *
 * @param laser_mm         VL53L1X 测距值 (mm)，0 表示无效/超量程
 * @param baro_altitude_m  BMP280 气压计海拔 (m)
 * @param dt_s             距上次 Update 的时间间隔 (秒)
 */
void Common_Height_Update(uint16_t laser_mm, float baro_altitude_m, float dt_s);

/**
 * @brief 获取融合后的高度（始终 ≥0）
 * @return 高度 (m)
 */
float Common_Height_GetFused(void);

/**
 * @brief 获取垂直速度估计
 * @return 垂直速度 (m/s)，正值 = 上升，负值 = 下降
 */
float Common_Height_GetVelocity(void);

#endif /* __COMMON_HEIGHT_H */
