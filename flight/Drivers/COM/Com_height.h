#ifndef __COMMON_HEIGHT_H
#define __COMMON_HEIGHT_H

#include "Com_config.h"
#include <stdint.h>

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光测距 + SPA06 气压计 + MPU6050 Z轴加速度
 * 输出：融合相对高度 (m) + 垂直速度 (m/s)
 *
 * 气压计校准由模块内部管理：
 *   1. 初始化时调用 Common_Height_Init()
 *   2. 每周期先调用 Common_Height_Calibrate(绝对海拔) 完成起飞基准采集
 *   3. 再调用 Common_Height_Update(激光, 相对海拔, dt) 进行融合
 *   4. 相对海拔通过 Common_Height_GetBaroRel() 获取
 *
 * 融合策略：
 *   激光=0 + 气压<5m  → 地面锁定
 *   激光=0 + 气压>5m  → 纯气压计
 *   激光有效             → 低空激光+气压融合 → 过渡切换 → 纯气压
 *
 * 垂直速度：高度微分 + Z轴加速度积分互补滤波
 * ============================================================================ */

void Common_Height_Init(void);

/**
 * @brief 气压计起飞基准校准（每周期调用）
 *
 * 传入 SPA06 绝对海拔，前 10 次自动锁定为起飞基准。
 * 校准完成后仅存储数据，不再修改基准。
 *
 * @param baro_abs_m  SPA06 绝对海拔 (m)
 * @retval 0  校准中
 * @retval 1  校准完成
 */
uint8_t Common_Height_Calibrate(float baro_abs_m);

/**
 * @brief 输入传感器数据，更新融合高度和垂直速度
 *
 * @param laser_mm    VL53L1X 测距值 (mm)，0 = 无效
 * @param baro_rel_m  气压计相对高度 (m)，校准完成前传 0
 * @param dt_s        距上次调用间隔 (秒)
 */
void Common_Height_Update(uint16_t laser_mm, float baro_rel_m, float dt_s);

float Common_Height_GetFused(void);
float Common_Height_GetBaroRel(void);
float Common_Height_GetVelocity(void);

#endif
