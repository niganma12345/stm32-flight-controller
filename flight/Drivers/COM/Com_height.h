#ifndef __COMMON_HEIGHT_H
#define __COMMON_HEIGHT_H

#include "Com_config.h"
#include <stdint.h>

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光测距 + SPA06 绝对海拔 + MPU6050 Z轴加速度
 * 输出：融合相对高度 (m) + 垂直速度 (m/s)
 *
 * 模块内部自动完成气压计起飞基准采集，APP 层只需透传 spa06.altitude 即可。
 *
 * 融合策略：
 *   激光=0 + 气压<5m  → 地面锁定
 *   激光=0 + 气压>5m  → 纯气压计（高空安全）
 *   激光有效             → 低空激光+气压融合 → 过渡区线性切换 → 纯气压
 *
 * 垂直速度：高度微分 + Z轴加速度积分互补滤波
 * ============================================================================ */

/**
 * @brief 初始化/重置高度模块（含气压计起飞基准）
 */
void Common_Height_Init(void);

/**
 * @brief 输入传感器原始数据，更新融合高度和垂直速度
 *
 * @param laser_mm   VL53L1X 测距值 (mm)，0 = 无效/超量程
 * @param baro_abs_m SPA06 绝对海拔 (m)，模块内自动减起飞基准
 * @param dt_s       距上次调用间隔 (秒)
 */
void Common_Height_Update(uint16_t laser_mm, float baro_abs_m, float dt_s);

/**
 * @brief 获取融合后的高度（始终 ≥0）
 * @return 高度 (m)，相对起飞点
 */
float Common_Height_GetFused(void);

/**
 * @brief 获取气压计相对高度（仅供调试显示）
 * @return 气压计相对高度 (m)，未校准时返回 0
 */
float Common_Height_GetBaroRel(void);

/**
 * @brief 获取垂直速度估计
 * @return 垂直速度 (m/s)，正值 = 上升
 */
float Common_Height_GetVelocity(void);

#endif /* __COMMON_HEIGHT_H */
