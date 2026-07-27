#ifndef __COMMON_HEIGHT_H
#define __COMMON_HEIGHT_H

#include "Com_config.h"
#include <stdint.h>

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光测距 + SPA06 气压计 + MPU6050 Z轴加速度
 * 输出：g_fused_height 融合相对高度 (m) + g_vertical_vel 垂直速度 (m/s)
 *
 * 使用顺序：
 *   1. Common_Height_Init()               — 初始化
 *   2. Common_Height_Calibrate(绝对海拔)   — 每周期校准起飞基准
 *   3. Common_Height_Update(激光, 相对, dt) — 融合计算，结果写入全局变量
 *   4. Common_Height_GetBaroRel()         — 获取气压计相对高度（调试）
 *
 * 融合策略：
 *   激光=0 + 气压<5m  → 地面锁定
 *   激光=0 + 气压>5m  → 纯气压计（高空安全）
 *   激光有效             → 低空激光+气压融合 → 过渡线性切换 → 纯气压
 *
 * 垂直速度：高度微分 + Z轴加速度积分互补滤波
 * ============================================================================ */

void    Common_Height_Init(void);
uint8_t Common_Height_Calibrate(float baro_abs_m);
void    Common_Height_Update(uint16_t laser_mm, float baro_rel_m, float dt_s);
float   Common_Height_GetBaroRel(void);

/* 模块输出：Update 后直接读取 */
extern float g_fused_height;   /* 融合相对高度 (m)，始终 ≥0 */
extern float g_vertical_vel;   /* 垂直速度 (m/s)，正值上升 */

#endif
