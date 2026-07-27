#include "Com_height.h"
#include "Com_imu.h"
#include "Com_filter.h"

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光测距 (mm) + SPA06 气压计相对高度 (m) + MPU6050 Z轴加速度
 * 输出：融合高度 (m) + 垂直速度 (m/s)
 *
 * 融合策略概要：
 *   激光有效 ─┬─ 首次从地面唤醒 → 直接赋值（消除起飞跳变）
 *             ├─ ≤1.2m 低空区 → 激光70% + 气压30% 快速LPF
 *             ├─ 1.2~2m 过渡区 → 线性加权切到气压计 慢速LPF
 *             └─ 其他 → 纯激光兜底
 *   激光无效 ─┬─ 气压>5m → 纯气压计（高空安全，激光超量程）
 *             └─ 气压≤5m → 锁定为0（地面消除气压漂移）
 *
 * 垂直速度：高度微分（低频准） + Z轴加速度积分（高频快）互补滤波
 * ============================================================================ */

/* ============================================================================
 * 融合参数
 * ============================================================================ */

/* 高度融合阈值 */
#define LASER_TRUST_MAX_MM    1200    /* 低空纯激光信任上限 (mm) */
#define LASER_BLEND_MAX_MM    2000    /* 激光→气压过渡完成上限 (mm) */
#define BARO_GROUND_MAX_M     5.0f    /* 地面判定 (m)：气压低于此值 + 激光无效 → 锁定地面 */

/* 低通滤波系数 (0~1，越大响应越快、噪声越多) */
#define HEIGHT_LPF_FAST        0.7f   /* 激光区快速LPF */
#define HEIGHT_LPF_SLOW        0.4f   /* 气压区慢速LPF */
#define HEIGHT_FUSION_LASER_W  0.7f   /* 低空区激光融合权重，余下30%来自气压计 */

/* 垂直速度互补滤波 */
#define VEL_DRIFT_K            0.15f  /* 加速度积分漂移修正系数：越大越信任高度微分 */
#define VEL_LPF_ALPHA          0.3f   /* 速度输出低通 (0~1) */
#define GRAVITY_1G             16384.0f /* MPU6050 ±16g 量程的 1g ADC 值 */

/* ============================================================================
 * 模块内部状态
 * ============================================================================ */
static float   fused_height_m    = 0.0f;   /* 融合高度 (m)，相对起飞点 */
static float   vertical_vel_mps  = 0.0f;   /* 垂直速度 (m/s)，正值上升 */
static uint8_t initialized       = 0;      /* 是否已初始化 */

static float   prev_height       = 0.0f;   /* 上一帧融合高度，用于微分求速度 */
static float   vel_from_imu      = 0.0f;   /* 加速度积分的垂直速度 (m/s) */
static uint8_t laser_was_ok      = 0;      /* 上一帧激光是否有效，用于检测跳变沿 */

/* ============================================================================
 * 公共 API
 * ============================================================================ */

/**
 * @brief 初始化/重置高度模块
 *
 * 起飞前调用，将所有内部状态归零，确保微分和积分都从干净的起点开始。
 */
void Common_Height_Init(void)
{
    fused_height_m   = 0.0f;
    prev_height      = 0.0f;
    vertical_vel_mps = 0.0f;
    vel_from_imu     = 0.0f;
    initialized      = 1;
}

/**
 * @brief 输入传感器原始数据，更新融合高度和垂直速度
 *
 * @param laser_mm         VL53L1X 测距值 (mm)，0 表示无效/超量程
 * @param baro_altitude_m  SPA06 相对高度 (m)，已减起飞基准
 * @param dt_s             距上次调用间隔 (秒)，用于速度微分和加速度积分
 */
void Common_Height_Update(uint16_t laser_mm, float baro_altitude_m, float dt_s)
{
    uint8_t laser_ok = (laser_mm > 0 && laser_mm <= LASER_BLEND_MAX_MM) ? 1 : 0;
    uint8_t baro_ok  = (baro_altitude_m > 0.01f || baro_altitude_m < -0.01f) ? 1 : 0;
    float   target_h;

    if (!initialized) Common_Height_Init();
    if (dt_s <= 0.0f) dt_s = 0.024f;   /* 兜底 1/42s，防止除零 */

    /* ================================================================
     * 1. 高度融合
     *
     * 核心思路：激光精度高但量程短 (≤2m)，气压计范围大但漂移。
     * 不同高度段采用不同策略，保证低空悬停精度和高空安全性。
     * ================================================================ */
    if (laser_ok)
    {
        float laser_m = (float)laser_mm / 1000.0f;

        /* 检测激光从无效→有效的跳变沿，用于起飞唤醒 */
        uint8_t laser_first = (laser_was_ok == 0);
        laser_was_ok = 1;

        if (laser_first && fused_height_m < 0.5f)
        {
            /* 从地面唤醒：直接赋激光值，不走 LPF，消除起飞瞬间的滞后跳变 */
            fused_height_m = laser_m;
        }
        else if (laser_mm <= LASER_TRUST_MAX_MM)
        {
            /* 低空精确悬停区 (<1.2m)：激光为主 (70%)，气压为辅 (30%)，快速响应 */
            if (baro_ok)
                target_h = laser_m * HEIGHT_FUSION_LASER_W
                         + baro_altitude_m * (1.0f - HEIGHT_FUSION_LASER_W);
            else
                target_h = laser_m;
            fused_height_m += HEIGHT_LPF_FAST * (target_h - fused_height_m);
        }
        else if (baro_ok)
        {
            /* 过渡区 (1.2~2m)：线性加权从激光平滑切换到气压计，慢速过渡 */
            float alpha = (float)(laser_mm - LASER_TRUST_MAX_MM)
                        / (float)(LASER_BLEND_MAX_MM - LASER_TRUST_MAX_MM);
            target_h = laser_m * (1.0f - alpha) + baro_altitude_m * alpha;
            fused_height_m += HEIGHT_LPF_SLOW * (target_h - fused_height_m);
        }
        else
        {
            /* 激光有效但气压无效/过渡区无气压：纯激光兜底 */
            target_h = laser_m;
            fused_height_m += HEIGHT_LPF_FAST * (target_h - fused_height_m);
        }
    }
    else
    {
        /* 激光无效，标记状态供下次跳变沿检测 */
        laser_was_ok = 0;

        if (baro_ok && fabsf(baro_altitude_m) > BARO_GROUND_MAX_M)
        {
            /* 高空 (>5m)：激光超量程，纯气压计兜底（飞行中气压远>5m，不会误入地面分支） */
            target_h = baro_altitude_m;
            fused_height_m += HEIGHT_LPF_SLOW * (target_h - fused_height_m);
        }
        else
        {
            /* 地面 (<5m)：激光无效且气压低 → 确认为地面，锁定高度为 0，消除气压漂移 */
            fused_height_m = 0.0f;
        }
    }

    /* ================================================================
     * 2. 垂直速度估计 — 互补滤波
     *
     *   vel_imu  = ∫(accZ - 1g) dt           ← 高频响应好，会漂移
     *   vel_diff = (h_now - h_prev) / dt      ← 低频准，高频噪声大
     *
     *   vel_imu += K · (vel_diff - vel_imu)   ← 用高度微分修正漂移
     *   output  = LPF(vel_imu)                ← 低通去噪
     * ================================================================ */
    {
        float vel_diff  = (fused_height_m - prev_height) / dt_s;
        float acc_z_mps2 = (Common_IMU_GetNormAccZ() - GRAVITY_1G) * 9.81f / GRAVITY_1G;

        vel_from_imu += acc_z_mps2 * dt_s;                              /* Z轴加速度积分 */
        vel_from_imu += VEL_DRIFT_K * (vel_diff - vel_from_imu);        /* 高度微分修正漂移 */

        vertical_vel_mps = Common_Filter_LowPass_Float(
            vel_from_imu, vertical_vel_mps, VEL_LPF_ALPHA);
    }

    prev_height = fused_height_m;
}

/**
 * @brief 获取融合后的相对高度（始终 ≥0）
 * @return 高度 (m)，负值钳位为 0
 */
float Common_Height_GetFused(void)
{
    if (!initialized) Common_Height_Init();
    if (fused_height_m < 0.0f) return 0.0f;
    return fused_height_m;
}

/**
 * @brief 获取垂直速度估计
 * @return 垂直速度 (m/s)，正值 = 上升，负值 = 下降
 */
float Common_Height_GetVelocity(void)
{
    return vertical_vel_mps;
}
