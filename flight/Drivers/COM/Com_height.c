#include "Com_height.h"
#include "Com_imu.h"
#include "Com_filter.h"
#include <math.h>

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光 + SPA06 气压计 + MPU6050 Z轴加速度
 * 输出：融合相对高度 (m) + 垂直速度 (m/s)
 *
 * 气压计基准由 Calibrate() 独立管理，Update() 只收相对海拔。
 * ============================================================================ */

/* ============================================================================
 * 融合参数
 * ============================================================================ */

/* 高度融合 */
#define LASER_TRUST_MAX_MM      1200    /* 低空激光信任上限 (mm) */
#define LASER_BLEND_MAX_MM      2000    /* 激光→气压过渡完成上限 (mm) */
#define BARO_GROUND_MAX_M       5.0f    /* 地面判定 (m) */

/* 低通滤波 */
#define HEIGHT_LPF_FAST          0.7f   /* 激光区 LPF */
#define HEIGHT_LPF_SLOW          0.4f   /* 气压区 LPF */
#define HEIGHT_FUSION_LASER_W    0.7f   /* 低空激光权重 (70%激光 + 30%气压) */

/* 气压计校准 */
#define BARO_CAL_SAMPLES         10     /* 起飞基准采样数 */

/* 垂直速度互补滤波 */
#define VEL_DRIFT_K              0.15f  /* 漂移修正系数 */
#define VEL_LPF_ALPHA            0.3f   /* 输出低通 */
#define GRAVITY_1G               16384.0f /* MPU6050 ±16g 量程 1g ADC 值 */

/* ============================================================================
 * 模块内部状态
 * ============================================================================ */

/* 融合输出 */
float   g_fused_height    = 0.0f;       /* 融合高度 (m)，外部直接读 */
float   g_vertical_vel    = 0.0f;       /* 垂直速度 (m/s)，外部直接读 */
static uint8_t initialized       = 0;

static float   prev_height       = 0.0f;
static float   vel_from_imu      = 0.0f;
static uint8_t laser_was_ok      = 0;

/* 气压计校准 */
static float   baro_alt0         = 0.0f;   /* 起飞绝对海拔基准 (m) */
static float   baro_abs_last     = 0.0f;   /* 最近一次绝对海拔 (m) */
static uint8_t baro_cal_cnt      = 0;
static float   baro_cal_sum      = 0.0f;

/* ============================================================================
 * 公共 API
 * ============================================================================ */

void Common_Height_Init(void)
{
    g_fused_height   = 0.0f;
    prev_height      = 0.0f;
    g_vertical_vel   = 0.0f;
    vel_from_imu     = 0.0f;
    laser_was_ok     = 0;

    baro_alt0    = 0.0f;
    baro_abs_last = 0.0f;
    baro_cal_cnt = 0;
    baro_cal_sum = 0.0f;

    initialized = 1;
}

/**
 * @brief 气压计起飞基准校准
 *
 * 前 BARO_CAL_SAMPLES 次调用的绝对海拔平均值锁定为起飞基准 baro_alt0。
 *
 * @param baro_abs_m  绝对海拔 (m)
 * @retval 0  校准中（气压计融合不可用）
 * @retval 1  校准完成
 */
uint8_t Common_Height_Calibrate(float baro_abs_m)
{
    if (!initialized) Common_Height_Init();

    baro_abs_last = baro_abs_m;

    if (baro_cal_cnt < BARO_CAL_SAMPLES)
    {
        baro_cal_sum += baro_abs_m;
        baro_cal_cnt++;
        if (baro_cal_cnt >= BARO_CAL_SAMPLES)
            baro_alt0 = baro_cal_sum / (float)BARO_CAL_SAMPLES;
        return 0;
    }
    return 1;
}

/**
 * @brief 获取气压计相对高度
 *
 * 相对高度 = 绝对海拔 − 起飞基准。校准完成前返回 0。
 */
float Common_Height_GetBaroRel(void)
{
    if (baro_cal_cnt < BARO_CAL_SAMPLES) return 0.0f;
    return baro_abs_last - baro_alt0;
}

/**
 * @brief 输入传感器数据，更新融合高度和垂直速度
 *
 * @param laser_mm   激光测距 (mm)，0 = 无效
 * @param baro_rel_m 气压计相对高度 (m)
 * @param dt_s       时间间隔 (秒)
 */
void Common_Height_Update(uint16_t laser_mm, float baro_rel_m, float dt_s)
{
    if (!initialized) Common_Height_Init();
    if (dt_s <= 0.0f) dt_s = 0.024f;

    /* 异常钳位 ±200m */
    if (baro_rel_m > 200.0f || baro_rel_m < -200.0f)
        baro_rel_m = 0.0f;

    /* 输入有效性判定 */
    uint8_t laser_ok = (laser_mm > 0 && laser_mm <= LASER_BLEND_MAX_MM) ? 1 : 0;
    uint8_t baro_ok  = (baro_rel_m > 0.01f || baro_rel_m < -0.01f) ? 1 : 0;
    float   target_h;

    /* ================================================================
     * 1. 高度融合
     *
     * 激光精度高量程短 (≤2m)，气压计范围大但漂移。
     *   激光有效 → 低空融合 → 过渡切换 → 纯气压
     *   激光无效 → 气压>5m 用气压 / 否则锁定地面
     * ================================================================ */
    if (laser_ok)
    {
        float laser_m = (float)laser_mm / 1000.0f;
        uint8_t laser_first = (laser_was_ok == 0);
        laser_was_ok = 1;

        if (laser_first && g_fused_height < 0.5f)
        {
            /* 地面唤醒：直接赋值，不走 LPF */
            g_fused_height = laser_m;
        }
        else if (laser_mm <= LASER_TRUST_MAX_MM)
        {
            /* 低空 (<1.2m)：激光 70% + 气压 30%，快速 LPF */
            target_h = baro_ok
                ? laser_m * HEIGHT_FUSION_LASER_W + baro_rel_m * (1.0f - HEIGHT_FUSION_LASER_W)
                : laser_m;
            g_fused_height += HEIGHT_LPF_FAST * (target_h - g_fused_height);
        }
        else if (baro_ok)
        {
            /* 过渡区 (1.2~2m)：线性加权激光→气压，慢速 LPF */
            float alpha = (float)(laser_mm - LASER_TRUST_MAX_MM)
                        / (float)(LASER_BLEND_MAX_MM - LASER_TRUST_MAX_MM);
            target_h = laser_m * (1.0f - alpha) + baro_rel_m * alpha;
            g_fused_height += HEIGHT_LPF_SLOW * (target_h - g_fused_height);
        }
        else
        {
            /* 无气压：纯激光 */
            target_h = laser_m;
            g_fused_height += HEIGHT_LPF_FAST * (target_h - g_fused_height);
        }
    }
    else
    {
        laser_was_ok = 0;

        if (baro_ok && fabsf(baro_rel_m) > BARO_GROUND_MAX_M)
        {
            /* 高空 (>5m)：纯气压计 */
            target_h = baro_rel_m;
            g_fused_height += HEIGHT_LPF_SLOW * (target_h - g_fused_height);
        }
        else
        {
            /* 地面：锁定为 0 */
            g_fused_height = 0.0f;
        }
    }

    /* ================================================================
     * 2. 垂直速度 — 互补滤波
     *
     *   vel_imu += accZ · dt                    ← 加速度积分（高频）
     *   vel_imu += K · (vel_diff − vel_imu)     ← 高度微分修正漂移
     *   output   = LPF(vel_imu)                 ← 低通去噪
     * ================================================================ */
    {
        float vel_diff   = (g_fused_height - prev_height) / dt_s;
        float acc_z_mps2 = (Common_IMU_GetNormAccZ() - GRAVITY_1G) * 9.81f / GRAVITY_1G;

        vel_from_imu += acc_z_mps2 * dt_s;
        vel_from_imu += VEL_DRIFT_K * (vel_diff - vel_from_imu);

        g_vertical_vel = Common_Filter_LowPass_Float(
            vel_from_imu, g_vertical_vel, VEL_LPF_ALPHA);
    }

    prev_height = g_fused_height;
}

