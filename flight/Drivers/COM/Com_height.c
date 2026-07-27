#include "Com_height.h"
#include "Com_imu.h"
#include "Com_filter.h"
#include <math.h>

/* ============================================================================
 * Com_height — 高度融合与垂直速度估计
 *
 * 输入：VL53L1X 激光 + SPA06 绝对海拔 + MPU6050 Z轴加速度
 * 输出：融合相对高度 (m) + 垂直速度 (m/s)
 *
 * 模块内部自动完成气压计起飞基准校准：前 10 次 Update 的 baro_abs 平均值锁定为
 * 起飞点基准 baro_alt0，后续 baro_rel = baro_abs − baro_alt0。
 * ============================================================================ */

/* ============================================================================
 * 融合参数
 * ============================================================================ */

/* 高度融合阈值 */
#define LASER_TRUST_MAX_MM      1200    /* 低空激光信任上限 (mm) */
#define LASER_BLEND_MAX_MM      2000    /* 激光→气压过渡完成上限 (mm) */
#define BARO_GROUND_MAX_M       5.0f    /* 地面判定 (m)：气压相对高度低于此值 + 激光无效 → 锁定地面 */

/* 低通滤波系数 (0~1，越大响应越快) */
#define HEIGHT_LPF_FAST          0.7f   /* 激光区快速 LPF */
#define HEIGHT_LPF_SLOW          0.4f   /* 气压区慢速 LPF */
#define HEIGHT_FUSION_LASER_W    0.7f   /* 低空区激光融合权重 (70%激光 + 30%气压) */

/* 气压计起飞基准校准 */
#define BARO_CAL_SAMPLES         10     /* 起飞前累积采样数 */

/* 垂直速度互补滤波 */
#define VEL_DRIFT_K              0.15f  /* 加速度积分漂移修正：越大越信高度微分 */
#define VEL_LPF_ALPHA            0.3f   /* 速度输出低通 */
#define GRAVITY_1G               16384.0f /* MPU6050 ±16g 量程的 1g ADC 值 */

/* ============================================================================
 * 模块内部状态
 * ============================================================================ */
static float   fused_height_m    = 0.0f;   /* 融合高度 (m) */
static float   vertical_vel_mps  = 0.0f;   /* 垂直速度 (m/s) */
static uint8_t initialized       = 0;      /* 已初始化标志 */

static float   prev_height       = 0.0f;   /* 上一帧高度，用于微分 */
static float   vel_from_imu      = 0.0f;   /* 加速度积分速度 (m/s) */
static uint8_t laser_was_ok      = 0;      /* 上一帧激光是否有效 */

/* 气压计起飞基准 */
static float   baro_alt0         = 0.0f;   /* 起飞点绝对海拔基准 (m) */
static float   baro_abs_current  = 0.0f;   /* 最近一次绝对海拔 (m)，供 GetBaroRel 读取 */
static uint8_t baro_cal_cnt      = 0;      /* 基准校准计数 */
static float   baro_cal_sum      = 0.0f;   /* 基准累加器 */

/* ============================================================================
 * 公共 API
 * ============================================================================ */

/**
 * @brief 初始化/重置高度模块，含气压计起飞基准
 */
void Common_Height_Init(void)
{
    fused_height_m   = 0.0f;
    prev_height      = 0.0f;
    vertical_vel_mps = 0.0f;
    vel_from_imu     = 0.0f;

    baro_alt0       = 0.0f;
    baro_abs_current = 0.0f;
    baro_cal_cnt    = 0;
    baro_cal_sum    = 0.0f;

    initialized = 1;
}

/**
 * @brief 输入传感器原始数据，更新融合高度和垂直速度
 *
 * @param laser_mm   VL53L1X 测距值 (mm)，0 = 无效/超量程
 * @param baro_abs_m SPA06 绝对海拔 (m)，模块内自动减起飞基准
 * @param dt_s       距上次调用间隔 (秒)
 */
void Common_Height_Update(uint16_t laser_mm, float baro_abs_m, float dt_s)
{
    if (!initialized) Common_Height_Init();
    if (dt_s <= 0.0f) dt_s = 0.024f;

    baro_abs_current = baro_abs_m;

    /* ================================================================
     * 0. 气压计起飞基准校准
     *
     * 前 N 次 Update 锁定起飞基准 baro_alt0。
     * 校准完成前禁用气压计融合（相对高度取 0）。
     * ================================================================ */
    float baro_rel;
    if (baro_cal_cnt < BARO_CAL_SAMPLES)
    {
        baro_cal_sum += baro_abs_m;
        baro_cal_cnt++;
        if (baro_cal_cnt >= BARO_CAL_SAMPLES)
            baro_alt0 = baro_cal_sum / (float)BARO_CAL_SAMPLES;
        baro_rel = 0.0f;
    }
    else
    {
        baro_rel = baro_abs_m - baro_alt0;
        if (baro_rel > 200.0f || baro_rel < -200.0f)
            baro_rel = 0.0f;
    }

    /* 输入有效性判定 */
    uint8_t laser_ok = (laser_mm > 0 && laser_mm <= LASER_BLEND_MAX_MM) ? 1 : 0;
    uint8_t baro_ok  = (baro_rel > 0.01f || baro_rel < -0.01f) ? 1 : 0;
    float   target_h;

    /* ================================================================
     * 1. 高度融合
     *
     * 激光精度高但量程短 (≤2m)，气压计范围大但会漂移。
     *   激光有效 → 低空融合 → 过渡切换 → 纯气压
     *   激光无效 → 气压>5m 用气压 / 否则锁定地面
     * ================================================================ */
    if (laser_ok)
    {
        float laser_m = (float)laser_mm / 1000.0f;

        /* 激光无效→有效跳变沿检测 */
        uint8_t laser_first = (laser_was_ok == 0);
        laser_was_ok = 1;

        if (laser_first && fused_height_m < 0.5f)
        {
            /* 从地面唤醒：直接赋值激光值，不走 LPF，消除起飞跳变 */
            fused_height_m = laser_m;
        }
        else if (laser_mm <= LASER_TRUST_MAX_MM)
        {
            /* 低空 (<1.2m)：激光 70% + 气压 30%，快速 LPF */
            target_h = baro_ok
                ? laser_m * HEIGHT_FUSION_LASER_W + baro_rel * (1.0f - HEIGHT_FUSION_LASER_W)
                : laser_m;
            fused_height_m += HEIGHT_LPF_FAST * (target_h - fused_height_m);
        }
        else if (baro_ok)
        {
            /* 过渡区 (1.2~2m)：线性加权激光→气压，慢速 LPF */
            float alpha = (float)(laser_mm - LASER_TRUST_MAX_MM)
                        / (float)(LASER_BLEND_MAX_MM - LASER_TRUST_MAX_MM);
            target_h = laser_m * (1.0f - alpha) + baro_rel * alpha;
            fused_height_m += HEIGHT_LPF_SLOW * (target_h - fused_height_m);
        }
        else
        {
            /* 激光有效但无气压：纯激光兜底 */
            target_h = laser_m;
            fused_height_m += HEIGHT_LPF_FAST * (target_h - fused_height_m);
        }
    }
    else
    {
        laser_was_ok = 0;

        if (baro_ok && fabsf(baro_rel) > BARO_GROUND_MAX_M)
        {
            /* 高空 (>5m)：激光超量程，纯气压计 */
            target_h = baro_rel;
            fused_height_m += HEIGHT_LPF_SLOW * (target_h - fused_height_m);
        }
        else
        {
            /* 地面：激光无效 + 气压近零 → 锁定为 0，消除气压漂移 */
            fused_height_m = 0.0f;
        }
    }

    /* ================================================================
     * 2. 垂直速度估计 — 互补滤波
     *
     *   vel_imu  = ∫(accZ − 1g) dt          ← 高频好，会漂移
     *   vel_diff = (h_now − h_prev) / dt     ← 低频准，噪声大
     *
     *   vel_imu += K · (vel_diff − vel_imu)  ← 高度微分修正漂移
     *   output   = LPF(vel_imu)              ← 低通去噪
     * ================================================================ */
    {
        float vel_diff   = (fused_height_m - prev_height) / dt_s;
        float acc_z_mps2 = (Common_IMU_GetNormAccZ() - GRAVITY_1G) * 9.81f / GRAVITY_1G;

        vel_from_imu += acc_z_mps2 * dt_s;
        vel_from_imu += VEL_DRIFT_K * (vel_diff - vel_from_imu);

        vertical_vel_mps = Common_Filter_LowPass_Float(
            vel_from_imu, vertical_vel_mps, VEL_LPF_ALPHA);
    }

    prev_height = fused_height_m;
}

/**
 * @brief 获取融合后的高度（始终 ≥0）
 */
float Common_Height_GetFused(void)
{
    if (!initialized) Common_Height_Init();
    if (fused_height_m < 0.0f) return 0.0f;
    return fused_height_m;
}

/**
 * @brief 获取气压计相对高度
 *
 * 相对高度 = 绝对海拔 − 起飞基准。未校准时返回 0。
 */
float Common_Height_GetBaroRel(void)
{
    if (baro_cal_cnt < BARO_CAL_SAMPLES) return 0.0f;
    return baro_abs_current - baro_alt0;
}

/**
 * @brief 获取垂直速度估计 (m/s)
 */
float Common_Height_GetVelocity(void)
{
    return vertical_vel_mps;
}
