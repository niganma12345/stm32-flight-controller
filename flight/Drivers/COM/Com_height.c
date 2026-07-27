#include "Com_height.h"
#include "Com_imu.h"
#include "Com_filter.h"

/* ============================================================================
 * 融合参数
 * ============================================================================ */
#define LASER_TRUST_MAX_MM   1200    /* 纯激光信任上限 (mm) */
#define LASER_BLEND_MAX_MM   2000    /* 融合过渡上限 (mm) */
#define BARO_GROUND_MAX_M    5.0f    /* 气压计地面判定阈值(m)：气压高度低于此值视为地面，
                                       防止气压漂移导致地面高度非零，同时保证高空超量
                                       程安全（飞行中气压值一定远大于此阈值） */

#define HEIGHT_LPF_FAST      0.7f    /* 激光区 LP 滤波系数 */
#define HEIGHT_LPF_SLOW      0.4f    /* 气压计区 LP 滤波系数 */
#define HEIGHT_FUSION_LASER_W 0.7f   /* 低空融合激光权重（激光70% + 气压30%） */

/* 垂直速度互补滤波参数 */
#define VEL_ACC_WEIGHT        0.6f
#define VEL_LPF_ALPHA         0.3f

/* ============================================================================
 * 模块内部状态
 * ============================================================================ */
static float   fused_height_m    = 0.0f;
static float   vertical_vel_mps  = 0.0f;
static uint8_t initialized       = 0;

static float   prev_height       = 0.0f;
static float   vel_from_imu      = 0.0f;

/* 激光有效状态追踪（用于检测无效→有效跳变） */
static uint8_t laser_was_ok     = 0;

/* ============================================================================
 * 公共 API
 * ============================================================================ */

void Common_Height_Init(void)
{
    fused_height_m    = 0.0f;
    prev_height       = 0.0f;
    vertical_vel_mps  = 0.0f;
    vel_from_imu      = 0.0f;
    initialized       = 1;
}

void Common_Height_Update(uint16_t laser_mm, float baro_altitude_m, float dt_s)
{
    uint8_t laser_ok = (laser_mm > 0 && laser_mm <= LASER_BLEND_MAX_MM) ? 1 : 0;
    uint8_t baro_ok  = (baro_altitude_m > 0.01f || baro_altitude_m < -0.01f) ? 1 : 0;
    float   target_h;

    if (!initialized) Common_Height_Init();
    if (dt_s <= 0.0f) dt_s = 0.024f;

    /* ---- 1. 选择目标高度 ---- */
    if (laser_ok)
    {
        float laser_m = (float)laser_mm / 1000.0f;
        uint8_t laser_first = (laser_was_ok == 0);  /* 激光无效→有效跳变 */
        laser_was_ok = 1;

        if (laser_first && fused_height_m < 0.5f)
        {
            /* 从地面唤醒（激光首帧生效）：直接赋值激光值，不走LPF，消除起飞跳变 */
            fused_height_m = laser_m;
        }
        else if (laser_mm <= LASER_TRUST_MAX_MM)
        {
            /* 低空精确悬停区：激光+气压计LPF融合 */
            if (baro_ok)
                target_h = laser_m * HEIGHT_FUSION_LASER_W
                         + baro_altitude_m * (1.0f - HEIGHT_FUSION_LASER_W);
            else
                target_h = laser_m;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_FAST)
                           + target_h * HEIGHT_LPF_FAST;
        }
        else if (laser_mm <= LASER_BLEND_MAX_MM && baro_ok)
        {
            /* 过渡区: 线性加权混合 laser → baro */
            float alpha = (float)(laser_mm - LASER_TRUST_MAX_MM)
                        / (float)(LASER_BLEND_MAX_MM - LASER_TRUST_MAX_MM);
            target_h = laser_m * (1.0f - alpha) + baro_altitude_m * alpha;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_SLOW)
                           + target_h * HEIGHT_LPF_SLOW;
        }
        else
        {
            /* 激光接近量程上限或无气压计：纯激光兜底 */
            target_h = laser_m;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_FAST)
                           + target_h * HEIGHT_LPF_FAST;
        }
    }
    else
    {
        laser_was_ok = 0;

        if (baro_ok && fabsf(baro_altitude_m) > BARO_GROUND_MAX_M)
        {
            /* 高空激光超量程：纯气压计（与地面锁定互斥，保证安全） */
            target_h = baro_altitude_m;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_SLOW)
                           + target_h * HEIGHT_LPF_SLOW;
        }
        else
        {
            /* 地面：激光无效 + 气压近零 → 高度锁定为0，消除气压漂移 */
            fused_height_m = 0.0f;
        }
    }

    /* ---- 2. 垂直速度估计 ---- */
    {
        float vel_from_height = (fused_height_m - prev_height) / dt_s;
        float normAccz = Common_IMU_GetNormAccZ();
        float acc_z_mps2 = (normAccz - 16384.0f) * 9.81f / 16384.0f;
        vel_from_imu += acc_z_mps2 * dt_s;

        float vel_fused = vel_from_imu * VEL_ACC_WEIGHT
                        + vel_from_height * (1.0f - VEL_ACC_WEIGHT);
        vel_from_imu += 0.1f * (vel_from_height - vel_from_imu);
        vertical_vel_mps = Common_Filter_LowPass_Float(
            vel_fused, vertical_vel_mps, VEL_LPF_ALPHA);
    }

    prev_height = fused_height_m;
}

float Common_Height_GetFused(void)
{
    if (!initialized) Common_Height_Init();
    return fused_height_m;
}

uint16_t Common_Height_GetFusedMM(void)
{
    if (!initialized) Common_Height_Init();
    if (fused_height_m < 0.0f) return 0;
    return (uint16_t)(fused_height_m * 1000.0f);
}

float Common_Height_GetVelocity(void)
{
    return vertical_vel_mps;
}
