#include "Com_height.h"
#include "Com_imu.h"
#include "Com_filter.h"

/* ============================================================================
 * 融合参数
 * ============================================================================ */
#define LASER_TRUST_MAX_MM   1200    /* 纯激光信任上限 (mm) */
#define LASER_BLEND_MAX_MM   2000    /* 融合过渡上限 (mm) */

#define HEIGHT_LPF_FAST      0.7f    /* 激光区 LP 滤波系数 */
#define HEIGHT_LPF_SLOW      0.4f    /* 气压计区 LP 滤波系数 */

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

        if (laser_mm <= LASER_TRUST_MAX_MM)
        {
            /* 低空: 纯激光 + 快速LPF */
            target_h = laser_m;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_FAST)
                           + target_h * HEIGHT_LPF_FAST;
        }
        else if (baro_ok)
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
            /* 无气压计: 过渡区也只用激光 */
            target_h = laser_m;
            fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_FAST)
                           + target_h * HEIGHT_LPF_FAST;
        }
    }
    else if (baro_ok)
    {
        /* 激光无效: 纯气压计 + 慢LPF */
        target_h = baro_altitude_m;
        fused_height_m = fused_height_m * (1.0f - HEIGHT_LPF_SLOW)
                       + target_h * HEIGHT_LPF_SLOW;
    }
    /* else: 两者都无效 → 保持上一次值 */

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
