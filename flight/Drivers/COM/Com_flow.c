#include "Com_flow.h"
#include "Com_filter.h"
#include <math.h>

/* ============================================================================
 * 内部处理步骤（static，仅 Com_Flow_Update 调用）
 * ============================================================================ */

/**
 * @brief 坐标系映射：PMW3901 → 飞机坐标系，判断数据有效性
 */
static void MapAxis(PMW3901_MotionData_t *pMotion, Flow_Data_t *pFlow)
{
    pFlow->is_valid = 0;

    if (pMotion->is_motion && !pMotion->is_overflow &&
        pMotion->squal >= FLOW_SQUAL_MIN)
        pFlow->is_valid = 1;

    pFlow->is_motion = pMotion->is_motion;
    pFlow->squal     = pMotion->squal;
    pFlow->disp.delta_x = pMotion->delta_x;
    pFlow->disp.delta_y = pMotion->delta_y;
}

/**
 * @brief 旋转补偿：像素层补偿飞机自转造成的伪位移
 *
 * rot_pix = 角速度(°/s) × dt(s) / FLOW_DEG_PER_PIXEL
 * 若悬停时速度系统偏一侧，把 += 改为 -=
 */
static void RemoveRotation(Flow_Data_t *pFlow,
    float gyro_x_dps, float gyro_y_dps, float dt_s)
{
    #define GYRO_DEADBAND_DPS  0.5f
    float gx = (fabsf(gyro_x_dps) < GYRO_DEADBAND_DPS) ? 0.0f : gyro_x_dps;
    float gy = (fabsf(gyro_y_dps) < GYRO_DEADBAND_DPS) ? 0.0f : gyro_y_dps;

    int16_t rot_pix = (int16_t)(gx * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);
    int16_t rot_piy = (int16_t)(gy * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);

    /* 旋转补偿量不能超过原始位移 */
    int16_t max_dx = (pFlow->disp.delta_x > 0) ? pFlow->disp.delta_x : -pFlow->disp.delta_x;
    int16_t max_dy = (pFlow->disp.delta_y > 0) ? pFlow->disp.delta_y : -pFlow->disp.delta_y;
    if (rot_piy >  max_dx) rot_piy =  max_dx;
    if (rot_piy < -max_dx) rot_piy = -max_dx;
    if (rot_pix >  max_dy) rot_pix =  max_dy;
    if (rot_pix < -max_dy) rot_pix = -max_dy;

    int16_t new_dx = pFlow->disp.delta_x + rot_piy;
    int16_t new_dy = pFlow->disp.delta_y + rot_pix;

    /* 钳位：最多补偿到零，防止符号反转 */
    if ((pFlow->disp.delta_x > 0 && new_dx < 0) ||
        (pFlow->disp.delta_x < 0 && new_dx > 0))
        new_dx = 0;
    if ((pFlow->disp.delta_y > 0 && new_dy < 0) ||
        (pFlow->disp.delta_y < 0 && new_dy > 0))
        new_dy = 0;

    pFlow->disp.delta_x = new_dx;
    pFlow->disp.delta_y = new_dy;
}

/**
 * @brief 高度补偿：像素位移 × 高度 × 比例因子 → 物理位移 (mm)
 */
static void ApplyHeightScale(Flow_Data_t *pFlow, uint16_t height_mm)
{
    if (height_mm == 0)
    {
        pFlow->pos.x = 0.0f;
        pFlow->pos.y = 0.0f;
        return;
    }

    /* 像素死区 */
    int16_t dx = pFlow->disp.delta_x;
    int16_t dy = pFlow->disp.delta_y;
    if (dx > -FLOW_PIXEL_DEADBAND && dx < FLOW_PIXEL_DEADBAND) dx = 0;
    if (dy > -FLOW_PIXEL_DEADBAND && dy < FLOW_PIXEL_DEADBAND) dy = 0;

    float scale = (float)height_mm * FLOW_SCALE_FACTOR;
    float raw_x = (float)dx * scale;
    float raw_y = (float)dy * scale;

    if (fabsf(raw_x) < FLOW_POS_DEADBAND_MM) raw_x = 0.0f;
    if (fabsf(raw_y) < FLOW_POS_DEADBAND_MM) raw_y = 0.0f;

    pFlow->pos.x = raw_x;
    pFlow->pos.y = raw_y;
}

/**
 * @brief 速度计算：位移滑动平均 → 瞬时速度 → LPF → 死区
 */
static void CalcVelocity(Flow_Data_t *pFlow, float dt_ms)
{
    static float disp_ma_x_buf[FLOW_DISP_MA_WINDOW] = {0};
    static float disp_ma_y_buf[FLOW_DISP_MA_WINDOW] = {0};
    static float disp_ma_sum_x = 0.0f;
    static float disp_ma_sum_y = 0.0f;
    static uint8_t disp_ma_idx = 0;
    static uint8_t disp_ma_full = 0;

    static float vx_filtered = 0.0f;
    static float vy_filtered = 0.0f;
    static uint8_t filter_init = 0;

    if (dt_ms <= 0.0f)
    {
        pFlow->vx = 0.0f;
        pFlow->vy = 0.0f;
        return;
    }

    /* 滑动平均 */
    {
        float new_x = pFlow->pos.x;
        float new_y = pFlow->pos.y;

        if (disp_ma_full)
        {
            disp_ma_sum_x -= disp_ma_x_buf[disp_ma_idx];
            disp_ma_sum_y -= disp_ma_y_buf[disp_ma_idx];
        }

        disp_ma_x_buf[disp_ma_idx] = new_x;
        disp_ma_y_buf[disp_ma_idx] = new_y;
        disp_ma_sum_x += new_x;
        disp_ma_sum_y += new_y;
        disp_ma_idx++;

        float n;
        if (disp_ma_idx >= FLOW_DISP_MA_WINDOW)
        {
            disp_ma_idx  = 0;
            disp_ma_full = 1;
            n = (float)FLOW_DISP_MA_WINDOW;
        }
        else
        {
            n = (float)disp_ma_idx;
        }

        pFlow->pos.x = disp_ma_sum_x / n;
        pFlow->pos.y = disp_ma_sum_y / n;
    }

    /* 瞬时速度 → LPF → 死区 */
    float dt_s = dt_ms / 1000.0f;
    float raw_vx = pFlow->pos.x / dt_s;
    float raw_vy = pFlow->pos.y / dt_s;

    if (!filter_init)
    {
        vx_filtered = raw_vx;
        vy_filtered = raw_vy;
        filter_init = 1;
    }
    else
    {
        vx_filtered = Common_Filter_LowPass_Float(raw_vx, vx_filtered, FLOW_VEL_LPF_ALPHA);
        vy_filtered = Common_Filter_LowPass_Float(raw_vy, vy_filtered, FLOW_VEL_LPF_ALPHA);
    }

    if (fabsf(vx_filtered) < FLOW_VEL_DEADBAND) vx_filtered = 0.0f;
    if (fabsf(vy_filtered) < FLOW_VEL_DEADBAND) vy_filtered = 0.0f;

    pFlow->vx = vx_filtered;
    pFlow->vy = vy_filtered;
}

/* ============================================================================
 * 公共 API
 * ============================================================================ */

/**
 * @brief 光流数据完整处理（一步完成）
 *
 * 坐标映射 → 旋转补偿 → 高度补偿 → 速度计算，dt 固定 30ms。
 */
void Com_Flow_Update(Flow_Data_t *pFlow,
    float gyro_x_dps, float gyro_y_dps, uint16_t height_mm)
{
    PMW3901_MotionData_t motion;
    PMW3901_ReadMotion(&motion);

    MapAxis(&motion, pFlow);
    RemoveRotation(pFlow, gyro_x_dps, gyro_y_dps, 0.030f);
    ApplyHeightScale(pFlow, height_mm);
    CalcVelocity(pFlow, 30.0f);
}
