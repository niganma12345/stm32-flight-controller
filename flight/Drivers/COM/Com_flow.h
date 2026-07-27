#ifndef __COM_FLOW_H
#define __COM_FLOW_H

#include "Com_config.h"
#include "PMW3901.h"

/* ============================================================================
 * Com_flow — PMW3901 光流数据处理
 *
 * 一调用完成：坐标系映射 → 旋转补偿 → 高度补偿 → 速度计算
 * 输出写入 Flow_Data_t 的 vx/vy (mm/s)、squal、is_valid 等字段。
 * ============================================================================ */

/* 像素张角（需实际飞行校准） */
#define FLOW_SCALE_FACTOR   0.021f

/* 旋转补偿：每像素对应张角 (°) */
#define FLOW_DEG_PER_PIXEL  0.1f

/* 表面质量最低阈值 */
#define FLOW_SQUAL_MIN      90

/* 位移滑动平均窗口 */
#define FLOW_DISP_MA_WINDOW    4

/* 速度滤波与死区 */
#define FLOW_VEL_LPF_ALPHA     0.1f
#define FLOW_PIXEL_DEADBAND    2
#define FLOW_POS_DEADBAND_MM   30.0f
#define FLOW_VEL_DEADBAND      30.0f

/* 光流修正量低通 */
#define FLOW_CORR_LPF_ALPHA    0.01f

/* ============================================================================
 * 数据结构
 * ============================================================================ */

typedef struct
{
    int16_t delta_x;    /* 飞机前后像素位移 */
    int16_t delta_y;    /* 飞机左右像素位移 */
} Flow_Displacement_t;

typedef struct
{
    float x;            /* 飞机前后物理位移 (mm) */
    float y;            /* 飞机左右物理位移 (mm) */
} Flow_RealPos_t;

typedef struct
{
    Flow_Displacement_t disp;
    Flow_RealPos_t      pos;
    float               vx;         /* 前后速度 (mm/s) */
    float               vy;         /* 左右速度 (mm/s) */
    uint8_t             squal;
    uint8_t             is_motion;
    uint8_t             is_valid;
} Flow_Data_t;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief 光流数据完整处理（一步完成）
 *
 * 内部读取 PMW3901 → 坐标映射 → 旋转补偿 → 高度补偿 → 速度计算
 *
 * @param pFlow      输出结构体
 * @param gyro_x_dps Roll 角速度 (°/s)
 * @param gyro_y_dps Pitch 角速度 (°/s)
 * @param height_mm  当前高度 (mm)
 */
void Com_Flow_Update(Flow_Data_t *pFlow,
    float gyro_x_dps, float gyro_y_dps, uint16_t height_mm);

#endif
