#ifndef __COM_FLOW_H
#define __COM_FLOW_H

#include "Com_config.h"
#include "PMW3901.h"

/*============================================================================*/
/* 光流尺度补偿常量（需实际飞行校准）                                          */
/*============================================================================*/
/*
 * PMW3901: 35×35 像素, FOV ≈ 42°
 *   单像素张角 ≈ 42° / 35 ≈ 1.2°
 *   K = tan(1.2°) ≈ 0.021
 *
 *   实际位移(mm) = 像素位移(count) × 高度(mm) × K
 *
 *   校准方法：在已知高度(如500mm)移动已知距离(如300mm)，
 *   记录像素位移均值，反算 K = 300 / (像素均值 × 500)
 */
#define FLOW_SCALE_FACTOR   0.021f   /* 单像素张角正切值（需校准） */
#define FLOW_DEG_PER_PIXEL  0.1f   /* 每像素对应张角(°) — 故意小于物理值以增大旋转补偿 */

/* 表面质量最低阈值（低于此值数据不可信） */
#define FLOW_SQUAL_MIN      90

/*============================================================================*/
/* 光流噪声抑制参数                                                            */
/*============================================================================*/
/*
 * 调参说明（FLOW_DISP_MA_WINDOW 和 FLOW_VEL_LPF_ALPHA 联合调优）：
 *   MA窗口越大 → 位移越平滑，但响应延迟越大。4帧@30ms ≈ 120ms 累积窗口。
 *   LPF alpha 越小 → 速度越平滑，但响应越慢。0.15 适合室内光流悬停。
 *   若飞行感觉"肉"（响应慢），先增大 alpha 到 0.20，再考虑减小窗口。
 *   若波形仍有毛刺，先减小 alpha 到 0.10，再考虑增大窗口。
 */
#define FLOW_DISP_MA_WINDOW    4       /* 位移滑动窗口大小（帧数） */
#define FLOW_VEL_LPF_ALPHA     0.1f   /* 速度低通滤波系数（0~1，越小越平滑） */
#define FLOW_PIXEL_DEADBAND    2       /* 像素死区（±像素），掐掉传感器量化噪声，高度无关 */
#define FLOW_POS_DEADBAND_MM   30.0f   /* 位移死区 (mm)，高度补偿后应用 */
#define FLOW_VEL_DEADBAND      30.0f   /* 速度死区 (mm/s)，LPF后应用 */

/* 光流修正量低通滤波（平滑速度PID输出，防止毛刺传入角度环） */
#define FLOW_CORR_LPF_ALPHA    0.01f   /* 修正量LPF系数（0~1，越小越平滑） */

/* 光流PID测试开关：置1=仅光流速度PID生效，角度/陀螺/偏航/定高全部禁用 */
#define FLOW_PID_TEST_ONLY     0      /* 0=正常  1=测试 */

/*============================================================================*/
/* 水平速度互补滤波参数（加速度积分 + 光流速度融合）                           */
/*============================================================================*/
#define FLOW_FUSION_ACC_BIAS_GAIN   0.0375f   /* 积分项: 0.01 × 30/8, 消除加速度零偏 */
#define FLOW_FUSION_VEL_P_GAIN      0.075f    /* 比例项: 0.02 × 30/8, 光流直接修正 */
#define FLOW_FUSION_ACC_SCALE       2.5f      /* 加速度→光流缩放因子 */
#define FLOW_GYRO_DELAY_N           2         /* 陀螺延迟深度: 2×30ms = 60ms 对齐光流管线 */

/*============================================================================*/
/* 数据结构                                                                    */
/*============================================================================*/

/* 像素位移（飞机坐标系：X=前后，Y=左右） */
typedef struct
{
    int16_t delta_x;    /* 飞机前后像素位移 (PMW delta_y 映射) */
    int16_t delta_y;    /* 飞机左右像素位移 (PMW delta_x 映射) */
} Flow_Displacement_t;

/* 真实物理位移（单位: mm） */
typedef struct
{
    float x;            /* 飞机前后实际位移 (mm) */
    float y;            /* 飞机左右实际位移 (mm) */
} Flow_RealPos_t;

/* 光流综合数据结构体 */
typedef struct
{
    Flow_Displacement_t disp;       /* 像素位移（原始计数）      */
    Flow_RealPos_t      pos;        /* 真实位移 (mm)             */
    float               vx;         /* 飞机前后速度 (mm/s)       */
    float               vy;         /* 飞机左右速度 (mm/s)       */
    uint8_t             squal;      /* 表面质量 (0~255)          */
    uint8_t             is_motion;  /* 是否有运动                 */
    uint8_t             is_valid;   /* 本次数据是否有效            */
} Flow_Data_t;

/*============================================================================*/
/* API                                                                         */
/*============================================================================*/

/**
 * @brief 坐标系映射：PMW3901 原始数据 → 飞机坐标系
 *        当前传感器安装方向：PMW X轴=飞机前后，Y轴=飞机左右，直接赋值不交换
 *        同时判断数据有效性（运动/溢出/表面质量）
 */
void Com_Flow_MapAxis(PMW3901_MotionData_t *pMotion, Flow_Data_t *pFlow);

/**
 * @brief 高度尺度补偿：像素位移 → 真实物理位移(mm)
 *        实际位移(mm) = 像素位移(count) × 高度(mm) × FLOW_SCALE_FACTOR
 * @param pFlow:     光流数据（disp 已填充像素位移）
 * @param height_mm: 当前高度 (mm)，来自 VL53L1X 等测距传感器
 */
void Com_Flow_ApplyHeightScale(Flow_Data_t *pFlow, uint16_t height_mm);

/**
 * @brief 速度计算（真实位移 → mm/s）
 * @param pFlow:  光流数据（pos 已填充真实位移）
 * @param dt_ms:  采样周期（毫秒）
 */
void Com_Flow_CalcVelocity(Flow_Data_t *pFlow, float dt_ms);

/**
 * @brief 基础处理：坐标系映射 + 速度计算（不含高度补偿）
 */
void Com_Flow_Process(PMW3901_MotionData_t *pMotion, Flow_Data_t *pFlow, float dt_ms);

/**
 * @brief 完整处理：坐标系映射 + 旋转补偿 + 高度补偿 + 速度计算
 * @param pMotion:     PMW3901 原始运动数据
 * @param pFlow:       输出的光流综合数据
 * @param height_mm:   当前高度 (mm)
 * @param dt_ms:       采样周期（毫秒）
 * @param gyro_x_dps:  X轴陀螺仪角速度 (°/s)，已换算为物理单位
 * @param gyro_y_dps:  Y轴陀螺仪角速度 (°/s)，已换算为物理单位
 */
void Com_Flow_ProcessFull(PMW3901_MotionData_t *pMotion,
                          Flow_Data_t             *pFlow,
                          uint16_t                 height_mm,
                          float                    dt_ms,
                          float                    gyro_x_dps,
                          float                    gyro_y_dps);

/**
 * @brief 陀螺仪旋转补偿 — 在像素层补偿飞机旋转造成的伪位移
 *
 * 原理：飞机旋转时，光流传感器跟随倾斜，地面图像产生视在移动。
 * 此函数在像素层补偿该效应，不依赖高度。
 *
 * 旋转伪像素 = 角速度(°/s) × dt(s) / FLOW_DEG_PER_PIXEL(°/像素)
 *
 * @param pFlow:       光流数据（disp 已填充原始像素位移，此函数修正它）
 * @param gyro_x_dps:  X轴陀螺仪角速度 (°/s) — Roll 轴
 * @param gyro_y_dps:  Y轴陀螺仪角速度 (°/s) — Pitch 轴
 * @param dt_s:        采样周期 (秒)
 *
 * @warning 补偿符号（+/-）依赖传感器安装方向，需实际飞行验证：
 *          悬停时若 vx/vy 系统性地偏向一侧，说明符号反了，
 *          将函数内的 += 改为 -= 即可。
 */
void Com_Flow_RemoveRotation(Flow_Data_t *pFlow,
                             float gyro_x_dps, float gyro_y_dps,
                             float dt_s);

/*============================================================================*/
/* 水平速度互补滤波                                                            */
/*============================================================================*/

/* 水平速度互补滤波状态（融合加速度积分 + 光流速度） */
typedef struct
{
    float   vel_x;         /* 融合后前后速度 (mm/s)，正值=前进 */
    float   vel_y;         /* 融合后左右速度 (mm/s)，正值=右移 */
    float   acc_bias_x;    /* X轴加速度零偏修正累积 (mm/s²) */
    float   acc_bias_y;    /* Y轴加速度零偏修正累积 (mm/s²) */
    uint8_t initialized;   /* 首次调用自动初始化 */
} FlowVelFusion_t;

/**
 * @brief 带陀螺延迟对齐的旋转补偿（替代 RemoveRotation）
 *        内置 FloatShifter 将陀螺仪延迟 2 个采样周期（~60ms），
 *        以对齐光流传感器的管线延迟，补偿更精确
 * @param pFlow:       光流数据（disp 已填充原始像素位移）
 * @param gyro_x_dps:  X轴陀螺仪角速度 (°/s) — Roll 轴
 * @param gyro_y_dps:  Y轴陀螺仪角速度 (°/s) — Pitch 轴
 * @param dt_s:        采样周期 (秒)
 */
void Com_Flow_RemoveRotationDelayed(Flow_Data_t *pFlow,
    float gyro_x_dps, float gyro_y_dps, float dt_s);

/**
 * @brief 水平速度互补滤波 — 融合加速度积分和光流速度
 *
 * 核心思路（与 Com_height 垂直速度融合同理）：
 *   加速度积分 → 高频动态响应好，但会漂移
 *   光流速度   → 低频绝对参考准确，但噪声大、有延迟
 *   互补滤波   → 加速度积分 + 光流误差PI修正
 *
 * 机体加速度通过欧拉角旋转矩阵转换到水平世界系，去除重力分量。
 *
 * @param f:            融合状态（调用方维护一个实例即可）
 * @param flow_vx:      光流解算的原始前后速度 (mm/s)
 * @param flow_vy:      光流解算的原始左右速度 (mm/s)
 * @param body_acc_x:   机体X轴加速度 (m/s²)
 * @param body_acc_y:   机体Y轴加速度 (m/s²)
 * @param body_acc_z:   机体Z轴加速度 (m/s²)，用于重力补偿
 * @param pitch_deg:    当前俯仰角 (°)
 * @param roll_deg:     当前横滚角 (°)
 * @param dt_s:         采样周期 (秒)
 */
void Com_Flow_VelocityFusion(FlowVelFusion_t *f,
    float flow_vx, float flow_vy,
    float body_acc_x, float body_acc_y, float body_acc_z,
    float pitch_deg, float roll_deg,
    float dt_s);

#endif /* __COM_FLOW_H */
