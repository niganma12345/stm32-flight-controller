#include "Com_flow.h"
#include "Com_filter.h"
#include <math.h>

/*============================================================================*/
/* 光流噪声抑制参数                                                            */
/*============================================================================*/
#define FLOW_VEL_LPF_ALPHA     0.4f
#define FLOW_POS_DEADBAND_MM   10.0f
#define FLOW_VEL_DEADBAND      15.0f

/**
 * @description: 坐标系映射：PMW3901 原始数据 → 飞机坐标系
 */
void Com_Flow_MapAxis(PMW3901_MotionData_t *pMotion, Flow_Data_t *pFlow)
{
    pFlow->is_valid = 0;

    if (pMotion->is_motion &&
        !pMotion->is_overflow &&
        pMotion->squal >= FLOW_SQUAL_MIN)
        pFlow->is_valid = 1;

    pFlow->is_motion = pMotion->is_motion;
    pFlow->squal     = pMotion->squal;
    pFlow->disp.delta_x = pMotion->delta_x;
    pFlow->disp.delta_y = pMotion->delta_y;
}

/**
 * @description: 高度尺度补偿 — 像素位移 → 物理位移(mm)
 */
void Com_Flow_ApplyHeightScale(Flow_Data_t *pFlow, uint16_t height_mm)
{
    if (height_mm == 0)
    {
        pFlow->pos.x = 0.0f;
        pFlow->pos.y = 0.0f;
        return;
    }

    float scale = (float)height_mm * FLOW_SCALE_FACTOR;
    float raw_x = (float)pFlow->disp.delta_x * scale;
    float raw_y = (float)pFlow->disp.delta_y * scale;

    if (fabsf(raw_x) < FLOW_POS_DEADBAND_MM) raw_x = 0.0f;
    if (fabsf(raw_y) < FLOW_POS_DEADBAND_MM) raw_y = 0.0f;

    pFlow->pos.x = raw_x;
    pFlow->pos.y = raw_y;
}

/**
 * @description: 旋转补偿 — 像素层，补偿飞机旋转造成的视在像素位移
 *               旋转角度 / 每像素张角 = 等效像素偏移，从原始位移中补偿
 *
 *   Gyro Y (Pitch, °/s) → 旋转角/每像素张角 → 补偿 delta_x (前后)
 *   Gyro X (Roll,  °/s) → 旋转角/每像素张角 → 补偿 delta_y (左右)
 *
 *   符号约定：rot_pi* 的正负号由角速度方向决定。
 *   当前使用 += 累加补偿量，等价于「测量值 + 修正项」。
 *   若实飞中补偿方向反了（悬停时速度偏向一侧），将 += 改为 -=。
 */
void Com_Flow_RemoveRotation(Flow_Data_t *pFlow,
                             float gyro_x_dps, float gyro_y_dps,
                             float dt_s)
{
    int16_t rot_pix = (int16_t)(gyro_x_dps * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);
    int16_t rot_piy = (int16_t)(gyro_y_dps * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);

    pFlow->disp.delta_x += rot_piy;  /* Pitch → 前后像素补偿 */
    pFlow->disp.delta_y += rot_pix;  /* Roll  → 左右像素补偿 */
}

/**
 * @description: 速度计算（真实位移 ÷ 时间 → mm/s），含低通滤波 + 死区
 */
void Com_Flow_CalcVelocity(Flow_Data_t *pFlow, float dt_ms)
{
    /* 滤波状态保持（静态变量） */
    static float vx_filtered = 0.0f;
    static float vy_filtered = 0.0f;
    static uint8_t filter_init = 0;

    if (dt_ms <= 0.0f)
    {
        pFlow->vx = 0.0f;
        pFlow->vy = 0.0f;
        return;
    }

    float dt_s = dt_ms / 1000.0f;
    float raw_vx = pFlow->pos.x / dt_s;
    float raw_vy = pFlow->pos.y / dt_s;

    /* 首次调用直接采用原始值，避免从 0 缓慢爬升 */
    if (!filter_init)
    {
        vx_filtered = raw_vx;
        vy_filtered = raw_vy;
        filter_init = 1;
    }
    else
    {
        /* 一阶低通滤波 */
        vx_filtered = Common_Filter_LowPass_Float(raw_vx, vx_filtered, FLOW_VEL_LPF_ALPHA);
        vy_filtered = Common_Filter_LowPass_Float(raw_vy, vy_filtered, FLOW_VEL_LPF_ALPHA);
    }

    /* ---- 速度死区：滤波后仍低于阈值视为静止 ---- */
    if (fabsf(vx_filtered) < FLOW_VEL_DEADBAND) vx_filtered = 0.0f;
    if (fabsf(vy_filtered) < FLOW_VEL_DEADBAND) vy_filtered = 0.0f;

    pFlow->vx = vx_filtered;
    pFlow->vy = vy_filtered;
}

/**
 * @description: 基础处理（不含高度补偿）
 */
void Com_Flow_Process(PMW3901_MotionData_t *pMotion, Flow_Data_t *pFlow, float dt_ms)
{
    Com_Flow_MapAxis(pMotion, pFlow);
    /* 无高度补偿时，pos 置零 */
    pFlow->pos.x = 0.0f;
    pFlow->pos.y = 0.0f;
    Com_Flow_CalcVelocity(pFlow, dt_ms);
}

/**
 * @description: 完整处理：坐标系映射 + 高度补偿 + 旋转补偿 + 速度计算
 */
void Com_Flow_ProcessFull(PMW3901_MotionData_t *pMotion,
                          Flow_Data_t             *pFlow,
                          uint16_t                 height_mm,
                          float                    dt_ms,
                          float                    gyro_x_dps,
                          float                    gyro_y_dps)
{
    Com_Flow_MapAxis(pMotion, pFlow);
    Com_Flow_RemoveRotation(pFlow, gyro_x_dps, gyro_y_dps, dt_ms / 1000.0f);
    Com_Flow_ApplyHeightScale(pFlow, height_mm);
    Com_Flow_CalcVelocity(pFlow, dt_ms);
}

/*============================================================================*/
/* 陀螺延迟旋转补偿 + 水平速度互补滤波                                         */
/*============================================================================*/

/* 互补滤波参数 — 参考项目 125Hz 调参值缩放至 30ms 调用周期 */
#define FLOW_FUSION_ACC_BIAS_GAIN   0.0375f   /* 积分项: 0.01 × 30/8, 消除加速度零偏 */
#define FLOW_FUSION_VEL_P_GAIN      0.075f    /* 比例项: 0.02 × 30/8, 光流直接修正 */
#define FLOW_FUSION_ACC_SCALE       2.5f      /* 加速度→光流缩放因子, 参考项目保留 */
#define FLOW_GYRO_DELAY_N           2         /* 陀螺延迟深度: 2×30ms = 60ms 对齐光流管线 */

/**
 * @brief 带陀螺延迟对齐的旋转补偿
 *        光流传感器有约30~50ms 管线延迟，陀螺仪数据比光流"新"。
 *        用 FloatShifter 将陀螺仪延迟 2 个采样周期(~60ms) 对齐时序。
 */
void Com_Flow_RemoveRotationDelayed(Flow_Data_t *pFlow,
    float gyro_x_dps, float gyro_y_dps, float dt_s)
{
    static float gx_delay_buf[FLOW_GYRO_DELAY_N] = {0};
    static float gy_delay_buf[FLOW_GYRO_DELAY_N] = {0};

    /* 延迟对齐后按常规方式补偿 */
    float gx_aligned = FloatShifter(gx_delay_buf, FLOW_GYRO_DELAY_N, gyro_x_dps);
    float gy_aligned = FloatShifter(gy_delay_buf, FLOW_GYRO_DELAY_N, gyro_y_dps);

    int16_t rot_pix = (int16_t)(gx_aligned * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);
    int16_t rot_piy = (int16_t)(gy_aligned * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);

    pFlow->disp.delta_x += rot_piy;  /* Pitch → 前后像素补偿 */
    pFlow->disp.delta_y += rot_pix;  /* Roll  → 左右像素补偿 */
}

/**
 * @brief 水平速度互补滤波 — 融合加速度积分与光流速度
 *
 * 加速度积分提供高频动态响应（但会漂移），光流速度提供低频绝对参考（但噪声大）。
 * PI型互补滤波：光流误差 → 修正加速度零偏(I) + 直接修正速度(P)。
 */
void Com_Flow_VelocityFusion(FlowVelFusion_t *f,
    float flow_vx, float flow_vy,
    float body_acc_x, float body_acc_y, float body_acc_z,
    float pitch_deg, float roll_deg,
    float dt_s)
{
    /* 首次调用：清零所有状态 */
    if (!f->initialized)
    {
        f->vel_x      = 0.0f;
        f->vel_y      = 0.0f;
        f->acc_bias_x = 0.0f;
        f->acc_bias_y = 0.0f;
        f->initialized = 1;
        return;
    }

    /* ---- 1. 机体加速度 → 水平世界系，含重力补偿 ---- */
    float sp = sinf(pitch_deg * 0.0174533f);
    float cp = cosf(pitch_deg * 0.0174533f);
    float sr = sinf(roll_deg  * 0.0174533f);
    float cr = cosf(roll_deg  * 0.0174533f);

    /* 参考项目公式：加速度在 g 单位下，乘 9.8*100 = 980 转为 cm/s² */
    float acc_fwd = ( body_acc_x * cp + body_acc_y * sr * sp + body_acc_z * cr * sp) * 980.0f;
    float acc_rgt = ( body_acc_y * cr - body_acc_z * sr) * 980.0f;

    /* ---- 2. X轴互补滤波 (前后) ---- */
    {
        float err = flow_vx - f->vel_x;
        f->acc_bias_x += FLOW_FUSION_ACC_BIAS_GAIN * err;
        f->vel_x += (-acc_fwd * FLOW_FUSION_ACC_SCALE + f->acc_bias_x) * dt_s;
        f->vel_x += FLOW_FUSION_VEL_P_GAIN * err;
    }

    /* ---- 3. Y轴互补滤波 (左右) ---- */
    {
        float err = flow_vy - f->vel_y;
        f->acc_bias_y += FLOW_FUSION_ACC_BIAS_GAIN * err;
        f->vel_y += (-acc_rgt * FLOW_FUSION_ACC_SCALE + f->acc_bias_y) * dt_s;
        f->vel_y += FLOW_FUSION_VEL_P_GAIN * err;
    }
}
