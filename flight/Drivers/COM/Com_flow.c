#include "Com_flow.h"
#include "Com_filter.h"
#include <math.h>

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

    /* 像素死区：在高度补偿前掐掉 ±1~2 像素的量化噪声，不受高度放大影响 */
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
    /* 陀螺零偏死区：低于阈值的角速度不计入旋转补偿，防止静止时漂移 */
    #define GYRO_DEADBAND_DPS  0.5f   /* °/s，低于此值视为零偏噪声 */
    float gx = (fabsf(gyro_x_dps) < GYRO_DEADBAND_DPS) ? 0.0f : gyro_x_dps;
    float gy = (fabsf(gyro_y_dps) < GYRO_DEADBAND_DPS) ? 0.0f : gyro_y_dps;

    int16_t rot_pix = (int16_t)(gx * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);
    int16_t rot_piy = (int16_t)(gy * dt_s / FLOW_DEG_PER_PIXEL + 0.5f);

    /* 钳位：旋转补偿最多到零，防止过补偿导致符号反转 */
    int16_t new_dx = pFlow->disp.delta_x + rot_piy;
    int16_t new_dy = pFlow->disp.delta_y + rot_pix;

    if ((pFlow->disp.delta_x > 0 && new_dx < 0) ||
        (pFlow->disp.delta_x < 0 && new_dx > 0))
        new_dx = 0;

    if ((pFlow->disp.delta_y > 0 && new_dy < 0) ||
        (pFlow->disp.delta_y < 0 && new_dy > 0))
        new_dy = 0;

    pFlow->disp.delta_x = new_dx;  /* Pitch → 前后像素补偿 */
    pFlow->disp.delta_y = new_dy;  /* Roll  → 左右像素补偿 */
}

/**
 * @description: 速度计算（位移滑动平均 → 瞬时速度 → 一阶LPF → 死区）
 *
 * 数据流（3 级递进滤波）：
 *   ① 位移滑动平均：4 帧窗口平均像素位移，消除量化尖峰
 *   ② 一阶低通滤波：alpha 抑制高频速度噪声
 *   ③ 速度死区：20mm/s 以下视为静止，消除微小幅值抖动
 *
 * PMW3901 量化噪声说明：
 *   在 500mm 高度时 1 像素 ≈ 10.5mm，单帧速度分辨率 350mm/s。
 *   位移 MA 将 4 帧的像素离散误差平均化，有效分辨率提升约 √4=2 倍。
 */
void Com_Flow_CalcVelocity(Flow_Data_t *pFlow, float dt_ms)
{
    /* ---- 位移滑动平均（消除像素量化噪声）---- */
    static float disp_ma_x_buf[FLOW_DISP_MA_WINDOW] = {0};
    static float disp_ma_y_buf[FLOW_DISP_MA_WINDOW] = {0};
    static float disp_ma_sum_x = 0.0f;
    static float disp_ma_sum_y = 0.0f;
    static uint8_t disp_ma_idx = 0;
    static uint8_t disp_ma_full = 0;

    /* ---- 速度 LPF 状态保持 ---- */
    static float vx_filtered = 0.0f;
    static float vy_filtered = 0.0f;
    static uint8_t filter_init = 0;

    if (dt_ms <= 0.0f)
    {
        pFlow->vx = 0.0f;
        pFlow->vy = 0.0f;
        return;
    }

    /* ① 更新滑动窗口：新位移进入 → 最旧退出 → 计算平均 */
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

        pFlow->pos.x = disp_ma_sum_x / n;  /* 平均位移替换原始值 */
        pFlow->pos.y = disp_ma_sum_y / n;
    }

    /* ② 瞬时速度 = 平均位移 / 时间 */
    float dt_s = dt_ms / 1000.0f;
    float raw_vx = pFlow->pos.x / dt_s;
    float raw_vy = pFlow->pos.y / dt_s;

    /* ③ 一阶低通滤波 */
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

    /* ④ 速度死区：滤波后仍低于阈值视为静止 */
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

    /* 钳位：旋转补偿最多到零，防止过补偿导致符号反转 */
    int16_t new_dx = pFlow->disp.delta_x + rot_piy;
    int16_t new_dy = pFlow->disp.delta_y + rot_pix;

    if ((pFlow->disp.delta_x > 0 && new_dx < 0) ||
        (pFlow->disp.delta_x < 0 && new_dx > 0))
        new_dx = 0;

    if ((pFlow->disp.delta_y > 0 && new_dy < 0) ||
        (pFlow->disp.delta_y < 0 && new_dy > 0))
        new_dy = 0;

    pFlow->disp.delta_x = new_dx;  /* Pitch → 前后像素补偿 */
    pFlow->disp.delta_y = new_dy;  /* Roll  → 左右像素补偿 */
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
