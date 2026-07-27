#include "App_flight.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Com_flow.h"
#include "Com_height.h"
#include "vl53l1.h"
#include "Int_VL53L1X.h"
#include "QMC5883P.h"
#include "QMC_MagCal.h"
#include "App_oled.h"

Gyro_Accel_Struct gyro_accel_data = {0};

uint8_t g_spa06_ok = 0;        /* SPA06 气压计是否可用 */
float   g_baro_alt0 = 0.0f;     /* 起飞点绝对海拔(m), 计算相对高度用 */

/* ---- 光流数据（flight_task 写入，nrf24l01_task 读取用于蓝牙输出）---- */
Flow_Data_t g_flow_data = {0};
uint16_t   g_flow_height_mm = 0;
Euler_struct euler_angle = {0};
Gyro_struct last_gyro = {0};
float gyro_z_sum = 0;

/* ---- QMC5883P 磁力计校准 ---- */
QMC_MagCal_t g_mag_cal;
uint8_t      g_mag_calibrated = 0;   /* 0=校准中, 1=已完成 */
int16_t      g_mag_ofs_x     = 0;    /* 硬铁校准 X 偏移 */
int16_t      g_mag_ofs_y     = 0;    /* 硬铁校准 Y 偏移 */

extern volatile Remote_Data remote_data;
extern volatile Flight_State flight_state;
extern TaskHandle_t nrf24l01_task_handle;
// 电机结构体
Motor left_top_motor = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor left_bottom_motor = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor right_top_motor = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor right_bottom_motor = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

// PID的调参先调内环再调外环
PID_Struct pitch_pid = {
    .kp = 7.00f,
    .ki = 0.00f,
    .kd = 0.00f,
    .integral_max = 10.0f,   /* 积分限幅 ±10° */
    .output_max   = 30.0f,   /* 输出限幅 ±30（°/s） */
};
// Y轴角速度结构体 => 对应俯仰角内环
PID_Struct gyro_y_pid = {
    .kp = -3.00f,
    .ki = 0.00f,
    .kd = -0.50f,
    .integral_max = 50.0f,   /* 积分限幅 ±50（电机单位） */
    .output_max   = 200.0f,  /* 输出限幅 ±200（电机单位） */
};

// 横滚PID结构体
PID_Struct roll_pid = {
    .kp = 4.50f,        /* 低于俯仰的7.0，避免横滚轴震荡 */
    .ki = 0.03f,        /* 消除重心偏移造成的稳态角度偏移 */
    .kd = 0.00f,
    .integral_max = 10.0f,   /* 积分限幅 ±10° */
    .output_max   = 30.0f,   /* 输出限幅 ±30（°/s） */
};
// X轴角速度结构体 => 对应横滚角内环
PID_Struct gyro_x_pid = {
    .kp = -2.50f,       /* 低于俯仰的-3.0，配合外环降低的增益 */
    .ki = 0.00f,
    .kd = -0.50f,
    .integral_max = 50.0f,   /* 积分限幅 ±50（电机单位） */
    .output_max   = 200.0f,  /* 输出限幅 ±200（电机单位） */
};


// 偏航PID结构体
PID_Struct yaw_pid = {
    .kp = -4.00f,       /* 负值: 磁力计heading与CCW方向反号, 翻转保证负反馈 */
    .ki = 0.01f,
    .kd = 0.05f,
    .integral_max = 15.0f,
    .output_max   = 30.0f,
};
// Z轴角速度结构体 → gyro_z_pid.output: 正值 → CCW逆时针（左前+右后加速）
PID_Struct gyro_z_pid = {
    .kp = 4.0f,        /* 正值已验证: +gyro_z → 负反馈抑制 */
    .ki = 0.0f,
    .kd = 0.30f,
    .integral_max = 50.0f,
    .output_max   = 100.0f,  /* 与混控中 Com_limit(100,-100) 一致，防止积分windup */
};

/*============================================================================*/
/* 定高串级 PID（位置外环 → 速度内环 → 油门补偿）                              */
/*============================================================================*/
/*
 * 控制逻辑（串级）：
 *   目标高度 ──→ [高度位置PID] ──→ 速度目标(m/s) ──→ [垂直速度PID] ──→ 油门
 *                  ↑ 融合高度                          ↑ 垂直速度(微分高度+LPF)
 *
 * 相比单级位置 PID 的改进：
 *   1. 速度内环提供阻尼，抑制上下震荡
 *   2. 飞机下落时速度内环立刻响应推油门，不需要等位置积分累积
 *   3. 悬停油门基准 = 进入定高瞬间的油门值，PID 只修正偏差部分
 */

/* 高度位置 PID（外环）→ 输出垂直速度目标 (m/s) */
PID_Struct height_pos_pid = {
    .kp = 10.5f,        /* 高度误差1m → 目标速度1.5m/s */
    .ki = 0.05f,       /* 消除稳态高度偏移 */
    .kd = 0.00f,
    .integral_max = 3.0f,   /* 积分限幅 ±3m/s */
    .output_max   = 2.0f,   /* 最大目标速度 ±2m/s */
};

/* 垂直速度 PID（内环）→ 输出油门补偿量 */
PID_Struct height_vel_pid = {
    .kp = 35.0f,       /* 速度误差1m/s → 油门补偿35 */
    .ki = 5.0f,        /* 消除稳态速度误差 */
    .kd = 3.0f,        /* 阻尼加速度突变 */
    .integral_max = 50.0f,   /* 积分限幅 ±50 油门 */
    .output_max   = 100.0f,  /* 输出限幅 ±100 油门 */
};

/* ---- 悬停油门基准：进入定高/定点时自动捕获 ---- */
static float g_hover_thr = 0.0f;

/* ---- 垂直速度估计 (m/s)，由 Com_height 模块统一管理 ---- */

/*============================================================================*/
/* 光流速度 PID —— 串级在角度环外层，实现水平悬停锁定                          */
/*============================================================================*/
/*
 * 控制逻辑：
 *   vel_x_pid: 飞机前后速度 → 叠加到 pitch_pid.desire（前后=俯仰）
 *   vel_y_pid: 飞机左右速度 → 叠加到 roll_pid.desire（左右=横滚）
 *
 * 符号推导（已验证）：
 *   flow.vx > 0（前移）→ err < 0 → output < 0 → 负俯仰 → 后倾 → 回拉 ✓
 *   flow.vy > 0（右移）→ err < 0 → output < 0 → 负横滚 → 左倾 → 回拉 ✓
 *
 * 调参建议：
 *   先飞起来不漂移，再逐步加大 kp 直到出现低频振荡，然后回调 30%
 *   如果悬停有稳态偏移（朝一个方向慢慢飘），加少量 ki
 *   kd 用于抑制速度突变，一般不需要太大
 *
 * output_max: 速度环输出的最大倾角修正量（°），防止异常时翻车
 */
PID_Struct vel_x_pid = {
    .kp = 6.0f,         /* °/(mm/s): 50mm/s → 100° */
    .ki = 0.1f,         /* 快速消除稳态漂移 */
    .kd = 0.0f,
    .integral_max = 5.0f,
    .output_max   = 15.0f,  /* 最大修正 ±15° */
};

PID_Struct vel_y_pid = {
    .kp = 6.0f,
    .ki = 0.1f,
    .kd = 0.0f,
    .integral_max = 5.0f,
    .output_max   = 15.0f,
};


//// 定高飞行的目标高度（单位: m）
extern volatile float fix_height;

// 上一次飞行状态（用于检测状态进入时刻）
static Flight_State prev_flight_state = LOCKED;

/**
 * @brief 飞控任务初始化 MPU6050初始化    启动电机
 *
 */
void App_flight_init(void)
{
    Int_MPU6050_Init();

    // 电机初始化
     Motor_Init(&left_top_motor);
     Motor_Init(&left_bottom_motor);
     Motor_Init(&right_top_motor);
     Motor_Init(&right_bottom_motor);

    // 初始化 SPA06-003 气压计
    {
        uint8_t code = SPA06_Init(&hi2c2);
        g_spa06_ok = (code == 0) ? 1 : 0;
        App_OLED_Postf(120, 0, OLED_6X8, "%d", code);
    }

    /* ---- PMW3901 光流传感器初始化 ---- */
    PMW3901_Init();

    /* ---- VL53L1X 激光测距初始化 ---- */
    Int_VL53L1X_Init();

    /* ---- QMC5883P 磁力计初始化 ---- */
    QMC5883P_Init(&hi2c2);
    QMC_MagCal_Init(&g_mag_cal);

    /* ---- 高度计算层初始化 ---- */
    Common_Height_Init();
}

/**
 * @brief 根据陀螺仪测量的数据 计算出欧拉角
 *
 */
void App_flight_get_euler_angle(void)
{
    // 1. 使用MPU6050的硬件接口 得到陀螺仪数据
    Int_MPU6050_Get_Data(&gyro_accel_data);

    // 2. 对角速度进行低通滤波  =>  因为原始采集数据的实时性比较高
    // 准确性要求没有那么高 但是一定要反应迅速
    // output = 加权系数 * last_output + ( 1 - 加权系数 )* 本次的采样值;
    gyro_accel_data.gyro.gyro_x = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x, last_gyro.gyro_x);
    gyro_accel_data.gyro.gyro_y = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y, last_gyro.gyro_y);
    gyro_accel_data.gyro.gyro_z = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z, last_gyro.gyro_z);
    last_gyro.gyro_x = gyro_accel_data.gyro.gyro_x;
    last_gyro.gyro_y = gyro_accel_data.gyro.gyro_y;
    last_gyro.gyro_z = gyro_accel_data.gyro.gyro_z;

    // 先打印角速度
//    debug_printf(":%hd,%hd,%hd\n", gyro_accel_data.gyro.gyro_x, gyro_accel_data.gyro.gyro_y, gyro_accel_data.gyro.gyro_z);

    // 3. 对测量变化比较大的加速度 使用更高级的滤波方式 => 卡尔曼滤波
    gyro_accel_data.accel.accel_x = Common_Filter_KalmanFilter(&kfs[0], gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = Common_Filter_KalmanFilter(&kfs[1], gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = Common_Filter_KalmanFilter(&kfs[2], gyro_accel_data.accel.accel_z);

    // 打印加速度
    // debug_printf(":%d,%d,%d\n", gyro_accel_data.accel.accel_x, gyro_accel_data.accel.accel_y, gyro_accel_data.accel.accel_z);

    // 4. 四元数姿态解算 + 磁力计倾斜补偿融合
    Common_IMU_GetEulerAngle(&gyro_accel_data, &euler_angle, 0.006,
        g_qmc_ok,
        qmc.mx, qmc.my, qmc.mz);

    // 俯仰角  横滚角  偏航角
//     debug_printf(":%.2f,%.2f,%.2f\n", euler_angle.pitch, euler_angle.roll, euler_angle.yaw);
}

/**
 * @brief 根据欧拉角 计算出PID的目标值
 *        光流有效时：速度PID叠加到角度外环目标值上，实现水平悬停锁定
 */
void App_flight_pid_process(void)
{
    /* ---- 光流速度 PID（角度环外层）---- */
    float flow_correction_pitch = 0.0f;  /* 叠加到俯仰角目标 */
    float flow_correction_roll  = 0.0f;  /* 叠加到横滚角目标 */

    /* ---- 光流修正量低通滤波（平滑速度PID的阶跃输出）---- */
    static float flow_corr_pitch_f = 0.0f;
    static float flow_corr_roll_f  = 0.0f;

    if (g_flow_data.is_valid)
    {
        if (flight_state == MANUAL)
        {
            vel_x_pid.integral = 0.0f;
            vel_x_pid.last_err = 0.0f;
            vel_x_pid.output   = 0.0f;
            vel_y_pid.integral = 0.0f;
            vel_y_pid.last_err = 0.0f;
            vel_y_pid.output   = 0.0f;
            flow_corr_pitch_f = Common_Filter_LowPass_Float(0.0f, flow_corr_pitch_f, FLOW_CORR_LPF_ALPHA);
            flow_corr_roll_f  = Common_Filter_LowPass_Float(0.0f, flow_corr_roll_f,  FLOW_CORR_LPF_ALPHA);
            flow_correction_pitch = flow_corr_pitch_f;
            flow_correction_roll  = flow_corr_roll_f;
        }
        else
        {
            /* 光流速度 PID：目标速度=0 → 抑制漂移 */
            vel_x_pid.desire  = 0.0f;
            vel_x_pid.measure = g_flow_data.vx;
            Com_PID_Calc(&vel_x_pid);
            flow_correction_pitch = vel_x_pid.output;

            vel_y_pid.desire  = 0.0f;
            vel_y_pid.measure = g_flow_data.vy;
            Com_PID_Calc(&vel_y_pid);
            flow_correction_roll = vel_y_pid.output;

            /* 低通平滑 */
            flow_corr_pitch_f = Common_Filter_LowPass_Float(flow_correction_pitch, flow_corr_pitch_f, FLOW_CORR_LPF_ALPHA);
            flow_corr_roll_f  = Common_Filter_LowPass_Float(flow_correction_roll,  flow_corr_roll_f,  FLOW_CORR_LPF_ALPHA);
            flow_correction_pitch = flow_corr_pitch_f;
            flow_correction_roll  = flow_corr_roll_f;
        }
    }
    else
    {
        vel_x_pid.integral = 0.0f;
        vel_x_pid.last_err = 0.0f;
        vel_x_pid.output   = 0.0f;
        vel_y_pid.integral = 0.0f;
        vel_y_pid.last_err = 0.0f;
        vel_y_pid.output   = 0.0f;
        flow_corr_pitch_f = Common_Filter_LowPass_Float(0.0f, flow_corr_pitch_f, FLOW_CORR_LPF_ALPHA);
        flow_corr_roll_f  = Common_Filter_LowPass_Float(0.0f, flow_corr_roll_f,  FLOW_CORR_LPF_ALPHA);
        flow_correction_pitch = flow_corr_pitch_f;
        flow_correction_roll  = flow_corr_roll_f;
    }

#if FLOW_PID_TEST_ONLY
    /* 测试模式：光流原始速度直接映射电机
     * squal=0或255→传感器异常(未连接/浮空)，强制清零 */
    if (g_flow_data.squal == 0 || g_flow_data.squal == 255) {
        gyro_x_pid.output = 0.0f;
        gyro_y_pid.output = 0.0f;
    } else {
        gyro_x_pid.output = -g_flow_data.vy * 1.5f;
        gyro_y_pid.output = -g_flow_data.vx * 1.5f;
    }
    gyro_z_pid.output = 0.0f;

    /* 清零所有 PID 状态 */
    pitch_pid.integral = 0.0f; pitch_pid.last_err = 0.0f; pitch_pid.output = 0.0f;
    roll_pid.integral  = 0.0f; roll_pid.last_err  = 0.0f; roll_pid.output  = 0.0f;
    gyro_y_pid.integral = 0.0f; gyro_y_pid.last_err = 0.0f;
    gyro_x_pid.integral = 0.0f; gyro_x_pid.last_err = 0.0f;
    gyro_z_pid.integral = 0.0f; gyro_z_pid.last_err = 0.0f;
    height_pos_pid.integral = 0.0f; height_pos_pid.last_err = 0.0f; height_pos_pid.output = 0.0f;
    height_vel_pid.integral = 0.0f; height_vel_pid.last_err = 0.0f; height_vel_pid.output = 0.0f;
#else
    /* ==================================================
     *  俯仰角
     * ================================================== */
    pitch_pid.desire = (remote_data.pit - 500) / 20.0f + flow_correction_pitch;
    if (pitch_pid.desire >  MAX_TILT_ANGLE) pitch_pid.desire =  MAX_TILT_ANGLE;
    if (pitch_pid.desire < -MAX_TILT_ANGLE) pitch_pid.desire = -MAX_TILT_ANGLE;
    pitch_pid.measure = euler_angle.pitch;
    gyro_y_pid.measure = (gyro_accel_data.gyro.gyro_y * 2000.0f / 32768.0f);
    Com_PID_Calc_Chain(&pitch_pid, &gyro_y_pid);

    // ==================================================
    //  横滚角
    // ==================================================
    roll_pid.desire = (remote_data.rol - 500) / 20.0f + flow_correction_roll;
    if (roll_pid.desire >  MAX_TILT_ANGLE) roll_pid.desire =  MAX_TILT_ANGLE;
    if (roll_pid.desire < -MAX_TILT_ANGLE) roll_pid.desire = -MAX_TILT_ANGLE;
    roll_pid.measure = euler_angle.roll;
    gyro_x_pid.measure = (gyro_accel_data.gyro.gyro_x * 2000.0f / 32768.0f);
    Com_PID_Calc_Chain(&roll_pid, &gyro_x_pid);

    // ==================================================
    //  偏航角
    // ==================================================
    if (g_mag_calibrated)
    {
        float yaw_stick = (remote_data.yaw - 500) / 50.0f;
        static float yaw_target = 0.0f;
        static Flight_State prev_yaw_state = LOCKED;

        /* 解锁瞬间 (LOCKED→活跃) 捕获当前航向 */
        {
            Flight_State cur = (Flight_State)flight_state;
            if (prev_yaw_state == LOCKED && cur != LOCKED)
            {
                yaw_target = qmc.heading;
            }
            prev_yaw_state = cur;
        }

        if (fabsf(yaw_stick) >= 1.0f)
        {
            yaw_target += yaw_stick * 0.006f;
            if      (yaw_target >= 360.0f) yaw_target -= 360.0f;
            else if (yaw_target <  0.0f)   yaw_target += 360.0f;
        }

        yaw_pid.measure = qmc.heading;
        yaw_pid.desire  = yaw_target;

        /* 误差归一化 ±180° */
        float err = yaw_pid.desire - yaw_pid.measure;
        if      (err >  180.0f) err -= 360.0f;
        else if (err < -180.0f) err += 360.0f;
        yaw_pid.desire = yaw_pid.measure + err;

        gyro_z_pid.measure = (gyro_accel_data.gyro.gyro_z * 2000.0f / 32768.0f);
        Com_PID_Calc_Chain(&yaw_pid, &gyro_z_pid);
    }
    else
    {
        gyro_z_pid.output = 0.0f;
    }
#endif
}

/**
 * @brief 根据PID的输出值 控制电机
 *
 */
void App_flight_control_motor(void)
{
#if FLOW_PID_TEST_ONLY
    /* 测试模式：光流速度 → 绝对值差速（基0，移动方向侧电机转） */
    int16_t py = (int16_t)fabsf(gyro_y_pid.output);
    int16_t rx = (int16_t)fabsf(gyro_x_pid.output);

    /* pitch: 前移→gyro_y<0→前侧加速 */
    if (gyro_y_pid.output < 0) {
        left_top_motor.speed     = py;
        right_top_motor.speed    = py;
        left_bottom_motor.speed  = 0;
        right_bottom_motor.speed = 0;
    } else {
        left_top_motor.speed     = 0;
        right_top_motor.speed    = 0;
        left_bottom_motor.speed  = py;
        right_bottom_motor.speed = py;
    }

    /* roll: 右移→gyro_x<0→左侧加速 */
    if (gyro_x_pid.output < 0) {
        left_top_motor.speed     += rx;
        left_bottom_motor.speed  += rx;
    } else {
        right_top_motor.speed    += rx;
        right_bottom_motor.speed += rx;
    }

    left_top_motor.speed     = Com_limit(left_top_motor.speed,     700, 0);
    left_bottom_motor.speed  = Com_limit(left_bottom_motor.speed,  700, 0);
    right_top_motor.speed    = Com_limit(right_top_motor.speed,    700, 0);
    right_bottom_motor.speed = Com_limit(right_bottom_motor.speed, 700, 0);

    Motor_SetSpeed(&left_top_motor);
    Motor_SetSpeed(&left_bottom_motor);
    Motor_SetSpeed(&right_top_motor);
    Motor_SetSpeed(&right_bottom_motor);
    return;
#else
    // 1. 首先判断当前飞机的飞行状态
    switch (flight_state)
    {
    case LOCKED:
    case IDLE:
        // 一直处于空闲/锁定状态 =>  需要将电机速度置为0
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
        break;
    case NORMAL:
    case MANUAL:
        // 自稳模式 — 对角线电机配对：左前+右后(gyro_z同号) vs 左后+右前(-gyro_z)
        left_top_motor.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_top_motor.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        break;
    case FIX_HEIGHT:
        // 定高模式 → 悬停油门基准 + 自稳 + 串级高度PID修正
        {
            int16_t base_thr = (int16_t)g_hover_thr;
            int16_t h_corr   = (int16_t)height_vel_pid.output;
            left_top_motor.speed     = base_thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + h_corr;
            left_bottom_motor.speed  = base_thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + h_corr;
            right_top_motor.speed    = base_thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + h_corr;
            right_bottom_motor.speed = base_thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + h_corr;
        }
        break;
    case FAIL:
        // 出现故障处理 => 一直减速  直到电机停转 修改状态为LOCKED
        // 6ms => 减速速度2点
        left_top_motor.speed -= 2;
        left_bottom_motor.speed -= 2;
        right_top_motor.speed -= 2;
        right_bottom_motor.speed -= 2;
        if (left_top_motor.speed <= 0 && left_bottom_motor.speed <= 0 && right_top_motor.speed <= 0 && right_bottom_motor.speed <= 0)
        {
            // 故障处理完毕，所有转速已置为0，转入锁定等待重新解锁
            flight_state = LOCKED;
            // debug_printf("FAIL MOTOR STOP -> LOCKED\r\n");
        }

        break;
    default:
        break;
    }

    // 限制电机速度的输出值
    // 通过限制提供速度阈值 让飞行更加平稳
    left_top_motor.speed = Com_limit(left_top_motor.speed, 700, 0);
    left_bottom_motor.speed = Com_limit(left_bottom_motor.speed, 700, 0);
    right_top_motor.speed = Com_limit(right_top_motor.speed, 700, 0);
    right_bottom_motor.speed = Com_limit(right_bottom_motor.speed, 700, 0);

    // 安全保护 => 当油门值为<100时 => 强制将速度置为0
    if (remote_data.thr < 100)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
    }

    // 2. 设置电机速度
    Motor_SetSpeed(&left_top_motor);
    Motor_SetSpeed(&left_bottom_motor);
    Motor_SetSpeed(&right_top_motor);
    Motor_SetSpeed(&right_bottom_motor);
#endif
}


/**
 * @brief 定高串级 PID 计算（位置外环 + 速度内环 + 悬停油门前馈）
 *
 * 每 24ms 调用一次（flight_task 中 height_tick 计数器控制）
 * 进入 FIX_HEIGHT/FIX_POSITION 时锁定目标高度 + 捕获悬停油门基准
 *
 * 控制链路：目标高度 → height_pos_pid → 速度目标 → height_vel_pid → 油门补偿
 *                       ↑ 融合高度                         ↑ 垂直速度(微分高度+LPF)
 */
void App_flight_fix_height_pid_process(void)
{
    /* ---- 1. 状态切换检测：刚进入定高时记录目标 + 捕获悬停油门 ---- */
    if (prev_flight_state != FIX_HEIGHT && flight_state == FIX_HEIGHT)
    {
        fix_height   = Common_Height_GetFused();
        g_hover_thr  = (float)remote_data.thr;   /* 捕获当前油门作为悬停基准 */
    }
    /* 更新上一次状态 */
    prev_flight_state = flight_state;

    /* ---- 2. 串级 PID 计算（仅定高模式）---- */
    if (flight_state == FIX_HEIGHT)
    {
        float fused = Common_Height_GetFused();   /* 融合高度 */

        /* 外环：高度位置 PID → 输出垂直速度目标 (m/s) */
        height_pos_pid.desire  = fix_height;
        height_pos_pid.measure = fused;
        Com_PID_Calc(&height_pos_pid);

        /* 内环：垂直速度 PID → 输出油门补偿（正值=加油门，负值=减油门） */
        height_vel_pid.desire  = height_pos_pid.output;  /* 位置环输出 = 速度目标 */
        height_vel_pid.measure = Common_Height_GetVelocity();
        Com_PID_Calc(&height_vel_pid);
    }
    else
    {
        /* 非定高状态：清零所有高度 PID 状态 */
        height_pos_pid.output   = 0.0f;
        height_pos_pid.integral = 0.0f;
        height_pos_pid.last_err = 0.0f;
        height_vel_pid.output   = 0.0f;
        height_vel_pid.integral = 0.0f;
        height_vel_pid.last_err = 0.0f;
        g_hover_thr  = 0.0f;
    }
}

/**
 * @brief  水平零偏校准 — 平放飞机后调用
 * @note   APP 层桥接 BSP(mpu6050) 和 COM(零偏变量)，符合分层规范
 */
void App_flight_calibrate_level(void)
{
    Accel_struct cur = {0}, last = {0};
    uint8_t stable = 0;

    Int_MPU6050_Get_Acc(&last);
    while (stable < 30)
    {
        Int_MPU6050_Get_Acc(&cur);
        if (abs(cur.accel_x - last.accel_x) < 200 &&
            abs(cur.accel_y - last.accel_y) < 200)
            stable++;
        else
            stable = 0;
        last = cur;
        vTaskDelay(10);
    }

    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (uint8_t i = 0; i < 100; i++)
    {
        Int_MPU6050_Get_Acc(&cur);
        ax_sum += cur.accel_x;
        ay_sum += cur.accel_y;
        az_sum += cur.accel_z;
        vTaskDelay(6);
    }

    float ax = (float)ax_sum / 100.0f;
    float ay = (float)ay_sum / 100.0f;
    float az = (float)az_sum / 100.0f;

    g_pitch_zero = atan2f(ax, az) * 57.29578f;
    g_roll_zero  = atan2f(ay, az) * 57.29578f;
}

/*============================================================================*/
/* 传感器数据采集（从 freertos_demo.c 提取，按 6ms 周期调用）                  */
/*============================================================================*/

/**
 * @brief 磁力计处理 — 读取 + 航向计算 + 硬铁校准
 *        每 6ms 调用一次
 */
void App_flight_process_mag(void)
{
    QMC5883P_ReadData(&hi2c2);

    /* 航向计算（校准后用偏移修正） */
    {
        float mx_f = (float)qmc.mx - (g_mag_calibrated ? (float)g_mag_ofs_x : 0.0f);
        float my_f = (float)qmc.my - (g_mag_calibrated ? (float)g_mag_ofs_y : 0.0f);
        float hd   = atan2f(my_f, mx_f) * 57.29578f;
        if (hd < 0.0f) hd += 360.0f;
        qmc.heading = hd;
    }

    /* 硬铁校准（上电 5 秒内旋转 360°） */
    {
        static TickType_t cal_start = 0;
        if (cal_start == 0) cal_start = xTaskGetTickCount();

        QMC_MagCal_Feed(&g_mag_cal, qmc.mx, qmc.my);

        if (!g_mag_calibrated)
        {
            TickType_t now  = xTaskGetTickCount();
            int32_t    left = 5000 - (int32_t)((now - cal_start) * portTICK_PERIOD_MS);
            if (left <= 0)
            {
                QMC_MagCal_Lock(&g_mag_cal);
                g_mag_ofs_x     = g_mag_cal.mx_offset;
                g_mag_ofs_y     = g_mag_cal.my_offset;
                g_mag_calibrated = 1;
            }
            else
            {
                /* 每 ~100ms 刷新校准界面 */
                static uint8_t cal_tick = 0;
                if (++cal_tick >= 17)
                {
                    cal_tick = 0;
                    App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
                        "%ds X:%d Y:%d", left / 1000 + 1, qmc.mx, qmc.my);
                    App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
                        "X[%d,%d] Y[%d,%d]",
                        g_mag_cal.mx_min, g_mag_cal.mx_max,
                        g_mag_cal.my_min, g_mag_cal.my_max);
                }
            }
        }
    }
}

/**
 * @brief 光流/陀螺/高度传感器处理
 *        每 6ms 调用，内部 30ms 分频执行重量级操作：
 *          - 陀螺仪 30ms 累积均值（旋转补偿用）
 *          - VL53L1X + SPA06 高度融合
 *          - PMW3901 光流读取 + 旋转补偿 + 高度补偿 + 速度解算
 *          - 机体加速度换算 + 水平速度互补滤波
 *          - OLED 调试输出
 */
void App_flight_process_flow_sensors(void)
{
    static uint8_t  flow_tick   = 0;
    static int32_t  gyro_sum_x  = 0;
    static int32_t  gyro_sum_y  = 0;

    /* ---- 每个周期累积陀螺仪 ADC 值（30ms 分频后取均值）---- */
    gyro_sum_x += gyro_accel_data.gyro.gyro_x;
    gyro_sum_y += gyro_accel_data.gyro.gyro_y;

    if (++flow_tick < 5) return;
    flow_tick = 0;

    /* ---- 以下每 30ms 执行一次 ---- */

    /* 陀螺仪 30ms 均值 → °/s（±2000°/s 量程） */
    float gyro_x_dps = (float)gyro_sum_x / 5.0f * (2000.0f / 32768.0f);
    float gyro_y_dps = (float)gyro_sum_y / 5.0f * (2000.0f / 32768.0f);
    gyro_sum_x = 0;
    gyro_sum_y = 0;

    /* ---- 高度传感器：VL53L1X 激光 + SPA06 气压计融合 ---- */
    {
        uint16_t laser_mm = Int_VL53L1X_GetDistance();
        float    baro_m   = 0.0f;

        if (g_spa06_ok)
        {
            SPA06_ReadData(&hi2c2);
            SPA06_ComputeAltitude();

            /* 前 10 次累积平均 = 起飞点海拔基准 */
            static uint8_t baro_cal_cnt = 0;
            static float   baro_cal_sum = 0.0f;
            if (baro_cal_cnt < 10)
            {
                baro_cal_sum += spa06.altitude;
                baro_cal_cnt++;
                if (baro_cal_cnt == 10)
                    g_baro_alt0 = baro_cal_sum / 10.0f;
            }

            baro_m = spa06.altitude - g_baro_alt0;

            /* 异常钳位 ±200m */
            if      (baro_m >  200.0f) baro_m = 0.0f;
            else if (baro_m < -200.0f) baro_m = 0.0f;
        }

        Common_Height_Update(laser_mm, baro_m, 0.030f);
        g_flow_height_mm = Common_Height_GetFusedMM();

        /* OLED: 气压/激光/融合高度 */
        {
            float h = Common_Height_GetFused();
            App_OLED_Postf(0, OLED_ROW_1, OLED_6X8,
                "B:%3d L:%3d F:%3d",
                (int)(baro_m * 100.0f),
                (int)(laser_mm / 10),
                (int)(h * 100.0f));
        }
    }

    /* ---- PMW3901 光流：读取 → 旋转补偿 → 高度补偿 → 速度 ---- */
    {
        PMW3901_MotionData_t motion;
        PMW3901_ReadMotion(&motion);

        Com_Flow_MapAxis(&motion, &g_flow_data);
        Com_Flow_RemoveRotation(&g_flow_data, gyro_x_dps, gyro_y_dps, 0.030f);
        Com_Flow_ApplyHeightScale(&g_flow_data, g_flow_height_mm);
        Com_Flow_CalcVelocity(&g_flow_data, 30.0f);
    }

    /* 水平速度来源：光流直接解算（不使用加速度积分融合） */

    /* ---- OLED 调试：光流速度(PID同源) + 航向 ---- */
    if (g_mag_calibrated)
    {
        App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
            "X:%5d Y:%5d S:%3d",
            (int)g_flow_data.vx,
            (int)g_flow_data.vy,
            g_flow_data.squal);

        App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
            "Y:%4.0f M:%4.0f", euler_angle.yaw, qmc.heading);
    }
}

