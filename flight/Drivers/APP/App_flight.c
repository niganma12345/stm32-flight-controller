#include "App_flight.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Com_flow.h"
#include "Com_height.h"
#include "vl53l1.h"
#include "Int_VL53L1X.h"
#include "QMC5883P.h"
#include "App_oled.h"



uint8_t g_spa06_ok = 1;        /* SPA06 气压计是否可用 1不可用 SPA06_Init后复制为0可用 */
uint16_t   g_flow_height_mm = 0;       /*融合高度*/
uint16_t   g_disp_laser_mm  = 0;       /* 激光原始值，供 OLED 显示 */

Gyro_Accel_Struct gyro_accel_data = {0};/*姿态数据*/
Flow_Data_t g_flow_data = {0};          /*光流数据*/
Euler_struct euler_angle = {0};
Gyro_struct last_gyro = {0};


/* ---- QMC5883P 磁力计 ---- */

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
    .kp = 7.00f,        
    .ki = 0.00f,       
    .kd = 0.00f,
    .integral_max = 10.0f,   /* 积分限幅 ±10° */
    .output_max   = 30.0f,   /* 输出限幅 ±30（°/s） */
};
// X轴角速度结构体 => 对应横滚角内环
PID_Struct gyro_x_pid = {
    .kp = -3.00f,       
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
    .integral_max = 30.0f,   /* 直接混控，积分上限适度放宽 */
    .output_max   = 100.0f,  /* 直接混控，与 Com_limit(100,-100) 一致 */
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
 * @brief 飞控任务初始化 
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
     g_spa06_ok= SPA06_Init(&hi2c2);

    /* ---- PMW3901 光流传感器初始化 ---- */
    PMW3901_Init();

    /* ---- VL53L1X 激光测距初始化 ---- */
    Int_VL53L1X_Init();

    /* ---- QMC5883P 磁力计初始化 ---- */
    QMC5883P_Init(&hi2c2);

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



    // 3. 对测量变化比较大的加速度 使用更高级的滤波方式 => 卡尔曼滤波
    gyro_accel_data.accel.accel_x = Common_Filter_KalmanFilter(&kfs[0], gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = Common_Filter_KalmanFilter(&kfs[1], gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = Common_Filter_KalmanFilter(&kfs[2], gyro_accel_data.accel.accel_z);



    // 4. 四元数姿态解算 + 磁力计倾斜补偿融合
    Common_IMU_GetEulerAngle(&gyro_accel_data, &euler_angle, 0.006,
        g_qmc_ok,
        qmc.mx, qmc.my, qmc.mz);

}

/**
 * @brief 根据欧拉角 计算出PID的目标值
 *        光流有效时：速度PID叠加到角度外环目标值上，实现水平悬停锁定
 */
void App_flight_pid_process(const Remote_Data *rc)
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

    // =================================================
    //  俯仰角
    //================================================== 
    pitch_pid.desire = (rc->pit - 500) / 20.0f + flow_correction_pitch;
    if (pitch_pid.desire >  MAX_TILT_ANGLE) pitch_pid.desire =  MAX_TILT_ANGLE;
    if (pitch_pid.desire < -MAX_TILT_ANGLE) pitch_pid.desire = -MAX_TILT_ANGLE;
    pitch_pid.measure = euler_angle.pitch;
    gyro_y_pid.measure = (gyro_accel_data.gyro.gyro_y * 2000.0f / 32768.0f);
    Com_PID_Calc_Chain(&pitch_pid, &gyro_y_pid);

    // ==================================================
    //  横滚角
    // ==================================================
    roll_pid.desire = (rc->rol - 500) / 20.0f + flow_correction_roll;
    if (roll_pid.desire >  MAX_TILT_ANGLE) roll_pid.desire =  MAX_TILT_ANGLE;
    if (roll_pid.desire < -MAX_TILT_ANGLE) roll_pid.desire = -MAX_TILT_ANGLE;
    roll_pid.measure = euler_angle.roll;
    gyro_x_pid.measure = (gyro_accel_data.gyro.gyro_x * 2000.0f / 32768.0f);
    Com_PID_Calc_Chain(&roll_pid, &gyro_x_pid);

    // ==================================================
    //  偏航角（磁力计硬编码偏移，始终可用）
    // ==================================================
    {
        float yaw_stick = (rc->yaw - 500) / 50.0f;
        static float yaw_target = 0.0f;
        static Flight_State prev_yaw_state = LOCKED;

        /* 解锁瞬间 (LOCKED→活跃) 捕获当前航向 */
        Flight_State cur = (Flight_State)flight_state;
        if (prev_yaw_state == LOCKED && cur != LOCKED)
            yaw_target = euler_angle.yaw;
        prev_yaw_state = cur;

        if (fabsf(yaw_stick) >= 1.0f)
        {
            yaw_target += yaw_stick * 0.006f;
            if      (yaw_target >= 360.0f) yaw_target -= 360.0f;
            else if (yaw_target <  0.0f)   yaw_target += 360.0f;
        }

        yaw_pid.measure = euler_angle.yaw;
        yaw_pid.desire  = yaw_target;

        /* 误差归一化 ±180° */
        float err = yaw_pid.desire - yaw_pid.measure;
        if      (err >  180.0f) err -= 360.0f;
        else if (err < -180.0f) err += 360.0f;
        yaw_pid.desire = yaw_pid.measure + err;

        Com_PID_Calc(&yaw_pid);
    }
}

/**
 * @brief 根据PID的输出值 控制电机
 *
 */
void App_flight_control_motor(const Remote_Data *rc)
{
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
        left_top_motor.speed = rc->thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(yaw_pid.output, 100, -100);
        left_bottom_motor.speed = rc->thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(yaw_pid.output, 100, -100);
        right_top_motor.speed = rc->thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(yaw_pid.output, 100, -100);
        right_bottom_motor.speed = rc->thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(yaw_pid.output, 100, -100);
        break;
    case FIX_HEIGHT:
        // 定高模式 → 悬停油门基准 + 自稳 + 串级高度PID修正
        {
            int16_t base_thr = (int16_t)g_hover_thr;
            int16_t h_corr   = (int16_t)height_vel_pid.output;
            left_top_motor.speed     = base_thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(yaw_pid.output, 100, -100) + h_corr;
            left_bottom_motor.speed  = base_thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(yaw_pid.output, 100, -100) + h_corr;
            right_top_motor.speed    = base_thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(yaw_pid.output, 100, -100) + h_corr;
            right_bottom_motor.speed = base_thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(yaw_pid.output, 100, -100) + h_corr;
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
    if (rc->thr < 100)
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
void App_flight_fix_height_pid_process(const Remote_Data *rc)
{
    /* ---- 1. 状态切换检测：刚进入定高时记录目标 + 捕获悬停油门 ---- */
    if (prev_flight_state != FIX_HEIGHT && flight_state == FIX_HEIGHT)
    {
        fix_height   = g_fused_height;
        g_hover_thr  = (float)rc->thr;   /* 捕获当前油门作为悬停基准 */
    }
    /* 更新上一次状态 */
    prev_flight_state = flight_state;

    /* ---- 2. 串级 PID 计算（仅定高模式）---- */
    if (flight_state == FIX_HEIGHT)
    {
        float fused = g_fused_height;   /* 融合高度 */

        /* 外环：高度位置 PID → 输出垂直速度目标 (m/s) */
        height_pos_pid.desire  = fix_height;
        height_pos_pid.measure = fused;
        Com_PID_Calc(&height_pos_pid);

        /* 内环：垂直速度 PID → 输出油门补偿（正值=加油门，负值=减油门） */
        height_vel_pid.desire  = height_pos_pid.output;  /* 位置环输出 = 速度目标 */
        height_vel_pid.measure = g_vertical_vel;
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
 * @brief 磁力计处理 — 读取 + 航向计算 
 *        每 6ms 调用一次
 */
void App_flight_process_mag(void)
{
    QMC5883P_ReadData(&hi2c2);
    QMC5883P_ComputeHeading();
}

/**
 * @brief 光流/陀螺/高度传感器处理
 *        每 6ms 调用，内部 30ms 分频执行重量级操作：
 *          - 陀螺仪 30ms 累积均值（旋转补偿用）
 *          - VL53L1X + SPA06 高度融合
 *          - PMW3901 光流读取 + 旋转补偿 + 高度补偿 + 速度解算
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

    /* ---- 高度传感器：VL53L1X 激光 + SPA06 气压计 ---- */
    {
        uint16_t laser_mm = Int_VL53L1X_GetDistance();
			  g_disp_laser_mm = laser_mm;/*屏幕显示*/

        if (g_spa06_ok==0)/*判断气压计是否可用*/
        {
            SPA06_Update(&hi2c2);/*输出数据*/
            Common_Height_Calibrate(spa06.altitude);/*校准  返回1校准完成*/
        }

        Common_Height_Update(laser_mm, Common_Height_GetBaroRel(), 0.030f);/*计算融合高度与速度*/
        g_flow_height_mm = (uint16_t)(g_fused_height * 1000.0f);/*融合高度*/
    }

    /* ---- PMW3901 光流：内部读取 → 坐标映射 → 旋转补偿 → 高度补偿 → 速度 ---- */
    Com_Flow_Update(&g_flow_data, gyro_x_dps, gyro_y_dps, g_flow_height_mm);

}

/**
 * @brief 统一 OLED 显示刷新
 *
 * ROW_0: 姿态 P/R   ROW_1: 高度 B/L/F
 * ROW_2: 校准界面 / 光流速度   ROW_3: 校准界面 / 航向
 */
void App_flight_display(void)
{
    static uint8_t tick = 0;
    if (++tick < 17) return;
    tick = 0;

    /* ---- 磁力计未校准 → 只显示校准界面，不显示其他数据 ---- */
    if (QMC_HARD_OFS_X == 0 && QMC_HARD_OFS_Y == 0)
    {
        App_OLED_Postf(0, OLED_ROW_0, OLED_8X16, "Mag Calib");
        App_OLED_Postf(0, OLED_ROW_1, OLED_6X8, "Rotate 360 slowly");
        App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
            "X:%d..%d o:%d",
            (int)qmc.mx_min, (int)qmc.mx_max,
            (int)((qmc.mx_min + qmc.mx_max) / 2));
        App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
            "Y:%d..%d o:%d",
            (int)qmc.my_min, (int)qmc.my_max,
            (int)((qmc.my_min + qmc.my_max) / 2));
        return;
    }

    /* ---- ROW_0: 姿态 ---- */
    App_OLED_Postf(0, OLED_ROW_0, OLED_8X16,
        "P:%.1f R:%.1f", euler_angle.pitch, euler_angle.roll);

    /* ---- ROW_1: 高度 B(气压) / L(激光) / F(融合) (cm) ---- */
    App_OLED_Postf(0, OLED_ROW_1, OLED_6X8,
        "B:%3d L:%3d F:%3d    ",
        (int)(Common_Height_GetBaroRel() * 100.0f),
        (int)(g_disp_laser_mm / 10),
        (int)(g_fused_height * 100.0f));

    /* ---- ROW_2+3: 光流速度 + 航向 ---- */
    App_OLED_Postf(0, OLED_ROW_2, OLED_6X8,
        "X:%5d Y:%5d S:%3d",
        (int)g_flow_data.vx,
        (int)g_flow_data.vy,
        g_flow_data.squal);
    App_OLED_Postf(0, OLED_ROW_3, OLED_6X8,
        "Y:%4.0f M:%4.0f", euler_angle.yaw, qmc.heading);
}

