#include "App_flight.h"
#include "FreeRTOS.h"
#include "task.h"

Gyro_Accel_Struct gyro_accel_data = {0};
Euler_struct euler_angle = {0};
Gyro_struct last_gyro = {0};
float gyro_z_sum = 0;

extern volatile Remote_Data remote_data;
extern volatile Flight_State flight_state;
extern TaskHandle_t nrf24l01_task_handle;
// 电机结构体
Motor left_top_motor = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor left_bottom_motor = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor right_top_motor = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor right_bottom_motor = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

// PID的调参先调内环再调外环
// 俯仰PID结构体  => 角度环需要用到专业的PID调参
PID_Struct pitch_pid = {.kp = 7.00, .ki = 0.00, .kd = 0.00};
// Y轴角速度结构体 => 对应俯仰角内环
// 俯仰参数 => 俯仰方向需要调试 => 原则是调整的值 越小越好
PID_Struct gyro_y_pid = {.kp = -3.00, .ki = 0.00, .kd = -0.50};

// 横滚PID结构体
PID_Struct roll_pid = {.kp = 7.00, .ki = 0.00, .kd = 0.00};
// X轴角速度结构体 => 对应横滚角内环
PID_Struct gyro_x_pid = {.kp = -3.00, .ki = 0.00, .kd = -0.50};

// 偏航不稳定是正常的 => 在平稳飞行的范围 只需要保证飞机不要在原地转圈 => 可以的自然范围
// 偏航PID结构体  => 最好只通过内环角度来实现
PID_Struct yaw_pid = {.kp = -3.00, .ki = 0.00, .kd = 0.00};
// Z轴角速度结构体 => 对应偏航角内环
PID_Struct gyro_z_pid = {.kp = 0.00, .ki = 0.00, .kd = 0.00};

// 定高的PID结构体
PID_Struct height_pid = {.kp = 0.00, .ki = 0.00, .kd = 0.00};

//// 定高飞行的目标高度（单位: m）
extern volatile float fix_height;

// 上一次飞行状态（用于检测FIX_HEIGHT进入时刻）
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

    // 初始化 BMP280 气压计
    if (BMP280_Init() == 0)
    {
        // 校准海平面气压（以当前位置作为高度零点）
        BMP280_Calibrate_SeaLevel(0.0f);
    }
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

    // 4. 通过加速度和角速度来计算当前飞机的倾斜角度 => 姿态解算
    // 使用互补滤波求欧拉角 => 融合使用加速度解算 => 俯仰角和横滚角可以使用
      euler_angle.pitch = atan2(gyro_accel_data.accel.accel_x * 1.0, gyro_accel_data.accel.accel_z) / 3.14159 * 180;

      euler_angle.roll = atan2(gyro_accel_data.accel.accel_y * 1.0, gyro_accel_data.accel.accel_z) / 3.14159 * 180;

    // // 偏航角 => 只能使用角速度积分
    // // 16位ADC数值转换为°/s  => 量程±2000°/s
//      gyro_z_sum += (gyro_accel_data.gyro.gyro_z * 2000.0 / 32768.0) * 0.006;
//      euler_angle.yaw = gyro_z_sum;

//    // 也可以使用四元数来做姿态解算
    Common_IMU_GetEulerAngle(&gyro_accel_data, &euler_angle, 0.006);

    // 俯仰角  横滚角  偏航角
//     debug_printf(":%.2f,%.2f,%.2f\n", euler_angle.pitch, euler_angle.roll, euler_angle.yaw);
}

/**
 * @brief 根据欧拉角 计算出PID的目标值
 *
 */
void App_flight_pid_process(void)
{
    // 俯仰角
    // 1. 需要赋值目标值和测量值
    // 外环的目标角度 => 保持平稳飞行 => 值为0 => 如果需要遥控飞行 => 目标角度就是遥控器的值
    // 数值转换 => remote_data.pit(0-1000,500为中间值)  控制范围在±10°
    pitch_pid.desire = (remote_data.pit - 500) / 20.0;
    // 外环的测量值 => 就是当前的俯仰角
    pitch_pid.measure = euler_angle.pitch;
    // 内环的测量值 => 当前的角速度  => 单位要转换一下
    gyro_y_pid.measure = (gyro_accel_data.gyro.gyro_y * 2000.0 / 32768.0);

    // 2. 进行PID计算
    Com_PID_Calc_Chain(&pitch_pid, &gyro_y_pid);

    // 先观察内环  => 角速度控制 => 目标位角速度为0
    // debug_printf(":%.2f,%.2f\n", gyro_y_pid.err, gyro_y_pid.output);

    // 横滚角
    // 1. 需要赋值目标值和测量值
    // 外环的目标角度 => 保持平稳飞行 => 值为0 => 如果需要遥控飞行 => 目标角度就是遥控器的值
    roll_pid.desire = (remote_data.rol - 500) / 20.0;
    // 内环的测量值
    roll_pid.measure = euler_angle.roll;
    // 外环的测量值
    gyro_x_pid.measure = (gyro_accel_data.gyro.gyro_x * 2000.0 / 32768.0);

    // 2. 进行PID计算
    Com_PID_Calc_Chain(&roll_pid, &gyro_x_pid);

    // 偏航角
    // 1. 需要赋值目标值和测量值
    yaw_pid.desire = (remote_data.yaw - 500) / 50.0;
    // 内环的测量值
    yaw_pid.measure = euler_angle.yaw;
    // 外环的测量值
    gyro_z_pid.measure = (gyro_accel_data.gyro.gyro_z * 2000.0 / 32768.0);

    // 2. 进行PID计算
    Com_PID_Calc_Chain(&yaw_pid, &gyro_z_pid);
}

/**
 * @brief 根据PID的输出值 控制电机
 *
 */
void App_flight_control_motor(void)
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
        // 飞行模式 => 当前需要角速度 => 需要自稳 => 需要一个基本的飞行效率 => 前两个电机正转 后两个反转
        // 根据不同需要程度的PID控制进行实际的输出叠加
        left_top_motor.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_top_motor.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        break;
    case FIX_HEIGHT:
        // 定高模式 => 在自稳基础上叠加高度PID输出
        // height_pid.output: 正值表示需要升力 → 增加油门
        left_top_motor.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + (int16_t)height_pid.output;
        left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + (int16_t)height_pid.output;
        right_top_motor.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + (int16_t)height_pid.output;
        right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + (int16_t)height_pid.output;
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

    // 安全保护 => 当油门值为<50时 => 强制将速度置为0
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
}

/**
 * @brief 进入定高功能之后的PID计算
 *
 * 每24ms调用一次（在 flight_task 中由 height_tick 计数器控制）
 * 检测状态变化时自动记录当前高度作为目标
 */
void App_flight_fix_height_pid_process(void)
{
    // 1. 检测是否刚进入 FIX_HEIGHT 状态
    if (prev_flight_state != FIX_HEIGHT && flight_state == FIX_HEIGHT)
    {
        // 首次进入定高模式，记录当前高度作为目标
        // 多次读取取平均值，减少噪声
        float alt_sum = 0.0f;
        for (uint8_t i = 0; i < 5; i++)
        {
            alt_sum += BMP280_Get_Altitude(BMP280_Get_SeaLevel_Pressure());
            vTaskDelay(2);
        }
        fix_height = alt_sum / 5.0f;
    }
    prev_flight_state = flight_state;

    // 2. 定高状态下进行高度PID计算
    if (flight_state == FIX_HEIGHT)
    {
        // 目标值 = 进入定高时记录的高度
        height_pid.desire = fix_height;
        // 测量值 = 当前气压计高度
        height_pid.measure = BMP280_Get_Altitude(BMP280_Get_SeaLevel_Pressure());

        // 3. 进行单级PID计算得到输出值
        Com_PID_Calc(&height_pid);

        // 限制高度PID输出范围，避免对油门影响过大
        height_pid.output = Com_limit((int16_t)height_pid.output, 80, -80);
    }
    else
    {
        // 非定高状态，清零高度PID输出
        height_pid.output = 0.0f;
        height_pid.integral = 0.0f;
        height_pid.last_err = 0.0f;
    }
}

