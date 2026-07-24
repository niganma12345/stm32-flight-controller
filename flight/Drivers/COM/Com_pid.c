#include "Com_pid.h"

/**
 * @brief 单次PID计算（含积分限幅和输出限幅）
 *        当 integral_max 或 output_max 为 0 时，不限幅（向后兼容）
 */
void Com_PID_Calc(PID_Struct *pid)
{
    // 1. 目标和测量 => 计算误差值（误差 = 目标值 - 测量值）
    pid->err = pid->desire - pid->measure;

    // 2. 计算 P 项和 D 项（不含积分，用于抗饱和判断）
    float der = pid->err - pid->last_err;
    float p_term = pid->kp * pid->err;
    float d_term = pid->kd * der / PID_PERIOD;
    float i_term = pid->ki * pid->integral * PID_PERIOD;

    // 3. 计算总输出
    pid->output = p_term + i_term + d_term;

    // 4. 输出限幅
    int saturated = 0;
    if (pid->output_max > 0.0f)
    {
        if (pid->output > pid->output_max)
        {
            pid->output = pid->output_max;
            saturated = 1;
        }
        else if (pid->output < -pid->output_max)
        {
            pid->output = -pid->output_max;
            saturated = -1;
        }
    }

    // 5. 条件积分（抗饱和）：输出饱和且误差与饱和方向一致时，不再累积积分
    if (saturated == 0 || (saturated == 1 && pid->err < 0) || (saturated == -1 && pid->err > 0))
    {
        pid->integral += pid->err;
    }

    // 6. 积分限幅
    if (pid->integral_max > 0.0f)
    {
        if (pid->integral > pid->integral_max)
            pid->integral = pid->integral_max;
        else if (pid->integral < -pid->integral_max)
            pid->integral = -pid->integral_max;
    }

    // 7. 保存上一次误差
    pid->last_err = pid->err;
}

// 串级PID计算
void Com_PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid)
{
    // 1.先计算外环
    Com_PID_Calc(out_pid);
    // 2.将外环的输出值作为内环的目标值
    in_pid->desire = out_pid->output;
    // 3. 计算内环
    Com_PID_Calc(in_pid);
}

/**
 * @brief 限制数值在正常的范围内
 *
 * @param speed
 * @param max_speed
 * @param min_speed
 * @return int16_t
 */
int16_t Com_limit(int16_t speed, int16_t max_speed, int16_t min_speed)
{
    if (speed > max_speed)
    {
        return max_speed;
    }
    else if (speed < min_speed)
    {
        return min_speed;
    }
    return speed;
}
