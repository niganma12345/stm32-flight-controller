#include "Com_IMU.h"
#include <stdbool.h>

extern uint8_t g_mag_calibrated;
extern int16_t g_mag_ofs_x;
extern int16_t g_mag_ofs_y;
/* ============================欧拉角计算================================== */
/* ===============================开始===================================== */

/* 计算欧拉角用到的3个参数 */
float RtA = 57.2957795f; // 弧度->度
// 陀螺仪初始化量程+-2000度/秒于 1/(65536 / 4000) = 0.03051756*2
// float Gyro_G = 0.03051756f * 2;
float Gyro_G = 4000.0 / 65536; // 度/s
// 度每秒,转换弧度每秒则 2*0.03051756 * 0.0174533f = 0.0005326*2
// float Gyro_Gr = 0.0005326f * 2;
float Gyro_Gr = 4000.0 / 65536 / 180 * 3.1415926; // 弧度/s


#define squa(Sq) (((float)Sq) * ((float)Sq))      /* 计算平方 */
/**
 * @description: 快速计算 1/sqrt(num)
 * @param {float} number
 */
static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration （第一次牛顿迭代）
    return y;
}

static float normAccz; /* z轴上的加速度 */
/**
 * @description: 根据mpu的6轴数据, 获取表征姿态的欧拉角
 * @param {GyroAccel_Struct} *gyroAccel mpu的6轴数据
 * @param {EulerAngle_Struct} *EulerAngle 计算后得到的欧拉角
 * @param {float} dt 采样周期 (单位s)
 * @return {*}
 */
void Common_IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt,
                              bool mag_valid,
                              int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    static Quaternion_Struct NumQ = {1, 0, 0, 0};
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // 加速度归一化
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accel_x) +
                       squa(gyroAccel->accel.accel_y) +
                       squa(gyroAccel->accel.accel_z));

    Acc.x = gyroAccel->accel.accel_x * NormQuat;
    Acc.y = gyroAccel->accel.accel_y * NormQuat;
    Acc.z = gyroAccel->accel.accel_z * NormQuat;
    // 向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // 角速度融合加速度积分补偿值
    Gyro.x = gyroAccel->gyro.gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; // 弧度制
    Gyro.y = gyroAccel->gyro.gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    /* ---- 磁力计偏航 PI 修正 (倾斜补偿角度法) ---- */
    /*
     * 用四元数已知的 pitch/roll 对磁力计做倾斜补偿，算出补偿后的航向角，
     * 再与四元数当前 yaw 求差，PI 叠加到 Gyro.z。
     *
     * 倾斜补偿公式（四元数提取 pitch, roll）：
     *   cosP = cos(pitch), sinP = sin(pitch), cosR = cos(roll), sinR = sin(roll)
     *   hx_comp = mx*cosP + my*sinR*sinP - mz*cosR*sinP
     *   hy_comp =           my*cosR         + mz*sinR
     *   heading = atan2(-hy_comp, hx_comp)
     */
    if (mag_valid)
    {
        static float mag_int = 0.0f;

        /* 从当前四元数提取 pitch, roll */
        float quat_pitch = asinf(2.0f * (NumQ.q0 * NumQ.q2 - NumQ.q1 * NumQ.q3));
        float quat_roll  = atan2f(2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3),
                                  1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2));
        float cosP = cosf(quat_pitch);
        float sinP = sinf(quat_pitch);
        float cosR = cosf(quat_roll);
        float sinR = sinf(quat_roll);

        /* 硬铁校准偏移量扣除 */
        float mx_raw = (float)mag_x;
        float my_raw = (float)mag_y;
        if (g_mag_calibrated)
        {
            mx_raw -= (float)g_mag_ofs_x;
            my_raw -= (float)g_mag_ofs_y;
        }

        /* 倾斜补偿（磁力计机体 → 水平面） */
        float hx = mx_raw * cosP
                 + my_raw * sinR * sinP
                 - (float)mag_z * cosR * sinP;
        float hy = my_raw * cosR
                 + (float)mag_z * sinR;

        /* 航向角 0~360 (atan2(hy, hx): 0=正北, 顺时针增加, 与OLED一致) */
        float mag_heading = atan2f(hy, hx) * RtA;
        if (mag_heading < 0.0f) mag_heading += 360.0f;

        /* 四元数当前偏航 */
        float quat_yaw = atan2f(2.0f * (NumQ.q0 * NumQ.q3 + NumQ.q1 * NumQ.q2),
                                1.0f - 2.0f * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3)) * RtA;
        if (quat_yaw < 0.0f) quat_yaw += 360.0f;

        /* 误差归一化到 ±180° */
        float yaw_err = mag_heading - quat_yaw;
        if      (yaw_err >  180.0f) yaw_err -= 360.0f;
        else if (yaw_err < -180.0f) yaw_err += 360.0f;

        /* PI: 误差(°) → °/s → rad/s */
        mag_int += IMU_MAG_KI * yaw_err * dt;
        if      (mag_int >  1.0f) mag_int =  1.0f;
        else if (mag_int < -1.0f) mag_int = -1.0f;

        Gyro.z += (IMU_MAG_KP * yaw_err + mag_int) * 0.0174533f;
    }

    // 一阶龙格库塔法, 更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /*机体坐标系下的Z方向向量*/
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;     /*矩阵(3,1)项*/
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;     /*矩阵(3,2)项*/
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*矩阵(3,3)项*/

    /* 偏航角: 从四元数提取 (已被磁力计 PI 修正，无磁力计时等同纯陀螺积分) */
    eulerAngle->yaw = atan2f(2.0f * (NumQ.q0 * NumQ.q3 + NumQ.q1 * NumQ.q2),
                             1.0f - 2.0f * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3)) * RtA;
    if (eulerAngle->yaw < 0.0f) eulerAngle->yaw += 360.0f;

    eulerAngle->pitch = asin(vecxZ) * RtA;            // 俯仰角
    eulerAngle->roll  = atan2f(vecyZ, veczZ) * RtA;   // 横滚角

    normAccz = gyroAccel->accel.accel_x * vecxZ + gyroAccel->accel.accel_y * vecyZ + gyroAccel->accel.accel_z * veczZ; /*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/
}

/**
 * @description: 获取Z轴上的加速度 (如果已经倾斜,会考虑z轴上加速度的合成)
 * @return {*}
 */
float Common_IMU_GetNormAccZ(void)
{
    return normAccz;
}
/* ======================欧拉角计算================================== */
/* ========================结束===================================== */
