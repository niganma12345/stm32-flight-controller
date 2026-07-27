#ifndef __COMMON_IMU_H
#define __COMMON_IMU_H
#include "Com_debug.h"
#include "Com_Config.h"
#include "math.h"
#include <stdbool.h>

/*============================================================================*/
/* 磁力计偏航修正增益                                                            */
/*============================================================================*/
/*
 * 磁力计修正已移至 PID 层，IMU 内部仅做加速度计修正(俯仰/横滚)。
 * 设为 0 彻底关闭 IMU 磁力计 PI，避免与 PID 偏航控制冲突。
 */
#define IMU_MAG_KP     2.0f    /* 磁力计PI：Kp越大拉得越快，过大易震荡 */
#define IMU_MAG_KI     0.01f   /* 磁力计PI：Ki越大稳态误差越小，过大会超调 */

/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

extern float RtA;
extern float Gyro_G;
extern float Gyro_Gr;

/**
 * @brief 四元数姿态解算 (Mahony 互补滤波)
 * @param gyroAccel:  MPU6050 6轴原始数据 (ADC counts)
 * @param eulerAngle: 输出的欧拉角 (pitch/roll/yaw)
 * @param dt:         采样周期 (s)
 * @param mag_valid:  磁力计数据是否有效
 * @param mag_x,y,z:  磁力计原始三轴 (raw counts)，仅 mag_valid=true 时有效
 *
 * @note  磁力计修正在四元数内部通过 Mahony 矢量交叉法完成，自动倾斜补偿，
 *        无需外部预计算航向角。
 */
void Common_IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt,
                              bool mag_valid,
                              int16_t mag_x, int16_t mag_y, int16_t mag_z);
float Common_IMU_GetNormAccZ(void);



#endif
