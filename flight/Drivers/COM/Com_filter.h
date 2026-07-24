#ifndef __COMMON_FILTER_H
#define __COMMON_FILTER_H
#include "Com_debug.h"

/* 卡尔曼滤波器结构体 */
typedef struct
{
    float LastP; // 上一时刻的状态方差（或协方差）
    float Now_P; // 当前时刻的状态方差（或协方差）
    float out;   // 滤波器的输出值，即估计的状态
    float Kg;    // 卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;     // 过程噪声的方差，反映系统模型的不确定性
    float R;     // 测量噪声的方差，反映测量过程的不确定性
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];
int16_t Common_Filter_LowPass(int16_t newValue, int16_t preFilteredValue);

/**
 * @brief float 版一阶低通滤波（光流速度/位置等浮点数据去噪）
 * @param newValue          本次采样值
 * @param preFilteredValue  上一次滤波输出
 * @param alpha             平滑系数 0~1，越小越强（0.15=强 0.3=中 0.5=弱）
 * @return 滤波后的值
 */
float Common_Filter_LowPass_Float(float newValue, float preFilteredValue, float alpha);

double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);

/**
 * @brief 滑动窗口延迟线 — 返回 N 个周期前的旧值，用于传感器时序对齐
 * @param buffer  长度为 N 的环形缓冲区（调用方维护）
 * @param n       缓冲区长度（延迟深度）
 * @param input   本次新采样值
 * @return        N 个周期前的旧值（首次调用返回 0）
 */
float FloatShifter(float *buffer, uint8_t n, float input);

#endif
