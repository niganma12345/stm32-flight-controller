#ifndef _COM_CONFIG_H
#define _COM_CONFIG_H

#include "main.h"

/*============================================================================
 * COM_CONFIG — 跨模块共享类型定义
 *
 * 所有被 COM 层定义、APP/BSP 层使用的结构体/枚举集中在此文件中。
 * 模块内部专用的类型留在各自的头文件中。
 *============================================================================*/

/* ---- 飞行状态 ---- */
typedef enum
{
    LOCKED = 0,   /* 未解锁（安全锁定） */
    IDLE,         /* 已解锁，空闲待命 */
    NORMAL,       /* 正常飞行模式 */
    FIX_HEIGHT,   /* 定高飞行模式 */
    MANUAL,       /* 手动飞行模式（角度自稳，不使用光流） */
    FAIL,         /* 故障模式 */
} Flight_State;

/* ---- 遥控数据 ---- */
typedef struct
{
    int16_t thr;        /* 油门 0~1000 */
    int16_t yaw;        /* 偏航 0~1000，中位 500 */
    int16_t pit;        /* 俯仰 0~1000，中位 500 */
    int16_t rol;        /* 横滚 0~1000，中位 500 */
    uint8_t shutdown;   /* 1: 关机  0: 正常 */
    uint8_t fix_height; /* 1: 切换定高  0: 不切换 */
} Remote_Data;

/* ---- 传感器原始数据 ---- */

/* 陀螺仪 16 位 ADC 值 */
typedef struct
{
    int16_t gyro_x; /* 往右转为正 → 横滚角速度 */
    int16_t gyro_y; /* 往前转为正 → 俯仰角速度 */
    int16_t gyro_z; /* 逆时针转为正 → 偏航角速度 */
} Gyro_struct;

/* 加速度计 16 位 ADC 值 */
typedef struct
{
    int16_t accel_x; /* 往前为正 */
    int16_t accel_y; /* 往左为正 */
    int16_t accel_z; /* 朝上为正 */
} Accel_struct;

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
} Gyro_Accel_Struct;

/* ---- 姿态解算结果 ---- */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

/* ---- 光流 ---- */

/* 像素位移（前后/左右） */
typedef struct
{
    int16_t delta_x;
    int16_t delta_y;
} Flow_Displacement_t;

/* 物理位移 (mm) */
typedef struct
{
    float x;
    float y;
} Flow_RealPos_t;

/* 光流完整数据 */
typedef struct
{
    Flow_Displacement_t disp;
    Flow_RealPos_t      pos;
    float               vx;         /* 前后速度 (mm/s) */
    float               vy;         /* 左右速度 (mm/s) */
    uint8_t             squal;      /* 表面质量 */
    uint8_t             is_motion;  /* 检测到移动 */
    uint8_t             is_valid;   /* 数据有效 */
} Flow_Data_t;

#endif // !_COM_CONFIG_H
