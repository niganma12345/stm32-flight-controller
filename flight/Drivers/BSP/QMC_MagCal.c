#include "QMC_MagCal.h"
#include <math.h>

/* ================================================================
 * QMC5883P 硬铁校准模块 —— 实现
 * ================================================================ */

/**
 * @brief   初始化校准器
 */
void QMC_MagCal_Init(QMC_MagCal_t *cal)
{
    cal->mx_min    =  32767;
    cal->mx_max    = -32768;
    cal->my_min    =  32767;
    cal->my_max    = -32768;
    cal->mx_offset = 0;
    cal->my_offset = 0;
    cal->done      = 0;
}

/**
 * @brief   输入原始值，自动更新 min/max
 */
void QMC_MagCal_Feed(QMC_MagCal_t *cal, int16_t mx, int16_t my)
{
    if (cal->done) return;

    if (mx < cal->mx_min) cal->mx_min = mx;
    if (mx > cal->mx_max) cal->mx_max = mx;
    if (my < cal->my_min) cal->my_min = my;
    if (my > cal->my_max) cal->my_max = my;
}

/**
 * @brief   锁定校准结果
 */
void QMC_MagCal_Lock(QMC_MagCal_t *cal)
{
    if (cal->done) return;

    cal->mx_offset = (cal->mx_max + cal->mx_min) / 2;
    cal->my_offset = (cal->my_max + cal->my_min) / 2;
    cal->done = 1;
}

/**
 * @brief   查询是否已锁定
 */
uint8_t QMC_MagCal_IsDone(QMC_MagCal_t *cal)
{
    return cal->done;
}

/**
 * @brief   计算校准后的航向角
 */
float QMC_MagCal_GetHeading(QMC_MagCal_t *cal, int16_t mx, int16_t my)
{
    float cx = (float)(mx - cal->mx_offset);
    float cy = (float)(my - cal->my_offset);
    float heading = atan2f(cy, cx) * 180.0f / 3.14159265f;
    if (heading < 0) heading += 360.0f;
    return heading;
}
