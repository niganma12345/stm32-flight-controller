#ifndef __QMC_MAGCAL_H
#define __QMC_MAGCAL_H

#include <stdint.h>

/* ================================================================
 * QMC5883P 硬铁校准模块（纯数据处理，无 RTOS/硬件/显示依赖）
 * ================================================================
 *
 * 使用流程：
 *   1. QMC_MagCal_Init(&cal)
 *   2. 循环读取磁力计，每次调用 QMC_MagCal_Feed(&cal, mx, my)
 *   3. 外部定时满足后调用 QMC_MagCal_Lock(&cal) 锁定偏移量
 *   4. 调用 QMC_MagCal_GetHeading(&cal, mx, my) 获取校准后的航向角
 */

/* 校准状态 */
typedef struct {
    int16_t  mx_min;         /* X 轴采集到的最小值 */
    int16_t  mx_max;         /* X 轴采集到的最大值 */
    int16_t  my_min;         /* Y 轴采集到的最小值 */
    int16_t  my_max;         /* Y 轴采集到的最大值 */
    int16_t  mx_offset;      /* X 轴硬铁偏移量（校准后自动计算） */
    int16_t  my_offset;      /* Y 轴硬铁偏移量（校准后自动计算） */
    uint8_t  done;           /* 0=校准中  1=已锁定 */
} QMC_MagCal_t;

/* ---- API ---- */

/**
 * @brief   初始化校准器（重置所有字段）
 * @param   cal  校准器指针
 */
void QMC_MagCal_Init(QMC_MagCal_t *cal);

/**
 * @brief   输入原始磁通量数据，自动更新 min/max
 * @param   cal  校准器指针
 * @param   mx   X 轴原始磁通量
 * @param   my   Y 轴原始磁通量
 * @note    校准锁定后（done=1）调用此函数无操作
 */
void QMC_MagCal_Feed(QMC_MagCal_t *cal, int16_t mx, int16_t my);

/**
 * @brief   锁定校准结果，根据已采集的 min/max 计算偏移量
 * @param   cal  校准器指针
 * @note    调用后 cal->done 置 1，后续 Feed 不再更新
 */
void QMC_MagCal_Lock(QMC_MagCal_t *cal);

/**
 * @brief   查询校准是否已完成
 * @param   cal  校准器指针
 * @retval  0=未完成  1=已完成
 */
uint8_t QMC_MagCal_IsDone(QMC_MagCal_t *cal);

/**
 * @brief   获取校准后的航向角
 * @param   cal  校准器指针
 * @param   mx   X 轴原始磁通量
 * @param   my   Y 轴原始磁通量
 * @return  航向角（0°~360°）
 * @note    校准完成前使用 offset=0 计算
 */
float QMC_MagCal_GetHeading(QMC_MagCal_t *cal, int16_t mx, int16_t my);

#endif /* __QMC_MAGCAL_H */
