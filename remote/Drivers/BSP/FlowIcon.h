#ifndef __FLOW_ICON_H
#define __FLOW_ICON_H

#include <stdint.h>

/* 光流方向指示器位图（16×16，OLED 纵向字节格式） */
extern const uint8_t FlowIcon_None[];   /* 仅圆（无移动）     */
extern const uint8_t FlowIcon_X[];      /* 圆 + 上下箭头      */
extern const uint8_t FlowIcon_Y[];      /* 圆 + 左右箭头      */
extern const uint8_t FlowIcon_XY[];     /* 圆 + 十字箭头      */

#endif /* __FLOW_ICON_H */
