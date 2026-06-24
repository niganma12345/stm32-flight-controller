#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"
#include "main.h"

/* 按键编号 */
#define KEY_ID_K1  0
#define KEY_ID_K2  1
#define KEY_ID_K3  2
#define KEY_ID_K4  3
#define KEY_ID_K5  4
#define KEY_ID_K6  5
#define KEY_COUNT  6

/* 按键位掩码（组合键） */
#define KEY_MASK_K1  (1U << 0)
#define KEY_MASK_K2  (1U << 1)
#define KEY_MASK_K3  (1U << 2)
#define KEY_MASK_K4  (1U << 3)
#define KEY_MASK_K5  (1U << 4)
#define KEY_MASK_K6  (1U << 5)

/* 按键事件 */
typedef enum {
    KEY_EVENT_NONE    = 0,
    KEY_EVENT_PRESS   = 1,   /* 按下瞬间     */
    KEY_EVENT_RELEASE = 2,   /* 释放瞬间     */
    KEY_EVENT_SHORT   = 3,   /* 短按（释放时）*/
} KeyEventType;

typedef struct {
    uint8_t      key_id;
    KeyEventType event;
    uint8_t      pressed_mask;  /* 当前所有按下键的位掩码 */
} KeyEvent;

/* 消抖次数（key_scan 每 5ms 调用一次，3 × 5 = 15ms 消抖） */
#define KEY_DEBOUNCE_CNT  3

/* API */
void    key_init(void);
KeyEvent key_scan(void);
uint8_t key_is_pressed(uint8_t id);
uint8_t key_get_mask(void);

#endif /* __KEY_H */
