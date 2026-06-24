/**
 * @file    key.c
 * @brief   按键驱动 —— 消抖 + 按下/释放/短按检测
 *
 * 每 5ms 调用 key_scan()，返回当前事件（无事件返回 KEY_EVENT_NONE）。
 * 无 FIFO、无长按计时，极简设计。
 */
#include "key.h"

/* GPIO 引脚表 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} KeyPinInfo;

static const KeyPinInfo key_pins[KEY_COUNT] = {
    {K1_GPIO_Port, K1_Pin},
    {K2_GPIO_Port, K2_Pin},
    {K3_GPIO_Port, K3_Pin},
    {K4_GPIO_Port, K4_Pin},
    {K5_GPIO_Port, K5_Pin},
    {K6_GPIO_Port, K6_Pin},
};

/* 单个按键状态 */
typedef struct {
    uint8_t stable;        /* 消抖后稳定状态: 0=释放, 1=按下 */
    uint8_t last_reading;  /* 上一次 GPIO 原始读数              */
    uint8_t db_cnt;        /* 消抖积分计数                     */
} KeySlot;

static KeySlot g_keys[KEY_COUNT];
static uint8_t g_pressed_mask;

void key_init(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        g_keys[i].stable       = 0;
        g_keys[i].last_reading = 1;   /* 上拉 → 默认高电平 */
        g_keys[i].db_cnt       = 0;
    }
    g_pressed_mask = 0;
}

KeyEvent key_scan(void)
{
    KeyEvent ev;
    ev.key_id       = 0;
    ev.event        = KEY_EVENT_NONE;
    ev.pressed_mask = g_pressed_mask;

    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        KeySlot *ks = &g_keys[i];

        /* 读取 GPIO（上拉输入：低电平=按下） */
        uint8_t reading = (HAL_GPIO_ReadPin(key_pins[i].port, key_pins[i].pin) == GPIO_PIN_RESET) ? 0 : 1;

        /* 消抖积分 */
        if (reading == ks->last_reading) {
            if (ks->db_cnt < KEY_DEBOUNCE_CNT) ks->db_cnt++;
        } else {
            ks->db_cnt = 0;
        }
        ks->last_reading = reading;

        /* 积分满 → 确认稳定状态 */
        if (ks->db_cnt < KEY_DEBOUNCE_CNT) continue;

        uint8_t new_stable = (reading == 0) ? 1 : 0;
        if (new_stable == ks->stable) continue;

        /* 状态变化 */
        ks->stable = new_stable;
        if (new_stable) {
            /* 按下 */
            g_pressed_mask |= (1U << i);
            if (ev.event == KEY_EVENT_NONE) {
                ev.key_id       = i;
                ev.event        = KEY_EVENT_PRESS;
                ev.pressed_mask = g_pressed_mask;
            }
        } else {
            /* 释放 */
            g_pressed_mask &= ~(1U << i);
            if (ev.event == KEY_EVENT_NONE) {
                ev.key_id       = i;
                ev.event        = KEY_EVENT_SHORT;  /* 释放即短按 */
                ev.pressed_mask = g_pressed_mask;
            }
        }
    }

    return ev;
}

uint8_t key_is_pressed(uint8_t id)
{
    if (id >= KEY_COUNT) return 0;
    return g_keys[id].stable;
}

uint8_t key_get_mask(void)
{
    return g_pressed_mask;
}
