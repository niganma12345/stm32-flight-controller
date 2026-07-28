#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "Com_config.h"

/* 飞行事件组标志位：nrf24l01_task 维护，led_task / 状态机查询 */
#define EVT_REMOTE_CONNECTED  (1 << 0)   /* 遥控器已连接 */

extern QueueHandle_t      remote_data_queue;   /* 遥控数据队列，深度1 */
extern EventGroupHandle_t flight_evt_group;    /* 飞行事件组 */

void freertos_start(void);

#endif
