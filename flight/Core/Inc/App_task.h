#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "Com_config.h"

extern QueueHandle_t remote_data_queue;

void freertos_start(void);

#endif
