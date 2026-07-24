#ifndef __INT_VL53L1X_H__
#define __INT_VL53L1X_H__

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include <stdbool.h>

void     Int_VL53L1X_Init(void);
uint16_t Int_VL53L1X_GetDistance(void);
bool     Int_VL53L1X_IsDataReady(void);
void     Int_VL53L1X_ClearData(void);

#endif
