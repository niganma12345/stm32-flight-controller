#include "Int_VL53L1X.h"
#include <stdbool.h>

#define VL53L1_DEV  0x52

static bool initialized = false;

void Int_VL53L1X_Init(void)
{
    uint8_t state = 0;

    int t = 100;
    while (state == 0 && t--) { VL53L1X_BootState(VL53L1_DEV, &state); HAL_Delay(2); }
    if (state == 0) return;

    VL53L1X_ERROR err = VL53L1X_SensorInit(VL53L1_DEV);
    if (err != 0) return;

    /* 修正中断配置, 确保 CheckForDataReady 能正确触发 */
    VL53L1_WrByte(VL53L1_DEV, 0x0030, 0x11);
    VL53L1_WrByte(VL53L1_DEV, 0x0031, 0x02);
    VL53L1_WrByte(VL53L1_DEV, 0x0046, 0x20);

    VL53L1X_StartRanging(VL53L1_DEV);

    /* 等第一次测量完成 (约50~100ms) */
    for (int i = 0; i < 6; i++)
    {
        HAL_Delay(50);
        uint8_t rs;
        VL53L1_RdByte(VL53L1_DEV, 0x0089, &rs);
        if (rs != 0) break;
    }

    initialized = true;
}

uint16_t Int_VL53L1X_GetDistance(void)
{
    uint16_t dist = 0;
    if (!initialized) return 0;

    VL53L1X_GetDistance(VL53L1_DEV, &dist);

    /* 清除中断, 触发下一次测量 */
    VL53L1X_ClearInterrupt(VL53L1_DEV);

    return dist;
}
