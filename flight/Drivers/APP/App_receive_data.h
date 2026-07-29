#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "NRF24L01.h"
#include "Com_config.h"


// 帧头校验值（接收方向：遥控→飞控）
#define FRAME_HEAD_CHECK_1 'n'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'm'

// 帧头（发送方向：飞控→遥控 遥测回传）
#define FRAME_HEAD_TELE_1 'a'
#define FRAME_HEAD_TELE_2 'l'
#define FRAME_HEAD_TELE_3 't'

// 最大重试次数（6ms周期，100次 = 600ms防抖动）
#define MAX_RETRY_TIMES 100

// 摇杆解锁参数（穿越机常规方式：油门最低 + 偏航最右）
#define UNLOCK_THR_MIN    100    // 油门必须低于此值
#define UNLOCK_YAW_MIN    800    // 偏航必须大于此值（最右）
#define UNLOCK_HOLD_MS    1000   // 保持时间（毫秒）

/**
 * @brief 接收遥控器发送的遥控数据 => 封装为结构体
 *
 *   NRF24L01 硬件自动应答 (EN_AA=0x3F) 已保证遥控端知道数据到达，
 *   此处不再额外阻塞发送，避免因应答发送失败而误断开连接。
 *
 * @return uint8_t 0:校验通过  1:没收到数据/校验失败
 */
uint8_t App_receive_data(void);


/**
 * @brief 处理连接状态的状态
 *
 * @param res 上一次接收数据的返回值
 */
void App_process_connect_state(uint8_t res);



/**
 * @brief 处理飞机的飞行状态
 *
 */
void App_process_flight_state(void);

/**
 * @brief 遥测数据结构（flight_task 打包 → 队列 → nrf24l01_task 发送）
 */
typedef struct
{
    float   altitude;     /* 融合高度 (m) */
    uint8_t flight_state; /* 飞行状态 */
    uint8_t flow_x;       /* 光流前后有移动 */
    uint8_t flow_y;       /* 光流左右有移动 */
} Telemetry_t;

/**
 * @brief 遥测回传：读队列 + 电池电压 → 打包发送
 * @return uint8_t 0:发送成功  1:无数据/失败
 */
uint8_t App_send_telemetry(void);

#endif // __APP_RECEIVE_DATA__
