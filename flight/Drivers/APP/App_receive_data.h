#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "NRF24L01.h"
#include "Com_config.h"


// 帧头校验值
#define FRAME_HEAD_CHECK_1 'n'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'm'

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
 * @brief 读取 BMP280 高度和电池电压，打包并通过 NRF24L01 回传给遥控端
 *
 *        数据包格式: 'a' 'l' 't' + int16高度(cm) + flight_state + int16电压(0.1V) + 填充 → 17字节
 *        调用前需确保 BMP280 已初始化且收到遥控数据。
 */
void App_send_telemetry(void);

#endif // __APP_RECEIVE_DATA__
