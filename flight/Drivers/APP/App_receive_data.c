#include "App_receive_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include "spa06.h"
#include "Com_height.h"
#include "adc.h"
#include "App_flight.h"   

extern QueueHandle_t remote_data_queue;
extern volatile uint8_t g_shutdown_req;

uint8_t rx_buff[NRF24L01_RX_PACKET_WIDTH] = {0};

// 遥控连接状态
extern volatile Remote_State remote_state;
// 飞行状态
extern volatile Flight_State flight_state;

// 重试次数
uint8_t retry_count = 0;

// 定高飞行目标高度（单位: m）
extern volatile float fix_height;

/**
 * @brief 接收遥控器发送的遥控数据 => 封装为结构体
 *
 *   NRF24L01 硬件自动应答 (EN_AA=0x3F) 已保证遥控端知道数据到达，
 *   此处不再额外阻塞发送，避免因应答发送失败而误断开连接。
 *
 * @return uint8_t 0:校验通过  1:没收到数据/校验失败
 */
uint8_t App_receive_data(void)
{
    memset(rx_buff, 0, NRF24L01_RX_PACKET_WIDTH);

    // 使用 NRF24L01 接收数据
    // 返回值: 0=无数据, 1=接收成功, 2=状态寄存器异常(已自恢复), 3=掉电模式(已自恢复)
    uint8_t res = NRF24L01_Receive();
    if (res != 1)
    {
        // 未收到数据或接收出错
        return 1;
    }

    // 收到数据，从全局接收缓冲区复制到本地缓冲区进行处理
    memcpy(rx_buff, NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);

    // 检查帧头非零即可（不用 strlen，二进制数据含 0x00 会误判）
    if (rx_buff[0] == 0)
    {
        return 1;
    }

    // 1. 帧头校验
    if (rx_buff[0] != FRAME_HEAD_CHECK_1 || rx_buff[1] != FRAME_HEAD_CHECK_2 || rx_buff[2] != FRAME_HEAD_CHECK_3)
    {
        return 1;
    }

    // 2. 帧尾校验（Byte0~12 累加和，4 字节 big-endian 存放在 Byte13~16）
    uint32_t sum = 0;
    uint32_t sum_receive = 0;

    for (uint8_t i = 0; i < 13; i++)
    {
        sum += rx_buff[i];
    }
    sum_receive = rx_buff[13] << 24 | rx_buff[14] << 16 | rx_buff[15] << 8 | rx_buff[16];

    if (sum != sum_receive)
    {
        return 1;
    }

    // 3. 数据解析 → 本地结构体 → 写入队列（原子发布）
    Remote_Data new_data;
    new_data.thr       = (rx_buff[3] << 8) | rx_buff[4];
    new_data.yaw       = (rx_buff[5] << 8) | rx_buff[6];
    new_data.pit       = (rx_buff[7] << 8) | rx_buff[8];
    new_data.rol       = (rx_buff[9] << 8) | rx_buff[10];
    new_data.shutdown  = rx_buff[11];
    new_data.fix_height = rx_buff[12];

    xQueueOverwrite(remote_data_queue, &new_data);

    return 0;
}

/**
 * @brief 处理遥控器连接状态
 *
 * @param res 上一次接收数据的返回值
 */
void App_process_connect_state(uint8_t res)
{
    if (res == 0)
    {
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else if (res == 1)
    {
        retry_count++;
        if (retry_count >= MAX_RETRY_TIMES)
        {
            remote_state = REMOTE_DISCONNECTED;
            retry_count = 0;
        }
    }
}

/**
 * @brief 处理飞机的飞行状态
 *
 *   穿越机常规解锁方式：油门最低 + 偏航最右 保持约1秒
 *   shutdown 信号：飞行中可紧急停机回到锁定状态
 */
void App_process_flight_state(void)
{
    // 摇杆解锁累计（每6ms调用一次，1000ms / 6 ≈ 167次）
    static uint16_t unlock_cnt = 0;
    // 防重复解锁：进入LOCKED后必须先松开解锁位再打回，才能开始计数
    static uint8_t  unlock_ready = 0;  // 0=等待松杆, 1=可以开始解锁

    /* 从队列读取当前遥控数据快照（本任务同时是写方，每次修改后覆盖写回） */
    Remote_Data rd;
    if (xQueuePeek(remote_data_queue, &rd, 0) != pdTRUE)
        return;  /* 队列为空，尚无数据 */

    switch (flight_state)
    {
    case LOCKED:
        // 只有在遥控已连接时才处理解锁逻辑，避免遥控断联时用旧数据误判
        if (remote_state != REMOTE_CONNECTED)
        {
            break;
        }

        // 检测解锁摇杆位置：油门最低 + 偏航最右
        if (rd.thr < UNLOCK_THR_MIN && rd.yaw > UNLOCK_YAW_MIN)
        {
            // 必须在松杆一次后（unlock_ready=1）才允许计数
            if (unlock_ready)
            {
                if (++unlock_cnt >= (UNLOCK_HOLD_MS / 6))  // 保持约1秒
                {
                    unlock_cnt = 0;
                    unlock_ready = 0;
                    flight_state = IDLE;
                }
            }
        }
        else
        {
            unlock_cnt = 0;   // 摇杆位置不满足则重置计数
            unlock_ready = 1; // 已松杆，下次打到位允许解锁
        }
        break;

    case IDLE:
        unlock_cnt = 0;
        unlock_ready = 0;
        // 已解锁空闲状态，直接进入飞行模式
        flight_state = NORMAL;
        break;

    case NORMAL:
        unlock_cnt = 0;
        unlock_ready = 0;
        // shutdown 信号紧急停机
        if (rd.shutdown == 1)
        {
            flight_state = LOCKED;
            rd.shutdown = 0;
            break;
        }
        // 定高/定点三态切换：NORMAL → FIX_HEIGHT
        if (rd.fix_height == 1)
        {
            flight_state = FIX_HEIGHT;
            rd.fix_height = 0;
        }
        // 判断是否故障失联
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FIX_HEIGHT:
        unlock_cnt = 0;
        unlock_ready = 0;
        // shutdown 信号紧急停机
        if (rd.shutdown == 1)
        {
            flight_state = LOCKED;
            rd.shutdown = 0;
            break;
        }
        // 三态循环切换：FIX_HEIGHT → MANUAL
        if (rd.fix_height == 1)
        {
            flight_state = MANUAL;
            rd.fix_height = 0;
        }
        // 判断故障
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case MANUAL:
        unlock_cnt = 0;
        unlock_ready = 0;
        // shutdown 信号紧急停机
        if (rd.shutdown == 1)
        {
            flight_state = LOCKED;
            rd.shutdown = 0;
            break;
        }
        // 三态循环切换：MANUAL → NORMAL（回到自稳）
        if (rd.fix_height == 1)
        {
            flight_state = NORMAL;
            rd.fix_height = 0;
        }
        // 判断故障
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;

    case FAIL:
        unlock_cnt = 0;
        unlock_ready = 0;
        // 故障状态不阻塞通信任务，由 flight_task 负责缓停电机并切回 LOCKED
        break;

    default:
        unlock_cnt = 0;
        unlock_ready = 0;
        break;
    }

    /* 将修改后的数据写回队列（shutdown/fix_height 可能已被清零） */
    xQueueOverwrite(remote_data_queue, &rd);
}

/**
 * @brief 读取高度和电池电压，打包并通过 NRF24L01 回传给遥控端
 *
 *        数据包格式: 'a' 'l' 't' + int16高度(cm) + flight_state + int16电压(0.1V) + 填充 → 17字节
 *        调用前需确保 BMP280 已初始化且收到遥控数据。
 */
void App_send_telemetry(void)
{
    // 电压分压比 = (R1+R2)/R2，R1=R2=100k → (200k/100k) = 2.0
    #define VOLTAGE_DIVIDER_RATIO 2.0f

    /* 读取融合高度（Com_height 激光+气压计融合，cm） */
    float altitude = g_fused_height;

    /* 读取电池电压：PB1 → ADC1_IN9 */
    float battery_voltage = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
        battery_voltage = (float)adc_val / 4096.0f * 3.3f * VOLTAGE_DIVIDER_RATIO;
    }

    /* 打包数据包: 帧头'alt' + int16高度(cm) + flight_state + int16电压(0.1V) + 填充 = 17字节 */
    extern volatile Flight_State flight_state;
    uint8_t alt_pkt[NRF24L01_TX_PACKET_WIDTH];
    memset(alt_pkt, 0, NRF24L01_TX_PACKET_WIDTH);
    alt_pkt[0] = 'a';
    alt_pkt[1] = 'l';
    alt_pkt[2] = 't';
    int16_t alt_cm = (int16_t)(altitude * 100.0f);
    alt_pkt[3] = (uint8_t)(alt_cm >> 8);
    alt_pkt[4] = (uint8_t)(alt_cm & 0xFF);
    alt_pkt[5] = (uint8_t)flight_state;
    uint16_t volt_dmv = (uint16_t)(battery_voltage * 10.0f);  /* 0.1V单位 */
    alt_pkt[6] = (uint8_t)(volt_dmv >> 8);
    alt_pkt[7] = (uint8_t)(volt_dmv & 0xFF);
    /* 光流方向标志（byte 8-9） */
    alt_pkt[8] = (g_flow_data.disp.delta_x != 0) ? 1 : 0;  /* 前后有移动 */
    alt_pkt[9] = (g_flow_data.disp.delta_y != 0) ? 1 : 0;  /* 左右有移动 */

    memcpy(NRF24L01_TxPacket, alt_pkt, NRF24L01_TX_PACKET_WIDTH);
    NRF24L01_Send();  /* 发送完成后自动切回接收模式 */
}
