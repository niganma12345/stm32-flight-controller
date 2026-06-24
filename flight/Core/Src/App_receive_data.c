#include "App_receive_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "com_debug.h"
#include "bmp280.h"
#include "adc.h"

extern volatile Remote_Data remote_data;
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

    // 3. 数据解析
    remote_data.thr = (rx_buff[3] << 8) | rx_buff[4];
    remote_data.yaw = (rx_buff[5] << 8) | rx_buff[6];
    remote_data.pit = (rx_buff[7] << 8) | rx_buff[8];
    remote_data.rol = (rx_buff[9] << 8) | rx_buff[10];
    remote_data.shutdown = rx_buff[11];
    remote_data.fix_height = rx_buff[12];

//     debug_printf(":%d,%d,%d,%d,%d,%d\n", remote_data.thr, remote_data.yaw, remote_data.pit, remote_data.rol, remote_data.shutdown, remote_data.fix_height);
    return 0;
}

/**
 * @brief 处理遥控器连接状态
 *
 * @param res 上一次接收数据的返回值
 */
void App_process_connect_state(uint8_t res)
{
    static Remote_State last_remote_state = REMOTE_CONNECTED;

    if (res == 0)
    {
        // 接收数据成功一次，即认为连接成功
        // 此处使用的全局变量，只在当前一个地方会修改LED灯控制，所以无需加锁读取使用
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else if (res == 1)
    {
        // 接收数据失败，累计次数
        retry_count++;
        if (retry_count >= MAX_RETRY_TIMES)
        {
            remote_state = REMOTE_DISCONNECTED;
            retry_count = 0;
        }
    }

    // 连接状态变化时打印
    if (remote_state != last_remote_state)
    {
        last_remote_state = remote_state;
        if (remote_state == REMOTE_CONNECTED)
            {} /* was: debug_printf("REMOTE CONNECTED\r\n"); */
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
    // 记录上一次状态，仅在状态变化时打印（初始值≠LOCKED确保首次打印）
    static Flight_State last_state = FAIL;

    switch (flight_state)
    {
    case LOCKED:
        // 只有在遥控已连接时才处理解锁逻辑，避免遥控断联时用旧数据误判
        if (remote_state != REMOTE_CONNECTED)
        {
            break;
        }

        // 检测解锁摇杆位置：油门最低 + 偏航最右
        if (remote_data.thr < UNLOCK_THR_MIN && remote_data.yaw > UNLOCK_YAW_MIN)
        {
            // 必须在松杆一次后（unlock_ready=1）才允许计数
            if (unlock_ready)
            {
                if (unlock_cnt == 0)
                {
                    // 首次检测到解锁位，打印当前值
                    // debug_printf("UNLOCKING... thr=%d yaw=%d (hold 1s)\r\n",
                    //     remote_data.thr, remote_data.yaw);
                }
                if (++unlock_cnt >= (UNLOCK_HOLD_MS / 6))  // 保持约1秒
                {
                    unlock_cnt = 0;
                    unlock_ready = 0;
                    flight_state = IDLE;
                    // debug_printf("UNLOCK OK -> IDLE\r\n");
                }
            }
        }
        else
        {
            if (unlock_cnt > 0)
            {
                // debug_printf("UNLOCK ABORT (stick moved)\r\n");
            }
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
        if (remote_data.shutdown == 1)
        {
            flight_state = LOCKED;
            remote_data.shutdown = 0;
            // debug_printf("SHUTDOWN -> LOCKED\r\n");
            g_shutdown_req = 1;
            break;
        }
        // 判断是否定高
        if (remote_data.fix_height == 1)
        {
            flight_state = FIX_HEIGHT;
            remote_data.fix_height = 0;
        }
        // 判断是否故障失联
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
            // debug_printf("DISCONNECT -> FAIL\r\n");
        }
        break;

    case FIX_HEIGHT:
        unlock_cnt = 0;
        unlock_ready = 0;
        // shutdown 信号紧急停机
        if (remote_data.shutdown == 1)
        {
            flight_state = LOCKED;
            remote_data.shutdown = 0;
            // debug_printf("SHUTDOWN -> LOCKED\r\n");
            g_shutdown_req = 1;
            break;
        }
        // 取消定高
        if (remote_data.fix_height == 1)
        {
            flight_state = NORMAL;
            remote_data.fix_height = 0;
        }
        // 判断故障
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
            // debug_printf("DISCONNECT -> FAIL\r\n");
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

    // 状态变化时统一打印当前状态
    if (flight_state != last_state)
    {
        last_state = flight_state;
        switch (flight_state)
        {
        case LOCKED:
            if (remote_state == REMOTE_CONNECTED)
                {} /* was: debug_printf("STATE = LOCKED (unlock: thr<100 + yaw>900 hold 1s)\r\n"); */
            break;
        case IDLE:      /* debug_printf("STATE = IDLE\r\n"); */      break;
        case NORMAL:    /* debug_printf("STATE = NORMAL\r\n"); */    break;
        case FIX_HEIGHT:/* debug_printf("STATE = FIX_HEIGHT\r\n"); */break;
        case FAIL:      /* debug_printf("STATE = FAIL\r\n"); */      break;
        default: break;
        }
    }
}

/**
 * @brief 读取 BMP280 高度和电池电压，打包并通过 NRF24L01 回传给遥控端
 *
 *        数据包格式: 'a' 'l' 't' + int16高度(cm) + flight_state + int16电压(0.1V) + 填充 → 17字节
 *        调用前需确保 BMP280 已初始化且收到遥控数据。
 */
void App_send_telemetry(void)
{
    // 电压分压比 = (R1+R2)/R2，R1=R2=100k → (200k/100k) = 2.0
    #define VOLTAGE_DIVIDER_RATIO 2.0f

    /* 读取 BMP280 高度 */
    extern volatile float g_bmp280_altitude;
    g_bmp280_altitude = BMP280_Get_Altitude(BMP280_Get_SeaLevel_Pressure());

    /* 读取电池电压：PB1 → ADC1_IN9 */
    extern volatile float g_battery_voltage;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
        g_battery_voltage = (float)adc_val / 4096.0f * 3.3f * VOLTAGE_DIVIDER_RATIO;
    }

    /* 打包数据包: 帧头'alt' + int16高度(cm) + flight_state + int16电压(0.1V) + 填充 = 17字节 */
    extern volatile Flight_State flight_state;
    uint8_t alt_pkt[NRF24L01_TX_PACKET_WIDTH];
    memset(alt_pkt, 0, NRF24L01_TX_PACKET_WIDTH);
    alt_pkt[0] = 'a';
    alt_pkt[1] = 'l';
    alt_pkt[2] = 't';
    int16_t alt_cm = (int16_t)(g_bmp280_altitude * 100.0f);
    alt_pkt[3] = (uint8_t)(alt_cm >> 8);
    alt_pkt[4] = (uint8_t)(alt_cm & 0xFF);
    alt_pkt[5] = (uint8_t)flight_state;
    uint16_t volt_dmv = (uint16_t)(g_battery_voltage * 10.0f);  /* 0.1V单位 */
    alt_pkt[6] = (uint8_t)(volt_dmv >> 8);
    alt_pkt[7] = (uint8_t)(volt_dmv & 0xFF);

    memcpy(NRF24L01_TxPacket, alt_pkt, NRF24L01_TX_PACKET_WIDTH);
    NRF24L01_Send();  /* 发送完成后自动切回接收模式 */
}
