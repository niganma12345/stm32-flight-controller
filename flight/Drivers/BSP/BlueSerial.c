#include "BlueSerial.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

char BlueSerial_RxPacket[100];      /* 接收数据包数组，数据包格式 "[MSG]" */
uint8_t BlueSerial_RxFlag;          /* 蓝牙串口接收标志位 */

/* HAL 接收中断用单字节缓冲 */
static uint8_t BlueSerial_RxByte;

/**
  * 函    数：蓝牙串口初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：USART2 硬件初始化已由 MX_USART2_UART_Init() 完成，
  *           本函数仅开启中断接收。
  */
void BlueSerial_Init(void)
{
    /* 启动中断接收：每次接收1个字节，接收完成后触发 HAL_UART_RxCpltCallback */
    HAL_UART_Receive_IT(&huart2, &BlueSerial_RxByte, 1);
}

/**
  * 函    数：蓝牙串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void BlueSerial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart2, &Byte, 1, HAL_MAX_DELAY);
}

/**
  * 函    数：蓝牙串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)
    {
        BlueSerial_SendByte(Array[i]);
    }
}

/**
  * 函    数：蓝牙串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void BlueSerial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        BlueSerial_SendByte(String[i]);
    }
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
static uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

/**
  * 函    数：蓝牙串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    BlueSerial_SendString(String);
}

/**
  * 函    数：HAL 串口接收完成回调
  * 参    数：huart 触发回调的串口句柄
  * 返 回 值：无
  * 说    明：使用状态机解析 "[MSG]" 格式的数据包
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        static uint8_t RxState = 0;     /* 状态机状态 */
        static uint8_t pRxPacket = 0;   /* 当前接收数据位置 */

        uint8_t RxData = BlueSerial_RxByte;

        /* 状态机解析数据包 */
        if (RxState == 0)
        {
            /* 状态0：等待包头 '[' */
            if (RxData == '[' && BlueSerial_RxFlag == 0)
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            /* 状态1：接收数据，等待包尾 ']' */
            if (RxData == ']')
            {
                RxState = 0;
                BlueSerial_RxPacket[pRxPacket] = '\0';
                BlueSerial_RxFlag = 1;
            }
            else
            {
                BlueSerial_RxPacket[pRxPacket] = RxData;
                pRxPacket ++;
            }
        }

        /* 重新启动中断接收下一个字节 */
        HAL_UART_Receive_IT(&huart2, &BlueSerial_RxByte, 1);
    }
}
