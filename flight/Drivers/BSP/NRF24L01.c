#include "stm32f1xx_hal.h"
#include "NRF24L01_Define.h"
#include "spi.h"

#define NRF24L01_SPI_TIMEOUT 100U

/********************* 【引脚宏定义 —— CE 和 CSN 由 GPIO 软件控制】 *********************/
#define NRF24L01_CE_PORT    GPIOA
#define NRF24L01_CE_PIN     GPIO_PIN_8

#define NRF24L01_CSN_PORT   GPIOA
#define NRF24L01_CSN_PIN    GPIO_PIN_4
/********************* 【引脚宏定义结束】 *********************/

/*全局变量*********************/
uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];

uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];
/*********************全局变量*/

/*引脚操作函数（仅 CE 和 CSN，SCK/MOSI/MISO 由硬件 SPI 接管）*/
void NRF24L01_W_CE(uint8_t BitValue)
{
    HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void NRF24L01_W_CSN(uint8_t BitValue)
{
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 硬件SPI交换字节 */
static uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
    uint8_t RxData;
    HAL_SPI_TransmitReceive(&hspi1, &Byte, &RxData, 1, NRF24L01_SPI_TIMEOUT);
    return RxData;
}

/*********************通信协议*/


/*指令实现*********************/

/**
  * 函    数：NRF24L01读取寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 返 回 值：指定寄存器的数据，范围：0x00~0xFF
  */
uint8_t NRF24L01_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，得到指定地址的数据*/
	Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回读到的一个字节数据*/
	return Data;
}

/**
  * 函    数：NRF24L01读取寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器的地址，范围：0x00~0x1F
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Count 指定读取的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，循环接收多次，得到指定地址下的多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将接收到的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：Data 要写入的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，在指定地址下写入数据*/
	NRF24L01_SPI_SwapByte(Data);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Count 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，循环发送多次，在指定地址下写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到指定地址中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01读取Rx有效载荷
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Count 指定读取的数量，范围：0~32
  * 返 回 值：无
  */
void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读取Rx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
	
	/*发送读取Rx有效载荷指令后，开始交换接收，循环接收多次，得到多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将读取的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入Tx有效载荷
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Count 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写入Tx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
	
	/*发送写入Tx有效载荷指令后，开始交换发送，循环发送多次，写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到Tx有效载荷中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01清空Tx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_FlushTx(void)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Tx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01清空Rx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_FlushRx(void)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Rx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01读取状态寄存器
  * 参    数：无
  * 返 回 值：状态寄存器的值，范围：0x00~0xFF
  */
uint8_t NRF24L01_ReadStatus(void)
{
	uint8_t Status;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，空指令*/
	/*第一个字节发送任意指令，都可以交换得到状态寄存器的值*/
	Status = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回状态寄存器的值*/
	return Status;
}

/*********************指令实现*/


/*功能函数*********************/

/**
  * 函    数：NRF24L01进入掉电模式（CE = 0，PWR_UP = 0）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_PowerDown(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config &= ~0x02;								//配置寄存器位1（PWR_UP）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函    数：NRF24L01进入待机模式1（CE = 0，PWR_UP = 1）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_StandbyI(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函    数：NRF24L01进入接收模式（CE = 1，PWR_UP = 1，PRIM_RX = 1）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_Rx(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x03;									//配置寄存器位1（PWR_UP）和位0（PRIM_RX）都置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为1，所以进入接收模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函    数：NRF24L01进入发送模式（CE = 1，PWR_UP = 1，PRIM_RX = 0）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_Tx(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	Config &= ~0x01;								//配置寄存器位0（PRIM_RX）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为0，所以进入发送模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函    数：NRF24L01初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：使用前，需要调用此初始化函数
  */
void NRF24L01_Init(void)
{
	/*先调用底层的端口初始化*/
//	NRF24L01_GPIO_Init();

//	/*初始化硬件SPI1*/
//	NRF24L01_SPI_Init();
	
	/*初始化配置一系列寄存器，寄存器值的意义需参考手册中的寄存器描述*/
	/*以下配置通信双方必须保持一致，否则无法进行通信*/
	NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08);		//配置寄存器，不屏蔽中断，使能CRC，CRC为1字节，PWR_UP = 0，PRIM_RX = 0
	NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3F);		//使能自动应答，开启接收通道0~通道5的自动应答
	NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01);	//使能接收通道，只开启接收通道0
	NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03);		//设置地址宽度，地址宽度为5字节
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x03);	//设置自动重传，间隔250us，重传3次
	NRF24L01_WriteReg(NRF24L01_RF_CH, 0x02);		//射频通道，频率为(2400 + 2)MHz = 2.402GHz
	NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x0E);		//射频设置，通信速率为2Mbps，发射功率为0dBm

	
	/*接收通道0的数据包宽度，设置为宏定义NRF24L01_RX_PACKET_WIDTH指定的值*/
	NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_RX_PACKET_WIDTH);
	
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);

	/*发送地址，须与接收通道0地址一致，否则ACK包会发到错误地址，导致通信不稳定*/
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_RxAddress, 5);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*清空Rx FIFO的所有数据*/
	NRF24L01_FlushRx();
	
	/*给状态寄存器的位4（MAX_RT）、位5（TX_DS）和位6（RX_DR）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	
	/*初始化配置完成，芯片默认进入接收模式*/
	NRF24L01_Rx();
}

/**
  * 函    数：NRF24L01发送数据包
  * 参    数：无
  * 返 回 值：发送标志位，方便用户了解发送状态
  * 			1：发送成功，无错误
  * 			2：达到了最大重发次数仍未收到应答，可能是收发双方配置不一致、接收方不存在、接收FIFO已满或者多个发送数据包碰撞
  * 			3：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			4：发送超时，可能是设备未初始化、断路、短路或者引脚配置不正确
  * 说    明：调用此函数前，直接修改全局数组NRF24L01_TxAddress和NRF24L01_TxPacket来设置发送的地址和数据
  */
uint8_t NRF24L01_Send(void)
{
	uint8_t Status;
	uint8_t SendFlag;
	uint32_t Timeout;
	
	/*发送地址，设置为全局数组NRF24L01_TxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5);
	
	/*接收通道0地址，此处必须也设置为发送地址，用于接收应答*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TxAddress, 5);
	
	/*写发送有效载荷，写入全局数组NRF24L01_TxPacket指定的数据，数据宽度为NRF24L01_TX_PACKET_WIDTH*/
	NRF24L01_WriteTxPayload(NRF24L01_TxPacket, NRF24L01_TX_PACKET_WIDTH);
	
	/*发送的地址和有效载荷写入完成，进入发送模式，开始发送数据*/
	NRF24L01_Tx();
	
	/*指定超时时间，即循环读取状态寄存器的次数，具体值可以实测确定*/
	Timeout = 20000;
	
	/*循环读取状态寄存器*/
	while (1)
	{
		/*读取状态寄存器，保存至Status变量*/
		Status = NRF24L01_ReadStatus();
		
		/*超时计次*/
		Timeout --;
		if (Timeout == 0)			//如果计次减至0
		{
			SendFlag = 4;			//发送超时，置标志位为4
			//发送出错，不重新初始化，避免长时间阻塞接收
			break;					//跳出循环
		}
		
		/*根据状态寄存器的值，判断发送状态*/
		if ((Status & 0x30) == 0x30)		//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
		{
			SendFlag = 3;			//状态寄存器的值不合法，置标志位为3
			//发送出错，不重新初始化，避免长时间阻塞接收
			break;					//跳出循环
		}
		else if ((Status & 0x10) == 0x10)	//状态寄存器位4（MAX_RT）为1
		{
			SendFlag = 2;			//达到了最大重发次数仍未收到应答，置标志位为2
			//发送出错，不重新初始化，避免长时间阻塞接收
			break;					//跳出循环
		}
		else if ((Status & 0x20) == 0x20)	//状态寄存器位5（TX_DS）为1
		{
			SendFlag = 1;			//发送成功，无错误，置标志位为1
			break;					//跳出循环
		}
	}
	
	/*给状态寄存器的位4（MAX_RT）和位5（TX_DS）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*发送完成后，恢复接收通道0原来的地址*/
	/*如果发送地址和接收通道0地址设置相同，则可不执行这一句*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*发送完成，芯片恢复为接收模式*/
	NRF24L01_Rx();
		
	/*返回发送标志位*/
	return SendFlag;
}

/**
  * 函    数：NRF24L01接收数据包
  * 参    数：无
  * 返 回 值：接收标志位，方便用户了解接收状态
  * 			0：未接收到数据包
  * 			1：成功接收到一个数据包
  * 			2：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			3：设备仍处于掉电模式，可能是设备未初始化、曾经断电过、断路、短路或者引脚配置不正确
  * 说    明：如果收到了数据包，则可直接从全局数组NRF24L01_RxPacket取数据
  */
uint8_t NRF24L01_Receive(void)
{
	uint8_t Status, Config;
	uint8_t ReceiveFlag;
	
	/*读取状态寄存器，保存至Status变量*/
	Status = NRF24L01_ReadStatus();
	
	/*读取配置寄存器，保存至Config变量*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	/*根据配置寄存器和状态寄存器的值，判断接收状态*/
	if ((Config & 0x02) == 0x00)		//配置寄存器位1（PWR_UP）为0
	{
		ReceiveFlag = 3;				//设备仍处于掉电模式，置标志位为3
		/* 软恢复：仅重新上电并进入接收模式，不完整重新初始化，避免掉电导致通讯中断 */
		NRF24L01_Rx();					//设置 PWR_UP=1, PRIM_RX=1, CE=1
	}
	else if ((Status & 0x30) == 0x30)	//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
	{
		ReceiveFlag = 2;				//状态寄存器的值不合法，置标志位为2
		/* 软恢复：清除所有中断标志并重新进入接收模式 */
		NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
		NRF24L01_Rx();
	}
	else if ((Status & 0x40) == 0x40)	//状态寄存器位6（RX_DR）为1
	{
		ReceiveFlag = 1;				//接收到数据，置标志位为1
		
		/*读接收有效载荷，存放在全局数组NRF24L01_RxPacket中，数据宽度为NRF24L01_RX_PACKET_WIDTH*/
		NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
		
		/*给状态寄存器的位6（RX_DR）写1，清标志位*/
		NRF24L01_WriteReg(NRF24L01_STATUS, 0x40);

		/*清空Rx FIFO的所有数据*/
		NRF24L01_FlushRx();
	}
	else
	{
		ReceiveFlag = 0;				//未接收到数据，置标志位为0
	}
	
	/*返回接收标志位*/
	return ReceiveFlag;
}

/**
  * 函    数：NRF24L01更新接收地址
  * 参    数：无
  * 返 回 值：无
  * 说    明：如果想在运行时动态修改接收地址，则可先向全局数组NRF24L01_RxAddress写入修改的地址
  * 		  然后再调用此函数，使修改的接收地址生效
  */
void NRF24L01_UpdateRxAddress(void)
{
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
}


/**
  * 函    数：NRF24L01写入ACK应答载荷
  * 参    数：DataArray 要写入的载荷数据数组
  * 参    数：Count 写入的数据长度（1~32字节）
  * 返 回 值：无
  * 说    明：写入的数据将在下一次接收到有效数据包时随ACK应答发送给发送方
  */
void NRF24L01_WriteAckPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送指令码，W_ACK_PAYLOAD | 通道号0*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_ACK_PAYLOAD | 0);
	
	/*循环发送载荷数据*/
	for (i = 0; i < Count; i ++)
	{
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/*********************功能函数*/


