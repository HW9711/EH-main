//uart5.c

#include "main.h"
#include "uart5.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"
//#include "data.h"

#include <string.h>

#define	UART5_TimeoutComp   3

static uint8_t Uart5_Flag_Last = 0;
static uint16_t Uart5_RecvWaitTimeCnt = 0;
static uint8_t Uart5_DMABuf[UART5_MAX_PACKET_SIZE] = { 0 };

static void Uart5_DMAConfiguration(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_5, Uart5_DMABuf, UART5_MAX_PACKET_SIZE);
}

void Uart5_Configuration(uint16_t baud)
{
  if (Bsp_UartInit(BSP_UART_PORT_5, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart5_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_5);
  memset(Uart5_DMABuf, 0, UART5_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_5, Uart5_DMABuf, UART5_MAX_PACKET_SIZE);
  Uart5_RecvWaitTimeCnt = 0;
  Uart5_Flag_Last = UART5_MAX_PACKET_SIZE;

}

void Uart5_Init(void)
{
  Uart5_DMAConfiguration();
}

void Uart5_SendPacket(uint8_t *pData, uint16_t Length)
{
  /* UART5 泵控制帧只发给步进驱动，不再镜像到 UART10 打印测试信息，避免串口输出干扰泵控制节拍。 */
  Bsp_UartTransmit(BSP_UART_PORT_5, pData, Length, 100);
}

uint16_t Uart5_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart5_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_5);

  if (RemainLen != Uart5_Flag_Last)
  {
	  Uart5_RecvWaitTimeCnt = 0;
	  Uart5_Flag_Last = RemainLen;
  }
  else
  {
	  if (Uart5_RecvWaitTimeCnt >= UART5_TimeoutComp)
	  {
	    if (RemainLen < UART5_MAX_PACKET_SIZE)
	    {
	      rlen = (UART5_MAX_PACKET_SIZE - RemainLen);

	      Common_CopyData(Uart5_DMABuf, data, rlen);
	      /* 驱动返回帧只保留给调用方读取，不再从 UART5 层转发到 UART10 输出测试文本。 */

	      Uart5_DMAReset();
	    }

	    Uart5_RecvWaitTimeCnt = 0;
	  }
  }

  return rlen;
}

void Uart5_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_5);
}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static bool Uart5_Flag = true;
	static uint8_t dat[16] = { 0 };
	static uint8_t Uart5_Index = 0;

	uint8_t dat1[16] = { 0 };
	uint16_t crc = 0;

	if (huart->Instance == UART5)
	{
		//帧头
		if ((Uart5_Index == 0) && (Uart5_RxData[0] == 0xFE) && Uart5_Flag)
		{
			Common_Memset(0, dat, 16);
			Uart5_Flag = false;
		}

		if (!Uart5_Flag)
			dat[Uart5_Index++] = Uart5_RxData[0];
		else
			return ;

		//帧头判断
		if ((Uart5_Index == 2) && (dat[1] != 0xEF))
		{
			Uart5_Index = 0;
			Uart5_Flag = true;
		}

		if (Uart5_Index < 10)
			return ;

		Common_CopyData (dat, dat1, Uart5_Index);
		Uart5_Index = 0;
		Uart5_Flag = true;

		//JT_Connect_Flag = Connect;
		//实时采样的脚踏数据 FE EF    B6 C1       01        01     DATA_H DATA_L CRCH CRCL
		crc = Common_Crc16(dat1, 8);
		if (crc == ((dat1[8] << 8) + dat1[9]))
		{
		  if ((dat1[2] == 0xB6) && (dat1[3] == 0xC1))
			{
			  SysFootPedalData.FootPedalADValue = (dat1[6] << 8) + dat1[7];  //JT_ADC_Value = (Uart1_Temp_Buff[6] << 8) + Uart1_Temp_Buff[7];
			}
			//主控板发送命令给脚踏板，脚踏板回复的读取已存储的低值
			else if ((dat1[2] == 0xD0) && (dat1[3] == 0xB4) && (dat1[4] == 0xB5) && (dat1[5] == 0xCD))  //B5 CD 表示低
			{
			  SysFootPedalData.FootPedalMemoryLValue = (dat1[6]<<8) + dat1[7]; //JT_Memory_L_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
			}
			//主控板发送命令给脚踏板，脚踏板回复的读取已存储的高值
			else if ((dat1[2] == 0xD0) && (dat1[3] == 0xB4) && (dat1[4] == 0xB8) && (dat1[5] == 0xDF))  //B8 DF表示高
			{
			  SysFootPedalData.FootPedalMemoryHValue = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
			}
			//电机处于停止状态，响应按键
			else if ((SysRunData.MotorRun == MotorStop) && (dat1[2] == 0xBB) && (dat1[3] == 0xAA) && (dat1[4] == 0xCC) && (dat1[5] == 0xDD))
			{
			  switch (dat1[7])
				{
					case 0x03 : SysFootPedalData.FootPedalKeyValue = 1; break; //手柄切换 长按C键1.2s
					case 0x05 : SysFootPedalData.FootPedalKeyValue = 2; break; //模式调节 短按C键200ms
					case 0x0A : SysFootPedalData.FootPedalKeyValue = 3; break; //参数- 短按B键200ms
					case 0x0B : SysFootPedalData.FootPedalKeyValue = 4; break; //参数+ 短按A键200ms
					default : break;
				}
			}

      SysFootPedalData.FootPedalOffTimes = 0;  //JT_Offtimes = 0;
			SysFootPedalData.FootPedalConnectFlag = Connect;  //JT_Flag = Connect;
		}
	}
}
#endif





