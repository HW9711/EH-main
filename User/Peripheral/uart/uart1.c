//uart1.c

#include "main.h"
#include "uart1.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART1_TimeoutComp 3

static uint8_t Uart1_Flag_Last = 0;
static uint16_t Uart1_RecvWaitTimeCnt = 0;
static uint8_t Uart1_DMABuf[UART1_MAX_PACKET_SIZE] = { 0 };

static void Uart1_DMAConfiguration(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_1, Uart1_DMABuf, UART1_MAX_PACKET_SIZE);
}

void Uart1_Configuration(uint16_t baud)
{
  if (Bsp_UartInit(BSP_UART_PORT_1, baud) != HAL_OK)
  {
	  Error_Handler();
  }
}

static void Uart1_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_1);
  memset(Uart1_DMABuf, 0, UART1_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_1, Uart1_DMABuf, UART1_MAX_PACKET_SIZE);
  Uart1_RecvWaitTimeCnt = 0;
  Uart1_Flag_Last = UART1_MAX_PACKET_SIZE;

}

void Uart1_Init(void)
{
  Uart1_DMAConfiguration();
}

void Uart1_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_1, pData, Length, 100);
//	HAL_Delay(20);
}

/**
 * @brief 通过DMA方式从UART1接收数据并查看接收到的数据长度
 * @param data 用于存储接收数据的缓冲区指针
 * @return uint16_t 实际接收到的数据长度
 */
uint16_t Uart1_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;  // DMA剩余未传输的数据长度
  uint16_t rlen = 0;       // 实际接收到的数据长度

  //------------------------------------------------------------------
  // 增加接收等待计数器，用于超时判断
  Uart1_RecvWaitTimeCnt++;
  // 获取DMA当前剩余计数器的值
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_1);

  // 判断剩余数据长度是否发生变化
  if (RemainLen != Uart1_Flag_Last)
  {
    // 如果发生变化，重置等待计数器并更新标志
    Uart1_RecvWaitTimeCnt = 0;
    Uart1_Flag_Last = RemainLen;
  }
  else
  {
    // 如果剩余长度未变化，检查是否超时
    if (Uart1_RecvWaitTimeCnt >= UART1_TimeoutComp)
    {
      // 检查剩余长度是否小于最大数据包大小
      if (RemainLen < UART1_MAX_PACKET_SIZE)
      {
        // 计算实际接收到的数据长度
        rlen = (UART1_MAX_PACKET_SIZE - RemainLen);

        // 将DMA缓冲区中的数据复制到输出缓冲区
		    Common_CopyData(Uart1_DMABuf, data, rlen);

        // 重置DMA接收状态
        Uart1_DMAReset();
      }

      // 重置等待计数器
      Uart1_RecvWaitTimeCnt = 0;
	  }
  }

  // 返回实际接收到的数据长度
  return rlen;
}

void Uart1_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_1);
}








