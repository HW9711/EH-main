//uart7.c

#include "main.h"
#include "uart7.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"
//#include "data.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define	UART7_TimeoutComp   3

static uint8_t Uart7_Flag_Last = 0;
static uint16_t Uart7_RecvWaitTimeCnt = 0;
static uint8_t Uart7_DMABuf[UART7_MAX_PACKET_SIZE] = { 0 };
static uint32_t Uart7_BaudRate = 115200U;

void Uart7_Configuration(uint16_t baud)
{
  Uart7_BaudRate = baud;

  /* UART7 初始化失败时进入统一故障处理，避免泵驱动串口处于半配置状态。 */
  if (Bsp_UartInit(BSP_UART_PORT_7, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart7_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_7);
  memset(Uart7_DMABuf, 0, UART7_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_7, Uart7_DMABuf, UART7_MAX_PACKET_SIZE);
  Uart7_RecvWaitTimeCnt = 0;
  Uart7_Flag_Last = UART7_MAX_PACKET_SIZE;

}

void Uart7_Init(void)
{
  Uart7_RecvWaitTimeCnt = 0;
  Uart7_Flag_Last = UART7_MAX_PACKET_SIZE;
}

uint8_t Uart7_SendPacket(uint8_t *pData, uint16_t Length)
{
  /* 首次发送失败时复位 UART7 并重建波特率，给泵控制帧一次恢复发送机会。 */
  if (Bsp_UartTransmit(BSP_UART_PORT_7, pData, Length, 100) != HAL_OK)
  {
    Bsp_UartAbort(BSP_UART_PORT_7);
    Bsp_UartDeInit(BSP_UART_PORT_7);
    /* 串口重新初始化成功后才允许重发，避免继续使用未恢复的硬件。 */
    if (Bsp_UartInit(BSP_UART_PORT_7, Uart7_BaudRate) == HAL_OK)
    {
      return (Bsp_UartTransmit(BSP_UART_PORT_7, pData, Length, 100) == HAL_OK) ? 1U : 0U;
    }

    return 0U;
  }

  return 1U;
}

uint16_t Uart7_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart7_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_7);

  /* DMA 剩余数仍在变化表示字节持续到达，重新开始帧间静默计时。 */
  if (RemainLen != Uart7_Flag_Last)
  {
	  Uart7_RecvWaitTimeCnt = 0;
	  Uart7_Flag_Last = RemainLen;
  }
  else
  {
	  /* 剩余数连续多个周期不变时认为一帧接收结束，开始搬运本帧数据。 */
	  if (Uart7_RecvWaitTimeCnt >= UART7_TimeoutComp)
	  {
	    /* DMA 已消耗至少一个字节时才复制，空帧不触发解析和 DMA 重置。 */
	    if (RemainLen < UART7_MAX_PACKET_SIZE)
	    {
	      rlen = (UART7_MAX_PACKET_SIZE - RemainLen);

	      Common_CopyData(Uart7_DMABuf, data, rlen);

	      Uart7_DMAReset();
	    }

	    Uart7_RecvWaitTimeCnt = 0;
	  }
  }

  return rlen;
}

void Uart7_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_7);
}






