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
  if (Bsp_UartTransmit(BSP_UART_PORT_7, pData, Length, 100) != HAL_OK)
  {
    Bsp_UartAbort(BSP_UART_PORT_7);
    Bsp_UartDeInit(BSP_UART_PORT_7);
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

  if (RemainLen != Uart7_Flag_Last)
  {
	  Uart7_RecvWaitTimeCnt = 0;
	  Uart7_Flag_Last = RemainLen;
  }
  else
  {
	  if (Uart7_RecvWaitTimeCnt >= UART7_TimeoutComp)
	  {
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






