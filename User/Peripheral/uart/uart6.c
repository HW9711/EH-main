//uart6.c

#include "main.h"
#include "uart6.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART6_TimeoutComp   3

static uint8_t Uart6_Flag_Last = 0;
static uint16_t Uart6_RecvWaitTimeCnt = 0;
static uint8_t Uart6_DMABuf[UART6_MAX_PACKET_SIZE] = { 0 };

static void Uart6_DMAConfiguration(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_6, Uart6_DMABuf, UART6_MAX_PACKET_SIZE);
}

void Uart6_Configuration(uint16_t baud)
{
  if (Bsp_UartInit(BSP_UART_PORT_6, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart6_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_6);
  memset(Uart6_DMABuf, 0, UART6_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_6, Uart6_DMABuf, UART6_MAX_PACKET_SIZE);
  Uart6_RecvWaitTimeCnt = 0;
  Uart6_Flag_Last = UART6_MAX_PACKET_SIZE;

}

void Uart6_Init(void)
{
  Uart6_DMAConfiguration();
	
}

void Uart6_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_6, pData, Length, 3);
	
}

uint16_t Uart6_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart6_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_6);

  if (RemainLen != Uart6_Flag_Last)
  {
    Uart6_RecvWaitTimeCnt = 0;
    Uart6_Flag_Last = RemainLen;
  }
  else
  {
    if (Uart6_RecvWaitTimeCnt >= UART6_TimeoutComp)
    {
      if (RemainLen < UART6_MAX_PACKET_SIZE)
      {
        rlen = (UART6_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart6_DMABuf, data, rlen);

        Uart6_DMAReset();
      }

      Uart6_RecvWaitTimeCnt = 0;
	}
  }

  return rlen;
}

void Uart6_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_6);
}








