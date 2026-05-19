//uart4.c

#include "main.h"
#include "uart4.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART4_TimeoutComp   3

static uint8_t Uart4_Flag_Last = 0;
static uint16_t Uart4_RecvWaitTimeCnt = 0;
static uint8_t Uart4_DMABuf[UART4_MAX_PACKET_SIZE] = { 0 };

static void Uart4_DMAConfiguration(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_4, Uart4_DMABuf, UART4_MAX_PACKET_SIZE);
}

void Uart4_Configuration(uint16_t baud)
{
  if (Bsp_UartInit(BSP_UART_PORT_4, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart4_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_4);
  memset(Uart4_DMABuf, 0, UART4_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_4, Uart4_DMABuf, UART4_MAX_PACKET_SIZE);
  Uart4_RecvWaitTimeCnt = 0;
  Uart4_Flag_Last = UART4_MAX_PACKET_SIZE;

}

void Uart4_Init(void)
{
  Uart4_DMAConfiguration();
}

void Uart4_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_4, pData, Length, 100);
	
}

uint16_t Uart4_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart4_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_4);

  if (RemainLen != Uart4_Flag_Last)
  {
    Uart4_RecvWaitTimeCnt = 0;
    Uart4_Flag_Last = RemainLen;
  }
  else
  {
    //if (Uart4_RecvWaitTimeCnt >= UART4_TimeoutComp)
      if (RemainLen < UART4_MAX_PACKET_SIZE)
      {
        rlen = (UART4_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart4_DMABuf, data, rlen);

        Uart4_DMAReset();
      }

      Uart4_RecvWaitTimeCnt = 0;
  }

  return rlen;
}

void Uart4_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_4);
}








