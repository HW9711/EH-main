//uart2.c

#include "main.h"
#include "uart2.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART2_TimeoutComp   3

static uint8_t Uart2_Flag_Last = 0;
static uint16_t Uart2_RecvWaitTimeCnt = 0;
static uint8_t Uart2_DMABuf[UART2_MAX_PACKET_SIZE] = { 0 };

extern UART_HandleTypeDef huart2;

static void Uart2_DMAConfiguration(void)
{
//	Delay_ms(300);

  HAL_UART_Receive_DMA(&huart2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
}

void Uart2_Configuration(uint16_t baud)
{
  huart2.Init.BaudRate = baud;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart2_DMAReset(void)
{

  HAL_UART_DMAStop(&huart2);
  memset(Uart2_DMABuf, 0, UART2_MAX_PACKET_SIZE);
  HAL_UART_Receive_DMA(&huart2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
  Uart2_RecvWaitTimeCnt = 0;
  Uart2_Flag_Last = UART2_MAX_PACKET_SIZE;

}

void Uart2_Init(void)
{
  Uart2_DMAConfiguration();
}

void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
  HAL_UART_Transmit(&huart2, pData, Length, 100);
}

uint16_t Uart2_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart2_RecvWaitTimeCnt++;
  RemainLen = __HAL_DMA_GET_COUNTER(huart2.hdmarx);

  if (RemainLen != Uart2_Flag_Last)
  {
    Uart2_RecvWaitTimeCnt = 0;
    Uart2_Flag_Last = RemainLen;
  }
  else
  {
    if (Uart2_RecvWaitTimeCnt >= UART2_TimeoutComp)
    {
      if (RemainLen < UART2_MAX_PACKET_SIZE)
      {
        rlen = (UART2_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart2_DMABuf, data, rlen);

        Uart2_DMAReset();
      }

      Uart2_RecvWaitTimeCnt = 0;
	  }
  }

  return rlen;
}

void Uart2_DeInit(void)
{
  HAL_UART_DeInit(&huart2);
}








