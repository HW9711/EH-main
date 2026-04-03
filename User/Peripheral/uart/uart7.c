//uart7.c

#include "main.h"
#include "uart7.h"
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

extern UART_HandleTypeDef huart7;

void Uart7_Configuration(uint16_t baud)
{
  huart7.Init.BaudRate = baud;

  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart7_DMAReset(void)
{

  HAL_UART_DMAStop(&huart7);
  memset(Uart7_DMABuf, 0, UART7_MAX_PACKET_SIZE);
  HAL_UART_Receive_DMA(&huart7, Uart7_DMABuf, UART7_MAX_PACKET_SIZE);
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
  if (HAL_UART_Transmit(&huart7, pData, Length, 100) != HAL_OK)
  {
    HAL_UART_Abort(&huart7);
    HAL_UART_DeInit(&huart7);
    if (HAL_UART_Init(&huart7) == HAL_OK)
    {
      return (HAL_UART_Transmit(&huart7, pData, Length, 100) == HAL_OK) ? 1U : 0U;
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
  RemainLen = __HAL_DMA_GET_COUNTER(huart7.hdmarx);

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
  HAL_UART_DeInit(&huart7);
}






