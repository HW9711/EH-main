//uart3.c

#include "main.h"
#include "uart3.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART3_TimeoutComp   3

static uint8_t Uart3_Flag_Last = 0;
static uint16_t Uart3_RecvWaitTimeCnt = 0;
static uint8_t Uart3_DMABuf[UART3_MAX_PACKET_SIZE] = { 0 };

extern UART_HandleTypeDef huart3;

static void Uart3_DMAConfiguration(void)
{
//	Delay_ms(300);

  HAL_UART_Receive_DMA(&huart3, Uart3_DMABuf, UART3_MAX_PACKET_SIZE);
}

void Uart3_Configuration(uint16_t baud)
{
  huart3.Init.BaudRate = baud;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart3_DMAReset(void)
{

  HAL_UART_DMAStop(&huart3);
  memset(Uart3_DMABuf, 0, UART3_MAX_PACKET_SIZE);
  HAL_UART_Receive_DMA(&huart3, Uart3_DMABuf, UART3_MAX_PACKET_SIZE);
  Uart3_RecvWaitTimeCnt = 0;
  Uart3_Flag_Last = UART3_MAX_PACKET_SIZE;

}

void Uart3_Init(void)
{
  Uart3_DMAConfiguration();
}

void Uart3_SendPacket(uint8_t *pData, uint16_t Length)
{
  HAL_UART_Transmit(&huart3, pData, Length, 100);
}

uint16_t Uart3_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  //Uart3_RecvWaitTimeCnt++;
  RemainLen = __HAL_DMA_GET_COUNTER(huart3.hdmarx);

//  if (RemainLen != Uart3_Flag_Last)
//  {
//    Uart3_RecvWaitTimeCnt = 0;
//    Uart3_Flag_Last = RemainLen;
//  }
 // else
 // {
//    if (Uart3_RecvWaitTimeCnt >= UART3_TimeoutComp)
//    {
      if (RemainLen < UART3_MAX_PACKET_SIZE)
      {
        rlen = (UART3_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart3_DMABuf, data, rlen);

        Uart3_DMAReset();
      }

      Uart3_RecvWaitTimeCnt = 0;
	 // }
  
 return rlen;
}

void Uart3_DeInit(void)
{
  HAL_UART_DeInit(&huart3);
}








