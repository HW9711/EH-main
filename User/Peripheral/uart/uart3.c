//uart3.c

#include "main.h"
#include "uart3.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART3_TimeoutComp   3

static uint8_t Uart3_DMABuf[UART3_MAX_PACKET_SIZE] = { 0 };

static void Uart3_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_3, Uart3_DMABuf, UART3_MAX_PACKET_SIZE);
}

void Uart3_Configuration(uint16_t baud)
{
  /* UART3 初始化失败时进入统一故障处理，避免对应业务串口继续使用无效配置。 */
  if (Bsp_UartInit(BSP_UART_PORT_3, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart3_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_3);
  memset(Uart3_DMABuf, 0, UART3_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_3, Uart3_DMABuf, UART3_MAX_PACKET_SIZE);
}

void Uart3_Init(void)
{
  Uart3_DmaInit();
}

void Uart3_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_3, pData, Length, 100);
}

/*
 * 函数功能：清空 UART3 当前 DMA 接收缓存并重新启动接收。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart3_ClearRecvData(void)
{
  Uart3_DMAReset(); /* RFID 发起新读命令前调用，避免把上一轮残留回包当成本轮刀具标签。 */
}

uint16_t Uart3_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  //Uart3_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_3);

//  if (RemainLen != Uart3_Flag_Last)
//  {
//    Uart3_RecvWaitTimeCnt = 0;
//    Uart3_Flag_Last = RemainLen;
//  }
 // else
 // {
//    if (Uart3_RecvWaitTimeCnt >= UART3_TimeoutComp)
//    {
      /* DMA 至少消耗一个字节时才复制数据，避免把空缓存当成有效帧。 */
      if (RemainLen < UART3_MAX_PACKET_SIZE)
      {
        rlen = (UART3_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart3_DMABuf, data, rlen);

        Uart3_DMAReset();
      }

	 // }
  
 return rlen;
}

void Uart3_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_3);
}








