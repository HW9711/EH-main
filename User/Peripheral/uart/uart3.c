//uart3.c

#include "main.h"
#include "uart3.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART3_TimeoutComp   3 /* 旧版等待次数；当前对应判断已注释掉，改此值不会改变 RFID 接收等待时间。 */

static uint8_t Uart3_DMABuf[UART3_MAX_PACKET_SIZE] = { 0 };

static void Uart3_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_3, Uart3_DMABuf, UART3_MAX_PACKET_SIZE);
}

/*
 * 函数功能：设置 UART3 RFID 串口波特率；失败时交给 Error_Handler 处理。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart3_Configuration(uint16_t baud)
{
  /* 初始化失败后不能继续用 RFID 串口收发。 */
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

/*
 * 函数功能：取出 UART3 当前已收到的字节，然后清缓存重新接收；本函数不等待完整帧。
 * 输入参数：data 指向至少 UART3_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；未收到字节时返回 0。
 */
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
      /* 至少收到 1 字节才复制；是否为完整 RFID 回包由上层检查。 */
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








