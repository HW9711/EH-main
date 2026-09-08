//uart6.c

#include "main.h"
#include "uart6.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART6_TimeoutComp   3 /* 连续 3 次检查未收到新字节后取出数据；单位是调用次数，不是毫秒。 */

static uint8_t Uart6_Flag_Last = 0;
static uint16_t Uart6_RecvWaitTimeCnt = 0;
static uint8_t Uart6_DMABuf[UART6_MAX_PACKET_SIZE] = { 0 };

static void Uart6_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_6, Uart6_DMABuf, UART6_MAX_PACKET_SIZE);
}

/*
 * 函数功能：设置触控屏 UART6 波特率；失败时交给 Error_Handler 处理。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart6_Configuration(uint16_t baud)
{
  /* 初始化失败后不能继续用触控屏串口收发。 */
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
  Uart6_DmaInit();
	
}

void Uart6_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_6, pData, Length, 3);
	
}

/*
 * 函数功能：连续多次未收到新字节后，取出 UART6 DMA 数据并重新接收。
 * 输入参数：data 指向至少 UART6_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；仍在接收或缓存为空时返回 0。
 */
uint16_t Uart6_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart6_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_6);

  /* 又收到屏幕数据时重新等待，避免把一帧分成两次取出。 */
  if (RemainLen != Uart6_Flag_Last)
  {
    Uart6_RecvWaitTimeCnt = 0;
    Uart6_Flag_Last = RemainLen;
  }
  else
  {
    /* 连续 3 次检查没有新字节后取出缓存；完整性由屏幕协议检查。 */
    if (Uart6_RecvWaitTimeCnt >= UART6_TimeoutComp)
    {
      /* DMA 至少收到一个字节时才复制，空帧不触发屏幕按键处理。 */
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








