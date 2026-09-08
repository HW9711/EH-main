//uart4.c

#include "main.h"
#include "uart4.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART4_TimeoutComp   3 /* 旧版等待次数；当前对应判断已注释掉，改此值不会改变脚踏接收等待时间。 */

static uint8_t Uart4_Flag_Last = 0;
static uint16_t Uart4_RecvWaitTimeCnt = 0;
static uint8_t Uart4_DMABuf[UART4_MAX_PACKET_SIZE] = { 0 };

static void Uart4_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_4, Uart4_DMABuf, UART4_MAX_PACKET_SIZE);
}

/*
 * 函数功能：设置脚踏 UART4 波特率；失败时交给 Error_Handler 处理。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart4_Configuration(uint16_t baud)
{
  /* 初始化失败后不能继续用脚踏串口收发。 */
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
  Uart4_DmaInit();
}

void Uart4_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_4, pData, Length, 100);
	
}

/*
 * 函数功能：本次检查没有新字节时，取出 UART4 当前数据并重新接收。
 * 输入参数：data 指向至少 UART4_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；仍有新字节到达或缓存为空时返回 0。
 */
uint16_t Uart4_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart4_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_4);

  /* 又收到脚踏数据时先不取出，等下一次检查接收长度是否停止变化。 */
  if (RemainLen != Uart4_Flag_Last)
  {
    Uart4_RecvWaitTimeCnt = 0;
    Uart4_Flag_Last = RemainLen;
  }
  else
  {
    //if (Uart4_RecvWaitTimeCnt >= UART4_TimeoutComp)
      /* 至少收到 1 字节才复制；脚踏任务还要检查回包是否完整有效。 */
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








