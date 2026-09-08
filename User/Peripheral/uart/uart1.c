//uart1.c

#include "main.h"
#include "uart1.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART1_TimeoutComp 3 /* 连续 3 次检查未收到新字节后取出数据；单位是调用次数，不是毫秒。 */

static uint8_t Uart1_Flag_Last = 0;
static uint16_t Uart1_RecvWaitTimeCnt = 0;
static uint8_t Uart1_DMABuf[UART1_MAX_PACKET_SIZE] = { 0 };

static void Uart1_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_1, Uart1_DMABuf, UART1_MAX_PACKET_SIZE);
}

/*
 * 函数功能：设置 UART1 波特率；初始化失败时交给 Error_Handler 处理。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart1_Configuration(uint16_t baud)
{
  /* 初始化失败后不能继续用这个串口收发。 */
  if (Bsp_UartInit(BSP_UART_PORT_1, baud) != HAL_OK)
  {
	  Error_Handler();
  }
}

static void Uart1_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_1);
  memset(Uart1_DMABuf, 0, UART1_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_1, Uart1_DMABuf, UART1_MAX_PACKET_SIZE);
  Uart1_RecvWaitTimeCnt = 0;
  Uart1_Flag_Last = UART1_MAX_PACKET_SIZE;

}

void Uart1_Init(void)
{
  Uart1_DmaInit();
}

/*
 * 函数功能：等待 UART1 手柄驱动数据发送结束，再返回发送结果。
 * 输入参数：pData 指向待发送数据；Length 为发送字节数。
 * 返回参数：HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT；零速命令发送失败时，上层不能继续发送启动命令。
 */
HAL_StatusTypeDef Uart1_SendPacket(uint8_t *pData, uint16_t Length)
{
  return Bsp_UartTransmit(BSP_UART_PORT_1, pData, Length, 100); /* 最多等待 100ms，返回发送结果供上层判断能否启动。 */
//	HAL_Delay(20);
}

/*
 * 函数功能：连续多次未收到新字节后，取出 UART1 DMA 数据并重新开始接收。
 * 输入参数：data 指向至少 UART1_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；仍在接收或缓存为空时返回 0。
 */
uint16_t Uart1_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;  // DMA剩余未传输的数据长度
  uint16_t rlen = 0;       // 实际接收到的数据长度

  //------------------------------------------------------------------
  // 增加接收等待计数器，用于超时判断
  Uart1_RecvWaitTimeCnt++;
  // 获取DMA当前剩余计数器的值
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_1);

  /* DMA 剩余空间变化说明又收到字节，重新等待，避免把一帧分成两次取出。 */
  if (RemainLen != Uart1_Flag_Last)
  {
    // 如果发生变化，重置等待计数器并更新标志
    Uart1_RecvWaitTimeCnt = 0;
    Uart1_Flag_Last = RemainLen;
  }
  else
  {
    /* 连续 3 次检查没有新字节后取出缓存；是否为完整有效帧仍由上层校验。 */
    if (Uart1_RecvWaitTimeCnt >= UART1_TimeoutComp)
    {
      /* 至少收到 1 字节才复制，空缓存不交给上层处理。 */
      if (RemainLen < UART1_MAX_PACKET_SIZE)
      {
        // 计算实际接收到的数据长度
        rlen = (UART1_MAX_PACKET_SIZE - RemainLen);

        // 将DMA缓冲区中的数据复制到输出缓冲区
		    Common_CopyData(Uart1_DMABuf, data, rlen);

        // 重置DMA接收状态
        Uart1_DMAReset();
      }

      // 重置等待计数器
      Uart1_RecvWaitTimeCnt = 0;
	  }
  }

  // 返回实际接收到的数据长度
  return rlen;
}

void Uart1_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_1);
}








