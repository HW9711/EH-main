//uart1.c

#include "main.h"
#include "uart1.h"
#include "diagnostic.h" /* 从已复制的接收数据记录原始字节，不再次读取DMA。 */
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART1_TimeoutComp 3 /* 连续 3 次检查未收到新字节后取出数据；单位是调用次数，不是毫秒。 */

static uint8_t Uart1_Flag_Last = 0;
static uint16_t Uart1_RecvWaitTimeCnt = 0;
static uint8_t Uart1_DMABuf[UART1_MAX_PACKET_SIZE] = { 0 };
static uint32_t s_uart1_rx_window_start_ms; /* 本轮DMA启动前的HAL时钟，供上层排除STOP前已进入缓存的反馈。 */

/*
 * 函数功能：启动首轮UART1 DMA接收，并为本轮反馈保存最早可能的接收时刻。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart1_DmaInit(void)
{
//	Delay_ms(300);

  s_uart1_rx_window_start_ms = HAL_GetTick(); /* 先取时刻再开DMA，确保这是本批字节可能到达的最早时刻。 */
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

/*
 * 函数功能：取走上一批UART1数据后重开DMA接收，分开保存新旧批次时刻。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart1_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_1);
  memset(Uart1_DMABuf, 0, UART1_MAX_PACKET_SIZE);
  s_uart1_rx_window_start_ms = HAL_GetTick(); /* 先取时刻再开DMA，确保这是本批字节可能到达的最早时刻。 */
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
 * 输入参数：data 指向至少 UART1_MAX_PACKET_SIZE 字节的输出缓存；rx_window_start_ms 指向必须提供的本批DMA接收起点输出变量。
 * 返回参数：复制的字节数；仍在接收或缓存为空时返回 0。
 */
uint16_t Uart1_DMARecvDataPeek(uint8_t *data, uint32_t *rx_window_start_ms)
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
        *rx_window_start_ms = s_uart1_rx_window_start_ms; /* 在DMA复位前返回旧批次起点，避免用下一批时刻证明旧包在STOP之后收到。 */

        // 将DMA缓冲区中的数据复制到输出缓冲区
		    Common_CopyData(Uart1_DMABuf, data, rlen);
        DIAG_WIRE(1U, 0U, 0x100U, data, rlen); /* 原始字节尚未经过上层CRC检查，不能当成有效反馈。 */

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








