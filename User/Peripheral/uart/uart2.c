//uart2.c

#include "main.h"
#include "uart2.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART2_TimeoutComp   3

static uint8_t Uart2_Flag_Last = 0;
static uint16_t Uart2_RecvWaitTimeCnt = 0;
static uint8_t Uart2_DMABuf[UART2_MAX_PACKET_SIZE] = { 0 };

static void Uart2_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
}

void Uart2_Configuration(uint16_t baud)
{
  /* UART2 初始化失败时进入统一故障处理，避免外控通信口处于不可预期状态。 */
  if (Bsp_UartInit(BSP_UART_PORT_2, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart2_DMAReset(void)
{
  Bsp_UartDmaStop(BSP_UART_PORT_2);                            /* 取走一包数据后先停止 USART2 RX DMA，防止旧 DMA 状态继续写入缓存。 */
  memset(Uart2_DMABuf, 0, UART2_MAX_PACKET_SIZE);              /* 清空接收缓存，避免上一包残留影响下一次解析。 */
  Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE); /* 重新打开 USART2 DMA 接收，恢复下行数据采集。 */
  Uart2_RecvWaitTimeCnt = 0;                                   /* DMA 重启后重新累计静默时间。 */
  Uart2_Flag_Last = UART2_MAX_PACKET_SIZE;                     /* 复位剩余长度快照，让下一包从空缓存状态开始判断。 */
}

void Uart2_Init(void)
{
  Uart2_DmaInit();
}

void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
  Bsp_UartTransmit(BSP_UART_PORT_2, pData, Length, 100);       /* 外部通信帧固定从 USART2 发出，保持与主控板外部通信接口一致。 */
}

uint16_t Uart2_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart2_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_2);

  /* DMA 剩余数发生变化说明本帧仍在接收，重新开始帧间静默计时。 */
  if (RemainLen != Uart2_Flag_Last)
  {
    Uart2_RecvWaitTimeCnt = 0;
    Uart2_Flag_Last = RemainLen;
  }
  else
  {
    /* 接收长度连续不变达到门限后，才把缓存交给外控协议层。 */
    if (Uart2_RecvWaitTimeCnt >= UART2_TimeoutComp)
    {
      /* DMA 至少接收一个字节时才复制，空帧不触发上层解析。 */
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
  Bsp_UartDeInit(BSP_UART_PORT_2);
}
