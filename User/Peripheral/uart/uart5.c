//uart5.c

#include "main.h"
#include "uart5.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"
//#include "data.h"

#include <string.h>

#define	UART5_TimeoutComp   3

static uint8_t Uart5_Flag_Last = 0;
static uint16_t Uart5_RecvWaitTimeCnt = 0;
static uint8_t Uart5_DMABuf[UART5_MAX_PACKET_SIZE] = { 0 };

static void Uart5_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_5, Uart5_DMABuf, UART5_MAX_PACKET_SIZE);
}

void Uart5_Configuration(uint16_t baud)
{
  /* UART5 初始化失败时进入统一故障处理，避免逻辑 A 泵出口保持半配置状态。 */
  if (Bsp_UartInit(BSP_UART_PORT_5, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart5_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_5);
  memset(Uart5_DMABuf, 0, UART5_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_5, Uart5_DMABuf, UART5_MAX_PACKET_SIZE);
  Uart5_RecvWaitTimeCnt = 0;
  Uart5_Flag_Last = UART5_MAX_PACKET_SIZE;

}

void Uart5_Init(void)
{
  Uart5_DmaInit();
}

void Uart5_SendPacket(uint8_t *pData, uint16_t Length)
{
  /* UART5 泵控制帧只发给步进驱动，不再镜像到 UART10 打印测试信息，避免串口输出干扰泵控制节拍。 */
  Bsp_UartTransmit(BSP_UART_PORT_5, pData, Length, 100);
}

uint16_t Uart5_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart5_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_5);

  /* DMA 剩余数变化说明数据仍在到达，重新开始帧间静默计时。 */
  if (RemainLen != Uart5_Flag_Last)
  {
	  Uart5_RecvWaitTimeCnt = 0;
	  Uart5_Flag_Last = RemainLen;
  }
  else
  {
	  /* 接收长度连续不变达到门限后，才把当前缓存判定为完整帧。 */
	  if (Uart5_RecvWaitTimeCnt >= UART5_TimeoutComp)
	  {
	    /* DMA 至少接收一个字节时才复制，空缓存不触发业务处理。 */
	    if (RemainLen < UART5_MAX_PACKET_SIZE)
	    {
	      rlen = (UART5_MAX_PACKET_SIZE - RemainLen);

	      Common_CopyData(Uart5_DMABuf, data, rlen);
	      /* 驱动返回帧只保留给调用方读取，不再从 UART5 层转发到 UART10 输出测试文本。 */

	      Uart5_DMAReset();
	    }

	    Uart5_RecvWaitTimeCnt = 0;
	  }
  }

  return rlen;
}

void Uart5_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_5);
}
