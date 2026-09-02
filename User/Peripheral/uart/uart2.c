//uart2.c

#include "main.h"
#include "uart2.h"
#include "bsp_uart.h"
#include "board.h"
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

/*
 * 函数功能：初始化外部通信 UART2 接收，并确保 V4.0 RS485 芯片处于接收状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart2_Init(void)
{
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 低电平关闭驱动并使能接收，避免初始化阶段占用 RS485 总线。 */
  Uart2_DmaInit(); /* 方向稳定为接收后再启动 DMA，保证外部设备下行帧可以立即进入缓存。 */
}

/*
 * 函数功能：切换 V4.0 RS485 芯片到发送状态，阻塞发送完整帧后恢复接收状态。
 * 输入参数：pData 为待发送数据缓冲区；Length 为本次发送的字节数。
 * 返回参数：无。
 */
void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_SET);   /* 高电平使能 DE 并关闭 /RE，避免发送数据被本机 DMA 回收。 */
  (void)Bsp_UartTransmit(BSP_UART_PORT_2, pData, Length, 100); /* 阻塞发送会等待 UART_TC；异常或超时返回后也继续释放总线。 */
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 最后停止位发送完成后立即恢复接收，允许外部设备应答。 */
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
