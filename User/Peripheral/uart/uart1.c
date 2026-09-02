//uart1.c

#include "main.h"
#include "uart1.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART1_TimeoutComp 3

static uint8_t Uart1_Flag_Last = 0;
static uint16_t Uart1_RecvWaitTimeCnt = 0;
static uint8_t Uart1_DMABuf[UART1_MAX_PACKET_SIZE] = { 0 };

static void Uart1_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_1, Uart1_DMABuf, UART1_MAX_PACKET_SIZE);
}

void Uart1_Configuration(uint16_t baud)
{
  /* UART1 初始化失败时进入统一故障处理，避免外设保持半配置状态继续收发。 */
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
 * 函数功能：阻塞发送一帧 UART1 手柄驱动数据，并向调用方返回底层 HAL 发送结果。
 * 输入参数：pData 指向待发送数据；Length 为发送字节数。
 * 返回参数：HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT，供上层在零速帧失败时禁止继续发送非零启动帧。
 */
HAL_StatusTypeDef Uart1_SendPacket(uint8_t *pData, uint16_t Length)
{
  return Bsp_UartTransmit(BSP_UART_PORT_1, pData, Length, 100); /* 直接透传 UART1 阻塞发送结果，供启动安全门禁使用。 */
//	HAL_Delay(20);
}

/**
 * @brief 通过DMA方式从UART1接收数据并查看接收到的数据长度
 * @param data 用于存储接收数据的缓冲区指针
 * @return uint16_t 实际接收到的数据长度
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

  /* DMA 剩余数发生变化说明字节仍在到达，重置静默计时以免拆开同一帧。 */
  if (RemainLen != Uart1_Flag_Last)
  {
    // 如果发生变化，重置等待计数器并更新标志
    Uart1_RecvWaitTimeCnt = 0;
    Uart1_Flag_Last = RemainLen;
  }
  else
  {
    /* DMA 剩余数连续不变达到门限后，才把当前缓存判定为一帧完整数据。 */
    if (Uart1_RecvWaitTimeCnt >= UART1_TimeoutComp)
    {
      /* DMA 至少收到一个字节时才复制，空缓存不提交给上层协议解析。 */
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








