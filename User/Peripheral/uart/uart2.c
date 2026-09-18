//uart2.c

#include "main.h"
#include "uart2.h"
#include "diagnostic.h" /* 诊断开启时UART2共用非阻塞发送队列。 */
#include "bsp_uart.h"
#include "board.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART2_TimeoutComp   3 /* 连续 3 次检查未收到新字节后取出数据；单位是调用次数，不是毫秒。 */

static uint8_t Uart2_Flag_Last = 0;
static uint16_t Uart2_RecvWaitTimeCnt = 0;
static uint8_t Uart2_DMABuf[UART2_MAX_PACKET_SIZE] = { 0 };

static void Uart2_DmaInit(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
}

/*
 * 函数功能：设置外部通信 UART2 波特率；失败时交给 Error_Handler 处理。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart2_Configuration(uint16_t baud)
{
  /* 初始化失败后不能继续用外部通信串口收发。 */
  if (Bsp_UartInit(BSP_UART_PORT_2, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 函数功能：清空 UART2 已取走的数据，并重新启动 DMA 接收。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart2_DMAReset(void)
{
  Bsp_UartDmaStop(BSP_UART_PORT_2);                            /* 清缓存前先停 DMA，避免清除时又有字节写进来。 */
  memset(Uart2_DMABuf, 0, UART2_MAX_PACKET_SIZE);              /* 清空接收缓存，避免上一包残留影响下一次解析。 */
  Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE); /* 重新接收外部设备发来的数据。 */
  Uart2_RecvWaitTimeCnt = 0;                                   /* 重新累计未收到新字节的检查次数。 */
  Uart2_Flag_Last = UART2_MAX_PACKET_SIZE;                     /* 记录缓存还没有数据，供下次比较接收长度。 */
}

/*
 * 函数功能：初始化外部通信 UART2 接收，并确保 V4.0 RS485 芯片处于接收状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart2_Init(void)
{
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 低电平关闭驱动并使能接收，避免初始化阶段占用 RS485 总线。 */
  Uart2_DmaInit(); /* 切到接收后再启动 DMA，接收外部设备发来的字节。 */
}

/*
 * 函数功能：发送外部通信帧；诊断关闭时阻塞发送，开启时复制到普通优先队列并由UART2 TC中断恢复接收。
 * 输入参数：pData 为待发送数据缓冲区；Length 为本次发送的字节数。
 * 返回参数：无。
 */
void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
#if (MAINCTRL_DIAG_ENABLE == 1U)
  Diagnostic_SendNormal(pData, Length); /* 普通帧优先排队，立即返回，不阻塞25ms泵任务。 */
#else
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_SET);   /* 高电平允许发送、关闭接收，避免自己收到刚发出的数据。 */
  (void)Bsp_UartTransmit(BSP_UART_PORT_2, pData, Length, 100); /* 正常时等待最后一位发完；失败或超过 100ms 后也返回。 */
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 不论发送成功还是失败，都恢复接收并释放 RS485 总线。 */
#endif
}

/*
 * 函数功能：诊断请求完整且 CRC 正确时立即取出；其它数据仍等连续三次未收到新字节后取出。
 * 输入参数：data 指向至少 UART2_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；仍在接收或缓存为空时返回 0。
 */
uint16_t Uart2_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart2_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_2);
#if (MAINCTRL_DIAG_ENABLE == 1U)
  if ((RemainLen < UART2_MAX_PACKET_SIZE) &&
      (Diagnostic_IsCompleteRequest(Uart2_DMABuf, (uint16_t)(UART2_MAX_PACKET_SIZE - RemainLen)) != 0U))
  {
    rlen = (uint16_t)(UART2_MAX_PACKET_SIZE - RemainLen); /* 诊断请求已收齐，不再额外等待约 30ms。 */
    Common_CopyData(Uart2_DMABuf, data, rlen); /* 复制已校验的请求，交给原外部通信解析器执行一次。 */
    Uart2_DMAReset(); /* 原请求已取走，立即准备接收下一条诊断请求。 */
    return rlen; /* 本周期即可处理并应答，普通帧仍走下面的原等待流程。 */
  }
#endif

  /* 又收到字节时重新等待，不急着把当前数据交给上层。 */
  if (RemainLen != Uart2_Flag_Last)
  {
    Uart2_RecvWaitTimeCnt = 0;
    Uart2_Flag_Last = RemainLen;
  }
  else
  {
    /* 连续 3 次检查没有新字节后取出缓存；完整性由外部通信协议检查。 */
    if (Uart2_RecvWaitTimeCnt >= UART2_TimeoutComp)
    {
      /* 至少收到 1 字节才复制，空缓存不交给上层处理。 */
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
