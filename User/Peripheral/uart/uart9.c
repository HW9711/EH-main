//uart9.c

#include "main.h"
#include "uart9.h"
#include "bsp_uart.h"
#include "common.h"

#include <string.h>
#include <stdio.h>

static uint8_t Uart9_DMABuf[UART9_MAX_PACKET_SIZE] = { 0U }; /* UART9 DMA 接收缓存，双串口模式下只接 B 通道 RFID 回包。 */

/*
 * 函数功能：启动 UART9 的 DMA 接收。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart9_DmaInit(void)
{
  (void)Bsp_UartReceiveDma(BSP_UART_PORT_9, Uart9_DMABuf, UART9_MAX_PACKET_SIZE); /* 让 UART9 后续收到的 RFID 字节直接进入本地 DMA 缓存。 */
}

/*
 * 函数功能：停止并重新启动 UART9 DMA 接收，清掉上一轮 RFID 残留数据。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart9_DMAReset(void)
{
  (void)Bsp_UartDmaStop(BSP_UART_PORT_9); /* 先停止 UART9 DMA，避免清缓存时硬件继续写入旧回包。 */
  memset(Uart9_DMABuf, 0, UART9_MAX_PACKET_SIZE); /* 清空 B 通道 RFID 接收缓存，防止旧标签回包被下一次请求复用。 */
  (void)Bsp_UartReceiveDma(BSP_UART_PORT_9, Uart9_DMABuf, UART9_MAX_PACKET_SIZE); /* 重新开启 UART9 DMA，等待下一次 B 通道 RFID 回包。 */
}

/*
 * 函数功能：初始化 UART9 应用层接收通道。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_Init(void)
{
  Uart9_DmaInit(); /* 系统启动后立即打开 B 通道 RFID DMA 接收，避免首包丢失。 */
}

/*
 * 函数功能：通过 UART9 发送一帧 RFID 命令。
 * 输入参数：pData 指向待发送命令缓冲区；Length 表示命令字节数。
 * 返回参数：无。
 */
void Uart9_SendPacket(uint8_t *pData, uint16_t Length)
{
  (void)Bsp_UartTransmit(BSP_UART_PORT_9, pData, Length, 100U); /* UART9 固定连 B 通道 RFID，发送超时保持与 UART3 一致。 */
}

/*
 * 函数功能：清空 UART9 当前 DMA 接收缓存并重新接收。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_ClearRecvData(void)
{
  Uart9_DMAReset(); /* 新 RFID 请求开始前调用，保证 B 通道只解析本轮回包。 */
}

/*
 * 函数功能：读取 UART9 DMA 缓存中已经收到的 RFID 字节。
 * 输入参数：data 指向调用方接收缓冲区。
 * 返回参数：本次取出的有效字节数，0 表示尚未收到回包。
 */
uint16_t Uart9_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0U; /* DMA 剩余计数，用于反推 UART9 已接收字节数。 */
  uint16_t rlen = 0U;      /* 返回给 RFID 任务的本轮有效数据长度。 */

  if (data == NULL)
  {
    return 0U; /* 调用方缓冲区无效时不访问 DMA 数据，避免异常写内存。 */
  }

  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_9); /* 读取 UART9 RX DMA 当前剩余搬运数量。 */
  if (RemainLen < UART9_MAX_PACKET_SIZE)
  {
    rlen = (uint16_t)(UART9_MAX_PACKET_SIZE - RemainLen); /* DMA 剩余数变小的部分就是本轮收到的 RFID 字节数。 */
    Common_CopyData(Uart9_DMABuf, data, rlen); /* 把 B 通道 RFID 回包复制到任务临时缓冲区，供协议层解析。 */
    Uart9_DMAReset(); /* 取走数据后立即清空并重启 DMA，下一轮 RFID 请求不受旧回包影响。 */
  }

  return rlen; /* 返回 0 时表示本周期还没有足够数据可解析。 */
}

/*
 * 函数功能：反初始化 UART9 外设。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_DeInit(void)
{
  (void)Bsp_UartDeInit(BSP_UART_PORT_9); /* 释放 UART9 硬件资源，保留给后续调试或低功耗流程调用。 */
}
