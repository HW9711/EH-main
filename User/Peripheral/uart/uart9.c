//uart9.c

#include "main.h"
#include "uart9.h"
#include "bsp_uart.h"
#include "common.h"

#include <string.h>
#include <stdio.h>

static uint8_t Uart9_DMABuf[UART9_MAX_PACKET_SIZE] = { 0U }; /* 只保存 UART9 收到的 RFID 数据；属于哪个手柄由上层请求决定。 */

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
  memset(Uart9_DMABuf, 0, UART9_MAX_PACKET_SIZE); /* 清掉旧 RFID 数据，避免下次请求读到上一张标签。 */
  (void)Bsp_UartReceiveDma(BSP_UART_PORT_9, Uart9_DMABuf, UART9_MAX_PACKET_SIZE); /* 重新接收 UART9 所连 RFID 模块的回包。 */
}

/*
 * 函数功能：开始接收 UART9 RFID 数据。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_Init(void)
{
  Uart9_DmaInit(); /* 启动 UART9 自己的 DMA，不更改 RFID 线束或手柄分配关系。 */
}

/*
 * 函数功能：通过 UART9 发送一帧 RFID 命令。
 * 输入参数：pData 指向待发送命令缓冲区；Length 表示命令字节数。
 * 返回参数：无。
 */
void Uart9_SendPacket(uint8_t *pData, uint16_t Length)
{
  (void)Bsp_UartTransmit(BSP_UART_PORT_9, pData, Length, 100U); /* 发到 UART9 所连 RFID 模块，最多等待 100ms。 */
}

/*
 * 函数功能：清空 UART9 当前 DMA 接收缓存并重新接收。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_ClearRecvData(void)
{
  Uart9_DMAReset(); /* 新 RFID 请求开始前清掉旧数据，避免误认成新请求的回包。 */
}

/*
 * 函数功能：取出 UART9 当前已收到的 RFID 字节并重新接收；本函数不等待完整帧。
 * 输入参数：data 指向至少 UART9_MAX_PACKET_SIZE 字节的输出缓存。
 * 返回参数：复制的字节数；未收到字节或 data 为空时返回 0。
 */
uint16_t Uart9_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0U; /* DMA 剩余计数，用于反推 UART9 已接收字节数。 */
  uint16_t rlen = 0U;      /* 返回给 RFID 任务的本轮有效数据长度。 */

  if (data == NULL)
  {
    return 0U; /* 调用方缓冲区无效时不访问 DMA 数据，避免异常写内存。 */
  }

  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_9); /* 读取 DMA 缓存还可接收多少字节。 */
  if (RemainLen < UART9_MAX_PACKET_SIZE)
  {
    rlen = (uint16_t)(UART9_MAX_PACKET_SIZE - RemainLen); /* DMA 剩余数变小的部分就是本轮收到的 RFID 字节数。 */
    Common_CopyData(Uart9_DMABuf, data, rlen); /* 复制给 RFID 任务；由任务判断数据属于哪个请求、是否完整。 */
    Uart9_DMAReset(); /* 取走数据后立即清空并重启 DMA，下一轮 RFID 请求不受旧回包影响。 */
  }

  return rlen; /* 返回本次实际取出的字节数，不代表 RFID 回包已经校验通过。 */
}

/*
 * 函数功能：关闭并清除 UART9 的外设配置。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart9_DeInit(void)
{
  (void)Bsp_UartDeInit(BSP_UART_PORT_9); /* 关闭 UART9；再次收发前需要重新初始化。 */
}
