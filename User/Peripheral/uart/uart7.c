//uart7.c

#include "main.h"
#include "uart7.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"
//#include "data.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

static uint8_t Uart7_DMABuf[UART7_MAX_PACKET_SIZE] = { 0 };
static uint32_t Uart7_BaudRate = 115200U;

/*
 * 函数功能：设置 UART7 泵驱动串口波特率，并记下该值供发送失败后重新初始化使用。
 * 输入参数：baud 为波特率，单位：bit/s。
 * 返回参数：无。
 */
void Uart7_Configuration(uint16_t baud)
{
  Uart7_BaudRate = baud;

  /* 初始化失败后不能继续用泵驱动串口收发。 */
  if (Bsp_UartInit(BSP_UART_PORT_7, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 函数功能：启动 UART7 DMA，把泵驱动回包存入 UART7 自己的接收缓存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart7_DmaInit(void)
{
  Bsp_UartReceiveDma(BSP_UART_PORT_7, Uart7_DMABuf, UART7_MAX_PACKET_SIZE); /* UART7 改为全双工后立即接收 B 泵驱动回包。 */
}

/*
 * 函数功能：清空 UART7 本轮接收缓存并立即重新启动循环 DMA，下一条泵命令回包从缓存起点写入。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart7_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_7);
  memset(Uart7_DMABuf, 0, UART7_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_7, Uart7_DMABuf, UART7_MAX_PACKET_SIZE);

}

/*
 * 函数功能：启动 UART7 DMA 接收，为 B 泵第一条控制命令的回包做好准备。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart7_Init(void)
{
  Uart7_DmaInit(); /* UART7 的 RX 引脚和 DMA 已由 HAL MSP 初始化，此处只启动实际接收。 */
}

/*
 * 函数功能：发送 UART7 泵控制命令；失败时重新初始化串口、恢复 DMA 接收，再重发一次。
 * 输入参数：pData 指向完整控制帧；Length 为发送字节数。
 * 返回参数：发送成功返回 1，重试后仍失败返回 0。
 */
uint8_t Uart7_SendPacket(uint8_t *pData, uint16_t Length)
{
  /* 第一次发送失败后按原波特率重新初始化 UART7，只重试一次。 */
  if (Bsp_UartTransmit(BSP_UART_PORT_7, pData, Length, 100) != HAL_OK)
  {
    Bsp_UartAbort(BSP_UART_PORT_7);
    Bsp_UartDeInit(BSP_UART_PORT_7);
    /* 串口重新初始化成功后才允许重发，避免继续使用未恢复的硬件。 */
    if (Bsp_UartInit(BSP_UART_PORT_7, Uart7_BaudRate) == HAL_OK)
    {
      Uart7_DmaInit(); /* HAL 重初始化会释放原 RX DMA，重发控制帧前必须恢复驱动反馈接收。 */
      return (Bsp_UartTransmit(BSP_UART_PORT_7, pData, Length, 100) == HAL_OK) ? 1U : 0U;
    }

    return 0U;
  }

  return 1U;
}

/*
 * 函数功能：取走 UART7 DMA 在上一泵周期收到的全部字节，并立即重启 DMA 接收下一帧反馈。
 * 输入参数：data 指向至少 UART7_MAX_PACKET_SIZE 字节的调用方缓存。
 * 返回参数：本次复制的字节数；没有新字节或参数无效时返回 0。
 */
uint16_t Uart7_DMARecvDataPeek(uint8_t *data)
{
  uint32_t remain_len; /* 保存 DMA 当前尚未写入的字节数，用于换算本周期已接收长度。 */
  uint16_t received_len; /* 保存本次需要交给泵协议层解析的实际字节数。 */

  if (data == NULL)
  {
    return 0U; /* 没有输出缓存就不取数据，保留 DMA 中已收到的回包。 */
  }

  remain_len = Bsp_UartRxDmaRemain(BSP_UART_PORT_7); /* 在下一条 25ms 泵命令发送前读取上一条命令的回包长度。 */
  if (remain_len >= UART7_MAX_PACKET_SIZE)
  {
    return 0U; /* DMA 尚未收到新字节时保持当前接收，不执行无意义的停止和重启。 */
  }

  received_len = (uint16_t)(UART7_MAX_PACKET_SIZE - remain_len); /* 用缓存容量减去 DMA 剩余空间，得到本次可取出的字节数。 */
  Common_CopyData(Uart7_DMABuf, data, received_len); /* 先把已收到的字节复制给泵任务，再清空 DMA 缓存。 */
  Uart7_DMAReset(); /* 复制完成后立即重启 DMA，保证随后发送的泵命令能够收到对应反馈。 */

  return received_len;
}

void Uart7_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_7);
}






