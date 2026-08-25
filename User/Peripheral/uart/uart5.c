//uart5.c

#include "main.h"
#include "uart5.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"
//#include "data.h"

#include <string.h>

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

/*
 * 函数功能：清空 UART5 本轮接收缓存并立即重新启动循环 DMA，下一条泵命令回包从缓存起点写入。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Uart5_DMAReset(void)
{

  Bsp_UartDmaStop(BSP_UART_PORT_5);
  memset(Uart5_DMABuf, 0, UART5_MAX_PACKET_SIZE);
  Bsp_UartReceiveDma(BSP_UART_PORT_5, Uart5_DMABuf, UART5_MAX_PACKET_SIZE);

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

/*
 * 函数功能：取走 UART5 DMA 在上一泵周期收到的全部字节，并立即重启 DMA 接收下一帧反馈。
 * 输入参数：data 指向至少 UART5_MAX_PACKET_SIZE 字节的调用方缓存。
 * 返回参数：本次复制的字节数；没有新字节或参数无效时返回 0。
 */
uint16_t Uart5_DMARecvDataPeek(uint8_t *data)
{
  uint32_t remain_len; /* 保存 DMA 当前尚未写入的字节数，用于换算本周期已接收长度。 */
  uint16_t received_len; /* 保存本次需要交给泵协议层解析的实际字节数。 */

  if (data == NULL)
  {
    return 0U; /* 调用方没有提供缓存时不得停止 DMA 或丢弃现场回包。 */
  }

  remain_len = Bsp_UartRxDmaRemain(BSP_UART_PORT_5); /* 在下一条 25ms 泵命令发送前读取上一条命令的回包长度。 */
  if (remain_len >= UART5_MAX_PACKET_SIZE)
  {
    return 0U; /* DMA 尚未收到新字节时保持当前接收，不执行无意义的停止和重启。 */
  }

  received_len = (uint16_t)(UART5_MAX_PACKET_SIZE - remain_len); /* 循环 DMA 每个泵周期都会重启，差值就是本周期完整接收长度。 */
  Common_CopyData(Uart5_DMABuf, data, received_len); /* 先复制不可变快照，再释放底层 DMA 缓冲供下一帧覆盖。 */
  Uart5_DMAReset(); /* 复制完成后立即重启 DMA，保证随后发送的泵命令能够收到对应反馈。 */

  return received_len;
}

void Uart5_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_5);
}
