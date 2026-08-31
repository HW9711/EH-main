
#ifndef __UART2_H
#define __UART2_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define UART2_MAX_PACKET_SIZE  150

/* 以下变量只供临时诊断固件在调试器中观察，不参与外控、手柄、脚踏、电机或泵的业务判定。 */
extern volatile uint32_t g_uart2_rx_error_count;          /* USART2 接收错误回调累计次数。 */
extern volatile uint32_t g_uart2_rx_last_error;           /* 最近一次 HAL UART 错误位。 */
extern volatile uint32_t g_uart2_rx_dma_restart_count;    /* 诊断任务成功恢复 RX DMA 的次数。 */
extern volatile uint32_t g_uart2_rx_dma_start_fail_count; /* RX DMA 初始启动或恢复失败的次数。 */
extern volatile uint32_t g_uart2_rx_packet_count;         /* 上层实际取走的 UART2 数据包数量。 */
extern volatile uint16_t g_uart2_rx_last_packet_length;   /* 最近一次交给命令解析器的数据包长度。 */

void Uart2_Init(void);

void Uart2_Configuration(uint32_t baud);

void Uart2_SendPacket(uint8_t *pData, uint16_t Length);

/*
 * 函数功能：按指定超时发送 UART2/RS485 数据并返回 HAL 结果，供故障后诊断导出统计失败。
 * 输入参数：data 为发送缓存；length 为字节数；timeout_ms 为单次阻塞上限。
 * 返回参数：HAL_OK 表示完整发送，HAL_ERROR/HAL_BUSY/HAL_TIMEOUT 表示失败。
 */
HAL_StatusTypeDef Uart2_SendPacketChecked(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

uint16_t Uart2_DMARecvDataPeek(uint8_t *data);

void Uart2_DeInit(void);

#endif


