
#ifndef __UART5_H
#define __UART5_H

#include <stdint.h>

#define UART5_MAX_PACKET_SIZE  150

/*
 * UART5_DEBUG_UART10_ENABLE 是历史 UART5 步进驱动联调诊断开关。
 * 当前正式控制路径不允许再把泵闭环测试信息转发到 UART10，默认必须保持 0。
 * UART5 真实控制帧仍由 Uart5_SendPacket() 发送到 BSP_UART_PORT_5，不受该历史开关影响。
 */
#ifndef UART5_DEBUG_UART10_ENABLE
#define UART5_DEBUG_UART10_ENABLE 0U
#endif

void Uart5_Configuration(uint16_t baud);

void Uart5_Init(void);

void Uart5_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart5_DMARecvDataPeek(uint8_t *data);

void Uart5_DeInit(void);

#endif


