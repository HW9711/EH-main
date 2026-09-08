
#ifndef __UART5_H
#define __UART5_H

#include <stdint.h>

#define UART5_MAX_PACKET_SIZE  150 /* UART5 泵驱动反馈缓存容量，单位：字节；不是泵的速度或流量上限。 */

/*
 * UART5_DEBUG_UART10_ENABLE 是旧版“把泵测试信息发到 UART10”的开关，保留值为 0。
 * 当前 UART5 发送函数已不使用此开关，改成 1 也不会恢复测试输出。
 * 泵控制命令始终由 Uart5_SendPacket() 发到 BSP_UART_PORT_5。
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


