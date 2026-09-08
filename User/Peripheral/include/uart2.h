
#ifndef __UART2_H
#define __UART2_H

#include <stdint.h>

#define UART2_MAX_PACKET_SIZE  150 /* 外部设备通信接收缓存容量，单位：字节；增大时也要检查调用方缓存大小。 */

void Uart2_Init(void);

void Uart2_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart2_DMARecvDataPeek(uint8_t *data);

void Uart2_DeInit(void);

#endif


