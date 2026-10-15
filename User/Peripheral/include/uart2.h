
#ifndef __UART2_H
#define __UART2_H

#include <stdint.h>

#define UART2_MAX_PACKET_SIZE  160 /* 外部通信DMA接收缓存为160字节，与正式协议整帧上限一致，避免较长心跳或粘包处理使用不同容量。 */

void Uart2_Init(void);

void Uart2_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart2_DMARecvDataPeek(uint8_t *data);

void Uart2_DeInit(void);

#endif


