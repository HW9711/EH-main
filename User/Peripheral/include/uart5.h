
#ifndef __UART5_H
#define __UART5_H

#include <stdint.h>

#define UART5_MAX_PACKET_SIZE  150

void Uart5_Configuration(uint16_t baud);

void Uart5_Init(void);

void Uart5_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart5_DMARecvDataPeek(uint8_t *data);

void Uart5_DeInit(void);

#endif


