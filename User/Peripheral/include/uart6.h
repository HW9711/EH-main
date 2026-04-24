
#ifndef __UART6_H
#define __UART6_H

#include <stdint.h>

#define UART6_MAX_PACKET_SIZE  150

void Uart6_Configuration(uint16_t baud);

void Uart6_Init(void);

void Uart6_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart6_DMARecvDataPeek(uint8_t *data);

void Uart6_DeInit(void);

#endif


