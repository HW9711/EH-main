
#ifndef __UART1_H
#define __UART1_H

#include <stdint.h>

#define UART1_MAX_PACKET_SIZE  255

void Uart1_Init(void);

void Uart1_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart1_DMARecvDataPeek(uint8_t *data);

void Uart1_DeInit(void);


#endif


