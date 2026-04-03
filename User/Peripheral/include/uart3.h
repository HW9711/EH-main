
#ifndef __UART3_H
#define __UART3_H

#include <stdint.h>

#define UART3_MAX_PACKET_SIZE  150

void Uart3_Init(void);

void Uart3_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart3_DMARecvDataPeek(uint8_t *data);

void Uart3_DeInit(void);


#endif


