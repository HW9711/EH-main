
#ifndef __UART4_H
#define __UART4_H

#include <stdint.h>

#define UART4_MAX_PACKET_SIZE  150

void Uart4_Init(void);

void Uart4_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart4_DMARecvDataPeek(uint8_t *data);

void Uart4_DeInit(void);


#endif


