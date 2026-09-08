
#ifndef __UART7_H
#define __UART7_H

#include <stdint.h>

#define UART7_MAX_PACKET_SIZE  150 /* UART7 泵驱动反馈缓存容量，单位：字节；不是泵的速度或流量上限。 */

void Uart7_Init(void);

uint8_t Uart7_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart7_DMARecvDataPeek(uint8_t *data);

void Uart7_DeInit(void);


#endif


