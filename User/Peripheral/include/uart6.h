
#ifndef __UART6_H
#define __UART6_H

#include <stdint.h>

#define UART6_MAX_PACKET_SIZE  150 /* 触控屏串口接收缓存容量，单位：字节；增大时也要检查屏幕解析用的缓存。 */

void Uart6_Configuration(uint16_t baud);

void Uart6_Init(void);

void Uart6_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart6_DMARecvDataPeek(uint8_t *data);

void Uart6_DeInit(void);

#endif


