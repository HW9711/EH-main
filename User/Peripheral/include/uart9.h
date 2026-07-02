#ifndef __UART9_H
#define __UART9_H

#include <stdint.h>

#define UART9_MAX_PACKET_SIZE  150U /* UART9 RFID 接收缓存长度，与 UART3 保持一致，避免协议解析缓冲区大小不一致。 */

void Uart9_Init(void);

void Uart9_SendPacket(uint8_t *pData, uint16_t Length);

void Uart9_ClearRecvData(void);

uint16_t Uart9_DMARecvDataPeek(uint8_t *data);

void Uart9_DeInit(void);

#endif
