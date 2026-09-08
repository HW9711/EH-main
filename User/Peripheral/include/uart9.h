#ifndef __UART9_H
#define __UART9_H

#include <stdint.h>

#define UART9_MAX_PACKET_SIZE  150U /* UART9 RFID 接收缓存容量，单位：字节；与 UART3 相同，修改时也要检查 RFID 任务的接收缓存。 */

void Uart9_Init(void);

void Uart9_SendPacket(uint8_t *pData, uint16_t Length);

void Uart9_ClearRecvData(void);

uint16_t Uart9_DMARecvDataPeek(uint8_t *data);

void Uart9_DeInit(void);

#endif
