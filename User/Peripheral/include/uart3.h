
#ifndef __UART3_H
#define __UART3_H

#include <stdint.h>

#define UART3_MAX_PACKET_SIZE  150 /* UART3 RFID 接收缓存容量，单位：字节；增大时也要检查 RFID 任务的接收缓存。 */

void Uart3_Init(void);

void Uart3_SendPacket(uint8_t *pData, uint16_t Length);

void Uart3_ClearRecvData(void);

uint16_t Uart3_DMARecvDataPeek(uint8_t *data);

void Uart3_DeInit(void);


#endif


