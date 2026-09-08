
#ifndef __UART1_H
#define __UART1_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define UART1_MAX_PACKET_SIZE  255 /* UART1 手柄驱动接收缓存容量，单位：字节；调用方接收缓存也必须足够大。 */

void Uart1_Init(void);

/*
 * 函数功能：等待 UART1 手柄驱动数据发送结束，再返回发送结果。
 * 输入参数：pData 指向待发送数据；Length 为发送字节数。
 * 返回参数：HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT；上层据此判断零速命令是否发送成功，再决定能否启动。
 */
HAL_StatusTypeDef Uart1_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart1_DMARecvDataPeek(uint8_t *data);

void Uart1_DeInit(void);


#endif


