
#ifndef __UART1_H
#define __UART1_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define UART1_MAX_PACKET_SIZE  255

void Uart1_Init(void);

/*
 * 函数功能：阻塞发送一帧 UART1 手柄驱动数据，并把 HAL 实际发送结果返回给业务层。
 * 输入参数：pData 指向待发送数据；Length 为发送字节数。
 * 返回参数：HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT，启动安全门禁据此判断零速帧是否真实发送成功。
 */
HAL_StatusTypeDef Uart1_SendPacket(uint8_t *pData, uint16_t Length);

uint16_t Uart1_DMARecvDataPeek(uint8_t *data);

void Uart1_DeInit(void);


#endif


