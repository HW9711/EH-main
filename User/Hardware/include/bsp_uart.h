#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "board_profile.h"

typedef enum
{
    BSP_UART_PORT_1 = 1,
    BSP_UART_PORT_2 = 2,
    BSP_UART_PORT_3 = 3,
    BSP_UART_PORT_4 = 4,
    BSP_UART_PORT_5 = 5,
    BSP_UART_PORT_6 = 6,
    BSP_UART_PORT_7 = 7,
    BSP_UART_PORT_9 = 9,
    BSP_UART_PORT_10 = 10
} bsp_uart_port_t;

UART_HandleTypeDef *Bsp_UartHandle(bsp_uart_port_t port);
HAL_StatusTypeDef Bsp_UartInit(bsp_uart_port_t port, uint32_t baud_rate);
HAL_StatusTypeDef Bsp_UartDeInit(bsp_uart_port_t port);
HAL_StatusTypeDef Bsp_UartTransmit(bsp_uart_port_t port, uint8_t *p_data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef Bsp_UartAbort(bsp_uart_port_t port);
HAL_StatusTypeDef Bsp_UartReceiveDma(bsp_uart_port_t port, uint8_t *p_data, uint16_t length);
HAL_StatusTypeDef Bsp_UartDmaStop(bsp_uart_port_t port);
uint32_t Bsp_UartRxDmaRemain(bsp_uart_port_t port);

#endif /* __BSP_UART_H */
