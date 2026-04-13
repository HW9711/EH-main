#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "board_profile.h"

typedef enum
{
    BSP_UART_PORT_10 = 10
} bsp_uart_port_t;

UART_HandleTypeDef *Bsp_UartHandle(bsp_uart_port_t port);
HAL_StatusTypeDef Bsp_UartTransmit(bsp_uart_port_t port, uint8_t *p_data, uint16_t length, uint32_t timeout);

#endif /* __BSP_UART_H */
