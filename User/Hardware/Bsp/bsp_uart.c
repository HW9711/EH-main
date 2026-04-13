#include "bsp_uart.h"

extern UART_HandleTypeDef huart10;

UART_HandleTypeDef *Bsp_UartHandle(bsp_uart_port_t port)
{
    switch (port)
    {
    case BSP_UART_PORT_10:
        return &huart10;
    default:
        return 0;
    }
}

HAL_StatusTypeDef Bsp_UartTransmit(bsp_uart_port_t port, uint8_t *p_data, uint16_t length, uint32_t timeout)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(uart, p_data, length, timeout);
}
