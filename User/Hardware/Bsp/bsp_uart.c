#include "bsp_uart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;

UART_HandleTypeDef *Bsp_UartHandle(bsp_uart_port_t port)
{
    switch (port)
    {
    case BSP_UART_PORT_1:
        return &huart1;
    case BSP_UART_PORT_2:
        return &huart2;
    case BSP_UART_PORT_3:
        return &huart3;
    case BSP_UART_PORT_4:
        return &huart4;
    case BSP_UART_PORT_5:
        return &huart5;
    case BSP_UART_PORT_6:
        return &huart6;
    case BSP_UART_PORT_7:
        return &huart7;
    case BSP_UART_PORT_9:
        return &huart9; /* UART9 固定连接 B 通道 RFID，供双串口模式独立收发。 */
    case BSP_UART_PORT_10:
        return &huart10;
    default:
        return 0;
    }
}

HAL_StatusTypeDef Bsp_UartInit(bsp_uart_port_t port, uint32_t baud_rate)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    uart->Init.BaudRate = baud_rate;

    return HAL_UART_Init(uart);
}

HAL_StatusTypeDef Bsp_UartDeInit(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DeInit(uart);
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

HAL_StatusTypeDef Bsp_UartAbort(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Abort(uart);
}

HAL_StatusTypeDef Bsp_UartReceiveDma(bsp_uart_port_t port, uint8_t *p_data, uint16_t length)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_DMA(uart, p_data, length);
}

HAL_StatusTypeDef Bsp_UartDmaStop(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DMAStop(uart);
}

uint32_t Bsp_UartRxDmaRemain(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    if ((uart == 0) || (uart->hdmarx == 0))
    {
        return 0;
    }

    return __HAL_DMA_GET_COUNTER(uart->hdmarx);
}
