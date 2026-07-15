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
        return &huart9; /* UART9固定连接逻辑B侧RFID，不跟随手柄物理接口交换。 */
    case BSP_UART_PORT_10:
        return &huart10;
    default:
        return 0;
    }
}

HAL_StatusTypeDef Bsp_UartInit(bsp_uart_port_t port, uint32_t baud_rate)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 端口号没有对应 HAL 句柄时拒绝初始化，避免访问空句柄并误改其它串口。 */
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

    /* 端口号无效时直接返回错误，不能把空句柄交给 HAL 反初始化。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DeInit(uart);
}

HAL_StatusTypeDef Bsp_UartTransmit(bsp_uart_port_t port, uint8_t *p_data, uint16_t length, uint32_t timeout)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 未找到指定串口时禁止发送，避免错误端口造成空指针访问或数据发错通道。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(uart, p_data, length, timeout);
}

HAL_StatusTypeDef Bsp_UartAbort(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 无效端口没有可中止的硬件事务，返回错误并保持所有真实串口不变。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Abort(uart);
}

HAL_StatusTypeDef Bsp_UartReceiveDma(bsp_uart_port_t port, uint8_t *p_data, uint16_t length)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 无效端口不能启动 DMA 接收，防止 HAL 使用空串口句柄配置 DMA。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_DMA(uart, p_data, length);
}

HAL_StatusTypeDef Bsp_UartDmaStop(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 无效端口没有可停止的 DMA，返回错误且不影响其它串口接收。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DMAStop(uart);
}

uint32_t Bsp_UartRxDmaRemain(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 串口或其 RX DMA 句柄未建立时按剩余 0 返回，避免读取无效 DMA 寄存器。 */
    if ((uart == 0) || (uart->hdmarx == 0))
    {
        return 0;
    }

    return __HAL_DMA_GET_COUNTER(uart->hdmarx);
}
