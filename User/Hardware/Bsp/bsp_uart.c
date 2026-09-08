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

/*
 * 函数功能：按串口编号取得 HAL 串口对象，不改变任何串口配置。
 * 输入参数：port 为 BSP_UART_PORT_x 编号。
 * 返回参数：对应串口对象的指针；编号不支持时返回 0。
 */
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
        return &huart9; /* UART9 固定返回自己的串口对象，不在这里交换手柄与 RFID 的对应关系。 */
    case BSP_UART_PORT_10:
        return &huart10;
    default:
        return 0;
    }
}

/*
 * 函数功能：按指定波特率初始化串口，其余设置使用该串口已有配置。
 * 输入参数：port 为串口编号；baud_rate 为波特率，单位：bit/s。
 * 返回参数：HAL 初始化结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartInit(bsp_uart_port_t port, uint32_t baud_rate)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不把空指针交给 HAL。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    uart->Init.BaudRate = baud_rate;

    return HAL_UART_Init(uart);
}

/*
 * 函数功能：关闭并清除指定串口的外设配置。
 * 输入参数：port 为串口编号。
 * 返回参数：HAL 关闭结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartDeInit(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不把空指针交给 HAL。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DeInit(uart);
}

/*
 * 函数功能：等待指定串口发送完成，或在失败、超时后返回。
 * 输入参数：port 为串口编号；p_data 为数据；length 为字节数；timeout 为最长等待时间，单位：ms。
 * 返回参数：HAL 发送结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartTransmit(bsp_uart_port_t port, uint8_t *p_data, uint16_t length, uint32_t timeout)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不发送数据。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(uart, p_data, length, timeout);
}

/*
 * 函数功能：中止指定串口正在进行的收发。
 * 输入参数：port 为串口编号。
 * 返回参数：HAL 中止结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartAbort(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不停止其他串口。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Abort(uart);
}

/*
 * 函数功能：启动 DMA 接收，让收到的字节写入指定缓存。
 * 输入参数：port 为串口编号；p_data 为接收缓存；length 为缓存容量，单位：字节。
 * 返回参数：HAL 启动接收结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartReceiveDma(bsp_uart_port_t port, uint8_t *p_data, uint16_t length)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不启动 DMA。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_DMA(uart, p_data, length);
}

/*
 * 函数功能：停止指定串口的 DMA 收发。
 * 输入参数：port 为串口编号。
 * 返回参数：HAL 停止结果；串口编号无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef Bsp_UartDmaStop(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 找不到该串口就返回错误，不停止其他串口的 DMA。 */
    if (uart == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DMAStop(uart);
}

/*
 * 函数功能：读取 DMA 接收缓存还能接收多少字节，供调用方计算已收长度。
 * 输入参数：port 为串口编号。
 * 返回参数：DMA 剩余字节数；串口或 DMA 对象无效时也返回 0，调用方不能只凭 0 判断收满。
 */
uint32_t Bsp_UartRxDmaRemain(bsp_uart_port_t port)
{
    UART_HandleTypeDef *uart = Bsp_UartHandle(port);

    /* 串口或接收 DMA 还不可用时返回 0，不读取无效地址。 */
    if ((uart == 0) || (uart->hdmarx == 0))
    {
        return 0;
    }

    return __HAL_DMA_GET_COUNTER(uart->hdmarx);
}
