/**
 * @file board.c
 * @brief 硬件配置实现文件 - 实现所有硬件接口的初始化
 *        使用board.h中的宏定义，修改引脚只需改board.h
 */
#include "stm32f4xx_hal.h"
#include "board.h"

/*============================================================================
 * 串口外设句柄声明 (来自CubeMX生成的文件)
 *============================================================================*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart10;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart10_rx;

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim14;

/*============================================================================
 * GPIO初始化实现
 *============================================================================*/
void Board_GPIOConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* 状态LED初始化 - 输出 */
    HAL_GPIO_WritePin(BOARD_STATE_LED_PORT, BOARD_STATE_LED_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = BOARD_STATE_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_STATE_LED_PORT, &GPIO_InitStruct);

    /* 蜂鸣器初始化 - 输出 */
    HAL_GPIO_WritePin(BOARD_BEEP_PORT, BOARD_BEEP_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BOARD_BEEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_BEEP_PORT, &GPIO_InitStruct);

    /* I2C引脚初始化 - 开漏输出 */
    GPIO_InitStruct.Pin = BOARD_IIC_SCL_PIN | BOARD_IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_IIC_SCL_PORT, &GPIO_InitStruct);

    /* 1-Wire I 引脚初始化 - 开漏输出 */
    GPIO_InitStruct.Pin = BOARD_ONEWIRE_I_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_ONEWIRE_I_PORT, &GPIO_InitStruct);

    /* 1-Wire II 引脚初始化 - 开漏输出 */
    GPIO_InitStruct.Pin = BOARD_ONEWIRE_II_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_ONEWIRE_II_PORT, &GPIO_InitStruct);

    /* 手柄数据输入 - 上拉输入 */
    GPIO_InitStruct.Pin = BOARD_M_D1_PIN | BOARD_M_D2_PIN | BOARD_M_D3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_M_D1_PORT, &GPIO_InitStruct);

    /* 手柄按键输入 - 上拉输入 */
    GPIO_InitStruct.Pin = BOARD_H_MD1_PIN | BOARD_H_MD2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_H_MD1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_H_MD3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_H_MD3_PORT, &GPIO_InitStruct);

    /* LED指示灯初始化 - 输出 */
    GPIO_InitStruct.Pin = BOARD_LED_H1_PIN | BOARD_LED_H2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_H1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_LED_H3_PIN | BOARD_LED_H4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_H3_PORT, &GPIO_InitStruct);

    /* R200-K8控制 - 输出 */
    HAL_GPIO_WritePin(BOARD_R200_K8_PORT, BOARD_R200_K8_PIN, GPIO_PIN_SET);  /* 默认关闭 */
    GPIO_InitStruct.Pin = BOARD_R200_K8_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_R200_K8_PORT, &GPIO_InitStruct);

    /* 继电器控制 - 输出 */
    /* 新板没有 K1/K2 时，跳过这组 GPIO 初始化。 */
#if (BOARD_HAS_K1K2 == 1U)
    GPIO_InitStruct.Pin = BOARD_K1_PIN | BOARD_K2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_K1_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BOARD_K1_PORT, BOARD_K1_PIN | BOARD_K2_PIN, GPIO_PIN_SET);  /* 默认关闭 */
#endif

    /* 外部中断输入 - EXTI */
    GPIO_InitStruct.Pin = BOARD_EXTI1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_EXTI1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_EXTI10_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_EXTI10_PORT, &GPIO_InitStruct);

    /* 配置NVIC中断优先级 */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*============================================================================
 * UART GPIO初始化 - 使用board.h中的宏定义
 *============================================================================*/
static void Board_UART_GPIO_Init(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        /* TX GPIO */
        GPIO_InitStruct.Pin = BOARD_UART1_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART1_TX_AF;
        HAL_GPIO_Init(BOARD_UART1_TX_PORT, &GPIO_InitStruct);

        /* RX GPIO */
        GPIO_InitStruct.Pin = BOARD_UART1_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART1_RX_AF;
        HAL_GPIO_Init(BOARD_UART1_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == USART2)
    {
        GPIO_InitStruct.Pin = BOARD_UART2_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART2_TX_AF;
        HAL_GPIO_Init(BOARD_UART2_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART2_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART2_RX_AF;
        HAL_GPIO_Init(BOARD_UART2_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == USART3)
    {
        GPIO_InitStruct.Pin = BOARD_UART3_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART3_TX_AF;
        HAL_GPIO_Init(BOARD_UART3_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART3_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART3_RX_AF;
        HAL_GPIO_Init(BOARD_UART3_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART4)
    {
        GPIO_InitStruct.Pin = BOARD_UART4_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART4_TX_AF;
        HAL_GPIO_Init(BOARD_UART4_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART4_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART4_RX_AF;
        HAL_GPIO_Init(BOARD_UART4_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART5)
    {
        GPIO_InitStruct.Pin = BOARD_UART5_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART5_TX_AF;
        HAL_GPIO_Init(BOARD_UART5_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART5_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART5_RX_AF;
        HAL_GPIO_Init(BOARD_UART5_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == USART6)
    {
        GPIO_InitStruct.Pin = BOARD_UART6_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART6_TX_AF;
        HAL_GPIO_Init(BOARD_UART6_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART6_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART6_RX_AF;
        HAL_GPIO_Init(BOARD_UART6_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART7)
    {
        GPIO_InitStruct.Pin = BOARD_UART7_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART7_TX_AF;
        HAL_GPIO_Init(BOARD_UART7_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART7_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART7_RX_AF;
        HAL_GPIO_Init(BOARD_UART7_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART8)
    {
        GPIO_InitStruct.Pin = BOARD_UART8_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART8_TX_AF;
        HAL_GPIO_Init(BOARD_UART8_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART8_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART8_RX_AF;
        HAL_GPIO_Init(BOARD_UART8_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == UART10)
    {
        GPIO_InitStruct.Pin = BOARD_UART10_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BOARD_UART10_TX_AF;
        HAL_GPIO_Init(BOARD_UART10_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = BOARD_UART10_RX_PIN;
        GPIO_InitStruct.Alternate = BOARD_UART10_RX_AF;
        HAL_GPIO_Init(BOARD_UART10_RX_PORT, &GPIO_InitStruct);
    }
}

/*============================================================================
 * UART DMA初始化 - 使用board.h中的宏定义
 *============================================================================*/
static void Board_UART_DMA_Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        hdma_usart1_rx.Instance = BOARD_UART1_DMA_RX;
        hdma_usart1_rx.Init.Channel = BOARD_UART1_DMA_CHANNEL;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart1_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (huart->Instance == USART2)
    {
        hdma_usart2_rx.Instance = BOARD_UART2_DMA_RX;
        hdma_usart2_rx.Init.Channel = BOARD_UART2_DMA_CHANNEL;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart2_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (huart->Instance == USART3)
    {
        hdma_usart3_rx.Instance = BOARD_UART3_DMA_RX;
        hdma_usart3_rx.Init.Channel = BOARD_UART3_DMA_CHANNEL;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart3_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (huart->Instance == UART4)
    {
        hdma_uart4_rx.Instance = BOARD_UART4_DMA_RX;
        hdma_uart4_rx.Init.Channel = BOARD_UART4_DMA_CHANNEL;
        hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart4_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart4_rx);
        HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
    else if (huart->Instance == UART5)
    {
        hdma_uart5_rx.Instance = BOARD_UART5_DMA_RX;
        hdma_uart5_rx.Init.Channel = BOARD_UART5_DMA_CHANNEL;
        hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart5_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart5_rx);
        HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (huart->Instance == USART6)
    {
        hdma_usart6_rx.Instance = BOARD_UART6_DMA_RX;
        hdma_usart6_rx.Init.Channel = BOARD_UART6_DMA_CHANNEL;
        hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart6_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);
        HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
    else if (huart->Instance == UART7)
    {
        hdma_uart7_rx.Instance = BOARD_UART7_DMA_RX;
        hdma_uart7_rx.Init.Channel = BOARD_UART7_DMA_CHANNEL;
        hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart7_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart7_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart7_rx);
        HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART7_IRQn);
    }
    else if (huart->Instance == UART8)
    {
        hdma_uart8_rx.Instance = BOARD_UART8_DMA_RX;
        hdma_uart8_rx.Init.Channel = BOARD_UART8_DMA_CHANNEL;
        hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart8_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart8_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart8_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart8_rx);
        HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART8_IRQn);
    }
    else if (huart->Instance == UART10)
    {
        hdma_uart10_rx.Instance = BOARD_UART10_DMA_RX;
        hdma_uart10_rx.Init.Channel = BOARD_UART10_DMA_CHANNEL;
        hdma_uart10_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart10_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart10_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart10_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart10_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart10_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart10_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart10_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart10_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart10_rx);
        HAL_NVIC_SetPriority(UART10_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART10_IRQn);
    }
}

/*============================================================================
 * UART初始化实现 - 使用board.h中的宏定义
 *============================================================================*/
void Board_UART_Init(void)
{
    /* USART1 - 9600 */
    BOARD_UART1_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart1);
    Board_UART_DMA_Init(&huart1);
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }

    /* USART2 - 9600 */
    BOARD_UART2_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart2);
    Board_UART_DMA_Init(&huart2);
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }

    /* USART3 - 115200 (显示屏) */
    BOARD_UART3_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart3);
    Board_UART_DMA_Init(&huart3);
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_2;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }

    /* UART4 - 115200 */
    BOARD_UART4_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart4);
    Board_UART_DMA_Init(&huart4);
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_2;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }

    /* UART5 - 9600 */
    BOARD_UART5_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart5);
    Board_UART_DMA_Init(&huart5);
    huart5.Init.BaudRate = 9600;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK) { Error_Handler(); }

    /* USART6 - 115200 */
    BOARD_UART6_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart6);
    Board_UART_DMA_Init(&huart6);
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_2;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }

    /* UART7 - 9600 */
    BOARD_UART7_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart7);
    Board_UART_DMA_Init(&huart7);
    huart7.Init.BaudRate = 9600;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart7) != HAL_OK) { Error_Handler(); }

    /* UART8 - 115200 */
    BOARD_UART8_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart8);
    Board_UART_DMA_Init(&huart8);
    huart8.Init.BaudRate = 115200;
    huart8.Init.WordLength = UART_WORDLENGTH_8B;
    huart8.Init.StopBits = UART_STOPBITS_1;
    huart8.Init.Parity = UART_PARITY_NONE;
    huart8.Init.Mode = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart8) != HAL_OK) { Error_Handler(); }

    /* UART10 - 115200 */
    BOARD_UART10_CLK_ENABLE();
    Board_UART_GPIO_Init(&huart10);
    Board_UART_DMA_Init(&huart10);
    huart10.Init.BaudRate = 115200;
    huart10.Init.WordLength = UART_WORDLENGTH_8B;
    huart10.Init.StopBits = UART_STOPBITS_1;
    huart10.Init.Parity = UART_PARITY_NONE;
    huart10.Init.Mode = UART_MODE_TX_RX;
    huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart10.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart10) != HAL_OK) { Error_Handler(); }
}

/*============================================================================
 * ADC初始化实现
 *============================================================================*/
void Board_ADC_Init(void)
{
    /* ADC初始化由CubeMX生成的代码负责
     * 通道配置定义在board.h中
     * 如需更改ADC通道，修改ADC配置并更新DMA配置 */
}

/*============================================================================
 * I2C初始化实现 (软件模拟)
 *============================================================================*/
void Board_I2C_Init(void)
{
    /* 软件模拟I2C - 初始化GPIO后即可使用
     * SCL和SDA默认置高 */
    BOARD_IIC_SCL_H();
    BOARD_IIC_SDA_H();
}

/*============================================================================
 * 1-Wire初始化实现
 *============================================================================*/
void Board_OneWire_Init(void)
{
    /* 1-Wire - 初始化GPIO后即可使用
     * 默认置高释放总线 */
    BOARD_ONEWIRE_I_H();
    BOARD_ONEWIRE_II_H();
}

/*============================================================================
 * 定时器初始化实现
 *============================================================================*/
void Board_TIM_Init(void)
{
    /* 定时器初始化由CubeMX生成的代码负责
     * TIM7: 系统滴答定时器
     * TIM10: 蜂鸣器驱动定时器
     * TIM14: 系统计时定时器(1秒) */
}

/*============================================================================
 * 统一硬件初始化函数
 *============================================================================*/
void Board_Hardware_Init(void)
{
    /* 按顺序初始化所有硬件 */

    /* 1. GPIO初始化 - 最先执行 */
    Board_GPIOConfiguration();

    /* 2. I2C初始化 - 必须在GPIO之后 */
    Board_I2C_Init();

    /* 3. 1-Wire初始化 - 必须在GPIO之后 */
    Board_OneWire_Init();

    /* 4. UART初始化 - 使用board.h中的宏定义 */
    Board_UART_Init();

    /* 5. ADC初始化 - 需要GPIO和外设 */
    Board_ADC_Init();

    /* 6. 定时器初始化 */
    Board_TIM_Init();
}
