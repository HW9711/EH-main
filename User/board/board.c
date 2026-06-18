/**
 * @file board.c
 * @brief 硬件配置实现文件 - 实现所有硬件接口的初始化
 *        使用board.h中的宏定义，修改引脚只需改board.h
 */
#include "stm32f4xx_hal.h"
#include "board.h"
#include "board_resource_map.h"

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
void Error_Handler(void);

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

    /* A/B手柄实体运行键输入 - 上拉输入，按下时PE12/PE13被拉低，由手柄按键任务轮询控制启停 */
    GPIO_InitStruct.Pin = BOARD_RES_HANDLE_RUN_KEY_A_PIN | BOARD_RES_HANDLE_RUN_KEY_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_RES_HANDLE_RUN_KEY_A_PORT, &GPIO_InitStruct);

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


