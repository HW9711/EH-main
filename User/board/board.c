/**
 * @file board.c
 * @brief 配置主控业务GPIO；串口、I2C和定时器由各自初始化函数设置。
 *        引脚来自board.h和board_resource_map.h，改接线还须核对board_profile.h的通道交换开关。
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
/*
 * 函数功能：初始化主控业务 GPIO，将外部 RS485 默认置为接收，并让A/B手柄复用脚默认进入普通按键输入模式。
 * 输入参数：无。
 * 返回参数：无。
 */
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

    /* 历史手柄数据输入仅保留 PD2/PD3；V4.0 已将原 PD4 改作外部 RS485 方向控制。 */
    GPIO_InitStruct.Pin = BOARD_M_D1_PIN | BOARD_M_D2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_M_D1_PORT, &GPIO_InitStruct);

    /* 先锁存低电平再切换为输出，避免 GPIO 模式切换瞬间误使能 RS485 发送器。 */
    HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BOARD_UART2_RS485_DIR_PIN;       /* PD4 同时连接 CA-IS3092W 的 DE 和 /RE。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出保证收发方向电平明确，不依赖外部上拉。 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;                    /* 板载 R222 已提供偏置，MCU 运行期直接驱动方向脚。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;           /* 方向切换频率低，低速沿可减少无必要的高频干扰。 */
    HAL_GPIO_Init(BOARD_UART2_RS485_DIR_PORT, &GPIO_InitStruct);

    /* 手柄按键输入 - 上拉输入 */
    GPIO_InitStruct.Pin = BOARD_H_MD1_PIN | BOARD_H_MD2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_H_MD1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_H_MD3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_H_MD3_PORT, &GPIO_InitStruct);

    /* A/B手柄复用脚默认作为上拉输入；普通手柄分别把PE2/PE0拉低表示按键按下。 */
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


