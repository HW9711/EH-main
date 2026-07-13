/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "board.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart9;
UART_HandleTypeDef huart10;

DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart9_rx;
DMA_HandleTypeDef hdma_uart10_rx;
/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_2;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/* UART8 init function */
void MX_UART8_Init(void)
{
  
  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    
    Error_Handler();
  }
/* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */
}

/* UART9 init function */
void MX_UART9_Init(void)
{

  /* USER CODE BEGIN UART9_Init 0 */

  /* USER CODE END UART9_Init 0 */

  /* USER CODE BEGIN UART9_Init 1 */

  /* USER CODE END UART9_Init 1 */
  huart9.Instance = UART9;
  huart9.Init.BaudRate = 115200;              /* UART9 作为 B 通道 RFID，波特率与 A 通道 USART3 保持一致。 */
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_2;     /* RFID 模块当前按 UART3 的 2 stop bits 时序工作，新串口保持同配置。 */
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart9) != HAL_OK)
  {

    Error_Handler();
  }
  /* USER CODE BEGIN UART9_Init 2 */

  /* USER CODE END UART9_Init 2 */
}

/* UART10 init function */
void MX_UART10_Init(void)
{
  
  /* USER CODE BEGIN UART10_Init 0 */

  /* USER CODE END UART10_Init 0 */

  /* USER CODE BEGIN UART10_Init 1 */

  /* USER CODE END UART10_Init 1 */
  huart10.Instance = UART10;
  huart10.Init.BaudRate = 115200;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    
    Error_Handler();
  }
  /* USER CODE BEGIN UART10_Init 2 */

  /* USER CODE END UART10_Init 2 */
}


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
 * @brief UART MSP初始化 - 使用board.h中的宏定义
 * @note 如需更改UART引脚，只需修改User/board/board.h中的宏定义
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (uartHandle->Instance == USART1)
  {
    /* USART1 clock enable */
    BOARD_UART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USART1 TX GPIO */
    GPIO_InitStruct.Pin = BOARD_UART1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART1_TX_AF;
    HAL_GPIO_Init(BOARD_UART1_TX_PORT, &GPIO_InitStruct);

    /* USART1 RX GPIO */
    GPIO_InitStruct.Pin = BOARD_UART1_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART1_RX_AF;
    HAL_GPIO_Init(BOARD_UART1_RX_PORT, &GPIO_InitStruct);

    /* USART1 DMA Init */
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
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    BOARD_UART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART2_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART2_TX_AF;
    HAL_GPIO_Init(BOARD_UART2_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART2_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART2_RX_AF;
    HAL_GPIO_Init(BOARD_UART2_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  else if (uartHandle->Instance == USART3)
  {
    BOARD_UART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART3_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART3_TX_AF;
    HAL_GPIO_Init(BOARD_UART3_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART3_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART3_RX_AF;
    HAL_GPIO_Init(BOARD_UART3_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
  else if (uartHandle->Instance == UART4)
  {
    BOARD_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART4_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART4_TX_AF;
    HAL_GPIO_Init(BOARD_UART4_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART4_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART4_RX_AF;
    HAL_GPIO_Init(BOARD_UART4_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart4_rx);
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if (uartHandle->Instance == UART5)
  {
    BOARD_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART5_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART5_TX_AF;
    HAL_GPIO_Init(BOARD_UART5_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART5_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART5_RX_AF;
    HAL_GPIO_Init(BOARD_UART5_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart5_rx);
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  }
  else if (uartHandle->Instance == USART6)
  {
    BOARD_UART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART6_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART6_TX_AF;
    HAL_GPIO_Init(BOARD_UART6_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART6_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART6_RX_AF;
    HAL_GPIO_Init(BOARD_UART6_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart6_rx);
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
  else if (uartHandle->Instance == UART7)
  {
    BOARD_UART7_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART7_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART7_TX_AF;
    HAL_GPIO_Init(BOARD_UART7_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART7_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART7_RX_AF;
    HAL_GPIO_Init(BOARD_UART7_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart7_rx);
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  }
  else if (uartHandle->Instance == UART8)
  {
    
    BOARD_UART8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART8_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART8_TX_AF;
    HAL_GPIO_Init(BOARD_UART8_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART8_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART8_RX_AF;
    HAL_GPIO_Init(BOARD_UART8_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart8_rx);
    HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
  }
  else if (uartHandle->Instance == UART9)
  {
    BOARD_UART9_CLK_ENABLE();        /* 使能 UART9 APB2 时钟，B 通道 RFID 独立串口才能收发。 */
    __HAL_RCC_GPIOD_CLK_ENABLE();    /* UART9 使用 PD14/PD15，必须先打开 GPIOD 时钟。 */

    GPIO_InitStruct.Pin = BOARD_UART9_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART9_TX_AF;
    HAL_GPIO_Init(BOARD_UART9_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART9_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART9_RX_AF;
    HAL_GPIO_Init(BOARD_UART9_RX_PORT, &GPIO_InitStruct);

    hdma_uart9_rx.Instance = BOARD_UART9_DMA_RX;
    hdma_uart9_rx.Init.Channel = BOARD_UART9_DMA_CHANNEL;
    hdma_uart9_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart9_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart9_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart9_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart9_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart9_rx.Init.Mode = DMA_CIRCULAR;    /* RFID 回包长度不固定，沿用 UART3 的环形 DMA 接收模式。 */
    hdma_uart9_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart9_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart9_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart9_rx);
    HAL_NVIC_SetPriority(UART9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
  }
  else if (uartHandle->Instance == UART10)
  {
    /*
     * UART10 的 TX/RX 引脚在 board.h 中定义为 `PE3/PE2`。
     * 因此这里必须使能 `GPIOE` 时钟，才能把这两个引脚正确配置成 UART10 复用功能。
     * 之前误开成了 `GPIOB` 时钟，会导致 `PE3/PE2` 实际没有完成串口复用初始化，
     * 现场现象就是程序里虽然调用了 `HAL_UART_Transmit(&huart10, ...)`，
     * 但外部串口工具收不到任何报文。
     */
    BOARD_UART10_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = BOARD_UART10_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BOARD_UART10_TX_AF;
    HAL_GPIO_Init(BOARD_UART10_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOARD_UART10_RX_PIN;
    GPIO_InitStruct.Alternate = BOARD_UART10_RX_AF;
    HAL_GPIO_Init(BOARD_UART10_RX_PORT, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_uart10_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart10_rx);
    HAL_NVIC_SetPriority(UART10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART10_IRQn);
  }
}

/**
 * @brief UART MSP反初始化 - 使用board.h中的宏定义
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART1_TX_PORT, BOARD_UART1_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART1_RX_PORT, BOARD_UART1_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART2_TX_PORT, BOARD_UART2_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART2_RX_PORT, BOARD_UART2_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART3_TX_PORT, BOARD_UART3_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART3_RX_PORT, BOARD_UART3_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
  else if (uartHandle->Instance == UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART4_TX_PORT, BOARD_UART4_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART4_RX_PORT, BOARD_UART4_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if (uartHandle->Instance == UART5)
  {
    __HAL_RCC_UART5_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART5_TX_PORT, BOARD_UART5_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART5_RX_PORT, BOARD_UART5_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  }
  else if (uartHandle->Instance == USART6)
  {
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART6_TX_PORT, BOARD_UART6_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART6_RX_PORT, BOARD_UART6_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  }
  else if (uartHandle->Instance == UART7)
  {
    __HAL_RCC_UART7_CLK_DISABLE();
    HAL_GPIO_DeInit(BOARD_UART7_TX_PORT, BOARD_UART7_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART7_RX_PORT, BOARD_UART7_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  }
  else if (uartHandle->Instance == UART9)
  {
    __HAL_RCC_UART9_CLK_DISABLE(); /* 释放 B 通道 RFID 串口时钟，低功耗或重初始化时使用。 */
    HAL_GPIO_DeInit(BOARD_UART9_TX_PORT, BOARD_UART9_TX_PIN);
    HAL_GPIO_DeInit(BOARD_UART9_RX_PORT, BOARD_UART9_RX_PIN);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART9_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
