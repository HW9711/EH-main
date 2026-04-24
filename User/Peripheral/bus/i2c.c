/**
 * @file i2c.c
 * @brief 硬件I2C驱动实现文件 - I2C2和I2C3外设初始化及读写接口
 *        基于STM32 HAL库实现，不依赖CubeMX生成代码
 *        GPIO与时钟配置在本驱动内完成，不再依赖board.c/board.h中的I2C配置
 */
#include "bsp_i2c_bus.h"
#include "main.h"

/* I2C2引脚定义：PB10(SCL,AF4) + PB3(SDA,AF9) */
#define I2C2_SCL_PORT        GPIOB
#define I2C2_SCL_PIN         GPIO_PIN_10
#define I2C2_SCL_AF          GPIO_AF4_I2C2
#define I2C2_SDA_PORT        GPIOB
#define I2C2_SDA_PIN         GPIO_PIN_3
#define I2C2_SDA_AF          GPIO_AF9_I2C2

/* I2C3引脚定义：PA8(SCL,AF4) + PC9(SDA,AF4) */
#define I2C3_SCL_PORT        GPIOA
#define I2C3_SCL_PIN         GPIO_PIN_8
#define I2C3_SCL_AF          GPIO_AF4_I2C3
#define I2C3_SDA_PORT        GPIOC
#define I2C3_SDA_PIN         GPIO_PIN_9
#define I2C3_SDA_AF          GPIO_AF4_I2C3

/*============================================================================
 * I2C句柄定义
 *============================================================================*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

/*============================================================================
 * I2C2外设初始化
 *============================================================================*/
void MX_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = I2C2_CLOCK_SPEED;
    hi2c2.Init.DutyCycle       = I2C2_DUTY_CYCLE;
    hi2c2.Init.OwnAddress1     = I2C2_OWN_ADDRESS;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

/*============================================================================
 * I2C3外设初始化
 *============================================================================*/
void MX_I2C3_Init(void)
{
    hi2c3.Instance             = I2C3;
    hi2c3.Init.ClockSpeed      = I2C3_CLOCK_SPEED;
    hi2c3.Init.DutyCycle       = I2C3_DUTY_CYCLE;
    hi2c3.Init.OwnAddress1     = I2C3_OWN_ADDRESS;
    hi2c3.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2     = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }
}

/*============================================================================
 * I2C统一初始化 (I2C2 + I2C3)
 *============================================================================*/
void MX_I2C_Init(void)
{
    MX_I2C2_Init();
    MX_I2C3_Init();
}

/*============================================================================
 * HAL_I2C_MspInit - HAL库I2C底层初始化回调
 * 说明: HAL_I2C_Init()内部会调用此函数，在这里完成I2C2/I2C3的GPIO和时钟配置
 *============================================================================*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C2)
    {
        /* 使能I2C2与GPIOB时钟 */
        __HAL_RCC_I2C2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* 配置I2C2 SCL: PB10, AF4, 开漏上拉 */
        GPIO_InitStruct.Pin = I2C2_SCL_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = I2C2_SCL_AF;
        HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);

        /* 配置I2C2 SDA: PB3, AF9, 开漏上拉 */
        GPIO_InitStruct.Pin = I2C2_SDA_PIN;
        GPIO_InitStruct.Alternate = I2C2_SDA_AF;
        HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);
    }
    else if (hi2c->Instance == I2C3)
    {
        /* 使能I2C3与对应GPIO时钟 */
        __HAL_RCC_I2C3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* 配置I2C3 SCL: PA8, AF4, 开漏上拉 */
        GPIO_InitStruct.Pin = I2C3_SCL_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = I2C3_SCL_AF;
        HAL_GPIO_Init(I2C3_SCL_PORT, &GPIO_InitStruct);

        /* 配置I2C3 SDA: PC9, AF4, 开漏上拉 */
        GPIO_InitStruct.Pin = I2C3_SDA_PIN;
        GPIO_InitStruct.Alternate = I2C3_SDA_AF;
        HAL_GPIO_Init(I2C3_SDA_PORT, &GPIO_InitStruct);
    }
}

/*============================================================================
 * HAL_I2C_MspDeInit - HAL库I2C底层反初始化回调
 *============================================================================*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(I2C2_SCL_PORT, I2C2_SCL_PIN);
        HAL_GPIO_DeInit(I2C2_SDA_PORT, I2C2_SDA_PIN);
    }
    else if (hi2c->Instance == I2C3)
    {
        __HAL_RCC_I2C3_CLK_DISABLE();
        HAL_GPIO_DeInit(I2C3_SCL_PORT, I2C3_SCL_PIN);
        HAL_GPIO_DeInit(I2C3_SDA_PORT, I2C3_SDA_PIN);
    }
}

/*============================================================================
 * I2C2 读写接口实现
 *============================================================================*/
HAL_StatusTypeDef I2C2_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Transmit(&hi2c2, DevAddr, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C2_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Receive(&hi2c2, DevAddr, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C2_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                  uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Write(&hi2c2, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C2_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(&hi2c2, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C2_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, DevAddr, Trials, Timeout);
}

/*============================================================================
 * I2C3 读写接口实现
 *============================================================================*/
HAL_StatusTypeDef I2C3_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Transmit(&hi2c3, DevAddr, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C3_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Receive(&hi2c3, DevAddr, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C3_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                  uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Write(&hi2c3, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C3_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(&hi2c3, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C3_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout)
{
    return HAL_I2C_IsDeviceReady(&hi2c3, DevAddr, Trials, Timeout);
}
