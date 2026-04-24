/**
 * @file i2c.h
 * @brief 硬件I2C驱动头文件 - I2C2和I2C3外设初始化及读写接口
 *        基于STM32 HAL库实现，不依赖CubeMX生成代码
 */
#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*============================================================================
 * I2C句柄声明
 *============================================================================*/
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/*============================================================================
 * I2C配置参数
 *============================================================================*/

/* I2C2默认配置 */
#define I2C2_CLOCK_SPEED        100000U     /* I2C2时钟速率: 100KHz (标准模式) */
#define I2C2_DUTY_CYCLE         I2C_DUTYCYCLE_2
#define I2C2_OWN_ADDRESS        0x00        /* I2C2自身地址 */

/* I2C3默认配置 */
#define I2C3_CLOCK_SPEED        100000U     /* I2C3时钟速率: 100KHz (标准模式) */
#define I2C3_DUTY_CYCLE         I2C_DUTYCYCLE_2
#define I2C3_OWN_ADDRESS        0x00        /* I2C3自身地址 */

/* I2C超时时间 (ms) */
#define I2C_TIMEOUT_DEFAULT     100U

/*============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief  I2C2和I2C3外设初始化
 * @note   GPIO初始化已在board.c中完成，此函数仅配置I2C外设寄存器
 * @retval None
 */
void MX_I2C_Init(void);

/**
 * @brief  I2C2外设初始化
 * @retval None
 */
void MX_I2C2_Init(void);

/**
 * @brief  I2C3外设初始化
 * @retval None
 */
void MX_I2C3_Init(void);

/*============================================================================
 * I2C读写接口 - I2C2
 *============================================================================*/

/**
 * @brief  I2C2主机发送数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  pData: 发送数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C2_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C2主机接收数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  pData: 接收数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C2_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C2向指定寄存器写数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  MemAddr: 寄存器地址
 * @param  MemAddrSize: 寄存器地址大小 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData: 发送数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C2_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                  uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C2从指定寄存器读数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  MemAddr: 寄存器地址
 * @param  MemAddrSize: 寄存器地址大小 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData: 接收数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C2_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  检测I2C2设备是否就绪
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  Trials: 重试次数
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C2_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout);

/*============================================================================
 * I2C读写接口 - I2C3
 *============================================================================*/

/**
 * @brief  I2C3主机发送数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  pData: 发送数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C3_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C3主机接收数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  pData: 接收数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C3_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C3向指定寄存器写数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  MemAddr: 寄存器地址
 * @param  MemAddrSize: 寄存器地址大小 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData: 发送数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C3_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                  uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  I2C3从指定寄存器读数据
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  MemAddr: 寄存器地址
 * @param  MemAddrSize: 寄存器地址大小 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData: 接收数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C3_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief  检测I2C3设备是否就绪
 * @param  DevAddr: 目标设备地址 (7位地址左移1位)
 * @param  Trials: 重试次数
 * @param  Timeout: 超时时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C3_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
