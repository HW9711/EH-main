#ifndef BSP_I2C_BUS_H
#define BSP_I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

#define I2C2_CLOCK_SPEED        100000U
#define I2C2_DUTY_CYCLE         I2C_DUTYCYCLE_2
#define I2C2_OWN_ADDRESS        0x00

#define I2C3_CLOCK_SPEED        100000U
#define I2C3_DUTY_CYCLE         I2C_DUTYCYCLE_2
#define I2C3_OWN_ADDRESS        0x00

#define I2C_TIMEOUT_DEFAULT     100U

void MX_I2C_Init(void);
void MX_I2C2_Init(void);
void MX_I2C3_Init(void);

HAL_StatusTypeDef I2C2_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C2_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C2_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C2_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C2_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout);

HAL_StatusTypeDef I2C3_Master_Transmit(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C3_Master_Receive(uint16_t DevAddr, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C3_Mem_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                 uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C3_Mem_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize,
                                uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C3_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif
