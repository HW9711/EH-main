#ifndef BSP_I2C_BUS_H
#define BSP_I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/* i2c.c实际使用这里的配置；Peripheral/include/i2c.h保留了同名旧定义，不能只改旧头文件。 */
#define I2C2_CLOCK_SPEED        100000U /* I2C2总线时钟，单位Hz，当前100kHz；改后影响该接口EEPROM读写速度，须按器件和接线验证。 */
#define I2C2_DUTY_CYCLE         I2C_DUTYCYCLE_2 /* HAL时钟占空比选项；不控制业务任务周期。 */
#define I2C2_OWN_ADDRESS        0x00 /* 主控自身的I2C地址，不是手柄EEPROM的0xA0设备地址。 */

#define I2C3_CLOCK_SPEED        100000U /* I2C3总线时钟，单位Hz，独立于I2C2设置；与屏幕A/B的对应关系还取决于接口交换配置。 */
#define I2C3_DUTY_CYCLE         I2C_DUTYCYCLE_2 /* I2C3的HAL时钟占空比选项。 */
#define I2C3_OWN_ADDRESS        0x00 /* 主控自身地址，不是从设备地址。 */

#define I2C_TIMEOUT_DEFAULT     100U /* 保留的默认毫秒数，当前接口使用传入的Timeout；单改本宏不会改变实际超时。 */

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
