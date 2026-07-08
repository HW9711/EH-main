//iic.h

#ifndef __IIC_H
#define __IIC_H

#include <stdint.h>

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	  8191
#define AT24C128	16383
#define AT24C256	32767

//Mini STM32开发板使用的是24c02，所以定义EE_TYPE为AT24C02
#define EE_TYPE AT24C02

#define AT24C32_IIC_WRITE_ADDR 0xA0U  /* 主控板 AT24C32 A0/A1/A2 接地，8 位写地址固定为 0xA0。 */
#define AT24C32_IIC_READ_ADDR  0xA1U  /* 主控板 AT24C32 A0/A1/A2 接地，8 位读地址固定为 0xA1。 */
#define AT24C32_IIC_PAGE_SIZE  32U    /* AT24C32 页写入边界为 32 字节，跨页必须拆成多次写。 */

void IIC_Init(void);
void IIC_Reset(void);

uint8_t IIC_WriteRead_Test(void);

uint8_t IIC_AT24CXX_ReadOneByte(uint16_t ReadAddr);
void IIC_AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite);
uint8_t IIC_AT24C32_ReadBytes(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
uint8_t IIC_AT24C32_WriteBytes(uint16_t WriteAddr, const uint8_t *pBuffer, uint16_t NumToWrite);

#endif //__IIC_H



