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

void IIC_Init(void);
void IIC_Reset(void);

uint8_t IIC_WriteRead_Test(void);

uint8_t IIC_AT24CXX_ReadOneByte(uint16_t ReadAddr);
void IIC_AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite);

#endif //__IIC_H



