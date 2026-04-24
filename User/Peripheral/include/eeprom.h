#ifndef __EEPROM_H
#define __EEPROM_H

#include <stdint.h>

void EEPROM_AT24CXX_Init(void); //初始化IIC

void EEPROM_AT24CXX_Write(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToWrite);

void EEPROM_AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);

#endif
