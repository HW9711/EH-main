#ifndef _FLASH_H
#define _FLASH_H

#include <stdint.h>

#define ADDR_FLASH_SERTOR_0  (0x8000000)
#define ADDR_FLASH_SERTOR_1  (0x8004000)  //16K
#define ADDR_FLASH_SERTOR_2  (0x8008000)  //16K
#define ADDR_FLASH_SERTOR_3  (0x800C000)  //16K
#define ADDR_FLASH_SERTOR_4  (0x8010000)  //16K
#define ADDR_FLASH_SERTOR_5  (0x8020000)  //64K
#define ADDR_FLASH_SERTOR_6  (0x8040000)  //128K
#define ADDR_FLASH_SERTOR_7  (0x8060000)  //128K
#define ADDR_FLASH_SERTOR_8  (0x8080000)  //128K
#define ADDR_FLASH_SERTOR_9  (0x80A0000)  //128K
#define ADDR_FLASH_SERTOR_10  (0x80C0000)  //128K
#define ADDR_FLASH_SERTOR_11  (0x80E0000)  //128K

int8_t Flash_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint16_t NumToWrite);

void Flash_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint16_t NumToRead);

#endif












