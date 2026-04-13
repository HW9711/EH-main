//handlekey.h

#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_gpio.h"

bool HandleKey_GetKeyValue(uint8_t keynum);

void HandleKeyScan_Init(void);

#define KEY0_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_KEY0_PORT, BOARD_RES_HANDLE_KEY0_PIN)
#define KEY1_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_KEY1_PORT, BOARD_RES_HANDLE_KEY1_PIN) 

//#define KEY0_H PCin(10)
//#define KEY1_H PDin(1)

#endif  //__HANDLEKEY_H



