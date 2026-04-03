//handlekey.h

#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include <stdint.h>
#include <stdbool.h>

bool HandleKey_GetKeyValue(uint8_t keynum);

void HandleKeyScan_Init(void);

#define KEY0_STATUS()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)
#define KEY1_STATUS()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) 

//#define KEY0_H PCin(10)
//#define KEY1_H PDin(1)

#endif  //__HANDLEKEY_H



