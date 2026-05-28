//handlekey.h

#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_gpio.h"

bool HandleKey_GetKeyValue(uint8_t keynum);

void HandleKeyScan_Init(void);

#define HANDLE_RUN_KEY_A_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_A_PORT, BOARD_RES_HANDLE_RUN_KEY_A_PIN) /* 读取A通道实体运行键，低电平表示按下。 */
#define HANDLE_RUN_KEY_B_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_B_PORT, BOARD_RES_HANDLE_RUN_KEY_B_PIN) /* 读取B通道实体运行键，低电平表示按下。 */

#define KEY0_STATUS()  HANDLE_RUN_KEY_A_STATUS() /* 兼容旧KEY0调用，实际读取A通道实体运行键。 */
#define KEY1_STATUS()  HANDLE_RUN_KEY_B_STATUS() /* 兼容旧KEY1调用，实际读取B通道实体运行键。 */

//#define KEY0_H PCin(10)
//#define KEY1_H PDin(1)

#endif  //__HANDLEKEY_H



