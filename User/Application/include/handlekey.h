#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include "bsp_gpio.h"

/*
 * 函数功能：创建并启动手柄实体按键 30ms 周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void);

#define HANDLE_RUN_KEY_A_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_A_PORT, BOARD_RES_HANDLE_RUN_KEY_A_PIN) /* GPIO模式读取A通道PE2实体键，低电平表示按下。 */
#define HANDLE_RUN_KEY_B_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_B_PORT, BOARD_RES_HANDLE_RUN_KEY_B_PIN) /* GPIO模式读取B通道PE0实体键，低电平表示按下。 */

#endif  //__HANDLEKEY_H
