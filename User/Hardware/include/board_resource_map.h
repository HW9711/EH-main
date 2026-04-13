#ifndef __BOARD_RESOURCE_MAP_H
#define __BOARD_RESOURCE_MAP_H

#include "board_profile.h"

/*
 * Stable semantic resource map.
 * Future hardware changes should prefer updating these mappings first.
 */

#define BOARD_RES_STATE_LED_PORT          BOARD_STATE_LED_PORT
#define BOARD_RES_STATE_LED_PIN           BOARD_STATE_LED_PIN

#define BOARD_RES_HANDLESCAN_A_SHORT_PORT GPIOD
#define BOARD_RES_HANDLESCAN_A_SHORT_PIN  GPIO_PIN_1
#define BOARD_RES_HANDLESCAN_B_SHORT_PORT GPIOD
#define BOARD_RES_HANDLESCAN_B_SHORT_PIN  GPIO_PIN_0

#define BOARD_RES_HANDLE_KEY0_PORT        GPIOC
#define BOARD_RES_HANDLE_KEY0_PIN         GPIO_PIN_10
#define BOARD_RES_HANDLE_KEY1_PORT        GPIOC
#define BOARD_RES_HANDLE_KEY1_PIN         GPIO_PIN_11

#endif /* __BOARD_RESOURCE_MAP_H */
