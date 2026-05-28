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

#define BOARD_RES_HANDLE_RUN_KEY_A_PORT   GPIOE       /* A通道手柄实体运行键，硬件默认上拉，按下为低电平。 */
#define BOARD_RES_HANDLE_RUN_KEY_A_PIN    GPIO_PIN_12 /* A通道手柄实体运行键接PE12，轮询读取电平保持运行。 */
#define BOARD_RES_HANDLE_RUN_KEY_B_PORT   GPIOE       /* B通道手柄实体运行键，硬件默认上拉，按下为低电平。 */
#define BOARD_RES_HANDLE_RUN_KEY_B_PIN    GPIO_PIN_13 /* B通道手柄实体运行键接PE13，轮询读取电平保持运行。 */

#define BOARD_RES_HANDLE_KEY0_PORT        BOARD_RES_HANDLE_RUN_KEY_A_PORT /* 兼容旧KEY0命名，实际指向A通道实体运行键。 */
#define BOARD_RES_HANDLE_KEY0_PIN         BOARD_RES_HANDLE_RUN_KEY_A_PIN  /* 兼容旧KEY0命名，避免旧宏继续指向PC10。 */
#define BOARD_RES_HANDLE_KEY1_PORT        BOARD_RES_HANDLE_RUN_KEY_B_PORT /* 兼容旧KEY1命名，实际指向B通道实体运行键。 */
#define BOARD_RES_HANDLE_KEY1_PIN         BOARD_RES_HANDLE_RUN_KEY_B_PIN  /* 兼容旧KEY1命名，避免旧宏继续指向PC11。 */

#define BOARD_RES_SOFT_IIC_SCL_PORT       BOARD_IIC_SCL_PORT
#define BOARD_RES_SOFT_IIC_SCL_PIN        BOARD_IIC_SCL_PIN
#define BOARD_RES_SOFT_IIC_SDA_PORT       BOARD_IIC_SDA_PORT
#define BOARD_RES_SOFT_IIC_SDA_PIN        BOARD_IIC_SDA_PIN

#define BOARD_RES_ONEWIRE_I_PORT          BOARD_ONEWIRE_I_PORT
#define BOARD_RES_ONEWIRE_I_PIN           BOARD_ONEWIRE_I_PIN
#define BOARD_RES_ONEWIRE_II_PORT         BOARD_ONEWIRE_II_PORT
#define BOARD_RES_ONEWIRE_II_PIN          BOARD_ONEWIRE_II_PIN

#endif /* __BOARD_RESOURCE_MAP_H */
