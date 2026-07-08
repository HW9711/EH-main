#ifndef __BOARD_PROFILE_H
#define __BOARD_PROFILE_H

#include "board.h"

/*
 * Board profile shell.
 * Keep board.h as current source of truth during migration.
 */

#define BOARD_PROFILE_HAS_K1K2            BOARD_HAS_K1K2
#define RFID_USE_DUAL_UART_MODE           1U  /* RFID 通道模式：1 表示 UART3=A、UART9=B，0 表示保留 UART3+R200-K8 切换。 */

#endif /* __BOARD_PROFILE_H */
