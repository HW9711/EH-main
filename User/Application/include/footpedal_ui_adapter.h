#ifndef __FOOTPEDAL_UI_ADAPTER_H
#define __FOOTPEDAL_UI_ADAPTER_H

#include <stdint.h>

/*
 * 函数功能：切换手柄时清除旧脚踏选框记录；新屏没有这个选框，不发送旧屏命令。
 * 输入参数：num 为旧选框编号，0xFF 表示清除当前及上次编号；当前无选框时直接返回。
 * 返回参数：无。
 */
void FootPedal_SelectWin(uint8_t num);

#endif /* __FOOTPEDAL_UI_ADAPTER_H */
