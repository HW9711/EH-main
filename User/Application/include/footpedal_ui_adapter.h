#ifndef __FOOTPEDAL_UI_ADAPTER_H
#define __FOOTPEDAL_UI_ADAPTER_H

#include <stdint.h>

/*
 * 脚踏旧任务移除后，主 UI 仍会在手柄切换时调用这个入口复位脚踏选框。
 * 该头文件只暴露这一个 UI 适配接口，避免重新引入旧脚踏任务声明。
 */
void FootPedal_SelectWin(uint8_t num);

#endif /* __FOOTPEDAL_UI_ADAPTER_H */
