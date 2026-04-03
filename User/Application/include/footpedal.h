//footpedal.h

#ifndef __FOOTPEDAL_H
#define __FOOTPEDAL_H

#include <stdint.h>

//============================================================================
// 函数名称: Display_PedalSelectWin()
// 功能描述: 切换手柄，复位脚踏设置选中框
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void FootPedal_SelectWin(uint8_t Num);

//============================================================================
void FootPedalTask_Init(void);

void FootThrottleTask_Init(void);

#endif   //__FOORPEDAL_H



