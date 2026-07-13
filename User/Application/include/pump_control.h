#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <stdint.h>

/*
 * 函数功能：统一处理屏幕、脚踏和外控发送的 A/B 泵档位、启停、调速与轻排按键。
 * 输入参数：key_value 为 Pubinterface 中定义的泵业务按键编号。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value);

#endif
