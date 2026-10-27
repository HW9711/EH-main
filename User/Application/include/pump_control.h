#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <stdint.h>

/*
 * 函数功能：统一处理屏幕、脚踏和外控发送的 A/B 泵档位、启停、调速与轻排按键。
 * 输入参数：key_value 为 Pubinterface 中定义的泵业务按键编号。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value);

/*
 * 函数功能：供简易外控按屏幕步进循环增加流量，已到上限时回到配置下限。
 * 输入参数：screen_key 为 SCREENKey_APUMP_Add 或 SCREENKey_BPUMP_Add。
 * 返回参数：接受调节返回1；键值无效或当前状态不允许调节时返回0。
 */
uint8_t Pump_CycleFlow(uint8_t screen_key);

#endif
