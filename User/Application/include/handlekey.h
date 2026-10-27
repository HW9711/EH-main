#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include "bsp_gpio.h"
#include <stdbool.h> /* 串口中断处理接口返回是否由临时测试接收，头文件自行提供 bool 定义。 */

/* GZYT 临时支持开关：1 允许 A/B 识别上线并开启 HAL4 正转、HAL5 反转；0 拒绝 GZYT 上线、关闭收帧和测试启动，恢复原 30ms 按键任务。 */
#ifndef GZYT_DIGITAL_TEST_ENABLE
#define GZYT_DIGITAL_TEST_ENABLE 1U
#endif

/*
 * 函数功能：创建手柄按键任务；GPIO 每 30ms 扫描，开启 GZYT 临时测试后每 10ms 处理串口。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void);

/*
 * 函数功能：把 GZYT 按钮串口中断收到的字节和接收时刻存入本通道缓存；中断内不修改电机请求。
 * 输入参数：channel=1 对应 A/UART10/PE2，channel=2 对应 B/UART8/PE0。
 * 返回参数：true 表示测试接收已处理该串口；false 表示测试未启用，由调用方继续交给 HAL。
 */
bool HandleKey_UartIrqHandler(uint8_t channel);

#define HANDLE_RUN_KEY_A_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_A_PORT, BOARD_RES_HANDLE_RUN_KEY_A_PIN) /* 读取 A 键当前电平：RESET 为按下，SET 为松开；仅 GPIO 模式使用，仍需连续确认。 */
#define HANDLE_RUN_KEY_B_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_B_PORT, BOARD_RES_HANDLE_RUN_KEY_B_PIN) /* 读取 B 键当前电平：RESET 为按下，SET 为松开；仅 GPIO 模式使用，仍需连续确认。 */

#endif  //__HANDLEKEY_H
