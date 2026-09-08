#ifndef WORK_ALARM_H
#define WORK_ALARM_H

#include <stdbool.h>
#include <stdint.h>

/* 主控内部报警编号，不是屏幕图片号，也不是可调整的门限；改号必须同步屏幕、蜂鸣和外控处理。 */
#define WORK_ALARM_NONE                  0U /* 无报警，也用于停止报警蜂鸣。 */
#define WORK_ALARM_HANDLE_NOT_CONNECTED  1U /* 手柄未连接或运行中拔出，屏幕使用 80 号图。 */
#define WORK_ALARM_MANUAL_SELECTED       2U /* 当前选中手控却尝试脚控启动，提示使用手柄按键。 */
#define WORK_ALARM_FOOT_SELECTED         3U /* 当前选中脚控却尝试手控启动，提示使用脚踏。 */
#define WORK_ALARM_MOTOR_OVERLOAD        4U /* 原电机过载编号，显示 87 号图。 */
#define WORK_ALARM_MOTOR_OVERLOAD_ALT    5U /* 驱动过流/堵转使用的过载编号，同样可显示 87 号图。 */
#define WORK_ALARM_FOOT_VALUE_ERROR      6U /* 脚踏存储的校准值错误。 */
#define WORK_ALARM_UID_ERROR             7U /* 手柄 UID 校验错误。 */
#define WORK_ALARM_MOTOR_COMM_ERROR      8U /* 历史名称为通信错误；当前驱动过压/欠压也映射到此编号。 */
#define WORK_ALARM_HALL_ERROR            9U /* 驱动报告霍尔断线或霍尔学习错误。 */
#define WORK_ALARM_HANDLE_MODEL_ERROR_A  10U /* A 通道手柄参数校验异常。 */
#define WORK_ALARM_HANDLE_MODEL_ERROR    WORK_ALARM_HANDLE_MODEL_ERROR_A /* 旧单通道名称，仍与 A 通道编号相同。 */
#define WORK_ALARM_SPEED_THRESHOLD       11U /* 速度/频率阈值提示的旧编号，与下方驱动板故障共用 11。 */
#define WORK_ALARM_MOTOR_DRIVER_BOARD    11U /* 驱动板故障编号；具体故障要结合原始 Err 判断。 */
#define WORK_ALARM_HANDLE_MODEL_ERROR_B  12U /* B 通道手柄参数校验异常。 */
#define WORK_ALARM_HANDLE_MODEL_ERROR_AB 14U /* A/B 两通道手柄参数均校验异常。 */
#define WORK_ALARM_PUMP_PRESSURE_BLOCKED 15U /* 泵压力达到报警条件，显示 89 号图；编号本身不表示停泵动作。 */

void WorkAlarm_Set(uint8_t alarm_value); /* 写入报警码并同步报警有效标志。 */
void WorkAlarm_Clear(void); /* 清除当前工作报警。 */
void WorkAlarm_ClearIf(uint8_t alarm_value); /* 只清除调用方负责的指定报警。 */
bool WorkAlarm_Is(uint8_t alarm_value); /* 判断当前是否为指定报警。 */

#endif
