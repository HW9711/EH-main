#ifndef WORK_ALARM_H
#define WORK_ALARM_H

#include <stdbool.h>
#include <stdint.h>

/* 报警码必须与屏幕图片、蜂鸣和外控协议保持一致，不能按模块自行重新编号。 */
#define WORK_ALARM_NONE                  0U
#define WORK_ALARM_HANDLE_NOT_CONNECTED  1U
#define WORK_ALARM_MANUAL_SELECTED       2U
#define WORK_ALARM_FOOT_SELECTED         3U
#define WORK_ALARM_MOTOR_OVERLOAD        4U
#define WORK_ALARM_MOTOR_OVERLOAD_ALT    5U
#define WORK_ALARM_FOOT_VALUE_ERROR      6U
#define WORK_ALARM_UID_ERROR             7U
#define WORK_ALARM_MOTOR_COMM_ERROR      8U
#define WORK_ALARM_HALL_ERROR            9U
#define WORK_ALARM_HANDLE_MODEL_ERROR_A  10U
#define WORK_ALARM_HANDLE_MODEL_ERROR    WORK_ALARM_HANDLE_MODEL_ERROR_A
#define WORK_ALARM_SPEED_THRESHOLD       11U
#define WORK_ALARM_MOTOR_DRIVER_BOARD    11U
#define WORK_ALARM_HANDLE_MODEL_ERROR_B  12U
#define WORK_ALARM_HANDLE_MODEL_ERROR_AB 14U
#define WORK_ALARM_PUMP_PRESSURE_BLOCKED 15U

void WorkAlarm_Set(uint8_t alarm_value); /* 写入报警码并同步报警有效标志。 */
void WorkAlarm_Clear(void); /* 清除当前工作报警。 */
void WorkAlarm_ClearIf(uint8_t alarm_value); /* 只清除调用方负责的指定报警。 */
bool WorkAlarm_Is(uint8_t alarm_value); /* 判断当前是否为指定报警。 */

#endif
