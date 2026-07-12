#ifndef __PUMP_BEHAVIOR_CORE_H
#define __PUMP_BEHAVIOR_CORE_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* PumpBehaviorChannel_t 只表示逻辑 A/B 通道，不能用它推断物理 UART 或压力传感器接线。 */
typedef enum
{
    PUMP_BEHAVIOR_CHANNEL_A = 0U, /* 逻辑 A 泵：使用 pumpMessageA、A 屏幕状态和 A 输出发布入口。 */
    PUMP_BEHAVIOR_CHANNEL_B = 1U, /* 逻辑 B 泵：使用 pumpMessageB、B 屏幕状态和 B 输出发布入口。 */
    PUMP_BEHAVIOR_CHANNEL_COUNT   /* 通道数量仅用于检查数组边界，不能作为有效通道传入。 */
} PumpBehaviorChannel_t;

/* PumpBehaviorMessage_t 保持原 A/B 队列消息布局，任务收到消息后仍只更新速度，不改设备识别出的泵类型。 */
typedef struct
{
    uint8_t pump_type; /* 保留原消息字段；当前接收端不使用该字段覆盖泵类型。 */
    uint16_t Value;    /* 本次控制请求给出的业务速度，单位沿用 pumpMessageA/B.speed_work。 */
} PumpBehaviorMessage_t;

/*
 * 函数功能：执行指定逻辑泵的一次 25ms 行为周期，处理队列速度、压力保护、排空、显示和 UART 下发。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue);

#endif /* __PUMP_BEHAVIOR_CORE_H */
