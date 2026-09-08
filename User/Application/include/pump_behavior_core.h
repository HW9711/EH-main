#ifndef __PUMP_BEHAVIOR_CORE_H
#define __PUMP_BEHAVIOR_CORE_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* PumpBehaviorChannel_t 只表示逻辑 A/B 通道，不能用它推断物理 UART 或压力传感器接线。 */
typedef enum
{
    PUMP_BEHAVIOR_CHANNEL_A = 0U, /* 逻辑 A 泵：读写 pumpMessageA，并更新屏幕 A 泵区域。 */
    PUMP_BEHAVIOR_CHANNEL_B = 1U, /* 逻辑 B 泵：读写 pumpMessageB，并更新屏幕 B 泵区域。 */
    PUMP_BEHAVIOR_CHANNEL_COUNT   /* 通道数量仅用于检查数组边界，不能作为有效通道传入。 */
} PumpBehaviorChannel_t;

/* PumpBehaviorMessage_t 保持原 A/B 队列消息布局，任务收到消息后仍只更新速度，不改设备识别出的泵类型。 */
typedef struct
{
    uint8_t pump_type; /* 保留原消息字段；当前接收端不使用该字段覆盖泵类型。 */
    uint16_t Value;    /* 新设定值：注水/灌注为 mL/min，抽吸为 0~15 的内部设定，不是协议速度。 */
} PumpBehaviorMessage_t;

/* 保存步进驱动最近一帧 CRC 正确的反馈；复制函数保证字段不会来自两次不同更新。 */
typedef struct
{
    uint32_t feedback_tick_ms; /* 主控完成本帧校验的 HAL 毫秒时刻，用于判断现场回包新旧。 */
    uint16_t sequence;         /* 每收到一帧 CRC 正确回包递增一次，16 位自然回绕。 */
    uint16_t actual_speed;     /* 驱动回包 byte2~3 的实际速度原始值，高字节在前。 */
    uint8_t direction;         /* 驱动回包 byte1 的实际方向原始值。 */
    uint8_t raw_error;         /* 驱动回包 byte4 的原始故障码，0 表示当前无故障。 */
    uint8_t valid;             /* 已收到至少一帧 CRC 正确回包时为 1。 */
} PumpDriverFeedbackSnapshot_t;

/*
 * 函数功能：每 25ms 处理指定泵的速度请求、驱动故障、压力提示和排空计时，再发送命令并刷新显示。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue);

/*
 * 函数功能：复制指定泵最近的有效驱动反馈，保证各字段来自同一次更新。
 * 输入参数：channel 为逻辑 A/B 通道；snapshot 指向调用方提供的结果缓存。
 * 返回参数：已有有效反馈且复制成功返回 1；否则返回 0，通道无效或连续三次遇到更新时清空结果。
 */
uint8_t PumpBehavior_CopyDriverFeedbackSnapshot(PumpBehaviorChannel_t channel,
                                                PumpDriverFeedbackSnapshot_t *snapshot);

#endif /* __PUMP_BEHAVIOR_CORE_H */
