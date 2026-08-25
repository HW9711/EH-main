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

/* PumpDriverFeedbackSnapshot_t 保留步进驱动器最近一帧 CRC 正确回包，供调试和后续状态上传一致读取。 */
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
 * 函数功能：执行指定逻辑泵的一次 25ms 行为周期，处理驱动回包、故障锁存、队列速度、压力保护、排空、显示和 UART 下发。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue);

/*
 * 函数功能：一致性复制指定逻辑泵最近一帧 CRC 正确的步进驱动回包快照。
 * 输入参数：channel 为逻辑 A/B 通道；snapshot 指向调用方提供的快照缓存。
 * 返回参数：快照有效且复制成功返回 1，否则返回 0 并把输出清零。
 */
uint8_t PumpBehavior_CopyDriverFeedbackSnapshot(PumpBehaviorChannel_t channel,
                                                PumpDriverFeedbackSnapshot_t *snapshot);

#endif /* __PUMP_BEHAVIOR_CORE_H */
