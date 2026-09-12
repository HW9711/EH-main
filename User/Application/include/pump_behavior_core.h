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

/* 泵驱动反馈第5字节的高4位表示定位状态；还须检查故障码和反馈时间，不能仅凭此枚举启动泵。 */
typedef enum
{
    PUMP_DRIVER_UNAVAILABLE = 0x00U, /* 没有可识别的定位状态，包括尚无回包或旧协议；反馈是否超时另行判断。 */
    PUMP_DRIVER_READY = 0x80U,       /* 驱动报告定位已完成、可以接受运行命令；主控仍须检查同帧故障码是否为0。 */
    PUMP_DRIVER_ALIGNING = 0x90U,    /* 驱动还未允许正常运行，可能正在等待定位条件、执行定位或等待定位后的零速确认。 */
    PUMP_DRIVER_ALIGN_FAILED = 0xA0U /* 驱动报告本轮定位失败；需要用户重新操作，主控不会因持续启动请求自动重试。 */
} PumpDriverAlignmentState_t;

/* 保存步进驱动最近一帧 CRC 正确的反馈；复制函数保证字段不会来自两次不同更新。 */
typedef struct
{
    uint32_t feedback_tick_ms; /* 主控完成本帧校验的 HAL 毫秒时刻，用于判断现场回包新旧。 */
    uint16_t sequence;         /* 每收到一帧 CRC 正确回包递增一次，16 位自然回绕。 */
    uint16_t actual_speed;     /* 驱动回包 byte2~3 的实际速度原始值，高字节在前。 */
    uint8_t direction;         /* 驱动回包 byte1 的实际方向原始值。 */
    uint8_t raw_error;         /* 新协议取第5字节低4位故障码；旧协议保留整字节。故障码0不代表定位已完成。 */
    uint8_t alignment_state;   /* 保存驱动报告的定位状态；旧协议或无法识别的状态记为UNAVAILABLE。 */
    uint8_t wire_status;       /* 保留回包第5字节原值，便于同时检查高4位定位状态和低4位故障码。 */
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

/*
 * 函数功能：检查驱动反馈是否有效且就绪、故障是否清除，以及主控是否已允许重新启动。
 * 输入参数：channel 为逻辑 A/B 泵通道。
 * 返回参数：允许新的开泵请求返回1；定位中、故障、反馈超时或旧请求尚未清掉时返回0。
 */
uint8_t PumpBehavior_DriverCanRun(PumpBehaviorChannel_t channel);

/*
 * 函数功能：检查A/B是否还有泵正在定位或准备重试定位；用于暂停普通按键提示音。
 * 输入参数：无。
 * 返回参数：有泵尚未结束定位返回1，否则返回0；报警蜂鸣不受本函数影响。
 */
uint8_t PumpBehavior_IsAlignmentBusy(void);

/*
 * 函数功能：两路定位都结束后，把待播结果交给蜂鸣任务并清零；只要有一路失败就提示双响。
 * 输入参数：无。
 * 返回参数：0没有新结果或仍有泵在定位，1成功单响，2失败双响；不改变泵启停条件。
 */
uint8_t PumpBehavior_TakeAlignmentBeepResult(void);

/*
 * 函数功能：记录屏幕要求失败泵重新定位的操作，由泵任务稍后发送零速定位命令，不启动输液。
 * 输入参数：channel 为逻辑A/B泵通道；只有屏幕重试操作调用，脚踏和上位机重复启动不能调用。
 * 返回参数：已记下重试请求返回1；不是定位失败、离线、运行中或已有待处理请求时返回0。
 */
uint8_t PumpBehavior_RequestRealign(PumpBehaviorChannel_t channel);

/*
 * 函数功能：全停时记录取消本泵定位的要求，并清掉尚未发出的重试；正常待机的周期零速不调用本函数。
 * 输入参数：channel 为逻辑 A/B 泵通道；由全停、退出外控或全局报警路径调用。
 * 返回参数：无；泵任务反复发送0x03零速取消命令，直到有效反馈不再表示无故障定位中。
 */
void PumpBehavior_CancelAlignment(PumpBehaviorChannel_t channel);

#endif /* __PUMP_BEHAVIOR_CORE_H */
