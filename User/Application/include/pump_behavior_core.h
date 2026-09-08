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

/* 对应驱动反馈byte4高半字节；无版本或反馈超时都不能按READY放行。 */
typedef enum
{
    PUMP_DRIVER_UNAVAILABLE = 0x00U, /* 尚无新协议有效反馈、旧驱动或反馈过期，保持零速。 */
    PUMP_DRIVER_READY = 0x80U,       /* 编码器已验证且新零速握手完成，同帧原始故障仍必须为0。 */
    PUMP_DRIVER_ALIGNING = 0x90U,    /* 等待安全条件、定位中或等待完成后的新零速，不是普通运行故障。 */
    PUMP_DRIVER_ALIGN_FAILED = 0xA0U /* 本轮定位失败，只允许明确请求新一轮定位，不能自动加力。 */
} PumpDriverAlignmentState_t;

/* 保存步进驱动最近一帧 CRC 正确的反馈；复制函数保证字段不会来自两次不同更新。 */
typedef struct
{
    uint32_t feedback_tick_ms; /* 主控完成本帧校验的 HAL 毫秒时刻，用于判断现场回包新旧。 */
    uint16_t sequence;         /* 每收到一帧 CRC 正确回包递增一次，16 位自然回绕。 */
    uint16_t actual_speed;     /* 驱动回包 byte2~3 的实际速度原始值，高字节在前。 */
    uint8_t direction;         /* 驱动回包 byte1 的实际方向原始值。 */
    uint8_t raw_error;         /* 新协议byte4低半字节的原App.Err；旧协议保留完整字节，0也不代表已就绪。 */
    uint8_t alignment_state;   /* 新协议byte4高半字节，旧协议或未知状态为UNAVAILABLE。 */
    uint8_t wire_status;       /* 原样保留byte4，便于核对版本、定位状态和原始保护码。 */
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
 * 函数功能：检查新协议就绪、无故障、新鲜反馈和主控释放门禁，供所有启动来源共同使用。
 * 输入参数：channel 为逻辑 A/B 泵通道。
 * 返回参数：可以接受新的运行请求返回1；定位、失败、失联或未释放旧请求时返回0。
 */
uint8_t PumpBehavior_DriverCanRun(PumpBehaviorChannel_t channel);

/*
 * 函数功能：检查任一路泵是否仍在标定或等待明确重标定应答，用于抑制普通提示音。
 * 输入参数：无。
 * 返回参数：仍有标定动作返回1，否则返回0；不用于屏蔽持续或限时安全报警。
 */
uint8_t PumpBehavior_IsAlignmentBusy(void);

/*
 * 函数功能：由蜂鸣任务在两路标定结束后一次消费合并结果，失败优先于成功。
 * 输入参数：无。
 * 返回参数：0无结果或仍忙、1成功单响、2失败双响；不会更改泵运行授权。
 */
uint8_t PumpBehavior_TakeAlignmentBeepResult(void);

/*
 * 函数功能：屏幕明确请求失败泵重新定位，只投递零速校准动作，不保存任何运行速度。
 * 输入参数：channel 为逻辑 A/B 泵通道；仅屏幕失败恢复入口调用，不供脚踏或持续外控保活调用。
 * 返回参数：成功投递独立定位请求返回1；非失败态、离线、运行中或已有请求时返回0。
 */
uint8_t PumpBehavior_RequestRealign(PumpBehaviorChannel_t channel);

/*
 * 函数功能：明确全停时取消本泵定位和待发重定位，不把普通零速保活误当取消。
 * 输入参数：channel 为逻辑 A/B 泵通道；由全停、退出外控或全局报警路径调用。
 * 返回参数：无；泵任务持续发送零速取消帧，直到可信反馈确认不再定位加力。
 */
void PumpBehavior_CancelAlignment(PumpBehaviorChannel_t channel);

#endif /* __PUMP_BEHAVIOR_CORE_H */
