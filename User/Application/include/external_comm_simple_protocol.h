#ifndef __EXTERNAL_COMM_SIMPLE_PROTOCOL_H
#define __EXTERNAL_COMM_SIMPLE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXTERNAL_COMM_SIMPLE_FRAME_SIZE 6U /* 简易协议固定帧长：AA BB CC FunCode EE FF。 */

/* 简易协议在共用 UART2 FIFO 中探测候选帧的结果。 */
typedef enum
{
    EXT_SIMPLE_PROBE_NOT_FOUND = 0U, /* 当前缓存没有完整 AA BB CC 帧头。 */
    EXT_SIMPLE_PROBE_INCOMPLETE,     /* 已找到帧头，但固定 6 字节尚未收全。 */
    EXT_SIMPLE_PROBE_READY           /* 已找到帧头且 6 字节候选帧已经收全。 */
} ExtSimpleProbeResult_t;

/* FIFO 窥探回调：从 skip_count 偏移读取 data_len 字节，成功返回 1，数据不足返回 0。 */
typedef uint8_t (*ExtSimpleReadFn)(uint16_t skip_count, uint8_t *data, uint16_t data_len);

/*
 * 函数功能：在共用 UART2 FIFO 中查找简易协议帧头，并判断固定 6 字节是否已经收全。
 * 输入参数：available 为 FIFO 已有字节数；reader 为只读窥探回调；head_offset 返回帧头偏移。
 * 返回参数：返回未找到、半帧或候选帧已就绪状态。
 */
ExtSimpleProbeResult_t ExtSimple_Probe(uint16_t available,
                                      ExtSimpleReadFn reader,
                                      uint16_t *head_offset);

/*
 * 函数功能：校验并执行一帧 AA BB CC FunCode EE FF 简易外控指令。
 * 输入参数：frame 指向候选帧；frame_len 为候选帧长度。
 * 返回参数：帧结构完整时返回 1；帧头、帧尾或长度错误时返回 0。
 */
uint8_t ExtSimple_HandleFrame(const uint8_t *frame, uint16_t frame_len);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_SIMPLE_PROTOCOL_H */
