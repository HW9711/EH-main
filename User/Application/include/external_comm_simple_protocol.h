#ifndef __EXTERNAL_COMM_SIMPLE_PROTOCOL_H
#define __EXTERNAL_COMM_SIMPLE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXTERNAL_COMM_SIMPLE_FRAME_SIZE 6U /* 简易协议固定帧长：AA BB CC FunCode EE FF。 */

/* 在 UART2 接收缓存中查找简易协议帧的结果；这里只检查帧头和剩余字节数。 */
typedef enum
{
    EXT_SIMPLE_PROBE_NOT_FOUND = 0U, /* 当前缓存没有完整 AA BB CC 帧头。 */
    EXT_SIMPLE_PROBE_INCOMPLETE,     /* 已找到帧头，但固定 6 字节尚未收全。 */
    EXT_SIMPLE_PROBE_READY           /* 已找到帧头且 6 字节已收全，还需检查帧尾和功能码。 */
} ExtSimpleProbeResult_t;

/* 读取缓存但不移除数据：从 skip_count 位置读 data_len 字节，成功返回 1，数据不足返回 0。 */
typedef uint8_t (*ExtSimpleReadFn)(uint16_t skip_count, uint8_t *data, uint16_t data_len);

/*
 * 函数功能：在共用 UART2 FIFO 中查找简易协议帧头，并判断固定 6 字节是否已经收全。
 * 输入参数：available 为接收缓存已有字节数；reader 读取缓存但不移除数据；head_offset 返回帧头位置。
 * 返回参数：返回未找到帧头、帧未收全或 6 字节已收全；已收全不代表帧尾正确。
 */
ExtSimpleProbeResult_t ExtSimple_Probe(uint16_t available,
                                      ExtSimpleReadFn reader,
                                      uint16_t *head_offset);

/*
 * 函数功能：校验并执行一帧 AA BB CC FunCode EE FF 简易外控指令。
 * 输入参数：frame 指向待检查的帧；frame_len 为帧的字节数。
 * 返回参数：帧结构完整时返回 1；帧头、帧尾或长度错误时返回 0。
 */
uint8_t ExtSimple_HandleFrame(const uint8_t *frame, uint16_t frame_len);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_SIMPLE_PROTOCOL_H */
