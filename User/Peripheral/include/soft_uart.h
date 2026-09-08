#ifndef __SOFT_UART_H
#define __SOFT_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* 两路新接口和默认通道 1 的旧接口共用这些返回值。 */
typedef enum {
    SOFT_UART_OK = 0,
    SOFT_UART_ERROR,
    SOFT_UART_BUSY,
    SOFT_UART_TIMEOUT
} SoftUART_Status;

/* 模拟串口通道编号定义。
 * SIM_UART_NONE 用于表示当前没有通道正在接收。 */
typedef enum {
    SIM_UART_1 = 0,
    SIM_UART_2 = 1,
    SIM_UART_COUNT = 2,
    SIM_UART_NONE = 0xFF
} sim_uart_channel_t;

/* 测试时可把一路压力串口的原始字节额外发到 UART10，方便用串口助手查看。
 * 默认关闭；开启会增加 UART10 发送等待，但不替代原有压力解析和字节队列。 */
#define SOFT_UART_TEST_FORWARD_OFF        0U /* 关闭测试输出。 */
#define SOFT_UART_TEST_FORWARD_SIM_UART_1 1U /* 只输出 SIM_UART_1/PE4 收到的 A 泵压力字节。 */
#define SOFT_UART_TEST_FORWARD_SIM_UART_2 2U /* 只输出 SIM_UART_2/PE6 收到的 B 泵压力字节。 */

#ifndef SOFT_UART_TEST_FORWARD_SOURCE
#define SOFT_UART_TEST_FORWARD_SOURCE SOFT_UART_TEST_FORWARD_OFF /* 选 0/1/2：关闭/A 泵/B 泵；正式使用默认关闭。 */
#endif

#ifndef SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS
#define SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS 20U /* 每批测试数据在 UART10 最多等待 20ms；仅开启测试输出时生效。 */
#endif

/* 每路串口的接收和丢字节计数，用于检查数据接收是否正常。 */
typedef struct {
    uint32_t received_bytes; /* 成功放入中断缓存的字节总数。 */
    uint32_t queue_overflow_count; /* 字节队列满后未能放入的次数；压力解析已在入队前完成。 */
    uint32_t buffer_overflow_count; /* 中断缓存满后丢弃新字节的次数。 */
    uint32_t overlap_drop_count; /* 兼容旧诊断接口；双定时器接收后正常应保持为 0。 */
    uint32_t framing_error_count; /* 起始位或停止位电平不符合串口要求的次数。 */
} SimUartStats;

/* 初始化两路模拟串口底层资源。
 * 包括 GPIO、EXTI、A 路 TIM11、B 路 TIM13、环形缓冲和静态消息队列。 */
void SimUart_InitAll(void);

/* 创建并启动 100ms 周期任务：取出中断缓存字节、解析压力帧、复制到旧接口队列并检查掉线。 */
void SimUartTask_Init(void);

/* 用 GPIO 电平逐位发送 1 字节，函数等待发送结束才返回。
 * 任一路正在接收时返回 SOFT_UART_BUSY，不开始发送。 */
SoftUART_Status SimUart_SendByte(sim_uart_channel_t channel, uint8_t data);

/* 连续发送指定长度的数据缓冲区。 */
SoftUART_Status SimUart_Send(sim_uart_channel_t channel, const uint8_t *data, uint16_t len);

/* 发送以 '\0' 结尾的字符串。 */
SoftUART_Status SimUart_SendString(sim_uart_channel_t channel, const char *str);

/* 从指定通道的 FreeRTOS 字节队列中读取 1 个字节。
 * 若当前无数据可读，则返回 SOFT_UART_TIMEOUT。 */
SoftUART_Status SimUart_ReadByte(sim_uart_channel_t channel, uint8_t *data);

/* 获取指定通道当前队列中的待取字节数。 */
uint32_t SimUart_GetPendingBytes(sim_uart_channel_t channel);

/* 一次复制指定通道的各项计数，复制时暂时关闭中断，避免计数在读取中途变化。 */
void SimUart_GetStats(sim_uart_channel_t channel, SimUartStats *stats);

/* 以下快捷接口用于按统计项分别读取诊断计数。 */
uint32_t SimUart_GetRxByteCount(sim_uart_channel_t channel);
uint32_t SimUart_GetQueueOverflowCount(sim_uart_channel_t channel);
uint32_t SimUart_GetBufferOverflowCount(sim_uart_channel_t channel);
uint32_t SimUart_GetOverlapDropCount(sim_uart_channel_t channel);

/* 在 HAL_GPIO_EXTI_Callback() 中调用。
 * 该函数负责识别软串口起始位下降沿并启动接收状态机。 */
void SimUart_HandleExti(uint16_t GPIO_Pin);

/* 在 TIM11/TIM13 中断入口中调用。
 * channel 指定本次读取 PE4 还是 PE6 的电平；两路各用自己的定时器。 */
void SimUart_TimerIrqHandler(sim_uart_channel_t channel);

/* 以下为单通道历史兼容接口，默认映射到通道 1。 */
SoftUART_Status Soft_UART_Init(void);
SoftUART_Status Soft_UART_SendByte(uint8_t data);
SoftUART_Status Soft_UART_ReceiveByte(uint8_t *data);
SoftUART_Status Soft_UART_Send(uint8_t *data, uint16_t len);
SoftUART_Status Soft_UART_Receive(uint8_t *data, uint16_t len);
SoftUART_Status Soft_UART_SendString(char *str);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_UART_H */
