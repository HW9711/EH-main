#ifndef __SOFT_UART_H
#define __SOFT_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* 模拟串口接口返回值。
 * 该状态码同时用于双通道新接口和单通道兼容包装接口。 */
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

/* 模拟串口测试透传配置。
 * 默认关闭；开启后仅把选中通道收到的原始字节透传到 UART10，
 * 不改变原有环形缓冲、任务搬运与队列读取逻辑。 */
#define SOFT_UART_TEST_FORWARD_OFF        0U
#define SOFT_UART_TEST_FORWARD_SIM_UART_1 1U
#define SOFT_UART_TEST_FORWARD_SIM_UART_2 2U

#ifndef SOFT_UART_TEST_FORWARD_SOURCE
#define SOFT_UART_TEST_FORWARD_SOURCE SOFT_UART_TEST_FORWARD_OFF
#endif

#ifndef SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS
#define SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS 20U
#endif

/* 每路模拟串口的运行统计信息。
 * 这些计数用于上层诊断接收稳定性、队列压力和异常重叠情况。 */
typedef struct {
    uint32_t received_bytes;
    uint32_t queue_overflow_count;
    uint32_t buffer_overflow_count;
    uint32_t overlap_drop_count; /* 兼容旧诊断接口；双定时器接收后正常应保持为 0。 */
    uint32_t framing_error_count;
} SimUartStats;

/* 初始化两路模拟串口底层资源。
 * 包括 GPIO、EXTI、A 路 TIM11、B 路 TIM13、环形缓冲和静态消息队列。 */
void SimUart_InitAll(void);

/* 创建并启动模拟串口后台任务。
 * 该任务按 100ms 周期运行，仅负责把 ISR 环形缓冲中的字节搬运到队列。 */
void SimUartTask_Init(void);

/* 发送单个字节。
 * 发送仍使用阻塞式 bit-bang，便于保持和现有工程的兼容性。 */
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

/* 读取指定通道的统计信息快照。 */
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
 * channel 固定指定本次要推进的 PE4 或 PE6 接收状态，两路可同时采样。 */
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
