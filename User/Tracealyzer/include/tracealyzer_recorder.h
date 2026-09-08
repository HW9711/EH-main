#ifndef TRACEALYZER_RECORDER_H
#define TRACEALYZER_RECORDER_H

#include <stdint.h>

typedef struct
{
    uint32_t initializeResult;
    uint32_t enableResult;
    uint8_t startupSucceeded;
    const char *lastError;
} TracealyzerRecorderDiagnostics_t;

/*
 * 调试记录编译开关：1=编入初始化及记录步骤，0=不执行这些步骤。
 * 当前main.c已注释掉Tracealyzer_RecorderInit调用，仅把本宏设为1不会自动启动记录。
 * 关闭后不再记录任务运行事件；部分代码仍参与编译，不代表相应RAM都会释放。
 */
#ifndef TRACEALYZER_SNAPSHOT_ENABLE
#define TRACEALYZER_SNAPSHOT_ENABLE 1U
#endif

/*
 * 调试记录保存方式：0=留在MCU内存环形缓冲区，1=通过J-Link RTT持续发给电脑。
 * 修改后调试工具须使用对应的读取方式；这不是主机业务通信的串口选择。
 */
#define TRACEALYZER_TRANSPORT_MODE_RINGBUFFER 0U
#define TRACEALYZER_TRANSPORT_MODE_JLINK_RTT  1U

#ifndef TRACEALYZER_TRANSPORT_MODE
#define TRACEALYZER_TRANSPORT_MODE TRACEALYZER_TRANSPORT_MODE_JLINK_RTT
#endif

/*
 * 内存记录模式使用的缓冲区大小，单位字节；调大可保留更多事件，但会多占用RAM。
 */
#ifndef TRACEALYZER_RING_BUFFER_SIZE_BYTES
#define TRACEALYZER_RING_BUFFER_SIZE_BYTES (32U * 1024U)
#endif

/*
 * RTT发送缓冲区大小，单位字节；保存准备发给电脑的调试事件。
 * 调大增加RAM占用；缓冲区能容纳的事件越多，越不容易因电脑读取不及时而丢失记录。
 */
#ifndef TRACEALYZER_RTT_UP_BUFFER_SIZE_BYTES
#define TRACEALYZER_RTT_UP_BUFFER_SIZE_BYTES (16U * 1024U)
#endif

#ifndef TRACEALYZER_RTT_DOWN_BUFFER_SIZE_BYTES
#define TRACEALYZER_RTT_DOWN_BUFFER_SIZE_BYTES 32U /* RTT接收缓冲区字节数，只接收电脑发来的调试控制命令。 */
#endif

#ifndef TRACEALYZER_RTT_UP_BUFFER_INDEX
#define TRACEALYZER_RTT_UP_BUFFER_INDEX 1U /* 向电脑发送事件的RTT通道编号，不是硬件UART编号；电脑须读取同一通道。 */
#endif

#ifndef TRACEALYZER_RTT_DOWN_BUFFER_INDEX
#define TRACEALYZER_RTT_DOWN_BUFFER_INDEX 1U /* 从电脑接收调试命令的RTT通道编号，须与电脑配置一致。 */
#endif

/*
 * Start the Tracealyzer recorder as early as possible, before application
 * tasks and synchronization objects are created, so startup activity is kept.
 */
void Tracealyzer_RecorderInit(void);

/*
 * Returns 1 when the recorder is enabled and actively accepting events.
 */
uint8_t Tracealyzer_RecorderIsRunning(void);

/*
 * In J-Link RTT streaming mode, completes the final xTraceEnable(TRC_START)
 * step from a normal FreeRTOS task context after the scheduler is already
 * running. Ring-buffer mode keeps this as a no-op.
 */
void Tracealyzer_RecorderTryStartStreaming(void);

/*
 * 调试器可直接查看的全局诊断状态。
 * 重点关注：
 * - initializeResult
 * - enableResult
 * - startupSucceeded
 * - lastError
 */
extern volatile TracealyzerRecorderDiagnostics_t g_tracealyzerRecorderDiagnostics;

/*
 * 返回最近一次 xTraceInitialize() 的结果。
 * - 0: TRC_SUCCESS
 * - 1: TRC_FAIL
 * 该值便于在调试器中快速判断 Streaming 初始化是否已经在最早阶段失败。
 */
uint32_t Tracealyzer_RecorderInitializeResult(void);

/*
 * 返回最近一次 xTraceEnable(TRC_START) 的结果。
 * - 0: TRC_SUCCESS
 * - 1: TRC_FAIL
 * 如果该步骤失败，通常说明 StreamPort、KernelPort 或底层时间戳端口未准备好。
 */
uint32_t Tracealyzer_RecorderEnableResult(void);

/*
 * 返回最近一次启动流程中 TraceRecorder 暴露的最后一条错误描述。
 * 返回 NULL 表示当前未取到明确错误文本。
 */
const char *Tracealyzer_RecorderLastError(void);


/*
 * Returns the primary recorder data pointer for debugger-assisted inspection.
 * - Ring buffer mode: address of the local ring buffer image.
 * - J-Link RTT mode: address of the SEGGER RTT control block.
 */
const void *Tracealyzer_RecorderDataPointer(void);

/*
 * Returns the size of the recorder data block returned by
 * Tracealyzer_RecorderDataPointer().
 */
uint32_t Tracealyzer_RecorderDataSize(void);

#endif
