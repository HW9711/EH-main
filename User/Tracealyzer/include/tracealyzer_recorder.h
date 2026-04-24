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
 * Tracealyzer global enable switch.
 * 1U: initialize and start the recorder during system startup.
 * 0U: keep recorder code compiled in, but do not start tracing.
 */
#ifndef TRACEALYZER_SNAPSHOT_ENABLE
#define TRACEALYZER_SNAPSHOT_ENABLE 1U
#endif

/*
 * Transport mode selection.
 * RINGBUFFER: keep the previous snapshot-style workflow in MCU RAM.
 * JLINK_RTT: stream trace data continuously to the PC through J-Link RTT.
 */
#define TRACEALYZER_TRANSPORT_MODE_RINGBUFFER 0U
#define TRACEALYZER_TRANSPORT_MODE_JLINK_RTT  1U

#ifndef TRACEALYZER_TRANSPORT_MODE
#define TRACEALYZER_TRANSPORT_MODE TRACEALYZER_TRANSPORT_MODE_JLINK_RTT
#endif

/*
 * Ring buffer size used by the local snapshot transport.
 */
#ifndef TRACEALYZER_RING_BUFFER_SIZE_BYTES
#define TRACEALYZER_RING_BUFFER_SIZE_BYTES (32U * 1024U)
#endif

/*
 * RTT buffer sizing used by the J-Link streaming transport.
 * The UP buffer carries trace data from target to host.
 * The DOWN buffer receives Tracealyzer control commands from host to target.
 */
#ifndef TRACEALYZER_RTT_UP_BUFFER_SIZE_BYTES
#define TRACEALYZER_RTT_UP_BUFFER_SIZE_BYTES (16U * 1024U)
#endif

#ifndef TRACEALYZER_RTT_DOWN_BUFFER_SIZE_BYTES
#define TRACEALYZER_RTT_DOWN_BUFFER_SIZE_BYTES 32U
#endif

#ifndef TRACEALYZER_RTT_UP_BUFFER_INDEX
#define TRACEALYZER_RTT_UP_BUFFER_INDEX 1U
#endif

#ifndef TRACEALYZER_RTT_DOWN_BUFFER_INDEX
#define TRACEALYZER_RTT_DOWN_BUFFER_INDEX 1U
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
