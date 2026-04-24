#include "tracealyzer_recorder.h"

#include "trcRecorder.h"
#include "trcError.h"
#include "trcStreamPort.h"

#if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
#include "SEGGER_RTT.h"

/*
 * Streaming 模式下默认改为“等待主机后再开始”。
 * 这样可以避免目标板在 Tracealyzer 完成 RTT 建链前就发出关键的
 * PSF 头部和对象表，导致主机侧收到字节但一直显示 0 events。
 *
 * 如需恢复旧行为，可在工程编译宏中覆盖：
 * TRACEALYZER_STREAMING_START_OPTION=TRC_START
 */
#endif

/*
 * 这些状态变量专门用于调试 Streaming 启动阶段。
 * 现象“RTT 已连上但 Total Events=0”时，可直接在调试器中查看：
 * 1. xTraceInitialize 是否成功
 * 2. xTraceEnable(TRC_START) 是否成功
 * 3. TraceRecorder 是否给出了最后错误字符串
 */
volatile TracealyzerRecorderDiagnostics_t g_tracealyzerRecorderDiagnostics =
{
    (uint32_t)TRC_FAIL,
    (uint32_t)TRC_FAIL,
    0U,
    (const char *)0
};

void Tracealyzer_RecorderInit(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    const char *errorText = (const char *)0;

    /*
     * Start tracing as early as possible so we keep startup behavior too.
     * In RTT mode the data is streamed continuously to the PC.
     * In ring-buffer mode the recorder still behaves like a local snapshot.
     */
    if (xTraceIsRecorderEnabled())
    {
        return;
    }

    g_tracealyzerRecorderDiagnostics.initializeResult = (uint32_t)xTraceInitialize();
    g_tracealyzerRecorderDiagnostics.enableResult = (uint32_t)TRC_FAIL;
    g_tracealyzerRecorderDiagnostics.startupSucceeded = 0U;
    g_tracealyzerRecorderDiagnostics.lastError = (const char *)0;

    if ((traceResult)g_tracealyzerRecorderDiagnostics.initializeResult == TRC_SUCCESS)
    {
        #if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
        g_tracealyzerRecorderDiagnostics.lastError = "Trace initialized; waiting for scheduler task to enable stream";
        #else
        g_tracealyzerRecorderDiagnostics.enableResult = (uint32_t)xTraceEnable(TRC_START);
        #endif
    }

    if (xTraceErrorGetLast(&errorText) == TRC_SUCCESS)
    {
        g_tracealyzerRecorderDiagnostics.lastError = errorText;
    }

    g_tracealyzerRecorderDiagnostics.startupSucceeded =
        (uint8_t)(xTraceIsRecorderEnabled() ? 1U : 0U);
#endif
}

void Tracealyzer_RecorderTryStartStreaming(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    const char *errorText = (const char *)0;

    #if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
    if (((traceResult)g_tracealyzerRecorderDiagnostics.initializeResult != TRC_SUCCESS) ||
        xTraceIsRecorderEnabled())
    {
        g_tracealyzerRecorderDiagnostics.startupSucceeded =
            (uint8_t)(xTraceIsRecorderEnabled() ? 1U : 0U);
        return;
    }

    g_tracealyzerRecorderDiagnostics.enableResult = (uint32_t)xTraceEnable(TRC_START);

    if (xTraceErrorGetLast(&errorText) == TRC_SUCCESS)
    {
        g_tracealyzerRecorderDiagnostics.lastError = errorText;
    }

    g_tracealyzerRecorderDiagnostics.startupSucceeded =
        (uint8_t)(xTraceIsRecorderEnabled() ? 1U : 0U);
    #else
    (void)errorText;
    #endif
#endif
}

uint8_t Tracealyzer_RecorderIsRunning(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    return (uint8_t)(xTraceIsRecorderEnabled() ? 1U : 0U);
#else
    return 0U;
#endif
}

uint32_t Tracealyzer_RecorderInitializeResult(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    return g_tracealyzerRecorderDiagnostics.initializeResult;
#else
    return (uint32_t)TRC_FAIL;
#endif
}

uint32_t Tracealyzer_RecorderEnableResult(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    return g_tracealyzerRecorderDiagnostics.enableResult;
#else
    return (uint32_t)TRC_FAIL;
#endif
}

const char *Tracealyzer_RecorderLastError(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    return g_tracealyzerRecorderDiagnostics.lastError;
#else
    return (const char *)0;
#endif
}

const void *Tracealyzer_RecorderDataPointer(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    #if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_RINGBUFFER)
    if (pxStreamPortData != (TraceStreamPortData_t *)0)
    {
        return (const void *)&pxStreamPortData->xRingBuffer;
    }

    return (const void *)0;
    #elif (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
    return (const void *)&_SEGGER_RTT;
    #else
    return (const void *)0;
    #endif
#else
    return (const void *)0;
#endif
}

uint32_t Tracealyzer_RecorderDataSize(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    #if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_RINGBUFFER)
    return (uint32_t)sizeof(TraceRingBuffer_t);
    #elif (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
    return (uint32_t)sizeof(_SEGGER_RTT);
    #else
    return 0U;
    #endif
#else
    return 0U;
#endif
}
