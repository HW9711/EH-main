/*
 * Trace Recorder for Tracealyzer v4.8.1
 * Copyright 2023 Percepio AB
 * www.percepio.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Project-specific stream port implementation.
 */

#include <trcRecorder.h>
#include "tracealyzer_recorder.h"

#if (TRC_USE_TRACEALYZER_RECORDER == 1)

#if (TRC_CFG_RECORDER_MODE == TRC_RECORDER_MODE_STREAMING)

#if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_RINGBUFFER)

/* Backwards compatibility with older snapshot tooling. */
typedef TraceRingBuffer_t RecorderData;
RecorderData* RecorderDataPtr TRC_CFG_RECORDER_DATA_ATTRIBUTE;

TraceStreamPortData_t* pxStreamPortData TRC_CFG_RECORDER_DATA_ATTRIBUTE;

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* pxBuffer)
{
    TraceRingBuffer_t* pxRingBuffer;

    TRC_ASSERT_EQUAL_SIZE(TraceStreamPortBuffer_t, TraceStreamPortData_t);

    if (pxBuffer == (void*)0)
    {
        return TRC_FAIL;
    }

    pxStreamPortData = (TraceStreamPortData_t*)pxBuffer;
    pxRingBuffer = &pxStreamPortData->xRingBuffer;
    RecorderDataPtr = pxRingBuffer;

    pxRingBuffer->xEventBuffer.uxSize = sizeof(pxRingBuffer->xEventBuffer.uiBuffer);

#if (TRC_CFG_STREAM_PORT_RINGBUFFER_MODE == TRC_STREAM_PORT_RINGBUFFER_MODE_OVERWRITE_WHEN_FULL)
    if (xTraceMultiCoreEventBufferInitialize(&pxStreamPortData->xMultiCoreEventBuffer,
                                             TRC_EVENT_BUFFER_OPTION_OVERWRITE,
                                             pxRingBuffer->xEventBuffer.uiBuffer,
                                             sizeof(pxRingBuffer->xEventBuffer.uiBuffer)) == TRC_FAIL)
    {
        return TRC_FAIL;
    }
#else
    if (xTraceMultiCoreEventBufferInitialize(&pxStreamPortData->xMultiCoreEventBuffer,
                                             TRC_EVENT_BUFFER_OPTION_SKIP,
                                             pxRingBuffer->xEventBuffer.uiBuffer,
                                             sizeof(pxRingBuffer->xEventBuffer.uiBuffer)) == TRC_FAIL)
    {
        return TRC_FAIL;
    }
#endif

    if (xTraceHeaderInitialize(&pxRingBuffer->xHeaderBuffer) == TRC_FAIL)
    {
        return TRC_FAIL;
    }

    if (xTraceEntryTableInitialize(&pxRingBuffer->xEntryTable) == TRC_FAIL)
    {
        return TRC_FAIL;
    }

    if (xTraceTimestampInitialize(&pxRingBuffer->xTimestampInfo) == TRC_FAIL)
    {
        return TRC_FAIL;
    }

    pxRingBuffer->END_MARKERS[0] = 0x0AU;
    pxRingBuffer->END_MARKERS[1] = 0x0BU;
    pxRingBuffer->END_MARKERS[2] = 0x0CU;
    pxRingBuffer->END_MARKERS[3] = 0x0DU;
    pxRingBuffer->END_MARKERS[4] = 0x71U;
    pxRingBuffer->END_MARKERS[5] = 0x72U;
    pxRingBuffer->END_MARKERS[6] = 0x73U;
    pxRingBuffer->END_MARKERS[7] = 0x74U;
    pxRingBuffer->END_MARKERS[8] = 0xF1U;
    pxRingBuffer->END_MARKERS[9] = 0xF2U;
    pxRingBuffer->END_MARKERS[10] = 0xF3U;
    pxRingBuffer->END_MARKERS[11] = 0xF4U;

    pxRingBuffer->START_MARKERS[0] = 0x05U;
    pxRingBuffer->START_MARKERS[1] = 0x06U;
    pxRingBuffer->START_MARKERS[2] = 0x07U;
    pxRingBuffer->START_MARKERS[3] = 0x08U;
    pxRingBuffer->START_MARKERS[4] = 0x75U;
    pxRingBuffer->START_MARKERS[5] = 0x76U;
    pxRingBuffer->START_MARKERS[6] = 0x77U;
    pxRingBuffer->START_MARKERS[7] = 0x78U;
    pxRingBuffer->START_MARKERS[8] = 0xF5U;
    pxRingBuffer->START_MARKERS[9] = 0xF6U;
    pxRingBuffer->START_MARKERS[10] = 0xF7U;
    pxRingBuffer->START_MARKERS[11] = 0xF8U;

    return TRC_SUCCESS;
}

traceResult xTraceStreamPortOnTraceBegin(void)
{
    return xTraceMultiCoreEventBufferClear(&pxStreamPortData->xMultiCoreEventBuffer);
}

#elif (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)

static TraceStreamPortBuffer_t* pxStreamPortRTT TRC_CFG_RECORDER_DATA_ATTRIBUTE;

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* pxBuffer)
{
#if (TRC_USE_INTERNAL_BUFFER == 1)
    TRC_ASSERT_EQUAL_SIZE(TraceStreamPortBuffer_t, TraceStreamPortRTT_t);
#endif

    if (pxBuffer == (void*)0)
    {
        return TRC_FAIL;
    }

    pxStreamPortRTT = (TraceStreamPortBuffer_t*)pxBuffer;

#if (TRC_USE_INTERNAL_BUFFER == 1)
    return xTraceInternalEventBufferInitialize(pxStreamPortRTT->bufferInternal, sizeof(pxStreamPortRTT->bufferInternal));
#else
    return TRC_SUCCESS;
#endif
}

traceResult xTraceStreamPortOnEnable(uint32_t uiStartOption)
{
    (void)uiStartOption;

    if (SEGGER_RTT_ConfigUpBuffer(TRC_CFG_STREAM_PORT_RTT_UP_BUFFER_INDEX,
                                  "TzData",
                                  pxStreamPortRTT->bufferUp,
                                  sizeof(pxStreamPortRTT->bufferUp),
                                  TRC_CFG_STREAM_PORT_RTT_MODE) < 0)
    {
        return TRC_FAIL;
    }

    if (SEGGER_RTT_ConfigDownBuffer(TRC_CFG_STREAM_PORT_RTT_DOWN_BUFFER_INDEX,
                                    "TzCtrl",
                                    pxStreamPortRTT->bufferDown,
                                    sizeof(pxStreamPortRTT->bufferDown),
                                    TRC_CFG_STREAM_PORT_RTT_MODE) < 0)
    {
        return TRC_FAIL;
    }

    return TRC_SUCCESS;
}

#else
#error "Unsupported TRACEALYZER_TRANSPORT_MODE"
#endif

#endif

#endif
