/*
 * Trace Recorder for Tracealyzer v4.8.1
 * Copyright 2023 Percepio AB
 * www.percepio.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Project-specific stream port glue. We keep one header and switch transport
 * behavior through TRACEALYZER_TRANSPORT_MODE so the rest of the recorder can
 * stay untouched.
 */

#ifndef TRC_STREAM_PORT_H
#define TRC_STREAM_PORT_H

#if (TRC_USE_TRACEALYZER_RECORDER == 1)

#if (TRC_CFG_RECORDER_MODE == TRC_RECORDER_MODE_STREAMING)

#include <trcTypes.h>
#include <trcStreamPortConfig.h>
#include <trcRecorder.h>
#include "tracealyzer_recorder.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_RINGBUFFER)

/*
 * Legacy local ring-buffer transport.
 */
#define TRC_EXTERNAL_BUFFERS 1
#define TRC_SEND_NAME_ONLY_ON_DELETE 1
#define TRC_USE_INTERNAL_BUFFER 0

#define TRC_STREAM_PORT_BUFFER_SIZE \
    (((uint32_t)(TRC_CFG_STREAM_PORT_BUFFER_SIZE) / sizeof(TraceUnsignedBaseType_t)) * sizeof(TraceUnsignedBaseType_t))

typedef struct TraceMultiCoreBuffer
{
    TraceUnsignedBaseType_t uxSize;
    uint8_t uiBuffer[TRC_STREAM_PORT_BUFFER_SIZE];
} TraceMultiCoreBuffer_t;

typedef struct TraceRingBuffer
{
    uint32_t reserved0;
    volatile uint8_t START_MARKERS[12];
    TraceHeaderBuffer_t xHeaderBuffer;
    TraceTimestampData_t xTimestampInfo;
    TraceEntryTable_t xEntryTable;
    TraceMultiCoreBuffer_t xEventBuffer;
    volatile uint8_t END_MARKERS[12];
    uint32_t reserved1;
} TraceRingBuffer_t;

typedef struct TraceStreamPortData
{
    TraceMultiCoreEventBuffer_t xMultiCoreEventBuffer;
    TraceRingBuffer_t xRingBuffer;
} TraceStreamPortData_t;

extern TraceStreamPortData_t* pxStreamPortData;

#define TRC_STREAM_PORT_DATA_BUFFER_SIZE (sizeof(TraceStreamPortData_t))

typedef struct TraceStreamPortBuffer
{
    uint8_t buffer[TRC_STREAM_PORT_DATA_BUFFER_SIZE];
} TraceStreamPortBuffer_t;

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* pxBuffer);
traceResult xTraceStreamPortOnTraceBegin(void);

#define xTraceStreamPortAllocate(_uiSize, _ppvData) \
    xTraceMultiCoreEventBufferAlloc(&pxStreamPortData->xMultiCoreEventBuffer, _uiSize, _ppvData)

#define xTraceStreamPortCommit(_pvData, _uiSize, _piBytesCommitted) \
    xTraceMultiCoreEventBufferAllocCommit(&pxStreamPortData->xMultiCoreEventBuffer, _pvData, _uiSize, _piBytesCommitted)

#define xTraceStreamPortWriteData(_pvData, _uiSize, _piBytesWritten) \
    TRC_COMMA_EXPR_TO_STATEMENT_EXPR_4((void)(_pvData), (void)(_uiSize), (void)(_piBytesWritten), TRC_SUCCESS)

#define xTraceStreamPortReadData(_pvData, _uiSize, _piBytesRead) \
    TRC_COMMA_EXPR_TO_STATEMENT_EXPR_4((void)(_pvData), (void)(_uiSize), (void)(_piBytesRead), TRC_SUCCESS)

#define xTraceStreamPortOnEnable(_uiStartOption) \
    TRC_COMMA_EXPR_TO_STATEMENT_EXPR_2((void)(_uiStartOption), TRC_SUCCESS)

#define xTraceStreamPortOnDisable() TRC_COMMA_EXPR_TO_STATEMENT_EXPR_1(TRC_SUCCESS)
#define xTraceStreamPortOnTraceEnd() TRC_COMMA_EXPR_TO_STATEMENT_EXPR_1(TRC_SUCCESS)

#elif (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)

/*
 * J-Link RTT streaming transport.
 */
#include <SEGGER_RTT_Conf.h>
#include <SEGGER_RTT.h>

#define TRC_USE_INTERNAL_BUFFER (TRC_CFG_STREAM_PORT_USE_INTERNAL_BUFFER)
#define TRC_INTERNAL_EVENT_BUFFER_WRITE_MODE (TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_WRITE_MODE)
#define TRC_INTERNAL_EVENT_BUFFER_TRANSFER_MODE (TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_TRANSFER_MODE)
#define TRC_INTERNAL_BUFFER_CHUNK_SIZE (TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_CHUNK_SIZE)
#define TRC_INTERNAL_BUFFER_CHUNK_TRANSFER_AGAIN_SIZE_LIMIT (TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_CHUNK_TRANSFER_AGAIN_SIZE_LIMIT)
#define TRC_INTERNAL_BUFFER_CHUNK_TRANSFER_AGAIN_COUNT_LIMIT (TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_CHUNK_TRANSFER_AGAIN_COUNT_LIMIT)

#define TRC_STREAM_PORT_INTERNAL_BUFFER_SIZE \
    ((((TRC_CFG_STREAM_PORT_INTERNAL_BUFFER_SIZE) + sizeof(TraceUnsignedBaseType_t) - 1U) / sizeof(TraceUnsignedBaseType_t)) * sizeof(TraceUnsignedBaseType_t))

#define TRC_STREAM_PORT_RTT_UP_BUFFER_SIZE \
    ((((TRC_CFG_STREAM_PORT_RTT_UP_BUFFER_SIZE) + sizeof(TraceUnsignedBaseType_t) - 1U) / sizeof(TraceUnsignedBaseType_t)) * sizeof(TraceUnsignedBaseType_t))

#define TRC_STREAM_PORT_RTT_DOWN_BUFFER_SIZE \
    ((((TRC_CFG_STREAM_PORT_RTT_DOWN_BUFFER_SIZE) + sizeof(TraceUnsignedBaseType_t) - 1U) / sizeof(TraceUnsignedBaseType_t)) * sizeof(TraceUnsignedBaseType_t))

typedef struct TraceStreamPortBuffer
{
#if (TRC_USE_INTERNAL_BUFFER == 1)
    uint8_t bufferInternal[TRC_STREAM_PORT_INTERNAL_BUFFER_SIZE];
#endif
    uint8_t bufferUp[TRC_STREAM_PORT_RTT_UP_BUFFER_SIZE];
    uint8_t bufferDown[TRC_STREAM_PORT_RTT_DOWN_BUFFER_SIZE];
} TraceStreamPortBuffer_t;

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* pxBuffer);
traceResult xTraceStreamPortOnEnable(uint32_t uiStartOption);

#if (TRC_USE_INTERNAL_BUFFER == 1)
    #if (TRC_INTERNAL_EVENT_BUFFER_WRITE_MODE == TRC_INTERNAL_EVENT_BUFFER_OPTION_WRITE_MODE_COPY)
        #define xTraceStreamPortAllocate(uiSize, ppvData) ((void)(uiSize), xTraceStaticBufferGet(ppvData))
    #else
        #define xTraceStreamPortAllocate(uiSize, ppvData) ((void)(uiSize), xTraceInternalEventBufferAlloc(uiSize, ppvData))
    #endif
#else
    #define xTraceStreamPortAllocate(uiSize, ppvData) ((void)(uiSize), xTraceStaticBufferGet(ppvData))
#endif

#if (TRC_USE_INTERNAL_BUFFER == 1)
    #if (TRC_INTERNAL_EVENT_BUFFER_WRITE_MODE == TRC_INTERNAL_EVENT_BUFFER_OPTION_WRITE_MODE_COPY)
        #define xTraceStreamPortCommit xTraceInternalEventBufferPush
    #else
        #define xTraceStreamPortCommit xTraceInternalEventBufferAllocCommit
    #endif
#else
    #define xTraceStreamPortCommit xTraceStreamPortWriteData
#endif

#if (defined(TRC_CFG_STREAM_PORT_RTT_NO_LOCK_WRITE) && (TRC_CFG_STREAM_PORT_RTT_NO_LOCK_WRITE == 1))
    #define xTraceStreamPortWriteData(pvData, uiSize, piBytesWritten) \
        TRC_COMMA_EXPR_TO_STATEMENT_EXPR_2(*(piBytesWritten) = (int32_t)SEGGER_RTT_WriteNoLock((TRC_CFG_STREAM_PORT_RTT_UP_BUFFER_INDEX), (const char*)(pvData), uiSize), TRC_SUCCESS)
#else
    #define xTraceStreamPortWriteData(pvData, uiSize, piBytesWritten) \
        TRC_COMMA_EXPR_TO_STATEMENT_EXPR_2(*(piBytesWritten) = (int32_t)SEGGER_RTT_Write((TRC_CFG_STREAM_PORT_RTT_UP_BUFFER_INDEX), (const char*)(pvData), uiSize), TRC_SUCCESS)
#endif

#define xTraceStreamPortReadData(pvData, uiSize, piBytesRead) \
    ((SEGGER_RTT_HASDATA(TRC_CFG_STREAM_PORT_RTT_DOWN_BUFFER_INDEX)) ? \
        (*(piBytesRead) = (int32_t)SEGGER_RTT_Read((TRC_CFG_STREAM_PORT_RTT_DOWN_BUFFER_INDEX), (char*)(pvData), uiSize), TRC_SUCCESS) : \
        TRC_SUCCESS)

#define xTraceStreamPortOnDisable() (void)(TRC_SUCCESS)
#define xTraceStreamPortOnTraceBegin() (void)(TRC_SUCCESS)
#define xTraceStreamPortOnTraceEnd() (void)(TRC_SUCCESS)

#else
#error "Unsupported TRACEALYZER_TRANSPORT_MODE"
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif

#endif
