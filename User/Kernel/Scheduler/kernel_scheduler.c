#include "kernel_scheduler.h"

#include "tracealyzer_recorder.h"
#include "trcRecorder.h"

static TraceStringHandle_t sKernelQueueTraceChannel = 0;
static TraceStringHandle_t sKernelQueueCreateFormat = 0;
static TraceStringHandle_t sKernelQueueSendFormat = 0;
static TraceStringHandle_t sKernelQueueReceiveFormat = 0;

static void Kernel_TraceEnsureQueueFormats(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    if (!xTraceIsRecorderEnabled())
    {
        return;
    }

    if (sKernelQueueTraceChannel == 0U)
    {
        (void)xTraceStringRegister("KernelQueue", &sKernelQueueTraceChannel);
    }

    if (sKernelQueueCreateFormat == 0U)
    {
        (void)xTraceStringRegister("CREATE queue=0x%x wait=%u result=%u", &sKernelQueueCreateFormat);
    }

    if (sKernelQueueSendFormat == 0U)
    {
        (void)xTraceStringRegister("SEND queue=0x%x wait=%u result=%u", &sKernelQueueSendFormat);
    }

    if (sKernelQueueReceiveFormat == 0U)
    {
        (void)xTraceStringRegister("RECV queue=0x%x wait=%u result=%u", &sKernelQueueReceiveFormat);
    }
#endif
}

static void Kernel_TraceNameQueue(QueueHandle_t queue, const char *name)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    if ((queue == NULL) || (name == NULL) || (!xTraceIsRecorderEnabled()))
    {
        return;
    }

    /*
     * FreeRTOS 队列本身已经由 Tracealyzer 内核端口记录；这里补充稳定对象名，
     * 让时间线中的 Queue Send/Receive 事件可以直接显示业务队列名称。
     */
    (void)xTraceObjectSetNameWithoutHandle((void *)queue, name);
#else
    (void)queue;
    (void)name;
#endif
}

static void Kernel_TraceQueueEvent(TraceStringHandle_t format,
                                   QueueHandle_t queue,
                                   TickType_t ticks_to_wait,
                                   BaseType_t result)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    if (!xTraceIsRecorderEnabled())
    {
        return;
    }

    Kernel_TraceEnsureQueueFormats();
    if ((sKernelQueueTraceChannel == 0U) || (format == 0U))
    {
        return;
    }

    (void)xTracePrintF3(sKernelQueueTraceChannel,
                        format,
                        (TraceUnsignedBaseType_t)queue,
                        (TraceUnsignedBaseType_t)ticks_to_wait,
                        (TraceUnsignedBaseType_t)result);
#else
    (void)format;
    (void)queue;
    (void)ticks_to_wait;
    (void)result;
#endif
}

int Kernel_TaskCreateNamed(kernel_task_t *task, const char *name, kernel_task_cb_t func)
{
    return app_task_create_named(task, name, func);
}

int Kernel_TaskStart(kernel_task_t *task, bool one_shot, uint32_t time_ms)
{
    return app_task_start(task, one_shot, time_ms);
}

int Kernel_TaskStop(kernel_task_t *task)
{
    return app_task_stop(task);
}

QueueHandle_t Kernel_QueueCreateNamed(UBaseType_t length, UBaseType_t item_size, const char *name)
{
    QueueHandle_t queue = xQueueCreate(length, item_size);

    Kernel_TraceEnsureQueueFormats();
    Kernel_TraceNameQueue(queue, name);
    Kernel_TraceQueueEvent(sKernelQueueCreateFormat, queue, 0U, (queue != NULL) ? pdPASS : pdFAIL);

    return queue;
}

BaseType_t Kernel_QueueSendNamed(QueueHandle_t queue, const void *item, TickType_t ticks_to_wait, const char *queue_name)
{
    BaseType_t result;

    Kernel_TraceNameQueue(queue, queue_name);
    result = xQueueSend(queue, item, ticks_to_wait);
    Kernel_TraceEnsureQueueFormats();
    Kernel_TraceQueueEvent(sKernelQueueSendFormat, queue, ticks_to_wait, result);

    return result;
}

BaseType_t Kernel_QueueReceiveNamed(QueueHandle_t queue, void *buffer, TickType_t ticks_to_wait, const char *queue_name)
{
    BaseType_t result;

    Kernel_TraceNameQueue(queue, queue_name);
    result = xQueueReceive(queue, buffer, ticks_to_wait);
    Kernel_TraceEnsureQueueFormats();
    Kernel_TraceQueueEvent(sKernelQueueReceiveFormat, queue, ticks_to_wait, result);

    return result;
}
