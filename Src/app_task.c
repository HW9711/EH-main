#include "app_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "tracealyzer_recorder.h"
#include "trcRecorder.h"

// 定义应用程序任务相关的枚举常量，用于配置任务参数
enum
{
    APP_TASK_STACK_DEPTH = 1024,  // 定义应用程序任务栈深度为1024字节
    APP_TASK_PRIORITY = tskIDLE_PRIORITY + 3,  // 定义应用程序任务优先级，比空闲任务优先级高3级
    APP_TASK_TICK_MS = 1,  // 定义应用程序任务的时间片 tick 为1毫秒
};

static task_t *app_task_list = NULL;
static TaskHandle_t app_task_scheduler_handle = NULL;
static TraceStringHandle_t sSoftTaskTraceChannel = 0;
volatile int32_t g_appTaskSchedulerCreateResult = (int32_t)pdFAIL;

#define TRACEALYZER_STREAM_START_DELAY_MS 1500U

static void AppTaskTrace_EnsureFormats(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    if (!xTraceIsRecorderEnabled())
    {
        return;
    }

    if (sSoftTaskTraceChannel == 0U)
    {
        (void)xTraceStringRegister("SoftTask", &sSoftTaskTraceChannel);
    }
#endif
}

static void AppTaskTrace_BuildBoundaryFormat(char *buffer,
                                             uint32_t bufferSize,
                                             const char *prefix,
                                             const char *taskName)
{
    uint32_t index = 0U;
    const char *suffix = " @ %u:%02u.%03u.%03u";

    if ((buffer == NULL) || (bufferSize == 0U))
    {
        return;
    }

    if (prefix != NULL)
    {
        while ((*prefix != '\0') && (index + 1U < bufferSize))
        {
            buffer[index++] = *prefix++;
        }
    }

    if (taskName != NULL)
    {
        while ((*taskName != '\0') && (index + 1U < bufferSize))
        {
            buffer[index++] = *taskName++;
        }
    }

    while ((*suffix != '\0') && (index + 1U < bufferSize))
    {
        buffer[index++] = *suffix++;
    }

    buffer[index] = '\0';
}

static void AppTaskTrace_RegisterFormat(const char *prefix, const char *taskName, uintptr_t *targetHandle)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    TraceStringHandle_t labelHandle = 0;
    char label[80];

    if ((prefix == NULL) || (taskName == NULL) || (targetHandle == NULL) || (*targetHandle != 0U))
    {
        return;
    }

    AppTaskTrace_BuildBoundaryFormat(label, (uint32_t)sizeof(label), prefix, taskName);
    if (xTraceStringRegister(label, &labelHandle) == TRC_SUCCESS)
    {
        *targetHandle = (uintptr_t)labelHandle;
    }
#else
    (void)prefix;
    (void)taskName;
    (void)targetHandle;
#endif
}

static bool AppTaskTrace_GetRuntimeParts(uint32_t *minutes,
                                         uint32_t *seconds,
                                         uint32_t *milliseconds,
                                         uint32_t *microseconds)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    uint32_t timestamp = 0U;
    uint32_t wraparounds = 0U;
    uint32_t period = 0U;
    TraceUnsignedBaseType_t frequency = 0U;
    uint64_t wrapTicks;
    uint64_t totalTicks;
    uint64_t totalMicroseconds;
    uint64_t remainder;

    if ((minutes == NULL) || (seconds == NULL) || (milliseconds == NULL) || (microseconds == NULL))
    {
        return false;
    }

    if ((xTraceTimestampGet(&timestamp) != TRC_SUCCESS) ||
        (xTraceTimestampGetWraparounds(&wraparounds) != TRC_SUCCESS) ||
        (xTraceTimestampGetFrequency(&frequency) != TRC_SUCCESS) ||
        (xTraceTimestampGetPeriod(&period) != TRC_SUCCESS) ||
        (frequency == 0U))
    {
        return false;
    }

    /* This project runs on Cortex-M4 with a free-running DWT counter.
     * When the recorder reports period 0, the counter wraps at 2^32. */
    wrapTicks = (period == 0U) ? (((uint64_t)UINT32_MAX) + 1ULL) : (uint64_t)period;
    totalTicks = ((uint64_t)wraparounds * wrapTicks) + (uint64_t)timestamp;
    totalMicroseconds = (totalTicks * 1000000ULL) / (uint64_t)frequency;

    *minutes = (uint32_t)(totalMicroseconds / 60000000ULL);
    remainder = totalMicroseconds % 60000000ULL;
    *seconds = (uint32_t)(remainder / 1000000ULL);
    remainder %= 1000000ULL;
    *milliseconds = (uint32_t)(remainder / 1000ULL);
    *microseconds = (uint32_t)(remainder % 1000ULL);

    return true;
#else
    (void)minutes;
    (void)seconds;
    (void)milliseconds;
    (void)microseconds;
    return false;
#endif
}

static void AppTaskTrace_RegisterTaskName(task_t *task)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    if ((task == NULL) || (task->name == NULL) ||
        ((task->traceBeginHandle != 0U) &&
         (task->traceEndHandle != 0U) &&
         (task->traceCreateHandle != 0U) &&
         (task->traceStartHandle != 0U) &&
         (task->traceStopHandle != 0U) &&
         (task->traceOneShotStopHandle != 0U)))
    {
        return;
    }

    if (!xTraceIsRecorderEnabled())
    {
        return;
    }

    AppTaskTrace_EnsureFormats();
    AppTaskTrace_RegisterFormat("CREATE ", task->name, &task->traceCreateHandle);
    AppTaskTrace_RegisterFormat("START ", task->name, &task->traceStartHandle);
    AppTaskTrace_RegisterFormat("STOP ", task->name, &task->traceStopHandle);
    AppTaskTrace_RegisterFormat("ONESHOT_STOP ", task->name, &task->traceOneShotStopHandle);
    AppTaskTrace_RegisterFormat("BEGIN ", task->name, &task->traceBeginHandle);
    AppTaskTrace_RegisterFormat("END ", task->name, &task->traceEndHandle);
#else
    (void)task;
#endif
}

static void AppTaskTrace_RecordHandle(task_t *task, uintptr_t rawHandle)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U)
    TraceStringHandle_t labelHandle = (TraceStringHandle_t)rawHandle;
    uint32_t minutes = 0U;
    uint32_t seconds = 0U;
    uint32_t milliseconds = 0U;
    uint32_t microseconds = 0U;

    if (task == NULL)
    {
        return;
    }

    if (!xTraceIsRecorderEnabled())
    {
        return;
    }

    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_EnsureFormats();

    if (labelHandle == 0U)
    {
        return;
    }

    if (sSoftTaskTraceChannel != 0U)
    {
        if (AppTaskTrace_GetRuntimeParts(&minutes, &seconds, &milliseconds, &microseconds))
        {
            (void)xTracePrintF4(sSoftTaskTraceChannel,
                               labelHandle,
                               (TraceUnsignedBaseType_t)minutes,
                               (TraceUnsignedBaseType_t)seconds,
                               (TraceUnsignedBaseType_t)milliseconds,
                               (TraceUnsignedBaseType_t)microseconds);
        }
        else
        {
            (void)xTracePrintF0(sSoftTaskTraceChannel, labelHandle);
        }
    }
#else
    (void)task;
    (void)rawHandle;
#endif
}

static void AppTaskTrace_RecordBoundary(task_t *task, bool isBegin)
{
    if (task == NULL)
    {
        return;
    }

    AppTaskTrace_RecordHandle(task, isBegin ? task->traceBeginHandle : task->traceEndHandle);
}

static void AppTaskTrace_RecordCreate(task_t *task)
{
    if (task == NULL)
    {
        return;
    }

    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_RecordHandle(task, task->traceCreateHandle);
}

static void AppTaskTrace_RecordStart(task_t *task)
{
    if (task == NULL)
    {
        return;
    }

    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_RecordHandle(task, task->traceStartHandle);
}

static void AppTaskTrace_RecordStop(task_t *task)
{
    if (task == NULL)
    {
        return;
    }

    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_RecordHandle(task, task->traceStopHandle);
}

static void AppTaskTrace_RecordOneShotStop(task_t *task)
{
    if (task == NULL)
    {
        return;
    }

    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_RecordHandle(task, task->traceOneShotStopHandle);
}

static void AppTaskScheduler(void *argument)
{
    task_t *current;
    TickType_t lastWakeTime = xTaskGetTickCount();
    (void)argument;

#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U) && (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
    /*
     * 当前 PC 侧会话虽然能连上 RTT，但没有稳定地下发 Start 命令到 down buffer。
     * 因此这里在一个已经确认会运行的普通调度任务里，延时后主动直启 Streaming。
     * 这样可以避开 main() 早期阻塞，也避免单独新建启动任务带来的调度和栈不确定性。
     */
    vTaskDelay(pdMS_TO_TICKS(TRACEALYZER_STREAM_START_DELAY_MS));
    Tracealyzer_RecorderTryStartStreaming();
#endif

    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_TASK_TICK_MS));

        current = app_task_list;
        while (current != NULL)
        {
            if (current->start && (current->func != NULL) && (current->period > 0U))
            {
                current->timerTick += APP_TASK_TICK_MS;
                if (current->timerTick >= current->period)
                {
                    current->timerTick = 0U;
                    /* 统一在软调度入口记录任务起止边界，便于在 Tracealyzer 中观察 AppTask 内部切换。 */
                    AppTaskTrace_RecordBoundary(current, true);
                    current->func(0U);
                    AppTaskTrace_RecordBoundary(current, false);
                    if (current->oneShot)
                    {
                        current->start = false;
                        /* 单次软任务由调度器自动停机时补一条生命周期事件，
                         * 便于在 Tracealyzer 中区分“业务主动 Stop”和“一次性任务执行完毕”。 */
                        AppTaskTrace_RecordOneShotStop(current);
                    }
                }
            }
            current = current->next;
        }
    }
}

int app_task_create_named(task_t *task, const char *name, cbFunc func)
{
    task_t *current;

    if ((task == NULL) || (func == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();

    task->period = 0U;
    task->timerTick = 0U;
    task->oneShot = true;
    task->start = false;
    task->name = name;
    task->traceBeginHandle = 0U;
    task->traceEndHandle = 0U;
    task->traceCreateHandle = 0U;
    task->traceStartHandle = 0U;
    task->traceStopHandle = 0U;
    task->traceOneShotStopHandle = 0U;
    task->func = func;
    task->next = NULL;

    if (app_task_list == NULL)
    {
        app_task_list = task;
    }
    else
    {
        current = app_task_list;
        while (current->next != NULL)
        {
            if (current == task)
            {
                taskEXIT_CRITICAL();
                return false;
            }
            current = current->next;
        }
        if (current == task)
        {
            taskEXIT_CRITICAL();
            return false;
        }
        current->next = task;
    }

    taskEXIT_CRITICAL();
    AppTaskTrace_RegisterTaskName(task);
    AppTaskTrace_RecordCreate(task);
    return true;
}

int app_task_create(task_t *task, cbFunc func)
{
    return app_task_create_named(task, NULL, func);
}

int app_task_start(task_t *task, bool one_shot, uint32_t time_ms)
{
    if ((task == NULL) || (time_ms == 0U))
    {
        return false;
    }

    taskENTER_CRITICAL();
    task->period = time_ms;
    task->oneShot = one_shot;
    task->timerTick = 0U;
    task->start = true;
    taskEXIT_CRITICAL();

    AppTaskTrace_RecordStart(task);
    return true;
}

int app_task_stop(task_t *task)
{
    if (task == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    task->start = false;
    task->timerTick = 0U;
    taskEXIT_CRITICAL();

    AppTaskTrace_RecordStop(task);
    return true;
}

void AppTaskScheduler_Init(void)
{
    BaseType_t result;

    if (app_task_scheduler_handle != NULL)
    {
        return;
    }

    result = xTaskCreate(AppTaskScheduler,
                         "AppTask",
                         APP_TASK_STACK_DEPTH,
                         NULL,
                         APP_TASK_PRIORITY,
                         &app_task_scheduler_handle);

    /*
     * 记录任务创建结果，便于在 configASSERT 卡死时仍能从调试器看到
     * 是否是堆不足或调度任务未创建导致 Streaming 后续无事件输出。
     */
    g_appTaskSchedulerCreateResult = (int32_t)result;
    configASSERT(result == pdPASS);
}

int32_t AppTaskScheduler_CreateResult(void)
{
    return g_appTaskSchedulerCreateResult;
}
