#include "app_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "tracealyzer_recorder.h"
#include "trcRecorder.h"

// 业务线程的优先级和等待时间；修改会影响任务响应速度与CPU占用。
enum
{
    APP_TASK_PRIORITY = tskIDLE_PRIORITY + 3,  // 定义应用程序任务优先级，比空闲任务优先级高3级
    APP_TASK_IDLE_SLEEP_MS = 1,  // 任务未启动时每隔1ms检查一次；调大会延后响应start请求。
};

static task_t *app_task_list = NULL;
static SemaphoreHandle_t sAppTaskRuntimeMutex = NULL;
static StaticSemaphore_t sAppTaskRuntimeMutexBuffer;
static volatile bool sTracealyzerStreamStartAttempted = false;
static TraceStringHandle_t sSoftTaskTraceChannel = 0;
volatile int32_t g_appTaskSchedulerCreateResult = (int32_t)pdPASS;

#define TRACEALYZER_STREAM_START_DELAY_MS 1500U /* 首个业务线程在进入业务循环前等待的毫秒数，然后尝试启动RTT；未初始化记录器时这段等待仍会发生。 */

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

static TickType_t AppTask_MsToTicks(uint32_t time_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(time_ms);

    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

/*
 * 函数功能：第一个进入此处的业务线程先等待，再尝试启动RTT；其它业务线程不等待。
 * 输入参数：无。
 * 返回参数：无。
 */
static void AppTaskTrace_StartStreamingOnce(void)
{
#if (TRACEALYZER_SNAPSHOT_ENABLE == 1U) && (TRACEALYZER_TRANSPORT_MODE == TRACEALYZER_TRANSPORT_MODE_JLINK_RTT)
    bool shouldStartStreaming = false;

    /*
     * 检查和设置标志期间暂不允许任务切换，保证只有一个任务负责启动RTT。
     * 其它业务任务不需要重复等待这段启动延时。
     */
    taskENTER_CRITICAL();
    if (!sTracealyzerStreamStartAttempted)
    {
        sTracealyzerStreamStartAttempted = true;
        shouldStartStreaming = true;
    }
    taskEXIT_CRITICAL();

    if (shouldStartStreaming)
    {
        vTaskDelay(pdMS_TO_TICKS(TRACEALYZER_STREAM_START_DELAY_MS));
        Tracealyzer_RecorderTryStartStreaming();
    }
#endif
}

/*
 * 函数功能：取得公共锁后执行一次业务回调，完成后释放锁；单次任务执行后清除启动标志。
 * 输入参数：task为要执行的任务，空指针或无回调函数时不执行。
 * 返回参数：无。
 */
static void AppTaskRuntimeGate(task_t *task)
{
    if ((task == NULL) || (task->func == NULL))
    {
        return;
    }

    /*
     * 泵、脚踏、屏幕、外控虽然各有线程，但使用同一把锁，每次只允许一个业务回调运行。
     * 因此其它回调仍要等待，不能把“独立线程”理解成能同时修改公共状态。
     */
    if ((sAppTaskRuntimeMutex == NULL) ||
        (xSemaphoreTake(sAppTaskRuntimeMutex, portMAX_DELAY) != pdTRUE))
    {
        return;
    }

    AppTaskTrace_RecordBoundary(task, true);
    task->func(0U);
    AppTaskTrace_RecordBoundary(task, false);

    if (task->oneShot)
    {
        taskENTER_CRITICAL();
        task->start = false;
        task->timerTick = 0U;
        taskEXIT_CRITICAL();
        AppTaskTrace_RecordOneShotStop(task);
    }

    (void)xSemaphoreGive(sAppTaskRuntimeMutex);
}

static void AppTaskWorker(void *argument)
{
    task_t *self = (task_t *)argument;
    TickType_t lastWakeTime = xTaskGetTickCount();

    configASSERT(self != NULL);
    AppTaskTrace_StartStreamingOnce();

    for (;;)
    {
        if ((self == NULL) || (!self->start) || (self->func == NULL) || (self->period == 0U))
        {
            if (self != NULL)
            {
                self->timerTick = 0U;
            }
            lastWakeTime = xTaskGetTickCount();
            vTaskDelay(AppTask_MsToTicks(APP_TASK_IDLE_SLEEP_MS));
            continue;
        }

        vTaskDelayUntil(&lastWakeTime, AppTask_MsToTicks(self->period));

        if (self->start && (self->func != NULL) && (self->period > 0U))
        {
            self->timerTick = 0U;
            AppTaskRuntimeGate(self);
        }
    }
}

int app_task_create_named(task_t *task, const char *name, cbFunc func)
{
    task_t *current;
    const char *threadName = name;

    if ((task == NULL) || (func == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();

    task->period = 0U;
    task->timerTick = 0U;
    task->oneShot = true;
    task->start = false;
    task->threadCreated = false;
    task->name = name;
    task->threadHandle = NULL;
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

    /*
     * 每个旧软任务在注册时立即创建一个静态 FreeRTOS 线程。
     * TCB 和栈都来自 task_t 自身，避免继续消耗 FreeRTOS heap，也方便调试器按任务名单独观察。
     */
    if (threadName == NULL)
    {
        threadName = "SoftTask";
    }

    task->threadHandle = xTaskCreateStatic(AppTaskWorker,
                                           threadName,
                                           APP_TASK_THREAD_STACK_DEPTH,
                                           task,
                                           APP_TASK_PRIORITY,
                                           task->threadStack,
                                           &task->threadTcb);
    task->threadCreated = (task->threadHandle != NULL);
    if (!task->threadCreated)
    {
        g_appTaskSchedulerCreateResult = (int32_t)pdFAIL;
        return false;
    }

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

/*
 * 函数功能：创建所有业务回调共用的锁，保证同一时刻只执行一个业务回调。
 * 输入参数：无。
 * 返回参数：无；创建失败会记录pdFAIL并触发断言。
 */
void AppTaskScheduler_Init(void)
{
    if (sAppTaskRuntimeMutex != NULL)
    {
        return;
    }

    /*
     * 各业务线程已经在app_task_create_named()中创建。
     * 它们执行回调前必须先取得这把锁，防止两个回调同时修改公共状态。
     */
    sAppTaskRuntimeMutex = xSemaphoreCreateMutexStatic(&sAppTaskRuntimeMutexBuffer);
    if (sAppTaskRuntimeMutex == NULL)
    {
        g_appTaskSchedulerCreateResult = (int32_t)pdFAIL;
    }
    configASSERT(sAppTaskRuntimeMutex != NULL);
}

int32_t AppTaskScheduler_CreateResult(void)
{
    return g_appTaskSchedulerCreateResult;
}
