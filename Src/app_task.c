#include "app_task.h"

#include "FreeRTOS.h"
#include "task.h"

// 定义应用程序任务相关的枚举常量，用于配置任务参数
enum
{
    APP_TASK_STACK_DEPTH = 1024,  // 定义应用程序任务栈深度为1024字节
    APP_TASK_PRIORITY = tskIDLE_PRIORITY + 3,  // 定义应用程序任务优先级，比空闲任务优先级高3级
    APP_TASK_TICK_MS = 1,  // 定义应用程序任务的时间片 tick 为1毫秒
};

static task_t *app_task_list = NULL;
static TaskHandle_t app_task_scheduler_handle = NULL;

static void AppTaskScheduler(void *argument)
{
    task_t *current;
    TickType_t lastWakeTime = xTaskGetTickCount();
    (void)argument;

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
                    current->func(0U);
                    if (current->oneShot)
                    {
                        current->start = false;
                    }
                }
            }
            current = current->next;
        }
    }
}

int app_task_create(task_t *task, cbFunc func)
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
    return true;
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

    configASSERT(result == pdPASS);
}