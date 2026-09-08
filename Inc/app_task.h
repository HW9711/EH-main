#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

typedef void (*cbFunc)(uint32_t event);

/*
 * 每个业务任务分配的栈大小，单位为StackType_t；当前MCU每项4字节，1024项就是4KB。
 * 每个task_t都有一份栈：调大后每个任务都多占RAM，调小则要检查最深调用路径是否栈溢出。
 */
#ifndef APP_TASK_THREAD_STACK_DEPTH
#define APP_TASK_THREAD_STACK_DEPTH 1024U
#endif

typedef struct task_s
{
    uint32_t period;
    uint32_t timerTick;
    bool oneShot;
    bool start;
    bool threadCreated;
    const char *name;
    /*
     * 每个任务的管理信息和栈直接放在该结构体中，创建线程时不再向FreeRTOS申请堆内存。
     */
    TaskHandle_t threadHandle;
    StaticTask_t threadTcb;
    StackType_t threadStack[APP_TASK_THREAD_STACK_DEPTH];
    /*
     * 保存Tracealyzer已登记的事件文字编号，用来显示该任务何时创建、启动、停止和执行。
     * 复用这些编号，避免每次运行都重新登记相同文字。
     */
    uintptr_t traceBeginHandle;
    uintptr_t traceEndHandle;
    uintptr_t traceCreateHandle;
    uintptr_t traceStartHandle;
    uintptr_t traceStopHandle;
    uintptr_t traceOneShotStopHandle;
    cbFunc func;
    struct task_s *next;
} task_t;

/*
 * 调试器可直接查看的调度任务创建结果。
 * 与FreeRTOS定义比较：pdPASS表示创建成功，pdFAIL表示创建失败；不要把0当成成功。
 */
extern volatile int32_t g_appTaskSchedulerCreateResult;

enum APP_TASK_MODE
{
    APP_TASK_ALWAYS = 0x00,
    APP_TASK_ONE_SHOT = 0x01,
};

int app_task_create_named(task_t *task, const char *name, cbFunc func);
int app_task_create(task_t *task, cbFunc func);
int app_task_start(task_t *task, bool one_shot, uint32_t time_ms);
int app_task_stop(task_t *task);
void AppTaskScheduler_Init(void);
int32_t AppTaskScheduler_CreateResult(void);

#endif
