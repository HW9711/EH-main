#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

typedef void (*cbFunc)(uint32_t event);

/*
 * APP_TASK_THREAD_STACK_DEPTH 是每个软任务迁移成独立 FreeRTOS 线程后的静态栈深度，单位为 StackType_t 字。
 * 原来所有软任务共用一个 AppTask 线程栈；现在每个 task_t 自带一份栈，方便调试器和 Tracealyzer 分别观察。
 * 这里保持 1024 words，优先保证旧业务回调的栈空间不因为拆线程而变小。
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
     * 独立线程运行资源全部静态保存在 task_t 内部。
     * 业务模块原来已经声明了全局 kernel_task_t/task_t 句柄，因此这里扩展结构体即可完成静态 TCB/栈分配，
     * 不需要额外集中数组，也不会引入 FreeRTOS heap 消耗。
     */
    TaskHandle_t threadHandle;
    StaticTask_t threadTcb;
    StackType_t threadStack[APP_TASK_THREAD_STACK_DEPTH];
    /*
     * Tracealyzer 事件格式句柄缓存。
     * AppTask 是一个承载多个软任务的真实 FreeRTOS 任务，单靠系统任务名无法区分内部软任务；
     * 因此每个软任务保存 CREATE/START/STOP/BEGIN/END 等格式句柄，用于在用户事件通道中标记
     * 软任务生命周期和执行边界，避免运行中反复注册字符串造成额外开销。
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
 * 0 通常表示 pdPASS；非 0 表示 xTaskCreate 失败。
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
