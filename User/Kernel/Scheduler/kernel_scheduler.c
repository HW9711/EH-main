#include "kernel_scheduler.h"

int Kernel_TaskCreate(kernel_task_t *task, kernel_task_cb_t func)
{
    return app_task_create(task, func);
}

int Kernel_TaskStart(kernel_task_t *task, bool one_shot, uint32_t time_ms)
{
    return app_task_start(task, one_shot, time_ms);
}

int Kernel_TaskStop(kernel_task_t *task)
{
    return app_task_stop(task);
}
