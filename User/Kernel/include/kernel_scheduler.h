#ifndef KERNEL_SCHEDULER_H
#define KERNEL_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "app_task.h"

typedef task_t kernel_task_t;
typedef cbFunc kernel_task_cb_t;

enum KERNEL_TASK_MODE
{
    KERNEL_TASK_ALWAYS = APP_TASK_ALWAYS,
    KERNEL_TASK_ONE_SHOT = APP_TASK_ONE_SHOT,
};

int Kernel_TaskCreate(kernel_task_t *task, kernel_task_cb_t func);
int Kernel_TaskStart(kernel_task_t *task, bool one_shot, uint32_t time_ms);
int Kernel_TaskStop(kernel_task_t *task);

#endif
