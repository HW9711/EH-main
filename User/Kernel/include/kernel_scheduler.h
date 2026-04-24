#ifndef KERNEL_SCHEDULER_H
#define KERNEL_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "app_task.h"
#include "queue.h"

typedef task_t kernel_task_t;
typedef cbFunc kernel_task_cb_t;

enum KERNEL_TASK_MODE
{
    KERNEL_TASK_ALWAYS = APP_TASK_ALWAYS,
    KERNEL_TASK_ONE_SHOT = APP_TASK_ONE_SHOT,
};

/*
 * 软任务统一通过宏补充函数名，避免各业务模块逐个手写名称。
 * Tracealyzer 侧会把该名称写入 AppTask 的 CREATE/START/STOP/BEGIN/END 事件，
 * 从而可以在同一个 FreeRTOS AppTask 线程内部区分不同软任务的运行边界。
 */
#define Kernel_TaskCreate(task, func) Kernel_TaskCreateNamed((task), #func, (func))

int Kernel_TaskCreateNamed(kernel_task_t *task, const char *name, kernel_task_cb_t func);
int Kernel_TaskStart(kernel_task_t *task, bool one_shot, uint32_t time_ms);
int Kernel_TaskStop(kernel_task_t *task);

QueueHandle_t Kernel_QueueCreateNamed(UBaseType_t length, UBaseType_t item_size, const char *name);
BaseType_t Kernel_QueueSendNamed(QueueHandle_t queue, const void *item, TickType_t ticks_to_wait, const char *queue_name);
BaseType_t Kernel_QueueReceiveNamed(QueueHandle_t queue, void *buffer, TickType_t ticks_to_wait, const char *queue_name);

/*
 * 队列命名与收发事件统一封装在 Kernel 层：业务模块仍保持 FreeRTOS 队列语义，
 * 但 Tracealyzer 可以看到清晰的对象名，以及 SEND/RECV 的返回值和阻塞等待参数。
 */
#define Kernel_QueueCreate(length, item_size, name) Kernel_QueueCreateNamed((length), (item_size), (name))
#define Kernel_QueueSend(queue, item, ticks_to_wait) Kernel_QueueSendNamed((queue), (item), (ticks_to_wait), #queue)
#define Kernel_QueueReceive(queue, buffer, ticks_to_wait) Kernel_QueueReceiveNamed((queue), (buffer), (ticks_to_wait), #queue)

#endif
