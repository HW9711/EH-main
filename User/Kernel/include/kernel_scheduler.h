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
 * 用回调函数名作为任务名称，省去各模块重复填写名称。
 * Tracealyzer据此显示每个业务任务的创建、启动、停止和执行记录。
 */
#define Kernel_TaskCreate(task, func) Kernel_TaskCreateNamed((task), #func, (func))

int Kernel_TaskCreateNamed(kernel_task_t *task, const char *name, kernel_task_cb_t func);
int Kernel_TaskStart(kernel_task_t *task, bool one_shot, uint32_t time_ms);
int Kernel_TaskStop(kernel_task_t *task);

QueueHandle_t Kernel_QueueCreateNamed(UBaseType_t length, UBaseType_t item_size, const char *name);
BaseType_t Kernel_QueueSendNamed(QueueHandle_t queue, const void *item, TickType_t ticks_to_wait, const char *queue_name);
BaseType_t Kernel_QueueReceiveNamed(QueueHandle_t queue, void *buffer, TickType_t ticks_to_wait, const char *queue_name);

/*
 * 这些宏调用FreeRTOS队列，并额外记录队列名称、发送/接收结果和等待时间。
 * 队列满、队列空及等待规则都不变；等待时间单位仍是系统tick，不是毫秒。
 */
#define Kernel_QueueCreate(length, item_size, name) Kernel_QueueCreateNamed((length), (item_size), (name))
#define Kernel_QueueSend(queue, item, ticks_to_wait) Kernel_QueueSendNamed((queue), (item), (ticks_to_wait), #queue)
#define Kernel_QueueReceive(queue, buffer, ticks_to_wait) Kernel_QueueReceiveNamed((queue), (buffer), (ticks_to_wait), #queue)

#endif
