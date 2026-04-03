#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*cbFunc)(uint32_t event);

typedef struct task_s
{
    uint32_t period;
    uint32_t timerTick;
    bool oneShot;
    bool start;
    cbFunc func;
    struct task_s *next;
} task_t;

enum APP_TASK_MODE
{
    APP_TASK_ALWAYS = 0x00,
    APP_TASK_ONE_SHOT = 0x01,
};

int app_task_create(task_t *task, cbFunc func);
int app_task_start(task_t *task, bool one_shot, uint32_t time_ms);
int app_task_stop(task_t *task);
void AppTaskScheduler_Init(void);

#endif