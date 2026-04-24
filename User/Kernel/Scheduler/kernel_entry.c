#include "kernel_entry.h"

#include "app_task.h"

void Kernel_Scheduler_Start(void)
{
    AppTaskScheduler_Init();
}
