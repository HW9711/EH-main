#ifndef KERNEL_OSAL_H
#define KERNEL_OSAL_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

static inline void Kernel_DelayUntilMs(uint32_t delay_ms)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(delay_ms));
}

#endif
