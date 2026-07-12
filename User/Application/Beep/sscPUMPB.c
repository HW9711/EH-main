#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "sscPUMPB.h"
#include "pump.h"
#include "pump_behavior_core.h"

/* B 泵继续使用独立队列，队列深度保持 2 条。 */
static QueueHandle_t PUMPBMsgQueue = NULL;
/* B 泵继续使用独立任务句柄和独立静态线程资源。 */
static kernel_task_t PUMPBBehaviorHandle;

/*
 * 函数功能：创建 B 泵独立消息队列，队列深度保持 2 条。
 * 输入参数：无。
 * 返回参数：无。
 */
static void PUMPBQueue_Init(void)
{
    PUMPBMsgQueue = Kernel_QueueCreate(2, sizeof(PumpBehaviorMessage_t), "PUMPBMsgQueue"); /* 保留 B 队列长度和消息布局。 */
}

/*
 * 函数功能：向 B 泵队列提交新的业务速度，相同类型和速度不重复入队。
 * 输入参数：pump_type 为调用方携带的泵类型；value 为新的业务速度。
 * 返回参数：无。
 */
void SendPumpBMessage(uint8_t pump_type, uint16_t value)
{
    static PumpBehaviorMessage_t msg; /* 保留上一次成功准备的命令，用于过滤连续重复消息。 */

    if (PUMPBMsgQueue == NULL)
    {
        return; /* 队列尚未创建时保持旧静默返回行为，不阻塞启动流程。 */
    }
    if ((msg.pump_type == pump_type) && (msg.Value == value))
    {
        return; /* 相同命令不重复入队，避免 25ms 任务反复处理同一设定。 */
    }

    msg.pump_type = pump_type; /* 保存调用方类型，仅用于下一次去重。 */
    msg.Value = value;         /* 保存本次业务速度，任务出队后只更新 speed_work。 */
    (void)Kernel_QueueSend(PUMPBMsgQueue, &msg, 0); /* 非阻塞发送，队列满时保持原失败处理。 */
}

/*
 * 函数功能：执行 B 泵一次独立任务周期。
 * 输入参数：event 为 Kernel 调度事件，当前固定不使用。
 * 返回参数：无。
 */
static void PUMPBBehaviorTask(uint32_t event)
{
    (void)event; /* 明确忽略调度事件，B 泵行为只按固定周期运行。 */
    PumpBehaviorCore_Run(PUMP_BEHAVIOR_CHANNEL_B, PUMPBMsgQueue); /* 把逻辑 B 和 B 独立队列交给共用行为内核。 */
}

/*
 * 函数功能：初始化 B 泵消息队列并按统一 25ms 周期启动 B 泵行为任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscPumpBTask_Init(void)
{
    PUMPBQueue_Init(); /* 先创建 B 独立队列，保证任务首次运行即可接收速度命令。 */
    Kernel_TaskCreate(&PUMPBBehaviorHandle, PUMPBBehaviorTask); /* 保留 B 独立任务和静态线程资源。 */
    Kernel_TaskStart(&PUMPBBehaviorHandle, KERNEL_TASK_ALWAYS, PUMP_BEHAVIOR_TASK_PERIOD_MS); /* 周期保持 25ms。 */
}
