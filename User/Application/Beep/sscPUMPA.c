#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "sscPUMPA.h"
#include "pump.h"
#include "pump_behavior_core.h"

/* A 泵继续使用独立任务句柄，不能与 B 泵任务合并。 */
kernel_task_t PUMPABehaviorHandle;
/* A 泵队列保持全局符号和原有长度，避免改变现有调试观察点。 */
QueueHandle_t PUMPAMsgQueue = NULL;

/*
 * 函数功能：创建 A 泵独立消息队列，队列深度保持 5 条。
 * 输入参数：无。
 * 返回参数：无。
 */
static void PUMPAQueue_Init(void)
{
    PUMPAMsgQueue = Kernel_QueueCreate(5, sizeof(PumpBehaviorMessage_t), "PUMPAMsgQueue"); /* 保留 A 队列长度和消息布局。 */
}

/*
 * 函数功能：向 A 泵队列提交新的业务速度，相同类型和速度不重复入队。
 * 输入参数：pump_type 为调用方携带的泵类型；value 为新的业务速度。
 * 返回参数：无。
 */
void SendPumpAMessage(uint8_t pump_type, uint16_t value)
{
    static PumpBehaviorMessage_t msg; /* 保留上一次成功准备的命令，用于过滤连续重复消息。 */

    if (PUMPAMsgQueue == NULL)
    {
        return; /* 队列尚未创建时保持旧静默返回行为，不阻塞启动流程。 */
    }
    if ((msg.pump_type == pump_type) && (msg.Value == value))
    {
        return; /* 相同命令不重复入队，避免 25ms 任务反复处理同一设定。 */
    }

    msg.pump_type = pump_type; /* 保存调用方类型，仅用于下一次去重。 */
    msg.Value = value;         /* 保存本次业务速度，任务出队后只更新 speed_work。 */
    (void)Kernel_QueueSend(PUMPAMsgQueue, &msg, 0); /* 非阻塞发送，队列满时保持原失败处理。 */
}

/*
 * 函数功能：执行 A 泵一次独立任务周期。
 * 输入参数：event 为 Kernel 调度事件，当前固定不使用。
 * 返回参数：无。
 */
void PUMPABehaviorTask(uint32_t event)
{
    (void)event; /* 明确忽略调度事件，A 泵行为只按固定周期运行。 */
    PumpBehaviorCore_Run(PUMP_BEHAVIOR_CHANNEL_A, PUMPAMsgQueue); /* 把逻辑 A 和 A 独立队列交给共用行为内核。 */
}

/*
 * 函数功能：初始化 A 泵消息队列并按统一 25ms 周期启动 A 泵行为任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscPumpATask_Init(void)
{
    PUMPAQueue_Init(); /* 先创建 A 独立队列，保证任务首次运行即可接收速度命令。 */
    Kernel_TaskCreate(&PUMPABehaviorHandle, PUMPABehaviorTask); /* 保留 A 独立任务和静态线程资源。 */
    Kernel_TaskStart(&PUMPABehaviorHandle, KERNEL_TASK_ALWAYS, PUMP_BEHAVIOR_TASK_PERIOD_MS); /* 周期保持 25ms。 */
}
