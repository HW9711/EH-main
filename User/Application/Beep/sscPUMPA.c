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
 * 函数功能：向 A 泵队列提交新的业务速度，仅过滤已经成功入队的连续重复消息。
 * 输入参数：pump_type 为调用方携带的泵类型；value 为新的业务速度。
 * 返回参数：无。
 */
void SendPumpAMessage(uint8_t pump_type, uint16_t value)
{
    static PumpBehaviorMessage_t last_queued_msg; /* 只保存最后一次成功入队的命令，队列满时不能提前覆盖它。 */
    static uint8_t last_queued_valid = 0U; /* 0表示尚无成功入队记录，保证上电后的首条零值命令也可正常提交。 */
    PumpBehaviorMessage_t new_msg; /* 本次待提交消息使用局部副本，发送失败后调用方下周期仍可重试同一设定。 */

    if (PUMPAMsgQueue == NULL)
    {
        return; /* 队列尚未创建时保持旧静默返回行为，不阻塞启动流程。 */
    }
    if ((last_queued_valid != 0U) &&
        (last_queued_msg.pump_type == pump_type) &&
        (last_queued_msg.Value == value))
    {
        return; /* 只有相同命令已经成功排队时才去重，避免队列满后永久丢失最新设定。 */
    }

    new_msg.pump_type = pump_type; /* 把调用方泵类型写入本次候选消息，不提前改变去重状态。 */
    new_msg.Value = value; /* 把最新业务速度写入候选消息，任务出队后只更新 speed_work。 */
    if (Kernel_QueueSend(PUMPAMsgQueue, &new_msg, 0) == pdPASS)
    {
        last_queued_msg = new_msg; /* 发送成功后才更新去重基准，表示该设定已经由泵任务排队接收。 */
        last_queued_valid = 1U; /* 标记去重基准有效，后续连续相同消息可以安全过滤。 */
    }
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
