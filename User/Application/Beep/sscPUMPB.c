#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "sscPUMPB.h"
#include "pump.h"
#include "pump_behavior_core.h"

/* B 泵速度队列最多存 2 条消息，不与 A 泵共用。 */
static QueueHandle_t PUMPBMsgQueue = NULL;
/* B 泵任务的调度信息，与 A 泵分开保存。 */
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
 * 函数功能：向 B 泵队列提交新设定；与上次成功入队的消息完全相同时不重复发送。
 * 输入参数：pump_type 为泵类型；value 为新设定，注水/灌注单位 mL/min，抽吸使用内部设定值。
 * 返回参数：无。
 */
void SendPumpBMessage(uint8_t pump_type, uint16_t value)
{
    static PumpBehaviorMessage_t last_queued_msg; /* 只保存最后一次成功入队的命令，队列满时不能提前覆盖它。 */
    static uint8_t last_queued_valid = 0U; /* 0表示尚无成功入队记录，保证上电后的首条零值命令也可正常提交。 */
    PumpBehaviorMessage_t new_msg; /* 本次待提交消息使用局部副本，发送失败后调用方下周期仍可重试同一设定。 */

    if (PUMPBMsgQueue == NULL)
    {
        return; /* 队列还没创建，直接返回，不等待也不保存本次消息。 */
    }
    if ((last_queued_valid != 0U) &&
        (last_queued_msg.pump_type == pump_type) &&
        (last_queued_msg.Value == value))
    {
        return; /* 只有相同命令已经成功排队时才去重，避免队列满后永久丢失最新设定。 */
    }

    new_msg.pump_type = pump_type; /* 保存本次消息的泵类型；接收端不据此改变已识别的设备类型。 */
    new_msg.Value = value; /* 保存新设定，任务取到消息后用它更新 speed_work。 */
    if (Kernel_QueueSend(PUMPBMsgQueue, &new_msg, 0) == pdPASS)
    {
        last_queued_msg = new_msg; /* 只在入队成功后记住消息；队列满时不改记录，方便下次重试。 */
        last_queued_valid = 1U; /* 已有成功入队记录，下次可据此跳过重复消息。 */
    }
}

/*
 * 函数功能：执行 B 泵一次独立任务周期。
 * 输入参数：event 为 Kernel 调度事件，当前固定不使用。
 * 返回参数：无。
 */
static void PUMPBBehaviorTask(uint32_t event)
{
    (void)event; /* 明确忽略调度事件，B 泵行为只按固定周期运行。 */
    PumpBehaviorCore_Run(PUMP_BEHAVIOR_CHANNEL_B, PUMPBMsgQueue); /* 使用公共泵处理函数，只读取 B 队列并操作 B 泵状态。 */
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
