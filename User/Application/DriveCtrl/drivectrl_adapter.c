#include "drivectrl.h"

#include "Pubinterface.h"
#include "pump.h"

#include <stddef.h>

/*
 * 驱动控制旧任务已经由 sscDRIVE/sscPUMPA/sscPUMPB 接管。
 * 当前文件只保留旧停泵接口：先清除泵运行和排空状态，再直接发送零速。
 * 是否停泵由调用方决定，不能仅凭报警名称认为一定会调用这里。
 */
/*
 * 函数功能：强制停止指定泵，并清除普通运行、定时排空和脚踏临时排空状态。
 * 输入参数：pump_msg 为待停止泵状态；set_speed 为对应泵硬件速度写入函数。
 * 返回参数：无。
 */
static void DriveCtrl_StopPump(pumpMessage_t *pump_msg, void (*set_speed)(uint32_t))
{
    /* 有泵状态时清除运行和排空记录；未传状态时仍可在下方单独发送零速。 */
    if (pump_msg != NULL)
    {
        pump_msg->run_flag = false;
        pump_msg->timingDrainage_flag = false;
        pump_msg->pedalDrainage_flag = false; /* 驱动层强制停泵时清掉脚踏轻踩来源，避免后续普通启动继承旧状态。 */
        pump_msg->timingDrainage_times = 0U;
        pump_msg->speed_work = 0U;
    }

    /* 提供了发送函数才调用，避免空函数指针导致程序异常。 */
    if (set_speed != NULL)
    {
        set_speed(0U);
    }
}

/*
 * 函数功能：清除 B 泵运行、排空和速度状态，并立即发送 B 泵零速命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void DriveCtrl_PumpFlag_B(void)
{
    DriveCtrl_StopPump(&pumpMessageB, Pump_SetSpeed_B);
}

/*
 * 函数功能：清除 A 泵运行、排空和速度状态，并立即发送 A 泵零速命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void DriveCtrl_PumpFlag_A(void)
{
    DriveCtrl_StopPump(&pumpMessageA, Pump_SetSpeed_A);
}
