#include "drivectrl.h"

#include "Pubinterface.h"
#include "pump.h"

#include <stddef.h>

/*
 * 驱动控制旧任务已经由 sscDRIVE/sscPUMPA/sscPUMPB 接管。
 * 当前文件只保留报警模块仍在使用的停泵适配入口：先清公共泵状态，
 * 再直接下发 0 速命令，保证报警发生时泵输出在本周期内被压到安全态。
 */
/*
 * 函数功能：强制停止指定泵，并清除普通运行、定时排空和脚踏临时排空状态。
 * 输入参数：pump_msg 为待停止泵状态；set_speed 为对应泵硬件速度写入函数。
 * 返回参数：无。
 */
static void DriveCtrl_StopPump(pumpMessage_t *pump_msg, void (*set_speed)(uint32_t))
{
    if (pump_msg != NULL)
    {
        pump_msg->run_flag = false;
        pump_msg->timingDrainage_flag = false;
        pump_msg->pedalDrainage_flag = false; /* 驱动层强制停泵时清掉脚踏临时70档，避免后续普通启动继承旧排空状态。 */
        pump_msg->timingDrainage_times = 0U;
        pump_msg->speed_work = 0U;
    }

    if (set_speed != NULL)
    {
        set_speed(0U);
    }
}

void DriveCtrl_PumpFlag_B(void)
{
    DriveCtrl_StopPump(&pumpMessageB, Pump_SetSpeed_B);
}

void DriveCtrl_PumpFlag_A(void)
{
    DriveCtrl_StopPump(&pumpMessageA, Pump_SetSpeed_A);
}
