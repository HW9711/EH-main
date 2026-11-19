#include "work_alarm.h"

#include "Pubinterface.h"
#include "handlescan.h" /* 不支持手柄的插入状态由扫描任务锁存，统一清报警入口不能绕过。 */

/*
 * 函数功能：保存报警编号并同步有效标志；不支持手柄尚未拔出时拒绝清零或降级为普通校验提示。
 * 输入参数：alarm_value 为需要写入的报警编号，WORK_ALARM_NONE 表示无报警。
 * 返回参数：无。
 */
void WorkAlarm_Set(uint8_t alarm_value)
{
	if (Handlescan_HasUnsupportedHandle() &&
		((alarm_value == WORK_ALARM_NONE) || Pubinterface_IsHandleVerifyAlarm(alarm_value)))
	{
		alarm_value = WORK_ALARM_HANDLE_UNSUPPORTED; /* 清报警命令或另一侧校验恢复不能解除插入锁存，驱动等其它故障仍可优先上报。 */
	}
	/* 报警编号和有效标志一起更新，避免出现“有编号但未报警”的不一致状态。 */
	WorkMessage.alarm_value = alarm_value;
	WorkMessage.alarm_flag = (alarm_value != WORK_ALARM_NONE);
}

/*
 * 函数功能：清除当前工作报警。
 * 输入参数：无。
 * 返回参数：无。
 */
void WorkAlarm_Clear(void)
{
	WorkAlarm_Set(WORK_ALARM_NONE);
}

/*
 * 函数功能：仅当当前报警属于调用方指定编号时清除，避免误清其它模块报警。
 * 输入参数：alarm_value 为允许清除的报警编号。
 * 返回参数：无。
 */
void WorkAlarm_ClearIf(uint8_t alarm_value)
{
	/* 只有当前报警号与参数一致才清除，避免某个模块恢复时把其他故障一起清掉。 */
	if ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value))
	{
		WorkAlarm_Clear();
	}
}

/*
 * 函数功能：判断当前有效报警是否为指定编号。
 * 输入参数：alarm_value 为需要检查的报警编号。
 * 返回参数：true 表示当前正是该报警，false 表示不是。
 */
bool WorkAlarm_Is(uint8_t alarm_value)
{
	return ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value));
}

/*
 * 函数功能：识别手柄IIC通信报警，供屏幕选75号图及双通道报警合并使用。
 * 输入参数：alarm_value为主控内部报警码。
 * 返回参数：true表示A/B/双路IIC通信失败，false表示其它报警。
 */
bool WorkAlarm_IsHandleIicError(uint8_t alarm_value)
{
    return ((alarm_value == WORK_ALARM_HANDLE_IIC_ERROR_A) || /* A通道通信失败。 */
            (alarm_value == WORK_ALARM_HANDLE_IIC_ERROR_B) || /* B通道通信失败。 */
            (alarm_value == WORK_ALARM_HANDLE_IIC_ERROR_AB)); /* 双路通信失败仍只显示75号图。 */
}

/*
 * 函数功能：识别EEPROM通信或数据校验报警，使75/90共用恢复及写屏重试流程。
 * 输入参数：alarm_value为主控内部报警码。
 * 返回参数：true表示EEPROM读取或数据校验失败，false表示其它逻辑报警。
 */
bool WorkAlarm_IsHandleEepromError(uint8_t alarm_value)
{
    return (WorkAlarm_IsHandleIicError(alarm_value) || /* EEPROM通信失败使用75号图，与数据校验失败共用恢复流程。 */
            (alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_A) || /* A通道数据校验失败显示90。 */
            (alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_B) || /* B通道数据校验失败显示90。 */
            (alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_AB)); /* 双路数据校验失败显示90。 */
}
