#include "work_alarm.h"

#include "Pubinterface.h"

/*
 * 函数功能：保存报警编号，编号非零时同时标记“有报警”；不在这里发送停机命令或刷新屏幕。
 * 输入参数：alarm_value 为需要写入的报警编号，WORK_ALARM_NONE 表示无报警。
 * 返回参数：无。
 */
void WorkAlarm_Set(uint8_t alarm_value)
{
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
