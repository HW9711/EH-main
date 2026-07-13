#include "work_alarm.h"

#include "Pubinterface.h"

/*
 * 函数功能：写入当前工作报警编号，并同步维护报警有效标志。
 * 输入参数：alarm_value 为需要写入的报警编号，WORK_ALARM_NONE 表示无报警。
 * 返回参数：无。
 */
void WorkAlarm_Set(uint8_t alarm_value)
{
	/* 所有新报警统一写 WorkMessage，避免再通过旧报警字段分散传递。 */
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
	/* 只清理调用方拥有的报警，避免一个模块恢复时误清另一个模块仍存在的故障。 */
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
