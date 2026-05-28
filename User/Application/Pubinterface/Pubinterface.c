// 调用对应头文件
#include "Pubinterface.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
// #include <screen.h>
// #include "datahand.h"
#include "sscDRIVE.h"
#include "sscUIDP.h"
#include "sscBEEP.h"

ChannelrecognizeMessage_t ChannelrecognizeMessageA;
ChannelrecognizeMessage_t ChannelrecognizeMessageB;
ControlSigleMessage_t ControlSigleMssage;
ControlSignalMessage_t ControlSignalMessage;

WorkMessage_t WorkMessage;		   // 工作信息
ChannelMemoryMessagr_t MemoryMsgA; // 通道记忆（增对可调节参数），用于切换手柄
ChannelMemoryMessagr_t MemoryMsgB; //
pumpMessage_t pumpMessageA;
pumpMessage_t pumpMessageB;
/* 当前控制权持有者，四种控制方式必须等待当前持有者结束后才能重新申请。 */
static volatile uint8_t s_control_owner = CONTROL_OWNER_NONE;
/* Page4 速度/频率阈值只驱动蜂鸣，不写 WorkMessage.alarm_flag，避免阈值提示阻塞降速操作。 */
static uint8_t s_speed_threshold_beep_active = 0U;
/* 阈值蜂鸣保持期间定时重发报警消息，避免普通按键蜂鸣覆盖后阈值提示静音。 */
static uint8_t s_speed_threshold_beep_refresh_ticks = 0U;

/* 电机反馈小于等于该阈值时认为机械输出已停止；0 表示必须等驱动反馈真实归零。 */
#define CONTROL_ARBITRATION_MOTOR_STOP_SPEED_THRESHOLD 0U

static void Pubinterface_SendHandleDisplay(uint8_t channel, uint8_t handle_model, bool enable_flag, bool light_flag)
{
	uint8_t display_value[10] = {0U};

	/* Value[0] 传手柄类型，sscUIDP::UIHANDLEDP() 用它决定显示哪一种手柄图标。 */
	display_value[0] = handle_model;
	/* Value[1] 传 A/B 通道号，1 表示 A 通道，2 表示 B 通道。 */
	display_value[1] = channel;
	/* Value[2] 传选中高亮状态，当前工作通道亮起，非当前通道只显示在线。 */
	display_value[2] = light_flag ? 1U : 0U;
	/* 手柄插拔事件已经完成状态更新后，通过 UIDP 队列刷新屏幕手柄区域。 */
	SendUIDSMessage(UI_HANDLE_ID, enable_flag, display_value);
}

static void Pubinterface_RefreshOnlineHandleDisplay(void)
{
	/* A 通道在线时刷新 A 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Aonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_A,
									   MemoryMsgA.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_A));
	}

	/* B 通道在线时刷新 B 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Bonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_B,
									   MemoryMsgB.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_B));
	}
}
void ChannelrecognizeMessageInit(void)
{
	memset(&ChannelrecognizeMessageA, 0, sizeof(ChannelrecognizeMessageA));
	memset(&ChannelrecognizeMessageB, 0, sizeof(ChannelrecognizeMessageB));
}
void WorkMessageInit(void)
{
	memset(&WorkMessage, 0, sizeof(WorkMessage));
}
void ChannelMemoryMessageInit(void)
{
	memset(&MemoryMsgA, 0, sizeof(MemoryMsgA));
	memset(&MemoryMsgB, 0, sizeof(MemoryMsgB));
}
void pumpMessageInit(void)
{
	memset(&pumpMessageA, 0, sizeof(pumpMessageA));
	memset(&pumpMessageB, 0, sizeof(pumpMessageB));
}

/*
 * 函数功能：读取指定通道从 EEPROM Page4 解析出的默认注水流量，供手柄上线、脚踏和外控默认启动注水泵共用。
 * 输入参数：channel 通道号，CHANNEL_A 读取 A 通道记忆，CHANNEL_B 读取 B 通道记忆。
 * 返回参数：默认注水流量，单位为当前泵业务流量值；通道无效时返回 0。
 */
uint16_t Pubinterface_GetChannelDefaultInjectionFlow(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return MemoryMsgA.default_injection_flow; /* A 通道返回本通道 Page4 默认流量，避免 A/B 手柄默认流量串用。 */
	}

	if (channel == CHANNEL_B)
	{
		return MemoryMsgB.default_injection_flow; /* B 通道返回本通道 Page4 默认流量，避免 A/B 手柄默认流量串用。 */
	}

	return 0U; /* 当前没有有效通道时不补注水默认值，避免无手柄时误启动注水泵。 */
}

/*
 * 函数功能：读取当前工作通道的默认注水流量。
 * 输入参数：无，函数内部读取 WorkMessage.channel_work。
 * 返回参数：当前工作通道默认注水流量；当前没有 A/B 工作通道时返回 0。
 */
uint16_t Pubinterface_GetCurrentDefaultInjectionFlow(void)
{
	return Pubinterface_GetChannelDefaultInjectionFlow(WorkMessage.channel_work); /* 注水默认流量跟随当前工作手柄，而不是固定跟随某个泵口。 */
}

/*
 * 函数功能：读取当前工作通道当前方向的默认电机速度，供脚踏启动时替代旧的固定 60000。
 * 输入参数：无，函数内部读取 WorkMessage.channel_work 和 WorkMessage.dir_work。
 * 返回参数：当前方向默认速度，单位沿用 WorkMessage.speed_set_work 的 x10 单位；无有效通道时返回 0。
 */
uint16_t Pubinterface_GetCurrentDefaultMotorSpeed(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前工作通道记忆结构，后续按方向读取该通道 Page4 默认速度。 */

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 当前工作通道为 A 时，默认速度来自 A 通道 Page4 解析结果。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 当前工作通道为 B 时，默认速度来自 B 通道 Page4 解析结果。 */
	}
	else
	{
		return 0U; /* 无工作通道时不能给电机补默认速度，避免未选中手柄时误运行。 */
	}

	if (WorkMessage.dir_work == FZDIR)
	{
		return memory->fz_speed; /* 反转时返回 Page4 默认速度装载到反转记忆后的值。 */
	}

	if (WorkMessage.dir_work == OSCDIR)
	{
		return memory->osc_speed; /* 往复时返回 Page4 默认速度装载到往复记忆后的值。 */
	}

	return memory->zz_speed; /* 默认按正转处理，和 Page4 默认方向正转保持一致。 */
}

/*
 * 函数功能：手柄上线时把当前通道 Page4 默认注水流量写入仍为 0 的注水泵速度。
 * 输入参数：channel 本次上线的手柄通道。
 * 返回参数：无。
 */
static void Pubinterface_ApplyChannelDefaultInjectionFlow(uint8_t channel)
{
	uint16_t default_flow = Pubinterface_GetChannelDefaultInjectionFlow(channel); /* 读取本通道 Page4 默认注水流量，解析阶段已经完成 /10 和 0~70 钳位。 */

	if ((pumpMessageA.type == INJECTWATER) && (pumpMessageA.speed_work == 0U))
	{
		pumpMessageA.speed_work = default_flow; /* A 泵是注水泵且尚未设置流量时，用当前手柄默认流量初始化。 */
	}

	if ((pumpMessageB.type == INJECTWATER) && (pumpMessageB.speed_work == 0U))
	{
		pumpMessageB.speed_work = default_flow; /* B 泵是注水泵且尚未设置流量时，用当前手柄默认流量初始化。 */
	}
}

/*
 * 函数功能：判断报警码是否属于手柄 EEPROM 识别/校验失败系列报警。
 * 输入参数：alarm_value 当前报警码，来自 WorkMessage.alarm_value 或临时提示报警。
 * 返回参数：true 表示这是 A/B/AB 手柄校验报警，false 表示普通阻塞报警。
 */
bool Pubinterface_IsHandleVerifyAlarm(uint8_t alarm_value)
{
	return ((alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_A) ||  /* A 通道 EEPROM 校验失败时，不阻塞 B 通道继续识别。 */
			(alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_B) ||  /* B 通道 EEPROM 校验失败时，不阻塞 A 通道继续识别。 */
			(alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_AB));  /* A/B 均失败时仍属于手柄校验系列，拔出任一坏手柄后再降级报警。 */
}

/*
 * 函数功能：把指定通道的 MemoryMsg 记忆参数装载到当前 WorkMessage 工作快照。
 * 输入参数：channel 目标通道，CHANNEL_A 表示装载 MemoryMsgA，CHANNEL_B 表示装载 MemoryMsgB。
 * 返回参数：无。
 */
void Pubinterface_LoadChannelMemory(uint8_t channel)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向即将成为当前工作快照的通道记忆结构。 */

	if (channel == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 手动或自动切到 A 通道时，所有运行参数都来自 A 通道记忆。 */
	}
	else if (channel == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 手动或自动切到 B 通道时，所有运行参数都来自 B 通道记忆。 */
	}
	else
	{
		return; /* 无效通道不改 WorkMessage，避免把当前有效工作态清成不可预期状态。 */
	}

	WorkMessage.current_work = memory->current_work;			  /* 同步当前通道保护电流，后续启动时按该手柄限流。 */
	WorkMessage.tool_reduction_ratio = memory->tool_reduction_ratio; /* 同步刀具减速比，保证速度换算跟随通道。 */
	WorkMessage.dir_work = memory->dir;						  /* 同步当前方向，速度选择依赖这个方向字段。 */
	WorkMessage.drivetype_work = memory->drive_type;			  /* 同步控制方式，切换通道后脚控/手控状态来自通道记忆。 */
	WorkMessage.freq_work = memory->freq;						  /* 同步往复频率，避免 A/B 频率串用。 */
	WorkMessage.tool_type = memory->tool_type;					  /* 同步刨/磨刀具类型，界面和限速逻辑都读取这里。 */
	WorkMessage.hand_model = memory->hand_model;				  /* 同步手柄型号，手柄按键扫描依赖当前型号判断。 */
	WorkMessage.channel_work = channel;						  /* 最后切换当前工作通道，避免中间状态被其它任务读成新通道旧参数。 */

	if (WorkMessage.dir_work == ZZDIR)
	{
		WorkMessage.speed_set_work = memory->zz_speed;			  /* 正转时装载本通道正转速度。 */
	}
	else if (WorkMessage.dir_work == FZDIR)
	{
		WorkMessage.speed_set_work = memory->fz_speed;			  /* 反转时装载本通道反转速度。 */
	}
	else
	{
		WorkMessage.speed_set_work = memory->osc_speed;		  /* 往复或异常方向按往复速度装载，保持旧逻辑兼容。 */
	}

	WorkMessage.speed_work = WorkMessage.speed_set_work;		  /* 切换通道后实际速度同步到设置速度，避免沿用上一个通道输出值。 */
}

/*
 * 函数功能：判断新插入且校验通过的通道是否允许自动成为当前选中通道。
 * 输入参数：channel 本次插入并已写入 MemoryMsg 的通道。
 * 返回参数：true 表示可以自动选中，false 表示只更新通道记忆等待用户手动切换。
 */
static bool Pubinterface_ShouldAutoSelectPluggedChannel(uint8_t channel)
{
	if (((channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == false)) ||
		((channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == false)))
	{
		return false; /* 插拔事件还没有把在线标志置位时，不允许装载该通道，避免选中一个未上线通道。 */
	}

	if (WorkMessage.runflag_work == false)
	{
		if ((WorkMessage.alarm_flag == true) && (Pubinterface_IsHandleVerifyAlarm(WorkMessage.alarm_value) == false))
		{
			return false; /* 普通系统报警期间暂停自动选中，等待报警解除后再由用户确认。 */
		}

		return true; /* 非运行状态下，最后一个校验通过的插入通道自动成为选中通道。 */
	}

	return false; /* 运行中插入另一路有效手柄只更新 MemoryMsg，不抢占当前工作通道。 */
}

/*
 * 函数功能：把扫描层 ChannelrecognizeMessage 的识别结果写入对应 MemoryMsg 通道记忆。
 * 输入参数：channel 本次插入并认证通过的通道。
 * 返回参数：无。
 */
static void Pubinterface_SaveRecognizeToMemory(uint8_t channel)
{
	ChannelrecognizeMessage_t *recognize = NULL; /* 指向扫描层刚刚完成认证的临时识别缓存。 */
	ChannelMemoryMessagr_t *memory = NULL;		  /* 指向要更新的 A/B 通道记忆结构。 */

	if (channel == CHANNEL_A)
	{
		recognize = &ChannelrecognizeMessageA;	  /* A 通道上线时，只读取 A 通道扫描缓存。 */
		memory = &MemoryMsgA;					  /* A 通道上线时，只写入 A 通道记忆。 */
	}
	else if (channel == CHANNEL_B)
	{
		recognize = &ChannelrecognizeMessageB;	  /* B 通道上线时，只读取 B 通道扫描缓存。 */
		memory = &MemoryMsgB;					  /* B 通道上线时，只写入 B 通道记忆。 */
	}
	else
	{
		return;									  /* 非 A/B 通道不写通道记忆，避免破坏当前工作态。 */
	}

	memset(memory, 0, sizeof(*memory));			  /* 新手柄上线前清掉该通道旧 EEPROM 参数，避免旧字段残留参与后续切换。 */
	memory->hand_model = recognize->handle_type;  /* 保存手柄型号，后续通道切换时再装载到 WorkMessage。 */
	memory->hand_type_raw_major = recognize->hand_type_raw_major; /* 保存 EEPROM 原始主类型，上位机心跳需要区分真实编码。 */
	memory->hand_type_raw_minor = recognize->hand_type_raw_minor; /* 保存 EEPROM 原始子类型，便于上位机显示和售后定位。 */
	memory->current_work = recognize->overloadThresholdFor;		  /* 保存保护电流，当前通道被选中后才影响 WorkMessage。 */
	memory->tool_reduction_ratio = recognize->meioticratio;		  /* 保存减速比，避免 A/B 刀具参数串用。 */
	memory->default_injection_flow = recognize->default_injection_flow; /* 保存 Page4 默认注水流量，选中该通道时初始化泵。 */
	memory->speed_alarm_for = recognize->speed_alarm_for;		  /* 保存正转速度报警阈值，运行阈值检查按当前通道读取。 */
	memory->speed_alarm_rev = recognize->speed_alarm_rev;		  /* 保存反转速度报警阈值，运行阈值检查按当前通道读取。 */
	memory->freq_alarm_osc = recognize->freq_alarm_osc;			  /* 保存往复频率报警阈值，运行阈值检查按当前通道读取。 */
	memory->dir = recognize->run_direction;						  /* 保存默认方向，后续装载 WorkMessage 时据此选速度。 */
	memory->freq = recognize->freq_default;						  /* 保存默认往复频率，避免插入未选中通道时改当前频率。 */
	memory->zz_speed = recognize->speed_zzdefault;				  /* 保存正转默认速度，只影响本通道记忆。 */
	memory->fz_speed = recognize->speed_fzdefault;				  /* 保存反转默认速度，只影响本通道记忆。 */
	memory->osc_speed = recognize->speed_oscdefault;			  /* 保存往复默认速度，只影响本通道记忆。 */
	memory->tool_type = recognize->tool_type;					  /* 保存刀具类型，切换到该通道后再影响当前工具显示。 */

	if (memory->hand_model == PXBA_ONLINES)
	{
		ControlSignalMessage.HMI_enable_flag = true;			  /* 带手控按键手柄上线后打开手控可用显示，但是否选中由通道策略决定。 */
		if ((WorkMessage.drivetype_work == TOUCHWORK) || (WorkMessage.drivetype_work == JTWORK))
		{
			memory->drive_type = WorkMessage.drivetype_work;	  /* 当前处于触控或脚控时，新通道记忆跟随当前控制方式，避免自动改手控。 */
		}
		else
		{
			memory->drive_type = HANDLEWORK;					  /* 没有其它控制方式占用时，带按键手柄默认记忆为手控。 */
		}
	}
	else
	{
		memory->drive_type = WorkMessage.drivetype_work;		  /* 非按键手柄保留当前控制方式记忆，切换通道时不额外改变控制来源。 */
	}
}

/*
 * 函数功能：按 Page4 的 2 字节速度报警阈值计算 WorkMessage 内部 x10 速度阈值。
 * 输入参数：alarm_value Page4 正转/反转速度报警阈值，单位与 WorkMessage.speed_work 一致，例如 55000 表示 5500.0rpm。
 * 返回参数：可与 WorkMessage.speed_work 直接比较的 x10 速度阈值；0 表示阈值关闭。
 */
static uint16_t Pubinterface_BuildSpeedAlarmThreshold(uint16_t alarm_value)
{
	return alarm_value; /* Page4 已按内部 x10 速度保存，直接返回可避免 1 字节截断导致阈值偏低。 */
}

/*
 * 函数功能：检查当前运行速度或往复频率是否触发 Page4 阈值，触发时只驱动蜂鸣器，不强制停机。
 * 输入参数：无，函数内部读取 WorkMessage 和当前通道记忆结构。
 * 返回参数：无。
 */
void Pubinterface_CheckSpeedThresholdAlarm(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前工作通道的 Page4 阈值记忆，用于区分 A/B 手柄。 */
	uint16_t threshold_alarm = 0U;		   /* 保存当前方向下 Page4 配置的速度/频率报警阈值，0 表示不报警。 */
	uint8_t should_beep = 0U;			   /* 保存本周期是否需要速度/频率阈值蜂鸣。 */

	if (WorkMessage.alarm_flag)
	{
		s_speed_threshold_beep_active = 0U;		   /* 真实系统报警已经接管蜂鸣时，本提示报警让位，避免清掉其它报警蜂鸣。 */
		s_speed_threshold_beep_refresh_ticks = 0U; /* 系统报警期间清掉本模块重发计数，恢复后重新判断阈值。 */
		return;
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 当前工作通道为 A 时，使用 A 通道 Page4 阈值。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 当前工作通道为 B 时，使用 B 通道 Page4 阈值。 */
	}
	else
	{
		memory = NULL; /* 无工作通道时没有可比较的 Page4 阈值。 */
	}

	if ((WorkMessage.runflag_work == true) && (memory != NULL))
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			threshold_alarm = memory->speed_alarm_for; /* 正转使用 Page4 正转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= Pubinterface_BuildSpeedAlarmThreshold(threshold_alarm)); /* 内部 x10 速度超过阈值时蜂鸣。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			threshold_alarm = memory->speed_alarm_rev; /* 反转使用 Page4 反转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= Pubinterface_BuildSpeedAlarmThreshold(threshold_alarm)); /* 内部 x10 速度超过阈值时蜂鸣。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			threshold_alarm = memory->freq_alarm_osc; /* 往复使用 Page4 往复转频率报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.freq_work >= threshold_alarm); /* 频率阈值与 WorkMessage.freq_work 使用同一 1 字节业务单位。 */
		}
	}

	if ((should_beep != 0U) && (s_speed_threshold_beep_active == 0U))
	{
		SendAlarmMessage(WORK_ALARM_SPEED_THRESHOLD); /* 阈值触发时只通知蜂鸣任务报警，不写 WorkMessage.alarm_flag，因此不强制停机。 */
		s_speed_threshold_beep_active = 1U;			  /* 记录本模块已经占用蜂鸣，避免每个周期重复投递队列。 */
		s_speed_threshold_beep_refresh_ticks = 0U;	  /* 第一次触发后从 0 开始计数，后续按周期重发保持蜂鸣状态。 */
	}
	else if ((should_beep != 0U) && (s_speed_threshold_beep_active != 0U))
	{
		++s_speed_threshold_beep_refresh_ticks; /* 阈值持续触发时累计周期，避免每 50ms 重发导致蜂鸣队列堆积。 */
		if (s_speed_threshold_beep_refresh_ticks >= 10U)
		{
			SendAlarmMessage(WORK_ALARM_SPEED_THRESHOLD); /* 约 500ms 重发一次阈值蜂鸣，防止普通按键蜂鸣覆盖报警状态。 */
			s_speed_threshold_beep_refresh_ticks = 0U;	  /* 重发后清零计数，下一轮继续按低频率保持报警。 */
		}
	}
	else if ((should_beep == 0U) && (s_speed_threshold_beep_active != 0U))
	{
		SendAlarmMessage(WORK_ALARM_NONE);		   /* 阈值恢复且没有其它系统报警时，释放本模块蜂鸣。 */
		s_speed_threshold_beep_active = 0U;		   /* 清除本模块蜂鸣占用状态，下一次超过阈值可重新触发。 */
		s_speed_threshold_beep_refresh_ticks = 0U; /* 阈值恢复后清掉重发计数，避免下一次触发立即重发。 */
	}
}

void WorkAlarm_Set(uint8_t alarm_value)
{
	/* 所有新报警统一写 WorkMessage，避免再通过旧报警字段分散传递。 */
	WorkMessage.alarm_value = alarm_value;
	WorkMessage.alarm_flag = (alarm_value != WORK_ALARM_NONE);
}

void WorkAlarm_Clear(void)
{
	WorkAlarm_Set(WORK_ALARM_NONE);
}

void WorkAlarm_ClearIf(uint8_t alarm_value)
{
	/* 只清理调用方拥有的报警，避免一个模块恢复时误清另一个模块仍存在的故障。 */
	if ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value))
	{
		WorkAlarm_Clear();
	}
}

bool WorkAlarm_Is(uint8_t alarm_value)
{
	return ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value));
}

/*
 * 控制信号结构体目前仍处于新旧模块并行迁移阶段，保留 ChannelMessageInit()
 * 与 ChannelFlagMessageInit() 两个历史声明入口，统一清零到新的控制信号容器。
 * 这样 userparser 或后续模块无论调用哪个初始化名，都不会留下未初始化的控制标志。
 */
static void ControlMessageInit(void)
{
	memset(&ControlSignalMessage, 0, sizeof(ControlSignalMessage));
	memset(&ControlSigleMssage, 0, sizeof(ControlSigleMssage));
}

/*
 * 函数功能：清除当前电机、脚踏、外控和 A/B 泵输出状态，用于外控进入、退出和超时安全停机。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ControlArbitration_StopMotionOutput(void)
{
	/* 仲裁切换控制权时，先停电机，避免上一个控制源留下运行状态。 */
	WorkMessage.runflag_work = false;
	/* 实际输出速度清零，驱动任务下一周期会按停止状态下发。 */
	WorkMessage.speed_work = 0U;
	/* 清除手柄按键运行标志，防止退出外控后沿用旧的手柄启动状态。 */
	ControlSignalMessage.handle_control_flag = false;
	/* 清除外部控制运行标志，真正的控制权由 s_control_owner 统一表达。 */
	ControlSignalMessage.HMI_control_flag = false;
	/* 清除左右脚踏电机控制标志，避免外控期间脚踏状态残留。 */
	ControlSignalMessage.jtL_control_flag = false;
	ControlSignalMessage.jtR_control_flag = false;
	/* 清除三类轻排泵运行标志，使泵状态和电机状态一起回到空闲。 */
	ControlSignalMessage.jtL_gentlypump_flag = false;
	ControlSignalMessage.jtR_gentlypump_flag = false;
	ControlSignalMessage.HMI_gentlypump_flag = false;
	/* 清除外控泵启动标志，避免 A/B 泵被旧外控请求继续占用。 */
	ControlSignalMessage.HMIL_pump_flag = false;
	ControlSignalMessage.HMIR_pump_flag = false;
	/* 停止 A 泵输出，并取消排空计时，避免退出外控后继续出水。 */
	pumpMessageA.run_flag = false;
	pumpMessageA.timingDrainage_flag = false;
	pumpMessageA.timingDrainage_times = 0U;
	pumpMessageA.speed_work = 0U;
	/* 停止 B 泵输出，并取消排空计时，保持两路泵的仲裁动作一致。 */
	pumpMessageB.run_flag = false;
	pumpMessageB.timingDrainage_flag = false;
	pumpMessageB.timingDrainage_times = 0U;
	pumpMessageB.speed_work = 0U;
}

/*
 * 函数功能：外部控制退出后，根据当前通道记忆和手柄在线类型计算应恢复的本机控制模式。
 * 输入参数：无，读取 WorkMessage.channel_work、MemoryMsgA/B.drive_type 和 WorkMessage.hand_model。
 * 返回参数：JTWORK/HANDLEWORK/NOWORK，表示外控退出后的本机驱动方式。
 */
static uint8_t ControlArbitration_GetLocalDriveTypeAfterExit(void)
{
	uint8_t drive_type = NOWORK;

	/* 退出外控后优先恢复当前通道自己的记忆控制方式。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		drive_type = MemoryMsgA.drive_type;
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		drive_type = MemoryMsgB.drive_type;
	}

	/* 只允许恢复脚踏或手柄，本次退出不能继续停留在 TOUCHWORK 外控模式。 */
	if ((drive_type == JTWORK) || (drive_type == HANDLEWORK))
	{
		return drive_type;
	}

	/* 当前是带按键手柄时，给本机手柄控制一个自然的回退入口。 */
	if ((WorkMessage.hand_model == PXBA_ONLINES) || (WorkMessage.hand_model == PXBB_ONLINES))
	{
		return HANDLEWORK;
	}

	/* 没有有效本机控制模式时保持空闲，等待脚踏或屏幕重新选择。 */
	return NOWORK;
}

/*
 * 函数功能：判断驱动板反馈的实际转速是否仍表示电机在转动。
 * 输入参数：无，直接读取 WorkMessage.driver_speed_feedback。
 * 返回参数：true 表示驱动反馈转速仍大于停止阈值；false 表示反馈已经低于停止阈值。
 */
static bool ControlArbitration_IsMotorFeedbackActive(void)
{
	/* 反馈转速来自 UART1 驱动板回包，用于避免刚下发停止命令但电机还未真实停稳时提前释放控制权。 */
	return (WorkMessage.driver_speed_feedback > CONTROL_ARBITRATION_MOTOR_STOP_SPEED_THRESHOLD);
}

/*
 * 函数功能：判断手柄电机当前是否处于忙状态。
 * 输入参数：无，读取 runflag_work 命令标志和 driver_speed_feedback 实际反馈。
 * 返回参数：true 表示电机被要求运行或反馈仍在转动；false 表示命令停止且反馈已停稳。
 */
static bool ControlArbitration_IsMotorBusy(void)
{
	/* runflag_work 是主控下发运行帧的命令源，置位时必须认为电机正在被控制。 */
	if (WorkMessage.runflag_work == true)
	{
		return true;
	}

	/* runflag_work 清零后继续看驱动反馈，保证仲裁释放跟真实停转绑定。 */
	return ControlArbitration_IsMotorFeedbackActive();
}

/*
 * 函数功能：当本地电机已经停稳时自动释放脚踏、屏幕或手柄的电机控制权。
 * 输入参数：无，读取当前 owner 和电机忙状态。
 * 返回参数：无；满足释放条件时把 s_control_owner 置为 CONTROL_OWNER_NONE。
 */
static void ControlArbitration_ReleaseLocalOwnerIfMotorIdle(void)
{
	/* 外部通信必须由主动退出、超时释放或急停释放，不能因为本地电机停稳自动退出。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 没有本地 owner 时不用处理，保持空闲状态。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return;
	}

	/* 电机命令和反馈都已经停止时，本地控制源结束，本地三种方式可以重新竞争。 */
	if (ControlArbitration_IsMotorBusy() == false)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

/*
 * 函数功能：校验传入的控制源编号是否合法。
 * 输入参数：owner 控制源编号，取 CONTROL_OWNER_EXTERNAL/FOOT/SCREEN/HANDLE。
 * 返回参数：true 表示控制源合法；false 表示非法编号，不能参与仲裁。
 */
static bool ControlArbitration_IsValidOwner(uint8_t owner)
{
	/* 只接受四种正式控制来源，防止错误参数把仲裁锁写成未知状态。 */
	return ((owner == CONTROL_OWNER_EXTERNAL) ||
			(owner == CONTROL_OWNER_FOOT) ||
			(owner == CONTROL_OWNER_SCREEN) ||
			(owner == CONTROL_OWNER_HANDLE));
}

/*
 * 函数功能：把按键队列中的控制来源类型转换为电机仲裁 owner。
 * 输入参数：control_type 按键消息来源，取 JTKey/HANDLEKey/SCREENKey/HMIkey/PLUGunPLUG 等。
 * 返回参数：CONTROL_OWNER_*；插拔和未知来源返回 CONTROL_OWNER_NONE。
 */
static uint8_t ControlArbitration_GetOwnerByKeySource(uint8_t control_type)
{
	/* 脚踏队列消息统一归属脚踏控制来源。 */
	if (control_type == JTKey)
	{
		return CONTROL_OWNER_FOOT;
	}

	/* 手柄按键队列消息统一归属手柄按键控制来源。 */
	if (control_type == HANDLEKey)
	{
		return CONTROL_OWNER_HANDLE;
	}

	/* 屏幕按钮和触控启动统一归属屏幕控制来源。 */
	if (control_type == SCREENKey)
	{
		return CONTROL_OWNER_SCREEN;
	}

	/* 历史 HMIkey 仍按外部链路处理，避免它在脚踏/屏幕/手柄占用时绕过互斥。 */
	if (control_type == HMIkey)
	{
		return CONTROL_OWNER_EXTERNAL;
	}

	/* 插拔和未知消息不参与运行控制权竞争。 */
	return CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断泵控制按键是否需要占用电机仲裁 owner。
 * 输入参数：key_value 业务按键值，包含脚踏泵键、屏幕泵键和外控 HMI 泵键。
 * 返回参数：本地泵键返回 CONTROL_OWNER_NONE；外控泵键返回 CONTROL_OWNER_EXTERNAL。
 */
static uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value)
{
	/* 脚踏泵按键和脚踏轻排按键只影响泵输出，不应该因为泵运行占用手柄电机 owner。 */
	if ((key_value == JTkey_left_short) ||
		(key_value == JTKey_left_long) ||
		(key_value == JTKey_right_short) ||
		(key_value == JTKey_right_long) ||
		(key_value == JTKey_Gently_left_start) ||
		(key_value == JTKey_Gently_left_stop) ||
		(key_value == JTKey_Gently_right_start) ||
		(key_value == JTKey_Gently_rigth_stop))
	{
		/* 脚踏泵键只控制泵，不占用手柄电机 owner；脚踏真正启动电机时由 FootControlTask 申请 FOOT owner。 */
		return CONTROL_OWNER_NONE;
	}

	/* 屏幕泵按钮只影响泵输出，不应该因为屏幕点过泵就锁住脚踏或手柄电机控制。 */
	if ((key_value == SCREENKey_APUMP_Add) ||
		(key_value == SCREENKey_APUMP_Sub) ||
		(key_value == SCREENKey_APUMP_control) ||
		(key_value == SCREENKey_BPUMP_Add) ||
		(key_value == SCREENKey_BPUMP_Sub) ||
		(key_value == SCREENKey_BPUMP_control))
	{
		/* 屏幕泵键只控制泵，不占用手柄电机 owner；屏幕触控启动电机时才申请 SCREEN owner。 */
		return CONTROL_OWNER_NONE;
	}

	/* 历史 HMI 泵按钮由外部来源占用。 */
	if ((key_value == HMIkey_APUMP_Add) ||
		(key_value == HMIkey_APUMP_Sub) ||
		(key_value == HMIkey_APUMP_control) ||
		(key_value == HMIkey_BPUMP_Add) ||
		(key_value == HMIkey_BPUMP_Sub) ||
		(key_value == HMIkey_BPUMP_control) ||
		(key_value == HMIkey_Gently_start) ||
		(key_value == HMIkey_Gently_stop))
	{
		return CONTROL_OWNER_EXTERNAL;
	}

	/* 其它泵函数入口不改变控制权。 */
	return CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断指定控制源是否为当前仲裁 owner。
 * 输入参数：owner 待查询的控制源编号。
 * 返回参数：true 表示当前 owner 与输入来源一致；false 表示不一致。
 */
bool ControlArbitration_IsOwner(uint8_t owner)
{
	/* 当前持有者和查询来源一致时，允许该来源继续控制或主动停止。 */
	return (s_control_owner == owner);
}

/*
 * 函数功能：判断指定控制源是否被其它控制源阻塞。
 * 输入参数：owner 当前准备执行动作的控制源编号。
 * 返回参数：true 表示需要阻塞当前动作；false 表示当前动作可以继续处理。
 */
bool ControlArbitration_IsBusyByOther(uint8_t owner)
{
	/* 查询前先释放已经停稳的本地 owner，避免停止后残留锁挡住下一种本地控制方式。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();

	/* 没有 owner 时表示电机仲裁空闲，任意本地来源都可以尝试控制。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前来源就是 owner 时允许继续控制或发送停止命令。 */
	if (s_control_owner == owner)
	{
		return false;
	}

	/* 外部通信授权期间，本地脚踏、屏幕、手柄都必须等待外控主动退出或超时释放。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return true;
	}

	/* 本地三种方式只在手柄电机忙时互斥；泵运行不参与电机控制权阻塞。 */
	return ControlArbitration_IsMotorBusy();
}

/*
 * 函数功能：尝试让指定控制源取得电机控制权。
 * 输入参数：owner 申请控制权的来源编号。
 * 返回参数：true 表示申请成功或已经持有；false 表示无效来源或被其它来源阻塞。
 */
bool ControlArbitration_TryEnter(uint8_t owner)
{
	/* 无效来源不能写入 owner，避免未知按键把仲裁状态写乱。 */
	if (ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 申请前先清掉已经停稳的本地 owner，保证电机停止后其它本地模式能接管。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();

	/* 已经持有控制权时重复申请按成功处理。 */
	if (s_control_owner == owner)
	{
		return true;
	}

	/* 外部通信授权期间，只有外部通信自己能继续控制。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return false;
	}

	/* 电机命令仍在运行或驱动反馈仍在转动时，不允许不同来源抢占。 */
	if (ControlArbitration_IsMotorBusy())
	{
		return false;
	}

	/* 电机已经空闲时记录新的控制源；本地泵动作不会调用本路径占用 owner。 */
	s_control_owner = owner;
	return true;
}

/*
 * 函数功能：释放指定控制源持有的仲裁 owner，只允许当前持有者释放自己。
 * 输入参数：owner 请求释放控制权的来源编号。
 * 返回参数：无。
 */
void ControlArbitration_Exit(uint8_t owner)
{
	/* 只有当前持有者本人才能释放，避免其它来源误清正在运行的控制权。 */
	if (s_control_owner == owner)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

/*
 * 函数功能：当本地控制源对应的电机已经停止时释放本地电机控制权。
 * 输入参数：owner 请求释放的本地控制源编号。
 * 返回参数：无。
 */
void ControlArbitration_ExitLocalControlIfIdle(uint8_t owner)
{
	/* 外部通信必须由外控退出、链路超时或急停释放，不能由本地空闲逻辑释放。 */
	if (owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 电机命令仍在运行或驱动反馈仍未归零时，本地来源还没有真正结束。 */
	if (ControlArbitration_IsMotorBusy())
	{
		return;
	}

	/* 电机停稳后释放当前本地来源；泵状态不再影响手柄电机仲裁。 */
	ControlArbitration_Exit(owner);
}

/*
 * 函数功能：周期刷新电机控制权释放状态。
 * 输入参数：无，由电机输出任务或反馈任务周期调用。
 * 返回参数：无；本地 owner 停稳后自动释放，外部 owner 不自动释放。
 */
void ControlArbitration_RefreshMotorOwner(void)
{
	/* 统一复用本地 owner 释放逻辑，保证手柄、脚踏、屏幕停止后不因反馈延迟永久占用。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();
}

/*
 * 函数功能：系统级强制释放当前仲裁 owner，用于急停或全局清状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ForceRelease(void)
{
	/* 急停或系统级清状态使用强制释放，让任何来源都不能继续占用控制权。 */
	s_control_owner = CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断外部通信控制权是否处于有效占用状态。
 * 输入参数：无，读取 s_control_owner 和 WorkMessage.hmiactive_work。
 * 返回参数：true 表示外控已持有仲裁且界面状态仍有效；false 表示外控未激活。
 */
bool ControlArbitration_IsExternalActive(void)
{
	/* 外控是否有效以仲裁持有者为准，hmiactive_work 只作为界面/状态同步标志。 */
	return ((s_control_owner == CONTROL_OWNER_EXTERNAL) && (WorkMessage.hmiactive_work != 0U));
}

/*
 * 函数功能：申请进入外部通信控制模式，并初始化外控占用状态。
 * 输入参数：无。
 * 返回参数：true 表示外控取得控制权；false 表示本机电机仍忙或其它来源占用。
 */
bool ControlArbitration_EnterExternalControl(void)
{
	bool already_external = ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL);

	/* 若脚踏、屏幕或手柄正在控制，上位机申请直接失败，不能抢停当前来源。 */
	if (ControlArbitration_TryEnter(CONTROL_OWNER_EXTERNAL) == false)
	{
		return false;
	}

	/* 首次进入外控时清理空闲残留输出；重复申请外控时不打断已在进行的外控动作。 */
	if (already_external == false)
	{
		ControlArbitration_StopMotionOutput();
	}

	/* 置位外部控制权锁，脚踏、屏幕、手柄按键会在各自入口被拦截。 */
	WorkMessage.hmiactive_work = 1U;
	/* 触控/外控占用标志同步置位，屏幕模式切换逻辑也能看到外控占用。 */
	WorkMessage.touchactive_work = TOUCHWORK;
	/* 当前驱动方式切到外部控制，心跳和驱动状态可以看到外控来源。 */
	WorkMessage.drivetype_work = TOUCHWORK;
	/* 外部控制已使能，但申请阶段不直接启动电机。 */
	ControlSignalMessage.HMI_enable_flag = true;
	ControlSignalMessage.HMI_control_flag = false;
	return true;
}

/*
 * 函数功能：退出外部通信控制模式，停止外控遗留输出并恢复本机可接管状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ReleaseExternalControl(void)
{
	/* 当前不是外控时，退出外控只清理外控显示残留，不能影响脚踏、屏幕或手柄正在进行的控制。 */
	if (s_control_owner != CONTROL_OWNER_EXTERNAL)
	{
		WorkMessage.hmiactive_work = 0U;
		ControlSignalMessage.HMI_enable_flag = false;
		return;
	}

	/* 释放外控时必须先停全部外控输出，防止泵或电机继续沿用上位机请求。 */
	ControlArbitration_StopMotionOutput();
	/* 清除外部控制权锁，允许后续脚踏、屏幕和手柄重新参与控制。 */
	WorkMessage.hmiactive_work = 0U;
	/* 清除触控/外控占用标志，使本机模式切换可以重新生效。 */
	WorkMessage.touchactive_work = 0U;
	/* 外控使能同步撤销，界面状态不再显示外部控制已占用。 */
	ControlSignalMessage.HMI_enable_flag = false;
	/* 恢复到本机可接管的控制方式，避免退出后仍停在 TOUCHWORK。 */
	WorkMessage.drivetype_work = ControlArbitration_GetLocalDriveTypeAfterExit();
	/* 外控完整退出后才释放公共控制权。 */
	ControlArbitration_Exit(CONTROL_OWNER_EXTERNAL);
}

/*
 * 函数功能：按键队列分发前判断当前消息是否应被仲裁阻塞。
 * 输入参数：control_type 按键来源类型；control_key 具体业务按键值。
 * 返回参数：true 表示本消息应丢弃等待当前 owner 结束；false 表示可继续分发。
 */
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type, uint8_t control_key)
{
	uint8_t key_owner = ControlArbitration_GetOwnerByKeySource(control_type);

	/* 手柄插拔事件只维护在线状态和通道记忆，不属于运行控制，互斥期间仍要接收。 */
	if (control_type == PLUGunPLUG)
	{
		return false;
	}

	/* 外控持有期间允许旧 HMI 和屏幕上的外控退出键通过，便于主动释放上位机控制。 */
	if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) &&
		(((control_type == HMIkey) && (control_key == HMIkey_HMI_EXIT)) ||
		 ((control_type == SCREENKey) && (control_key == SCREENKey_HMI_EXIT))))
	{
		return false;
	}

	/* 不属于四类控制来源的消息不参与仲裁。 */
	if (key_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前已有其它来源占用时，丢弃该按键，必须等当前控制方式结束。 */
	return ControlArbitration_IsBusyByOther(key_owner);
}

void ChannelMessageInit(void)
{
	ControlMessageInit();
}

void ChannelFlagMessageInit(void)
{
	ControlMessageInit();
}

/**
 * @brief HMI退出活动处理函数
 * @param key_value 按键值，用于判断不同的退出触发条件
 */
void HmiExitActive(uint8_t key_value)
{
	ControlArbitration_ReleaseExternalControl();
	// 通知HMI标志位界面消失
	//  判断是否为外部命令触发的HMI退出
	if (key_value == HMIkey_HMI_EXIT)
	{
		// 外部命令，说我要退出外部，全部交于动力自主
		// 如果外部控制中，则停止相关内容，比如泵和电机
	}
	// 判断是否为导航界面按钮触发的强制退出
	else if (key_value == SCREENKey_HMI_EXIT)
	{
		// 导航界面按钮强制退出，通知一下上位机，若要控制请重头校验识别
	}
}

/*
 * 函数功能：处理屏幕/HMI 的控制方式切换与屏幕触控启动、停止动作。
 * 输入参数：key_value 屏幕或 HMI 下发的控制方式按键值。
 * 返回参数：无。
 */
void ControlTypeActive(uint8_t key_value)
{
	if (WorkMessage.alarm_flag == true)
		return;
	switch (key_value)
	{
	case SCREENKey_JTActi:
	case HMIkey_JTActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		// 脚踏控制
		if (WorkMessage.drivetype_work == JTWORK || WorkMessage.touchactive_work == TOUCHWORK)
			return;
		WorkMessage.drivetype_work = JTWORK;
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = JTWORK;
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = JTWORK;
		// 注意界面更新
	}
	break;
	case SCREENKey_HandleActi:
	case HMIkey_HandleActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		// 手动控制
		if (WorkMessage.drivetype_work == HANDLEWORK || WorkMessage.touchactive_work == TOUCHWORK)
			return;
		WorkMessage.drivetype_work = HANDLEWORK;
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = HANDLEWORK;
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = HANDLEWORK;
		// 界面调整更新
	}
	break;
	case SCREENKey_TouchActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		// 触摸模式
		if (WorkMessage.touchactive_work == TOUCHWORK)
			return;
		// 弹出窗模式
		/* 只激活触控模式，不直接落入 TouchStart，避免点“触控激活”时误启动电机。 */
		break;
	}
	case SCREENKey_TouchStart: // 启动
		if (WorkMessage.runflag_work == true)
			return;
		/* 屏幕触控启动前先申请屏幕控制权，若脚踏/手柄/外控正在控制则直接等待。 */
		if (ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN) == false)
			return;
		/* 触控启动后置位触控占用，直到屏幕触控退出才释放其它控制方式。 */
		WorkMessage.touchactive_work = TOUCHWORK;
		WorkMessage.runflag_work = true; // drive执行
		break;
	case SCREENKey_TouchEXIT: // 停止
		/* 屏幕触控退出时先停电机，再清触控占用，控制权释放要继续等待驱动反馈归零。 */
		WorkMessage.runflag_work = false;
		WorkMessage.speed_work = 0U;
		WorkMessage.touchactive_work = 0U;
		/* 这里不能直接 Exit，否则刚下发停止但电机尚未停稳时其它来源会提前接管。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN);
		// 退出触控界面
		break;
	}
}

void SpeedActive(uint8_t key_value)
{
	uint16_t speed_value;
	uint16_t speed_step;
	uint16_t speed_max;
	uint16_t speed_min;

	if (WorkMessage.alarm_flag == true)
		return;
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_zzstep;
			speed_max = ChannelrecognizeMessageA.speed_zzmax;
			speed_min = ChannelrecognizeMessageA.speed_zzmin;
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_fzstep;
			speed_max = ChannelrecognizeMessageA.speed_fzmax;
			speed_min = ChannelrecognizeMessageA.speed_fzmin;
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_oscstep;
			speed_max = ChannelrecognizeMessageA.speed_oscmax;
			speed_min = ChannelrecognizeMessageA.speed_oscmin;
		}
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_zzstep;
			speed_max = ChannelrecognizeMessageA.speed_zzmax;
			speed_min = ChannelrecognizeMessageA.speed_zzmin;
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_fzstep;
			speed_max = ChannelrecognizeMessageA.speed_fzmax;
			speed_min = ChannelrecognizeMessageA.speed_fzmin;
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_oscstep;
			speed_max = ChannelrecognizeMessageA.speed_oscmax;
			speed_min = ChannelrecognizeMessageA.speed_oscmin;
		}
	}
	switch (key_value)
	{
	case HANDLEKey_speed_add: // 忘记快加快减
	case HMIkey_SPEED_Add:
	case SCREENKey_SPEED_Add:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			speed_value = WorkMessage.speed_set_work + speed_step;
			if (speed_value > speed_max)
				speed_value = speed_max;
			WorkMessage.speed_set_work = speed_value;
			if (WorkMessage.dir_work == ZZDIR)
				MemoryMsgA.zz_speed = speed_value;
			else if (WorkMessage.dir_work == FZDIR)
				MemoryMsgA.fz_speed = speed_value;
			else if (WorkMessage.dir_work == OSCDIR)
				MemoryMsgA.osc_speed = speed_value;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			speed_value = WorkMessage.speed_set_work + speed_step;
			if (speed_value > speed_max)
				speed_value = speed_max;
			WorkMessage.speed_set_work = speed_value;
			if (WorkMessage.dir_work == ZZDIR)
				MemoryMsgB.zz_speed = speed_value;
			else if (WorkMessage.dir_work == FZDIR)
				MemoryMsgB.fz_speed = speed_value;
			else if (WorkMessage.dir_work == OSCDIR)
				MemoryMsgB.osc_speed = speed_value;
		}
		// 速度加
		break;
	case HANDLEKey_speed_sub:
	case HMIkey_SPEED_Sub:
	case SCREENKey_SPEED_Sub:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			speed_value = WorkMessage.speed_set_work - speed_step;
			if (speed_value < speed_min)
				speed_value = speed_min;
			WorkMessage.speed_set_work = speed_value;
			if (WorkMessage.dir_work == ZZDIR)
				MemoryMsgA.zz_speed = speed_value;
			else if (WorkMessage.dir_work == FZDIR)
				MemoryMsgA.fz_speed = speed_value;
			else if (WorkMessage.dir_work == OSCDIR)
				MemoryMsgA.osc_speed = speed_value;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			speed_value = WorkMessage.speed_set_work - speed_step;
			if (speed_value < speed_min)
				speed_value = speed_min;
			WorkMessage.speed_set_work = speed_value;
			if (WorkMessage.dir_work == ZZDIR)
				MemoryMsgB.zz_speed = speed_value;
			else if (WorkMessage.dir_work == FZDIR)
				MemoryMsgB.fz_speed = speed_value;
			else if (WorkMessage.dir_work == OSCDIR)
				MemoryMsgB.osc_speed = speed_value;
		}
		// 速度减
		break;
	case HANDLEKey_greaI: // 快速档位速度切换(现保留，考虑中)
		break;
	case HANDLEKey_greaII:
		break;
	case HANDLEKey_greaIII:
		// 高倍模式
		break;
	case HANDLEKey_greaIV:
		break;
	case HANDLEKey_greaV:
		break;
	}
}
void FreqActive(uint8_t key_value)
{
	uint8_t freq_value;
	uint8_t freq_step = 10;
	if (WorkMessage.alarm_flag == true || WorkMessage.tool_type != PLANER)
		return;
	switch (key_value)
	{
	case HMIkey_FREQ_Add:
	case SCREENKey_FREQ_Add:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			freq_value = WorkMessage.freq_work + freq_step;
			if (freq_value > ChannelrecognizeMessageA.freq_max)
				freq_value = ChannelrecognizeMessageA.freq_max;
			WorkMessage.freq_work = MemoryMsgA.freq = freq_value;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			freq_value = WorkMessage.freq_work + freq_step;
			if (freq_value > ChannelrecognizeMessageB.freq_max)
				freq_value = ChannelrecognizeMessageB.freq_max;
			WorkMessage.freq_work = MemoryMsgB.freq = freq_value;
		}
		// 频率加
		break;

	case HMIkey_FREQ_Sub:
	case SCREENKey_FREQ_Sub:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			freq_value = WorkMessage.freq_work - freq_step;
			if (freq_value < ChannelrecognizeMessageA.freq_min)
				freq_value = ChannelrecognizeMessageA.freq_min;
			WorkMessage.freq_work = MemoryMsgA.freq = freq_value;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			freq_value = WorkMessage.freq_work - freq_step;
			if (freq_value > ChannelrecognizeMessageB.freq_min)
				freq_value = ChannelrecognizeMessageB.freq_min;
			WorkMessage.freq_work = MemoryMsgB.freq = freq_value;
		}
		// 频率减
		break;
	}
}
void DirActive(uint8_t key_value)
{
	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;
	switch (key_value)
	{
	case HANDLEKey_dir_Forward:
	case HMIkey_Dir_Forward:
	case SCREENKey_Dir_Forward:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = ZZDIR;
			WorkMessage.speed_set_work = MemoryMsgA.zz_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = ZZDIR;
			WorkMessage.speed_set_work = MemoryMsgB.zz_speed;
		}
		// 正向
		break;
	case HANDLEKey_dir_Reverse:
	case HMIkey_Dir_Reverse:
	case SCREENKey_Dir_Reverse:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = FZDIR;
			WorkMessage.speed_set_work = MemoryMsgA.fz_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = FZDIR;
			WorkMessage.speed_set_work = MemoryMsgB.fz_speed;
		}
		// 反向
		break;
	case HANDLEKey_dir_OSC:
	case HMIkey_Dir_OSC:
	case SCREENKey_Dir_OSC:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = OSCDIR;
			WorkMessage.speed_set_work = MemoryMsgA.osc_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = OSCDIR;
			WorkMessage.speed_set_work = MemoryMsgB.osc_speed;
		}
		// 往复
		break;
	}
}
/**
 * @brief 刨头按钮和磨头按钮
 *
 * @param key_value 输入的按键值，用于判断执行哪个操作
 */
void PlanerGridH(uint8_t key_value)
{
	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;
	switch (key_value)
	{
	case SCREENKey_PlanerH: // 屏幕平面磨床水平控制按键
	case HMIkey_PlanerH:	// HMI平面磨床水平控制按键
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.tool_type = MemoryMsgA.tool_type = PLANER;
			WorkMessage.freq_work = MemoryMsgA.freq;
			WorkMessage.dir_work = MemoryMsgA.dir;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.zz_speed;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.fz_speed;
			}
			else if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.osc_speed;
			}
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.tool_type = MemoryMsgB.tool_type = PLANER;
			WorkMessage.freq_work = MemoryMsgB.freq;
			WorkMessage.dir_work = MemoryMsgB.dir;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.zz_speed;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.fz_speed;
			}
			else if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.osc_speed;
			}
		}
		// 刨头
		break;
	case HMIkey_GrindH:	   // HMI磨头水平控制按键
	case SCREENKey_GrindH: // 屏幕磨头水平控制按键
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.tool_type = MemoryMsgA.tool_type = PLANER;
			WorkMessage.freq_work = MemoryMsgA.freq;
			WorkMessage.dir_work = MemoryMsgA.dir;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.zz_speed;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.fz_speed;
			}
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.tool_type = MemoryMsgB.tool_type = PLANER;
			WorkMessage.freq_work = MemoryMsgB.freq;
			WorkMessage.dir_work = MemoryMsgB.dir;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.zz_speed;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.fz_speed;
			}
		}
		// 磨头
		break;
	}
}

/*
 * 函数功能：处理屏幕、脚踏或上位机发起的 A/B 手柄手动切换。
 * 输入参数：key_value 切换按键值，支持 SCREENKey/HMIkey A/B 和脚踏中键长按。
 * 返回参数：无。
 */
void HandleSwitchActive(uint8_t key_value) // 2026,4,19
{
	uint8_t target_channel = CHANNEL_NONE; /* 保存本次用户确认想切到的目标通道，默认不切换。 */

	if ((WorkMessage.runflag_work == true) || (WorkMessage.alarm_flag == true))
	{
		return; /* 运行中或系统报警中不允许切换，避免两个手柄同时参与输出。 */
	}

	if ((key_value == HMIkey_HANDLE_A) || (key_value == SCREENKey_HANDLE_A))
	{
		target_channel = CHANNEL_A; /* 屏幕或上位机明确点 A 时，目标通道就是 A。 */
	}
	else if ((key_value == HMIkey_HANDLE_B) || (key_value == SCREENKey_HANDLE_B))
	{
		target_channel = CHANNEL_B; /* 屏幕或上位机明确点 B 时，目标通道就是 B。 */
	}
	else if (key_value == JTKey_middle_long)
	{
		if ((WorkMessage.channel_work == CHANNEL_A) && (WorkMessage.Channel_Bonline == true))
		{
			target_channel = CHANNEL_B; /* 当前选中 A 且 B 在线时，脚踏长按只切到 B。 */
		}
		else if ((WorkMessage.channel_work == CHANNEL_B) && (WorkMessage.Channel_Aonline == true))
		{
			target_channel = CHANNEL_A; /* 当前选中 B 且 A 在线时，脚踏长按只切到 A。 */
		}
	}

	if ((target_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == true))
	{
		Pubinterface_LoadChannelMemory(CHANNEL_A); /* 用户确认切到 A 后，把 A 通道记忆装载为当前工作快照。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 A 被点亮。 */
	}
	else if ((target_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == true))
	{
		Pubinterface_LoadChannelMemory(CHANNEL_B); /* 用户确认切到 B 后，把 B 通道记忆装载为当前工作快照。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 B 被点亮。 */
	}
}

/*
 * 函数功能：处理 A/B 手柄插入和拔出事件；插入先写 MemoryMsg，只有策略允许时才装载到 WorkMessage。
 * 输入参数：key_value 插拔事件按键值，SCREENKey_PLUG_A/B 表示上线，SCREENKey_UNPLUG_A/B 表示离线。
 * 返回参数：无。
 */
void PlugORunPLUGActive(uint8_t key_value)
{
	uint8_t current_channel_unplugged = 0U; /* 记录拔出的是否为当前选中通道，用于防止自动切到另一通道。 */

	switch (key_value)
	{
	case SCREENKey_PLUG_A: // 插入A
		WorkMessage.Channel_Aonline = true;					   /* A 通道校验通过后才置在线，后续心跳和显示都读取这个标志。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_A);		   /* 扫描结果只先进入 MemoryMsgA，运行中不会直接覆盖 WorkMessage。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_A))
		{
			Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非运行状态下最后插入且校验通过的 A 通道成为当前选中通道。 */
			Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* A 被选中时，才用 A 的 Page4 默认流量初始化注水泵。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 A 在线图标和当前高亮状态。 */
		break;

	case SCREENKey_PLUG_B: // 插入B
		WorkMessage.Channel_Bonline = true;					   /* B 通道校验通过后才置在线，坏手柄不会进入在线态。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_B);		   /* 扫描结果只先进入 MemoryMsgB，避免运行中插入 B 抢占 A。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_B))
		{
			Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非运行状态下最后插入且校验通过的 B 通道成为当前选中通道。 */
			Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* B 被选中时，才用 B 的 Page4 默认流量初始化注水泵。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 B 在线图标和当前高亮状态。 */
		break;

	case SCREENKey_UNPLUG_A: // 拔出A
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_A); /* 先记录拔出前 A 是否为当前选中通道。 */
		WorkMessage.Channel_Aonline = false;					   /* A 拔出后立刻离线，心跳会报告 A 不可用。 */
		memset(&MemoryMsgA, 0, sizeof(MemoryMsgA));			   /* A 离线时清空 A 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U)
		{
			if ((WorkMessage.runflag_work == false) &&
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Bonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非工作状态拔掉当前 A 时，若 B 仍在线，则回落选中 B，保持“非工作态有在线手柄即有选中通道”。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* B 成为当前选中通道后，用 B 的 Page4 默认流量补齐仍为 0 的注水泵。 */
			}
			else
			{
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 B 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 B 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* 工作中当前通道拔出仍不自动切到 B，等待用户手动确认。 */
			}
		}
		Pubinterface_SendHandleDisplay(CHANNEL_A, 0U, false, false); /* A 通道拔出后立即暗灭 A 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 B 仍在线，只显示 B 在线但不因 A 拔出自动高亮 B。 */
		break;

	case SCREENKey_UNPLUG_B: // 拔出B
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_B); /* 先记录拔出前 B 是否为当前选中通道。 */
		WorkMessage.Channel_Bonline = false;					   /* B 拔出后立刻离线，心跳会报告 B 不可用。 */
		memset(&MemoryMsgB, 0, sizeof(MemoryMsgB));			   /* B 离线时清空 B 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U)
		{
			if ((WorkMessage.runflag_work == false) &&
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Aonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非工作状态拔掉当前 B 时，若 A 仍在线，则回落选中 A，匹配现场先插 A 再插 B 再拔 B 的预期。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* A 成为当前选中通道后，用 A 的 Page4 默认流量补齐仍为 0 的注水泵。 */
			}
			else
			{
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 A 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 A 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* 工作中当前通道拔出仍不自动切到 A，等待用户手动确认。 */
			}
		}
		Pubinterface_SendHandleDisplay(CHANNEL_B, 0U, false, false); /* B 通道拔出后立即暗灭 B 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 A 仍在线，只显示 A 在线但不因 B 拔出自动高亮 A。 */
		break;

	default:
		break;												   /* 其它按键不是插拔事件，本函数不处理。 */
	}
}

void ToolPosActive(uint8_t key_value)
{

	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;

	if (WorkMessage.hand_model == PXBA_ONLINES || WorkMessage.hand_model == PXBB_ONLINES)
	{
		if (WorkMessage.tool_reduction_ratio & 0xffff == 500) // 5碚减速比\100
		{

			switch (key_value)
			{
			case HMIkey_OpenPos_ClockWise:
			case SCREENKey_OpenPos_ClockWise:
				ToolPosMay(WorkMessage.channel_work, 1, 1); // 一度
				// 逆时针
				break;
			case JTKey_middle_short:
			case HMIkey_OpenPos_AntiClockWise:
			case SCREENKey_OpenPos_AntiClockWise:
				ToolPosMay(WorkMessage.channel_work, 2, 1); // 一度
				// 顺时针
				break;
			}
		}
	}
}

/*
 * 函数功能：处理脚踏、屏幕和 HMI 的 A/B 泵档位、启停、轻排和排空控制。
 * 输入参数：key_value 触发泵控制的业务按键值。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value)
{
	static uint8_t PumpA_Gear = 0;
	static uint8_t PumpB_Gear = 0;
	static bool PumpA_Start_flag = 0;
	static bool PumpB_Start_flag = 0;
	uint8_t pump_owner = ControlArbitration_GetOwnerByPumpKey(key_value);

	switch (key_value)
	{
	case JTkey_left_short:
		PumpA_Gear++;
		if (PumpA_Gear > 6)
			PumpA_Gear = 0;
		if (pumpMessageA.type == 0)
		{
			PumpA_Gear = 0; // 队列通知界面暗黑
			return;
		}
		else if (pumpMessageA.type == DRAWWATER) // 抽水
		{
			pumpMessageA.speed_work = 3 * PumpA_Gear; // 每一档位增加30ml水
		}
		else if (pumpMessageA.type == INJECTWATER) // 注水
		{
			if (pumpMessageA.timingDrainage_flag == true)
				return;
			switch (PumpA_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
				pumpMessageA.speed_work = PumpA_Gear * 10;
				break;
			case PUMPGEAR_IV:
				pumpMessageA.speed_work = 50;
			case PUMPGEAR_V:
				pumpMessageA.speed_work = 70;
				break;
			}
		}
		else if (pumpMessageA.type == POURWATER) // 灌注
		{

			switch (PumpA_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
			case PUMPGEAR_IV:
				pumpMessageA.speed_work = 50 * PumpA_Gear; // 每一档位增加50ml水
				break;
			case PUMPGEAR_V:
				pumpMessageA.speed_work = 300;
				break;
			}
		}
		// 队列通知ui更新界面
		break;
	case JTKey_left_long:
	case HMIkey_APUMP_control:
	case SCREENKey_APUMP_control:
		if (pumpMessageA.type == 0)
		{
			pumpMessageA.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			// 队列A停止
			return;
		}
		else
		{
			if (!pumpMessageA.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				pumpMessageA.run_flag = true;
				if (pumpMessageA.type == INJECTWATER)
				{
					pumpMessageA.timingDrainage_flag = true;
				}
				// 队列发送界面按钮和数字变黄，A
				// 队列发送泵运行设置数据
			}
			else
			{
				pumpMessageA.run_flag = false;
				pumpMessageA.timingDrainage_flag = false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑,A
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		break;
	case JTKey_right_short:
		PumpB_Gear++;
		if (PumpB_Gear > 5)
			PumpB_Gear = 0;
		if (pumpMessageB.type == 0)
		{
			PumpB_Gear = 0;
			// 队列通知界面暗黑
		}
		else if (pumpMessageB.type == DRAWWATER) // 抽水
		{
			pumpMessageB.speed_work = 3 * PumpB_Gear; // 每一档位增加30ml水
		}
		else if (pumpMessageB.type == INJECTWATER) // 注水
		{
			switch (PumpB_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
				pumpMessageB.speed_work = PumpB_Gear * 10;
				break;
			case PUMPGEAR_IV:
				pumpMessageB.speed_work = 50;
			case PUMPGEAR_V:
				pumpMessageB.speed_work = 70;
				break;
			}
		}
		else if (pumpMessageB.type == POURWATER) // 灌注
		{
			switch (PumpB_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
			case PUMPGEAR_IV:
				pumpMessageB.speed_work = 50 * PumpB_Gear; // 每一档位增加50ml水
				break;
			case PUMPGEAR_V:
				pumpMessageB.speed_work = 300; // 最大300ml水，记得宏定义
				break;
			}
		}
		break;
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		if (pumpMessageB.type == 0)
		{
			pumpMessageB.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			// 对列通知为0;
			return;
		}
		else
		{
			if (!pumpMessageB.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				pumpMessageB.run_flag = true;
				if (pumpMessageB.type == INJECTWATER)
				{
					pumpMessageB.timingDrainage_flag = true;
				}
				// 队列发送界面按钮和数字变黄
				// 队列发送泵运行设置数据
			}
			else
			{
				pumpMessageB.timingDrainage_flag = false;
				pumpMessageB.run_flag = false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		break;
	case HMIkey_APUMP_Add:
	case SCREENKey_APUMP_Add:
	case HMIkey_APUMP_Sub:
	case SCREENKey_APUMP_Sub:
		switch (pumpMessageA.type)
		{
		case DRAWWATER: // 抽
			pumpMessageA.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			if (pumpMessageA.timingDrainage_flag == true)
				return;
			pumpMessageA.speed_step_value = 5;
			break;
		case POURWATER: // 灌
			pumpMessageA.speed_step_value = 30;
			break;
		}
		if (key_value == HMIkey_APUMP_Add || key_value == SCREENKey_APUMP_Add)
		{
			pumpMessageA.speed_work += pumpMessageA.speed_step_value;
			if (pumpMessageA.speed_work > pumpMessageA.speed_Max)
			{
				pumpMessageA.speed_work = pumpMessageA.speed_Max;
			}
		}
		else
		{
			pumpMessageA.speed_work -= pumpMessageA.speed_step_value;
			if (pumpMessageA.speed_work < pumpMessageA.speed_Min)
			{
				pumpMessageA.speed_work = pumpMessageA.speed_Min;
			}
		}
		if (pumpMessageA.run_flag == true)
		{
			// 队列通知A泵运行设置数据
		}
		break;

	case HMIkey_BPUMP_Add:
	case SCREENKey_BPUMP_Add:
	case HMIkey_BPUMP_Sub:
	case SCREENKey_BPUMP_Sub:
		switch (pumpMessageB.type)
		{
		case DRAWWATER: // 抽
			pumpMessageB.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			if (pumpMessageB.timingDrainage_flag == true)

				return;
			pumpMessageB.speed_step_value = 5;
			break;
		case POURWATER: // 灌
			pumpMessageB.speed_step_value = 30;
			break;
		}
		if (key_value == HMIkey_APUMP_Add || key_value == SCREENKey_APUMP_Add)
		{
			pumpMessageB.speed_work += pumpMessageB.speed_step_value;
			if (pumpMessageB.speed_work > pumpMessageB.speed_Max)
			{
				pumpMessageB.speed_work = pumpMessageB.speed_Max;
			}
		}
		else
		{
			pumpMessageB.speed_work -= pumpMessageB.speed_step_value;
			if (pumpMessageB.speed_work < pumpMessageB.speed_Min)
			{
				pumpMessageB.speed_work = pumpMessageB.speed_Min;
			}
		}

		if (pumpMessageB.run_flag == true)
		{
			// 队列通知B泵运行设置数据
		}
		break;
	case JTKey_Gently_left_start: // 其实可以判断手柄类型决定是否给与注水
		/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		if (pumpMessageA.type == INJECTWATER) // 注水
		{
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				pumpMessageA.run_flag = true;
			}
		}
		else if (pumpMessageB.type == INJECTWATER)
		{

			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				pumpMessageB.run_flag = true;
				// 队列发送启动B
			}
		}
		break;
	case JTKey_Gently_left_stop:
		if (pumpMessageA.type == INJECTWATER) // 注水
		{
			// 队列通知A泵停
			pumpMessageA.run_flag = false;
			;
		}
		else if (pumpMessageB.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageB.run_flag = false;
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		break;
	case JTKey_Gently_right_start:
		/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		if (pumpMessageB.type == INJECTWATER) // 注水
		{
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				pumpMessageB.run_flag = true;
			}
		}
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列发送
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				pumpMessageA.run_flag = true;
			}
		}

		break;
	case JTKey_Gently_rigth_stop:
		if (pumpMessageB.type == INJECTWATER) // 注水
		{
			// 队列通知A泵停
			pumpMessageB.run_flag = false;
			;
		}
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageA.run_flag = false;
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		break;
	case HMIkey_Gently_start:
		/* 历史 HMI 轻排属于外部来源，启动前也必须通过统一控制权仲裁。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;

		if (WorkMessage.channel_work == 1)
		{
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				pumpMessageA.run_flag = true;
			}
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				pumpMessageB.run_flag = true;
			}
		}
		else if (WorkMessage.channel_work == 2)
		{
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				pumpMessageB.run_flag = true;
			}
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				pumpMessageA.run_flag = true;
			}
		}

		break;
	case HMIkey_Gently_stop:
		if (WorkMessage.channel_work == 1)
		{
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送停止A
				pumpMessageA.run_flag = false;
			}
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
			}
		}
		else if (WorkMessage.channel_work == 2)
		{
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
			}
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列停止B
				pumpMessageA.run_flag = false;
			}
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);

		break;
	default:
		break;
	}
}
