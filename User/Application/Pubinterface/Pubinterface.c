//调用对应头文件
#include "Pubinterface.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include <screen.h>
//#include "datahand.h"
#include "sscDRIVE.h"
#include "sscUIDP.h"





ChannelrecognizeMessage_t ChannelrecognizeMessageA;
ChannelrecognizeMessage_t ChannelrecognizeMessageB;
ControlSigleMessage_t ControlSigleMssage;
ControlSignalMessage_t ControlSignalMessage;


WorkMessage_t WorkMessage;//工作信息
ChannelMemoryMessagr_t MemoryMsgA;//通道记忆（增对可调节参数），用于切换手柄
ChannelMemoryMessagr_t MemoryMsgB;//
pumpMessage_t pumpMessageA;
pumpMessage_t pumpMessageB;
/* 当前控制权持有者，四种控制方式必须等待当前持有者结束后才能重新申请。 */
static volatile uint8_t s_control_owner = CONTROL_OWNER_NONE;

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
	if(WorkMessage.Channel_Aonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_A,
									  MemoryMsgA.hand_model,
									  true,
									  (WorkMessage.channel_work == CHANNEL_A));
	}

	/* B 通道在线时刷新 B 手柄图标，并按当前工作通道决定是否高亮。 */
	if(WorkMessage.Channel_Bonline)
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
	if(WorkMessage.channel_work == CHANNEL_A)
	{
		drive_type = MemoryMsgA.drive_type;
	}
	else if(WorkMessage.channel_work == CHANNEL_B)
	{
		drive_type = MemoryMsgB.drive_type;
	}

	/* 只允许恢复脚踏或手柄，本次退出不能继续停留在 TOUCHWORK 外控模式。 */
	if((drive_type == JTWORK) || (drive_type == HANDLEWORK))
	{
		return drive_type;
	}

	/* 当前是带按键手柄时，给本机手柄控制一个自然的回退入口。 */
	if((WorkMessage.hand_model == PXBA_ONLINES) || (WorkMessage.hand_model == PXBB_ONLINES))
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
	if(WorkMessage.runflag_work == true)
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
	if(s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 没有本地 owner 时不用处理，保持空闲状态。 */
	if(s_control_owner == CONTROL_OWNER_NONE)
	{
		return;
	}

	/* 电机命令和反馈都已经停止时，本地控制源结束，本地三种方式可以重新竞争。 */
	if(ControlArbitration_IsMotorBusy() == false)
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
	if(control_type == JTKey)
	{
		return CONTROL_OWNER_FOOT;
	}

	/* 手柄按键队列消息统一归属手柄按键控制来源。 */
	if(control_type == HANDLEKey)
	{
		return CONTROL_OWNER_HANDLE;
	}

	/* 屏幕按钮和触控启动统一归属屏幕控制来源。 */
	if(control_type == SCREENKey)
	{
		return CONTROL_OWNER_SCREEN;
	}

	/* 历史 HMIkey 仍按外部链路处理，避免它在脚踏/屏幕/手柄占用时绕过互斥。 */
	if(control_type == HMIkey)
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
	if((key_value == JTkey_left_short) ||
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
	if((key_value == SCREENKey_APUMP_Add) ||
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
	if((key_value == HMIkey_APUMP_Add) ||
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
	if(s_control_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前来源就是 owner 时允许继续控制或发送停止命令。 */
	if(s_control_owner == owner)
	{
		return false;
	}

	/* 外部通信授权期间，本地脚踏、屏幕、手柄都必须等待外控主动退出或超时释放。 */
	if(s_control_owner == CONTROL_OWNER_EXTERNAL)
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
	if(ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 申请前先清掉已经停稳的本地 owner，保证电机停止后其它本地模式能接管。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();

	/* 已经持有控制权时重复申请按成功处理。 */
	if(s_control_owner == owner)
	{
		return true;
	}

	/* 外部通信授权期间，只有外部通信自己能继续控制。 */
	if(s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return false;
	}

	/* 电机命令仍在运行或驱动反馈仍在转动时，不允许不同来源抢占。 */
	if(ControlArbitration_IsMotorBusy())
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
	if(s_control_owner == owner)
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
	if(owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 电机命令仍在运行或驱动反馈仍未归零时，本地来源还没有真正结束。 */
	if(ControlArbitration_IsMotorBusy())
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
	if(ControlArbitration_TryEnter(CONTROL_OWNER_EXTERNAL) == false)
	{
		return false;
	}

	/* 首次进入外控时清理空闲残留输出；重复申请外控时不打断已在进行的外控动作。 */
	if(already_external == false)
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
	if(s_control_owner != CONTROL_OWNER_EXTERNAL)
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
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type,uint8_t control_key)
{
	uint8_t key_owner = ControlArbitration_GetOwnerByKeySource(control_type);

	/* 手柄插拔事件只维护在线状态和通道记忆，不属于运行控制，互斥期间仍要接收。 */
	if(control_type == PLUGunPLUG)
	{
		return false;
	}

	/* 外控持有期间允许旧 HMI 和屏幕上的外控退出键通过，便于主动释放上位机控制。 */
	if(ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) &&
	   (((control_type == HMIkey) && (control_key == HMIkey_HMI_EXIT)) ||
		((control_type == SCREENKey) && (control_key == SCREENKey_HMI_EXIT))))
	{
		return false;
	}

	/* 不属于四类控制来源的消息不参与仲裁。 */
	if(key_owner == CONTROL_OWNER_NONE)
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
	//通知HMI标志位界面消失
    // 判断是否为外部命令触发的HMI退出
	if(key_value==HMIkey_HMI_EXIT)
	{
		//外部命令，说我要退出外部，全部交于动力自主
		//如果外部控制中，则停止相关内容，比如泵和电机
	}
    // 判断是否为导航界面按钮触发的强制退出
	else if(key_value==SCREENKey_HMI_EXIT)
	{
		//导航界面按钮强制退出，通知一下上位机，若要控制请重头校验识别
	}
 }

/*
 * 函数功能：处理屏幕/HMI 的控制方式切换与屏幕触控启动、停止动作。
 * 输入参数：key_value 屏幕或 HMI 下发的控制方式按键值。
 * 返回参数：无。
 */
 void ControlTypeActive(uint8_t key_value)
 {
	if(WorkMessage.alarm_flag==true)return;
	switch(key_value)
	{
		case SCREENKey_JTActi:
		case HMIkey_JTActi:
		{
			if(WorkMessage.runflag_work==true)return;
			//脚踏控制
			if(WorkMessage.drivetype_work==JTWORK||WorkMessage.touchactive_work==TOUCHWORK)return;
			WorkMessage.drivetype_work=JTWORK;
			if(WorkMessage.channel_work==CHANNEL_A)MemoryMsgA.drive_type=JTWORK;
			else if(WorkMessage.channel_work==CHANNEL_B)MemoryMsgB.drive_type=JTWORK;
			//注意界面更新
		}
		break;
		case SCREENKey_HandleActi:
		case HMIkey_HandleActi:
		{
			if(WorkMessage.runflag_work==true)return;
			//手动控制
			if(WorkMessage.drivetype_work==HANDLEWORK||WorkMessage.touchactive_work==TOUCHWORK)return;
			WorkMessage.drivetype_work=HANDLEWORK;
			if(WorkMessage.channel_work==CHANNEL_A)MemoryMsgA.drive_type=HANDLEWORK;
			else if(WorkMessage.channel_work==CHANNEL_B)MemoryMsgB.drive_type=HANDLEWORK;
			//界面调整更新
		}
		break;
		case SCREENKey_TouchActi:
		{
			if(WorkMessage.runflag_work==true)return;
			//触摸模式
            if(WorkMessage.touchactive_work==TOUCHWORK)return;
			//弹出窗模式
			/* 只激活触控模式，不直接落入 TouchStart，避免点“触控激活”时误启动电机。 */
			break;
		}
		case SCREENKey_TouchStart://启动
		if(WorkMessage.runflag_work==true)return;
		/* 屏幕触控启动前先申请屏幕控制权，若脚踏/手柄/外控正在控制则直接等待。 */
		if(ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN) == false)return;
		/* 触控启动后置位触控占用，直到屏幕触控退出才释放其它控制方式。 */
		WorkMessage.touchactive_work=TOUCHWORK;
		WorkMessage.runflag_work=true;//drive执行
		break;
		case SCREENKey_TouchEXIT://停止
		/* 屏幕触控退出时先停电机，再清触控占用，控制权释放要继续等待驱动反馈归零。 */
		WorkMessage.runflag_work=false;
		WorkMessage.speed_work=0U;
		WorkMessage.touchactive_work=0U;
		/* 这里不能直接 Exit，否则刚下发停止但电机尚未停稳时其它来源会提前接管。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN);
		//退出触控界面
		break;
	}
 }

void SpeedActive(uint8_t key_value)
{
	uint16_t speed_value;
	uint16_t speed_step;
	uint16_t speed_max;
	uint16_t speed_min;

	if(WorkMessage.alarm_flag==true)return;
	 if(WorkMessage.channel_work==CHANNEL_A)
		 {
			if(WorkMessage.dir_work==ZZDIR){
			speed_step=ChannelrecognizeMessageA.speed_zzstep;
			speed_max=ChannelrecognizeMessageA.speed_zzmax;
			speed_min=ChannelrecognizeMessageA.speed_zzmin;
			}
			else if(WorkMessage.dir_work==FZDIR){
			speed_step=ChannelrecognizeMessageA.speed_fzstep;
			speed_max=ChannelrecognizeMessageA.speed_fzmax;
			speed_min=ChannelrecognizeMessageA.speed_fzmin;
			}
			else if(WorkMessage.dir_work==OSCDIR){
			speed_step=ChannelrecognizeMessageA.speed_oscstep;
			speed_max=ChannelrecognizeMessageA.speed_oscmax;
			speed_min=ChannelrecognizeMessageA.speed_oscmin;
			}
		 }
		 else if(WorkMessage.channel_work==CHANNEL_B)
		 {
			if(WorkMessage.dir_work==ZZDIR){
			speed_step=ChannelrecognizeMessageB.speed_zzstep;
			speed_max=ChannelrecognizeMessageA.speed_zzmax;
			speed_min=ChannelrecognizeMessageA.speed_zzmin;
			}
			else if(WorkMessage.dir_work==FZDIR){
			speed_step=ChannelrecognizeMessageB.speed_fzstep;
			speed_max=ChannelrecognizeMessageA.speed_fzmax;
			speed_min=ChannelrecognizeMessageA.speed_fzmin;
			}
			else if(WorkMessage.dir_work==OSCDIR){
			speed_step=ChannelrecognizeMessageB.speed_oscstep;
			speed_max=ChannelrecognizeMessageA.speed_oscmax;
			speed_min=ChannelrecognizeMessageA.speed_oscmin;
			}
		 }
	switch(key_value)
	{
		case HANDLEKey_speed_add://忘记快加快减
		case HMIkey_SPEED_Add:
		case SCREENKey_SPEED_Add:
		 if(WorkMessage.channel_work==CHANNEL_A)
		 {
			speed_value= WorkMessage.speed_set_work+speed_step;
			if(speed_value>speed_max)speed_value=speed_max;
			WorkMessage.speed_set_work=speed_value;
			if(WorkMessage.dir_work==ZZDIR)MemoryMsgA.zz_speed=speed_value;
			else if(WorkMessage.dir_work==FZDIR)MemoryMsgA.fz_speed=speed_value;
			else if (WorkMessage.dir_work==OSCDIR)MemoryMsgA.osc_speed=speed_value;
		}
		 else if(WorkMessage.channel_work==CHANNEL_B)
		 {
			speed_value= WorkMessage.speed_set_work+speed_step;
			if(speed_value>speed_max)speed_value=speed_max;
			WorkMessage.speed_set_work=speed_value;
			if(WorkMessage.dir_work==ZZDIR)MemoryMsgB.zz_speed=speed_value;
			else if(WorkMessage.dir_work==FZDIR)MemoryMsgB.fz_speed=speed_value;
			else if (WorkMessage.dir_work==OSCDIR)MemoryMsgB.osc_speed=speed_value;
		 }
		//速度加
		break;
		case HANDLEKey_speed_sub:
		case HMIkey_SPEED_Sub:
		case SCREENKey_SPEED_Sub:
		if(WorkMessage.channel_work==CHANNEL_A)
		 {
			speed_value= WorkMessage.speed_set_work-speed_step;
			if(speed_value<speed_min)speed_value=speed_min;
			WorkMessage.speed_set_work=speed_value;
			if(WorkMessage.dir_work==ZZDIR)MemoryMsgA.zz_speed=speed_value;
			else if(WorkMessage.dir_work==FZDIR)MemoryMsgA.fz_speed=speed_value;
			else if (WorkMessage.dir_work==OSCDIR)MemoryMsgA.osc_speed=speed_value;
		 }
		 else if(WorkMessage.channel_work==CHANNEL_B)
		 {
			speed_value= WorkMessage.speed_set_work-speed_step;
			if(speed_value<speed_min)speed_value=speed_min;
			WorkMessage.speed_set_work=speed_value;
			if(WorkMessage.dir_work==ZZDIR)MemoryMsgB.zz_speed=speed_value;
			else if(WorkMessage.dir_work==FZDIR)MemoryMsgB.fz_speed=speed_value;
			else if (WorkMessage.dir_work==OSCDIR)MemoryMsgB.osc_speed=speed_value;
		 }
		//速度减
		break;
		case HANDLEKey_greaI://快速档位速度切换(现保留，考虑中)
		break;
		case HANDLEKey_greaII:
		break;
		case HANDLEKey_greaIII:
		//高倍模式
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
	uint8_t freq_step=10;
		if(WorkMessage.alarm_flag==true||WorkMessage.tool_type!=PLANER)return;
		switch(key_value)
		{
			case HMIkey_FREQ_Add:
			case SCREENKey_FREQ_Add:
			  if(WorkMessage.channel_work==CHANNEL_A)
			  {
					freq_value= WorkMessage.freq_work+freq_step;
					if(freq_value>ChannelrecognizeMessageA.freq_max)freq_value=ChannelrecognizeMessageA.freq_max;
				  WorkMessage.freq_work=MemoryMsgA.freq=freq_value;
			  }
			  else if(WorkMessage.channel_work==CHANNEL_B)
			  {
				freq_value= WorkMessage.freq_work+freq_step;
				if(freq_value>ChannelrecognizeMessageB.freq_max)freq_value=ChannelrecognizeMessageB.freq_max;
				  WorkMessage.freq_work=MemoryMsgB.freq=freq_value;
			  }
			//频率加
			break;
	
			case HMIkey_FREQ_Sub:
			case SCREENKey_FREQ_Sub:
			if(WorkMessage.channel_work==CHANNEL_A)
			  {
					freq_value= WorkMessage.freq_work-freq_step;
					if(freq_value<ChannelrecognizeMessageA.freq_min)freq_value=ChannelrecognizeMessageA.freq_min;
				    WorkMessage.freq_work=MemoryMsgA.freq=freq_value;
			  }
			  else if(WorkMessage.channel_work==CHANNEL_B)
			  {
				freq_value= WorkMessage.freq_work-freq_step;
				if(freq_value>ChannelrecognizeMessageB.freq_min)freq_value=ChannelrecognizeMessageB.freq_min;
				  WorkMessage.freq_work=MemoryMsgB.freq=freq_value;
			  }
			//频率减
			break;
		}
}
void DirActive(uint8_t key_value)
 {
	if(WorkMessage.runflag_work==true||WorkMessage.alarm_flag==true)return;
		switch(key_value)
		{
			case HANDLEKey_dir_Forward:
			case HMIkey_Dir_Forward:
			case SCREENKey_Dir_Forward:
            if(WorkMessage.channel_work==CHANNEL_A)
			{
                 WorkMessage.dir_work=MemoryMsgA.dir=ZZDIR;
				 WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
			}
			else if(WorkMessage.channel_work==CHANNEL_B)
			{
				  WorkMessage.dir_work=MemoryMsgB.dir=ZZDIR;
				  WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
			}
			//正向
			break;
			case HANDLEKey_dir_Reverse:
			case HMIkey_Dir_Reverse:
			case SCREENKey_Dir_Reverse:
			 if(WorkMessage.channel_work==CHANNEL_A)
			{
                 WorkMessage.dir_work=MemoryMsgA.dir=FZDIR;
				 WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
			}
			else if(WorkMessage.channel_work==CHANNEL_B)
			{
				  WorkMessage.dir_work=MemoryMsgB.dir=FZDIR;
				  WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
			}
			//反向
			break;
			case HANDLEKey_dir_OSC:
			case HMIkey_Dir_OSC:
			case SCREENKey_Dir_OSC:
			if(WorkMessage.channel_work==CHANNEL_A)
			{
                 WorkMessage.dir_work=MemoryMsgA.dir=OSCDIR;
				 WorkMessage.speed_set_work=MemoryMsgA.osc_speed;
			}
			else if(WorkMessage.channel_work==CHANNEL_B)
			{
				  WorkMessage.dir_work=MemoryMsgB.dir=OSCDIR;
				  WorkMessage.speed_set_work=MemoryMsgB.osc_speed;
			}
			//往复
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
	if(WorkMessage.runflag_work==true||WorkMessage.alarm_flag==true)return;
	switch(key_value)
	{
		case SCREENKey_PlanerH:    // 屏幕平面磨床水平控制按键
		case HMIkey_PlanerH:       // HMI平面磨床水平控制按键
		if(WorkMessage.channel_work==CHANNEL_A)
		{
			WorkMessage.tool_type=MemoryMsgA.tool_type=PLANER;
			WorkMessage.freq_work=MemoryMsgA.freq;
			WorkMessage.dir_work=MemoryMsgA.dir;
			if(WorkMessage.dir_work==ZZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
			}
			else if(WorkMessage.dir_work==FZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
			}
			else if(WorkMessage.dir_work==OSCDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.osc_speed;
			}
		}
		else if(WorkMessage.channel_work==CHANNEL_B)
		{
			WorkMessage.tool_type=MemoryMsgB.tool_type=PLANER;
			WorkMessage.freq_work=MemoryMsgB.freq;
			WorkMessage.dir_work=MemoryMsgB.dir;
			if(WorkMessage.dir_work==ZZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
			}
			else if(WorkMessage.dir_work==FZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
			}
			else if(WorkMessage.dir_work==OSCDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgB.osc_speed;
			}
		}
		//刨头
		break;
		case HMIkey_GrindH:       // HMI磨头水平控制按键
		case SCREENKey_GrindH:    // 屏幕磨头水平控制按键
		if(WorkMessage.channel_work==CHANNEL_A)
		{
			WorkMessage.tool_type=MemoryMsgA.tool_type=PLANER;
			WorkMessage.freq_work=MemoryMsgA.freq;
			WorkMessage.dir_work=MemoryMsgA.dir;
			if(WorkMessage.dir_work==ZZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
			}
			else if(WorkMessage.dir_work==FZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
			}
			
		}
		else if(WorkMessage.channel_work==CHANNEL_B)
		{
			WorkMessage.tool_type=MemoryMsgB.tool_type=PLANER;
			WorkMessage.freq_work=MemoryMsgB.freq;
			WorkMessage.dir_work=MemoryMsgB.dir;
			if(WorkMessage.dir_work==ZZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
			}
			else if(WorkMessage.dir_work==FZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
			}
			
		}
		//磨头
		break;
	}
}

void HandleSwitchActive(uint8_t key_value)//2026,4,19
 {
	if(WorkMessage.runflag_work==true||WorkMessage.alarm_flag==true)return;
	if(WorkMessage.channel_work==1)
	{
		if(WorkMessage.Channel_Bonline)
		 {
			switch(key_value)
			{
				case JTKey_middle_long:
				case HMIkey_HANDLE_B:
				case SCREENKey_HANDLE_B:
					WorkMessage.current_work=MemoryMsgB.current_work;
					WorkMessage.tool_reduction_ratio=MemoryMsgB.tool_reduction_ratio;
					WorkMessage.dir_work=MemoryMsgB.dir;
					WorkMessage.drivetype_work=MemoryMsgB.drive_type;
					WorkMessage.freq_work=MemoryMsgB.freq;
					if(WorkMessage.dir_work==ZZDIR)
					{
						WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
					}
					else if(WorkMessage.dir_work==FZDIR)
					{
						WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
					}
					else if(WorkMessage.dir_work==OSCDIR)
					{
						WorkMessage.speed_set_work=MemoryMsgB.osc_speed;
					}
					WorkMessage.tool_type=MemoryMsgB.tool_type;
					WorkMessage.hand_model=MemoryMsgB.hand_model;
					
					WorkMessage.channel_work=2;
					//队列通知界面通知
				break;
			}
		 }
	}
	else if(WorkMessage.channel_work==2)
	{
		if(WorkMessage.Channel_Aonline)
		{
			switch(key_value)
			{
				case JTKey_middle_long: 
				case HMIkey_HANDLE_A:
				case SCREENKey_HANDLE_A:
				WorkMessage.current_work=MemoryMsgA.current_work;
				WorkMessage.tool_reduction_ratio=MemoryMsgA.tool_reduction_ratio;
				WorkMessage.dir_work=MemoryMsgA.dir;
				WorkMessage.drivetype_work=MemoryMsgA.drive_type;
				WorkMessage.freq_work=MemoryMsgA.freq;
				if(WorkMessage.dir_work==ZZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
					
				}
				else if(WorkMessage.dir_work==FZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
				}
				else if(WorkMessage.dir_work==OSCDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.osc_speed;
				}
				WorkMessage.tool_type=MemoryMsgA.tool_type;
				WorkMessage.hand_model=MemoryMsgA.hand_model;
				
				WorkMessage.channel_work=2;
				//切换A口操作
				break;
			}
		}
	}
	//切换手柄，通知界面更改，参数记录调整（记忆功能）相对难一点，建立一个两路的全部信息数据结构体
}

void PlugORunPLUGActive(uint8_t key_value)
{
  switch(key_value)
  {
	  case SCREENKey_PLUG_A://插入A
			WorkMessage.hand_model=MemoryMsgA.hand_model=ChannelrecognizeMessageA.handle_type;//手柄类型
			if(WorkMessage.hand_model==PXBA_ONLINES)//PXBA或者其他类型手柄带按键手控
			{
				ControlSignalMessage.HMI_enable_flag=true;//白色使能，但是不选中
				if(WorkMessage.drivetype_work==TOUCHWORK||WorkMessage.drivetype_work==JTWORK)
				{
				     
				}
				else
				{
					MemoryMsgA.drive_type= WorkMessage.drivetype_work=HANDLEWORK;//等于手控
				}
			}
			 WorkMessage.current_work=MemoryMsgA.current_work=ChannelrecognizeMessageA.overloadThresholdFor;//电流


			WorkMessage.tool_reduction_ratio=MemoryMsgA.tool_reduction_ratio=ChannelrecognizeMessageA.meioticratio;//减速比
			WorkMessage.dir_work=MemoryMsgA.dir=ChannelrecognizeMessageA.run_direction;//方向
			WorkMessage.freq_work=MemoryMsgA.freq=ChannelrecognizeMessageA.freq_default;//A的默认频率
			if(WorkMessage.dir_work==ZZDIR)
				{
					MemoryMsgA.zz_speed=ChannelrecognizeMessageA.speed_zzstep;
					WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
				}
				else if(WorkMessage.dir_work==FZDIR)
				{
					MemoryMsgA.fz_speed=ChannelrecognizeMessageA.speed_fzstep;
					WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
				}
				else if(WorkMessage.dir_work==OSCDIR)
				{
					MemoryMsgA.osc_speed=ChannelrecognizeMessageA.speed_oscstep;
					WorkMessage.speed_set_work=MemoryMsgA.osc_speed;
				}
			WorkMessage.tool_type=MemoryMsgA.tool_type=ChannelrecognizeMessageA.tool_type;
			
			if(WorkMessage.Channel_Bonline)
			{
				//B插头区域，显示为连接状态头
			}
			WorkMessage.channel_work=CHANNEL_A;
			/* A 通道认证上线后立即刷新手柄区域，A 作为当前工作通道高亮，B 若在线则改为普通在线显示。 */
			Pubinterface_RefreshOnlineHandleDisplay();
	  break;
	  case SCREENKey_PLUG_B://插入B

			WorkMessage.hand_model=MemoryMsgB.hand_model=ChannelrecognizeMessageB.handle_type;//手柄类型
			if(WorkMessage.hand_model==PXBA_ONLINES)//PXBA或者其他类型手柄带按键手控
			{
				ControlSignalMessage.HMI_enable_flag=true;//白色使能，但是不选中
				if(WorkMessage.drivetype_work==TOUCHWORK||WorkMessage.drivetype_work==JTWORK)
				{

				}
				else
				{
				  MemoryMsgB.drive_type= WorkMessage.drivetype_work=HANDLEWORK;//等于手控
				}
			}
		    WorkMessage.current_work=MemoryMsgB.current_work=ChannelrecognizeMessageB.overloadThresholdFor;//电流
			WorkMessage.tool_reduction_ratio=MemoryMsgB.tool_reduction_ratio=ChannelrecognizeMessageB.meioticratio;//减速比
			WorkMessage.dir_work=MemoryMsgB.dir=ChannelrecognizeMessageB.run_direction;//方向
			WorkMessage.freq_work=MemoryMsgB.freq=ChannelrecognizeMessageB.freq_default;//A的默认频率
			if(WorkMessage.dir_work==ZZDIR)
				{
					MemoryMsgB.zz_speed=ChannelrecognizeMessageB.speed_zzstep;
					WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
				}
				else if(WorkMessage.dir_work==FZDIR)
				{
					MemoryMsgB.fz_speed=ChannelrecognizeMessageB.speed_fzstep;
					WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
				}
				else if(WorkMessage.dir_work==OSCDIR)
				{
					MemoryMsgB.osc_speed=ChannelrecognizeMessageB.speed_oscstep;
					WorkMessage.speed_set_work=MemoryMsgB.osc_speed;
				}
			WorkMessage.tool_type=MemoryMsgB.tool_type=ChannelrecognizeMessageB.tool_type;
			if(WorkMessage.Channel_Aonline)
			{
				//A插头区域，显示为连接状态头
			}
			WorkMessage.channel_work=CHANNEL_B;
			/* B 通道认证上线后立即刷新手柄区域，B 作为当前工作通道高亮，A 若在线则改为普通在线显示。 */
			Pubinterface_RefreshOnlineHandleDisplay();
	  break;
	  case SCREENKey_UNPLUG_A://拔出A

	  WorkMessage.Channel_Aonline=false;
	  //a区域显示手柄未连接，泵联动问题需要考虑
	  if(WorkMessage.Channel_Bonline)
	  {
           WorkMessage.current_work=MemoryMsgB.current_work;
		   WorkMessage.tool_reduction_ratio=MemoryMsgB.tool_reduction_ratio;
		   WorkMessage.dir_work=MemoryMsgB.dir;
		   WorkMessage.freq_work=MemoryMsgB.freq;

		  if(WorkMessage.dir_work==ZZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgB.zz_speed;
				}
				else if(WorkMessage.dir_work==FZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgB.fz_speed;
				}
				else if(WorkMessage.dir_work==OSCDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgB.osc_speed;
				}

		   WorkMessage.tool_type=MemoryMsgB.tool_type;
		   WorkMessage.drivetype_work=MemoryMsgB.drive_type;
		   WorkMessage.hand_model=MemoryMsgB.hand_model;
		   WorkMessage.channel_work=CHANNEL_B;
	  }
	  else
	  {
		  WorkMessage.runflag_work=false;
		  WorkMessage.hand_model=0;
		  WorkMessage.tool_type=0;
		  WorkMessage.channel_work=0;
          //界面暗黑无手柄接入（速度，频率，暗黑）
	  }
	  /* A 通道拔出后暗灭 A 手柄区域，若 B 通道仍在线则刷新 B 通道高亮状态。 */
	  Pubinterface_SendHandleDisplay(CHANNEL_A, 0U, false, false);
	  Pubinterface_RefreshOnlineHandleDisplay();
	  break;
	  case SCREENKey_UNPLUG_B://拔出B
	  WorkMessage.Channel_Bonline=false;
	  if(WorkMessage.Channel_Aonline)
	  {
           WorkMessage.current_work=MemoryMsgA.current_work;
		   WorkMessage.tool_reduction_ratio=MemoryMsgA.tool_reduction_ratio;
		   WorkMessage.dir_work=MemoryMsgA.dir;
		   WorkMessage.freq_work=MemoryMsgA.freq;
		   if(WorkMessage.dir_work==ZZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.zz_speed;
				}
				else if(WorkMessage.dir_work==FZDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.fz_speed;
				}
				else if(WorkMessage.dir_work==OSCDIR)
				{
					WorkMessage.speed_set_work=MemoryMsgA.osc_speed;
				}
		   WorkMessage.tool_type=MemoryMsgA.tool_type;
		   WorkMessage.drivetype_work=MemoryMsgA.drive_type;
		   WorkMessage.hand_model=MemoryMsgA.hand_model;
		   WorkMessage.channel_work=CHANNEL_A;
	  }
	  else
	  {
		  WorkMessage.runflag_work=false;
		  WorkMessage.hand_model=0;
		  WorkMessage.tool_type=0;
		  WorkMessage.channel_work=0;
				//界面暗黑无手柄接入（速度，频率，暗黑）
	  }
	  /* B 通道拔出后暗灭 B 手柄区域，若 A 通道仍在线则刷新 A 通道高亮状态。 */
	  Pubinterface_SendHandleDisplay(CHANNEL_B, 0U, false, false);
	  Pubinterface_RefreshOnlineHandleDisplay();
	  break;
  }
}

void ToolPosActive(uint8_t key_value)
{
	
	if(WorkMessage.runflag_work==true||WorkMessage.alarm_flag==true)return;
	
	if(WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES)
		{
			if(WorkMessage.tool_reduction_ratio&0xffff==500)//5碚减速比\100
			{
				
				switch(key_value)
				{
					case HMIkey_OpenPos_ClockWise:
					case SCREENKey_OpenPos_ClockWise:
					ToolPosMay(WorkMessage.channel_work,1,1);//一度
					//逆时针
					break;
					case JTKey_middle_short:
					case HMIkey_OpenPos_AntiClockWise:
					case SCREENKey_OpenPos_AntiClockWise:
					ToolPosMay(WorkMessage.channel_work,2,1);//一度
					//顺时针
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
    static uint8_t PumpA_Gear=0;
	static uint8_t PumpB_Gear=0;
    static bool PumpA_Start_flag=0;
	static bool PumpB_Start_flag=0;
	uint8_t pump_owner = ControlArbitration_GetOwnerByPumpKey(key_value);

	switch(key_value)
	{
			case JTkey_left_short: 
			PumpA_Gear++;
			if(PumpA_Gear>6)PumpA_Gear=0;
			if(pumpMessageA.type==0)
			{
                PumpA_Gear=0;	//队列通知界面暗黑
				return;
			}
			else if(pumpMessageA.type==DRAWWATER)//抽水
			{
				pumpMessageA.speed_work=3*PumpA_Gear;//每一档位增加30ml水
            }
			else if(pumpMessageA.type==INJECTWATER)//注水
			{
				if(pumpMessageA.timingDrainage_flag==true)
					return;
               switch(PumpA_Gear)
			   {
					case PUMPGEAR_ZERO:
					case PUMPGEAR_I :
					case PUMPGEAR_II:
					case PUMPGEAR_III :
					pumpMessageA.speed_work=PumpA_Gear*10;
					break;
					case PUMPGEAR_IV:
					pumpMessageA.speed_work=50;
					case PUMPGEAR_V:
					pumpMessageA.speed_work=70;
					break;
				
				}
			}
			else if(pumpMessageA.type==POURWATER)//灌注
			{
				
			   switch(PumpA_Gear)
			   {
				    case PUMPGEAR_ZERO:
					case PUMPGEAR_I :
					case PUMPGEAR_II:
					case PUMPGEAR_III :
					case PUMPGEAR_IV:
					pumpMessageA.speed_work=50*PumpA_Gear;//每一档位增加50ml水
					break;
					case PUMPGEAR_V:
					pumpMessageA.speed_work=300;
					break;
					
			   }
			}
			//队列通知ui更新界面
		break;
		case JTKey_left_long: 
		case HMIkey_APUMP_control:
		case SCREENKey_APUMP_control:
        	 if(pumpMessageA.type==0)
			 {
				pumpMessageA.run_flag=false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				//队列A停止
				return;
			 }
			 else 
			 {
				if(!pumpMessageA.run_flag)
				{
					/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
					if((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))return;
					pumpMessageA.run_flag=true;
					if(pumpMessageA.type==INJECTWATER)
					{
						pumpMessageA.timingDrainage_flag=true;
					}
					//队列发送界面按钮和数字变黄，A
					//队列发送泵运行设置数据
				}
				else
				{
					pumpMessageA.run_flag=false;
					pumpMessageA.timingDrainage_flag=false;
					ControlArbitration_ExitLocalControlIfIdle(pump_owner);
					//队列发送界面按钮和数字变黑,A
					//队列发送泵停止设置数据（数据变为0）
				}
				
			 }  
			 

		break;
		case JTKey_right_short: 
			PumpB_Gear++;
			if(PumpB_Gear>5)PumpB_Gear=0;
			if(pumpMessageB.type==0)
			{
                PumpB_Gear=0;
				//队列通知界面暗黑
			}
			else if(pumpMessageB.type==DRAWWATER)//抽水
			{
				pumpMessageB.speed_work=3*PumpB_Gear;//每一档位增加30ml水
            }
			else if(pumpMessageB.type==INJECTWATER)//注水
			{
               switch(PumpB_Gear)
			   {
					case PUMPGEAR_ZERO:
					case PUMPGEAR_I :
					case PUMPGEAR_II:
					case PUMPGEAR_III :
					pumpMessageB.speed_work=PumpB_Gear*10;
					break;
					case PUMPGEAR_IV:
					pumpMessageB.speed_work=50;
					case PUMPGEAR_V:
					pumpMessageB.speed_work=70;
					break;
				
				}
			}
			else if(pumpMessageB.type==POURWATER)//灌注
			{
			   switch(PumpB_Gear)
			   {
				    case PUMPGEAR_ZERO:
					case PUMPGEAR_I :
					case PUMPGEAR_II:
					case PUMPGEAR_III :
					case PUMPGEAR_IV:
						pumpMessageB.speed_work=50*PumpB_Gear;//每一档位增加50ml水
						break;
					case PUMPGEAR_V:
					    pumpMessageB.speed_work=300;//最大300ml水，记得宏定义
					break;
			   }
			}
		break;
		case JTKey_right_long: 
		case SCREENKey_BPUMP_control:
		case HMIkey_BPUMP_control:
				if(pumpMessageB.type==0)
				{
					pumpMessageB.run_flag=false;
					ControlArbitration_ExitLocalControlIfIdle(pump_owner);
					//对列通知为0;
					return;
				}
				else
				{
					if(!pumpMessageB.run_flag)
					{
						/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
						if((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))return;
						pumpMessageB.run_flag=true;
						if(pumpMessageB.type==INJECTWATER)
						{
							pumpMessageB.timingDrainage_flag=true;
						}
						//队列发送界面按钮和数字变黄
						//队列发送泵运行设置数据
					}
					else
					{
						pumpMessageB.timingDrainage_flag=false;
						pumpMessageB.run_flag=false;
						ControlArbitration_ExitLocalControlIfIdle(pump_owner);
						//队列发送界面按钮和数字变黑
						//队列发送泵停止设置数据（数据变为0）
					}
					
				}

		break;
		case HMIkey_APUMP_Add:
	    case SCREENKey_APUMP_Add:
		case HMIkey_APUMP_Sub:
		case SCREENKey_APUMP_Sub:
		    switch(pumpMessageA.type)
			{
				    case DRAWWATER: //抽
					pumpMessageA.speed_step_value=1;
					break;
					case INJECTWATER: //注
					if(pumpMessageA.timingDrainage_flag==true)
					return;
					pumpMessageA.speed_step_value=5;
					break;
					case POURWATER: //灌
					pumpMessageA.speed_step_value=30;
					break;
			}
			if(key_value==HMIkey_APUMP_Add||key_value==SCREENKey_APUMP_Add)
			{
				pumpMessageA.speed_work+=pumpMessageA.speed_step_value;
				if(pumpMessageA.speed_work>pumpMessageA.speed_Max)
				{
					pumpMessageA.speed_work=pumpMessageA.speed_Max;
				}
			}
			else
			{
				pumpMessageA.speed_work-=pumpMessageA.speed_step_value;
				if(pumpMessageA.speed_work<pumpMessageA.speed_Min)
				{
					pumpMessageA.speed_work=pumpMessageA.speed_Min;
				}
				
			}
			if(pumpMessageA.run_flag==true)
			{
				//队列通知A泵运行设置数据
			}
			break;

		case HMIkey_BPUMP_Add:
	    case SCREENKey_BPUMP_Add:
		case HMIkey_BPUMP_Sub:
		case SCREENKey_BPUMP_Sub:
		switch(pumpMessageB.type)
			{
				case DRAWWATER: //抽
				pumpMessageB.speed_step_value=1;
				break;
				case INJECTWATER: //注
				if(pumpMessageB.timingDrainage_flag==true)
				
				return;
				pumpMessageB.speed_step_value=5;
				break;
				case POURWATER: //灌
				pumpMessageB.speed_step_value=30;
				break;
			}
			if(key_value==HMIkey_APUMP_Add||key_value==SCREENKey_APUMP_Add)
			{
				pumpMessageB.speed_work+=pumpMessageB.speed_step_value;
				if(pumpMessageB.speed_work>pumpMessageB.speed_Max)
				{
					pumpMessageB.speed_work=pumpMessageB.speed_Max;
				}
			}
			else
			{
				pumpMessageB.speed_work-=pumpMessageB.speed_step_value;
				if(pumpMessageB.speed_work<pumpMessageB.speed_Min)
				{
					pumpMessageB.speed_work=pumpMessageB.speed_Min;
				}
				
			}

			if(pumpMessageB.run_flag==true)
			{
				//队列通知B泵运行设置数据
			}
		    break;
		case JTKey_Gently_left_start://其实可以判断手柄类型决定是否给与注水
				/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
				if((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))return;
				if(pumpMessageA.type==INJECTWATER)//注水
				{
					if(!WorkMessage.channel_work)//手柄在线，就可以运行A泵
					{
						//队列发送启动A
						if(pumpMessageA.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageA.timingDrainage_flag=false;
						}
						pumpMessageA.run_flag=true;
					}
				}
				else if(pumpMessageB.type==INJECTWATER)
				{
						
					if(!WorkMessage.channel_work)//手柄在线，就可以运行A泵
					{
						if(pumpMessageB.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageB.timingDrainage_flag=false;
						}
						pumpMessageB.run_flag=true;
							//队列发送启动B
					}
				}
		break;
		case JTKey_Gently_left_stop:
			if(pumpMessageA.type==INJECTWATER)//注水
			{
				//队列通知A泵停
				pumpMessageA.run_flag=false;;
			}
			else if(pumpMessageB.type==INJECTWATER)
			{
				//队列通知B泵停
				pumpMessageB.run_flag=false;;
			}
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		break;
		case JTKey_Gently_right_start:
			   /* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
			   if((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))return;
			   if(pumpMessageB.type==INJECTWATER)//注水
				{
					if(!WorkMessage.channel_work)//手柄在线，就可以运行A泵
					{
						//队列发送启动B
						if(pumpMessageB.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageB.timingDrainage_flag=false;
						}
						pumpMessageB.run_flag=true;
					}
				}
				else if(pumpMessageA.type==INJECTWATER)
				{
					//队列发送
					if(!WorkMessage.channel_work)//手柄在线，就可以运行A泵
					{
						//队列发送启动A
						if(pumpMessageA.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageA.timingDrainage_flag=false;
						}
						pumpMessageA.run_flag=true;
					}
				}

		break;
		case JTKey_Gently_rigth_stop:
			if(pumpMessageB.type==INJECTWATER)//注水
			{
				//队列通知A泵停
				pumpMessageB.run_flag=false;;
			}
			else if(pumpMessageA.type==INJECTWATER)
			{
				//队列通知B泵停
				pumpMessageA.run_flag=false;;
			}
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		break;
		case HMIkey_Gently_start:
		/* 历史 HMI 轻排属于外部来源，启动前也必须通过统一控制权仲裁。 */
		if((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))return;
  
		if(WorkMessage.channel_work==1)
		{
			if(pumpMessageA.type==INJECTWATER)
			{
				//队列发送启动A
			if(pumpMessageA.timingDrainage_flag==true)//如果正在排空
				{
					pumpMessageA.timingDrainage_flag=false;
				}
				pumpMessageA.run_flag=true;
			}
			else if(pumpMessageB.type==INJECTWATER)
			{
				//队列发送启动B
				if(pumpMessageB.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageB.timingDrainage_flag=false;
						}
						pumpMessageB.run_flag=true;
			}
		}
		else if(WorkMessage.channel_work==2)
		{
			if(pumpMessageB.type==INJECTWATER)
			{
				//队列发送启动B
				if(pumpMessageB.timingDrainage_flag==true)//如果正在排空
						{
							pumpMessageB.timingDrainage_flag=false;
						}
						pumpMessageB.run_flag=true;
			}
			else if(pumpMessageA.type==INJECTWATER)
			{
				//队列发送启动A
				if(pumpMessageA.timingDrainage_flag==true)//如果正在排空
				{
					pumpMessageA.timingDrainage_flag=false;
				}
				pumpMessageA.run_flag=true;
			}
		}
		 

		break;
		case HMIkey_Gently_stop:
       if(WorkMessage.channel_work==1)
		{
			if(pumpMessageA.type==INJECTWATER)
			{
				//队列发送停止A
				pumpMessageA.run_flag=false;
			}
			else if(pumpMessageB.type==INJECTWATER)
			{
				//队列发送停止B
				pumpMessageB.run_flag=false;
			}
		}
		else if(WorkMessage.channel_work==2)
		{
			if(pumpMessageB.type==INJECTWATER)
			{
				//队列发送停止B
                pumpMessageB.run_flag=false;
			}
			else if(pumpMessageA.type==INJECTWATER)
			{
				//队列停止B
				pumpMessageA.run_flag=false;
			}
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);

		break;
		default:
		break;
	}
}
