#include "control_arbitration.h"

#include "motoruartdata.h"
#include "motor_foot_trace.h"
#include "Pubinterface.h"
#include "sscUIDP.h"

/* 当前独占控制源只由本模块维护，业务模块通过公开接口申请、查询和释放。 */
static volatile uint8_t s_control_owner = CONTROL_OWNER_NONE;

/*
 * 函数功能：只读返回当前电机控制权持有者，供跨模块诊断快照使用。
 * 输入参数：无。
 * 返回参数：CONTROL_OWNER_* 当前值。
 */
uint8_t ControlArbitration_GetCurrentOwner(void)
{
	return s_control_owner; /* 单字节读在当前 MCU 上原子完成，诊断接口不得改写 owner。 */
}

/*
 * 函数功能：仲裁切换或释放控制权时统一停止电机、脚踏/外控标志和 A/B 泵输出。
 * 输入参数：无，函数直接清理 WorkMessage、ControlSignalMessage 和 pumpMessageA/B。
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
	/* 外控申请只停止 A 泵输出，不清除 speed_work 设定值，避免屏幕在外控授权后把泵速度显示成 0。 */
	/* 停止 B 泵输出，并取消排空计时，保持两路泵的仲裁动作一致。 */
	pumpMessageB.run_flag = false;
	pumpMessageB.timingDrainage_flag = false;
	pumpMessageB.timingDrainage_times = 0U;
	/* 外控申请只停止 B 泵输出，不清除 speed_work 设定值，保证停止态仍显示原来的泵速度参数。 */
	Pubinterface_RefreshPumpADisplay(); /* 仲裁停泵后刷新 A 泵数值和按钮，避免退出外控后屏幕保留旧速度。 */
	Pubinterface_RefreshPumpBDisplay(); /* 仲裁停泵后刷新 B 泵数值和按钮，避免泵已停但屏幕仍显示运行。 */
}

/*
 * 函数功能：触控或外部控制退出后，按“脚踏优先、手控次之”的规则计算本机控制模式。
 * 输入参数：无，读取脚踏在线标志、当前手柄型号和当前通道记忆。
 * 返回参数：JTWORK/HANDLEWORK/NOWORK，表示退出占用后的本机驱动方式。
 */
uint8_t ControlArbitration_GetLocalDriveTypeAfterExit(void)
{
	uint8_t drive_type = NOWORK; /* 默认保持无控制方式，只有明确满足脚踏或手控条件时才恢复本机入口。 */

	if (ControlSignalMessage.jt_enable_flag == true)
	{
		drive_type = JTWORK; /* 脚踏在线时优先回到脚控选中，满足触控退出后的脚控优先级要求。 */
	}
	else if (Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model))
	{
		drive_type = HANDLEWORK; /* 脚踏不在线且当前手柄支持手控时，回到手控选中。 */
	}
	else
	{
		drive_type = NOWORK; /* 没有脚踏也没有可手控手柄时，不虚亮任何控制方式。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		MemoryMsgA.drive_type = drive_type; /* A 通道同步新的回落结果，避免退出触控后仍记着 TOUCHWORK。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		MemoryMsgB.drive_type = drive_type; /* B 通道同步新的回落结果，保证后续切回通道时显示一致。 */
	}

	return drive_type;
}

/*
 * 函数功能：判断近期驱动板反馈的实际转速是否仍表示电机在转动。
 * 输入参数：无，通过电机回包模块读取带毫秒时刻的一致性快照。
 * 返回参数：true表示近期反馈仍非零；false表示已反馈零速、尚无反馈或旧反馈已经超过有效时间窗。
 */
static bool ControlArbitration_IsMotorMoving(void)
{
	/* 反馈新鲜时继续等待真实零速；停止后回包中断时，过期的最后非零值不再永久占用控制权。 */
	return MotorUart_IsRecentFeedbackMoving();
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
	return ControlArbitration_IsMotorMoving();
}

/*
 * 函数功能：当本地电机已经停稳时自动释放脚踏、屏幕或手柄的电机控制权。
 * 输入参数：无，读取当前 owner 和电机忙状态。
 * 返回参数：无；满足释放条件时把 s_control_owner 置为 CONTROL_OWNER_NONE。
 */
static void ControlArbitration_ReleaseLocalIfIdle(void)
{
	uint8_t old_owner; /* 保存自动释放前的本地 owner，诊断日志需要同时看到前后值。 */

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
		old_owner = s_control_owner; /* 只在确认停稳的真实释放边沿保存旧值。 */
		s_control_owner = CONTROL_OWNER_NONE;
		MotorFootTrace_Record(MF_TRACE_OWNER_EXIT, old_owner, CONTROL_OWNER_NONE); /* 自动释放也属于 owner 变化，不能只记录显式 Exit。 */
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
static uint8_t ControlArbitration_OwnerFromKey(uint8_t control_type)
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
uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value)
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
 * 函数功能：判断当前按键是否只是运行参数调节，不应被电机运行 owner 互斥丢弃。
 * 输入参数：control_type 为按键来源；control_key 为来源内的业务按键值。
 * 返回参数：true 表示该键只调整速度、频率或泵流量；false 表示仍按普通运行控制键仲裁。
 */
static bool ControlArbitration_IsRuntimeKey(uint8_t control_type, uint8_t control_key)
{
	/* 手柄实体速度加减只修改当前目标速度，不改变脚控、手控、触控或外控的运行归属。 */
	if (control_type == HANDLEKey)
	{
		return ((control_key == HANDLEKey_speed_add) ||
				(control_key == HANDLEKey_speed_sub));
	}

	/* 外部通信的速度、频率、A/B 泵加减属于参数调节，运行中也需要交给业务层刷新设定值。 */
	if (control_type == HMIkey)
	{
		return ((control_key == HMIkey_SPEED_Add) ||
				(control_key == HMIkey_SPEED_Sub) ||
				(control_key == HMIkey_FREQ_Add) ||
				(control_key == HMIkey_FREQ_Sub) ||
				(control_key == HMIkey_APUMP_Add) ||
				(control_key == HMIkey_APUMP_Sub) ||
				(control_key == HMIkey_BPUMP_Add) ||
				(control_key == HMIkey_BPUMP_Sub));
	}

	/* 新屏的速度四键、频率键和泵流量加减键都只是改参数，不能因为电机正在运行被队列层丢弃。 */
	if (control_type == SCREENKey)
	{
		return ((control_key == SCREENKey_SPEED_Add) ||
				(control_key == SCREENKey_SPEED_Sub) ||
				(control_key == SCREENKey_SPEED_Sub_Large) ||
				(control_key == SCREENKey_SPEED_Sub_Small) ||
				(control_key == SCREENKey_SPEED_Add_Small) ||
				(control_key == SCREENKey_SPEED_Add_Large) ||
				(control_key == SCREENKey_FREQ_Add) ||
				(control_key == SCREENKey_FREQ_Sub) ||
				(control_key == SCREENKey_APUMP_Add) ||
				(control_key == SCREENKey_APUMP_Sub) ||
				(control_key == SCREENKey_BPUMP_Add) ||
				(control_key == SCREENKey_BPUMP_Sub));
	}

	/* 脚踏短按由 Foot_ParseDataS 转成 A/B 泵调档键，本质也是泵流量调节。 */
	if (control_type == JTKey)
	{
		return ((control_key == JTkey_left_short) ||
				(control_key == JTKey_right_short));
	}

	/* 其它来源或其它键仍交给普通 owner 仲裁，避免误放行启动/模式切换类动作。 */
	return false;
}

/*
 * 函数功能：判断当前按键是否属于泵业务入口，泵输出不应被手柄电机 owner 直接拦截。
 * 输入参数：control_type 为按键来源；control_key 为来源内的泵业务按键值。
 * 返回参数：true 表示应进入 PUMPActive 继续按泵类型、在线状态和排空状态判断；false 表示不是泵业务键。
 */
static bool ControlArbitration_IsPumpKey(uint8_t control_type, uint8_t control_key)
{
	/* 脚踏短按调档、长按启停/排空、轻排开始/停止都通过同一个泵业务函数处理。 */
	if (control_type == JTKey)
	{
		return ((control_key == JTkey_left_short) ||
				(control_key == JTKey_left_long) ||
				(control_key == JTKey_right_short) ||
				(control_key == JTKey_right_long) ||
				(control_key == JTKey_Gently_left_start) ||
				(control_key == JTKey_Gently_left_stop) ||
				(control_key == JTKey_Gently_right_start) ||
				(control_key == JTKey_Gently_rigth_stop));
	}

	/* 外部通信泵加减、启停和轻排命令都要进入 PUMPActive，再由泵业务层决定是否真正生效。 */
	if (control_type == HMIkey)
	{
		return ((control_key == HMIkey_APUMP_Add) ||
				(control_key == HMIkey_APUMP_Sub) ||
				(control_key == HMIkey_APUMP_control) ||
				(control_key == HMIkey_BPUMP_Add) ||
				(control_key == HMIkey_BPUMP_Sub) ||
				(control_key == HMIkey_BPUMP_control) ||
				(control_key == HMIkey_Gently_start) ||
				(control_key == HMIkey_Gently_stop));
	}

	/* 屏幕 A/B 泵加减和启停也只属于泵业务，不占用手柄电机运行 owner。 */
	if (control_type == SCREENKey)
	{
		return ((control_key == SCREENKey_APUMP_Add) ||
				(control_key == SCREENKey_APUMP_Sub) ||
				(control_key == SCREENKey_APUMP_control) ||
				(control_key == SCREENKey_BPUMP_Add) ||
				(control_key == SCREENKey_BPUMP_Sub) ||
				(control_key == SCREENKey_BPUMP_control));
	}

	/* 非泵键不能绕过电机 owner 仲裁。 */
	return false;
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
	ControlArbitration_ReleaseLocalIfIdle();

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
	uint8_t old_owner; /* 保存成功进入前的 owner，失败申请不产生伪变化记录。 */

	/* 无效来源不能写入 owner，避免未知按键把仲裁状态写乱。 */
	if (ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 申请前先清掉已经停稳的本地 owner，保证电机停止后其它本地模式能接管。 */
	ControlArbitration_ReleaseLocalIfIdle();

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
	old_owner = s_control_owner; /* 成功写入前保存旧值，通常为 NONE。 */
	s_control_owner = owner;
	MotorFootTrace_Record(MF_TRACE_OWNER_ENTER, old_owner, owner); /* 记录真实成功进入，不改变仲裁返回值。 */
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
		MotorFootTrace_Record(MF_TRACE_OWNER_EXIT, owner, CONTROL_OWNER_NONE); /* 保存显式释放前后 owner。 */
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
	Pubinterface_ServiceTransientAlarms(); /* 电机任务周期统一维护三类限时弹窗，不把报警生命周期散落在仲裁模块。 */
	/* 统一复用本地 owner 释放逻辑，保证手柄、脚踏、屏幕停止后不因反馈延迟永久占用。 */
	ControlArbitration_ReleaseLocalIfIdle();
}

/*
 * 函数功能：系统级强制释放当前仲裁 owner，用于急停或全局清状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ForceRelease(void)
{
	uint8_t old_owner = s_control_owner; /* 急停清理前保存真实持有者。 */

	/* 急停或系统级清状态使用强制释放，让任何来源都不能继续占用控制权。 */
	s_control_owner = CONTROL_OWNER_NONE;
	MotorFootTrace_Record(MF_TRACE_OWNER_FORCE_RELEASE, old_owner, CONTROL_OWNER_NONE); /* 强制释放单独使用事件11，便于和正常停稳退出区分。 */
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
		ControlSignalMessage.HMI_control_flag = false; /* 首次进入外控时确认电机未启动；重复保活申请不能清运行标志，否则小电脑会从 40 退回 39。 */
	}

	/* 置位外部控制权锁，脚踏、屏幕、手柄按键会在各自入口被拦截。 */
	WorkMessage.hmiactive_work = 1U;
	/* 触控/外控占用标志同步置位，屏幕模式切换逻辑也能看到外控占用。 */
	WorkMessage.touchactive_work = TOUCHWORK;
	/* 当前驱动方式切到外部控制，心跳和驱动状态可以看到外控来源。 */
	WorkMessage.drivetype_work = TOUCHWORK;
	/* 外部控制已使能，但申请阶段不直接启动电机。 */
	ControlSignalMessage.HMI_enable_flag = true;
	/* 申请成功只表示外控 owner 已取得，不能再打开 UI_TOUCH_ID，否则屏幕会弹出 70 号触控工作区。 */
	Pubinterface_RefreshControlModeDisplay(); /* 外控内部复用 TOUCHWORK 做互斥，但显示层仍按小电脑图标表达外控状态。 */
	if (already_external == false)
	{
		Pubinterface_RefreshExternalCommDisplay(true, false); /* 首次申请外控成功但尚未控制输出时只显示 39 白色小电脑。 */
	}
	return true;
}

/*
 * 函数功能：退出外部通信控制模式，停止外控遗留输出并恢复本机可接管状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ReleaseExternalControl(void)
{
	uint8_t data[10] = {0U}; /* 退出外控时给 UI_TOUCH_ID 发送全零参数，确保触控弹窗关闭。 */

	/* 当前不是外控时，退出外控只清理外控显示残留，不能影响脚踏、屏幕或手柄正在进行的控制。 */
	if (s_control_owner != CONTROL_OWNER_EXTERNAL)
	{
		WorkMessage.hmiactive_work = 0U; /* 非外控 owner 时只清界面外控标志，不能停本机脚踏/手柄/触控运行。 */
		ControlSignalMessage.HMI_enable_flag = false; /* 上位机退出命令到来后撤销外控可用标志，避免状态继续显示为外控占用。 */
		if (WorkMessage.touchactive_work != TOUCHWORK)
		{
			SendUIDSMessage(UI_TOUCH_ID, false, data); /* 未处于本机触控时才清理残留外控弹窗，避免误关本机触控工作区。 */
		}
		Pubinterface_RefreshControlModeDisplay(); /* UITOUCHDP(false) 会把触控主按钮置暗，这里重绘为当前本机白/黄状态。 */
		Pubinterface_RefreshExternalCommDisplay(false, false); /* 退出外部通信后不再保留 39/40 小电脑图标，避免误提示仍在线。 */
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
	SendUIDSMessage(UI_TOUCH_ID, false, data); /* 外控释放完成后关闭触控/外控窗口，UI 不再显示占用状态。 */
	Pubinterface_RefreshControlModeDisplay(); /* 外控退出后按本机状态重绘脚控/手控/触控，清掉外控期间的残留高亮。 */
	Pubinterface_RefreshExternalCommDisplay(false, false); /* 主动退出外控后按需求隐藏小电脑图标，不再显示白色在线态。 */
}

/*
 * 函数功能：按键队列分发前判断当前消息是否应被仲裁阻塞。
 * 输入参数：control_type 按键来源类型；control_key 具体业务按键值。
 * 返回参数：true 表示本消息应丢弃等待当前 owner 结束；false 表示可继续分发。
 */
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type, uint8_t control_key)
{
	uint8_t key_owner = ControlArbitration_OwnerFromKey(control_type);

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

	/* 运行参数调节允许跨本地 owner 生效；但外控已独占时，本地脚踏/屏幕/手柄不能绕过外控锁。 */
	if (ControlArbitration_IsRuntimeKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			if (control_type == SCREENKey)
			{
				return false; /* 外控 owner 期间允许屏幕修改速度/频率/泵流量参数，但屏幕启停键仍走后面的业务键拦截。 */
			}
			return true; /* 外部控制占用时只允许外部来源继续调参，防止本地按键改变外控运行设定。 */
		}
		return false; /* 非外控独占时放行到 SpeedActive/FreqActive/PUMPActive，由业务函数刷新设定值。 */
	}

	/* 泵业务和手柄电机 owner 分离，先放行到 PUMPActive，再由泵类型、在线状态和排空状态判断。 */
	if (ControlArbitration_IsPumpKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			return true; /* 外控占用期间本地泵键也不能抢改泵状态，保持外部控制一致性。 */
		}
		return false; /* 脚踏、屏幕和外部泵键都进入统一泵业务入口，不再被电机运行 owner 丢弃。 */
	}

	/* 不属于四类控制来源的消息不参与仲裁。 */
	if (key_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前已有其它来源占用时，丢弃该按键，必须等当前控制方式结束。 */
	return ControlArbitration_IsBusyByOther(key_owner);
}
