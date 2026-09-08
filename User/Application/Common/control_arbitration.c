#include "control_arbitration.h"

#include "motoruartdata.h"
#include "Pubinterface.h"
#include "sscUIDP.h"

/* 记录当前由谁控制手柄电机；其他模块只能通过下方接口申请、查询或释放。 */
static volatile uint8_t s_control_owner = CONTROL_OWNER_NONE;

/*
 * 函数功能：切换或退出外控时，清除电机和 A/B 泵运行请求，以及各控制来源的启动标志。
 * 输入参数：无，函数直接清理 WorkMessage、ControlSignalMessage 和 pumpMessageA/B。
 * 返回参数：无。
 */
static void ControlArbitration_StopMotionOutput(void)
{
	/* 切换控制来源时先撤销电机运行请求，不能继续沿用上一次启动状态。 */
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
	pumpMessageA.pedalDrainage_flag = false; /* 同时结束 A 泵的脚踏临时排空，下一次启动不能沿用旧排空状态。 */
	pumpMessageA.timingDrainage_times = 0U;
	/* 外控申请只停止 A 泵输出，不清除 speed_work 设定值，避免屏幕在外控授权后把泵速度显示成 0。 */
	/* 与 A 泵一样，撤销 B 泵运行请求并取消排空计时。 */
	pumpMessageB.run_flag = false;
	pumpMessageB.timingDrainage_flag = false;
	pumpMessageB.pedalDrainage_flag = false; /* 同时结束 B 泵的脚踏临时排空，避免普通启动沿用旧排空状态。 */
	pumpMessageB.timingDrainage_times = 0U;
	/* 外控申请只停止 B 泵输出，不清除 speed_work 设定值，保证停止态仍显示原来的泵速度参数。 */
	Pubinterface_RefreshPumpADisplay(); /* 刷新 A 泵数值和按钮，显示停止后的状态。 */
	Pubinterface_RefreshPumpBDisplay(); /* 刷新 B 泵数值和按钮，显示停止后的状态。 */
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
		MemoryMsgA.drive_type = drive_type; /* 把退出后选择的本机控制方式记到 A 通道，不能继续保留触控状态。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		MemoryMsgB.drive_type = drive_type; /* 把退出后选择的本机控制方式记到 B 通道，切回此通道时仍显示同一方式。 */
	}

	return drive_type;
}

/*
 * 函数功能：判断近期驱动板反馈的实际转速是否仍表示电机在转动。
 * 输入参数：无，读取同一份驱动反馈中的速度和接收时刻。
 * 返回参数：true表示近期反馈仍非零；false表示已反馈零速、尚无反馈或旧反馈已经超过有效时间窗。
 */
static bool ControlArbitration_IsMotorMoving(void)
{
	/* 最近反馈非零时继续等待；反馈超过有效时间后，不再用旧速度阻止切换控制方式。 */
	return MotorUart_IsRecentFeedbackMoving();
}

/*
 * 函数功能：判断是否还有电机运行请求，或最近驱动反馈仍显示电机在转。
 * 输入参数：无，读取 runflag_work 命令标志和 driver_speed_feedback 实际反馈。
 * 返回参数：true 表示有运行请求或最近反馈仍非零；false 表示无运行请求，且反馈为零、无反馈或已超时。
 */
static bool ControlArbitration_IsMotorBusy(void)
{
	/* runflag_work 是主控下发运行帧的命令源，置位时必须认为电机正在被控制。 */
	if (WorkMessage.runflag_work == true)
	{
		return true;
	}

	/* 没有运行请求时继续检查反馈；零速或反馈超时后，才允许其他来源申请控制。 */
	return ControlArbitration_IsMotorMoving();
}

/*
 * 函数功能：本地来源不再要求运行，且反馈为零或超时后，释放电机控制权。
 * 输入参数：无，读取当前控制来源和电机运行、反馈状态。
 * 返回参数：无；满足释放条件时把 s_control_owner 置为 CONTROL_OWNER_NONE。
 */
static void ControlArbitration_ReleaseLocalIfIdle(void)
{
	/* 外部通信必须由主动退出、超时释放或急停释放，不能因为本地电机停稳自动退出。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 没有控制来源占用时，无需释放。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return;
	}

	/* 无运行请求，且反馈为零或超时后，脚踏、屏幕和手柄按键可以重新申请控制。 */
	if (ControlArbitration_IsMotorBusy() == false)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

/*
 * 函数功能：校验传入的控制源编号是否合法。
 * 输入参数：owner 控制源编号，取 CONTROL_OWNER_EXTERNAL/FOOT/SCREEN/HANDLE。
 * 返回参数：true 表示编号属于四种控制来源；false 表示编号无效，不允许申请控制。
 */
static bool ControlArbitration_IsValidOwner(uint8_t owner)
{
	/* 只接受外控、脚踏、屏幕和手柄按键，不能记录无法识别的控制来源。 */
	return ((owner == CONTROL_OWNER_EXTERNAL) ||
			(owner == CONTROL_OWNER_FOOT) ||
			(owner == CONTROL_OWNER_SCREEN) ||
			(owner == CONTROL_OWNER_HANDLE));
}

/*
 * 函数功能：根据按键消息来源，确定它代表哪一种电机控制方式。
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
 * 函数功能：判断泵按键属于外控，还是不占用电机控制权的本地操作。
 * 输入参数：key_value 业务按键值，包含脚踏泵键、屏幕泵键和外控 HMI 泵键。
 * 返回参数：本地泵键返回 CONTROL_OWNER_NONE；外控泵键返回 CONTROL_OWNER_EXTERNAL。
 */
uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value)
{
	/* 脚踏泵键和轻排键只控制泵，不占用手柄电机控制权。 */
	if ((key_value == JTkey_left_short) ||
		(key_value == JTKey_left_long) ||
		(key_value == JTKey_right_short) ||
		(key_value == JTKey_right_long) ||
		(key_value == JTKey_Gently_left_start) ||
		(key_value == JTKey_Gently_left_stop) ||
		(key_value == JTKey_Gently_right_start) ||
		(key_value == JTKey_Gently_rigth_stop))
	{
		/* 脚踏启动电机时才由 FootControlTask 申请控制权；这些泵键不申请。 */
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
		/* 屏幕触控启动电机时才申请控制权；这些泵键不申请。 */
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
 * 函数功能：判断按键是否只改速度、频率或泵流量，以便在电机运行时仍允许调节。
 * 输入参数：control_type 为按键来源；control_key 为来源内的业务按键值。
 * 返回参数：true 表示只调参数；false 表示还要按普通按键检查当前由谁控制。
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

	/* 其他按键仍要检查当前由谁控制，不能把启停或模式切换当成参数调节放行。 */
	return false;
}

/*
 * 函数功能：识别泵按键；本地泵操作由泵模块判断，不因手柄电机正在运行而直接丢弃。
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

	/* 屏幕 A/B 泵加减和启停只控制泵，不占用手柄电机控制权。 */
	if (control_type == SCREENKey)
	{
		return ((control_key == SCREENKey_APUMP_Add) ||
				(control_key == SCREENKey_APUMP_Sub) ||
				(control_key == SCREENKey_APUMP_control) ||
				(control_key == SCREENKey_BPUMP_Add) ||
				(control_key == SCREENKey_BPUMP_Sub) ||
				(control_key == SCREENKey_BPUMP_control));
	}

	/* 其他按键还要继续检查电机控制来源。 */
	return false;
}

/*
 * 函数功能：判断指定来源是否正在控制电机。
 * 输入参数：owner 待查询的控制源编号。
 * 返回参数：true 表示当前控制来源与输入一致；false 表示不一致。
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
	/* 查询前先检查本地控制是否已结束，避免旧的控制来源挡住下一次操作。 */
	ControlArbitration_ReleaseLocalIfIdle();

	/* 没人占用时，允许新的来源申请控制。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 申请者已经持有控制权时，允许继续操作或停止。 */
	if (s_control_owner == owner)
	{
		return false;
	}

	/* 外部通信授权期间，本地脚踏、屏幕、手柄都必须等待外控主动退出或超时释放。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return true;
	}

	/* 只有手柄电机仍有运行请求或最近反馈非零时，才阻止其他本地来源接管；泵状态不参与。 */
	return ControlArbitration_IsMotorBusy();
}

/*
 * 函数功能：尝试让指定控制源取得电机控制权。
 * 输入参数：owner 申请控制权的来源编号。
 * 返回参数：true 表示申请成功或已经持有；false 表示无效来源或被其它来源阻塞。
 */
bool ControlArbitration_TryEnter(uint8_t owner)
{
	/* 先检查来源编号，无效来源不能取得控制权。 */
	if (ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 申请前检查上一次本地控制是否已结束，结束后才允许新来源接管。 */
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

	/* 允许切换后记下新控制来源；本地单独操作泵不会调用这里。 */
	s_control_owner = owner;
	return true;
}

/*
 * 函数功能：结束指定来源的控制；只有当前控制来源与参数一致时才释放。
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
 * 函数功能：本地来源没有运行请求，且反馈为零或超时后，释放它的电机控制权。
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

	/* 仍有运行请求，或有效期内的反馈非零时，暂不允许其他来源接管。 */
	if (ControlArbitration_IsMotorBusy())
	{
		return;
	}

	/* 满足释放条件后结束当前来源的控制；不检查泵是否运行。 */
	ControlArbitration_Exit(owner);
}

/*
 * 函数功能：周期刷新电机控制权释放状态。
 * 输入参数：无，由电机输出任务或反馈任务周期调用。
 * 返回参数：无；本地来源结束后自动释放，外控必须由退出或超时等处理主动释放。
 */
void ControlArbitration_RefreshMotorOwner(void)
{
	Pubinterface_ServiceTransientAlarms(); /* 检查三类限时报警是否到期，需要结束时由公共界面模块清除。 */
	/* 检查本地控制是否可以结束，避免反馈中断后一直占用控制权。 */
	ControlArbitration_ReleaseLocalIfIdle();
}

/*
 * 函数功能：无条件清除当前控制来源；本函数本身不发送停机或停泵命令。
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
 * 返回参数：true 表示当前控制来源是外控，且外控启用标志非零；false 表示条件不满足。
 */
bool ControlArbitration_IsExternalActive(void)
{
	/* 当前控制来源和外控启用标志必须同时符合，才表示外控已启用。 */
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

	/* 标记外控已启用；本地启停会受限制，屏幕调参数仍可按键规则放行。 */
	WorkMessage.hmiactive_work = 1U;
	/* 触控/外控占用标志同步置位，屏幕模式切换逻辑也能看到外控占用。 */
	WorkMessage.touchactive_work = TOUCHWORK;
	/* 当前驱动方式切到外部控制，心跳和驱动状态可以看到外控来源。 */
	WorkMessage.drivetype_work = TOUCHWORK;
	/* 外部控制已使能，但申请阶段不直接启动电机。 */
	ControlSignalMessage.HMI_enable_flag = true;
	/* 外控申请成功不代表进入本机触控界面，不能打开 UI_TOUCH_ID 的 70 号窗口。 */
	Pubinterface_RefreshControlModeDisplay(); /* 外控内部使用 TOUCHWORK 状态，但屏幕应显示小电脑图标。 */
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
		WorkMessage.hmiactive_work = 0U; /* 当前不是外控在操作时，只清外控显示标志，不能停止正在进行的本机操作。 */
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
	/* 清除外控启用标志，后续再释放控制来源，允许本机重新操作。 */
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
 * 函数功能：处理按键前检查当前控制来源，决定该按键是否应丢弃。
 * 输入参数：control_type 按键来源类型；control_key 具体业务按键值。
 * 返回参数：true 表示丢弃本条按键消息，不会留到稍后执行；false 表示继续交给按键处理函数。
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

	/* 本地运行中仍可调参数；外控期间也允许屏幕调参，但不允许脚踏和手柄按键改参数。 */
	if (ControlArbitration_IsRuntimeKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			if (control_type == SCREENKey)
			{
				return false; /* 外控期间允许屏幕修改速度、频率和泵流量；屏幕启停键不在此处分支中。 */
			}
			return true; /* 屏幕已在上方放行，这里拒绝脚踏和手柄按键在外控期间改参数。 */
		}
		return false; /* 非外控独占时放行到 SpeedActive/FreqActive/PUMPActive，由业务函数刷新设定值。 */
	}

	/* 泵键由 PUMPActive 检查泵类型、连接和排空状态；只有外控期间才在这里拒绝本地泵键。 */
	if (ControlArbitration_IsPumpKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			return true; /* 外控占用期间本地泵键也不能抢改泵状态，保持外部控制一致性。 */
		}
		return false; /* 把泵键交给泵处理函数，不因手柄电机正在运行而丢弃。 */
	}

	/* 没有对应电机控制来源的消息，继续交给后续处理。 */
	if (key_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前已有其它来源占用时，丢弃该按键，必须等当前控制方式结束。 */
	return ControlArbitration_IsBusyByOther(key_owner);
}
