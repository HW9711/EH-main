//调用对应头文件
#include "Pubinterface.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include <screen.h>
//#include "datahand.h"
#include "sscDRIVE.h"





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
	/* 所有新报警统一写 WorkMessage，避免再通过 SysRunData.WarnID/StuckFlag 分散传递。 */
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

static bool ControlArbitration_IsValidOwner(uint8_t owner)
{
	/* 只接受四种正式控制来源，防止错误参数把仲裁锁写成未知状态。 */
	return ((owner == CONTROL_OWNER_EXTERNAL) ||
			(owner == CONTROL_OWNER_FOOT) ||
			(owner == CONTROL_OWNER_SCREEN) ||
			(owner == CONTROL_OWNER_HANDLE));
}

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

static uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value)
{
	/* 脚踏泵按键和脚踏轻排按键都属于脚踏控制。 */
	if((key_value == JTkey_left_short) ||
	   (key_value == JTKey_left_long) ||
	   (key_value == JTKey_right_short) ||
	   (key_value == JTKey_right_long) ||
	   (key_value == JTKey_Gently_left_start) ||
	   (key_value == JTKey_Gently_left_stop) ||
	   (key_value == JTKey_Gently_right_start) ||
	   (key_value == JTKey_Gently_rigth_stop))
	{
		return CONTROL_OWNER_FOOT;
	}

	/* 屏幕泵按钮由屏幕控制来源占用。 */
	if((key_value == SCREENKey_APUMP_Add) ||
	   (key_value == SCREENKey_APUMP_Sub) ||
	   (key_value == SCREENKey_APUMP_control) ||
	   (key_value == SCREENKey_BPUMP_Add) ||
	   (key_value == SCREENKey_BPUMP_Sub) ||
	   (key_value == SCREENKey_BPUMP_control))
	{
		return CONTROL_OWNER_SCREEN;
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

bool ControlArbitration_IsOwner(uint8_t owner)
{
	/* 当前持有者和查询来源一致时，允许该来源继续控制或主动停止。 */
	return (s_control_owner == owner);
}

bool ControlArbitration_IsBusyByOther(uint8_t owner)
{
	/* 空闲时任何合法来源都可以尝试申请控制权。 */
	if(s_control_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 同一个来源可以持续发送控制或停止命令，不视为互斥冲突。 */
	if(s_control_owner == owner)
	{
		return false;
	}

	/* 只要已有其它来源占用，当前来源必须等待对方结束。 */
	return true;
}

bool ControlArbitration_TryEnter(uint8_t owner)
{
	/* 无效来源不允许拿控制权，避免未知按键把系统锁死。 */
	if(ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 当前空闲时记录新的控制来源。 */
	if(s_control_owner == CONTROL_OWNER_NONE)
	{
		s_control_owner = owner;
		return true;
	}

	/* 当前来源已经持有控制权时，重复申请按成功处理。 */
	if(s_control_owner == owner)
	{
		return true;
	}

	/* 其它来源正在控制，必须等它释放后才能接管。 */
	return false;
}

void ControlArbitration_Exit(uint8_t owner)
{
	/* 只有当前持有者本人才能释放，避免其它来源误清正在运行的控制权。 */
	if(s_control_owner == owner)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

void ControlArbitration_ExitLocalControlIfIdle(uint8_t owner)
{
	/* 上位机控制必须由退出外控或急停释放，不能因为单个泵停止就自动失效。 */
	if(owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 电机仍在运行时，本地来源还没有结束。 */
	if(WorkMessage.runflag_work == true)
	{
		return;
	}

	/* 任意脚踏联动标志仍有效时，脚踏控制还没有完全松开。 */
	if(ControlSignalMessage.jtL_control_flag ||
	   ControlSignalMessage.jtR_control_flag ||
	   ControlSignalMessage.jtL_gentlypump_flag ||
	   ControlSignalMessage.jtR_gentlypump_flag)
	{
		return;
	}

	/* A/B 泵仍在运行或定时排空时，屏幕/脚踏泵控制还没有结束。 */
	if(pumpMessageA.run_flag ||
	   pumpMessageB.run_flag ||
	   pumpMessageA.timingDrainage_flag ||
	   pumpMessageB.timingDrainage_flag)
	{
		return;
	}

	/* 所有本地输出都已经停下，释放当前本地控制来源。 */
	ControlArbitration_Exit(owner);
}

void ControlArbitration_ForceRelease(void)
{
	/* 急停或系统级清状态使用强制释放，让任何来源都不能继续占用控制权。 */
	s_control_owner = CONTROL_OWNER_NONE;
}

bool ControlArbitration_IsExternalActive(void)
{
	/* 外控是否有效以仲裁持有者为准，hmiactive_work 只作为界面/状态同步标志。 */
	return ((s_control_owner == CONTROL_OWNER_EXTERNAL) && (WorkMessage.hmiactive_work != 0U));
}

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
		/* 屏幕触控退出时先停电机，再清触控占用，最后释放屏幕控制权。 */
		WorkMessage.runflag_work=false;
		WorkMessage.speed_work=0U;
		WorkMessage.touchactive_work=0U;
		ControlArbitration_Exit(CONTROL_OWNER_SCREEN);
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
					/* A 泵由屏幕/脚踏/HMI 启动前先占用对应控制来源，互斥其它控制方式。 */
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
						/* B 泵由屏幕/脚踏/HMI 启动前先占用对应控制来源，互斥其它控制方式。 */
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
				/* 脚踏轻排启动前先占用脚踏控制权，避免上位机/屏幕/手柄同时改泵。 */
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
			   /* 脚踏轻排启动前先占用脚踏控制权，避免上位机/屏幕/手柄同时改泵。 */
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

