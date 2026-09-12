#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "Pubinterface.h"
#include "screenkey.h"
#include "sscKEYBH.h"

  kernel_task_t    KeyBehaviorsHandle;
QueueHandle_t KeyBehivQueue = NULL;

typedef struct
{
	uint8_t control_type;//脚踏key,显示屏key,外部通讯key，手控key
	uint8_t control_key;
	uint32_t queued_tick_ms; /* 记录准备放入队列时的毫秒数；取出时丢弃过期的触控按住信号，防止延迟启动。 */
}KeyBehMessage_t;
static void KeyBehivQueue_Init(void)
{
    KeyBehivQueue = Kernel_QueueCreate(20, sizeof(KeyBehMessage_t), "KeyBehivQueue");
}
/*
 * 函数功能：把按键或插拔消息放进队列；队列满就立即返回失败，不在当前业务任务里等待。
 * 输入参数：control_type 为事件来源类型；control_key 为该来源下的具体按键或插拔事件值。
 * 返回参数：true表示消息已入队；false表示队列未创建或已满，手柄扫描任务会保留必须发送的拔出消息并重试。
 */
bool SendKeyBehMessage(uint8_t control_type,uint8_t control_key)
{
	// static uint8_t control_types=0;
	// static uint8_t control_keys=0;
	// if(control_types==control_type&&control_keys==control_key)return;
	// else
	// {
	// 	control_types=control_type;
	// 	control_keys=control_key;
	// }
	if(KeyBehivQueue == NULL) return false; /* 按键队列尚未创建时不能投递事件，返回失败让关键状态事件保留并在下个扫描周期重试。 */
	KeyBehMessage_t msg;
	msg.control_type =control_type ;
	msg.control_key = control_key;
	msg.queued_tick_ms = HAL_GetTick(); /* 普通按键只记录时间；触控按住信号取出时会检查是否已过 420ms。 */
	return (Kernel_QueueSend(KeyBehivQueue, &msg, 0U) == pdPASS); /* 处理队列的任务也要取得同一把业务锁；当前任务等待会挡住它，因此队列满时直接返回失败。 */
}
/*
 * 函数功能：按脚踏按键编号执行泵控制、开口定位或手柄切换。
 * 输入参数：key_value 为脚踏按键编号。
 * 返回参数：无。
 */
void JTKeyBehavior(uint8_t key_value)
{
	switch (key_value)
	{
	  case JTkey_left_short:
	  case JTKey_left_long:
	  case JTKey_right_short:
	  case JTKey_right_long:
	  case JTKey_Gently_left_start://左侧踏板开始轻排空。
	  case JTKey_Gently_left_stop://左侧踏板停止轻排空。
	  case JTKey_Gently_right_start://右侧踏板开始轻排空。
	  case JTKey_Gently_rigth_stop://右侧踏板停止轻排空。
	  PUMPActive(key_value);
	  break;
	  case JTKey_middle_short:
	  ToolPosActive(key_value);//默认顺时针
	  break;
	  case JTKey_middle_long:
	  HandleSwitchActive(key_value);
	  break;
	
	}
}
/*
 * 函数功能：按 HMI 按键编号调用泵、速度、方向或控制方式的处理函数。
 * 输入参数：key_value 为 HMI 按键编号。
 * 返回参数：无。
 */
void HMIkeyBehanior(uint8_t key_value)
{
	switch(key_value)
	{
		case HMIkey_APUMP_Add:
		case HMIkey_APUMP_Sub:
		case HMIkey_BPUMP_Add:
		case HMIkey_BPUMP_Sub:
	    case HMIkey_APUMP_control:
		case HMIkey_BPUMP_control:
		case HMIkey_Gently_start://开始轻排空。
		case HMIkey_Gently_stop://停止轻排空。
		PUMPActive(key_value);
		break;
		case HMIkey_SPEED_Add:
		case HMIkey_SPEED_Sub:
		SpeedActive(key_value);
		break;
		case HMIkey_FREQ_Add:
		case HMIkey_FREQ_Sub:
		FreqActive(key_value);
		break;
		case HMIkey_HANDLE_A:
		case HMIkey_HANDLE_B:
		HandleSwitchActive(key_value);
		break;
		case HMIkey_PlanerH:
		case HMIkey_GrindH:
		PlanerGridH(key_value);
		break;
		case HMIkey_Dir_Forward:
		case HMIkey_Dir_Reverse:
		case HMIkey_Dir_OSC:
		DirActive(key_value);
		break;
		case HMIkey_OpenPos_ClockWise:
		case HMIkey_OpenPos_AntiClockWise:
		ToolPosActive(key_value);
		break;
		case HMIkey_JTActi:
		case HMIkey_HandleActi:
		//case HMIkey_TouchActi:
		ControlTypeActive(key_value);
		break;
		
	
	
		case HMIkey_HMI_EXIT:
		HmiExitActive(key_value);
		break;
		default:
		break;
	}
}

/*
 * 函数功能：处理手柄按键；启动前检查刀具和控制方式，启停时同步控制跟随注水泵。
 * 输入参数：key_value 为手柄按键编号。
 * 返回参数：无。
 */
void HANDLEKeyBehavior(uint8_t key_value)
{
	switch(key_value)
	{
		case HANDLEKey_speed_add: 
		case HANDLEKey_speed_sub: 
		case HANDLEKey_greaI:
		//速度一档
		case HANDLEKey_greaII:
		//速度二挡
		case HANDLEKey_greaIII:
		//速度三档
		case HANDLEKey_greaIV:
		//速度四档
		case HANDLEKey_greaV:
		SpeedActive(key_value);
		break;
		case HANDLEKey_motor_start: 
		if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)return; /* 手柄队列启动电机前检查公共接头 EPC 刀具头，缺失时只报警不运行。 */
		/* 即使启动消息不是由实体按键扫描产生，也要确认当前允许手柄控制。 */
		if(ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE) == false)return;
		WorkMessage.runflag_work = true;
		ControlSignalMessage.handle_control_flag = true;
		Pubinterface_SetHandleInjectionPumpRun(true); /* 手柄队列启动电机时按泵类型和当前通道同步启动 A/B 注水冷却泵，保证绕过实体键直发消息时也保持联动。 */
		break;
		case HANDLEKey_motor_stop: 
		WorkMessage.runflag_work = false;
		ControlSignalMessage.handle_control_flag = false;
		Pubinterface_SetHandleInjectionPumpRun(false); /* 手柄队列停止电机时同步关闭本次跟随的 A/B 注水冷却泵，避免手柄停转后冷却泵继续运行。 */
		/* 手柄停止消息完成后，如果没有其它本地输出，就释放手柄控制权。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE);
		break;
		case HANDLEKey_dir_Forward:
		//设置正转。
		case HANDLEKey_dir_Reverse: 
		//设置反转
		case HANDLEKey_dir_OSC: 
		//设置往复
		DirActive(key_value);
		break;
		
		default:
		break;
	}
}

/*
 * 函数功能：分发新串口屏按键事件到对应业务处理入口。
 * 输入参数：key_value 为 ScreenKey_Scan() 解析后的 SCREENKey_* 逻辑键值。
 * 返回参数：无。
 */
void SCREENKeyBehanior(uint8_t key_value)
{
	switch(key_value)
	{
		case SCREENKey_APUMP_Add: 
		case SCREENKey_APUMP_Sub: 
		case SCREENKey_BPUMP_Add: 
		case SCREENKey_BPUMP_Sub: 
		case SCREENKey_APUMP_control:
		case SCREENKey_BPUMP_control: 
		PUMPActive(key_value);
		break;
		case SCREENKey_SPEED_Add: 
		case SCREENKey_SPEED_Sub: 
		case SCREENKey_SPEED_Sub_Large:
		case SCREENKey_SPEED_Sub_Small:
		case SCREENKey_SPEED_Add_Small:
		case SCREENKey_SPEED_Add_Large:
		SpeedActive(key_value);
		break;
		case SCREENKey_FREQ_Add: 
		case SCREENKey_FREQ_Sub: 
		FreqActive(key_value);
		break;
		case SCREENKey_HANDLE_A: 
		case SCREENKey_HANDLE_B: 
		HandleSwitchActive(key_value);
		break;
		// case SCREENKey_PLUG_A:
		// case SCREENKey_PLUG_B:
                // case SCREENKey_UNPLUG_A:
                // case SCREENKey_UNPLUG_B:
                // PlugORunPLUGActive(key_value);
                case SCREENKey_PlanerH:
		case SCREENKey_GrindH: 
		PlanerGridH(key_value);
		break;
		case SCREENKey_AutoIdentify:
		AutoIdentifyActive(key_value);
		break;
		case SCREENKey_Dir_Forward: 
		case SCREENKey_Dir_Reverse: 
		case SCREENKey_Dir_OSC: 
		DirActive(key_value);
		break;
		case SCREENKey_OpenPos_ClockWise: 
		case SCREENKey_OpenPos_AntiClockWise: 
		ToolPosActive(key_value);
		break;
		case SCREENKey_JTActi:
		case SCREENKey_HandleActi:
		case SCREENKey_TouchActi:
		case SCREENKey_TouchKeepAlive:
		case SCREENKey_TouchEXIT:
		ControlTypeActive(key_value);
		break;
		case SCREENKey_HMI_EXIT: 
		HmiExitActive(key_value);
		break;
		default:
		break;

	}
}

void PlugunPLUGActive(uint8_t key_value)
{
	switch(key_value)
	{
		case SCREENKey_PLUG_A:
		case SCREENKey_PLUG_B:
		case SCREENKey_UNPLUG_A:
		case SCREENKey_UNPLUG_B:
		PlugORunPLUGActive(key_value);
		break;
	}
}
/*
 * 函数功能：依次取出按键和插拔消息，检查能否执行后交给对应处理函数。
 * 输入参数：无。
 * 返回参数：无。
 */
void KeyBehaviors()
{
	/* 任务可能先于队列有效消息运行，句柄为空时本周期不做任何按键分发。 */
	if(KeyBehivQueue == NULL)
	return;
	KeyBehMessage_t msg={0};
	/* 每 30ms 调度一次时尽量清空队列，避免被阻塞的本地控制键长期压住后面的插拔/RFID 刷新事件。 */
	while(Kernel_QueueReceive(KeyBehivQueue, &msg, 0) == pdTRUE)
	{
		uint8_t control_type=msg.control_type; /* 记录消息来自脚踏、手柄、屏幕还是插拔，后面据此检查和处理。 */
		uint8_t control_key=msg.control_key; /* 当前队列项的业务按键值，保持与来源类型一一对应。 */
		uint32_t queued_age_ms=(uint32_t)(HAL_GetTick()-msg.queued_tick_ms); /* 按这条触控按住消息的入队时间判断是否过期，不能用后到的新消息时间代替。 */
		if((control_type==SCREENKey) &&
		   (control_key==SCREENKey_TouchKeepAlive) &&
		   ((queued_age_ms>=SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_MS) ||
		    (ScreenKey_IsAcceptedTouchKeepAliveFresh()==0U)))
		{
			continue; /* 按住信号已过期，或触控已因报警、退出而取消时，丢弃旧信号，不能启动电机。 */
		}
		/* 其他控制方式正在使用设备时，跳过不允许的本地按键，但仍继续处理后面的插拔和 RFID 刷新。 */
		if(ControlArbitration_ShouldBlockLocalKey(control_type, control_key))
		{
			continue;
		}
		switch(control_type)
		{
			case JTKey:
				JTKeyBehavior(control_key);
			break;
			case HANDLEKey:
				HANDLEKeyBehavior(control_key);
			break;
			case HMIkey:
				HMIkeyBehanior(control_key);
			break;
			case SCREENKey:
			   SCREENKeyBehanior(control_key);
			break;
			case PLUGunPLUG:
			 PlugunPLUGActive(control_key);
			break;
			default:
			break;
		}
	}
}
void KeyBehaviorsTask(uint32_t event)
{
	(void)event;
	
	KeyBehaviors();

}
void SscKeyBehaviorTask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
  	KeyBehivQueue_Init();
	Kernel_TaskCreate(&KeyBehaviorsHandle, KeyBehaviorsTask);
	Kernel_TaskStart(&KeyBehaviorsHandle, KERNEL_TASK_ALWAYS, 30);
}
