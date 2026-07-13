#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "Pubinterface.h"
#include "sscKEYBH.h"

  kernel_task_t    KeyBehaviorsHandle;
QueueHandle_t KeyBehivQueue = NULL;

#define KEYBEH_PLUG_EVENT_WAIT_TICKS pdMS_TO_TICKS(60U) /* 手柄插拔和 RFID 二次刷新是状态同步事件，队列满时短等待，避免上位机/屏幕丢在线刷新。 */

typedef struct
{
	uint8_t control_type;//脚踏key,显示屏key,外部通讯key，手控key
	uint8_t control_key;
}KeyBehMessage_t;
static void KeyBehivQueue_Init(void)
{
    KeyBehivQueue = Kernel_QueueCreate(20, sizeof(KeyBehMessage_t), "KeyBehivQueue");
}
void SendKeyBehMessage(uint8_t control_type,uint8_t control_key)
{
	// static uint8_t control_types=0;
	// static uint8_t control_keys=0;
	TickType_t wait_ticks = 0U; /* 普通按键仍保持非阻塞，避免脚踏/屏幕高频按键拖慢控制任务。 */
	// if(control_types==control_type&&control_keys==control_key)return;
	// else
	// {
	// 	control_types=control_type;
	// 	control_keys=control_key;
	// }
	if(KeyBehivQueue == NULL) return;
	KeyBehMessage_t msg;
	msg.control_type =control_type ;
	msg.control_key = control_key;
	if (control_type == PLUGunPLUG)
	{
		wait_ticks = KEYBEH_PLUG_EVENT_WAIT_TICKS; /* 插拔/RFID 刷新必须尽量入队，否则 MemoryMsg 和心跳会停在旧状态。 */
	}
	(void)Kernel_QueueSend(KeyBehivQueue, &msg, wait_ticks);
}
void JTKeyBehavior(uint8_t key_value)
{
	switch (key_value)
	{
	  case JTkey_left_short:
	  case JTKey_left_long:
	  case JTKey_right_short:
	  case JTKey_right_long:
	  case JTKey_Gently_left_start://轻排开始左测踏板
	  case JTKey_Gently_left_stop://轻轻排空停止左测踏板
	  case JTKey_Gently_right_start://轻排开始左测踏板
	  case JTKey_Gently_rigth_stop://轻轻排空停止右测踏板
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
		case HMIkey_Gently_start://轻轻排空开始
		case HMIkey_Gently_stop://轻轻排空停止
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
		/* 队列层再次确认手柄控制权，防止绕过手柄扫描任务直接投递启动消息。 */
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
		//设置正传
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
 * 函数功能：从统一按键队列中取出并分发所有待处理事件。
 * 输入参数：无。
 * 返回参数：无。
 */
void KeyBehaviors()
{
	if(KeyBehivQueue == NULL)
	return;
	KeyBehMessage_t msg={0};
	/* 每 30ms 调度一次时尽量清空队列，避免被阻塞的本地控制键长期压住后面的插拔/RFID 刷新事件。 */
	while(Kernel_QueueReceive(KeyBehivQueue, &msg, 0) == pdTRUE)
	{
		uint8_t control_type=msg.control_type; /* 当前队列项的来源类型，用于仲裁和后续分发。 */
		uint8_t control_key=msg.control_key; /* 当前队列项的业务按键值，保持与来源类型一一对应。 */
		/* 任一控制方式被其它来源持有时，只丢弃当前本地控制键，继续处理后续插拔/RFID 状态事件。 */
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
