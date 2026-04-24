#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "Pubinterface.h"
#include "sscKEYBH.h"

  kernel_task_t    KeyBehaviorsHandle;
QueueHandle_t KeyBehivQueue = NULL;

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
	if(KeyBehivQueue == NULL) return;
	KeyBehMessage_t msg;
	msg.control_type =control_type ;
	msg.control_key = control_key;
	(void)Kernel_QueueSend(KeyBehivQueue, &msg, 0);
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
		WorkMessage.runflag_work = true;
		ControlSignalMessage.handle_control_flag = true;
		break;
		case HANDLEKey_motor_stop: 
		WorkMessage.runflag_work = false;
		ControlSignalMessage.handle_control_flag = false;
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
		case SCREENKey_PLUG_A:
		case SCREENKey_PLUG_B:
		case SCREENKey_UNPLUG_A:
		case SCREENKey_UNPLUG_B:
		PlugORunPLUGActive(key_value);
		break;
		case SCREENKey_PlanerH: 
		case SCREENKey_GrindH: 
		PlanerGridH(key_value);
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
		case SCREENKey_TouchStart:
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
void KeyBehaviors()
{
	//判断keybehavior的队列是否为空
    uint8_t control_type=0;
	uint8_t control_key=0;
	if(KeyBehivQueue == NULL)
	return;
	KeyBehMessage_t msg={0};
   		 // 非阻塞方式接收消息
    if(Kernel_QueueReceive(KeyBehivQueue, &msg, 0) == pdTRUE)
    {
        control_type=msg.control_type;
        control_key=msg.control_key;
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
