//handlekey.c

#include "bsp_gpio.h"
#include "handlekey.h"
#include "data.h"
#include "screen.h"
//#include "adc.h"
#include "soft_uart.h"

#include "kernel_scheduler.h"
#include "datahand.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscKEYBH.h"
kernel_task_t HANDLEKEYTaskHandle;

//键值按下状态
static bool sHandleKEYValue[2] = { false };

//============================================================================
// 函数名称: HAL_GPIO_EXTI_Callback()
// 功能描述: 手柄（PXBA）按键中断回调函数
// 输　  入:
// 输    出:
// 函数说明: false抬起，true按下
//============================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SimUart_HandleExti(GPIO_Pin);

	switch (GPIO_Pin)
	{
		case BOARD_RES_HANDLE_KEY1_PIN : sHandleKEYValue[1] = (sHandleKEYValue[1] ? false : true); break; //H_KEY1
		case BOARD_RES_HANDLE_KEY0_PIN : sHandleKEYValue[0] = (sHandleKEYValue[0] ? false : true); break; //H_KEY
		default : break;
	}
}

//============================================================================
// 函数名称: HandleKey_GetKeyValue()
// 功能描述: 手柄（PXBA）按键状态
// 输　  入: 
// 输    出:
// 函数说明: 
//============================================================================
bool HandleKey_GetKeyValue(uint8_t keynum)
{
	return 0; //sHandleKEYValue[keynum];
}

/*
 * 手柄按键迁移到 V1.8 新接口后，不再直接写旧的运行/报警全局标志。
 * 这里保留原有按键去抖和长按窗口，只把输出改成 WorkMessage、ControlSignalMessage
 * 以及 SendKeyBehMessage()/SendAlarmMessage()，确保后续 sscKEYBH 统一分发。
 */
static void HandleKey_SetAlarm(uint8_t alarm_value)
{
	/* 手柄按键产生的普通报警统一交给 WorkAlarm_Set，同步 WorkMessage、蜂鸣和 sscUIDP 屏幕显示。 */
	WorkAlarm_Set(alarm_value);
}

static void HandleKey_ClearAlarm(uint8_t alarm_value)
{
	/* 只清当前按键模块自己关心的报警码，避免误清其它模块仍存在的故障。 */
	WorkAlarm_ClearIf(alarm_value);
}

static void HandleKey_SetMotorRun(bool enable)
{
	if (enable)
	{
		/* 手柄按键启动电机前先占用手柄控制权，若其它方式正在控制则本次按键无效。 */
		if (ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE) == false)
		{
			return;
		}
	}
	ControlSignalMessage.handle_control_flag = enable;
	WorkMessage.runflag_work = enable;
	if (enable == false)
	{
		/* 手柄停止后释放手柄控制权，允许脚踏、屏幕或上位机重新申请。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE);
	}
	SendKeyBehMessage(HANDLEKey, enable ? HANDLEKey_motor_start : HANDLEKey_motor_stop);
}


//
void HandleKey_Scan0SSC()
{
	static uint8_t KEY0_ADC_time = 0;
  static uint8_t key0_up_down = 0;//按键按松开标志
	static uint8_t start_flags=0;
	static uint8_t activation_flag=0;
	
	if(WorkMessage.channel_work!=CHANNEL_A||WorkMessage.hand_model!=PXBA_ONLINES){
		return;
	}
if(WorkMessage.alarm_value==11||WorkMessage.alarm_value==12){return;}
		if(KEY0_STATUS() == 0)
			{
					key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(WorkMessage.drivetype_work==JTWORK)
						{
							//报警，请选择脚控启动
							HandleKey_SetAlarm(WORK_ALARM_FOOT_SELECTED);
							HandleKey_SetMotorRun(false);
							return;
						}
						
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
								if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_COMM_ERROR)
								{
									activation_flag=0;
									HandleKey_SetMotorRun(false);
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										HandleKey_SetMotorRun(true);
									}
									else
									{
										activation_flag=0;
										HandleKey_SetMotorRun(false);
									}
								} 
					}
					}
			}
			else
			{
				KEY0_ADC_time=0;
				key0_up_down++;
				if(key0_up_down<10)return;
				start_flags=1;
				
				if(key0_up_down>55)
				{
					if(WorkMessage.alarm_value==WORK_ALARM_FOOT_SELECTED)
					{
						HandleKey_ClearAlarm(WORK_ALARM_FOOT_SELECTED);
						key0_up_down=0;
					}
					else if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_COMM_ERROR)
					{
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
						HandleKey_ClearAlarm(WORK_ALARM_MOTOR_COMM_ERROR);
						key0_up_down=0;
						}
					}
					else
					{
						key0_up_down=0;
					}
				}
				
			}
	}
void HandleKey_Scan1SSC()
{
	static uint8_t KEY0_ADC_time = 0;
  static uint8_t key0_up_down = 0;//按键按松开标志
	static uint8_t start_flags=0;
	static uint8_t activation_flag=0;
	if(WorkMessage.channel_work!=CHANNEL_B||WorkMessage.hand_model!=PXBA_ONLINES){
		return;}
			if(WorkMessage.alarm_value==11||WorkMessage.alarm_value==12){return;}
			if(KEY1_STATUS() == 0)
			{
				key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(WorkMessage.drivetype_work==JTWORK)
						{
							//报警，请选择脚控启动
							HandleKey_SetAlarm(WORK_ALARM_FOOT_SELECTED);
							HandleKey_SetMotorRun(false);
					
							return;
						}
				
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
								if(WorkMessage.alarm_value==WORK_ALARM_HALL_ERROR)
								{
									activation_flag=0;
									HandleKey_SetMotorRun(false);
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										HandleKey_SetMotorRun(true);
									}
									else
									{
										activation_flag=0;
										HandleKey_SetMotorRun(false);
									}
						}
					}
					}
			}
			else
			{
				KEY0_ADC_time=0;
				key0_up_down++;
				if(key0_up_down<10)return;
				start_flags=1;
				
				if(key0_up_down>55)
				{
					if(WorkMessage.alarm_value==WORK_ALARM_FOOT_SELECTED)
					{
						HandleKey_ClearAlarm(WORK_ALARM_FOOT_SELECTED);
						key0_up_down=0;
						activation_flag=0;
					}
					else if(WorkMessage.alarm_value==WORK_ALARM_HALL_ERROR)
					{
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
						HandleKey_ClearAlarm(WORK_ALARM_HALL_ERROR);
						key0_up_down=0;
							activation_flag=0;
						}
					}
					else
					{
						key0_up_down=0;
					}
				}
			}
}

//============================================================================
// 函数名称: HandleKey0_Scan()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 手柄（PXBA）按键扫描
//============================================================================
void HandleKey_Scan0(void)
{
  static uint8_t KEY0_ADC_time = 0;
  static uint8_t key0_up_down = 0;//按键按松开标志
	
//  if ((SysRunData.MotorNum != MotorNum2) || (SysInterface.InterfaceSwitchNo2 != 1))
//		return ;

  if (KEY0_STATUS() == 0) 
	{
		KEY0_ADC_time++;
		if (KEY0_ADC_time > 5)   //7 * 11	
		{
      KEY0_ADC_time = 0;
			if (key0_up_down == 0)		  
			{
				key0_up_down = 1;	
			}	
			if (SysRunData.StartingMethod == FootCtrl)
			{
				SysRunData.HandleKeyValue[0] = 0;
				if (SysFootPedalData.FootPedalADValue <= (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset))
				{
					WorkAlarm_Set(WORK_ALARM_FOOT_SELECTED);
				}
			}      
		}
	}
	else
  {
		KEY0_ADC_time = 0;
		if (key0_up_down == 1)
		{
			key0_up_down = 0;
			if (SysRunData.StartingMethod == ManualCtrl)  //Motor_Number
			{
				if (SysRunData.HandleKeyValue[0] == 0)
				{
					SysRunData.HandleKeyValue[0] = 1;
				}
				else
				{
					SysRunData.HandleKeyValue[0] = 0;
				}
			}

		}		
	}
}

//============================================================================
// 函数名称: HandleKey1_Scan()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 手柄（PXBA）按键扫描
//============================================================================
void HandleKey_Scan1(void)
{

 
  static uint8_t KEY1_ADC_time = 0;
  static uint8_t key1_up_down = 0;//按键按松开标志
	
//  if ((SysRunData.MotorNum != MotorNum2) || (SysInterface.InterfaceSwitchNo2 != 2))
//		return ;
  
  if (KEY1_STATUS() == 0) 
	{
		KEY1_ADC_time++;
		if (KEY1_ADC_time > 5)   //7 * 11	
		{
      KEY1_ADC_time = 0;
			if (key1_up_down == 0)		  
			{
				key1_up_down = 1;	
			}		
    	if (SysRunData.StartingMethod == FootCtrl)
			{
				SysRunData.HandleKeyValue[1] = 0;
				if (SysFootPedalData.FootPedalADValue <= (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset))
				{
					WorkAlarm_Set(WORK_ALARM_FOOT_SELECTED);
				}
			}				
		}
	}
	else
	{
		KEY1_ADC_time = 0;
		if (key1_up_down == 1)
		{
			key1_up_down = 0;
			if (SysRunData.StartingMethod == ManualCtrl)  //Motor_Number
			{
				if (SysRunData.HandleKeyValue[1] == 0)
				{
					SysRunData.HandleKeyValue[1] = 1;
					 
				}
				else
				{
					SysRunData.HandleKeyValue[1] = 0;
				}
			}		
		}			
	}
 
}

/* USER CODE BEGIN Header_HANDLEKEYTaskFunc */
/**
* @brief Function implementing the HANDLEKEYTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HANDLEKEYTaskFunc */
void HANDLEKEYTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN HANDLEKEYTaskFunc */
  /* Infinite loop */
	if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))
		return;
  HandleKey_Scan0SSC();
	//HandleKey_Scan0SSC();
	HandleKey_Scan1SSC();
	//HandleKey_Scan1SSC();
  /* USER CODE END HANDLEKEYTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 15
 */
//============================================================================
void HandleKeyScan_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	Kernel_TaskCreate(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc);
	Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
