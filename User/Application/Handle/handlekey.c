//handlekey.c

#include "stm32f4xx_hal.h"
#include "handlekey.h"
#include "data.h"
#include "screen.h"
//#include "adc.h"

#include "app_task.h"
#include "datahand.h"
task_t HANDLEKEYTaskHandle;

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
	switch (GPIO_Pin)
	{
		case GPIO_PIN_11 : sHandleKEYValue[1] = (sHandleKEYValue[1] ? false : true); break; //H_KEY1
		case GPIO_PIN_10 : sHandleKEYValue[0] = (sHandleKEYValue[0] ? false : true); break; //H_KEY
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


//
void HandleKey_Scan0SSC()
{
	static uint8_t KEY0_ADC_time = 0;
  static uint8_t key0_up_down = 0;//按键按松开标志
	static uint8_t start_flags=0;
	static uint8_t activation_flag=0;
	
	if(Workvalue_s.select_channel!=1||Workvalue_s.hand_model!=PXBA_ONLINE){
		return;
	}
if(Workvalue_s.Alarm_value==11||Workvalue_s.Alarm_value==12){return;}
		if(KEY0_STATUS() == 0)
			{
					key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(Workvalue_s.set_Way==footcontrol)
						{
							//报警，请选择脚控启动
							Workvalue_s.Alarm_value=6;//脚踏值错误
							Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.Handle_MOTORWorking_flag=0;
							Workvalue_s.MOTORWorking_flag=0;
							return;
						}
						
						if(Workvalue_s.set_Way==handelcontrol)
						{
								if(Workvalue_s.Alarm_value==8)
								{
									activation_flag=0;
									Workvalue_s.Handle_MOTORWorking_flag=0;
									Workvalue_s.MOTORWorking_flag=0;
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										Workvalue_s.Handle_MOTORWorking_flag=1;
										Workvalue_s.MOTORWorking_flag=1;
									}
									else
									{
										activation_flag=0;
										Workvalue_s.Handle_MOTORWorking_flag=0;
										Workvalue_s.MOTORWorking_flag=0;
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
					if(Workvalue_s.Alarm_value==6)
					{
							Workvalue_s.Alarm_value=0;
						Workvalue_s.beep_Alarm_flag=0;//报警
						key0_up_down=0;
					}
					else if(Workvalue_s.Alarm_value==8)
					{
						if(Workvalue_s.set_Way==handelcontrol)
						{
								Workvalue_s.Alarm_value=0;
						Workvalue_s.beep_Alarm_flag=0;//报警
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
	if(Workvalue_s.select_channel!=2||Workvalue_s.hand_model!=PXBA_ONLINE){
		return;}
			if(Workvalue_s.Alarm_value==11||Workvalue_s.Alarm_value==12){return;}
			if(KEY1_STATUS() == 0)
			{
				key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(Workvalue_s.set_Way==footcontrol)
						{
							//报警，请选择脚控启动
							Workvalue_s.Alarm_value=7;//脚踏值错误
							Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.Handle_MOTORWorking_flag=0;
							Workvalue_s.MOTORWorking_flag=0;
					
							return;
						}
				
						if(Workvalue_s.set_Way==handelcontrol)
						{
								if(Workvalue_s.Alarm_value==9)
								{
									activation_flag=0;
									Workvalue_s.Handle_MOTORWorking_flag=0;
									Workvalue_s.MOTORWorking_flag=0;
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										Workvalue_s.Handle_MOTORWorking_flag=1;
										Workvalue_s.MOTORWorking_flag=1;
									}
									else
									{
										activation_flag=0;
										Workvalue_s.Handle_MOTORWorking_flag=0;
										Workvalue_s.MOTORWorking_flag=0;
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
					if(Workvalue_s.Alarm_value==7)
					{
							Workvalue_s.Alarm_value=0;
						Workvalue_s.beep_Alarm_flag=0;//报警
						key0_up_down=0;
						activation_flag=0;
					}
					else if(Workvalue_s.Alarm_value==9)
					{
						if(Workvalue_s.set_Way==handelcontrol)
						{
								Workvalue_s.Alarm_value=0;
						Workvalue_s.beep_Alarm_flag=0;//报警
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
					SysRunData.WarnID = 12;
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
					SysRunData.WarnID = 12;
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
	if(Workvalue_s.HMI_Control_flag)
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
	app_task_create(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc);
	app_task_start(&HANDLEKEYTaskHandle, APP_TASK_ALWAYS, 30);
}




