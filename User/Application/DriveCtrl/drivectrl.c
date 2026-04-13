//drivectrl.c

#include "drivectrl.h"
#include "data.h"
#include "Motor.h"
#include "pedal.h"
#include "lcd.h"
#include "common.h"
#include "iwdg.h"
#include "pump.h"
#include "screen.h"
#include "datahand.h"

#include <stdint.h>

#include "kernel_scheduler.h"

kernel_task_t MOTORCURRENTTaskHandle;
kernel_task_t UIREFRESHDATATaskHandle;
kernel_task_t MOTOR123TaskHandle;

kernel_task_t MOTORJMITaskHandle;

//static uint8_t STMotor_Flag = 0;//ssc屏蔽
static uint8_t HandleAllowSwitchTime = 0;

//============================================================================
//泵的再次启动被允许标志
//============================================================================
void DriveCtrl_PumpFlag_B(void)
{
 // STMotor_Flag = 1;

  SysRunData.PumpMotorSetSpeed_B = 0;
  Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);
}

//============================================================================
//泵的再次启动被允许标志
//============================================================================
void DriveCtrl_PumpFlag_A(void)
{
 // STMotor_Flag = 1;

  SysRunData.PumpMotorSetSpeed_A = 0;
//  Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);
}
//============================================================================
//电机驱动板电流状态获取
//============================================================================
void DriveCtrl_Motor1CurrentTask_Fun(void)
{
  /********************驱动板电流检测**********************/
  //uint8_t GetMotorCurrentCMD[8] = {0x01, 0x03, 0x00, 0x21, 0x00, 0x01, 0xD4, 0x00};  //只读 驱动板实时电流
  /*
  Uart2_Send_Buff[3] = DB_Current_Adress;
  Get_DB_Status(Get_Motor_Current_Com);
  */

	/*
  if (SysRunData.MotorNum == MotorNum1)
    Motor_GetStatus(GetMotorCurrentCMD);
	*/
}

//============================================================================
// 53
//============================================================================
/* USER CODE BEGIN Header_MOTORCURRENTTaskFunc */
/**
* @brief Function implementing the MOTORCURRENTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTORCURRENTTaskFunc */
void MOTORCURRENTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTORCURRENTTaskFunc */
  /* Infinite loop */
  DriveCtrl_Motor1CurrentTask_Fun();
  /* USER CODE END MOTORCURRENTTaskFunc */
}

void DriveCtrl_Motor1CurrentTask_Init(void)
{
  /* definition and creation of BEEPTask */
	Kernel_TaskCreate(&MOTORCURRENTTaskHandle, MOTORCURRENTTaskFunc);
	Kernel_TaskStart(&MOTORCURRENTTaskHandle, KERNEL_TASK_ALWAYS, 30);
}

//============================================================================
//UI实时更新 当前的转速、频率、流量
//============================================================================
void DriveCtrl_UIRefreshDataTask_Fun(void)
{
  uint32_t Motor_Speed = 0;

  if (SysRunData.MotorRun != MotorWorking)  //电机停
  {
    //转速
    LCD_Show_4byte_Number(0x3420, 0);
	  //频率
//    LCD_Show_4byte_Number(0x3470, 0);
		
	  ////////////////////B/////////////////////
	  if (SysRunData.PumpONOFF_B == 2)  //泵关
	  {
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
			LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
//			LCD_Show_4byte_Number(0x3450,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
			#else
		  LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);  //设置流量
//	    LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB));  //实时流量
			#endif

	  }
	  else  //泵开
	  {
	    if (SysRunData.PumpDrain_B == 0)  //B泵排空关
	    {
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
					LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10)); 
				#else
				 LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
				#endif
//		   LCD_Show_4byte_Number(0x3450, 0);
	    }
	    else  //泵排空开
	    {
		    LCD_Show_4byte_Number(0x3550, 70);
//		    LCD_Show_4byte_Number(0x3450, Common_FolatToHex(70));
	    }
	  }
		
	  ////////////////////A/////////////////////		
	  if (SysRunData.PumpONOFF_A == 2)  //泵关
	  {
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
//			LCD_Show_4byte_Number(0x34D0,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
			LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
			#else
		  LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);  //设置流量
//	    LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA));  //实时流量
			#endif
	  }
	  else  //泵开
	  {				
	    if (SysRunData.PumpDrain_A == 0)  //A泵排空关
	    {
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
					LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10)); 
				#else
				 LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
				#endif
//		   LCD_Show_4byte_Number(0x3430, 0);
	    }
	    else  //泵排空开
	    {
		    LCD_Show_4byte_Number(0x3530, 70);
//		    LCD_Show_4byte_Number(0x3430, Common_FolatToHex(70));
	    }				
		}
		
	return ;
  }
	switch (SysRunData.MotorNum)
  {
	  case MotorNum1 :
	  {
	    if ((SysRunData.MotorRealSpeed % 100) >= 50)
		    SysRunData.MotorRealSpeed = (SysRunData.MotorRealSpeed / 100 + 1) * 100;  //清除十位数以下的不稳定的数据
	    else
		    SysRunData.MotorRealSpeed = (SysRunData.MotorRealSpeed / 100) * 100;

	    if(SysRunData.MotorRealSpeed > 12000)
		    SysRunData.MotorRealSpeed = 12000;

	    Motor_Speed = SysRunData.MotorRealSpeed * 10;

	    //转速
			if( (SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
	    LCD_Show_4byte_Number(0x3420, Motor_Speed*2);
			else
				  LCD_Show_4byte_Number(0x3420, Motor_Speed);

			if ((SysRunData.PumpONOFF_A == 1)&&( SysRunData.PumpMotorSetSpeed_A > 0 )) 		
			{
				//流量
					#ifdef  WATER_UPTAKE//如果定义吸水ssc
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA));  //流速 SysRunData.NumberFluidSet
				#endif			
			}				
			if ((SysRunData.PumpONOFF_B == 1)&&( SysRunData.PumpMotorSetSpeed_B > 0 ))
			{
				//流量
					#ifdef  WATER_UPTAKE//如果定义吸水ssc
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB));  //流速 SysRunData.NumberFluidSet
				#endif
			}
	}
 break;
	  case MotorNum2 :
	  {
	    Motor_Speed = SysRunData.MotorRealSpeed;
			if ((Motor_Speed % 10) >= 50)
		    Motor_Speed = (Motor_Speed / 10 + 1) * 100;  //清除十位数一下的不稳定的数据
	    else
		    Motor_Speed = (Motor_Speed / 10) * 100;
			//转速
				if( (SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
	    LCD_Show_4byte_Number(0x3420, Motor_Speed*2);
				else
		  LCD_Show_4byte_Number(0x3420, Motor_Speed );

	    //流量
			if ((SysRunData.PumpONOFF_A == 1)&&( SysRunData.PumpMotorSetSpeed_A > 0 )) 		
			{			
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA));  //流速 SysRunData.NumberFluidSet
				#endif
      }			
			if ((SysRunData.PumpONOFF_B == 1)&&( SysRunData.PumpMotorSetSpeed_B > 0 )) 		
			{			
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB));  //流速 SysRunData.NumberFluidSet
				#endif
      }
			
	    //频率
			LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
	  }
	  break;
	  case MotorNum3 :
	  {
	    Motor_Speed = SysRunData.MotorRealSpeed * 10;

	    //转速
				if( (SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
	    LCD_Show_4byte_Number(0x3420, Motor_Speed*2);
				else 
					 LCD_Show_4byte_Number(0x3420, Motor_Speed);

	    //流量
			if ((SysRunData.PumpONOFF_A == 1)&&( SysRunData.PumpMotorSetSpeed_A > 0 )) 		
			{	
				#ifdef  WATER_UPTAKE//如果定义吸水ssc	
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3430, Common_FolatToHex(SysRunData.FlowRateA));  //流速 SysRunData.NumberFluidSet
				#endif				
			}
			if ((SysRunData.PumpONOFF_B == 1)&&( SysRunData.PumpMotorSetSpeed_B > 0 )) 		
			{				
				#ifdef  WATER_UPTAKE//如果定义吸水ssc	
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB/10));  //流速 SysRunData.NumberFluidSet
				#else
				LCD_Show_4byte_Number(0x3450, Common_FolatToHex(SysRunData.FlowRateB));  //流速 SysRunData.NumberFluidSet
				#endif
      }
	    //频率
	    LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
	  }
	  break;
	  default : break;
  }
}


//============================================================================
//  100
//============================================================================
/* USER CODE BEGIN Header_UIREFRESHDATATaskFunc */
/**
* @brief Function implementing the UIREFRESHDATATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UIREFRESHDATATaskFunc */
void UIREFRESHDATATaskFunc(uint32_t event)
{
  /* USER CODE BEGIN UIREFRESHDATATaskFunc */
  /* Infinite loop */
  DriveCtrl_UIRefreshDataTask_Fun();
  /* USER CODE END UIREFRESHDATATaskFunc */
}

void DriveCtrl_UIRefreshDataTask_Init(void)
{
  /* definition and creation of UIREFRESHDATATask */
	Kernel_TaskCreate(&UIREFRESHDATATaskHandle, UIREFRESHDATATaskFunc);
	Kernel_TaskStart(&UIREFRESHDATATaskHandle, KERNEL_TASK_ALWAYS, 100);
}

//55Ms一次，原本
void MotorControlSSC()
{
	static uint8_t stop_channel=0;
	static uint8_t	stop_channel_value=0;
	static uint8_t un_ex=0;
	uint32_t MX_YIM_play_speed=0;
	uint32_t handle_speeds=0;
	uint8_t temp1=0;
	static uint8_t stop_counts=0;
	uint8_t zhengsuqufan=0;


//	if(Workvalue_s.Alarm_value==13)
//	{
//		
//		
//		Pump_SetSpeed_A(0);//泵停止运行
//		//Pump_SetSpeed_B(0);//泵停止运行
//		if(Workvalue_s.hand_model==PX_YIP_ONLINE||Workvalue_s.hand_model==PX_YIM_ONLINE)
//			{
//				BrushlessMotor_Stop(Workvalue_s.select_channel+2,Workvalue_s.set_Direction,Workvalue_s.set_Freq,Workvalue_s.Handle_type);
//				
//			}
//			else{
//			BrushlessMotor_Stop(Workvalue_s.select_channel,Workvalue_s.set_Direction,Workvalue_s.set_Freq,Workvalue_s.Handle_type);
//			}
//	return;
//	}
	if(Workvalue_s.select_channel==0)
	{
		if(Workvalue_s.HMI_Working_flag==1)
		{
			Workvalue_s.Alarm_value=1;
			Workvalue_s.beep_Alarm_flag=13;
		}
		Workvalue_s.MOTORWorking_flag=0;
	}
	
	if(Workvalue_s.MOTORWorking_flag)//在任何报警情况下 MOTORWorking_flag置为false,让电机线停止动作
	{
			if(Workvalue_s.hand_model==PXBA_ONLINE||Workvalue_s.hand_model==PXBB_ONLINE)Workvalue_s.Handle_type=2;
			else if(Workvalue_s.hand_model==PX_YIM_ONLINE||Workvalue_s.hand_model==PX_YIP_ONLINE)Workvalue_s.Handle_type=3;
			else Workvalue_s.Handle_type=1;
		
			if(Workvalue_s.hand_model==PX_YIP_ONLINE||Workvalue_s.hand_model==PX_YIM_ONLINE)
				{
					stop_channel=1;
				}
				else{
				stop_channel=0;
				}
				
				
		LCD_Show_2byte_Number(0x9423,0xffE0);
		if(Workvalue_s.Handle_MOTORWorking_flag==1||Workvalue_s.TouchActivation_flag==1||Workvalue_s.HMI_Control_flag==1)
			{
				handle_speeds=Workvalue_s.set_speed/10;
				temp1=APump(Workvalue_s.set_Injection);
				Pump_SetSpeed_A(temp1);//泵运行流量
				if(Workvalue_s.Injection_drain_flag)
				{
					Workvalue_s.Injection_drain_flag=0;
					injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
					//停止有两个信号来源，正常按键停止
					draindisplay(1);//按钮更新图标（白色）
				}
			}
			else
			{
				handle_speeds=Workvalue_s.MotorRealSpeed;
			}
			un_ex=1;
			stop_counts=0;
			speeddisplay(1,handle_speeds*10 );

		
		if(Workvalue_s.select_channel==1)
		{
			stop_channel_value=1;
			switch(Workvalue_s.hand_model)
			{
				case JMB_ONLINE:
						
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,330); break;  
				case TMBB_ONLINE:
					
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,330); break;  
				break;
				case MX_YIM_ONLINE:
					
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds/2 ,350); break;  //报警值要修改
				break;
				case MX_YIM16_ONLINE:
						if( Workvalue_s.set_Direction==1)
						zhengsuqufan=2;
					else if(Workvalue_s.set_Direction==2)
						zhengsuqufan=1;
					  BrushlessMotor_Run(Workvalue_s.select_channel, zhengsuqufan, 0, 0x01, handle_speeds/2.783 ,450); break;   //ok
				break;
				case MX_YIP_ONLINE:
					
				    BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,260); break;  
				break;
			break;
				case TMBA_ONLINE:
				case EMBA_ONLINE:
				case EMBB_ONLINE:
			BrushlessMotor_Run(Workvalue_s.select_channel,  Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,450); break;  
				
			break;
				case PXBA_ONLINE:
				case PXBB_ONLINE:
						if(Workvalue_s.tool_model==1)//刨模式
						{
							if(Workvalue_s.set_Direction==3){
								BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x02,handle_speeds * 4.96 ,120); 
								}
								else
								{
									BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01,handle_speeds * 4.96 ,120); 
								}
								break;
							//BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x02, handle_speeds * 4.96 ,120); break;
						}
						else
						{
							if(paoxueSpeciValue_F[8]==0x8c)
							{
								BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01, handle_speeds ,220); break;//平速
							}
							else if(paoxueSpeciValue_F[8]==0x1c)
							{
								if( Workvalue_s.set_Direction==1)
									zhengsuqufan=2;
								else if(Workvalue_s.set_Direction==2)
									zhengsuqufan=1;
								BrushlessMotor_Run(Workvalue_s.select_channel, zhengsuqufan, Workvalue_s.set_Freq, 0x01, handle_speeds/2 ,360); break;//两倍增速，
							}
							else
							{
								BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01, handle_speeds * 1.96 ,280); break;
							}
						}
					break;
				case PX_YIM_ONLINE://一体磨
					if(handle_speeds<=300)
					{
						BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 2.15 ,66); break;
					}
					else
					{
						BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 2.0 ,66); break;
					}
					break;
				case PX_YIP_ONLINE://一体刨
					
				if(handle_speeds<=100)
					{
						if(Workvalue_s.set_Direction==2)//正向
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,32); break;
						}
						else if(Workvalue_s.set_Direction==1)
						{
								BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,32); break;
						}
						else if(Workvalue_s.set_Direction==3)
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,40); break;
						}
				}
				else
				{
					if(Workvalue_s.set_Direction==2)//正向
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds* 4.5 ,28); break;
						}
						else if(Workvalue_s.set_Direction==1)
						{
								BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,28); break;
						}
						else if(Workvalue_s.set_Direction==3)
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,40); break;
						}
				}
				break;
				
			}
		}
		else if(Workvalue_s.select_channel==2)
		{
			stop_channel_value=2;
			switch(Workvalue_s.hand_model)
			{
				case JMB_ONLINE:
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,330); break;  
				case TMBB_ONLINE:
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01,handle_speeds ,330); break;  
				break;
				case MX_YIM_ONLINE:
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds/2,350); break;  
				break;
				case MX_YIM16_ONLINE:
						if( Workvalue_s.set_Direction==1)
									zhengsuqufan=2;
								else if(Workvalue_s.set_Direction==2)
									zhengsuqufan=1;
						BrushlessMotor_Run(Workvalue_s.select_channel, zhengsuqufan, 0, 0x01, handle_speeds/2.783 ,450); break;  
				
				case MX_YIP_ONLINE:
				BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, 0, 0x01, handle_speeds,270); break;  
				break;
			break;
				case TMBA_ONLINE:
				case EMBA_ONLINE:
				case EMBB_ONLINE:
					if(handle_speeds>3000)
					BrushlessMotor_Run(Workvalue_s.select_channel,  Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,450);
					else
					BrushlessMotor_Run(Workvalue_s.select_channel,  Workvalue_s.set_Direction, 0, 0x01, handle_speeds ,470); break;
			break;
			case PXBA_ONLINE:
			case PXBB_ONLINE:
			
			if(Workvalue_s.tool_model==1)//刨 模 式
				{
					if(Workvalue_s.set_Direction==3){
					BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x02,handle_speeds * 4.96 ,120); 
					}
					else
					{
						BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01,handle_speeds * 4.96 ,120); 
					}
					break;
				}
				else
				{
					if(paoxueSpeciValue_F[8]==0x8c)
					{
							BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01,handle_speeds ,220); break;
					}
					else if(paoxueSpeciValue_F[8]==0x1c)
					{
							if( Workvalue_s.set_Direction==1)
									zhengsuqufan=2;
								else if(Workvalue_s.set_Direction==2)
									zhengsuqufan=1;
							BrushlessMotor_Run(Workvalue_s.select_channel,zhengsuqufan, Workvalue_s.set_Freq, 0x01,handle_speeds/2 ,360); break;
					}
					else
					{
							BrushlessMotor_Run(Workvalue_s.select_channel, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x01,handle_speeds * 1.96 ,280); break;
					}
				}
			  break;
				case PX_YIM_ONLINE://一体磨
					if(handle_speeds<=300)
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 2.15 ,66); break;
						}
						else
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 2.0 ,66); break;
						}
						
				case PX_YIP_ONLINE://一体刨
					
				if(handle_speeds<=100)
					{
						if(Workvalue_s.set_Direction==2)//正向
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,32); break;
						}
						else if(Workvalue_s.set_Direction==1)
						{
								BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03,handle_speeds * 4.5 ,32); break;
						}
						else if(Workvalue_s.set_Direction==3)
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,40); break;
						}
				}
				else
				{
					if(Workvalue_s.set_Direction==2)//正向
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,28); break;
						}
						else if(Workvalue_s.set_Direction==1)
						{
								BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03, handle_speeds * 4.5 ,28); break;
						}
						else if(Workvalue_s.set_Direction==3)
						{
							BrushlessMotor_Run(Workvalue_s.select_channel+2, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x03,handle_speeds * 4.5 ,40); break;
						}
				}
				break;
			}
		}
		
	}
	else
	{
		if(un_ex==1){
				LCD_Show_2byte_Number(0x9423,0xffFF);
		//停止电机运行
				speeddisplay(1,Workvalue_s.set_speed);
			
				Pump_SetSpeed_A(0);//泵停止运行
				un_ex=0;
		}
		BrushlessMotor_Stop(stop_channel_value+2,Workvalue_s.set_Direction,Workvalue_s.set_Freq,Workvalue_s.Handle_type);
		//stop_counts++;
//		if(stop_counts<5)
//			{
//				if(stop_channel)
//				{
//					BrushlessMotor_Stop(stop_channel_value+2,Workvalue_s.set_Direction,Workvalue_s.set_Freq,Workvalue_s.Handle_type);//AA 01 00 01 00 00 02 00 00  BB AA
//				}
//				else{
//					//	BrushlessMotor_Run(stop_channel_value, Workvalue_s.set_Direction, Workvalue_s.set_Freq, 0x02,0 ,140);
//					BrushlessMotor_Stop(stop_channel_value,Workvalue_s.set_Direction,Workvalue_s.set_Freq,Workvalue_s.Handle_type);
//				}
//			}
				
		 }
}


//============================================================================
// 函数名称: Drive_Motor1Ctrl_Module()
// 功能描述: 驱动板1电机控制模块
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void DriveCtrl_Motor1Ctrl_Module(void)
{  
	uint8_t index = SysInterface.BeSelectNum - 1;
	
	if (SysModelConfig.HandlePortA == 1)
	{
		if (SysInterface.HandleType[1] == Handle_Type_3)//EC16
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,280); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,280); break;  //反向
				default : break;
			} 	
		}
		else if(SysInterface.HandleType[1] == Handle_Type_2)//ssc 一体磨 电流报警值待修改 
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //反向
				default : break;
			} 	
		}
		else if(SysInterface.HandleType[1] == Handle_Type_4)// ssc 一体刨  电流报警值待修改
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //反向
				default : break;
			} 	
		}
		else //EC13
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,400); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,400); break;  //反向
				default : break;
			} 	
		}	
	}
	if (SysModelConfig.HandlePortB ==  1) 
	{
		if (SysInterface.HandleType[4] == Handle_Type_3) //EC16
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,350); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,350); break;  //反向
				default : break;
			} 	
		}
		else if(SysInterface.HandleType[4] == Handle_Type_2)//ssc 一体磨 电流报警值待修改 
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //反向
				default : break;
			} 	
		}
		else if(SysInterface.HandleType[4] == Handle_Type_4)// ssc 一体刨  电流报警值待修改
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,270); break;  //反向
				default : break;
			} 	
		}
		else //EC13
		{
			if(SysRunData.MotorRealSpeed >= 3000)
			{
				switch (SysSetParam[index].MotorModel)
				{
					case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,450); break;  //正向
					case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,450); break;  //反向
					default : break;
				} 			
			}
			else
      {
				switch (SysSetParam[index].MotorModel)
				{
					case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 2, 0, 0x01, SysRunData.MotorRealSpeed ,470); break;  //正向
					case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01, SysRunData.MotorRealSpeed ,470); break;  //反向
					default : break;
				} 			
			}
		}	
	}		

}

//============================================================================
// 函数名称: Drive_Motor2Ctrl_Module()
// 功能描述: 驱动板2电机控制模块
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void DriveCtrl_Motor2Ctrl_Module(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;
	if (SysModelConfig.HandlePortA ==  1)
	{
		if (SysSetParam[index].DJSetPDMT == 0)  //刨
		{

			switch (SysSetParam[index].MotorModel)
			{
				
				case 1 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 3, SysRunData.Frequency, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //往复
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 2, 0, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 1, 0, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //反向
				default : break;
			}		
			
		}
		else  //磨
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 2, 0, 0x02,  SysRunData.MotorRealSpeed * 1.96 ,350); break;   //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 1, 0, 0x02,  SysRunData.MotorRealSpeed * 1.96 ,350); break;   //反向
				default : break;
			}
		}		
	}

	if (SysModelConfig.HandlePortB ==  1)
	{
		if (SysSetParam[index].DJSetPDMT == 0)  //刨
		{

			switch (SysSetParam[index].MotorModel)
			{
				case 1 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 3, SysRunData.Frequency, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //往复
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 2, 0, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 1, 0, 0x02, SysRunData.MotorRealSpeed * 4.96 ,130); break;  //反向
				default : break;
			}		
			
		}
		else  //磨
		{
			switch (SysSetParam[index].MotorModel)
			{
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 2, 0, 0x02,  SysRunData.MotorRealSpeed * 1.96 ,300); break;   //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo2, 1, 0, 0x02,  SysRunData.MotorRealSpeed * 1.96 ,300); break;   //反向
				default : break;
			}
		}		
	}	
	
}

//============================================================================
// 函数名称: Drive_Motor3Ctrl_Module()
// 功能描述: 驱动板3电机控制模块

// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void DriveCtrl_Motor3Ctrl_Module(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if (SysHandleData.Ds2431BuffBB[SysInterface.InterfaceSwitchNo3 - 1][4] == 1)  //刨刀
  {

			if (SysRunData.MotorRealSpeed <= 100)
			{
				switch (SysSetParam[index].MotorModel)
				{				
				case 1 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 3, SysRunData.Frequency, 3, SysRunData.MotorRealSpeed * 4.5 ,40); break;   //往复
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 2, 0, 3, SysRunData.MotorRealSpeed * 4.5 ,36); break;   //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 1, 0, 3, SysRunData.MotorRealSpeed * 4.5 ,28); break;   //反向
				default : break;			
				}			
			}
			else
      {
				switch (SysSetParam[index].MotorModel)
				{				
				case 1 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 3, SysRunData.Frequency, 3, SysRunData.MotorRealSpeed * 4.3 ,40); break;   //往复
				case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 2, 0, 3, SysRunData.MotorRealSpeed * 4.3 ,36); break;   //正向
				case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 1, 0, 3, SysRunData.MotorRealSpeed * 4.3 ,28); break;   //反向
				default : break;			
				}
		  }				
  }
  else  //磨 头
  {
		switch (SysSetParam[index].MotorModel)
		{
			case 2 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 2, 0, 3, SysRunData.MotorRealSpeed * 2.0 ,80); break;   //正向
			case 3 : BrushlessMotor_Run(SysInterface.InterfaceSwitchNo3+2, 1, 0, 3, SysRunData.MotorRealSpeed * 2.0 ,80); break;   //反向
			default : break;
		}				 
  }
}

//============================================================================
// 函数名称: Drive_Motor123CtrlStop_Module()
// 功能描述: 驱动板1、2、3电机停控制模块
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void Drive_Motor123CtrlStop_Module(void)
{
	static bool step1 = true, step2 = true, step3 = true;

  SysRunData.MotorRun = MotorStop;
  
  SysRunData.MotorRealSpeed = 0;

  if (SysRunData.PumpSteping5sNum_B == 0 && SysRunData.PumpDrain_B == 0)
	{DriveCtrl_PumpFlag_B();}//B泵停止
  if (SysRunData.PumpSteping5sNum_A == 0 && SysRunData.PumpDrain_A == 0)
	{DriveCtrl_PumpFlag_A();}  //A泵停止

  switch (SysRunData.MotorNum)
  {
	  case MotorNum1 :
	  {
			//电机1   //电机2
       BrushlessMotor_Stop((step1 ? 1 : 2), 1, 0, 0x01);  

			step1 = step1 ? false : true;
	  }
	  break;
	  case MotorNum2 :
	  {
	    if (++SysRunData.Motor2StopTime >= 3)  //600ms 设置方向后  640ms  540ms
	    {
	      SysRunData.Motor2StopTime = 5;

			  //电机1   //电机2
         BrushlessMotor_Stop((step2 ? 1 : 2), 1, 0, 0x02);  

			  step2 = step2 ? false : true;
	    }
	  }
	  break;
	  case MotorNum3 :
	  {
			//电机1  //电机2
			BrushlessMotor_Stop((step3 ? 3 : 4), 1, 0, 3);

			step3 = step3 ? false : true;
	  }
	  break;
	  default : Motor_ErrorEmergencyStop_Ctrl(); break;
  }

  if (++HandleAllowSwitchTime >= 3)  //400ms -> 480ms -> 540ms
    SysRunData.HandleAllowSwitchFlag = 0;
}

//============================================================================
// 函数名称: Drive_Motor123Ctrl_Fun()
// 功能描述: 驱动板1、2、3电机控制
// 输　  入:
// 输    出:
// 函数说明: 电机速度控制
//============================================================================
void DriveCtrl_Motor123Task_Fun(void)    //Set_Drive_Board
{
  static uint8_t NOMotor1TimeCnt = 0;
  static uint8_t StopSendFlag = 1;	
  static uint8_t StopSendTimeCnt = 0;
	
  if (SysRunData.MotorNum != MotorNum1)
  {
		
	  if (++NOMotor1TimeCnt < 3)  //165ms
	    return ;
  }
  NOMotor1TimeCnt = 0;
  //电机速度控制
  switch (SysRunData.StartingMethod)  //脚控 1、2、3号电机
  {
	  case FootCtrl :
	  {
	    if ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) && \
		      (SysFootPedalData.FootPedalConnectOkNo == Connect) && (Data_GetOnLineCnt() >  0) && \
		      (SysRunData.PumpDrain_B == 0) && (SysRunData.PumpDrain_A == 0) && (SysRunData.StuckFlag == No_Error) && (SysFootPedalData.FootPedalReadFlag == No_Error) && \
		      (SysRunData.StuckFlag3 == No_Error) && (SysFootPedalData.FootPedalLiftFlag == Lift) && (SysRunData.CommunicatFlag == Connect) && \
		      ((SysRunData.MotorNum == MotorNum1) || (SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3)))
	      {

 
				if (SysFootPedalData.FootPedalType == 1)
				{
						if (SysModelConfig.HandlePortA == 1)
						{
							if (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryMValue_Left + FootPedalValueOffset))
							{
								if (SysFootPedalData.FootPedalADValue > SysFootPedalData.FootPedalMemoryHValue_Left)
									SysFootPedalData.FootPedalADValue = SysFootPedalData.FootPedalMemoryHValue_Left;  //防止脚踏定标错误后，读取的存储最大值比实际最大值小							
								 SysRunData.MotorRealSpeed = (SysFootPedalData.FootPedalADValue - SysFootPedalData.FootPedalMemoryMValue_Left - 20) * SysRunData.MotorSetSpeed / (SysFootPedalData.FootPedalMemoryHValue_Left - SysFootPedalData.FootPedalMemoryMValue_Left-50) / 10;		 						
							}
							else
							{
							   SysRunData.MotorRealSpeed = 0;
							}
						}
						if (SysModelConfig.HandlePortB == 1)
						{
							if (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryMValue_Right + FootPedalValueOffset))
							{							
								if (SysFootPedalData.FootPedalADValue > SysFootPedalData.FootPedalMemoryHValue_Right)
									SysFootPedalData.FootPedalADValue = SysFootPedalData.FootPedalMemoryHValue_Right;  //防止脚踏定标错误后，读取的存储最大值比实际最大值小							
								 SysRunData.MotorRealSpeed = (SysFootPedalData.FootPedalADValue - SysFootPedalData.FootPedalMemoryMValue_Right - 20) * SysRunData.MotorSetSpeed / (SysFootPedalData.FootPedalMemoryHValue_Right - SysFootPedalData.FootPedalMemoryMValue_Right-50) / 10; 						
					    } 
							else
							{
							   SysRunData.MotorRealSpeed = 0;
							}							
					 }						
				}
				else
        {
					if (SysFootPedalData.FootPedalADValue > SysFootPedalData.FootPedalMemoryHValue)
						SysFootPedalData.FootPedalADValue = SysFootPedalData.FootPedalMemoryHValue;  //防止脚踏定标错误后，读取的存储最大值比实际最大值小					
					SysRunData.MotorRealSpeed = (SysFootPedalData.FootPedalADValue - SysFootPedalData.FootPedalMemoryLValue - 20) * SysRunData.MotorSetSpeed / (SysFootPedalData.FootPedalMemoryHValue - SysFootPedalData.FootPedalMemoryLValue - 40) / 10;
					//将脚踏的值转换为相应的速度值。/10为驱动板的速度是*10的，所以控制板的命令是要/10的。//脚踏高值和低值之间相差要大于40，不踩脚踏读取值不能大于低值20.
				}					
				if (SysRunData.MotorRealSpeed > (SysRunData.MotorSetSpeed / 10))
					SysRunData.MotorRealSpeed = SysRunData.MotorSetSpeed / 10;				
				

		    //电机运行，禁止切换手柄的操作
		    HandleAllowSwitchTime = 0;
		    SysRunData.HandleAllowSwitchFlag = 1;
        StopSendFlag = 1;//发送停止命令
				StopSendTimeCnt = 0;
				
		    switch (SysRunData.MotorNum)
		    {
		      //1号电机 Motor_Number
		      case MotorNum1 :
		      {
			      if (SysRunData.MotorRealSpeed < 500)  //最低 5000rpm 的速度
			      {
			        SysRunData.MotorRealSpeed = 0;

			        SysRunData.MotorRun = MotorStop;

//			      BrushlessMotor_Stop(1, 1, 0, 0x01);
							BrushlessMotor_Stop(SysInterface.InterfaceSwitchNo1, 1, 0, 0x01); 
 			      }
            else
						{
							DriveCtrl_Motor1Ctrl_Module();  //BrushlessMotor_Run(1, 1, 0, 0x02, SysRunData.MotorRealSpeed);	  //发送电机转速码 DanXiang							
						}
			      SysRunData.MotorRun = MotorWorking;   //电机正在工作标志

						if (SysModelConfig.HandlePortA == 1)
						{
							SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);   
	            DriveCtrl_PumpFlag_B();  //B泵停止
						}
						if (SysModelConfig.HandlePortB == 1)
						{
							SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
							Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  
              DriveCtrl_PumpFlag_A();  //A泵停止							
						}							

			    }
		      break;
		      //2、3号电机
		      case MotorNum2 :
		      case MotorNum3 :
		      {
			      if (SysRunData.MotorRealSpeed < 50) //最低 5000rpm 的速度
			      {
			        SysRunData.MotorRealSpeed = 0;

			        SysRunData.Motor2StopTime = 5;

			        SysRunData.MotorRun = MotorStop;

			        if (SysRunData.MotorNum == MotorNum2)
			        {  
								BrushlessMotor_Stop(SysInterface.InterfaceSwitchNo2, 1, 0, 0x02);								
			        }
			        else
			        {
				        BrushlessMotor_Stop(1, 1, 0 ,3);
								BrushlessMotor_Stop(2, 1, 0 ,3);	
			        }
 
			      }
						else
            {
							if (SysRunData.MotorNum == MotorNum2)  //2号电机  Motor_Number
							{
								SysRunData.Motor2StopTime = 5;  //电机2停止时间计数器

								DriveCtrl_Motor2Ctrl_Module();
							}
							else  //3号电机
							{
								DriveCtrl_Motor3Ctrl_Module();
							}						
						}
						
						SysRunData.MotorRun = MotorWorking;  //电机正工作标志
						
						if (SysModelConfig.HandlePortA == 1)
						{
							SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);   
	            DriveCtrl_PumpFlag_B();  //B泵停止
						}
						if (SysModelConfig.HandlePortB == 1)
						{
							SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
							Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  
              DriveCtrl_PumpFlag_A();  //A泵停止							
						}	
			    }
		      break;
		      default : break;
		    }
	    }
	    else  //【停】脚踏模式
	    {
		    if (SysFootPedalData.FootPedalADValue <= (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset))  //脚踏未踩下
		    {
		      SysRunData.StuckFlag = No_Error;
		      SysRunData.StuckFlag3 = No_Error;

		      SysRunData.HALLErrFlag = 0;
		      SysFootPedalData.FootPedalLiftFlag =	Lift;
		    }
		    else if (SysFootPedalData.FootPedalLiftFlag == LiftS)
		    {
		      if (++SysFootPedalData.FootPedalLiftTimeCnt >= 2)
		      {
			      SysFootPedalData.FootPedalLiftTimeCnt = 0;
			      SysFootPedalData.FootPedalLiftFlag =	Lift;
		      }
		    }
				//发送停止码2.2s后不发送
				if (StopSendFlag == 1)
				{
					StopSendTimeCnt++;
				  if (StopSendTimeCnt >= 40)
					{
					  StopSendTimeCnt = 0;
						StopSendFlag = 0;
					}
 		      Drive_Motor123CtrlStop_Module();				
				}

	    }
	  }
	  break;
    case ManualCtrl :    //手控 仅2号电机支持手控
	  {
	    if (((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press) && (SysRunData.MotorNum == MotorNum2)) && \
	         (Data_GetOnLineCnt() > 0) && (SysRunData.CommunicatFlag == Connect) && \
		       (SysRunData.StuckFlag3 == No_Error) && (SysRunData.PumpDrain_B == 0) && (SysRunData.PumpDrain_A == 0))   //Motor_Number
	    {
				//电机运行，禁止切换手柄的操作
		    HandleAllowSwitchTime = 0;
		    SysRunData.HandleAllowSwitchFlag = 1;

		    SysRunData.MotorRun = MotorWorking;  //马达运行标志

		    SysRunData.MotorRealSpeed = SysRunData.MotorSetSpeed / 10;   //设置转速

		    SysRunData.Motor2StopTime = 5;   //电机2停转时间计数器

		    DriveCtrl_Motor2Ctrl_Module();

				if (SysModelConfig.HandlePortA == 1)
				{
					SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
//					Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);   
					DriveCtrl_PumpFlag_B();  //B泵停止
				}
				if (SysModelConfig.HandlePortB == 1)
				{
					SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
					Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  
					DriveCtrl_PumpFlag_A();  //A泵停止							
				}	
		    
	    }
	    else  //【停】手控模式
	    {
	      if(SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] != Press)  //按键未按下...  ---LDY
		    {
		      SysRunData.StuckFlag3 = No_Error;
		      SysRunData.HALLErrFlag = 0;
		    }
				Drive_Motor123CtrlStop_Module();
	    }
	  }
	  break;
	  default : break;
  }
}

//============================================================================
//  55
//============================================================================
/* USER CODE BEGIN Header_MOTOR123TaskFunc */
/**
* @brief Function implementing the MOTOR123Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTOR123TaskFunc */
void MOTOR123TaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTOR123TaskFunc */
  /* Infinite loop */
 // DriveCtrl_Motor123Task_Fun();//原本信息
 	MotorControlSSC();
  /* USER CODE END MOTOR123TaskFunc */
}

void DriveCtrl_Motor123Task_Init(void)
{
  /* definition and creation of MOTOR123Task */
	Kernel_TaskCreate(&MOTOR123TaskHandle, MOTOR123TaskFunc);
	Kernel_TaskStart(&MOTOR123TaskHandle, KERNEL_TASK_ALWAYS, 50);
}


void HMIMotorControlSSC(void)
{
	//互斥
	static uint8_t mutualexclusion_flag=0;
	if(Workvalue_s.beep_Alarm_flag)
	{
		Workvalue_s.HMI_Working_flag=0;
		Workvalue_s.MOTORWorking_flag=0;
		return;
	}
	if(Workvalue_s.HMI_Control_flag)
		{
			if(Workvalue_s.HMI_Working_flag)//但凡遇到各种报警在HMI控制下都置为false
			{
				//运行
				mutualexclusion_flag=1;
				Workvalue_s.MOTORWorking_flag=1;
				
			}
			else
			{
				//停止
				if(mutualexclusion_flag){
						mutualexclusion_flag=0;
						Workvalue_s.MOTORWorking_flag=0;
				}
			}
		}
		else
		{
			//控制权交还于主机，是脚控还是手控，这里要用way()函数，判断时手控还是脚控，还是触控
		}
}


void MOTORHMITaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTOR123TaskFunc */
  /* Infinite loop */
 // DriveCtrl_Motor123Task_Fun();//原本信息
 	HMIMotorControlSSC();
  /* USER CODE END MOTOR123TaskFunc */
}

void DriveCtrl_HMITask_Init(void)
{
  /* definition and creation of MOTOR123Task */
	Kernel_TaskCreate(&MOTORJMITaskHandle, MOTORHMITaskFunc);
	Kernel_TaskStart(&MOTORJMITaskHandle, KERNEL_TASK_ALWAYS, 55);
}



















