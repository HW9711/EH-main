//pump.c

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "pump.h"
#include "data.h"
#include "bsp_board.h"
#include "common.h"
#include "uart5.h"
#include "uart7.h"
#include <stdint.h>
#include "lcd.h"
#include "kernel_scheduler.h"
#include "kernel_osal.h"


uint8_t pum_close_flag_A=0; 
uint8_t pum_close_flag_B=0; 
kernel_task_t PUMPTaskHandle;
kernel_task_t PEDAL2PUMP5STaskHandle;

static void PumpTaskDelayMs(uint32_t delay_ms)
{
	Kernel_DelayUntilMs(delay_ms);
}

void PumpDebugPoint(uint16_t point, uint16_t value, uint8_t detail)
{
	(void)value;
	(void)detail;
	LCD_Show_2byte_Number(0x8008,34);
	LCD_IntegratedCutterData_Update(0x4200, point, value, detail);
}

//static uint16_t PumpSpeed = 0;
//static uint16_t PumpSpeedLast = 0;
//static uint8_t PumpRunFlag = 0;
//static uint8_t Index = 0;

//泵缓启动
uint16_t PumpStartUp[25] =
{
  9374,
  3124,
  2082,
  1561,
  1249,
  985,
  749,
  567,
  479,
  415,
  359,
  316,
  283,
  227,
  188,
  162,
  141,
  125,
  111,
  102,
  93,
  86,
  79,
  79,
};


//============================================================================
// 函数名称: Pump_SetSpeed_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵的脉冲翻转周期设置
//============================================================================
void Pump_SetSpeed_B(uint32_t s)
{
	//ssc  加上标志位
	if(s)
	{
		LCD_Show_2byte_Number(0x9553,0xffE0);
	}
	else
	{
			LCD_Show_2byte_Number(0x9553,0xffff);
	}
		
		uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		dat[0] = 0xAA;
		#ifdef water_uptake
		dat[1] = 0x01;
		#else
		dat[1] = 0x00;
		#endif
		dat[2] = ((s & 0xFF00) >> 8);//0x03;
		dat[3] = (s & 0x00FF);//0xE8;
		dat[4] = 0xBB;
		dat[5] = 0xAA;
		//PumpDebugPoint((s > 0U) ? 301U : 302U, (uint16_t)s, dat[3]);

		Uart7_SendPacket(dat, 6);
		//PumpDebugPoint(sendResult ? 311U : 312U, (uint16_t)s, dat[3]);
		PumpTaskDelayMs(10);
		Uart7_SendPacket(dat, 6);
		//PumpDebugPoint(sendResult ? 321U : 322U, (uint16_t)s, dat[3]);
	
  //2.有流量
  //2.1计算脉宽
//  PumpSpeed = 18750 / s - 1;
}

//============================================================================
// 函数名称: Pump_SetSpeed_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵的脉冲翻转周期设置
//============================================================================
void Pump_SetSpeed_A(uint32_t s)
{
 static	uint8_t repeat_data;
	//ssc  加上标志位
	
	if(repeat_data!=s){
		if(s)
		{
			LCD_Show_2byte_Number(0x9533,0xffE0);
		}
		else
		{
			LCD_Show_2byte_Number(0x9533,0xffff);
		}
	uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	dat[0] = 0xAA;
	dat[1] = 0x01;
	dat[2] = ((s & 0xFF00) >> 8);//0x03;
	dat[3] = (s & 0x00FF);//0xE8;
	dat[4] = 0xBB;
	dat[5] = 0xAA;

	Uart5_SendPacket(dat, 6);
	PumpTaskDelayMs(10);
	Uart5_SendPacket(dat, 6);
		repeat_data=s;
}
  //2.有流量
  //2.1计算脉宽
//  PumpSpeed = 18750 / s - 1;
}
//============================================================================
// 函数名称: Pump_RunTask()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵的定时器周期改变任务
//============================================================================
void Pump_RunTask(void)
{
////  if ((SysRunData.PumpONOFF_B == ON) && ((SysRunData.MotorRun == MotorWorking) || (SysRunData.Pump5sRun_B == 1)))
////  {
////	  if(SysRunData.PumpMotorSetSpeed_B > 0)
////	  {
////	   
//////// 	    Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);

////	  }
////  }
////  else
////  {
////////      Pump_SetSpeed_B(0);
////  }
}

/* USER CODE BEGIN Header_PUMPTaskFunc */
/**
* @brief Function implementing the PUMPTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PUMPTaskFunc */
void PUMPTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN PUMPTaskFunc */
  /* Infinite loop */
  Pump_RunTask();
  /* USER CODE END PUMPTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 50
 */
//============================================================================
void Pump_RunTask_Init(void)
{
  /* definition and creation of PUMPTask */
	Kernel_TaskCreate(&PUMPTaskHandle, PUMPTaskFunc);
	Kernel_TaskStart(&PUMPTaskHandle, KERNEL_TASK_ALWAYS, 50);
}

//============================================================================
// 函数名称: Pump_Pedal2Motor5sFun_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏快速踩2次，泵运行5s
//============================================================================
void Pump_Pedal2Pump5sFun_A(void)    //Pedal2_stepping_motor5s
{
  static uint8_t SteppingTIME = 0;
  static uint8_t SteppingTIME1 = 0;
	static uint8_t Stepping_Flag1 = 0;

  static uint16_t SteppingMotorOFFTIME1 = 0;
  static uint16_t SteppingMotorOFFTIME = 0;

  static uint8_t STMotor5S_Flag = 0;
  static uint8_t STMotor10S_Flag = 0;

  ////////////////////////////////////////////	
	
	if (SysRunData.PumpModel_A == 2)
	{
		if (SysRunData.PumpPourIntoONOFF_A == 1)
		{
			SysRunData.PumpMotorSetSpeed_A  = Common_CurrentVelocity(SysRunData.FlowRateA,0);
			//Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A); 		
		}
		else
		{
			SysRunData.PumpMotorSetSpeed_A = 0;
//			Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);			
		}
	}
	else 
	{		
		//1.当前无手柄在线
		if (SysRunData.PumpONOFF_A == 0) //if (SysRunData.MotorNum == MotorNone)  //Motor_Number  Motor_Number  Motor_Number
		{
			SteppingMotorOFFTIME = 0;
			SteppingMotorOFFTIME1 = 0;

			STMotor5S_Flag = 0;
			STMotor10S_Flag = 0;

			SysRunData.PumpDrain_A = 0;
			SysRunData.PumpSteping5sNum_A = 0;
			SysRunData.Pump5sRun_A = 0;

			SysRunData.PumpMotorSetSpeed_A = 0;
//			Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
	 
		}
		else  //2.脚踏踩下、抬起判断
		{	
			if (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryHValue - 20))   //脚踏踩下
			{
				if (++SteppingTIME > 5)  //50ms防抖
				{
					SteppingTIME = 0;

					if (Stepping_Flag1 == 0)
					{
						Stepping_Flag1 = 1;

						SteppingMotorOFFTIME = 0;
						SteppingTIME1 = 0;

						if (++SysRunData.PumpSteping5sNum_A >= 2)
							SysRunData.PumpSteping5sNum_A = 2;
					}
				}
				
			}
			else if (SysFootPedalData.FootPedalADValue < (SysFootPedalData.FootPedalMemoryLValue + 20))  //脚踏松开
			{
				Stepping_Flag1 = 0;
				SteppingTIME = 0;

				if (SysRunData.PumpSteping5sNum_A != 2)
				{
					if (++SteppingTIME1 > 30)  //松开脚踏超300ms，不在累加连续操作功能
					{
						SteppingTIME1 = 0;

						SteppingMotorOFFTIME = 0;
						SteppingTIME = 0;

						SysRunData.PumpSteping5sNum_A = 0;
					}
				}
			}
			else
			{
				SteppingTIME = 0;
				SteppingTIME1 = 0;
			}

		//手柄未运行...

			if (SysRunData.MotorRun == MotorStop)
			{
				//泵运行5s标志、排空标志...
				if ((SysRunData.PumpSteping5sNum_A == 2) || (SysRunData.PumpDrain_A == 1))
				{
					//泵运行5s标志...
					if (SysRunData.PumpSteping5sNum_A == 2)
					{
						SysRunData.PumpDrain_A = 0;
						STMotor10S_Flag = 0;

						SteppingMotorOFFTIME1 = 0;

						if (STMotor5S_Flag == 0)
						{
							STMotor5S_Flag = 1;

							//按当前设置流量运行5s
							SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
						}

						SysRunData.Pump5sRun_A = 1;   //泵运行5s标志 1置上
						if(pum_close_flag_A==1)
						{
							SteppingMotorOFFTIME=400;
						}
						if (++SteppingMotorOFFTIME >= 400)  //4s
						{
							SteppingMotorOFFTIME = 0;
							STMotor5S_Flag = 0;

							SysRunData.PumpSteping5sNum_A = 0;
							SysRunData.Pump5sRun_A = 0;

							SysRunData.PumpMotorSetSpeed_A = 0;
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
							if(pum_close_flag_A==1)
							pum_close_flag_A=2;
						}
					}
					else if (SysRunData.PumpDrain_A == 1)
					{
						SysRunData.PumpSteping5sNum_A = 0;

						if (STMotor10S_Flag == 0)
						{
							STMotor10S_Flag = 1;
							//按最大流量运行10s
							////Delay_ms(500);
						
							SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(70,0);
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
						 
						}
						SysRunData.Pump5sRun_A = 1;   //泵运行5s标志 1置上
						if(pum_close_flag_A==1)
						{
							SteppingMotorOFFTIME1=850;
						}
						if (++SteppingMotorOFFTIME1 >= 850)  //8.5s
						{
							SteppingMotorOFFTIME1 = 0;
							STMotor10S_Flag = 0;

							SysRunData.PumpDrain_A = 0;
							SysRunData.Pump5sRun_A = 0;
							SysRunData.PumpMotorSetSpeed_A = 0;
//							Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
							if(pum_close_flag_A==1)
							pum_close_flag_A=2;
		//		    Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A); 
						}
					}
				}
				else
				{
					
						SysRunData.PumpMotorSetSpeed_A = 0;
//						Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);
						SteppingMotorOFFTIME = 0;
						SteppingMotorOFFTIME1 = 0;
						STMotor10S_Flag = 0;

						SysRunData.PumpDrain_A = 0;
						SysRunData.Pump5sRun_A = 0;
				}
			}
			else
			{
				SysRunData.PumpDrain_A = 0;  //泵排空关

				STMotor10S_Flag = 0;

				if (++SteppingTIME1 > 30)		//300ms//问题所在（bug）
				{
					SteppingMotorOFFTIME = 0;
					SteppingMotorOFFTIME1 = 0;
					SteppingTIME1 = 0;
					SteppingTIME = 0;
					SysRunData.PumpSteping5sNum_A = 0;  //超300ms，累计脚踏踩的连续中断
				}
			}
	  }
	}
}

//============================================================================
// 函数名称: Pump_Pedal2Motor5sFun_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏快速踩2次，泵运行5s
//============================================================================
void Pump_Pedal2Pump5sFun_B(void)    //Pedal2_stepping_motor5s
{
  static uint8_t SteppingTIME = 0;
  static uint8_t SteppingTIME1 = 0;
	static uint8_t Stepping_Flag1 = 0;

  static uint16_t SteppingMotorOFFTIME1 = 0;
  static uint16_t SteppingMotorOFFTIME = 0;

  static uint8_t STMotor5S_Flag = 0;
  static uint8_t STMotor10S_Flag = 0;

	
	
  if (SysRunData.PumpModel_B == 2)
	{
		if (SysRunData.PumpPourIntoONOFF_B == 1)
		{
			SysRunData.PumpMotorSetSpeed_B  = Common_CurrentVelocity(SysRunData.FlowRateB,1);
			Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B); 		
		}
		else
		{
			SysRunData.PumpMotorSetSpeed_B = 0;
			Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);			
		}
	}
  else	
	{
		//1.当前无手柄在线
		if (SysRunData.PumpONOFF_B == 0) //if (SysRunData.MotorNum == MotorNone)  //Motor_Number  Motor_Number  Motor_Number
		{
			SteppingMotorOFFTIME = 0;
			SteppingMotorOFFTIME1 = 0;

			STMotor5S_Flag = 0;
			STMotor10S_Flag = 0;

			SysRunData.PumpDrain_B = 0;
			SysRunData.PumpSteping5sNum_B = 0;
			SysRunData.Pump5sRun_B = 0;

			SysRunData.PumpMotorSetSpeed_B = 0;
			Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
	 
		}
		else //2.脚踏踩下、抬起判断
		{
			if (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryHValue - 20))   //脚踏踩下
			{
				if (++SteppingTIME > 5)  //50ms防抖
				{
					SteppingTIME = 0;

					if (Stepping_Flag1 == 0)
					{
						Stepping_Flag1 = 1;

						SteppingMotorOFFTIME = 0;
						SteppingTIME1 = 0;

						if (++SysRunData.PumpSteping5sNum_B >= 2)
							SysRunData.PumpSteping5sNum_B = 2;
					}
				}
				
			}
			else if (SysFootPedalData.FootPedalADValue < (SysFootPedalData.FootPedalMemoryLValue + 20))  //脚踏松开
			{
				Stepping_Flag1 = 0;
				SteppingTIME = 0;

				if (SysRunData.PumpSteping5sNum_B != 2)
				{
					if (++SteppingTIME1 > 30)  //松开脚踏超300ms，不在累加连续操作功能
					{
						SteppingTIME1 = 0;

						SteppingMotorOFFTIME = 0;
						SteppingTIME = 0;

						SysRunData.PumpSteping5sNum_B = 0;
					}
				}
			}
			else
			{
				SteppingTIME = 0;
				SteppingTIME1 = 0;
			}

			//手柄未运行...

				if (SysRunData.MotorRun == MotorStop)
				{
					//泵运行5s标志、排空标志...
					if ((SysRunData.PumpSteping5sNum_B == 2) || (SysRunData.PumpDrain_B == 1))
					{
						//泵运行5s标志...
						if (SysRunData.PumpSteping5sNum_B == 2)
						{
							SysRunData.PumpDrain_B = 0;
							STMotor10S_Flag = 0;

							SteppingMotorOFFTIME1 = 0;

							if (STMotor5S_Flag == 0)
							{
								STMotor5S_Flag = 1;

								//按当前设置流量运行5s
								SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
								Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
							}

							SysRunData.Pump5sRun_B = 1;   //泵运行5s标志 1置上
							if(pum_close_flag_B==1)
							{
								SteppingMotorOFFTIME=400;
							}
							if (++SteppingMotorOFFTIME >= 400)  //4s
							{
								SteppingMotorOFFTIME = 0;
								STMotor5S_Flag = 0;

								SysRunData.PumpSteping5sNum_B = 0;
								SysRunData.Pump5sRun_B = 0;

								SysRunData.PumpMotorSetSpeed_B = 0;
								Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
								if(pum_close_flag_B==1)
								pum_close_flag_B=2;
							}
						}
						else if (SysRunData.PumpDrain_B == 1)
						{
							SysRunData.PumpSteping5sNum_B = 0;

							if (STMotor10S_Flag == 0)
							{
								STMotor10S_Flag = 1;
								//按最大流量运行10s
								////Delay_ms(500);
							
								SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(70,0);
								Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
							 
							}
							SysRunData.Pump5sRun_B = 1;   //泵运行5s标志 1置上
							if(pum_close_flag_B==1)
							{
								SteppingMotorOFFTIME1=850;
							}
							if (++SteppingMotorOFFTIME1 >= 850)  //8.5s
							{
								SteppingMotorOFFTIME1 = 0;
								STMotor10S_Flag = 0;

								SysRunData.PumpDrain_B = 0;
								SysRunData.Pump5sRun_B = 0;
								SysRunData.PumpMotorSetSpeed_B = 0;
								Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / SysRunData.STMotorSetSpeed - 1); //TIM7_Init(18750 / SysRunData.STMotorSetSpeed - 1, 72 - 1);
								if(pum_close_flag_B==1)
								pum_close_flag_B=2;
			//		    Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B); 
							}
						}
					}
					else
					{
						
							SysRunData.PumpMotorSetSpeed_B = 0;
							Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);
							SteppingMotorOFFTIME = 0;
							SteppingMotorOFFTIME1 = 0;
							STMotor10S_Flag = 0;

							SysRunData.PumpDrain_B = 0;
							SysRunData.Pump5sRun_B = 0;
					}
				}
				else
				{
					SysRunData.PumpDrain_B = 0;  //泵排空关

					STMotor10S_Flag = 0;

					if (++SteppingTIME1 > 30)		//300ms//问题所在（bug）
					{
						SteppingMotorOFFTIME = 0;
						SteppingMotorOFFTIME1 = 0;
						SteppingTIME1 = 0;
						SteppingTIME = 0;
						SysRunData.PumpSteping5sNum_B = 0;  //超300ms，累计脚踏踩的连续中断
					}
				}
			}
  }
}

/* USER CODE BEGIN Header_PEDAL2PUMP5STaskFunc */
/**
* @brief Function implementing the PEDAL2PUMP5STask thread.
* @param argument: Not used
* @retval None 
*/
/* USER CODE END Header_PEDAL2PUMP5STaskFunc */
void PEDAL2PUMP5STaskFunc(uint32_t event)
{
  /* USER CODE BEGIN PEDAL2PUMP5STaskFunc */
  /* Infinite loop */
		#ifndef WATER_UPTAKE
		Pump_Pedal2Pump5sFun_A();
		Pump_Pedal2Pump5sFun_B();	
		#endif
  /* USER CODE END PEDAL2PUMP5STaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 10
 */
//============================================================================
void Pump_Pedal2Pump5sTask_Init(void)
{
  /* definition and creation of PEDAL2PUMP5STask */
	Kernel_TaskCreate(&PEDAL2PUMP5STaskHandle, PEDAL2PUMP5STaskFunc);
	Kernel_TaskStart(&PEDAL2PUMP5STaskHandle, KERNEL_TASK_ALWAYS, 10);
}










