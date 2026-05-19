//pump.c

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "pump.h"
#include "Pubinterface.h"
#include "pump_pressure_control.h"
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

static const pumpMessage_t *PumpLegacy_GetPressureSourceA(void)
{
	/* 固定读取 pumpMessageA；当前现场映射为 A 泵压力传感器接 SIM_UART_2/PE6。 */
	return &pumpMessageA;
}

static const pumpMessage_t *PumpLegacy_GetPressureSourceB(void)
{
	/* 固定读取 pumpMessageB；当前现场映射为 B 泵压力传感器接 SIM_UART_1/PE4。 */
	return &pumpMessageB;
}

static uint32_t PumpLegacy_ApplyPressureLimit(pumpMessage_t *runtime_msg,
											  const pumpMessage_t *pressure_source,
											  uint32_t output_value)
{
	/* protected_value 保存最终允许下发到泵驱动的 16 位速度/脉冲数据。 */
	uint16_t protected_value;
	/* weight_x10 从压力源拷贝到局部变量，保证本次限速比较使用同一次读取结果。 */
	uint32_t weight_x10;
	/* threshold_g 从压力源拷贝到局部变量，和 weight_x10 一起组成当前闭环判断条件。 */
	uint16_t threshold_g;
	/* force_stop 为 1 表示压力超过硬停倍率，需要把本次输出压到 0。 */
	uint8_t force_stop;

	/*
	 * runtime_msg 旧参数保留用于接口兼容，但压力保护现在只暂停本次输出，不清 run_flag。
	 * 这样旧直接输出入口在压力恢复后也可以继续沿用原来的运行请求自动恢复。
	 */
	(void)runtime_msg;

	/* 压力源为空时不能做闭环，直接保持原输出，避免空指针导致异常停机。 */
	if (pressure_source == NULL)
	{
		/* 返回原始值，保持旧路径在异常配置下的行为不变。 */
		return output_value;
	}

	/* 旧泵协议只发送 16 位速度字段，这里显式截成 16 位后再进入公共闭环算法。 */
	protected_value = (uint16_t)(output_value & 0xFFFFU);
	/* 读取当前压力重量，单位 0.1g，来自 CS1237 解析后的 pumpMessageA/B。 */
	weight_x10 = pressure_source->weight_x10;
	/* 读取当前压力阈值，单位 g，来自压力模块上报的 ThresholdG。 */
	threshold_g = pressure_source->pressure_threshold;
	/* 判断是否已经进入宏配置硬停倍率的停泵区间。 */
	force_stop = PumpPressureControl_ShouldForceStop(weight_x10, threshold_g);
	/* 对旧的直接输出值同样做线性限速，避免旧 UI/参数路径绕过 sscPUMPA/sscPUMPB。 */
	protected_value = PumpPressureControl_Apply(protected_value, weight_x10, threshold_g);

	/* 进入硬停泵区间时只把本次输出压为 0，不清运行标志，压力恢复后允许自动续转。 */
	if (force_stop != 0U)
	{
		/* 硬停泵输出必须为 0，即使线性算法后续调整也不能重新放大。 */
		protected_value = 0U;
	}

	/* 返回最终允许写入泵 UART 帧的速度字段。 */
	return (uint32_t)protected_value;
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
	/* 旧 B 泵直接输出入口也套压力保护，防止屏幕/参数路径绕过 sscPUMPB 的闭环。 */
	s = PumpLegacy_ApplyPressureLimit(&pumpMessageB, PumpLegacy_GetPressureSourceB(), s);
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

#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
		/* 当前整机逻辑 B 泵对应原 A 泵物理出口，旧直连入口也必须跟随互换到 UART5。 */
		Uart5_SendPacket(dat, 6);
#else
		/* 关闭互换时保持旧版接线：逻辑 B 泵走 UART7。 */
		Uart7_SendPacket(dat, 6);
#endif
		//PumpDebugPoint(sendResult ? 311U : 312U, (uint16_t)s, dat[3]);
		PumpTaskDelayMs(10);
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
		/* 第二帧重复发送也保持同一个物理出口，避免两路泵同时收到不同节拍的重复帧。 */
		Uart5_SendPacket(dat, 6);
#else
		/* 关闭互换时保持旧版接线：逻辑 B 泵走 UART7。 */
		Uart7_SendPacket(dat, 6);
#endif
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
	/* 旧 A 泵直接输出入口也套压力保护，防止手柄/报警停泵路径绕过 sscPUMPA 的闭环。 */
	s = PumpLegacy_ApplyPressureLimit(&pumpMessageA, PumpLegacy_GetPressureSourceA(), s);
	//ssc  加上标志位
	
	if((repeat_data!=s) || (s == 0U)){
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

#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	/* 当前整机逻辑 A 泵对应原 B 泵物理出口，旧直连入口也必须跟随互换到 UART7。 */
	Uart7_SendPacket(dat, 6);
#else
	/* 关闭互换时保持旧版接线：逻辑 A 泵走 UART5。 */
	Uart5_SendPacket(dat, 6);
#endif
	PumpTaskDelayMs(10);
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	/* 第二帧重复发送也保持同一个物理出口，避免逻辑 A 泵仍打到右侧物理泵。 */
	Uart7_SendPacket(dat, 6);
#else
	/* 关闭互换时保持旧版接线：逻辑 A 泵走 UART5。 */
	Uart5_SendPacket(dat, 6);
#endif
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










