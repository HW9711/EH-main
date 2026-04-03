//param.c

#include "param.h"
#include "screenkey.h"
#include "data.h"
#include "common.h"
#include "lcd.h"
#include "screen.h"
#include "eeprom.h"
#include "pump.h"
#include "motor.h"
#include "delay.h"

//============================================================================
//1.”参数“设置
//============================================================================

//============================================================================
// 函数名称: Param_0x5010_0x5030_Minus()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 转速减、减减
//============================================================================
void Param_0x5010_0x5030_Minus(uint8_t type)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  switch (SysRunData.MotorNum)  //Motor_Number
  {
	  case MotorNum1 :
	  {
	    if (SysInterface.HandleType[index] == Handle_Type_2)  //JMB-ssc改为一体磨
			{
				if( SysSetParam[index].RunMotorSpeed>20000)
	      SysSetParam[index].RunMotorSpeed -= 5000;
				else 
				SysSetParam[index].RunMotorSpeed -= 1000;
			}
	    else
	      SysSetParam[index].RunMotorSpeed -= 2000;
	  }
	  break;
	  case MotorNum2 :
	  {
	    if (SysSetParam[index].DJSetPDMT == 0)  //刨刀
		    SysSetParam[index].RunMotorSpeed -= 500;
	    else  //磨头
		    SysSetParam[index].RunMotorSpeed -= 1000;
	  }
	  break;
	  case MotorNum3 :
	  {
	    if (SysHandleData.Ds2431BuffBB[SysInterface.InterfaceSwitchNo3 - 1][4] == 1 )
		    SysSetParam[index].RunMotorSpeed -= 500;
	    else  //磨头
		    SysSetParam[index].RunMotorSpeed -= 1000;
	  }
    break;
	  default : break;
  }
  if ((SysRunData.MotorNum >= MotorNum1) && (SysRunData.MotorNum <= MotorNum3))
  {
    if (SysSetParam[index].RunMotorSpeed < SysSetParam[index].MinMotorSpeed)
	    SysSetParam[index].RunMotorSpeed = SysSetParam[index].MinMotorSpeed;

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].RunMotorSpeed;
	  SysRunData.MotorSetSpeed = SysSetParam[index].RunMotorSpeed;
if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
else
	 LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
}

//============================================================================
// 函数名称: Param_0x5050_0x5070_Add()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 转速加、加加
//============================================================================
void Param_0x5050_0x5070_Add(uint8_t type)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  switch (SysRunData.MotorNum)  //Motor_Number)
  {
	  case MotorNum1 :
	  {
	    if (SysInterface.HandleType[index] == Handle_Type_2)  //JMB
			{  
				if(	SysSetParam[index].RunMotorSpeed>=20000)
				SysSetParam[index].RunMotorSpeed += 5000;
				else
		    SysSetParam[index].RunMotorSpeed += 1000;
			}
			else
				SysSetParam[index].RunMotorSpeed += 2000;
	  }
 	  break;
	  case MotorNum2 :
	  {
	    if (SysSetParam[index].DJSetPDMT == 0)
		    SysSetParam[index].RunMotorSpeed += 500;
	    else
	 	    SysSetParam[index].RunMotorSpeed += 1000;
	  }
	  break;
	  case MotorNum3 :
	  {
	    if (SysHandleData.Ds2431BuffBB[SysInterface.InterfaceSwitchNo3 - 1][4] == 1)
		    SysSetParam[index].RunMotorSpeed += 500;
	    else
	      SysSetParam[index].RunMotorSpeed += 1000;
	  }
	  break;
	  default : break;
  }

  if ((SysRunData.MotorNum >= MotorNum1) && (SysRunData.MotorNum <= MotorNum3))
  {
    if (SysSetParam[index].RunMotorSpeed > SysSetParam[index].MaxMotorSpeed)
      SysSetParam[index].RunMotorSpeed = SysSetParam[index].MaxMotorSpeed;

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].RunMotorSpeed;
    SysRunData.MotorSetSpeed = SysSetParam[index].RunMotorSpeed;
if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
else
		LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
}

//============================================================================
// 函数名称: Param_0x5110_0x5130_Minus()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 频率减，转速 Ⅰ 档
//============================================================================
void Param_0x5110_0x5130_Minus(uint8_t type)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if ((((SysRunData.MotorNum == MotorNum3) || (SysRunData.MotorNum == MotorNum2)) && (SysSetParam[index].MotorModel != RECIPROCATING)) || (SysRunData.MotorNum == MotorNum1))
  {
    /*
	  LCD_Show_Picture(0x1304, 223);
	  LCD_Disappear_Number(0x9460);
	 
	  LCD_Disappear_Number(0x9480);
	  */

	  SysSetParam[index].GearPositionHz = 4;  //Ⅰ档

    Info_HZ(SysSetParam[index].GearPositionHz);
	  /*
	  switch (SysRunData.MotorNumber)
	  {
	    case 1 :
	    {
        SysSetParam[index].StartMotorSpeed = SysSetParam[index].ISpeed;
		    SysSetParam[index].RunMotorSpeed =  SysSetParam[index].StartMotorSpeed;

		    SysRunData.SetISpeed = SysSetParam[index].ISpeed;
		    SysRunData.StartSetMotorSpeed = SysRunData.SetISpeed;
		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
 	    case 2 :
	    {
	      SysRunData.StartSetMotorSpeed = SysRunData.SetISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 3 :
	    {
		    SysRunData.StartSetMotorSpeed = SysRunData.SetISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    default : break;
	  }
	  */

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].ISpeed;
	  SysSetParam[index].RunMotorSpeed =  SysSetParam[index].StartMotorSpeed;

	  SysRunData.SetISpeed = SysSetParam[index].ISpeed;
	  SysRunData.StartSetMotorSpeed = SysRunData.SetISpeed;
	  SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;
		if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
				LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
		else
	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
  else
  {
	  if (SysSetParam[index].HzSet > 5)
	    SysSetParam[index].HzSet -= 5;

	  SysRunData.Frequency = SysSetParam[index].HzSet;
		 LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));

//	  LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
  }
}

//============================================================================
// 函数名称: Param_0x5150_0x5170_Add()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 转速 Ⅱ 档
//============================================================================
void Param_0x2405_Er(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  /*
  LCD_Show_Picture(0x1304, 224);
  LCD_Disappear_Number(0x9460);

  LCD_Disappear_Number(0x9480);
  */

  SysSetParam[index].GearPositionHz = 5;  //Ⅱ档
	
  Info_HZ(SysSetParam[index].GearPositionHz);
 
  /*
  switch (SysRunData.MotorNumber)
  {
	  case 1 :
	  {
	    SysSetParam[index].StartMotorSpeed =  SysSetParam[index].IISpeed;
      SysSetParam[index].RunMotorSpeed =  SysSetParam[index].StartMotorSpeed;

	    SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
	    SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
	    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

	    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
    }
	  break;
	  case 2 :
	  {
	    SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

	    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

  	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	  }
	  break;
	  case 3 :
	  {
	    SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

	    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

	    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	  }
	  break;
	  default : break;
  }
  */

  SysSetParam[index].StartMotorSpeed =  SysSetParam[index].IISpeed;
  SysSetParam[index].RunMotorSpeed =  SysSetParam[index].StartMotorSpeed;

  SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
  SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
  SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
else
	 LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
}

//============================================================================
// 函数名称: Param_0x5150_0x5170_Add()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 频率加，转速 Ⅲ 档
//============================================================================
void Param_0x5150_0x5170_Add(uint8_t type)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if ((((SysRunData.MotorNum == MotorNum3) || (SysRunData.MotorNum == MotorNum2)) && (SysSetParam[index].MotorModel != RECIPROCATING)) || (SysRunData.MotorNum == MotorNum1))
  {
    /*
	  LCD_Show_Picture(0x1304, 225);
	  LCD_Disappear_Number(0x9460);
	 
	  LCD_Disappear_Number(0x9480);
	  */

	  SysSetParam[index].GearPositionHz = 6;  //Ⅲ档
		
    Info_HZ(SysSetParam[index].GearPositionHz); 
 
	  /*
	  switch (SysRunData.MotorNumber)
	  {
	    case 1 :
	    {
        SysSetParam[index].StartMotorSpeed = SysSetParam[index].IIISpeed;
		    SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

		    SysRunData.SetIIISpeed = SysSetParam[index].IIISpeed;
		    SysRunData.StartSetMotorSpeed = SysRunData.SetIIISpeed;
		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 2 :
	    {
		    SysRunData.StartSetMotorSpeed = SysRunData.SetIIISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
  	  break;
	    case 3 :
	    {
		    SysRunData.StartSetMotorSpeed = SysRunData.SetIIISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    default : break;
	  }
	  */

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].IIISpeed;
	  SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

	  SysRunData.SetIIISpeed = SysSetParam[index].IIISpeed;
	  SysRunData.StartSetMotorSpeed = SysRunData.SetIIISpeed;
	  SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;
	if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2))
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
	else
	 LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
  else
  {
	  if (SysSetParam[index].HzSet < 40)
	    SysSetParam[index].HzSet += 5;

	  SysRunData.Frequency = SysSetParam[index].HzSet;

	  LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
  }
}
//============================================================================
// 函数名称: Param_0x2413_Pump_OnOff_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵开关 1开，0关
//============================================================================
void Param_0x2413_Pump_OnOff_A(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;
	
  SysRunData.PumpDrain_A = 0;  //泵排空关
  if (SysSetParam[index].PumpOffOnA == ON)
  {
	  SysSetParam[index].PumpOffOnA = OFF;
		
	  SysRunData.PumpMotorSetSpeed_A = 0;
    Info_A(5); //Screen_InformationBarImage_Update(1, 2, SysSetParam[index].GearPositionHz, 2);
  }
  else
  {
	  SysSetParam[index].PumpOffOnA = ON;
 
    Info_A(1); //Screen_InformationBarImage_Update(1, 1, SysSetParam[index].GearPositionHz, 1);
		#ifdef  WATER_UPTAKE//如果定义吸水ssc
//	  LCD_Show_4byte_Number(0x3530, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
		LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
		#else
//	  LCD_Show_4byte_Number(0x3530, 70);   //70mL/min --> 1.0L/min
	  LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);  //流速  SysRunData.NumberFluidSet);
		#endif
//		LCD_Show_4byte_Number(0x3430, 0);
  }
  Info_HZ(SysSetParam[index].GearPositionHz);
  SysRunData.PumpONOFF_A = SysSetParam[index].PumpOffOnA;
}

//============================================================================
// 函数名称: Param_0x2406_Pump_OnOff_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵开关 1开，0关
//============================================================================
void Param_0x2406_Pump_OnOff_B(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  SysRunData.PumpDrain_B = 0;  //泵排空关	
  if (SysSetParam[index].PumpOffOnB == ON)
  {
	  SysSetParam[index].PumpOffOnB = OFF;

	  SysRunData.PumpMotorSetSpeed_B = 0;
		Info_B(5); //    Screen_InformationBarImage_Update(1, 2, SysSetParam[index].GearPositionHz, 2);
  }
  else
  {
	  SysSetParam[index].PumpOffOnB = ON;
    Info_B(1); //    Screen_InformationBarImage_Update(1, 1, SysSetParam[index].GearPositionHz, 1);
		#ifdef  WATER_UPTAKE//如果定义吸水ssc
//	  LCD_Show_4byte_Number(0x3550, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
		LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
		#else
//	  LCD_Show_4byte_Number(0x3550, 70);   //70mL/min --> 1.0L/min
	  LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);  //流速  SysRunData.NumberFluidSet);
		#endif
//		LCD_Show_4byte_Number(0x3450, 0);
  }
  Info_HZ(SysSetParam[index].GearPositionHz);
  SysRunData.PumpONOFF_B = SysSetParam[index].PumpOffOnB;
}

//============================================================================
// 函数名称: Param_0x5210_0x5230_Minus_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 1流量减 2减减
//============================================================================
uint8_t Param_0x5210_0x5230_Minus_B(uint8_t type)
{
  uint8_t ret = 0xff, temp = 0;
  uint8_t index = SysInterface.BeSelectNum - 1;
	
  if (SysRunData.PumpModel_B == 1 || SysRunData.PumpModel_B == 3) 
	{
 
		 #ifdef WATER_UPTAKE
			
			if (SysSetParam[index].PumpVelocitySetB >0)
						SysSetParam[index].PumpVelocitySetB-- ;
			#else
				 if (SysSetParam[index].PumpVelocitySetB <= 1)
				SysSetParam[index].PumpVelocitySetB = 0;
			else if (SysSetParam[index].PumpVelocitySetB <= 6)
				SysSetParam[index].PumpVelocitySetB -= 1;
			else if (SysSetParam[index].PumpVelocitySetB <= 20)
				SysSetParam[index].PumpVelocitySetB -= 2;
			else
				SysSetParam[index].PumpVelocitySetB -= 5;
			#endif
		 

			SysRunData.FlowRateB = SysSetParam[index].PumpVelocitySetB;

			if (SysRunData.MotorNum == MotorNum1)
			{
				SysRunData.PumpFlowEEPOM = SysRunData.FlowRateB;

				temp = (index != 4) ? 0x21 : 0x23;

				if (type == 1)
					EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);
				else
					ret = 1;
			}
  }
	else if (SysRunData.PumpModel_B == 2) 
  {
		 
		 if (SysRunData.PumpPourIntoVelocityB <= 10)
				SysRunData.PumpPourIntoVelocityB = 0;
		 else  
				SysRunData.PumpPourIntoVelocityB -= 10;	
		 SysRunData.FlowRateB = SysRunData.PumpPourIntoVelocityB;
	}
	
	#ifdef  WATER_UPTAKE//如果定义吸水ssc
 	LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
	#else
	LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
	#endif

	if (SysRunData.MotorRun == MotorWorking)
	{
			SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
			Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / Stepping_Speed - 1); //TIM7_Init((18750 / Stepping_Speed - 1), (72 - 1));
  }
	
  #ifdef TESTPUMP
  TestTask_Run(SysRunData.NumberFluidSet);
  #endif

  return ret;
}

//============================================================================
// 函数名称: Param_0x5250_0x5270_Add_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 1流量加 2加加
//============================================================================
uint8_t Param_0x5250_0x5270_Add_B(uint8_t type)
{
  uint8_t ret = 0xff, temp = 0;
  uint8_t index = SysInterface.BeSelectNum - 1;
	
	if ( SysRunData.PumpModel_B == 1 || SysRunData.PumpModel_B == 3) 
	{
	 
		#ifdef WATER_UPTAKE
					 if (SysSetParam[index].PumpVelocitySetB <15)
						SysSetParam[index].PumpVelocitySetB++;
			#else
			if (SysSetParam[index].PumpVelocitySetB >= 70)
				SysSetParam[index].PumpVelocitySetB = 70;
			else if (SysSetParam[index].PumpVelocitySetB >= 20)
				SysSetParam[index].PumpVelocitySetB += 5;
			else if (SysSetParam[index].PumpVelocitySetB >= 6)
				SysSetParam[index].PumpVelocitySetB += 2;
			else
				SysSetParam[index].PumpVelocitySetB += 1;
		#endif
			SysRunData.FlowRateB = SysSetParam[index].PumpVelocitySetB;

			if (SysRunData.MotorNum == MotorNum1)
			{
				SysRunData.PumpFlowEEPOM = SysRunData.FlowRateB;

				temp = (index != 4) ? 0x21 : 0x23;

				if (type == 1)
					EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);
				else
					ret = 1;
			}
	}
	else if ( SysRunData.PumpModel_B == 2) 
	{
	 
		if (SysRunData.PumpPourIntoVelocityB >= 300)
			SysRunData.PumpPourIntoVelocityB = 300;
		else  
			SysRunData.PumpPourIntoVelocityB += 10;
		SysRunData.FlowRateB = SysRunData.PumpPourIntoVelocityB;		
	}
	

 	#ifdef  WATER_UPTAKE//如果定义吸水ssc
 	LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
	#else
	LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
	#endif
	
  if (SysRunData.MotorRun == MotorWorking)
	{
		SysRunData.PumpMotorSetSpeed_B = Common_CurrentVelocity(SysRunData.FlowRateB,0);
		Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);  //TIM7_IRQPeriod(18750 / Stepping_Speed - 1); //TIM7_Init(18750 / Stepping_Speed - 1, (72 - 1));
		
	}

  #ifdef TESTPUMP
  TestTask_Run(SysRunData.NumberFluidSet);
  #endif

  return ret;
}

//============================================================================
// 函数名称: Param_0x5280_0x52A0_Minus_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 1流量减 2减减
//============================================================================
uint8_t Param_0x5280_0x52A0_Minus_A(uint8_t type)
{
  uint8_t ret = 0xff, temp = 0;
  uint8_t index = SysInterface.BeSelectNum - 1;
	
	if ( SysRunData.PumpModel_A == 1 || SysRunData.PumpModel_A == 3) 
	{
		 
		 #ifdef WATER_UPTAKE
			
			if (SysSetParam[index].PumpVelocitySetA >0)
						SysSetParam[index].PumpVelocitySetA-- ;
			#else
				 if (SysSetParam[index].PumpVelocitySetA <= 1)
				SysSetParam[index].PumpVelocitySetA = 0;
			else if (SysSetParam[index].PumpVelocitySetA <= 6)
				SysSetParam[index].PumpVelocitySetA -= 1;
			else if (SysSetParam[index].PumpVelocitySetA <= 20)
				SysSetParam[index].PumpVelocitySetA -= 2;
			else
				SysSetParam[index].PumpVelocitySetA -= 5;
			#endif
		 

			SysRunData.FlowRateA = SysSetParam[index].PumpVelocitySetA;

			if (SysRunData.MotorNum == MotorNum1)
			{
				SysRunData.PumpFlowEEPOM = SysRunData.FlowRateA;

				temp = (index != 4) ? 0x25 : 0x27;

				if (type == 1)
					EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);
				else
					ret = 1;
			}
	}
  else if ( SysRunData.PumpModel_A == 2) 
	{
		  
		 if (SysRunData.PumpPourIntoVelocityA <= 5)
				SysRunData.PumpPourIntoVelocityA = 0;
		 else  
				SysRunData.PumpPourIntoVelocityA -= 5;	
		 SysRunData.FlowRateA = SysRunData.PumpPourIntoVelocityA;	
	}
	
	#ifdef  WATER_UPTAKE//如果定义吸水ssc
 	LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
	#else
	LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
	#endif

	if (SysRunData.MotorRun == MotorWorking)
	{
			SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
		//	Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / Stepping_Speed - 1); //TIM7_Init((18750 / Stepping_Speed - 1), (72 - 1));
  }
	
  #ifdef TESTPUMP
  TestTask_Run(SysRunData.NumberFluidSet);
  #endif

  return ret;
}


//============================================================================
// 函数名称: Param_0x52B0_0x52D0_Add_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 1流量加 2加加
//============================================================================
uint8_t Param_0x52B0_0x52D0_Add_A(uint8_t type)
{
  uint8_t ret = 0xff, temp = 0;
  uint8_t index = SysInterface.BeSelectNum - 1;

	if ( SysRunData.PumpModel_A == 1 || SysRunData.PumpModel_A == 3) 
	{
 
		#ifdef WATER_UPTAKE
					 if (SysSetParam[index].PumpVelocitySetA <15)
						SysSetParam[index].PumpVelocitySetA++;
			#else
			if (SysSetParam[index].PumpVelocitySetA >= 70)
				SysSetParam[index].PumpVelocitySetA = 70;
			else if (SysSetParam[index].PumpVelocitySetA >= 20)
				SysSetParam[index].PumpVelocitySetA += 5;
			else if (SysSetParam[index].PumpVelocitySetA >= 6)
				SysSetParam[index].PumpVelocitySetA += 2;
			else
				SysSetParam[index].PumpVelocitySetA += 1;
		#endif
			SysRunData.FlowRateA = SysSetParam[index].PumpVelocitySetA;

			if (SysRunData.MotorNum == MotorNum1)
			{
				SysRunData.PumpFlowEEPOM = SysRunData.FlowRateA;

				temp = (index != 4) ? 0x25 : 0x27;

				if (type == 1)
					EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);
				else
					ret = 1;
			}
	}
	else if ( SysRunData.PumpModel_A == 2) 
	{
	 
		if (SysRunData.PumpPourIntoVelocityA >= 150)
			SysRunData.PumpPourIntoVelocityA = 150;
		else  
			SysRunData.PumpPourIntoVelocityA += 5;
		SysRunData.FlowRateA = SysRunData.PumpPourIntoVelocityA;		
	}
 	#ifdef  WATER_UPTAKE//如果定义吸水ssc
 	LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
	#else
	LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
	#endif
	
  if (SysRunData.MotorRun == MotorWorking)
	{
		SysRunData.PumpMotorSetSpeed_A = Common_CurrentVelocity(SysRunData.FlowRateA,0);
//		Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);  //TIM7_IRQPeriod(18750 / Stepping_Speed - 1); //TIM7_Init(18750 / Stepping_Speed - 1, (72 - 1));
		
	}

  #ifdef TESTPUMP
  TestTask_Run(SysRunData.NumberFluidSet);
  #endif

  return ret;
}

//============================================================================
// 函数名称: Param_0x2411_Pump_Drain_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵排空标志_B
//============================================================================
void Param_0x2411_Pump_Drain_B(void)
{
  if (SysRunData.PumpDrain_B == OFF)
	{
		SysRunData.PumpDrain_B = ON;
	}
	else
  {
	  SysRunData.PumpDrain_B = OFF;
	  SysRunData.PumpMotorSetSpeed_B = 0;
  }
}


//============================================================================
// 函数名称: Param_0x2414_Pump_Drain_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵排空标志_A
//============================================================================
void Param_0x2414_Pump_Drain_A(void)
{
  if (SysRunData.PumpDrain_A == OFF)
	{ 
		SysRunData.PumpDrain_A = ON;
	}
  else
  {
	  SysRunData.PumpDrain_A = OFF;
 	  SysRunData.PumpMotorSetSpeed_A = 0;
  }
}


//============================================================================
// 函数名称: Param_0x24010001_Pump_Drain_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明:  
//============================================================================
void Param_0x24010001_Pump_Drain_A(void)
{
  static uint8_t index = 4;
	
	if (index == 4) 
	{
		index = 2; 
		Info_A(index);	
		SysRunData.FlowRateA = SysRunData.PumpPourIntoVelocityA;
    LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);		
	}	
	else
	{
		index = 4; 
		Info_A(index);		
		SysRunData.PumpPourIntoONOFF_A = 0;	
		SysRunData.PumpMotorSetSpeed_A = 0;
//		Pump_SetSpeed_A(SysRunData.PumpMotorSetSpeed_A);			
	}
}

//============================================================================
// 函数名称: Param_0x24010004_Pump_Drain_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵排空标志_B
//============================================================================
void Param_0x24010004_Pump_Drain_B(void)
{
  static uint8_t index = 4;
	
	if (index == 4) 
	{
		index = 2; 
		Info_B(index);	
		SysRunData.FlowRateB = SysRunData.PumpPourIntoVelocityB;
    LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);		
	}	
	else
	{
		index = 4; 
		Info_B(index);	
		SysRunData.PumpPourIntoONOFF_B = 0;	
		SysRunData.PumpMotorSetSpeed_B = 0;
		Pump_SetSpeed_B(SysRunData.PumpMotorSetSpeed_B);			
	}
}
//============================================================================
// 函数名称: Param_0x2408_ToAndFro()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 往复
//============================================================================
void Param_0x2408_ToAndFro(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if ((SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3))
  {
	  SysSetParam[index].MotorModel = RECIPROCATING;
  }

  SysSetParam[index].ModeForward = 2; //3往复显
//  SysSetParam[index].ModeReciprocating = 4;
//  SysSetParam[index].ModeReverse = 1;

  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换  往复
		
////  if (SysSetParam[index].PumpOffOnB == ON)
////	{Info_B(1);}//  Screen_InformationBarImage_Update(1, 1, 1, 1);
////  else
////	{Info_B(4);}//  Screen_InformationBarImage_Update(1, 2, 1, 2);

////  if (SysSetParam[index].PumpOffOnA == ON)
////	{Info_A(1);}//  Screen_InformationBarImage_Update(1, 1, 1, 1);
////  else
////	{Info_A(4);}//  Screen_InformationBarImage_Update(1, 2, 1, 2);	
	
  SysSetParam[index].GearPositionHz = 1;  //频率
	Info_HZ(1);
}

//============================================================================
//往复、正向、反向切换挡位条的变化
uint8_t Param_GearPosition(void)
{
  uint8_t ret = 0;
  uint8_t index = SysInterface.BeSelectNum - 1;

  if (SysRunData.MotorSetSpeed <= SysSetParam[index].ISpeed)
    ret = 4;
  else if (SysRunData.MotorSetSpeed <= SysSetParam[index].IISpeed)
	  ret = 5;
  else
	  ret = 6;

  return ret;
}

//============================================================================
// 函数名称: Param_0x2407_ForwardDirection()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 正向
//============================================================================
void Param_0x2407_ForwardDirection(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if (SysSetParam[index].ReciprocatingFlag == 1)
  {
//  	if ((SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3))
	  {
	    SysSetParam[index].MotorModel = FORWARD;
	  }

	  SysSetParam[index].ModeForward = 1;//3正显
//	  SysSetParam[index].ModeReciprocating = 3;
//	  SysSetParam[index].ModeReverse = 1;

	  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换  往复

	  SysSetParam[index].GearPositionHz = Param_GearPosition(); //档位
    Info_HZ(SysSetParam[index].GearPositionHz);
 
  }
  else if (SysSetParam[index].ReciprocatingFlag == 0)//((SysSetParam[index].ReciprocatingFlag == 0) && (SysRunData.MotorNum != MotorNum1))
  {
//	  if ((SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3))
	  {
	    SysSetParam[index].MotorModel = FORWARD;
	  }

	  SysSetParam[index].ModeForward = 4; //2正显
//	  SysSetParam[index].ModeReciprocating = 0;
//	  SysSetParam[index].ModeReverse = 1;

	  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换  往复

	  SysSetParam[index].GearPositionHz = Param_GearPosition(); //档位

    Info_HZ(SysSetParam[index].GearPositionHz);
  }

  //默认 Ⅱ 档
  #if 0
  if ((SysSetParam[index].ReciprocatingFlag == 1) || ((SysSetParam[index].ReciprocatingFlag == 0) && (SysRunData.MotorNum != MotorNum1)))
  {
	  /*
	  switch (SysRunData.MotorNumber)
	  {
	    case 1 :
	    {
	      SysSetParam[index].StartMotorSpeed = SysSetParam[index].IISpeed;
        SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

		    SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
		    SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 2 :
	    {
	      SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

	      LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 3 :
	    {
	      SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    default : break;
	  }
	  */

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].IISpeed;
	  SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

	  SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
	  SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
	  SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
  #endif
}

//============================================================================
// 函数名称: Param_0x2412_OppositeDirection()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 反向
//============================================================================
void Param_0x2412_OppositeDirection(void)
{
  uint8_t index = SysInterface.BeSelectNum - 1;

  if (SysSetParam[index].ReciprocatingFlag == 1)
  {
//    if ((SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3))
	  {
	    SysSetParam[index].MotorModel = REVERSE;
	  }

	  SysSetParam[index].ModeForward = 3;//3反显
//	  SysSetParam[index].ModeReciprocating = 3;
//	  SysSetParam[index].ModeReverse = 2;

	  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换  往复

	  SysSetParam[index].GearPositionHz = Param_GearPosition(); //档位

    Info_HZ(SysSetParam[index].GearPositionHz);
  }
  else if (SysSetParam[index].ReciprocatingFlag == 0)//((SysSetParam[index].ReciprocatingFlag == 0) && (SysRunData.MotorNum != MotorNum1))
  {
//	  if ((SysRunData.MotorNum == MotorNum2) || (SysRunData.MotorNum == MotorNum3))
	  {
	    SysSetParam[index].MotorModel = REVERSE;
	  }

	  SysSetParam[index].ModeForward = 5;//2反显
//	  SysSetParam[index].ModeReciprocating = 0;
//	  SysSetParam[index].ModeReverse = 2;

	  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换  往复

	  SysSetParam[index].GearPositionHz = Param_GearPosition(); //档位

    Info_HZ(SysSetParam[index].GearPositionHz);
  }

  //默认 Ⅱ 档
  #if 0
  if ((SysSetParam[index].ReciprocatingFlag == 1) || ((SysSetParam[index].ReciprocatingFlag == 0) && (SysRunData.MotorNum != MotorNum1)))
  {
	  /*
	  switch (SysRunData.MotorNumber)
	  {
	    case 1 :
	    {
	      SysSetParam[index].StartMotorSpeed = SysSetParam[index].IISpeed;
        SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

		    SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
		    SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 2 :
	    {
	      SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

		    SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    case 3 :
	    {
	      SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;

	      SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	    }
	    break;
	    default : break;
	  }
	  */

	  SysSetParam[index].StartMotorSpeed = SysSetParam[index].IISpeed;
	  SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

	  SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
	  SysRunData.StartSetMotorSpeed = SysRunData.SetIISpeed;
	  SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;

	  LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
  }
  #endif
}

//============================================================================
// 函数名称: Param_0x2409_PedalSelect()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏选中
//============================================================================
void Param_0x2409_PedalSelect(void)
{
  Screen_FootPedalConnectState_Update(5); //脚踏控制图片

  SysSetParam[SysInterface.BeSelectNum-1].FootAndHandCtrl = FootCtrl;
  SysRunData.StartingMethod = FootCtrl; //脚踏控制

  SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;
}

//============================================================================
// 函数名称: Param_0x2410_FingerControl()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 手控选中
//============================================================================
void Param_0x2410_FingerControl(void)
{
  Screen_FootPedalConnectState_Update(4); //手柄控制图片

  SysSetParam[SysInterface.BeSelectNum-1].FootAndHandCtrl = ManualCtrl;
  SysRunData.StartingMethod = ManualCtrl; //手柄控制

  SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;
}

//============================================================================
// 函数名称: Param_0x2A03_LeftAngleSet()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 开口 左调整
//============================================================================
void Param_0x2A03_LeftAngleSet(void)
{
 
  SysRunData.Motor2StopTime = 0;

//  Delay_ms(20);

  BrushlessMotor_SetPosition(SysInterface.InterfaceSwitchNo2, 4, 1);

//  Delay_ms(20);
}

//============================================================================
// 函数名称: Param_0x2A04_RightAngleSet()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 开口 右调整
//============================================================================
void Param_0x2A04_RightAngleSet(void)
{
 
  SysRunData.Motor2StopTime = 0;

//  Delay_ms(20);

  BrushlessMotor_SetPosition(SysInterface.InterfaceSwitchNo2, 5, 1);

//  Delay_ms(20);
}




