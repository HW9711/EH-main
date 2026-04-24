//handledata.c


#include "handledata.h"
#include "data.h"
#include "eeprom.h"

//============================================================================
// 函数名称: HandleData_No1()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: No.1 【分离式，无霍尔（）】
//============================================================================
//void HandleData_No1(uint8_t Number, uint8_t *Param)
//{
//  SysRunData.MotorType = MOTORTYPE_EC16; //电机类型 电流大

//  //5:TMBA 6:EMBD 7:EMBC
//  if ((SysInterface.HandleType[Number] >= Handle_Type_5) && (SysInterface.HandleType[Number] <= Handle_Type_7))
//  {
//	  if (SysRunData.ParamInitFlag[Number] == 0)
//	  {
//	    //SysRunData.ParamInitFlag[Number] = 1;

//	    /*
//	    SysRunData.MaxSetMotorSpeed1 = 70000;
//	    SysRunData.StartSetMotorSpeed1 = 70000;
//	    SysRunData.MinSetMotorSpeed1 = 20000;

//	    SysRunData.SetISpeed1 = 20000;
//	    SysRunData.SetIISpeed1 = 50000;
//	    SysRunData.SetIIISpeed1 = 70000;

//	    SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
//	    */

//	    SysSetParam[Number].MaxMotorSpeed = 70000;
//	    SysSetParam[Number].StartMotorSpeed = 70000;
//	    SysSetParam[Number].MinMotorSpeed = 10000;

//	    SysSetParam[Number].ISpeed = 20000;
//	    SysSetParam[Number].IISpeed = 50000;
//	    SysSetParam[Number].IIISpeed = 70000;

//	    SysSetParam[Number].RunMotorSpeed = 70000;
//	  }

//	  SysRunData.MotorType = MOTORTYPE_EC13; //电机类型 电流小
//  }
//  //JMB 无霍尔
//  else if (SysInterface.HandleType[Number] == Handle_Type_2)
//  {
//	  if (SysRunData.ParamInitFlag[Number] == 0)
//	  {
//	    //SysRunData.ParamInitFlag[Number] = 1;

//	    /*
//	    SysRunData.MaxSetMotorSpeed1 = 30000;
//	    SysRunData.StartSetMotorSpeed1 = 30000;
//	    SysRunData.MinSetMotorSpeed1 = 10000;

//	    SysRunData.SetISpeed1 = 10000;
//	    SysRunData.SetIISpeed1 = 20000;
//	    SysRunData.SetIIISpeed1 = 30000;

//	    SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
//	    */
//	    SysSetParam[Number].MaxMotorSpeed = 30000;
//	    SysSetParam[Number].StartMotorSpeed = 30000;
//	    SysSetParam[Number].MinMotorSpeed = 10000;

//	    SysSetParam[Number].ISpeed = 10000;
//	    SysSetParam[Number].IISpeed = 20000;
//	    SysSetParam[Number].IIISpeed = 30000;

//	    SysSetParam[Number].RunMotorSpeed = 30000;
//	  }
//  }
//  //TMBB TMBC（未生产）
//  else
//  {
//	  if (SysRunData.ParamInitFlag[Number] == 0)
//	  {
//	    //SysRunData.ParamInitFlag[Number] = 1;

//	    /*
//	    SysRunData.MaxSetMotorSpeed1 = 60000;
//	    SysRunData.StartSetMotorSpeed1 = 60000;
//	    SysRunData.MinSetMotorSpeed1 = 10000;  //20000

//	    SysRunData.SetISpeed1 = 20000;
//	    SysRunData.SetIISpeed1 = 50000;
//	    SysRunData.SetIIISpeed1 = 60000;

//	    SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
//	    */

//	    SysSetParam[Number].MaxMotorSpeed = 60000;
//	    SysSetParam[Number].StartMotorSpeed = 60000;
//	    SysSetParam[Number].MinMotorSpeed = 10000;  //20000

//	    SysSetParam[Number].ISpeed = 20000;
//	    SysSetParam[Number].IISpeed = 40000;
//	    SysSetParam[Number].IIISpeed = 60000;

//	    SysSetParam[Number].RunMotorSpeed = 60000;
//	  }
//  }

//  //该接口相同的初值
//  if (SysRunData.ParamInitFlag[Number] == 0)
//  {
//	  SysRunData.ParamInitFlag[Number] = 1;

//	  SysSetParam[Number].HzSet = 0;

//	  EEPROM_AT24CXX_Read(0x21, &SysRunData.PumpFlowEEPOM, 1);   //泵流速 读取
//	  SysSetParam[Number].PumpVelocitySetB = SysRunData.PumpFlowEEPOM; //流速的设定

//	  SysSetParam[Number].PumpOffOnB = 1;  //泵开关

//    SysSetParam[Number].GearPositionHz = 6;  //挡位 频率栏显示状态

//	  SysSetParam[Number].FootAndHandCtrl = 0;  //脚控

//	  SysSetParam[Number].ReciprocatingFlag = 0;
//	  SysSetParam[Number].MotorModel = 2;
////	  SysSetParam[Number].ForwardReverseFlag = 0;

//	  SysSetParam[Number].ModeReciprocating = 0;
//	  SysSetParam[Number].ModeForward = 2;
//	  SysSetParam[Number].ModeReverse = 0;
//  }

//  //********************************
//  SysRunData.Frequency = SysSetParam[Number].HzSet;      //频率的设定
//  //********************************
//  SysRunData.FlowRateB = SysSetParam[Number].PumpVelocitySetB; //流速的设定
//  //********************************
//  SysRunData.MaxSetMotorSpeed = SysSetParam[Number].MaxMotorSpeed;
//  SysRunData.StartSetMotorSpeed = SysSetParam[Number].StartMotorSpeed;
//  SysRunData.MinSetMotorSpeed = SysSetParam[Number].MinMotorSpeed;

//  SysRunData.SetISpeed = SysSetParam[Number].ISpeed;
//  SysRunData.SetIISpeed = SysSetParam[Number].IISpeed;
//  SysRunData.SetIIISpeed = SysSetParam[Number].IIISpeed;

//  SysRunData.MotorSetSpeed = SysSetParam[Number].RunMotorSpeed;
//  //********************************
//  SysRunData.PumpONOFF_B = SysSetParam[Number].PumpOffOnB;
//  //********************************

//  //********************************
//  Param[3] = 1; //DisplayInterfaceData.State_Info1 = 1; //信息状态转速

//  //信息状态流量
//  if (SysRunData.PumpONOFF_B == 1)
//  {
//	  Param[4] = 1; //DisplayInterfaceData.State_Info2 = 1; 显
//  }
//  else
//  {
//	  Param[4] = 4;  //DisplayInterfaceData.State_Info2 = 2; 隐
//  }

//  Param[5] = SysSetParam[Number].GearPositionHz;  //DisplayInterfaceData.State_Info3 = 6; //信息状态频率
//  //********************************

//  Param[0] = 0;  //DisplayInterfaceData.State_diam   = 0; //直径
//  Param[1] = 0;  //DisplayInterfaceData.State_length = 0; //长度
//  Param[2] = 0;  //DisplayInterfaceData.State_angle  = 0; //角度

//  Param[6] = 0;  //DisplayInterfaceData.State_Cutter1 = 0;  //刀具图片   (3号接口)
//  Param[7] = 0;  //DisplayInterfaceData.State_Cutter2 = 0;  //刀具信息   (3号接口)
//  //********************************
//  //********************************
//  if (SysFootPedalData.FootPedalConnectOkNo != Connect)
//	  Param[8] = 1;  //DisplayInterfaceData.State_FootPedal = 1;   //脚控 0不显示脚控/手控, 1脚控未连接 未选中, 2脚控 已选中 选中, 3手控 已选中 选中, 4脚控已连接\手控已选中, 5脚控已选中\手控已连接
//  else
//	  Param[8] = 2;  //DisplayInterfaceData.State_FootPedal = 2;

//  SysRunData.StartingMethod = SysSetParam[Number].FootAndHandCtrl; //脚踏控制
//  //********************************

////					W_UART;     //串口切换(1号接口电机通信串口)

//  SysRunData.MotorNum = MotorNum1;       //1号接口手柄

////	SysRunData.PWMValue = 700;        //PWM输出处置（两极输出）

//  SysRunData.BeepTimeMS = 100;
//}

//============================================================================
// 函数名称: HandleData_No2()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: No.2 【分离式，有霍尔（PXBA、PXBB）】
//============================================================================
void HandleData_No2(uint8_t Number, uint8_t *Param)
{
  uint8_t temp = 0;

  SysRunData.MotorType = MOTORTYPE_EC13;

  temp = Number ? 4 : 1;

  if (SysRunData.ParamInitFlag[temp] == 0)
  {
	  SysRunData.ParamInitFlag[temp] = 1;

	  //刀具连接判断
	  if (SysRunData.CutterON == 1)   //刀具连接
	  {
	    if ((SysHandleData.EPCBuffAA[Number][5] & 0xf0) ==  0x30)
	    {
		    /*
	      SysRunData.DXWFFlag = 1;

		    SysRunData.MotorModel2 = 1;  //直流电机 单向-往复切换  0单向  1往复

		    SysRunData.StateDX = 3;     //单向
		    SysRunData.StateWF = 4;     //往复

		    Param[5] = 1;  //DisplayInterfaceData.State_Info3 = 1; //信息状态频率

		    SysRunData.MinSetMotorSpeed = 500;
		    */
		    SysSetParam[temp].DJSetPDMT = 0;

		    SysSetParam[temp].ReciprocatingFlag = 1;
		    SysSetParam[temp].MotorModel = 1;
//		  SysSetParam[temp].ForwardReverseFlag = 0;

		    SysSetParam[temp].ModeForward = 2;//3往复显
//		    SysSetParam[temp].ModeReciprocating = 4;
//		    SysSetParam[temp].ModeReverse = 1;

		    SysSetParam[temp].GearPositionHz = 1;

		    SysSetParam[temp].MinMotorSpeed = 500;
	    }
	    else
	    {
		    /*
		    SysRunData.DXWFFlag = 0;

		    SysRunData.MotorModel2 = 0;  //直流电机 单向-往复切换  0单向  1往复

		    SysRunData.StateDX = 4;     //单向
		    SysRunData.StateWF = 0;     //往复

		    Param[5] = 5;  //DisplayInterfaceData.State_Info3 = 5; //信息状态频率

		    SysRunData.MinSetMotorSpeed = 3000;
		    */
		    SysSetParam[temp].DJSetPDMT = 1;

		    SysSetParam[temp].ReciprocatingFlag = 0;
		    SysSetParam[temp].MotorModel = 2;  //正向
//		SysSetParam[temp].ForwardReverseFlag = 0;

		    SysSetParam[temp].ModeForward = 4;//2正显
//		    SysSetParam[temp].ModeReciprocating = 0;
//		    SysSetParam[temp].ModeReverse = 1;

		    SysSetParam[temp].GearPositionHz = 5;

		    SysSetParam[temp].MinMotorSpeed = 3000;
	    }

	    SysSetParam[temp].MaxMotorSpeed = SysHandleData.EPCBuffAA[Number][8] * 500;
	    SysSetParam[temp].StartMotorSpeed = SysHandleData.EPCBuffAA[Number][9] * 500;
	  }
	  else
	  {
	    if (SysRunData.DJOldValueFlag[Number] == 0)  // 0 刨  1 磨
	    {
		    /*
	      SysRunData.DXWFFlag = 1;

		    SysRunData.MotorModel2 = 1;  //直流电机 单向-往复切换  0单向  1往复

		    SysRunData.StateDX = 3;     //单向
		    SysRunData.StateWF = 4;     //往复

		    Param[5] = 1;  //DisplayInterfaceData.State_Info3 = 1; //信息状态频率

		    SysRunData.MaxSetMotorSpeed = 6000;
		    SysRunData.StartSetMotorSpeed = 4000;
		    SysRunData.MinSetMotorSpeed = 500;
		    */
		    SysSetParam[temp].DJSetPDMT = 0;

		    SysSetParam[temp].ReciprocatingFlag = 1;
		    SysSetParam[temp].MotorModel = 1;
//		SysSetParam[temp].ForwardReverseFlag = 0;

		    SysSetParam[temp].ModeForward = 2;//3往复显
//		    SysSetParam[temp].ModeReciprocating = 4;
//		    SysSetParam[temp].ModeReverse = 1;

		    SysSetParam[temp].GearPositionHz = 1;

        SysSetParam[temp].MaxMotorSpeed = 6000;
		    SysSetParam[temp].StartMotorSpeed = 4000;
		    SysSetParam[temp].MinMotorSpeed = 500;
	    }
	    else
	    {
		    /*
		    SysRunData.DXWFFlag = 0;

		    SysRunData.MotorModel2 = 0;

		    SysRunData.StateDX = 4;     //单向
		    SysRunData.StateWF = 0;     //往复

		    Param[5] = 5;  //DisplayInterfaceData.State_Info3 = 5; //信息状态频率

		    SysRunData.MaxSetMotorSpeed = 13000;
		    SysRunData.StartSetMotorSpeed = 10000;
		    SysRunData.MinSetMotorSpeed = 3000;
		    */

		    SysSetParam[temp].DJSetPDMT = 1;

		    SysSetParam[temp].ReciprocatingFlag = 0;
		    SysSetParam[temp].MotorModel = 2;  //正
//		SysSetParam[temp].ForwardReverseFlag = 0;

		    SysSetParam[temp].ModeForward = 4;//2正显
//		    SysSetParam[temp].ModeReciprocating = 0;
//		    SysSetParam[temp].ModeReverse = 1;

		    SysSetParam[temp].GearPositionHz = 5;

        SysSetParam[temp].MaxMotorSpeed = 13000;
		    SysSetParam[temp].StartMotorSpeed = 10000;
		    SysSetParam[temp].MinMotorSpeed = 3000;
	    }
	  }

	  SysSetParam[temp].ISpeed = SysSetParam[temp].MinMotorSpeed;
	  SysSetParam[temp].IISpeed = SysSetParam[temp].StartMotorSpeed;
	  SysSetParam[temp].IIISpeed = SysSetParam[temp].MaxMotorSpeed;

	  SysSetParam[temp].RunMotorSpeed = SysSetParam[temp].StartMotorSpeed;

	  //********************************
	  SysSetParam[temp].HzSet = SysHandleData.EPCBuffAA[Number][10];      //频率的设定
	  if (SysSetParam[temp].HzSet > FREQUENCYMAX)
	    SysSetParam[temp].HzSet = FREQUENCYMAX;

		if (SysModelConfig.HandlePortA == 1)
		{
			SysSetParam[temp].PumpVelocitySetA = SysHandleData.EPCBuffAA[Number][11];   //流速的设定
			
			#ifdef WATER_UPTAKE
				if (SysSetParam[temp].PumpVelocitySetA > 15)
				SysSetParam[temp].PumpVelocitySetA = 15;
			#else
				if (SysSetParam[temp].PumpVelocitySetA > PUMPMLUNITMAX)
				SysSetParam[temp].PumpVelocitySetA = PUMPMLUNITMAX;
			#endif		
		}
		
		if (SysModelConfig.HandlePortB == 1)
		{
			SysSetParam[temp].PumpVelocitySetB = SysHandleData.EPCBuffAA[Number][11];   //流速的设定
			
			#ifdef WATER_UPTAKE
				if (SysSetParam[temp].PumpVelocitySetB > 15)
				SysSetParam[temp].PumpVelocitySetB = 15;
			#else
				if (SysSetParam[temp].PumpVelocitySetB > PUMPMLUNITMAX)
				SysSetParam[temp].PumpVelocitySetB = PUMPMLUNITMAX;
			#endif		
		}		
		
 
	  SysSetParam[temp].HzSet = FREQUENCYMAX;      //频率的设定

	  //SysSetParam[temp].PumpVelocitySetB = 30;//SSC


	  SysSetParam[temp].FootAndHandCtrl = 0;

	  SysSetParam[temp].DJAutoGetFlag = 0;  //默认”自动设别“刀具模式

    /*
	  //更新刀具信息
	  SysRunData.DJManualRefreshFlag = 1;

	  SysRunData.CutterOFF = 0;
	  SysRunData.ReciveOKTime = 120;
	  */
  }
  else
  {
	  SysRunData.CutterOFF = 0;
	  SysRunData.ReciveOKTime = 80;
  }

  SysRunData.CutterON = 0;  //接上 即使刀具信息与上一次刀具信息一样也允许更新一次参数

  //********************************
  SysRunData.Frequency = SysSetParam[temp].HzSet;      //频率的设定
  //********************************
  SysRunData.FlowRateB = SysSetParam[temp].PumpVelocitySetB; //流速的设定
  SysRunData.FlowRateA = SysSetParam[temp].PumpVelocitySetA; //流速的设定	
	
  //********************************
  SysRunData.MaxSetMotorSpeed = SysSetParam[temp].MaxMotorSpeed;
  SysRunData.StartSetMotorSpeed = SysSetParam[temp].StartMotorSpeed;
  SysRunData.MinSetMotorSpeed = SysSetParam[temp].MinMotorSpeed;

  SysRunData.SetISpeed = SysSetParam[temp].ISpeed;
  SysRunData.SetIISpeed = SysSetParam[temp].IISpeed;
  SysRunData.SetIIISpeed = SysSetParam[temp].IIISpeed;

  SysRunData.MotorSetSpeed = SysSetParam[temp].RunMotorSpeed;
  //********************************
 
  //********************************
//  SysRunData.DXWFFlag = SysSetParam[temp].ReciprocatingFlag;  //单向往复标志 0：单向 1：往复
//  SysRunData.MotorModel2 = (SysSetParam[temp].MotorModel == 1) ? 1 : 0;  //直流电机 单向-往复切换  0单向  1往复
//	SysRunData.CWCCW = SysSetParam[temp].ForwardReverseFlag;  //0正向、1反向
	//********************************

  //********************************
  Param[3] = 1;  //DisplayInterfaceData.State_Info1 = 1; //信息状态转速

  //信息状态流量
  if (SysModelConfig.HandlePortA == 1)
	{
		Param[4] = 4;  //DisplayInterfaceData.State_Info2 = 2; 显
	  Param[9] = 1;  //DisplayInterfaceData.State_Info2 = 2; 隐//流量1
		
    SysSetParam[Number].PumpOffOnA = 1;  //A泵开关
		//********************************
		SysRunData.PumpONOFF_A = SysSetParam[Number].PumpOffOnA;
		//********************************	
    SysSetParam[Number].PumpOffOnB = 0;  //B泵开关
		//********************************
		SysRunData.PumpONOFF_B = SysSetParam[Number].PumpOffOnB;
		//********************************	 
	}

  //信息状态流量
  if (SysModelConfig.HandlePortB == 1)
	{
		Param[4] = 1; //DisplayInterfaceData.State_Info2 = 1; 显
		Param[9] = 4;  //DisplayInterfaceData.State_Info2 = 2; 隐//流量1
		
    SysSetParam[Number].PumpOffOnB = 1;  //B泵开关
		//********************************
		SysRunData.PumpONOFF_B = SysSetParam[Number].PumpOffOnB;
		//********************************				
    SysSetParam[Number].PumpOffOnA = 0;  //A泵开关
		//********************************
		SysRunData.PumpONOFF_A = SysSetParam[Number].PumpOffOnA;
		//********************************			
	}

 
  Param[5] = SysSetParam[temp].GearPositionHz;  //DisplayInterfaceData.State_Info3 = 6; //信息状态频率
  //********************************

  //********************************
  Param[0] = SysHandleData.EPCBuffAA[Number][1];  //DisplayInterfaceData.State_diam   = EPC_Buff[1]; //直径
  Param[1] = SysHandleData.EPCBuffAA[Number][2];  //DisplayInterfaceData.State_length = EPC_Buff[2]; //长度
  Param[2] = SysHandleData.EPCBuffAA[Number][3];  //DisplayInterfaceData.State_angle  = EPC_Buff[3]; //角度

  if (SysHandleData.EPCBuffAA[Number][3] == 0)
  {
	  Param[6] = 0;  //DisplayInterfaceData.State_Cutter1 = 0;  //刀具图片   (3号接口)
	  Param[7] = 3;  //DisplayInterfaceData.State_Cutter2 = 3;  //刀具信息   (3号接口)
  }
  else
  {
	  Param[6] = 0;  //DisplayInterfaceData.State_Cutter1 = 0;  //刀具图片   (3号接口)
	  Param[7] = 2;  //DisplayInterfaceData.State_Cutter2 = 2;  //刀具信息   (3号接口)
  }
  //********************************

  //********************************
  //脚控判断
  if (SysInterface.HandleType[temp] == Handle_Type_22)  //PXBA
  {
	  if (SysFootPedalData.FootPedalConnectOkNo != Connect)  //脚踏未连接
	  {
	    Param[8] = 3;  //DisplayInterfaceData.State_FootPedal = 3;   //脚控 0不显示脚控/手控, 1脚控未连接 未选中, 2脚控 已选中 选中, 3手控 已选中 选中, 4脚控已连接\手控已选中, 5脚控已选中\手控已连接
  	  SysSetParam[temp].FootAndHandCtrl = 1;
	    SysRunData.StartingMethod = SysSetParam[temp].FootAndHandCtrl;  //手按控制
	  }
	  else
	  {
	    if (SysSetParam[temp].FootAndHandCtrl)
		    Param[8] = 4;  //DisplayInterfaceData.State_FootPedal = 5;
  	  else
		    Param[8] = 5;  //DisplayInterfaceData.State_FootPedal = 5;

	    SysRunData.StartingMethod = SysSetParam[temp].FootAndHandCtrl;  //脚踏控制
	  }
  }
  else
  {
	  if (SysFootPedalData.FootPedalConnectOkNo != Connect)
	    Param[8] = 1;  //DisplayInterfaceData.State_FootPedal = 1;
	  else
	    Param[8] = 2;  //DisplayInterfaceData.State_FootPedal = 2;

	  SysRunData.StartingMethod = SysSetParam[temp].FootAndHandCtrl;  //脚踏控制
  }
  //********************************

  //				  H_UART;     //串口切换(2号接口电机通信串口)

  SysRunData.MotorNum = MotorNum2;       //电机号

  SysRunData.BeepTimeMS = 100;

  SysRunData.SystemTime = 0;  //刀具设别蜂鸣器响判断
}

//============================================================================
// 函数名称: Interface_No3()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: No.3 【一体式】
//============================================================================
void HandleData_No3(uint8_t Number, uint8_t *Param)
{
  uint8_t temp = 0;

  SysRunData.MotorType = MOTORTYPE_EC100;

  temp = Number ? 3 : 2;

  if (SysRunData.ParamInitFlag[temp] == 0)
  {
	  SysRunData.ParamInitFlag[temp] = 1;

	  if ((SysHandleData.Ds2431BuffBB[Number][5] & 0xf0) == 0x30)
	  {
	    /*
	    SysRunData.DXWFFlag = 1;

	    SysRunData.MotorModel3 = 1;  //直流电机 单向-往复切换  0单向  1往复

	    SysRunData.StateDX = 3;     //单向
	    SysRunData.StateWF = 4;     //往复

	    Param[5] = 1;  //DisplayInterfaceData.State_Info3 = 1;  //信息状态频率

	    SysRunData.MinSetMotorSpeed = 500;
	    */
	    SysSetParam[temp].ReciprocatingFlag = 1;  //DXWFFlag
  	  SysSetParam[temp].MotorModel = 1;  //1：往复 2：正向 3：反向  MotorModel3
//		SysSetParam[temp].ForwardReverseFlag = 0;  //CWCCW

	    SysSetParam[temp].ModeForward = 2;  //3往复显
//	    SysSetParam[temp].ModeReciprocating = 4;  //往复StateWF
//	    SysSetParam[temp].ModeReverse = 1;  //反向

	    SysSetParam[temp].GearPositionHz = 1;    //挡位 频率栏显示状态

	    SysSetParam[temp].MinMotorSpeed = 500;  //最小转速
	  }
	  else
	  {
	    /*
	    SysRunData.DXWFFlag = 0;

	    SysRunData.MotorModel3 = 0;

	    SysRunData.StateDX = 4;     //单向
	    SysRunData.StateWF = 0;     //往复

	    Param[5] = 5;  //DisplayInterfaceData.State_Info3 = 5;  //信息状态频率

	    SysRunData.MinSetMotorSpeed = 3000;
	    */
	    SysSetParam[temp].ReciprocatingFlag = 0;  //DXWFFlag
	    SysSetParam[temp].MotorModel = 2;  //1：往复 2：正向 3：反向  MotorModel3
//		SysSetParam[temp].ForwardReverseFlag = 0;  //CWCCW

	    SysSetParam[temp].ModeForward = 4;  //2正显
//	    SysSetParam[temp].ModeReciprocating = 0;  //往复  StateWF
//	    SysSetParam[temp].ModeReverse = 1;  //反向

	    SysSetParam[temp].GearPositionHz = 5;    //挡位 频率栏显示状态

	    SysSetParam[temp].MinMotorSpeed = 3000;  //最小转速
    }

	  SysSetParam[temp].MaxMotorSpeed = SysHandleData.Ds2431BuffBB[Number][8] * 500;
	  SysSetParam[temp].StartMotorSpeed = SysHandleData.Ds2431BuffBB[Number][9] * 500;

    if (SysSetParam[temp].StartMotorSpeed > SysSetParam[temp].MaxMotorSpeed)
	    SysSetParam[temp].StartMotorSpeed = SysSetParam[temp].MaxMotorSpeed;

	  SysSetParam[temp].ISpeed = SysSetParam[temp].MinMotorSpeed;
	  SysSetParam[temp].IISpeed = SysSetParam[temp].StartMotorSpeed;
	  SysSetParam[temp].IIISpeed = SysSetParam[temp].MaxMotorSpeed;

	  SysSetParam[temp].RunMotorSpeed = SysSetParam[temp].StartMotorSpeed;

	  SysSetParam[temp].HzSet = SysHandleData.Ds2431BuffBB[Number][10];      //频率的设定
	  if (SysSetParam[temp].HzSet > FREQUENCYMAX)
	    SysSetParam[temp].HzSet = FREQUENCYMAX;

    if (SysModelConfig.HandlePortA == 1)
		{
			SysSetParam[temp].PumpVelocitySetA = SysHandleData.Ds2431BuffBB[Number][11];   //流速的设定
			#ifdef WATER_UPTAKE
			 if (SysSetParam[temp].PumpVelocitySetA > 15)
				SysSetParam[temp].PumpVelocitySetA = 15;
			#else 
			 if (SysSetParam[temp].PumpVelocitySetA > PUMPMLUNITMAX)
				SysSetParam[temp].PumpVelocitySetA = PUMPMLUNITMAX;
			#endif

		}
		
    if (SysModelConfig.HandlePortB == 1)
		{
			SysSetParam[temp].PumpVelocitySetB = SysHandleData.Ds2431BuffBB[Number][11];   //流速的设定
			#ifdef WATER_UPTAKE
			 if (SysSetParam[temp].PumpVelocitySetB > 15)
				SysSetParam[temp].PumpVelocitySetB = 15;
			#else 
			 if (SysSetParam[temp].PumpVelocitySetB > PUMPMLUNITMAX)
				SysSetParam[temp].PumpVelocitySetB = PUMPMLUNITMAX;
			#endif

  
		}		
	  SysSetParam[temp].FootAndHandCtrl = 0;  //脚控【注：每次插上手柄默认脚控】

  }

  //********************************
  SysRunData.Frequency = SysSetParam[temp].HzSet;      //频率的设定
  //********************************

	if (SysModelConfig.HandlePortA == 1)
	{	
		SysSetParam[temp].PumpOffOnA = 1;  //泵开关【注：每次插上手柄默认开启】	
		
		SysRunData.FlowRateA = SysSetParam[temp].PumpVelocitySetA; //流速的设定	
		SysRunData.PumpONOFF_A = SysSetParam[temp].PumpOffOnA;	
		
		//********************************	
    SysSetParam[temp].PumpOffOnB = 0;  //B泵开关
		//********************************
		SysRunData.PumpONOFF_B = SysSetParam[temp].PumpOffOnB;
		//********************************	 	
		
		
		//泵打开与否判断
		if (SysRunData.PumpONOFF_A == 1)
		{
			Param[9] = 1;  //DisplayInterfaceData.State_Info2 = 1;  //信息状态流量
		}
		else
		{
			Param[9] = 4;  //DisplayInterfaceData.State_Info2 = 2;
		}	
		Param[4] = 4;  //DisplayInterfaceData.State_Info2 = 2;		
	}
	if (SysModelConfig.HandlePortB == 1)
	{	
		SysSetParam[temp].PumpOffOnB = 1;  //泵开关【注：每次插上手柄默认开启】
 
		SysRunData.FlowRateB = SysSetParam[temp].PumpVelocitySetB; //流速的设定
		SysRunData.PumpONOFF_B = SysSetParam[temp].PumpOffOnB;		
		
		SysSetParam[temp].PumpOffOnA = 0;  //A泵开关
		//********************************
		SysRunData.PumpONOFF_A = SysSetParam[temp].PumpOffOnA;
		//********************************	
    
  		
		
		//泵打开与否判断
		if (SysRunData.PumpONOFF_B == 1)
		{
			Param[4] = 1;  //DisplayInterfaceData.State_Info2 = 1;  //信息状态流量
		}
		else
		{
			Param[4] = 4;  //DisplayInterfaceData.State_Info2 = 2;
		}	
		Param[9] = 4;  //DisplayInterfaceData.State_Info2 = 2; 隐//流量1		
	} 

  //********************************
  SysRunData.MaxSetMotorSpeed = SysSetParam[temp].MaxMotorSpeed;
  SysRunData.StartSetMotorSpeed = SysSetParam[temp].StartMotorSpeed;
  SysRunData.MinSetMotorSpeed = SysSetParam[temp].MinMotorSpeed;

  SysRunData.SetISpeed = SysSetParam[temp].ISpeed;
  SysRunData.SetIISpeed = SysSetParam[temp].IISpeed;
  SysRunData.SetIIISpeed = SysSetParam[temp].IIISpeed;

  SysRunData.MotorSetSpeed = SysSetParam[temp].RunMotorSpeed;
  //********************************

  //********************************
//  SysRunData.DXWFFlag = SysSetParam[temp].ReciprocatingFlag;  //单向往复标志 0：单向 1：往复
//  SysRunData.MotorModel3 = (SysSetParam[temp].MotorModel == 1) ? 1 : 0;  //直流电机 单向-往复切换  0单向  1往复
//	SysRunData.CWCCW = SysSetParam[temp].ForwardReverseFlag;  //0正向、1反向
  //********************************
  Param[3] = 1;  //DisplayInterfaceData.State_Info1 = 1; //信息状态转速


  Param[5] = SysSetParam[temp].GearPositionHz;  //DisplayInterfaceData.State_Info3 = 6; //信息状态频率

  //********************************

  //********************************
  Param[0] = SysHandleData.Ds2431BuffBB[Number][1];  //DisplayInterfaceData.State_diam   = SysIntegratedData.Ds2431Buff[0][1];  //直径
  Param[1] = SysHandleData.Ds2431BuffBB[Number][2];  //DisplayInterfaceData.State_length = SysIntegratedData.Ds2431Buff[0][2];  //长度
  Param[2] = SysHandleData.Ds2431BuffBB[Number][3];  //DisplayInterfaceData.State_angle  = SysIntegratedData.Ds2431Buff[0][3];  //角度

  Param[6] = 1;  //DisplayInterfaceData.State_Cutter1 = 1;  //刀具图片   (3号接口)
  Param[7] = 0;  //DisplayInterfaceData.State_Cutter2 = 0;  //刀具信息   (3号接口)
  //********************************

  //********************************
  if (SysFootPedalData.FootPedalConnectOkNo != Connect)
	  Param[8] = 1;  //DisplayInterfaceData.State_FootPedal = 1;   //脚控 0不显示脚控/手控, 1脚控未连接 未选中, 2脚控 已选中 选中, 3手控 已选中 选中, 4脚控已连接\手控已选中, 5脚控已选中\手控已连接
  else
	  Param[8] = 2;  //DisplayInterfaceData.State_FootPedal = 2;

  SysRunData.StartingMethod = SysSetParam[temp].FootAndHandCtrl;  //脚踏控制
  //********************************

  //					No3Switch;
  //          S_UART;     //串口切换(3号接口电机通信串口)

  SysRunData.MotorNum = MotorNum3;       //手柄号

  SysRunData.BeepTimeMS = 100;
}

//============================================================================
// 函数名称: HandleData_No21()
// 功能描述: 
// 输　  入: 
// 输    出: 
// 函数说明: No.2-1（14芯设别原本属于8芯设别的手柄）
//============================================================================
void HandleData_No21(uint8_t Number, uint8_t *Param)
{
	SysRunData.MotorType = MOTORTYPE_EC16; //电机类型 电流大

	//TMBA（EMBA EMBB）(5)
	if (SysInterface.HandleType[Number] == Handle_Type_5)
	{
		if (SysRunData.ParamInitFlag[Number] == 0)
		{
			//SysRunData.Motor1SpeedEEPOMFlag[temp] = 1;

			/*
			SysRunData.MaxSetMotorSpeed1 = 70000;
			SysRunData.StartSetMotorSpeed1 = 70000;
			SysRunData.MinSetMotorSpeed1 = 20000;

			SysRunData.SetISpeed1 = 20000;
			SysRunData.SetIISpeed1 = 50000;
			SysRunData.SetIIISpeed1 = 70000;

			SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
			*/

	    SysSetParam[Number].MaxMotorSpeed = 70000;
	    SysSetParam[Number].StartMotorSpeed = 70000;
	    SysSetParam[Number].MinMotorSpeed = 10000;

	    SysSetParam[Number].ISpeed = 20000;
	    SysSetParam[Number].IISpeed = 50000;
	    SysSetParam[Number].IIISpeed = 70000;

	    SysSetParam[Number].RunMotorSpeed = 70000;
		}

		SysRunData.MotorType = MOTORTYPE_EC13; //电机类型 电流小
	}
	//JMB(7) 无霍尔
	else if (SysInterface.HandleType[Number] == Handle_Type_2)
	{
		if (SysRunData.ParamInitFlag[Number] == 0)
		{
			//SysRunData.Motor1SpeedEEPOMFlag[temp] = 1;
			/*
			SysRunData.MaxSetMotorSpeed1 = 30000;
			SysRunData.StartSetMotorSpeed1 = 30000;
			SysRunData.MinSetMotorSpeed1 = 10000;

			SysRunData.SetISpeed1 = 10000;
			SysRunData.SetIISpeed1 = 20000;
			SysRunData.SetIIISpeed1 = 30000;

			SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
			*/

	    SysSetParam[Number].MaxMotorSpeed = 60000;
	    SysSetParam[Number].StartMotorSpeed = 60000;
	    SysSetParam[Number].MinMotorSpeed = 5000;

	    SysSetParam[Number].ISpeed = 10000;
	    SysSetParam[Number].IISpeed = 40000;
	    SysSetParam[Number].IIISpeed = 60000;

	    SysSetParam[Number].RunMotorSpeed = 60000;
		}
	}
	else if(SysInterface.HandleType[Number] == Handle_Type_4)//TMBC(4)（未生产）
	{
		if (SysRunData.ParamInitFlag[Number] == 0)
		{
				SysSetParam[Number].MaxMotorSpeed = 60000;
				SysSetParam[Number].StartMotorSpeed = 60000;
				SysSetParam[Number].MinMotorSpeed = 10000;  //20000

				SysSetParam[Number].ISpeed = 20000;
				SysSetParam[Number].IISpeed = 40000;
				SysSetParam[Number].IIISpeed = 60000;

				SysSetParam[Number].RunMotorSpeed = 60000;
		}
	}
	else																//TMBB(3) 
	{
		if (SysRunData.ParamInitFlag[Number] == 0)
		{
			//SysRunData.Motor1SpeedEEPOMFlag[temp] = 1;

			/*
			SysRunData.MaxSetMotorSpeed1 = 60000;
			SysRunData.StartSetMotorSpeed1 = 60000;
			SysRunData.MinSetMotorSpeed1 = 10000;  //20000

			SysRunData.SetISpeed1 = 20000;
			SysRunData.SetIISpeed1 = 50000;
			SysRunData.SetIIISpeed1 = 60000;

			SysRunData.MotorSetSpeed1 = SysRunData.StartSetMotorSpeed1;
			*/

	    SysSetParam[Number].MaxMotorSpeed = 60000;
	    SysSetParam[Number].StartMotorSpeed = 60000;
	    SysSetParam[Number].MinMotorSpeed = 10000;  //20000

	    SysSetParam[Number].ISpeed = 20000;
	    SysSetParam[Number].IISpeed = 40000;
	    SysSetParam[Number].IIISpeed = 60000;

	    SysSetParam[Number].RunMotorSpeed = 60000;
		}
	}

  //该接口相同的初值
  if (SysRunData.ParamInitFlag[Number] == 0)
  {
	  SysRunData.ParamInitFlag[Number] = 1;

	  SysSetParam[Number].HzSet = 0;

		EEPROM_AT24CXX_Read(0x23, &SysRunData.PumpFlowEEPOM, 1);   //泵流速 读取
		SysSetParam[Number].PumpVelocitySetB = SysRunData.PumpFlowEEPOM; //流速的设定
		EEPROM_AT24CXX_Read(0x25, &SysRunData.PumpFlowEEPOM, 1);   //泵流速 读取
		SysSetParam[Number].PumpVelocitySetA = SysRunData.PumpFlowEEPOM; //流速的设定
		
    SysSetParam[Number].GearPositionHz = 6;  //挡位 频率栏显示状态

	  SysSetParam[Number].FootAndHandCtrl = 0;  //脚控

	  SysSetParam[Number].ReciprocatingFlag = 0;
	  SysSetParam[Number].MotorModel = 2;
//	  SysSetParam[Number].ForwardReverseFlag = 0;
		
		if(SysInterface.HandleType[Number] == Handle_Type_4)
		{
			SysSetParam[Number].ModeForward = 6; //  往复单独
		}
		else
		{
			SysSetParam[Number].ModeForward = 4; //2正显  磨单独
		}
//	  SysSetParam[Number].ModeReciprocating = 0;
//	  SysSetParam[Number].ModeReverse = 0;
  }

	

		
	 	
	
  //********************************
  SysRunData.Frequency = SysSetParam[Number].HzSet;      //频率的设定
  //********************************
  SysRunData.FlowRateB = SysSetParam[Number].PumpVelocitySetB; //B流速的设定
  SysRunData.FlowRateA = SysSetParam[Number].PumpVelocitySetA; //A流速的设定	
	
  //********************************
  SysRunData.MaxSetMotorSpeed = SysSetParam[Number].MaxMotorSpeed;
  SysRunData.StartSetMotorSpeed = SysSetParam[Number].StartMotorSpeed;
  SysRunData.MinSetMotorSpeed = SysSetParam[Number].MinMotorSpeed;

  SysRunData.SetISpeed = SysSetParam[Number].ISpeed;
  SysRunData.SetIISpeed = SysSetParam[Number].IISpeed;
  SysRunData.SetIIISpeed = SysSetParam[Number].IIISpeed;

  SysRunData.MotorSetSpeed = SysSetParam[Number].RunMotorSpeed;
 

  //********************************
  Param[3] = 1; //DisplayInterfaceData.State_Info1 = 1; //信息状态转速
	
  if (SysModelConfig.HandlePortA == 1)
	{
		Param[4] = 4;  //DisplayInterfaceData.State_Info2 = 2; 显
	  Param[9] = 1;  //DisplayInterfaceData.State_Info2 = 2; 隐//流量1
		
    SysSetParam[Number].PumpOffOnA = 1;  //A泵开关
		//********************************
		SysRunData.PumpONOFF_A = SysSetParam[Number].PumpOffOnA;
		//********************************	
    SysSetParam[Number].PumpOffOnB = 0;  //B泵开关
		//********************************
		SysRunData.PumpONOFF_B = SysSetParam[Number].PumpOffOnB;
		//********************************	 
	}

  //信息状态流量
  if (SysModelConfig.HandlePortB == 1)
	{
		Param[4] = 1; //DisplayInterfaceData.State_Info2 = 1; 显
		Param[9] = 4;  //DisplayInterfaceData.State_Info2 = 2; 隐//流量1
		
    SysSetParam[Number].PumpOffOnB = 1;  //B泵开关
		//********************************
		SysRunData.PumpONOFF_B = SysSetParam[Number].PumpOffOnB;
		//********************************				
    SysSetParam[Number].PumpOffOnA = 0;  //A泵开关
		//********************************
		SysRunData.PumpONOFF_A = SysSetParam[Number].PumpOffOnA;
		//********************************			
	}

  Param[5] = SysSetParam[Number].GearPositionHz;  //DisplayInterfaceData.State_Info3 = 6; //信息状态频率

  //********************************

  Param[0] = 0;  //DisplayInterfaceData.State_diam   = 0; //直径
  Param[1] = 0;  //DisplayInterfaceData.State_length = 0; //长度
  Param[2] = 0;  //DisplayInterfaceData.State_angle  = 0; //角度

  Param[6] = 0;  //DisplayInterfaceData.State_Cutter1 = 0;  //刀具图片   (3号接口)
  Param[7] = 0;  //DisplayInterfaceData.State_Cutter2 = 0;  //刀具信息   (3号接口)
  //********************************
  //********************************
  if (SysFootPedalData.FootPedalConnectOkNo != Connect)
	  Param[8] = 1;  //DisplayInterfaceData.State_FootPedal = 1;   //脚控 0不显示脚控/手控, 1脚控未连接 未选中, 2脚控 已选中 选中, 3手控 已选中 选中, 4脚控已连接\手控已选中, 5脚控已选中\手控已连接
  else
	  Param[8] = 2;  //DisplayInterfaceData.State_FootPedal = 2;

  SysRunData.StartingMethod = SysSetParam[Number].FootAndHandCtrl; //脚踏控制
  //********************************

//					W_UART;     //串口切换(1号接口电机通信串口)

  SysRunData.MotorNum = MotorNum1;       //1号接口手柄

//	SysRunData.PWMValue = 700;        //PWM输出处置（两极输出）

  SysRunData.BeepTimeMS = 100;
}

