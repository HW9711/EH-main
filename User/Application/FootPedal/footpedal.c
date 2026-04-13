//footpedal.c

#include "footpedal.h"
#include "data.h"
#include "screen.h"
#include "pedal.h"
#include "motor.h"
#include "iwdg.h"
#include "delay.h"
#include "pedal.h"
#include "param.h"
#include "pump.h"
#include "datahand.h"
#include "kernel_scheduler.h"

kernel_task_t FOOTPEDALTaskHandle;
kernel_task_t FootThrottleHandle;

//脚踏
typedef struct PedalKeyTag
{
  //0不显示脚控/手控,
  //1脚控未连接 未选中,
  //2脚控 已选中 选中,
  //3手控 已选中 选中,
  //4脚控已连接\手控已选中,
  //5脚控已选中\手控已连接
  //u8 State_FootPedal;    //脚控 状态 8

  uint8_t SelectWindowNum;
  uint8_t SelectWindowNumLast;
  uint16_t WindowDisappearTimeCnt;

} PedalKey;

static PedalKey PedalKeyData = { 0 };

//============================================================================
// 函数名称: PedalSacn_KeyMinus()
// 功能描述: 脚踏键值 -
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void FootPedal_KeyMinus(uint8_t win)
{
  switch (win)
  {
	  case 1 : //转速-
	  {
	    if (SysUIDisplayData.UIDisplay0x1303 == 2)
	    {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x5010_0x5030_Minus(1);
	    }
	  }
	  break;
	  case 2 : //流量-
	  {
//	    if (SysUIDisplayData.UIDisplay0x1304 < 2)
//	      break;

//	    if (SysRunData.MotorNum != MotorNone)
//	    {
//	      SysRunData.BeepTimeMS = 100;

//		    if (SysUIDisplayData.UIDisplay0x1304 == 2)
//		      Param_0x5110_0x5130_Minus(1);
//		    else if (SysUIDisplayData.UIDisplay0x1304 == 5)
//		      Param_0x2405_Er();
//		    else if (SysUIDisplayData.UIDisplay0x1304 == 4)
//		      Param_0x5110_0x5130_Minus(1);
//	    }
	    if ((SysUIDisplayData.UIDisplay0x1305 == 2) && (SysSetParam[SysInterface.BeSelectNum-1].PumpOffOnB == ON) && (SysRunData.PumpDrain_B == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x5210_0x5230_Minus_B(1);
	    }
	    if ((SysUIDisplayData.UIDisplay0x1318 == 2) && (SysSetParam[SysInterface.BeSelectNum-1].PumpOffOnA == ON) && (SysRunData.PumpDrain_A == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x5280_0x52A0_Minus_A(1);
	    }				
			
	  }
	  break;
	  case 3 : //模式（正向、往复、反向）
	  {
	    //if ((SysRunData.MotorRun != MotorStop) && (SysUIDisplayData.UIDisplay0x1310 != 2) && (SysUIDisplayData.UIDisplay0x1310 != 3))
			if (SysRunData.MotorRun != MotorStop)	
		    break;

	    //往复->正向->反向
	    if (SysUIDisplayData.UIDisplay0x1310 == 1)
	    {
        SysRunData.BeepTimeMS = 100;

		    Param_0x2412_OppositeDirection();  //反向
	    }
	    else if (SysUIDisplayData.UIDisplay0x1310 == 2)
	    {
        SysRunData.BeepTimeMS = 100;
        Param_0x2407_ForwardDirection();  //正向
		    
	    }
	    else if (SysUIDisplayData.UIDisplay0x1310 == 3)
	    {
		    if (SysUIDisplayData.UIDisplay0x1311 == 1)  //判断往复是否存在
		    {
          if (SysSetParam[SysInterface.BeSelectNum-1].ReciprocatingFlag == 1)
		      {
            SysRunData.BeepTimeMS = 100;

			      Param_0x2408_ToAndFro();  //往复
		      }
		    }
		    else
		    {
          SysRunData.BeepTimeMS = 100;
         
		      Param_0x2407_ForwardDirection();  //正向Param_0x2412_OppositeDirection();  //反向
		    }
	    }		
			
	  }
	  break;
	  case 4 : //开口 左调整
	  {
	    if ((SysUIDisplayData.UIDisplay0x1403 == 1) && (SysRunData.MotorNum == MotorNum2) && (SysRunData.MotorRun == MotorStop) && \
		      (SysSetParam[SysInterface.BeSelectNum-1].DJSetPDMT == 0) && (SysSetParam[SysInterface.BeSelectNum-1].ReciprocatingFlag == 1))
	    {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x2A04_RightAngleSet();
	    }
	  }
	  break;
	  case 5 : //模式（往复、正向、反向）
	  {

	  }
	  break;
	  case 6 : //脚控、手控
	  {
	    if (SysUIDisplayData.UIDisplay0x1313 < 2)
		    break;

	    if ((SysUIDisplayData.UIDisplay0x1313 == 3) && (SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				  (((SysInterface.HandleType[1] == Handle_Type_22) && (SysInterface.BeSelectNum == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && (SysInterface.BeSelectNum == 5)))) 	//Handle_Status_Flag2  State_Handle_Connect2
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2409_PedalSelect();
	    }
	    else if ((SysUIDisplayData.UIDisplay0x1313 == 2) && (SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				       (((SysInterface.HandleType[1] == Handle_Type_22) && (SysInterface.BeSelectNum == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && (SysInterface.BeSelectNum == 5))))   //Handle_Status_Flag2  State_Handle_Connect2
	    {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x2410_FingerControl();
	    }
	  }
	  break;
	  default : break;
  }
}

//============================================================================
// 函数名称: PedalSacn_KeyPlus()
// 功能描述: 脚踏键值 +
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void FootPedal_KeyPlus(uint8_t win)
{
  switch (win)
  {
	  case 1 : //转速+
	  {
	    if (SysUIDisplayData.UIDisplay0x1303 == 2)
	    {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x5050_0x5070_Add(1);
	    }
	  }
	  break;
	  case 2 :  //流量+ 
	  {
 
	    if ((SysUIDisplayData.UIDisplay0x1305 == 2) && (SysSetParam[SysInterface.BeSelectNum-1].PumpOffOnB == ON) && (SysRunData.PumpDrain_B == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x5250_0x5270_Add_B(1);
	    }
	    if ((SysUIDisplayData.UIDisplay0x1318 == 2) && (SysSetParam[SysInterface.BeSelectNum-1].PumpOffOnA == ON) && (SysRunData.PumpDrain_A == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x52B0_0x52D0_Add_A(1);
	    }					
	  }
	  break;
	  case 3 : //模式（往复、正向、反向）
	  {
	    //if ((SysRunData.MotorRun != MotorStop) && (SysUIDisplayData.UIDisplay0x1310 != 2) && (SysUIDisplayData.UIDisplay0x1310 != 3))
			if (SysRunData.MotorRun != MotorStop)	
		    break;

	    //往复->正向->反向
	    if (SysUIDisplayData.UIDisplay0x1310 == 3)
	    {
        SysRunData.BeepTimeMS = 100;

		    Param_0x2407_ForwardDirection();  //正向
	    }
	    else if (SysUIDisplayData.UIDisplay0x1310 == 2)
	    {
        SysRunData.BeepTimeMS = 100;
        Param_0x2412_OppositeDirection();  //反向
		    
	    }
	    else if (SysUIDisplayData.UIDisplay0x1310 == 1)
	    {
		    if (SysUIDisplayData.UIDisplay0x1311 == 1)  //判断往复是否存在
		    {
          if (SysSetParam[SysInterface.BeSelectNum-1].ReciprocatingFlag == 1)
		      {
            SysRunData.BeepTimeMS = 100;

			      Param_0x2408_ToAndFro();  //往复
		      }
		    }
		    else
		    {
          SysRunData.BeepTimeMS = 100;
         
		      Param_0x2412_OppositeDirection();  //反向Param_0x2407_ForwardDirection();  //正向
		    }
	    }	
	  }
	  break;
	  case 4 : //开口 右调整
	  {
	    if ((SysUIDisplayData.UIDisplay0x1403 == 1) && (SysRunData.MotorNum == MotorNum2) && (SysRunData.MotorRun == MotorStop) && \
		      (SysSetParam[SysInterface.BeSelectNum-1].DJSetPDMT == 0) && (SysSetParam[SysInterface.BeSelectNum-1].ReciprocatingFlag == 1))
	    {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x2A03_LeftAngleSet();
	    }
	  }
	  break;
	  case 5 : //模式（往复、正向、反向）
	  {

	  }
	  break;
	  case 6 : //脚控、手控
	  {
	    if (SysUIDisplayData.UIDisplay0x1313 < 2)
		    break;

	    if ((SysUIDisplayData.UIDisplay0x1313 == 3) && (SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				  (((SysInterface.HandleType[1] == Handle_Type_22) && (SysInterface.BeSelectNum == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && (SysInterface.BeSelectNum == 5)))) 	//Handle_Status_Flag2  State_Handle_Connect2
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2409_PedalSelect();
	    }
	    else if ((SysUIDisplayData.UIDisplay0x1313 == 2) && (SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				       (((SysInterface.HandleType[1] == Handle_Type_22) && (SysInterface.BeSelectNum == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && (SysInterface.BeSelectNum == 5))))   //Handle_Status_Flag2  State_Handle_Connect2
      {
		    SysRunData.BeepTimeMS = 100;

		    Param_0x2410_FingerControl();
	    }
	  }
	  break;
	  default : break;
  }
}

//============================================================================
// 函数名称: Display_PedalSelectWin()
// 功能描述: 切换手柄，复位脚踏设置选中框
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void FootPedal_SelectWin(uint8_t Num)
{
  if (PedalKeyData.SelectWindowNum > 0)
  {
	  if (Num == 0xff)
	  {
	    Screen_WindowSwitch_Update((PedalKeyData.SelectWindowNum-1), 0xff);
	    PedalKeyData.SelectWindowNumLast = PedalKeyData.SelectWindowNum = 0;
	  }
	  else if (PedalKeyData.SelectWindowNum != 1)
	  {
	    PedalKeyData.SelectWindowNum = 1;
	    Screen_WindowSwitch_Update((PedalKeyData.SelectWindowNumLast-1), (PedalKeyData.SelectWindowNum-1));

	    PedalKeyData.SelectWindowNumLast = PedalKeyData.SelectWindowNum;
	  }

	  PedalKeyData.WindowDisappearTimeCnt = 0;
  }
}

//
void FootPedal_KeyScanSSC(void)
{
	switch(SysFootPedalData.FootPedalKeyValue)
	{
		case 1://中间长按
			Workvalue_s.Foot_Key_value=6;
			break;
		case 2://中间短按
			Workvalue_s.Foot_Key_value=5;
			break;
		case 3://左边短按
			Workvalue_s.Foot_Key_value=1;
			break;
		case 4://右边短按
			Workvalue_s.Foot_Key_value=3;
			break;
		case 5://右边長按
			Workvalue_s.Foot_Key_value=2;
			break;
	}
	SysFootPedalData.FootPedalKeyValue=0;
}

//============================================================================
// 函数名称: FootPedal_KeyScan()
// 功能描述: 脚踏键值处理
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void FootPedal_KeyScan(void)
{
  uint8_t OnLineCnt = 0;

  static uint32_t FlickerFreq = 0;

  if (PedalKeyData.SelectWindowNum > 0)
  {
	  PedalKeyData.WindowDisappearTimeCnt++;
	  FlickerFreq++;

	  //闪烁框消失 5s
	  if ((PedalKeyData.WindowDisappearTimeCnt > 200) || (SysRunData.MotorRun == MotorWorking))
	  {
	    PedalKeyData.WindowDisappearTimeCnt = 0;
	    Screen_WindowSwitch_Update((PedalKeyData.SelectWindowNum-1), 0xff);
	    PedalKeyData.SelectWindowNumLast = PedalKeyData.SelectWindowNum = 0;
	  }
	  //闪烁被选框 300ms
	  else if (FlickerFreq == 12)  //隐
      Screen_WindowSwitch_Update((PedalKeyData.SelectWindowNum-1), 0xff);
	  else if (FlickerFreq == 24)  //显
	  {
	    FlickerFreq = 0;
      Screen_WindowSwitch_Update(0xff, (PedalKeyData.SelectWindowNum-1));
		}
  }
  else
  {
	  PedalKeyData.WindowDisappearTimeCnt = 0;
	  FlickerFreq = 0;
  }

  switch (SysFootPedalData.FootPedalKeyValue)
  {
	  case 1 :  //手柄切换	长按C键1.2s
	  {
			if (PedalKeyData.SelectWindowNum == 0)
			{
				OnLineCnt = Data_GetOnLineCnt();   //当前在线手柄数量

				if ((SysRunData.HandleAllowSwitchFlag == 0) && (OnLineCnt > 1))
				{
					SysInterface.BeSelectIndexUI++;
					if (SysInterface.BeSelectIndexUI > OnLineCnt)
						SysInterface.BeSelectIndexUI = 1;

					SysInterface.LcdKeyValue = SysInterface.BeSelectIndexUI;
					SysInterface.DisplayUpdateFlag = 2;
				}
			}
	  }
	  break;
	  case 2 :  //模式调节 短按C键200ms
	  {
	    if (SysUIDisplayData.UIDisplay0x1303 < 2)
		    break;

	    PedalKeyData.SelectWindowNum++;
	    if ((PedalKeyData.SelectWindowNum == 4) && (SysUIDisplayData.UIDisplay0x1403 != 1))
		    PedalKeyData.SelectWindowNum++;

//	    if ((PedalKeyData.SelectWindowNum == 5) && (SysUIDisplayData.UIDisplay0x1310 != 2) && (SysUIDisplayData.UIDisplay0x1310 != 3))
//		    PedalKeyData.SelectWindowNum++;

//	    if ((PedalKeyData.SelectWindowNum == 6) && (SysUIDisplayData.UIDisplay0x1313 < 2))
//		    PedalKeyData.SelectWindowNum++;

	    if (PedalKeyData.SelectWindowNum > 4)
		    PedalKeyData.SelectWindowNum = 1;

 	    SysRunData.BeepTimeMS = 100;
		  Screen_WindowSwitch_Update((PedalKeyData.SelectWindowNumLast-1), (PedalKeyData.SelectWindowNum-1));

	    PedalKeyData.SelectWindowNumLast = PedalKeyData.SelectWindowNum;
		  PedalKeyData.WindowDisappearTimeCnt = 0;
	  }
	  break;
	  case 3 :  //参数-短按B键200ms
	  {
      FootPedal_KeyMinus(PedalKeyData.SelectWindowNum);
	    PedalKeyData.WindowDisappearTimeCnt = 0;
	  }
	  break;
	  case 4 :  //参数+ 短按A键200ms
	  {
      FootPedal_KeyPlus(PedalKeyData.SelectWindowNum);
	    PedalKeyData.WindowDisappearTimeCnt = 0;
	  }
	  break;
	  default : break;
  }
	SysFootPedalData.FootPedalKeyValue = 0;
}

//============================================================================
// 函数名称: FootPedal_ConnectScan()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================

void FootPedal_ConnectScanSSC(void)//25ms执行一次
{
	static uint8_t   read_onetimes_flag=1;
	static uint8_t  repeat_flag=1;//重复值
		static uint8_t  fall_repeat_flag=1;//重复值
	static uint8_t	shake_times=0;
	static uint8_t	fall_shake_times=0;
	static uint8_t Memory_times_d=0;
	static uint8_t Memory_times_s=0;
	
	SysFootPedalData.FootPedalOffTimes++;
	if(SysFootPedalData.FootPedalOffTimes>20)
	{
		SysFootPedalData.FootPedalConnectFlag = No_Connect;
	}
	if(SysFootPedalData.FootPedalConnectFlag == Connect)//有脚踏数据上传，先去抖动
	{
		fall_shake_times=0;
		shake_times++;
		if(shake_times>=3)
		{
			Workvalue_s.footcontrol_online_flag=1;//在线
			shake_times=0;
			
		}
		
	}
	else
	{
		shake_times=0;
		fall_shake_times++;
		if(fall_shake_times>=3)
		{
			fall_shake_times=0;
			Workvalue_s.footcontrol_online_flag=0;//掉线
			Workvalue_s.Footmemory_reads_signal=0;
			read_onetimes_flag=1;
			Workvalue_s.Foot_type=0;
			if(!Workvalue_s.Handle_MOTORWorking_flag){
			Workvalue_s.MOTORWorking_flag=0;
			Workvalue_s.MotorRealSpeed=0;
			}
			
		}
	}
	if(Workvalue_s.footcontrol_online_flag)
	{
		
		fall_repeat_flag=1;
		if(repeat_flag)
		{	
			KeyBeep_flag=1;
			repeat_flag=0;
		
			Workvalue_s.set_Way=footcontrol;
			//这里给一个触发信号
			Workvalue_s.Footmemory_reads_signal=1;
				ssc_Connectfootpedal(2);
					if(Workvalue_s.hand_model==5){
					ssc_Connecthandel(1);
					}
					else{ssc_Connecthandel(0);}
					ssc_Connecttouch(1);
		}
	}
	else
	{
		if(fall_repeat_flag)
			{
				KeyBeep_flag=1;
				Workvalue_s.FootThrottletask_flag=0;
			//	Workvalue_s.Foot_start_flag=0;这里加在那个地方ssc重点，加在这个地方，热插拔，点击不停
				fall_repeat_flag=0;
					ssc_Connectfootpedal(0);
			
					if(Workvalue_s.Injection_drain_flag)
					{
						
					}
					else if(Workvalue_s.Foot_start_flag)
						{
							Workvalue_s.Foot_start_flag=0;
							Pump_SetSpeed_A(0);//泵运行流量
						
							if(Workvalue_s.hand_model==5){
								ssc_Connecthandel(2);
								}
								else{ssc_Connecthandel(0);}
								ssc_Connecttouch(1);
					}
					if(Workvalue_s.hand_model==5)
					{
						Workvalue_s.set_Way=handelcontrol;
						ssc_Connectfootpedal(0);
						ssc_Connecthandel(2);
						ssc_Connecttouch(1);
					}
					SysFootPedalData.FootPedalMemoryMValue_Right=0; //掉线脚踏值置为0
			}
			repeat_flag=1;
			//清楚报警，判断一下，如果是电机过载，或者手柄值错误，又或者是没有连接手柄启动脚踏，
			// 1 手柄未连接，请连接手柄
			// 2 脚踏未连接，请连接脚踏
			// 3 刀具未连接，请连接刀具
			// 4 霍尔型号错误，请联系售后
			// 5 点击通讯故障，请联系售后
			// 6 电机过载，请松开脚踏后运行，或者检查刀具是否卡死
			// 7 手柄未连接，请连接手柄后再启动脚踏
			// 8 电机相位错误，请联系售后！
			// 9 脚踏存储值错误，请联系售后！
			// 10 手柄型号错误，请联系售后！
			// 11 UID错误，请联系售后！
			// 12 脚控已选中，请使用脚控启动手柄！
			// 13 手控已选中，请使用手控启动手柄！
			// 14 触控已选中，请使用手控启动手柄！
			// 15右脚踏异常，请使用左脚踏并联系生产商！
			// 16左脚踏异常，请使用右脚踏并联系生产商！
			
			if(Workvalue_s.Alarm_value==1||Workvalue_s.Alarm_value==2||Workvalue_s.Alarm_value==3||Workvalue_s.Alarm_value==4||Workvalue_s.Alarm_value==11||Workvalue_s.Alarm_value==12||Workvalue_s.Alarm_value==13)
			{
				Workvalue_s.Alarm_value=0;//清除错误值
				Workvalue_s.beep_Alarm_flag=0;//清除错误信号
			}
			else if(Workvalue_s.Alarm_value==8||Workvalue_s.Alarm_value==9)
			{
				if(Workvalue_s.set_Way==footcontrol)
				{
							Workvalue_s.Alarm_value=0;//清除错误值
						Workvalue_s.beep_Alarm_flag=0;//清除错误信号
				}
			}
		}
	if(Workvalue_s.Footmemory_reads_signal)//发送 高值，中间值，低值储存值信号（发送之后，一段时间了，进行判断是否符合区间要求，定标值，如果不符合报警）
	{
		if(read_onetimes_flag){
	
		//开一个标识,这里分开，分别为单踏板和双踏板
		if(Workvalue_s.Foot_type==1)//单踏板
		{ 	
			Memory_times_d++;
			if(Memory_times_d==1)	      Pedal_ReadLValue();
			else if(Memory_times_d==3)	Pedal_ReadLValue();
			else if(Memory_times_d==5)	Pedal_ReadHValue();
			else if(Memory_times_d==7)	Pedal_ReadHValue();
			else if(Memory_times_d==9)	Pedal_ReadMValue();
			else if(Memory_times_d==11)	Pedal_ReadMValue();
			else if(Memory_times_d==14)
			{
				read_onetimes_flag=0;
				//判断值在没在区间
				if(((SysFootPedalData.FootPedalMemoryHValue > 750) && (SysFootPedalData.FootPedalMemoryHValue < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue > 500) && (SysFootPedalData.FootPedalMemoryLValue < 700)))	//脚踏值正常范围 低值500-700  高值750-95}
							{
								//单踏板正常，
								Workvalue_s.FootThrottletask_flag=1;
							}
							else
							{
								Workvalue_s.Alarm_value=11;//脚踏值错误
								Workvalue_s.beep_Alarm_flag=1;//报警
								Workvalue_s.FootThrottletask_flag=0;
								if(Workvalue_s.Injection_drain_flag)
								{
									Workvalue_s.Injection_drain_flag=0;
									Pump_SetSpeed_A(0);//停止排空
									injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
									//停止有两个信号来源，正常按键停止
									draindisplay(1);//按钮更新图标（白色）
								}
								
							}
			}
		}
		else if(Workvalue_s.Foot_type==2)//双踏板
		{
			Memory_times_s++;
			if(Memory_times_s==1)	      Pedal_ReadLValue();//右边命令和单踏板一样
			else if(Memory_times_s==3)	Pedal_ReadLValue();
			
			else if(Memory_times_s==5)	Pedal_ReadHValue();
				else if(Memory_times_s==7)	Pedal_ReadHValue();
			
			else if(Memory_times_s==9)	Pedal_ReadMValue();
				else if(Memory_times_s==11)	Pedal_ReadMValue();
			
			else if(Memory_times_s==13) Pedal_ReadMValue_Left();
				else if(Memory_times_s==15) Pedal_ReadMValue_Left();
			
			else if(Memory_times_s==17) Pedal_ReadLValue_Left();
				else if(Memory_times_s==19) Pedal_ReadLValue_Left();
			
			else if(Memory_times_s==21) Pedal_ReadHValue_Left();//
				else if(Memory_times_s==23) Pedal_ReadHValue_Left();//
			if(Memory_times_s==26)
			{
					read_onetimes_flag=0;
				//判断值是否正确
				if(((SysFootPedalData.FootPedalMemoryHValue_Left > 750) && (SysFootPedalData.FootPedalMemoryHValue_Left < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue_Left > 500) && (SysFootPedalData.FootPedalMemoryLValue_Left < 700)) && \
             ((SysFootPedalData.FootPedalMemoryHValue_Right > 750) && (SysFootPedalData.FootPedalMemoryHValue_Right < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue_Right > 500) && (SysFootPedalData.FootPedalMemoryLValue_Right < 700)))	//脚踏值正常范围 低值500-700  高值750-950
						{
							//双踏板值正常，不报警
							Workvalue_s.FootThrottletask_flag=1;
						}
						else
						{
							Workvalue_s.Alarm_value=12;//脚踏值错误
							Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.FootThrottletask_flag=0;
								if(Workvalue_s.Injection_drain_flag)
								{
									Workvalue_s.Injection_drain_flag=0;
									Pump_SetSpeed_A(0);//停止排空
									injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
									//停止有两个信号来源，正常按键停止
									draindisplay(1);//按钮更新图标（白色）
								}
						
							
						}
			}
		}
	}
	else
	{
		Memory_times_d=0;
		Memory_times_s=0;
	}
}
	
	
}


//脚踏油门任务，3ms 任务

void FootThrottleTask(uint32_t event)
{
	(void)event;
	uint8_t temp1;
	static	uint8_t fenti_counts=0;
	uint16_t difference_value=0;
	uint16_t foot_AD_value=0;
	uint16_t FootPedalADValue_right=0;
	uint16_t FootPedalADValue_left=0;
	uint32_t MX_SET_REAL_SPEED=0;
	static  uint8_t A_B_switch_counts=0;
	static  uint8_t B_A_switch_counts=0;
	if(Workvalue_s.HMI_Control_flag)
		return;
	if(Workvalue_s.Foot_type==1)
	{
		foot_AD_value=SysFootPedalData.FootPedalADValue;//右值判断
	}
	else
	{
		FootPedalADValue_left=SysFootPedalData.FootPedalADValue;
		FootPedalADValue_right=SysFootPedalData.FootPedalADValue_Right;
	}
	if(Workvalue_s.FootThrottletask_flag)//脚踏油门启动标志位（当脚踏值不报警的情况下）
	{
	if(!Workvalue_s.beep_Alarm_flag&&Workvalue_s.set_Way==footcontrol)
	{
		if(Workvalue_s.select_channel==1)				//A通道
		{
			if(Workvalue_s.Foot_type==1)//如果类型是单脚踏
			{
				if(foot_AD_value>SysFootPedalData.FootPedalMemoryLValue_Right)
				difference_value=foot_AD_value-SysFootPedalData.FootPedalMemoryLValue_Right;
				else difference_value=0;
				if(difference_value>50)
				{
					//启动泵运行
					if(Workvalue_s.Injection_drain_flag==start_flag)
					{
							Workvalue_s.Injection_drain_flag=stop_flag;
							injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
							//停止有两个信号来源，正常按键停止
							draindisplay(1);//按钮更新图标（白色）
						
					}
						Workvalue_s.Foot_start_flag=start_flag;
						temp1=APump(Workvalue_s.set_Injection);
						Pump_SetSpeed_A(temp1);//泵运行流量
						if(foot_AD_value>SysFootPedalData.FootPedalMemoryMValue_Right)
							{
								if(foot_AD_value>SysFootPedalData.FootPedalMemoryHValue_Right){foot_AD_value=SysFootPedalData.FootPedalMemoryHValue_Right;}
								
									difference_value=foot_AD_value-SysFootPedalData.FootPedalMemoryMValue_Right;
							}
						else{difference_value=0;}
							if(difference_value>20)
							{
								//点击启动代码
//							if(Workvalue_s.hand_model==MX_YIM_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2;
//							}
//							else if(Workvalue_s.hand_model==MX_YIM16_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2.783;
//							}
//							else
//							{
								MX_SET_REAL_SPEED=Workvalue_s.set_speed;
							//}
								Workvalue_s.MotorRealSpeed = (foot_AD_value - SysFootPedalData.FootPedalMemoryMValue_Right - 20) * \
								MX_SET_REAL_SPEED/ (SysFootPedalData.FootPedalMemoryHValue_Right - SysFootPedalData.FootPedalMemoryMValue_Right-50) / 10;		
								if (Workvalue_s.MotorRealSpeed > (MX_SET_REAL_SPEED/ 10))
								Workvalue_s.MotorRealSpeed = MX_SET_REAL_SPEED / 10;				

						if(Workvalue_s.fenti_switch_flag)
							{
								fenti_counts++;
								if(fenti_counts>50)
								{
									Workvalue_s.fenti_switch_flag=0;	
								}
							}
						else
							{
								fenti_counts=0;
								Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
							}								
					}
					else
					{
						Workvalue_s.MotorRealSpeed =0;
						Workvalue_s.MOTORWorking_flag=stop_flag;
					}
				}
				else
				{
						Workvalue_s.Foot_start_flag=stop_flag;
						if(Workvalue_s.Injection_drain_flag==stop_flag)
						Pump_SetSpeed_A(0);//泵运行流量
							Workvalue_s.MotorRealSpeed =0;
								Workvalue_s.MOTORWorking_flag=stop_flag;
						if(Workvalue_s.beep_Alarm_flag)
						{
							if(Workvalue_s.Alarm_value==8)//无手柄启动脚踏，松开脚踏报警消失
							{
								Workvalue_s.beep_Alarm_flag=0;
								Workvalue_s.Alarm_value=0;
							}
						}
				}
		}
			else if(Workvalue_s.Foot_type==2)
			{
				if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryLValue_Left)
				difference_value=FootPedalADValue_left-SysFootPedalData.FootPedalMemoryLValue_Left;
				else difference_value=0;
				if(difference_value>20)
				{
						//Workvalue_s.Injection_start_flag=start_flag;
					if(Workvalue_s.Injection_drain_flag==start_flag)
					{
						Workvalue_s.Injection_drain_flag=stop_flag;
						injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
							//停止有两个信号来源，正常按键停止
						draindisplay(1);//按钮更新图标（白色）
					}
					Workvalue_s.Foot_start_flag=start_flag;
					
					temp1=APump(Workvalue_s.set_Injection);
					Pump_SetSpeed_A(temp1);//泵运行流量
					if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryMValue_Left)//当大于中间值的时候，
						{
							if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryHValue_Left)FootPedalADValue_left=SysFootPedalData.FootPedalMemoryHValue_Left;
							
								difference_value=FootPedalADValue_left-SysFootPedalData.FootPedalMemoryMValue_Left;
						}
						else
						{
							difference_value=0;
						}
						if(difference_value>20)
						{
//							if(Workvalue_s.hand_model==MX_YIM_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2;
//							}
//							else if(Workvalue_s.hand_model==MX_YIM16_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2.783;
//							}
//							else
//							{
								MX_SET_REAL_SPEED=Workvalue_s.set_speed;
						//	}
								Workvalue_s.MotorRealSpeed = (FootPedalADValue_left - SysFootPedalData.FootPedalMemoryMValue_Left - 20) * \
								MX_SET_REAL_SPEED / (SysFootPedalData.FootPedalMemoryHValue_Left - SysFootPedalData.FootPedalMemoryMValue_Left-50) / 10;		 
								if (Workvalue_s.MotorRealSpeed > (MX_SET_REAL_SPEED/ 10))
								Workvalue_s.MotorRealSpeed = MX_SET_REAL_SPEED / 10;	
								if(Workvalue_s.fenti_switch_flag)
									{
										fenti_counts++;
										if(fenti_counts>50)
										{
											
											Workvalue_s.fenti_switch_flag=0;	
										}
								  }
								else{
									fenti_counts=0;
								Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
								}									
						}
						else
						{
								Workvalue_s.MOTORWorking_flag=stop_flag;
								Workvalue_s.MotorRealSpeed=0;
						}
				}
						
				
				else
				{
					if(Workvalue_s.Rightfoot_share_flag==stop_close){
					Workvalue_s.MOTORWorking_flag=stop_flag;
								Workvalue_s.MotorRealSpeed=0;
						Workvalue_s.Foot_start_flag=stop_flag;
					if(Workvalue_s.Injection_drain_flag==stop_flag)
						Pump_SetSpeed_A(0);//泵运行流量
				 }
				}
				
				if(Workvalue_s.Bchanell_online_flag==stop_flag)
				{
					A_B_switch_counts=0;
						if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryLValue_Right)
					difference_value=FootPedalADValue_right-SysFootPedalData.FootPedalMemoryLValue_Right;
					else difference_value=0;
					
					if(difference_value>20)
					{
						Workvalue_s.Rightfoot_share_flag=start_flag;
						//启动泵运行
						if(Workvalue_s.Injection_drain_flag==start_flag)
						{
							Workvalue_s.Injection_drain_flag=stop_flag;
								injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
								//停止有两个信号来源，正常按键停止
								draindisplay(1);//按钮更新图标（白色）
							
						}
								Workvalue_s.Foot_start_flag=start_flag;
								temp1=APump(Workvalue_s.set_Injection);
								Pump_SetSpeed_A(temp1);//泵运行流量
							if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryMValue_Right)
								{
									if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryHValue_Right){FootPedalADValue_right=SysFootPedalData.FootPedalMemoryHValue_Right;}
									
										difference_value=FootPedalADValue_right-SysFootPedalData.FootPedalMemoryMValue_Right;
								}
							else{difference_value=0;}
								if(difference_value>20)
								{
									//点击启动代码
//								if(Workvalue_s.hand_model==MX_YIM_ONLINE)
//								{
//									MX_SET_REAL_SPEED=Workvalue_s.set_speed/2;
//								}
//								else if(Workvalue_s.hand_model==MX_YIM16_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2.783;
//							}
//								else
//								{
									MX_SET_REAL_SPEED=Workvalue_s.set_speed;
								//}
							
									Workvalue_s.MotorRealSpeed = (FootPedalADValue_right - SysFootPedalData.FootPedalMemoryMValue_Right - 20) * \
									MX_SET_REAL_SPEED / (SysFootPedalData.FootPedalMemoryHValue_Right - SysFootPedalData.FootPedalMemoryMValue_Right-50) / 10;		
							if (Workvalue_s.MotorRealSpeed > (MX_SET_REAL_SPEED/ 10))
									Workvalue_s.MotorRealSpeed = MX_SET_REAL_SPEED / 10;		


								if(Workvalue_s.fenti_switch_flag)
									{
										fenti_counts++;
										if(fenti_counts>50)
										{
											
											Workvalue_s.fenti_switch_flag=0;	
										}
								  }
								else{
									fenti_counts=0;
								Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
								}										
								//	Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
									Workvalue_s.Rightfoot_share_flag=start_flag;
								}
								else
								{
									Workvalue_s.MotorRealSpeed =0;
									Workvalue_s.MOTORWorking_flag=stop_flag;
								}
					}
					else
					{
						if(Workvalue_s.Rightfoot_share_flag==start_flag){
							Workvalue_s.Foot_start_flag=stop_flag;
							if(Workvalue_s.Injection_drain_flag==stop_flag)
							Pump_SetSpeed_A(0);//泵运行流量
							Workvalue_s.MotorRealSpeed =0;
							Workvalue_s.MOTORWorking_flag=stop_flag;
							Workvalue_s.Rightfoot_share_flag=stop_close;
						}
					}
				}
				else
				{
					if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryLValue_Right)
					difference_value=FootPedalADValue_right-SysFootPedalData.FootPedalMemoryLValue_Right;
					else difference_value=0;
					
					if(difference_value>20)
					{
						//切换
						A_B_switch_counts++;
						if(A_B_switch_counts>70){
						Workvalue_s.ScreenKey_data=25;
							A_B_switch_counts=0;
						}
					}else{A_B_switch_counts=0;}
				}
			
			}
		}
		else if(Workvalue_s.select_channel==2)	//B通道
		{
			if(Workvalue_s.Foot_type==1){
				FootPedalADValue_right=foot_AD_value;
			}
				if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryLValue_Right)
				difference_value=FootPedalADValue_right-SysFootPedalData.FootPedalMemoryLValue_Right;
				else difference_value=0;
				
				if(difference_value>20)
				{
				//	Workvalue_s.leftfoot_share_flag=start_flag;
					//启动泵运行
					if(Workvalue_s.Injection_drain_flag==start_flag)
					{
						Workvalue_s.Injection_drain_flag=stop_flag;
							injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
							//停止有两个信号来源，正常按键停止
							draindisplay(1);//按钮更新图标（白色）
						
					}
							Workvalue_s.Foot_start_flag=start_flag;
							temp1=APump(Workvalue_s.set_Injection);
							Pump_SetSpeed_A(temp1);//泵运行流量
						if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryMValue_Right)
							{
								if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryHValue_Right){FootPedalADValue_right=SysFootPedalData.FootPedalMemoryHValue_Right;}
								
									difference_value=FootPedalADValue_right-SysFootPedalData.FootPedalMemoryMValue_Right;
							}
						else{difference_value=0;}
							if(difference_value>20)
							{
								//点击启动代码
//							if(Workvalue_s.hand_model==MX_YIM_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2;
//							}
//							else if(Workvalue_s.hand_model==MX_YIM16_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2.783;
//							}
//							else
//							{
								MX_SET_REAL_SPEED=Workvalue_s.set_speed;
						//	}
						
								Workvalue_s.MotorRealSpeed = (FootPedalADValue_right - SysFootPedalData.FootPedalMemoryMValue_Right - 20) * \
								MX_SET_REAL_SPEED / (SysFootPedalData.FootPedalMemoryHValue_Right - SysFootPedalData.FootPedalMemoryMValue_Right-50) / 10;		
						if (Workvalue_s.MotorRealSpeed > (MX_SET_REAL_SPEED/ 10))
								Workvalue_s.MotorRealSpeed = MX_SET_REAL_SPEED / 10;		

						if(Workvalue_s.fenti_switch_flag)
															{
																fenti_counts++;
																if(fenti_counts>50)
																{
																	
																	Workvalue_s.fenti_switch_flag=0;	
																}
															}
														else{
															fenti_counts=0;
														Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
														}									
								//Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
							}
							else
							{
								Workvalue_s.MotorRealSpeed =0;
								Workvalue_s.MOTORWorking_flag=stop_flag;
							}
				}
				else
				{
						if(Workvalue_s.leftfoot_share_flag==stop_close){
						Workvalue_s.Foot_start_flag=stop_flag;
						if(Workvalue_s.Injection_drain_flag==stop_flag)
						Pump_SetSpeed_A(0);//泵运行流量
						Workvalue_s.MotorRealSpeed =0;
						Workvalue_s.MOTORWorking_flag=stop_flag;
					}
				}
				
				if(Workvalue_s.Achanell_online_flag==stop_flag)
				{
					B_A_switch_counts=0;
					if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryLValue_Left)
				difference_value=FootPedalADValue_left-SysFootPedalData.FootPedalMemoryLValue_Left;
				else difference_value=0;
				if(difference_value>20)
				{
						Workvalue_s.leftfoot_share_flag=start_flag;
					if(Workvalue_s.Injection_drain_flag==start_flag)
					{
						Workvalue_s.Injection_drain_flag=stop_flag;
						injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
							//停止有两个信号来源，正常按键停止
						draindisplay(1);//按钮更新图标（白色）
					}
					
					
					temp1=APump(Workvalue_s.set_Injection);
					Pump_SetSpeed_A(temp1);//泵运行流量
					if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryMValue_Left)//当大于中间值的时候，
						{
							if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryHValue_Left)FootPedalADValue_left=SysFootPedalData.FootPedalMemoryHValue_Left;
							
								difference_value=FootPedalADValue_left-SysFootPedalData.FootPedalMemoryMValue_Left;
						}
						else
						{
							difference_value=0;
						}
						if(difference_value>20)
						{
//							if(Workvalue_s.hand_model==MX_YIM_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2;
//							}
//							else if(Workvalue_s.hand_model==MX_YIM16_ONLINE)
//							{
//								MX_SET_REAL_SPEED=Workvalue_s.set_speed/2.783;
//							}
//							else
//							{
								MX_SET_REAL_SPEED=Workvalue_s.set_speed;
							//}
								Workvalue_s.MotorRealSpeed = (FootPedalADValue_left - SysFootPedalData.FootPedalMemoryMValue_Left - 20) * \
								MX_SET_REAL_SPEED / (SysFootPedalData.FootPedalMemoryHValue_Left - SysFootPedalData.FootPedalMemoryMValue_Left-50) / 10;		 
								if (Workvalue_s.MotorRealSpeed > (MX_SET_REAL_SPEED/ 10))
								Workvalue_s.MotorRealSpeed = MX_SET_REAL_SPEED / 10;		

								if(Workvalue_s.fenti_switch_flag)
																	{
																		fenti_counts++;
																		if(fenti_counts>50)
																		{
																			
																			Workvalue_s.fenti_switch_flag=0;	
																		}
																	}
																else{
																	fenti_counts=0;
																Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
																}			

								
								//Workvalue_s.MOTORWorking_flag=start_flag;//工作运行速度
						}
						else
						{
								Workvalue_s.MOTORWorking_flag=stop_flag;
								Workvalue_s.MotorRealSpeed=0;
						}
				}
						
				
				else
				{
					if(Workvalue_s.leftfoot_share_flag==start_flag){
						Workvalue_s.leftfoot_share_flag=stop_flag;
					if(Workvalue_s.Rightfoot_share_flag==stop_close){
					Workvalue_s.MOTORWorking_flag=stop_flag;
								Workvalue_s.MotorRealSpeed=0;
						Workvalue_s.Foot_start_flag=stop_flag;
					if(Workvalue_s.Injection_drain_flag==stop_flag)
						Pump_SetSpeed_A(0);//泵运行流量
				}
				
		}
	}
		}
		else
		{
			if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryLValue_Left)
			difference_value=FootPedalADValue_left-SysFootPedalData.FootPedalMemoryLValue_Left;
			else difference_value=0;
			if(difference_value>20)
			{
					B_A_switch_counts++;
				if(B_A_switch_counts>50){
					Workvalue_s.ScreenKey_data=24;
					B_A_switch_counts=0;
				}
			}
			else{B_A_switch_counts=0;}
		}
	}
	  }
	  if(Workvalue_s.Foot_type==1)
			{
				if(foot_AD_value>SysFootPedalData.FootPedalMemoryLValue_Right)
				{
					if((foot_AD_value-SysFootPedalData.FootPedalMemoryLValue_Right)>20)
					{
						if(Workvalue_s.select_channel==0){
								Workvalue_s.Alarm_value=1;//沒有手柄啟動
								Workvalue_s.beep_Alarm_flag=1;//报警
								Workvalue_s.MOTORWorking_flag=0;
						}
						else if(Workvalue_s.set_Way==handelcontrol)
						{
							//报警，无手柄启动脚踏
								Workvalue_s.Alarm_value=2;//手控下启动右脚踏
								Workvalue_s.beep_Alarm_flag=1;//报警
								 Workvalue_s.MOTORWorking_flag=0;
						}
					}
					else
						{
							if(Workvalue_s.Alarm_value==1||Workvalue_s.Alarm_value==2)
							{
								Workvalue_s.Alarm_value=0;
								Workvalue_s.beep_Alarm_flag=0;
								Workvalue_s.MOTORWorking_flag=stop_flag;
							}
							if(Workvalue_s.set_Way==footcontrol)
							{
								if(Workvalue_s.Alarm_value==8||Workvalue_s.Alarm_value==9)
								{
									Workvalue_s.Alarm_value=0;
									Workvalue_s.beep_Alarm_flag=0;
									Workvalue_s.MOTORWorking_flag=0;
								}
							}
							if(Workvalue_s.Alarm_value==13)
							{
								Workvalue_s.Alarm_value=0;
								Workvalue_s.beep_Alarm_flag=0;
								Workvalue_s.MOTORWorking_flag=stop_flag;		
							}
						}
				}
				else
				{
					if(Workvalue_s.Alarm_value==1||Workvalue_s.Alarm_value==2)
					{
						Workvalue_s.Alarm_value=0;
						Workvalue_s.beep_Alarm_flag=0;
						Workvalue_s.MOTORWorking_flag=0;
					}
					if(Workvalue_s.set_Way==footcontrol)
							{
								if(Workvalue_s.Alarm_value==8||Workvalue_s.Alarm_value==9)
								{
									Workvalue_s.Alarm_value=0;
									Workvalue_s.beep_Alarm_flag=0;
									Workvalue_s.MOTORWorking_flag=0;
								}
							}
							if(Workvalue_s.Alarm_value==13)
							{
									Workvalue_s.Alarm_value=0;
									Workvalue_s.beep_Alarm_flag=0;
									Workvalue_s.MOTORWorking_flag=stop_flag;		
							}
				}
			}
			else if(Workvalue_s.Foot_type==2)
			{
				if(FootPedalADValue_right>SysFootPedalData.FootPedalMemoryLValue_Right)
				{
					if((FootPedalADValue_right-SysFootPedalData.FootPedalMemoryLValue_Right)>20)
					{
						if(Workvalue_s.select_channel==0){
						//报警，无手柄启动脚踏
								Workvalue_s.Alarm_value=1;//脚踏值错误
								Workvalue_s.beep_Alarm_flag=1;//无手柄启动右脚踏
								Workvalue_s.MOTORWorking_flag=0;
						}
						else if(Workvalue_s.set_Way==handelcontrol)
						{
							//报警，无手柄启动脚踏
								Workvalue_s.Alarm_value=2;//手控下启动右脚踏
								Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.MOTORWorking_flag=0;
						}
						
					}
					else
					{
						if(Workvalue_s.Alarm_value==1||Workvalue_s.Alarm_value==2)//清楚状态
						{
							Workvalue_s.Alarm_value=0;
							Workvalue_s.beep_Alarm_flag=0;
							
						}
						if(Workvalue_s.select_channel!=1)
							{
								if(Workvalue_s.Alarm_value==9&&Workvalue_s.set_Way==footcontrol)//电机过载
								{
									if(Workvalue_s.leftfoot_share_flag==stop_flag)
									Workvalue_s.Alarm_value=0;
									Workvalue_s.beep_Alarm_flag=0;
									
								}
							}
							else
							{
								if(Workvalue_s.Rightfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
							}
							if(Workvalue_s.Alarm_value==13)
							{
								if(Workvalue_s.select_channel==2)
								{
										if(Workvalue_s.leftfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
										Workvalue_s.beep_Alarm_flag=0;
										Workvalue_s.MOTORWorking_flag=stop_flag;
										}
									
								}
								else if(Workvalue_s.select_channel==1)
								{
									if(Workvalue_s.Rightfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
							}
						}
				
				}
				else
				{
					if(Workvalue_s.Alarm_value==1||Workvalue_s.Alarm_value==2)//清楚状态
						{
							Workvalue_s.Alarm_value=0;
							Workvalue_s.beep_Alarm_flag=0;
									Workvalue_s.MOTORWorking_flag=stop_flag;
						}
						if(Workvalue_s.select_channel!=1)
							{
								if(Workvalue_s.Alarm_value==9&&Workvalue_s.set_Way==footcontrol)//电机过载
								{
									if(Workvalue_s.leftfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
										Workvalue_s.beep_Alarm_flag=0;
										Workvalue_s.MOTORWorking_flag=stop_flag;
									}
										
								}
						}
						else
						{
							if(Workvalue_s.Rightfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
						}
					if(Workvalue_s.Alarm_value==13)
							{
								if(Workvalue_s.select_channel==2)
								{
									if(Workvalue_s.leftfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
										Workvalue_s.beep_Alarm_flag=0;
										Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
								else if(Workvalue_s.select_channel==1)
								{
									if(Workvalue_s.Rightfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
							}
				}
				
			if(FootPedalADValue_left>SysFootPedalData.FootPedalMemoryLValue_Left)
				{
					if((FootPedalADValue_left-SysFootPedalData.FootPedalMemoryLValue_Left>20))
					{
						if(Workvalue_s.select_channel==0){
						//报警，无手柄启动脚踏
								Workvalue_s.Alarm_value=3;//无手柄启动左脚踏
								Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.MOTORWorking_flag=0;
						}
						else if(Workvalue_s.set_Way==handelcontrol)
						{
							//报警，无手柄启动脚踏
								Workvalue_s.Alarm_value=4;//手控模式下启动左脚踏
								Workvalue_s.beep_Alarm_flag=1;//报警
							Workvalue_s.MOTORWorking_flag=0;
						}
					}
					else
					{
						if(Workvalue_s.Alarm_value==3||Workvalue_s.Alarm_value==4)
						{
							Workvalue_s.Alarm_value=0;
							Workvalue_s.beep_Alarm_flag=0;
							
						}
						if(Workvalue_s.select_channel!=2)
							{
									if(Workvalue_s.Alarm_value==8&&Workvalue_s.set_Way==footcontrol)//电机过载
									{
										if(Workvalue_s.Rightfoot_share_flag==stop_flag){
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
										}
									}
							}
							else
							{
								if(Workvalue_s.leftfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
							}
						if(Workvalue_s.Alarm_value==13)
							{
									if(Workvalue_s.select_channel==1)
								{
										if(Workvalue_s.Rightfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
										Workvalue_s.beep_Alarm_flag=0;
										Workvalue_s.MOTORWorking_flag=stop_flag;
										}
								}
								else if(Workvalue_s.select_channel==2)
								{
									if(Workvalue_s.leftfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
							}
					}
				}
				else
				{
						if(Workvalue_s.Alarm_value==3||Workvalue_s.Alarm_value==4)
						{
							Workvalue_s.Alarm_value=0;
							Workvalue_s.beep_Alarm_flag=0;
							Workvalue_s.MOTORWorking_flag=stop_flag;
						}
					if(Workvalue_s.select_channel!=2)
						{
								if(Workvalue_s.Alarm_value==8&&Workvalue_s.set_Way==footcontrol)//电机过载
								{
										if(Workvalue_s.Rightfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
									Workvalue_s.beep_Alarm_flag=0;
									Workvalue_s.MOTORWorking_flag=stop_flag;
										}
								}
						}
						else
						{
							if(Workvalue_s.leftfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
						}
					if(Workvalue_s.Alarm_value==13)////ssc250603
					{
								if(Workvalue_s.select_channel==1)
								{
									if(Workvalue_s.Rightfoot_share_flag==stop_flag){
										Workvalue_s.Alarm_value=0;
										Workvalue_s.beep_Alarm_flag=0;
										Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
								else if(Workvalue_s.select_channel==2)
								{
									if(Workvalue_s.leftfoot_share_flag==start_flag)
									{
											Workvalue_s.Alarm_value=0;
											Workvalue_s.beep_Alarm_flag=0;
											Workvalue_s.MOTORWorking_flag=stop_flag;
									}
								}
						}
				}
			}
		}
}

//ssc,2025,0522插入错误的脚踏，自己启动蠕动泵，直接加以判断了，应该要等校验值通过，拔出脚踏并未停止转动？？？

void FootThrottleTask_Init(void)
{
  /* definition and creation of FOOTPEDALTask */
	Kernel_TaskCreate(&FootThrottleHandle, FootThrottleTask);
	Kernel_TaskStart(&FootThrottleHandle, KERNEL_TASK_ALWAYS, 10);
}

void FootPedal_ConnectScan(void)
{
  static uint8_t JT_Status_Flag = 0;
  static uint8_t JT_Status_Flag_last = 1;

  static uint8_t JT_linkOK_count = 0;
  static uint8_t JT_linkNO_count = 0;

  uint8_t StatePedal = 0, temp = 0;

  if (SysFootPedalData.FootPedalConnectFlag == Connect)  //脚踏已连接
  {
	  if (++JT_linkOK_count >= 2)  //去抖
	  {
	    JT_linkOK_count = 0;
	    JT_Status_Flag = (JT_Status_Flag > 1) ? JT_Status_Flag : 2;
	  }
		JT_linkNO_count = 0;
  }
  else  //脚踏未连接
  {
	  if (++JT_linkNO_count >= 2)
	  {
	    JT_linkNO_count = 0;
	    JT_Status_Flag = 1;  //未连接
	  }

	  JT_linkOK_count = 0;
  }

  if (JT_Status_Flag_last != JT_Status_Flag)
  {
	  JT_Status_Flag_last = JT_Status_Flag;

		if (SysInterface.InterfaceSwitchNo2 == 1)
      temp = SysInterface.HandleType[1];
	  else
      temp = SysInterface.HandleType[4];

	  switch(JT_Status_Flag)
	  {
	    case 1 :
	    {
		    if((SysRunData.MotorNum == MotorNum2) && (temp == Handle_Type_22))
		    {
		      StatePedal = 3;
		      SysRunData.StartingMethod = ManualCtrl; //手控
		    }
		    else
		    {
		      StatePedal = 1;
		      SysRunData.StartingMethod = FootCtrl; //脚踏控制
		    }

		    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

		    SysFootPedalData.FootPedalMemoryLValue = 0;
		    SysFootPedalData.FootPedalMemoryHValue = 0;

		    SysFootPedalData.FootPedalADValue = 0;//防止在电机转动时，脚踏断开，电机仍在运行

		    SysFootPedalData.FootPedalConnectOkNo = No_Connect;
		    SysFootPedalData.FootPedalReadFlag = No_Error;  //No_Error = 0

		    SysRunData.BeepTimeMS = 100;

//		  Motor_ErrorEmergencyStop_Ctrl(150);  //390ms

		    Screen_FootPedalConnectState_Update(StatePedal); //脚踏连接图片后面更改工艺文件
	    }
	    break;
	    case 3 :
	    case 5 :
	    {
			  if (SysFootPedalData.FootPedalType == 1)
				{
          Pedal_ReadHValue_Left();
				}
		    JT_Status_Flag++;
	    }
	    break;
	    case 7 :
	    case 9 :
	    {
			  if (SysFootPedalData.FootPedalType == 1)
				{				
          Pedal_ReadLValue_Left();
				}
		    JT_Status_Flag++;
	    }
	    break;				
	    case 10 :
	    case 11 :
	    {
			  if (SysFootPedalData.FootPedalType == 1)
				{				
          Pedal_ReadMValue_Left();
				}				
		    JT_Status_Flag++;
	    }
	    break;
	    case 2 :
	    case 6 :
	    {
        Pedal_ReadHValue();
		    JT_Status_Flag++;
	    }
	    break;
	    case 4 :
	    case 8 :
	    {
        Pedal_ReadLValue();
		    JT_Status_Flag++;
	    }
	    break;
	    case 12 :
	    case 13 :
	    {
			  if (SysFootPedalData.FootPedalType == 1)
				{				
          Pedal_ReadMValue();
				}						
		    JT_Status_Flag++;
	    }			
	    break;			
	    case 14 :
	    case 15 :
	    {
		    JT_Status_Flag++;
	    }			
	    break;						
	    case 16 :
	    {
		    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;
				
				if (SysFootPedalData.FootPedalType == 1)
				{
					if(((SysFootPedalData.FootPedalMemoryHValue_Left > 750) && (SysFootPedalData.FootPedalMemoryHValue_Left < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue_Left > 500) && (SysFootPedalData.FootPedalMemoryLValue_Left < 700)) && \
             ((SysFootPedalData.FootPedalMemoryHValue_Right > 750) && (SysFootPedalData.FootPedalMemoryHValue_Right < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue_Right > 500) && (SysFootPedalData.FootPedalMemoryLValue_Right < 700)))	//脚踏值正常范围 低值500-700  高值750-950
					{
						if((SysRunData.MotorNum == MotorNum2) && (temp == Handle_Type_22))
						{
							StatePedal = 5;
						}
						else
						{
							StatePedal = 2;
						}

						SysRunData.StartingMethod = FootCtrl;  //脚踏控制

						SysFootPedalData.FootPedalMemoryHValue = SysFootPedalData.FootPedalMemoryHValue_Right;
						SysFootPedalData.FootPedalMemoryLValue = SysFootPedalData.FootPedalMemoryLValue_Right;
						
						SysFootPedalData.FootPedalReadFlag = No_Error;  //No_Error = 0
					}
					else
					{
						if((SysRunData.MotorNum == MotorNum2) && (temp == Handle_Type_22))
						{
							StatePedal = 3;
							SysRunData.StartingMethod = ManualCtrl; //手控
						}
						else
						{
							StatePedal = 1;
							SysRunData.StartingMethod = FootCtrl;  //脚踏控制
						}

						SysFootPedalData.FootPedalReadFlag = Error;  //Error       1
					}					
				}
				else
        {
					if(((SysFootPedalData.FootPedalMemoryHValue > 750) && (SysFootPedalData.FootPedalMemoryHValue < 950)) && \
						 ((SysFootPedalData.FootPedalMemoryLValue > 500) && (SysFootPedalData.FootPedalMemoryLValue < 700)))	//脚踏值正常范围 低值500-700  高值750-950
					{
						if((SysRunData.MotorNum == MotorNum2) && (temp == Handle_Type_22))
						{
							StatePedal = 5;
						}
						else
						{
							StatePedal = 2;
						}

						SysRunData.StartingMethod = FootCtrl;  //脚踏控制

						SysFootPedalData.FootPedalReadFlag = No_Error;  //No_Error = 0
					}
					else
					{
						if((SysRunData.MotorNum == MotorNum2) && (temp == Handle_Type_22))
						{
							StatePedal = 3;
							SysRunData.StartingMethod = ManualCtrl; //手控
						}
						else
						{
							StatePedal = 1;
							SysRunData.StartingMethod = FootCtrl;  //脚踏控制
						}
						SysFootPedalData.FootPedalReadFlag = Error;  //Error       1
					}				
				}

		    SysFootPedalData.FootPedalConnectOkNo = Connect;

		    SysRunData.BeepTimeMS = 100;

		    Screen_FootPedalConnectState_Update(StatePedal); //脚踏连接图片
	    }
	    break;
	    default: break;
	  }
  }
}

/* USER CODE BEGIN Header_FOOTPEDALTaskFunc */
/**
* @brief Function implementing the FOOTPEDALTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FOOTPEDALTaskFunc */
void FOOTPEDALTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN FOOTPEDALTaskFunc */
  /* Infinite loop */
	//FootPedal_ConnectScan();  //脚踏”连接“扫描
	if(Workvalue_s.HMI_Control_flag)
		return;
	if(Workvalue_s.set_Way==touchcontrol)return;
	FootPedal_ConnectScanSSC();
	//FootPedal_KeyScan();  //脚踏”按键“扫描
	FootPedal_KeyScanSSC();
  /* USER CODE END FOOTPEDALTaskFunc */
}

void FootPedalTask_Init(void)
{
  /* definition and creation of FOOTPEDALTask */
	Kernel_TaskCreate(&FOOTPEDALTaskHandle, FOOTPEDALTaskFunc);
	Kernel_TaskStart(&FOOTPEDALTaskHandle, KERNEL_TASK_ALWAYS, 25);
}







