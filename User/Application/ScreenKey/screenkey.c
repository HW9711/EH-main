//screenkey.c

#include "screenkey.h"
#include "param.h"
#include "uart6.h"
#include "data.h"
#include "common.h"
#include "eeprom.h"
#include "screen.h"
#include "Pubinterface.h"
#include "sscKEYBH.h"

#include "kernel_scheduler.h"

kernel_task_t SCREENKEYLONGTaskHandle;
kernel_task_t SCREENKEYTaskHandle;

/*
 * 屏幕串口协议仍沿用旧的页面地址和按键编号，但业务出口改为 V1.8 新接口事件。
 * 这里集中维护旧 `ScreenKey_data` 数字到 `SCREENKey_*` 枚举的映射：
 * 1. 解析层继续按原 HMI 帧格式识别按键，避免改动串口协议；
 * 2. 行为层统一交给 sscKEYBH 分发，逐步替代旧屏幕模块的按键仓库职责；
 * 3. 无新接口等价项的旧码暂时静默，后续迁 UI/RFID 时再补专用事件。
 */
static void ScreenKey_PostLegacyAction(uint8_t legacy_key)
{
  uint8_t screen_key = 0U;

  switch (legacy_key)
  {
    case 1U:
    case 2U:
      screen_key = SCREENKey_SPEED_Add;
      break;

    case 3U:
    case 4U:
      screen_key = SCREENKey_SPEED_Sub;
      break;

    case 5U:
      screen_key = SCREENKey_BPUMP_Add;
      break;

    case 6U:
      screen_key = SCREENKey_BPUMP_Sub;
      break;

    case 7U:
      screen_key = SCREENKey_APUMP_Add;
      break;

    case 8U:
      screen_key = SCREENKey_APUMP_Sub;
      break;

    case 9U:
      screen_key = SCREENKey_FREQ_Add;
      break;

    case 10U:
      screen_key = SCREENKey_FREQ_Sub;
      break;

    case 11U:
      screen_key = SCREENKey_BPUMP_control;
      break;

    case 12U:
      screen_key = SCREENKey_APUMP_control;
      break;

    case 13U:
      screen_key = SCREENKey_Dir_Forward;
      break;

    case 14U:
      screen_key = SCREENKey_Dir_Reverse;
      break;

    case 15U:
      screen_key = SCREENKey_Dir_OSC;
      break;

    case 16U:
      screen_key = SCREENKey_JTActi;
      break;

    case 17U:
      screen_key = SCREENKey_HandleActi;
      break;

    case 18U:
      screen_key = SCREENKey_TouchActi;
      break;

    case 20U:
      screen_key = SCREENKey_GrindH;
      break;

    case 21U:
      screen_key = SCREENKey_PlanerH;
      break;

    case 22U:
      screen_key = SCREENKey_OpenPos_ClockWise;
      break;

    case 23U:
      screen_key = SCREENKey_OpenPos_AntiClockWise;
      break;

    case 24U:
      screen_key = SCREENKey_HANDLE_A;
      break;

    case 25U:
      screen_key = SCREENKey_HANDLE_B;
      break;

    case 26U:
      screen_key = SCREENKey_UNPLUG_A;
      break;

    case 27U:
      screen_key = SCREENKey_UNPLUG_B;
      break;

    case 28U:
      screen_key = SCREENKey_PLUG_A;
      break;

    case 29U:
      screen_key = SCREENKey_PLUG_B;
      break;

    case 40U:
      screen_key = SCREENKey_TouchEXIT;
      break;

    case 41U:
      screen_key = SCREENKey_TouchStart;
      break;

    case 42U:
      screen_key = SCREENKey_TouchEXIT;
      break;

    case 43U:
      screen_key = SCREENKey_HMI_EXIT;
      break;

    default:
      break;
  }

  if (screen_key != 0U)
  {
    SendKeyBehMessage(SCREENKey, screen_key);
  }
}

//============================================================================
//1.屏”按键“
//============================================================================

//============================================================================
// 函数名称: ScreenKey_LongPressTask()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 长按操作150ms
//============================================================================
void ScreenKey_LongPressTask(void)
{
  static uint8_t RenewEEPOMflag_B = 0;
  static uint8_t RenewEEPOMflag_A = 0;
	uint8_t temp = 0;

  switch (SysRunData.KeyLongPressValue)
  {
	  case KEY_SPEEDLONGPRESSREDUCE : Param_0x5010_0x5030_Minus(2); break;  //速度--
	  case KEY_SPEEDLONGPRESSPLUS : Param_0x5050_0x5070_Add(2); break;  //速度++
	  case KEY_FLOWRATELONGPRESSREDUCE_B :  //B流量--
	  {
	    if ((SysSetParam[SysInterface.BeSelectNum - 1].PumpOffOnB == ON) && (SysRunData.PumpDrain_B  == 0))
	    {
	      if (Param_0x5210_0x5230_Minus_B(2) == 1)
          RenewEEPOMflag_B = 1;
	    }
	  }
	  break;
	  case KEY_FLOWRATELONGPRESSPLUS_B :  //B流量 ++
	  {
	    if ((SysSetParam[SysInterface.BeSelectNum - 1].PumpOffOnB == ON) && (SysRunData.PumpDrain_B == 0))
	    {
		    if (Param_0x5250_0x5270_Add_B(2) == 1)
		      RenewEEPOMflag_B = 1;
      }
	  }
	  break;
	  case KEY_FLOWRATELONGPRESSREDUCE_A :  //A流量--
	  {
	    if ((SysSetParam[SysInterface.BeSelectNum - 1].PumpOffOnA == ON) && (SysRunData.PumpDrain_A  == 0))
	    {
	      if (Param_0x5280_0x52A0_Minus_A(2) == 1)
          RenewEEPOMflag_A = 1;
	    }
	  }
	  break;
	  case KEY_FLOWRATELONGPRESSPLUS_A :  //A流量 ++
	  {
	    if ((SysSetParam[SysInterface.BeSelectNum - 1].PumpOffOnA == ON) && (SysRunData.PumpDrain_A == 0))
	    {
		    if (Param_0x52B0_0x52D0_Add_A(2) == 1)
		      RenewEEPOMflag_A = 1;
      }
	  }
	  break;		
 
	  default : break;
  }

  if ((SysRunData.KeyLongPressValue == KEY_NONE) && (SysRunData.MotorNum == MotorNum1))
  {
	  if (RenewEEPOMflag_B == 1)
	  {
	    RenewEEPOMflag_B = 0;

	    temp = (SysInterface.BeSelectNum != 5) ? 0x21 : 0x23;
	    EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);
	  }
		if (RenewEEPOMflag_A == 1)
		{
		  RenewEEPOMflag_A = 0;
	    temp = (SysInterface.BeSelectNum != 5) ? 0x25 : 0x27;
	    EEPROM_AT24CXX_Write(temp, &SysRunData.PumpFlowEEPOM, 1);		
		}
		
  }
}

//============================================================================
//屏”长按键“任务初始化 150
//============================================================================
/* USER CODE BEGIN Header_SCREENKEYLONGTaskFunc */
/**
* @brief Function implementing the SCREENKEYLONGTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SCREENKEYLONGTaskFunc */
void SCREENKEYLONGTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN SCREENKEYLONGTaskFunc */
  /* Infinite loop */
  ScreenKey_LongPressTask();
  /* USER CODE END SCREENKEYLONGTaskFunc */
}

void ScreenKey_LongPressTaskInit(void)
{
  /* definition and creation of SCREENKEYLONGTask */
	Kernel_TaskCreate(&SCREENKEYLONGTaskHandle, SCREENKEYLONGTaskFunc);
	Kernel_TaskStart(&SCREENKEYLONGTaskHandle, KERNEL_TASK_ALWAYS, 150);
}

//============================================================================
// 函数名称: ScreenKey_ParamSet()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void ScreenKey_ParamSet(void)
{
  uint8_t index = 0;

  if (SysRunData.MotorNum == MotorNone)  //当前无被选中的电机
  {
	  SysRunData.KeyValue = KEY_NONE;
	  SysRunData.KeyLongPressValue = KEY_NONE;

	  return ;
  }

  index = SysInterface.BeSelectNum - 1;

  //2号电机且未运行,（PXBA、PXBB）
  if ((SysRunData.MotorNum == MotorNum2) && (SysRunData.MotorRun == MotorStop)) //&& Stepping_Speed == 0 Motor_Number
  {
	  switch (SysRunData.KeyValue)
	  {
	    case KEY_AUTOMANUAL :  //刀具”手动/自动“设别
	    {
		    SysRunData.KeyValue = KEY_NONE;

		    SysRunData.BeepTimeMS = 100;

		    if (SysSetParam[index].DJAutoGetFlag == 0)  //刀具”自动设别“ 开关 0：自动设别 1：手动选择
		    {
		      SysSetParam[index].DJAutoGetFlag = 1;  //手动选择

		      SysRunData.DJManualRefreshFlag = 1;  //更新刀具参数的标志 1：

		      SysSetParam[index].DJSetPDMT = 1;  //默认为”磨“

		      SysSetParam[index].MotorModel = 2;  //正向
		    }
		    else
		    {
		      SysSetParam[index].DJAutoGetFlag = 0;	//自动设别

//		    SysSetParam[index].DJSetPDMT = 0;  //默认为”刨“sscdel

//		    SysSetParam[index].MotorModel = 1;  //往复sscdel

					SysRunData.CutterOFF = 0;
		      SysRunData.ReciveOKTime = 120;
		    }
	    }
	    break;
	    case KEY_PLANING :  //刨
	    {
	      SysRunData.KeyValue = KEY_NONE;

		    if ((SysSetParam[index].DJAutoGetFlag == 1) && (SysSetParam[index].DJSetPDMT != 0))  //刀具信息 ”手动选择“ 刨刀
		    {
		      SysRunData.BeepTimeMS = 100;

		      SysSetParam[index].DJSetPDMT = 0;  //0刨刀  1磨头

		      SysRunData.DJManualRefreshFlag = 1;  //更新2号刀具参数
		    }
	    }
  	  break;
	    case KEY_GRINDINGHEAD :  //磨
	    {
		    SysRunData.KeyValue = KEY_NONE;

		    if ((SysSetParam[index].DJAutoGetFlag == 1) && (SysSetParam[index].DJSetPDMT != 1))  //刀具信息 ”手动选择“ 磨头
		    {
		      SysRunData.BeepTimeMS = 100;

		      SysSetParam[index].DJSetPDMT = 1;  //0刨刀  1磨头

		      SysRunData.DJManualRefreshFlag = 1;  //更新2号刀具参数
		    }
	    }
	    break;
	    case KEY_OPENLEFT :  //左开口
	    {
		    SysRunData.KeyValue = KEY_NONE;

		    if ((SysSetParam[index].DJSetPDMT == 0) && (SysSetParam[index].ReciprocatingFlag == 1))  //SysRunData.DXWFFlag
		    {
		      SysRunData.BeepTimeMS = 100;
          Param_0x2A04_RightAngleSet();
		      
		    }
	    }
	    break;
	    case KEY_OPENRIGHT :  //右开口
	    {
		    SysRunData.KeyValue = KEY_NONE;

		    if ((SysSetParam[index].DJSetPDMT == 0) && (SysSetParam[index].ReciprocatingFlag == 1))
		    {
		      SysRunData.BeepTimeMS = 100;

		      Param_0x2A03_LeftAngleSet();
		    }
	    }
	    break;
	    default : break;
	  }
  }

  //
  if (((SysRunData.KeyValue >= KEY_AUTOMANUAL) && (SysRunData.KeyValue <= KEY_OPENRIGHT)) || (SysRunData.KeyValue == KEY_NONE))
  {
    SysRunData.KeyValue = KEY_NONE;
	  return ;
  }

  switch (SysRunData.KeyValue)
  {
    case KEY_EMPTY_B :  //排空_B / 注水启动
	  {
	    SysRunData.KeyValue = KEY_NONE;
			
      if (SysRunData.PumpModel_B == 2)
			{
				SysRunData.BeepTimeMS = 100;
				SysRunData.PumpPourIntoONOFF_B = 1;
 				SysRunData.FlowRateB = SysRunData.PumpPourIntoVelocityB; 
			}			
      else
      {
				if ((SysSetParam[index].PumpOffOnB == 1) && (SysRunData.MotorRun == MotorStop) && (SysModelConfig.HandlePortB == 1))
				{
					SysRunData.BeepTimeMS = 100;
					Param_0x2411_Pump_Drain_B();
				}			
			}

	  }
	  break;
	  case KEY_FLOWRATESWITCH_B :  //流量_B 开关 / 注水停止
	  {
	    SysRunData.KeyValue = KEY_NONE; 
	    if (SysRunData.PumpModel_B == 2)
			{
				SysRunData.BeepTimeMS = 100;
				SysRunData.PumpPourIntoONOFF_B = 0;			
			}				
		  else
      {
				if((SysRunData.MotorRun == MotorStop) && (SysModelConfig.HandlePortB == 1))
				{
					SysRunData.BeepTimeMS = 100;
					Param_0x2406_Pump_OnOff_B();
				}
			}
	  }
		break;
	  case KEY_FLOWRATESWITCH_A :  //流量_A 开关 / 注水停止
	  {
	    SysRunData.KeyValue = KEY_NONE;
		  if (SysRunData.PumpModel_A == 2)//灌注
			{
				SysRunData.BeepTimeMS = 100;
				SysRunData.PumpPourIntoONOFF_A = 0;			
			}				
		  else
      {			
				if((SysRunData.MotorRun == MotorStop) && (SysModelConfig.HandlePortA == 1))		
				{
					SysRunData.BeepTimeMS = 100;
					Param_0x2413_Pump_OnOff_A();
				}
			}
	  }	
    break; 		
    case KEY_EMPTY_A :  //排空_A  / 注水启动
	  {
			SysRunData.KeyValue = KEY_NONE;
			
      if (SysRunData.PumpModel_A == 2)
			{
				SysRunData.BeepTimeMS = 100;
				SysRunData.PumpPourIntoONOFF_A = 1;
 				SysRunData.FlowRateA = SysRunData.PumpPourIntoVelocityA; 
			}	
		  else
      {
				if ((SysSetParam[index].PumpOffOnA == 1) && (SysRunData.MotorRun == MotorStop) && (SysModelConfig.HandlePortA == 1))
				{
					SysRunData.BeepTimeMS = 100;
					Param_0x2414_Pump_Drain_A();
				}
			}
	  }		
	  break;
    case KEY_SWITCH_A :  //切换A 不能灌注
	  {
////			SysRunData.KeyValue = KEY_NONE;
////      if(SysRunData.MotorRun == MotorStop && SysModelConfig.HandlePortA == 0)
////	    {
////	      SysRunData.BeepTimeMS = 100;
////        Param_0x24010001_Pump_Drain_A();
////	    }
	  }		
	  break;	
    case KEY_SWITCH_B :  //切换B
	  {
			SysRunData.KeyValue = KEY_NONE;
      if(SysRunData.MotorRun == MotorStop && SysModelConfig.HandlePortB == 0)
	    {
	      SysRunData.BeepTimeMS = 100;
        Param_0x24010004_Pump_Drain_B();
	    }
	  }		
	  break;			
	  case KEY_FOOTCTRL :  //脚踏选择
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				  (((SysInterface.HandleType[1] == Handle_Type_22) && ((index + 1) == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && ((index + 1) == 5)))) 	//Handle_Status_Flag2  State_Handle_Connect2
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2409_PedalSelect();
	    }
	  }
	  break;
	  case KEY_MANUALCTRL :  //手控选择
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysFootPedalData.FootPedalConnectOkNo == Connect) && (SysRunData.MotorRun == MotorStop) && \
				  (((SysInterface.HandleType[1] == Handle_Type_22) && ((index + 1) == 2)) || ((SysInterface.HandleType[4] == Handle_Type_22) && ((index + 1) == 5))))   //Handle_Status_Flag2  State_Handle_Connect2
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2410_FingerControl();
	    }
	  }
	  break;
	  case KEY_SPEEDREDUCE :  //速度-
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    SysRunData.BeepTimeMS = 100;

	    Param_0x5010_0x5030_Minus(1);
	  }
	  break;
	  case KEY_SPEEDPLUS :  //速度+
    {
	    SysRunData.KeyValue = KEY_NONE;

	    SysRunData.BeepTimeMS = 100;

	    Param_0x5050_0x5070_Add(1);
    }
	  break;
	  case KEY_FLOWRATEREDUCE_B :  //B流速-
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysRunData.PumpModel_B != 0) && (SysRunData.PumpDrain_B == 0))
	    {
 		    SysRunData.BeepTimeMS = 100;
		    Param_0x5210_0x5230_Minus_B(1);
	    }
	  }
	  break;
	  case KEY_FLOWRATEPLUS_B :  //B流速+
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysRunData.PumpModel_B != 0) && (SysRunData.PumpDrain_B == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x5250_0x5270_Add_B(1);
	    }
	  }
	  break;
	  case KEY_FLOWRATEREDUCE_A :  //A流速-
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysRunData.PumpModel_A != 0) && (SysRunData.PumpDrain_A == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x5280_0x52A0_Minus_A(1);
	    }
	  }
	  break;		
	  case KEY_FLOWRATEPLUS_A :  //A流速+
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysRunData.PumpModel_A != 0) && (SysRunData.PumpDrain_A == 0))
	    {
        SysRunData.BeepTimeMS = 100;
		    Param_0x52B0_0x52D0_Add_A(1);
	    }
	  }
	  break;			
	  case KEY_FREQREDUCE :  //频率-
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if (SysRunData.MotorNum != MotorNone)
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x5110_0x5130_Minus(1);
	    }
	  }
	  break;
	  case KEY_FREQPLUS :  //频率+
	  {
	    SysRunData.KeyValue = KEY_NONE;

      if (SysRunData.MotorNum != MotorNone)
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x5150_0x5170_Add(1);
	    }
	  }
	  break;
	  case KEY_FORWARD :  //正向
	  {
	    SysRunData.KeyValue = KEY_NONE;

      if(SysRunData.MotorRun == MotorStop)
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2407_ForwardDirection();
	    }
	  }
	  break;
	  case KEY_REVERSE :  //反向
	  {
	    SysRunData.KeyValue = KEY_NONE;

      if(SysRunData.MotorRun == MotorStop)
	    {
	      SysRunData.BeepTimeMS = 100;

		    Param_0x2412_OppositeDirection();
	    }
	  }
	  break;
	  case KEY_RECIPROCATING :  //往复
	  {
	    SysRunData.KeyValue = KEY_NONE;

      if ((SysRunData.MotorRun == MotorStop) && (SysSetParam[index].ReciprocatingFlag == 1))
	    {
        SysRunData.BeepTimeMS = 100;

		    Param_0x2408_ToAndFro();
	    }
	  }
	  break;
	  case KEY_HANDLEONE :  //手柄1
	  {
	    SysRunData.KeyValue = KEY_NONE;

	    if ((SysRunData.MotorRun != MotorStop) || (SysRunData.HandleAllowSwitchFlag != 0) || (SysUIDisplayData.UIDisplay0x1500 == 0))
	      return ;
			
	    if (SysInterface.BackgroundPag == 2)
	    {
        SysInterface.LcdKeyValue = 1;
	      SysInterface.DisplayUpdateFlag = 1;
				SysModelConfig.HandlePortA = 1;
				SysModelConfig.HandlePortB = 0;				
	    }
	  }
    break;
    case KEY_HANDLETWO :  //手柄2
    {
	    SysRunData.KeyValue = KEY_NONE;

      if ((SysRunData.MotorRun != MotorStop) || (SysRunData.HandleAllowSwitchFlag != 0) || (SysUIDisplayData.UIDisplay0x1501 == 0))
	      return ;

	    if (SysInterface.BackgroundPag == 2)  //
	    {
	      SysInterface.LcdKeyValue = 2;
	      SysInterface.DisplayUpdateFlag = 1;
				SysModelConfig.HandlePortA = 0;
				SysModelConfig.HandlePortB = 1;				
	    }
    }
    break;
////    case KEY_HANDLETHREE :  //手柄3
////    {
////	    SysRunData.KeyValue = KEY_NONE;

////	    OnLineCnt = Data_GetOnLineCnt();

////      if ((SysRunData.MotorRun != MotorStop) || (SysRunData.HandleAllowSwitchFlag != 0) || (OnLineCnt < 3))
////	      return ;

////      if (SysInterface.BackgroundPag == 4)  //Background
////	      SysInterface.LcdKeyValue = 3;
////	    else
////	      SysInterface.LcdKeyValue = 4;

////	    SysInterface.DisplayUpdateFlag = 1;
////    }
////    break;
////    case KEY_HANDLEFOUR :  //手柄4
////    {
////	    SysRunData.KeyValue = KEY_NONE;

////	    OnLineCnt = Data_GetOnLineCnt();

////	    if ((SysRunData.MotorRun == MotorStop) && (SysRunData.HandleAllowSwitchFlag == 0) && (OnLineCnt == 4))
////	    {
////        SysInterface.LcdKeyValue = 1;
////	      SysInterface.DisplayUpdateFlag = 1;
////	    }
////    }
////    break;
    case KEY_SECONDGEAR :  //II档
    {
	    SysRunData.KeyValue = KEY_NONE;

	    if (((SysRunData.MotorNum == MotorNum3) && (SysSetParam[index].MotorModel != 1)) || \
	        ((SysRunData.MotorNum == MotorNum2) && (SysSetParam[index].MotorModel != 1)) || \
	         (SysRunData.MotorNum == MotorNum1))
	    {
	      SysRunData.BeepTimeMS = 100;

	      Param_0x2405_Er();
      }
    }
    break;
    default : break;
  }
}

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
//    帧头  | 长度 | 指令 | 变量地址  | 读出长度 | 读出的数据
//0x5A 0xA5 | 0x06 | 0x83 | 0x10 0x03 |   0x01   | 0x00 0x1F
//============================================================================
void ScreenKey_Scan(void)
{
  static uint8_t ScrLOGOKeyCnt = 0;
  uint8_t rlen = 0, slen = 0, i = 0, len = 0;
  uint8_t dat[UART6_MAX_PACKET_SIZE] = { 0 }, dat1[16] = { 0 };

  //读取串口数据
  rlen = Uart6_DMARecvDataPeek(dat);
  if (rlen < 9)   //不够一个数据包大小
    return;

  slen = rlen;

  //查询本帧数据包的帧头0x5A 0xA5
  for (i = 0; i < (rlen - 8); i++)
  {
	  if ((dat[i] == 0x5A) && (dat[i + 1] == 0xA5) && (dat[i + 3] == 0x83))  //帧头 指令
	  {
	    len = dat[i + 2] + 3;

	    if (slen < len)  //剩余长度应满足数据帧长度
		    break ;

	    Common_CopyData(&dat[i], dat1, len);    //截取数据

	    //键值...
	    switch (dat1[4])
	    {
		    case 0x20 :  //第一幅图“LOGO连续点击”进入管理者模式 0_开机界面
		    {
		      if (dat1[5] == 0x01)
		      {
			      if (++ScrLOGOKeyCnt >= 5)
			      {
			        ScrLOGOKeyCnt = 0;
			        SysRunData.KeyValue = KEY_CONTINUOUSCLICK;
			      }
		      }
		    }
		   break;
		  	case 0x24 :  // 
		    {
		      switch (dat1[5])
		      {				
            case 0x00 : // 手柄
						{
			         switch (dat1[8])
							 {
								 	case 0x01 : ScreenKey_PostLegacyAction(24U);  break;//1号手柄
									case 0x02 : ScreenKey_PostLegacyAction(25U);  break;//2号手柄	
//									case 0x01 :SysRunData.KeyValue = KEY_HANDLEONE;  break;//1号手柄
//									case 0x02 :SysRunData.KeyValue = KEY_HANDLETWO;  break;//2号手柄										
							    default : break;								 
							 }
             }break; 
            case 0x01 : // 
						{
			         switch (dat1[8])
							 {
									case 0x01 : ScreenKey_PostLegacyAction(21U); break;	//刨刀
									case 0x02 : ScreenKey_PostLegacyAction(20U); break;  //磨头
									case 0x03 : ScreenKey_PostLegacyAction(22U); break;	//开口左
									case 0x04 : ScreenKey_PostLegacyAction(23U); break;	//开口右
								 	case 0x05 : ScreenKey_PostLegacyAction(36U); break;	//自动识别按钮开关
								 default : break;
							 }
						}break;						
           	case 0x05 :// 注水+，-，排空
						{ 
						
								switch (dat1[8])
								{
									case 0x01 : ScreenKey_PostLegacyAction(7U); break;  //
									case 0x02 : ScreenKey_PostLegacyAction(8U); break;       //
									case 0x03 : ScreenKey_PostLegacyAction(12U); break;    //
//								case 0x04 : SysRunData.KeyValue = KEY_OPENLEFT; break;      //4开口位置 左调整
//								case 0x05 : SysRunData.KeyValue = KEY_OPENRIGHT; break;     //5开口位置 右调整
									default : break;
								}
						}break; 	

					case 0x06 :// 运动方向 正反往复
								{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(13U); break;    //
										case 0x02 : ScreenKey_PostLegacyAction(15U); break;    //
										case 0x03 : ScreenKey_PostLegacyAction(14U); break;    //
										default : break;
									}
							}break;	
						case 0x07 :// 控制方式，脚控手控，触控
									{ 
										switch (dat1[8])
										{
											case 0x01 : ScreenKey_PostLegacyAction(16U); break;    //
											case 0x02 : ScreenKey_PostLegacyAction(17U); break;    //
											case 0x03 : ScreenKey_PostLegacyAction(18U); break;    //
											case 0x04 : ScreenKey_PostLegacyAction(40U); break;
											case 0x05 : ScreenKey_PostLegacyAction(43U); break;
											default : break;
										}
								}break;	
								case 0x08 :// 频率加频率减
								{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(10U); break;      //
										case 0x02 : ScreenKey_PostLegacyAction(9U);	 break;      //
									
										default : break;
									}
								}break;	
								case 0x09 :// 灌注+，-，启动
								{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(5U); break;      //
										case 0x02 : ScreenKey_PostLegacyAction(6U);	 break;      //
										case 0x03 : ScreenKey_PostLegacyAction(11U);	 break;      //
										default : break;
									}
								}break;	
            case 0x20 ://  定标按键
						{ 
							switch (dat1[8])
							{
								case 0x01 : SysRunData.KeyValue = KEY_STORAGEMIN; break;  //存储小值(低值左边)
								case 0x02 : SysRunData.KeyValue = KEY_STORAGEMAX; break;  //存储大值（高值左边）
								case 0x03 : SysRunData.KeyValue = KEY_STORAGEMIN2; break;  //存储小值2（低值右边）
								case 0x04 : SysRunData.KeyValue = KEY_STORAGEMAX2; break;  //存储大值2（高值右边）
								case 0x07 : SysRunData.KeyValue = KEY_STORAMEDIAN; break;  //存储中间值（左边中间）
								case 0x08 : SysRunData.KeyValue = KEY_STORAMEDIAN2; break;  //存储中间值2（右边中间）
								
								default : break;
							}
						}break; 

						default : break;
					}
		    }
		    break;
		    case 0x51 :  //A泵  
		    {
		      switch (dat1[5])
		      {  
						///////////////////A泵/////////////////////////
//		        case 0x10 : SysRunData.KeyValue = KEY_FLOWRATEREDUCE_A; break; //A流量减    按压一次
//			      case 0x20 : SysRunData.KeyLongPressValue = KEY_FLOWRATELONGPRESSREDUCE_A; break; //A流量减-- 持续按压  SysRunData.KeyValue = KEY_FLOWRATELONGPRESSREDUCE_B;
//			      case 0x30 : SysRunData.KeyLongPressValue = KEY_NONE; break; //A流量减--  持续按压松开  SysRunData.KeyValue = KEY_FLOWRATEENDLONGPRESSREDUCE_B;

//		        case 0x50 : SysRunData.KeyValue = KEY_FLOWRATEPLUS_A; break; //A流量加    按压一次
//		        case 0x60 : SysRunData.KeyLongPressValue = KEY_FLOWRATELONGPRESSPLUS_A; break; //A流量减++ 持续按压  SysRunData.KeyValue = KEY_FLOWRATELONGPRESSPLUS_B;
//		        case 0x70 : SysRunData.KeyLongPressValue = KEY_NONE; break; //A流量减++  持续按压松开  SysRunData.KeyValue = KEY_FLOWRATEENDLONGPRESSPLUS_B;						
						default : break;
	        }
		    }
		    break;
		    case 0x52 :   //B泵  
		    {
		      switch (dat1[5])
		      { ///////////////////B泵/////////////////////////
//		        case 0x10 : SysRunData.KeyValue = KEY_FLOWRATEREDUCE_B; break; //B流量减    按压一次
//			      case 0x20 : SysRunData.KeyLongPressValue = KEY_FLOWRATELONGPRESSREDUCE_B; break; //B流量减-- 持续按压  SysRunData.KeyValue = KEY_FLOWRATELONGPRESSREDUCE_B;
//			      case 0x30 : SysRunData.KeyLongPressValue = KEY_NONE; break; //B流量减--  持续按压松开  SysRunData.KeyValue = KEY_FLOWRATEENDLONGPRESSREDUCE_B;

//		        case 0x50 : SysRunData.KeyValue = KEY_FLOWRATEPLUS_B; break; //B流量加    按压一次
//		        case 0x60 : SysRunData.KeyLongPressValue = KEY_FLOWRATELONGPRESSPLUS_B; break; //B流量减++ 持续按压  SysRunData.KeyValue = KEY_FLOWRATELONGPRESSPLUS_B;
//		        case 0x70 : SysRunData.KeyLongPressValue = KEY_NONE; break; //B流量减++  持续按压松开  SysRunData.KeyValue = KEY_FLOWRATEENDLONGPRESSPLUS_B;
						default : break;
	        }
		    }
		    break;
		    case 0x53 :  //转速  
		    {
		      switch (dat1[5])
		      {
		        case 0x10 : ScreenKey_PostLegacyAction(2U); break;  //速度减    按压一次
		       // case 0x20 : legacy_key=2; break; //速度减--  持续按压  SysRunData.KeyValue = KEY_SPEEDLONGPRESSREDUCE;
			      //case 0x30 : SysRunData.KeyLongPressValue = KEY_NONE; break;  //速度减--  持续按压松开  SysRunData.KeyValue = KEY_SPEEDENDLONGPRESSREDUCE;

			      case 0x50 : ScreenKey_PostLegacyAction(4U); break;  //速度加    按压一次
			    //  case 0x60 : legacy_key=4; break; //速度加++  持续按压  SysRunData.KeyValue = KEY_SPEEDLONGPRESSPLUS;
			    //  case 0x70 : SysRunData.KeyLongPressValue = KEY_NONE; break;  //速度加++  持续按压松开  SysRunData.KeyValue = KEY_SPEEDENDLONGPRESSPLUS;
			      default : break;
		      }
		    }
		    break;	

			case 0x54 :  //转速  
		    {
		      switch (dat1[5])
		      {
		        case 0x10 : ScreenKey_PostLegacyAction(1U); break;  //速度减    按压一次
		       //case 0x20 : legacy_key=1; break; //速度减--  持续按压  SysRunData.KeyValue = KEY_SPEEDLONGPRESSREDUCE;
			     // case 0x30 : SysRunData.KeyLongPressValue = KEY_NONE; break;  //速度减--  持续按压松开  SysRunData.KeyValue = KEY_SPEEDENDLONGPRESSREDUCE;

			      case 0x50 : ScreenKey_PostLegacyAction(3U); break;  //速度加    按压一次
			    //  case 0x60 :legacy_key=3; break; //速度加++  持续按压  SysRunData.KeyValue = KEY_SPEEDLONGPRESSPLUS;
			      //case 0x70 : SysRunData.KeyLongPressValue = KEY_NONE; break;  //速度加++  持续按压松开  SysRunData.KeyValue = KEY_SPEEDENDLONGPRESSPLUS;
			      default : break;
		      }
		    }
		    break;
			case 0x55:
				 switch (dat1[5])
					{
						 case 0x10 : ScreenKey_PostLegacyAction(41U); break;  //触控启动
						case 0x30 : ScreenKey_PostLegacyAction(42U); break;  //触控停止
					}
					break;
							
		    default : break;
				
	    }

//	    //参数设置---键值范围
//	    if ((SysRunData.KeyValue > KEY_CONTINUOUSCLICK) && (SysRunData.KeyValue <= KEY_SWITCH_B))
//	      ScreenKey_ParamSet();

	    Common_Memset(0, dat1, 15);
  	  i += (len - 1);
	    slen -= len;
	  }
  }
}

//============================================================================
//屏”按键“串口接收的任务初始化 22
//============================================================================
/* USER CODE BEGIN Header_SCREENKEYTaskFunc */
/**
* @brief Function implementing the SCREENKEYTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SCREENKEYTaskFunc */
void SCREENKEYTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN SCREENKEYTaskFunc */
  /* Infinite loop */
	
  ScreenKey_Scan();
  /* USER CODE END SCREENKEYTaskFunc */
}

void ScreenKey_ScanInit(void)
{
  /* definition and creation of SCREENKEYTask */
	Kernel_TaskCreate(&SCREENKEYTaskHandle, SCREENKEYTaskFunc);
	Kernel_TaskStart(&SCREENKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
