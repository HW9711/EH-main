//warn.c

#include "stm32f4xx_hal.h"
#include "warn.h"

#include "data.h"
#include "pedal.h"
#include "motor.h"
#include "iwdg.h"
#include "screen.h"
#include "delay.h"
//#include "adc.h"
#include "pump.h"
#include "board.h"
#include "drivectrl.h"
#include "handlekey.h"
 
#include "app_task.h"

task_t WARNERRSCANTaskHandle;
task_t WARNSTATUSSCANTaskHandle;

static uint16_t KEY_ADC_time = 0;
static uint8_t KEY_ADC_F = 0;

//============================================================================
//脚踏、手控皆在线时，扫描手控被选中时踩脚踏、脚踏被选中时按手控
//============================================================================
void Warn_RunErrScanTask_Fun(void)
{
  uint8_t temp1 = Data_GetOnLineCnt();

  static uint8_t Warn_time = 0;
  static uint8_t Warn_time1 = 0;

  if (SysRunData.StartingMethod == ManualCtrl)  //手柄控制
  {
	  if ((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == NO_Press) && \
		    (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) && \
		    (SysFootPedalData.FootPedalConnectOkNo == Connect))  //脚踏运行
	    SysRunData.WarnID = 13;	//手控运行脚踏报警

    if(SysRunData.StuckFlag == Error ) //手柄脚踏已连接，然后电机启动过载或卡死:电机不转动并且电流达到极限值
	    SysRunData.WarnID = 5;
  }
  else //脚踏控制
  {
	  if ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) ||     
			  (SysFootPedalData.FootPedalADValue_Right > (SysFootPedalData.FootPedalMemoryLValue_Right + FootPedalValueOffset)) || 
		    (SysFootPedalData.FootPedalADValue_Left > (SysFootPedalData.FootPedalMemoryLValue_Left + FootPedalValueOffset)))
	  {
	    if ((SysFootPedalData.FootPedalConnectOkNo == Connect) && (temp1 < 1)) //手柄均未连接，然后启动脚踏	Handle_Number
		    SysRunData.WarnID = 1;
	    else if ((SysFootPedalData.FootPedalConnectOkNo == Connect) && (temp1 > 0)) //手柄脚踏都连接 Handle_Number
	    {
//	      if ((SysRunData.DJFlag == 0) && (SysRunData.MotorNumber == 2)) //直流有霍尔无刷 Motor_Number
//		    SysRunData.WarnDisplayFlag = 2;  //刀具未连接，然后启动脚踏  【无法触发，因为(SysRunData.DJFlag == 0)永远不会被满足】

		    if (SysRunData.StuckFlag == Error)  //手柄脚踏已连接，然后电机启动过载或卡死:电机不转动并且电流达到极限
		      SysRunData.WarnID = 5;
	    }

	    if ((SysRunData.HALLErrFlag == 1) && (SysRunData.MotorNum == MotorNum2)) //霍尔错误  Handle_Status_Flag2
	    {
		    Warn_time++;
		    if (Warn_time >= 10)  //1s
		    {
		      Warn_time = 0;

		      //霍尔错误
		      SysRunData.WarnID = 6;
		    }
	    }
	  }
  }

  if ((SysFootPedalData.FootPedalConnectOkNo == Connect) && ((SysFootPedalData.FootPedalMemoryLValue == 0) || \
	    (SysFootPedalData.FootPedalMemoryHValue == 0) || (SysFootPedalData.FootPedalReadFlag == Error)))  //脚踏存储值读取错误
    SysRunData.WarnID = 7;

  if (SysRunData.EncryptionCheckFlag == Error) //UID错误
  {
	  SysRunData.WarnID = 9;
  }

  //2、3驱动板通信判断
  {
	  if ((SysRunData.MotorNum != MotorNum2) && (SysRunData.MotorNum != MotorNum3))
	  {
	    Warn_time1 = 0;
	      return ;
	  }

	  if (Warn_time1++ < 8)  //900ms
	    return ;

	  Warn_time1 = 0;

	  if (SysRunData.CommunicatFlag == No_Connect)
	  {
	    SysRunData.WarnID = 11;

	    //Motor_Stop_Motor(Normal_stop, SysRunData.MotorNumber);  //Stop_Motor(Normal_stop); Motor_Number
	  }
  }
}

//============================================================================
//  100
//============================================================================
/* USER CODE BEGIN Header_WARNERRSCANTaskFunc */
/**
* @brief Function implementing the WARNERRSCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WARNERRSCANTaskFunc */
void WARNERRSCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN WARNERRSCANTaskFunc */
  /* Infinite loop */
  Warn_RunErrScanTask_Fun();
  /* USER CODE END WARNERRSCANTaskFunc */
}

void Warn_RunErrScanTask_Init(void)
{
  /* definition and creation of WARNERRSCANTask */
	app_task_create(&WARNERRSCANTaskHandle, WARNERRSCANTaskFunc);
	app_task_start(&WARNERRSCANTaskHandle, APP_TASK_ALWAYS, 100);
}

//============================================================================
//============================================================================
uint8_t Warn_KeyValueScan(void)
{
	//LDY
	#if 0
  uint16_t KEY_ADC = 0;

  if (SysInterface.InterfaceSwitchNo2 == 1)
    KEY_ADC = Adc_GetData(2, 25);  //1ms  10
  else
	  KEY_ADC = Adc_GetData(0, 25);  //1ms  4

  if (KEY_ADC < 3080)  //2.5v
	#else
	if (HandleKey_GetKeyValue(SysInterface.InterfaceSwitchNo2 - 1))
	#endif
  {
    if(++KEY_ADC_time < 3)   //3
	    return 0;

	  KEY_ADC_time = 0;

	  if (KEY_ADC_F != 0)
	    return 0;

	  KEY_ADC_F = 1;

	  if ((SysRunData.MotorNum == MotorNum2) && (SysRunData.StartingMethod == ManualCtrl))
	  {
			KEY_ADC_time = 0;

	    if (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == NO_Press)
	    {
		    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = Press;
	    }
	    else
	    {
	      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

		    SysRunData.StartingMethod = ManualCtrl;

		    SysRunData.StuckFlag = No_Error;
		    SysRunData.StuckFlag3 = No_Error;

				//停止电机.....
        //Motor_ErrorEmergencyStop_Ctrl(150);  //390ms

		    return 1;
	    }
	  }
  }
  else
  {
	  KEY_ADC_time = 0;
	  KEY_ADC_F = 0;
  }

  return 0;
}

//============================================================================
//报警状态扫描
//============================================================================
void Warn_StatusScanTask(void)
{
	#if 0
  uint16_t KEY_ADC = 0;
	#endif

	uint8_t temp = 0;

	uint8_t WhileTimeCnt = 0;  //报警while循环时间
  uint8_t MotorStopCntTime = 0;  //电机心跳发送计时
  uint16_t HandleFlagTime = 0;  //手柄连接判断计数

  if ((SysRunData.StuckFlag3 == Error) || (SysRunData.StuckFlag == Error))  //手柄脚踏已连接，然后电机启动过载或卡死:电机不转动并且电流达到极限值
	  SysRunData.WarnID = 5;

  //无异常，返回
  if (SysRunData.WarnID == 0)
  {
	  SysRunData.WarningBeep = 0;

	  return;
  }

  switch(SysRunData.WarnID)
  {
    case 1 :  //手柄未连接，请连接手柄与刀具！
	  {
	    SysRunData.MotorRealSpeed = 0;
 	    SysRunData.MotorRun = MotorStop;

	    HandleFlagTime = 0;
	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
//	    while ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) || \
//		         ((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press) && (SysRunData.MotorNum == MotorNum2)))  //Motor_Number
			
      while  ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) ||    
			       (SysFootPedalData.FootPedalADValue_Right > (SysFootPedalData.FootPedalMemoryLValue_Right + FootPedalValueOffset)) ||  
		         (SysFootPedalData.FootPedalADValue_Left > (SysFootPedalData.FootPedalMemoryLValue_Left + FootPedalValueOffset))  ||  
			       ((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press) && (SysRunData.MotorNum == MotorNum2)) )			
	    {
	      Iwdg_Reset(); //喂狗

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(1); //报警显示 LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1; //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

				//脚踏离线300ms......
		    if (++SysFootPedalData.FootPedalOffTimes >= 10)
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

		      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行
		      SysFootPedalData.FootPedalMemoryHValue = 0;
		      SysFootPedalData.FootPedalMemoryLValue = 0;

		      break;
		    }

		    //PXBA 手控按键判断500ms......
		    if (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press)
	 	    {
		      if (++HandleFlagTime >= 50)
		      {
			      HandleFlagTime = 0;

			      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

			      SysRunData.StartingMethod = ManualCtrl;  //手控

			      SysRunData.StuckFlag = No_Error;
			      SysRunData.StuckFlag3 = No_Error;

			      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

			      break;
		      }
		    }

				#ifdef PASeries
				temp = ((H_MD2_STATUS() << 2) | (H_MD3_STATUS() << 1) | H_MD1_STATUS());
				#else
				if (SysInterface.InterfaceSwitchNo2 == 1)
					temp = ((H_MD2_STATUS() << 2) | (H_MD3_STATUS() << 1) | H_MD1_STATUS());
				else
					temp = ((M_D2_STATUS() << 2) | (M_D3_STATUS() << 1) | M_D1_STATUS());
				#endif

		    //PXBA 拔掉判断......
		    if ((temp == 7) && (SysRunData.MotorNum == MotorNum2) && (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press))
		    {
		      if (++HandleFlagTime >= 50)  //50
		      {
			      HandleFlagTime = 0;

			      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

			      SysRunData.StartingMethod = FootCtrl;  //脚控

			      SysRunData.StuckFlag = No_Error;
			      SysRunData.StuckFlag3 = No_Error;

			      SysFootPedalData.FootPedalMemoryHValue = 0;
			      SysFootPedalData.FootPedalMemoryLValue = 0;
			      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

			      break;
		      }
		    }

				//手控按键抬起....
		    if (Warn_KeyValueScan())
		      break;

 	    }
	  }
	  break;
	  case 2 :  //刀具未连接，请连接刀具！
    {

	  }
	  break ;
	  case 3 :  //电机相位错误，请联系售后！
	  {

	  }
 	  break;
	  case 4 : //电机霍尔错误！【×】
	  {
	    /*
	    KEY_H_onoff = 0;

	    while (JT_ADC_Value  > (JT_Value_Offset + JT_Memory_L_value))
	    {
		    Motor_rut = 0;
		    IWDG_Feed(); //喂狗
		    LCD_Show_Error(4); //报警显示
		    if (Warning_Buzzer_Flag == 1)
		    {
		      Warning_Buzzer_Flag = 0;
		      Warning_Beep = 1;//有错误的提示音
		    }
	    }
	    */
    }
	  break;
	  case 5 : //电机过载，请松开脚踏后再运行；或检查刀具是否卡死！
	  {
	    SysRunData.MotorRealSpeed = 0;
	    SysRunData.MotorRun = MotorStop;

	    HandleFlagTime = 0;
	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

      KEY_ADC_F = 1;
	    KEY_ADC_time = 0;

	    SysFootPedalData.FootPedalOffTimes = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) || \
			       ((SysRunData.StartingMethod == ManualCtrl) && (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press)))
	    {
	      Iwdg_Reset();  //喂狗

	      SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(5); //报警显示 LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1; //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

		    if (SysRunData.StartingMethod == FootCtrl)  //脚控判断
		    {
		      if (++SysFootPedalData.FootPedalOffTimes >= 16)   //延时480ms检测脚踏是否连接
		      {
		        SysFootPedalData.FootPedalOffTimes = 0;

			      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

			      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

			      SysRunData.StartingMethod = ManualCtrl;

			      SysRunData.StuckFlag = No_Error;
			      SysRunData.StuckFlag3 = No_Error;

			      SysFootPedalData.FootPedalMemoryLValue = 0;
			      SysFootPedalData.FootPedalMemoryHValue = 0;
			      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

			      break;
		      }
		    }

				//PXBA 按键判断......
		    if (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press)
		    {
		      if (++HandleFlagTime >= 10)
		      {
			      HandleFlagTime = 0;

			      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

			      SysRunData.StartingMethod = ManualCtrl;

			      SysRunData.StuckFlag = No_Error;
			      SysRunData.StuckFlag3 = No_Error;

			      SysFootPedalData.FootPedalADValue = 0;//防止在电机转动时，脚踏断开，电机仍在运行

			      break;
		      }
	      }

				#ifdef PASeries
				temp = ((H_MD2_STATUS() << 2) | (H_MD3_STATUS() << 1) | H_MD1_STATUS());
				#else
				if (SysInterface.InterfaceSwitchNo2 == 1)
					temp = ((H_MD2_STATUS() << 2) | (H_MD3_STATUS() << 1) | H_MD1_STATUS());
				else
					temp = ((M_D2_STATUS() << 2) | (M_D3_STATUS() << 1) | M_D1_STATUS());
				#endif

		    //PXBA 拔掉...
		    if ((temp == 7) && (SysRunData.MotorNum == MotorNum2))
		    {
		      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

		      SysRunData.StartingMethod = FootCtrl;;

		      SysRunData.StuckFlag = No_Error;
		      SysRunData.StuckFlag3 = No_Error;

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

		      break;
		    }

		    if (Warn_KeyValueScan())
					break;

	    }
	  }
	  break;
	  case 6 : //电机霍尔信号线脱落
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    SysRunData.MotorRealSpeed = 0;
	    SysRunData.MotorRun = MotorStop;

	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while ((SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) || \
		         ((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press) && (SysRunData.MotorNum == MotorNum2)))  //Motor_Number
	    {
	      Iwdg_Reset();  //喂狗

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(4); //报警显示LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1; //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

		    if (++SysFootPedalData.FootPedalOffTimes >= 16)  //延时480ms检测脚踏是否连接
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect;  //脚踏没有连接

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

          break;
		    }
	    }
	  }
	  break;
	  case 7 : //脚踏存储值读取错误
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    HandleFlagTime = 0;
	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while (SysFootPedalData.FootPedalConnectFlag == Connect)
	    {
		    Iwdg_Reset();  //喂狗

		    SysRunData.MotorRealSpeed = 0;

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(8); //报警显示LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1;  //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

				//读取脚踏高低值
				if (HandleFlagTime == 0)
				{
					HandleFlagTime = 1;
					Pedal_ReadHValue();
				}
				else
				{
					HandleFlagTime = 0;
					Pedal_ReadLValue();
				}

		    if (((SysFootPedalData.FootPedalMemoryHValue > 750) && (SysFootPedalData.FootPedalMemoryHValue < 950)) && \
			      ((SysFootPedalData.FootPedalMemoryLValue > 500) && (SysFootPedalData.FootPedalMemoryLValue < 700)))	 //脚踏值正常范围 低值500-700  高值750-950
		    {
  		    SysFootPedalData.FootPedalReadFlag = No_Error;
		      SysFootPedalData.FootPedalConnectOkNo = 1;

		      SysRunData.WarningBeep = 1;

		      break;
		    }

		    if (++SysFootPedalData.FootPedalOffTimes >= 10)  //延时300ms检测脚踏是否连接
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行

		      break;
		    }
	    }
	  }
	  break;
	  case 8 :  //系统供电电压不稳定！
	  {

	  }
	  break;
	  case 9 :  //设备加密UID错误
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    HandleFlagTime = 0;
	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while(1)
	    {
		    Iwdg_Reset();  //喂狗

		    SysRunData.MotorRealSpeed = 0;
		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(9); //报警显示LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1; //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 150ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;
	    }
	  }
//	  break;   //执行不到此处，此break;无意义 编译“警告”
	  case 10 :  //手柄型号错误，请联系售后！
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    SysRunData.MotorRealSpeed = 0;
	    SysRunData.MotorRun = MotorStop;

	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while (SysFootPedalData.FootPedalADValue > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset))
	    {
		    Iwdg_Reset();  //喂狗

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(6); //报警显示LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1;//有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();		

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

		    if (++SysFootPedalData.FootPedalOffTimes >= 16)  //延时480ms检测脚踏是否连接
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0; //防止在电机转动时，脚踏断开，电机仍在运行

          break;
		    }
	    }
	  }
	  break;
	  case 11 : //驱动板故障，请联系售后！
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    SysRunData.MotorRealSpeed = 0;
 	    SysRunData.MotorRun = MotorStop;

	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while(SysRunData.DriveBoardConnectFlag == No_Connect)
	    {
	      Iwdg_Reset();  //喂狗

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(7); //报警显示LCD_Show_Error

		    if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1;//有错误的提示音
		    }

				Delay_ms(2);

				//刷新驱动端的串口数据...
		    //.......................

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (WhileTimeCnt++ >= 6)  // ≈ 210ms
		    {
		      WhileTimeCnt = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;
	    }
	  }
	  break;
	  case 12 : //手控报警，脚踏运行！
	  {
	    SysRunData.MotorRealSpeed = 0;

	    SysRunData.MotorRun = MotorStop;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    //LDY
			#if 0
	    if (SysInterface.InterfaceSwitchNo2 == 1)
		    KEY_ADC = Adc_GetData(2, 25);  //1ms  10
	    else
		    KEY_ADC = Adc_GetData(0, 25);  //1ms  4

	    while (KEY_ADC < 3080)
			#else
//			while (HandleKey_GetKeyValue(SysInterface.InterfaceSwitchNo2 - 1)) 
		  while ((KEY1_STATUS() == 0)||(KEY0_STATUS() == 0))
			#endif
	    {
		    Iwdg_Reset();  //喂狗

		    SysRunData.MotorRun = 0;

		    Screen_TipInfo_Update(12); //报警显示LCD_Show_Error

		    //LDY
				#if 0
		    if (SysInterface.InterfaceSwitchNo2 == 1)
		      KEY_ADC = Adc_GetData(2, 25);  //1ms  10
		    else
		      KEY_ADC = Adc_GetData(0, 25);  //1ms  4
				#endif

	      if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1;  //有错误的提示音
		    }

		    Delay_ms(300);
        Motor_ErrorEmergencyStop_Ctrl();

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

	      SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

		    if (++SysFootPedalData.FootPedalOffTimes >= 50)  //延时300ms检测脚踏是否连接
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0;//防止在电机转动时，脚踏断开，电机仍在运行

          break;
		    }
	    }
	  }
	  break;
	  case 13 : //手控，运行脚踏！
	  {
	    SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] = NO_Press;

	    SysRunData.MotorRealSpeed = 0;
	    SysRunData.MotorRun = MotorStop;

	    MotorStopCntTime = 0;
	    WhileTimeCnt = 0;

	    DriveCtrl_PumpFlag_B();  //B泵停止
      DriveCtrl_PumpFlag_A();  //A泵停止
	    while ((SysFootPedalData.FootPedalADValue  > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) || \
		         ((SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press) && (SysRunData.MotorNum == MotorNum2)))
	    {
	      Iwdg_Reset();  //喂狗

		    SysRunData.MotorRun = MotorStop;

		    Screen_TipInfo_Update(13); //报警显示LCD_Show_Error  5ms

	      if (SysRunData.SustainedBuzzerFlag >= 2)
		    {
		      SysRunData.SustainedBuzzerFlag = 0;
		      SysRunData.WarningBeep = 1; //有错误的提示音
		    }

				Delay_ms(2);

				//刷新脚踏的串口数据...
		    PedalRecv_Scan();

				//.....................
				if (WhileTimeCnt++ < 14)
					continue ;

				WhileTimeCnt = 0;

		    if (MotorStopCntTime++ >= 6)  // ≈ 210ms
		    {
		      MotorStopCntTime = 0;
          Motor_ErrorEmergencyStop_Ctrl();
		    }

		    SysHandleData.IntegratedOffTiming[0] = 0;
		    SysHandleData.IntegratedOffTiming[1] = 0;

		    if (++SysFootPedalData.FootPedalOffTimes >= 16)  //延时480ms检测脚踏是否连接
		    {
		      SysFootPedalData.FootPedalOffTimes = 0;

		      SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

		      SysFootPedalData.FootPedalMemoryLValue = 0;
		      SysFootPedalData.FootPedalMemoryHValue = 0;

		      SysFootPedalData.FootPedalADValue = 0;//防止在电机转动时，脚踏断开，电机仍在运行
          break;
		    }
	    }
	  }
	  break;
	  default : break;
  }

  SysRunData.WarnID = 0;

  SysRunData.WarningBeep = 0;

  Screen_TipInfo_Update(0); // 清空报警显示	LCD_Show_Error
}

//============================================================================
//  15
//============================================================================
/* USER CODE BEGIN Header_WARNSTATUSSCANTaskFunc */
/**
* @brief Function implementing the WARNSTATUSSCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WARNSTATUSSCANTaskFunc */
void WARNSTATUSSCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN WARNSTATUSSCANTaskFunc */
  /* Infinite loop */
  Warn_StatusScanTask();
  /* USER CODE END WARNSTATUSSCANTaskFunc */
}

void Warn_StatusScanTask_Init(void)
{
  /* definition and creation of WARNSTATUSSCANTask */
	app_task_create(&WARNSTATUSSCANTaskHandle, WARNSTATUSSCANTaskFunc);
	app_task_start(&WARNSTATUSSCANTaskHandle, APP_TASK_ALWAYS, 15);
}








