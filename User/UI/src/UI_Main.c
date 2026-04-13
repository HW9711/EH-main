//UI_Main.c

#include "stm32f4xx_hal.h"
#include "UI_Main.h"
#include "data.h"
#include "common.h"
#include "lcd.h"
#include "handledata.h"
#include "footpedal.h"
#include "screen.h"
#include "motor.h"
#include "led.h"
#include "bsp_board.h"

#include "kernel_scheduler.h"

kernel_task_t UIMAINTaskHandle;

//显示接口参数结构
static UIInterface UIInterfaceData = { 0 };

//0不显示脚控/手控,
//1脚控未连接 未选中,
//2脚控 已选中 选中,
//3手控 已选中 选中,
//4脚控已连接\手控已选中,
//5脚控已选中\手控已连接
//u8 State_FootPedal = 0;    //脚控 状态

/*
1.接入：当前Cnt > 上一次Cnt
        每次刷新后就将当前的每个在线接口状态+1；
				再一个接入时，刷新的在线状态 与 上一次的在线状态对比，前次的在线状态哪一个没有就是最后接入的

2.断开：当前Cnt < 上一次Cnt
        断开一个时，刷新的在线状态 与 上一次的在线状态对比，刷新的在线状态哪一个没有就是当前断开的
				提取出在线中的最小值即当前该选中的

3.切换：当前Cnt = 上一次Cnt（按键选择）
        按键时，对在线的接口进行提取排序，将当前选中的接口赋最小值，未选中在线接口相应的向后移，然后提取最小值选中刷新显示
*/

//找到最小值的下标
static uint8_t UICommon_InterfaceMin_i(void)
{
  uint8_t i = 0, n = 0, k = 0;
  uint16_t TempInterface1[5][2] = { 0 };

  //对非0的接口排序
  for (i = 0; i < 5; i++)
  {
    if (UIInterfaceData.UIInterfaceS[i] > 0)
	  {
	    TempInterface1[n][0] = UIInterfaceData.UIInterfaceS[i];
	    TempInterface1[n][1] = i;
	    n++;
	  }
  }

  //提取最小值
  k = 0;
  for (i = 1; i < n; i++)
  {
    if (TempInterface1[k][0] > TempInterface1[i][0])
	    k = i;  //最小值下标
  }

  return TempInterface1[k][1];
}

//当前"接入"的接口判断
static uint8_t UICommon_InsertJudgment(void)
{
  uint8_t i = 0, ret = 0;

  for (i = 0; i < 5; i++)
  {
    if ((SysInterface.Interface[i] > 0) && (UIInterfaceData.UIInterfaceS[i] == 0))
      ret = i + 1;
  }

  return ret;
}

//接入时确定了当前的接入的手柄号后更新显示的接口状态
//将已显示的手柄号状态+1，并将当前接入的手柄号状态赋值2
static void UICommon_InsertInterfaceState(uint8_t Number)
{
  uint8_t i = 0;

  for (i = 0; i < 5; i++)
  {
    if (UIInterfaceData.UIInterfaceS[i] > 0)
      UIInterfaceData.UIInterfaceS[i]++;
  }

  UIInterfaceData.UIInterfaceS[Number - 1] = 2;
}

//当前"断开"的接口判断
static uint8_t UICommon_ExitJudgment(void)
{
  uint8_t i = 0, ret = 0;

  for (i = 0; i < 5; i++)
  {
    if ((SysInterface.Interface[i] == 0) && (UIInterfaceData.UIInterfaceS[i] > 0))
      ret = i + 1;
  }

  return ret;
}

//UI显示的手柄状态更新
static uint8_t UICommon_DisInterfaceState(uint8_t *HConn)
{
  uint8_t i = 0, ret = 0xff;

  //无在线手柄
  if (UIInterfaceData.UIInterfaceCnt < 1)
    return ret;

  //在线手柄仅一个
  if (UIInterfaceData.UIInterfaceCnt == 1)
  {
    for (i = 0; i < 5; i++)
	  {
	    if (UIInterfaceData.UIInterfaceS[i] > 0)
	    {
		    HConn[i] = 2;   //2在线且选中，1在线
		    ret = i;
		    break;
	    }
	  }
  }
  else
  {
    ret = UICommon_InterfaceMin_i();

	  //UI确定在线的手柄, 选中的手柄
	  for (i = 0; i < 5; i++)
	  {
	    if (UIInterfaceData.UIInterfaceS[i] == 0)
		    HConn[i] = 0;
	    else if (i == ret)
		    HConn[i] = 2;
	    else
		    HConn[i] = 1;
	  }
  }

  return ret;
}

//按键切换排序
static uint8_t UICommon_KeySwitch(uint8_t *UHO, uint8_t keyV)
{
  uint8_t i = 0;

  uint8_t HandleNum = (UHO[keyV] - 1);   //按键选中的手柄号
  uint8_t ret = UICommon_InterfaceMin_i();   //已显示的选中手柄下标

  if (ret == HandleNum)
    return 0;

  //将已显示被选中的接口的顺序号与当前按键选中的接口的顺序号互换
  //会失去上一次被选中的整体顺序
  for (i = 0; i < 5; i++)
  {
	  if ((i != HandleNum) && (UIInterfaceData.UIInterfaceS[i] > 0))
	    UIInterfaceData.UIInterfaceS[i]++;
  }

  UIInterfaceData.UIInterfaceS[HandleNum] = 2;

  return 1;
}

//============================================================================
// 函数名称: UIMain_Refresh()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 主UI 刷新...
//============================================================================
void UIMain_Refresh(void)
{
  uint8_t OnLineCnt = 0;
  uint8_t BeSelectedNum = 0;   //被选中的接口

  uint8_t HandleComTemp[5] = { 0 };

  uint8_t *result = { 0 };

  //接口的手柄类型
  static uint8_t HandleTypelast[5] = { 0 };

  static uint8_t UIHandleOrderKey[4] = { 0 };

  if ((Common_CompareData(HandleTypelast, SysInterface.HandleType, 5) != 1) || (SysInterface.DisplayUpdateFlag > 0))
  {
	  SysRunData.KeyLongPressValue = KEY_NONE;  //长按功能失能

	  SysFootPedalData.FootPedalLiftFlag = LiftS;
	  SysFootPedalData.FootPedalLiftTimeCnt = 0;

	  if (SysRunData.MotorRun == MotorStop)
	  {
	    OnLineCnt = Data_GetOnLineCnt();   //当前在线手柄数量

	    //0.开机前已插上多个手柄的情况
	    do
	    {
        //1.接入：当前Cnt > 上一次Cnt
        //        每次刷新后就将当前的每个在线接口状态+1；
		    //	      再一个接入时，刷新的在线状态 与 上一次的在线状态对比，前次的在线状态哪一个没有就是最后接入的
		    if (OnLineCnt > UIInterfaceData.UIInterfaceCnt)
		    {
		      BeSelectedNum = UICommon_InsertJudgment();   //最新接入的手柄
		      UICommon_InsertInterfaceState(BeSelectedNum);  //接入后更新显示的手柄状态
				                                                         //显示时选中的是最小的一个
		      UIInterfaceData.UIInterfaceCnt++;            //最新接入的手柄数量
		    }
        //2.断开：当前Cnt < 上一次Cnt
        //        断开一个时，刷新的在线状态 与 上一次的在线状态对比，刷新的在线状态哪一个没有就是当前断开的
        //				提取出在线中的最小值即当前该选中的
		    else if (OnLineCnt < UIInterfaceData.UIInterfaceCnt)
		    {
		      BeSelectedNum = UICommon_ExitJudgment();   //最新断开的手柄
		      UIInterfaceData.UIInterfaceS[BeSelectedNum - 1] = 0;   //断开后更新显示的手柄状态
				                                                         //显示时选中的是最小的一个
		      UIInterfaceData.UIInterfaceCnt--;            //最新接入的手柄数量
		    }
        //3.切换：当前Cnt = 上一次Cnt（按键选择）
        //        按键时，对在线的接口进行提取排序，将当前选中的接口赋最小值，未选中在线接口相应的向后移，然后提取最小值选中刷新显示
        else  //(OnLineCnt == DisplayInterfaceData.DisplayInterfaceCnt) 按键选择
		    {
	        //按键选择时，将选中的那个赋最小值，
		      //显示时选中的是最小的一个
		      if (SysInterface.LcdKeyValue > 0)  //有按键变化
		        UICommon_KeySwitch(UIHandleOrderKey, (SysInterface.LcdKeyValue - 1));

		      SysInterface.LcdKeyValue = 0;
		    }
	    }while(OnLineCnt > UIInterfaceData.UIInterfaceCnt);

	    //4.确定在线的手柄以及被选中（提取除0外的最小值）的手柄
	    BeSelectedNum = UICommon_DisInterfaceState(HandleComTemp);

	    //5.根据被选中的手柄赋初值
	    switch (BeSelectedNum)
	    {
		    case 0xff :
		    {
		      SysRunData.MaxSetMotorSpeed = 0;   //最大转速
		      SysRunData.StartSetMotorSpeed = 0; //起始转速
		      SysRunData.MinSetMotorSpeed = 0;   //最小转速

		      SysRunData.SetISpeed = 0;    // Ⅰ 档设定速度
		      SysRunData.SetIISpeed = 0;   // Ⅱ 档设定速度
		      SysRunData.SetIIISpeed = 0;  // Ⅲ 档设定速度

					SysModelConfig.HandlePortA = 0;
					SysModelConfig.HandlePortB = 0; 
					    
		      SysRunData.FlowRateB = 0;   //流速的设定 30
          SysRunData.FlowRateA = 0;   //流速的设定 30
					
		      SysRunData.Frequency = 0;       //频率的设定

		      SysRunData.MotorNum = MotorNone;        //电机号

		      SysRunData.MotorSetSpeed = SysRunData.StartSetMotorSpeed;    //将起始转速赋值当前转速

		      LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0);   //刀具参数复位 (3号接口)

		      SysHandleData.EPCBuffAA[0][1] = 0; //直径
		      SysHandleData.EPCBuffAA[0][2] = 0; //长度
		      SysHandleData.EPCBuffAA[0][3] = 0; //角度

		      SysHandleData.EPCBuffAA[1][1] = 0; //直径
		      SysHandleData.EPCBuffAA[1][2] = 0; //长度
		      SysHandleData.EPCBuffAA[1][3] = 0; //角度

		      UIInterfaceData.State_diam = 0;   //直径
		      UIInterfaceData.State_length = 0; //长度
		      UIInterfaceData.State_angle = 0;  //角度

		      UIInterfaceData.State_Cutter1 = 0;  //刀具图片   (3号接口)
		      UIInterfaceData.State_Cutter2 = 0;  //刀具信息   (3号接口)

		      UIInterfaceData.State_Info1 = 2; //信息状态转速
		      UIInterfaceData.State_Info2 = 4; //信息状态流量2
		      UIInterfaceData.State_Info3 = 3; //信息状态频率
          UIInterfaceData.State_Info4 = 4; //信息状态流量1
								
	 
		      if (SysFootPedalData.FootPedalConnectOkNo != Connect)
		        UIInterfaceData.State_FootPedal = 1;   //脚控 0不显示脚控/手控, 1脚控未连接 未选中, 2脚控 已选中 选中, 3手控 已选中 选中, 4脚控已连接\手控已选中, 5脚控已选中\手控已连接
		      else
			      UIInterfaceData.State_FootPedal = 2;

		      SysRunData.StartingMethod = FootCtrl; //0脚控 1手控

		      SysRunData.BeepTimeMS = 100;   //蜂鸣器“响”时长
					K1_OFF();
          K2_OFF();	
					Info_A(4);
					Info_B(4);
					Led_HandleState(0);
		    }
		    break;
		    case 0 :
		    {
//					SysInterface.InterfaceSwitchNo1 = 2;
//		      HandleData_No1(0, (uint8_t *)&UIInterfaceData + 6);
//					Led_HandleState(4);
		    }
		    break;
		    case 1 :
		    {
          SysInterface.InterfaceSwitchNo1 = 1;					
		      SysInterface.InterfaceSwitchNo2 = 1;

          SysModelConfig.HandlePortA = 1;
          SysModelConfig.HandlePortB = 0;
					if ((SysInterface.HandleType[1] == Handle_Type_22) || (SysInterface.HandleType[1] == Handle_Type_23))
					{
						HandleData_No2(0, (uint8_t *)&UIInterfaceData + 6);
						K1_ON();
            K2_OFF();
					}
					else
					  HandleData_No21(1, (uint8_t *)&UIInterfaceData + 6);

					R200_K8_2ON();  //2号接口射频读刀具信息切换
					 
					Led_HandleState(1);
		    }
		    break;
		    case 2 :
		    {
		      SysInterface.InterfaceSwitchNo3 = 1;
			    SysModelConfig.HandlePortA = 1;
          SysModelConfig.HandlePortB = 0;
 	        HandleData_No3(0, (uint8_t *)&UIInterfaceData + 6);
					Led_HandleState(1);//ssc-del

	      }
		    break;
		    case 3 :
		    {
		      SysInterface.InterfaceSwitchNo3 = 2;
					SysModelConfig.HandlePortA = 0;
					SysModelConfig.HandlePortB = 1;
 				  HandleData_No3(1, (uint8_t *)&UIInterfaceData + 6);
					Led_HandleState(2);
				
		    }
		    break;
		    case 4 :
		    {
		      SysInterface.InterfaceSwitchNo1 = 2;
		      SysInterface.InterfaceSwitchNo2 = 2;
 
					SysModelConfig.HandlePortA = 0;
					SysModelConfig.HandlePortB = 1;					
					if ((SysInterface.HandleType[4] == Handle_Type_22) || (SysInterface.HandleType[4] == Handle_Type_23))
					{
 					  HandleData_No2(1, (uint8_t *)&UIInterfaceData + 6);
						K1_OFF();
            K2_ON();						
					}
					else
					  HandleData_No21(4, (uint8_t *)&UIInterfaceData + 6);

					R200_K8_5ON();  //2号接口射频读刀具信息切换
			 
					Led_HandleState(2);
		    }
		    break;
		    default : break;
	    }

	    //切换手柄，复位脚踏设置选中框
	    FootPedal_SelectWin(BeSelectedNum);

	    SysInterface.BeSelectNum = 0xff;
	    if (BeSelectedNum != 0xff)
	      SysInterface.BeSelectNum = BeSelectedNum + 1;

	    //保存当前刷新状态, 下次判断刷新与否
	    Common_CopyData(SysInterface.HandleType, HandleTypelast, 5);

	    if (SysInterface.DisplayUpdateFlag > 1)  //脚踏切换手柄，响两声
	      SysRunData.BeepTimeMS =  300;

	    SysInterface.DisplayUpdateFlag = 0;

	    SysRunData.HandleKeyValue[0] = NO_Press;  //2号分离式手柄上的按键 键值
	    SysRunData.HandleKeyValue[1] = NO_Press;  //2号分离式手柄上的按键 键值

	    //6.根据在线手柄数量判断当前该显示的被背景页

//		  LCD_Show_Which_Map(2); //运行界面 //运行界面 背景

	    //一、转速栏、流量栏、频率栏 ”高亮与否“
       Screen_InformationBarImage_Update(UIInterfaceData.State_Info1, UIInterfaceData.State_Info2, UIInterfaceData.State_Info3, UIInterfaceData.State_Info4);

	    //二、手柄栏
	    result = Screen_HandleConnectState_Update(HandleComTemp, SysInterface.HandleType);
      SysInterface.BeSelectIndexUI = result[4];
	    Common_CopyData(result, UIHandleOrderKey, 4);   //保存当前显示的手柄顺序

	    //三、脚踏栏
	    Screen_FootPedalConnectState_Update(UIInterfaceData.State_FootPedal); //脚踏连接图片LCD_Show_FootPedal
 
	    //四、模式栏
	    (BeSelectedNum != 0xFF) ? \
	    Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[BeSelectedNum].ModeForward) : \
	    Screen_ElectricalMachineryDirectionState_Update2(0);// 

      //五、顶栏【刀具信息】
	    if (UIInterfaceData.State_length == 0)  //刀具信息判断  脚踏公头有外部物理损坏。导致插头变形接触不良。
	    {
	      Screen_IntegratedCutterPic_Update(0);    //刀具连接图片LCD_Show_Cutter
		    LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0); //刀具参数

		    if(SysRunData.MotorNum == MotorNum2)  //2号电机  2号手柄信息
		    {
		      if(SysSetParam[BeSelectedNum].DJAutoGetFlag == 0)  //自动设别刀具模式
		        Screen_SeparatingCutterPic_Update(1, SysSetParam[BeSelectedNum].ReciprocatingFlag);  //LCD_Show_Cutter2(1);
		      else  //手动选择模式
		      {
		        if (SysSetParam[BeSelectedNum].DJSetPDMT == 0) //刨刀
			        Screen_SeparatingCutterPic_Update(4, SysSetParam[BeSelectedNum].ReciprocatingFlag);  //LCD_Show_Cutter2(4);
			      else  //磨头
			        Screen_SeparatingCutterPic_Update(5, SysSetParam[BeSelectedNum].ReciprocatingFlag); //LCD_Show_Cutter2(5);
		      }
		    }
		    else
		      Screen_SeparatingCutterPic_Update(0, 0);   //LCD_Show_Cutter2(0);
	    }
	    else
	    {
	      if(SysRunData.MotorNum == MotorNum2)  //2号手柄信息
		    {
		      Screen_IntegratedCutterPic_Update(0); 
					//一体式 刀具连接图片
  		    LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0); //刀具参数

		      if(SysSetParam[BeSelectedNum].DJAutoGetFlag == 0)  //自动设别模式SysRunData.DJAutoManualFlag
		        Screen_SeparatingCutterPic_Update(UIInterfaceData.State_Cutter2, SysSetParam[BeSelectedNum].ReciprocatingFlag);  //LCD_Show_Cutter2(State_Cutter2);   //分离式 刀具连接图片
		      else  //手动选择模式
		      {
			      if (SysSetParam[BeSelectedNum].DJSetPDMT == 0) //刨刀SysRunData.DJManualPDMT
			        Screen_SeparatingCutterPic_Update(4, SysSetParam[BeSelectedNum].ReciprocatingFlag);  //LCD_Show_Cutter2(4);
			      else  //磨头
			        Screen_SeparatingCutterPic_Update(5, SysSetParam[BeSelectedNum].ReciprocatingFlag);  //LCD_Show_Cutter2(5);
		      }
		    }
		    else if(SysRunData.MotorNum == MotorNum3)  //3号手柄信息
		    {  
		      Screen_SeparatingCutterPic_Update(0, 0);  //LCD_Show_Cutter2(0);   //分离式 刀具连接图片
  		    Screen_IntegratedCutterPic_Update(UIInterfaceData.State_Cutter1);    //一体式 刀具连接图片	(3号接口)
		      specidisplay(1, UIInterfaceData.State_length * 5, UIInterfaceData.State_diam, UIInterfaceData.State_angle); //刀具参数	 (3号接口)
		    }
		    else  //1号手柄信息
		    {
		      Screen_SeparatingCutterPic_Update(0, 0);  //LCD_Show_Cutter2(0);
		      Screen_IntegratedCutterPic_Update(0);    //刀具连接图片
		      LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0); //刀具参数
		    }
	    }

	    //Ⅰ、转速
	    if (SysRunData.MotorNum != MotorNone)
	    {
				if( (SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2)){
		    LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed*2);  //最大转速显示
		    LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);   //设定转速显示
				}
				else
				{
				  LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed);  //最大转速显示
					LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);   //设定转速显示
				}
		    LCD_Show_4byte_Number(0x3420, 0);
	    }

	    //Ⅱ、泵之流量
	    if ((SysRunData.MotorNum != MotorNone) && (UIInterfaceData.State_Info2 == 1)) //B泵
	    {
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
				
//				  LCD_Show_4byte_Number(0x3550, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
					LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
				#else
//					LCD_Show_4byte_Number(0x3550, 70);   //70mL/min --> 1.0L/min
				  if(SysRunData.FlowRateB >= 70) {SysRunData.FlowRateB = 70;}
					LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB); 		//流速  SysRunData.NumberFluidSet);
				#endif
//				LCD_Show_4byte_Number(0x3450, 0);
	    }
	    if ((SysRunData.MotorNum != MotorNone) && (UIInterfaceData.State_Info4 == 1))//A泵
	    {
				#ifdef  WATER_UPTAKE//如果定义吸水ssc
				
//				  LCD_Show_4byte_Number(0x3530, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
					LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
				#else
//					LCD_Show_4byte_Number(0x3530, 70);   //70mL/min --> 1.0L/min
				  if(SysRunData.FlowRateA >= 70) {SysRunData.FlowRateA = 70;}
					LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);  //流速  SysRunData.NumberFluidSet);
				#endif
//				LCD_Show_4byte_Number(0x3430, 0);
	    }			
			

	    //Ⅲ、往复之频率
	    if ((SysRunData.MotorNum != MotorNone) && (UIInterfaceData.State_Info3 == 1))
	    {
	      LCD_Show_4byte_Number(0x3460, 0x40800000);
		    LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
//		    LCD_Show_4byte_Number(0x3470, 0);
	    }
	  }
	  else
	  {
	    if ((SysFootPedalData.FootPedalADValue  > (SysFootPedalData.FootPedalMemoryLValue + FootPedalValueOffset)) || (SysRunData.HandleKeyValue[SysInterface.InterfaceSwitchNo2 - 1] == Press))  //踩脚踏运行  SysRunData.KEYHandleNo2OnOff
	    {
		    SysRunData.WarnID = 1;

        //390ms电机运行过程中，手柄进行插拔、或切换，急停电机并报警
		    Motor_ErrorEmergencyStop_Ctrl();
	    }
	  }
  }
}

//============================================================================
// 函数名称: UIMain_RefreshTaskInit()
// 功能描述: 主UI 刷新任务
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
/* USER CODE BEGIN Header_UIMAINTaskFunc */
/**
* @brief Function implementing the UIMAINTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UIMAINTaskFunc */
void UIMAINTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN UIMAINTaskFunc */
  /* Infinite loop */
  UIMain_Refresh();
  /* USER CODE END UIMAINTaskFunc */
}

void UIMain_RefreshTaskInit(void)
{
  /* definition and creation of UIMAINTask */
	Kernel_TaskCreate(&UIMAINTaskHandle, UIMAINTaskFunc);
	Kernel_TaskStart(&UIMAINTaskHandle, KERNEL_TASK_ALWAYS, 50);
}









