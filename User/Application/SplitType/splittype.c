//splittype.c

#include "splittype.h"
#include "radiofreq.h"
#include "data.h"
#include "common.h"
#include "screen.h"
#include "lcd.h"
#include "datahand.h"
#include "uart3.h"
#include "bsp_board.h"

#include "kernel_scheduler.h"

kernel_task_t AUTOMODEGETDATATaskHandle;
kernel_task_t AUTOMODEREADDATATaskHandle;
kernel_task_t CUTTERSCANTaskHandle;

static uint8_t ReciveOk = 0;
static uint8_t EPCBuffold[20] = { 0 };
static uint8_t DJoldVLHZ = 0;
static uint8_t ssc_shoudong_uiflag=0;
static uint8_t ssc_shoudong_rfidflag=0;
uint8_t  SplitType_AutoModeDataRead_Task(uint8_t beep_flag);

//80ms发送一次
void SplitType_AutoModeGetData_Task(void)         //Check_Connect(void)
{
	static uint8_t switch_sign=0;
	static uint8_t	mutual_exclusion_flag=1;
  static uint8_t EPC_Send_time = 0;
  static uint8_t InterfaceSwitchNo2Last = 0;
	static uint8_t  error_times=0;
  uint8_t NO_MASK3_READ_USER[16] = {0xBB, 0x00, 0x39, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x4d, 0x7E}; //无掩码读取USER

  //每次手柄的切换都初始化射频读取刀具信息的计时
	if(Workvalue_s.select_channel!=switch_sign)
	{
		switch_sign=Workvalue_s.select_channel;
		EPCBuffold[14]=0;
	  EPCBuffold[15]=0;
		mutual_exclusion_flag=1;
	}
	if(Workvalue_s.hand_model==PXBA_ONLINE||Workvalue_s.hand_model==PXBB_ONLINE){
		if(Workvalue_s.Handle_mutual_flag==1)
		{
			if(!Workvalue_s.MOTORWorking_flag)
				{
					EPC_Send_time++;
					if(EPC_Send_time%2){
					Uart3_SendPacket(NO_MASK3_READ_USER, 16);  //120ms读标签 两次解析射频数据
					}
					else
					{
						if(SplitType_AutoModeDataRead_Task(mutual_exclusion_flag)==0)
						{
							error_times++;
							if(error_times>=10)
							{
								if(mutual_exclusion_flag){
								//掉线
									KeyBeep_flag=1;
								 error_times=0;
								 specidisplay(0,0,0,0);
								 Tooldisplay(1);
								 EPCBuffold[14]=0;
								 EPCBuffold[15]=0;
								 mutual_exclusion_flag=0;
								}
							}
						}
						else
						{
							mutual_exclusion_flag=1;
							error_times=0;
						}
					}
				 }
				else
				{
					error_times=0;
				}
		}
		else
		{
			mutual_exclusion_flag=1;
				EPCBuffold[14]=0xff;
				EPCBuffold[15]=0xff;
				error_times=0;
		}
	}
	else
	{
		 EPCBuffold[14]=0;
		 EPCBuffold[15]=0;
			error_times=0;
	}
}





//============================================================================
// 函数名称: SplitType_AutoModeGetData_Task()
// 功能描述: 自动模式下刀具数据请求
// 输　  入:
// 输    出:
// 函数说明: 200ms
//============================================================================
void SplitType_AutoModeGetData_Tasks(void)         //Check_Connect(void)
{
  static uint8_t EPC_Send_time = 0;
  static uint8_t InterfaceSwitchNo2Last = 0;

  uint8_t NO_MASK3_READ_USER[16] = {0xBB, 0x00, 0x39, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x4d, 0x7E}; //无掩码读取USER

  //每次手柄的切换都初始化射频读取刀具信息的计时
  if ((SysInterface.InterfaceSwitchNo2 != InterfaceSwitchNo2Last) || (SysRunData.MotorNum != MotorNum2)||ssc_shoudong_rfidflag)
  {
    EPC_Send_time = 0;
	  InterfaceSwitchNo2Last = SysInterface.InterfaceSwitchNo2;
	  return ;
  }

  if (++EPC_Send_time < 4)  //8 * 10ms
	  return ;

  EPC_Send_time = 0;

  Uart3_SendPacket(NO_MASK3_READ_USER, 16);  //120ms读标签 两次解析射频数据
}

/* USER CODE BEGIN Header_AUTOMODEGETDATATaskFunc */
/**
* @brief Function implementing the AUTOMODEGETDATATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AUTOMODEGETDATATaskFunc */
void AUTOMODEGETDATATaskFunc(uint32_t event)
{
  /* USER CODE BEGIN AUTOMODEGETDATATaskFunc */
  /* Infinite loop */
  SplitType_AutoModeGetData_Task();
  /* USER CODE END AUTOMODEGETDATATaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 48
 */
//============================================================================
void SplitType_AutoModeGetData_Init(void)
{
  /* definition and creation of AUTOMODEGETDATATask */
	Kernel_TaskCreate(&AUTOMODEGETDATATaskHandle, AUTOMODEGETDATATaskFunc);
	Kernel_TaskStart(&AUTOMODEGETDATATaskHandle, KERNEL_TASK_ALWAYS, 80);
}

//============================================================================
// 函数名称: SplitType_AutoModeDataAnalysiss_Task()
// 功能描述: 自动模式下刀具数据读取解析
// 输　  入:
// 输    出:
// 函数说明: 50ms
//============================================================================
uint8_t SplitType_AutoModeDataRead_Task(uint8_t beep_flag)
{
	
  uint16_t crc = 0;
  uint8_t rlen = 0;
  uint8_t EPCresult = 0, index = 0;
  uint8_t dat[UART3_MAX_PACKET_SIZE] = { 0 }, buf[20] = { 0 };

  //读取串口数据
  rlen = Uart3_DMARecvDataPeek(dat);
  if (rlen < 16)   //不够一个数据包大小
	  return 0;

  EPCresult = RadioFreq_Analysiss(dat, buf);  //解析射频板数据【将接收到的刀具数据放入当前选中的刀具信息缓冲中】
  if (EPCresult == 2)
  {
	  crc = Common_Crc16(buf, 14);	 //刀具数据CRC计算
	  if (crc == ((buf[14] << 8) + buf[15]))
	  {
	    index = SysInterface.InterfaceSwitchNo2 - 1;

	    Common_CopyData(buf, paoxueSpeciValue_F, 16);

			//最新接收数据的CRC与上一次的CRC进行对比判断
	    if ((paoxueSpeciValue_F[14] == EPCBuffold[14]) && (paoxueSpeciValue_F[15] == EPCBuffold[15]))
	    {
				//和上次一样，不做处理
	    }
	    else
	    {
				Tooldisplay(0);
				if(!beep_flag)
					KeyBeep_flag=1;
				//将当前被选中的接口接收到的刀具数据保存
		    EPCBuffold[14] = paoxueSpeciValue_F[14];
		    EPCBuffold[15] = paoxueSpeciValue_F[15];
				//更新参数不一样
				specidisplay(1,paoxueSpeciValue_F[1]*5,paoxueSpeciValue_F[2],paoxueSpeciValue_F[3]);//显示刀具信息
				if(paoxueSpeciValue_F[5]>>4==3)
				{
					//刨
					Workvalue_s.tool_model=1;
					//界面通知方向，转速
					AutomaticAxtion(1);
					Workvalue_s.fenti_jiyi_flag=1;
				}
				else
				{
					AutomaticAxtion(0);
					//磨
					Workvalue_s.tool_model=0;
					//界面通知方向，转速
					Workvalue_s.fenti_jiyi_flag=0;
				}
			}
			return 1;
	  }
  }
}

/* USER CODE BEGIN Header_AUTOMODEREADDATATaskFunc */
/**
* @brief Function implementing the AUTOMODEREADDATATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AUTOMODEREADDATATaskFunc */
void AUTOMODEREADDATATaskFunc(uint32_t event)
{
  /* USER CODE BEGIN AUTOMODEREADDATATaskFunc */
  /* Infinite loop */
 // SplitType_AutoModeDataRead_Task();
  /* USER CODE END AUTOMODEREADDATATaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 5
 */
//============================================================================
void SplitType_AutoModeDataRead_Init(void)
{
  /* definition and creation of AUTOMODEREADDATATask */
//	cola_timer_create(&AUTOMODEREADDATATaskHandle, AUTOMODEREADDATATaskFunc);
//	cola_timer_start(&AUTOMODEREADDATATaskHandle, TIMER_ALWAYS, 5);
}

//============================================================================
// 函数名称: SplitType_DisconnectUpdata_Task()
// 功能描述: 自动模式下刀具”断开“数据UI刷新
// 输　  入:
// 输    出:
// 函数说明: 1.2s 刀具断开 数据更新
//============================================================================
void SplitType_DisconnectUpdata_Task(void)         //Check_Connect(void)
{
  uint8_t index = 0, index1 = 0;

  SysRunData.ReciveOKTime++;
  if (SysRunData.ReciveOKTime < 12)  //1.2s
	  return ;

  SysRunData.ReciveOKTime = 0;

  if (SysRunData.CutterOFF != 0)
	  return ;

  SysRunData.CutterOFF = 1;

  SysRunData.CutterON = 0;  //允许初始化刀具信息

  if (SysRunData.SystemTime >= 3)  //切换或接上该手柄，设别到手柄断开过了3s
	  SysRunData.BeepTimeMS = 100;

  if (SysRunData.MotorNum != MotorNum2)  //Motor_Number
	  return ;

  index = SysInterface.InterfaceSwitchNo2 - 1;  //当前被选中的手柄
  index1 = SysInterface.BeSelectNum - 1;  //当前被选中的接口

  Screen_IntegratedCutterPic_Update(0);  //LCD_IntegratedCutterPic_Update(0);    //刀具连接图片
  LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0);

  if (SysRunData.DJOldValueFlag[index] == 0)  //0：刨 1：磨
  {
	  SysSetParam[index1].MinMotorSpeed = 500;

	  //SysRunData.MinSetMotorSpeed = 500;
	  if (SysRunData.DJOldInit[index] == 0)
    {
	    SysSetParam[index1].MaxMotorSpeed = 6000;
	    SysSetParam[index1].StartMotorSpeed = 4000;

	    SysSetParam[index1].ISpeed = SysSetParam[index1].MinMotorSpeed;
	    SysSetParam[index1].IISpeed = SysSetParam[index1].StartMotorSpeed;
	    SysSetParam[index1].IIISpeed = SysSetParam[index1].MaxMotorSpeed;

	    SysSetParam[index1].RunMotorSpeed = SysSetParam[index1].StartMotorSpeed;
	    //SysRunData.MaxSetMotorSpeed = 6000;
	    //SysRunData.StartSetMotorSpeed = 4000;

	    SysSetParam[index1].HzSet = 40;

		 if (SysModelConfig.HandlePortA == 1)
		 {
				#ifdef WATER_UPTAKE
				SysSetParam[index1].PumpVelocitySetA = 10;
				#else
				SysSetParam[index1].PumpVelocitySetA = 30;
				#endif

				SysSetParam[index1].PumpOffOnA = 1;		
				SysRunData.FlowRateA = SysSetParam[index1].PumpVelocitySetA;
				SysRunData.PumpONOFF_A = SysSetParam[index1].PumpOffOnA;				 
		 }
		 if (SysModelConfig.HandlePortB == 1)
		 {
				#ifdef WATER_UPTAKE
				SysSetParam[index1].PumpVelocitySetB = 10;
				#else
				SysSetParam[index1].PumpVelocitySetB = 30;
				#endif

				SysSetParam[index1].PumpOffOnB = 1;		
				SysRunData.FlowRateB = SysSetParam[index1].PumpVelocitySetB;
				SysRunData.PumpONOFF_B = SysSetParam[index1].PumpOffOnB;	 
		 }		 


	    SysRunData.Frequency = SysSetParam[index1].HzSet;
 
	    if (SysSetParam[index1].MotorModel != 1)  //往复判断 SysRunData.MotorModel2
	    {
		    SysSetParam[index1].ModeForward = 1;  //3正显
//		    SysSetParam[index1].ModeReciprocating = 3;  //StateWF
//		    SysSetParam[index1].ModeReverse = 1;

				//信息状态流量
				if (SysModelConfig.HandlePortA == 1)
				{
					if (SysRunData.PumpONOFF_A == 1)
						Info_A(1);//Screen_InformationBarImage_Update(1, 2, 5, 1);     //LCD_Show_Info
					else
						Info_A(4);//Screen_InformationBarImage_Update(1, 2, 5, 2);     //LCD_Show_Info					
				}
				//信息状态流量
				if (SysModelConfig.HandlePortB == 1)
				{
					if (SysRunData.PumpONOFF_B == 1)
						Info_B(1);//Screen_InformationBarImage_Update(1, 1, 5, 2);     //LCD_Show_Info
					else
						Info_B(4);//Screen_InformationBarImage_Update(1, 2, 5, 2);     //LCD_Show_Info
				}		

		    SysSetParam[index1].GearPositionHz = 5;
	    }
      else
	    {
		    SysSetParam[index1].ModeForward = 2;  //3往复显
//		    SysSetParam[index1].ModeReciprocating = 4;  //StateWF
//		    SysSetParam[index1].ModeReverse = 1;

				//信息状态流量
				if (SysModelConfig.HandlePortA == 1)
				{
					if (SysRunData.PumpONOFF_A == 1)
						Info_A(1);//Screen_InformationBarImage_Update(1, 2, 1, 1);     //LCD_Show_Info
					else
						Info_A(4);//Screen_InformationBarImage_Update(1, 2, 1, 2);     //LCD_Show_Info					
				}
				//信息状态流量
				if (SysModelConfig.HandlePortB == 1)
				{
					if (SysRunData.PumpONOFF_B == 1)
						Info_B(1);//Screen_InformationBarImage_Update(1, 1, 1, 2);     //LCD_Show_Info
					else
						Info_B(4);//Screen_InformationBarImage_Update(1, 2, 1, 2);     //LCD_Show_Info
				}
				
        SysSetParam[index1].GearPositionHz = 1;

		    LCD_Show_4byte_Number(0x3460, 0x40800000);
//		    LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
//		    LCD_Show_4byte_Number(0x3470, 0);
	    }
    }

    SysSetParam[index1].DJSetPDMT = 0;  //0刨刀  1磨头

	  SysSetParam[index1].ReciprocatingFlag = 1;  //SysRunData.DXWFFlag = 1;
  }
  else
  {
	  //SysSetParam[index1].MotorModel = 2;
	  //SysRunData.MotorModel2 = (SysSetParam[index1].MotorModel == 1) ? 1 : 0;

	  //SysRunData.MinSetMotorSpeed = 3000;

	  SysSetParam[index1].MinMotorSpeed = 3000;
	  if (SysRunData.DJOldInit[index] == 0)
	  {
	    SysSetParam[index1].MaxMotorSpeed = 13000;
	    SysSetParam[index1].StartMotorSpeed = 10000;

	    SysSetParam[index1].ISpeed = SysSetParam[index1].MinMotorSpeed;
	    SysSetParam[index1].IISpeed = SysSetParam[index1].StartMotorSpeed;
	    SysSetParam[index1].IIISpeed = SysSetParam[index1].MaxMotorSpeed;

	    SysSetParam[index1].RunMotorSpeed = SysSetParam[index1].StartMotorSpeed;

	    SysSetParam[index1].HzSet = 40;

			
		 if (SysModelConfig.HandlePortA == 1)
		 {
				#ifdef WATER_UPTAKE
				SysSetParam[index1].PumpVelocitySetA = 10;
				#else
				SysSetParam[index1].PumpVelocitySetA = 30;
				#endif

				SysSetParam[index1].PumpOffOnA = 1;		
				SysRunData.FlowRateA = SysSetParam[index1].PumpVelocitySetA;
				SysRunData.PumpONOFF_A = SysSetParam[index1].PumpOffOnA;				 
		 }
		 if (SysModelConfig.HandlePortB == 1)
		 {
				#ifdef WATER_UPTAKE
				SysSetParam[index1].PumpVelocitySetB = 10;
				#else
				SysSetParam[index1].PumpVelocitySetB = 30;
				#endif

				SysSetParam[index1].PumpOffOnB = 1;		
				SysRunData.FlowRateB = SysSetParam[index1].PumpVelocitySetB;
				SysRunData.PumpONOFF_B = SysSetParam[index1].PumpOffOnB;	 
		 }				

//      SysSetParam[index1].ForwardReverseFlag = 0;  //SysRunData.CWCCW = 0;

	    SysRunData.Frequency = SysSetParam[index1].HzSet;
 
      SysSetParam[index1].GearPositionHz = 5;

	    SysSetParam[index1].ModeForward = 4;  //2显
//	    SysSetParam[index1].ModeReciprocating = 0;  //StateWF
//	    SysSetParam[index1].ModeReverse = 1;

      /*
	    LCD_Show_Picture(0x1304, 224);
	    LCD_Disappear_Number(0x9460);

	    LCD_Disappear_Number(0x9480);

	  
	   
	 
	    */
			//信息状态流量
			if (SysModelConfig.HandlePortA == 1)
			{
				if (SysRunData.PumpONOFF_A == 1)
					Info_A(1);//Screen_InformationBarImage_Update(1, 2, SysSetParam[index1].GearPositionHz, 1);     //LCD_Show_Info
				else
					Info_A(4);//Screen_InformationBarImage_Update(1, 2, SysSetParam[index1].GearPositionHz, 2);     //LCD_Show_Info					
			}
			//信息状态流量
			if (SysModelConfig.HandlePortB == 1)
			{
				if (SysRunData.PumpONOFF_B == 1)
					Info_B(1);//Screen_InformationBarImage_Update(1, 1, SysSetParam[index1].GearPositionHz, 2);     //LCD_Show_Info
				else
					Info_B(4);//Screen_InformationBarImage_Update(1, 2, SysSetParam[index1].GearPositionHz, 2);     //LCD_Show_Info
			}			
      Info_HZ(SysSetParam[index1].GearPositionHz);
			
	  }

	  SysSetParam[index1].DJSetPDMT = 1;  //0刨刀  1磨头

	  SysSetParam[index1].ReciprocatingFlag = 0;  //SysRunData.DXWFFlag = 0;
  }

  //-----------------------------------------------
  {
	  SysRunData.MaxSetMotorSpeed = SysSetParam[index1].MaxMotorSpeed;
	  SysRunData.StartSetMotorSpeed = SysSetParam[index1].StartMotorSpeed;
	  SysRunData.MinSetMotorSpeed = SysSetParam[index1].MinMotorSpeed;

	  SysRunData.SetISpeed = SysSetParam[index1].ISpeed;
	  SysRunData.SetIISpeed = SysSetParam[index1].IISpeed;
	  SysRunData.SetIIISpeed = SysSetParam[index1].IIISpeed;

	  SysRunData.MotorSetSpeed = SysSetParam[index1].RunMotorSpeed;
  }

  //------------------------------------------
  {
    if (SysModelConfig.HandlePortA == 1)
		{
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
//			LCD_Show_4byte_Number(0x3530, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
			LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
			#else
//			LCD_Show_4byte_Number(0x3530, 70);
			LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
			#endif
//			LCD_Show_4byte_Number(0x3430, 0);		
		}		
		
    if (SysModelConfig.HandlePortB == 1)
		{
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
//			LCD_Show_4byte_Number(0x3550, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
			LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
			#else
//			LCD_Show_4byte_Number(0x3550, 70);
			LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
			#endif
//			LCD_Show_4byte_Number(0x3550, 0);		
		}

		if( (SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2)){
			LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed*2);
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
		}
		else
		{
			LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed);
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
		}

	  Screen_SeparatingCutterPic_Update(1, SysSetParam[index1].ReciprocatingFlag);  //LCD_Show_Cutter2(1);   = 1;  //DXWFFlag

	  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index1].ModeForward); //运行模式切换 单向，往复  LCD_Show_Direction
  }
}

//============================================================================
// 函数名称: SplitType_ConnectUpdata_Task()
// 功能描述: 自动模式下刀具”连接“数据UI刷新
// 输　  入:
// 输    出:
// 函数说明: 通过射频读取到一次不同的刀具信息时，初始化参数
//           刀具在线 刷新周期370ms
//============================================================================
void SplitType_ConnectUpdata_Task(void)         //Check_Connect(void)
{
  uint8_t index = 0, index1 = 0;

  if ((ReciveOk != 1) || (SysRunData.MotorNum != MotorNum2))   //Motor_Number
	  return ;

  SysRunData.ReciveOKTime = 0;  //1.2s计时器
  ReciveOk = 0;  //接收到正确的刀具数据标志复位

  if (SysRunData.CutterON != 0)  //是否允许初始化刀具信息
	  return ;

  SysRunData.CutterOFF = 0;

  SysRunData.CutterON = 1;  //新读取到的刀具信息已初始化标志

  if (SysRunData.SystemTime >= 3)  //切换到该手柄，已超过3s设别到刀具
	  SysRunData.BeepTimeMS = 100;

  index = SysInterface.InterfaceSwitchNo2 - 1;  //当前被选中的手柄
  index1 = SysInterface.BeSelectNum - 1;  //当前被选中的接口

  Screen_IntegratedCutterPic_Update(0);    //一体式 刀具连接图片
  LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0); //刀具参数

  //判断读取到的刀具信息是否为空
  if ((SysHandleData.EPCBuffAA[index][0] != 0) && (SysHandleData.EPCBuffAA[index][1] != 0) && (SysHandleData.EPCBuffAA[index][2] != 0))
  {
	  SysRunData.DJOldInit[index] = 1;  //刀具信息已初始化标志

	  //通过判断该刀具的默认运行模式（0x10：正，0x20：反转，0x30：往复）确定其是刨刀还是磨头
	  if ((SysHandleData.EPCBuffAA[index][5] & 0xf0) ==  0x30)   //刨刀【往复】
	  {
	    //------------------------------------------------------
	    //------------------------------------------------------
	    SysRunData.DJOldValueFlag[index] = 0;  //0：刨

	    SysSetParam[index1].ReciprocatingFlag = 1;  //DXWFFlag

      SysSetParam[index1].DJSetPDMT = 0;  //0刨刀

	    SysSetParam[index1].MinMotorSpeed = 500;

 	    if (DJoldVLHZ == 1)
	    {
				ssc_shoudong_uiflag=0;
		    SysSetParam[index1].MaxMotorSpeed = SysHandleData.EPCBuffAA[index][8] * 500;
		    SysSetParam[index1].StartMotorSpeed = SysHandleData.EPCBuffAA[index][9] * 500;

		    SysSetParam[index1].ISpeed = SysSetParam[index1].MinMotorSpeed;
		    SysSetParam[index1].IISpeed = SysSetParam[index1].StartMotorSpeed;
		    SysSetParam[index1].IIISpeed = SysSetParam[index1].MaxMotorSpeed;

		    SysSetParam[index1].RunMotorSpeed = SysSetParam[index1].StartMotorSpeed;

		    SysSetParam[index1].HzSet = SysHandleData.EPCBuffAA[index][10];      //频率的设定
		    if (SysSetParam[index1].HzSet > 40)
		      SysSetParam[index1].HzSet = 40;
				
				if (SysModelConfig.HandlePortA == 1)
				{
					SysSetParam[index1].PumpVelocitySetA = SysHandleData.EPCBuffAA[index][11];   //流速的设定
					 #ifndef WATER_UPTAKE
					if (SysSetParam[index1].PumpVelocitySetA > 70)
						SysSetParam[index1].PumpVelocitySetA = 70;
					#else
					if (SysSetParam[index1].PumpVelocitySetA > 15)
						SysSetParam[index1].PumpVelocitySetA = 15;
					#endif

					SysSetParam[index1].PumpOffOnA = 1;  //每次接上默认打开流量							
				}
				if (SysModelConfig.HandlePortB == 1)
				{
					SysSetParam[index1].PumpVelocitySetB = SysHandleData.EPCBuffAA[index][11];   //流速的设定
					 #ifndef WATER_UPTAKE
					if (SysSetParam[index1].PumpVelocitySetB > 70)
						SysSetParam[index1].PumpVelocitySetB = 70;
					#else
					if (SysSetParam[index1].PumpVelocitySetB > 15)
						SysSetParam[index1].PumpVelocitySetB = 15;
					#endif

					SysSetParam[index1].PumpOffOnB = 1;  //每次接上默认打开流量					
				}				


		    SysSetParam[index1].MotorModel = 1;  //1：往复 2：正向 3：反向

		    SysRunData.Frequency = SysSetParam[index1].HzSet;

		    //SysRunData.MotorModel2 = (SysSetParam[index1].MotorModel == 1) ? 1 : 0;
		    if (SysSetParam[index1].MotorModel != 1)  //往复判断 SysRunData.MotorModel2
		    {
		      SysSetParam[index1].ModeForward = 1;  //3正显
//		      SysSetParam[index1].ModeReciprocating = 3;  //UI --- 往复
//		      SysSetParam[index1].ModeReverse = 1;  //UI --- 反向

		      SysSetParam[index1].GearPositionHz = 5;
	      }
        else
	      {
		      SysSetParam[index1].ModeForward = 2;  //3往复显
//		      SysSetParam[index1].ModeReciprocating = 4;  //UI --- 往复
//		      SysSetParam[index1].ModeReverse = 1;  //UI --- 反向

		      SysSetParam[index1].GearPositionHz = 1;

		      LCD_Show_4byte_Number(0x3460, 0x40800000);
//		      LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
//		      LCD_Show_4byte_Number(0x3470, 0);
	      }
	    }
	  }
	  else  //0x10正向、0x20反向
	  {
	    SysRunData.DJOldValueFlag[index] = 1;  //磨

	    SysSetParam[index1].ReciprocatingFlag = 0;  //DXWFFlag

      SysSetParam[index1].DJSetPDMT = 1;  //1磨头

      SysSetParam[index1].MinMotorSpeed = 3000;

 	    if (DJoldVLHZ == 1)
	    {
					ssc_shoudong_uiflag=0;
	      SysSetParam[index1].MaxMotorSpeed = SysHandleData.EPCBuffAA[index][8] * 500;
		    SysSetParam[index1].StartMotorSpeed = SysHandleData.EPCBuffAA[index][9] * 500;

		    SysSetParam[index1].ISpeed = SysSetParam[index1].MinMotorSpeed;
		    SysSetParam[index1].IISpeed = SysSetParam[index1].StartMotorSpeed;
		    SysSetParam[index1].IIISpeed = SysSetParam[index1].MaxMotorSpeed;

		    SysSetParam[index1].RunMotorSpeed = SysSetParam[index1].StartMotorSpeed;

        if (SysModelConfig.HandlePortA == 1)
				{
					SysSetParam[index1].PumpVelocitySetA = SysHandleData.EPCBuffAA[index][11];   //流速的设定
					#ifndef WATER_UPTAKE
					if (SysSetParam[index1].PumpVelocitySetA > 70)
						SysSetParam[index1].PumpVelocitySetA = 70;
					#else
					if (SysSetParam[index1].PumpVelocitySetA > 15)
						SysSetParam[index1].PumpVelocitySetA = 15;
					#endif

					SysSetParam[index1].PumpOffOnA = 1;  //每次接上默认打开流量
        }				
				
        if (SysModelConfig.HandlePortB == 1)
				{
					SysSetParam[index1].PumpVelocitySetB = SysHandleData.EPCBuffAA[index][11];   //流速的设定
					#ifndef WATER_UPTAKE
					if (SysSetParam[index1].PumpVelocitySetB > 70)
						SysSetParam[index1].PumpVelocitySetB = 70;
					#else
					if (SysSetParam[index1].PumpVelocitySetB > 15)
						SysSetParam[index1].PumpVelocitySetB = 15;
					#endif

					SysSetParam[index1].PumpOffOnB = 1;  //每次接上默认打开流量
        }

		    SysSetParam[index1].MotorModel = 2;  //正向
//        SysSetParam[index1].ForwardReverseFlag = 0;  //SysRunData.CWCCW = 0;

        //SysRunData.MotorModel2 = (SysSetParam[index1].MotorModel == 1) ? 1 : 0;

		    SysSetParam[index1].ModeForward = 4;  //2正显
//		    SysSetParam[index1].ModeReciprocating = 0;  //UI --- 往复
//		    SysSetParam[index1].ModeReverse = 1;  //UI --- 反向

		    SysSetParam[index1].GearPositionHz = 5;
	    }

	    /*
	    LCD_Show_Picture(0x1304, 224);
	    LCD_Disappear_Number(0x9460);

	    LCD_Disappear_Number(0x9480);

	    
	    
 
	    */
	  }

	  if (SysHandleData.EPCBuffAA[index][3] == 0)
  	  Screen_SeparatingCutterPic_Update(3, SysSetParam[index1].ReciprocatingFlag);   //LCD_Show_Cutter2(3);
	  else
	    Screen_SeparatingCutterPic_Update(2, SysSetParam[index1].ReciprocatingFlag);   //LCD_Show_Cutter2(2);
  }
  /*
  else  //CRC计算通过，但是接收的数据没有值
  {
    SysRunData.DJOldValueFlag[index] = 0;  //刨

	  LCD_SeparatingCutterPic_Update(1, SysSetParam[index1].ReciprocatingFlag);    //LCD_Show_Cutter2

	  SysSetParam[index1].MinMotorSpeed = 500;

	  if (SysRunData.DJOldInit[index] == 0)  //判断是否已读取到并初始化过，
	  {
	    SysSetParam[index1].MaxMotorSpeed = 6000;
	    SysSetParam[index1].StartMotorSpeed = 4000;

	    SysSetParam[index1].ISpeed = SysSetParam[index1].MinMotorSpeed;
	    SysSetParam[index1].IISpeed = SysSetParam[index1].StartMotorSpeed;
	    SysSetParam[index1].IIISpeed = SysSetParam[index1].MaxMotorSpeed;

	    SysSetParam[index1].RunMotorSpeed = SysSetParam[index1].StartMotorSpeed;

      SysSetParam[index1].HzSet = 40;      //频率的设定

	    #ifndef SUCTIONL
	    SysSetParam[index1].PumpVelocitySetB = 30;
	    #else
	    SysSetParam[index1].PumpVelocitySetB = 10;   //流速的设定 0.5L/min
	    #endif

	    SysSetParam[index1].PumpOffOn = 1;
	  }

	  SysRunData.NumberHZSet = SysSetParam[index1].HzSet;

	  SysSetParam[index1].DJSetPDMT = 0;  //刨刀

    SysSetParam[index1].ReciprocatingFlag = 1;  //SysRunData.DXWFFlag = 1;

	  SysSetParam[index1].ForwardReverseFlag = 0;  //SysRunData.CWCCW = 0;

    SysRunData.MotorModel2 = (SysSetParam[index1].MotorModel == 1) ? 1 : 0;
	  if (SysRunData.MotorModel2 == 0)
	  {
//	    SysSetParam[index1].ModeForward = 4;
//	    SysSetParam[index1].ModeReciprocating = 3;
//	    SysSetParam[index1].ModeReverse = 1;

	    SysSetParam[index1].GearPositionHz = 5;
	  }
	  else
	  {
//	    SysSetParam[index1].ModeForward = 3;
//	    SysSetParam[index1].ModeReciprocating = 4;
//	    SysSetParam[index1].ModeReverse = 1;

	    SysSetParam[index1].GearPositionHz = 1;

	    IWDG_Feed();   //喂狗
	    LCD_Show_4byte_Number(0x3460, 0x40800000);
//	    LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.NumberHZSet / 10.0));
	    LCD_Show_4byte_Number(0x3470, 0);
	    IWDG_Feed(); //喂狗
	  }

	  IWDG_Feed(); //喂狗
  }
  */

  //-----------------------------------------------
  {
	  SysRunData.MaxSetMotorSpeed = SysSetParam[index1].MaxMotorSpeed;
	  SysRunData.StartSetMotorSpeed = SysSetParam[index1].StartMotorSpeed;
	  SysRunData.MinSetMotorSpeed = SysSetParam[index1].MinMotorSpeed;

	  SysRunData.SetISpeed = SysSetParam[index1].ISpeed;
	  SysRunData.SetIISpeed = SysSetParam[index1].IISpeed;
	  SysRunData.SetIIISpeed = SysSetParam[index1].IIISpeed;

	  SysRunData.MotorSetSpeed = SysSetParam[index1].RunMotorSpeed;

		
		

  }
	
	if (SysModelConfig.HandlePortA == 1)
	{
		
	  SysRunData.FlowRateA = SysSetParam[index1].PumpVelocitySetA;
    SysRunData.PumpONOFF_A = SysSetParam[index1].PumpOffOnA;		
		
		if (SysRunData.PumpONOFF_A == 1)
		{
			Info_A(1);//Screen_InformationBarImage_Update(1, 1, SysSetParam[index1].GearPositionHz, 1);     //LCD_Show_Info
			
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
//			LCD_Show_4byte_Number(0x3530, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
			LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
			#else
//			LCD_Show_4byte_Number(0x3530, 70);
			LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
			#endif
//			LCD_Show_4byte_Number(0x3430, 0);		
		}
    else
	  {Info_A(4);}//Screen_InformationBarImage_Update(1, 2, SysSetParam[index1].GearPositionHz, 2);     //LCD_Show_Info		
		Info_HZ(SysSetParam[index1].GearPositionHz);
	}		
	
	if (SysModelConfig.HandlePortB == 1)
	{
	  SysRunData.FlowRateB = SysSetParam[index1].PumpVelocitySetB;
    SysRunData.PumpONOFF_B = SysSetParam[index1].PumpOffOnB;		
		
		if (SysRunData.PumpONOFF_B == 1)
		{
			Info_B(1);//Screen_InformationBarImage_Update(1, 1, SysSetParam[index1].GearPositionHz, 1);     //LCD_Show_Info
					
			#ifdef  WATER_UPTAKE//如果定义吸水ssc
//			LCD_Show_4byte_Number(0x3550, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
			LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
			#else
//			LCD_Show_4byte_Number(0x3550, 70);
			LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
			#endif
//			LCD_Show_4byte_Number(0x3450, 0);	
		}		
    else
	  {Info_B(4);}// Screen_InformationBarImage_Update(1, 2, SysSetParam[index1].GearPositionHz, 2);     //LCD_Show_Info		
		Info_HZ(SysSetParam[index1].GearPositionHz);
	}
  	if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2)){
			LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed*2);
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
		}
		else
		{
			LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed);
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
		}

  Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index1].ModeForward); //运行模式切换 单向，往复
}

//============================================================================
// 函数名称: Connect_ManualUIUpdata_Task()
// 功能描述: 手动模式下刀具数据UI刷新
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void SplitType_ManualUIUpdata_Task(void)         //Check_Connect(void)
{
  uint8_t index = 0;
	ssc_shoudong_uiflag=1;

  if(SysRunData.DJManualRefreshFlag != 1)
	  return ;

  index = SysInterface.BeSelectNum - 1;

  SysRunData.DJManualRefreshFlag = 0;

  DJoldVLHZ = 1;

  Common_Memset(0, EPCBuffold, 20);
  Common_Memset(0, EPCBuffold, 20);

  Screen_IntegratedCutterPic_Update(0);
  LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0); //刀具参数

  if (SysSetParam[index].DJSetPDMT == 0) //刨刀 SysRunData.DJManualPDMT
  {
	  //------------------------------------------------------
	  SysSetParam[index].MaxMotorSpeed = 6000;
	  SysSetParam[index].StartMotorSpeed = 4000;
	  SysSetParam[index].MinMotorSpeed = 500;

	  SysSetParam[index].ISpeed = SysSetParam[index].MinMotorSpeed;
	  SysSetParam[index].IISpeed = SysSetParam[index].StartMotorSpeed;
	  SysSetParam[index].IIISpeed = SysSetParam[index].MaxMotorSpeed;

    SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

	  //------------------------------------------------------
	  SysSetParam[index].ReciprocatingFlag = 1;
	  SysSetParam[index].MotorModel = 1;
//    SysSetParam[index].ForwardReverseFlag = 0;

	  SysSetParam[index].ModeForward = 2;  //3往复
//	  SysSetParam[index].ModeReciprocating = 4;  //UI --- 往复
//	  SysSetParam[index].ModeReverse = 1;  //UI --- 反向

	  SysSetParam[index].HzSet = 40;      //频率的设定


    if (SysModelConfig.HandlePortA == 1)
		{
			#ifdef WATER_UPTAKE
				SysSetParam[index].PumpVelocitySetA = 10;
				#else
				SysSetParam[index].PumpVelocitySetA = 30;
				#endif

			SysSetParam[index].PumpOffOnA = 1;		
		}
		
    if (SysModelConfig.HandlePortB == 1)
		{
			#ifdef WATER_UPTAKE
				SysSetParam[index].PumpVelocitySetB = 10;
				#else
				SysSetParam[index].PumpVelocitySetB = 30;
				#endif

			SysSetParam[index].PumpOffOnB = 1;		
		}

	  SysSetParam[index].GearPositionHz = 1;

    SysRunData.Frequency = SysSetParam[index].HzSet;      //频率的设定

	  SysRunData.DJOldValueFlag[SysInterface.InterfaceSwitchNo2 - 1] = 0;  //0：刨 1：磨

    /*
	  LCD_Show_Picture(0x1304, 221);
	  LCD_Show_Number (0x9460, 0x3460);
 
	  LCD_Show_Number (0x3470, 0x3470);

	 
	 
	   

    IWDG_Feed(); //喂狗
	  LCD_Show_4byte_Number(0x3460, 0x40800000);
//	  LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.NumberHZSet / 10.0));
	  LCD_Show_4byte_Number(0x3470, 0);
    IWDG_Feed(); //喂狗
    */

	  Screen_SeparatingCutterPic_Update(4, SysSetParam[index].ReciprocatingFlag);  //LCD_Show_Cutter2(4);
  }
  else
  {
	  //------------------------------------------------------
	  SysSetParam[index].MaxMotorSpeed = 13000;
	  SysSetParam[index].StartMotorSpeed = 10000;
	  SysSetParam[index].MinMotorSpeed = 3000;

	  SysSetParam[index].ISpeed = SysSetParam[index].MinMotorSpeed;
	  SysSetParam[index].IISpeed = SysSetParam[index].StartMotorSpeed;
	  SysSetParam[index].IIISpeed = SysSetParam[index].MaxMotorSpeed;

	  SysSetParam[index].RunMotorSpeed = SysSetParam[index].StartMotorSpeed;

	  //------------------------------------------------------
	  SysSetParam[index].ReciprocatingFlag = 0;
	  SysSetParam[index].MotorModel = 2;
//    SysSetParam[index].ForwardReverseFlag = 0;

	  SysSetParam[index].ModeForward = 4;  //2正显
//	  SysSetParam[index].ModeReciprocating = 0;  //UI --- 往复  WF
//	  SysSetParam[index].ModeReverse = 1;  //UI --- 反向

	  SysSetParam[index].HzSet = 40;      //频率的设定

    if (SysModelConfig.HandlePortA == 1)
		{
			#ifdef WATER_UPTAKE
				SysSetParam[index].PumpVelocitySetA = 10;
				#else
				SysSetParam[index].PumpVelocitySetA = 30;
				#endif

			SysSetParam[index].PumpOffOnA = 1;		
		}
		
    if (SysModelConfig.HandlePortB == 1)
		{
			#ifdef WATER_UPTAKE
				SysSetParam[index].PumpVelocitySetB = 10;
				#else
				SysSetParam[index].PumpVelocitySetB = 30;
				#endif

			SysSetParam[index].PumpOffOnB = 1;		
		}
 
	  SysSetParam[index].GearPositionHz = 5;

    SysRunData.Frequency = SysSetParam[index].HzSet;      //频率的设定

	  SysRunData.DJOldValueFlag[SysInterface.InterfaceSwitchNo2 - 1] = 1;  //0：往复 1：单向

    /*
	  LCD_Show_Picture(0x1304, 224);
	  LCD_Disappear_Number(0x9460);

	  LCD_Disappear_Number(0x9480);

	  
	 
	 
	  */

	  Screen_SeparatingCutterPic_Update(5, SysSetParam[index].ReciprocatingFlag);  //LCD_Show_Cutter2(5);
  }

  //------------------------------------------------------
  {
	  SysRunData.MaxSetMotorSpeed = SysSetParam[index].MaxMotorSpeed;
	  SysRunData.StartSetMotorSpeed = SysSetParam[index].StartMotorSpeed;
	  SysRunData.MinSetMotorSpeed = SysSetParam[index].MinMotorSpeed;

	  SysRunData.SetISpeed = SysSetParam[index].ISpeed;
	  SysRunData.SetIISpeed = SysSetParam[index].IISpeed;
	  SysRunData.SetIIISpeed = SysSetParam[index].IIISpeed;

	  SysRunData.MotorSetSpeed = SysSetParam[index].RunMotorSpeed;

    //------------------------------------------------------
	  //SysRunData.MotorModel2 = (SysSetParam[index].MotorModel == 1) ? 1 : 0;
//	  SysRunData.CWCCW = SysSetParam[index].ForwardReverseFlag;


  }
	if (SysModelConfig.HandlePortA == 1)
	{
    SysRunData.FlowRateA = SysSetParam[index].PumpVelocitySetA;  //流量
	  SysRunData.PumpONOFF_A = SysSetParam[index].PumpOffOnA;	
	  Info_A(1);//Screen_InformationBarImage_Update(1, 1, SysSetParam[index].GearPositionHz, 1);
	}
	if (SysModelConfig.HandlePortB == 1)
	{
    SysRunData.FlowRateB = SysSetParam[index].PumpVelocitySetB;  //流量
	  SysRunData.PumpONOFF_B = SysSetParam[index].PumpOffOnB;	
	  Info_B(1);//Screen_InformationBarImage_Update(1, 1, SysSetParam[index].GearPositionHz, 1);
	}
  Info_HZ(SysSetParam[index].GearPositionHz);
  //------------------------------------------------------
  if (SysSetParam[index].GearPositionHz == 1)
  {
	  LCD_Show_4byte_Number(0x3460, 0x40800000);
	  LCD_Show_4byte_Number(0x3470, Common_FolatToHex(SysRunData.Frequency / 10.0));
//	  LCD_Show_4byte_Number(0x3470, 0);
  }
	
	if (SysModelConfig.HandlePortA == 1)
	{
		#ifdef  WATER_UPTAKE//如果定义吸水ssc
//	  LCD_Show_4byte_Number(0x3530, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
		LCD_Show_4byte_Number(0x3530,Common_FolatToHex(SysRunData.FlowRateA/10));  //流速  SysRunData.NumberFluidSet);
		#else
//		LCD_Show_4byte_Number(0x3530, 70);
		LCD_Show_4byte_Number(0x3530, SysRunData.FlowRateA);
		#endif
//		LCD_Show_4byte_Number(0x3430, 0);
	}	
	if (SysModelConfig.HandlePortB == 1)
	{
		#ifdef  WATER_UPTAKE//如果定义吸水ssc
//	  LCD_Show_4byte_Number(0x3550, 0x3FC00000);   // --> 1.0L/min 单精度浮点转换为16hex
		LCD_Show_4byte_Number(0x3550,Common_FolatToHex(SysRunData.FlowRateB/10));  //流速  SysRunData.NumberFluidSet);
		#else
//		LCD_Show_4byte_Number(0x3550, 70);
		LCD_Show_4byte_Number(0x3550, SysRunData.FlowRateB);
		#endif
//		LCD_Show_4byte_Number(0x3450, 0);
	}
 if((SysModelConfig.HandlePortA == 1&&SysInterface.HandleType[1] == Handle_Type_2)||(SysModelConfig.HandlePortB == 1&&SysInterface.HandleType[4] == Handle_Type_2)){
		LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed*2);
		LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed*2);
 }
 else
	{
			LCD_Show_4byte_Number(0x3400, SysRunData.MaxSetMotorSpeed);
			LCD_Show_4byte_Number(0x3410, SysRunData.MotorSetSpeed);
	}

		Screen_ElectricalMachineryDirectionState_Update2(SysSetParam[index].ModeForward); //运行模式切换 单向，往复  LCD_Show_Direction
}

//============================================================================
// 函数名称: SplitType_CutterScan_Task()
// 功能描述: 检测刀具(分离试手柄)
// 输　  入:
// 输    出:
// 函数说明: 2号手柄刀具 100ms
//============================================================================
void SplitType_CutterScan_Task(void)         //Check_Connect(void)
{
  if ((SysRunData.MotorRun == MotorStop) && (SysRunData.MotorNum == MotorNum2))  //&& Stepping_Speed == 0  Motor_Number
  {
    if (SysSetParam[SysInterface.BeSelectNum-1].DJAutoGetFlag == 0)  //自动模式  SysRunData.DJAutoManualFlag
	  {
			ssc_shoudong_rfidflag=0;
	    //1.2s 检测刀具断开
	    SplitType_DisconnectUpdata_Task();

	    //100ms 检测刀具连接
	    SplitType_ConnectUpdata_Task();
	  }
	  else  //手动模式
	  {
			ssc_shoudong_rfidflag=1;
	    SplitType_ManualUIUpdata_Task();
	  }
  }
  else if (SysRunData.MotorNum == MotorNone)  //Motor_Number
  {
	  SysRunData.CutterON = 0;		//无手柄连接复位允许初始化参数标志

	  Common_Memset(0, EPCBuffold, 20);
	  Common_Memset(0, EPCBuffold, 20);
  }
  else
  {
	  //相同参数刀具，拔了手柄再次插上时，不能正确读取刀具信息
	  //手柄拔掉...
	  if (SysRunData.ParamInitFlag[4] != 1)
	    Common_Memset(0, EPCBuffold, 20);

	  //手柄拔掉...
	  if (SysRunData.ParamInitFlag[1] != 1)
	    Common_Memset(0, EPCBuffold, 20);
  }
}

/* USER CODE BEGIN Header_CUTTERSCANTaskFunc */
/**
* @brief Function implementing the CUTTERSCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CUTTERSCANTaskFunc */
void CUTTERSCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN CUTTERSCANTaskFunc */
  /* Infinite loop */
  SplitType_CutterScan_Task();
  /* USER CODE END CUTTERSCANTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 100
 */
//============================================================================
void SplitType_CutterScan_Init(void)
{
  /* definition and creation of CUTTERSCANTask */
	Kernel_TaskCreate(&CUTTERSCANTaskHandle, CUTTERSCANTaskFunc);
	Kernel_TaskStart(&CUTTERSCANTaskHandle, KERNEL_TASK_ALWAYS, 100);
}

