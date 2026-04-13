//userparser.c

#include "userparser.h"
#include "iwdg.h"
#include "sysrunled.h"
#include "flash.h"
#include "data.h"
#include "delay.h"
#include "eeprom.h"
#include "bsp_board.h"
#include "hw_bootstrap.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "uart4.h"
#include "uart5.h"
#include "uart6.h"
#include "uart7.h"

//#include "adc.h"
#include "lcd.h"
#include "screen.h"
#include "motor.h"
#include "UI_Start.h"
#include "radiofreq.h"
#include "common.h"
#include "beep.h"

#include "Connectscan.h"
#include "handlescan.h"
#include "drivectrl.h"
#include "footpedal.h"
#include "warn.h"
#include "pedal.h"
#include "motoruartdata.h"
#include "dircurrent.h"
#include "screenkey.h"
#include "pump.h"
#include "splittype.h"
#include "UI_Main.h"
#include "handlekey.h"

#include "iic.h"

//============================================================================
//手柄模式 读取Flash
void Storage_HandleMode_init(void)
{
  Flash_Read(ADDR_BASE, (uint32_t*)&SysModelConfig.type[0], 1);
  if (SysModelConfig.type[0] == 0xff)
  {
    SysModelConfig.type[0] = 0x03;
    SysModelConfig.type[1] = 20;

    Flash_Write(ADDR_BASE, (uint32_t*)&SysModelConfig.type[0], 1);
//    Handle_Model = SysModelConfig.type[0];
//		SysRunData.PumpFlowEEPOM = SysModelConfig.type[1];
  }
  else
  {
//		Handle_Model = SysModelConfig.type[0] & 0x0F;
//    SysRunData.PumpFlowEEPOM = SysModelConfig.type[1];
  }
}

void Storage_PumpFlow_init(void)
{
  uint8_t EEPOMDat[2] = { 0 };
 
  //泵流量读取 ---1
  EEPROM_AT24CXX_Read(0x20, EEPOMDat, 2);
  if (EEPOMDat[0] == 0x55)
  {
		SysRunData.PumpFlowEEPOM = EEPOMDat[1];
    if (SysRunData.PumpFlowEEPOM > PUMPMLUNITMAX)
    {
      SysRunData.PumpFlowEEPOM = 20;
	    EEPROM_AT24CXX_Write(0x21, &SysRunData.PumpFlowEEPOM, 1); 
			EEPROM_AT24CXX_Write(0x23, &SysRunData.PumpFlowEEPOM, 1);
	  }
		
		EEPROM_AT24CXX_Read(0x25, &SysRunData.PumpFlowEEPOM, 1);
    if (SysRunData.PumpFlowEEPOM > PUMPMLUNITMAX)
    {
      SysRunData.PumpFlowEEPOM = 20;
 			EEPROM_AT24CXX_Write(0x25, &SysRunData.PumpFlowEEPOM, 1);
			EEPROM_AT24CXX_Write(0x27, &SysRunData.PumpFlowEEPOM, 1);
	  }		
  }
  else
  {
	  EEPOMDat[0] = 0x55;
	  SysRunData.PumpFlowEEPOM = EEPOMDat[1] = 20;

	  EEPROM_AT24CXX_Write(0x20, EEPOMDat, 2);
	  EEPROM_AT24CXX_Write(0x23, &SysRunData.PumpFlowEEPOM, 1);
  	EEPROM_AT24CXX_Write(0x25, &SysRunData.PumpFlowEEPOM, 1);
  	EEPROM_AT24CXX_Write(0x27, &SysRunData.PumpFlowEEPOM, 1);		

  }
}


//============================================================================
void Userparser_Init(void)
{
  LCD_Show_Which_Map(0);  //开机页

  Iwdg_Reset();

  //1.GPIO
  Hardware_BoardGpioInit();

  //2.IIC、1-@Wire
  EEPROM_AT24CXX_Init();

  //3.UART
  Uart1_Init();  //无刷
  Uart2_Init();  //外部通讯
  Uart3_Init();  //射频
  Uart4_Init();  //脚踏
  Uart5_Init();  //步进1
	Uart6_Init();  //屏
  Uart7_Init();  //步进2

	//4.分体按键
	//............	
  Iwdg_Reset();

  Screen_TipInfo_Update(0);     //清除报警显示

  SysFootPedalData.FootPedalLiftFlag = No_Lift;   //脚踏抬起

  Motor_ErrorEmergencyStop_Ctrl();  //21ms 电机停止发送...

  //”出厂配置模式“等待
  UI_Start_Fun();//脚踏定标界面，关系界面
  Storage_HandleMode_init();  //读取存储的手柄模式（本设备允许的手柄） 
  Storage_PumpFlow_init();    //流量（耳磨）...
  Iwdg_Reset();
  Delay_ms(500);
	Iwdg_Reset();
	Delay_ms(500);
	Common_Memset(0, SysHandleData.Ds2431BuffBB[0], 14);
  Common_Memset(0, SysHandleData.Ds2431BuffBB[1], 14);
	RadioFreq_Init();  //150ms射频初始化...串口3
  Iwdg_Reset();
	
	Workvalue_s.FootThrottletask_flag=0;
	//ssc任务初始化开始
	IwdgTaskInit();
	LEDTaskInit(); 
	BeepControlTask_Init();
	HandlescanTaskInit();//手柄扫描

	PedalRecvTask_Init();  //3ms 脚踏数据接收
	FootPedalTask_Init(); //脚踏扫描链接
	ScreenKeyTask_Init();//显示屏按键逻辑初始化
	FootKeyTask_Init();//脚踏按键任务
	ScreenKey_ScanInit();  //22ms  屏幕按键
	FootThrottleTask_Init();
	DriveCtrl_Motor123Task_Init();
	HandleKeyScan_Init();//手柄按键扫描
	
	MotorUartData_Init(); 
	SplitType_AutoModeGetData_Init();  //200ms 请求分体式手柄的刀具信息（自动设别刀具模式）
	DriveCtrl_HMITask_Init();
	PUMPBTask_Init();

//ssc任务初始化结束

//  UI_Show_init();  //U初始化...

//  //2.任务初始化
//  IwdgTaskInit();  //300ms  看门狗 高于正常优先级

//  LEDTaskInit();   //200ms  运行灯

//  HandlescanTaskInit();  //10ms  手柄连接扫描 100ms防抖【耳磨、分体、一体】

//  ConnectscanTaskInit();  //20ms  驱动板\脚踏 断开扫描

//  DriveCtrl_Motor1CurrentTask_Init();  //53ms 驱动板电流状态获取
//  DriveCtrl_UIRefreshDataTask_Init();  //100ms 获取实时转速 200ms更新UI
//  DriveCtrl_Motor123Task_Init();  //55ms 电机控制

//  FootPedalTask_Init();  //25ms 脚踏连接扫描 100ms防抖
//  PedalRecvTask_Init();  //3ms 脚踏数据接收

//  Warn_RunErrScanTask_Init();  //100ms 运行错误状态
//  Warn_StatusScanTask_Init();  //15ms 错误报警

//  MotorUartData_Init();  //3ms 驱动板数据接收 6ms判断是否正接收数据中

////  DirCurrentTask_Judge_Init();  //13ms 驱动电流判断

//  ScreenKey_LongPressTaskInit();  //150ms 屏幕“长按”
//  ScreenKey_ScanInit();  //22ms  屏幕按键

//////  Pump_RunTask_Init();   //50ms  泵缓启动判断
//  Pump_Pedal2Pump5sTask_Init();  //10ms 快速踩两脚判断 运行5s、排空

//  SplitType_AutoModeGetData_Init();  //200ms 请求分体式手柄的刀具信息（自动设别刀具模式）
//  SplitType_AutoModeDataRead_Init();  //50ms 扫描分体式手柄刀具信息的接收缓存
//  SplitType_CutterScan_Init();  //100ms ①刷新刀具连接信息 ②1.2s判断刀具的断开信息 ③更新手动模式下刀具信息

//  UIMain_RefreshTaskInit();  //50ms 主页显示刷新

//  HandleKeyScan_Init();  //15ms 手柄按键扫描
	LCD_Show_Which_Map(4);
	PoweronInit();
//	Workvalue_s.set_speed=60000;

}






