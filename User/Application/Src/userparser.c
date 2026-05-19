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
#include "common.h"

#include "handlescan.h"
#include "drivectrl.h"
#include "warn.h"
#include "pedal.h"
#include "motoruartdata.h"
#include "external_comm_task.h"
#include "screenkey.h"
#include "pump.h"
#include "UI_Main.h"
#include "soft_uart.h"
#include "handlekey.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscDRIVE.h"
#include "sscFOOT.h"
#include "sscKEYBH.h"
#include "sscPUMPA.h"
#include "sscPUMPB.h"
#include "sscRFID.h"
#include "sscUIDP.h"

#include "iic.h"

/*
 * V1.8 新接口数据容器初始化。
 * 当前阶段先把 WorkMessage、通道识别、通道记忆、泵状态和控制信号统一清零，
 * 保证后续逐步切换 handlescan、脚踏、按键、泵和 UI 时不会读到随机状态。
 * 任务启动顺序仍保持当前工程原有架构，避免在旧模块尚未完全下线前改变硬件时序。
 */
static void Userparser_PubinterfaceInit(void)
{
  ChannelrecognizeMessageInit();
  WorkMessageInit();
  ChannelMemoryMessageInit();
  pumpMessageInit();
  ChannelFlagMessageInit();
}

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
  Userparser_PubinterfaceInit();
	SscRadioFreq_Init();  //150ms射频初始化...串口3
  Iwdg_Reset();
	
	//ssc任务初始化开始
	IwdgTaskInit();
	LEDTaskInit(); 
	SscBeepControlTask_Init();
	SscKeyBehaviorTask_Init();
	HandlescanTaskInit();//手柄扫描

	SscFootControlTask_Init(); //脚踏解析和行为事件统一进入新接口
	ScreenKey_ScanInit();  //22ms  屏幕按键
	SscDriveMotorTask_Init();
	HandleKeyScan_Init();//手柄按键扫描
	
	MotorUartData_Init(); 
	ExternalComm_Init();  //UART2 外部通信协议任务，独立接收下行帧并周期上传心跳。
	SscSplitTypeAutoModeGetData_Init();  //200ms 请求分体式手柄的刀具信息（自动设别刀具模式）
	SscPumpATask_Init();
	SscPumpBTask_Init();
	SscUIDisplayTask_Init();
	SimUartTask_Init();

	LCD_Show_Which_Map(4);
	PoweronInit();

}
