//userparser.c

#include "userparser.h"
#include "iwdg.h"
#include "sysrunled.h"
#include "delay.h"
#include "eeprom.h"
#include "bsp_board.h"
#include "board_profile.h"
#include "hw_bootstrap.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "uart4.h"
#include "uart5.h"
#include "uart6.h"
#include "uart7.h"
#include "uart9.h"

//#include "adc.h"
#include "lcd.h"
#include "screen_address.h"
#include "motor.h"
#include "UI_Start.h"
#include "common.h"

#include "handlescan.h"
#include "drivectrl.h"
#include "pedal.h"
#include "motoruartdata.h"
#include "external_comm_task.h"
#include "screenkey.h"
#include "pump.h"
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

void Userparser_Init(void)
{
  Iwdg_Reset();

  //1.GPIO
  Hardware_BoardGpioInit();

  //2.IIC、1-@Wire
  EEPROM_AT24CXX_Init();

  //3.UART
  Uart1_Init();  //无刷
  Uart2_Init();  //外部通讯
  Uart3_Init();  //射频
#if (RFID_USE_DUAL_UART_MODE == 1U)
  Uart9_Init();  //B 通道射频独立串口，旧模式关闭时不占用 PD14/PD15。
#endif
  Uart4_Init();  //脚踏
  Uart5_Init();  //步进1
	Uart6_Init();  //屏
  LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_STARTUP);  //串口6初始化后再强制切启动页，保证 EX8 实际收到 page0 切换帧
  Uart7_Init();  //步进2

	//4.分体按键
	//............	
  Iwdg_Reset();

  LCD_Disappear_Picture(UIDP_LCD_VP_ALARM_TIP);     //清除新屏报警显示区，旧屏提示接口不再参与开机流程

  Motor_ErrorEmergencyStop_Ctrl();  //21ms 电机停止发送...

  //”出厂配置模式“等待
  UI_Start_Fun();//脚踏定标界面，关系界面
  Iwdg_Reset();
  Delay_ms(500);
	Iwdg_Reset();
	Delay_ms(500);
  	Delay_ms(500);
    	Delay_ms(500);
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

	SendUIDSMessage(UI_POWERINIT_ID, false, NULL); //屏幕开机初始化由 UIDP 任务统一刷新，避免绕过统一 UI 入口

}
