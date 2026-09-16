//userparser.c

#include "userparser.h"
#include "iwdg.h"
#include "sysrunled.h"
#include "delay.h"
#include "eeprom.h"
#include "mainboard_software_version.h"
#include "board.h"
#include "board_profile.h"
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
#include "pump_behavior_core.h" /* 开机直接下发两台泵的停止帧，不依赖尚未启动的泵任务。 */
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
 * 函数功能：在启动业务任务前，设置手柄、当前工作参数、泵和控制标志的初始值。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Userparser_PubinterfaceInit(void)
{
  ChannelrecognizeMessageInit();
  WorkMessageInit();
  ChannelMemoryMessageInit();
  pumpMessageInit();
  ChannelFlagMessageInit();
}

/*
 * 函数功能：完成主控业务启动初始化，依次初始化 GPIO、IIC、UART、公共状态和各周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void Userparser_Init(void)
{

  //1.GPIO
  Board_GPIOConfiguration(); /* 先配置业务引脚和中断，后续总线读写才能使用正确的引脚状态。 */
  Iwdg_BootStage(IWDG_STAGE_EEPROM); /* 外设及GPIO已经准备好，开始限制EEPROM初始化总用时。 */

  //2.IIC、1-@Wire
  EEPROM_AT24CXX_Init();
  (void)MainboardSoftwareVersion_Sync();  //主控板 AT24C32 Page1 只保存软件版本；同步失败不阻塞原有主控启动流程
  Iwdg_BootStage(IWDG_STAGE_OUTPUT_STOP); /* EEPROM步骤已返回，开始准备串口及开机停止输出。 */

  //3.UART
  Uart1_Init();  //手柄电机驱动通信，包含无刷和有刷类型。
  Uart2_Init();  //外部通讯
  Uart3_Init();  //射频
#if (RFID_USE_DUAL_UART_MODE == 1U)
  Uart9_Init();  //启动逻辑B侧RFID DMA；固定UART线束不跟随手柄物理交换。
#endif
  Uart4_Init();  //脚踏
  Uart5_Init();  //步进1
	Uart6_Init();  //屏
  LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_STARTUP);  //串口6初始化后再强制切启动页，保证 EX8 实际收到 page0 切换帧
  Uart7_Init();  //步进2

	//4.分体按键
	//............	

  LCD_Disappear_Picture(UIDP_LCD_VP_ALARM_TIP);     //清除新屏报警显示区，旧屏提示接口不再参与开机流程

  PumpBehaviorCore_StopOnStartup(Iwdg_WasAbnormalReset()); /* 两泵先发零速；异常重启还取消驱动定位，普通上电保留原定位过程。 */
  Motor_ErrorEmergencyStop_Ctrl(); /* 向两个物理电机通道按支持类型发送零速帧，不依赖上次手柄记忆。 */

  //设置启动页状态，允许后续通过界面进入脚踏定标。
  Iwdg_BootStage(IWDG_STAGE_STARTUP_PAGE); /* 开机停止命令已下发，进入有总时限的启动页等待。 */
  UI_Start_Fun();//初始化启动页和脚踏定标入口。
  Iwdg_BootStage(IWDG_STAGE_SETTLE); /* 启动页正常结束后再进入固定2秒等待，不能靠通用延时喂狗。 */
  Delay_ms(500);
	Delay_ms(500);
  Delay_ms(500);
  Delay_ms(500);
  Userparser_PubinterfaceInit();
	Iwdg_BootStage(IWDG_STAGE_RFID); /* 运行请求和通道记忆已重新初始化，再准备射频模块。 */
	SscRadioFreq_Init();  //按单/双串口硬件模式初始化两侧RFID模块，旧模式由R200-K8依次选通。
  Iwdg_BootStage(IWDG_STAGE_TASKS); /* 射频初始化返回，接下来检查业务任务创建进度。 */
	
	//ssc任务初始化开始
	LEDTaskInit(); 
	SscBeepControlTask_Init();
	SscKeyBehaviorTask_Init();
	HandlescanTaskInit();//手柄扫描

	SscFootControlTask_Init(); //创建脚踏控制任务，由其处理踩下、松开和泵联动。
	ScreenKey_ScanInit();  //创建屏幕按键接收处理任务，周期在该函数内部设置。
	SscDriveMotorTask_Init();
	HandleKeyScan_Init();//手柄按键扫描
	
	MotorUartData_Init(); 
	ExternalComm_Init();  //UART2 外部通信协议任务，独立接收下行帧并周期上传心跳。
	SscSplitTypeAutoModeGetData_Init();  //创建分体式手柄刀具信息读取任务，用于自动识别刀具。
	SscPumpATask_Init();
	SscPumpBTask_Init();
	SscUIDisplayTask_Init();
	SimUartTask_Init();

	SendUIDSMessage(UI_POWERINIT_ID, false, NULL); //通知显示任务刷新开机页面，业务初始化函数不直接重复发送页面参数。

}
