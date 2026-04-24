//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "data.h"
#include "uart1.h"
#include "uart2.h"
#include "pump.h"
#include <string.h>
#include "lcd.h"
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "sscKEYBH.h"
#include "sscBEEP.h"

kernel_task_t MOTORUARTTaskHandle;

/*
 * 外部 HMI 串口仍沿用原 6 字节命令帧，只把命令结果投递到新的公共接口。
 * 这样可以保留串口解析时序，同时避免继续写旧屏幕按键数据仓库。
 */
static void MotorUart_PostHmiKey(uint8_t hmi_key)
{
	SendKeyBehMessage(HMIkey, hmi_key);
}

static void MotorUart_SetHmiActive(uint8_t active)
{
	WorkMessage.hmiactive_work = (active != 0U);
	ControlSignalMessage.HMI_enable_flag = (active != 0U);
	if(active == 0U)
	{
		WorkMessage.runflag_work = false;
		ControlSignalMessage.HMI_control_flag = false;
	}
}

static void MotorUart_SetHmiRun(uint8_t run)
{
	if(WorkMessage.alarm_flag)
	{
		return;
	}
	if(!WorkMessage.hmiactive_work)
	{
		return;
	}
	WorkMessage.runflag_work = (run != 0U);
	ControlSignalMessage.HMI_control_flag = (run != 0U);
}

static void MotorUart_SetAlarm(uint8_t alarm_value)
{
	WorkMessage.alarm_value = alarm_value;
	WorkMessage.alarm_flag = (alarm_value != 0U);
	SendAlarmMessage(alarm_value);
}

static void MotorUart_ClearAlarm(void)
{
	WorkMessage.alarm_value = 0U;
	WorkMessage.alarm_flag = false;
	SendAlarmMessage(0U);
}

static void MotorUart_StopAllWork(void)
{
	WorkMessage.runflag_work = false;
	ControlSignalMessage.handle_control_flag = false;
	ControlSignalMessage.HMI_control_flag = false;
	ControlSignalMessage.jtL_control_flag = false;
	ControlSignalMessage.jtR_control_flag = false;
	pumpMessageA.run_flag = false;
	pumpMessageA.speed_work = 0U;
	pumpMessageB.run_flag = false;
	pumpMessageB.speed_work = 0U;
}

//============================================================================
//2.驱动板接收
//============================================================================

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 外部控制
//============================================================================
void BrushedMotorUartData_ReceiveData(void)
{
  uint8_t rlen = 0, i = 0;
  uint8_t dat[UART2_MAX_PACKET_SIZE] = { 0 }, dat1[22] = { 0 };

	
  //读取串口数据
  rlen = Uart2_DMARecvDataPeek(dat);
  if (rlen < 6)   //不够一个数据包大小
	  return;

  for (i = 0; i < (rlen-5); i++)    //
  {
	  if (dat[i] == 0xAA) 
	  {
			if(dat[i+4]==0xee&&dat[5]==0xff)
			{
				SendKeyBeepMessage(1U);
				Common_CopyData(&dat[i], dat1, 6);    //截取6个数据
				switch(dat1[3])
				{
					case 1:
						MotorUart_SetHmiActive(1U);
					LCD_Show_Picture(0x1450,540);
					
						break;
					case 2:
						LCD_Disappear_Picture(0x1450);
						if(pumpMessageB.run_flag)
						{
							MotorUart_PostHmiKey(HMIkey_BPUMP_control);
						}
						
						MotorUart_SetHmiActive(0U);
						MotorUart_PostHmiKey(HMIkey_HMI_EXIT);
						MotorUart_ClearAlarm();
						break;
					case 3:
							if(WorkMessage.alarm_flag)
								return;
						MotorUart_SetHmiRun(1U);
						break;
					case 4:
						MotorUart_SetHmiRun(0U);
						break;
					case 5:
							if(WorkMessage.hmiactive_work)
						MotorUart_PostHmiKey(HMIkey_SPEED_Add);
						break;
					case 6:
							if(WorkMessage.hmiactive_work)
						MotorUart_PostHmiKey(HMIkey_SPEED_Sub);
						break;
					case 7:
					if(WorkMessage.hmiactive_work){
						if(WorkMessage.channel_work==CHANNEL_A)
						{
							MotorUart_PostHmiKey(HMIkey_HANDLE_B);
						}
						else
						{
							MotorUart_PostHmiKey(HMIkey_HANDLE_A);
						}
					}
					break;
					  case 8:
								if(WorkMessage.hmiactive_work)
							MotorUart_PostHmiKey(HMIkey_BPUMP_control);
							break;
						case 9:
							if(WorkMessage.hmiactive_work)
							MotorUart_PostHmiKey(HMIkey_APUMP_control);
							break;
							case 10:
								if(WorkMessage.hmiactive_work){
									MotorUart_PostHmiKey(HMIkey_APUMP_Add);
								}
							break;
						case 11:
								if(WorkMessage.hmiactive_work){
									MotorUart_PostHmiKey(HMIkey_BPUMP_Add);
							}
							break;
					
					case 12:
							if(WorkMessage.hmiactive_work){
						MotorUart_ClearAlarm();
						break;
					}
				}
			}
			memset(dat1, 0, sizeof(dat1));//
			return;
		}
	}
}

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 无刷
//============================================================================
void BrushlessMotorUartData_ReceiveData(void)
{
  uint8_t rlen = 0, i = 0;
  uint8_t dat[UART1_MAX_PACKET_SIZE] = { 0 }, dat1[22] = { 0 };
  uint16_t CRC_Check_Vaule=0;
	
  //读取串口数据
  rlen = Uart1_DMARecvDataPeek(dat);
  if (rlen < 11)   //不够一个数据包大小
	  return;

  //查询本帧数据包的帧头0x01
  for (i = 0; i < (rlen - 11); i++)    //最小帧数据包为4[0x01 0x03 0 0 0 0 0]
  {
	  if (dat[i] == 0xAA)  
	  {
		  CRC_Check_Vaule = Common_Crc16(&dat[i],10);//ssc
			if(CRC_Check_Vaule==dat[i+10]+(dat[i+11]<<8))
			{
				Common_CopyData(&dat[i], dat1, 12);    //截取10个数据
			//	LCD_Show_4byte_Number(0x3550,(dat1[9]));//鹏红测试
					if (dat1[7] == 0)
					{
						SysRunData.StuckFlag3 = No_Error;
					}
					else
					{
					SysRunData.StuckFlag3 = Error;	
						if(WorkMessage.channel_work==CHANNEL_A){
							MotorUart_SetAlarm(8U);
						}
						else if(WorkMessage.channel_work==CHANNEL_B)
						{
							MotorUart_SetAlarm(9U);
						}
						Pump_SetSpeed_A(0);//泵停止运行
						//Pump_SetSpeed_B(0);//泵停止运行
						MotorUart_StopAllWork();
					}	
				switch (dat1[1])
				{
					case 0x01 :  //正向
					{

					}
					break;
					case 0x02 :  //反向HALLEErrFlag
					{

					}
					break;
					case 0x03 :  //往复
					{

					}
					break;
					case 0x04 :
					case 0x05 :
					{
					}
					break;
					default : break;
				}

				SysRunData.DriveBoardOffTimes = 0;  //Communicat_Status_Time = 0;
				SysRunData.DriveBoardConnectFlag = Connect;  //Communicat_Status = No_Error;

				memset(dat1, 0, 15);
				i += 12;
			}
	  }
  }
}

//============================================================================
//与主板进行串口通讯的任务初始化 2
//============================================================================
/* USER CODE BEGIN Header_MOTORUARTTaskFunc */
/**
* @brief Function implementing the MOTORUARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTORUARTTaskFunc */
void MOTORUARTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTORUARTTaskFunc */
  /* Infinite loop */
  BrushedMotorUartData_ReceiveData();
	BrushlessMotorUartData_ReceiveData();
  /* USER CODE END MOTORUARTTaskFunc */
}

void MotorUartData_Init(void)
{
  /* definition and creation of MOTORUARTTask */
	Kernel_TaskCreate(&MOTORUARTTaskHandle, MOTORUARTTaskFunc);
	Kernel_TaskStart(&MOTORUARTTaskHandle, KERNEL_TASK_ALWAYS, 3);//3
}






