//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "data.h"
#include "screen.h"
#include "uart1.h"
#include "uart2.h"
#include "pump.h"
#include <string.h>
#include "lcd.h"
#include "kernel_scheduler.h"

kernel_task_t MOTORUARTTaskHandle;

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
				KeyBeep_flag=1;
				Common_CopyData(&dat[i], dat1, 6);    //截取6个数据
				switch(dat1[3])
				{
					case 1:
						Workvalue_s.HMI_Control_flag=1;//使能HMI控制
					LCD_Show_Picture(0x1450,540);
					
						break;
					case 2:
						LCD_Disappear_Picture(0x1450);
						Workvalue_s.HMI_Control_flag=0;
						Workvalue_s.HMI_Working_flag=0;
						if(Workvalue_s.Irrigate_start_flag)
						{
							Workvalue_s.ScreenKey_data=11;//失去之前停掉，灌注
							Workvalue_s.HMI_Injection_stop_flag=1;
						}
						
						if(Workvalue_s.beep_Alarm_flag)
						{
							Workvalue_s.beep_Alarm_flag=0;
							Workvalue_s.Alarm_value=0;
								Workvalue_s.HMI_Working_flag=0;
						}
						break;
					case 3:
							if(Workvalue_s.beep_Alarm_flag)
								return;
						if(Workvalue_s.HMI_Control_flag)
						Workvalue_s.HMI_Working_flag=1;
						break;
					case 4:
							if(Workvalue_s.HMI_Control_flag)
						Workvalue_s.HMI_Working_flag=0;
						break;
					case 5:
							if(Workvalue_s.HMI_Control_flag)
						Workvalue_s.ScreenKey_data=1;//速度1
						break;
					case 6:
							if(Workvalue_s.HMI_Control_flag)
						Workvalue_s.ScreenKey_data=3;//速度+
						break;
					case 7:
					if(Workvalue_s.HMI_Control_flag){
						if(Workvalue_s.select_channel==1)
						{
							Workvalue_s.ScreenKey_data=25;//切换手柄
						}
						else
						{
							Workvalue_s.ScreenKey_data=24;//切换手柄
						}
					}
					break;
					  case 8:
								if(Workvalue_s.HMI_Control_flag)
							Workvalue_s.ScreenKey_data=11;//灌注开关
							break;
						case 9:
							if(Workvalue_s.HMI_Control_flag)
							Workvalue_s.ScreenKey_data=12;//注水开关
							break;
							case 10:
								if(Workvalue_s.HMI_Control_flag){
									Workvalue_s.FastGear_flag=1;
									Workvalue_s.ScreenKey_data=7;//注水档位
								}
							break;
						case 11:
								if(Workvalue_s.HMI_Control_flag){
										Workvalue_s.FastGear_flag=1;
									Workvalue_s.ScreenKey_data=5;//灌注档位
							}
							break;
					
					case 12:
							if(Workvalue_s.HMI_Control_flag){
						if(Workvalue_s.beep_Alarm_flag)
						{
							Workvalue_s.beep_Alarm_flag=0;
							Workvalue_s.Alarm_value=0;
								Workvalue_s.HMI_Working_flag=0;
						}
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
						if(Workvalue_s.select_channel==1){
							Workvalue_s.Alarm_value=8;
						}
						else if(Workvalue_s.select_channel==2)
						{
							Workvalue_s.Alarm_value=9;
						}
						Workvalue_s.beep_Alarm_flag=1;
						Pump_SetSpeed_A(0);//泵停止运行
						//Pump_SetSpeed_B(0);//泵停止运行
						Workvalue_s.Handle_MOTORWorking_flag=0;
						Workvalue_s.MOTORWorking_flag=0;
						Workvalue_s.Foot_start_flag=0;
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


















