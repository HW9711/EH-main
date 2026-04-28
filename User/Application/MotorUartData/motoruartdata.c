//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "data.h"
#include "uart1.h"
#include "pump.h"
#include <string.h>
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "sscBEEP.h"

kernel_task_t MOTORUARTTaskHandle;

/*
 * UART2 已由 ExternalComm 独立任务接管。
 * 本模块只保留 UART1 驱动板接收解析，避免旧 6 字节 HMI 短帧再次读取 UART2 DMA 缓冲。
 */
static void MotorUart_SetAlarm(uint8_t alarm_value)
{
	WorkMessage.alarm_value = alarm_value;
	WorkMessage.alarm_flag = (alarm_value != 0U);
	SendAlarmMessage(alarm_value);
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
  /*
   * UART2 已由 ExternalComm 独立任务接管，用于新的外部通信协议。
   * 本任务只保留 UART1 驱动板接收，避免两个任务同时读取 UART2 DMA 缓冲。
   */
  BrushlessMotorUartData_ReceiveData();
  /* USER CODE END MOTORUARTTaskFunc */
}

void MotorUartData_Init(void)
{
  /* definition and creation of MOTORUARTTask */
	Kernel_TaskCreate(&MOTORUARTTaskHandle, MOTORUARTTaskFunc);
	Kernel_TaskStart(&MOTORUARTTaskHandle, KERNEL_TASK_ALWAYS, 3);//3
}






