//pedal.c

#include "pedal.h"
#include "uart4.h"
#include "common.h"
#include "delay.h"
#include "data.h"
#include "screen.h"
#include "kernel_scheduler.h"
#include "kernel_osal.h"

kernel_task_t PEDALRECVTaskHandle;

static void PedalTaskDelayMs(uint32_t delay_ms)
{
	Kernel_DelayUntilMs(delay_ms);
}

static uint8_t Pedal_StorageHDataCMD[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB8, 0xDF, 0x6C, 0x8B};
static uint8_t Pedal_StorageLDataCMD[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB5, 0xCD, 0xF1, 0x0F};
static uint8_t Pedal_StorageMDataCMD[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB2, 0xEE, 0x7F, 0x3D};

static uint8_t Pedal_StorageHDataCMD_Left[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB8, 0xDF, 0x6C, 0x9B};
static uint8_t Pedal_StorageLDataCMD_Left[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB5, 0xCD, 0xF1, 0x1F};
static uint8_t Pedal_StorageMDataCMD_Left[8] = {0xFE, 0xEF, 0xD0, 0xB4,0xB2, 0xEE, 0x7F, 0x4D};

static uint8_t Pedal_ReadHDataCMD[8] =      {0xFE, 0xEF, 0xB6, 0xC1, 0xB8, 0xDF, 0x3E, 0x84};
static uint8_t Pedal_ReadHDataCMD_Left[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB8, 0xDF, 0x3E, 0x94};
static uint8_t Pedal_ReadLDataCMD[8] =      {0xFE, 0xEF, 0xB6, 0xC1, 0xB5, 0xCD, 0xA3, 0x00};
static uint8_t Pedal_ReadLDataCMD_Left[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB5, 0xCD, 0xA3, 0x10};
static uint8_t Pedal_ReadMDataCMD[8] =      {0xFE, 0xEF, 0xB6, 0xC1, 0xB2, 0xEE, 0x42, 0x29};
static uint8_t Pedal_ReadMDataCMD_Left[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB2, 0xEE, 0x42, 0x28}; 



//============================================================================
// 函数名称: Pedal_StorageHValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储高值
//============================================================================
void Pedal_StorageHValue(void)  //FE EF D0 B4 B8 DF 6C 8B  写高值
{
  Uart4_SendPacket(Pedal_StorageHDataCMD, 8);
}

//============================================================================
// 函数名称: Pedal_StorageLValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储低值
//============================================================================
void Pedal_StorageLValue(void)  //FE EF D0 B4 B5 CD F1 0F  写低值
{
  Uart4_SendPacket(Pedal_StorageLDataCMD, 8);
}

//============================================================================
// 函数名称: Pedal_StorageMValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储中间
//============================================================================
void Pedal_StorageMValue(void)  //FE EF D0 B4 B2 EE 7F 3D  写中间
{
  Uart4_SendPacket(Pedal_StorageMDataCMD, 8);
}

//============================================================================
// 函数名称: Pedal_StorageHValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储高值
//============================================================================
void Pedal_StorageHValue_Left(void)  //FE EF D0 B4 B8 DF 6C 9B  写高值
{
  Uart4_SendPacket(Pedal_StorageHDataCMD_Left, 8);
}

//============================================================================
// 函数名称: Pedal_StorageMValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储中间值
//============================================================================
void Pedal_StorageMValue_Left(void)  //FE EF D0 B4 B2 EE 7F 4D  写中间值
{
  Uart4_SendPacket(Pedal_StorageMDataCMD_Left, 8);

}

//============================================================================
// 函数名称: Pedal_StorageLValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 脚踏存储低值
//============================================================================
void Pedal_StorageLValue_Left(void)  //FE EF D0 B4 B5 CD F1 1F  写低值
{
  Uart4_SendPacket(Pedal_StorageLDataCMD_Left, 8);
}

//============================================================================
// 函数名称: Pedal_ReadHValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读高值
//============================================================================
void Pedal_ReadHValue(void)  //FE EF B6 C1 B8 DF 3E 84  读取高值
{
  Uart4_SendPacket(Pedal_ReadHDataCMD, 8);
}

//============================================================================
// 函数名称: Pedal_ReadHValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读高值左
//============================================================================
void Pedal_ReadHValue_Left(void)  //FE EF B6 C1 B8 DF 3E 94  读取高值
{
  Uart4_SendPacket(Pedal_ReadHDataCMD_Left, 8);
}


//============================================================================
// 函数名称: Pedal_ReadMValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读中间值
//============================================================================
void Pedal_ReadMValue(void)  //FE EF B6 C1 B2 EE 42 29  读取中间值
{
  Uart4_SendPacket(Pedal_ReadMDataCMD, 8);
}

//============================================================================
// 函数名称: Pedal_ReadMValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读中间值
//============================================================================
void Pedal_ReadMValue_Left(void)  //FE EF B6 C1 B2 EE 42 28  读取中间值
{
  Uart4_SendPacket(Pedal_ReadMDataCMD_Left, 8);
}

//============================================================================
// 函数名称: Pedal_ReadLValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读低值
//============================================================================
void Pedal_ReadLValue(void)  //FE EF B6 C1 B5 CD A3 00  读取低值
{
  Uart4_SendPacket(Pedal_ReadLDataCMD, 8);
}
//============================================================================
// 函数名称: Pedal_ReadLValue_Left()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读低值
//============================================================================
void Pedal_ReadLValue_Left(void)  //FE EF B6 C1 B5 CD A3 10  读取低值
{
  Uart4_SendPacket(Pedal_ReadLDataCMD_Left, 8);
}

//============================================================================
// 函数名称: Pedal_ReadStorageHLValue()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 读取脚踏高低值
//============================================================================
void Pedal_ReadStorageHLValue(void)
{
  Pedal_ReadHValue();
	PedalTaskDelayMs(50);

  Pedal_ReadLValue();
	PedalTaskDelayMs(50);
}

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 帧头byte | 数据长度byte | 数据类型byte | *data[byte] | crc8
//============================================================================
void PedalRecv_Scan(void)
{
  uint16_t crc = 0;

  uint16_t rlen = 0, i = 0;
  uint8_t dat[UART4_MAX_PACKET_SIZE] = { 0 }, dat1[UART4_MAX_PACKET_SIZE] = { 0 };


	
  //读取串口数据
  rlen = Uart4_DMARecvDataPeek(dat);
  if (rlen < 10)   //不够一个数据包大小
    return;

  //查询本帧数据包的帧头0xFE 0xEF
  for (i = 0; i < (rlen - 9); i++)    //最小帧数据包为4[0xFE 0xEF 0x61 0x8F 0 0 0 0 0 0]
  {
	  if ((dat[i] == 0xFE) && (dat[i + 1] == 0xEF))  //帧头
	  {
	    Common_CopyData(&dat[i], dat1, 10);    //截取10个数据

	    crc = Common_Crc16(dat1, 8);

	    //JT_Connect_Flag = Connect;
	    //实时采样的脚踏数据 FE EF    B6 C1       01        01     DATA_H DATA_L CRCH CRCL
	    if (crc == ((dat1[8] << 8) + dat1[9]))
	    {	
	      if ((dat1[2] == 0xB6) && (dat1[3] == 0xC1))
		    {
					
					if ((dat1[4] == 0x01) && (dat1[5] == 0x01)) 
					{
					  //SysFootPedalData.FootPedalType = 0;   //单踏板
						//Workvalue_s.Foot_model=2;//ssc
						Workvalue_s.Foot_type=1;
					}
          else if(((dat1[4] == 0x01) && (dat1[5] == 0x0A)) || ((dat1[4] == 0x01) && (dat1[5] == 0x0B))) 
          {
					 // SysFootPedalData.FootPedalType = 1;		//双踏板		
					//	Workvalue_s.Foot_model=1; 
						Workvalue_s.Foot_type=2;
					}
					
					if (Workvalue_s.Foot_type==2)
					{
						
							 if ((dat1[4] == 0x01) && (dat1[5] == 0x0A))//左边值
							 {
									SysFootPedalData.FootPedalADValue = (dat1[6] << 8) + dat1[7];   //左边AD值
								 
							 }
               else if ((dat1[4] == 0x01) && (dat1[5] == 0x0B))//右边的AD值
							 {
								  SysFootPedalData.FootPedalADValue_Right = (dat1[6] << 8) + dat1[7];   
								 
							 }								 
						}
						else
						{
							if ((dat1[4] == 0x01) && (dat1[5] == 0x01))
							{
								SysFootPedalData.FootPedalADValue = (dat1[6] << 8) + dat1[7];  //默认左边AD值
								//ysFootPedalData.FootPedalADValue_Right=SysFootPedalData.FootPedalADValue;
							} 					
						}				
		    }
		    //主控板发送命令给脚踏板，脚踏板回复的读取已存储
		    else if ((dat1[2] == 0xD0) && (dat1[3] == 0xB4))  
		    {
					if (Workvalue_s.Foot_type == 2)
					{
						if((dat1[4] == 0xB5) && (dat1[5] == 0xCD))//B5 CD 表示低的低值右
						{
							SysFootPedalData.FootPedalMemoryLValue_Right = (dat1[6]<<8) + dat1[7]; //JT_Memory_L_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];											
						}
						else if((dat1[4] == 0xB6) && (dat1[5] == 0xCD))//B6 CD 表示低的低值左
						{
							SysFootPedalData.FootPedalMemoryLValue_Left = (dat1[6]<<8) + dat1[7]; //JT_Memory_L_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];	
						}
	 
						if((dat1[4] == 0xB8) && (dat1[5] == 0xDF))//B8 DF表示高右
						{
							SysFootPedalData.FootPedalMemoryHValue_Right = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
						}
						else if((dat1[4] == 0xB9) && (dat1[5] == 0xDF))//B9 DF表示高左
						{
							SysFootPedalData.FootPedalMemoryHValue_Left = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];					
						}	

						if((dat1[4] == 0xB2) && (dat1[5] == 0xEE))//B2 EE表示中右
						{
							SysFootPedalData.FootPedalMemoryMValue_Right = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
						}
						else if((dat1[4] == 0xB3) && (dat1[5] == 0xEE))//B3 EE表示中左
						{
							SysFootPedalData.FootPedalMemoryMValue_Left = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];			
														
						}	
						
					}						
          else if(Workvalue_s.Foot_type==1)
          {
						if((dat1[4] == 0xB5) && (dat1[5] == 0xCD))//B5 CD 表示低的低值右
						{
							SysFootPedalData.FootPedalMemoryLValue = (dat1[6]<<8) + dat1[7]; //JT_Memory_L_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];	
							SysFootPedalData.FootPedalMemoryLValue_Right=	SysFootPedalData.FootPedalMemoryLValue;							
						}
						if((dat1[4] == 0xB8) && (dat1[5] == 0xDF))//B8 DF表示高右
						{
							SysFootPedalData.FootPedalMemoryHValue = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
							SysFootPedalData.FootPedalMemoryHValue_Right=SysFootPedalData.FootPedalMemoryHValue;
						}	
						if((dat1[4] == 0xB2) && (dat1[5] == 0xee))//中
						{
							
							SysFootPedalData.FootPedalMemoryMValue_Right = (dat1[6] << 8) + dat1[7];  //JT_Memory_H_value = (Uart1_Temp_Buff[6]<<8) + Uart1_Temp_Buff[7];
							//SysFootPedalData.FootPedalMemoryMValue_Right=SysFootPedalData.FootPedalMemoryHValue;
						}
	
						
					}
		    }
		    //电机处于停止状态，响应按键
		    else if ((SysRunData.MotorRun == MotorStop) && (dat1[2] == 0xBB) && (dat1[3] == 0xAA) && (dat1[4] == 0xCC) && (dat1[5] == 0xDD))
		    {
		      switch (dat1[7])
		      {
			      case 0x03 : SysFootPedalData.FootPedalKeyValue = 1; break; //手柄切换 长按C键1.2s
			      case 0x05 : SysFootPedalData.FootPedalKeyValue = 2;SysRunData.KeyValue=M_KEY_FOOT; break; //模式调节 短按C键200ms
			      case 0x0A : SysFootPedalData.FootPedalKeyValue = 3;SysRunData.KeyValue=L_KEY_FOOT; break; //参数- 短按B键200ms
			      case 0x0B : SysFootPedalData.FootPedalKeyValue = 4; SysRunData.KeyValue=R_KEY_FOOT;break; //参数+ 短按A键200ms
						case 0x0C : SysFootPedalData.FootPedalKeyValue = 5; break; //右間廠按
			      default : break;
		      }
		    }
		    else if ((SysRunData.MotorRun == MotorStop) && (dat1[2] == 0xBB) && (dat1[3] == 0xAA) && (dat1[4] == 0xCC) && (dat1[5] == 0x02))
		    {
		      switch (dat1[7])
		      {
			      case 0x03 : SysFootPedalData.FootPedalKeyValue = 1; 
						
						break; //手柄切换 长按C键1.2s
			      case 0x05 : SysFootPedalData.FootPedalKeyValue = 2; SysRunData.KeyValue=M_KEY_FOOT; break; //模式调节 短按C键200ms
			      case 0x0A : SysFootPedalData.FootPedalKeyValue = 3; SysRunData.KeyValue=L_KEY_FOOT; break; //左邊
			      case 0x0B : SysFootPedalData.FootPedalKeyValue = 4; SysRunData.KeyValue=R_KEY_FOOT; break; //右邊
						case 0x0C : SysFootPedalData.FootPedalKeyValue = 5; break; //右間廠按
			      default : break;
		      }
		    }
        SysFootPedalData.FootPedalOffTimes = 0;  //JT_Offtimes = 0;
		    SysFootPedalData.FootPedalConnectFlag = Connect;  //JT_Flag = Connect;

		    Common_Memset(0, dat1, 15);
		    i += 9;
	    }
	  }
  }
}

/* USER CODE BEGIN Header_PEDALRECVTaskFunc */
/**
* @brief Function implementing the PEDALRECVTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PEDALRECVTaskFunc */
void PEDALRECVTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN PEDALRECVTaskFunc */
  /* Infinite loop */
	if(Workvalue_s.HMI_Control_flag)
		return;
		PedalRecv_Scan();
  /* USER CODE END PEDALRECVTaskFunc */
}

void PedalRecvTask_Init(void)
{
  /* definition and creation of PEDALRECVTask */
	Kernel_TaskCreate(&PEDALRECVTaskHandle, PEDALRECVTaskFunc);
	Kernel_TaskStart(&PEDALRECVTaskHandle, KERNEL_TASK_ALWAYS, 3);
}












