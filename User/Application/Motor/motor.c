//motor.c

#include "motor.h"
#include "delay.h"
#include "uart1.h"
#include "uart2.h"
#include "data.h"
#include "common.h"
#include "board.h"

//============================================================================
// 有刷电机控制（Brushed motor control）
//============================================================================

/*
 0	 帧头	        AA
 1	 模式	        01正转、02反转、03往复
 2	 往复频率	    0-100
 3	 电机选择	    1电机1 /2电机2
 4	 设定转速H	  xx
 5	 设定转速L	  xx
 6	 帧尾/校验	  BB
 7	 帧尾/校验	  AA
*/

//============================================================================
// 函数名称: BrushedMotor_Stop()
// 功能描述: 发送停止数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
// 输    出: 无
// 函数说明:
//============================================================================
void BrushedMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0};

	if ((MotorNum > 5) || (Mode > 6) || (Freq > 100))
		return ;

	dat[1] = Mode;
	dat[2] = Freq;
	dat[3] = MotorNum;

	Uart2_SendPacket(dat, 11);
}

//============================================================================
// 函数名称: BrushedMotor_Run()
// 功能描述: 发送运行数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Speed：转速
//           Current：电流
// 输    出: 无
// 函数说明:
//============================================================================
void BrushedMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint16_t Speed, uint16_t Current)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0};

	if ((MotorNum > 5) || (Mode > 6) || (Freq > 100))
		return ;

	dat[1] = Mode;
	dat[2] = Freq;
	dat[3] = MotorNum;
	dat[4] = 0x01;
	dat[5] = ((Speed & 0xFF00) >> 8);
	dat[6] = (Speed & 0x00FF);
	dat[7] = ((Current & 0xFF00) >> 8);
	dat[8] = (Current & 0x00FF);
	Uart2_SendPacket(dat, 11);
}


//============================================================================
// 无刷电机控制（Brushless motor control）
//============================================================================

/*
       帧格式	             内容
 [0]	  帧头        0XAA
 [1]	  控制模式:   [0x01]正转；[0x02]:反转；[0x03]:往复正反转
 [2]	  往复的频率  0-100；
 [3] 	  电机选择：  1：电机1  2：电机2
 [4]	  转速H：
 [5] 	  转速L:
 [6]	  闭环方式：0x02 方波霍尔
 [7]	  0xBB/CRC
 [8]	  0xAA/CRC
*/

//============================================================================
// 函数名称: BrushlessMotor_Stop()
// 功能描述: 发送停止数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Mode1：闭环方式 0x01 无霍尔  0x02 有霍尔,0x03有刷电机
// 输    出: 无
// 函数说明:
//============================================================================


//AA 01 00 01 00 00 02 00 00  BB AA

void BrushlessMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1)  
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0xbb, 0xaa};
	//uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0};
	uint16_t MotorCRC;
//	if ((MotorNum > 5) || (Mode > 6) || (Freq > 100))
//		return ;

//	dat[1] = Mode;
//	dat[2] = Freq;
//	dat[3] = MotorNum;
//	dat[6] = Mode1;
//	dat[7] = 0;
//	dat[8] = 0;
	
//  MotorCRC = Common_Crc16(dat, 9);
//	dat[9] =  (MotorCRC & 0x00FF);
//	dat[10] = ((MotorCRC & 0xFF00) >> 8);
	
	Uart1_SendPacket(dat, 11);
	dat[6]=0x02;
	Delay_ms(5);
	Uart1_SendPacket(dat, 11);
	Delay_ms(5);
	dat[1] = 0x02;
		Uart1_SendPacket(dat, 11);
		Delay_ms(5);
			dat[6]=0x01;
				Uart1_SendPacket(dat, 11);
	
}

//============================================================================
// 函数名称: BrushlessMotor_Run()
// 功能描述: 发送运行数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Mode1：闭环方式 0x01 无霍尔  0x02 有霍尔
//           Speed：转速
// 输    出: 无
// 函数说明:
//============================================================================
void BrushlessMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1, uint16_t Speed, uint16_t Current)
{

	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0};
	uint16_t MotorCRC;
	if ((MotorNum > 5) || (Mode > 6) || (Freq > 100))
		return ;

	dat[1] = Mode;
	dat[2] = Freq;
	dat[3] = MotorNum;
	dat[4] = ((Speed & 0xFF00) >> 8);
	dat[5] = (Speed & 0x00FF);
	dat[6] = Mode1;
	dat[7] = ((Current & 0xFF00) >> 8);
	dat[8] = (Current & 0x00FF);
	
  MotorCRC = Common_Crc16(dat, 9);
	dat[9] =  (MotorCRC & 0x00FF);
	dat[10] = ((MotorCRC & 0xFF00) >> 8);	
	
	Uart1_SendPacket(dat, 11);
}

//============================================================================
// 函数名称: BrushlessMotor_SetPosition()
// 功能描述: 设置电机位置
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void BrushlessMotor_SetPosition(uint8_t MotorNum, uint8_t Type, uint16_t NUM)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0};
  uint16_t MotorCRC;

	dat[1] = Type;
	dat[2] = 0;
	dat[3] = MotorNum;
	dat[4] = ((NUM & 0xFF00) >> 8);
	dat[5] = (NUM & 0x00FF);
	dat[6] = 0x02;
	dat[7] = 0;
	dat[8] = 0;
	
  MotorCRC = Common_Crc16(dat, 9);
	dat[9] =  (MotorCRC & 0x00FF);
	dat[10] = ((MotorCRC & 0xFF00) >> 8);
		
	Uart1_SendPacket(dat, 11);
}

//============================================================================
// 函数名称: Motor_ErrorEmergencyStop_Ctrl()
// 功能描述: 电机急停安装到主板上
// 输　  入:
// 输    出:
// 函数说明: 
//============================================================================
void Motor_ErrorEmergencyStop_Ctrl(void)
{
	//无刷无霍尔电机1停
  BrushlessMotor_Stop(1, 1, 0, 1);

	//有刷电机1停
	BrushlessMotor_Stop(1, 1, 0, 3);

	//无刷有霍尔电机1停
  BrushlessMotor_Stop(1, 1, 0, 2);

	//无刷无霍尔电机2停
  BrushlessMotor_Stop(2, 1, 0, 1);

	//有刷电机2停
	BrushlessMotor_Stop(2, 1, 0, 3);

	//无刷有霍尔电机2停
  BrushlessMotor_Stop(2, 1, 0, 2);
}






