//motor.c

#include "motor.h"
#include "delay.h"
#include "uart1.h"
#include "data.h"
#include "common.h"
#include "board.h"

#define MOTOR_CONTROL_FRAME_DATA_LENGTH 9U /* 手柄驱动控制帧前 9 字节为业务数据，末 2 字节保存 CRC16。 */

/*
 * 函数功能：为旧电机控制接口生成真实 CRC16，确保紧急停止帧不会被严格校验的手柄驱动拒绝。
 * 输入参数：frame 指向至少 11 字节的控制帧缓存。
 * 返回参数：无；空指针时不访问缓存。
 */
static void Motor_UpdateControlFrameCrc(uint8_t *frame)
{
	uint16_t crc; /* 保存地址至保护电流字段的 CRC16/MODBUS 结果。 */

	if (frame == NULL)
	{
		return; /* 防御无效内部调用，避免写入未知地址。 */
	}

	crc = Common_Crc16(frame, MOTOR_CONTROL_FRAME_DATA_LENGTH); /* 与手柄驱动端 CRC_Calc 的覆盖范围保持一致。 */
	frame[9] = (uint8_t)(crc & 0x00FFU); /* 驱动协议先发送 CRC 低字节。 */
	frame[10] = (uint8_t)((crc >> 8U) & 0x00FFU); /* 驱动协议后发送 CRC 高字节。 */
}

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
/*
 * 函数功能：校验有刷电机停止参数并保留旧接口组帧，不再占用已改作外控的 UART2。
 * 输入参数：MotorNum 为电机号，Mode 为方向模式，Freq 为往复频率。
 * 返回参数：无；参数非法时不生成后续控制字段。
 */
void BrushedMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0};

	/* 参数超过驱动协议范围时直接退出，避免无效电机号、模式或频率进入旧组帧数据。 */
	if ((MotorNum > 5) || (Mode > 6) || (Freq > 100))
		return ;

	dat[1] = Mode;
	dat[2] = Freq;
	dat[3] = MotorNum;

	/*
	 * UART2 已切换为外部通信协议专用口。
	 * 有刷电机运行链路当前由 sscDrive.c 统一走 UART1，本旧接口保留参数组帧但不再占用 UART2。
	 */
	(void)dat;
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
/*
 * 函数功能：校验有刷电机运行参数并保留旧接口组帧，不再占用已改作外控的 UART2。
 * 输入参数：MotorNum 为电机号，Mode 为方向模式，Freq 为往复频率，Speed/Current 为目标转速和电流。
 * 返回参数：无；参数非法时不生成后续控制字段。
 */
void BrushedMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint16_t Speed, uint16_t Current)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0};

	/* 参数超过驱动协议范围时直接退出，避免错误控制字段进入保留的有刷电机帧。 */
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
	/*
	 * UART2 已切换为外部通信协议专用口。
	 * 有刷电机运行链路当前由 sscDrive.c 统一走 UART1，本旧接口保留参数组帧但不再占用 UART2。
	 */
	(void)dat;
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

/*
 * 函数功能：向 UART1 驱动板连续发送无霍尔/有霍尔、正转/反转四组停止帧，确保未知当前模式也能停机。
 * 输入参数：MotorNum、Mode、Freq、Mode1 为历史接口参数；当前停止序列使用固定广播帧覆盖全部模式。
 * 返回参数：无。
 */
void BrushlessMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1)  
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	//uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0};
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
	
	Motor_UpdateControlFrameCrc(dat); /* 第一帧按正转、无霍尔字段生成对应 CRC。 */
	/* 先发送正转、无霍尔停止帧，覆盖当前驱动若处于无霍尔正转的情况。 */
	Uart1_SendPacket(dat, 11);
	dat[6]=0x02;
	Motor_UpdateControlFrameCrc(dat); /* 闭环类型切换到有霍尔后必须重算 CRC，不能沿用上一帧校验值。 */
	/* 两帧之间保留 5ms，给驱动板完整接收和处理上一帧的时间。 */
	Delay_ms(5);
	/* 再发送正转、有霍尔停止帧，不能依赖主控保存的闭环类型。 */
	Uart1_SendPacket(dat, 11);
	Delay_ms(5);
	dat[1] = 0x02;
	Motor_UpdateControlFrameCrc(dat); /* 方向切换到反转后重算 CRC，保证第三帧仍可被驱动接受。 */
		/* 切到反转模式并发送有霍尔停止帧，覆盖驱动当前处于反转的情况。 */
		Uart1_SendPacket(dat, 11);
		Delay_ms(5);
			dat[6]=0x01;
				Motor_UpdateControlFrameCrc(dat); /* 最后一帧恢复无霍尔字段后再次重算 CRC。 */
				/* 最后发送反转、无霍尔停止帧，使四种运行组合都收到明确停止命令。 */
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
/*
 * 函数功能：按私有协议生成无刷电机运行帧并通过 UART1 下发。
 * 输入参数：MotorNum 为电机号，Mode 为方向，Freq 为往复频率，Mode1 为闭环类型，Speed/Current 为目标值。
 * 返回参数：无；参数超出协议范围时不发送控制帧。
 */
void BrushlessMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1, uint16_t Speed, uint16_t Current)
{

	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0};
	uint16_t MotorCRC;
	/* 驱动协议不接受超范围电机号、模式和频率，拒绝发送可避免驱动进入未定义状态。 */
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






