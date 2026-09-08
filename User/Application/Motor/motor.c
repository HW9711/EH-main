//motor.c

#include "motor.h"
#include "delay.h"
#include "uart1.h"
#include "data.h"
#include "common.h"
#include "board.h"

#define MOTOR_CONTROL_FRAME_DATA_LENGTH 9U /* 手柄驱动控制帧前 9 字节为业务数据，末 2 字节保存 CRC16。 */

/*
 * 函数功能：计算电机控制帧的 CRC16，写入末尾两字节，供驱动板检查数据是否完整。
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

/* 下方两个有刷接口只保留旧组帧代码，不发送数据。当前有刷控制由 sscDrive.c 通过 UART1 发送。 */

//============================================================================
// 函数名称: BrushedMotor_Stop()
// 功能描述: 旧有刷停止接口，当前只组帧、不发送。
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
// 输    出: 无
// 函数说明:
//============================================================================
/*
 * 函数功能：检查旧有刷停止接口的参数并在局部数组中组帧；不发送数据，不会直接让电机停止。
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
// 功能描述: 旧有刷运行接口，当前只组帧、不发送。
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Speed：转速
//           Current：电流
// 输    出: 无
// 函数说明:
//============================================================================
/*
 * 函数功能：检查旧有刷运行接口的参数并在局部数组中组帧；不发送数据，不会直接启动电机。
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

/* 当前发送的 11 字节帧，下标从 0 开始：
 * [0] 0xAA；[1] 方向；[2] 往复频率；[3] 电机通道/类型；
 * [4..5] 速度，高字节在前；[6] 运行方式；[7..8] 保护电流，高字节在前；
 * [9..10] 前 9 字节的 CRC16，低字节在前。末尾不再固定写 BB AA。
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


//零速帧示例字段：AA 01 00 01 00 00 02 00 00，末尾两字节根据这些字段计算 CRC。

/*
 * 函数功能：向 UART1 固定发送物理通道 1 的四组零速帧，依次为正转无霍尔、正转有霍尔、反转有霍尔、反转无霍尔。
 * 输入参数：MotorNum、Mode、Freq、Mode1 均为保留参数，当前不使用；不能靠传入 MotorNum=2 停止通道 2。
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
				/* 最后发送通道 1 的反转、无霍尔零速帧，完成这四组固定命令。 */
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
 * 函数功能：把调用方给出的电机参数直接写入控制帧，并通过 UART1 发送。
 * 输入参数：MotorNum 为电机号，Mode 为方向，Freq 为频率协议值，Mode1 为运行方式；Speed 单位 10rpm，Current 单位 0.01A。
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

/*
 * 函数功能：按调用方给定的模式和位置值组装定位帧，通过 UART1 发送。
 * 输入参数：MotorNum 为协议电机号；Type 原样写入模式字段；NUM 原样拆成高低字节，调用方须保证符合驱动定位协议。
 * 返回参数：无；本函数不检查参数范围。
 */
void BrushlessMotor_SetPosition(uint8_t MotorNum, uint8_t Type, uint16_t NUM)
{
	uint8_t dat[12] = {0xAA, 0x01, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0};
  uint16_t MotorCRC;

	dat[1] = Type; /* 直接使用调用方指定的定位模式。 */
	dat[2] = 0; /* 定位帧不设置往复频率。 */
	dat[3] = MotorNum; /* 原样选择协议电机号，不在此处交换 A/B 通道。 */
	dat[4] = ((NUM & 0xFF00) >> 8); /* 定位值高字节先发送。 */
	dat[5] = (NUM & 0x00FF); /* 定位值低字节后发送，不额外换算单位。 */
	dat[6] = 0x02; /* 固定使用有霍尔定位方式。 */
	dat[7] = 0; /* 保护电流高字节为 0。 */
	dat[8] = 0; /* 保护电流整体为 0，使用驱动内部默认设置。 */
	
  MotorCRC = Common_Crc16(dat, 9);
	dat[9] =  (MotorCRC & 0x00FF);
	dat[10] = ((MotorCRC & 0xFF00) >> 8);
		
	Uart1_SendPacket(dat, 11);
}

/*
 * 函数功能：保留旧急停调用顺序，连续调用六次停止接口。
 * 输入参数：无。
 * 返回参数：无。注意被调用函数当前忽略参数，因此实际重复发送通道 1 的四组零速帧，并未按这里的参数分别停两路。
 */
void Motor_ErrorEmergencyStop_Ctrl(void)
{
	//保留旧参数“通道 1、无霍尔”；实际帧由 BrushlessMotor_Stop 的固定序列决定。
  BrushlessMotor_Stop(1, 1, 0, 1);

	//保留旧参数“通道 1、有刷”；当前停止接口不使用该模式参数。
	BrushlessMotor_Stop(1, 1, 0, 3);

	//保留旧参数“通道 1、有霍尔”；再次发送同一组固定零速帧。
  BrushlessMotor_Stop(1, 1, 0, 2);

	//保留旧参数“通道 2、无霍尔”；当前停止接口不使用通道参数，实际仍发通道 1。
  BrushlessMotor_Stop(2, 1, 0, 1);

	//保留旧参数“通道 2、有刷”；当前停止接口不使用通道和模式参数。
	BrushlessMotor_Stop(2, 1, 0, 3);

	//保留旧参数“通道 2、有霍尔”；实际仍重复通道 1 的固定零速帧。
  BrushlessMotor_Stop(2, 1, 0, 2);
}






