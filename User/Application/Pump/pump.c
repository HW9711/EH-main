//pump.c

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "pump.h"
#include "Pubinterface.h"
#include "pump_pressure_control.h"
#include "data.h"
#include "board.h"
#include "common.h"
#include "uart5.h"
#include "uart7.h"
#include <stdint.h>
#include "lcd.h"
#include "screen_address.h"
#include "kernel_osal.h"


uint8_t pum_close_flag_A=0; 
uint8_t pum_close_flag_B=0; 

/*
 * 函数功能：按任务定时基准等待指定时间，用于间隔发送两帧相同命令。
 * 输入参数：delay_ms 为等待时间，单位 ms。
 * 返回参数：无。
 */
static void PumpTaskDelayMs(uint32_t delay_ms)
{
	Kernel_DelayUntilMs(delay_ms);
}

/*
 * 函数功能：旧直发接口在压力板未就绪时输出零速；排空时不检查压力，真实超压不减速或停泵。
 * 输入参数：runtime_msg 为对应逻辑泵状态；output_value 为旧协议待发送速度值。
 * 返回参数：返回协议 16 位速度；压力检查启用、非排空且压力板未就绪时返回 0。
 */
static uint32_t PumpLegacy_ApplyReadiness(const pumpMessage_t *runtime_msg, uint32_t output_value)
{
	uint16_t output_speed = (uint16_t)(output_value & 0xFFFFU); /* 保持旧协议速度字段的 16 位范围。 */

	if (runtime_msg->timingDrainage_flag || runtime_msg->pedalDrainage_flag)
	{
		return output_speed; /* 定时排空和脚踏/HMI 轻排均忽略压力状态。 */
	}
	return PumpPressureControl_ApplyReadiness(output_speed, runtime_msg->weight_x10); /* 真实超压仍返回原速度，报警由对应 25ms 泵任务确认。 */
}

/*
 * 函数功能：把泵调试数据写到刀具规格显示区域，供现场查看。
 * 输入参数：point 为调试位置编号；value 为数值；detail 为附加状态。
 * 返回参数：无。
 */
void PumpDebugPoint(uint16_t point, uint16_t value, uint8_t detail)
{
	(void)value;
	(void)detail;
	LCD_Show_2byte_Number(UIDP_LCD_SP_TOOL_SPEC_COLOR,34);
	LCD_IntegratedCutterData_Update(UIDP_LCD_VP_TOOL_SPEC_TEXT, point, value, detail);
}

//static uint16_t PumpSpeed = 0;
//static uint16_t PumpSpeedLast = 0;
//static uint8_t PumpRunFlag = 0;
//static uint8_t Index = 0;

//泵缓启动
uint16_t PumpStartUp[25] =
{
  9374,
  3124,
  2082,
  1561,
  1249,
  985,
  749,
  567,
  479,
  415,
  359,
  316,
  283,
  227,
  188,
  162,
  141,
  125,
  111,
  102,
  93,
  86,
  79,
  79,
};


/*
 * 函数功能：检查 B 泵压力板是否就绪并限制命令最大值，再发送速度、刷新颜色；真实超压不改速度。
 * 输入参数：s 为协议速度整数，0 表示停止；不是流量值，不在此换算。
 * 返回参数：无。
 */
void Pump_SetSpeed_B(uint32_t s)
{
	s = PumpLegacy_ApplyReadiness(&pumpMessageB, s); /* 排空不检查压力；非排空且未就绪时输出零速，真实超压保持原速度。 */
	if (s > PUMP_DRIVER_COMMAND_SPEED_MAX)
	{
		s = PUMP_DRIVER_COMMAND_SPEED_MAX; /* 旧直连入口同样限制到当前整机最大合法驱动速度 630。 */
	}
	// 颜色按准备下发的速度命令更新。
	/* 非零命令显示黄色，零速显示白色；颜色不是驱动实测转速反馈。 */
	if(s)
	{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffE0);
	}
	else
	{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffff);
	}
		
		uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		uint16_t crc; /* 旧直连入口也必须生成与周期任务相同的 CRC16 控制帧。 */
		dat[0] = 0xAA;
		#ifdef water_uptake
		dat[1] = 0x01;
		#else
		dat[1] = 0x00;
		#endif
		dat[2] = ((s & 0xFF00) >> 8);//0x03;
		dat[3] = (s & 0x00FF);//0xE8;
		crc = Common_Crc16(dat, 4U);          /* CRC 覆盖帧头、方向和速度高低字节。 */
		dat[4] = (uint8_t)(crc & 0x00FFU);    /* 第 5 字节发送 CRC 低字节。 */
		dat[5] = (uint8_t)((crc >> 8U) & 0x00FFU); /* 第 6 字节发送 CRC 高字节。 */
		//PumpDebugPoint((s > 0U) ? 301U : 302U, (uint16_t)s, dat[3]);

#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
		/* 当前整机逻辑 B 泵对应原 A 泵物理出口，旧直连入口也必须跟随互换到 UART5。 */
		Uart5_SendPacket(dat, 6);
#else
		/* 关闭互换时保持旧版接线：逻辑 B 泵走 UART7。 */
		Uart7_SendPacket(dat, 6);
#endif
		//PumpDebugPoint(sendResult ? 311U : 312U, (uint16_t)s, dat[3]);
		PumpTaskDelayMs(10);
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
		/* 第二帧重复发送也保持同一个物理出口，避免两路泵同时收到不同节拍的重复帧。 */
		Uart5_SendPacket(dat, 6);
#else
		/* 关闭互换时保持旧版接线：逻辑 B 泵走 UART7。 */
		Uart7_SendPacket(dat, 6);
#endif
		//PumpDebugPoint(sendResult ? 321U : 322U, (uint16_t)s, dat[3]);
	
  //2.有流量
  //2.1计算脉宽
//  PumpSpeed = 18750 / s - 1;
}

/*
 * 函数功能：检查 A 泵压力板是否就绪并限制命令最大值，再发送速度、刷新颜色；真实超压不改速度。
 * 输入参数：s 为协议速度整数，0 表示停止；不是流量值，不在此换算。
 * 返回参数：无。
 */
void Pump_SetSpeed_A(uint32_t s)
{
 static	uint8_t repeat_data; /* 旧接口只保存上次命令的低 8 位；大于 255 的速度仍会被重复发送。 */
	s = PumpLegacy_ApplyReadiness(&pumpMessageA, s); /* 排空不检查压力；非排空且未就绪时输出零速，真实超压保持原速度。 */
	if (s > PUMP_DRIVER_COMMAND_SPEED_MAX)
	{
		s = PUMP_DRIVER_COMMAND_SPEED_MAX; /* 旧直连入口同样限制到当前整机最大合法驱动速度 630。 */
	}
	// 按上次记录的命令值判断是否需要再次发送。
	
	/* 与 8 位历史记录不同或本次为零速时发送；零速总是重发，避免漏掉停泵命令。 */
	if((repeat_data!=s) || (s == 0U)){
		/* 非零命令显示黄色，零速显示白色；颜色不证明泵已经实际转动。 */
		if(s)
		{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,0xffE0);
		}
		else
		{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,0xffff);
		}
	uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint16_t crc; /* 旧直连入口也必须生成与周期任务相同的 CRC16 控制帧。 */
	
	dat[0] = 0xAA;
	dat[1] = 0x01;
	dat[2] = ((s & 0xFF00) >> 8);//0x03;
	dat[3] = (s & 0x00FF);//0xE8;
	crc = Common_Crc16(dat, 4U);          /* CRC 覆盖帧头、方向和速度高低字节。 */
	dat[4] = (uint8_t)(crc & 0x00FFU);    /* 第 5 字节发送 CRC 低字节。 */
	dat[5] = (uint8_t)((crc >> 8U) & 0x00FFU); /* 第 6 字节发送 CRC 高字节。 */

#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	/* 当前整机逻辑 A 泵对应原 B 泵物理出口，旧直连入口也必须跟随互换到 UART7。 */
	Uart7_SendPacket(dat, 6);
#else
	/* 关闭互换时保持旧版接线：逻辑 A 泵走 UART5。 */
	Uart5_SendPacket(dat, 6);
#endif
	PumpTaskDelayMs(10);
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	/* 第二帧重复发送也保持同一个物理出口，避免逻辑 A 泵仍打到右侧物理泵。 */
	Uart7_SendPacket(dat, 6);
#else
	/* 关闭互换时保持旧版接线：逻辑 A 泵走 UART5。 */
	Uart5_SendPacket(dat, 6);
#endif
		repeat_data=s;
}
  //2.有流量
  //2.1计算脉宽
//  PumpSpeed = 18750 / s - 1;
}
//============================================================================

