//pump.c

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "pump.h"
#include "Pubinterface.h"
#include "pump_pressure_control.h"
#include "data.h"
#include "bsp_board.h"
#include "common.h"
#include "uart5.h"
#include "uart7.h"
#include <stdint.h>
#include "lcd.h"
#include "screen_address.h"
#include "kernel_osal.h"


uint8_t pum_close_flag_A=0; 
uint8_t pum_close_flag_B=0; 

static void PumpTaskDelayMs(uint32_t delay_ms)
{
	Kernel_DelayUntilMs(delay_ms);
}

/*
 * 函数功能：取得旧 A 泵直接输出入口使用的逻辑压力状态。
 * 输入参数：无。
 * 返回参数：返回 pumpMessageA；模拟串口层已把 SIM_UART_1/PE4 的 A 泵压力帧写入该结构。
 */
static const pumpMessage_t *PumpLegacy_GetPressureSourceA(void)
{
	/* 固定读取 pumpMessageA；当前压力线束映射为 A 泵压力传感器接 SIM_UART_1/PE4。 */
	return &pumpMessageA;
}

/*
 * 函数功能：取得旧 B 泵直接输出入口使用的逻辑压力状态。
 * 输入参数：无。
 * 返回参数：返回 pumpMessageB；模拟串口层已把 SIM_UART_2/PE6 的 B 泵压力帧写入该结构。
 */
static const pumpMessage_t *PumpLegacy_GetPressureSourceB(void)
{
	/* 固定读取 pumpMessageB；当前压力线束映射为 B 泵压力传感器接 SIM_UART_2/PE6。 */
	return &pumpMessageB;
}

static uint32_t PumpLegacy_ApplyPressureLimit(pumpMessage_t *runtime_msg,
											  const pumpMessage_t *pressure_source,
											  uint32_t output_value)
{
	/* protected_value 保存最终允许下发到泵驱动的 16 位速度/脉冲数据。 */
	uint16_t protected_value;
	/* weight_x10 从压力源拷贝到局部变量，保证本次限速比较使用同一次读取结果。 */
	uint32_t weight_x10;
	/* threshold_g 从压力源拷贝到局部变量，和 weight_x10 一起组成当前闭环判断条件。 */
	uint16_t threshold_g;
	/* force_stop 为 1 表示压力超过硬停倍率，需要把本次输出压到 0。 */
	uint8_t force_stop;

	/*
	 * runtime_msg 旧参数保留用于接口兼容，但压力保护现在只暂停本次输出，不清 run_flag。
	 * 这样旧直接输出入口在压力恢复后也可以继续沿用原来的运行请求自动恢复。
	 */
	(void)runtime_msg;

	/* 压力源为空时不能做闭环，直接保持原输出，避免空指针导致异常停机。 */
	if (pressure_source == NULL)
	{
		/* 返回原始值，保持旧路径在异常配置下的行为不变。 */
		return output_value;
	}

	/* 旧泵协议只发送 16 位速度字段，这里显式截成 16 位后再进入公共闭环算法。 */
	protected_value = (uint16_t)(output_value & 0xFFFFU);
	/* 读取当前压力重量，单位 0.1g，来自 CS1237 解析后的 pumpMessageA/B。 */
	weight_x10 = pressure_source->weight_x10;
	/* 读取当前压力阈值，单位 g，来自压力模块上报的 ThresholdG。 */
	threshold_g = pressure_source->pressure_threshold;
	/* 判断是否已经进入宏配置硬停倍率的停泵区间。 */
	force_stop = PumpPressureControl_ShouldForceStop(weight_x10, threshold_g);
	/* 对旧的直接输出值同样做线性限速，避免旧 UI/参数路径绕过 sscPUMPA/sscPUMPB。 */
	protected_value = PumpPressureControl_Apply(protected_value, weight_x10, threshold_g);

	/* 进入硬停泵区间时只把本次输出压为 0，不清运行标志，压力恢复后允许自动续转。 */
	if (force_stop != 0U)
	{
		/* 硬停泵输出必须为 0，即使线性算法后续调整也不能重新放大。 */
		protected_value = 0U;
	}

	/* 返回最终允许写入泵 UART 帧的速度字段。 */
	return (uint32_t)protected_value;
}

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


//============================================================================
// 函数名称: Pump_SetSpeed_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵的脉冲翻转周期设置
//============================================================================
void Pump_SetSpeed_B(uint32_t s)
{
	/* 旧 B 泵直接输出入口也套压力保护，防止屏幕/参数路径绕过 sscPUMPB 的闭环。 */
	s = PumpLegacy_ApplyPressureLimit(&pumpMessageB, PumpLegacy_GetPressureSourceB(), s);
	//ssc  加上标志位
	if(s)
	{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffE0);
	}
	else
	{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffff);
	}
		
		uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		dat[0] = 0xAA;
		#ifdef water_uptake
		dat[1] = 0x01;
		#else
		dat[1] = 0x00;
		#endif
		dat[2] = ((s & 0xFF00) >> 8);//0x03;
		dat[3] = (s & 0x00FF);//0xE8;
		dat[4] = 0xBB;
		dat[5] = 0xAA;
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

//============================================================================
// 函数名称: Pump_SetSpeed_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵的脉冲翻转周期设置
//============================================================================
void Pump_SetSpeed_A(uint32_t s)
{
 static	uint8_t repeat_data;
	/* 旧 A 泵直接输出入口也套压力保护，防止手柄/报警停泵路径绕过 sscPUMPA 的闭环。 */
	s = PumpLegacy_ApplyPressureLimit(&pumpMessageA, PumpLegacy_GetPressureSourceA(), s);
	//ssc  加上标志位
	
	if((repeat_data!=s) || (s == 0U)){
		if(s)
		{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,0xffE0);
		}
		else
		{
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,0xffff);
		}
	uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	dat[0] = 0xAA;
	dat[1] = 0x01;
	dat[2] = ((s & 0xFF00) >> 8);//0x03;
	dat[3] = (s & 0x00FF);//0xE8;
	dat[4] = 0xBB;
	dat[5] = 0xAA;

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

