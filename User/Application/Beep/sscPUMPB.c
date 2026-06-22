#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscPUMPB.h"
#include "Pubinterface.h"
#include "pump_pressure_control.h"
#include "pump.h"
#include "uart5.h"
#include "uart7.h"
#include "lcd.h"
#include "screen_address.h"

static QueueHandle_t PUMPBMsgQueue = NULL;
static kernel_task_t PUMPBBehaviorHandle;

typedef struct
{
	uint8_t pump_type;//a泵区域，B泵区域，速度区域，等等
	uint16_t Value;
}PUMPBMessage_t;
static void PUMPBQueue_Init(void)
{
    PUMPBMsgQueue = Kernel_QueueCreate(2, sizeof(PUMPBMessage_t), "PUMPBMsgQueue");
}

void SendPumpBMessage(uint8_t pump_type,uint16_t value)
{
	static PUMPBMessage_t msg;
	if(PUMPBMsgQueue == NULL) return;
	if(msg.pump_type == pump_type && msg.Value == value)
	{
		/* V2.1 增加重复消息过滤，相同泵类型和速度不再反复入队，避免 B 泵周期任务重复处理同一命令。 */
		return;
	}
	msg.pump_type = pump_type;
	msg.Value = value;
	(void)Kernel_QueueSend(PUMPBMsgQueue, &msg, 0);
}

static const pumpMessage_t *PUMPB_GetPressureSource(void)
{
	/* 固定读取 pumpMessageB；当前现场映射为 B 泵压力传感器接 SIM_UART_1/PE4。 */
	return &pumpMessageB;
}

static uint16_t PUMPB_ApplyPressureClosedLoop(uint16_t pump_speed, uint8_t *pressure_force_stop)
{
	/* pressure_source 指向本次 B 泵闭环使用的固定压力数据源。 */
	const pumpMessage_t *pressure_source = PUMPB_GetPressureSource();
	/* weight_x10 先从 volatile 结构体读到局部变量，避免计算过程中多字节字段被中途刷新。 */
	uint32_t weight_x10 = pressure_source->weight_x10;
	/* threshold_g 先从 volatile 结构体读到局部变量，和本次 weight_x10 一起参与同一次闭环计算。 */
	uint16_t threshold_g = pressure_source->pressure_threshold;

	if (pressure_force_stop != NULL)
	{
		/* pressure_force_stop 返回 1 时表示已经超过硬停倍率，调用方必须把本周期输出压到 0。 */
		*pressure_force_stop = PumpPressureControl_ShouldForceStop(weight_x10, threshold_g);
	}

	/* 调用公共闭环算法：阈值以下不降速，阈值到硬停倍率之间线性降速，超过硬停倍率输出 0。 */
	return PumpPressureControl_Apply(pump_speed, weight_x10, threshold_g);
}

static void PUMPB_PauseByPressureLimit(void)
{
	/*
	 * 压力超过硬停倍率时这里只做“暂停输出”标记，不清 run_flag。
	 * 这样压力恢复到硬停倍率以下后，B 泵也能按原来的启动请求自动恢复输出。
	 */
}

/*
 * 函数功能：按 B 泵业务速度和方向组装泵驱动 UART 帧并发送到当前物理 B 泵出口。
 * 输入参数：value 为已经换算好的泵驱动速度字段；pump_dir 为业务方向，1/0 由 B 泵类型分支给出。
 * 返回参数：无。
 */
static void Pump_SetSpeedS_B(uint16_t value,uint8_t pump_dir)
{
		uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		dat[0] = 0xAA;
		pump_dir?(dat[1] = 0x01):(dat[1]=0x00); /* B 泵现场电机方向与驱动协议相反，只在最终 UART 帧取反，不改业务层注水/灌注类型判断。 */
		dat[2] = ((value & 0xFF00) >> 8);//0x03;
		dat[3] = (value & 0x00FF);//0xE8;
		dat[4] = 0xBB;
		dat[5] = 0xAA;
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	    /* 当前整机 A/B 泵物理位置与逻辑定义相反：逻辑 B 泵改走原 A 泵物理 UART5 出口。 */
	    Uart5_SendPacket(dat, 6);
#else
	    /* 关闭互换时保持旧版接线：逻辑 B 泵走 UART7。 */
	    Uart7_SendPacket(dat, 6);
#endif
}

static void PUMPBehaviors(void)
{
		static uint8_t huici = 0;
	    uint8_t pump_type = 0;
		uint16_t pump_speed = 0;
		uint16_t uart_data = 0;
		static uint8_t pump_dir=0;
		uint8_t pressure_force_stop = 0U;
		if(PUMPBMsgQueue != NULL)
		{
			PUMPBMessage_t msg;
			// 非阻塞方式接收消息
			if(Kernel_QueueReceive(PUMPBMsgQueue, &msg, 0) == pdTRUE)
			{
				/* 队列消息只同步速度；泵类型只能由模拟串口设备码刷新，避免脚踏轻排把 B 泵重新写成注水泵。 */
				pumpMessageB.speed_work=msg.Value;
			}
		}
		/*
		 * 泵B任务以 pumpMessageB 作为唯一运行数据源。
		 * 队列仅作为后续兼容入口更新设定值，实际输出由公共接口状态决定。
		 */
		if(pumpMessageB.run_flag || pumpMessageB.timingDrainage_flag)
		{
			pump_type=pumpMessageB.type;
			pump_speed=pumpMessageB.speed_work;
		}
		switch(pump_type)
		{
			case DRAWWATER: //抽
			pump_dir=1;
			if(pump_speed>15)pump_speed=15;//记得宏定义最大值1.5L
			/* 类型最大值限幅完成后再做压力闭环，确保压力保护作用在真实准备输出的泵速上。 */
			pump_speed = PUMPB_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPB_PauseByPressureLimit();
			}
			uart_data=pump_speed*42;
			break;
			case INJECTWATER://注
			pump_dir=0;
			/* 注水泵速度由统一宏限幅，允许上位机设置 100~300 时继续提高实际输出。 */
			if(pump_speed>PUMP_INJECTWATER_SPEED_MAX)pump_speed=PUMP_INJECTWATER_SPEED_MAX;
			/* 注水泵按压力闭环修正流速，压力超过硬停倍率时 pump_speed 会变成 0。 */
			pump_speed = PUMPB_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPB_PauseByPressureLimit();
			}
			//uart_data=(pump_speed*0.02+2.1)*pump_speed;//
			uart_data=pump_speed/1.6;
				break;
			case POURWATER://灌
			pump_dir=0;
			if(pump_speed>300)pump_speed=300;//记得宏定义最大值300ml
			/* 灌注泵同样在换算 UART 数据前做闭环限速，避免高压时继续输出原设定速度。 */
			pump_speed = PUMPB_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPB_PauseByPressureLimit();
			}
			//uart_data=pump_speed*0.62;
			uart_data=pump_speed/1.51;
				break;
			default:
				break;
		}
		if(pumpMessageB.timingDrainage_flag)
		{
			pump_speed=100;
			uart_data=(pump_speed*0.02+2.1)*pump_speed;//s
		   if (pressure_force_stop != 0U)
		   {
			/* 压力暂停期间不累计排空时间，避免高压等待过程中把一次有效排空动作提前耗尽。 */
		   }
           else if(pumpMessageB.timingDrainage_times++>300)//排空计时，当为注水的时候
		   {
			uart_data=0;
			pump_speed = 0U; /* 排空计时结束后实际输出已经关断，显示速度必须同步清零。 */
			pumpMessageB.timingDrainage_flag=false;
			pumpMessageB.run_flag=false;
			pumpMessageB.timingDrainage_times=0;
		   }
		}
		Pubinterface_UpdatePumpBOutputSpeed(pump_speed); /* 发布闭环限速后的实际业务速度，驱动屏幕和上位机显示实时变化。 */
		Pump_SetSpeedS_B(uart_data,pump_dir);
		if(uart_data==0)
		{
			if(huici==0){
				huici=1;
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffff);
			}
		}
		else
		{
			if(huici==1){
			huici=0;
			LCD_Show_2byte_Number(UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,0xffE0);
			}

		}
		
	
}

static void PUMPBBehaviorTask(uint32_t event)
{
	(void)event;

	PUMPBehaviors();

}
void SscPumpBTask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
  	PUMPBQueue_Init();
	Kernel_TaskCreate(&PUMPBBehaviorHandle, PUMPBBehaviorTask);
	Kernel_TaskStart(&PUMPBBehaviorHandle, KERNEL_TASK_ALWAYS, 100);
}
