#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscPUMPA.h"
#include "Pubinterface.h"
#include "pump_pressure_control.h"
#include "pump.h"
#include "uart5.h"
#include "uart7.h"


kernel_task_t PUMPABehaviorHandle;
QueueHandle_t PUMPAMsgQueue = NULL;
typedef struct
{
	uint8_t pump_type;//a泵区域，B泵区域，速度区域，等等
	uint16_t Value;
}PUMPAMessage_t;
static void PUMPAQueue_Init(void)
{
    PUMPAMsgQueue = Kernel_QueueCreate(5, sizeof(PUMPAMessage_t), "PUMPAMsgQueue");
}

void SendPumpAMessage(uint8_t pump_type,uint16_t value)
{
	static PUMPAMessage_t msg;
	if(PUMPAMsgQueue == NULL) return;
	if(msg.pump_type == pump_type && msg.Value == value)
	{
		/* V2.1 增加重复消息过滤，相同泵类型和速度不再反复入队，降低 25ms/100ms 任务抖动。 */
		return;
	}
	msg.pump_type = pump_type;
	msg.Value = value;
	(void)Kernel_QueueSend(PUMPAMsgQueue, &msg, 0);
}

static const pumpMessage_t *PUMPA_GetPressureSource(void)
{
	/* 固定读取 pumpMessageA；当前现场映射为 A 泵压力传感器接 SIM_UART_2/PE6。 */
	return &pumpMessageA;
}

static uint16_t PUMPA_ApplyPressureClosedLoop(uint16_t pump_speed, uint8_t *pressure_force_stop)
{
	/* pressure_source 指向本次 A 泵闭环使用的固定压力数据源。 */
	const pumpMessage_t *pressure_source = PUMPA_GetPressureSource();
	/* weight_x10 先从 volatile 结构体读到局部变量，保证一次闭环计算使用同一份重量数据。 */
	uint32_t weight_x10 = pressure_source->weight_x10;
	/* threshold_g 先从 volatile 结构体读到局部变量，保证限速比例和停止点来自同一帧阈值。 */
	uint16_t threshold_g = pressure_source->pressure_threshold;
	/* force_stop 保存本次是否已经进入硬停倍率区，返回给调用方后用于决定本周期是否暂停输出。 */
	uint8_t force_stop = PumpPressureControl_ShouldForceStop(weight_x10, threshold_g);
	/* protected_speed 保存压力闭环限速后的目标泵速，最终会再换算成 UART5 速度字段。 */
	uint16_t protected_speed = PumpPressureControl_Apply(pump_speed, weight_x10, threshold_g);

	if (pressure_force_stop != NULL)
	{
		/* pressure_force_stop 返回 1 时表示已经超过硬停倍率，调用方必须把本周期输出压到 0。 */
		*pressure_force_stop = force_stop;
	}

	/* 调用公共闭环算法：阈值以下不降速，阈值到硬停倍率之间线性降速，超过硬停倍率输出 0。 */
	return protected_speed;
}

static void PUMPA_PauseByPressureLimit(void)
{
	/*
	 * 压力超过硬停倍率时这里只做“暂停输出”标记，不清 run_flag 和手动/跟随请求。
	 * 这样压力恢复到硬停倍率以下后，PUMPAehaviors() 会继续使用原来的 speed_work 自动恢复转动。
	 */
}

void Pump_SetSpeedS_A(uint16_t uart_data,uint8_t pump_dir)
{
  	uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		dat[0] = 0xAA;
		pump_dir?(dat[1] = 0x00):(dat[1]=0x01);
		dat[2] = ((uart_data & 0xFF00) >> 8);//0x03;
		dat[3] = (uart_data & 0x00FF);//0xE8;
		dat[4] = 0xBB;
		dat[5] = 0xAA;
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
	    /* 当前整机 A/B 泵物理位置与逻辑定义相反：逻辑 A 泵改走原 B 泵物理 UART7 出口。 */
	    Uart7_SendPacket(dat, 6);
#else
	    /* 关闭互换时保持旧版接线：逻辑 A 泵走 UART5。 */
	    Uart5_SendPacket(dat, 6);
#endif
}

void PUMPAehaviors()
{

	     uint8_t pump_type = 0;
	     uint16_t pump_speed = 0;
		uint16_t uart_data = 0;
		static uint8_t pump_dir = 0;
		uint8_t pressure_force_stop = 0U;
		if(PUMPAMsgQueue != NULL)
		{
			PUMPAMessage_t msg;
			// 非阻塞方式接收消息
			if(Kernel_QueueReceive(PUMPAMsgQueue, &msg, 0) == pdTRUE)
			{
				/* 队列消息只同步公共泵状态，实际输出继续由 pumpMessageA 统一驱动，避免绕开 ExternalComm/CS1237/压力闭环链路。 */
				pumpMessageA.type=msg.pump_type;
				pumpMessageA.speed_work=msg.Value;
			}
		}
		/*
		 * 泵A任务以 pumpMessageA 作为唯一运行数据源。
		 * V2.1 队列去重可以保留，但不能改成只使用局部队列值，否则外部通信、CS1237 和压力闭环都无法统一控制泵输出。
		 */
		if(pumpMessageA.run_flag || pumpMessageA.timingDrainage_flag)
		{
			pump_type=pumpMessageA.type;
			pump_speed=pumpMessageA.speed_work;
		}
		switch(pump_type)
		{
			case DRAWWATER: //抽
			pump_dir=0;
			if(pump_speed>15)pump_speed=15;//记得宏定义最大值1.5L
			/* 类型最大值限幅完成后再做压力闭环，确保压力保护作用在真实准备输出的泵速上。 */
			pump_speed = PUMPA_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPA_PauseByPressureLimit();
			}
			uart_data=pump_speed*42;
			break;
			case INJECTWATER://注
			pump_dir=1;
			/* 注水泵速度由统一宏限幅，允许上位机设置 100~300 时继续提高实际输出。 */
			if(pump_speed>PUMP_INJECTWATER_SPEED_MAX)pump_speed=PUMP_INJECTWATER_SPEED_MAX;
			/* 注水泵按压力闭环修正流速，压力超过硬停倍率时 pump_speed 会变成 0。 */
			pump_speed = PUMPA_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPA_PauseByPressureLimit();
			}
			uart_data=(pump_speed*0.02+2.1)*pump_speed;//
				break;
			case POURWATER://灌
			pump_dir=1;
			if(pump_speed>300)pump_speed=300;//记得宏定义最大值300ml
			/* 灌注泵同样在换算 UART 数据前做闭环限速，避免高压时继续输出原设定速度。 */
			pump_speed = PUMPA_ApplyPressureClosedLoop(pump_speed, &pressure_force_stop);
			if (pressure_force_stop != 0U)
			{
				/* 高压硬停时只暂停本周期输出，不清运行请求，压力恢复后可自动继续转动。 */
				PUMPA_PauseByPressureLimit();
			}
			uart_data=pump_speed*0.62;
				break;
			default:
				break;
		}
		if(pumpMessageA.timingDrainage_flag)
		{
		   if (pressure_force_stop != 0U)
		   {
			/* 压力暂停期间不累计排空时间，避免高压等待过程中把一次有效排空动作提前耗尽。 */
		   }
           else if(pumpMessageA.timingDrainage_times++>100)
		   {
			uart_data=0;
			pumpMessageA.timingDrainage_flag=false;
			pumpMessageA.run_flag=false;
			pumpMessageA.timingDrainage_times=0;
		   }
		}
		Pump_SetSpeedS_A(uart_data,pump_dir);

}
void PUMPABehaviorTask(uint32_t event)
{
	(void)event;

	PUMPAehaviors();

}
void SscPumpATask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	PUMPAQueue_Init();
	Kernel_TaskCreate(&PUMPABehaviorHandle, PUMPABehaviorTask);
	Kernel_TaskStart(&PUMPABehaviorHandle, KERNEL_TASK_ALWAYS, 25);
}
