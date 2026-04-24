#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscPUMPA.h"
#include "Pubinterface.h"
#include "uart5.h"


kernel_task_t PUMPABehaviorHandle;
QueueHandle_t PUMPAMsgQueue = NULL;
typedef struct 
{
	uint8_t pump_type;//a泵区域，B泵区域，速度区域，等等
	uint16_t Value;
}PUMPAMessage_t;
static void PUMPAQueue_Init(void)
{
    PUMPAMsgQueue = Kernel_QueueCreate(2, sizeof(PUMPAMessage_t), "PUMPAMsgQueue");
}
void SendPumpAMessage(uint8_t pump_type,uint16_t value)
{
	if(PUMPAMsgQueue == NULL) return;
	PUMPAMessage_t msg;
	msg.pump_type =pump_type ;
	msg.Value = value;
	(void)Kernel_QueueSend(PUMPAMsgQueue, &msg, 0);
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
	    Uart5_SendPacket(dat, 6);
}

void PUMPAehaviors()
{
	   
	     uint8_t pump_type = 0;
	     uint16_t pump_speed = 0;
		uint16_t uart_data = 0;
		static uint8_t pump_dir = 0;
		if(PUMPAMsgQueue != NULL)
		{
			PUMPAMessage_t msg;
			// 非阻塞方式接收消息
			if(Kernel_QueueReceive(PUMPAMsgQueue, &msg, 0) == pdTRUE)
			{
				pumpMessageA.type=msg.pump_type;
				pumpMessageA.speed_work=msg.Value;
			}
		}
		/*
		 * 泵A任务以 pumpMessageA 作为唯一运行数据源。
		 * 队列仅作为后续兼容入口更新设定值，实际输出由公共接口状态决定。
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
			uart_data=pump_speed*42;
			break;
			case INJECTWATER://注
			pump_dir=1;
			if(pump_speed>70)pump_speed=70;//记得宏定义最大值70ml
			uart_data=(pump_speed*0.02+2.1)*pump_speed;//
				break;
			case POURWATER://灌
			pump_dir=1;
			if(pump_speed>300)pump_speed=300;//记得宏定义最大值300ml
			uart_data=pump_speed*0.62;
				break;
			default:
				break;
		}
		if(pumpMessageA.timingDrainage_flag)
		{
           if(pumpMessageA.timingDrainage_times++>100)
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
	Kernel_TaskStart(&PUMPABehaviorHandle, KERNEL_TASK_ALWAYS, 100);
}
