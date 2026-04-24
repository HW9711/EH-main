#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscPUMPB.h"
#include "Pubinterface.h"
#include "uart7.h"

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
	if(PUMPBMsgQueue == NULL) return;
	PUMPBMessage_t msg;
	msg.pump_type =pump_type ;
	msg.Value = value;
	(void)Kernel_QueueSend(PUMPBMsgQueue, &msg, 0);
}


static void Pump_SetSpeedS_B(uint16_t value,uint8_t pump_dir)
{
		uint8_t dat[6] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
		dat[0] = 0xAA;
		pump_dir?(dat[1] = 0x00):(dat[1]=0x01);
		dat[2] = ((value & 0xFF00) >> 8);//0x03;
		dat[3] = (value & 0x00FF);//0xE8;
		dat[4] = 0xBB;
		dat[5] = 0xAA;
	    Uart7_SendPacket(dat, 6);
}

static void PUMPBehaviors(void)
{
	    uint8_t pump_type = 0;
		uint16_t pump_speed = 0;
		uint16_t uart_data = 0;
		static uint8_t pump_dir=0;
		if(PUMPBMsgQueue != NULL)
		{
			PUMPBMessage_t msg;
			// 非阻塞方式接收消息
			if(Kernel_QueueReceive(PUMPBMsgQueue, &msg, 0) == pdTRUE)
			{
				pumpMessageB.type=msg.pump_type;
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
			uart_data=pump_speed*42;
			break;
			case INJECTWATER://注
			pump_dir=0;
			if(pump_speed>70)pump_speed=70;//记得宏定义最大值70ml
			uart_data=(pump_speed*0.02+2.1)*pump_speed;//
				break;
			case POURWATER://灌
			pump_dir=0;
			if(pump_speed>300)pump_speed=300;//记得宏定义最大值300ml
			uart_data=pump_speed*0.62;
				break;
			default:
				break;
		}
		if(pumpMessageB.timingDrainage_flag)
		{
           if(pumpMessageB.timingDrainage_times++>100)//排空计时，当为注水的时候
		   {
			uart_data=0;
			pumpMessageB.timingDrainage_flag=false;
			pumpMessageB.run_flag=false;
			pumpMessageB.timingDrainage_times=0;
		   }
		}
		Pump_SetSpeedS_B(uart_data,pump_dir); 
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
