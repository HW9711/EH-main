
#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscBEEP.h"
#include "board.h" 

static kernel_task_t  BeepHandle;

//蜂鸣器消息队列
static QueueHandle_t BeepMsgQueue = NULL;

//蜂鸣器消息类型
typedef struct {
    uint8_t msgType;         //消息类型: 1=按键响应, 2=报警
    uint8_t keyBeepTime;     //按键蜂鸣器响应时长
    uint8_t alarmFlag;       //报警标志位
} BeepMessage_t;

//消息类型定义


//发送按键蜂鸣器消息
 void SendKeyBeepMessage(uint8_t time)
{
    if(BeepMsgQueue == NULL) return;
    BeepMessage_t msg;
    msg.msgType = BEEP_MSG_KEY;
    msg.keyBeepTime = time;
    msg.alarmFlag = 0;
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0);
}

//发送报警消息
 void SendAlarmMessage(uint8_t flag)
{
    if(BeepMsgQueue == NULL) return;
    BeepMessage_t msg;
    msg.msgType = BEEP_MSG_ALARM;
    msg.keyBeepTime = 0;
    msg.alarmFlag = flag;
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0);
}

//初始化蜂鸣器消息队列
static void BeepQueue_Init(void)
{
    BeepMsgQueue = Kernel_QueueCreate(5, sizeof(BeepMessage_t), "BeepMsgQueue");
}


static void BeepControl(void)
{
static uint8_t alarmCounter = 0;     // 每个阶段的计数器 (0-9, 共10次=100ms)
static uint8_t key_flag = 0;
static uint8_t alarm_flag = 0;

// 从消息队列获取消息
if(BeepMsgQueue != NULL)
{
    BeepMessage_t msg;
    // 非阻塞方式接收消息
    if(Kernel_QueueReceive(BeepMsgQueue, &msg, 0) == pdTRUE)
    {
        if(msg.msgType == BEEP_MSG_KEY)
        {
            // 按键响应消息：keyBeepTime不为0表示需要按键蜂鸣
            key_flag = msg.keyBeepTime;
            alarm_flag = 0;
        }
        else if(msg.msgType == BEEP_MSG_ALARM)
        {
            // 报警消息：alarmFlag不为0表示有报警
            alarm_flag = msg.alarmFlag;
            key_flag = 0;
        }
    }
}

	// 按键响应模式（优先级高）
	if(key_flag && !alarm_flag)
	{
		BEEP_ON();
		// 延时100ms (beep_time = 100ms)
		vTaskDelay(pdMS_TO_TICKS(100));
		BEEP_OFF();
	key_flag=0;
	}
	// 报警模式
	else if(alarm_flag)
	{
		//ALARMdisplay();
		if(!alarmCounter)
		{
			alarmCounter=1;
			BEEP_ON();
		}
		else
		{
			alarmCounter=0;
			BEEP_OFF();
		}
	}
	// 正常状态（无按键、无报警）
	else 
	{
		BEEP_OFF();
		alarmCounter = 0;
		//ALARMdisplay();
	}
}


static void BeepControlTask(uint32_t event)
{
	(void)event;
	BeepControl();
}

void SscBeepControlTask_Init(void)
{
	//初始化蜂鸣器消息队列
	BeepQueue_Init();
	
	Kernel_TaskCreate(&BeepHandle, BeepControlTask);
	Kernel_TaskStart(&BeepHandle, KERNEL_TASK_ALWAYS, 100);
}
