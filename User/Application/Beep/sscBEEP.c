
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
    uint16_t alarmHoldTicks;  //限时报警保持周期，单位为蜂鸣任务100ms周期
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
    msg.alarmHoldTicks = 0U;
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
    msg.alarmHoldTicks = 0U;
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0);
}

/*
 * 函数功能：发送限时报警蜂鸣消息，主要用于运行中另一路手柄校验失败的 3 秒提示。
 * 输入参数：flag 报警码；duration_ms 蜂鸣保持时间，单位毫秒。
 * 返回参数：无。
 */
 void SendAlarmMessageTimed(uint8_t flag, uint16_t duration_ms)
{
    BeepMessage_t msg;

    if(BeepMsgQueue == NULL) return;

    msg.msgType = BEEP_MSG_ALARM_TIMED;                  /* 限时报警不写 WorkMessage，只控制蜂鸣任务内部报警锁存。 */
    msg.keyBeepTime = 0U;                                /* 报警模式下不使用普通按键蜂鸣时长。 */
    msg.alarmFlag = flag;                                /* 保存报警码，非 0 时进入报警翻转蜂鸣。 */
    msg.alarmHoldTicks = (uint16_t)((duration_ms + 99U) / 100U); /* 蜂鸣任务100ms执行一次，毫秒时长向上折算成任务周期。 */
    if ((flag != 0U) && (msg.alarmHoldTicks == 0U))
    {
        msg.alarmHoldTicks = 1U;                         /* 非 0 报警至少保持一个任务周期，避免短时长被折算成 0。 */
    }

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
static uint16_t alarm_limited_ticks = 0U; //限时报警剩余周期，递减到0后自动关闭蜂鸣

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
            alarm_limited_ticks = 0U;
        }
        else if(msg.msgType == BEEP_MSG_ALARM)
        {
            // 报警消息：alarmFlag不为0表示有报警
            alarm_flag = msg.alarmFlag;
            alarm_limited_ticks = 0U;                     /* 普通报警保持到外部发送 0，不使用内部倒计时。 */
            key_flag = 0;
        }
        else if(msg.msgType == BEEP_MSG_ALARM_TIMED)
        {
            alarm_flag = msg.alarmFlag;                   /* 限时报警进入同一个蜂鸣翻转状态，保证声音形式一致。 */
            alarm_limited_ticks = msg.alarmHoldTicks;     /* 保存倒计时，到期后本任务自己清报警蜂鸣。 */
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
		if(alarm_limited_ticks > 0U)
		{
			--alarm_limited_ticks;                         /* 限时报警每 100ms 扣减一次，满足运行中插入坏手柄只响 3 秒。 */
			if(alarm_limited_ticks == 0U)
			{
				alarm_flag = 0U;                           /* 倒计时结束后退出报警蜂鸣，不影响 WorkMessage 的真实报警状态。 */
				alarmCounter = 0U;                         /* 清翻转相位，下一次报警从响开始。 */
				BEEP_OFF();                                /* 到期立即关闭蜂鸣，避免最后一个周期残留为高电平。 */
			}
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
