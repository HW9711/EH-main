
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
    uint8_t msgType;         //消息类型: 1=按键响应, 2=报警, 3=限时报警, 4=报警空闲时按键响应
    uint8_t keyBeepTime;     //按键蜂鸣器响应时长
    uint8_t alarmFlag;       //报警标志位
    uint16_t alarmHoldTicks;  //限时报警保持周期，单位为蜂鸣任务100ms周期
} BeepMessage_t;

//消息类型定义


//发送按键蜂鸣器消息
 void SendKeyBeepMessage(uint8_t time)
{
    if(BeepMsgQueue == NULL) return; /* 蜂鸣队列尚未初始化时不能投递按键音，直接返回避免访问空队列。 */
    BeepMessage_t msg;
    msg.msgType = BEEP_MSG_KEY;
    msg.keyBeepTime = time;
    msg.alarmFlag = 0;
    msg.alarmHoldTicks = 0U;
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0);
}

/*
 * 函数功能：向蜂鸣任务发送仅在报警空闲时播放的普通提示音。
 * 输入参数：time 为提示音保持的蜂鸣任务周期数，每周期 100ms。
 * 返回参数：无。
 */
void SendKeyBeepMessageIfIdle(uint8_t time)
{
    BeepMessage_t msg; /* 独立消息类型让蜂鸣任务决定是否播放，调用者不直接读取任务内部报警状态。 */

    /* 蜂鸣队列尚未初始化时不能投递提示音，直接返回避免访问空队列。 */
    if(BeepMsgQueue == NULL)
    {
        return;
    }

    msg.msgType = BEEP_MSG_KEY_IF_IDLE; /* 该消息不得使用会清除报警锁存的普通按键音类型。 */
    msg.keyBeepTime = time;             /* 保存需要播放的 100ms 周期数，本次通道提示传入 1。 */
    msg.alarmFlag = 0U;                 /* 空闲提示不携带报警码，保持现有报警来源不变。 */
    msg.alarmHoldTicks = 0U;            /* 空闲提示不创建限时报警倒计时。 */
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0); /* 非阻塞投递，避免外控任务等待蜂鸣队列。 */
}

/*
 * 函数功能：向蜂鸣任务发送持续报警状态，相同报警码不重复入队。
 * 输入参数：flag 为报警码，0 表示退出报警蜂鸣。
 * 返回参数：无。
 */
 void SendAlarmMessage(uint8_t flag)
{
    static uint8_t flag_bijiao=0; /* 记录上一次已投递的报警码，避免同一报警持续占满蜂鸣队列。 */
    if(flag_bijiao==flag)return; /* 当前蜂鸣状态已经对应此报警码，无需重复发送相同消息。 */
    else
    {
        flag_bijiao=flag; /* 报警码发生变化时先更新缓存，包括记住 0 号退出状态，保证下一次报警仍可入队。 */
    }
    if(BeepMsgQueue == NULL) return; /* 蜂鸣队列尚未初始化时无法更新报警状态，保持当前硬件输出不变。 */
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

    if(BeepMsgQueue == NULL) return; /* 蜂鸣队列尚未初始化时不接收限时报警，避免向空队列写入。 */

    msg.msgType = BEEP_MSG_ALARM_TIMED;                  /* 限时报警不写 WorkMessage，只控制蜂鸣任务内部报警锁存。 */
    msg.keyBeepTime = 0U;                                /* 报警模式下不使用普通按键蜂鸣时长。 */
    msg.alarmFlag = flag;                                /* 保存报警码，非 0 时进入报警翻转蜂鸣。 */
    msg.alarmHoldTicks = (uint16_t)((duration_ms + 99U) / 100U); /* 蜂鸣任务100ms执行一次，毫秒时长向上折算成任务周期。 */
    /* 非零报警即使配置时长不足 100ms，也至少保留一个任务周期，保证用户能听到提示。 */
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

/*
 * 函数功能：按 100ms 调度周期处理按键蜂鸣和报警蜂鸣，函数内部不阻塞其它业务软任务。
 * 输入参数：无。
 * 返回参数：无。
 */
static void BeepControl(void)
{
static uint8_t alarmCounter = 0;     // 每个阶段的计数器 (0-9, 共10次=100ms)
static uint8_t key_flag = 0;
static uint8_t alarm_flag = 0;
static uint16_t alarm_limited_ticks = 0U; //限时报警剩余周期，递减到0后自动关闭蜂鸣

// 从消息队列获取消息
/* 队列初始化成功后才允许读取消息，避免任务启动早于队列创建时访问空句柄。 */
if(BeepMsgQueue != NULL)
{
    BeepMessage_t msg;
    // 非阻塞方式接收消息
    /* 只有本周期实际收到新消息时才切换蜂鸣模式，队列为空时继续执行原状态。 */
    if(Kernel_QueueReceive(BeepMsgQueue, &msg, 0) == pdTRUE)
    {
        /* 普通按键音优先结束旧报警缓存，并按消息中的周期数短响。 */
        if(msg.msgType == BEEP_MSG_KEY)
        {
            // 按键响应消息：keyBeepTime不为0表示需要按键蜂鸣
            key_flag = msg.keyBeepTime;
            alarm_flag = 0;
            alarm_limited_ticks = 0U;
        }
        /* 外控通道切换提示只能在报警空闲时播放，不得清除持续报警或限时报警。 */
        else if(msg.msgType == BEEP_MSG_KEY_IF_IDLE)
        {
            /* 报警蜂鸣占用时静默忽略提示；报警结束后不补响，避免产生与操作时点不一致的声音。 */
            if(alarm_flag == 0U)
            {
                key_flag = msg.keyBeepTime; /* 仅更新普通按键音周期数，不改报警码和报警倒计时。 */
            }
        }
        /* 持续报警由外部发送 0 才结束，因此清掉内部限时倒计时。 */
        else if(msg.msgType == BEEP_MSG_ALARM)
        {
            // 报警消息：alarmFlag不为0表示有报警
            alarm_flag = msg.alarmFlag;
            alarm_limited_ticks = 0U;                     /* 普通报警保持到外部发送 0，不使用内部倒计时。 */
            key_flag = 0;
        }
        /* 限时报警使用消息自带倒计时，到期后由蜂鸣任务自行关闭。 */
        else if(msg.msgType == BEEP_MSG_ALARM_TIMED)
        {
            alarm_flag = msg.alarmFlag;                   /* 限时报警进入同一个蜂鸣翻转状态，保证声音形式一致。 */
            alarm_limited_ticks = msg.alarmHoldTicks;     /* 保存倒计时，到期后本任务自己清报警蜂鸣。 */
            key_flag = 0;
        }
    }
}

	// 按键响应模式（优先级高）
	/* 没有报警占用时才播放按键音，保证报警蜂鸣不会被普通按键提示覆盖。 */
	if(key_flag && !alarm_flag)
	{
		BEEP_ON();                                     /* 按键蜂鸣采用非阻塞计数，当前 100ms 周期打开蜂鸣器后立即返回。 */
		key_flag--;                                    /* 每个任务周期扣减一次，time=1 时保持一个 100ms 蜂鸣周期。 */
	}
	// 报警模式
	/* 报警有效时进入周期翻转蜂鸣，并按需要维护限时报警倒计时。 */
	else if(alarm_flag)
	{
		//ALARMdisplay();
		/* 相位为 0 时打开蜂鸣器，下一周期再关闭，形成报警间歇声。 */
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
		/* 只有限时报警设置了剩余周期时才递减；持续报警保持到外部清除。 */
		if(alarm_limited_ticks > 0U)
		{
			--alarm_limited_ticks;                         /* 限时报警每 100ms 扣减一次，满足运行中插入坏手柄只响 3 秒。 */
			/* 倒计时刚好结束时立即复位报警相位并关闭蜂鸣输出。 */
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
		key_flag = 0;
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
