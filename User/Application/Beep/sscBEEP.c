
#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscBEEP.h"
#include "pump_behavior_core.h" /* 泵标定期间屏蔽普通提示，结果由蜂鸣任务合并消费；安全报警仍独立播放。 */
#include "board.h"
#include "delay.h"

static kernel_task_t  BeepHandle;

//蜂鸣器消息队列
static QueueHandle_t BeepMsgQueue = NULL;

//蜂鸣器消息类型
typedef struct {
    uint8_t msgType;         //消息类型: 1=按键响应, 2=报警, 3=限时报警, 4=报警空闲时按键响应, 5=报警空闲时双响
    uint8_t keyBeepTime;     //按键音持续的任务周期数，每周期 100ms；0 表示不响。
    uint8_t alarmFlag;       //报警码，0 表示停止报警蜂鸣，非 0 表示间歇蜂鸣。
    uint16_t alarmHoldTicks;  //限时报警持续的任务周期数，每周期 100ms。
} BeepMessage_t;

//消息类型定义

/*
 * 函数功能：在启动期蜂鸣任务尚未创建时，直接驱动蜂鸣器输出一次100ms按键音。
 * 输入参数：无。
 * 返回参数：无。
 */
void Beep_Pulse100ms(void)
{
    if (PumpBehavior_IsAlignmentBusy() != 0U)
    {
        return; /* 已进入泵标定时不播放直接按键音，避免绕过蜂鸣任务的静音规则。 */
    }
    BEEP_ON();       /* 启动页GPIO已经初始化，可直接打开蜂鸣器而不依赖尚未创建的消息队列。 */
    Delay_ms(100U);  /* 保持与老工程按键反馈一致的100ms响声。 */
    BEEP_OFF();      /* 阻塞提示结束后立即关闭，避免进入定标循环后蜂鸣器保持高电平。 */
}

/*
 * 函数功能：非泵标定期发送按键音请求；原普通按键行为不变，标定期间丢弃且结束后不补响。
 * 输入参数：time 为响声持续的 100ms 周期数，0 表示不响。
 * 返回参数：无。
 */
 void SendKeyBeepMessage(uint8_t time)
{
    if (PumpBehavior_IsAlignmentBusy() != 0U)
    {
        return; /* 标定中连接、切页及普通按键提示均不入队，不能在READY之后挤成多声。 */
    }
    if(BeepMsgQueue == NULL) return; /* 蜂鸣队列尚未初始化时不能投递按键音，直接返回避免访问空队列。 */
    BeepMessage_t msg;
    msg.msgType = BEEP_MSG_KEY;
    msg.keyBeepTime = time;
    msg.alarmFlag = 0;
    msg.alarmHoldTicks = 0U;
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0);
}

/*
 * 函数功能：非泵标定期向蜂鸣任务发送仅在报警空闲时播放的普通提示音。
 * 输入参数：time 为提示音保持的蜂鸣任务周期数，每周期 100ms。
 * 返回参数：无。
 */
void SendKeyBeepMessageIfIdle(uint8_t time)
{
    BeepMessage_t msg; /* 独立消息类型让蜂鸣任务决定是否播放，调用者不直接读取任务内部报警状态。 */

    if (PumpBehavior_IsAlignmentBusy() != 0U)
    {
        return; /* 标定等待及自动补试均静音，不排队保留普通操作音。 */
    }

    /* 蜂鸣队列尚未初始化时不能投递提示音，直接返回避免访问空队列。 */
    if(BeepMsgQueue == NULL)
    {
        return;
    }

    msg.msgType = BEEP_MSG_KEY_IF_IDLE; /* 此类提示只在无报警时播放，不会像普通按键音那样清除报警蜂鸣。 */
    msg.keyBeepTime = time;             /* 保存需要播放的 100ms 周期数，本次通道提示传入 1。 */
    msg.alarmFlag = 0U;                 /* 空闲提示不携带报警码，保持现有报警来源不变。 */
    msg.alarmHoldTicks = 0U;            /* 空闲提示不创建限时报警倒计时。 */
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0); /* 非阻塞投递，避免外控任务等待蜂鸣队列。 */
}

/*
 * 函数功能：蜂鸣任务没有报警占用时播放两声 100ms 故障提示音。
 * 输入参数：无。
 * 返回参数：无。
 */
void SendDoubleBeepMessageIfIdle(void)
{
    BeepMessage_t msg; /* 双响由 100ms 蜂鸣任务生成，故障解析任务不直接阻塞延时。 */

    if (BeepMsgQueue == NULL)
    {
        return; /* 蜂鸣任务尚未创建时不访问空队列。 */
    }

    msg.msgType = BEEP_MSG_DOUBLE_IF_IDLE; /* 使用双响类型，让两声之间停顿 100ms。 */
    msg.keyBeepTime = 0U; /* 双响不使用普通按键音计数字段。 */
    msg.alarmFlag = 0U; /* 本提示不创建或清除任何报警码。 */
    msg.alarmHoldTicks = 0U; /* 响、停、响、停四步由蜂鸣任务另行计数，不用报警倒计时。 */
    (void)Kernel_QueueSend(BeepMsgQueue, &msg, 0); /* 非阻塞投递，不拉长 25ms 泵控制周期。 */
}

/*
 * 函数功能：向蜂鸣任务发送持续报警状态，相同报警码不重复入队。
 * 输入参数：flag 为报警码，0 表示退出报警蜂鸣。
 * 返回参数：无。
 */
 void SendAlarmMessage(uint8_t flag)
{
    static uint8_t flag_bijiao=0; /* 记录上一次已投递的报警码，避免同一报警持续占满蜂鸣队列。 */
    if(flag_bijiao==flag)return; /* 与上次记录的报警码相同就不重复发送；这里只比较发送记录，不是硬件反馈。 */
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

    msg.msgType = BEEP_MSG_ALARM_TIMED;                  /* 只让蜂鸣器限时报警，不修改 WorkMessage 中的设备报警状态。 */
    msg.keyBeepTime = 0U;                                /* 报警模式下不使用普通按键蜂鸣时长。 */
    msg.alarmFlag = flag;                                /* 保存报警码；非 0 时按“响 100ms、停 100ms”重复提示。 */
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
 * 函数功能：每100ms处理报警、普通消息与泵标定合并结果；标定中普通声音静音，安全报警优先。
 * 输入参数：无。
 * 返回参数：无。
 */
static void BeepControl(void)
{
static uint8_t alarmCounter = 0;     // 报警响停标志：0 时本次打开蜂鸣器并改为 1；1 时本次关闭并改为 0。
static uint8_t key_flag = 0;
static uint8_t alarm_flag = 0;
static uint16_t alarm_limited_ticks = 0U; //限时报警剩余周期，递减到0后自动关闭蜂鸣
static uint8_t double_beep_phase = 0U; //双响剩余步骤：4=响、3=停、2=响、1=停，每步 100ms，0 表示结束。
static uint8_t alignment_beep_phase = 0U; /* 标定结果专用步骤：成功从2开始、失败从4开始，普通消息不得截断这一组声音。 */
uint8_t alignment_busy = PumpBehavior_IsAlignmentBusy(); /* 使用两路当前状态，任一路尚在标定时都不能提前播放结果。 */
uint8_t alignment_result; /* 本周期最多消费一组两路合并结果，不占用可能拥塞的普通蜂鸣队列。 */

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
        if((msg.msgType == BEEP_MSG_KEY) && (alignment_busy == 0U) && (alignment_beep_phase == 0U))
        {
            // 按键响应消息：keyBeepTime不为0表示需要按键蜂鸣
            key_flag = msg.keyBeepTime;
            alarm_flag = 0;
            alarm_limited_ticks = 0U;
            double_beep_phase = 0U; /* 收到普通按键音后，停止尚未播完的双响。 */
        }
        /* 外控通道切换提示只能在报警空闲时播放，不得清除持续报警或限时报警。 */
        else if(msg.msgType == BEEP_MSG_KEY_IF_IDLE)
        {
            /* 报警蜂鸣占用时静默忽略提示；报警结束后不补响，避免产生与操作时点不一致的声音。 */
            if((alarm_flag == 0U) && (alignment_busy == 0U) && (alignment_beep_phase == 0U))
            {
                key_flag = msg.keyBeepTime; /* 仅更新普通按键音周期数，不改报警码和报警倒计时。 */
                double_beep_phase = 0U; /* 改播普通提示音时停止双响，避免两种声音交替干扰。 */
            }
        }
        /* 泵驱动故障双响只在报警空闲时接管蜂鸣，不覆盖持续或限时报警。 */
        else if(msg.msgType == BEEP_MSG_DOUBLE_IF_IDLE)
        {
            if((alarm_flag == 0U) && (alignment_busy == 0U) && (alignment_beep_phase == 0U))
            {
                key_flag = 0U; /* 停止普通按键音，从头播放故障双响。 */
                alarmCounter = 0U; /* 清除旧报警的响停记录。 */
                double_beep_phase = 4U; /* 开始响、停、响、停四步，每步 100ms。 */
            }
        }
        /* 持续报警由外部发送 0 才结束，因此清掉内部限时倒计时。 */
        else if(msg.msgType == BEEP_MSG_ALARM)
        {
            // 报警消息：alarmFlag不为0表示有报警
            alarm_flag = msg.alarmFlag;
            alarm_limited_ticks = 0U;                     /* 普通报警保持到外部发送 0，不使用内部倒计时。 */
            key_flag = 0;
            double_beep_phase = 0U;                       /* 报警优先级高于双响，到达后立即结束双响。 */
            alignment_beep_phase = 0U;                    /* 真正报警立即抢占标定结果，不为凑齐提示次数延迟安全声音。 */
        }
        /* 限时报警使用消息自带倒计时，到期后由蜂鸣任务自行关闭。 */
        else if(msg.msgType == BEEP_MSG_ALARM_TIMED)
        {
            alarm_flag = msg.alarmFlag;                   /* 限时报警和持续报警都使用相同的间歇响声。 */
            alarm_limited_ticks = msg.alarmHoldTicks;     /* 保存倒计时，到期后本任务自己清报警蜂鸣。 */
            key_flag = 0;
            double_beep_phase = 0U;                       /* 限时报警同样高于双响提示。 */
            alignment_beep_phase = 0U;                    /* 限时安全报警也可打断结果提示，结束后不补播过期结果。 */
        }
    }
}

	if (alignment_busy != 0U)
	{
		key_flag = 0U; /* 已排队或已开始的普通提示在标定期也结束，不能仅靠发送入口过滤。 */
		double_beep_phase = 0U; /* 普通低优先级双响不穿插到标定过程；报警状态不在此清除。 */
		alignment_beep_phase = 0U; /* 新标定开始即停止上一组结果音，不把旧成功误当本轮成功。 */
	}
	alignment_result = PumpBehavior_TakeAlignmentBeepResult(); /* 仍忙时返回0且保留结果；两路结束后只消费一次。 */
	if ((alignment_result != 0U) && (alarm_flag == 0U))
	{
		key_flag = 0U; /* 结果音优先于普通按键，成功的一声不与旧提示拼接。 */
		double_beep_phase = 0U; /* 标定失败已合并为一组，不能再叠加普通双响。 */
		alignment_beep_phase = (alignment_result == 2U) ? 4U : 2U; /* 成功响停两步，失败响停响停四步，每步100ms。 */
	}

	/* 标定结果独立计数，播放期间普通队列消息不能截断第二声或延长第一声。 */
	if ((alignment_beep_phase > 0U) && (alarm_flag == 0U))
	{
		if ((alignment_beep_phase & 1U) == 0U)
		{
			BEEP_ON(); /* 偶数步骤播放一声100ms，成功只有步骤2，失败还有步骤4。 */
		}
		else
		{
			BEEP_OFF(); /* 奇数步骤保留100ms间隔，保证单响与双响可分辨。 */
		}
		--alignment_beep_phase; /* 本组完成后恢复原普通提示行为，重复READY/FAILED不再触发。 */
	}
	// 按键响应模式（优先级高）
	/* 按消息处理后的状态输出；普通按键消息已在上面清除报警，空闲提示消息则不会清除。 */
	else if(key_flag && !alarm_flag)
	{
		BEEP_ON();                                     /* 按键蜂鸣采用非阻塞计数，当前 100ms 周期打开蜂鸣器后立即返回。 */
		key_flag--;                                    /* 每个任务周期扣减一次，time=1 时保持一个 100ms 蜂鸣周期。 */
	}
	/* 没有报警时按响、停、响、停四步播放故障双响，每步 100ms。 */
	else if((double_beep_phase > 0U) && (alarm_flag == 0U))
	{
		if((double_beep_phase & 1U) == 0U)
		{
			BEEP_ON(); /* 剩余步骤为 4 或 2 时，分别播放第一声和第二声。 */
		}
		else
		{
			BEEP_OFF(); /* 剩余步骤为 3 或 1 时，分别停顿在两声之间和结尾。 */
		}
		--double_beep_phase; /* 每 100ms 完成一步，减到 0 后结束双响。 */
	}
	// 报警模式
	/* 有报警时每 100ms 切换一次响或停；限时报警同时扣减剩余时间。 */
	else if(alarm_flag)
	{
		//ALARMdisplay();
		/* 上次未响时本次打开，下次关闭，形成间歇报警声。 */
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
			/* 倒计时结束后立即清除响停记录并关闭蜂鸣器。 */
			if(alarm_limited_ticks == 0U)
			{
				alarm_flag = 0U;                           /* 倒计时结束后退出报警蜂鸣，不影响 WorkMessage 的真实报警状态。 */
				alarmCounter = 0U;                         /* 清除响停记录，下次报警从第一声开始。 */
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
		double_beep_phase = 0U; /* 没有声音要播放时，清除剩余双响步骤。 */
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
