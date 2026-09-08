#ifndef __SSC_BEEP_H__
#define __SSC_BEEP_H__

#include <stdint.h>

/* 以下数值是蜂鸣队列消息编号，不是时长或功能开关；发送方和蜂鸣任务必须使用同一编号。 */
#define BEEP_MSG_KEY    1   //播放普通按键音，并结束已有报警蜂鸣和双响。
#define BEEP_MSG_ALARM  2   //持续间歇报警；报警码为 0 时停止。
#define BEEP_MSG_ALARM_TIMED 3 //限时的间歇报警，到期自动停止蜂鸣，不清除设备报警状态。
#define BEEP_MSG_KEY_IF_IDLE 4 //无报警时才播放普通提示音，有报警就丢弃，不在事后补响。
#define BEEP_MSG_DOUBLE_IF_IDLE 5 //无报警时播放响、停、响、停四步，每步 100ms。

void SendKeyBeepMessage(uint8_t time);
/*
 * 函数功能：在蜂鸣任务和消息队列尚未创建时，直接输出一次100ms按键提示音。
 * 输入参数：无。
 * 返回参数：无。
 */
void Beep_Pulse100ms(void);
/*
 * 函数功能：仅在蜂鸣任务没有报警占用时发送普通提示音。
 * 输入参数：time 为提示音保持的蜂鸣任务周期数，每周期 100ms。
 * 返回参数：无。
 */
void SendKeyBeepMessageIfIdle(uint8_t time);
/*
 * 函数功能：蜂鸣任务没有报警占用时播放两声 100ms 故障提示音。
 * 输入参数：无。
 * 返回参数：无。
 */
void SendDoubleBeepMessageIfIdle(void);
void SendAlarmMessage(uint8_t flag);
void SendAlarmMessageTimed(uint8_t flag, uint16_t duration_ms);

/*
 * 函数功能：创建蜂鸣消息队列，并启动每 100ms 执行一次的蜂鸣任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscBeepControlTask_Init(void);

#endif /* __SSC_BEEP_H__ */
