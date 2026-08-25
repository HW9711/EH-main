#ifndef __SSC_BEEP_H__
#define __SSC_BEEP_H__

#include <stdint.h>

#define BEEP_MSG_KEY    1   //按键响应消息
#define BEEP_MSG_ALARM  2   //报警消息
#define BEEP_MSG_ALARM_TIMED 3 //限时报警消息，到期后蜂鸣任务自动退出报警
#define BEEP_MSG_KEY_IF_IDLE 4 //空闲提示音消息，报警蜂鸣占用时静默丢弃
#define BEEP_MSG_DOUBLE_IF_IDLE 5 //报警空闲时播放响-停-响-停四相位故障提示

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
 * V1.8 蜂鸣器任务使用独立的 Ssc 前缀，避免在旧屏幕模块尚未移除前
 * 与旧 BeepControlTask_Init() 发生链接重名。旧任务下线后再统一清理调用面。
 */
void SscBeepControlTask_Init(void);

#endif /* __SSC_BEEP_H__ */
