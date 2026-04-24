#ifndef __SSC_BEEP_H__
#define __SSC_BEEP_H__

#include <stdint.h>

#define BEEP_MSG_KEY    1   //按键响应消息
#define BEEP_MSG_ALARM  2   //报警消息

void SendKeyBeepMessage(uint8_t time);
void SendAlarmMessage(uint8_t flag);

/*
 * V1.8 蜂鸣器任务使用独立的 Ssc 前缀，避免在旧屏幕模块尚未移除前
 * 与旧 BeepControlTask_Init() 发生链接重名。旧任务下线后再统一清理调用面。
 */
void SscBeepControlTask_Init(void);

#endif /* __SSC_BEEP_H__ */
