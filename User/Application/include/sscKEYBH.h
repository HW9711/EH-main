#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
/*
 * 函数功能：把按键或插拔消息放入队列，稍后由按键任务处理。
 * 输入参数：control_type 为消息来源；control_key 为该来源的按键或事件编号。
 * 返回参数：true 表示放入成功；false 表示队列未创建或队列已满。
 */
bool SendKeyBehMessage(uint8_t control_type,uint8_t control_key);
void SscKeyBehaviorTask_Init(void);
