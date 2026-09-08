#include "stm32f4xx_hal.h"
#include <string.h>
/*
 * 函数功能：创建 B 泵队列并启动 25ms 周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscPumpBTask_Init(void);
/*
 * 函数功能：把 B 泵新设定放入消息队列，重复的已入队消息不再发送。
 * 输入参数：pump_type 为泵类型；value 为新设定，注水/灌注单位 mL/min，抽吸使用内部设定值。
 * 返回参数：无；队列满时本次不保存，调用方可再次发送。
 */
void SendPumpBMessage(uint8_t pump_type,uint16_t value);
