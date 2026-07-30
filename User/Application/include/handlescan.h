//handlescan.h

#ifndef __HANDLESCAN_H
#define __HANDLESCAN_H

#include <stdbool.h>
#include <stdint.h>

void HandlescanTaskInit(void);
bool Handlescan_RestoreSplitHandleEepromRuntime(uint8_t channel, uint8_t manual_tool_type); /* 分体式手柄切回手动模式时，重新从手柄 EEPROM 装载倍率、速度边界和步进。 */
void Handlescan_PrepareSplitAutoIdentify(uint8_t channel); /* 分体式手柄重新进入自动识别前复位本通道 RFID 监测游标，不清当前业务参数。 */
bool Handlescan_TakeVerifyAlarmCloseRequest(uint8_t channel); /* 消费校验失败手柄拔出后的关窗请求，供插拔任务在 UI 队列复位后补发关闭90号图。 */

#endif  //__HANDLESCAN_H


