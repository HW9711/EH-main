//handlescan.h

#ifndef __HANDLESCAN_H
#define __HANDLESCAN_H

#include <stdbool.h>
#include <stdint.h>

void HandlescanTaskInit(void); /* 创建并启动每 10ms 检查 A/B 手柄插拔和识别状态的任务。 */
bool Handlescan_RestoreSplitHandleEepromRuntime(uint8_t channel, uint8_t manual_tool_type); /* 切回手动模式时，从该手柄 EEPROM 恢复倍率、转速范围和每次调速的增减量。 */
void Handlescan_PrepareSplitAutoIdentify(uint8_t channel); /* 重新启用自动识别前清掉旧 RFID 结果编号和计数，保留当前刀具参数。 */
bool Handlescan_TakeVerifyAlarmCloseRequest(uint8_t channel); /* 取出并清掉关窗请求；插拔任务清空屏幕队列后须重新发送关闭 90 号报警窗的消息。 */
bool Handlescan_IsChannelPhysicallyInserted(uint8_t channel); /* 读取该通道插拔检测脚，低电平返回 true；这里只看本次电平，不保证识别已通过。 */
bool Handlescan_IsNavigationReady(uint8_t channel); /* 该通道完成识别并处于在线阶段才返回 true，供外部导航检查是否允许读写。 */
uint32_t Handlescan_GetNavigationGeneration(uint8_t channel); /* 返回拔出检查的变化计数；读取导航数据前后计数不同，就应放弃旧请求。 */

#endif  //__HANDLESCAN_H


