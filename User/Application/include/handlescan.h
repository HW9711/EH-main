//handlescan.h

#ifndef __HANDLESCAN_H
#define __HANDLESCAN_H

#include <stdbool.h>
#include <stdint.h>

/* 产品功能选择：只接受下列两类手柄，未列出的型号在任何模式下均拒绝上线。 */
#define HANDLE_FUNCTION_BOTH          1U /* 同时支持磨削和刨削。 */
#define HANDLE_FUNCTION_GRINDING_ONLY 2U /* 仅支持 TMBB、TMBA、EMBA、EMBB、公共接头和 DHYTM。 */
#define HANDLE_FUNCTION_SHAVING_ONLY  3U /* 仅支持 PXBA、PXBB、PXYTP 和 PXYTM。 */
#ifndef HANDLE_FUNCTION_MODE /* 允许通过编译选项指定产品功能，未指定时使用下方选择的模式。 */
#define HANDLE_FUNCTION_MODE HANDLE_FUNCTION_BOTH /* 当前主机同时支持磨削和刨削；修改模式后须重新编译。 */
#endif
#if (HANDLE_FUNCTION_MODE != HANDLE_FUNCTION_BOTH) && \
    (HANDLE_FUNCTION_MODE != HANDLE_FUNCTION_GRINDING_ONLY) && \
    (HANDLE_FUNCTION_MODE != HANDLE_FUNCTION_SHAVING_ONLY)
#error "HANDLE_FUNCTION_MODE must be 1, 2 or 3"
#endif

bool Handlescan_IsStartupPending(void); /* 已插入手柄尚在识别或参数装载时为true，明确失败或空接口不阻塞切页。 */

uint8_t Handlescan_GetVerifyAlarmForDisplay(void); /* 返回当前有效的A/B/AB校验提示；已恢复或其它持续故障占用时返回0，供显示任务可靠收尾。 */

bool Handlescan_HasUnsupportedHandle(void); /* 任一通道拒绝的手柄尚未确认拔出时为 true，按键或清报警命令不能解除。 */

void HandlescanTaskInit(void); /* 创建并启动每 10ms 检查 A/B 手柄插拔和识别状态的任务。 */
bool Handlescan_IsRfidToolReady(uint8_t channel); /* 查询当前有效RFID是否已完整装载，供外控数据有效性判断，不作为启动门禁。 */
bool Handlescan_RestoreSplitHandleEepromRuntime(uint8_t channel, uint8_t manual_tool_type); /* 切回手动模式时，从该手柄 EEPROM 恢复倍率、转速范围和每次调速的增减量。 */
void Handlescan_PrepareSplitAutoIdentify(uint8_t channel); /* 重新启用自动识别前清掉旧 RFID 结果编号和计数，保留当前刀具参数。 */
bool Handlescan_TakeVerifyAlarmCloseRequest(uint8_t channel); /* 取出并清掉关窗请求；插拔任务清空屏幕队列后须重新发送关闭 90 号报警窗的消息。 */
bool Handlescan_IsChannelPhysicallyInserted(uint8_t channel); /* 读取该通道插拔检测脚，低电平返回 true；这里只看本次电平，不保证识别已通过。 */
bool Handlescan_IsNavigationReady(uint8_t channel); /* 该通道完成识别并处于在线阶段才返回 true，供外部导航检查是否允许读写。 */
uint32_t Handlescan_GetNavigationGeneration(uint8_t channel); /* 返回拔出检查的变化计数；读取导航数据前后计数不同，就应放弃旧请求。 */

#endif  //__HANDLESCAN_H


