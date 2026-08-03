#ifndef HANDLE_CONTROL_H
#define HANDLE_CONTROL_H

#include <stdint.h>

void ToolPosActive(uint8_t key_value); /* 处理当前通道刨刀开口定位。 */
void AutoIdentifyActive(uint8_t key_value); /* 切换当前分体手柄的 RFID 自动识别状态。 */
void PlanerGridH(uint8_t key_value); /* 切换当前分体手柄的手动磨头/刨刀类型。 */
void PlugORunPLUGActive(uint8_t key_value); /* 处理 A/B 手柄插入、拔出及当前通道切换。 */
uint32_t Handle_GetIdentityGeneration(uint8_t channel); /* 读取指定通道的手柄身份代数，供跨周期任务判断插拔或换柄。 */

#endif
