#ifndef HANDLE_CONTROL_H
#define HANDLE_CONTROL_H

#include <stdint.h>

void ToolPosActive(uint8_t key_value); /* 处理当前刨刀开口点动，每次转 1 度；运行中、报警中或不支持定位时不动作。 */
void AutoIdentifyActive(uint8_t key_value); /* 切换当前分体手柄的 RFID 自动识别状态。 */
void PlanerGridH(uint8_t key_value); /* 切换当前分体手柄的手动磨头/刨刀类型。 */
void PlugORunPLUGActive(uint8_t key_value); /* 处理 A/B 手柄插入、拔出及当前通道切换。 */
uint32_t Handle_GetIdentityGeneration(uint8_t channel); /* 读取该通道的插拔变化计数；与任务开始时不同时，必须停止读取旧手柄的数据。 */

#endif
