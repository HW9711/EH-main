//handlescan.h

#ifndef __HANDLESCAN_H
#define __HANDLESCAN_H

#include <stdbool.h>
#include <stdint.h>

void HandlescanTaskInit(void);
bool Handlescan_RestoreSplitHandleEepromRuntime(uint8_t channel, uint8_t manual_tool_type); /* 分体式手柄切回手动模式时，重新从手柄 EEPROM 装载倍率、速度边界和步进。 */

#endif  //__HANDLESCAN_H


