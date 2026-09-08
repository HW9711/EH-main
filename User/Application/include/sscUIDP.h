#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern uint8_t DisPlayData[10];

void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value);
void SscUIDisplayTask_Init(void);
void UIDP_ForceNoHandleDisplay(void); /* 最后一个手柄拔出后，清掉旧显示消息并立即显示“无手柄”。 */
void UIDP_RequestHandleDisplayReplay(void); /* 拔出后再补刷两次当前 A/B 连接状态；间隔至少约 60ms，队列忙时延后。 */
