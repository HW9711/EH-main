#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern uint8_t DisPlayData[10];

void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value);
void SscUIDisplayTask_Init(void);
void UIDP_ForceNoHandleDisplay(void); /* 最后一个手柄拔出时强制刷新无手柄关键控件，并丢弃旧 UI 队列残留。 */
void UIDP_RequestHandleDisplayReplay(void); /* 手柄拔出状态落地后，在约 60ms/120ms 补发两次 A/B 权威连接快照。 */
