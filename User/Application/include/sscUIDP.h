#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value);
void SscUIDisplayTask_Init(void);
