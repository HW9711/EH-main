//OneWireII.h

#ifndef __ONEWIREII_H
#define __ONEWIREII_H

#include <stdint.h>

//============================================================================
//============================================================================
//功能：  读EPROM
//参数：  tgaddr--目标地址;0~128
//        len--要读取的字节数;
//       *buffer--存放地址
//返回：0--操作成功；1--总线不可用;
//============================================================================
uint8_t OneWireII_DS2431_ReadMemory(uint8_t tgaddr, uint8_t len, uint8_t *buffer);

#endif //__ONEWIREII_H



