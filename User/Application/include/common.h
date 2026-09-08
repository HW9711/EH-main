//Common.h

#ifndef __COMMON_H
#define __COMMON_H

#include <stdint.h>

//#define TaskOver    0
//#define TaskStart   1



uint16_t Common_Crc16(uint8_t *data, uint16_t Len); /* 计算 Len 字节的 CRC16/MODBUS 值，不在这里决定发送字节顺序。 */

//============================================================================
// 函数名称: CopyData()
// 功能描述: 复制数据
// 输　  入: *sdat 源数据地址
//           *tdat 目标数据地址
// 输    出: 复制数据的大小
// 函数说明:
//============================================================================
uint16_t Common_CopyData(uint8_t *sdat, uint8_t *tdat, uint16_t num);

uint16_t Common_Memset(uint8_t sdat, uint8_t *tdat, uint16_t num); /* 将目标 num 个字节填成 sdat，返回 num。 */

//============================================================================
// 函数名称: Common_FolatToHex()
// 功能描述: 先将浮点数加 0.01，再返回其 32 位内存表示，不是整数取整。
// 输　  入: fdata：单精度浮点数。
// 输    出: 调整后浮点数的原始位数据。
// 函数说明: 按当前小端存储方式读取四个字节。
//============================================================================
uint32_t Common_FolatToHex(float fdata);

//============================================================================
// 函数名称: Common_CurrentVelocity()
// 功能描述: 旧版泵换算，当前工程无调用；实际换算见 pump_behavior_core.c。
// 输　  入: NumFluid：旧流量设定值；is_Irrigate_FLAG：非零为灌注，零为注水。
// 输    出: 旧公式算出的整数驱动值，不是实测流量。
// 函数说明: 保留旧接口，修改这里不会改变当前 A/B 泵速度。
//============================================================================
uint32_t Common_CurrentVelocity(float NumFluid,uint8_t is_Irrigate_FLAG);

//============================================================================
//============================================================================
uint8_t Common_CompareData(uint8_t *dat1, uint8_t *dat2, uint16_t num);



#endif //__COMMON_H


