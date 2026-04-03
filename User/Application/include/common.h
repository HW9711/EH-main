//Common.h

#ifndef __COMMON_H
#define __COMMON_H

#include <stdint.h>

//#define TaskOver    0
//#define TaskStart   1



uint16_t Common_Crc16(uint8_t *data, uint16_t Len);

//============================================================================
// 函数名称: CopyData()
// 功能描述: 复制数据
// 输　  入: *sdat 源数据地址
//           *tdat 目标数据地址
// 输    出: 复制数据的大小
// 函数说明:
//============================================================================
uint16_t Common_CopyData(uint8_t *sdat, uint8_t *tdat, uint16_t num);

uint16_t Common_Memset(uint8_t sdat, uint8_t *tdat, uint16_t num);

//============================================================================
// 函数名称: Common_FolatToHex()
// 功能描述: 单精度(float) 转 十六进制(HEX)
// 输　  入:
// 输    出:
// 函数说明: 指针法
//============================================================================
uint32_t Common_FolatToHex(float fdata);

//============================================================================
// 函数名称: Common_CurrentVelocity()
// 功能描述: 泵流速计算
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
uint32_t Common_CurrentVelocity(float NumFluid,uint8_t is_Irrigate_FLAG);

//============================================================================
//============================================================================
uint8_t Common_CompareData(uint8_t *dat1, uint8_t *dat2, uint16_t num);



#endif //__COMMON_H


