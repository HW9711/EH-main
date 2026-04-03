//pump.h

#ifndef __PUMP_H
#define __PUMP_H

#include <stdint.h>

extern uint8_t pum_close_flag_B; 
extern uint8_t pum_close_flag_A; 
//============================================================================
// 函数名称: Pump_SetSpeed_B()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵转速设置
//============================================================================
void Pump_SetSpeed_B(uint32_t s);

//============================================================================
// 函数名称: Pump_SetSpeed_A()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 泵转速设置
//============================================================================
void Pump_SetSpeed_A(uint32_t s);

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None
 */
//============================================================================
void Pump_RunTask_Init(void);

//============================================================================
void Pump_Pedal2Pump5sTask_Init(void);

#endif //__PUMP_H


