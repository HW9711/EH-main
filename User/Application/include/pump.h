//pump.h

#ifndef __PUMP_H
#define __PUMP_H

#include <stdint.h>

/*
 * PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE 用于适配当前整机线束中 A/B 泵物理位置反接的情况。
 * 1 表示逻辑 A 泵最终从原 B 泵物理 UART 口输出，逻辑 B 泵最终从原 A 泵物理 UART 口输出。
 * 这样脚踏、上位机、屏幕、手柄按键仍然操作逻辑 A/B，只有最后一层硬件出口做位置修正。
 */
#ifndef PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE
#define PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE 1U
#endif

/*
 * PUMP_INJECTWATER_SPEED_MAX 是注水泵业务速度上限。
 * 上位机调泵时会把 16 位速度直接写入 pumpMessageA/B.speed_work，泵任务在换算 UART 驱动帧前按本宏限幅。
 * 旧代码固定为 70，导致上位机设置 100 以上时最终仍按 70 输出；当前联调需要 100~300 区间继续有速度变化。
 */
#ifndef PUMP_INJECTWATER_SPEED_MAX
#define PUMP_INJECTWATER_SPEED_MAX 300U
#endif

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


