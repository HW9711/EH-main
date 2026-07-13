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
#define PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE 0U
#endif

/*
 * PUMP_INJECTWATER_SPEED_MAX 是注水泵业务速度上限。
 * 屏幕、脚踏和上位机设置最终都会受本宏限制；当前配置为 100。
 * 如果把屏幕固定排空速度改到 100 以上，还必须同步提高本上限，否则实际输出仍会被限制为 100。
 */
#ifndef PUMP_INJECTWATER_SPEED_MAX
#define PUMP_INJECTWATER_SPEED_MAX 100U
#endif

/* PUMP_BEHAVIOR_TASK_PERIOD_MS 表示 A/B 泵行为任务统一调度周期，排空计时按该周期换算真实毫秒。 */
#define PUMP_BEHAVIOR_TASK_PERIOD_MS 25U
/* PUMP_TIMING_DRAINAGE_DURATION_MS 表示注水泵屏幕排空总时长，10000ms 对应现场要求的 10 秒。 */
#define PUMP_TIMING_DRAINAGE_DURATION_MS 10000U
/* PUMP_TIMING_DRAINAGE_SPEED 只表示屏幕点击排空按钮后的固定速度；脚踏轻踩和手柄联动不得使用本配置。 */
#define PUMP_TIMING_DRAINAGE_SPEED 100U
/* PUMP_TIMING_DRAINAGE_TICKS 表示 10 秒排空在 25ms 泵任务中的累计次数，当前结果为 400 次。 */
#define PUMP_TIMING_DRAINAGE_TICKS (PUMP_TIMING_DRAINAGE_DURATION_MS / PUMP_BEHAVIOR_TASK_PERIOD_MS)

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

#endif //__PUMP_H


