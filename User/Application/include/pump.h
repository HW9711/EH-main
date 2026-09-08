//pump.h

#ifndef __PUMP_H
#define __PUMP_H

#include <stdint.h>

/*
 * A/B 泵驱动串口互换开关，只能取 0 或 1。
 * 0：A 泵使用 UART5，B 泵使用 UART7；1：A 泵使用 UART7，B 泵使用 UART5。
 * 修改后会交换控制帧和驱动回包的串口，不交换屏幕 A/B 名称或压力传感器接线；必须与线束一致。
 */
#ifndef PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE
#define PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE 0U
#endif

/*
 * 注水泵正常运行和手柄联动时的最大设定流量，单位 mL/min，当前允许 0~300。
 * 修改后会改变屏幕、脚踏和上位机注水请求的上限；还应核对驱动换算和实测出液量。
 * 屏幕定时排空单独使用 PUMP_TIMING_DRAINAGE_SPEED，不按本宏限制排空流量。
 */
#ifndef PUMP_INJECTWATER_SPEED_MAX
#define PUMP_INJECTWATER_SPEED_MAX 300U
#endif

/*
 * 发给步进驱动的速度整数上限，不是 mL/min，也不是实测转速；0 表示停止。
 * 当前按抽吸泵最大设定 15 × 42 得到 630，所有发送入口都会把更大值限制到此值。
 * 修改前须核对驱动端允许范围，避免主控允许的速度被驱动拒绝。
 */
#define PUMP_DRIVER_COMMAND_SPEED_MAX 630U

/* A/B 泵任务执行间隔，单位 ms，必须大于 0；改变它会同时改变命令发送间隔、报警检查间隔和排空计数。 */
#define PUMP_BEHAVIOR_TASK_PERIOD_MS 25U
/* 屏幕注水泵定时排空的时长，单位 ms；当前为 10 秒，宜取任务周期的整数倍，修改后会改变自动停泵时间。 */
#define PUMP_TIMING_DRAINAGE_DURATION_MS 10000U
/* 屏幕定时排空的固定流量，单位 mL/min；当前 100，0 会使排空不转；脚踏轻排和手柄联动仍用各自设定流量。 */
#define PUMP_TIMING_DRAINAGE_SPEED 100U
/* 排空计数上限，单位为任务执行次数；由时长/周期算得当前 400 次，不应单独修改此表达式。 */
#define PUMP_TIMING_DRAINAGE_TICKS (PUMP_TIMING_DRAINAGE_DURATION_MS / PUMP_BEHAVIOR_TASK_PERIOD_MS)

extern uint8_t pum_close_flag_B; 
extern uint8_t pum_close_flag_A; 
/*
 * 函数功能：通过旧直发接口给 B 泵发送速度命令，并更新屏幕颜色。
 * 输入参数：s 为协议速度整数，0 表示停泵，不进行流量换算。
 * 返回参数：无。
 */
void Pump_SetSpeed_B(uint32_t s);

/*
 * 函数功能：通过旧直发接口给 A 泵发送速度命令，并更新屏幕颜色。
 * 输入参数：s 为协议速度整数，0 表示停泵，不进行流量换算。
 * 返回参数：无。
 */
void Pump_SetSpeed_A(uint32_t s);

#endif //__PUMP_H


