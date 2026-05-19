#ifndef __PUMP_PRESSURE_CONTROL_H
#define __PUMP_PRESSURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PUMP_PRESSURE_CONTROL_ENABLE 控制压力闭环总开关，默认 1 让所有控制来源都经过 1.5 倍阈值硬停保护。 */
#ifndef PUMP_PRESSURE_CONTROL_ENABLE
#define PUMP_PRESSURE_CONTROL_ENABLE 1U
#endif

/*
 * PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 控制压力闭环临时调试输出。
 * 旧联调时曾用 UART10 打印压力判断和 UART5 控制帧；现在现场运行必须保持 0，避免测试文本影响泵控制节拍。
 * 若以后需要重新抓闭环日志，应单独开临时分支验证，不能把该开关作为默认配置带入烧录版本。
 */
#ifndef PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE
#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U
#endif

/*
 * PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM / DEN 配置硬停倍率，默认 3/2 = 1.5 倍阈值。
 * 例如想改成 1.3 倍时，可设 NUM=13U、DEN=10U；想改成 2.0 倍时，可设 NUM=2U、DEN=1U。
 */
#ifndef PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM
#define PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM 3U
#endif

/* 分母必须非 0；如果误配置为 0，pump_pressure_control.c 会退化为 1.0 倍阈值保护，避免除 0。 */
#ifndef PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN
#define PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN 2U
#endif

/* PUMP_PRESSURE_CONTROL_SOURCE_AUTO 仅保留旧宏值兼容，当前默认配置不再使用自动回退。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_AUTO 0U
/* PUMP_PRESSURE_CONTROL_SOURCE_PUMPA 表示闭环压力数据来自 pumpMessageA，也就是 SIM_UART_2/PE6 解析结果。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPA 1U
/* PUMP_PRESSURE_CONTROL_SOURCE_PUMPB 表示闭环压力数据来自 pumpMessageB，也就是 SIM_UART_1/PE4 解析结果。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPB 2U

/*
 * PUMP_PRESSURE_CONTROL_A_SOURCE 用于记录 A 泵闭环压力源固定配置。
 * 现场线束已固定为 A 泵压力传感器接 PE6，因此 A 泵固定读取 pumpMessageA，不再自动回退。
 */
#ifndef PUMP_PRESSURE_CONTROL_A_SOURCE
#define PUMP_PRESSURE_CONTROL_A_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPA
#endif

/*
 * PUMP_PRESSURE_CONTROL_B_SOURCE 用于记录 B 泵闭环压力源固定配置。
 * 现场线束已固定为 B 泵压力传感器接 PE4，因此 B 泵固定读取 pumpMessageB，不再切换到 A 源。
 */
#ifndef PUMP_PRESSURE_CONTROL_B_SOURCE
#define PUMP_PRESSURE_CONTROL_B_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPB
#endif

/*
 * PumpPressureControl_Apply 根据压力阈值对目标泵速做闭环限速。
 * target_speed 是上层业务原本准备输出的泵速，weight_x10 是压力模块换算重量 0.1g，threshold_g 是阈值 g。
 */
uint16_t PumpPressureControl_Apply(uint16_t target_speed, uint32_t weight_x10, uint16_t threshold_g);

/*
 * PumpPressureControl_ShouldForceStop 判断压力是否已经到达硬停泵区间。
 * 返回 1 表示 WeightX10 >= ThresholdG * 硬停倍率，需要把本周期输出压到 0；返回 0 表示只做普通限速或保持原速。
 */
uint8_t PumpPressureControl_ShouldForceStop(uint32_t weight_x10, uint16_t threshold_g);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_PRESSURE_CONTROL_H */
