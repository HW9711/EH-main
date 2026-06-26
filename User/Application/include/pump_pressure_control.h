#ifndef __PUMP_PRESSURE_CONTROL_H
#define __PUMP_PRESSURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PUMP_PRESSURE_CONTROL_ENABLE 控制压力闭环总开关，默认 1 让所有控制来源都经过按泵速变化的压力闭环保护。 */
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

/* PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN 表示 50 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN 50U
/* PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN 表示 110 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN 110U
/* PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN 表示 140 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN 140U
/* PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN 表示 200 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN 200U
/* PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN 表示 260 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN 260U
/* PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN 表示 300 ml/min 实测堵管阈值标定点。 */
#define PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN 300U

/* PUMP_PRESSURE_CONTROL_REDUCE_50_G 表示 50 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_50_G 80U
/* PUMP_PRESSURE_CONTROL_REDUCE_110_G 表示 110 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_110_G 1700U
/* PUMP_PRESSURE_CONTROL_REDUCE_140_G 表示 140 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_140_G 220U
/* PUMP_PRESSURE_CONTROL_REDUCE_200_G 表示 200 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_200_G 220U
/* PUMP_PRESSURE_CONTROL_REDUCE_260_G 表示 260 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_260_G 280U
/* PUMP_PRESSURE_CONTROL_REDUCE_300_G 表示 300 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_300_G 280U

/* PUMP_PRESSURE_CONTROL_STOP_50_G 表示 50 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_50_G 125U
/* PUMP_PRESSURE_CONTROL_STOP_110_G 表示 110 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_110_G 245U
/* PUMP_PRESSURE_CONTROL_STOP_140_G 表示 140 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_140_G 285U
/* PUMP_PRESSURE_CONTROL_STOP_200_G 表示 200 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_200_G 320U
/* PUMP_PRESSURE_CONTROL_STOP_260_G 表示 260 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_260_G 350U
/* PUMP_PRESSURE_CONTROL_STOP_300_G 表示 300 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_300_G 370U

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
 * PumpPressureControl_Apply 根据当前泵速对应的压力阈值对目标泵速做闭环限速。
 * target_speed 是上层业务原本准备输出的泵速，weight_x10 是压力模块换算重量 0.1g，threshold_g 仅作为压力上报有效性标志。
 */
uint16_t PumpPressureControl_Apply(uint16_t target_speed, uint32_t weight_x10, uint16_t threshold_g);

/*
 * PumpPressureControl_ShouldForceStop 判断压力是否已经到达绝对硬停泵区间。
 * 该接口没有泵速参数，因此只使用 300 ml/min 的硬停阈值做兜底锁存；随泵速变化的停泵输出由 PumpPressureControl_Apply 完成。
 */
uint8_t PumpPressureControl_ShouldForceStop(uint32_t weight_x10, uint16_t threshold_g);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_PRESSURE_CONTROL_H */
