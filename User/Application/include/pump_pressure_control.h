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
#define PUMP_PRESSURE_CONTROL_REDUCE_50_G 250U
/* PUMP_PRESSURE_CONTROL_REDUCE_110_G 表示 110 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_110_G 500U
/* PUMP_PRESSURE_CONTROL_REDUCE_140_G 表示 140 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_140_G 450U
/* PUMP_PRESSURE_CONTROL_REDUCE_200_G 表示 200 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_200_G 450U
/* PUMP_PRESSURE_CONTROL_REDUCE_260_G 表示 260 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_260_G 450U
/* PUMP_PRESSURE_CONTROL_REDUCE_300_G 表示 300 ml/min 开始限速的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_REDUCE_300_G 450U

/* PUMP_PRESSURE_CONTROL_STOP_50_G 表示 50 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_50_G 280U
/* PUMP_PRESSURE_CONTROL_STOP_110_G 表示 110 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_110_G 6000U
/* PUMP_PRESSURE_CONTROL_STOP_140_G 表示 140 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_140_G 500U
/* PUMP_PRESSURE_CONTROL_STOP_200_G 表示 200 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_200_G 500U
/* PUMP_PRESSURE_CONTROL_STOP_260_G 表示 260 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_260_G 500U
/* PUMP_PRESSURE_CONTROL_STOP_300_G 表示 300 ml/min 输出压到 0 的压力，单位 g。 */
#define PUMP_PRESSURE_CONTROL_STOP_300_G 500U

/* PUMP_PRESSURE_CONTROL_SOURCE_AUTO 仅保留旧宏值兼容，当前默认配置不再使用自动回退。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_AUTO 0U
/* PUMP_PRESSURE_CONTROL_SOURCE_PUMPA 表示闭环压力数据来自 pumpMessageA；当前由 SIM_UART_1/PE4 解析结果写入该结构。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPA 1U
/* PUMP_PRESSURE_CONTROL_SOURCE_PUMPB 表示闭环压力数据来自 pumpMessageB；当前由 SIM_UART_2/PE6 解析结果写入该结构。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPB 2U

/*
 * PUMP_PRESSURE_CONTROL_A_SOURCE 用于记录 A 泵闭环压力源固定配置。
 * 当前压力线束为 A 泵压力传感器接 PE4，模拟串口层写入 pumpMessageA；泵驱动串口不参与该映射。
 */
#ifndef PUMP_PRESSURE_CONTROL_A_SOURCE
#define PUMP_PRESSURE_CONTROL_A_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPA
#endif

/*
 * PUMP_PRESSURE_CONTROL_B_SOURCE 用于记录 B 泵闭环压力源固定配置。
 * 当前压力线束为 B 泵压力传感器接 PE6，模拟串口层写入 pumpMessageB；泵驱动串口不参与该映射。
 */
#ifndef PUMP_PRESSURE_CONTROL_B_SOURCE
#define PUMP_PRESSURE_CONTROL_B_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPB
#endif

/*
 * PumpPressureControl_Apply 根据压力模块阈值和当前泵速限速或停泵。
 * target_speed 是上层业务原本准备输出的泵速；weight_x10 是压力模块换算重量 0.1g；threshold_g 是压力模块报警阈值，达到 threshold_g*10 时主控必须同步停泵。
 */
uint16_t PumpPressureControl_Apply(uint16_t target_speed, uint32_t weight_x10, uint16_t threshold_g);

/*
 * PumpPressureControl_IsPressureStopReached 判断当前压力是否已经达到压力模块上报的停泵阈值。
 * target_speed 是本周期准备输出的泵速，用于过滤 0 速请求；weight_x10 是压力模块上报 0.1g 单位重量；threshold_g 是压力模块报警阈值。
 */
uint8_t PumpPressureControl_IsPressureStopReached(uint16_t target_speed, uint32_t weight_x10, uint16_t threshold_g);

/*
 * PumpPressureControl_ShouldForceStop 判断旧直连泵入口是否已经到达压力模块硬停阈值。
 * 该接口没有泵速参数，因此直接使用 threshold_g*10 硬停，防止绕过 A/B 泵任务时继续转泵。
 */
uint8_t PumpPressureControl_ShouldForceStop(uint32_t weight_x10, uint16_t threshold_g);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_PRESSURE_CONTROL_H */
