#ifndef __PUMP_PRESSURE_CONTROL_H
#define __PUMP_PRESSURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 压力检查总开关：0 关闭报警且忽略 6000 未就绪值；非 0 启用报警并在未就绪时输出零速。超压只弹窗和蜂鸣，不停泵或手柄。 */
#ifndef PUMP_PRESSURE_CONTROL_ENABLE
#define PUMP_PRESSURE_CONTROL_ENABLE 1U
#endif

/*
 * 旧版 UART10 压力调试开关，保留 0=关闭、1=开启的历史定义。
 * 当前代码没有读取该宏，改成 1 不会产生调试日志；正式配置保持 0。
 */
#ifndef PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE
#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U
#endif

/* 压力板未完成自标定时发送的特殊值，单位 0.1g，6000 表示“未就绪”而非真实超压。需与压力板协议一致；非排空时输出零速且不报堵塞。 */
#define PUMP_PRESSURE_CONTROL_NOT_READY_X10 6000U

/* 第 1 个报警标定流量，单位 mL/min；不超过此流量时使用 ALARM_50_G，改值会改变低流量阈值范围。 */
#define PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN 50U
/* 第 2 个报警标定流量，单位 mL/min，对应 ALARM_110_G；须大于前一点、小于后一点，改值会改变相邻区间阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN 110U
/* 第 3 个报警标定流量，单位 mL/min，对应 ALARM_140_G；须大于前一点、小于后一点，改值会改变相邻区间阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN 140U
/* 第 4 个报警标定流量，单位 mL/min，对应 ALARM_200_G；须大于前一点、小于后一点，改值会改变相邻区间阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN 200U
/* 第 5 个报警标定流量，单位 mL/min，对应 ALARM_260_G；须大于前一点、小于后一点，改值会改变相邻区间阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN 260U
/* 最高报警标定流量，单位 mL/min；须大于前一点，超过此流量仍使用 ALARM_300_G，改值会改变最高区间范围。 */
#define PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN 300U

/* 不超过第 1 标定流量时的报警阈值，单位 g，当前 100；调大后更晚报警，调小后更易报警，仅影响提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_50_G 100U
/* 第 2 标定流量的报警阈值，单位 g，当前 180；参与相邻区间计算，须不低于前一点，仅影响提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_110_G 180U
/* 第 3 标定流量的报警阈值，单位 g，当前 210；参与相邻区间计算，须不低于前一点，仅影响提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_140_G 210U
/* 第 4 标定流量的报警阈值，单位 g，当前 240；参与相邻区间计算，须不低于前一点，仅影响提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_200_G 240U
/* 第 5 标定流量的报警阈值，单位 g，当前 280；参与相邻区间计算，须不低于前一点，仅影响提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_260_G 280U
/* 最高标定流量及以上的报警阈值，单位 g，当前 300；须不低于前一点，调大后更晚提示，调小后更易提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_300_G 300U

/* 压力连续达到或超过报警线多久才提示，单位 ms；当前 1000，增大会延后提示，减小会更容易响应短时压力升高。 */
#define PUMP_PRESSURE_ALARM_CONFIRM_MS 1000U
/* 压力连续降到恢复线或以下多久才允许下次报警，单位 ms；当前 1000，增大会延长两次报警之间的恢复确认。 */
#define PUMP_PRESSURE_ALARM_RECOVER_MS 1000U
/* 恢复线比报警线低多少，单位 g，当前 20；值越大，压力需降得越低才允许再次报警，避免临界值附近反复提示。 */
#define PUMP_PRESSURE_ALARM_HYSTERESIS_G 20U
/* 新压力帧允许的最大间隔，单位 ms，当前 1000；间隔达到此值便重新计时，调大可容忍更长断帧但更易沿用旧计时。 */
#define PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS 1000U

/* 旧版压力源编号 0：自动选择；当前任务不读取这些压力源宏，不能用本值切换接线。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_AUTO 0U
/* 旧版压力源编号 1：pumpMessageA，对应 SIM_UART_1/PE4；此处只保留编号定义。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPA 1U
/* 旧版压力源编号 2：pumpMessageB，对应 SIM_UART_2/PE6；此处只保留编号定义。 */
#define PUMP_PRESSURE_CONTROL_SOURCE_PUMPB 2U

/*
 * A 泵旧版压力源配置，取上面的 SOURCE_* 编号；当前任务直接读取 pumpMessageA，不读取本宏。
 * 只改本宏不会换压力源。当前 A 压力传感器接 PE4，由模拟串口写入 pumpMessageA，与泵驱动串口无关。
 */
#ifndef PUMP_PRESSURE_CONTROL_A_SOURCE
#define PUMP_PRESSURE_CONTROL_A_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPA
#endif

/*
 * B 泵旧版压力源配置，取上面的 SOURCE_* 编号；当前任务直接读取 pumpMessageB，不读取本宏。
 * 只改本宏不会换压力源。当前 B 压力传感器接 PE6，由模拟串口写入 pumpMessageB，与泵驱动串口无关。
 */
#ifndef PUMP_PRESSURE_CONTROL_B_SOURCE
#define PUMP_PRESSURE_CONTROL_B_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPB
#endif

/* 每路泵任务独占一份报警状态，避免 A/B 压力样本和确认计时互相影响。 */
typedef struct
{
    uint32_t condition_tick_ms; /* 当前超限或恢复条件开始时间，单位 ms。 */
    uint32_t last_sample_tick_ms; /* 最近一个新压力样本时间，用于断帧后重新确认。 */
    uint16_t target_speed; /* 上次计算报警线用的流量设定；设定变化后重新开始超限或恢复计时。 */
    uint8_t last_sequence; /* 最近处理的压力帧序号；序号相同就不重复计算持续时间。 */
    uint8_t sequence_valid; /* 已接收首个有效样本时置 1，允许首帧序号为 0。 */
    uint8_t alarm_active; /* 本次超压事件已经提示，压力稳定回落前不重复提示。 */
    uint8_t timing_active; /* 1 表示正在计算超限或恢复持续时间，不控制泵和手柄启停。 */
} PumpPressureAlarmState_t;

/* 停止、排空或读数无效时清除单路报警状态，不修改输出请求。 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state);
/* 依据新压力帧及持续时间判断是否产生一次报警事件，返回 1 时只允许提示。 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint32_t weight_x10,
                                        uint16_t threshold_g,
                                        uint8_t sequence,
                                        uint32_t now_ms);
/* 压力板报告未就绪时返回零速；真实压力超限仍返回原目标速度。 */
uint16_t PumpPressureControl_ApplyReadiness(uint16_t target_speed, uint32_t weight_x10);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_PRESSURE_CONTROL_H */
