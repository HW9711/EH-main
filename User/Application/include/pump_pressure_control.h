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

/* 第1个压力报警参考流量，单位mL/min；流量不超过50时使用ALARM_50_G，改大后更多低流量档使用同一报警阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN 50U
/* 第2个压力报警参考流量，单位mL/min，对应ALARM_110_G；中间流量的报警阈值按相邻两点计算，数值须按从小到大排列。 */
#define PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN 110U
/* 第3个压力报警参考流量，单位mL/min，对应ALARM_140_G；改值会改变110到200mL/min之间的报警阈值计算。 */
#define PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN 140U
/* 第4个压力报警参考流量，单位mL/min，对应ALARM_200_G；改值会改变140到260mL/min之间的报警阈值计算。 */
#define PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN 200U
/* 第5个压力报警参考流量，单位mL/min，对应ALARM_260_G；改值会改变200到300mL/min之间的报警阈值计算。 */
#define PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN 260U
/* 最高压力报警参考流量，单位mL/min；达到300及以上时使用ALARM_300_G，改大后要更高流量才使用该固定阈值。 */
#define PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN 300U

/* 流量不超过50mL/min时的压力报警阈值，当前130g；调大后需要更高压力才提示，不改变泵或手柄启停。 */
#define PUMP_PRESSURE_CONTROL_ALARM_50_G 130U
/* 流量110mL/min时的压力报警阈值，当前210g；中间流量按相邻两点计算，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_110_G 210U
/* 流量140mL/min时的压力报警阈值，当前240g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_140_G 240U
/* 流量200mL/min时的压力报警阈值，当前270g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_200_G 270U
/* 流量260mL/min时的压力报警阈值，当前310g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_260_G 310U
/* 流量达到300mL/min及以上时的压力报警阈值，当前360g；调大后更晚提示，调小后更易提示，不改变启停。 */
#define PUMP_PRESSURE_CONTROL_ALARM_300_G 360U

/* 压力曲线编号只用于选择阈值，不改变首次确认、恢复回差或重复报警算法。 */
#define PUMP_PRESSURE_PROFILE_LEGACY        0U /* 灌注、抽吸、未覆盖手柄及未测流量沿用原曲线。 */
#define PUMP_PRESSURE_PROFILE_INJECT_EM     1U /* EMBA/EMBB/EMBC/EMBD 注水口相同，共用 EMBB 实测表。 */
#define PUMP_PRESSURE_PROFILE_INJECT_PX     2U /* PXBA/PXBB 共用 PXBB 实测表，仍直接比较原压力帧。 */
#define PUMP_PRESSURE_PROFILE_INJECT_TM     3U /* TMBA/TMBB 注水口相同，共用 TMBB 实测表。 */
#define PUMP_PRESSURE_PROFILE_INJECT_PXY    4U /* PXYTP/PXYTM 注水口相同，共用 PXYTM 实测表。 */
#define PUMP_PRESSURE_PROFILE_INJECT_COMMON 5U /* DHYTM 与已装刀具的公共接头共用公共接头实测表。 */

/*
 * 注水细分表仅覆盖 20~70mL/min，按相邻实测流量点线性插值；范围外直接沿用上方原曲线。
 * 下列初始阈值依据 2026-09-12 单样件堵管前正常压力及堵管记录选取，仍需实机复核。
 * 每项单位为 g 换算读数：调大后更晚提示，调小后更易提示；不修改泵流量或手柄转速。
 */
#define PUMP_PRESSURE_INJECT_POINT_COUNT 4U /* 每个手柄组有四个流量点；新增点时须同步全部组的表项。 */
#define PUMP_PRESSURE_INJECT_FLOW_20    20U /* 细分曲线下限，单位mL/min；低于本点沿用原曲线，不向下外推。 */
#define PUMP_PRESSURE_INJECT_FLOW_30    30U /* 第二个实测流量，单位mL/min；20到30之间按两点阈值插值。 */
#define PUMP_PRESSURE_INJECT_FLOW_50    50U /* 第三个实测流量，单位mL/min；30到50之间按两点阈值插值。 */
#define PUMP_PRESSURE_INJECT_FLOW_70    70U /* 细分曲线上限，单位mL/min；大于本点沿用原曲线，不向上外推。 */

#define PUMP_PRESSURE_INJECT_EM_20_G    100U /* EM组20mL/min报警线；正常样本最大1.6g，调大将延后低流量堵管提示。 */
#define PUMP_PRESSURE_INJECT_EM_30_G    120U /* EM组30mL/min报警线；正常样本最大3.3g，调大将延后此档堵管提示。 */
#define PUMP_PRESSURE_INJECT_EM_50_G    180U /* EM组50mL/min报警线；正常样本最大46.1g，调小会缩短与正常压力的间隔。 */
#define PUMP_PRESSURE_INJECT_EM_70_G    250U /* EM组70mL/min报警线；正常样本最大146.3g，调大需更高压力才提示。 */
#define PUMP_PRESSURE_INJECT_PX_20_G    180U /* PX组20mL/min报警线；正常样本最大129.5g，不再使用原130g临界值。 */
#define PUMP_PRESSURE_INJECT_PX_30_G    350U /* PX组30mL/min报警线；堵管前最大253.9g，恢复阶段仍可能因脉动重报。 */
#define PUMP_PRESSURE_INJECT_PX_50_G    350U /* PX组50mL/min报警线；正常脉动可高于此值，依靠原1秒确认排除短峰。 */
#define PUMP_PRESSURE_INJECT_PX_70_G    350U /* PX组70mL/min报警线；原20g回差可能使正常脉动下恢复不及时，未改变算法。 */
#define PUMP_PRESSURE_INJECT_TM_20_G    100U /* TM组20mL/min报警线；正常样本最大2.3g，调大将延后低流量堵管提示。 */
#define PUMP_PRESSURE_INJECT_TM_30_G    120U /* TM组30mL/min报警线；正常样本最大1.0g，调大将延后此档堵管提示。 */
#define PUMP_PRESSURE_INJECT_TM_50_G    170U /* TM组50mL/min报警线；正常样本最大33.6g，调小会更接近正常压力。 */
#define PUMP_PRESSURE_INJECT_TM_70_G    240U /* TM组70mL/min报警线；正常样本最大120.3g，调大需更高压力才提示。 */
#define PUMP_PRESSURE_INJECT_PXY_20_G   120U /* PXY组20mL/min报警线；正常样本最大3.5g，首次关口试运行不作为正常基线。 */
#define PUMP_PRESSURE_INJECT_PXY_30_G   250U /* PXY组30mL/min报警线；正常样本最大1.8g，调大将延后此档堵管提示。 */
#define PUMP_PRESSURE_INJECT_PXY_50_G   380U /* PXY组50mL/min报警线；正常样本最大1.5g，调小将更早响应升压。 */
#define PUMP_PRESSURE_INJECT_PXY_70_G   390U /* PXY组70mL/min报警线；正常读数仍接近零，不套用PX分体手柄高压线。 */
#define PUMP_PRESSURE_INJECT_COMMON_20_G 100U /* 公共接头/DHYTM组20mL/min报警线；正常样本最大1.3g，调大将延后提示。 */
#define PUMP_PRESSURE_INJECT_COMMON_30_G 120U /* 公共接头/DHYTM组30mL/min报警线；正常样本最大0.6g，调大将延后提示。 */
#define PUMP_PRESSURE_INJECT_COMMON_50_G 190U /* 公共接头/DHYTM组50mL/min报警线；正常样本最大64.6g，调小会更接近正常压力。 */
#define PUMP_PRESSURE_INJECT_COMMON_70_G 270U /* 公共接头/DHYTM组70mL/min报警线；正常样本最大193.3g，避免套用原157g。 */

/* 压力连续达到或超过报警线多久才提示，单位 ms；当前 1000，增大会延后提示，减小会更容易响应短时压力升高。 */
#define PUMP_PRESSURE_ALARM_CONFIRM_MS 1000U
/* 从上次报警触发起等待多久再检查重复提示，单位ms；当前10秒，调大后持续超压时的提示间隔更长。 */
#define PUMP_PRESSURE_ALARM_REPEAT_MS 10000U
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
    uint32_t last_alarm_tick_ms; /* 最近一次触发压力提示的毫秒时刻；持续超压时据此间隔10秒重报，重置报警状态时清零。 */
    uint16_t target_speed; /* 上次计算报警线用的流量设定；设定变化后重新开始超限或恢复计时。 */
    uint16_t alarm_threshold_g; /* 上次实际使用的g单位报警线；切换手柄导致阈值变化时重新确认，不继承旧阈值计时。 */
    uint8_t last_sequence; /* 最近处理的压力帧序号；序号相同就不重复计算持续时间。 */
    uint8_t sequence_valid; /* 已接收首个有效样本时置 1，允许首帧序号为 0。 */
    uint8_t alarm_active; /* 首次确认超压后置1，之后按10秒间隔提示；压力稳定回落或状态重置时清零。 */
    uint8_t timing_active; /* 1 表示正在计算超限或恢复持续时间，不控制泵和手柄启停。 */
} PumpPressureAlarmState_t;

/* 停止、排空或读数无效时清除单路报警状态，不修改输出请求。 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state);
/* 依据新压力帧及持续时间判断是否产生一次报警事件，返回 1 时只允许提示。 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint8_t pressure_profile, /* 注水手柄曲线编号；LEGACY或20~70范围外使用原阈值。 */
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
