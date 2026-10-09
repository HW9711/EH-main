#ifndef __PUMP_PRESSURE_CONTROL_H
#define __PUMP_PRESSURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 压力控制总开关：0 关闭压力减速、停泵和提示；非 0 按泵类型执行压力处理。 */
#ifndef PUMP_PRESSURE_CONTROL_ENABLE
#define PUMP_PRESSURE_CONTROL_ENABLE 1U
#endif

/* 灌注泵压力保护开关：0 暂时屏蔽 A/B 灌注泵的压力报警、减速和停机；1 在总开关开启时恢复原保护。 */
#ifndef PUMP_PRESSURE_POUR_PROTECTION_ENABLE
#define PUMP_PRESSURE_POUR_PROTECTION_ENABLE 0U
#endif

/*
 * 旧版 UART10 压力调试开关，保留 0=关闭、1=开启的历史定义。
 * 当前代码没有读取该宏，改成 1 不会产生调试日志；正式配置保持 0。
 */
#ifndef PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE
#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U
#endif

/* 压力板未完成自标定时发送的特殊值，单位 0.1g；三种泵都不因该值归零，也不拿它累计超压或恢复时间。 */
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

/* 灌注流量不超过50mL/min时的原压力阈值，当前130g；调大后减速起点和停转确认线都随之升高。 */
#define PUMP_PRESSURE_CONTROL_ALARM_50_G 130U
/* 流量110mL/min时的压力报警阈值，当前210g；中间流量按相邻两点计算，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_110_G 210U
/* 流量140mL/min时的压力报警阈值，当前240g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_140_G 240U
/* 流量200mL/min时的压力报警阈值，当前270g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_200_G 270U
/* 流量260mL/min时的压力报警阈值，当前310g；须不低于前一档，调大后附近流量更晚提示。 */
#define PUMP_PRESSURE_CONTROL_ALARM_260_G 310U
/* 灌注流量达到300mL/min及以上时的原压力阈值，当前360g；调大后减速和停转都更晚发生。 */
#define PUMP_PRESSURE_CONTROL_ALARM_300_G 360U

/* 灌注平均压力高于原标定阈值的 70% 时每周期降低一档；取值 1~99，调小会更早降速。 */
#define PUMP_PRESSURE_POUR_SLOWDOWN_PERCENT 70U
/* 灌注平均压力低于原阈值的 65% 并稳定满恢复确认时间后，直接恢复设定流量；调小会更难恢复。 */
#define PUMP_PRESSURE_POUR_RECOVER_PERCENT 65U
#if ((PUMP_PRESSURE_POUR_SLOWDOWN_PERCENT >= 100U) || \
     (PUMP_PRESSURE_POUR_RECOVER_PERCENT == 0U) || \
     (PUMP_PRESSURE_POUR_RECOVER_PERCENT >= PUMP_PRESSURE_POUR_SLOWDOWN_PERCENT))
#error PUMP_PRESSURE_POUR_percentages_must_satisfy_0_recover_slowdown_100
#endif
/* 灌注逐档降速时的最低非零流量，单位 mL/min；当前 4 是最低实测档，用户设定更低时不抬高。 */
#define PUMP_PRESSURE_POUR_MIN_FLOW_ML_MIN 4U
/* 灌注统计平均压力并调一档流量的周期，单位 ms；当前 500，调大后每档等待更久、每档流量变化也更大。 */
#define PUMP_PRESSURE_POUR_ADJUST_MS 500U
/* 平均压力持续偏高时从设定流量降到最低档的参考时长，单位 ms；当前 6000，共十二档，调大后每档减得更少。 */
#define PUMP_PRESSURE_POUR_RAMP_DOWN_MS 6000U
#if ((PUMP_PRESSURE_POUR_ADJUST_MS == 0U) || \
     (PUMP_PRESSURE_POUR_RAMP_DOWN_MS < PUMP_PRESSURE_POUR_ADJUST_MS))
#error PUMP_PRESSURE_POUR_ramp_time_must_cover_one_nonzero_adjust_period
#endif
/* 灌注平均压力连续低于恢复线的确认时长，单位 ms；当前 2000，确认完成后直接恢复设定流量，调大将延后恢复。 */
#define PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS 2000U
/* 灌注原始新压力帧连续高于标定阈值 2000ms 后下发零速；每帧独立检查，不等待500ms平均值。 */
#define PUMP_PRESSURE_POUR_STOP_CONFIRM_MS 2000U
/* 注水泵统一的压力提示阈值，单位 g；不再依据手柄型号或流量选择报警线，调大将延后提示。 */
#define PUMP_PRESSURE_INJECT_ALARM_G 800U

/* 注水泵高于 800g、抽吸泵达到原阈值后连续多久才提示，单位 ms；当前 1000，调大将延后提示。 */
#define PUMP_PRESSURE_ALARM_CONFIRM_MS 1000U
/* 注水泵和抽吸泵持续超压时的重复提示间隔，单位 ms；灌注泵停转期间不重复提示。 */
#define PUMP_PRESSURE_ALARM_REPEAT_MS 10000U
/* 注水/抽吸解除压力提示记录的低压确认时长，单位 ms；当前 1000，调大后更晚允许下一次独立超压提示。 */
#define PUMP_PRESSURE_ALARM_RECOVER_MS 1000U
/* 注水泵和抽吸泵恢复线比提示线低多少，单位 g；灌注泵使用上面配置的65%平均压力恢复线。 */
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
    uint32_t condition_tick_ms; /* 注水/抽吸超限或恢复条件的开始时间；灌注只用于原始压力连续超限计时，单位 ms。 */
    uint32_t last_sample_tick_ms; /* 最近一个新压力样本时间，用于断帧后重新确认。 */
    uint32_t last_alarm_tick_ms; /* 最近一次提示时刻；注水/抽吸据此每10秒重报，灌注只记录首次停转时间。 */
    uint16_t target_speed; /* 上次处理的设定值；灌注和抽吸调速后重新确认，注水固定 800g 时不因调速中断计时。 */
    uint16_t alarm_threshold_g; /* 上次使用的压力阈值，单位 g；灌注随原设定流量变化，注水固定使用 800g。 */
    uint16_t last_output_speed; /* 灌注泵最近一次计算的下发流量；短时无新压力帧时保持该流量，不因旧帧恢复转动。 */
    uint64_t pour_pressure_sum; /* 当前500ms窗口内有效新压力帧之和，单位0.1g；窗口重开时清零或用当前帧重新开始，重复帧不加入。 */
    uint32_t pour_window_start_ms; /* 当前压力窗口首帧时刻，单位ms；设定改变、无效帧、长时间断帧和超压停泵后重新建立窗口。 */
    uint32_t pour_sample_count; /* 当前窗口参与平均的有效新帧数；0表示等待首帧，重复帧不增加，窗口结束后以当前帧开始下一窗。 */
    uint32_t pour_low_duration_ms; /* 连续平均压力低于65%的窗口时长，单位ms，最多保存恢复确认时长；进入保持/减速区或重开统计时清零。 */
    uint32_t pour_step_remainder; /* 降速步进除法的余数；保持区继续保存，恢复设定流量或重开统计时清零，避免低流量取整后始终不变化。 */
    uint8_t pour_adjust_direction; /* 0为未减速或已恢复设定流量，1为已按压力减速；保持区和重新统计保留原值，低压确认完成、停止请求或切换泵用途时清零。 */
    uint8_t last_sequence; /* 最近处理的压力帧序号；序号相同就不重复计算持续时间。 */
    uint8_t sequence_valid; /* 已接收首个有效样本时置 1，允许首帧序号为 0。 */
    uint8_t alarm_active; /* 注水/抽吸已提示或灌注已因超压停转时置1；恢复确认或运行请求结束时清零。 */
    uint8_t timing_active; /* 1 表示正在计算超限或恢复持续时间；条件中断时清零。 */
    uint8_t mode; /* 0 未使用，1 抽吸提示，2 注水提示，3 灌注减速及停转；类型改变时清旧状态。 */
} PumpPressureAlarmState_t;

/* 停止或泵类型切换时清除单路压力记录，不修改开泵请求。 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state);
/* 注水固定按 800g、抽吸沿用原曲线，依据新压力帧判断是否产生提示事件。 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint8_t is_injection, /* 1 为注水泵，0 为抽吸泵。 */
                                        uint32_t weight_x10,
                                        uint16_t threshold_g,
                                        uint8_t sequence,
                                        uint32_t now_ms);
/* 灌注每500ms按平均压力逐档减速或保持；原始压力连续超限2秒返回零速，平均低压连续正常2秒后直接恢复设定流量。 */
uint16_t PumpPressureControl_ApplyPour(PumpPressureAlarmState_t *state,
                                       uint16_t target_speed,
                                       uint32_t weight_x10,
                                       uint16_t threshold_g,
                                       uint8_t sequence,
                                       uint32_t now_ms,
                                       uint8_t *alarm_event);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_PRESSURE_CONTROL_H */
