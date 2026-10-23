#ifndef __PUMP_PRESSURE_CONTROL_H
#define __PUMP_PRESSURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 压力控制总开关：0 关闭压力减速和提示；非 0 按泵类型执行压力处理。 */
#ifndef PUMP_PRESSURE_CONTROL_ENABLE
#define PUMP_PRESSURE_CONTROL_ENABLE 1U
#endif

/* 灌注保护总开关：1启用分级降流、保压和提示；0仅保留用户流量，注水与抽吸逻辑不受影响。 */
#ifndef PUMP_PRESSURE_POUR_PROTECTION_ENABLE
#define PUMP_PRESSURE_POUR_PROTECTION_ENABLE 1U /* 当前启用动态调流；所有超压只降流和报警，压力数据失效仅提示并保持当前输出。 */
#endif

/* 按已确认的逐档mmHg目标，用P90-MEDIAN-PIECEWISE-20261021-V1离线反算；主控仅比较0.1g整数。 */
/* 完整流量/启动/恢复表位于pump_pressure_control.c的PumpPressure_GetPourThresholds；按当前输出选档。 */
/* 100mL/min及以下使用100mmHg启动线：256.689861g向上取为256.7g；20档用此线判断是否进入5档。 */
#define PUMP_PRESSURE_POUR_100_HIGH_X10 2567U
/* 100mL/min及以下使用50mmHg恢复线：32.662982g向下取为32.6g，保留50mmHg回差。 */
#define PUMP_PRESSURE_POUR_100_RECOVER_X10 326U
/* 300mmHg对应423.817079g；向上取到423.9g，持续确认后只提示一次，流量继续按当前阶段调节，不停泵。 */
#define PUMP_PRESSURE_POUR_URGENT_X10 4239U
/* 正常运行首次超压须持续500ms；短于此时长的波动不启动一级降流。 */
#define PUMP_PRESSURE_POUR_HIGH_CONFIRM_MS 500U
/* 二级试升后再次超压仍确认200ms；独立计时避免首次降流延时影响恢复失败判定。 */
#define PUMP_PRESSURE_POUR_RECOVER_HIGH_CONFIRM_MS 200U
/* 高压力线确认200ms，避免单个尖峰触发提示，不跳过一级慢调过程。 */
#define PUMP_PRESSURE_POUR_URGENT_CONFIRM_MS 200U
/* 每次降档后至少等待500ms，并收到新压力后才决定下一档；不按旧帧补调多档。 */
#define PUMP_PRESSURE_POUR_ADJUST_MS 500U
/* 当前输出达到200mL/min时使用快速档，不按用户设定值反复触发。 */
#define PUMP_PRESSURE_POUR_FAST_FROM_ML_MIN 200U
/* 高流量确认超压后一次降到160mL/min，再进入反馈细调。 */
#define PUMP_PRESSURE_POUR_FAST_FLOW_ML_MIN 160U
/* 当前输出达到100mL/min时，每次细调减少20mL/min。 */
#define PUMP_PRESSURE_POUR_LARGE_STEP_FROM_ML_MIN 100U
/* 100至199mL/min区间的单次降幅，单位mL/min。 */
#define PUMP_PRESSURE_POUR_LARGE_STEP_ML_MIN 20U
/* 低于100mL/min时的单次降幅，单位mL/min。 */
#define PUMP_PRESSURE_POUR_SMALL_STEP_ML_MIN 10U
/* 一级工作下限为20mL/min；用户设定更低时以用户设定为下限，不能因保护而加速。 */
#define PUMP_PRESSURE_POUR_WORK_MIN_ML_MIN 20U
/* 二级低流量为5mL/min；此档所有真实超压都只报警，不能因压力数值触发零速。 */
#define PUMP_PRESSURE_POUR_LOW_FLOW_ML_MIN 5U
/* 一级确认超压后累计高压达到5000ms进入二级；短暂回落、用户调速和自动调档不清零。 */
#define PUMP_PRESSURE_POUR_HIGH_TOTAL_MS 5000U
/* 压力低于恢复线连续1000ms后才增加一档；每次增流都重新确认1000ms。 */
#define PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS 1000U
/* 每次恢复增加10mL/min，最高不超过当前用户设定，不能直接恢复高流量。 */
#define PUMP_PRESSURE_POUR_RECOVER_STEP_ML_MIN 10U
/* 二级低流量仍持续超压500ms时仅提示；同一轮调流只提示一次，不定时重报。 */
#define PUMP_PRESSURE_POUR_LOW_ALARM_CONFIRM_MS 500U
/* 三类泵运行中500ms没有有效新压力即提示；压力板明确无效或未就绪立即提示，均保持当前输出。 */
#define PUMP_PRESSURE_SAMPLE_TIMEOUT_MS 500U

#if ((PUMP_PRESSURE_POUR_100_RECOVER_X10 >= PUMP_PRESSURE_POUR_100_HIGH_X10) ||      (PUMP_PRESSURE_POUR_100_HIGH_X10 >= PUMP_PRESSURE_POUR_URGENT_X10))
#error PUMP_PRESSURE_POUR_thresholds_must_satisfy_recover_high_urgent
#endif
#if ((PUMP_PRESSURE_POUR_LOW_FLOW_ML_MIN == 0U) ||      (PUMP_PRESSURE_POUR_LOW_FLOW_ML_MIN >= PUMP_PRESSURE_POUR_WORK_MIN_ML_MIN) ||      (PUMP_PRESSURE_POUR_FAST_FLOW_ML_MIN >= PUMP_PRESSURE_POUR_FAST_FROM_ML_MIN) ||      (PUMP_PRESSURE_POUR_ADJUST_MS == 0U) || (PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS == 0U))
#error PUMP_PRESSURE_POUR_flow_and_timing_configuration_invalid
#endif

/* 阶段0：正常按设定流量运行，等待普通超压连续确认。 */
#define PUMP_PRESSURE_POUR_NORMAL 0U
/* 阶段1：一级逐档降流、保持和小步恢复，保留本轮累计高压时间。 */
#define PUMP_PRESSURE_POUR_ADJUST 1U
/* 阶段2：保持不高于5mL/min，稳定低压后允许一次自动恢复尝试。 */
#define PUMP_PRESSURE_POUR_LOW 2U
/* 阶段3：已经使用本轮唯一的自动恢复机会，再次确认超压即退回低流量并锁住自动恢复。 */
#define PUMP_PRESSURE_POUR_RECOVER 3U
/* 阶段4：恢复失败后保持低流量并提示人工检查，低压也不再自动试升。 */
#define PUMP_PRESSURE_POUR_LOW_LOCKED 4U

/* 返回事件位0：请求现有蜂鸣和压力弹窗，不修改运行请求。 */
#define PUMP_PRESSURE_EVENT_ALARM 0x01U
/* 返回事件位1：压力检测失效，仅蜂鸣三声并显示74号图窗2秒，不改变运行请求。 */
#define PUMP_PRESSURE_EVENT_SENSOR_FAULT 0x02U

/*
 * 旧版 UART10 压力调试开关，保留 0=关闭、1=开启的历史定义。
 * 当前代码没有读取该宏，改成 1 不会产生调试日志；正式配置保持 0。
 */
#ifndef PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE
#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U
#endif

/* 压力板未完成自标定时发送的特殊值，单位0.1g；灌注、注水和抽吸均按检测失效提示，不改变运行请求。 */
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

/* 抽吸沿用的历史最低档提示阈值，当前130g；灌注动态调流不读取以下历史曲线。 */
#define PUMP_PRESSURE_CONTROL_ALARM_50_G 130U
/* 历史110mL/min参考阈值210g，保留抽吸旧曲线接口，不参与灌注调流。 */
#define PUMP_PRESSURE_CONTROL_ALARM_110_G 210U
/* 历史140mL/min参考阈值240g，保留原曲线递增顺序，不参与灌注调流。 */
#define PUMP_PRESSURE_CONTROL_ALARM_140_G 240U
/* 历史200mL/min参考阈值270g，保留原曲线接口，不参与灌注调流。 */
#define PUMP_PRESSURE_CONTROL_ALARM_200_G 270U
/* 历史260mL/min参考阈值310g，保留原曲线接口，不参与灌注调流。 */
#define PUMP_PRESSURE_CONTROL_ALARM_260_G 310U
/* 历史300mL/min参考阈值360g，保留原曲线接口，不参与灌注调流。 */
#define PUMP_PRESSURE_CONTROL_ALARM_300_G 360U

/* 注水泵统一的压力提示阈值，单位 g；不再依据手柄型号或流量选择报警线，调大将延后提示。 */
#define PUMP_PRESSURE_INJECT_ALARM_G 800U

/* 注水泵高于 800g、抽吸泵达到原阈值后连续多久才提示，单位 ms；当前 1000，调大将延后提示。 */
#define PUMP_PRESSURE_ALARM_CONFIRM_MS 1000U
/* 注水/抽吸持续超压时仍按10秒重报；灌注不使用重复间隔，同一轮只提示一次。 */
#define PUMP_PRESSURE_ALARM_REPEAT_MS 10000U
/* 注水/抽吸解除压力提示记录的低压确认时长，单位 ms；当前 1000，调大后更晚允许下一次独立超压提示。 */
#define PUMP_PRESSURE_ALARM_RECOVER_MS 1000U
/* 注水泵和抽吸泵恢复线比提示线低多少，单位g；灌注恢复阈值随当前输出流量选取。 */
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

/* 每路泵任务独占一份压力状态，A/B通道的流量、计时及提示不能互相影响。 */
typedef struct
{
    uint32_t condition_tick_ms; /* 首个连续高压样本时刻；注水/抽吸仍用于其超限或恢复确认，单位ms。 */
    uint32_t last_sample_tick_ms; /* 最近有效新压力样本时刻；三类泵超时500ms只提示，灌注同时暂停压力调流。 */
    uint32_t last_alarm_tick_ms; /* 最近一次提示时刻；注水/抽吸每10秒重报，灌注只记录本轮首次提示。 */
    uint16_t target_speed; /* 当前用户设定流量；自动降流不覆盖该值，用户调速不清累计超压。 */
    uint16_t alarm_threshold_g; /* 最近一次判定使用的整g启动线；灌注按当前输出选档后四舍五入显示，实际比较0.1g。 */
    uint16_t last_output_speed; /* 上次实际下发的流量目标；短时重复压力帧时保持，不能回跳原设定。 */
    uint8_t last_sequence; /* 最近有效压力帧序号；相同序号不能重复参与确认和调档。 */
    uint8_t sequence_valid; /* 已接收有效样本时置1，使首次序号为0仍能正常处理。 */
    uint8_t alarm_active; /* 本轮已提示时置1；灌注须恢复原设定后再稳定低压1秒才清零，该标志不代表停泵。 */
    uint8_t timing_active; /* 正在累计连续高压时置1；注水/抽吸仍沿用原提示确认规则。 */
    uint8_t mode; /* 0未使用、1抽吸、2注水、3灌注；用途切换时清除上一用途的状态。 */
    uint8_t pour_phase; /* 灌注阶段：正常、一级调节、二级低流量、恢复尝试、低流量锁定；检测失效不改变本阶段。 */
    uint32_t pour_adjust_tick_ms; /* 最近一次调档时刻；至少等待500ms及新压力后才继续降档。 */
    uint32_t pour_low_tick_ms; /* 首个连续低压样本时刻；每次增流后重新确认1秒。 */
    uint32_t pour_high_total_ms; /* 一级确认后累计高压时间，最多5000ms；短暂回落和调档不清零。 */
    uint32_t pour_urgent_tick_ms; /* 首个达到423.9g的样本时刻；确认200ms后只提示一次，不改变当前慢调节奏。 */
    uint8_t pour_last_high; /* 上一新样本是否达到当时输出流量的启动线；仅相邻两帧均超限才累计其间隔。 */
    uint8_t pour_low_timing_active; /* 连续低于或等于当前输出流量的恢复线时置1，回升或调档后重新确认。 */
    uint8_t sensor_fault_active; /* 本通道检测失效已提示标志；持续失效不重报，有效新帧恢复后清零。 */
    uint8_t pour_urgent_timing_active; /* 连续达到423.9g且输出高于低流量档时置1，回落即清零。 */
} PumpPressureAlarmState_t;

/* 停止或泵类型切换时清除单路压力记录，不修改开泵请求。 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state);
/* 注水固定按800g、抽吸沿用原曲线；无效或超时压力返回独立检测失效提示事件。 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint8_t is_injection, /* 1 为注水泵，0 为抽吸泵。 */
                                        uint32_t weight_x10,
                                        uint16_t threshold_g,
                                        uint8_t sequence,
                                        uint32_t now_ms);
/* 灌注保护开启时按当前输出选取逐档g阈值，执行分级降流、保压、恢复和提示；压力检测失效只产生独立提示事件并保持当前输出。 */
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
