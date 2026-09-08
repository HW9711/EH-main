#include "pump_pressure_control.h"
#include <string.h> /* 清除每路报警状态，停止和排空后不继承旧确认时间。 */

#if (PUMP_PRESSURE_CONTROL_ENABLE != 0U)
/*
 * 函数功能：按当前流量在两个标定流量之间的位置，按比例算出报警阈值。
 * 输入参数：target_speed 为当前流量；speed_low/speed_high 为相邻标定流量，单位 mL/min；value_low/value_high 为对应阈值，单位 g。
 * 返回参数：返回 target_speed 对应的 g 单位压力阈值。
 */
static uint16_t PumpPressure_Interpolate(uint16_t target_speed,
                                         uint16_t speed_low,
                                         uint16_t speed_high,
                                         uint16_t value_low,
                                         uint16_t value_high)
{
    /* 两个标定流量的差值，作为按比例计算的分母。 */
    uint32_t speed_span = (uint32_t)speed_high - (uint32_t)speed_low;
    /* 当前流量比低端标定流量大多少，决定报警阈值应增加多少。 */
    uint32_t speed_offset = (uint32_t)target_speed - (uint32_t)speed_low;
    /* 两个报警阈值的差值；配置表要求后一点不小于前一点。 */
    uint32_t value_span = (uint32_t)value_high - (uint32_t)value_low;
    /* value_g 保存插值后的 g 单位压力阈值，使用 32 位避免乘法中间值溢出。 */
    uint32_t value_g;

    /* 如果两个标定泵速误配成相同值，直接使用低速阈值，避免除 0 影响泵任务。 */
    if (speed_span == 0U)
    {
        /* 使用低端阈值，避免错误配置导致除零。 */
        return value_low;
    }

    /* 按线性插值计算阈值，speed_span / 2 用于四舍五入，减少整数除法系统性偏低。 */
    value_g = (uint32_t)value_low + (((value_span * speed_offset) + (speed_span / 2U)) / speed_span);
    /* 返回插值后的 g 单位阈值，供上层再转换成 WeightX10 的 0.1g 单位。 */
    return (uint16_t)value_g;
}

/*
 * 函数功能：根据目标业务流量查询主控压力报警阈值，相邻标定点之间线性插值。
 * 输入参数：target_speed 为目标业务流量，注水和灌注单位为 ml/min。
 * 返回参数：返回 g 单位报警阈值；抽吸泵传入 0~15 的内部设定，因此使用最低标定阈值。
 */
static uint16_t PumpPressure_AlarmThreshold(uint16_t target_speed)
{
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN)
    {
        return PUMP_PRESSURE_CONTROL_ALARM_50_G; /* 0~50 ml/min 沿用最低标定点，避免低流量阈值外推。 */
    }
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN)
    {
        /* 50~110 ml/min 按相邻报警值插值，使流量调节时报警阈值连续变化。 */
        return PumpPressure_Interpolate(target_speed,
                                        PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_ALARM_50_G,
                                        PUMP_PRESSURE_CONTROL_ALARM_110_G);
    }
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN)
    {
        /* 110~140 ml/min 按相邻报警值插值，使流量调节时报警阈值连续变化。 */
        return PumpPressure_Interpolate(target_speed,
                                        PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_ALARM_110_G,
                                        PUMP_PRESSURE_CONTROL_ALARM_140_G);
    }
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN)
    {
        /* 140~200 ml/min 按相邻报警值插值，使流量调节时报警阈值连续变化。 */
        return PumpPressure_Interpolate(target_speed,
                                        PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_ALARM_140_G,
                                        PUMP_PRESSURE_CONTROL_ALARM_200_G);
    }
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN)
    {
        /* 200~260 ml/min 按相邻报警值插值，使流量调节时报警阈值连续变化。 */
        return PumpPressure_Interpolate(target_speed,
                                        PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_ALARM_200_G,
                                        PUMP_PRESSURE_CONTROL_ALARM_260_G);
    }
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN)
    {
        /* 260~300 ml/min 按相邻报警值插值，使流量调节时报警阈值连续变化。 */
        return PumpPressure_Interpolate(target_speed,
                                        PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN,
                                        PUMP_PRESSURE_CONTROL_ALARM_260_G,
                                        PUMP_PRESSURE_CONTROL_ALARM_300_G);
    }
    return PUMP_PRESSURE_CONTROL_ALARM_300_G; /* 超过最高标定流量时沿用 300 ml/min 报警值。 */
}
#endif

/*
 * 函数功能：清除单路泵的压力报警确认和恢复状态，用于停泵、排空或压力数据无效时。
 * 输入参数：state 指向该泵独立的报警状态，A/B 泵不得共用。
 * 返回参数：无。
 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state)
{
    memset(state, 0, sizeof(*state)); /* 只清压力报警记录，不修改运行请求、速度设定或驱动故障记录。 */
}

/*
 * 函数功能：用新压力帧确认持续超限，产生一次报警事件，并在压力稳定回落后重新允许报警。
 * 输入参数：state 为单路状态；target_speed 为目标流量；weight_x10 单位 0.1g；threshold_g 为压力帧有效性字段；
 *           sequence 为压力帧序号；now_ms 为本周期 HAL 毫秒时刻。
 * 返回参数：新确认的一次超限返回 1，其余返回 0；本函数不控制泵和手柄启停。
 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint32_t weight_x10,
                                        uint16_t threshold_g,
                                        uint8_t sequence,
                                        uint32_t now_ms)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE == 0U)
    (void)target_speed; /* 报警关闭时不根据目标流量建立新事件。 */
    (void)weight_x10; /* 报警关闭时忽略压力读数。 */
    (void)threshold_g; /* 报警关闭时不使用有效性字段。 */
    (void)sequence; /* 报警关闭时不记录帧序号。 */
    (void)now_ms; /* 报警关闭时不累计确认时间。 */
    PumpPressureControl_ResetAlarm(state); /* 清掉本通道旧报警记录，避免重新启用后继承旧事件。 */
    return 0U; /* 总开关关闭时不产生报警提示。 */
#else
    uint32_t alarm_x10; /* 当前业务流量对应的报警阈值，统一换为 0.1g 比较。 */
    uint32_t recover_x10; /* 报警阈值减回差后的恢复阈值，防止临界压力反复提示。 */
    uint32_t confirm_ms; /* 根据当前是否已报警，选择超限或恢复的确认时长。 */
    uint8_t condition_met; /* 未报警时检查超限，已报警时检查是否稳定低于恢复线。 */

    if ((target_speed == 0U) || (threshold_g == 0U) ||
        (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10))
    {
        PumpPressureControl_ResetAlarm(state); /* 停止或无效读数不能作为持续超压证据。 */
        return 0U; /* 6000 是未完成标定的特殊值，不按真实超压报警。 */
    }

    if ((state->target_speed != target_speed) ||
        ((state->sequence_valid != 0U) &&
         ((uint32_t)(now_ms - state->last_sample_tick_ms) >= PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS)))
    {
        state->timing_active = 0U; /* 流量变化或断帧后重新确认，不把不同阈值或旧读数累计成连续事件。 */
    }
    state->target_speed = target_speed; /* 记录本次阈值对应的流量，已报警标志仍等压力回落才解除。 */

    if ((state->sequence_valid != 0U) && (state->last_sequence == sequence))
    {
        return 0U; /* 25ms 泵任务重复读取同一压力帧时，不能靠重复旧值触发报警或恢复。 */
    }
    state->last_sequence = sequence; /* 只在帧序号变化时接收新压力样本，支持 255 回绕到 0。 */
    state->sequence_valid = 1U; /* 记录已经读取过有效帧，首次序号为 0 也不会被丢弃。 */
    state->last_sample_tick_ms = now_ms; /* 保存本次新样本时刻，后续识别采样间断。 */

    alarm_x10 = (uint32_t)PumpPressure_AlarmThreshold(target_speed) * 10U; /* 宏值单位 g，压力板数据单位 0.1g。 */
    recover_x10 = (uint32_t)PUMP_PRESSURE_ALARM_HYSTERESIS_G * 10U; /* 把恢复线与报警线的差值从 g 换成 0.1g。 */
    recover_x10 = (alarm_x10 > recover_x10) ? (alarm_x10 - recover_x10) : 0U; /* 防止误配大回差造成无符号下溢。 */

    if (state->alarm_active == 0U)
    {
        condition_met = (weight_x10 >= alarm_x10) ? 1U : 0U; /* 达到报警线才开始确认，单帧尖峰不立即提示。 */
        confirm_ms = PUMP_PRESSURE_ALARM_CONFIRM_MS; /* 连续超限必须达到配置时长。 */
    }
    else
    {
        condition_met = (weight_x10 <= recover_x10) ? 1U : 0U; /* 已报警后必须回落超过回差，临界抖动不重置报警事件。 */
        confirm_ms = PUMP_PRESSURE_ALARM_RECOVER_MS; /* 持续低于恢复线后才允许下一次超压提示。 */
    }

    if (condition_met == 0U)
    {
        state->timing_active = 0U; /* 条件中断即清计时，分散尖峰或分散低压样本不能累加。 */
        return 0U; /* 保留当前报警状态，不触发新的蜂鸣或弹窗。 */
    }
    if (state->timing_active == 0U)
    {
        state->condition_tick_ms = now_ms; /* 从首个满足条件的新样本开始计时，避免提前确认。 */
        state->timing_active = 1U; /* 标记开始计时，即使开始时刻为 0 也能与“未计时”区分。 */
    }
    if ((uint32_t)(now_ms - state->condition_tick_ms) < confirm_ms)
    {
        return 0U; /* 使用无符号时间差，跨 HAL tick 回绕时仍按真实持续时间判断。 */
    }

    state->timing_active = 0U; /* 当前确认完成，下一次压力变化重新开始计时。 */
    if (state->alarm_active == 0U)
    {
        state->alarm_active = 1U; /* 记住本次已经提示，压力持续偏高时不再重复弹窗和蜂鸣。 */
        return 1U; /* 仅首次确认超限时通知上层弹窗和蜂鸣。 */
    }
    state->alarm_active = 0U; /* 压力已稳定回落，允许下一次独立超压事件重新报警。 */
    return 0U; /* 恢复只更新报警记录，不操作泵或手柄运行状态。 */
#endif
}

/*
 * 函数功能：压力板尚未完成自标定时将输出设为零；真实压力超限不改变输出。
 * 输入参数：target_speed 为调用方待输出的数值，原样返回、不换算单位；weight_x10 为压力板 0.1g 读数。
 * 返回参数：压力检查启用且读数为 6000 时返回 0，否则返回原值；排空时调用方不调用本函数。
 */
uint16_t PumpPressureControl_ApplyReadiness(uint16_t target_speed, uint32_t weight_x10)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE != 0U)
    if (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10)
    {
        return 0U; /* 6000 表示压力板未就绪，先发送零速等待标定完成，不报超压。 */
    }
#else
    (void)weight_x10; /* 压力检查关闭时连未就绪值也不检查，直接使用请求值。 */
#endif
    return target_speed; /* 不是未就绪值就返回原请求，真实超压不会在这里减速或停泵。 */
}
