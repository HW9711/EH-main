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
 * 函数功能：按原灌注压力标定点查询当前设定流量对应的阈值，抽吸泵继续使用原最低档。
 * 输入参数：target_speed 为灌注设定流量，单位 mL/min；抽吸传入 0~15 的内部设定值。
 * 返回参数：返回 g 单位压力阈值，流量超出标定范围时使用最近的端点。
 */
static uint16_t PumpPressure_LegacyAlarmThreshold(uint16_t target_speed)
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

/*
 * 函数功能：重新开始本泵的平均压力统计并清除步进余数，保留已下发流量、压力降速标志及超压停泵记录。
 * 输入参数：state 指向本泵独立的压力状态，不能为 NULL。
 * 返回参数：无。
 */
static void PumpPressure_ResetPourAdjustment(PumpPressureAlarmState_t *state)
{
    state->pour_pressure_sum = 0U; /* 丢弃旧窗口的压力，不能把无效帧或设定改变前后的样本混在一起。 */
    state->pour_window_start_ms = 0U; /* 下一份有效新帧到来时再记录窗口起点，不补算等待期间的时间。 */
    state->pour_sample_count = 0U; /* 标记当前没有窗口，必须重新采集满一个调节周期。 */
    state->pour_low_duration_ms = 0U; /* 清除平均低压确认，不能用中断前的低压力让泵恢复或加速。 */
    state->pour_step_remainder = 0U; /* 新过程重新累计不足一档的流量，避免沿用另一设定的余数。 */
}

/*
 * 函数功能：平均压力偏高时只降低一档流量，按设定流量计算降幅，连续十二档降至最低有效流量。
 * 输入参数：state 为本泵状态；target_speed/minimum_speed 为设定上限/最低非零流量，单位mL/min。
 * 返回参数：返回更新后的下发流量，保持在 minimum_speed 到 target_speed 之间；调用前本泵必须已获准输出非零流量。
 */
static uint16_t PumpPressure_StepPourFlow(PumpPressureAlarmState_t *state,
                                         uint16_t target_speed,
                                         uint16_t minimum_speed)
{
    uint32_t numerator; /* 保存设定流量差乘单档周期及上次余数，避免低流量每档取整为零。 */
    uint32_t flow_step; /* 本次可减的整数流量，单位mL/min；先与剩余可减范围比较，防止减到最低档以下。 */

    state->pour_adjust_direction = 1U; /* 记录本泵已经按压力减速，正常压力确认完成前不能因上调设定而直接加速。 */
    numerator = (uint32_t)(target_speed - minimum_speed) * PUMP_PRESSURE_POUR_ADJUST_MS + state->pour_step_remainder; /* 每档幅度按用户设定和降速时长计算，输出变小后不继续缩小步长。 */
    flow_step = numerator / PUMP_PRESSURE_POUR_RAMP_DOWN_MS; /* 当前每500ms减少全流量差的十二分之一，让下降过程更缓。 */
    state->pour_step_remainder = numerator % PUMP_PRESSURE_POUR_RAMP_DOWN_MS; /* 未满一个流量单位的部分留到下次下降，进入保持区也不丢弃。 */
    if (flow_step >= (uint32_t)(state->last_output_speed - minimum_speed)) /* 剩余可减流量不足一整档时，只降至最低有效值。 */
    {
        state->last_output_speed = minimum_speed; /* 不产生低于最低有效流量的命令，也不因取整变成零速。 */
        state->pour_step_remainder = 0U; /* 已到最低档，清掉不能继续执行的下降量。 */
    }
    else
    {
        state->last_output_speed = (uint16_t)(state->last_output_speed - flow_step); /* 保存本档减少后的流量，后续平均压力仍偏高时再降低一档。 */
    }
    return state->last_output_speed; /* 返回本泵应下发的流量，A/B泵分别保存各自的减速记录。 */
}

#endif

/*
 * 函数功能：清除单路泵的压力确认和恢复状态，用于停泵或泵类型切换时。
 * 输入参数：state 指向该泵独立的报警状态，A/B 泵不得共用。
 * 返回参数：无。
 */
void PumpPressureControl_ResetAlarm(PumpPressureAlarmState_t *state)
{
    memset(state, 0, sizeof(*state)); /* 只清压力统计、流量调节和停转记录，不修改运行请求、速度设定或驱动故障记录。 */
}

/*
 * 函数功能：注水固定按 800g、抽吸按原最低档确认超压；首次提示后每隔 10 秒仍超限就重报。
 * 输入参数：state 为单路状态；target_speed 为业务设定；is_injection 为 1 表示注水、0 表示抽吸；weight_x10 单位 0.1g；threshold_g 为压力帧有效性字段；
 *           sequence 为压力帧序号；now_ms 为本周期 HAL 毫秒时刻。
 * 返回参数：首次确认超限或到达重复提示条件时返回1，其余返回0；本函数不控制泵和手柄启停。
 */
uint8_t PumpPressureControl_UpdateAlarm(PumpPressureAlarmState_t *state,
                                        uint16_t target_speed,
                                        uint8_t is_injection,
                                        uint32_t weight_x10,
                                        uint16_t threshold_g,
                                        uint8_t sequence,
                                        uint32_t now_ms)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE == 0U)
    (void)target_speed; /* 报警关闭时不根据目标流量建立新事件。 */
    (void)is_injection; /* 总开关关闭时不判断泵类型。 */
    (void)weight_x10; /* 报警关闭时忽略压力读数。 */
    (void)threshold_g; /* 报警关闭时不使用有效性字段。 */
    (void)sequence; /* 报警关闭时不记录帧序号。 */
    (void)now_ms; /* 报警关闭时不累计确认时间。 */
    PumpPressureControl_ResetAlarm(state); /* 清掉本通道旧报警记录，避免重新启用后继承旧事件。 */
    return 0U; /* 总开关关闭时不产生报警提示。 */
#else
    uint16_t alarm_threshold_g; /* 注水使用固定 800g，抽吸沿用原最低标定点。 */
    uint32_t alarm_x10; /* 当前业务流量对应的报警阈值，统一换为 0.1g 比较。 */
    uint32_t recover_x10; /* 报警阈值减回差后的恢复阈值，防止临界压力反复提示。 */
    uint32_t confirm_ms; /* 根据当前是否已报警，选择超限或恢复的确认时长。 */
    uint8_t condition_met; /* 未报警时检查超限，已报警时检查是否稳定低于恢复线。 */
    uint8_t mode = (is_injection != 0U) ? 2U : 1U; /* 抽吸和注水使用不同提示阈值，切换泵用途时不能沿用旧计时。 */

    if (state->mode != mode)
    {
        PumpPressureControl_ResetAlarm(state); /* 泵用途改变后清掉另一种泵的压力提示和计时。 */
        state->mode = mode; /* 保存本次泵用途，后续新压力帧按对应阈值累计。 */
    }

    if ((target_speed == 0U) || (threshold_g == 0U) ||
        (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10))
    {
        PumpPressureControl_ResetAlarm(state); /* 停止或无效读数不能作为持续超压证据。 */
        return 0U; /* 6000 是未完成标定的特殊值，不按真实超压报警。 */
    }

    alarm_threshold_g = (is_injection != 0U) ? PUMP_PRESSURE_INJECT_ALARM_G : PumpPressure_LegacyAlarmThreshold(target_speed); /* 注水不再按手柄型号或流量选阈值。 */
    if (((is_injection == 0U) && (state->target_speed != target_speed)) ||
        (state->alarm_threshold_g != alarm_threshold_g) ||
        ((state->sequence_valid != 0U) &&
         ((uint32_t)(now_ms - state->last_sample_tick_ms) >= PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS)))
    {
        state->timing_active = 0U; /* 抽吸设定或阈值变化、压力断帧后重新确认；注水调流量不打断 800g 计时。 */
    }
    state->target_speed = target_speed; /* 记录本次阈值对应的流量，已报警标志仍等压力回落才解除。 */
    state->alarm_threshold_g = alarm_threshold_g; /* 保存实际报警线供下一周期比较，不清已报警状态或最近提示时间。 */

    if ((state->sequence_valid != 0U) && (state->last_sequence == sequence))
    {
        return 0U; /* 25ms 泵任务重复读取同一压力帧时，不能靠重复旧值触发报警或恢复。 */
    }
    state->last_sequence = sequence; /* 只在帧序号变化时接收新压力样本，支持 255 回绕到 0。 */
    state->sequence_valid = 1U; /* 记录已经读取过有效帧，首次序号为 0 也不会被丢弃。 */
    state->last_sample_tick_ms = now_ms; /* 保存本次新样本时刻，后续识别采样间断。 */

    alarm_x10 = (uint32_t)alarm_threshold_g * 10U; /* 把本次 g 单位阈值换成 0.1g，与压力帧使用相同单位。 */
    recover_x10 = (uint32_t)PUMP_PRESSURE_ALARM_HYSTERESIS_G * 10U; /* 把恢复线与报警线的差值从 g 换成 0.1g。 */
    recover_x10 = (alarm_x10 > recover_x10) ? (alarm_x10 - recover_x10) : 0U; /* 防止误配大回差造成无符号下溢。 */

    /* 已经提示过且本帧仍达到报警线时，距上次提示满10秒就再次通知；旧帧已在上面排除。 */
    if ((state->alarm_active != 0U) && ((is_injection != 0U) ? (weight_x10 > alarm_x10) : (weight_x10 >= alarm_x10)) &&
        ((uint32_t)(now_ms - state->last_alarm_tick_ms) >= PUMP_PRESSURE_ALARM_REPEAT_MS))
    {
        state->last_alarm_tick_ms = now_ms; /* 本次重报作为下一轮10秒等待的起点，避免每个任务周期都提示。 */
        state->timing_active = 0U; /* 压力再次超限，取消此前尚未完成的低压恢复计时。 */
        return 1U; /* 请求重新显示2秒弹窗并蜂鸣两声，不修改任何运行输出。 */
    }

    if (state->alarm_active == 0U)
    {
        condition_met = ((is_injection != 0U) ? (weight_x10 > alarm_x10) : (weight_x10 >= alarm_x10)) ? 1U : 0U; /* 注水必须超过 800g；抽吸保持原达到阈值的判断。 */
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
        state->alarm_active = 1U; /* 记录首次超压已经确认，之后检查恢复条件和10秒重复提示。 */
        state->last_alarm_tick_ms = now_ms; /* 从首次提示时开始计时，不从第一次超限样本开始计时。 */
        return 1U; /* 首次持续超限确认完成，通知上层弹窗和蜂鸣。 */
    }
    state->alarm_active = 0U; /* 压力已稳定回落，允许下一次独立超压事件重新报警。 */
    return 0U; /* 恢复只更新报警记录，不操作泵或手柄运行状态。 */
#endif
}

/*
 * 函数功能：每500ms按平均压力逐档减速或保持；平均低压连续正常2秒后直接恢复设定，原始压力连续超限2秒立即返回零速。
 * 输入参数：state 为本泵独立状态且不能为NULL；target_speed 为用户设定流量，单位mL/min；weight_x10 为压力板0.1g读数。
 *           threshold_g 为压力帧有效性字段；sequence 为压力帧序号；now_ms 为主控毫秒时刻；alarm_event 为非NULL的一次性提示标志地址。
 * 返回参数：返回本周期应下发的流量，单位mL/min；0表示用户设为零或已确认超压停泵，不清除用户开泵请求。
 */
uint16_t PumpPressureControl_ApplyPour(PumpPressureAlarmState_t *state,
                                       uint16_t target_speed,
                                       uint32_t weight_x10,
                                       uint16_t threshold_g,
                                       uint8_t sequence,
                                       uint32_t now_ms,
                                       uint8_t *alarm_event)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE == 0U)
    (void)weight_x10; /* 总开关关闭时不根据压力改变灌注流量。 */
    (void)threshold_g; /* 总开关关闭时不检查压力板附带字段。 */
    (void)sequence; /* 总开关关闭时不累计压力帧。 */
    (void)now_ms; /* 总开关关闭时不进行超压和恢复计时。 */
    *alarm_event = 0U; /* 不发送本泵压力提示。 */
    PumpPressureControl_ResetAlarm(state); /* 清掉此前的停转记录，关闭压力控制后恢复原运行请求。 */
    return target_speed; /* 直接使用用户原设定流量。 */
#else
    uint16_t minimum_speed; /* 本次最低非零流量，通常为4mL/min；用户设定更低时使用用户设定。 */
    uint16_t alarm_threshold_g; /* 始终按用户原设定查询标定阈值，降速后的输出不参与查表。 */
    uint32_t alarm_x10; /* 原标定阈值换成0.1g，与原始压力读数比较以判断连续超压。 */
    uint32_t slowdown_x10; /* 平均压力高于此线就下降一档，当前为原阈值的70%。 */
    uint32_t recover_x10; /* 平均压力低于此线才累计恢复时间，当前为原阈值的65%。 */
    uint32_t sample_gap_ms; /* 与上一份有效新压力帧相隔多久，单位ms；长时间断帧后重新建立压力窗口。 */
    uint32_t window_elapsed_ms; /* 当前平均窗口覆盖的实际毫秒数，用于判断窗口完成及低压恢复时间。 */
    uint8_t pressure_band; /* 本窗口平均压力分类：0保持当前流量，1偏高需下降，2偏低可累计恢复。 */

    *alarm_event = 0U; /* 本周期默认不提示，只有第一次确认连续超压停泵时置1。 */
    if (state->mode != 3U) /* 泵用途改变后不能继承注水或抽吸的报警计时。 */
    {
        PumpPressureControl_ResetAlarm(state); /* 清除其它泵用途留下的样本、流量记录和报警状态。 */
        state->mode = 3U; /* 后续调用使用本泵独立的灌注统计和停泵条件。 */
    }
    if (target_speed == 0U) /* 用户已经撤销流量，不等待压力窗口或渐降过程。 */
    {
        PumpPressureControl_ResetAlarm(state); /* 下一次开泵重新采样，不继承本次减速或自动恢复条件。 */
        return 0U; /* 本周期直接下发零速。 */
    }
    if ((state->sequence_valid != 0U) && (state->last_output_speed > target_speed)) /* 用户调低设定时，即使暂时没有新压力帧也要立即限幅。 */
    {
        state->last_output_speed = target_speed; /* 保存本周期已经采用的较低输出，后续统计不能把流量抬回旧值。 */
    }
    if ((threshold_g == 0U) || (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10)) /* 无效帧和6000未就绪值不参与平均或持续超压判断。 */
    {
        state->timing_active = 0U; /* 中断原始压力连续超限计时，不能拼接无效帧前后的时间。 */
        PumpPressure_ResetPourAdjustment(state); /* 重新等待有效压力窗口，无效期间既不补降也不确认恢复。 */
        return (state->alarm_active != 0U) ? 0U :
               ((state->sequence_valid != 0U) ? state->last_output_speed : target_speed); /* 已停泵仍返回零速；其余沿用最近输出，尚无有效帧时保持原请求。 */
    }

    sample_gap_ms = (uint32_t)(now_ms - state->last_sample_tick_ms); /* 使用无符号差值，HAL毫秒计时回绕时仍能判断真实间隔。 */
    if ((state->sequence_valid != 0U) && (state->last_sequence == sequence)) /* 25ms任务可能多次读到同一帧，不能重复累计该读数。 */
    {
        if (sample_gap_ms >= PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS) /* 旧帧已保持过久，本次仍保持输出，下一份新帧重新统计。 */
        {
            state->timing_active = 0U; /* 长时间没有新数据，原始压力超限计时不再有效。 */
            PumpPressure_ResetPourAdjustment(state); /* 丢弃旧平均值和低压确认，防止断帧后一次补调多档。 */
        }
        return (state->alarm_active != 0U) ? 0U : state->last_output_speed; /* 重复帧不调流量，也不能让已经停泵的通道恢复。 */
    }

    alarm_threshold_g = PumpPressure_LegacyAlarmThreshold(target_speed); /* 只按用户设定选择原标定阈值，保持A/B泵原来的压力基准。 */
    if ((state->sequence_valid == 0U) ||
        ((state->target_speed != target_speed) && (state->alarm_active == 0U) &&
         (state->last_output_speed == state->target_speed) && (state->pour_adjust_direction == 0U))) /* 首次运行或未受压力限制的正常调流量，继续立即采用用户设定。 */
    {
        state->last_output_speed = target_speed; /* 压力已经降低过流量时不会进入此分支，用户加大设定也不绕过正常压力确认。 */
    }
    if ((state->sequence_valid == 0U) || (state->target_speed != target_speed) ||
        (state->alarm_threshold_g != alarm_threshold_g) || (sample_gap_ms >= PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS)) /* 新运行、改设定或长间隔后，前后压力不能放在同一统计窗口。 */
    {
        state->timing_active = 0U; /* 与原行为一致：设定对应的阈值改变或断帧后重新确认连续超限。 */
        PumpPressure_ResetPourAdjustment(state); /* 从当前实际下发流量重新调节，不恢复此前被压力压低的流量。 */
    }
    state->target_speed = target_speed; /* 记录本次查表使用的用户设定，供下一份新帧检查设定是否改变。 */
    state->alarm_threshold_g = alarm_threshold_g; /* 保存本泵原标定阈值，诊断仍可按原接口读取。 */
    state->last_sequence = sequence; /* 本次只处理新的帧序号，255回到0也视为新帧。 */
    state->sequence_valid = 1U; /* 第一帧序号允许为0，后续才开始过滤重复序号。 */
    state->last_sample_tick_ms = now_ms; /* 保存本次有效新样本时间，后续据此识别长时间断帧。 */
    minimum_speed = (target_speed < PUMP_PRESSURE_POUR_MIN_FLOW_ML_MIN) ? target_speed : PUMP_PRESSURE_POUR_MIN_FLOW_ML_MIN; /* 最低流量不能高于用户设定。 */
    if ((state->alarm_active == 0U) && (state->last_output_speed < minimum_speed)) /* 原设定低于4而本次已允许至少4时，非零输出须回到最低有效档。 */
    {
        state->last_output_speed = minimum_speed; /* 避免后续下降计算小于零；已因超压停泵的通道仍保持0。 */
    }
    alarm_x10 = (uint32_t)alarm_threshold_g * 10U; /* 标定表单位为g，原始压力帧和平均值判断统一使用0.1g。 */
    slowdown_x10 = (alarm_x10 * PUMP_PRESSURE_POUR_SLOWDOWN_PERCENT) / 100U; /* 得到本档平均压力的减速判断线。 */
    recover_x10 = (alarm_x10 * PUMP_PRESSURE_POUR_RECOVER_PERCENT) / 100U; /* 得到较低的恢复判断线，两条线之间保持当前流量。 */

    if ((state->alarm_active == 0U) && (weight_x10 > alarm_x10)) /* 每份原始新帧独立检查超压，不让平均值延迟停泵保护。 */
    {
        if (state->timing_active == 0U) /* 第一份超过原阈值的新压力帧开始本次连续确认。 */
        {
            state->condition_tick_ms = now_ms; /* 记录真实超限起点，而不是平均窗口结束的时间。 */
            state->timing_active = 1U; /* 后续每份新帧必须仍高于原阈值才继续确认。 */
        }
        if ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_POUR_STOP_CONFIRM_MS) /* 原始压力连续超限已满2秒，优先停止输出。 */
        {
            state->alarm_active = 1U; /* 锁住本泵非零输出，必须重新确认平均压力恢复后才允许开泵。 */
            state->timing_active = 0U; /* 本次停泵确认结束，停泵期间不重复发送报警事件。 */
            state->last_alarm_tick_ms = now_ms; /* 保留首次停泵时刻，供诊断定位实际触发时间。 */
            state->last_output_speed = 0U; /* 记录本周期应下发零速，重复帧和无效帧仍保持停止。 */
            PumpPressure_ResetPourAdjustment(state); /* 停泵后的恢复只能使用新采集的平均压力，不能继承停泵前的低压窗口。 */
            *alarm_event = 1U; /* 只在首次停泵时请求原有弹窗和蜂鸣，不清用户开泵请求。 */
            return 0U; /* 立即把零速交给泵任务下发，不等待本轮500ms调节结束。 */
        }
    }
    else
    {
        state->timing_active = 0U; /* 原始压力回到阈值及以下，或本泵已经停转，取消本次连续超限计时。 */
    }

    if (state->pour_sample_count == 0U) /* 首帧只能开始统计，不能凭一次波峰直接选择最低流量。 */
    {
        state->pour_window_start_ms = now_ms; /* 用第一份有效新帧建立500ms统计起点。 */
        state->pour_pressure_sum = weight_x10; /* 当前帧作为窗口第一个压力值，不混入旧设定或断帧前的读数。 */
        state->pour_sample_count = 1U; /* 当前仅有一份有效新样本，等待后续压力数据。 */
        return state->last_output_speed; /* 新窗口完成前保持实际下发流量，已停泵时该值为0。 */
    }
    state->pour_pressure_sum += weight_x10; /* 只累加前面已确认的新帧，64位求和避免高压力读数累加溢出。 */
    state->pour_sample_count++; /* 平均时按实际收到的新帧数计算，重复帧不会放大某个波峰或波谷。 */
    window_elapsed_ms = (uint32_t)(now_ms - state->pour_window_start_ms); /* 统计实际经过时间，不按25ms任务执行次数猜测压力采样周期。 */
    if (window_elapsed_ms < PUMP_PRESSURE_POUR_ADJUST_MS) /* 本轮尚未覆盖500ms，只收集样本，不改变下发流量。 */
    {
        return state->last_output_speed; /* 两档之间保持稳定命令，压力瞬时变化不会引起上下跳速。 */
    }
    pressure_band = 0U; /* 平均压力落在两条判断线之间时，默认保持当前流量。 */
    if (state->pour_pressure_sum > (uint64_t)slowdown_x10 * state->pour_sample_count) /* 用总和比较阈值乘帧数，避免均值取整把略超70%的压力误判为保持。 */
    {
        pressure_band = 1U; /* 本窗口平均压力偏高，本次允许下降一档。 */
    }
    else if (state->pour_pressure_sum < (uint64_t)recover_x10 * state->pour_sample_count) /* 平均压力必须严格低于65%才算恢复条件成立。 */
    {
        pressure_band = 2U; /* 本窗口用于累计稳定低压时间，确认完成后才允许恢复。 */
    }
    state->pour_window_start_ms = now_ms; /* 本次调节后从当前帧重新统计，晚到的新帧也只执行一档，不一次补调。 */
    state->pour_pressure_sum = weight_x10; /* 当前帧同时作为下一窗口的首帧，让相邻窗口覆盖连续时间。 */
    state->pour_sample_count = 1U; /* 下一窗口重新计算帧数，同一序号仍由函数前面的重复帧判断过滤。 */

    if (pressure_band != 2U) /* 平均压力进入保持区或减速区，之前尚未确认的恢复时间立即作废。 */
    {
        state->pour_low_duration_ms = 0U; /* 分散的低压窗口不能累加成连续2秒恢复。 */
        if ((pressure_band == 1U) && (state->alarm_active == 0U)) /* 只有未因超压停泵的通道才允许逐档下降。 */
        {
            return PumpPressure_StepPourFlow(state, target_speed, minimum_speed); /* 从本泵当前流量减一档，达到原阈值也不再跳到4。 */
        }
        return state->last_output_speed; /* 保持区保留当前流量及小数余量；已停泵的通道继续返回0。 */
    }
    if (state->pour_low_duration_ms < PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS) /* 平均低压尚未确认满2秒，继续累计本窗口覆盖的时间。 */
    {
        if (window_elapsed_ms >= (PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS - state->pour_low_duration_ms)) /* 本窗口已足够完成确认，直接限制在所需时长。 */
        {
            state->pour_low_duration_ms = PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS; /* 确认值到达上限后不继续增长，长时间低压也不会计数溢出。 */
        }
        else
        {
            state->pour_low_duration_ms += window_elapsed_ms; /* 只累计连续平均低压窗口，保持区、高压和无效数据都已在上面清零。 */
        }
    }
    if (state->pour_low_duration_ms < PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS) /* 不足2秒时仍保持当前输出，不能提前恢复设定流量。 */
    {
        return state->last_output_speed; /* 本轮不加速，短时压力回落不会直接恢复用户设定。 */
    }
    state->alarm_active = 0U; /* 平均压力已连续正常满2秒，解除本泵的压力停转记录；未停泵时也使用相同恢复条件。 */
    state->last_output_speed = target_speed; /* 本周期直接恢复用户设定流量，取消逐档加速和停泵后先使用最低档的等待。 */
    state->pour_step_remainder = 0U; /* 恢复完成后丢弃旧下降余数，下次平均压力偏高时重新计算每档降幅。 */
    state->pour_adjust_direction = 0U; /* 本泵已退出压力减速状态，之后正常调整设定流量可立即生效。 */
    return state->last_output_speed; /* 交给原泵任务换算并下发恢复后的命令，本函数不修改用户开泵请求。 */
#endif
}
