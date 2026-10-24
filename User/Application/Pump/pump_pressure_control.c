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
 * 函数功能：三类泵压力检测失效时产生一次独立提示，保留超压记录；灌注暂停压力调流。
 * 输入参数：state为A/B任一路压力状态；now_ms为当前时刻，供灌注重新建立压力观察起点。
 * 返回参数：首次失效返回SENSOR_FAULT事件，持续失效返回0；不修改泵速及任何运行请求。
 */
static uint8_t PumpPressure_ReportSensorFault(PumpPressureAlarmState_t *state, uint32_t now_ms)
{
    if (state->sensor_fault_active == 0U) /* 每路泵每次连续失效只提示一次，不能每25ms重复弹窗。 */
    {
        state->sensor_fault_active = 1U; /* 独立于超压提示锁存，检测故障与本轮超压不会互相吞掉提示。 */
        state->timing_active = 0U; /* 无效压力不能延续超压确认，恢复后必须重新取得连续样本。 */
        state->pour_low_timing_active = 0U; /* 检测中断不能被误算为稳定低压，防止恢复后直接加速。 */
        state->pour_last_high = 0U; /* 清除旧样本的高压分类，检测恢复后用新帧重新确认，保留本轮堵塞记录。 */
        state->pour_adjust_tick_ms = now_ms; /* 灌注暂停压力调档；注水和抽吸不读取该调档字段。 */
        return PUMP_PRESSURE_EVENT_SENSOR_FAULT; /* 三类泵统一请求三声蜂鸣和74号2秒提示，不产生停泵事件。 */
    }
    return 0U; /* 同一段失效不重复提示；有效新帧恢复后允许下一次独立提示。 */
}

#if (PUMP_PRESSURE_POUR_PROTECTION_ENABLE != 0U)
/*
 * 函数功能：按当前下发流量选取灌注启动线和恢复线，中间流量插值；主控不进行mmHg单位换算。
 * 输入参数：output_speed为当前输出目标，单位mL/min；high_x10和recover_x10接收0.1g阈值，指针不能为NULL。
 * 返回参数：无；100及以下沿用100/50mmHg对应值，300及以上沿用200/150mmHg对应值。
 */
static void PumpPressure_GetPourThresholds(uint16_t output_speed, uint32_t *high_x10, uint32_t *recover_x10)
{
    /* 每行为流量mL/min、启动线0.1g、恢复线0.1g；启动向上取整、恢复向下取整，均已离线反算。 */
    static const uint16_t points[][3] = {
        {100U, PUMP_PRESSURE_POUR_100_HIGH_X10, PUMP_PRESSURE_POUR_100_RECOVER_X10}, /* 100及以下：100/50mmHg；首次超压按100确认，20档观察后仍未恢复则降5并提示，试升须保持低于50。 */
        {120U, 3057U, 579U}, /* 120档：110/60mmHg，对应305.7/57.9g，避免继续套用最高档启动线。 */
        {140U, 3546U, 1082U}, /* 140档：120/70mmHg，对应354.6/108.2g，恢复线随本档同步选择。 */
        {160U, 3585U, 1580U}, /* 160档：130/80mmHg，对应358.5/158.0g，高流量快降后立即使用本档阈值。 */
        {180U, 3624U, 2077U}, /* 180档：140/90mmHg，对应362.4/207.7g，保持区位于两线之间。 */
        {200U, 3662U, 2566U}, /* 200档：150/100mmHg，对应366.2/256.6g，达到启动线才执行快降。 */
        {240U, 3739U, 3545U}, /* 240档：170/120mmHg，对应373.9/354.5g，中间流量只对已换算的g值插值。 */
        {300U, 3854U, 3661U} /* 300档：200/150mmHg，对应385.4/366.1g，保留已确认的最高档目标。 */
    };
    uint32_t upper = 1U; /* 从第二个节点开始查找包围当前输出的相邻流量，节点按流量递增。 */
    uint32_t span; /* 相邻流量差，用于对0.1g阈值插值，不能用原用户设定代替当前输出。 */
    uint32_t offset; /* 当前输出距低端节点的流量差，降档后会在下一周期重新计算。 */

    *high_x10 = points[0][1]; /* 低于100时维持100mmHg首次启动线；已确认堵塞后的提示由进入5档统一触发。 */
    *recover_x10 = points[0][2]; /* 低流量恢复使用50mmHg对应值，不能沿用高流量恢复线而误加速。 */
    if (output_speed <= points[0][0]) /* 首节点以下不外推，避免低流量阈值变成零或负值。 */
    {
        return; /* 两个低流量阈值已写入，继续由状态机执行确认计时。 */
    }
    while ((upper < (sizeof(points) / sizeof(points[0]) - 1U)) && (output_speed > points[upper][0])) /* 逐段查找，超过最高档时停在末节点。 */
    {
        ++upper; /* 只移动表索引，不改变运行阶段、流量或报警状态。 */
    }
    if (output_speed >= points[upper][0]) /* 恰好命中节点或超过最高档时，直接使用对应离线值。 */
    {
        *high_x10 = points[upper][1]; /* 启动线不向表外外推，避免高设定导致保护阈值无限升高。 */
        *recover_x10 = points[upper][2]; /* 恢复线与启动线取自同一档，防止恢复条件高于降流条件。 */
        return; /* 完成节点查找，不再执行中间流量插值。 */
    }
    span = (uint32_t)points[upper][0] - points[upper - 1U][0]; /* 表内流量严格递增，跨度非零。 */
    offset = (uint32_t)output_speed - points[upper - 1U][0]; /* 当前输出已位于相邻节点之间，差值不会下溢。 */
    *high_x10 = points[upper - 1U][1] + (((uint32_t)(points[upper][1] - points[upper - 1U][1]) * offset + span - 1U) / span); /* 启动线向上取到0.1g，保持与离线表一致的取整方向。 */
    *recover_x10 = points[upper - 1U][2] + ((uint32_t)(points[upper][2] - points[upper - 1U][2]) * offset / span); /* 恢复线向下取到0.1g，不能因取整提前满足恢复条件。 */
}

/*
 * 函数功能：同一轮灌注超压过程只发一次提示，降流、低流量等待和恢复失败均共用该记录。
 * 输入参数：state为单路压力状态；now_ms为当前毫秒时刻；alarm_event接收本周期事件位。
 * 返回参数：无；只置提示事件，不修改流量和运行请求。
 */
static void PumpPressure_NotifyPourOnce(PumpPressureAlarmState_t *state, uint32_t now_ms, uint8_t *alarm_event)
{
    if (state->alarm_active == 0U) /* 本轮尚未提示才发送，避免在不同调流阶段反复打断操作。 */
    {
        state->alarm_active = 1U; /* 锁存本轮已提示状态；短暂低压、进入5档和恢复失败都不清除。 */
        state->last_alarm_tick_ms = now_ms; /* 记录首次提示时刻供诊断，不用于灌注定时重报。 */
        *alarm_event |= PUMP_PRESSURE_EVENT_ALARM; /* 上层仅蜂鸣和弹窗，不撤销运行请求。 */
    }
}

/*
 * 函数功能：一级降流仍未恢复时进入不高于5mL/min并提示一次；试升失败回到5等待，压力稳定恢复后可再次逐步升速。
 * 输入参数：state为单路状态；target_speed为用户设定；now_ms为当前时刻；alarm_event接收本轮低流量保护提示。
 * 返回参数：本次低流量目标，单位mL/min；不清除用户设定或运行请求。
 */
static uint16_t PumpPressure_EnterPourLow(PumpPressureAlarmState_t *state, uint16_t target_speed,
                                          uint32_t now_ms, uint8_t *alarm_event)
{
    uint16_t low_flow = (target_speed < PUMP_PRESSURE_POUR_LOW_FLOW_ML_MIN) ? target_speed : PUMP_PRESSURE_POUR_LOW_FLOW_ML_MIN; /* 用户设定低于5时不能被保护逻辑抬高。 */
    if (state->last_output_speed > low_flow) /* 只降低当前输出，原本更低的有效流量保持不变。 */
    {
        state->last_output_speed = low_flow; /* 后续真实超压仅提示，不能据此返回零速。 */
    }
    state->pour_phase = PUMP_PRESSURE_POUR_LOW; /* 每次退回5都进入低流量等待，压力重新稳定后仍允许自动恢复，不锁住后续升速。 */
    state->pour_adjust_tick_ms = now_ms; /* 从低流量命令下发时刻开始观察新的压力响应。 */
    state->condition_tick_ms = now_ms; /* 新的低流量阶段从当前时刻观察，后续试升不能借用上一档的异常持续时间。 */
    state->timing_active = 0U; /* 进入5时压力可能已低于100mmHg，不能把历史堵塞误标成当前仍连续超压。 */
    state->pour_low_timing_active = 0U; /* 进入低流量后必须重新取得稳定低压1秒，不能沿用之前的低谷。 */
    PumpPressure_NotifyPourOnce(state, now_ms, alarm_event); /* 进入5表明已确认的堵塞未在一级解除，即使减速后压力低于100mmHg也补发本轮一次提示。 */
    return state->last_output_speed; /* 继续灌注低流量，不产生检测失效提示事件。 */
}

/*
 * 函数功能：一级超压时降低一档；高流量先快速退出200以上区间，之后按20或10的小步长慢调。
 * 输入参数：state为单路状态；minimum_speed为本次一级工作下限；now_ms为当前时刻。
 * 返回参数：无；更新状态中的下发流量及本次调档时刻。
 */
static void PumpPressure_LowerPourFlow(PumpPressureAlarmState_t *state, uint16_t minimum_speed, uint32_t now_ms)
{
    uint16_t step; /* 当前流量对应的下降幅度，单位mL/min，不按原设定反复跳回快速档。 */
    if (state->last_output_speed >= PUMP_PRESSURE_POUR_FAST_FROM_ML_MIN) /* 只在实际下发目标仍处于高流量区时快速降档。 */
    {
        state->last_output_speed = PUMP_PRESSURE_POUR_FAST_FLOW_ML_MIN; /* 一次下发160，之后从160继续慢调。 */
    }
    else
    {
        step = (state->last_output_speed >= PUMP_PRESSURE_POUR_LARGE_STEP_FROM_ML_MIN) ? PUMP_PRESSURE_POUR_LARGE_STEP_ML_MIN : PUMP_PRESSURE_POUR_SMALL_STEP_ML_MIN; /* 100以上减20，低于100减10。 */
        if ((state->last_output_speed <= minimum_speed) || ((state->last_output_speed - minimum_speed) <= step)) /* 先检查剩余可降范围，防止越过一级下限或无符号下溢。 */
        {
            state->last_output_speed = minimum_speed; /* 到20或用户更低设定即保持，后续新压力决定是否进入二级。 */
        }
        else
        {
            state->last_output_speed = (uint16_t)(state->last_output_speed - step); /* 只减一档，不能因任务迟到一次补降多档。 */
        }
    }
    state->pour_adjust_tick_ms = now_ms; /* 下一档必须等待至少500ms以及新的压力反馈。 */
    state->pour_low_timing_active = 0U; /* 调档后的恢复确认使用新压力，不能承接之前的低压片段。 */
}

/*
 * 函数功能：压力稳定低于恢复线后只增加一档，并重新等待1秒，防止直接跳回大流量。
 * 输入参数：state为单路状态；target_speed为当前用户设定上限；now_ms为当前时刻。
 * 返回参数：无；更新下发目标，完成原设定后的稳定观察由调用者继续执行。
 */
static void PumpPressure_RaisePourFlow(PumpPressureAlarmState_t *state, uint16_t target_speed, uint32_t now_ms)
{
    uint16_t remaining = (uint16_t)(target_speed - state->last_output_speed); /* 调用前已限幅，剩余上升空间非负。 */
    state->last_output_speed = (remaining <= PUMP_PRESSURE_POUR_RECOVER_STEP_ML_MIN) ? target_speed : (uint16_t)(state->last_output_speed + PUMP_PRESSURE_POUR_RECOVER_STEP_ML_MIN); /* 每次最多增加20，为原步长的两倍，绝不超过用户设定。 */
    state->pour_low_tick_ms = now_ms; /* 增流后重新等待完整1秒的稳定低压，再决定下一档。 */
    state->pour_low_timing_active = 1U; /* 当前低压样本作为新观察起点，不在同一周期连续增流。 */
    state->pour_adjust_tick_ms = now_ms; /* 增流后若再次超压，一级细调仍遵守500ms反馈间隔。 */
}
#endif /* 灌注保护关闭时不编译其调流辅助函数。 */
#endif /* 压力总开关关闭时不编译压力判断辅助函数。 */

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
 * 函数功能：注水固定按800g、抽吸按原最低档提示超压；两者检测失效独立提示，持续超压仍每10秒重报。
 * 输入参数：state 为单路状态；target_speed 为业务设定；is_injection 为 1 表示注水、0 表示抽吸；weight_x10 单位 0.1g；threshold_g 为压力帧有效性字段；
 *           sequence 为压力帧序号；now_ms 为本周期 HAL 毫秒时刻。
 * 返回参数：超压返回ALARM事件，检测失效首次返回SENSOR_FAULT事件，其余返回0；不控制泵和手柄启停。
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

    if (target_speed == 0U) /* 注水和抽吸停止时结束本次检测失效记录，下一次运行重新判断。 */
    {
        PumpPressureControl_ResetAlarm(state); /* 停止状态不产生压力提示，保留原本停泵入口行为。 */
        return 0U; /* 无提示事件；本函数返回值不是流量命令。 */
    }
    if ((threshold_g == 0U) || (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10)) /* 注水、抽吸也必须报告明确无效或未就绪的压力数据。 */
    {
        return PumpPressure_ReportSensorFault(state, now_ms); /* 只报74号图窗，不改变调用方保留的泵速。 */
    }
    if ((state->sequence_valid != 0U) && (state->sensor_fault_active == 0U) &&
        ((uint32_t)(now_ms - state->last_sample_tick_ms) >= PUMP_PRESSURE_SAMPLE_TIMEOUT_MS)) /* 三类泵统一按500ms无新帧识别检测失败。 */
    {
        return PumpPressure_ReportSensorFault(state, now_ms); /* 持续失效不重报；迟到的新帧不能抹掉本次中断。 */
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
    state->sensor_fault_active = 0U; /* 通过有效性和新序号检查后解除检测失效锁存，下一次失效可再次提示。 */
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
        return 1U; /* 请求重新显示3秒弹窗并蜂鸣三声，不修改任何运行输出。 */
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
 * 函数功能：灌注按逐档g阈值确认超压，20档观察500ms仍未恢复则进入5并提示；试升后持续高于恢复线即退回5等待再次恢复，检测失效保持输出，停止请求优先。
 * 输入参数：state为单路独立状态；target_speed为用户设定mL/min；weight_x10为压力板0.1g读数；threshold_g只检查0表示无效。
 *           sequence为压力帧序号；now_ms为当前毫秒时刻；alarm_event接收本周期超压或检测失效提示事件，指针不能为NULL。
 * 返回参数：本周期应下发的流量目标；不修改用户设定，0仅表示用户停止请求。
 */
uint16_t PumpPressureControl_ApplyPour(PumpPressureAlarmState_t *state,
                                       uint16_t target_speed,
                                       uint32_t weight_x10,
                                       uint16_t threshold_g,
                                       uint8_t sequence,
                                       uint32_t now_ms,
                                       uint8_t *alarm_event)
{
    *alarm_event = 0U; /* 每周期重新建立事件，避免沿用上一周期超压或检测失效提示位。 */
#if ((PUMP_PRESSURE_CONTROL_ENABLE == 0U) || (PUMP_PRESSURE_POUR_PROTECTION_ENABLE == 0U))
    (void)weight_x10; /* 显式关闭保护时不读取压力值。 */
    (void)threshold_g; /* 显式关闭保护时不检查压力板状态。 */
    (void)sequence; /* 显式关闭保护时不建立新帧确认。 */
    (void)now_ms; /* 显式关闭保护时不推进调流计时。 */
    PumpPressureControl_ResetAlarm(state); /* 清除旧限流和提示锁存，关闭保护后仅按用户请求运行。 */
    return target_speed; /* 关闭开关的行为保持明确，不产生任何压力事件。 */
#else
    uint32_t sample_gap_ms; /* 距离上一有效新样本的间隔，用于识别500ms压力数据中断。 */
    uint16_t minimum_speed; /* 本轮一级工作下限，始终不高于当前用户设定。 */
    uint16_t previous_output; /* 记录用户改设定前的输出，用于识别需要重新观察压力响应的变化。 */
    uint32_t high_x10; /* 当前输出流量对应的启动线，单位0.1g，每次调档后重新查表。 */
    uint32_t recover_x10; /* 与当前输出配套的恢复线，保持区位于恢复线和启动线之间。 */
    uint8_t is_high; /* 当前新样本是否达到当前输出流量的启动线。 */
    uint8_t low_confirmed = 0U; /* 只有连续低于恢复线满1秒才允许本周期增流或完成本轮恢复。 */

    if (target_speed == 0U) /* 人工停止立即生效，不等待压力或任何调档计时。 */
    {
        PumpPressureControl_ResetAlarm(state); /* 人工停止结束本轮过程，下次启动允许重新调节和提示。 */
        return 0U; /* 零设定不经过5mL/min下限，直接停止输出。 */
    }
    if (state->mode != 3U) /* 新启动或切换为灌注时只初始化本通道。 */
    {
        PumpPressureControl_ResetAlarm(state); /* 清除另一泵用途留下的计时和提示状态。 */
        state->mode = 3U; /* 标记当前状态专用于灌注，后续调流不反复重置。 */
        state->target_speed = target_speed; /* 保存用户设定，保护过程只改变last_output_speed。 */
        state->last_output_speed = target_speed; /* 首次运行从用户设定建立输出，立即检查本周期压力有效性。 */
        state->pour_adjust_tick_ms = now_ms; /* 建立调档观察起点，后续慢调按实际时刻推进。 */
    }
    previous_output = state->last_output_speed; /* 后续判断用户是否实际改变了输出，而不因仅改设定清除本轮超压记录。 */
    if (state->target_speed != target_speed) /* 屏幕、脚踏和外控修改设定都保持同一压力过程。 */
    {
        if ((state->pour_phase == PUMP_PRESSURE_POUR_NORMAL) && (state->timing_active == 0U) && (state->pour_last_high == 0U) && (state->sensor_fault_active == 0U)) /* 正常且压力检测有效时允许用户调速；失效期间不能把保持输出抬高。 */
        {
            state->last_output_speed = target_speed; /* 用户主动调速继续生效，不把正常灌注限制在160。 */
        }
        else if ((state->pour_phase == PUMP_PRESSURE_POUR_NORMAL) && (target_speed > state->last_output_speed)) /* 正在确认高压时用户调高设定，先保留已输出的流量。 */
        {
            state->pour_phase = PUMP_PRESSURE_POUR_ADJUST; /* 后续须按低压确认小步增加，不能借设定变化直接跳高。 */
        }
        state->target_speed = target_speed; /* 只更新上限，不清本轮堵塞及已提示记录，后续仍按压力逐步恢复。 */
    }
    if (state->last_output_speed > target_speed) /* 用户调低设定立即限幅，任何保护阶段都不能超出用户要求。 */
    {
        state->last_output_speed = target_speed; /* 下调设定不等待500ms，避免继续输出过大的流量。 */
    }
    if (previous_output != state->last_output_speed) /* 用户主动改变输出后，旧低压不能证明新流量已经稳定。 */
    {
        state->pour_low_timing_active = 0U; /* 重新确认低压1秒，但保留本轮堵塞及提示锁存。 */
        state->pour_adjust_tick_ms = now_ms; /* 细调观察从本次新命令开始，避免同周期再补降一档。 */
    }
    PumpPressure_GetPourThresholds(state->last_output_speed, &high_x10, &recover_x10); /* 按已下发目标查表；降流后不能继续使用原设定的高门槛。 */
    state->alarm_threshold_g = (uint16_t)((high_x10 + 5U) / 10U); /* 诊断协议只显示整g；实际调流与恢复仍比较0.1g阈值。 */
    if ((threshold_g == 0U) || (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10)) /* 压力板明确无效或6000未就绪属于检测故障，不属于真实超压。 */
    {
        *alarm_event = PumpPressure_ReportSensorFault(state, now_ms); /* 包括5档在内仅产生独立74号提示，不清运行请求。 */
        return state->last_output_speed; /* 压力未知时保留当前输出，不把检测失败作为零速条件。 */
    }
    sample_gap_ms = (state->sequence_valid != 0U) ? (uint32_t)(now_ms - state->last_sample_tick_ms) : 0U; /* 无符号差支持毫秒计数回绕，首帧不补算历史时间。 */
    if ((state->sequence_valid != 0U) && (sample_gap_ms >= PUMP_PRESSURE_SAMPLE_TIMEOUT_MS) && (state->sensor_fault_active == 0U)) /* 首次断帧达到500ms只提示一次，迟到的新帧也不能掩盖已经发生的中断。 */
    {
        *alarm_event = PumpPressure_ReportSensorFault(state, now_ms); /* 500ms断帧按检测失败提示一次。 */
        return state->last_output_speed; /* 保留输出，后续收到有效新序号才恢复压力调节。 */
    }
    if ((state->sequence_valid != 0U) && (state->last_sequence == sequence)) /* 25ms任务重复读取同一压力帧时只保持输出。 */
    {
        return state->last_output_speed; /* 旧帧不能推进降流、恢复、提示或连续超压确认。 */
    }
    if (state->sensor_fault_active != 0U) /* 已通过有效性和新序号检查，才确认本通道压力检测恢复。 */
    {
        state->sensor_fault_active = 0U; /* 恢复后允许下一次独立检测失效提示，不清本轮超压报警记录。 */
        state->pour_adjust_tick_ms = now_ms; /* 新压力恢复后先观察完整调档间隔，不立即追赶失效期间的降档。 */
    }
    state->last_sequence = sequence; /* 保存本次新帧序号，255回到0也按新帧处理。 */
    state->sequence_valid = 1U; /* 首帧序号为0时也建立有效样本状态。 */
    state->last_sample_tick_ms = now_ms; /* 只有新有效帧刷新超时基准，旧值不能延长允许运行时间。 */
    is_high = (weight_x10 >= high_x10) ? 1U : 0U; /* 使用当前流量的离线g启动线，不执行mmHg曲线换算。 */
    if ((is_high != 0U) ||
        ((state->pour_phase == PUMP_PRESSURE_POUR_RECOVER) && (weight_x10 > recover_x10))) /* 首次降流仍按原分档启动线；试升必须保持在本档恢复线以下，进入保持区也算恢复失败候选。 */
    {
        if (state->timing_active == 0U) /* 从本次高压片段的首个新样本开始确认。 */
        {
            state->condition_tick_ms = now_ms; /* 从首次超压或试升后首次超过恢复线的新帧开始计时，不能沿用之前稳定低压的时间。 */
            state->timing_active = 1U; /* 单独标志允许计时从系统时刻0开始。 */
        }
        if ((is_high != 0U) && ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_POUR_HIGH_CONFIRM_MS)) /* 本轮堵塞记录仍须原启动线连续确认500ms，试升失败候选不改变正常高流量的启动标准。 */
        {
            state->pour_blocked_latched = 1U; /* 记住本轮已确认超压，降流后回到保持区也不能把堵塞当成解除。 */
        }
    }
    else
    {
        state->timing_active = 0U; /* 普通阶段未超压或试升已回到恢复线以下时，中断本段确认；本轮堵塞记录仍由稳定低压解除。 */
    }
    state->pour_last_high = is_high; /* 记录本帧分类，用户改设定时据此禁止绕过正在进行的超压确认。 */
    if (weight_x10 <= recover_x10) /* 当前档的恢复线低于启动线，防止降流后沿用过高恢复门槛而立即加速。 */
    {
        if (state->pour_low_timing_active == 0U) /* 重新进入低压区后必须等待完整1秒。 */
        {
            state->pour_low_tick_ms = now_ms; /* 记录低压片段起点，不沿用之前的短暂低谷。 */
            state->pour_low_timing_active = 1U; /* 标记本轮低压观察已开始。 */
        }
        low_confirmed = ((uint32_t)(now_ms - state->pour_low_tick_ms) >= PUMP_PRESSURE_POUR_RECOVER_CONFIRM_MS) ? 1U : 0U; /* 只在新帧上确认低压，旧帧不会触发增流。 */
        if (low_confirmed != 0U) /* 稳定低压1秒才解除本轮堵塞记录，单个压力低谷不能中止持续降档。 */
        {
            state->pour_blocked_latched = 0U; /* 连续低压已证明本轮压力恢复，解除保持区继续降档，后续允许小步增流。 */
        }
    }
    else
    {
        state->pour_low_timing_active = 0U; /* 压力进入保持区或高压区，取消尚未完成的增流确认。 */
    }
    minimum_speed = (target_speed < PUMP_PRESSURE_POUR_WORK_MIN_ML_MIN) ? target_speed : PUMP_PRESSURE_POUR_WORK_MIN_ML_MIN; /* 一级下限不能高于用户设定。 */
    if (state->pour_phase == PUMP_PRESSURE_POUR_LOW) /* 压力未稳定恢复时维持非零低流量，继续等待新压力决定是否升速。 */
    {
        if (low_confirmed != 0U) /* 每次压力重新稳定低于恢复线1秒都允许升速，不限制本轮恢复次数。 */
        {
            state->pour_phase = PUMP_PRESSURE_POUR_RECOVER; /* 开始逐档恢复；若压力再次升高则退回5等待，下一段稳定低压仍可恢复。 */
            if (state->last_output_speed < target_speed) /* 原设定已低于等于5时不抬高流量，继续原设定的稳定观察。 */
            {
                PumpPressure_RaisePourFlow(state, target_speed, now_ms); /* 首次只加20，后续每一档仍独立确认1秒，恢复斜率为原来的两倍。 */
            }
        }
        return state->last_output_speed; /* 低流量等待阶段保持当前非零目标，用户停止仍由函数入口立即处理。 */
    }
    if (state->pour_phase == PUMP_PRESSURE_POUR_RECOVER) /* 二级恢复单独观察压力，持续超过恢复线即退回5并重新确认低压。 */
    {
        if ((weight_x10 > recover_x10) && ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_POUR_RECOVER_HIGH_CONFIRM_MS)) /* 试升后持续500ms超过本档恢复线即判失败；25档使用50mmHg，不能在50至100mmHg之间长期保持。 */
        {
            return PumpPressure_EnterPourLow(state, target_speed, now_ms, alarm_event); /* 回到不高于5重新等待稳定低压，允许后续再次恢复，提示仍受同一轮一次限制。 */
        }
    }
    else if (state->pour_phase == PUMP_PRESSURE_POUR_NORMAL) /* 正常运行只在连续超压达到500ms后启动一级，过滤更短的压力波动。 */
    {
        if ((is_high != 0U) && ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_POUR_HIGH_CONFIRM_MS)) /* 瞬时压力升高不改变正常灌注。 */
        {
            state->pour_phase = PUMP_PRESSURE_POUR_ADJUST; /* 本轮开始动态调流，后续保持记忆的输出而不回跳用户设定。 */
            if (state->last_output_speed <= minimum_speed) /* 本来已经在一级工作下限，确认超压后无需再等待无效降档。 */
            {
                return PumpPressure_EnterPourLow(state, target_speed, now_ms, alarm_event); /* 直接进入非零低流量，不因超压停止。 */
            }
            PumpPressure_LowerPourFlow(state, minimum_speed, now_ms); /* 高流量快速降160，其余流量首次仅下降一个20或10的档位。 */
        }
        return state->last_output_speed; /* 正常或首个下降动作完成，后续新压力再决定下一档。 */
    }
    else if ((state->pour_phase == PUMP_PRESSURE_POUR_ADJUST) &&
             ((is_high != 0U) || (state->pour_blocked_latched != 0U)) &&
             (weight_x10 > recover_x10)) /* 本轮超压未解除时，保持区也每500ms继续降一档；落入低压区先等待稳定确认，不继续压低输出。 */
    {
        if ((state->last_output_speed <= minimum_speed) && /* 到20或用户更低设定后，本轮已确认且未恢复的超压也必须继续降到5。 */
             ((uint32_t)(now_ms - state->pour_adjust_tick_ms) >= PUMP_PRESSURE_POUR_ADJUST_MS) && /* 到档后仍观察完整500ms及新压力，不能借用上一档时长立即连降。 */
             ((state->pour_blocked_latched != 0U) ||
              ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_POUR_HIGH_CONFIRM_MS))) /* 保持区沿用本轮确认记录；未建立记录时仍要求连续超压500ms。 */
        {
            return PumpPressure_EnterPourLow(state, target_speed, now_ms, alarm_event); /* 升级仅降到5，不发送零速或清除请求。 */
        }
        if ((state->last_output_speed > minimum_speed) && ((uint32_t)(now_ms - state->pour_adjust_tick_ms) >= PUMP_PRESSURE_POUR_ADJUST_MS)) /* 高于下限才降一档；到20后只观察压力，不用空降档反复重置计时。 */
        {
            PumpPressure_LowerPourFlow(state, minimum_speed, now_ms); /* 只执行一次下降，迟到的新帧不能一次追赶多个档位。 */
        }
        return state->last_output_speed; /* 高压时只降流或保持，不进入自动增流。 */
    }
    if (low_confirmed != 0U) /* 一级或二级恢复只有稳定低压1秒后才能增加一档。 */
    {
        if (state->last_output_speed < target_speed) /* 仍低于用户设定时继续小步恢复，保持区不触发该动作。 */
        {
            PumpPressure_RaisePourFlow(state, target_speed, now_ms); /* 每次最多加20，并重新等待完整1秒，不能跳回用户高流量。 */
        }
        else
        {
            state->pour_phase = PUMP_PRESSURE_POUR_NORMAL; /* 原设定恢复后又稳定低压1秒，才结束整轮调流过程。 */
            state->alarm_active = 0U; /* 重新允许下一轮独立超压提示，短暂回落或中途试升不会清此锁存。 */
            state->pour_low_timing_active = 0U; /* 正常运行重新观察，不能把已完成的低压确认用于下一轮恢复。 */
        }
    }
    return state->last_output_speed; /* 未确认超压或尚未完成低压恢复时保持输出；已确认且未恢复的一级过程由上方继续降档。 */
#endif
}
