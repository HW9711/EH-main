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

#if (PUMP_PRESSURE_POUR_VALVE_TEST_ENABLE == 0U)
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

#endif /* 原灌注减速算法只在阀门测试关闭时编译。 */

#if (PUMP_PRESSURE_POUR_VALVE_TEST_ENABLE != 0U)
/*
 * 函数功能：结束本次灌注自动恢复过程，持续返回零速并要求上层撤销本泵运行请求。
 * 输入参数：state为本泵独立压力状态；alarm_event接收事件位，保留调用前已经置位的报警事件。
 * 返回参数：固定返回0，表示本周期下发零流量；不代表已经取得驱动停转反馈。
 */
static uint16_t PumpPressure_StopValveTest(PumpPressureAlarmState_t *state, uint8_t *alarm_event)
{
    state->pour_test_phase = PUMP_PRESSURE_TEST_STOPPED; /* 进入彻底停机阶段，后续低压不能恢复本次运行。 */
    state->last_output_speed = 0U; /* 重复帧或后续有效帧都继续返回零速。 */
    state->alarm_active = 1U; /* 诊断记录本泵已被压力处理停止，人工重新启动前不解除。 */
    state->timing_active = 0U; /* 结束原始压力的连续超压计时。 */
    state->pour_test_drop_seen = 0U; /* 清掉本次降压记录，不能留给下一次人工启动使用。 */
    state->pour_test_low_active = 0U; /* 清掉未完成的低压确认。 */
    *alarm_event |= PUMP_PRESSURE_EVENT_FINAL_STOP; /* 要求上层清除本机及外控请求，不能只发送零速而保留旧启动。 */
    return 0U; /* 零流量由原泵任务换算并发送，不在本函数内访问驱动串口。 */
}

/*
 * 函数功能：对探测阶段最近5份新压力取中值，抑制单个脉动对开阀判断的影响。
 * 输入参数：state保存本泵中值窗口；weight_x10为新帧压力，单位0.1g；调用前须已排除重复或失效帧。
 * 返回参数：窗口已有样本的中值，单位0.1g；上层还须等raw_count满5后才能判断开阀。
 */
static uint32_t PumpPressure_FilterValveTest(PumpPressureAlarmState_t *state, uint32_t weight_x10)
{
    uint32_t sorted[PUMP_PRESSURE_TEST_FILTER_COUNT]; /* 只排序局部副本，不破坏按时间轮换的原始压力窗口。 */
    uint32_t value; /* 插入排序时保存当前压力，单位0.1g。 */
    uint8_t i; /* 遍历本泵已经收到的有效窗口样本。 */
    uint8_t j; /* 在已排序部分向前寻找当前压力的位置。 */

    state->pour_test_raw[state->pour_test_raw_index] = weight_x10; /* 用新压力覆盖窗口最旧位置。 */
    state->pour_test_raw_index = (uint8_t)((state->pour_test_raw_index + 1U) % PUMP_PRESSURE_TEST_FILTER_COUNT); /* 下次写入下一个位置，保持窗口最多5份。 */
    if (state->pour_test_raw_count < PUMP_PRESSURE_TEST_FILTER_COUNT)
    {
        state->pour_test_raw_count++; /* 首次建立窗口时只统计实际收到的样本，不把初始零值当压力。 */
    }
    for (i = 0U; i < state->pour_test_raw_count; ++i)
    {
        value = state->pour_test_raw[i]; /* 复制本次待排序压力，不修改跨周期保存的原值。 */
        j = i; /* 当前压力先从已排序数据末端开始比较。 */
        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U]; /* 把较大的压力后移，留出当前压力的插入位置。 */
            --j; /* 继续比较前一个压力，最多移动4次。 */
        }
        sorted[j] = value; /* 插入后局部副本继续按压力从小到大排列。 */
    }
    return sorted[state->pour_test_raw_count / 2U]; /* 满5份时取中间第3份，单个峰值或谷值不会直接决定开阀。 */
}

/*
 * 函数功能：在固定20mL/min阶段，用约1秒下降至少一半及随后连续低压判断是否疑似开阀。
 * 输入参数：state为本泵状态；weight_x10为新帧压力，单位0.1g；now_ms为当前毫秒数，允许自然回绕。
 * 返回参数：1表示本次已完成候选开阀确认；0表示继续探测，不能据此声称阀门机械位置已经确认。
 */
static uint8_t PumpPressure_DetectValveOpen(PumpPressureAlarmState_t *state, uint32_t weight_x10, uint32_t now_ms)
{
    uint32_t filtered; /* 当前5点中值压力，单位0.1g。 */
    uint32_t age; /* 当前时刻距某个历史压力点的间隔，单位ms。 */
    uint32_t best_age = PUMP_PRESSURE_TEST_DROP_WINDOW_MS + PUMP_PRESSURE_TEST_DROP_TOLERANCE_MS + 1U; /* 超出允许范围的初值表示尚未找到约1秒前的压力。 */
    uint32_t previous = 0U; /* 保存最接近1秒前的滤波压力，单位0.1g；只在找到合格时间点后使用。 */
    uint8_t i; /* 遍历本泵最多16个历史点。 */
    uint8_t latest; /* 本泵历史环形缓存中最近写入的位置。 */

    if ((uint32_t)(now_ms - state->pour_test_probe_tick_ms) < PUMP_PRESSURE_TEST_SETTLE_MS)
    {
        return 0U; /* 探测启动后的前2秒不保存开阀样本，避免沿用停泵期间的降压曲线。 */
    }
    filtered = PumpPressure_FilterValveTest(state, weight_x10); /* 只有有效新帧进入中值窗口，不按25ms任务次数重复采样。 */
    if (state->pour_test_raw_count < PUMP_PRESSURE_TEST_FILTER_COUNT)
    {
        return 0U; /* 样本不足5份时继续20mL/min，不用尚未形成的中值确认开阀。 */
    }
    for (i = 0U; i < state->pour_test_history_count; ++i)
    {
        age = (uint32_t)(now_ms - state->pour_test_history_tick[i]); /* 用无符号时间差支持HAL毫秒计数自然回绕。 */
        if ((age >= PUMP_PRESSURE_TEST_DROP_WINDOW_MS) && (age < best_age))
        {
            best_age = age; /* 选取1.0到1.2秒内、最接近1秒的历史点，不使用更早的高压力。 */
            previous = state->pour_test_history[i]; /* 保存同一个历史时刻的压力，用于计算下降比例。 */
        }
    }
    if ((best_age <= (PUMP_PRESSURE_TEST_DROP_WINDOW_MS + PUMP_PRESSURE_TEST_DROP_TOLERANCE_MS)) &&
        ((uint64_t)previous * PUMP_PRESSURE_TEST_MMHG_PER_G_X1M >= (uint64_t)PUMP_PRESSURE_TEST_DROP_START_MMHG * PUMP_PRESSURE_TEST_MMHG_SCALE) &&
        ((uint64_t)filtered * 100U <= (uint64_t)previous * PUMP_PRESSURE_TEST_DROP_REMAIN_PERCENT))
    {
        state->pour_test_drop_seen = 1U; /* 约1秒前至少20mmHg且本次已下降一半，允许继续检查是否稳定低压。 */
        state->pour_test_drop_tick_ms = now_ms; /* 只让近期降压参与后续确认，不能永久保留一次降压记录。 */
    }
    if ((state->pour_test_drop_seen != 0U) &&
        ((uint32_t)(now_ms - state->pour_test_drop_tick_ms) > PUMP_PRESSURE_TEST_DROP_VALID_MS))
    {
        state->pour_test_drop_seen = 0U; /* 快速降压超过2秒仍未完成低压确认，本次不再据此恢复。 */
        state->pour_test_low_active = 0U; /* 后续必须重新出现明显降压，并重新连续低压1秒。 */
    }
    latest = (uint8_t)((state->pour_test_history_index + PUMP_PRESSURE_TEST_HISTORY_COUNT - 1U) % PUMP_PRESSURE_TEST_HISTORY_COUNT); /* 找到最近历史点，首次为空时下方不会读取它。 */
    if ((state->pour_test_history_count == 0U) ||
        ((uint32_t)(now_ms - state->pour_test_history_tick[latest]) >= PUMP_PRESSURE_TEST_HISTORY_MIN_MS))
    {
        state->pour_test_history[state->pour_test_history_index] = filtered; /* 保存当前滤波压力，供后续约1秒下降比例比较。 */
        state->pour_test_history_tick[state->pour_test_history_index] = now_ms; /* 同下标保存采样时间，不能把不等间隔样本当作固定周期。 */
        state->pour_test_history_index = (uint8_t)((state->pour_test_history_index + 1U) % PUMP_PRESSURE_TEST_HISTORY_COUNT); /* 下次覆盖最旧历史点。 */
        if (state->pour_test_history_count < PUMP_PRESSURE_TEST_HISTORY_COUNT)
        {
            state->pour_test_history_count++; /* 未填满时只遍历实际保存过的历史点。 */
        }
    }
    if ((state->pour_test_drop_seen == 0U) ||
        ((uint64_t)filtered * PUMP_PRESSURE_TEST_MMHG_PER_G_X1M >= (uint64_t)PUMP_PRESSURE_TEST_OPEN_MMHG * PUMP_PRESSURE_TEST_MMHG_SCALE))
    {
        state->pour_test_low_active = 0U; /* 没有近期快速降压或压力回到10mmHg及以上，取消连续低压确认。 */
        return 0U; /* 保持20mL/min探测，不能把普通波动直接当成开阀。 */
    }
    if (state->pour_test_low_active == 0U)
    {
        state->pour_test_low_tick_ms = now_ms; /* 从第一份符合条件的低压中值开始计时。 */
        state->pour_test_low_active = 1U; /* 后续每份有效新帧都须继续低于10mmHg。 */
    }
    return ((uint32_t)(now_ms - state->pour_test_low_tick_ms) >= PUMP_PRESSURE_TEST_OPEN_CONFIRM_MS) ? 1U : 0U; /* 连续低压满1秒才允许上层恢复原流量。 */
}

/*
 * 函数功能：执行灌注固定压力测试：超过100mmHg连续500ms停泵报警，低于100mmHg后以20mL/min探测最多15秒。
 * 输入参数：state为本泵状态；target_speed为当前用户流量，单位mL/min；weight_x10为0.1g压力；threshold_g只用0值判断帧无效；
 *           sequence为压力序号；now_ms为毫秒计时；alarm_event接收报警、彻底停机或恢复原流量事件位。
 * 返回参数：本周期下发流量；确认开阀返回超压时保存的原流量，超时或压力数据失效返回0并要求撤销运行请求。
 */
static uint16_t PumpPressure_ApplyValveTest(PumpPressureAlarmState_t *state, uint16_t target_speed,
                                           uint32_t weight_x10, uint16_t threshold_g, uint8_t sequence,
                                           uint32_t now_ms, uint8_t *alarm_event)
{
    uint64_t pressure_scaled; /* 0.1g读数乘换算系数，与放大10000000倍的mmHg阈值比较，避免浮点和阈值取整。 */

    *alarm_event = 0U; /* 每周期重新生成事件，重复帧不会重复请求蜂鸣。 */
    if (state->mode != 3U)
    {
        PumpPressureControl_ResetAlarm(state); /* 切换到灌注用途时丢弃另一种泵的压力状态。 */
        state->mode = 3U; /* 后续周期继续使用本泵独立的灌注测试过程。 */
    }
    if (target_speed == 0U)
    {
        PumpPressureControl_ResetAlarm(state); /* 人工把流量设为0立即结束测试，下次启动重新确认压力。 */
        return 0U; /* 用户停止优先于探测和自动恢复。 */
    }
    if (state->pour_test_phase == PUMP_PRESSURE_TEST_STOPPED)
    {
        return PumpPressure_StopValveTest(state, alarm_event); /* 请求尚未被上层撤销时继续要求停机，压力下降不能解除。 */
    }
    if ((state->pour_test_phase == PUMP_PRESSURE_TEST_PROBE) &&
        ((uint32_t)(now_ms - state->pour_test_probe_tick_ms) >= PUMP_PRESSURE_TEST_PROBE_MS))
    {
        return PumpPressure_StopValveTest(state, alarm_event); /* 先检查固定15秒期限，重复帧、调速及临界开阀都不能延长本次探测。 */
    }
    if ((threshold_g == 0U) || (weight_x10 == PUMP_PRESSURE_CONTROL_NOT_READY_X10))
    {
        return PumpPressure_StopValveTest(state, alarm_event); /* 压力无效或压力板未就绪时结束自动恢复，不能在未知压力下继续20mL/min。 */
    }
    if ((state->sequence_valid != 0U) &&
        ((uint32_t)(now_ms - state->last_sample_tick_ms) >= PUMP_PRESSURE_ALARM_MAX_SAMPLE_GAP_MS))
    {
        return PumpPressure_StopValveTest(state, alarm_event); /* 压力断帧达到1秒即停机；迟到的新帧也不能补算超压或开阀持续时间。 */
    }
    if ((state->sequence_valid != 0U) && (state->last_sequence == sequence))
    {
        return state->last_output_speed; /* 旧帧只能保持上次输出，不能触发低压探测或恢复。 */
    }
    state->last_sequence = sequence; /* 本次新序号只处理一次，255回到0仍是新帧。 */
    state->sequence_valid = 1U; /* 允许本次运行第一份有效帧的序号就是0。 */
    state->last_sample_tick_ms = now_ms; /* 保存最后有效新压力时刻，后续检查断帧。 */
    state->alarm_threshold_g = (uint16_t)(((uint64_t)PUMP_PRESSURE_TEST_STOP_MMHG * PUMP_PRESSURE_TEST_MMHG_SCALE / PUMP_PRESSURE_TEST_MMHG_PER_G_X1M + 5U) / 10U); /* 整g诊断字段约为191；实际比较仍使用100mmHg，不使用这个取整值。 */
    pressure_scaled = (uint64_t)weight_x10 * PUMP_PRESSURE_TEST_MMHG_PER_G_X1M; /* 将当前原始压力换到统一整数倍率，原始压力字段保持不变。 */
    if (state->pour_test_phase == PUMP_PRESSURE_TEST_NORMAL)
    {
        state->target_speed = target_speed; /* 正常阶段使用用户当前设定，但设定变化不打断固定100mmHg超压计时。 */
        state->last_output_speed = target_speed; /* 试验阶段不执行旧阈值减速，未确认超压时保持正常流量。 */
    }
    if ((state->pour_test_phase != PUMP_PRESSURE_TEST_WAIT_LOW) &&
        (pressure_scaled > (uint64_t)PUMP_PRESSURE_TEST_STOP_MMHG * PUMP_PRESSURE_TEST_MMHG_SCALE))
    {
        if (state->timing_active == 0U)
        {
            state->condition_tick_ms = now_ms; /* 从第一份严格高于100mmHg的新帧开始确认。 */
            state->timing_active = 1U; /* 下一份新帧仍超压才继续计时，短尖峰不会立即停机。 */
        }
        if ((uint32_t)(now_ms - state->condition_tick_ms) >= PUMP_PRESSURE_TEST_STOP_CONFIRM_MS)
        {
            *alarm_event = PUMP_PRESSURE_EVENT_ALARM; /* 连续超压已满500ms，本周期请求原有蜂鸣和压力弹窗。 */
            state->last_alarm_tick_ms = now_ms; /* 记录本次超压停泵时刻，供诊断核对。 */
            if (state->pour_test_phase == PUMP_PRESSURE_TEST_PROBE)
            {
                return PumpPressure_StopValveTest(state, alarm_event); /* 20mL/min仍持续超压时结束测试，不再启动另一个15秒循环。 */
            }
            state->pour_test_resume_flow = target_speed; /* 保存本次超压时的原流量，后续固定20期间不覆盖。 */
            state->pour_test_phase = PUMP_PRESSURE_TEST_WAIT_LOW; /* 初次超压先保持零速，等待新压力低于100mmHg。 */
            state->alarm_active = 1U; /* 标记本泵已经因压力停转，等待期间不重复蜂鸣。 */
            state->timing_active = 0U; /* 初次超压确认结束，探测若再次超压必须重新确认满500ms。 */
            state->last_output_speed = 0U; /* 本周期立即向泵任务返回零流量。 */
            return 0U; /* 尚未进入20mL/min阶段，不在报警当次继续运行。 */
        }
    }
    else
    {
        state->timing_active = 0U; /* 压力回到100mmHg及以下或处于零速等待时，取消尚未完成的超压确认。 */
    }
    if (state->pour_test_phase == PUMP_PRESSURE_TEST_WAIT_LOW)
    {
        if (pressure_scaled < (uint64_t)PUMP_PRESSURE_TEST_PROBE_START_MMHG * PUMP_PRESSURE_TEST_MMHG_SCALE)
        {
            state->pour_test_phase = PUMP_PRESSURE_TEST_PROBE; /* 有效新压力已低于100mmHg，本周期开始固定20mL/min探测。 */
            state->pour_test_probe_tick_ms = now_ms; /* 15秒期限只在此处设置一次，后续压力升降不会重开计时。 */
            state->pour_test_raw_count = 0U; /* 清掉上次测试的中值窗口有效数量，旧数组内容不再参与判断。 */
            state->pour_test_raw_index = 0U; /* 本次探测从中值窗口首位置重新采样。 */
            state->pour_test_history_count = 0U; /* 禁止把停泵期间的降压当成20mL/min开阀证据。 */
            state->pour_test_history_index = 0U; /* 新探测的带时间压力历史从首位置开始。 */
            state->pour_test_drop_seen = 0U; /* 本次还没有观察到稳定运行后的快速降压。 */
            state->pour_test_low_active = 0U; /* 本次低压确认必须使用后续新帧重新开始。 */
            state->last_output_speed = PUMP_PRESSURE_TEST_PROBE_FLOW; /* 只改变实际下发流量，保留超压前的原流量。 */
        }
        return state->last_output_speed; /* 未到100mmHg以下继续零速；刚进入探测时返回20。 */
    }
    if ((state->pour_test_phase == PUMP_PRESSURE_TEST_PROBE) &&
        (PumpPressure_DetectValveOpen(state, weight_x10, now_ms) != 0U))
    {
        state->last_output_speed = state->pour_test_resume_flow; /* 候选开阀条件成立，本周期恢复超压时保存的原流量。 */
        state->target_speed = state->pour_test_resume_flow; /* 同步压力诊断中的原流量，不能用探测20覆盖原设定。 */
        state->pour_test_phase = PUMP_PRESSURE_TEST_NORMAL; /* 本次探测结束，恢复后的新超压再按超过100mmHg连续500ms处理。 */
        state->alarm_active = 0U; /* 本泵已完成本次自动恢复，解除压力停转记录。 */
        state->timing_active = 0U; /* 不把探测期间未完成的超压计时带入正常运行。 */
        state->pour_test_drop_seen = 0U; /* 本次降压已用于恢复，下次必须重新观察。 */
        state->pour_test_low_active = 0U; /* 结束本次连续低压确认。 */
        *alarm_event = PUMP_PRESSURE_EVENT_RESTORE_FLOW; /* 通知上层同步原设定，不再次蜂鸣或弹窗。 */
    }
    return state->last_output_speed; /* 正常返回用户流量，探测返回20，零速等待已在前面的分支返回。 */
}
#endif

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
 * 函数功能：阀门测试开启时按超过100mmHg持续500ms停泵、低于100mmHg开始探测；关闭测试时继续原来的平均压力减速和恢复。
 * 输入参数：state 为本泵独立状态且不能为NULL；target_speed 为用户设定流量，单位mL/min；weight_x10 为压力板0.1g读数。
 *           threshold_g为压力帧有效性字段；sequence为压力帧序号；now_ms为主控毫秒时刻；alarm_event返回PUMP_PRESSURE_EVENT_*事件位。
 * 返回参数：返回本周期应下发的流量，单位mL/min；0为零速请求，FINAL_STOP事件要求上层撤销本机和外控运行请求。
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
#elif (PUMP_PRESSURE_POUR_VALVE_TEST_ENABLE != 0U)
    return PumpPressure_ApplyValveTest(state, target_speed, weight_x10, threshold_g, sequence, now_ms, alarm_event); /* 测试开启时完全跳过旧流量阈值和减速恢复逻辑。 */
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
