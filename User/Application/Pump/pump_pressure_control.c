#include "pump_pressure_control.h"

#if (PUMP_PRESSURE_CONTROL_ENABLE != 0U)
/*
 * 函数功能：把 g 单位压力阈值转换为 WeightX10 使用的 0.1g 单位。
 * 输入参数：pressure_g 为 g 单位压力阈值。
 * 返回参数：返回放大 10 倍后的 0.1g 单位压力阈值。
 */
static uint32_t PumpPressureControl_BuildPressureX10(uint16_t pressure_g)
{
    /* 将 g 单位阈值转换成 0.1g 单位，保证后续和压力模块 WeightX10 使用同一量纲比较。 */
    return (uint32_t)pressure_g * 10U;
}

/*
 * 函数功能：在两个实测泵速阈值点之间做线性插值。
 * 输入参数：target_speed 为当前目标泵速；speed_low/speed_high 为相邻泵速点；value_low/value_high 为对应压力阈值。
 * 返回参数：返回 target_speed 对应的 g 单位压力阈值。
 */
static uint16_t PumpPressureControl_InterpolateG(uint16_t target_speed,
                                                 uint16_t speed_low,
                                                 uint16_t speed_high,
                                                 uint16_t value_low,
                                                 uint16_t value_high)
{
    /* speed_span 保存两个标定泵速点之间的跨度，用于按当前目标泵速线性插值。 */
    uint32_t speed_span = (uint32_t)speed_high - (uint32_t)speed_low;
    /* speed_offset 保存当前泵速距离低速标定点的偏移量，决定阈值往高速端靠近多少。 */
    uint32_t speed_offset = (uint32_t)target_speed - (uint32_t)speed_low;
    /* value_span 保存两个压力阈值点之间的跨度，当前表格保持单调递增。 */
    uint32_t value_span = (uint32_t)value_high - (uint32_t)value_low;
    /* value_g 保存插值后的 g 单位压力阈值，使用 32 位避免乘法中间值溢出。 */
    uint32_t value_g;

    /* 如果两个标定泵速误配成相同值，直接使用低速阈值，避免除 0 影响泵任务。 */
    if (speed_span == 0U)
    {
        /* 返回低速端阈值，保持保护偏保守。 */
        return value_low;
    }

    /* 按线性插值计算阈值，speed_span / 2 用于四舍五入，减少整数除法系统性偏低。 */
    value_g = (uint32_t)value_low + (((value_span * speed_offset) + (speed_span / 2U)) / speed_span);
    /* 返回插值后的 g 单位阈值，供上层再转换成 WeightX10 的 0.1g 单位。 */
    return (uint16_t)value_g;
}

/*
 * 函数功能：根据当前目标泵速计算开始限速压力阈值。
 * 输入参数：target_speed 为当前准备输出的泵速，单位 ml/min。
 * 返回参数：返回 g 单位开始限速压力阈值。
 */
static uint16_t PumpPressureControl_BuildReduceThresholdG(uint16_t target_speed)
{
    /* 低于最小实测泵速时沿用 50 ml/min 阈值，避免低速堵管还按高阈值放行。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN)
    {
        /* 返回 50 ml/min 开始限速阈值。 */
        return PUMP_PRESSURE_CONTROL_REDUCE_50_G;
    }
    /* 50~110 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN)
    {
        /* 返回 50~110 ml/min 对应的开始限速阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_REDUCE_50_G,
                                                PUMP_PRESSURE_CONTROL_REDUCE_110_G);
    }
    /* 110~140 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN)
    {
        /* 返回 110~140 ml/min 对应的开始限速阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_REDUCE_110_G,
                                                PUMP_PRESSURE_CONTROL_REDUCE_140_G);
    }
    /* 140~200 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN)
    {
        /* 返回 140~200 ml/min 对应的开始限速阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_REDUCE_140_G,
                                                PUMP_PRESSURE_CONTROL_REDUCE_200_G);
    }
    /* 200~260 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN)
    {
        /* 返回 200~260 ml/min 对应的开始限速阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_REDUCE_200_G,
                                                PUMP_PRESSURE_CONTROL_REDUCE_260_G);
    }
    /* 260~300 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN)
    {
        /* 返回 260~300 ml/min 对应的开始限速阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_REDUCE_260_G,
                                                PUMP_PRESSURE_CONTROL_REDUCE_300_G);
    }
    /* 高于 300 ml/min 时沿用当前最高实测档位阈值，避免外推导致保护阈值继续抬高。 */
    return PUMP_PRESSURE_CONTROL_REDUCE_300_G;
}

/*
 * 函数功能：根据当前目标泵速计算输出压到 0 的停泵压力阈值。
 * 输入参数：target_speed 为当前准备输出的泵速，单位 ml/min。
 * 返回参数：返回 g 单位停泵压力阈值。
 */
static uint16_t PumpPressureControl_BuildStopThresholdG(uint16_t target_speed)
{
    /* 低于最小实测泵速时沿用 50 ml/min 停泵阈值，避免低速堵管不能停泵。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN)
    {
        /* 返回 50 ml/min 输出压到 0 的阈值。 */
        return PUMP_PRESSURE_CONTROL_STOP_50_G;
    }
    /* 50~110 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN)
    {
        /* 返回 50~110 ml/min 对应的停泵阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_50_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_STOP_50_G,
                                                PUMP_PRESSURE_CONTROL_STOP_110_G);
    }
    /* 110~140 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN)
    {
        /* 返回 110~140 ml/min 对应的停泵阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_110_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_STOP_110_G,
                                                PUMP_PRESSURE_CONTROL_STOP_140_G);
    }
    /* 140~200 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN)
    {
        /* 返回 140~200 ml/min 对应的停泵阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_140_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_STOP_140_G,
                                                PUMP_PRESSURE_CONTROL_STOP_200_G);
    }
    /* 200~260 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN)
    {
        /* 返回 200~260 ml/min 对应的停泵阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_200_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_STOP_200_G,
                                                PUMP_PRESSURE_CONTROL_STOP_260_G);
    }
    /* 260~300 ml/min 之间按实测点线性插值。 */
    if (target_speed <= PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN)
    {
        /* 返回 260~300 ml/min 对应的停泵阈值。 */
        return PumpPressureControl_InterpolateG(target_speed,
                                                PUMP_PRESSURE_CONTROL_SPEED_260_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_SPEED_300_ML_MIN,
                                                PUMP_PRESSURE_CONTROL_STOP_260_G,
                                                PUMP_PRESSURE_CONTROL_STOP_300_G);
    }
    /* 高于 300 ml/min 时沿用当前最高实测档位阈值，避免外推导致停泵压力继续升高。 */
    return PUMP_PRESSURE_CONTROL_STOP_300_G;
}
#endif

/*
 * 函数功能：判断压力是否达到无泵速参数接口下的绝对兜底停泵阈值。
 * 输入参数：weight_x10 为压力模块上报重量，单位 0.1g；threshold_g 仅用于判断压力上报阈值是否有效。
 * 返回参数：返回 1 表示需要进入兜底硬停泵路径，返回 0 表示未达到兜底硬停泵条件。
 */
uint8_t PumpPressureControl_ShouldForceStop(uint32_t weight_x10, uint16_t threshold_g)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE == 0U)
    /* 闭环总开关关闭时不读取重量值，保证关闭后不会误改运行状态。 */
    (void)weight_x10;
    /* 闭环总开关关闭时不读取阈值，避免未使用参数告警。 */
    (void)threshold_g;
    /* 返回 0 表示不触发硬停泵，保持原有调试/生产路径。 */
    return 0U;
#else
    /* stop_x10 保存绝对硬停阈值，当前接口没有泵速参数，因此使用最高实测档 300 ml/min 的停泵阈值兜底。 */
    uint32_t stop_x10;

    /* 阈值为 0 表示压力模块尚未给出有效保护阈值，此时不能用 0 阈值误触发停泵。 */
    if (threshold_g == 0U)
    {
        /* 无有效阈值时只允许上层保持原输出，不做硬停泵锁存。 */
        return 0U;
    }

    /* 把最高档停泵阈值转换为 0.1g，作为无泵速参数接口的兜底硬停锁存点。 */
    stop_x10 = PumpPressureControl_BuildPressureX10(PUMP_PRESSURE_CONTROL_STOP_300_G);

    /* 当前重量达到或超过绝对兜底停泵阈值时返回 1，调用方据此暂停输出并下发 0 速帧。 */
    if (weight_x10 >= stop_x10)
    {
        /* 返回 1 明确告诉泵任务进入硬停泵路径。 */
        return 1U;
    }

    /* 未达到硬停泵点时返回 0，调用方继续使用线性限速结果。 */
    return 0U;
#endif
}

/*
 * 函数功能：根据当前泵速对应压力阈值，对目标泵速做线性限速或停泵输出。
 * 输入参数：target_speed 为上层目标泵速；weight_x10 为压力模块上报重量，单位 0.1g；threshold_g 仅用于判断压力上报阈值是否有效。
 * 返回参数：返回压力闭环处理后的泵速，0 表示本周期停止输出。
 */
uint16_t PumpPressureControl_Apply(uint16_t target_speed, uint32_t weight_x10, uint16_t threshold_g)
{
#if (PUMP_PRESSURE_CONTROL_ENABLE == 0U)
    /* 闭环关闭时不读取压力字段，保证编译宏关闭后输出路径完全等同原逻辑。 */
    (void)weight_x10;
    /* 闭环关闭时不读取阈值字段，避免未使用参数告警。 */
    (void)threshold_g;
    /* 返回上层目标泵速，等同于不做压力保护。 */
    return target_speed;
#else
    /* reduce_x10 表示当前泵速下开始限速的压力阈值，单位 0.1g。 */
    uint32_t reduce_x10;
    /* stop_x10 表示当前泵速下输出压到 0 的压力阈值，单位 0.1g。 */
    uint32_t stop_x10;
    /* available_margin_x10 表示开始限速点到停泵点之间可用于线性减速的总区间。 */
    uint32_t available_margin_x10;
    /* remaining_margin_x10 表示当前压力距离停泵点还剩多少 0.1g 区间。 */
    uint32_t remaining_margin_x10;
    /* scaled_speed 使用 64 位保存乘法中间值，避免 target_speed 和区间相乘时溢出 32 位。 */
    uint64_t scaled_speed;

    /* 目标泵速本来就是 0 时不需要进入闭环计算，直接保持停止输出。 */
    if (target_speed == 0U)
    {
        /* 返回 0，避免后续阈值计算影响停泵状态。 */
        return 0U;
    }

    /* 阈值为 0 表示压力模块尚未给出有效保护阈值，此时保持原泵速，避免误停泵。 */
    if (threshold_g == 0U)
    {
        /* 返回上层目标泵速，等待后续有效 ThresholdG 上报后再进入闭环。 */
        return target_speed;
    }

    /* 根据当前目标泵速查表并插值得到开始限速阈值，替代原来单一 ThresholdG。 */
    reduce_x10 = PumpPressureControl_BuildPressureX10(PumpPressureControl_BuildReduceThresholdG(target_speed));
    /* 根据当前目标泵速查表并插值得到停泵阈值，压力达到后本周期输出压到 0。 */
    stop_x10 = PumpPressureControl_BuildPressureX10(PumpPressureControl_BuildStopThresholdG(target_speed));

    /* 当前压力没有超过阈值时不降速，保持用户设定的目标泵速。 */
    if (weight_x10 <= reduce_x10)
    {
        /* 返回未调整泵速，保证阈值以下不影响正常注水/抽水/灌注。 */
        return target_speed;
    }

    /* 当前压力达到或超过当前泵速对应的停泵阈值时，立即把输出泵速压到 0。 */
    if (weight_x10 >= stop_x10)
    {
        /* 返回 0 表示本周期停止泵转动；run_flag 保持原样，便于压力恢复后自动继续闭环输出。 */
        return 0U;
    }

    /* 如果停泵点误配置得不高于限速点，则超过限速点后直接输出 0，避免除法区间异常。 */
    if (stop_x10 <= reduce_x10)
    {
        /* 返回 0 表示进入保守保护，防止错误宏配置导致压力越高反而继续输出。 */
        return 0U;
    }

    /* 计算线性减速区间总宽度，区间越窄限速越陡。 */
    available_margin_x10 = stop_x10 - reduce_x10;
    /* 理论上上方已经保证不为 0，这里仍做防御，避免异常参数导致除 0。 */
    if (available_margin_x10 == 0U)
    {
        /* 无可用区间时采用最保守策略，直接停止泵输出。 */
        return 0U;
    }

    /* 计算当前压力到停止点的剩余区间，越接近停止点，该值越小。 */
    remaining_margin_x10 = stop_x10 - weight_x10;
    /* 按剩余区间比例缩放目标泵速：限速阈值处 100%，停泵阈值处 0%。 */
    scaled_speed = (uint64_t)target_speed * (uint64_t)remaining_margin_x10;
    /* 除以总区间得到闭环后的泵速，整数除法自动向下取整，避免超出安全侧。 */
    return (uint16_t)(scaled_speed / available_margin_x10);
#endif
}
