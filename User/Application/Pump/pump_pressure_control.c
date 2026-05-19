#include "pump_pressure_control.h"

#if (PUMP_PRESSURE_CONTROL_ENABLE != 0U)
static uint32_t PumpPressureControl_BuildThresholdX10(uint16_t threshold_g)
{
    /* 将压力阈值从 g 转成 0.1g 单位，确保后续比较和 WeightX10 使用同一量纲。 */
    return (uint32_t)threshold_g * 10U;
}

static uint32_t PumpPressureControl_BuildStopX10(uint32_t threshold_x10)
{
#if (PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN == 0U)
    /* 分母误配置为 0 时不能做除法，退化为 1.0 倍阈值保护，保证编译和运行都不会除 0。 */
    return threshold_x10;
#else
    /* stop_x10 按宏配置的硬停倍率计算，默认 3/2 即 1.5 倍阈值，使用 64 位避免乘法溢出。 */
    uint64_t stop_x10 = (uint64_t)threshold_x10 * (uint64_t)PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM;
    /* 整数除法向下取整，和原先 threshold + threshold / 2 的 1.5 倍算法保持同样的保守侧行为。 */
    return (uint32_t)(stop_x10 / (uint64_t)PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN);
#endif
}
#endif

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
    /* threshold_x10 保存放大 10 倍后的阈值，避免每个调用点重复写单位换算。 */
    uint32_t threshold_x10;
    /* stop_x10 保存硬停倍率对应的停泵边界，默认是 1.5 倍阈值。 */
    uint32_t stop_x10;

    /* 阈值为 0 表示压力模块尚未给出有效保护阈值，此时不能用 0 阈值误触发停泵。 */
    if (threshold_g == 0U)
    {
        /* 无有效阈值时只允许上层保持原输出，不做硬停泵锁存。 */
        return 0U;
    }

    /* 把 g 阈值转换为 0.1g，和压力 MCU 上报的 WeightX10 对齐。 */
    threshold_x10 = PumpPressureControl_BuildThresholdX10(threshold_g);
    /* 根据阈值和宏配置倍率计算硬停点，超过该点后本周期泵速必须压到 0。 */
    stop_x10 = PumpPressureControl_BuildStopX10(threshold_x10);

    /* 当前重量达到或超过硬停倍率时返回 1，调用方据此暂停输出并下发 0 速帧。 */
    if (weight_x10 >= stop_x10)
    {
        /* 返回 1 明确告诉泵任务进入硬停泵路径。 */
        return 1U;
    }

    /* 未达到硬停泵点时返回 0，调用方继续使用线性限速结果。 */
    return 0U;
#endif
}

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
    /* threshold_x10 把 g 阈值转换成 0.1g 单位，和 pressure MCU 上报的 WeightX10 保持同一量纲。 */
    uint32_t threshold_x10;
    /* stop_x10 表示硬停倍率阈值，压力达到或超过该点时输出泵速直接变成 0。 */
    uint32_t stop_x10;
    /* available_margin_x10 表示从 1.0 倍阈值到硬停倍率之间可用于线性减速的总区间。 */
    uint32_t available_margin_x10;
    /* remaining_margin_x10 表示当前压力距离硬停倍率停止点还剩多少 0.1g 区间。 */
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

    /* 将 ThresholdG 从 g 放大 10 倍，得到和 WeightX10 相同的 0.1g 单位。 */
    threshold_x10 = PumpPressureControl_BuildThresholdX10(threshold_g);
    /* 计算宏配置的硬停倍率停止点，默认等价于 1.5 倍阈值。 */
    stop_x10 = PumpPressureControl_BuildStopX10(threshold_x10);

    /* 当前压力没有超过阈值时不降速，保持用户设定的目标泵速。 */
    if (weight_x10 <= threshold_x10)
    {
        /* 返回未调整泵速，保证阈值以下不影响正常注水/抽水/灌注。 */
        return target_speed;
    }

    /* 当前压力达到或超过硬停倍率时，立即把输出泵速压到 0。 */
    if (weight_x10 >= stop_x10)
    {
        /* 返回 0 表示本周期停止泵转动；run_flag 保持原样，便于压力恢复后自动继续闭环输出。 */
        return 0U;
    }

    /* 计算线性减速区间总宽度，默认 1.5 倍时等于 threshold_x10 / 2。 */
    available_margin_x10 = stop_x10 - threshold_x10;
    /* 理论上阈值非 0 时该值不会为 0，这里仍做防御，避免异常参数导致除 0。 */
    if (available_margin_x10 == 0U)
    {
        /* 无可用区间时采用最保守策略，直接停止泵输出。 */
        return 0U;
    }

    /* 计算当前压力到停止点的剩余区间，越接近停止点，该值越小。 */
    remaining_margin_x10 = stop_x10 - weight_x10;
    /* 按剩余区间比例缩放目标泵速：阈值处 100%，硬停倍率处 0%。 */
    scaled_speed = (uint64_t)target_speed * (uint64_t)remaining_margin_x10;
    /* 除以总区间得到闭环后的泵速，整数除法自动向下取整，避免超出安全侧。 */
    return (uint16_t)(scaled_speed / available_margin_x10);
#endif
}
