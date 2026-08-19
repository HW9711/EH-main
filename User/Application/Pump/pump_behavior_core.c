#include "pump_behavior_core.h"

#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "common.h"
#include "lcd.h"
#include "pump.h"
#include "pump_pressure_control.h"
#include "screen_address.h"
#include "uart5.h"
#include "uart7.h"

/* PumpBehaviorBinding_t 集中记录 A/B 固定差异，公共算法不再散落通道判断。 */
typedef struct
{
    pumpMessage_t *message;                    /* 本通道唯一的泵运行状态和压力数据。 */
    uint8_t public_channel;                     /* 压力锁止时交给 Pubinterface 的逻辑通道号。 */
    uint16_t output_color_address;              /* 本通道在当前屏幕映射下的运行颜色地址。 */
    uint8_t draw_direction;                     /* 抽吸类型使用的原业务方向值。 */
    uint8_t inject_direction;                   /* 注水和灌注类型使用的原业务方向值。 */
    uint8_t invert_protocol_direction;          /* 为 1 时发送前取反，保留 A 泵原协议方向规则。 */
    void (*publish_output_speed)(uint16_t);     /* 发布压力修正后的实际业务速度。 */
    void (*send_packet)(uint8_t *, uint16_t);   /* 把完整 6 字节控制帧送到固定物理 UART。 */
} PumpBehaviorBinding_t;

/* PumpBehaviorRuntime_t 保存每路任务自己的跨周期状态，A/B 不能互相覆盖。 */
typedef struct
{
    uint8_t last_request_active; /* 上一周期是否有运行请求，用于只在新启动沿解除压力锁止。 */
    uint8_t output_was_stopped;  /* 对应旧 huici 状态，只在启停变化时刷新屏幕颜色。 */
    uint8_t business_direction;  /* 保留上一次业务方向，停泵帧继续沿用原方向字节时序。 */
} PumpBehaviorRuntime_t;

/* 两个静态元素分别属于 A/B 独立任务，不共享启动边沿、颜色或方向状态。 */
static PumpBehaviorRuntime_t s_pump_runtime[PUMP_BEHAVIOR_CHANNEL_COUNT];

/*
 * 函数功能：把逻辑 A 泵控制帧发送到当前配置对应的物理串口。
 * 输入参数：data 为完整控制帧；length 为帧长度，当前固定传入 6。
 * 返回参数：无。
 */
static void PumpBehavior_SendPacketA(uint8_t *data, uint16_t length)
{
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
    Uart7_SendPacket(data, length); /* 互换开启时逻辑 A 走原 B 泵 UART7，只改变最后硬件出口。 */
#else
    Uart5_SendPacket(data, length); /* 互换关闭时保持原接线，逻辑 A 走 UART5。 */
#endif
}

/*
 * 函数功能：把逻辑 B 泵控制帧发送到当前配置对应的物理串口。
 * 输入参数：data 为完整控制帧；length 为帧长度，当前固定传入 6。
 * 返回参数：无。
 */
static void PumpBehavior_SendPacketB(uint8_t *data, uint16_t length)
{
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
    Uart5_SendPacket(data, length); /* 互换开启时逻辑 B 走原 A 泵 UART5，只改变最后硬件出口。 */
#else
    Uart7_SendPacket(data, length); /* 互换关闭时保持原接线，逻辑 B 走 UART7。 */
#endif
}

/* A/B 固定绑定只描述真实差异，不提供运行时注册，避免重新增加抽象层。 */
static const PumpBehaviorBinding_t s_pump_binding[PUMP_BEHAVIOR_CHANNEL_COUNT] =
{
    {
        &pumpMessageA,                          /* A 压力源保持 SIM_UART_1/PE4 写入的 pumpMessageA。 */
        CHANNEL_A,                              /* A 压力锁止继续处理逻辑 A 通道。 */
        UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,         /* A 实际输出颜色继续使用屏幕映射后的 A 地址。 */
        0U,                                     /* A 抽吸业务方向保持 0。 */
        1U,                                     /* A 注水和灌注业务方向保持 1。 */
        1U,                                     /* A 协议方向保持发送前取反。 */
        Pubinterface_UpdatePumpAOutputSpeed,     /* A 实际速度继续发布到 pumpMessageA.speed_output。 */
        PumpBehavior_SendPacketA                 /* A 帧继续走 A 逻辑出口。 */
    },
    {
        &pumpMessageB,                          /* B 压力源保持 SIM_UART_2/PE6 写入的 pumpMessageB。 */
        CHANNEL_B,                              /* B 压力锁止继续处理逻辑 B 通道。 */
        UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,         /* B 实际输出颜色继续使用屏幕映射后的 B 地址。 */
        1U,                                     /* B 抽吸业务方向保持 1。 */
        0U,                                     /* B 注水和灌注业务方向保持 0。 */
        0U,                                     /* B 协议方向保持不取反。 */
        Pubinterface_UpdatePumpBOutputSpeed,     /* B 实际速度继续发布到 pumpMessageB.speed_output。 */
        PumpBehavior_SendPacketB                 /* B 帧继续走 B 逻辑出口。 */
    }
};

/*
 * 函数功能：读取本通道队列中的一条最新速度命令，保持原来每周期最多处理一条消息的节拍。
 * 输入参数：binding 指向本通道固定配置；message_queue 为本通道独立队列，可为 NULL。
 * 返回参数：无。
 */
static void PumpBehavior_ReceiveSpeed(const PumpBehaviorBinding_t *binding, QueueHandle_t message_queue)
{
    PumpBehaviorMessage_t message; /* 局部消息只在成功出队时生效，不保留跨周期临时值。 */

    if (message_queue == NULL)
    {
        return; /* 队列尚未初始化时保持公共状态不变，沿用旧任务的静默返回行为。 */
    }

    if (Kernel_QueueReceive(message_queue, &message, 0) == pdTRUE)
    {
        binding->message->speed_work = message.Value; /* 仍只同步速度，泵类型必须由设备识别流程更新。 */
    }
}

/*
 * 函数功能：在本通道从无请求变为有请求时解除上一次压力锁止。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道独立跨周期状态；request_active 表示本周期是否请求运行或排空。
 * 返回参数：无。
 */
static void PumpBehavior_ClearHoldOnStart(const PumpBehaviorBinding_t *binding,
                                                       PumpBehaviorRuntime_t *runtime,
                                                       uint8_t request_active)
{
    if ((request_active != 0U) && (runtime->last_request_active == 0U))
    {
        binding->message->pressure_hold_flag = false; /* 只允许新的控制启动沿解除锁止，压力自然下降不能自动复转。 */
        binding->message->pressure_recover_ms = 0U;   /* 清零保留计时字段，避免调试时误判为仍在恢复计时。 */
    }

    runtime->last_request_active = request_active; /* 连续踩住脚踏或持续触控不重复视为新启动。 */
}

/*
 * 函数功能：按当前压力和锁止状态修正本周期业务速度，达到停泵点时执行现有公共安全处理。
 * 输入参数：binding 为本通道固定配置；pump_speed 为类型限幅后的业务速度；force_stop 用于返回本周期是否必须停泵。
 * 返回参数：压力闭环修正后的实际业务速度，锁止或达到停泵点时返回 0。
 */
static uint16_t PumpBehavior_ApplyPressure(const PumpBehaviorBinding_t *binding,
                                           uint16_t pump_speed,
                                           uint8_t *force_stop)
{
    uint32_t weight_x10 = binding->message->weight_x10;          /* 一次读取本周期压力，避免多字节 volatile 值前后不一致。 */
    uint16_t threshold_g = binding->message->pressure_threshold; /* 阈值字段仍只作为压力帧有效门禁。 */

    if (force_stop != NULL)
    {
        *force_stop = 0U; /* 默认没有硬停，只有保持态或新触发停泵时改为 1。 */
    }

    if (binding->message->pressure_hold_flag == true)
    {
        binding->message->pressure_recover_ms = 0U; /* 现策略不按时间自动恢复，保持字段清零。 */
        Pubinterface_ServicePumpPressureHold(binding->public_channel); /* 保持停泵，并按原规则处理注水手柄联动。 */
        if (force_stop != NULL)
        {
            *force_stop = 1U; /* 通知排空分支禁止重新生成非零输出。 */
        }
        return 0U; /* 压力锁止期间始终发送零速，直到新的启动沿解除锁止。 */
    }

    if (PumpPressureControl_IsPressureStopReached(pump_speed, weight_x10, threshold_g) != 0U)
    {
        binding->message->pressure_hold_flag = true; /* 第一次达到停泵点后建立锁止，禁止压力回落即复转。 */
        binding->message->pressure_recover_ms = 0U;  /* 记录当前没有自动恢复倒计时。 */
        if (force_stop != NULL)
        {
            *force_stop = 1U; /* 本周期立即进入硬停输出。 */
        }
        Pubinterface_HandlePumpPressureBlocked(binding->public_channel); /* 沿用原报警、蜂鸣和控制请求撤销链。 */
        return 0U; /* 达到停泵点的首周期立即输出零速。 */
    }

    if (force_stop != NULL)
    {
        *force_stop = PumpPressureControl_ShouldForceStop(pump_speed, weight_x10, threshold_g); /* 保留旧硬停结果回传。 */
    }

    return PumpPressureControl_Apply(pump_speed, weight_x10, threshold_g); /* 未锁止时按原线性闭环限速。 */
}

/*
 * 函数功能：按泵类型完成方向选择、普通运行速度上限和 UART 数值换算，再执行压力闭环。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道独立状态；pump_type 为识别出的泵类型；pump_speed 指向待处理业务速度；drainage_active 表示当前是否为屏幕定时排空；force_stop 返回压力硬停状态。
 * 返回参数：换算后的 16 位 UART 速度字段。
 */
static uint16_t PumpBehavior_ConvertOutput(const PumpBehaviorBinding_t *binding,
                                           PumpBehaviorRuntime_t *runtime,
                                           uint8_t pump_type,
                                           uint16_t *pump_speed,
                                           uint8_t drainage_active,
                                           uint8_t *force_stop)
{
    uint16_t uart_data = 0U; /* 未识别类型或零速状态默认发送 0。 */

    switch (pump_type)
    {
        case DRAWWATER:
            runtime->business_direction = binding->draw_direction; /* 保留 A/B 原抽吸业务方向差异。 */
            if (*pump_speed > 15U)
            {
                *pump_speed = 15U; /* 抽吸泵继续按原 1.5L 档位上限裁剪。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, *pump_speed, force_stop); /* 限幅后再做压力保护。 */
            uart_data = (uint16_t)(*pump_speed * 42U); /* 抽吸泵继续使用原 42 倍驱动换算。 */
            break;

        case INJECTWATER:
            runtime->business_direction = binding->inject_direction; /* 保留 A/B 原注水业务方向差异。 */
            if ((drainage_active == 0U) && (*pump_speed > PUMP_INJECTWATER_SPEED_MAX))
            {
                *pump_speed = PUMP_INJECTWATER_SPEED_MAX; /* 普通运行和手柄联动最多输出 300，屏幕定时排空仍保留独立的 100 档。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, *pump_speed, force_stop); /* 换算 UART 前先应用压力保护。 */
            uart_data = (uint16_t)(*pump_speed / 1.6); /* 保留原浮点除数和 AC5 截断结果。 */
            break;

        case POURWATER:
            runtime->business_direction = binding->inject_direction; /* 灌注与注水继续使用同一业务方向。 */
            if (*pump_speed > 300U)
            {
                *pump_speed = 300U; /* 灌注最大速度继续保持 300ml。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, *pump_speed, force_stop); /* 换算 UART 前先应用压力保护。 */
            uart_data = (uint16_t)(*pump_speed / 1.51); /* 保留原浮点除数和 AC5 截断结果。 */
            break;

        default:
            break; /* 未识别类型不改变上次方向，只发送零速，与旧代码一致。 */
    }

    return uart_data;
}

/*
 * 函数功能：维持注水泵 10 秒排空计时，并在压力停泵或计时结束时强制零输出。
 * 输入参数：message 为本通道泵状态；timing_drainage_active 为本周期开始时锁存的排空标志；force_stop 为压力硬停状态；pump_speed 和 uart_data 为待发布输出。
 * 返回参数：无。
 */
static void PumpBehavior_ServiceDrainage(pumpMessage_t *message,
                                         uint8_t timing_drainage_active,
                                         uint8_t force_stop,
                                         uint16_t *pump_speed,
                                         uint16_t *uart_data)
{
    if (timing_drainage_active == 0U)
    {
        return; /* 非排空周期不修改排空计数和正常运行输出。 */
    }

    if (force_stop != 0U)
    {
        *uart_data = 0U;  /* 压力硬停时禁止排空逻辑重新生成非零驱动速度。 */
        *pump_speed = 0U; /* 屏幕和上位机同步显示真实零输出。 */
    }
    else if (message->timingDrainage_times >= PUMP_TIMING_DRAINAGE_TICKS)
    {
        *uart_data = 0U;                        /* 达到 10 秒后立即发送零速。 */
        *pump_speed = 0U;                       /* 实际业务输出同步清零。 */
        message->timingDrainage_flag = false;   /* 结束本次排空请求。 */
        message->run_flag = false;              /* 保留旧逻辑，同时撤销普通运行请求。 */
        message->timingDrainage_times = 0U;      /* 清零计数，等待下一次排空重新累计。 */
    }
    else
    {
        message->timingDrainage_times++; /* 每个 25ms 周期加一，400 次对应现场 10 秒。 */
    }
}

/*
 * 函数功能：按严格 CRC16 协议组装并发送 6 字节泵驱动帧。
 * 输入参数：binding 为本通道固定配置；uart_data 为驱动速度；business_direction 为该泵原业务方向值。
 * 返回参数：无。
 */
static void PumpBehavior_SendFrame(const PumpBehaviorBinding_t *binding,
                                   uint16_t uart_data,
                                   uint8_t business_direction)
{
    uint8_t frame[6] = {0xAAU, 0U, 0U, 0U, 0U, 0U}; /* 固定布局为帧头、方向、速度高低字节、CRC低高字节。 */
    uint16_t crc; /* CRC16/MODBUS 只覆盖前 4 字节，结果按低字节在前发送。 */

    if (uart_data > PUMP_DRIVER_COMMAND_SPEED_MAX)
    {
        uart_data = PUMP_DRIVER_COMMAND_SPEED_MAX; /* 主控发送前再次限幅，防止未来新增路径绕过业务换算上限。 */
    }

    if (binding->invert_protocol_direction != 0U)
    {
        frame[1] = (business_direction != 0U) ? 0U : 1U; /* A 泵协议层继续把业务方向取反。 */
    }
    else
    {
        frame[1] = (business_direction != 0U) ? 1U : 0U; /* B 泵协议层继续直接使用业务方向。 */
    }
    frame[2] = (uint8_t)((uart_data >> 8) & 0xFFU); /* 速度高字节继续先发送。 */
    frame[3] = (uint8_t)(uart_data & 0xFFU);        /* 速度低字节紧随高字节。 */
    crc = Common_Crc16(frame, 4U);                  /* 对帧头、方向和速度字段计算 Modbus CRC16。 */
    frame[4] = (uint8_t)(crc & 0x00FFU);            /* 驱动协议第 5 字节固定发送 CRC 低字节。 */
    frame[5] = (uint8_t)((crc >> 8U) & 0x00FFU);    /* 驱动协议第 6 字节固定发送 CRC 高字节。 */

    binding->send_packet(frame, 6U); /* 每个 25ms 周期发送一帧 CRC 命令，用于续租驱动端 100ms 通信授权。 */
}

/*
 * 函数功能：仅在泵启停状态发生变化时刷新屏幕输出颜色，避免每 25ms 重复写屏。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道独立颜色状态；uart_data 为本周期实际驱动速度。
 * 返回参数：无。
 */
static void PumpBehavior_UpdateColor(const PumpBehaviorBinding_t *binding,
                                     PumpBehaviorRuntime_t *runtime,
                                     uint16_t uart_data)
{
    if (uart_data == 0U)
    {
        if (runtime->output_was_stopped == 0U)
        {
            runtime->output_was_stopped = 1U; /* 记录已经显示停止色，后续零速周期不重复写屏。 */
            LCD_Show_2byte_Number(binding->output_color_address, 0xFFFFU); /* 白色表示当前泵无实际输出。 */
        }
    }
    else if (runtime->output_was_stopped != 0U)
    {
        runtime->output_was_stopped = 0U; /* 记录已经恢复运行色，后续运行周期不重复写屏。 */
        LCD_Show_2byte_Number(binding->output_color_address, 0xFFE0U); /* 黄色表示当前泵正在实际输出。 */
    }
}

/*
 * 函数功能：执行指定逻辑泵的一次 25ms 行为周期，处理队列速度、压力保护、排空、显示和 UART 下发。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue)
{
    const PumpBehaviorBinding_t *binding; /* 指向本周期固定的 A/B 硬件和状态绑定。 */
    PumpBehaviorRuntime_t *runtime;       /* 指向本通道独立跨周期状态。 */
    uint8_t pump_type = 0U;               /* 无运行请求时保持旧默认类型 0，不进入任何换算分支。 */
    uint16_t pump_speed = 0U;             /* 本周期实际业务速度默认 0。 */
    uint16_t uart_data;                   /* 本周期最终下发给泵驱动的速度字段。 */
    uint8_t force_stop = 0U;              /* 压力保护是否要求本周期硬停。 */
    uint8_t drainage_active;              /* 锁存周期开始时的10秒定时排空状态，保持原分支判断时序。 */
    uint8_t request_active;               /* 运行或排空任一有效都视为有输出请求。 */

    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return; /* 非法通道不访问数组、不发送泵帧，避免错误调用影响现场输出。 */
    }

    binding = &s_pump_binding[channel]; /* 选定本周期固定通道差异。 */
    runtime = &s_pump_runtime[channel]; /* 选定本通道独立运行态。 */

    PumpBehavior_ReceiveSpeed(binding, message_queue); /* 每周期最多取一条速度消息，等待时间保持 0。 */

    request_active = (binding->message->run_flag || binding->message->timingDrainage_flag) ? 1U : 0U; /* 锁存本周期请求状态。 */
    PumpBehavior_ClearHoldOnStart(binding, runtime, request_active); /* 只在新启动沿清除压力锁止。 */
    drainage_active = binding->message->timingDrainage_flag ? 1U : 0U; /* 压力处理前锁存排空状态。 */
    if (binding->message->run_flag || binding->message->timingDrainage_flag)
    {
        pump_type = (uint8_t)binding->message->type; /* 运行时使用设备识别流程写入的真实泵类型。 */
        if ((pump_type == INJECTWATER) && (binding->message->speed_work > PUMP_INJECTWATER_SPEED_MAX))
        {
            binding->message->speed_work = PUMP_INJECTWATER_SPEED_MAX; /* 收回异常越界设定，确保排空结束后正常注水最多保持 300。 */
        }
        pump_speed = (drainage_active != 0U) ? PUMP_TIMING_DRAINAGE_SPEED : binding->message->speed_work; /* 只有屏幕定时排空使用固定速度；脚踏轻踩和普通联动都读取屏幕设定速度。 */
    }

    uart_data = PumpBehavior_ConvertOutput(binding, runtime, pump_type, &pump_speed, drainage_active, &force_stop); /* 排空标志用于区分固定 100 档与普通注水 300 档上限。 */
    PumpBehavior_ServiceDrainage(binding->message, drainage_active, force_stop, &pump_speed, &uart_data); /* 保持 10 秒排空与压力停泵优先级。 */
    binding->publish_output_speed(pump_speed); /* 先发布实际速度，再按原顺序发送驱动帧和刷新颜色。 */
    PumpBehavior_SendFrame(binding, uart_data, runtime->business_direction); /* 每个周期继续发送 6 字节帧。 */
    PumpBehavior_UpdateColor(binding, runtime, uart_data); /* 只在启停边沿刷新对应逻辑泵颜色。 */
}
