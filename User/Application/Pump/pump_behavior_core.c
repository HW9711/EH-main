#include "pump_behavior_core.h"

#include <string.h>

#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "common.h"
#include "lcd.h"
#include "pump.h"
#include "pump_pressure_control.h"
#include "screen_address.h"
#include "sscBEEP.h"
#include "uart5.h"
#include "uart7.h"

#define PUMP_DRIVER_FEEDBACK_FRAME_HEAD       0xAAU /* 步进驱动回包首字节固定为 0xAA。 */
#define PUMP_DRIVER_FEEDBACK_FRAME_SIZE       7U    /* 回包固定为帧头、方向、实际速度2字节、故障码和CRC2字节。 */
#define PUMP_DRIVER_FEEDBACK_CRC_DATA_SIZE    5U    /* CRC16/MODBUS 覆盖回包前 5 字节。 */
#define PUMP_DRIVER_ERROR_NONE                0U    /* 驱动回包故障码 0 表示保护已解除。 */
#define PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE   UART5_MAX_PACKET_SIZE /* A/B 解析缓存按现有 UART5/7 共同 DMA 容量分配。 */

#if (UART5_MAX_PACKET_SIZE != UART7_MAX_PACKET_SIZE)
#error "Pump driver UART5/UART7 DMA buffer sizes must match"
#endif

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
    uint16_t (*receive_data)(uint8_t *);        /* 取走同一物理 UART 上一命令对应的步进驱动回包。 */
} PumpBehaviorBinding_t;

/* PumpBehaviorRuntime_t 保存每路任务自己的跨周期状态，A/B 不能互相覆盖。 */
typedef struct
{
    uint8_t last_request_active; /* 上一周期是否有运行请求，用于只在新启动沿解除压力锁止。 */
    uint8_t output_was_stopped;  /* 对应旧 huici 状态，只在启停变化时刷新屏幕颜色。 */
    uint8_t business_direction;  /* 保留上一次业务方向，停泵帧继续沿用原方向字节时序。 */
    uint8_t driver_fault_hold;   /* 收到非零驱动故障后锁存停机，故障清零且控制源释放后才解除。 */
    uint8_t last_feedback_error; /* 保存最近一帧 CRC 正确回包的原始故障码。 */
} PumpBehaviorRuntime_t;

/* 两个静态元素分别属于 A/B 独立任务，不共享启动边沿、颜色或方向状态。 */
static PumpBehaviorRuntime_t s_pump_runtime[PUMP_BEHAVIOR_CHANNEL_COUNT];
/* 每路快照仅由对应泵任务写入，调试或遥测读取方通过版本号复制。 */
static PumpDriverFeedbackSnapshot_t s_pump_feedback_snapshot[PUMP_BEHAVIOR_CHANNEL_COUNT];
/* 偶数表示快照稳定，奇数表示对应泵任务正在更新多字节字段。 */
static volatile uint32_t s_pump_feedback_version[PUMP_BEHAVIOR_CHANNEL_COUNT];

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

/*
 * 函数功能：从当前配置的物理串口取走逻辑 A 泵上一条命令回包。
 * 输入参数：data 指向至少 PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE 字节的解析缓存。
 * 返回参数：本次从 DMA 复制的实际字节数。
 */
static uint16_t PumpBehavior_ReceivePacketA(uint8_t *data)
{
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
    return Uart7_DMARecvDataPeek(data); /* 互换开启时逻辑 A 必须读取与发送一致的 UART7 回包。 */
#else
    return Uart5_DMARecvDataPeek(data); /* 当前正式映射下逻辑 A 读取 UART5 回包。 */
#endif
}

/*
 * 函数功能：从当前配置的物理串口取走逻辑 B 泵上一条命令回包。
 * 输入参数：data 指向至少 PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE 字节的解析缓存。
 * 返回参数：本次从 DMA 复制的实际字节数。
 */
static uint16_t PumpBehavior_ReceivePacketB(uint8_t *data)
{
#if (PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE == 1U)
    return Uart5_DMARecvDataPeek(data); /* 互换开启时逻辑 B 必须读取与发送一致的 UART5 回包。 */
#else
    return Uart7_DMARecvDataPeek(data); /* 当前正式映射下逻辑 B 读取 UART7 回包。 */
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
        PumpBehavior_SendPacketA,                /* A 帧继续走 A 逻辑出口。 */
        PumpBehavior_ReceivePacketA              /* A 回包从同一逻辑出口对应的物理串口取回。 */
    },
    {
        &pumpMessageB,                          /* B 压力源保持 SIM_UART_2/PE6 写入的 pumpMessageB。 */
        CHANNEL_B,                              /* B 压力锁止继续处理逻辑 B 通道。 */
        UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,         /* B 实际输出颜色继续使用屏幕映射后的 B 地址。 */
        1U,                                     /* B 抽吸业务方向保持 1。 */
        0U,                                     /* B 注水和灌注业务方向保持 0。 */
        0U,                                     /* B 协议方向保持不取反。 */
        Pubinterface_UpdatePumpBOutputSpeed,     /* B 实际速度继续发布到 pumpMessageB.speed_output。 */
        PumpBehavior_SendPacketB,                /* B 帧继续走 B 逻辑出口。 */
        PumpBehavior_ReceivePacketB              /* B 回包从同一逻辑出口对应的物理串口取回。 */
    }
};

/*
 * 函数功能：把一帧 CRC 正确的步进驱动回包发布为指定逻辑泵的一致性快照。
 * 输入参数：channel 为逻辑 A/B 通道；frame 指向已通过帧头和 CRC 检查的 7 字节回包。
 * 返回参数：无。
 */
static void PumpBehavior_RecordDriverFeedback(PumpBehaviorChannel_t channel, const uint8_t *frame)
{
    if ((channel >= PUMP_BEHAVIOR_CHANNEL_COUNT) || (frame == NULL))
    {
        return; /* 非法通道或空帧不能更新调试快照，保留最后一份可信数据。 */
    }

    ++s_pump_feedback_version[channel]; /* 先发布奇数版本，阻止读取方复制半更新字段。 */
    __DMB(); /* 保证写入中标志先于后续快照字段可见。 */
    s_pump_feedback_snapshot[channel].feedback_tick_ms = HAL_GetTick(); /* 记录主控完成本帧协议校验的时刻。 */
    s_pump_feedback_snapshot[channel].sequence = (uint16_t)(s_pump_feedback_snapshot[channel].sequence + 1U); /* 每份有效回包递增，便于断点判断回包是否持续到达。 */
    s_pump_feedback_snapshot[channel].direction = frame[1]; /* 保留驱动端实际方向原始值，不用主控业务方向替代。 */
    s_pump_feedback_snapshot[channel].actual_speed = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]); /* 按协议高字节在前还原驱动实际速度。 */
    s_pump_feedback_snapshot[channel].raw_error = frame[4]; /* 保留驱动原始故障码，方便现场直接查看 0x04 等保护原因。 */
    s_pump_feedback_snapshot[channel].valid = 1U; /* 所有字段完成后标记本通道已有可信回包。 */
    __DMB(); /* 保证快照字段先于最终偶数版本对其它任务可见。 */
    ++s_pump_feedback_version[channel]; /* 写入完成后恢复偶数版本，开放一致性复制。 */
}

/*
 * 函数功能：一致性复制指定逻辑泵最近一帧 CRC 正确的步进驱动回包快照。
 * 输入参数：channel 为逻辑 A/B 通道；snapshot 指向调用方提供的快照缓存。
 * 返回参数：快照有效且复制成功返回 1，否则返回 0 并把输出清零。
 */
uint8_t PumpBehavior_CopyDriverFeedbackSnapshot(PumpBehaviorChannel_t channel,
                                                PumpDriverFeedbackSnapshot_t *snapshot)
{
    uint8_t attempt; /* 跨任务复制最多尝试三次，避免诊断读取阻塞 25ms 泵任务。 */

    if (snapshot == NULL)
    {
        return 0U; /* 调用方未提供目标缓存时不读取共享数组。 */
    }
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        memset(snapshot, 0, sizeof(*snapshot)); /* 通道越界时返回明确无效快照，不保留调用方旧值。 */
        return 0U;
    }

    for (attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t version_before = s_pump_feedback_version[channel]; /* 复制前版本必须为稳定偶数。 */
        uint32_t version_after; /* 复制后再次读取版本，确认字段属于同一回包。 */

        if ((version_before & 1U) != 0U)
        {
            continue; /* 对应泵任务正在写入时立即重试，不返回混合字段。 */
        }

        __DMB(); /* 版本检查完成后再复制共享快照。 */
        *snapshot = s_pump_feedback_snapshot[channel]; /* 后续诊断只读调用方私有副本。 */
        __DMB(); /* 字段复制结束后再复核最终版本。 */
        version_after = s_pump_feedback_version[channel];
        if ((version_before == version_after) && ((version_after & 1U) == 0U))
        {
            return (snapshot->valid != 0U) ? 1U : 0U; /* 前后版本一致时返回快照自身有效位。 */
        }
    }

    memset(snapshot, 0, sizeof(*snapshot)); /* 三次均碰到并发写入时返回明确无效数据。 */
    return 0U;
}

/*
 * 函数功能：扫描本周期 UART DMA 字节，校验并解析所有完整的 7 字节步进驱动回包。
 * 输入参数：channel 为逻辑泵通道；binding 为本通道物理串口绑定；runtime 为本通道独立运行状态。
 * 返回参数：无。
 */
static void PumpBehavior_ParseDriverFeedback(PumpBehaviorChannel_t channel,
                                             const PumpBehaviorBinding_t *binding,
                                             PumpBehaviorRuntime_t *runtime)
{
    uint8_t rx_data[PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE] = {0U}; /* 局部副本保证 DMA 重启后不会覆盖当前解析数据。 */
    uint16_t received_len; /* 保存上一条命令后 DMA 实际收到的字节数。 */
    uint16_t offset = 0U; /* 扫描偏移允许跳过异常字节，并连续解析粘连回包。 */

    if ((binding == NULL) || (runtime == NULL) || (binding->receive_data == NULL))
    {
        return; /* 内部绑定异常时不读 DMA，避免影响现有泵命令下发。 */
    }

    received_len = binding->receive_data(rx_data); /* 每个 25ms 周期取走上一命令对应的所有回包字节。 */
    while ((uint16_t)(offset + PUMP_DRIVER_FEEDBACK_FRAME_SIZE) <= received_len)
    {
        uint16_t calculated_crc; /* 主控对当前候选帧前 5 字节重新计算的 CRC16/MODBUS。 */
        uint16_t received_crc; /* 候选帧 byte5~6 按低字节在前还原的驱动 CRC。 */
        uint8_t driver_error; /* 当前有效帧 byte4 的驱动原始故障码。 */

        if (rx_data[offset] != PUMP_DRIVER_FEEDBACK_FRAME_HEAD)
        {
            ++offset; /* 非 0xAA 字节不可能是帧起点，只跳一字节继续寻找。 */
            continue;
        }

        calculated_crc = Common_Crc16(&rx_data[offset], PUMP_DRIVER_FEEDBACK_CRC_DATA_SIZE); /* 帧头至故障码共 5 字节参与校验。 */
        received_crc = (uint16_t)((uint16_t)rx_data[offset + 5U] | ((uint16_t)rx_data[offset + 6U] << 8U)); /* 驱动按 CRC 低字节、高字节顺序回传。 */
        if (calculated_crc != received_crc)
        {
            ++offset; /* CRC 异常不改故障状态，从下一字节继续寻找后续有效帧。 */
            continue;
        }

        PumpBehavior_RecordDriverFeedback(channel, &rx_data[offset]); /* 先发布原始快照，断点可直接看方向、实际速度和故障码。 */
        driver_error = rx_data[offset + 4U]; /* 仅 CRC 正确帧才能驱动故障状态机。 */
        runtime->last_feedback_error = driver_error; /* 记住最新驱动状态，供故障锁存判断是否允许解除。 */
        if ((driver_error != PUMP_DRIVER_ERROR_NONE) && (runtime->driver_fault_hold == 0U))
        {
            runtime->driver_fault_hold = 1U; /* 非零故障首帧立即锁存，后续正常回包不能自动复转。 */
            Pubinterface_HandlePumpDriverFault(binding->public_channel); /* 同步停本泵和各控制源，若为手柄冷却泵则联动停手柄。 */
            SendDoubleBeepMessageIfIdle(); /* 故障边沿只投递一次双响，已存在高优先级报警时不覆盖。 */
        }

        offset = (uint16_t)(offset + PUMP_DRIVER_FEEDBACK_FRAME_SIZE); /* 有效帧整帧跳过，继续处理 DMA 中可能粘连的下一帧。 */
    }
}

/*
 * 函数功能：在步进驱动故障锁存期持续保持安全停机，并完成明确的重启解锁。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道独立状态；request_active 是停机处理前锁存的本周期控制请求。
 * 返回参数：本周期仍需故障停机返回 1；未锁存或已满足解锁条件返回 0。
 */
static uint8_t PumpBehavior_ServiceDriverFaultHold(const PumpBehaviorBinding_t *binding,
                                                   PumpBehaviorRuntime_t *runtime,
                                                   uint8_t request_active)
{
    if (runtime->driver_fault_hold == 0U)
    {
        return 0U; /* 未收到驱动故障时不改现有压力保护和泵运行时序。 */
    }

    if ((runtime->last_feedback_error == PUMP_DRIVER_ERROR_NONE) && (request_active == 0U))
    {
        runtime->driver_fault_hold = 0U; /* 只有驱动已回报 0 且用户已释放运行源才结束本次故障锁存。 */
        runtime->last_request_active = 0U; /* 解锁后保留无请求状态，下一次明确启动可形成新上升沿。 */
        return 0U;
    }

    Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 每周期清除本泵、外控和必要的手柄联动请求，阻止持续控制源复转。 */
    runtime->last_request_active = request_active; /* 保留停机处理前的实际控制源状态，用于等待释放。 */
    return 1U;
}

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
 * 函数功能：执行指定逻辑泵的一次 25ms 行为周期，处理驱动回包、故障锁存、队列速度、压力保护、排空、显示和 UART 下发。
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
    uint8_t driver_fault_active;          /* 步进驱动故障锁存期为 1，本周期必须跳过所有非零输出路径。 */

    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return; /* 非法通道不访问数组、不发送泵帧，避免错误调用影响现场输出。 */
    }

    binding = &s_pump_binding[channel]; /* 选定本周期固定通道差异。 */
    runtime = &s_pump_runtime[channel]; /* 选定本通道独立运行态。 */

    PumpBehavior_ReceiveSpeed(binding, message_queue); /* 每周期最多取一条速度消息，等待时间保持 0。 */

    request_active = (binding->message->run_flag || binding->message->timingDrainage_flag) ? 1U : 0U; /* 在故障处理清状态前锁存真实控制源，供释放门禁判断。 */
    PumpBehavior_ParseDriverFeedback(channel, binding, runtime); /* 发送下一命令前先解析上一条命令的驱动回包。 */
    driver_fault_active = PumpBehavior_ServiceDriverFaultHold(binding, runtime, request_active); /* 故障锁存期持续清除运行源并输出零速。 */
    if (driver_fault_active == 0U)
    {
        PumpBehavior_ClearHoldOnStart(binding, runtime, request_active); /* 驱动故障已解锁时，才允许新启动沿清除压力锁止。 */
        drainage_active = binding->message->timingDrainage_flag ? 1U : 0U; /* 压力处理前锁存排空状态。 */
    }
    else
    {
        drainage_active = 0U; /* 驱动故障期禁止排空逻辑递增计时或生成固定非零速度。 */
    }
    if ((driver_fault_active == 0U) && (binding->message->run_flag || binding->message->timingDrainage_flag))
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
