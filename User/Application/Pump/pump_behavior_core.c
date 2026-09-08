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
#define PUMP_DRIVER_ERROR_NONE                0U    /* 驱动回包故障码 0 表示当前无故障；主控仍需等运行请求释放后才允许重启。 */
#define PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE   UART5_MAX_PACKET_SIZE /* A/B 解析缓存按现有 UART5/7 共同 DMA 容量分配。 */

#if (UART5_MAX_PACKET_SIZE != UART7_MAX_PACKET_SIZE)
#error "Pump driver UART5/UART7 DMA buffer sizes must match"
#endif

/* 每路泵对应的运行数据、屏幕地址、方向和串口函数，让同一套处理代码可用于 A/B 两路。 */
typedef struct
{
    pumpMessage_t *message;                    /* 本通道唯一的泵运行状态和压力数据。 */
    uint8_t public_channel;                     /* 报警提示和驱动故障处理使用的逻辑通道号。 */
    uint16_t output_color_address;              /* 本通道在当前屏幕映射下的运行颜色地址。 */
    uint8_t draw_direction;                     /* 抽吸类型使用的原业务方向值。 */
    uint8_t inject_direction;                   /* 注水和灌注类型使用的原业务方向值。 */
    uint8_t invert_protocol_direction;          /* 为 1 时发送前取反，保留 A 泵原协议方向规则。 */
    void (*publish_output_speed)(uint16_t);     /* 保存本周期最终输出设定，不是实测转速；真实超压不改变该值。 */
    void (*send_packet)(uint8_t *, uint16_t);   /* 把完整 6 字节控制帧送到固定物理 UART。 */
    uint16_t (*receive_data)(uint8_t *);        /* 取走同一物理 UART 上一命令对应的步进驱动回包。 */
} PumpBehaviorBinding_t;

/* PumpBehaviorRuntime_t 保存每路任务自己的跨周期状态，A/B 不能互相覆盖。 */
typedef struct
{
    PumpPressureAlarmState_t pressure_alarm; /* 本通道独立维护持续超限和恢复确认，只产生报警事件。 */
    uint8_t output_was_stopped;  /* 对应旧 huici 状态，只在启停变化时刷新屏幕颜色。 */
    uint8_t business_direction;  /* 记住按泵类型选定的方向，停泵命令也使用同一方向。 */
    uint8_t driver_fault_hold;   /* 收到非零驱动故障后锁存停机，故障清零且控制源释放后才解除。 */
    uint8_t last_feedback_error; /* 保存最近一帧 CRC 正确回包的原始故障码。 */
} PumpBehaviorRuntime_t;

/* A/B 各保存一份启停、颜色和方向记录，互不覆盖。 */
static PumpBehaviorRuntime_t s_pump_runtime[PUMP_BEHAVIOR_CHANNEL_COUNT];
/* A/B 各保存最近一帧驱动反馈；读取时检查版本号，避免拿到更新到一半的数据。 */
static PumpDriverFeedbackSnapshot_t s_pump_feedback_snapshot[PUMP_BEHAVIOR_CHANNEL_COUNT];
/* 偶数表示数据已写完，奇数表示对应泵任务正在更新字段。 */
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

/* 固定列出 A/B 两路使用的状态、屏幕地址、方向和收发串口，运行中不改变此表。 */
static const PumpBehaviorBinding_t s_pump_binding[PUMP_BEHAVIOR_CHANNEL_COUNT] =
{
    {
        &pumpMessageA,                          /* A 压力源保持 SIM_UART_1/PE4 写入的 pumpMessageA。 */
        CHANNEL_A,                              /* A 报警与驱动故障处理保持逻辑 A 通道。 */
        UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR,         /* 按下发命令的零速/非零状态更新屏幕 A 泵颜色。 */
        0U,                                     /* A 抽吸业务方向保持 0。 */
        1U,                                     /* A 注水和灌注业务方向保持 1。 */
        1U,                                     /* A 协议方向保持发送前取反。 */
        Pubinterface_UpdatePumpAOutputSpeed,     /* A 最终输出设定写入 pumpMessageA.speed_output，不是实测转速。 */
        PumpBehavior_SendPacketA,                /* A 帧继续走 A 逻辑出口。 */
        PumpBehavior_ReceivePacketA              /* A 回包从同一逻辑出口对应的物理串口取回。 */
    },
    {
        &pumpMessageB,                          /* B 压力源保持 SIM_UART_2/PE6 写入的 pumpMessageB。 */
        CHANNEL_B,                              /* B 报警与驱动故障处理保持逻辑 B 通道。 */
        UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR,         /* 按下发命令的零速/非零状态更新屏幕 B 泵颜色。 */
        1U,                                     /* B 抽吸业务方向保持 1。 */
        0U,                                     /* B 注水和灌注业务方向保持 0。 */
        0U,                                     /* B 协议方向保持不取反。 */
        Pubinterface_UpdatePumpBOutputSpeed,     /* B 最终输出设定写入 pumpMessageB.speed_output，不是实测转速。 */
        PumpBehavior_SendPacketB,                /* B 帧继续走 B 逻辑出口。 */
        PumpBehavior_ReceivePacketB              /* B 回包从同一逻辑出口对应的物理串口取回。 */
    }
};

/*
 * 函数功能：保存指定泵的一帧有效驱动反馈，并用版本号标明何时正在写入、何时已经写完。
 * 输入参数：channel 为逻辑 A/B 通道；frame 指向已通过帧头和 CRC 检查的 7 字节回包。
 * 返回参数：无。
 */
static void PumpBehavior_RecordDriverFeedback(PumpBehaviorChannel_t channel, const uint8_t *frame)
{
    if ((channel >= PUMP_BEHAVIOR_CHANNEL_COUNT) || (frame == NULL))
    {
        return; /* 通道无效或没有回包数据时不写入，保留上一次有效反馈。 */
    }

    ++s_pump_feedback_version[channel]; /* 先发布奇数版本，阻止读取方复制半更新字段。 */
    __DMB(); /* 先让读取方看到“正在写入”，然后再修改数据字段。 */
    s_pump_feedback_snapshot[channel].feedback_tick_ms = HAL_GetTick(); /* 记录主控完成本帧协议校验的时刻。 */
    s_pump_feedback_snapshot[channel].sequence = (uint16_t)(s_pump_feedback_snapshot[channel].sequence + 1U); /* 每份有效回包递增，便于断点判断回包是否持续到达。 */
    s_pump_feedback_snapshot[channel].direction = frame[1]; /* 保留驱动端实际方向原始值，不用主控业务方向替代。 */
    s_pump_feedback_snapshot[channel].actual_speed = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]); /* 按协议高字节在前还原驱动实际速度。 */
    s_pump_feedback_snapshot[channel].raw_error = frame[4]; /* 保留驱动原始故障码，方便现场直接查看 0x04 等保护原因。 */
    s_pump_feedback_snapshot[channel].valid = 1U; /* 所有字段完成后标记本通道已有可信回包。 */
    __DMB(); /* 先写完所有数据，再标记“已经写完”，避免读到新旧混合数据。 */
    ++s_pump_feedback_version[channel]; /* 恢复偶数版本，读取方此时可以复制完整反馈。 */
}

/*
 * 函数功能：复制指定泵最近的有效驱动反馈，保证各字段来自同一次更新。
 * 输入参数：channel 为逻辑 A/B 通道；snapshot 指向调用方提供的结果缓存。
 * 返回参数：已有有效反馈且复制成功返回 1；否则返回 0，通道无效或连续三次遇到更新时清空结果。
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
        memset(snapshot, 0, sizeof(*snapshot)); /* 通道越界时清空结果，避免调用方误用上一次数据。 */
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

        __DMB(); /* 先确认没有写入，再开始复制反馈字段。 */
        *snapshot = s_pump_feedback_snapshot[channel]; /* 后续诊断只读调用方私有副本。 */
        __DMB(); /* 字段复制结束后再复核最终版本。 */
        version_after = s_pump_feedback_version[channel];
        if ((version_before == version_after) && ((version_after & 1U) == 0U))
        {
            return (snapshot->valid != 0U) ? 1U : 0U; /* 复制期间版本未变，按是否收到过有效反馈返回结果。 */
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

        PumpBehavior_RecordDriverFeedback(channel, &rx_data[offset]); /* 先保存原始反馈，断点可直接看方向、回报速度和故障码。 */
        driver_error = rx_data[offset + 4U]; /* 只有 CRC 正确的故障码才参与停泵和重启判断。 */
        runtime->last_feedback_error = driver_error; /* 记住最新驱动状态，供故障锁存判断是否允许解除。 */
        if ((driver_error != PUMP_DRIVER_ERROR_NONE) && (runtime->driver_fault_hold == 0U))
        {
            runtime->driver_fault_hold = 1U; /* 非零故障首帧立即锁存，后续正常回包不能自动复转。 */
            Pubinterface_HandlePumpDriverFault(binding->public_channel); /* 同步停本泵和各控制源，若为手柄冷却泵则联动停手柄。 */
            SendDoubleBeepMessageIfIdle(); /* 首次发现故障时请求双响，已有高优先级报警时不打断它。 */
        }

        offset = (uint16_t)(offset + PUMP_DRIVER_FEEDBACK_FRAME_SIZE); /* 有效帧整帧跳过，继续处理 DMA 中可能粘连的下一帧。 */
    }
}

/*
 * 函数功能：驱动报故障后持续停泵，直到驱动回报正常且运行请求已经释放才允许再次启动。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道状态；request_active 表示故障处理前是否还有运行或排空请求。
 * 返回参数：本周期仍需故障停机返回 1；未锁存或已满足解锁条件返回 0。
 */
static uint8_t PumpBehavior_ServiceDriverFaultHold(const PumpBehaviorBinding_t *binding,
                                                   PumpBehaviorRuntime_t *runtime,
                                                   uint8_t request_active)
{
    if (runtime->driver_fault_hold == 0U)
    {
        return 0U; /* 没有驱动故障记录时，不干预后续压力检查和泵输出。 */
    }

    if ((runtime->last_feedback_error == PUMP_DRIVER_ERROR_NONE) && (request_active == 0U))
    {
        runtime->driver_fault_hold = 0U; /* 只有驱动已回报 0 且用户已释放运行源才结束本次故障锁存。 */
        return 0U;
    }

    Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 每周期清除本泵、外控和必要的手柄联动请求，阻止持续控制源复转。 */
    return 1U;
}

/*
 * 函数功能：按入队顺序取出本通道的一条速度命令，每周期最多处理一条。
 * 输入参数：binding 指向本通道固定配置；message_queue 为本通道独立队列，可为 NULL。
 * 返回参数：无。
 */
static void PumpBehavior_ReceiveSpeed(const PumpBehaviorBinding_t *binding, QueueHandle_t message_queue)
{
    PumpBehaviorMessage_t message; /* 局部消息只在成功出队时生效，不保留跨周期临时值。 */

    if (message_queue == NULL)
    {
        return; /* 队列还未创建时不等待、不改速度，直接结束本次接收。 */
    }

    if (Kernel_QueueReceive(message_queue, &message, 0) == pdTRUE)
    {
        binding->message->speed_work = message.Value; /* 仍只同步速度，泵类型必须由设备识别流程更新。 */
    }
}

/*
 * 函数功能：用新压力帧判断是否提示超压；只有压力板未完成自标定时才把输出设为零。
 * 输入参数：binding 为本通道固定配置；runtime 保存独立报警状态；pump_speed 为类型限幅后的业务速度。
 * 返回参数：有效压力始终返回原业务速度，压力板未就绪时返回 0。
 */
static uint16_t PumpBehavior_ApplyPressure(const PumpBehaviorBinding_t *binding,
                                           PumpBehaviorRuntime_t *runtime,
                                           uint16_t pump_speed)
{
    uint32_t weight_x10; /* 本次读取的压力值，单位 0.1g，须与阈值字段、序号来自同一帧。 */
    uint16_t threshold_g; /* 压力帧中此字段为 0 表示数据无效；实际报警阈值由主控配置表计算。 */
    uint8_t sequence; /* 压力帧序号用于排除泵任务重复读取的旧样本。 */

    taskENTER_CRITICAL(); /* 与压力串口写入方互斥，避免把不同帧的读数和序号拼成一个样本。 */
    weight_x10 = binding->message->weight_x10; /* 复制当前完整压力帧的重量值。 */
    threshold_g = binding->message->pressure_threshold; /* 复制同一帧的有效性字段。 */
    sequence = binding->message->seq; /* 最后复制同一帧序号，供持续超限确认使用。 */
    taskEXIT_CRITICAL(); /* 三个字段复制完立即恢复正常调度，报警消息在临界区外发送。 */

    if (PumpPressureControl_UpdateAlarm(&runtime->pressure_alarm, pump_speed,
                                         weight_x10, threshold_g, sequence, HAL_GetTick()) != 0U)
    {
        Pubinterface_HandlePumpPressureBlocked(binding->public_channel); /* 确认超限后只弹窗和蜂鸣，不清运行源也不停止联动手柄。 */
    }

    return PumpPressureControl_ApplyReadiness(pump_speed, weight_x10); /* 真实压力超限保持设定输出，仅未就绪协议状态仍等待标定。 */
}

/*
 * 函数功能：按两类泵统一的 /1.51 基础比例换算 UART 速度，并对 200ml/min 以上流量应用实测补偿。
 * 输入参数：pump_flow 为压力处理后的注水泵或灌注泵业务流量。
 * 返回参数：200 及以下返回 /1.51 截断结果；200 以上返回按实测曲线反算并四舍五入后的 UART 速度。
 */
static uint16_t PumpBehavior_ConvertWaterFlowToUart(uint16_t pump_flow)
{
    uint32_t corrected_flow_x3; /* 保存补偿后等效流量的 3 倍值，避免 4/3 补偿过程提前丢失小数。 */
    const uint32_t uart_divisor_x3 = 151U * 3U; /* 两类泵统一使用 /1.51；乘 3 后与 corrected_flow_x3 的倍率对应。 */

    if (pump_flow <= 200U)
    {
        return (uint16_t)(((uint32_t)pump_flow * 100U) / 151U); /* 低流量统一按 /1.51 截断，注水泵与灌注泵发送相同速度。 */
    }

    corrected_flow_x3 = ((uint32_t)pump_flow * 4U) - 200U; /* 实测拟合为实际值=3/4设定值+50，反算得到等效流量=(4设定值-200)/3。 */

    return (uint16_t)(((corrected_flow_x3 * 100U) + (uart_divisor_x3 / 2U)) / uart_divisor_x3); /* 高流量结果四舍五入，减小驱动整数速度带来的流量误差。 */
}

/*
 * 函数功能：按泵类型限制设定值并换算驱动速度；正常运行检查压力，注水排空不检查压力。
 * 输入参数：binding 为本通道配置；runtime 为独立运行态；pump_type 为泵类型；pump_speed 为业务速度；drainage_active 为屏幕定时排空标志。
 * 返回参数：换算后的 16 位 UART 速度字段。
 */
static uint16_t PumpBehavior_ConvertOutput(const PumpBehaviorBinding_t *binding,
                                           PumpBehaviorRuntime_t *runtime,
                                           uint8_t pump_type,
                                           uint16_t *pump_speed,
                                           uint8_t drainage_active)
{
    uint16_t uart_data = 0U; /* 未识别类型或零速状态默认发送 0。 */

    switch (pump_type)
    {
        case DRAWWATER:
            runtime->business_direction = binding->draw_direction; /* 保留 A/B 原抽吸业务方向差异。 */
            if (*pump_speed > 15U)
            {
                *pump_speed = 15U; /* 抽吸泵的内部设定最多为 15，之后按 42 倍换成协议速度。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, runtime, *pump_speed); /* 抽吸沿用原阈值档位判断，超压只提示。 */
            uart_data = (uint16_t)(*pump_speed * 42U); /* 抽吸泵继续使用原 42 倍驱动换算。 */
            break;

        case INJECTWATER:
            runtime->business_direction = binding->inject_direction; /* 保留 A/B 原注水业务方向差异。 */
            if ((drainage_active == 0U) && (*pump_speed > PUMP_INJECTWATER_SPEED_MAX))
            {
                *pump_speed = PUMP_INJECTWATER_SPEED_MAX; /* 普通注水和手柄联动最多设为 300 mL/min；屏幕排空单独设定流量。 */
            }
            if ((drainage_active == 0U) && (binding->message->pedalDrainage_flag == false))
            {
                *pump_speed = PumpBehavior_ApplyPressure(binding, runtime, *pump_speed); /* 正常注水和手柄冷却超压只报警，泵和手柄继续按原请求运行。 */
            }
            else
            {
                PumpPressureControl_ResetAlarm(&runtime->pressure_alarm); /* 排空不读取压力，并清旧确认时间，退出排空后重新判断持续超限。 */
            }
            uart_data = PumpBehavior_ConvertWaterFlowToUart(*pump_speed); /* 注水泵按统一 /1.51 基础比例换算，并在 200 以上叠加实测补偿。 */
            break;

        case POURWATER:
            runtime->business_direction = binding->inject_direction; /* 灌注与注水继续使用同一业务方向。 */
            if (*pump_speed > 300U)
            {
                *pump_speed = 300U; /* 灌注设定流量最多为 300 mL/min。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, runtime, *pump_speed); /* 灌注同样只产生压力报警，保持设定流量和高流量补偿。 */
            uart_data = PumpBehavior_ConvertWaterFlowToUart(*pump_speed); /* 灌注泵调用同一函数，确保基础比例和高流量补偿均与注水泵一致。 */
            break;

        default:
            PumpPressureControl_ResetAlarm(&runtime->pressure_alarm); /* 停止、离线或驱动故障期间清报警确认记录，下一次运行重新采样。 */
            break; /* 未识别类型不改变上次方向，只发送零速，与旧代码一致。 */
    }

    return uart_data;
}

/*
 * 函数功能：维持注水泵 10 秒排空计时，并在计时结束时强制零输出。
 * 输入参数：message 为本通道泵状态；timing_drainage_active 为本周期开始时锁存的排空标志；pump_speed 和 uart_data 为待发布输出。
 * 返回参数：无。
 */
static void PumpBehavior_ServiceDrainage(pumpMessage_t *message,
                                         uint8_t timing_drainage_active,
                                         uint16_t *pump_speed,
                                         uint16_t *uart_data)
{
    if (timing_drainage_active == 0U)
    {
        return; /* 非排空周期不修改排空计数和正常运行输出。 */
    }

    if (message->timingDrainage_times >= PUMP_TIMING_DRAINAGE_TICKS)
    {
        *uart_data = 0U;                        /* 达到 10 秒后立即发送零速。 */
        *pump_speed = 0U;                       /* 屏幕使用的输出设定也清零，不再显示排空流量。 */
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
 * 函数功能：按协议组装 6 字节泵命令，计算 CRC16 校验后发送。
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

    binding->send_packet(frame, 6U); /* 每 25ms 发一帧，使驱动持续收到有效命令，避免触发 100ms 通信超时。 */
}

/*
 * 函数功能：仅在泵启停状态发生变化时刷新屏幕输出颜色，避免每 25ms 重复写屏。
 * 输入参数：binding 为本通道固定配置；runtime 为本通道颜色记录；uart_data 为本周期下发的速度，不是实测转速。
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
            LCD_Show_2byte_Number(binding->output_color_address, 0xFFFFU); /* 白色表示本周期下发零速命令。 */
        }
    }
    else if (runtime->output_was_stopped != 0U)
    {
        runtime->output_was_stopped = 0U; /* 记录已经恢复运行色，后续运行周期不重复写屏。 */
        LCD_Show_2byte_Number(binding->output_color_address, 0xFFE0U); /* 黄色表示下发了非零速度命令，不代表已收到转动反馈。 */
    }
}

/*
 * 函数功能：每 25ms 处理指定泵的速度请求、驱动故障、压力提示和排空计时，再发送命令并刷新显示。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue)
{
    const PumpBehaviorBinding_t *binding; /* 指向本周期固定的 A/B 硬件和状态绑定。 */
    PumpBehaviorRuntime_t *runtime;       /* 指向本通道独立跨周期状态。 */
    uint8_t pump_type = 0U;               /* 无运行请求时保持旧默认类型 0，不进入任何换算分支。 */
    uint16_t pump_speed = 0U;             /* 本周期待输出设定默认 0；不是驱动回报的实测速度。 */
    uint16_t uart_data;                   /* 本周期最终下发给泵驱动的速度字段。 */
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

    request_active = (binding->message->run_flag || binding->message->timingDrainage_flag) ? 1U : 0U; /* 先记住是否还有运行或排空请求，故障处理清标志后仍用此值判断是否允许重启。 */
    PumpBehavior_ParseDriverFeedback(channel, binding, runtime); /* 发送下一命令前先解析上一条命令的驱动回包。 */
    driver_fault_active = PumpBehavior_ServiceDriverFaultHold(binding, runtime, request_active); /* 故障锁存期持续清除运行源并输出零速。 */
    if (driver_fault_active == 0U)
    {
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

    uart_data = PumpBehavior_ConvertOutput(binding, runtime, pump_type, &pump_speed, drainage_active); /* 超压只报警，未就绪时输出零速，排空不查压力；驱动故障时始终输出零速。 */
    PumpBehavior_ServiceDrainage(binding->message, drainage_active, &pump_speed, &uart_data); /* 屏幕排空只保留 10 秒计时，排空阶段不再读取压力停泵结果。 */
    binding->publish_output_speed(pump_speed); /* 先保存本周期输出设定，再发送驱动帧和刷新颜色；该值不是实测转速。 */
    PumpBehavior_SendFrame(binding, uart_data, runtime->business_direction); /* 每个周期继续发送 6 字节帧。 */
    PumpBehavior_UpdateColor(binding, runtime, uart_data); /* 只在零速与非零速度之间切换时刷新对应泵颜色。 */
}
