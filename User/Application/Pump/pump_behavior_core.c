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
#define PUMP_DRIVER_ERROR_ALIGNMENT           6U    /* 失败态即使原故障为0也按未定位处理，不能当作普通待机。 */
#define PUMP_DRIVER_FEEDBACK_TIMEOUT_MS     100U    /* 超过四个任务周期没有可信回包即撤销运行请求，不能沿用旧READY。 */
#define PUMP_DRIVER_REALIGN_COMMAND         0x02U   /* 命令byte1的独立重定位操作码，速度字段必须为0，旧驱动会拒绝。 */
#define PUMP_DRIVER_CANCEL_COMMAND          0x03U   /* 全停专用零速操作码，可重复发送直到定位停止，不启动任何新一轮加力。 */
#define PUMP_DRIVER_REALIGN_ACK_MS          500U    /* 明确请求后最多等待这么久看到BUSY，不自动重发或再次加力。 */
#define PUMP_DRIVER_COLOR_ALIGNING          0x07FFU /* 现有流量数值显示青色：尚在定位或等待新零速握手。 */
#define PUMP_DRIVER_COLOR_FAULT             0xF800U /* 现有流量数值显示红色：定位失败、真实保护或通信不可用。 */
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
    uint16_t output_color;       /* 记住最近发送的RGB565颜色，定位状态变化也必须刷新。 */
    uint8_t business_direction;  /* 记住按泵类型选定的方向，停泵命令也使用同一方向。 */
    volatile uint8_t driver_fault_hold; /* 未就绪或驱动故障后阻止旧请求，READY且本周期无请求才解除。 */
    uint8_t last_feedback_error; /* 保存最近一帧 CRC 正确回包的原始故障码。 */
    volatile uint8_t alignment_notice_state; /* 保存已确认的标定提示状态；普通运行故障清除不产生新的成功提示。 */
    volatile uint8_t alignment_beep_result; /* 0无待播结果、1成功、2失败；两路全部结束后合并一次，失败优先。 */
    volatile uint8_t realign_phase; /* 0无请求，3/2先发两帧零速，1单次发重定位，4仅等待BUSY应答。 */
    volatile uint8_t cancel_phase; /* 0无取消，1必须先发一次，2重复零速取消直到可信反馈不再定位。 */
    uint32_t realign_tick;       /* 屏幕明确请求时刻，超时只结束等待，不重复请求新一轮加力。 */
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
    uint8_t alignment_state; /* 独立解析版本化状态，不再把故障码0当作已建立编码器零点。 */
    if ((channel >= PUMP_BEHAVIOR_CHANNEL_COUNT) || (frame == NULL))
    {
        return; /* 通道无效或没有回包数据时不写入，保留上一次有效反馈。 */
    }

    alignment_state = (uint8_t)(frame[4] & 0xF0U); /* 仅使用高半字节识别新协议状态。 */
    if (((alignment_state != PUMP_DRIVER_READY) && (alignment_state != PUMP_DRIVER_ALIGNING) &&
         (alignment_state != PUMP_DRIVER_ALIGN_FAILED)) || ((frame[4] & 0x0FU) > PUMP_DRIVER_ERROR_ALIGNMENT))
    {
        alignment_state = PUMP_DRIVER_UNAVAILABLE; /* 旧协议或未知组合只留诊断，不提供启动许可。 */
    }

    ++s_pump_feedback_version[channel]; /* 先发布奇数版本，阻止读取方复制半更新字段。 */
    __DMB(); /* 先让读取方看到“正在写入”，然后再修改数据字段。 */
    s_pump_feedback_snapshot[channel].feedback_tick_ms = HAL_GetTick(); /* 记录主控完成本帧协议校验的时刻。 */
    s_pump_feedback_snapshot[channel].sequence = (uint16_t)(s_pump_feedback_snapshot[channel].sequence + 1U); /* 每份有效回包递增，便于断点判断回包是否持续到达。 */
    s_pump_feedback_snapshot[channel].direction = frame[1]; /* 保留驱动端实际方向原始值，不用主控业务方向替代。 */
    s_pump_feedback_snapshot[channel].actual_speed = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]); /* 按协议高字节在前还原驱动实际速度。 */
    s_pump_feedback_snapshot[channel].raw_error = (alignment_state != PUMP_DRIVER_UNAVAILABLE) ? (uint8_t)(frame[4] & 0x0FU) : frame[4]; /* 新协议拆出原Err，旧协议保留原字节供检查混装。 */
    s_pump_feedback_snapshot[channel].alignment_state = alignment_state; /* 状态和故障必须来自同一CRC正确回包。 */
    s_pump_feedback_snapshot[channel].wire_status = frame[4]; /* 保留完整状态字节，方便核对两端版本。 */
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
 * 函数功能：取得仍在100ms有效期内的新协议反馈，旧驱动或陈旧READY均不能通过。
 * 输入参数：channel 为逻辑通道；snapshot 为调用方私有结果缓存。
 * 返回参数：新协议反馈完整且新鲜返回1，否则返回0。
 */
static uint8_t PumpBehavior_GetFreshFeedback(PumpBehaviorChannel_t channel,
                                             PumpDriverFeedbackSnapshot_t *snapshot)
{
    return ((PumpBehavior_CopyDriverFeedbackSnapshot(channel, snapshot) != 0U) &&
            (snapshot->alignment_state != PUMP_DRIVER_UNAVAILABLE) &&
            ((uint32_t)(HAL_GetTick() - snapshot->feedback_tick_ms) <= PUMP_DRIVER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U; /* 无符号时间差允许HAL计数回绕，旧0故障码不能通过。 */
}

/*
 * 函数功能：判断是否还有泵在标定或等待明确重标定请求应答，供普通蜂鸣静音使用。
 * 输入参数：无。
 * 返回参数：任一路尚未结束返回1；两路均无标定动作返回0，不影响真正报警的蜂鸣。
 */
uint8_t PumpBehavior_IsAlignmentBusy(void)
{
    uint8_t channel; /* 逐路检查，不能在A结束而B仍标定时提前发成功音。 */
    for (channel = 0U; channel < PUMP_BEHAVIOR_CHANNEL_COUNT; ++channel)
    {
        if ((s_pump_runtime[channel].alignment_notice_state == PUMP_DRIVER_ALIGNING) ||
            (s_pump_runtime[channel].realign_phase != 0U))
        {
            return 1U; /* 包含请求的两帧零速准备与BUSY应答等待，期间不播放普通操作音。 */
        }
    }
    return 0U; /* 未连接且从未报告BUSY的通道不阻塞另一泵的结果提示。 */
}

/*
 * 函数功能：两路标定全部结束后一次取走合并结果，避免两次成功单响被误听为失败双响。
 * 输入参数：无；仅由100ms蜂鸣任务消费结果。
 * 返回参数：0无结果或仍忙、1本批全部成功、2本批至少一路失败。
 */
uint8_t PumpBehavior_TakeAlignmentBeepResult(void)
{
    uint8_t channel; /* 本次合并涵盖两路已结束的标定，不修改任何运行授权。 */
    uint8_t result = 0U; /* 默认静音，重复反馈不能重复生成声音。 */
    taskENTER_CRITICAL(); /* 取走结果与跨任务的新重标定请求互斥，避免漏掉新一轮状态。 */
    if (PumpBehavior_IsAlignmentBusy() == 0U)
    {
        for (channel = 0U; channel < PUMP_BEHAVIOR_CHANNEL_COUNT; ++channel)
        {
            if (s_pump_runtime[channel].alignment_beep_result > result)
            {
                result = s_pump_runtime[channel].alignment_beep_result; /* 失败2优先于成功1，只播放一组结果音。 */
            }
            s_pump_runtime[channel].alignment_beep_result = 0U; /* 本批结果只消费一次，持续READY或FAILED不反复响。 */
        }
    }
    taskEXIT_CRITICAL(); /* 不在临界区操作蜂鸣GPIO、消息队列或阻塞等待。 */
    return result; /* 蜂鸣任务仍须遵守持续报警和限时报警的优先级。 */
}

/*
 * 函数功能：根据CRC正确的新协议状态记录标定开始和最终结果，自动补试BUSY保持静音。
 * 输入参数：runtime 为当前泵提示状态；snapshot 为本帧完整的驱动反馈。
 * 返回参数：无；只登记待播结果，不在25ms泵任务中直接发声。
 */
static void PumpBehavior_RecordAlignmentNotice(PumpBehaviorRuntime_t *runtime,
                                               const PumpDriverFeedbackSnapshot_t *snapshot)
{
    taskENTER_CRITICAL(); /* 与蜂鸣任务消费结果和屏幕登记重标定动作互斥更新。 */
    if ((snapshot->alignment_state == PUMP_DRIVER_ALIGNING) &&
        (snapshot->raw_error == PUMP_DRIVER_ERROR_NONE) && (runtime->cancel_phase == 0U))
    {
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGNING; /* 连续BUSY及自动第二次尝试均属于同一未结束标定。 */
        runtime->alignment_beep_result = 0U; /* 新一轮开始前清本通道旧结果，不能在重标定中补播上轮声音。 */
    }
    else if ((snapshot->alignment_state == PUMP_DRIVER_ALIGN_FAILED) ||
             ((runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING) && (snapshot->raw_error != PUMP_DRIVER_ERROR_NONE)))
    {
        if (runtime->alignment_notice_state != PUMP_DRIVER_ALIGN_FAILED)
        {
            runtime->alignment_beep_result = 2U; /* 最终失败或首次收到失败终态才登记双响，不在内部补试时提示失败。 */
        }
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGN_FAILED; /* 保留已提示终态，重复失败及超时后的取消应答不重复双响。 */
    }
    else if (snapshot->alignment_state == PUMP_DRIVER_READY)
    {
        if ((snapshot->raw_error == PUMP_DRIVER_ERROR_NONE) &&
            ((runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING) ||
             (runtime->alignment_notice_state == PUMP_DRIVER_UNAVAILABLE)))
        {
            runtime->alignment_beep_result = 1U; /* 只接受无故障READY；普通运行保护的BUSY带故障码，不作为新标定起点。 */
        }
        runtime->alignment_notice_state = PUMP_DRIVER_READY; /* 带故障READY也记住已有基准，后续只清故障时不误响标定成功。 */
    }
    taskEXIT_CRITICAL(); /* 未知协议不伪造标定结果，原运行门禁继续按不可用处理。 */
}

/*
 * 函数功能：集中检查泵驱动就绪与释放门禁，任何来源都不能缓存未就绪运行请求。
 * 输入参数：channel 为逻辑 A/B 通道。
 * 返回参数：新协议READY、无故障、反馈新鲜且旧请求已释放返回1，否则返回0。
 */
uint8_t PumpBehavior_DriverCanRun(PumpBehaviorChannel_t channel)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 使用同一版本快照，避免把故障前后状态混合。 */
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return 0U; /* 非法通道不访问运行数组，不产生任何运动授权。 */
    }
    return ((PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U) &&
            (snapshot.alignment_state == PUMP_DRIVER_READY) && (snapshot.raw_error == PUMP_DRIVER_ERROR_NONE) &&
            (s_pump_runtime[channel].driver_fault_hold == 0U) && (s_pump_runtime[channel].realign_phase == 0U) &&
            (s_pump_runtime[channel].cancel_phase == 0U)) ? 1U : 0U; /* 驱动READY也必须等主控清除完成边界前的请求和取消动作。 */
}

/*
 * 函数功能：接受屏幕明确的失败重定位操作，先零速再单次发送校准帧，完成后仍须新启动。
 * 输入参数：channel 为逻辑 A/B 通道；只允许屏幕失败恢复入口调用。
 * 返回参数：新请求已投递返回1；状态不符或已有请求返回0。
 */
uint8_t PumpBehavior_RequestRealign(PumpBehaviorChannel_t channel)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 明确请求必须针对最近确实失败的驱动。 */
    PumpBehaviorRuntime_t *runtime; /* 只修改当前通道的单槽请求，不能波及另一泵。 */
    uint8_t accepted = 0U; /* 默认不投递，避免离线或并发启动时误重定位。 */
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT ||
        PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U || snapshot.alignment_state != PUMP_DRIVER_ALIGN_FAILED)
    {
        return 0U; /* 定位中、已就绪或旧协议都不能通过此动作复位尝试次数。 */
    }
    runtime = &s_pump_runtime[channel]; /* 已检查数组边界，下面只操作本泵邮箱。 */
    taskENTER_CRITICAL(); /* 与25ms泵任务互斥发布请求阶段和时间，不在临界区发送串口或写屏。 */
    if ((runtime->realign_phase == 0U) && (runtime->cancel_phase == 0U) && (s_pump_binding[channel].message->online_flag != false) &&
        (s_pump_binding[channel].message->type != 0U) && (s_pump_binding[channel].message->run_flag == false) &&
        (s_pump_binding[channel].message->timingDrainage_flag == false) &&
        (WorkMessage.runflag_work == false) && (WorkMessage.alarm_flag == false))
    {
        runtime->realign_tick = HAL_GetTick(); /* 超时基准属于本次屏幕动作，不随周期通信刷新。 */
        runtime->driver_fault_hold = 1U; /* 请求即保持停止，不能混入已缓存的普通运行状态。 */
        runtime->realign_phase = 3U; /* 两个零速周期之后只发送一次独立校准帧，不自动重发。 */
        runtime->alignment_beep_result = 0U; /* 明确开始本通道新一轮时丢弃其旧提示，等待本次真实结果。 */
        accepted = 1U; /* 仅说明请求已登记，不表示物理条件满足或定位已成功。 */
    }
    taskEXIT_CRITICAL(); /* 立即恢复调度，硬件保护和普通停止不受等待应答影响。 */
    return accepted;
}

/*
 * 函数功能：明确全停撤销定位授权，跨任务只投递取消状态，不在调用任务发送泵串口。
 * 输入参数：channel 为逻辑 A/B 通道，普通停止保活不得调用。
 * 返回参数：无。
 */
void PumpBehavior_CancelAlignment(PumpBehaviorChannel_t channel)
{
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT){ return; } /* 非法通道不访问其它泵的控制状态。 */
    taskENTER_CRITICAL(); /* 取消、重定位撤销和运行门禁作为同一次全停动作发布。 */
    s_pump_runtime[channel].realign_phase = 0U; /* 清掉尚未发出或尚未应答的重定位，恢复条件后也不能继续旧请求。 */
    s_pump_runtime[channel].driver_fault_hold = 1U; /* 立即拒绝任何并发的新启动，等待泵任务停止输出。 */
    if (s_pump_runtime[channel].cancel_phase == 0U){ s_pump_runtime[channel].cancel_phase = 1U; } /* 至少实际发送一次，重复全停不重置等待状态。 */
    taskEXIT_CRITICAL(); /* 串口发送和应答等待留在原25ms泵任务，不占用临界区。 */
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
        if (rx_data[offset + 1U] > 1U)
        {
            ++offset; /* 实测方向不是0/1时不发布READY，继续寻找后续完整有效帧。 */
            continue;
        }

        PumpBehavior_RecordDriverFeedback(channel, &rx_data[offset]); /* 先保存原始反馈，断点可直接看方向、回报速度和故障码。 */
        PumpBehavior_RecordAlignmentNotice(runtime, &s_pump_feedback_snapshot[channel]); /* 标定提示跟随可信状态边沿，不跟随按键、连接或第一次内部失败。 */
        driver_error = s_pump_feedback_snapshot[channel].raw_error; /* 保护处理只使用拆出的原始故障，不把BUSY高位当故障。 */
        if ((s_pump_feedback_snapshot[channel].alignment_state == PUMP_DRIVER_ALIGN_FAILED) && (driver_error == PUMP_DRIVER_ERROR_NONE))
        {
            driver_error = PUMP_DRIVER_ERROR_ALIGNMENT; /* 失败状态始终停机并提示，不能因原Err已清零而放行。 */
        }
        if ((driver_error != PUMP_DRIVER_ERROR_NONE) && (runtime->last_feedback_error == PUMP_DRIVER_ERROR_NONE))
        {
            runtime->driver_fault_hold = 1U; /* 非零故障首帧立即锁存，后续正常回包不能自动复转。 */
            Pubinterface_HandlePumpDriverFault(binding->public_channel); /* 同步停本泵和各控制源，若为手柄冷却泵则联动停手柄。 */
            if ((runtime->alignment_notice_state != PUMP_DRIVER_ALIGNING) &&
                (runtime->alignment_notice_state != PUMP_DRIVER_ALIGN_FAILED))
            {
                if ((PumpBehavior_IsAlignmentBusy() != 0U) || (runtime->alignment_beep_result != 0U))
                {
                    runtime->alignment_beep_result = 2U; /* 另一泵仍标定或成功尚未播出时合并为失败双响，不能被静音吞掉或误报成功。 */
                }
                else
                {
                    SendDoubleBeepMessageIfIdle(); /* 正常运行保护后等待零速也回报BUSY带故障码，保留原故障双响，不算重新标定。 */
                }
            }
        }
        runtime->last_feedback_error = driver_error; /* 保存本次真实故障，BUSY和READY正常帧为0；同一失败不重复双响。 */
        if ((runtime->realign_phase == 4U) && (s_pump_feedback_snapshot[channel].alignment_state == PUMP_DRIVER_ALIGNING))
        {
            runtime->realign_phase = 0U; /* 只在已经发出独立命令后以BUSY应答结束等待，后续持续普通零速。 */
        }

        offset = (uint16_t)(offset + PUMP_DRIVER_FEEDBACK_FRAME_SIZE); /* 有效帧整帧跳过，继续处理 DMA 中可能粘连的下一帧。 */
    }
}

/*
 * 函数功能：定位、失联和故障期间清除请求；新READY之后至少经过一个无请求周期才能接受新启动。
 * 输入参数：channel 为逻辑通道；binding 为本通道配置；runtime 为本通道状态；request_active 为处理前的运行/排空请求。
 * 返回参数：本周期仍需故障停机返回 1；未锁存或已满足解锁条件返回 0。
 */
static uint8_t PumpBehavior_ServiceDriverFaultHold(PumpBehaviorChannel_t channel,
                                                   const PumpBehaviorBinding_t *binding,
                                                   PumpBehaviorRuntime_t *runtime,
                                                   uint8_t request_active)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 就绪和故障必须依据同一份新鲜回包，不依据设备识别耗时。 */
    uint8_t fresh = PumpBehavior_GetFreshFeedback(channel, &snapshot); /* 旧驱动、没有反馈或超过100ms都按不可用处理。 */
    if ((fresh == 0U) && (runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING))
    {
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGN_FAILED; /* 标定回包失联不能无限等成功；先结束本通道提示状态，下面仍发送明确取消。 */
        runtime->alignment_beep_result = 2U; /* 失联作为本轮失败加入合并结果，后到的FAILED取消应答不再次双响。 */
    }
    if ((fresh == 0U) || (snapshot.alignment_state != PUMP_DRIVER_READY) ||
        (snapshot.raw_error != PUMP_DRIVER_ERROR_NONE) || (runtime->realign_phase != 0U) || (runtime->cancel_phase != 0U))
    {
        if ((fresh == 0U) && (runtime->driver_fault_hold == 0U) && (snapshot.valid != 0U))
        {
            if ((PumpBehavior_IsAlignmentBusy() != 0U) || (runtime->alignment_beep_result != 0U))
            {
                runtime->alignment_beep_result = 2U; /* 等另一泵标定时本泵失联也合并为失败，不能在稍后播出已过期的成功音。 */
            }
            else
            {
                SendDoubleBeepMessageIfIdle(); /* 普通运行许可因反馈失联撤销时仍双响，上电从未有回包时不误报。 */
            }
        }
        if (fresh == 0U && snapshot.valid != 0U && snapshot.alignment_state == PUMP_DRIVER_ALIGNING)
        {
            PumpBehavior_CancelAlignment(channel); /* 单向回包丢失时普通零速仍会续约定位，必须明确取消未知进度的加力。 */
        }
        runtime->driver_fault_hold = 1U; /* BUSY也锁住完成边界前的旧请求，但不产生定位故障双响。 */
        if (request_active != 0U)
        {
            Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 撤销本泵、外控及必要的手柄冷却来源，连续脚踏必须先松开。 */
        }
        return 1U; /* 定位期间所有速度和排空路径都跳过，周期通信仍发送零速。 */
    }
    if (runtime->driver_fault_hold == 0U)
    {
        return 0U; /* 持续READY且没有旧请求锁存时，正常运行路径不改变。 */
    }
    if (request_active == 0U)
    {
        runtime->driver_fault_hold = 0U; /* 已确认新READY和请求释放，本周期仍发零速，下个新启动才可运行。 */
        return 1U;
    }

    Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 每周期清除本泵、外控和必要的手柄联动请求，阻止持续控制源复转。 */
    return 1U;
}

/*
 * 函数功能：优先确认全停取消，再推进屏幕重定位的两帧零速准备和单次控制帧。
 * 输入参数：channel 为逻辑通道；runtime 为本泵请求状态。
 * 返回参数：本周期独立零速操作码0x02或0x03；返回0时按普通零速/运行处理。
 */
static uint8_t PumpBehavior_TakeRealignCommand(PumpBehaviorChannel_t channel, PumpBehaviorRuntime_t *runtime)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 每次发送前重新确认驱动反馈，不能使用陈旧状态结束取消。 */
    if (runtime->cancel_phase != 0U)
    {
        if (runtime->cancel_phase == 2U && PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U &&
            (snapshot.alignment_state != PUMP_DRIVER_ALIGNING || snapshot.raw_error != PUMP_DRIVER_ERROR_NONE))
        {
            runtime->cancel_phase = 0U; /* 已发取消且驱动反馈已停止定位或进入保护，继续普通零速而非重新加力。 */
            return 0U;
        }
        runtime->cancel_phase = 2U; /* 未确认停止时每周期重复幂等取消，丢一帧不能留下仍被零速续约的定位。 */
        return PUMP_DRIVER_CANCEL_COMMAND; /* 全停优先于任何待发重定位，最后出口强制零速度。 */
    }
    if (runtime->realign_phase == 0U){ return 0U; } /* 没有明确屏幕请求，普通零速绝不能触发新校准轮。 */
    if ((uint32_t)(HAL_GetTick() - runtime->realign_tick) >= PUMP_DRIVER_REALIGN_ACK_MS ||
        PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U ||
        s_pump_binding[channel].message->online_flag == false ||
        WorkMessage.runflag_work != false || WorkMessage.alarm_flag != false)
    {
        runtime->realign_phase = 0U; /* 超时、离线或全局运行/报警撤销本次请求，不缓存到条件恢复后执行。 */
        return 0U;
    }
    if (runtime->realign_phase == 4U){ return 0U; } /* 已单次发出控制帧，只等BUSY或超时，不能反复复位两次尝试上限。 */
    if (snapshot.alignment_state != PUMP_DRIVER_ALIGN_FAILED)
    {
        runtime->realign_phase = 0U; /* 准备期间驱动状态改变就撤销未发请求，不能在READY时重置编码器。 */
        return 0U;
    }
    if (runtime->realign_phase > 1U)
    {
        runtime->realign_phase--; /* 3和2各对应一个完整零速发送周期，为驱动提供新的安全零速握手。 */
        return 0U;
    }
    runtime->realign_phase = 4U; /* 在发送前标记只等应答，防止并发重复投递导致无限重定位。 */
    return PUMP_DRIVER_REALIGN_COMMAND; /* 发送者必须把方向操作码改为0x02并强制两个速度字节为0。 */
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
 * 输入参数：binding 为本通道配置；uart_data 为驱动速度；business_direction 为原方向；control 为独立零速校准/取消操作码。
 * 返回参数：无。
 */
static void PumpBehavior_SendFrame(const PumpBehaviorBinding_t *binding,
                                   uint16_t uart_data,
                                   uint8_t business_direction,
                                   uint8_t control)
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
    if (control != 0U)
    {
        frame[1] = control; /* 独立校准或取消操作码不复用正反转，不会被新驱动解释成普通运行。 */
        uart_data = 0U; /* 即使上层错误带入速度，校准和全停帧在最后出口也只能携带零速。 */
    }
    frame[2] = (uint8_t)((uart_data >> 8) & 0xFFU); /* 速度高字节继续先发送。 */
    frame[3] = (uint8_t)(uart_data & 0xFFU);        /* 速度低字节紧随高字节。 */
    crc = Common_Crc16(frame, 4U);                  /* 对帧头、方向和速度字段计算 Modbus CRC16。 */
    frame[4] = (uint8_t)(crc & 0x00FFU);            /* 驱动协议第 5 字节固定发送 CRC 低字节。 */
    frame[5] = (uint8_t)((crc >> 8U) & 0x00FFU);    /* 驱动协议第 6 字节固定发送 CRC 高字节。 */

    binding->send_packet(frame, 6U); /* 每 25ms 发一帧，使驱动持续收到有效命令，避免触发 100ms 通信超时。 */
}

/*
 * 函数功能：用现有数值颜色显示定位、失败及启停状态，不新增未经屏幕工程验证的图片或文字VP。
 * 输入参数：channel 为逻辑通道；binding 为本泵配置；runtime 为颜色记录；uart_data 为下发速度，不是实测转速。
 * 返回参数：无。
 */
static void PumpBehavior_UpdateColor(PumpBehaviorChannel_t channel,
                                     const PumpBehaviorBinding_t *binding,
                                     PumpBehaviorRuntime_t *runtime,
                                     uint16_t uart_data)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 颜色与同一可信反馈的定位状态和故障对应。 */
    uint16_t color = (uart_data == 0U) ? 0xFFFFU : 0xFFE0U; /* 正常READY仍沿用白色停止、黄色命令运行。 */
    if (binding->message->type != 0U)
    {
        if (PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U || snapshot.raw_error != PUMP_DRIVER_ERROR_NONE ||
            snapshot.alignment_state == PUMP_DRIVER_ALIGN_FAILED)
        {
            color = PUMP_DRIVER_COLOR_FAULT; /* 已识别泵但驱动不可用或失败时红色常驻，不能被普通零速清掉。 */
        }
        else if (snapshot.alignment_state == PUMP_DRIVER_ALIGNING || runtime->driver_fault_hold != 0U)
        {
            color = PUMP_DRIVER_COLOR_ALIGNING; /* 定位及完成握手期间青色，不冒充正在输送液体的黄色。 */
        }
    }
    if (color != runtime->output_color)
    {
        runtime->output_color = color; /* 记录实际发送的状态色，同一状态不重复占用屏幕串口。 */
        LCD_Show_2byte_Number(binding->output_color_address, color); /* 使用已验证的A/B数值颜色SP，不影响其它报警弹窗。 */
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
    uint8_t control;                      /* 独立重定位或全停操作码；二者均不能携带运行速度。 */
    PumpDriverFeedbackSnapshot_t snapshot; /* 全局报警取消仍在BUSY的定位，不影响正常零速保活。 */

    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return; /* 非法通道不访问数组、不发送泵帧，避免错误调用影响现场输出。 */
    }

    binding = &s_pump_binding[channel]; /* 选定本周期固定通道差异。 */
    runtime = &s_pump_runtime[channel]; /* 选定本通道独立运行态。 */

    PumpBehavior_ReceiveSpeed(binding, message_queue); /* 每周期最多取一条速度消息，等待时间保持 0。 */

    request_active = (binding->message->run_flag || binding->message->timingDrainage_flag) ? 1U : 0U; /* 先记住是否还有运行或排空请求，故障处理清标志后仍用此值判断是否允许重启。 */
    PumpBehavior_ParseDriverFeedback(channel, binding, runtime); /* 发送下一命令前先解析上一条命令的驱动回包。 */
    if (WorkMessage.alarm_flag != false && (runtime->realign_phase != 0U ||
        (PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U && snapshot.alignment_state == PUMP_DRIVER_ALIGNING)))
    {
        PumpBehavior_CancelAlignment(channel); /* 全局报警不允许继续定位加力，也撤销尚未发送的校准请求。 */
    }
    driver_fault_active = PumpBehavior_ServiceDriverFaultHold(channel, binding, runtime, request_active); /* 定位、失联和故障期间统一撤销旧请求并维持零速。 */
    control = PumpBehavior_TakeRealignCommand(channel, runtime); /* 状态门禁之后优先全停，只有明确屏幕恢复才取出单次校准码。 */
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
    PumpBehavior_SendFrame(binding, uart_data, runtime->business_direction, control); /* 周期帧长度不变，明确校准或取消均强制独立零速帧。 */
    PumpBehavior_UpdateColor(channel, binding, runtime, uart_data); /* 根据真实定位状态和命令启停状态刷新对应泵颜色。 */
}
