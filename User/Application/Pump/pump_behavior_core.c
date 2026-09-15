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
#define PUMP_DRIVER_FEEDBACK_FRAME_SIZE       7U    /* 7字节回包依次为帧头、方向、实际速度高低字节、状态/故障字节和CRC低高字节。 */
#define PUMP_DRIVER_FEEDBACK_CRC_DATA_SIZE    5U    /* CRC16/MODBUS 覆盖回包前 5 字节。 */
#define PUMP_DRIVER_ERROR_NONE                0U    /* 故障码0表示驱动未报告故障；还须检查定位状态，并确认旧运行请求已清零，才能重新启动。 */
#define PUMP_DRIVER_ERROR_ALIGNMENT           6U    /* 主控用故障码6表示定位失败；即使回包低4位为0，也不允许开泵。 */
#define PUMP_DRIVER_FEEDBACK_TIMEOUT_MS     100U    /* 驱动反馈有效期100ms，相当于4个25ms泵任务周期；超时清掉运行请求，调大后失联停机更晚。 */
#define PUMP_DRIVER_REALIGN_COMMAND         0x02U   /* 命令第2字节为0x02时要求驱动重新定位，同时速度必须为0；该值与驱动协议配套，不作方向值使用。 */
#define PUMP_DRIVER_CANCEL_COMMAND          0x03U   /* 命令第2字节为0x03时取消定位，同时速度必须为0；未收到取消确认时继续发送，不要求驱动重新定位。 */
#define PUMP_DRIVER_REALIGN_ACK_MS          500U    /* 从屏幕要求重新定位起最多等500ms；期间收不到定位中状态就放弃请求，调大后等待更久，但不会重发定位命令。 */
#define PUMP_DRIVER_COLOR_ALIGNING          0x07FFU /* RGB565青色，用于表示驱动正在定位或主控尚未解除停机限制；仅改变屏幕流量文字颜色。 */
#define PUMP_DRIVER_COLOR_FAULT             0xF800U /* RGB565红色，用于表示定位失败、驱动故障或无有效反馈；仅改变屏幕流量文字颜色。 */
#define PUMP_DRIVER_FEEDBACK_RX_BUFFER_SIZE   UART5_MAX_PACKET_SIZE /* A/B 解析缓存按现有 UART5/7 共同 DMA 容量分配。 */

#if (UART5_MAX_PACKET_SIZE != UART7_MAX_PACKET_SIZE)
#error "Pump driver UART5/UART7 DMA buffer sizes must match"
#endif

/* 每路泵对应的运行数据、屏幕地址、方向和串口函数，让同一套处理代码可用于 A/B 两路。 */
typedef struct
{
    pumpMessage_t *message;                    /* 指向本泵的类型、运行请求、设定流量和压力数据。 */
    uint8_t public_channel;                     /* 报警提示和驱动故障处理使用的逻辑通道号。 */
    uint16_t output_color_address;              /* 本泵流量文字的颜色地址，显示任务向此地址写入RGB565颜色。 */
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
    uint8_t business_direction;  /* 保存本泵按类型选定的方向，发送普通零速命令时也使用该方向。 */
    volatile uint8_t driver_fault_hold; /* 1表示暂不允许启动；驱动已就绪、没有故障且旧运行/排空请求都清零后改为0。 */
    uint8_t last_feedback_error; /* 保存上次处理的故障码，用来判断本次是否刚出现故障；定位失败会统一记为6。 */
    volatile uint8_t alignment_notice_state; /* 保存已处理的定位中、成功或失败状态；用它避免同一结果每次回包都重复蜂鸣。 */
    volatile uint8_t alignment_beep_result; /* 等待蜂鸣任务读取的定位结果：0不提示、1成功单响、2失败双响；读走后清零，A/B有一路失败就双响。 */
    volatile uint8_t realign_phase; /* 重新定位步骤：0空闲；3、2各发一帧普通零速；1发一次0x02定位命令；4等待驱动报告定位中。 */
    volatile uint8_t cancel_phase; /* 取消定位步骤：0无请求；1尚未发送取消命令；2已发送，继续发0x03直到有效回包表明不再正常定位。 */
    uint32_t realign_tick;       /* 记录屏幕要求重新定位的毫秒时刻，用于500ms超时判断；周期通信不会重置这个时间。 */
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
    uint8_t alignment_state; /* 保存回包高4位表示的定位状态，必须与低4位故障码分别判断。 */
    if ((channel >= PUMP_BEHAVIOR_CHANNEL_COUNT) || (frame == NULL))
    {
        return; /* 通道无效或没有回包数据时不写入，保留上一次有效反馈。 */
    }

    alignment_state = (uint8_t)(frame[4] & 0xF0U); /* 取第5字节的高4位，识别就绪0x80、定位中0x90或失败0xA0。 */
    if (((alignment_state != PUMP_DRIVER_READY) && (alignment_state != PUMP_DRIVER_ALIGNING) &&
         (alignment_state != PUMP_DRIVER_ALIGN_FAILED)) || ((frame[4] & 0x0FU) > PUMP_DRIVER_ERROR_ALIGNMENT))
    {
        alignment_state = PUMP_DRIVER_UNAVAILABLE; /* 状态值不认识或故障码超出0到6时，只保存回包，不允许据此开泵。 */
    }

    ++s_pump_feedback_version[channel]; /* 先把更新计数改成奇数，通知读取任务暂时不要复制这组反馈。 */
    __DMB(); /* 先让读取方看到“正在写入”，然后再修改数据字段。 */
    s_pump_feedback_snapshot[channel].feedback_tick_ms = HAL_GetTick(); /* 记录主控完成本帧协议校验的时刻。 */
    s_pump_feedback_snapshot[channel].sequence = (uint16_t)(s_pump_feedback_snapshot[channel].sequence + 1U); /* 每份有效回包递增，便于断点判断回包是否持续到达。 */
    s_pump_feedback_snapshot[channel].direction = frame[1]; /* 保留驱动端实际方向原始值，不用主控业务方向替代。 */
    s_pump_feedback_snapshot[channel].actual_speed = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]); /* 按协议高字节在前还原驱动实际速度。 */
    s_pump_feedback_snapshot[channel].raw_error = (alignment_state != PUMP_DRIVER_UNAVAILABLE) ? (uint8_t)(frame[4] & 0x0FU) : frame[4]; /* 新协议只取低4位故障码；不认识的协议保留第5字节原值，方便检查驱动版本。 */
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
    uint8_t attempt; /* 最多尝试复制3次；若泵任务一直在更新反馈，本次就返回失败，不长时间等待。 */

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
        uint32_t version_before = s_pump_feedback_version[channel]; /* 复制前保存更新计数，偶数表示泵任务已写完全部反馈字段。 */
        uint32_t version_after; /* 复制后再次读取版本，确认字段属于同一回包。 */

        if ((version_before & 1U) != 0U)
        {
            continue; /* 对应泵任务正在写入时立即重试，不返回混合字段。 */
        }

        __DMB(); /* 先确认没有写入，再开始复制反馈字段。 */
        *snapshot = s_pump_feedback_snapshot[channel]; /* 把整帧反馈复制给调用方，后续判断不直接读取正在更新的共享数据。 */
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
 * 函数功能：读取最近100ms内收到且定位状态可识别的驱动反馈；过期或旧协议回包返回失败。
 * 输入参数：channel 为逻辑A/B泵通道；snapshot 用于接收这一帧反馈数据。
 * 返回参数：反馈可识别且未超过100ms返回1，否则返回0；返回1仍不代表驱动没有故障。
 */
static uint8_t PumpBehavior_GetFreshFeedback(PumpBehaviorChannel_t channel,
                                             PumpDriverFeedbackSnapshot_t *snapshot)
{
    return ((PumpBehavior_CopyDriverFeedbackSnapshot(channel, snapshot) != 0U) &&
            (snapshot->alignment_state != PUMP_DRIVER_UNAVAILABLE) &&
            ((uint32_t)(HAL_GetTick() - snapshot->feedback_tick_ms) <= PUMP_DRIVER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U; /* 检查回包是否过期；无符号减法可跨毫秒计数回绕，旧协议即使故障码为0也不能通过。 */
}

/*
 * 函数功能：检查A/B是否还有泵正在定位或准备重新定位；有时暂停普通按键提示音。
 * 输入参数：无。
 * 返回参数：任一路定位未结束返回1，两路都结束返回0；报警蜂鸣不受本函数影响。
 */
uint8_t PumpBehavior_IsAlignmentBusy(void)
{
    uint8_t channel; /* 逐路检查，不能在A结束而B仍标定时提前发成功音。 */
    for (channel = 0U; channel < PUMP_BEHAVIOR_CHANNEL_COUNT; ++channel)
    {
        if ((s_pump_runtime[channel].alignment_notice_state == PUMP_DRIVER_ALIGNING) ||
            (s_pump_runtime[channel].realign_phase != 0U))
        {
            return 1U; /* 准备发送定位命令、等待定位中回包及实际定位期间，都先不播放普通按键音。 */
        }
    }
    return 0U; /* 两路都没有定位记录或待执行的定位请求，允许播放结果提示。 */
}

/*
 * 函数功能：两路标定全部结束后一次取走合并结果，避免两次成功单响被误听为失败双响。
 * 输入参数：无；由每100ms执行一次的蜂鸣任务调用。
 * 返回参数：0表示仍有泵在定位或没有新结果；1表示只收到成功结果；2表示至少一台泵失败。
 */
uint8_t PumpBehavior_TakeAlignmentBeepResult(void)
{
    uint8_t channel; /* 依次读取A/B等待提示的定位结果，不修改两泵的运行请求。 */
    uint8_t result = 0U; /* 默认静音，重复反馈不能重复生成声音。 */
    taskENTER_CRITICAL(); /* 读取并清零A/B结果时禁止任务切换，防止屏幕任务同时开始新一轮定位。 */
    if (PumpBehavior_IsAlignmentBusy() == 0U)
    {
        for (channel = 0U; channel < PUMP_BEHAVIOR_CHANNEL_COUNT; ++channel)
        {
            if (s_pump_runtime[channel].alignment_beep_result > result)
            {
                result = s_pump_runtime[channel].alignment_beep_result; /* 失败2优先于成功1，只播放一组结果音。 */
            }
            s_pump_runtime[channel].alignment_beep_result = 0U; /* 结果已交给蜂鸣任务，立即清零，避免下周期重复播放同一结果。 */
        }
    }
    taskEXIT_CRITICAL(); /* 不在临界区操作蜂鸣GPIO、消息队列或阻塞等待。 */
    return result; /* 蜂鸣任务仍须遵守持续报警和限时报警的优先级。 */
}

/*
 * 函数功能：根据有效回包记录定位开始、成功或失败，供蜂鸣任务稍后播放提示。
 * 输入参数：runtime 保存本泵的定位和提示记录；snapshot 是刚收到并通过校验的反馈。
 * 返回参数：无；只登记待播结果，不在25ms泵任务中直接发声。
 */
static void PumpBehavior_RecordAlignmentNotice(PumpBehaviorRuntime_t *runtime,
                                               const PumpDriverFeedbackSnapshot_t *snapshot)
{
    taskENTER_CRITICAL(); /* 更新定位提示记录时禁止任务切换，避免蜂鸣任务同时读走结果或屏幕任务同时重试。 */
    if ((snapshot->alignment_state == PUMP_DRIVER_ALIGNING) &&
        (snapshot->raw_error == PUMP_DRIVER_ERROR_NONE) && (runtime->cancel_phase == 0U))
    {
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGNING; /* 回包表示定位中且没有故障，继续等待最终结果，不按收到的回包次数蜂鸣。 */
        runtime->alignment_beep_result = 0U; /* 新一轮开始前清本通道旧结果，不能在重标定中补播上轮声音。 */
    }
    else if ((snapshot->alignment_state == PUMP_DRIVER_ALIGN_FAILED) ||
             ((runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING) && (snapshot->raw_error != PUMP_DRIVER_ERROR_NONE)))
    {
        if (runtime->alignment_notice_state != PUMP_DRIVER_ALIGN_FAILED)
        {
            runtime->alignment_beep_result = 2U; /* 第一次得知定位失败时记下双响提示；连续收到相同失败结果不重复登记。 */
        }
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGN_FAILED; /* 记住本泵已判定定位失败，后续相同失败回包不再增加提示音。 */
    }
    else if (snapshot->alignment_state == PUMP_DRIVER_READY)
    {
        if ((snapshot->raw_error == PUMP_DRIVER_ERROR_NONE) &&
            ((runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING) ||
             (runtime->alignment_notice_state == PUMP_DRIVER_UNAVAILABLE)))
        {
            runtime->alignment_beep_result = 1U; /* 原来在定位或尚无定位记录，本次收到无故障的就绪回包，登记一次成功提示。 */
        }
        runtime->alignment_notice_state = PUMP_DRIVER_READY; /* 保存驱动已就绪的状态；若同时有故障则不播成功音，之后故障清除也不重复报定位成功。 */
    }
    taskEXIT_CRITICAL(); /* 结束提示记录更新；不认识的定位状态不会产生成功或失败提示。 */
}

/*
 * 函数功能：检查本泵是否允许接受新的启动请求，供屏幕、脚踏联动和外控共同判断。
 * 输入参数：channel 为逻辑 A/B 通道。
 * 返回参数：反馈未超时、驱动已就绪且无故障、主控停机限制已解除、无定位或取消请求时返回1，否则返回0。
 */
uint8_t PumpBehavior_DriverCanRun(PumpBehaviorChannel_t channel)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 使用同一帧反馈检查故障和就绪状态，避免把两次回包的数据混在一起。 */
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return 0U; /* 不是A/B泵通道时直接返回不允许启动，避免数组越界。 */
    }
    return ((PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U) &&
            (snapshot.alignment_state == PUMP_DRIVER_READY) && (snapshot.raw_error == PUMP_DRIVER_ERROR_NONE) &&
            (s_pump_runtime[channel].driver_fault_hold == 0U) && (s_pump_runtime[channel].realign_phase == 0U) &&
            (s_pump_runtime[channel].cancel_phase == 0U)) ? 1U : 0U; /* 即使驱动已就绪，主控仍须完成旧请求清理和定位取消，才能允许下一次启动。 */
}

/*
 * 函数功能：屏幕要求失败泵重试定位时，登记先发两帧零速、再发一次定位命令的步骤；本次不启动输液。
 * 输入参数：channel 为逻辑 A/B 通道；只允许屏幕失败恢复入口调用。
 * 返回参数：已记下重新定位请求返回1；泵状态不允许或已有待处理请求时返回0。
 */
uint8_t PumpBehavior_RequestRealign(PumpBehaviorChannel_t channel)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 保存最近有效回包，只允许定位失败的泵执行这次重试。 */
    PumpBehaviorRuntime_t *runtime; /* 指向本泵的定位步骤和计时记录，不修改另一台泵。 */
    uint8_t accepted = 0U; /* 先按未接受处理，只有全部条件通过后才改为1。 */
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT ||
        PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U || snapshot.alignment_state != PUMP_DRIVER_ALIGN_FAILED)
    {
        return 0U; /* 通道错误、反馈过期或并非定位失败时，不登记重试。 */
    }
    runtime = &s_pump_runtime[channel]; /* 已确认通道号有效，取出该泵专用的定位记录。 */
    taskENTER_CRITICAL(); /* 禁止任务切换，保证泵任务读到配套的步骤和开始时间；这里不发送串口数据。 */
    if ((runtime->realign_phase == 0U) && (runtime->cancel_phase == 0U) && (s_pump_binding[channel].message->online_flag != false) &&
        (s_pump_binding[channel].message->type != 0U) && (s_pump_binding[channel].message->run_flag == false) &&
        (s_pump_binding[channel].message->timingDrainage_flag == false) &&
        (WorkMessage.runflag_work == false) && (WorkMessage.alarm_flag == false))
    {
        runtime->realign_tick = HAL_GetTick(); /* 从本次屏幕操作开始计算500ms等待时间，不随泵任务运行而延后。 */
        runtime->driver_fault_hold = 1U; /* 重新定位期间禁止普通开泵，避免旧运行请求使泵开始输液。 */
        runtime->realign_phase = 3U; /* 两个零速周期之后只发送一次独立校准帧，不自动重发。 */
        runtime->alignment_beep_result = 0U; /* 明确开始本通道新一轮时丢弃其旧提示，等待本次真实结果。 */
        accepted = 1U; /* 仅说明请求已登记，不表示物理条件满足或定位已成功。 */
    }
    taskEXIT_CRITICAL(); /* 定位请求记录完成后恢复任务切换，具体发送由后续泵任务处理。 */
    return accepted;
}

/*
 * 函数功能：全停时清掉本泵待执行的定位请求，并记下取消定位要求；实际命令由泵任务发送。
 * 输入参数：channel 为逻辑A/B泵通道；只用于全停或异常取消，不用于正常待机时的周期零速通信。
 * 返回参数：无。
 */
void PumpBehavior_CancelAlignment(PumpBehaviorChannel_t channel)
{
    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT){ return; } /* 非法通道不访问其它泵的控制状态。 */
    taskENTER_CRITICAL(); /* 禁止任务切换，确保取消定位、清除重试和禁止启动这三项记录一起更新。 */
    s_pump_runtime[channel].realign_phase = 0U; /* 清掉尚未发出或尚未应答的重定位，恢复条件后也不能继续旧请求。 */
    s_pump_runtime[channel].driver_fault_hold = 1U; /* 立即拒绝任何并发的新启动，等待泵任务停止输出。 */
    if (s_pump_runtime[channel].cancel_phase == 0U){ s_pump_runtime[channel].cancel_phase = 1U; } /* 尚未要求取消时记为待发送；重复全停不把已经发送的步骤退回起点。 */
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
        uint8_t driver_error; /* 本帧的驱动故障码；新协议从第5字节低4位取出，定位失败另外按故障6处理。 */

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
            ++offset; /* 方向字节不是0或1时丢弃本帧，从下一字节继续寻找有效回包。 */
            continue;
        }

        PumpBehavior_RecordDriverFeedback(channel, &rx_data[offset]); /* 先保存原始反馈，断点可直接看方向、回报速度和故障码。 */
        PumpBehavior_RecordAlignmentNotice(runtime, &s_pump_feedback_snapshot[channel]); /* 按本帧定位状态更新提示记录；只有开始或结果发生变化时才登记新的提示音。 */
        driver_error = s_pump_feedback_snapshot[channel].raw_error; /* 故障处理使用低4位故障码，不把高4位的定位中状态误当成故障。 */
        if ((s_pump_feedback_snapshot[channel].alignment_state == PUMP_DRIVER_ALIGN_FAILED) && (driver_error == PUMP_DRIVER_ERROR_NONE))
        {
            driver_error = PUMP_DRIVER_ERROR_ALIGNMENT; /* 驱动报告定位失败时统一按故障6处理，即使低4位为0也禁止运行。 */
        }
        if ((driver_error != PUMP_DRIVER_ERROR_NONE) && (runtime->last_feedback_error == PUMP_DRIVER_ERROR_NONE))
        {
            runtime->driver_fault_hold = 1U; /* 第一次收到非零故障就禁止启动；后续故障清零还须等旧运行请求清掉，不能自动恢复转动。 */
            Pubinterface_HandlePumpDriverFault(binding->public_channel); /* 同步停本泵和各控制源，若为手柄冷却泵则联动停手柄。 */
            if ((runtime->alignment_notice_state != PUMP_DRIVER_ALIGNING) &&
                (runtime->alignment_notice_state != PUMP_DRIVER_ALIGN_FAILED))
            {
                if ((PumpBehavior_IsAlignmentBusy() != 0U) || (runtime->alignment_beep_result != 0U))
                {
                    runtime->alignment_beep_result = 2U; /* 另一泵还在定位或成功提示尚未播放时，把待播结果改为失败双响，避免稍后误报成功。 */
                }
                else
                {
                    SendDoubleBeepMessageIfIdle(); /* 这是正常运行期间新出现的故障，使用普通故障双响；不把带故障的定位中状态当成重新定位。 */
                }
            }
        }
        runtime->last_feedback_error = driver_error; /* 记住本次故障码，下次只在无故障变为有故障时再次处理，避免连续相同回包重复双响。 */
        if ((runtime->realign_phase == 4U) && (s_pump_feedback_snapshot[channel].alignment_state == PUMP_DRIVER_ALIGNING))
        {
            runtime->realign_phase = 0U; /* 已经发过0x02命令且收到定位中回包，结束命令应答等待，随后继续发送普通零速。 */
        }

        offset = (uint16_t)(offset + PUMP_DRIVER_FEEDBACK_FRAME_SIZE); /* 有效帧整帧跳过，继续处理 DMA 中可能粘连的下一帧。 */
    }
}

/*
 * 函数功能：驱动未就绪或有故障时清掉开泵请求；恢复后先确认旧请求已清零，再允许下一次启动。
 * 输入参数：channel 为A/B泵通道；binding 为本泵配置；runtime 为停机记录；request_active 表示本周期开始时是否有运行或排空请求。
 * 返回参数：本周期必须发送零速返回1；可以按运行请求计算输出时返回0，解除停机限制的当周期仍返回1。
 */
static uint8_t PumpBehavior_ServiceDriverFaultHold(PumpBehaviorChannel_t channel,
                                                   const PumpBehaviorBinding_t *binding,
                                                   PumpBehaviorRuntime_t *runtime,
                                                   uint8_t request_active)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 用同一帧未超时的驱动回包判断就绪和故障，不能只看压力板是否识别到泵。 */
    uint8_t fresh = PumpBehavior_GetFreshFeedback(channel, &snapshot); /* 旧驱动、没有反馈或超过100ms都按不可用处理。 */
    if ((fresh == 0U) && (runtime->alignment_notice_state == PUMP_DRIVER_ALIGNING))
    {
        runtime->alignment_notice_state = PUMP_DRIVER_ALIGN_FAILED; /* 定位期间反馈超时，先把本泵结果记为失败，后续发送取消定位命令。 */
        runtime->alignment_beep_result = 2U; /* 记下一次失败提示；以后收到取消后的失败回包，也不为同一次故障重复双响。 */
    }
    if ((fresh == 0U) || (snapshot.alignment_state != PUMP_DRIVER_READY) ||
        (snapshot.raw_error != PUMP_DRIVER_ERROR_NONE) || (runtime->realign_phase != 0U) || (runtime->cancel_phase != 0U))
    {
        if ((fresh == 0U) && (runtime->driver_fault_hold == 0U) && (snapshot.valid != 0U))
        {
            if ((PumpBehavior_IsAlignmentBusy() != 0U) || (runtime->alignment_beep_result != 0U))
            {
                runtime->alignment_beep_result = 2U; /* 本泵失联时将待播定位结果改为失败，避免另一泵结束后播放旧的成功音。 */
            }
            else
            {
                SendDoubleBeepMessageIfIdle(); /* 之前收到过有效回包、现在首次失联时提示双响；上电后从未收到反馈不走此提示。 */
            }
        }
        if (fresh == 0U && snapshot.valid != 0U && snapshot.alignment_state == PUMP_DRIVER_ALIGNING)
        {
            PumpBehavior_CancelAlignment(channel); /* 主控收不到定位反馈时，普通零速并不能取消定位；改为发送0x03，要求驱动退出定位。 */
        }
        runtime->driver_fault_hold = 1U; /* 驱动还在定位时也禁止开泵；这行只记停机限制，不把正常定位中状态当成新故障报警。 */
        if (request_active != 0U)
        {
            Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 清掉本泵和外控的运行请求；若本泵用于手柄冷却，也停止手柄并要求先松开控制再重试。 */
        }
        return 1U; /* 定位期间所有速度和排空路径都跳过，周期通信仍发送零速。 */
    }
    if (runtime->driver_fault_hold == 0U)
    {
        return 0U; /* 驱动已就绪、无故障且没有主控停机限制，本周期可以按运行请求输出。 */
    }
    if (request_active == 0U)
    {
        runtime->driver_fault_hold = 0U; /* 驱动已就绪且旧运行/排空请求都已清零，解除限制；本周期仍发零速，下次新请求才允许开泵。 */
        return 1U;
    }

    Pubinterface_ServicePumpDriverFaultHold(binding->public_channel); /* 旧运行请求仍未清零，继续撤销本泵及相关联动请求，避免驱动恢复后被持续踩踏直接启动。 */
    return 1U;
}

/*
 * 函数功能：决定本周期是否发送取消定位或重新定位命令；取消优先，重新定位前先发两帧普通零速。
 * 输入参数：channel 为逻辑A/B泵通道；runtime 保存本泵的定位步骤及开始时间。
 * 返回参数：0x02要求重新定位，0x03要求取消定位；0表示不发这两类命令，速度仍由本周期运行条件决定。
 */
static uint8_t PumpBehavior_TakeRealignCommand(PumpBehaviorChannel_t channel, PumpBehaviorRuntime_t *runtime)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 发送命令前重新读取反馈，只有未超时的回包才能确认取消已完成。 */
    if (runtime->cancel_phase != 0U)
    {
        if (runtime->cancel_phase == 2U && PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U &&
            (snapshot.alignment_state != PUMP_DRIVER_ALIGNING || snapshot.raw_error != PUMP_DRIVER_ERROR_NONE))
        {
            runtime->cancel_phase = 0U; /* 已发送取消，且有效回包不再表示无故障定位中，结束重复取消，随后按普通零速通信。 */
            return 0U;
        }
        runtime->cancel_phase = 2U; /* 尚未收到取消确认，每周期继续发同一个取消命令，避免因单帧丢失而留下未结束的定位。 */
        return PUMP_DRIVER_CANCEL_COMMAND; /* 优先返回0x03取消命令；发送函数会把速度字段强制清零。 */
    }
    if (runtime->realign_phase == 0U){ return 0U; } /* 没有屏幕重新定位请求时不发0x02，正常待机的零速命令不会开始新一轮定位。 */
    if ((uint32_t)(HAL_GetTick() - runtime->realign_tick) >= PUMP_DRIVER_REALIGN_ACK_MS ||
        PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U ||
        s_pump_binding[channel].message->online_flag == false ||
        WorkMessage.runflag_work != false || WorkMessage.alarm_flag != false)
    {
        runtime->realign_phase = 0U; /* 等待超时、反馈无效、泵离线、手柄运行或出现报警时，丢弃本次重试，不等条件恢复后自动执行。 */
        return 0U;
    }
    if (runtime->realign_phase == 4U){ return 0U; } /* 0x02定位命令已发过一次，后续只等定位中回包或超时，不重复发送。 */
    if (snapshot.alignment_state != PUMP_DRIVER_ALIGN_FAILED)
    {
        runtime->realign_phase = 0U; /* 发命令前若驱动已不再报告定位失败，就取消待发请求，避免对已经就绪的驱动重新定位。 */
        return 0U;
    }
    if (runtime->realign_phase > 1U)
    {
        runtime->realign_phase--; /* 步骤3、2各发送一次普通零速，减到1后下个周期才能发定位命令。 */
        return 0U;
    }
    runtime->realign_phase = 4U; /* 即将发出唯一一次定位命令，先把步骤设为4，后续周期只等待应答。 */
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
 * 函数功能：为本泵选择注水手柄压力曲线；单注水泵跟随选中手柄，双注水泵分别使用同侧手柄。
 * 输入参数：binding为本泵固定绑定；调用方须在临界区内调用，保证泵类型、在线标志和手柄型号一起读取。
 * 返回参数：PUMP_PRESSURE_PROFILE_*编号；非注水泵、无对应在线手柄或未覆盖型号返回LEGACY。
 */
static uint8_t PumpBehavior_GetPressureProfile(const PumpBehaviorBinding_t *binding)
{
    uint8_t hand_channel = WorkMessage.channel_work; /* 只有一个注水泵时，由当前选中的手柄决定供水管路。 */
    uint8_t hand_model = 0U; /* 未取得在线手柄型号时保持0，后面使用原阈值，不继承另一手柄的配置。 */

    if (binding->message->type != INJECTWATER)
    {
        return PUMP_PRESSURE_PROFILE_LEGACY; /* 灌注和抽吸不使用手柄注水孔实测表。 */
    }
    if ((pumpMessageA.type == INJECTWATER) && (pumpMessageB.type == INJECTWATER))
    {
        hand_channel = binding->public_channel; /* 双注水泵沿用同侧冷却关系，不能都套用当前选中手柄的阈值。 */
    }
    if ((hand_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline != false))
    {
        hand_model = MemoryMsgA.hand_model; /* 读取A手柄自己的型号，避免切通道时混用B手柄参数。 */
    }
    else if ((hand_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline != false))
    {
        hand_model = MemoryMsgB.hand_model; /* 单B注水泵也可能给A手柄供水；这里依据手柄通道而非泵字母读取型号。 */
    }

    switch (hand_model)
    {
        case EMBA_ONLINES: /* EMBA与EMBB注水口相同。 */
        case EMBB_ONLINES: /* 本次EM组实际测试型号。 */
        case EMBC_ONLINES: /* EMBC沿用同一EM注水口参数。 */
        case EMBD_ONLINES: /* EMBD沿用同一EM注水口参数。 */
            return PUMP_PRESSURE_PROFILE_INJECT_EM; /* EM四种型号共用按流量细分的EM报警线。 */
        case PXBA_ONLINES: /* PXBA与PXBB注水口相同。 */
        case PXBB_ONLINES: /* 本次PX组实际测试型号。 */
            return PUMP_PRESSURE_PROFILE_INJECT_PX; /* PX高正常压力单独使用PX表，不影响灌注。 */
        case TMBA_ONLINES: /* TMBA与TMBB注水口相同。 */
        case TMBB_ONLINES: /* 本次TM组实际测试型号。 */
            return PUMP_PRESSURE_PROFILE_INJECT_TM; /* 两种TM手柄共用TM报警线。 */
        case PX_YIM_ONLINES: /* PXYTM是一体磨手柄，本次已测试。 */
        case PX_YIP_ONLINES: /* PXYTP与PXYTM注水口相同。 */
            return PUMP_PRESSURE_PROFILE_INJECT_PXY; /* PXY一体式使用独立低正常压力表，不套用PX分体式。 */
        case COMMON_SOCKET_ONLINES: /* 公共接头按已装刀具且统一注水口的组合取值。 */
        case DHYTM_ONLINES: /* DHYTM与公共接头手柄注水口相同。 */
            return PUMP_PRESSURE_PROFILE_INJECT_COMMON; /* 两类组合共用公共接头实测表。 */
        default:
            return PUMP_PRESSURE_PROFILE_LEGACY; /* 无手柄或未覆盖型号保持原报警线，不擅自套用已测型号。 */
    }
}

/*
 * 函数功能：用新压力帧和对应手柄阈值判断是否提示超压；只有压力板未完成自标定时才把输出设为零。
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
    uint8_t pressure_profile; /* 本泵用途及对应手柄决定的报警曲线，不改变泵运行请求。 */

    taskENTER_CRITICAL(); /* 复制压力数据时禁止任务切换，避免压力接收任务更新到一半就被读走。 */
    weight_x10 = binding->message->weight_x10; /* 复制当前完整压力帧的重量值。 */
    threshold_g = binding->message->pressure_threshold; /* 复制同一帧的有效性字段。 */
    sequence = binding->message->seq; /* 最后复制同一帧序号，供持续超限确认使用。 */
    pressure_profile = PumpBehavior_GetPressureProfile(binding); /* 与压力数据一起读取本泵对应手柄，避免两路泵混用当前选中手柄。 */
    taskEXIT_CRITICAL(); /* 压力字段和手柄曲线读取完立即恢复正常调度，报警消息在临界区外发送。 */

    if (PumpPressureControl_UpdateAlarm(&runtime->pressure_alarm, pump_speed, pressure_profile,
                                         weight_x10, threshold_g, sequence, HAL_GetTick()) != 0U)
    {
        Pubinterface_HandlePumpPressureBlocked(binding->public_channel); /* 压力持续超限后只弹窗和蜂鸣，不清开泵请求，也不停止正在联动的手柄。 */
    }

    return PumpPressureControl_ApplyReadiness(pump_speed, weight_x10); /* 真实压力超限保持设定输出，仅未就绪协议状态仍等待标定。 */
}

/*
 * 函数功能：按原六泵测试及本轮A/B高流量复测，分段反算注水和灌注所需的UART速度。
 * 输入参数：pump_flow为压力处理后的目标流量，单位mL/min；201～300使用本轮高流量修正，200及以下保持原换算。
 * 返回参数：四舍五入后的UART整数速度；0仍返回0，未测范围沿用7ee5705原换算。
 */
static uint16_t PumpBehavior_ConvertWaterFlowToUart(uint16_t pump_flow)
{
    /* 《流量测试记录》每档六泵各测1分钟，测试固件为7ee5705。
     * 第一列是六个实测流量之和，即平均流量的6倍，保留小数而不使用浮点数。
     * 第二列是该次测试实际发送的UART速度，已包含旧版高流量补偿；查表后不能再次补偿。
     * 六泵共用此表，不按主机编号区分；接手柄后的流量仍需另行实测。
     */
    static const uint16_t calibration[][2] =
    {
        {17U, 2U},     /* 原设定4mL/min：六泵实测合计17，原命令2。 */
        {60U, 6U},     /* 原设定10mL/min：六泵实测合计60，原命令6。 */
        {99U, 10U},    /* 原设定16mL/min：六泵实测合计99，原命令10。 */
        {149U, 15U},   /* 原设定24mL/min：六泵实测合计149，原命令15。 */
        {188U, 19U},   /* 原设定30mL/min：六泵实测合计188，原命令19。 */
        {256U, 26U},   /* 原设定40mL/min：六泵实测合计256，原命令26。 */
        {324U, 33U},   /* 原设定50mL/min：六泵实测合计324，原命令33。 */
        {383U, 39U},   /* 原设定60mL/min：六泵实测合计383，原命令39。 */
        {450U, 46U},   /* 原设定70mL/min：六泵实测合计450，原命令46。 */
        {653U, 66U},   /* 原设定100mL/min：六泵实测合计653，原命令66。 */
        {978U, 99U},   /* 原设定150mL/min：六泵实测合计978，原命令99。 */
        {1265U, 132U}, /* 原设定200mL/min：六泵实测合计1265，原命令132。 */
        {1407U, 150U}, /* 原设定220mL/min：六泵实测合计1407，原命令150。 */
        {1547U, 168U}, /* 原设定240mL/min：六泵实测合计1547，原命令168。 */
        {1674U, 185U}, /* 原设定260mL/min：六泵实测合计1674，原命令185。 */
        {1910U, 221U}  /* 原设定300mL/min：六泵实测合计1910，原命令221。 */
    };
    uint32_t flow_x6; /* 目标流量乘6后与实测合计比较，不先截断六泵平均值。 */
    uint32_t flow_span; /* 相邻两档实测合计之差；表按递增顺序排列，插值分母大于0。 */
    uint32_t uart_numerator; /* 目标在本区间内增加的流量乘命令差，用32位保存中间乘积。 */
    uint8_t point; /* 插值区间右端点；从1开始，左端点为point-1。 */
    uint32_t corrected_flow_x3; /* 保存补偿后等效流量的 3 倍值，避免 4/3 补偿过程提前丢失小数。 */
    const uint32_t uart_divisor_x3 = 151U * 3U; /* 仅供未测范围沿用旧公式，不能再次用于已查表的结果。 */

    /* 本轮测试使用当前插值版固件，A/B平均实测为212.5、230.5、249、284mL/min。
     * 200档未新增实测，保留当前命令125作为衔接点；低流量继续使用下方原表。
     * 第一列为平均流量的2倍，第二列为本轮测试时实际发送的UART整数速度。
     */
    static const uint16_t high_calibration[][2] =
    {
        {400U, 125U}, /* 200mL/min衔接点：沿用当前命令，不作为新增实测数据。 */
        {425U, 139U}, /* 设定220：A泵211、B泵214，平均212.5，对应当前命令139。 */
        {461U, 154U}, /* 设定240：A泵227、B泵234，平均230.5，对应当前命令154。 */
        {498U, 170U}, /* 设定260：A泵245、B泵253，平均249，对应当前命令170。 */
        {568U, 204U}  /* 设定300：A泵281、B泵287，平均284，对应当前命令204。 */
    };
    uint32_t flow_x2; /* 高流量目标乘2后与A/B平均实测比较，保留0.5mL/min精度。 */

    if ((pump_flow > 200U) && (pump_flow <= 300U)) /* 仅本轮高流量使用新表，200及以下保持已有换算。 */
    {
        flow_x2 = (uint32_t)pump_flow * 2U; /* 目标与高流量表使用相同倍率。 */
        for (point = 1U; point < ((sizeof(high_calibration) / sizeof(high_calibration[0])) - 1U); ++point) /* 最多选择最后一段，供284以上外推。 */
        {
            if (flow_x2 <= high_calibration[point][0]) /* 找到包含目标的区间后停止搜索。 */
            {
                break; /* 使用本档与上一档的实测斜率反算UART速度。 */
            }
        }
        flow_span = (uint32_t)high_calibration[point][0] - high_calibration[point - 1U][0]; /* 各段流量递增，分母始终大于0。 */
        uart_numerator = (flow_x2 - high_calibration[point - 1U][0]) * (high_calibration[point][1] - high_calibration[point - 1U][1]); /* 按本段斜率求增加的命令；284～300沿最后一段外推，需复测。 */
        return (uint16_t)(high_calibration[point - 1U][1] + ((uart_numerator + (flow_span / 2U)) / flow_span)); /* 四舍五入后直接返回，不再叠加旧表补偿。 */
    }

    if ((pump_flow >= 4U) && (pump_flow <= 300U)) /* 只修正实测覆盖的设定范围，不外推低流量或更高流量。 */
    {
        flow_x6 = (uint32_t)pump_flow * 6U; /* 目标与六泵实测合计使用相同倍率。 */
        for (point = 1U; point < (sizeof(calibration) / sizeof(calibration[0])); ++point) /* 依次找出目标所在的相邻实测区间。 */
        {
            if (flow_x6 <= calibration[point][0]) /* 上一档低于目标，本档达到目标，按两档之间的流量比例反算速度。 */
            {
                flow_span = (uint32_t)calibration[point][0] - calibration[point - 1U][0]; /* 实测合计严格递增，不会除以0。 */
                uart_numerator = (flow_x6 - calibration[point - 1U][0]) * (calibration[point][1] - calibration[point - 1U][1]); /* 求相对左端应增加的命令量分子。 */
                return (uint16_t)(calibration[point - 1U][1] + ((uart_numerator + (flow_span / 2U)) / flow_span)); /* 四舍五入到最近整数命令，直接返回，避免重复套用旧补偿。 */
            }
        }
    }

    /* 正常4～300目标均已在表内返回；未测范围保留旧换算，零流量仍发送零速。 */
    if (pump_flow <= 200U)
    {
        return (uint16_t)(((uint32_t)pump_flow * 100U) / 151U); /* 未测低流量继续按 /1.51 截断，不凭本次数据改变0～3档。 */
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
            uart_data = PumpBehavior_ConvertWaterFlowToUart(*pump_speed); /* 注水及排空按目标流量查实测表，仅修正速度，不改变排空计时。 */
            break;

        case POURWATER:
            runtime->business_direction = binding->inject_direction; /* 灌注与注水继续使用同一业务方向。 */
            if (*pump_speed > 300U)
            {
                *pump_speed = 300U; /* 灌注设定流量最多为 300 mL/min。 */
            }
            *pump_speed = PumpBehavior_ApplyPressure(binding, runtime, *pump_speed); /* 灌注仍按目标流量判断压力，不能用修正后的UART命令代替流量。 */
            uart_data = PumpBehavior_ConvertWaterFlowToUart(*pump_speed); /* 灌注和注水共用流量曲线，高流量按本轮A/B平均实测修正。 */
            break;

        default:
            PumpPressureControl_ResetAlarm(&runtime->pressure_alarm); /* 停止、离线或驱动故障期间清报警确认记录，下一次运行重新采样。 */
            break; /* 未识别类型不改变上次方向，只发送零速，与旧代码一致。 */
    }

    return uart_data;
}

/*
 * 函数功能：维持注水泵 10 秒排空计时，并在计时结束时强制零输出。
 * 输入参数：message 为本泵状态；timing_drainage_active 表示本周期是否按定时排空处理；pump_speed 和uart_data 分别接收流量设定和驱动速度。
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
        *uart_data = 0U;                        /* 达到10秒后将待发送速度改为0，由本周期末尾的发送函数下发。 */
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
 * 函数功能：改变屏幕流量文字颜色，显示本泵正在定位、发生故障、已停止或已发送运行命令。
 * 输入参数：channel 为逻辑通道；binding 为本泵配置；runtime 为颜色记录；uart_data 为下发速度，不是实测转速。
 * 返回参数：无。
 */
static void PumpBehavior_UpdateColor(PumpBehaviorChannel_t channel,
                                     const PumpBehaviorBinding_t *binding,
                                     PumpBehaviorRuntime_t *runtime,
                                     uint16_t uart_data)
{
    PumpDriverFeedbackSnapshot_t snapshot; /* 保存未超时的驱动反馈，用于判断应显示定位颜色还是故障颜色。 */
    uint16_t color = (uart_data == 0U) ? 0xFFFFU : 0xFFE0U; /* 默认零速命令显示白色、非零命令显示黄色；黄色不等于已经测得泵转动。 */
    if (binding->message->type != 0U)
    {
        if (PumpBehavior_GetFreshFeedback(channel, &snapshot) == 0U || snapshot.raw_error != PUMP_DRIVER_ERROR_NONE ||
            snapshot.alignment_state == PUMP_DRIVER_ALIGN_FAILED)
        {
            color = PUMP_DRIVER_COLOR_FAULT; /* 已识别到泵，但驱动反馈无效、定位失败或有故障时显示红色，即使发送零速也保持红色。 */
        }
        else if (snapshot.alignment_state == PUMP_DRIVER_ALIGNING || runtime->driver_fault_hold != 0U)
        {
            color = PUMP_DRIVER_COLOR_ALIGNING; /* 驱动在定位或主控还未解除停机限制时显示青色，不按普通非零命令显示黄色。 */
        }
    }
    if (color != runtime->output_color)
    {
        runtime->output_color = color; /* 记录实际发送的状态色，同一状态不重复占用屏幕串口。 */
        LCD_Show_2byte_Number(binding->output_color_address, color); /* 向本泵流量文字的颜色地址写入颜色值，不修改报警弹窗内容。 */
    }
}

/*
 * 函数功能：每 25ms 处理指定泵的速度请求、驱动故障、压力提示和排空计时，再发送命令并刷新显示。
 * 输入参数：channel 为逻辑 A/B 通道；message_queue 为该通道原有的独立 FreeRTOS 消息队列。
 * 返回参数：无。
 */
void PumpBehaviorCore_Run(PumpBehaviorChannel_t channel, QueueHandle_t message_queue)
{
    const PumpBehaviorBinding_t *binding; /* 指向本泵使用的状态数据、串口函数、方向和屏幕地址。 */
    PumpBehaviorRuntime_t *runtime;       /* 指向本泵的停机、定位、压力计时和颜色记录。 */
    uint8_t pump_type = 0U;               /* 无运行请求时保持旧默认类型 0，不进入任何换算分支。 */
    uint16_t pump_speed = 0U;             /* 本周期待输出设定默认 0；不是驱动回报的实测速度。 */
    uint16_t uart_data;                   /* 本周期最终下发给泵驱动的速度字段。 */
    uint8_t drainage_active;              /* 记下本周期是否执行10秒排空，后续流量计算和排空计时使用同一个值。 */
    uint8_t request_active;               /* 运行或排空任一有效都视为有输出请求。 */
    uint8_t driver_fault_active;          /* 1表示本周期不允许开泵，包括定位、失联、故障及刚解除停机限制的情况。 */
    uint8_t control;                      /* 独立重定位或全停操作码；二者均不能携带运行速度。 */
    PumpDriverFeedbackSnapshot_t snapshot; /* 报警出现时读取定位反馈，用于判断是否还需要发送取消定位命令。 */

    if (channel >= PUMP_BEHAVIOR_CHANNEL_COUNT)
    {
        return; /* 非法通道不访问数组、不发送泵帧，避免错误调用影响现场输出。 */
    }

    binding = &s_pump_binding[channel]; /* 按A/B通道号选择对应状态、串口和屏幕地址。 */
    runtime = &s_pump_runtime[channel]; /* 取出本泵上周期保存的停机、定位和计时记录。 */

    PumpBehavior_ReceiveSpeed(binding, message_queue); /* 每周期最多取一条速度消息，等待时间保持 0。 */

    request_active = (binding->message->run_flag || binding->message->timingDrainage_flag) ? 1U : 0U; /* 先记住是否还有运行或排空请求，故障处理清标志后仍用此值判断是否允许重启。 */
    PumpBehavior_ParseDriverFeedback(channel, binding, runtime); /* 发送下一命令前先解析上一条命令的驱动回包。 */
    if (WorkMessage.alarm_flag != false && (runtime->realign_phase != 0U ||
        (PumpBehavior_GetFreshFeedback(channel, &snapshot) != 0U && snapshot.alignment_state == PUMP_DRIVER_ALIGNING)))
    {
        PumpBehavior_CancelAlignment(channel); /* 全局报警不允许继续定位加力，也撤销尚未发送的校准请求。 */
    }
    driver_fault_active = PumpBehavior_ServiceDriverFaultHold(channel, binding, runtime, request_active); /* 定位、失联和故障期间统一撤销旧请求并维持零速。 */
    control = PumpBehavior_TakeRealignCommand(channel, runtime); /* 先检查是否需要取消定位，再处理屏幕提出的重试请求，确定本周期的特殊命令。 */
    if (driver_fault_active == 0U)
    {
        drainage_active = binding->message->timingDrainage_flag ? 1U : 0U; /* 驱动允许输出时，先记下本周期是否正在定时排空，再计算流量。 */
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
    PumpBehavior_ServiceDrainage(binding->message, drainage_active, &pump_speed, &uart_data); /* 处理屏幕排空的10秒计时；到时清掉排空请求和待发送速度，不按压力值提前结束。 */
    binding->publish_output_speed(pump_speed); /* 先保存本周期输出设定，再发送驱动帧和刷新颜色；该值不是实测转速。 */
    PumpBehavior_SendFrame(binding, uart_data, runtime->business_direction, control); /* 周期帧长度不变，明确校准或取消均强制独立零速帧。 */
    PumpBehavior_UpdateColor(channel, binding, runtime, uart_data); /* 根据真实定位状态和命令启停状态刷新对应泵颜色。 */
}
