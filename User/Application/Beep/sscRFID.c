#include "sscRFID.h"

#include "FreeRTOS.h"
#include "Pubinterface.h"
#include "board.h"
#include "board_profile.h"
#include "kernel_scheduler.h"
#include "queue.h"
#include "sscBEEP.h"
#include "delay.h"
#include "uart3.h"
#include "uart9.h"

#include <string.h>

#define RFID_FRAME_HEAD                 0xBBU /* RFID 模块帧头，所有回包都从该字节开始。 */
#define RFID_REGION_CHINA_920           0x01U /* 中国 920.125~924.875MHz 频段。 */
#define RFID_REGION_US                  0x02U /* 美国 902.25~927.75MHz 频段。 */
#define RFID_REGION_EUROPE              0x03U /* 欧洲 865.1~867.9MHz 频段。 */
#define RFID_REGION_CHINA_840           0x04U /* 中国 840.125~844.875MHz 频段。 */
#define RFID_REGION_KOREA               0x06U /* 韩国 917.1~923.3MHz 频段。 */

/* 修改下面这一项即可切换 RFID 地区频段；当前产品配置由该宏唯一决定。 */
#define RFID_REGION_SELECT              RFID_REGION_US

#if ((RFID_REGION_SELECT != RFID_REGION_CHINA_920) && \
     (RFID_REGION_SELECT != RFID_REGION_US) && \
     (RFID_REGION_SELECT != RFID_REGION_EUROPE) && \
     (RFID_REGION_SELECT != RFID_REGION_CHINA_840) && \
     (RFID_REGION_SELECT != RFID_REGION_KOREA))
#error "RFID_REGION_SELECT is invalid"
#endif

#define RFID_REGION_COMMAND_CHECKSUM    (0x08U + RFID_REGION_SELECT) /* 区域命令校验和随地区编号变化，避免切换地区后继续发送旧校验值。 */
#define RFID_FRAME_TAIL                 0x7EU /* RFID 模块帧尾，帧尾前一字节是累加和校验。 */
#define RFID_RESP_SECOND_EPC            0x02U /* EPC 读取回包第二字节，协议指定为 BB 02 22。 */
#define RFID_RESP_CMD_EPC               0x22U /* EPC 读取回包命令码。 */
#define RFID_PAYLOAD_EPC_OFFSET         8U    /* EPC 标签数据从整帧 data[8] 开始。 */
#define RFID_FRAME_LENGTH_FIELD_OFFSET  4U    /* RFID 帧第 4 下标字节保存模块数据区长度。 */
#define RFID_FRAME_MIN_SIZE             7U    /* 最短帧至少包含头、命令、长度、校验和尾。 */
#define RFID_QUEUE_LENGTH               4U    /* 队列保存少量屏幕/扫描层触发请求，避免按键抖动丢入口。 */
#define RFID_FAST_ATTEMPTS              10U   /* 快速识别最多尝试 10 个任务周期，覆盖基座刚上线。 */
#define RFID_NORMAL_ATTEMPTS            3U    /* 普通周期识别只尝试 3 次，避免空闲监测长期占用 RFID 串口。 */
#define RFID_DEBUG_BEEP_EVERY_UART_RESPONSE 0U /* 临时调试开关：1 表示当前 RFID 串口收到任意模块回包都蜂鸣，定位完成后关闭以避免影响业务调度。 */
#define RFID_DEBUG_BEEP_EVERY_VALID_READ 0U   /* 有效帧调试蜂鸣关闭，蜂鸣只在缓存确认新刀具信息时触发。 */

#if (UART9_MAX_PACKET_SIZE > UART3_MAX_PACKET_SIZE)
#define RFID_UART_PACKET_SIZE          UART9_MAX_PACKET_SIZE /* 双串口模式下临时解析缓存按更大的 UART 缓冲区预留。 */
#else
#define RFID_UART_PACKET_SIZE          UART3_MAX_PACKET_SIZE /* 当前 UART3/UART9 都是 150 字节，保持旧解析缓存容量不变。 */
#endif

typedef struct
{
    uint8_t channel;             /* 请求所属通道，只有 A/B 通道有效。 */
    bool start;                  /* true 启动一次读取，false 停止当前读取。 */
    bool fast_mode;              /* true 使用快速识别尝试次数，false 使用普通监测尝试次数。 */
    RfidReadSource_t source;     /* 本次读取来源，最终协议只允许 EPC。 */
    uint16_t generation;         /* 请求入队时的通道代次，清刀具后旧代次请求会被丢弃。 */
} RFIDMessage_t;

typedef struct
{
    bool valid;                                  /* true 表示该通道曾经读到过完整标签，可用于“同一刀具不重复蜂鸣”的历史判重。 */
    RfidReadSource_t source;                     /* 记录历史标签来源，当前固定为 EPC，保留字段用于判重完整性。 */
    uint8_t payload_length;                      /* 记录历史 payload 长度，避免不同协议长度误比较。 */
    uint8_t payload[RFID_PAYLOAD_EPC_LENGTH];    /* 保存最近一次 EPC 12 字节原始 payload，用于同一刀具判重。 */
} RfidPayloadMemory_t;

static uint8_t NO_MASK3_WRITE_EPC[7] = {0XBB, 0X00, 0X22, 0X00, 0X00, 0X22, 0X7E}; /* 无掩码读取 EPC 区。 */
static unsigned char hop_ch[] = {0XBB, 0X00, 0XAD, 0X00, 0X01, 0XFF, 0XAD, 0X7E}; /* 开启跳频，保持现有射频初始化流程。 */
static unsigned char  pa_gain10[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X03, 0Xe8, 0Xa3, 0X7E};//发射功率
static uint8_t region_set_command[] = {0xBBU, 0x00U, 0x07U, 0x00U, 0x01U, RFID_REGION_SELECT, RFID_REGION_COMMAND_CHECKSUM, 0x7EU}; /* 根据上方地区配置生成唯一一条有效区域命令，避免未选地区数组产生告警。 */

static kernel_task_t AUTOMODEGETDATATaskHandle;     /* RFID 轮询任务句柄，任务实际按请求工作。 */
static kernel_task_t CUTTERSCANTaskHandle;          /* 预留旧句柄，不启动，避免破坏工程外部引用假设。 */
static QueueHandle_t RFIDMsgQueue = NULL;           /* handlescan/屏幕自动识别键发送 RFID 请求的队列。 */
static RfidToolResult_t s_last_result[2];           /* A/B 通道最后一次有效 RFID 原始标签和缓存状态。 */
static RfidPayloadMemory_t s_payload_memory[2];      /* A/B 通道最近刀具 payload 记忆；清当前结果时不清它，用于同一刀具恢复时不重复蜂鸣。 */
static uint16_t s_result_sequence[2] = {0U, 0U};    /* A/B 通道成功结果序号，便于扫描层判断新结果。 */
static uint16_t s_presence_sequence[2] = {0U, 0U};  /* A/B 通道有效读到标签的存在序号，在线监测用它判断刀具头是否仍可读到。 */
static uint16_t s_request_generation[2] = {0U, 0U}; /* A/B 通道 RFID 请求代次，清刀具时递增以作废旧排队请求。 */
static bool s_request_active = false;               /* true 表示任务当前有一次未完成 RFID 请求。 */
static uint8_t s_request_channel = CHANNEL_NONE;    /* 当前活动请求所属通道。 */
static RfidReadSource_t s_request_source = RFID_READ_SOURCE_NONE; /* 当前活动请求读取来源，最终协议只允许 EPC。 */
static uint8_t s_request_attempts_left = 0U;        /* 当前请求剩余发送次数，归零后任务停止本次请求。 */
static bool s_request_fast_mode = false;            /* true 表示本次请求来自上线/重试快速识别，同标签也要让扫描层重新消费。 */

/*
 * 函数功能：把业务通道号转换成结果缓存数组下标。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；index 用于返回数组下标。
 * 返回参数：true 表示通道有效，false 表示通道无效。
 */
static bool Rfid_ChannelToIndex(uint8_t channel, uint8_t *index)
{
    if (index == NULL)
    {
        return false; /* 下标输出为空时不访问数组，避免异常调用破坏缓存。 */
    }

    if (channel == CHANNEL_A)
    {
        *index = 0U; /* A 通道使用缓存数组第 0 项。 */
        return true; /* 通道合法，调用方可以继续访问结果缓存。 */
    }

    if (channel == CHANNEL_B)
    {
        *index = 1U; /* B 通道使用缓存数组第 1 项。 */
        return true; /* 通道合法，调用方可以继续访问结果缓存。 */
    }

    return false; /* 非 A/B 通道不允许发起或保存 RFID 结果。 */
}

/*
 * 函数功能：判断读取来源是否为当前支持的 EPC。
 * 输入参数：source 为外部传入的读取来源。
 * 返回参数：true 表示来源有效，false 表示来源无效。
 */
static bool Rfid_IsSourceValid(RfidReadSource_t source)
{
    return (source == RFID_READ_SOURCE_EPC); /* 最终射频协议只允许 EPC 参与识别。 */
}

/*
 * 函数功能：按 RFID 硬件模式选择射频通道。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void Rfid_SelectHardwareChannel(uint8_t channel)
{
#if (RFID_USE_DUAL_UART_MODE == 0U)
    if (channel == CHANNEL_A)
    {
        R200_K8_SELECT_A(); /* 旧硬件 A 通道通过 R200-K8 高电平连接到 UART3。 */
        return; /* A 通道已经完成选通，不再继续判断 B 通道。 */
    }

    if (channel == CHANNEL_B)
    {
        R200_K8_SELECT_B(); /* 旧硬件 B 通道通过 R200-K8 低电平连接到 UART3。 */
    }
#else
    (void)channel; /* 双串口模式下 A/B RFID 已经固定到 UART3/UART9，不再驱动 R200-K8。 */
#endif
}

/*
 * 函数功能：按业务通道清空对应 RFID 串口接收缓存。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void Rfid_ClearChannelUartData(uint8_t channel)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    if (channel == CHANNEL_B)
    {
        Uart9_ClearRecvData(); /* 新硬件 B 通道独占 UART9，清缓存只影响 B 通道 RFID 回包。 */
        return; /* B 通道已经清理完成，不再误清 A 通道 UART3 缓存。 */
    }
#else
    (void)channel; /* 旧硬件 A/B 仍共用 UART3，通道参数只用于 R200-K8 选通。 */
#endif
    Uart3_ClearRecvData(); /* A 通道或旧硬件共用模式都清 UART3，防止旧回包污染下一次解析。 */
}

/*
 * 函数功能：按业务通道发送 RFID 命令帧。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；pData 指向命令缓冲区；Length 为命令长度。
 * 返回参数：无。
 */
static void Rfid_SendPacketForChannel(uint8_t channel, uint8_t *pData, uint16_t Length)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    if (channel == CHANNEL_B)
    {
        Uart9_SendPacket(pData, Length); /* 新硬件 B 通道命令固定从 UART9 发往 B 侧 RFID 模块。 */
        return; /* B 通道已经发送完成，不再从 UART3 重发。 */
    }
#else
    (void)channel; /* 旧硬件由 R200-K8 决定 UART3 当前连到 A 还是 B。 */
#endif
    Uart3_SendPacket(pData, Length); /* A 通道或旧硬件模式继续使用 UART3，保持原 RFID 命令时序。 */
}

/*
 * 函数功能：按业务通道读取对应 RFID 串口 DMA 缓存。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；data 指向调用方接收缓冲区。
 * 返回参数：本次读取到的字节数，0 表示没有新回包。
 */
static uint16_t Rfid_PeekChannelUartData(uint8_t channel, uint8_t *data)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    if (channel == CHANNEL_B)
    {
        return Uart9_DMARecvDataPeek(data); /* 新硬件 B 通道只解析 UART9 收到的 RFID 回包。 */
    }
#else
    (void)channel; /* 旧硬件共用 UART3，实际通道由 R200-K8 保证。 */
#endif
    return Uart3_DMARecvDataPeek(data); /* A 通道或旧硬件模式继续从 UART3 DMA 缓存取数据。 */
}

/*
 * 函数功能：判断本次 RFID 请求是否允许按当前通道状态执行。
 * 输入参数：channel 为请求读取的业务通道。
 * 返回参数：true 表示允许读取，false 表示当前状态禁止读取。
 */
static bool Rfid_IsRequestAllowedForCurrentSelection(uint8_t channel)
{
    bool requested_channel_online = false; /* 记录请求通道是否已经完成插入事件，用于区分后插入手柄上线前的 RFID 预读。 */

    if (WorkMessage.runflag_work == true)
    {
        return false; /* 电机运行中不允许切换模拟开关或刷新 RFID 参数，避免运行状态被识别流程打断。 */
    }

    if (channel == CHANNEL_A)
    {
        requested_channel_online = WorkMessage.Channel_Aonline; /* A 通道在线标志决定它是已在线监测还是刚插入预读。 */
    }
    else if (channel == CHANNEL_B)
    {
        requested_channel_online = WorkMessage.Channel_Bonline; /* B 通道在线标志决定它是已在线监测还是刚插入预读。 */
    }
    else
    {
        return false; /* 非 A/B 通道没有对应模拟开关位置，不能发起 RFID 读取。 */
    }

    if (requested_channel_online == false)
    {
        return true; /* 后插入 RFID 手柄在插入事件正式选中前需要先读刀具信息，因此允许上线前预读。 */
    }

    if ((WorkMessage.Channel_Aonline == true) && (WorkMessage.Channel_Bonline == true))
    {
        return (WorkMessage.channel_work == channel); /* 双手柄都在线后只允许当前选中通道继续读 RFID，避免非选中通道残留刷新屏幕。 */
    }

    return true; /* 单手柄在线时允许该通道继续在线监测 RFID，保持刀具头拔插检测能力。 */
}

/*
 * 函数功能：根据读取来源返回 EPC 原始标签数据长度。
 * 输入参数：source 为读取来源，当前只接受 RFID_READ_SOURCE_EPC。
 * 返回参数：EPC 协议标签数据长度，无效来源返回 0。
 */
static uint8_t Rfid_GetPayloadLength(RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        return RFID_PAYLOAD_EPC_LENGTH; /* 公共接头和 PXBA/PXBB 最终协议 EPC 固定提取 12 字节。 */
    }

    return 0U; /* 无效来源没有标签长度。 */
}

/*
 * 函数功能：根据读取来源返回 EPC 原始标签数据在整帧中的起始偏移。
 * 输入参数：source 为读取来源，当前只接受 RFID_READ_SOURCE_EPC。
 * 返回参数：EPC 协议指定的数据偏移，无效来源返回 0。
 */
static uint8_t Rfid_GetPayloadOffset(RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        return RFID_PAYLOAD_EPC_OFFSET; /* EPC 按 data[8..19] 提取。 */
    }

    return 0U; /* 无效来源不应继续提取。 */
}

/*
 * 函数功能：根据帧头第二字节和命令码判断当前回包来源。
 * 输入参数：frame 指向一帧起始位置。
 * 返回参数：识别出的来源；无法识别时返回 RFID_READ_SOURCE_NONE。
 */
static RfidReadSource_t Rfid_GetFrameSource(const uint8_t *frame)
{
    if (frame == NULL)
    {
        return RFID_READ_SOURCE_NONE; /* 防御空指针，避免解析异常缓冲。 */
    }

    if ((frame[0] == RFID_FRAME_HEAD) &&
        (frame[1] == RFID_RESP_SECOND_EPC) &&
        (frame[2] == RFID_RESP_CMD_EPC))
    {
        return RFID_READ_SOURCE_EPC; /* BB 02 22 是公共接头 EPC 回包。 */
    }

    return RFID_READ_SOURCE_NONE; /* 其它回包不作为刀具标签数据处理。 */
}

/*
 * 函数功能：校验 RFID 帧的模块数据长度字段。
 * 输入参数：buffer 为 DMA 缓冲，start_pos 为帧头位置，tail_pos 为帧尾位置。
 * 返回参数：true 表示长度字段与实际帧长度一致。
 */
static bool Rfid_LengthMatches(const uint8_t *buffer, uint16_t start_pos, uint16_t tail_pos)
{
    uint16_t actual_len; /* 保存根据帧尾反推的模块数据区长度。 */

    if (buffer == NULL)
    {
        return false; /* 缓冲为空时不能读取长度字段。 */
    }

    if (tail_pos <= (uint16_t)(start_pos + 6U))
    {
        return false; /* 帧太短时长度字段没有可信意义。 */
    }

    actual_len = (uint16_t)(tail_pos - start_pos - 6U); /* 模块协议长度字段不含头、类型、命令、长度、校验和尾。 */
    return (buffer[start_pos + RFID_FRAME_LENGTH_FIELD_OFFSET] == (uint8_t)actual_len); /* 长度一致才继续校验 checksum。 */
}

/*
 * 函数功能：校验 RFID 帧的累加和。
 * 输入参数：buffer 为 DMA 缓冲，start_pos 为帧头位置，tail_pos 为帧尾位置。
 * 返回参数：true 表示 checksum 正确。
 */
static bool Rfid_ChecksumMatches(const uint8_t *buffer, uint16_t start_pos, uint16_t tail_pos)
{
    uint16_t i;        /* 遍历参与累加的数据字节。 */
    uint16_t total = 0U; /* 保存除 BB 以外、checksum 以前所有字节的累加值。 */

    if (buffer == NULL)
    {
        return false; /* 缓冲为空时不能校验。 */
    }

    if (tail_pos <= (uint16_t)(start_pos + 2U))
    {
        return false; /* 帧尾前至少需要 checksum 和一个数据字节。 */
    }

    for (i = (uint16_t)(start_pos + 1U); i < (uint16_t)(tail_pos - 1U); i++)
    {
        total = (uint16_t)(total + buffer[i]); /* 协议要求排除 BB，从第二字节累加到 checksum 前一字节。 */
    }

    return (((uint8_t)total) == buffer[tail_pos - 1U]); /* 低 8 位必须等于帧尾前一字节。 */
}

/*
 * 函数功能：向指定业务通道发送 RFID EPC 读取命令。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；source 指定读取来源，当前只允许 EPC。
 * 返回参数：无。
 */
static void Rfid_SendReadCommand(uint8_t channel, RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        Rfid_SendPacketForChannel(channel, NO_MASK3_WRITE_EPC, (uint16_t)sizeof(NO_MASK3_WRITE_EPC)); /* A/B 通道按宏选择 UART3 或 UART9 发送 EPC 读取命令。 */
    }
}

/*
 * 函数功能：按完整原始标签数据更新通道 RFID 缓存。
 * 输入参数：result 指向本次解析出的 RFID 标签结果。
 * 返回参数：true 表示缓存更新成功，false 表示通道或结果无效。
 */
static bool Rfid_UpdateParsedCache(RfidToolResult_t *result)
{
    uint8_t index;                    /* 保存 A/B 通道缓存数组下标。 */
    RfidToolResult_t *last_result;     /* 指向该通道上一次有效 RFID 标签结果。 */
    RfidPayloadMemory_t *payload_memory; /* 指向该通道历史 payload 记忆，用它判断是否同一刀具。 */
    bool same_payload = false;         /* true 表示原始标签数据完全一致。 */

    if ((result == NULL) || (result->valid == false))
    {
        return false; /* 只缓存校验通过且结构完整的标签结果。 */
    }

    if (Rfid_ChannelToIndex(result->channel, &index) == false)
    {
        return false; /* 未知通道不能写入缓存，避免 A/B 结果串通道。 */
    }

    last_result = &s_last_result[index]; /* 当前可上报结果只表示刀具头此刻是否已确认在线。 */
    payload_memory = &s_payload_memory[index]; /* 历史 payload 记忆在刀具头临时移开时保留，用于避免同一信息重复蜂鸣。 */
    if ((payload_memory->valid == true) &&
        (payload_memory->source == result->source) &&
        (payload_memory->payload_length == result->payload_length) &&
        (memcmp(payload_memory->payload, result->payload, result->payload_length) == 0))
    {
        same_payload = true; /* 完整 EPC 原始数据一致时才视为同一个标签数据。 */
    }

    result->cache_hit = same_payload; /* 缓存命中时 handlescan 可以直接复用参数。 */
    result->changed = (bool)((payload_memory->valid == true) && (same_payload == false)); /* 有历史标签且原始数据不同才是刀具头变化。 */
    s_presence_sequence[index]++; /* 只要本次帧有效就刷新存在序号，即使 payload 完全相同也说明刀具头仍在射频场内。 */
    result->presence_sequence = s_presence_sequence[index]; /* 把存在序号随结果带出，供 handlescan 在线监测清除丢失刀具头。 */

    if (same_payload != false)
    {
        if ((s_request_fast_mode != false) || (last_result->valid == false))
        {
            s_result_sequence[index]++; /* 快速识别或当前结果已被清除时，同一标签也要发布序号让业务重新装载。 */
            result->sequence = s_result_sequence[index]; /* 发布可消费序号，但 cache_hit 仍为 true，业务层不会把它当新刀具蜂鸣。 */
            s_last_result[index] = *result; /* 恢复当前可上报结果，让上位机重新显示同一刀具信息。 */
            return true; /* 当前没有可消费结果或处于快速上线阶段，需要通知 handlescan 刷新业务状态。 */
        }

        result->sequence = last_result->sequence; /* 相同 RFID 标签保持旧序号，避免在线监测把同一刀具头当成新结果。 */
        last_result->presence_sequence = result->presence_sequence; /* 相同标签仍要刷新存在序号，避免在线监测误判刀具头已拔掉。 */
        last_result->cache_hit = true; /* 最近一次读到的是缓存命中结果，后续调试读取能看出不是新刀具信息。 */
        return false; /* 普通在线监测不刷新缓存，防止空闲周期重复刷新刀具信息。 */
    }

    s_result_sequence[index]++; /* 每次读到有效标签都递增，包含相同标签掉线后恢复。 */
    result->sequence = s_result_sequence[index]; /* 把序号写入结果，供扫描层判断是否处理过。 */
    payload_memory->valid = true; /* 新 payload 成为该通道新的历史记忆，后续相同数据不再触发新刀具蜂鸣。 */
    payload_memory->source = result->source; /* 同步历史来源，当前固定为 EPC，用于保持判重字段完整。 */
    payload_memory->payload_length = result->payload_length; /* 同步历史长度，保证 memcmp 只比较有效数据。 */
    memcpy(payload_memory->payload, result->payload, result->payload_length); /* 记录完整原始 payload，用于下一次判重。 */
    s_last_result[index] = *result; /* 保存完整原始标签和缓存状态。 */

    return true; /* 缓存已经按完整标签数据更新。 */
}

/*
 * 函数功能：接收外部 RFID 请求并刷新当前活动请求状态。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Rfid_ReceiveRequestMessage(void)
{
    RFIDMessage_t msg; /* 临时保存队列中的一次 RFID 请求。 */
    uint8_t index; /* 保存本次请求通道对应的代次数组下标。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时不能接收请求。 */
    }

    if (Kernel_QueueReceive(RFIDMsgQueue, &msg, 0) != pdTRUE)
    {
        return; /* 本周期无新请求，继续处理已有活动请求。 */
    }

    if (msg.start == false)
    {
        s_request_active = false; /* 停止请求用于兼容旧 down 接口和运行态暂停。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 清掉来源，避免下周期误发命令。 */
        s_request_channel = CHANNEL_NONE; /* 清掉通道归属，避免旧结果串通道。 */
        s_request_attempts_left = 0U; /* 清掉剩余尝试次数。 */
        s_request_fast_mode = false; /* 停止请求后清掉快速识别标志，避免影响下一次普通周期读取。 */
        return; /* 停止消息处理完毕。 */
    }

    if ((Rfid_IsSourceValid(msg.source) == false) ||
        (Rfid_ChannelToIndex(msg.channel, &index) == false))
    {
        s_request_active = false; /* 无效请求不进入活动状态，避免当前 RFID 串口发错命令。 */
        s_request_channel = CHANNEL_NONE; /* 无效通道清零。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 无效来源清零。 */
        s_request_attempts_left = 0U; /* 无效请求不尝试。 */
        s_request_fast_mode = false; /* 无效请求不能保留快速识别状态。 */
        return; /* 无效消息处理完毕。 */
    }

    if (msg.generation != s_request_generation[index])
    {
        return; /* 清刀具前排队的旧代次请求直接丢弃，避免旧读命令重新复活刀具信息。 */
    }

    if (Rfid_IsRequestAllowedForCurrentSelection(msg.channel) == false)
    {
        s_request_active = false; /* 当前运行状态或选中通道不允许读取时，取消本次出队请求。 */
        s_request_channel = CHANNEL_NONE; /* 清掉请求通道，避免后续 RFID 回包被误认为属于旧通道。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 清掉读取来源，避免下周期按旧协议发命令。 */
        s_request_attempts_left = 0U; /* 禁止请求继续消耗 RFID 发送次数。 */
        s_request_fast_mode = false; /* 清掉快速识别状态，避免影响下一次合法请求。 */
        return; /* 非法状态下不切换 R200-K8，也不启动 RFID 读取。 */
    }

    Rfid_SelectHardwareChannel(msg.channel); /* 出队后按请求通道切 R200-K8，保证后续读命令进入正确手柄通道。 */
    s_request_channel = msg.channel; /* 保存请求通道，解析成功后写回同通道缓存。 */
    s_request_source = msg.source; /* 保存本次应读取的来源，当前只允许 EPC。 */
    s_request_attempts_left = msg.fast_mode ? RFID_FAST_ATTEMPTS : RFID_NORMAL_ATTEMPTS; /* 快速/普通识别使用不同尝试次数。 */
    s_request_fast_mode = msg.fast_mode; /* 保存本次请求模式，决定同标签回包是否刷新序号。 */
    Rfid_ClearChannelUartData(s_request_channel); /* 新请求开始前按通道清理串口缓存，防止旧标签回包被重新识别。 */
    s_request_active = true; /* 标记任务从本周期开始处理本次 RFID 请求。 */
}

/*
 * 函数功能：初始化 RFID 请求队列。
 * 输入参数：无。
 * 返回参数：无。
 */
static void RFIDQueue_Init(void)
{
    RFIDMsgQueue = Kernel_QueueCreate(RFID_QUEUE_LENGTH, sizeof(RFIDMessage_t), "RFIDMsgQueue"); /* 创建 handlescan 到 RFID 任务的请求队列。 */
}

/*
 * 函数功能：按当前活动请求处理 RFID 命令发送、串口接收和标签解析。
 * 输入参数：无。
 * 返回参数：无。
 */
static void SplitType_AutoModeGetData_Task(void)
{
    uint16_t rlen;                                      /* 保存本周期从当前 RFID 串口 DMA 取到的字节数。 */
    uint8_t dat[RFID_UART_PACKET_SIZE] = {0U};          /* 临时接收缓冲，取出后对应通道 DMA 会被串口驱动复位。 */
    RfidToolResult_t parsed_result;                     /* 保存本周期解析出的标签结果。 */

    Rfid_ReceiveRequestMessage(); /* 先处理新请求，让屏幕键或 handlescan 能立即切换读取来源。 */

    if (s_request_active == false)
    {
        return; /* 没有活动请求时不占用 RFID 串口。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        s_request_active = false; /* 电机运行中停止当前请求，确保运行态不发送也不解析 RFID。 */
        s_request_attempts_left = 0U; /* 清空剩余尝试次数，运行结束后由 handlescan 重新请求。 */
        s_request_fast_mode = false; /* 运行中取消识别时清掉请求模式，防止停止后误刷新序号。 */
        return; /* 运行态禁止继续访问 RFID 模块。 */
    }

    if (Rfid_IsRequestAllowedForCurrentSelection(s_request_channel) == false)
    {
        s_request_active = false; /* 通道选择变化后当前请求失效，停止识别以免非选中通道刷新刀具信息。 */
        Rfid_ClearChannelUartData(s_request_channel); /* 活动请求失效时先清原通道串口，避免晚到回包污染下一次请求。 */
        s_request_channel = CHANNEL_NONE; /* 清掉活动通道，避免晚到回包写入错误 A/B 缓存。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 清掉读取来源，避免下一周期误按旧来源解析。 */
        s_request_attempts_left = 0U; /* 禁止继续发送 RFID 命令，等待扫描层按当前通道重新发起请求。 */
        s_request_fast_mode = false; /* 清掉快速识别标志，避免后续合法请求继承旧状态。 */
        return; /* 当前选中通道不允许读取时直接退出。 */
    }

    rlen = Rfid_PeekChannelUartData(s_request_channel, dat); /* 电机未运行时才取 DMA 数据，避免运行态处理新 RFID 结果。 */
    if (rlen >= RFID_FRAME_MIN_SIZE)
    {
#if (RFID_DEBUG_BEEP_EVERY_UART_RESPONSE == 1U)
        SendKeyBeepMessage(1U); /* 调试阶段只要收到 RFID 模块回包就蜂鸣，便于区分“模块无响应”和“模块回错误帧”。 */
#endif
        if (Rfid_ParseReceivedFrame(dat, rlen, s_request_source, &parsed_result) == true)
        {
            parsed_result.channel = s_request_channel; /* 解析函数只负责协议，通道归属由当前请求补齐。 */
            (void)Rfid_UpdateParsedCache(&parsed_result); /* RFID 任务只维护标签缓存；是否蜂鸣由 handlescan 在刀具信息真正装载或变化后统一判断，避免同一标签重复回包一直响。 */
            s_request_active = false; /* 单次请求读到结果后结束，周期读取由 handlescan 下一轮再发起。 */
            s_request_attempts_left = 0U; /* 清空尝试次数，避免成功后继续发命令。 */
            s_request_fast_mode = false; /* 成功处理后清掉快速识别标志，下一次请求重新决定行为。 */
            return; /* 本周期已处理成功结果。 */
        }
    }

    if (s_request_attempts_left == 0U)
    {
        s_request_active = false; /* 尝试次数耗尽后结束本次请求，由扫描层决定是否重试或标记丢失。 */
        s_request_fast_mode = false; /* 超时后清掉请求模式，避免后续普通请求继承快速识别语义。 */
        return; /* 本次请求超时结束。 */
    }

    Rfid_SelectHardwareChannel(s_request_channel); /* 发读命令前再次确认 R200-K8 指向目标通道，避免排队期间通道被切走。 */
    Rfid_SendReadCommand(s_request_channel, s_request_source); /* 未读到有效帧时发送下一次读命令。 */
    //s_request_attempts_left--; /* 记录已消耗一次命令发送机会。 */
}

/*
 * 函数功能：RFID 周期任务入口。
 * 输入参数：event 为内核调度事件，本任务未使用。
 * 返回参数：无。
 */
static void AUTOMODEGETDATATaskFunc(uint32_t event)
{
    (void)event; /* 当前任务由固定周期调度，不依赖事件值。 */
    SplitType_AutoModeGetData_Task(); /* 执行一次 RFID 请求状态机。 */
}

/*
 * 函数功能：初始化指定业务通道上的 RFID 模块参数。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void Rfid_InitModuleOnChannel(uint8_t channel)
{
    Rfid_SendPacketForChannel(channel, pa_gain10, (uint16_t)sizeof(pa_gain10)); /* 按通道发送发射功率配置，保证 A/B 模块功率一致。 */
    Delay_ms(50); /* 等待 RFID 模块处理功率命令，避免连续命令粘连。 */

    Rfid_SendPacketForChannel(channel, region_set_command, (uint16_t)sizeof(region_set_command)); /* 按通道发送当前选定地区的频段配置，A/B 两个 RFID 模块保持一致。 */
    Delay_ms(50); /* 等待 RFID 模块处理区域命令，避免下一条跳频命令被吞掉。 */

    Rfid_SendPacketForChannel(channel, hop_ch, (uint16_t)sizeof(hop_ch)); /* 按通道发送跳频配置，保持原射频初始化行为。 */
    Delay_ms(50); /* 等待 RFID 模块完成跳频配置，保证后续读 EPC 命令可用。 */
}

/*
 * 函数功能：初始化当前硬件模式下启用的 RFID 模块。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscRadioFreq_Init(void)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    Rfid_InitModuleOnChannel(CHANNEL_A); /* 双串口模式先初始化 A 通道 UART3 上的 RFID 模块。 */
    Rfid_InitModuleOnChannel(CHANNEL_B); /* 双串口模式再初始化 B 通道 UART9 上的 RFID 模块。 */
#else
    Rfid_InitModuleOnChannel(CHANNEL_A); /* 旧模式保持原行为，只初始化 UART3 当前经 R200-K8 选通的模块。 */
#endif
}

/*
 * 函数功能：创建并启动 RFID 请求处理任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscSplitTypeAutoModeGetData_Init(void)
{
    RFIDQueue_Init(); /* 先创建请求队列，保证 handlescan 和屏幕键可以投递请求。 */
    Kernel_TaskCreate(&AUTOMODEGETDATATaskHandle, AUTOMODEGETDATATaskFunc); /* 创建 RFID 周期任务。 */
    Kernel_TaskStart(&AUTOMODEGETDATATaskHandle, KERNEL_TASK_ALWAYS, 100); /* 100ms 周期检查请求和当前 RFID 串口回包。 */
    (void)CUTTERSCANTaskHandle; /* 旧独立刀具扫描句柄不启动，保留变量避免旧工程符号假设失效。 */
}

/*
 * 函数功能：发起一次 RFID 刀具头读取请求。
 * 输入参数：channel 为 A/B 通道；source 为 EPC；fast_mode 为 true 时使用快速识别尝试次数。
 * 返回参数：true 表示请求已入队，false 表示队列未初始化或参数无效。
 */
bool Rfid_RequestToolRead(uint8_t channel, RfidReadSource_t source, bool fast_mode)
{
    RFIDMessage_t msg; /* 保存要送入 RFID 任务队列的请求。 */
    uint8_t index;     /* 仅用于校验通道是否有效。 */

    if (RFIDMsgQueue == NULL)
    {
        if(channel == CHANNEL_A)
        {
            SendKeyRFIDMessageAdown();
        }
        else if(channel == CHANNEL_B)
        {
            SendKeyRFIDMessageBdown();
        }
      
        return false; /* 队列未初始化时不能接受请求。 */
    }
    /*
     * RFID 来源已经由 handlescan 按 EEPROM 第二页和协议选择：
     * 公共接头 COMMON_SOCKET_ONLINES 与 PXBA/PXBB 分体式当前都走 EPC。
     * 这里不能再用当前 WorkMessage.hand_model 做 PXBA-only 门禁，
     * 因为公共接头基座刚上线时当前工作通道可能尚未装载到 WorkMessage。
     */
    if ((Rfid_ChannelToIndex(channel, &index) == false) || (Rfid_IsSourceValid(source) == false))
    {
       if(channel == CHANNEL_A)
        {
            SendKeyRFIDMessageAdown();
        }
        else if(channel == CHANNEL_B)
        {
            SendKeyRFIDMessageBdown();
        }
        return false; /* 只有有效 A/B 通道和 EPC 来源才能发起读取。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return false; /* 电机运行中禁止新 RFID 请求入队，避免运行参数被新刀具信息覆盖。 */
    }

    if (Rfid_IsRequestAllowedForCurrentSelection(channel) == false)
    {
        return false; /* 当前运行状态或双手柄选中状态不允许读取，避免非目标通道刷新 RFID 刀具参数。 */
    }

    msg.channel = channel; /* 保存请求通道。 */
    msg.source = source; /* 保存读取来源，最终协议固定为 EPC。 */
    msg.start = true; /* true 表示启动一次读取。 */
    msg.fast_mode = fast_mode; /* 保存快速或普通识别模式。 */
    msg.generation = s_request_generation[index]; /* 记录入队时的通道代次，清刀具后旧请求会被出队校验丢弃。 */

    return (Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0)) == pdTRUE); /* 入队成功才表示请求被任务接收。 */
}

/*
 * 函数功能：复制指定通道最后一次有效 RFID 结果。
 * 输入参数：channel 为 A/B 通道；result 指向输出结构。
 * 返回参数：true 表示复制到有效结果，false 表示无有效结果。
 */
bool Rfid_CopyLastResult(uint8_t channel, RfidToolResult_t *result)
{
    uint8_t index; /* 保存通道缓存下标。 */

    if (result == NULL)
    {
        return false; /* 输出为空时不访问，避免异常调用。 */
    }

    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        memset(result, 0, sizeof(*result)); /* 无效通道返回空结果，避免调用方读到旧栈数据。 */
        return false; /* 通道无效，不存在有效 RFID 结果。 */
    }

    *result = s_last_result[index]; /* 复制缓存快照，避免调用方直接操作内部缓存。 */
    return result->valid; /* 只有校验通过的历史结果才算有效。 */
}

/*
 * 函数功能：解析 RFID 串口 DMA 缓冲中的 EPC RFID 回包。
 * 输入参数：uartx_rf_buff 为 DMA 数据；length 为有效字节数；expected_source 为期望来源；result 为输出结果。
 * 返回参数：true 表示找到并提取出一帧校验通过的标签数据。
 */
bool Rfid_ParseReceivedFrame(const uint8_t *uartx_rf_buff,
                             uint16_t length,
                             RfidReadSource_t expected_source,
                             RfidToolResult_t *result)
{
    uint16_t start_pos;             /* 当前尝试的帧头位置。 */
    uint16_t tail_pos;              /* 当前找到的帧尾位置。 */
    RfidReadSource_t frame_source;  /* 当前帧实际来源。 */
    uint8_t payload_offset;         /* EPC 标签数据起始偏移。 */
    uint8_t payload_length;         /* EPC 标签数据长度。 */

    if (result != NULL)
    {
        memset(result, 0, sizeof(*result)); /* 先清输出，失败时调用方不会误读旧结果。 */
    }

    if ((uartx_rf_buff == NULL) || (result == NULL) || (length < RFID_FRAME_MIN_SIZE))
    {
        return false; /* 输入不完整时不能解析。 */
    }

    if ((expected_source != RFID_READ_SOURCE_NONE) && (Rfid_IsSourceValid(expected_source) == false))
    {
        return false; /* 期望来源无效时不接受任何帧。 */
    }

    for (start_pos = 0U; start_pos < length; start_pos++)
    {
        if (uartx_rf_buff[start_pos] != RFID_FRAME_HEAD)
        {
            continue; /* 不是帧头则继续找下一个字节。 */
        }

        if ((uint16_t)(start_pos + RFID_FRAME_MIN_SIZE - 1U) >= length)
        {
            break; /* 剩余字节不足一帧，停止搜索。 */
        }

        for (tail_pos = (uint16_t)(start_pos + RFID_FRAME_MIN_SIZE - 1U); tail_pos < length; tail_pos++)
        {
            if (uartx_rf_buff[tail_pos] != RFID_FRAME_TAIL)
            {
                continue; /* 未找到帧尾时继续向后扫描。 */
            }

            frame_source = Rfid_GetFrameSource(&uartx_rf_buff[start_pos]); /* 判断当前帧是否为 EPC 回包。 */
            if (frame_source == RFID_READ_SOURCE_NONE)
            {
                break; /* 当前 BB 开头不是刀具识别回包，换下一个帧头搜索。 */
            }

            if ((expected_source != RFID_READ_SOURCE_NONE) && (frame_source != expected_source))
            {
                break; /* 回包来源与请求来源不一致，不能误用旧响应。 */
            }

            if (Rfid_LengthMatches(uartx_rf_buff, start_pos, tail_pos) == false)
            {
                break; /* 长度字段不匹配，说明不是完整目标帧。 */
            }

            if (Rfid_ChecksumMatches(uartx_rf_buff, start_pos, tail_pos) == false)
            {
                break; /* checksum 不通过，不能提取刀具数据。 */
            }

            payload_offset = Rfid_GetPayloadOffset(frame_source); /* 按 EPC 来源取协议偏移。 */
            payload_length = Rfid_GetPayloadLength(frame_source); /* 按 EPC 来源取协议长度。 */
            if ((payload_length == 0U) ||
                ((uint16_t)(start_pos + payload_offset + payload_length) > (uint16_t)(tail_pos - 1U)))
            {
                break; /* 原始标签区越过 checksum，说明帧不符合协议。 */
            }

            result->valid = true; /* 当前帧已经通过头、尾、长度和 checksum 校验。 */
            result->source = frame_source; /* 保存 EPC 来源。 */
            result->payload_length = payload_length; /* 保存原始标签数据长度。 */
            memcpy(result->payload, &uartx_rf_buff[start_pos + payload_offset], payload_length); /* 拷贝完整原始标签数据用于缓存比较。 */
            return true; /* 成功提取一帧标签数据。 */
        }
    }

    return false; /* 遍历缓冲后没有找到有效目标帧。 */
}

/*
 * 函数功能：清除指定通道 RFID 结果缓存。
 * 输入参数：channel 为 A/B 通道。
 * 返回参数：无。
 */
void Rfid_ClearChannelResult(uint8_t channel)
{
    uint8_t index; /* 保存通道缓存下标。 */

    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return; /* 无效通道不清任何缓存。 */
    }

    memset(&s_last_result[index], 0, sizeof(s_last_result[index])); /* 清除该通道最后有效标签。 */
    s_result_sequence[index] = 0U; /* 清除结果序号，下一次成功从 1 开始。 */
    s_presence_sequence[index] = 0U; /* 同步清除存在序号，下一次读到标签会被视为新的在线状态变化。 */
    s_request_generation[index]++; /* 清通道会作废清理前所有排队请求，防止旧请求稍后重新启动 RFID 读取。 */
    if ((s_request_active != false) && (s_request_channel == channel))
    {
        s_request_active = false; /* 清刀具时同步取消该通道未完成读取，避免丢失判定后晚到回包复活旧刀具。 */
        Rfid_ClearChannelUartData(channel); /* 清刀具时按目标通道丢弃晚到回包，防止旧标签重新上报。 */
        s_request_channel = CHANNEL_NONE; /* 清掉活动请求通道，后续回包没有合法归属。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 清掉读取来源，避免下一周期误按旧来源解析。 */
        s_request_attempts_left = 0U; /* 清掉剩余发送次数，停止本次识别尝试。 */
        s_request_fast_mode = false; /* 清掉快速识别标志，避免影响后续普通在线监测。 */
    }
}

/*
 * 函数功能：把旧接口 rfid_data 转成当前工程统一使用的 EPC 来源。
 * 输入参数：rfid_data 为旧屏/旧业务传入值，保留形参仅兼容旧函数签名。
 * 返回参数：EPC 读取来源。
 */
static RfidReadSource_t Rfid_LegacyTypeToSource(uint8_t rfid_data)
{
    (void)rfid_data; /* 分体式 PXBA/PXBB 与公共接头已经统一 EPC 协议，旧参数不再参与来源选择。 */

    return RFID_READ_SOURCE_EPC; /* 所有旧 up 入口统一读取 EPC。 */
}

/*
 * 函数功能：旧 A 通道 RFID up 入口，转为新请求接口。
 * 输入参数：rfid_data 为旧业务传入读取类型。
 * 返回参数：无。
 */
void SendKeyRFIDMessageAup(uint8_t rfid_data)
{
    (void)Rfid_RequestToolRead(CHANNEL_A, Rfid_LegacyTypeToSource(rfid_data), false); /* 旧入口只做普通读取，同一标签仅刷新存在序号，不重复发布刀具变化。 */
}

/*
 * 函数功能：旧 A 通道 RFID down 入口，停止当前读取请求。
 * 输入参数：无。
 * 返回参数：无。
 */
void SendKeyRFIDMessageAdown(void)
{
    RFIDMessage_t msg; /* 停止消息用于兼容旧接口。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时无请求可停。 */
    }

    msg.channel = CHANNEL_A; /* 停止 A 通道旧请求。 */
    msg.source = RFID_READ_SOURCE_NONE; /* 停止消息不需要来源。 */
    msg.start = false; /* false 表示停止当前活动请求。 */
    msg.fast_mode = false; /* 停止消息不使用快速模式。 */
    (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0)); /* 停止失败也不影响主流程。 */
}

/*
 * 函数功能：旧 B 通道 RFID up 入口，补齐原空实现并转为新请求接口。
 * 输入参数：rfid_data 为旧业务传入读取类型。
 * 返回参数：无。
 */
void SendKeyRFIDMessageBup(uint8_t rfid_data)
{
    (void)Rfid_RequestToolRead(CHANNEL_B, Rfid_LegacyTypeToSource(rfid_data), false); /* B 通道旧入口同样使用普通读取，避免同一刀具信息反复触发业务刷新。 */
}

/*
 * 函数功能：旧 B 通道 RFID down 入口，停止当前读取请求。
 * 输入参数：无。
 * 返回参数：无。
 */
void SendKeyRFIDMessageBdown(void)
{
    RFIDMessage_t msg; /* 停止消息用于兼容旧接口。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时无请求可停。 */
    }

    msg.channel = CHANNEL_B; /* 停止 B 通道旧请求。 */
    msg.source = RFID_READ_SOURCE_NONE; /* 停止消息不需要来源。 */
    msg.start = false; /* false 表示停止当前活动请求。 */
    msg.fast_mode = false; /* 停止消息不使用快速模式。 */
    (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0)); /* 停止失败也不影响主流程。 */
}

/*
 * 函数功能：兼容旧 RfidHandle 调用，解析一次缓冲并写入新缓存。
 * 输入参数：uartx_rf_buff 为 UART3 缓冲；interface 为通道；enable_rfid 控制是否启用；rfid_type 为旧来源值。
 * 返回参数：无。
 */
void RfidHandle(uint8_t *uartx_rf_buff, uint8_t interface, bool enable_rfid, uint8_t rfid_type)
{
    RfidToolResult_t result; /* 保存旧入口本次解析出的标签结果。 */

    if (enable_rfid == false)
    {
        return; /* 旧调用显式关闭 RFID 时不解析。 */
    }

    if (Rfid_ParseReceivedFrame(uartx_rf_buff,
                                UART3_MAX_PACKET_SIZE,
                                Rfid_LegacyTypeToSource(rfid_type),
                                &result) == false)
    {
        return; /* 缓冲中没有符合旧请求来源的有效帧。 */
    }

    result.channel = interface; /* 旧接口传入的 interface 作为结果通道归属。 */
    (void)Rfid_UpdateParsedCache(&result); /* 写入新缓存，供后续统一读取。 */
}
