#include "sscRFID.h"

#include "FreeRTOS.h"
#include "Pubinterface.h"
#include "kernel_scheduler.h"
#include "queue.h"
#include "sscBEEP.h"
#include "delay.h"
#include "uart3.h"

#include <string.h>

#define RFID_FRAME_HEAD                 0xBBU /* RFID 模块帧头，所有回包都从该字节开始。 */
#define RFID_FRAME_TAIL                 0x7EU /* RFID 模块帧尾，帧尾前一字节是累加和校验。 */
#define RFID_RESP_SECOND_EPC            0x02U /* EPC 读取回包第二字节，协议指定为 BB 02 22。 */
#define RFID_RESP_SECOND_USER           0x01U /* USER 读取回包第二字节，协议指定为 BB 01 39。 */
#define RFID_RESP_CMD_EPC               0x22U /* EPC 读取回包命令码。 */
#define RFID_RESP_CMD_USER              0x39U /* USER 读取回包命令码。 */
#define RFID_PAYLOAD_EPC_OFFSET         8U    /* EPC 标签数据从整帧 data[8] 开始。 */
#define RFID_PAYLOAD_USER_OFFSET        20U   /* USER 标签数据从整帧 data[20] 开始。 */
#define RFID_FRAME_LENGTH_FIELD_OFFSET  4U    /* RFID 帧第 4 下标字节保存模块数据区长度。 */
#define RFID_FRAME_MIN_SIZE             7U    /* 最短帧至少包含头、命令、长度、校验和尾。 */
#define RFID_QUEUE_LENGTH               4U    /* 队列保存少量屏幕/扫描层触发请求，避免按键抖动丢入口。 */
#define RFID_FAST_ATTEMPTS              10U   /* 快速识别最多尝试 10 个任务周期，覆盖基座刚上线。 */
#define RFID_NORMAL_ATTEMPTS            3U    /* 普通周期识别只尝试 3 次，避免空闲监测占用 UART3。 */
#define RFID_DEBUG_BEEP_EVERY_UART_RESPONSE 0U /* 临时调试开关：1 表示 UART3 收到任意 RFID 模块回包都蜂鸣，定位完成后关闭以避免影响业务调度。 */
#define RFID_DEBUG_BEEP_EVERY_VALID_READ 0U   /* 有效帧调试蜂鸣关闭，蜂鸣只在缓存确认新刀具信息时触发。 */

typedef struct
{
    uint8_t channel;             /* 请求所属通道，只有 A/B 通道有效。 */
    bool start;                  /* true 启动一次读取，false 停止当前读取。 */
    bool fast_mode;              /* true 使用快速识别尝试次数，false 使用普通监测尝试次数。 */
    RfidReadSource_t source;     /* 本次读取 EPC 还是 USER，由 handlescan 按 EEPROM 第二页决定。 */
    uint16_t generation;         /* 请求入队时的通道代次，清刀具后旧代次请求会被丢弃。 */
} RFIDMessage_t;

typedef struct
{
    bool valid;                                  /* true 表示该通道曾经读到过完整标签，可用于“同一刀具不重复蜂鸣”的历史判重。 */
    RfidReadSource_t source;                     /* 记录历史标签来源，EPC 和 USER 数据不能互相判为同一刀具。 */
    uint8_t payload_length;                      /* 记录历史 payload 长度，避免不同协议长度误比较。 */
    uint8_t payload[RFID_PAYLOAD_USER_LENGTH];   /* 保存最近一次刀具头原始 payload；USER 16 字节最长，EPC 只使用前 12 字节。 */
} RfidPayloadMemory_t;

static uint8_t NO_MASK3_WRITE_EPC[7] = {0XBB, 0X00, 0X22, 0X00, 0X00, 0X22, 0X7E}; /* 无掩码读取 EPC 区。 */
static uint8_t NO_MASK3_READ_USER[16] = {0xBB, 0x00, 0x39, 0x00, 0x09, 0x00, 0x00, 0x00,
                                         0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x4d, 0x7E}; /* 无掩码读取 USER 区。 */
static unsigned char hop_ch[] = {0XBB, 0X00, 0XAD, 0X00, 0X01, 0XFF, 0XAD, 0X7E}; /* 开启跳频，保持现有射频初始化流程。 */
static unsigned char pa_gain0[] = {0XBB, 0X00, 0XB6, 0X00, 0X02, 0X00, 0X00, 0XB8, 0X7E}; /* 发射功率设为 0。 */
static unsigned char region_set_CHAIN[] = {0XBB, 0x00, 0x07, 0x00, 0x01, 0x01, 0x09, 0x7E}; /* 中国频段 920.125-924.875M。 */

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
static RfidReadSource_t s_request_source = RFID_READ_SOURCE_NONE; /* 当前活动请求读取 EPC 还是 USER。 */
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
 * 函数功能：判断读取来源是否为当前支持的 EPC 或 USER。
 * 输入参数：source 为外部传入的读取来源。
 * 返回参数：true 表示来源有效，false 表示来源无效。
 */
static bool Rfid_IsSourceValid(RfidReadSource_t source)
{
    return ((source == RFID_READ_SOURCE_EPC) || (source == RFID_READ_SOURCE_USER)); /* 只允许两种协议来源参与识别。 */
}

/*
 * 函数功能：根据读取来源返回原始标签数据长度。
 * 输入参数：source 为 EPC 或 USER。
 * 返回参数：对应协议的标签数据长度，无效来源返回 0。
 */
static uint8_t Rfid_GetPayloadLength(RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        return RFID_PAYLOAD_EPC_LENGTH; /* 公共接头 EPC 固定提取 12 字节。 */
    }

    if (source == RFID_READ_SOURCE_USER)
    {
        return RFID_PAYLOAD_USER_LENGTH; /* PXBA/PXBB USER 固定提取 16 字节。 */
    }

    return 0U; /* 无效来源没有标签长度。 */
}

/*
 * 函数功能：根据读取来源返回原始标签数据在整帧中的起始偏移。
 * 输入参数：source 为 EPC 或 USER。
 * 返回参数：协议指定的数据偏移，无效来源返回 0。
 */
static uint8_t Rfid_GetPayloadOffset(RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        return RFID_PAYLOAD_EPC_OFFSET; /* EPC 按 data[8..19] 提取。 */
    }

    if (source == RFID_READ_SOURCE_USER)
    {
        return RFID_PAYLOAD_USER_OFFSET; /* USER 按 data[20..35] 提取。 */
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

    if ((frame[0] == RFID_FRAME_HEAD) &&
        (frame[1] == RFID_RESP_SECOND_USER) &&
        (frame[2] == RFID_RESP_CMD_USER))
    {
        return RFID_READ_SOURCE_USER; /* BB 01 39 是分体式 USER 回包。 */
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
 * 函数功能：发送当前请求对应的 RFID 读命令。
 * 输入参数：source 指定 EPC 或 USER。
 * 返回参数：无。
 */
static void Rfid_SendReadCommand(RfidReadSource_t source)
{
    if (source == RFID_READ_SOURCE_EPC)
    {
        Uart3_SendPacket(NO_MASK3_WRITE_EPC, (uint16_t)sizeof(NO_MASK3_WRITE_EPC)); /* 公共接头刀具头读取 EPC。 */
        return; /* EPC 命令已发送，本周期不再发送 USER。 */
    }

    if (source == RFID_READ_SOURCE_USER)
    {
        Uart3_SendPacket(NO_MASK3_READ_USER, (uint16_t)sizeof(NO_MASK3_READ_USER)); /* 分体式 PXBA/PXBB 刀具头读取 USER。 */
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
        same_payload = true; /* 完整 EPC/USER 原始数据一致时才视为同一个标签数据。 */
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
    payload_memory->source = result->source; /* 同步历史来源，避免 EPC/USER 互相命中。 */
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
        s_request_active = false; /* 无效请求不进入活动状态，避免 UART3 发错命令。 */
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

    s_request_channel = msg.channel; /* 保存请求通道，解析成功后写回同通道缓存。 */
    s_request_source = msg.source; /* 保存本次应读取 EPC 还是 USER。 */
    s_request_attempts_left = msg.fast_mode ? RFID_FAST_ATTEMPTS : RFID_NORMAL_ATTEMPTS; /* 快速/普通识别使用不同尝试次数。 */
    s_request_fast_mode = msg.fast_mode; /* 保存本次请求模式，决定同标签回包是否刷新序号。 */
    Uart3_ClearRecvData(); /* 新请求开始前丢弃 UART3 残留帧，避免刀具头已拔掉后旧标签回包被重新识别并触发蜂鸣。 */
    s_request_active = true; /* 标记任务从本周期开始处理本次 RFID 请求。 */
}

/*
 * 函数功能：丢弃 RFID 队列中指定通道的旧请求，保留其它通道请求。
 * 输入参数：channel 为需要清理的 A/B 通道。
 * 返回参数：无。
 */
static void Rfid_DiscardQueuedMessagesForChannel(uint8_t channel)
{
    RFIDMessage_t msg; /* 暂存从 RFID 队列取出的请求。 */
    RFIDMessage_t keep_msgs[RFID_QUEUE_LENGTH]; /* 队列最多 4 项，先保存其它通道请求再放回，避免边取边放反复读到同一项。 */
    uint8_t keep_count = 0U; /* 记录需要放回队列的其它通道请求数量。 */
    uint8_t scan_count; /* 限定最多扫描队列容量次，避免并发投递时在清理函数内停留过久。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时没有排队请求可清。 */
    }

    for (scan_count = 0U; scan_count < RFID_QUEUE_LENGTH; scan_count++)
    {
        if (Kernel_QueueReceive(RFIDMsgQueue, &msg, 0) != pdTRUE)
        {
            break; /* 当前队列已经取空，本轮清理完成。 */
        }

        if ((msg.channel != channel) && (keep_count < RFID_QUEUE_LENGTH))
        {
            keep_msgs[keep_count] = msg; /* 非目标通道请求暂存，避免清 A 时误丢 B 的识别请求。 */
            ++keep_count; /* 增加待恢复请求数量。 */
        }
    }

    for (scan_count = 0U; scan_count < keep_count; scan_count++)
    {
        (void)Kernel_QueueSend(RFIDMsgQueue, &keep_msgs[scan_count], pdMS_TO_TICKS(0)); /* 把其它通道请求放回 RFID 队列，放回失败不影响目标通道清理。 */
    }
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
 * 函数功能：按当前活动请求处理 UART3 RFID 发送和接收。
 * 输入参数：无。
 * 返回参数：无。
 */
static void SplitType_AutoModeGetData_Task(void)
{
    uint16_t rlen;                                      /* 保存本周期从 UART3 DMA 取到的字节数。 */
    uint8_t dat[UART3_MAX_PACKET_SIZE] = {0U};          /* 临时接收缓冲，取出后 DMA 会被 uart3 驱动复位。 */
    RfidToolResult_t parsed_result;                     /* 保存本周期解析出的标签结果。 */

    Rfid_ReceiveRequestMessage(); /* 先处理新请求，让屏幕键或 handlescan 能立即切换读取来源。 */

    if (s_request_active == false)
    {
        return; /* 没有活动请求时不占用 UART3。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        s_request_active = false; /* 电机运行中停止当前请求，确保运行态不发送也不解析 RFID。 */
        s_request_attempts_left = 0U; /* 清空剩余尝试次数，运行结束后由 handlescan 重新请求。 */
        s_request_fast_mode = false; /* 运行中取消识别时清掉请求模式，防止停止后误刷新序号。 */
        return; /* 运行态禁止继续访问 RFID 模块。 */
    }

    rlen = Uart3_DMARecvDataPeek(dat); /* 电机未运行时才取 DMA 数据，避免运行态处理新 RFID 结果。 */
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

    Rfid_SendReadCommand(s_request_source); /* 未读到有效帧时发送下一次读命令。 */
    s_request_attempts_left--; /* 记录已消耗一次命令发送机会。 */
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
 * 函数功能：初始化射频模块功率、区域和跳频。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscRadioFreq_Init(void)
{
    Uart3_SendPacket(pa_gain0, (uint16_t)sizeof(pa_gain0)); /* 设置发射功率，保持原工程默认值。 */
    Delay_ms(50); /* 等待模块处理功率设置命令。 */

    Uart3_SendPacket(region_set_CHAIN, (uint16_t)sizeof(region_set_CHAIN)); /* 设置中国频段。 */
    Delay_ms(50); /* 等待模块处理区域设置命令。 */

    Uart3_SendPacket(hop_ch, (uint16_t)sizeof(hop_ch)); /* 开启跳频，保持原工程射频初始化行为。 */
    Delay_ms(50); /* 等待模块处理跳频命令。 */
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
    Kernel_TaskStart(&AUTOMODEGETDATATaskHandle, KERNEL_TASK_ALWAYS, 100); /* 100ms 周期检查请求和 UART3 回包。 */
    (void)CUTTERSCANTaskHandle; /* 旧独立刀具扫描句柄不启动，保留变量避免旧工程符号假设失效。 */
}

/*
 * 函数功能：发起一次 RFID 刀具头读取请求。
 * 输入参数：channel 为 A/B 通道；source 为 EPC/USER；fast_mode 为 true 时使用快速识别尝试次数。
 * 返回参数：true 表示请求已入队，false 表示队列未初始化或参数无效。
 */
bool Rfid_RequestToolRead(uint8_t channel, RfidReadSource_t source, bool fast_mode)
{
    RFIDMessage_t msg; /* 保存要送入 RFID 任务队列的请求。 */
    uint8_t index;     /* 仅用于校验通道是否有效。 */

    if (RFIDMsgQueue == NULL)
    {
        return false; /* 队列未初始化时不能接受请求。 */
    }

    if ((Rfid_ChannelToIndex(channel, &index) == false) || (Rfid_IsSourceValid(source) == false))
    {
        return false; /* 只有有效 A/B 通道和 EPC/USER 来源才能发起读取。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return false; /* 电机运行中不发起 RFID 请求，防止运行参数被新刀具头改变。 */
    }

    msg.channel = channel; /* 保存请求通道。 */
    msg.source = source; /* 保存读取 EPC 还是 USER。 */
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
 * 函数功能：解析 UART3 DMA 缓冲中的 EPC/USER RFID 回包。
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
    uint8_t payload_offset;         /* EPC/USER 标签数据起始偏移。 */
    uint8_t payload_length;         /* EPC/USER 标签数据长度。 */

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

            frame_source = Rfid_GetFrameSource(&uartx_rf_buff[start_pos]); /* 判断当前帧是 EPC 还是 USER。 */
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

            payload_offset = Rfid_GetPayloadOffset(frame_source); /* 按 EPC/USER 来源取协议偏移。 */
            payload_length = Rfid_GetPayloadLength(frame_source); /* 按 EPC/USER 来源取协议长度。 */
            if ((payload_length == 0U) ||
                ((uint16_t)(start_pos + payload_offset + payload_length) > (uint16_t)(tail_pos - 1U)))
            {
                break; /* 原始标签区越过 checksum，说明帧不符合协议。 */
            }

            result->valid = true; /* 当前帧已经通过头、尾、长度和 checksum 校验。 */
            result->source = frame_source; /* 保存 EPC 或 USER 来源。 */
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
    Rfid_DiscardQueuedMessagesForChannel(channel); /* 同步丢弃该通道已排队但未执行的旧请求，避免清刀具后又启动一次旧读。 */
    if ((s_request_active != false) && (s_request_channel == channel))
    {
        s_request_active = false; /* 清刀具时同步取消该通道未完成读取，避免丢失判定后晚到回包复活旧刀具。 */
        s_request_channel = CHANNEL_NONE; /* 清掉活动请求通道，后续回包没有合法归属。 */
        s_request_source = RFID_READ_SOURCE_NONE; /* 清掉读取来源，避免下一周期误按旧来源解析。 */
        s_request_attempts_left = 0U; /* 清掉剩余发送次数，停止本次识别尝试。 */
        s_request_fast_mode = false; /* 清掉快速识别标志，避免影响后续普通在线监测。 */
        Uart3_ClearRecvData(); /* 丢弃 UART3 里可能晚到的旧回包，防止上位机刀具信息被旧标签重新兜底上报。 */
    }
}

/*
 * 函数功能：把旧接口 rfid_data 转成明确的 EPC/USER 来源。
 * 输入参数：rfid_data 为旧屏/旧业务传入值，1 沿用旧注释表示 USER，其它值按 EPC 处理。
 * 返回参数：EPC 或 USER 读取来源。
 */
static RfidReadSource_t Rfid_LegacyTypeToSource(uint8_t rfid_data)
{
    if (rfid_data == 1U)
    {
        return RFID_READ_SOURCE_USER; /* 旧 A 通道入口注释为读取 USER，保持兼容。 */
    }

    return RFID_READ_SOURCE_EPC; /* 其它旧值默认读取 EPC，供公共接头自动识别使用。 */
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
