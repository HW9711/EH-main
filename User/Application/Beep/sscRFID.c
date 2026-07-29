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

#define RFID_TX_POWER_0_DBM_X100        0U     /* 0dBm，对应模块协议功率值 0。 */
#define RFID_TX_POWER_10_DBM_X100       1000U  /* 10dBm，对应模块协议功率值 1000。 */
#define RFID_TX_POWER_13_DBM_X100       1300U  /* 13dBm，对应模块协议功率值 1300。 */
#define RFID_TX_POWER_15_DBM_X100       1500U  /* 15dBm，对应模块协议功率值 1500。 */

/* 修改下面这一项即可切换 RFID 地区频段；当前产品配置由该宏唯一决定。 */
#define RFID_REGION_SELECT              RFID_REGION_US

/* 修改下面这一项即可切换 RFID 发射功率；默认保持重构前实际使用的 10dBm。 */
#define RFID_TX_POWER_SELECT            RFID_TX_POWER_10_DBM_X100

#if ((RFID_REGION_SELECT != RFID_REGION_CHINA_920) && \
     (RFID_REGION_SELECT != RFID_REGION_US) && \
     (RFID_REGION_SELECT != RFID_REGION_EUROPE) && \
     (RFID_REGION_SELECT != RFID_REGION_CHINA_840) && \
     (RFID_REGION_SELECT != RFID_REGION_KOREA))
#error "RFID_REGION_SELECT is invalid"
#endif

#if ((RFID_TX_POWER_SELECT != RFID_TX_POWER_0_DBM_X100) && \
     (RFID_TX_POWER_SELECT != RFID_TX_POWER_10_DBM_X100) && \
     (RFID_TX_POWER_SELECT != RFID_TX_POWER_13_DBM_X100) && \
     (RFID_TX_POWER_SELECT != RFID_TX_POWER_15_DBM_X100))
#error "RFID_TX_POWER_SELECT is invalid"
#endif

#define RFID_REGION_COMMAND_CHECKSUM    (0x08U + RFID_REGION_SELECT) /* 区域命令校验和随地区编号变化，避免切换地区后继续发送旧校验值。 */
#define RFID_TX_POWER_HIGH_BYTE         ((RFID_TX_POWER_SELECT >> 8U) & 0xFFU) /* 功率协议值高字节，随选择宏自动生成。 */
#define RFID_TX_POWER_LOW_BYTE          (RFID_TX_POWER_SELECT & 0xFFU) /* 功率协议值低字节，随选择宏自动生成。 */
#define RFID_TX_POWER_COMMAND_CHECKSUM  ((0xB8U + RFID_TX_POWER_HIGH_BYTE + RFID_TX_POWER_LOW_BYTE) & 0xFFU) /* 功率命令累加和低8位，避免手工修改功率后校验字节错误。 */
#define RFID_FRAME_TAIL                 0x7EU /* RFID 模块帧尾，帧尾前一字节是累加和校验。 */
#define RFID_RESP_SECOND_EPC            0x02U /* EPC 读取回包第二字节，协议指定为 BB 02 22。 */
#define RFID_RESP_CMD_EPC               0x22U /* EPC 读取回包命令码。 */
#define RFID_PAYLOAD_EPC_OFFSET         8U    /* EPC 标签数据从整帧 data[8] 开始。 */
#define RFID_FRAME_LENGTH_FIELD_OFFSET  4U    /* RFID 帧第 4 下标字节保存模块数据区长度。 */
#define RFID_FRAME_MIN_SIZE             7U    /* 最短帧至少包含头、命令、长度、校验和尾。 */
#define RFID_QUEUE_LENGTH               4U    /* 队列保存少量屏幕/扫描层触发请求，避免按键抖动丢入口。 */
#define RFID_FAST_ATTEMPTS              10U   /* 快速识别最多尝试 10 个任务周期，覆盖基座刚上线。 */
#define RFID_NORMAL_ATTEMPTS            3U    /* 普通周期识别只尝试 3 次，避免空闲监测长期占用 RFID 串口。 */
#define RFID_NORMAL_HARD_TIMEOUT_TICKS  5U    /* 普通请求最多占用 5 个 100ms 周期，保证半帧等待也有明确终态。 */
#define RFID_FAST_HARD_TIMEOUT_TICKS    15U   /* 快速识别最多占用 15 个 100ms 周期，覆盖新刀具刚进入射频场的建立时间。 */
#define RFID_PARTIAL_GRACE_TICKS        1U    /* 收到半帧后额外等待一个任务周期，避免新读命令打断剩余字节。 */
#define RFID_DEBUG_BEEP_EVERY_UART_RESPONSE 0U /* 临时调试开关：1 表示当前 RFID 串口收到任意模块回包都蜂鸣，定位完成后关闭以避免影响业务调度。 */
#define RFID_DEBUG_BEEP_EVERY_VALID_READ 0U   /* 有效帧调试蜂鸣关闭，蜂鸣只在缓存确认新刀具信息时触发。 */

#if (UART9_MAX_PACKET_SIZE > UART3_MAX_PACKET_SIZE)
#define RFID_UART_PACKET_SIZE          UART9_MAX_PACKET_SIZE /* 双串口模式下临时解析缓存按更大的 UART 缓冲区预留。 */
#else
#define RFID_UART_PACKET_SIZE          UART3_MAX_PACKET_SIZE /* 当前 UART3/UART9 都是 150 字节，保持旧解析缓存容量不变。 */
#endif
#define RFID_RX_ACCUM_SIZE             (RFID_UART_PACKET_SIZE * 2U) /* 跨两个 DMA 批次累计接收，允许完整 EPC 帧跨越 100ms 任务边界。 */

typedef struct
{
    uint8_t channel;             /* 请求所属通道，只有 A/B 通道有效。 */
    bool start;                  /* true 启动一次读取，false 停止当前读取。 */
    bool fast_mode;              /* true 使用快速识别尝试次数，false 使用普通监测尝试次数。 */
    RfidReadSource_t source;     /* 本次读取来源，最终协议只允许 EPC。 */
    uint16_t generation;         /* 请求入队时的通道代次，清刀具后旧代次请求会被丢弃。 */
    uint16_t ticket;             /* 本次逻辑事务票据，供 handlescan 查询明确完成结果。 */
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
static uint8_t rfid_tx_power_command[] = {0xBBU, 0x00U, 0xB6U, 0x00U, 0x02U, RFID_TX_POWER_HIGH_BYTE, RFID_TX_POWER_LOW_BYTE, RFID_TX_POWER_COMMAND_CHECKSUM, 0x7EU}; /* 根据功率选择宏生成唯一命令，避免未选功率数组产生编译告警。 */
static uint8_t region_set_command[] = {0xBBU, 0x00U, 0x07U, 0x00U, 0x01U, RFID_REGION_SELECT, RFID_REGION_COMMAND_CHECKSUM, 0x7EU}; /* 根据上方地区配置生成唯一一条有效区域命令，避免未选地区数组产生告警。 */

static kernel_task_t AUTOMODEGETDATATaskHandle;     /* RFID 轮询任务句柄，任务实际按请求工作。 */
static kernel_task_t CUTTERSCANTaskHandle;          /* 预留旧句柄，不启动，避免破坏工程外部引用假设。 */
static QueueHandle_t RFIDMsgQueue = NULL;           /* handlescan/屏幕自动识别键发送 RFID 请求的队列。 */
static RfidToolResult_t s_last_result[2];           /* A/B 通道最后一次有效 RFID 原始标签和缓存状态。 */
static RfidPayloadMemory_t s_payload_memory[2];      /* A/B 通道最近刀具 payload 记忆；清当前结果时不清它，用于同一刀具恢复时不重复蜂鸣。 */
static uint16_t s_result_sequence[2] = {0U, 0U};    /* A/B 通道成功结果序号，便于扫描层判断新结果。 */
static uint16_t s_presence_sequence[2] = {0U, 0U};  /* A/B 通道有效读到标签的存在序号，在线监测用它判断刀具头是否仍可读到。 */
static uint16_t s_request_generation[2] = {0U, 0U}; /* A/B 通道 RFID 请求代次，清刀具时递增以作废旧排队请求。 */
static uint16_t s_next_ticket[2] = {0U, 0U};        /* A/B 通道独立生成非零事务票据，避免跨通道结果混用。 */
static uint16_t s_outstanding_ticket[2] = {0U, 0U}; /* A/B 通道当前排队或执行中的逻辑事务票据。 */
static uint16_t s_completed_ticket[2] = {0U, 0U};   /* A/B 通道最近一次进入终态的事务票据。 */
static RfidRequestResult_t s_completed_result[2] = {RFID_REQUEST_RESULT_UNKNOWN, RFID_REQUEST_RESULT_UNKNOWN}; /* 保存最近票据的成功、超时或取消终态。 */
static bool s_outstanding_valid[2] = {false, false}; /* true 表示该通道已有排队或执行事务，新请求必须合并而不能替换。 */
static bool s_outstanding_fast_mode[2] = {false, false}; /* 记录合并事务是否需要快速识别语义。 */
static bool s_request_active = false;               /* true 表示任务当前有一次未完成 RFID 请求。 */
static uint8_t s_request_channel = CHANNEL_NONE;    /* 当前活动请求所属通道。 */
static RfidReadSource_t s_request_source = RFID_READ_SOURCE_NONE; /* 当前活动请求读取来源，最终协议只允许 EPC。 */
static uint8_t s_request_attempts_left = 0U;        /* 当前请求剩余发送次数，归零后任务停止本次请求。 */
static bool s_request_fast_mode = false;            /* true 表示本次请求来自上线/重试快速识别，同标签也要让扫描层重新消费。 */
static uint16_t s_request_ticket = 0U;               /* 当前活动逻辑事务票据，完成时只结算对应通道的调用方。 */
static uint8_t s_request_elapsed_ticks = 0U;         /* 当前事务已经占用的 100ms 周期数，用于硬超时兜底。 */
static uint8_t s_request_hard_timeout_ticks = 0U;    /* 当前事务普通或快速模式对应的最大占用周期。 */
static uint8_t s_partial_grace_ticks = 0U;           /* 收到不完整数据后暂停补发命令的剩余周期。 */
static uint8_t s_rx_accumulator[RFID_RX_ACCUM_SIZE]; /* 保存跨任务周期到达的 RFID 分片，直到组成合法完整帧。 */
static uint16_t s_rx_accum_length = 0U;              /* 当前跨周期接收缓存中的有效字节数。 */
static bool s_request_saw_rx_data = false;           /* true 表示本事务收到过串口数据，超时时用于区分无字节和非EPC异常。 */
#if (RFID_LINK_STATS_ENABLE == 1U)
static RfidLinkStatistics_t s_link_statistics[2];   /* A/B 通道请求应答、在线监测和确认掉线累计值。 */
static bool s_link_response_pending[2] = {false, false}; /* true 表示该通道最近一条读取命令尚未收到有效回包。 */
#endif

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
 * 函数功能：清空当前逻辑事务的跨周期接收缓存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Rfid_ResetReceiveAccumulator(void)
{
    memset(s_rx_accumulator, 0, sizeof(s_rx_accumulator)); /* 清除旧事务残留字节，防止晚到数据被下一把刀具误用。 */
    s_rx_accum_length = 0U; /* 新事务从空接收缓存开始拼帧。 */
    s_partial_grace_ticks = 0U; /* 清除半帧等待状态，下一事务按正常发送节拍启动。 */
    s_request_saw_rx_data = false; /* 新事务尚未收到任何串口字节。 */
}

/*
 * 函数功能：把本周期 DMA 字节追加到逻辑事务缓存，允许一帧跨越多个 100ms 周期。
 * 输入参数：data 为本周期字节；length 为有效长度。
 * 返回参数：无。
 */
static void Rfid_AppendReceiveData(const uint8_t *data, uint16_t length)
{
    uint16_t overflow_length; /* 保存缓存溢出时需要丢弃的最旧字节数。 */

    if ((data == NULL) || (length == 0U))
    {
        return; /* 本周期没有新字节时保留既有半帧，等待后续 DMA 数据。 */
    }

    s_request_saw_rx_data = true; /* 记录事务收到过数据，超时诊断可区分无响应和非EPC异常。 */
    s_partial_grace_ticks = RFID_PARTIAL_GRACE_TICKS; /* 半帧到达后暂停一次补发，避免新命令打断剩余字节。 */
    if (length >= (uint16_t)sizeof(s_rx_accumulator))
    {
        memcpy(s_rx_accumulator,
               &data[length - (uint16_t)sizeof(s_rx_accumulator)],
               sizeof(s_rx_accumulator)); /* 单次数据过长时只保留尾部，帧解析器仍可从其中寻找最后一帧。 */
        s_rx_accum_length = (uint16_t)sizeof(s_rx_accumulator); /* 标记整个累计缓存均为有效数据。 */
        return; /* 超长批次已经完整覆盖旧缓存，不再执行普通追加。 */
    }

    if ((uint16_t)(s_rx_accum_length + length) > (uint16_t)sizeof(s_rx_accumulator))
    {
        overflow_length = (uint16_t)(s_rx_accum_length + length - (uint16_t)sizeof(s_rx_accumulator)); /* 计算追加后超出的最旧数据长度。 */
        memmove(s_rx_accumulator,
                &s_rx_accumulator[overflow_length],
                (size_t)(s_rx_accum_length - overflow_length)); /* 仅移走最旧噪声，保留最接近当前时刻的可能帧头。 */
        s_rx_accum_length = (uint16_t)(s_rx_accum_length - overflow_length); /* 更新移动后保留的有效长度。 */
    }

    memcpy(&s_rx_accumulator[s_rx_accum_length], data, length); /* 把本周期分片接到上一周期半帧之后。 */
    s_rx_accum_length = (uint16_t)(s_rx_accum_length + length); /* 更新完整累计长度，供协议解析器一次扫描。 */
}

/*
 * 函数功能：把指定通道事务票据写成明确终态。
 * 输入参数：channel 为 A/B 通道；ticket 为事务票据；result 为成功、超时或取消。
 * 返回参数：无。
 */
static void Rfid_CompleteTicket(uint8_t channel, uint16_t ticket, RfidRequestResult_t result)
{
    uint8_t index; /* 保存事务所属通道的数组下标。 */

    if ((ticket == 0U) || (Rfid_ChannelToIndex(channel, &index) == false))
    {
        return; /* 无效票据或通道没有可结算对象。 */
    }

    if ((s_outstanding_valid[index] == false) || (s_outstanding_ticket[index] != ticket))
    {
        return; /* 过期事务不得覆盖该通道后续请求的完成结果。 */
    }

    s_completed_ticket[index] = ticket; /* 保存调用方查询所需的终态票据。 */
    s_completed_result[index] = result; /* 保存本事务明确成功、超时或取消的结论。 */
    s_outstanding_valid[index] = false; /* 终态产生后允许该通道提交下一事务。 */
    s_outstanding_fast_mode[index] = false; /* 清除合并请求的快速识别语义。 */
}

/*
 * 函数功能：清理当前活动事务的运行字段和跨周期接收缓存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Rfid_ResetActiveRequest(void)
{
    s_request_active = false; /* 当前硬件事务结束，下一周期才允许出队下一条。 */
    s_request_channel = CHANNEL_NONE; /* 清除活动通道，晚到回包不再有合法归属。 */
    s_request_source = RFID_READ_SOURCE_NONE; /* 清除协议来源，防止空闲周期误解析。 */
    s_request_attempts_left = 0U; /* 清除剩余发送次数。 */
    s_request_fast_mode = false; /* 清除快速识别语义，下一事务独立决定。 */
    s_request_ticket = 0U; /* 清除活动票据，避免重复结算。 */
    s_request_elapsed_ticks = 0U; /* 清除硬超时计时。 */
    s_request_hard_timeout_ticks = 0U; /* 清除当前事务超时上限。 */
    Rfid_ResetReceiveAccumulator(); /* 丢弃已完成事务的帧和噪声，不污染下一事务。 */
}

/*
 * 函数功能：结算并清理当前活动 RFID 逻辑事务。
 * 输入参数：result 为本事务成功、超时或取消终态。
 * 返回参数：无。
 */
static void Rfid_CompleteActiveRequest(RfidRequestResult_t result)
{
    uint8_t channel = s_request_channel; /* 在清理活动状态前保存事务通道。 */
    uint16_t ticket = s_request_ticket; /* 在清理活动状态前保存事务票据。 */

    Rfid_ResetActiveRequest(); /* 先释放硬件事务，下一周期可以处理队列中的另一通道。 */
    Rfid_CompleteTicket(channel, ticket, result); /* 再向 handlescan 发布对应票据的明确终态。 */
}

/*
 * 函数功能：取消指定通道尚未完成的统计请求，但不把业务主动中止计为射频丢包。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void Rfid_LinkStatsCancelPending(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 无效通道没有统计对象，不能访问 A/B 数组。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return;
    }

    /* 运行、切通道、清刀具等业务中止不是硬件丢包，只结束等待状态。 */
    s_link_response_pending[index] = false;
#else
    (void)channel; /* 关闭统计时编译为空操作，不增加状态内存和运行行为。 */
#endif
}

/*
 * 函数功能：记录一条 RFID 读取命令，并在发送新命令前结算上一条未响应命令。
 * 输入参数：channel 为实际发送命令的逻辑 A/B 通道。
 * 返回参数：无。
 */
static void Rfid_LinkStatsRecordRequest(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 无效通道不能生成可追溯的请求统计。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return;
    }

    /* 新命令发出前上一条仍无有效回包，说明该已完成请求发生一次丢失。 */
    if (s_link_response_pending[index] != false)
    {
        s_link_statistics[index].lost_response_count++;
    }

    /* 每次真正下发 EPC 读取命令都增加请求总数。 */
    s_link_statistics[index].request_count++;
    /* 新命令进入等待状态，直到收到有效帧或业务主动取消。 */
    s_link_response_pending[index] = true;
#else
    (void)channel; /* 关闭统计时不改变 RFID 命令发送时序。 */
#endif
}

/*
 * 函数功能：记录一条通过协议校验的 RFID 有效回包。
 * 输入参数：channel 为当前活动请求所属逻辑通道。
 * 返回参数：无。
 */
static void Rfid_LinkStatsRecordValidResponse(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 只有合法 A/B 通道且确有待响应命令时才计一次成功。 */
    if ((Rfid_ChannelToIndex(channel, &index) == false) ||
        (s_link_response_pending[index] == false))
    {
        return;
    }

    /* 有效回包只结算最近一条读取命令，避免重复 DMA 数据重复累计。 */
    s_link_statistics[index].valid_response_count++;
    /* 本条命令已经得到有效响应，不再被下一条命令结算为丢失。 */
    s_link_response_pending[index] = false;
#else
    (void)channel; /* 关闭统计时不改变解析成功后的业务流程。 */
#endif
}

/*
 * 函数功能：在一次 RFID 请求自然耗尽且仍无有效回包时结算最后一条丢失命令。
 * 输入参数：channel 为当前活动请求所属逻辑通道。
 * 返回参数：无。
 */
static void Rfid_LinkStatsRecordLostResponse(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 只有合法通道且存在待响应命令时才结算一次丢失。 */
    if ((Rfid_ChannelToIndex(channel, &index) == false) ||
        (s_link_response_pending[index] == false))
    {
        return;
    }

    /* 自然超时结束的最后一条命令没有下一次重发机会，需要在这里计入丢失。 */
    s_link_statistics[index].lost_response_count++;
    /* 本次请求已经结束，清除等待标志避免后续重复结算。 */
    s_link_response_pending[index] = false;
#else
    (void)channel; /* 关闭统计时不改变请求耗尽后的退出行为。 */
#endif
}

/*
 * 函数功能：记录一次收到数据但未解析出有效 EPC 帧的异常接收批次。
 * 输入参数：channel 为当前活动请求所属逻辑通道。
 * 返回参数：无。
 */
static void Rfid_LinkStatsRecordInvalidFrame(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 仅统计合法通道的异常数据，防止无归属噪声污染 A/B 结果。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return;
    }

    /* 16 位异常计数达到上限后保持饱和，避免长期运行回绕成 0 误导现场判断。 */
    if (s_link_statistics[index].invalid_frame_count < 0xFFFFU)
    {
        ++s_link_statistics[index].invalid_frame_count; /* 异常帧只影响统计，不结束当前命令等待。 */
    }
#else
    (void)channel; /* 关闭统计时不增加异常帧处理分支。 */
#endif
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
    uint8_t physical_channel = channel; /* 默认保持逻辑A/B与R200-K8物理A/B一致。 */

#if (RFID_R200_AB_SWAP_ENABLE == 1U)
    /* 旧硬件线束交叉时，逻辑A必须选通R200-K8物理B。 */
    if (channel == CHANNEL_A)
    {
        physical_channel = CHANNEL_B;
    }
    /* 旧硬件线束交叉时，逻辑B必须选通R200-K8物理A。 */
    else if (channel == CHANNEL_B)
    {
        physical_channel = CHANNEL_A;
    }
#endif

    /* R200-K8物理A侧由高电平接入UART3。 */
    if (physical_channel == BOARD_PROFILE_HANDLE_CHANNEL_A)
    {
        R200_K8_SELECT_A();
        return; /* 物理A已经完成选通，不再继续判断物理B。 */
    }

    /* R200-K8物理B侧由低电平接入UART3。 */
    if (physical_channel == BOARD_PROFILE_HANDLE_CHANNEL_B)
    {
        R200_K8_SELECT_B();
    }
#else
    (void)channel; /* 双串口模式下原物理A/B已固定到UART3/UART9，不再驱动R200-K8。 */
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
    /* 双串口线束没有随手柄接口交叉，逻辑B始终清理UART9，避免B回包进入A缓存。 */
    if (channel == CHANNEL_B)
    {
        Uart9_ClearRecvData();
        return; /* B通道缓存已清理，不再误清A通道UART3缓存。 */
    }
#else
    (void)channel; /* 旧硬件 A/B 仍共用 UART3，通道参数只用于 R200-K8 选通。 */
#endif
    Uart3_ClearRecvData(); /* 逻辑A或旧硬件共用模式清UART3，防止旧回包污染下一次解析。 */
}

/*
 * 函数功能：按业务通道发送 RFID 命令帧。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；pData 指向命令缓冲区；Length 为命令长度。
 * 返回参数：无。
 */
static void Rfid_SendPacketForChannel(uint8_t channel, uint8_t *pData, uint16_t Length)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    /* 双串口线束固定不交换，逻辑B命令始终由UART9发往B侧RFID模块。 */
    if (channel == CHANNEL_B)
    {
        Uart9_SendPacket(pData, Length);
        return; /* B通道已经发送完成，不再从A通道UART3重复发送。 */
    }
#else
    (void)channel; /* 旧硬件由 R200-K8 决定 UART3 当前连到 A 还是 B。 */
#endif
    Uart3_SendPacket(pData, Length); /* 逻辑A或旧硬件模式继续使用UART3，保持原RFID命令时序。 */
}

/*
 * 函数功能：按业务通道读取对应 RFID 串口 DMA 缓存。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；data 指向调用方接收缓冲区。
 * 返回参数：本次读取到的字节数，0 表示没有新回包。
 */
static uint16_t Rfid_PeekChannelUartData(uint8_t channel, uint8_t *data)
{
#if (RFID_USE_DUAL_UART_MODE == 1U)
    /* 双串口线束固定不交换，逻辑B只读取UART9，确保结果继续写入B缓存和B界面。 */
    if (channel == CHANNEL_B)
    {
        return Uart9_DMARecvDataPeek(data);
    }
#else
    (void)channel; /* 旧硬件共用 UART3，实际通道由 R200-K8 保证。 */
#endif
    return Uart3_DMARecvDataPeek(data); /* 逻辑A或旧硬件模式继续从UART3 DMA缓存取数据。 */
}

/*
 * 函数功能：判断本次 RFID 请求是否允许按当前通道状态执行。
 * 输入参数：channel 为请求读取的业务通道。
 * 返回参数：true 表示允许读取，false 表示当前状态禁止读取。
 */
static bool Rfid_IsRequestAllowed(uint8_t channel)
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
        Rfid_LinkStatsRecordRequest(channel); /* 发送前结算上一条未响应命令，并把本条命令纳入统计分母。 */
        Rfid_SendPacketForChannel(channel, NO_MASK3_WRITE_EPC, (uint16_t)sizeof(NO_MASK3_WRITE_EPC)); /* 双串口模式按逻辑A=UART3、逻辑B=UART9发送EPC读取命令。 */
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

    if ((RFIDMsgQueue == NULL) || (s_request_active != false))
    {
        return; /* 活动事务完成前不出队下一条，避免新请求替换事务并清空正在到达的新刀具回包。 */
    }

    if (Kernel_QueueReceive(RFIDMsgQueue, &msg, 0) != pdTRUE)
    {
        return; /* 本周期无排队请求，保持 RFID 硬件空闲。 */
    }

    if (msg.start == false)
    {
        if (Rfid_ChannelToIndex(msg.channel, &index) != false)
        {
            Rfid_CompleteTicket(msg.channel, s_outstanding_ticket[index], RFID_REQUEST_RESULT_CANCELED); /* 空闲期停止消息只取消对应通道排队事务。 */
        }
        return; /* 停止消息不得影响另一通道的活动或排队事务。 */
    }

    if ((Rfid_IsSourceValid(msg.source) == false) ||
        (Rfid_ChannelToIndex(msg.channel, &index) == false))
    {
        Rfid_CompleteTicket(msg.channel, msg.ticket, RFID_REQUEST_RESULT_CANCELED); /* 无效请求形成取消终态，调用方不能把它累计为射频缺失。 */
        return; /* 无效消息不切换硬件通道。 */
    }

    if (msg.generation != s_request_generation[index])
    {
        Rfid_CompleteTicket(msg.channel, msg.ticket, RFID_REQUEST_RESULT_CANCELED); /* 清刀具前排队的旧代次请求只结算取消。 */
        return; /* 旧代次不能重新启动 RFID 读取。 */
    }

    if (Rfid_IsRequestAllowed(msg.channel) == false)
    {
        Rfid_CompleteTicket(msg.channel, msg.ticket, RFID_REQUEST_RESULT_CANCELED); /* 运行或模式门禁拒绝属于业务取消。 */
        return; /* 非法状态下不切换 R200-K8，也不访问串口。 */
    }

    Rfid_SelectHardwareChannel(msg.channel); /* 只在事务真正启动时切换目标 RFID 模块。 */
    s_request_channel = msg.channel; /* 保存活动事务通道，解析成功后只写回同侧缓存。 */
    s_request_source = msg.source; /* 保存活动事务协议来源。 */
    s_request_ticket = msg.ticket; /* 保存逻辑事务票据，完成时发布明确结果。 */
    s_request_fast_mode = (bool)(msg.fast_mode || s_outstanding_fast_mode[index]); /* 同通道合并请求可把普通事务升级为快速识别。 */
    s_request_attempts_left = s_request_fast_mode ? RFID_FAST_ATTEMPTS : RFID_NORMAL_ATTEMPTS; /* 按最终合并模式设置发送预算。 */
    s_request_hard_timeout_ticks = s_request_fast_mode ? RFID_FAST_HARD_TIMEOUT_TICKS : RFID_NORMAL_HARD_TIMEOUT_TICKS; /* 设置独立硬超时防止半帧长期占用。 */
    s_request_elapsed_ticks = 0U; /* 新事务从第一个 100ms 周期开始计时。 */
    Rfid_ClearChannelUartData(s_request_channel); /* 只在事务起点清一次物理 DMA，活动过程中不再因普通请求清空半帧。 */
    Rfid_ResetReceiveAccumulator(); /* 新事务使用独立跨周期接收缓存。 */
    s_request_active = true; /* 标记硬件事务正式启动，完成前下一条请求只能等待。 */
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
    uint16_t rlen; /* 保存本周期从当前 RFID 串口 DMA 取到的字节数。 */
    uint8_t dat[RFID_UART_PACKET_SIZE] = {0U}; /* 临时接收本周期新字节，随后追加到事务缓存。 */
    RfidToolResult_t parsed_result; /* 保存跨周期缓存中解析出的完整标签。 */

    Rfid_ReceiveRequestMessage(); /* 只有硬件空闲时才从队列启动下一事务。 */
    if (s_request_active == false)
    {
        return; /* 没有活动事务时不占用 RFID 串口。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        Rfid_LinkStatsCancelPending(s_request_channel); /* 电机启动属于业务安全取消，不记作射频丢包。 */
        Rfid_ClearChannelUartData(s_request_channel); /* 丢弃运行前晚到字节，避免停机后误识别。 */
        Rfid_CompleteActiveRequest(RFID_REQUEST_RESULT_CANCELED); /* 向请求方发布取消终态并释放硬件事务。 */
        return; /* 运行态禁止继续访问 RFID 模块。 */
    }

    if (Rfid_IsRequestAllowed(s_request_channel) == false)
    {
        Rfid_LinkStatsCancelPending(s_request_channel); /* 通道或模式变化属于业务取消，不计链路丢失。 */
        Rfid_ClearChannelUartData(s_request_channel); /* 清原通道晚到回包，避免污染下一事务。 */
        Rfid_CompleteActiveRequest(RFID_REQUEST_RESULT_CANCELED); /* 完成取消后下一周期才能处理另一通道。 */
        return; /* 当前状态不允许继续识别。 */
    }

    if (s_request_elapsed_ticks < 0xFFU)
    {
        ++s_request_elapsed_ticks; /* 每个 100ms 周期只累计一次硬超时计数。 */
    }

    rlen = Rfid_PeekChannelUartData(s_request_channel, dat); /* 取出本周期 DMA 新字节，物理缓冲复位不影响逻辑累计缓存。 */
    if (rlen > 0U)
    {
#if (RFID_DEBUG_BEEP_EVERY_UART_RESPONSE == 1U)
        SendKeyBeepMessage(1U); /* 调试开关启用时，任意模块字节到达都蜂鸣。 */
#endif
        Rfid_AppendReceiveData(dat, rlen); /* 把短帧或半帧追加到跨周期缓存，不立即判定失败。 */
    }

    if ((s_rx_accum_length >= RFID_FRAME_MIN_SIZE) &&
        (Rfid_ParseReceivedFrame(s_rx_accumulator,
                                 s_rx_accum_length,
                                 s_request_source,
                                 &parsed_result) == true))
    {
        Rfid_LinkStatsRecordValidResponse(s_request_channel); /* 完整合法 EPC 结算最近一条读取命令成功。 */
        parsed_result.channel = s_request_channel; /* 解析器只管协议，事务补齐 A/B 归属。 */
        (void)Rfid_UpdateParsedCache(&parsed_result); /* 更新标签缓存；蜂鸣仍由 handlescan 按产品语义决定。 */
        Rfid_CompleteActiveRequest(RFID_REQUEST_RESULT_SUCCESS); /* 发布成功终态，在线监测此后才结算本轮。 */
        return; /* 成功事务本周期结束。 */
    }

    if ((s_request_elapsed_ticks >= s_request_hard_timeout_ticks) ||
        ((s_request_attempts_left == 0U) && (s_partial_grace_ticks == 0U)))
    {
        Rfid_LinkStatsRecordLostResponse(s_request_channel); /* 超时只结算最后一条未响应读取命令。 */
        if (s_request_saw_rx_data != false)
        {
            Rfid_LinkStatsRecordInvalidFrame(s_request_channel); /* 收到过字节但始终无完整 EPC 时只累计一次异常事务。 */
        }
        Rfid_CompleteActiveRequest(RFID_REQUEST_RESULT_TIMEOUT); /* 明确超时后 handlescan 才允许累计一次缺失。 */
        return; /* 超时事务本周期结束。 */
    }

    if (s_partial_grace_ticks > 0U)
    {
        --s_partial_grace_ticks; /* 半帧到达后空出一个周期等待剩余字节。 */
        return; /* 等待期间不补发新读命令，避免模块应答相互穿插。 */
    }

    Rfid_SelectHardwareChannel(s_request_channel); /* 每次发命令前确认共享 R200-K8 仍指向目标通道。 */
    Rfid_SendReadCommand(s_request_channel, s_request_source); /* 在无半帧等待时发送下一次 EPC 读取。 */
    --s_request_attempts_left; /* 每次真实发送只消耗一次预算，活动事务不会被下一请求重置。 */
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
    Rfid_SendPacketForChannel(channel, rfid_tx_power_command, (uint16_t)sizeof(rfid_tx_power_command)); /* 按通道发送当前选定功率，保证 A/B 模块使用同一配置。 */
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
    Rfid_InitModuleOnChannel(CHANNEL_A); /* 双串口模式先通过UART3初始化逻辑A侧RFID模块。 */
    Rfid_InitModuleOnChannel(CHANNEL_B); /* 双串口模式再通过UART9初始化逻辑B侧RFID模块。 */
#else
    Rfid_SelectHardwareChannel(CHANNEL_A); /* 旧模式先按独立交换宏选通逻辑A对应的物理RFID模块。 */
    Rfid_InitModuleOnChannel(CHANNEL_A); /* 通过共享UART3配置逻辑A当前选中的模块。 */
    Rfid_SelectHardwareChannel(CHANNEL_B); /* 再选通逻辑B对应模块，保证两套RFID都完成上电配置。 */
    Rfid_InitModuleOnChannel(CHANNEL_B); /* 通过共享UART3配置逻辑B当前选中的模块。 */
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
bool Rfid_RequestToolReadTracked(uint8_t channel,
                                 RfidReadSource_t source,
                                 bool fast_mode,
                                 uint16_t *ticket)
{
    RFIDMessage_t msg; /* 保存要送入 RFID 任务队列的逻辑事务。 */
    uint8_t index; /* 保存 A/B 通道对应的票据数组下标。 */
    uint16_t new_ticket; /* 保存本次新分配或合并返回的非零票据。 */

    if (ticket != NULL)
    {
        *ticket = 0U; /* 失败路径默认不给调用方留下可查询的旧票据。 */
    }

    if (RFIDMsgQueue == NULL)
    {
        if (channel == CHANNEL_A)
        {
            SendKeyRFIDMessageAdown(); /* 保留原A侧队列未初始化时的离线兼容行为。 */
        }
        else if (channel == CHANNEL_B)
        {
            SendKeyRFIDMessageBdown(); /* 保留原B侧队列未初始化时的离线兼容行为。 */
        }
        return false; /* 队列未初始化时不能创建逻辑事务。 */
    }

    if ((Rfid_ChannelToIndex(channel, &index) == false) || (Rfid_IsSourceValid(source) == false))
    {
        if (channel == CHANNEL_A)
        {
            SendKeyRFIDMessageAdown(); /* 无效A侧请求沿用原离线兼容入口。 */
        }
        else if (channel == CHANNEL_B)
        {
            SendKeyRFIDMessageBdown(); /* 无效B侧请求沿用原离线兼容入口。 */
        }
        return false; /* 只允许有效 A/B 通道和 EPC 来源。 */
    }

    if ((WorkMessage.runflag_work == true) || (Rfid_IsRequestAllowed(channel) == false))
    {
        return false; /* 运行或模式门禁期间不创建事务，避免运行参数被新标签覆盖。 */
    }

    if (s_outstanding_valid[index] != false)
    {
        if (fast_mode != false)
        {
            s_outstanding_fast_mode[index] = true; /* 同通道重复请求只升级识别预算，不重复排队。 */
            if ((s_request_active != false) && (s_request_channel == channel))
            {
                s_request_fast_mode = true; /* 活动普通事务被上线请求合并时同步升级当前语义。 */
                s_request_attempts_left = RFID_FAST_ATTEMPTS; /* 为刚进入场区的新刀具补足快速读取机会。 */
                s_request_hard_timeout_ticks = RFID_FAST_HARD_TIMEOUT_TICKS; /* 同步延长硬超时但仍保持明确上限。 */
            }
        }
        if (ticket != NULL)
        {
            *ticket = s_outstanding_ticket[index]; /* 返回已有票据，调用方继续等待同一事务终态。 */
        }
        return true; /* 同通道排队或活动事务不可被普通启动请求替换。 */
    }

    new_ticket = (uint16_t)(s_next_ticket[index] + 1U); /* 为该通道生成下一个事务票据。 */
    if (new_ticket == 0U)
    {
        new_ticket = 1U; /* 票据回绕时跳过0，0固定表示无有效事务。 */
    }
    s_next_ticket[index] = new_ticket; /* 记录该通道最近分配值，A/B互不影响。 */

    msg.channel = channel; /* 保存请求通道。 */
    msg.source = source; /* 保存读取来源，当前协议固定为 EPC。 */
    msg.start = true; /* true 表示启动一次逻辑事务。 */
    msg.fast_mode = fast_mode; /* 保存普通或快速识别语义。 */
    msg.generation = s_request_generation[index]; /* 记录入队代次，清刀具后旧消息会被取消。 */
    msg.ticket = new_ticket; /* 队列消息携带票据，完成结果可以回到原调用方。 */

    s_outstanding_valid[index] = true; /* 入队前占住本通道事务槽，防止并发重复排队。 */
    s_outstanding_ticket[index] = new_ticket; /* 保存排队或活动事务票据。 */
    s_outstanding_fast_mode[index] = fast_mode; /* 保存合并时需要继承的快速识别语义。 */
    if (Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0)) != pdTRUE)
    {
        s_outstanding_valid[index] = false; /* 队列满时回滚事务槽，允许后续周期重试。 */
        s_outstanding_fast_mode[index] = false; /* 回滚尚未入队的快速语义。 */
        return false; /* 未入队的请求没有可查询终态。 */
    }

    if (ticket != NULL)
    {
        *ticket = new_ticket; /* 入队成功后向调用方返回唯一事务票据。 */
    }
    return true; /* 请求已排队，活动事务结束后按FIFO执行。 */
}

/*
 * 函数功能：兼容原有无票据 RFID 刀具读取接口。
 * 输入参数：channel 为 A/B 通道；source 为 EPC；fast_mode 选择普通或快速识别。
 * 返回参数：true 表示请求已排队或合并，false 表示当前不能请求。
 */
bool Rfid_RequestToolRead(uint8_t channel, RfidReadSource_t source, bool fast_mode)
{
    return Rfid_RequestToolReadTracked(channel, source, fast_mode, NULL); /* 旧调用复用同一不可抢占事务入口。 */
}

/*
 * 函数功能：按通道和票据查询 RFID 逻辑事务完成状态。
 * 输入参数：channel 为 A/B 通道；ticket 为提交时返回的非零票据。
 * 返回参数：PENDING、SUCCESS、TIMEOUT、CANCELED 或 UNKNOWN。
 */
RfidRequestResult_t Rfid_QueryRequestResult(uint8_t channel, uint16_t ticket)
{
    uint8_t index; /* 保存查询通道对应的事务数组下标。 */

    if ((ticket == 0U) || (Rfid_ChannelToIndex(channel, &index) == false))
    {
        return RFID_REQUEST_RESULT_UNKNOWN; /* 无效通道或0票据不能用于累计缺失。 */
    }

    if ((s_outstanding_valid[index] != false) && (s_outstanding_ticket[index] == ticket))
    {
        return RFID_REQUEST_RESULT_PENDING; /* 排队和活动阶段都必须继续等待，不能提前计缺失。 */
    }

    if (s_completed_ticket[index] == ticket)
    {
        return s_completed_result[index]; /* 返回该票据最近发布的明确终态。 */
    }

    return RFID_REQUEST_RESULT_UNKNOWN; /* 票据已过期或从未存在时不推断成功或失败。 */
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
 * 函数功能：复制指定逻辑通道的 RFID 请求/应答累计统计。
 * 输入参数：channel 为 A/B 通道；statistics 为调用方提供的统计快照缓存。
 * 返回参数：统计开关已启用且通道有效时返回 true，否则清空输出并返回 false。
 */
bool Rfid_CopyLinkStatistics(uint8_t channel, RfidLinkStatistics_t *statistics)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */
#endif

    /* 输出为空时不能写入统计快照。 */
    if (statistics == NULL)
    {
        return false;
    }

    /* 先清空输出，关闭统计或通道无效时调用方不会读到旧栈数据。 */
    memset(statistics, 0, sizeof(*statistics));
#if (RFID_LINK_STATS_ENABLE == 1U)
    /* 只允许复制 A/B 两个逻辑通道的统计。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return false;
    }

    /* Cortex-M4 对齐 32 位读写是原子的；复制只提供当前时刻快照，不修改统计状态。 */
    *statistics = s_link_statistics[index];
    return true;
#else
    (void)channel; /* 关闭统计时保留稳定 API，但不分配内部计数数组。 */
    return false;
#endif
}

/*
 * 函数功能：记录一次已经得到成功或未响应结论的 RFID 在线监测。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
void Rfid_RecordMonitorCompletion(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 非 A/B 通道没有可归属的在线监测，禁止污染任一侧统计。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return;
    }

    /* 成功确认标签仍在或等待到下一监测周期未确认，都属于一次已完成监测。 */
    ++s_link_statistics[index].monitor_completion_count;
#else
    (void)channel; /* 关闭统计时编译为空操作，不增加运行状态和 RAM。 */
#endif
}

/*
 * 函数功能：记录一次达到 handlescan 既有缺失时间阈值的 RFID 刀具确认掉线。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
void Rfid_RecordConfirmedDropout(uint8_t channel)
{
#if (RFID_LINK_STATS_ENABLE == 1U)
    uint8_t index; /* 保存逻辑通道对应的统计数组下标。 */

    /* 非 A/B 通道不能形成有效掉线事件。 */
    if (Rfid_ChannelToIndex(channel, &index) == false)
    {
        return;
    }

    /* 掉线计数使用 16 位饱和值，保证长期运行后不会从最大值回绕到 0。 */
    if (s_link_statistics[index].confirmed_dropout_count < 0xFFFFU)
    {
        ++s_link_statistics[index].confirmed_dropout_count; /* 每次真实在线到离线边沿只由扫描层调用一次。 */
    }
#else
    (void)channel; /* 关闭统计时保持扫描层调用稳定，不产生额外计数。 */
#endif
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
        Rfid_LinkStatsCancelPending(channel); /* 清刀具属于业务主动复位，不把取消中的命令记为链路丢失。 */
        Rfid_ClearChannelUartData(channel); /* 丢弃活动事务晚到字节，防止旧标签重新上报。 */
        Rfid_CompleteActiveRequest(RFID_REQUEST_RESULT_CANCELED); /* 发布取消终态并完整释放活动事务及累计半帧。 */
    }
    else if (s_outstanding_valid[index] != false)
    {
        Rfid_CompleteTicket(channel,
                            s_outstanding_ticket[index],
                            RFID_REQUEST_RESULT_CANCELED); /* 取消尚未启动的排队事务，调用方不得把它累计为射频缺失。 */
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
    uint8_t index = 0U; /* A 通道固定使用事务数组第 0 项。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时无请求可停。 */
    }

    msg.channel = CHANNEL_A; /* 停止 A 通道旧请求。 */
    msg.source = RFID_READ_SOURCE_NONE; /* 停止消息不需要来源。 */
    msg.start = false; /* false 表示停止当前活动请求。 */
    msg.fast_mode = false; /* 停止消息不使用快速模式。 */
    msg.generation = s_request_generation[index]; /* 带上当前代次，避免结构体未初始化字节进入队列。 */
    msg.ticket = s_outstanding_valid[index] ? s_outstanding_ticket[index] : 0U; /* 有待处理事务时记录对应票据，否则明确为无票据。 */
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
    uint8_t index = 1U; /* B 通道固定使用事务数组第 1 项。 */

    if (RFIDMsgQueue == NULL)
    {
        return; /* 队列未初始化时无请求可停。 */
    }

    msg.channel = CHANNEL_B; /* 停止 B 通道旧请求。 */
    msg.source = RFID_READ_SOURCE_NONE; /* 停止消息不需要来源。 */
    msg.start = false; /* false 表示停止当前活动请求。 */
    msg.fast_mode = false; /* 停止消息不使用快速模式。 */
    msg.generation = s_request_generation[index]; /* 带上当前代次，避免结构体未初始化字节进入队列。 */
    msg.ticket = s_outstanding_valid[index] ? s_outstanding_ticket[index] : 0U; /* 有待处理事务时记录对应票据，否则明确为无票据。 */
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

    if (Rfid_ParseReceivedFrame(uartx_rf_buff, /* 旧入口只有解析出完整合法标签帧后才允许更新兼容结果。 */
                                UART3_MAX_PACKET_SIZE,
                                Rfid_LegacyTypeToSource(rfid_type),
                                &result) == false)
    {
        return; /* 缓冲中没有符合旧请求来源的有效帧。 */
    }

    result.channel = interface; /* 旧接口传入的 interface 作为结果通道归属。 */
    (void)Rfid_UpdateParsedCache(&result); /* 写入新缓存，供后续统一读取。 */
}
