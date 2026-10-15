#include "diagnostic.h"
#if (MAINCTRL_DIAG_ENABLE == 1U)

#include <string.h>
#include "main.h"
#include "app_task.h"
#include "Pubinterface.h"
#include "sscDRIVE.h"
#include "motoruartdata.h"
#include "pump_behavior_core.h"
#include "pump.h"
#include "soft_uart.h"
#include "board.h"
#include "board_profile.h"
#include "external_comm_protocol.h"
#include "iwdg.h"
#include "usart.h"

#define DIAG_REFRESH_MS 1000U /* 相同数据每 1000ms 补记一次；数据有变化就立即记录。调小会产生更多记录，更快覆盖旧日志。 */
#define DIAG_CACHE_COUNT 128U /* 最多记住 128 组数据的上次值，用于判断是否变化；用完后直接记日志。调大会增加 RAM 占用。 */
#define DIAG_TX_DEPTH 8U /* 普通 UART2 应答最多排队 8 帧；队列满后丢弃新帧并计数。调大会增加 RAM 占用。 */
#define DIAG_TX_TIMEOUT_MS 40U /* UART2 发送超过 40ms 仍未完成就中止本帧。160 字节在 115200 波特率下约需 14ms；调小会更早中止。 */
#define DIAG_LINK_TIMEOUT_MS 2500U /* 上位机每秒查询诊断状态；连续 2500ms 无诊断请求后恢复普通通信。调大会延后恢复，调小容易受 USB 延迟影响。 */
#define DIAG_WIRE_CHUNK 28U /* 每条串口日志最多放 28 个原始字节，较长报文拆成多条；修改此值必须同步上位机解码。 */

/* 一条日志固定占 48 字节，直接按此格式发送给上位机；记录时须持有公共业务互斥锁，不能从中断调用。 */
typedef struct
{
    uint32_t sequence; /* 本次采集的日志顺序号，从 1 开始；旧日志被覆盖后，新日志仍继续递增。 */
    uint32_t tick_ms; /* 写下这条日志时的主控毫秒数；不是报文到达上位机的时间。 */
    uint16_t kind; /* 日志内容的编号；上位机据此区分脚踏、手柄、泵和串口数据。 */
    uint8_t channel; /* 通道字段的含义由 kind 决定：A/B、串口号、任务号或操作阶段；0 的含义也须按该组解释。 */
    uint8_t words; /* 低 4 位是 value 中有效整数个数；高 4 位是采集时正在执行的任务号，0 表示启动阶段或其它任务。 */
    uint32_t value[DIAG_RECORD_WORDS]; /* 最多 9 个数值；每项含义由 kind 决定，不能把运行请求误读为电机或泵的反馈。 */
} DiagnosticRecord_t;
typedef char DiagnosticRecordSizeCheck[(sizeof(DiagnosticRecord_t) == 48U) ? 1 : -1]; /* 结构体不是 48 字节就让编译报错，防止上位机按旧格式读错数据。 */

typedef struct
{
    uint32_t tick_ms; /* 上一次真正把本组数据写入日志的主控毫秒数，用于每秒补记相同数据。 */
    uint16_t kind; /* 本项保存哪一组设备数据；0 表示本项尚未使用。重新开始采集时清零。 */
    uint8_t channel; /* 与 kind 一起确定比较的是哪台设备，A/B 泵或不同串口的数据不能互相比较。 */
    uint8_t words; /* 上次保存的 32 位整数个数；个数改变时需要重新记一条日志。 */
    uint32_t value[DIAG_RECORD_WORDS]; /* 本组上次收到的全部数值；即使本轮不记日志，也更新它，供下一轮比较。 */
} DiagnosticCache_t;

typedef struct
{
    uint8_t data[EXTERNAL_COMM_MAX_FRAME_SIZE]; /* 保存整帧的独立副本；入队后，调用方可以修改原来的组帧数组。 */
    uint16_t length; /* 本帧实际字节数，范围为 1～160；发送时只取这些字节。 */
} DiagnosticTxFrame_t;

typedef struct
{
    uint32_t begin; /* 取得公共业务互斥锁后、本轮业务函数开始前的主控毫秒数。 */
    uint32_t previous_begin; /* 上一轮业务函数的开始时刻，用于计算两轮实际间隔，单位 ms。 */
    uint32_t gap; /* 相邻两轮业务函数开始时间之差，单位 ms；首轮为 0。 */
    uint32_t maximum_gap; /* 本次采集内观察到的最大开始间隔，单位 ms；重新开始采集时清零。 */
    uint32_t maximum_duration; /* 本次采集内最长一次业务函数耗时，单位 ms；不包含本模块在函数返回后的采集时间。 */
    uint32_t count; /* 本次采集内业务函数正常返回的次数；重新开始采集时清零。 */
} DiagnosticTaskStats_t;

static DiagnosticRecord_t s_records[DIAG_RECORD_CAPACITY]; /* 循环保存最近 1024 条日志，共占 48KiB RAM；写满后从最旧记录的位置继续写。 */
static DiagnosticCache_t s_cache[DIAG_CACHE_COUNT]; /* 保存各组数据的上次值；本轮相同且不足 1 秒时，不重复写入日志。 */
static DiagnosticTaskStats_t s_task_stats[15]; /* 下标 1～14 与看门狗任务编号相同；下标 0 保存未列入该编号表的任务计时。 */
static uint32_t s_wire_stats[2][13][4]; /* 依次按接收/发送、串口号索引；每项为包数、最近毫秒时刻、发送失败数、最大包长，均由任务更新。 */
static uint32_t s_next_sequence = 1U; /* 下一条日志序号，从 1 递增；达到 32 位上限时停止记录，重新开始采集后回到 1。 */
static uint32_t s_count; /* 当前还能读取的日志条数，上限为 1024；重新开始采集时清零。 */
static uint32_t s_overwritten; /* 本次采集中已有多少条旧日志被覆盖；重新开始采集时清零。 */
static uint32_t s_cache_misses; /* 找不到空余数据比较项的次数；这种情况下仍然写日志，不代表丢了记录。 */
static uint32_t s_session = 1U; /* 本次开机中的采集批次号；每次重新开始加 1，读取旧批次的命令会被拒绝。 */
static uint8_t s_recording = 1U; /* 1 允许记日志，诊断版上电默认为 1；冻结或序号用尽时清零，重新开始采集时置 1。 */
static uint8_t s_active_task; /* 正在执行的业务任务号；进入业务函数前设置，记录完该轮数据后清零。 */
static uint32_t s_last_stats_tick; /* 上次写通信和任务统计的主控毫秒数，用于控制每秒记录一次。 */
static DiagnosticTxFrame_t s_normal_tx[DIAG_TX_DEPTH]; /* 普通外部通信应答的排队数组；发送时优先取这里的帧，再发送诊断应答。 */
static uint8_t s_tx_head; /* 普通发送队列中下一条待发帧的位置，取走一帧后向后移动，到末尾后回到开头。 */
static uint8_t s_tx_count; /* 普通发送队列中尚未取走的帧数；入队加 1，取走后减 1。 */
static DiagnosticTxFrame_t s_active_tx; /* UART2 正在发送的完整帧副本；发送完成或被中止前不能覆盖。 */
static volatile uint8_t s_tx_busy; /* 启动发送前置 1；最后停止位发完、发送超时或 HAL 拒绝发送后清零，0 才允许换下一帧。 */
static uint32_t s_tx_started; /* 本帧开始发送的主控毫秒数，用于判断是否超过 DIAG_TX_TIMEOUT_MS。 */
static uint32_t s_tx_dropped; /* 普通帧未能入队的累计次数，包含空指针、非法长度和队列满；按本次开机累计。 */
static uint32_t s_tx_failed; /* HAL 拒绝发送或 40ms 内未完成发送的累计次数；按本次开机累计。 */
static uint32_t s_diag_busy_requests; /* 前一条诊断应答尚未取出发送时，又收到不同诊断请求的累计次数。 */
static DiagnosticTxFrame_t s_reply; /* 保存最近生成的诊断应答；等待发送时不能覆盖，发送后仍留着供相同请求重试。 */
static uint8_t s_reply_pending; /* 生成应答或收到相同请求重试时置 1；应答取出到 s_active_tx 后清零。 */
static uint8_t s_request_key[12]; /* 保存上一条诊断请求的操作码、长度和参数；全部相同才复用应答，避免重试时再次清空日志。 */
static uint32_t s_link_tick; /* 最近收到格式正确诊断请求的主控毫秒数，每次请求及重试都更新。 */
static uint8_t s_link_seen; /* 上电为 0；首次诊断请求置 1，是否仍在诊断模式由请求间隔判断。 */
static uint8_t s_request_valid; /* 上电为 0；成功生成过诊断应答后置 1，此后才能比较上一条请求。 */

/*
 * 函数功能：向日志数组追加一条记录；写满后覆盖最旧记录，并累计覆盖数量。
 * 输入参数：kind 为数据组号；channel 的含义由该组决定；values 指向 words 个 32 位值，words 为 0～9。调用时须持有公共业务互斥锁。
 * 返回参数：无。
 */
static void Diagnostic_Append(uint16_t kind, uint8_t channel, const uint32_t *values, uint8_t words)
{
    DiagnosticRecord_t *record; /* 指向本次要写的日志位置；中断不会读取或写入该数组。 */
    if ((s_recording == 0U) || (values == NULL) || (words > DIAG_RECORD_WORDS)) return; /* 记录已冻结、数值指针为空或字段过多时直接返回，本条日志不写入。 */
    if (s_next_sequence == 0xFFFFFFFFUL)
    {
        s_recording = 0U; /* 日志序号即将用完，停止记录；须重新开始采集才继续，避免序号回到 0 后无法区分先后。 */
        return; /* 序号已用完，本次不再写入；现有日志仍可读取。 */
    }
    record = &s_records[(s_next_sequence - 1U) % DIAG_RECORD_CAPACITY]; /* 按日志序号计算数组位置；超过 1024 条后回到开头覆盖最旧记录。 */
    record->sequence = s_next_sequence++; /* 先保存本条序号，再把下一条序号加 1，保证记录有明确先后。 */
    record->tick_ms = HAL_GetTick(); /* 保存本条日志产生时的主控时间，方便与其它设备记录对齐。 */
    record->kind = kind; /* 保存数据类别，上位机据此解释每个数值及其单位。 */
    record->channel = channel; /* 保留调用方指定的通道或操作阶段，不用当前屏幕选中通道替换。 */
    record->words = (uint8_t)(words | (s_active_task << 4U)); /* 把正在执行的任务号放在高 4 位，便于查看哪轮任务之后数据发生变化。 */
    memset(record->value, 0, sizeof(record->value)); /* 清掉本位置上一条日志的数值，防止未使用字段带出旧数据。 */
    memcpy(record->value, values, (uint32_t)words * sizeof(uint32_t)); /* 把调用方已准备好的数值复制进日志，不在这里读硬件。 */
    if (s_count < DIAG_RECORD_CAPACITY) ++s_count; /* 缓存尚未满时增加保留条数。 */
    else ++s_overwritten; /* 数组已满，本次写入覆盖了一条旧日志，累计覆盖次数加 1。 */
}

/*
 * 函数功能：将本组数据与上次值比较；首次出现、数据变化或距离上次记录已满 1 秒时写日志。
 * 输入参数：kind 为组号，channel 为该组通道，values 含 words 个值（0～9）；ignore_mask 的第 n 位为 1 时，第 n 项变化不单独触发记录。
 * 返回参数：无。
 */
static void Diagnostic_State(uint16_t kind, uint8_t channel, const uint32_t *values, uint8_t words, uint16_t ignore_mask)
{
    uint16_t slot; /* 逐项查找本组数据上次存在哪里，最多检查 128 项。 */
    uint8_t i; /* 本组数值的下标，范围为 0～8。 */
    uint8_t changed; /* 1 表示新组、字段数量变化或需要比较的数值发生变化；本轮据此决定是否立即记日志。 */
    uint32_t now = HAL_GetTick(); /* 取得本轮主控毫秒数，用于判断相同数据是否已满 1 秒。 */
    DiagnosticCache_t *cache = NULL; /* 指向找到的比较项；没有空闲项时直接写日志，不跳过本条数据。 */
    if ((s_recording == 0U) || (values == NULL) || (words > DIAG_RECORD_WORDS)) return; /* 冻结期间或参数无效时不比较、不更新上次值，也不写日志。 */
    for (slot = 0U; slot < DIAG_CACHE_COUNT; ++slot)
    {
        if ((s_cache[slot].kind == kind) && (s_cache[slot].channel == channel)) { cache = &s_cache[slot]; break; } /* 只有组号和通道都相同，才可用这一项判断数据是否变化。 */
        if ((cache == NULL) && (s_cache[slot].kind == 0U)) cache = &s_cache[slot]; /* 先记住第一个空闲项；继续往后找，避免把已有组再次建成新项。 */
    }
    if (cache == NULL)
    {
        ++s_cache_misses; /* 比较数组已用满，累计一次不足次数，供上位机查看。 */
        Diagnostic_Append(kind, channel, values, words); /* 没有位置保存上次值，本轮直接写日志，保证新数据仍被记录。 */
        return; /* 本条已经写入，结束处理，不再进行数据比较。 */
    }
    changed = ((cache->kind == 0U) || (cache->words != words)) ? 1U : 0U; /* 此前未保存过本组，或字段个数不同，都需要马上写一条完整记录。 */
    for (i = 0U; i < words; ++i)
        if (((ignore_mask & (1U << i)) == 0U) && (cache->value[i] != values[i])) changed = 1U; /* 仅检查 ignore_mask 中对应位为 0 的数值；其中任一项改变就立即记日志。 */
    cache->kind = kind; cache->channel = channel; cache->words = words; /* 保存本项对应的组号、通道和字段个数，供下次查找。 */
    memcpy(cache->value, values, (uint32_t)words * sizeof(uint32_t)); /* 更新本组全部上次值，包括本轮不用于触发立即记录的时间和计数。 */
    if ((changed != 0U) || ((uint32_t)(now - cache->tick_ms) >= DIAG_REFRESH_MS))
    {
        cache->tick_ms = now; /* 本轮即将写日志，从现在重新计算下一次补记的等待时间。 */
        Diagnostic_Append(kind, channel, values, words); /* 首次出现、数值改变或已经过 1 秒时，把本组数据写入日志。 */
    }
}

/*
 * 函数功能：按结构体字段顺序，每 9 个整数写成一条日志，剩余字段使用下一个组号。
 * 输入参数：kind 为首条组号；channel 为该组通道；data 指向只含 32 位字段的数据，length 为其字节数，必须是 4 的倍数。
 * 返回参数：无。
 */
void Diagnostic_StateObject(uint16_t kind, uint8_t channel, const void *data, uint16_t length)
{
    uint32_t words[DIAG_RECORD_WORDS]; /* 一次最多取 9 个整数；先复制到对齐的局部数组，避免直接读取未对齐地址。 */
    uint16_t offset = 0U; /* 已从 data 中处理的字节数，初始为 0。 */
    if ((data == NULL) || ((length & 3U) != 0U)) return; /* 空指针或不足整项 32 位字段时不记录，避免读出不完整数据。 */
    while (offset < length)
    {
        uint16_t bytes = (uint16_t)(length - offset); /* 计算还剩多少字节未处理。 */
        if (bytes > sizeof(words)) bytes = sizeof(words); /* 一条日志最多保存 9 个整数，即 36 字节，多余数据留给下一条。 */
        memcpy(words, (const uint8_t *)data + offset, bytes); /* 从调用方传入的数据复制本条所需字节，不修改调用方的数组。 */
        Diagnostic_State(kind++, channel, words, (uint8_t)(bytes / 4U), 0U); /* 每条用下一个连续组号保存，方便上位机按声明顺序还原整组字段。 */
        offset = (uint16_t)(offset + bytes); /* 跳过本次已复制的字节，继续处理剩余字段。 */
    }
}

/*
 * 函数功能：记录业务函数已经做出的检查或操作；相同组、通道和参数每秒补记一次，变化立即记录。
 * 输入参数：event 为 DIAG_EVENT_* 编号；channel 和 a/b/c 的含义见 diagnostic.h 对应事件说明。
 * 返回参数：无。
 */
void Diagnostic_Event(uint16_t event, uint8_t channel, uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t value[4]; /* 前三项保存事件参数，第四项保存事件发生时正在执行的任务号。 */
    value[0] = a; value[1] = b; value[2] = c; value[3] = s_active_task; /* 按固定顺序复制参数和任务号，供上位机解释同一次操作。 */
    Diagnostic_State(event, channel, value, 4U, 0U); /* 同组同通道的事件参数变化就立即记录；完全相同时每秒补记，避免重复拒绝占满日志。 */
}
 /* 串口日志来自业务函数已经收发的字节；接收分段尚未通过 CRC 时，不能据此判断设备参数已更新。 */
/*
 * 函数功能：保存业务函数已有的串口字节和收发结果；长报文分段记录，相同分段每秒补记一次。
 * 输入参数：port 为 UART 编号，11/12 表示 A/B 压力串口；transmit=0 接收、1 发送；result 为 HAL 结果或 0x100 未校验标记；data 含 length 个字节，length 为 1～255。
 * 返回参数：无。
 */
void Diagnostic_Wire(uint8_t port, uint8_t transmit, uint16_t result, const uint8_t *data, uint16_t length)
{
    uint16_t offset; /* 本条分段数据在原始报文中的起始字节位置。 */
    uint32_t value[DIAG_RECORD_WORDS]; /* 前两个整数保存报文长度、处理结果和分段位置，后 28 字节保存串口原始数据。 */
    uint32_t *stats; /* 指向本串口、本收发方向的统计；屏幕切换通道不会改变这些计数。 */
    if ((port > 12U) || (transmit > 1U) || (data == NULL) || (length == 0U) || (length > 255U)) return; /* 串口编号、方向、数据指针或长度不合法时不记录；本接口最多接受 255 字节。 */
    if ((port == 2U) || (port == 6U)) return; /* UART2 的诊断应答不再记入自身日志；UART6 屏幕数据由显示消息记录，避免保存大量图形字节。 */
    stats = s_wire_stats[transmit][port]; /* 找到该串口接收或发送方向的统计数组。 */
    ++stats[0]; stats[1] = HAL_GetTick(); /* 每次调用都累计包数和时间；即使字节相同、不重复记日志，也能看出通信仍在继续。 */
    if ((transmit != 0U) && (result != 0U)) ++stats[2]; /* 只有发送的 HAL 返回值非 0 才增加失败数；接收的 CRC 错误另由解析函数记录。 */
    if (length > stats[3]) stats[3] = length; /* 保存本串口见过的最大字节数，辅助判断一次是否收到了多帧数据。 */
    for (offset = 0U; offset < length; offset = (uint16_t)(offset + DIAG_WIRE_CHUNK))
    {
        uint16_t bytes = (uint16_t)(length - offset); /* 计算原始报文中还剩多少字节。 */
        if (bytes > DIAG_WIRE_CHUNK) bytes = DIAG_WIRE_CHUNK; /* 本条最多保存 28 字节，剩余部分由后续循环继续记录。 */
        memset(value, 0, sizeof(value)); /* 清零未使用字节，避免最后一段不足 28 字节时带出旧内容。 */
        value[0] = (uint32_t)result | ((uint32_t)length << 16U); /* 低 16 位保存处理结果，高 16 位保存本次收发的原始字节总数。 */
        value[1] = offset; /* 保存分段起点；上位机据此判断这些字节在原始报文中的位置。 */
        memcpy(&value[2], &data[offset], bytes); /* 只复制调用方已有字节，不再执行一次串口收发。 */
        Diagnostic_State((uint16_t)(128U + transmit + (offset / DIAG_WIRE_CHUNK) * 2U), port, value, 9U, 0U); /* 从组 128 起，偶数组为接收、奇数组为发送；每个分段单独比较，相同分段每秒补记。 */
    }
}

/*
 * 函数功能：查询当前哪种控制方式占用了电机控制权；只读取，不申请或释放控制权。
 * 输入参数：无。
 * 返回参数：0 无人占用，1 外部上位机，2 脚踏，3 屏幕触控，4 手柄按键。
 */
static uint32_t Diagnostic_GetOwner(void)
{
    uint8_t owner; /* 依次检查四种控制方式；本函数只查询，不申请或释放控制权。 */
    for (owner = 1U; owner <= 4U; ++owner)
        if (ControlArbitration_IsOwner(owner)) return owner; /* 找到当前占用电机控制权的方式后返回其编号。 */
    return 0U; /* 四种方式都未占用电机控制权，记录为 0。 */
}

/*
 * 函数功能：记录当前选中手柄的设置、运行请求、在线标志，以及注水泵联动请求。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Diagnostic_CaptureWork(void)
{
    uint32_t value[22]; /* 当前手柄和联动泵相关的 22 个值，按下面的固定顺序导出。 */
    value[0] = WorkMessage.speed_work; /* 电机当前运行请求中的目标转速，单位 rpm，不是实测转速。 */
    value[1] = WorkMessage.speed_set_work; /* 用户为当前手柄设置的转速，单位 rpm，可与脚踏调速后的请求值比较。 */
    value[2] = WorkMessage.tool_reduction_ratio; /* 刀具倍率：高 16 位为增速比、低 16 位为减速比，两者都放大 100 倍保存。 */
    value[3] = WorkMessage.freq_work; /* 往复频率指令原值；不在诊断中换算为 Hz。 */
    value[4] = WorkMessage.dir_work; /* 屏幕采用的方向编号：0 正转、1 反转、2 往复；实际驱动命令方向另行记录。 */
    value[5] = WorkMessage.current_work; /* 当前手柄的过流保护设定，单位 0.01A，不是驱动反馈电流。 */
    value[6] = WorkMessage.runflag_work; /* 1 表示软件请求电机运行，0 表示撤销请求；不能据此认定电机已转动或已停。 */
    value[7] = WorkMessage.channel_work; /* 当前选中的手柄通道，1 为 A、2 为 B，用于检查通道是否真正切换。 */
    value[8] = WorkMessage.drivetype_work; /* 当前驱动方式：1 脚控、2 手控、3 触控或外控。 */
    value[9] = WorkMessage.hand_model; /* 当前选中手柄的型号编号，用于核对切换通道后是否采用了对应手柄参数。 */
    value[10] = WorkMessage.tool_type; /* 控制时采用的刀具类别，例如刨刀或磨刀。 */
    value[11] = WorkMessage.raw_tool_type; /* 识别到的具体刀具编号，保留比刨刀/磨刀分类更详细的型号。 */
    value[12] = WorkMessage.auto_identify; /* 当前通道为 1 时采用自动识别，0 时采用手动设置。 */
    value[13] = WorkMessage.Channel_Aonline; /* 业务保存的 A 手柄在线标志；不代表 A 电机正在运行。 */
    value[14] = WorkMessage.Channel_Bonline; /* 业务保存的 B 手柄在线标志；不代表 B 电机正在运行。 */
    value[15] = WorkMessage.alarm_value; /* 当前报警码。 */
    value[16] = WorkMessage.alarm_flag; /* 1 表示持续报警正在生效，0 表示没有持续报警。 */
    value[17] = WorkMessage.hmiactive_work; /* 业务保存的外控激活标志，用于和实际控制占用者比较。 */
    value[18] = WorkMessage.touchactive_work; /* 业务保存的屏幕触控激活标志，用于和实际控制占用者比较。 */
    value[19] = Pubinterface_GetHandleInjectionPumpFollowMask(); /* 位 0 表示 A 注水泵、位 1 表示 B 注水泵已被手柄联动请求选中；不代表泵已转动。 */
    value[20] = Pubinterface_IsPressureBlockStopLatched(); /* 读取泵驱动故障要求联动停止的标志；函数名含 Pressure，但此值不表示压力超限。 */
    value[21] = Diagnostic_GetOwner(); /* 记录当前占用电机控制权的方式：0 无、1 外控、2 脚踏、3 触控、4 手柄按键。 */
    Diagnostic_StateObject(1U, 0U, value, sizeof(value)); /* 按组 1、2、3 保存这 22 项数据，通道填 0 表示整机当前工作设置。 */
}

/*
 * 函数功能：分别记录脚踏、手柄按键和外部上位机的允许控制标志、运行请求及排空请求。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Diagnostic_CaptureSignals(void)
{
    uint32_t value[14]; /* 按固定顺序保存 14 个控制标志，每项 0 表示未请求或不允许，1 表示请求或允许。 */
    value[0] = ControlSignalMessage.jt_enable_flag; /* 是否允许脚踏提出电机控制请求，最终启动还需检查其它条件。 */
    value[1] = ControlSignalMessage.handle_enable_flag; /* 是否允许手柄按键提出电机控制请求。 */
    value[2] = ControlSignalMessage.HMI_enable_flag; /* 是否允许外部上位机提出控制请求。 */
    value[3] = ControlSignalMessage.jtL_control_flag; /* 左脚是否请求电机运行；右脚请求另存，不能互相替代。 */
    value[4] = ControlSignalMessage.jtR_control_flag; /* 右脚是否请求电机运行。 */
    value[5] = ControlSignalMessage.handle_control_flag; /* 手柄按键是否请求电机运行。 */
    value[6] = ControlSignalMessage.HMI_control_flag; /* 外部上位机是否请求当前通道电机运行。 */
    value[7] = ControlSignalMessage.jtL_gentlypump_flag; /* 左脚是否请求轻踩排空，不表示已向泵发送非零命令。 */
    value[8] = ControlSignalMessage.jtR_gentlypump_flag; /* 右脚是否请求轻踩排空。 */
    value[9] = ControlSignalMessage.HMI_gentlypump_flag; /* 外部上位机是否请求轻踩排空。 */
    value[10] = ControlSignalMessage.JTSCREENL_pump_flag; /* 原样保存左侧脚控泵的历史标志；判断泵是否运行还要看泵命令和反馈。 */
    value[11] = ControlSignalMessage.JTSCREENR_pump_flag; /* 原样保存右侧脚控泵的历史标志。 */
    value[12] = ControlSignalMessage.HMIL_pump_flag; /* 外部控制保存的左侧泵标志，和泵运行请求分开记录。 */
    value[13] = ControlSignalMessage.HMIR_pump_flag; /* 外部控制保存的右侧泵标志。 */
    Diagnostic_StateObject(5U, 0U, value, sizeof(value)); /* 按组 5、6 保存 14 个标志，通道为 0，不按当前选中通道隐藏另一侧请求。 */
}

/*
 * 函数功能：记录指定手柄通道保存的速度、方向和识别参数，用于核对切换前后的设置。
 * 输入参数：channel=1 为 A、2 为 B；memory 指向该通道的 ChannelMemoryMessage_t，不能为空。
 * 返回参数：无。
 */
static void Diagnostic_CaptureMemory(uint8_t channel, const ChannelMemoryMessage_t *memory)
{
    uint32_t value[18]; /* 将指定通道保存的 18 个参数按固定顺序复制，A/B 通道分别记录。 */
    value[0] = memory->zz_speed; /* 本通道保存正转速度rpm。 */
    value[1] = memory->fz_speed; /* 本通道保存反转速度rpm。 */
    value[2] = memory->osc_speed; /* 本通道保存往复速度rpm。 */
    value[3] = memory->tool_reduction_ratio; /* 本通道保存的刀具倍率：高 16 位增速比、低 16 位减速比，均放大 100 倍。 */
    value[4] = memory->freq; /* 本通道保存的往复频率指令值，切回该通道时使用。 */
    value[5] = memory->dir; /* 本通道保存的屏幕方向编号，实际驱动命令方向另记。 */
    value[6] = memory->current_work; /* 本通道过流阈值0.01A。 */
    value[7] = memory->default_injection_flow; /* 默认注水流量mL/min。 */
    value[8] = memory->hand_model; /* 本通道手柄编号。 */
    value[9] = memory->hand_type_raw_major; /* EEPROM原始类型高字节。 */
    value[10] = memory->hand_type_raw_minor; /* EEPROM原始类型低字节。 */
    value[11] = memory->tool_type; /* 刀具类别。 */
    value[12] = memory->raw_tool_type; /* 原始刀具型号。 */
    value[13] = memory->drive_type; /* 本通道保存的驱动方式，切回该通道时用于恢复控制设置。 */
    value[14] = memory->auto_identify; /* 本通道保存的识别方式：1 自动识别、0 手动设置。 */
    value[15] = memory->speed_alarm_for; /* 正转提示速度rpm。 */
    value[16] = memory->speed_alarm_rev; /* 反转提示速度rpm。 */
    value[17] = memory->freq_alarm_osc; /* 往复提示频率协议值。 */
    Diagnostic_StateObject(10U, channel, value, sizeof(value)); /* 按组 10、11 保存本通道参数，记录头保留 A/B 通道号。 */
}

/*
 * 函数功能：记录指定手柄通道识别出的刀具速度范围、调节量、方向和倍率。
 * 输入参数：channel=1 为 A、2 为 B；tool 指向该通道的刀具识别数据，不能为空。
 * 返回参数：无。
 */
static void Diagnostic_CaptureTool(uint8_t channel, const ChannelrecognizeMessage_t *tool)
{
    uint32_t value[18]; /* 将识别出的 18 个刀具参数按固定顺序复制，用于核对速度限制和方向。 */
    value[0] = tool->speed_zzmin; /* 正转下限rpm。 */
    value[1] = tool->speed_zzmax; /* 正转上限rpm。 */
    value[2] = tool->speed_fzmin; /* 反转下限rpm。 */
    value[3] = tool->speed_fzmax; /* 反转上限rpm。 */
    value[4] = tool->speed_oscmin; /* 往复下限rpm。 */
    value[5] = tool->speed_oscmax; /* 往复上限rpm。 */
    value[6] = tool->freq_min; /* 往复频率下限协议值。 */
    value[7] = tool->freq_max; /* 往复频率上限协议值。 */
    value[8] = tool->tool_reduction_ratio; /* 识别得到的刀具倍率：高 16 位增速比、低 16 位减速比，均放大 100 倍。 */
    value[9] = tool->speed_zzstep; /* 普通加减速操作一次改变的正转 rpm 数。 */
    value[10] = tool->speed_fzstep; /* 普通加减速操作一次改变的反转 rpm 数。 */
    value[11] = tool->speed_oscstep; /* 普通加减速操作一次改变的往复 rpm 数。 */
    value[12] = tool->speed_zzstep_large; /* 快速加减速操作一次改变的正转 rpm 数。 */
    value[13] = tool->speed_fzstep_large; /* 快速加减速操作一次改变的反转 rpm 数。 */
    value[14] = tool->speed_oscstep_large; /* 快速加减速操作一次改变的往复 rpm 数。 */
    value[15] = tool->run_direction; /* 刀具识别数据给出的方向编号，和屏幕所选方向分开记录。 */
    value[16] = tool->control_mode; /* 刀具识别数据给出的控制模式编号。 */
    value[17] = tool->rfid_cache_hit; /* 1 表示本次 RFID 识别使用了已保存的刀具参数；0 表示未使用。 */
    Diagnostic_StateObject(12U, channel, value, sizeof(value)); /* 按组 12、13 保存本通道识别参数，记录头保留 A/B 通道号。 */
}

/*
 * 函数功能：记录指定泵的设置、运行和排空请求、输出设定，以及已收到的压力数据。
 * 输入参数：channel=1 为 A 泵、2 为 B 泵；pump 指向对应的 pumpMessage_t，不能为空。
 * 返回参数：无。
 */
static void Diagnostic_CapturePump(uint8_t channel, const pumpMessage_t *pump)
{
    uint32_t value[18]; /* 本泵的 18 个设置和压力值，顺序与上位机字段表一致。 */
    value[0] = pump->type; /* 本泵类型：1 吸引、2 注水、3 灌注。 */
    value[1] = pump->speed_work; /* 用户泵速设定；注水和灌注单位为 mL/min，吸引使用内部档值。 */
    value[2] = pump->speed_output; /* 本轮换算后保存的泵输出设定，仍是命令值，不是实测流量或转速。 */
    value[3] = pump->online_flag; /* 1 表示业务已识别压力板和泵类型；是否允许开泵还要检查驱动反馈。 */
    value[4] = pump->run_flag; /* 普通运行或手柄联动的开泵请求，0 表示无该类请求，1 表示请求开泵。 */
    value[5] = pump->timingDrainage_flag; /* 屏幕定时排空请求，和脚踏轻踩排空分别记录。 */
    value[6] = pump->pedalDrainage_flag; /* 脚踏或外控的排空请求，不能用它代替泵实际反馈。 */
    value[7] = pump->timingDrainage_times; /* 本次屏幕定时排空已经过的泵任务周期数，每轮 25ms。 */
    value[8] = pump->direction; /* 业务保存的泵方向值；实际串口命令方向另行记录。 */
    value[9] = pump->step_value; /* 当前泵档位。 */
    value[10] = pump->associated_channel; /* 本泵保存的手柄通道关联标志，用于核对联动时为何选择该泵。 */
    value[11] = pump->speed_Min; /* 本泵设置下限，单位按类型。 */
    value[12] = pump->speed_Max; /* 本泵设置上限，单位按类型。 */
    value[13] = pump->speed_step_value; /* 每次调节量，单位按类型。 */
    value[14] = pump->losses_times; /* 已确认压力板掉线的累计次数，达到 255 后不再增加。 */
    value[15] = (uint32_t)pump->pressure_value; /* 压力板上报的 ADC 原始值，原本为有符号数，此处按原 32 位内容保存。 */
    value[16] = pump->weight_x10; /* 压力板上报的重量，单位 0.1g；6000 表示未就绪，不能当作正常测量值。 */
    value[17] = pump->pressure_threshold; /* 压力板上报阈值g。 */
    Diagnostic_StateObject(60U, channel, value, sizeof(value)); /* 按组 60、61 保存本泵请求与压力，记录头保留 A/B 泵号。 */
}

/*
 * 函数功能：记录最近通过 CRC 校验的电机反馈，并判断距该反馈是否已超过允许时间。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Diagnostic_CaptureMotorFeedback(void)
{
    MotorUartFeedbackSnapshot_t feedback = {0}; /* 先清零反馈副本；尚未收到有效回包时，valid 为 0，数值不可当作实测结果。 */
    (void)MotorUart_CopyFeedbackSnapshot(&feedback); /* 复制电机反馈模块已保存的最近有效回包，不额外向电机驱动查询。 */
    uint32_t value[7]; /* 保存反馈时间、序号、转速、电流、故障和两个有效标志。 */
    value[0] = feedback.feedback_tick_ms; /* 最近CRC正确反馈的主控毫秒时刻。 */
    value[1] = feedback.sequence; /* 有效反馈包计数，可回绕。 */
    value[2] = feedback.speed_rpm; /* 驱动回报实测速度rpm，协议未含通道。 */
    value[3] = feedback.current_x100; /* 驱动回报电流0.01A。 */
    value[4] = feedback.raw_error; /* 驱动原始故障码。 */
    value[5] = feedback.valid; /* 1 表示曾收到 CRC 正确的反馈；是否已超时还要看下一项。 */
    value[6] = feedback.valid && ((uint32_t)(HAL_GetTick() - feedback.feedback_tick_ms) <= MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS); /* 曾有有效反馈，且距该反馈不超过 MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS 时记 1，否则记 0。 */
    Diagnostic_State(82U, 0U, value, 7U, 3U); /* 组 82 不指定 A/B，因为回包没有通道号；掩码 3 只忽略时间和序号变化，其它字段变化立即记录。 */
}

/*
 * 函数功能：记录指定泵最近有效回包中的速度、方向、故障和定位状态，并检查回包是否超时。
 * 输入参数：channel=1 为 A 泵、2 为 B 泵；内部转成 0/1 后读取对应泵驱动反馈。
 * 返回参数：无。
 */
static void Diagnostic_CapturePumpFeedback(uint8_t channel)
{
    PumpDriverFeedbackSnapshot_t feedback = {0}; /* 先清零本泵反馈副本；valid 为 0 时不能把默认零速当作驱动已停止。 */
    (void)PumpBehavior_CopyDriverFeedbackSnapshot((PumpBehaviorChannel_t)(channel - 1U), &feedback); /* 把公开通道 1/2 转成内部 0/1，复制对应泵最近通过校验的回包。 */
    uint32_t value[9]; /* 保存本泵反馈的 9 项值，包括回包时间和是否仍在有效期。 */
    value[0] = feedback.feedback_tick_ms; /* 最近CRC正确反馈的毫秒时刻。 */
    value[1] = feedback.sequence; /* 有效反馈包计数。 */
    value[2] = feedback.actual_speed; /* 泵驱动回报的速度原值，没有换算成 mL/min。 */
    value[3] = feedback.direction; /* 泵驱动回包中的方向值，和主控准备发送的方向分开。 */
    value[4] = feedback.raw_error; /* 驱动原始故障码。 */
    value[5] = feedback.alignment_state; /* 驱动定位状态：0 不可用，0x80 就绪，0x90 定位中，0xA0 定位失败。 */
    value[6] = feedback.wire_status; /* 完整回包状态字节。 */
    value[7] = feedback.valid; /* 1 表示曾收到本泵的有效回包；该回包可能已超时，须同时看下一项。 */
    value[8] = feedback.valid && ((uint32_t)(HAL_GetTick() - feedback.feedback_tick_ms) <= 100U); /* 曾有有效反馈，且距该反馈不超过 100ms 时记 1，否则记 0。 */
    Diagnostic_State(83U, channel, value, 9U, 3U); /* 组 83 按 A/B 泵分别保存；掩码 3 只忽略回包时间和序号变化，其它反馈变化立即记录。 */
}

/*
 * 函数功能：记录电机控制任务最近生成的速度、方向和运行命令；这些是主控命令，不是驱动已执行的证明。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Diagnostic_CaptureMotorCommand(void)
{
    MotorDriveCommandSnapshot_t command = {0}; /* 先清零电机命令副本；尚无命令时记录默认值，不能当作驱动已执行。 */
    (void)MotorDrive_CopyCommandSnapshot(&command); /* 复制电机控制任务保存的命令；UART1 发送是否成功另看串口日志。 */
    uint32_t value[9]; /* 保存电机控制任务生成的 9 项命令数据，顺序与上位机一致。 */
    value[0] = command.evaluated_tick_ms; /* 最近一次判断电机应运行还是停止的主控毫秒数。 */
    value[1] = command.sent_tick_ms; /* 最近一次把有变化的电机命令交给 UART1 的主控毫秒数，不代表驱动已执行。 */
    value[2] = command.source_speed_rpm; /* 进行刀具倍率换算前的目标转速，单位 rpm。 */
    value[3] = command.command_speed_rpm; /* 发给驱动的速度字段乘 10 后得到的转速，单位 rpm。 */
    value[4] = command.sequence; /* 不同电机命令的累计序号，可用于判断请求变化后是否生成了新命令。 */
    value[5] = command.channel; /* 生成该命令时选中的逻辑通道，1 为 A、2 为 B。 */
    value[6] = command.run_state; /* 主控在该命令中采用的运行状态，和软件原始运行请求分别记录。 */
    value[7] = command.direction; /* 驱动串口采用的方向或控制模式编号，如 1 正转、2 反转、3 往复。 */
    value[8] = ((uint32_t)command.motor_type) ; /* 本条命令的电机类型编号，只保存 motor_type，不包含原始运行请求标志。 */
    Diagnostic_State(80U, 0U, value, 9U, 19U); /* 组 80 保存电机命令；掩码 19 忽略两项时刻和命令序号变化，速度、方向等变化立即记录。 */
}

/*
 * 函数功能：在业务函数执行前记下开始时间，并计算距上一轮开始已过多久。
 * 输入参数：task 指向当前任务，不能为空；调用前该任务已取得公共业务互斥锁。
 * 返回参数：无。
 */
void Diagnostic_TaskBegin(const struct task_s *task)
{
    DiagnosticTaskStats_t *stats; /* 指向当前任务的诊断计时数据；不写看门狗的完成记录。 */
    uint32_t now = HAL_GetTick(); /* 此时任务已取得公共业务互斥锁；之前等锁的时间不计入本轮业务耗时。 */
    s_active_task = (task->watchdog_id <= 14U) ? task->watchdog_id : 0U; /* 沿用现有看门狗任务号 1～14；未列入的任务用 0，供本轮日志标记采集任务。 */
    stats = &s_task_stats[s_active_task]; /* 选择本任务的计时项，未列入编号表的任务合用下标 0。 */
    stats->begin = now; /* 保存本轮开始时刻，业务函数返回后用它计算耗时。 */
    stats->gap = (stats->count != 0U) ? (uint32_t)(now - stats->previous_begin) : 0U; /* 已经完成过一轮才计算相邻开始间隔；首轮间隔记为 0。 */
    stats->previous_begin = now; /* 更新最近开始时刻，供下一轮计算间隔。 */
    if (stats->gap > stats->maximum_gap) stats->maximum_gap = stats->gap; /* 保留观察到的最长任务间隔，辅助判断是否等锁或未及时得到调度。 */
}

/*
 * 函数功能：业务函数返回后统计耗时，并集中记录脚踏、手柄和泵的数据；不启动新任务或额外硬件通信。
 * 输入参数：task 指向刚执行完业务函数的任务，不能为空；此时仍持有公共业务互斥锁。
 * 返回参数：无。
 */
void Diagnostic_TaskEnd(const struct task_s *task)
{
    DiagnosticTaskStats_t *stats = &s_task_stats[s_active_task]; /* 取出 TaskBegin 为本轮业务函数保存的计时数据。 */
    uint32_t duration = (uint32_t)(HAL_GetTick() - stats->begin); /* 计算业务函数从开始到返回经过的毫秒数；其中包含函数内部等待，不含下面的诊断采集。 */
    uint8_t channel; /* 依次取 A、B 的诊断数据，避免只记录屏幕当前选中的通道。 */
    if (duration > stats->maximum_duration) stats->maximum_duration = duration; /* 本轮比之前更慢时，更新该任务的最大执行耗时。 */
    ++stats->count; /* 业务函数已返回，完成次数加 1；这只是诊断计数，不用于喂看门狗。 */
    if ((duration > task->period) || (stats->gap > task->period * 2U)) /* 本轮耗时超过配置周期，或两轮开始间隔超过两倍周期时，补记一条任务延误。 */
        Diagnostic_Event(DIAG_EVENT_TASK_SLOW, s_active_task, duration, stats->gap, task->period); /* 保存延误的任务号、执行耗时和开始间隔，供上位机定位发生时间。 */
    Diagnostic_CaptureWork(); /* 每轮结束都检查手柄运行请求和选中通道，记录头同时带本轮任务号。 */
    Diagnostic_CaptureSignals(); /* 记录左右脚踏、手柄按键和外控请求，便于找出哪个请求没有撤销。 */
    Diagnostic_CapturePump(1U, &pumpMessageA); /* 记录 A 泵请求，包含其它任务刚修改、但 A 泵任务尚未处理的请求。 */
    Diagnostic_CapturePump(2U, &pumpMessageB); /* 独立记录 B 泵请求，不能把 A 泵数据当作 B 泵使用。 */
    if ((s_active_task == 1U) || (s_active_task == 2U)) /* 脚踏解析任务 1 或控制任务 2 结束后，读取脚踏数据。 */
    {
        DiagnosticFootState_t state = {0}; /* 临时保存脚踏数据，先清零，避免未填字段带入旧栈内容。 */
        Foot_CopyDiagnosticState(&state); /* 复制脚踏已保存的 AD、定标和松脚判断，不重新处理踩踏动作。 */
        Diagnostic_StateObject(20U, 0U, &state, sizeof(state)); /* 脚踏数据按组 20～23 连续保存，上位机按字段声明顺序解释。 */
    }
    for (channel = 1U; channel <= 2U; ++channel)
    {
        if (s_active_task == 3U) /* 手柄扫描任务 3 结束后，复制 A/B 各自的识别进度。 */
        {
            DiagnosticHandlescanState_t state = {0}; /* 本次循环只保存指定通道的手柄扫描数据。 */
            Handlescan_CopyDiagnosticState(channel, &state); /* 复制本通道认证记录和插入电平，不额外读取 EEPROM。 */
            Diagnostic_StateObject(30U, channel, &state, sizeof(state)); /* 按组 30、31 保存手柄认证、RFID 和插拔数据，记录头标明 A/B。 */
        }
        if (s_active_task == 4U) /* 手柄按键任务 4 结束后，复制两侧按键数据。 */
        {
            DiagnosticHandleKeyState_t state = {0}; /* 临时保存本通道按键确认结果和引脚用途，UART 模式的电平不能当作按键。 */
            HandleKey_CopyDiagnosticState(channel, &state); /* 复制已有按键计数并读取当前引脚电平，不增加连续按下或松开次数。 */
            Diagnostic_StateObject(40U, channel, &state, sizeof(state)); /* 组 40 保存本侧按键数据，记录头标明 A/B 通道。 */
        }
        if ((s_active_task == 3U) || (s_active_task == 11U)) /* 手柄识别任务 3 或按键行为任务 11 结束后，检查通道参数是否变化。 */
        {
            Diagnostic_CaptureMemory(channel, (channel == 1U) ? &MemoryMsgA : &MemoryMsgB); /* 读取指定通道保存的设置，用于检查切换通道后的参数是否对应。 */
            Diagnostic_CaptureTool(channel, (channel == 1U) ? &ChannelrecognizeMessageA : &ChannelrecognizeMessageB); /* 读取指定通道识别出的刀具参数，用于检查速度范围、方向和倍率。 */
        }
        if (s_active_task == (uint8_t)(channel + 6U)) /* 任务 7 是 A 泵、8 是 B 泵；只在对应泵任务结束后读取其内部处理记录。 */
        {
            DiagnosticPumpBehaviorState_t state = {0}; /* 临时保存本泵的定位步骤、停机限制和压力报警计数。 */
            PumpBehavior_CopyDiagnosticState(channel, &state); /* 复制本泵处理记录，不取走等待蜂鸣任务播放的定位提示。 */
            Diagnostic_StateObject(50U, channel, &state, sizeof(state)); /* 按组 50、51 保存驱动启动限制，用于解释有开泵请求时为何仍发零速。 */
            Diagnostic_CapturePumpFeedback(channel); /* 同时保存本泵有效回包中的速度、故障、定位状态和回包时间。 */
        }
    }
    if (s_active_task == 9U) /* 压力任务 9 结束后，检查两路压力板的接收和掉线计时。 */
    {
        for (channel = 1U; channel <= 2U; ++channel)
        {
            DiagnosticPressureState_t state = {0}; /* 临时保存本通道压力解析函数已有的时间和计数。 */
            SimUart_CopyDiagnosticState(channel, &state); /* 复制压力帧时间及越界后恢复计数，不取走接收队列中的字节。 */
            Diagnostic_StateObject(70U, channel, &state, sizeof(state)); /* 组 70 按 A/B 保存压力通信数据，和泵驱动串口反馈分开判断。 */
        }
    }
    if (s_active_task == 5U) Diagnostic_CaptureMotorCommand(); /* 电机控制任务 5 结束后，记录它本轮生成的最终命令。 */
    if ((s_active_task == 5U) || (s_active_task == 6U)) Diagnostic_CaptureMotorFeedback(); /* 电机控制或反馈任务结束后记录 UART1 回包；回包没有通道号，不按屏幕通道推定 A/B。 */
    s_active_task = 0U; /* 本轮采集结束，清掉当前任务号，后续日志不再误标为上一任务。 */
}

/*
 * 函数功能：把一个 32 位整数按低字节在前的顺序写入诊断应答数据。
 * 输入参数：data 为至少 4 字节的可写数组，不能为空；value 为要写入的整数。
 * 返回参数：无。
 */
static void Diagnostic_Write32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8U); /* 按低字节在前的顺序写入整数的第 0、1 字节。 */
    data[2] = (uint8_t)(value >> 16U); data[3] = (uint8_t)(value >> 24U); /* 继续写入第 2、3 字节，完成 32 位整数编码。 */
}

/*
 * 函数功能：从诊断命令中读取一个低字节在前的 32 位整数。
 * 输入参数：data 指向至少 4 字节的已接收数据，不能为空；调用方须先检查命令长度。
 * 返回参数：这 4 个字节组成的 32 位无符号整数。
 */
static uint32_t Diagnostic_Read32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U); /* 把四个字节分别移到对应位置再合并，不把未对齐地址直接当作 32 位指针。 */
}

/*
 * 函数功能：判断上位机是否仍在使用诊断通信，供普通外部通信让出 RS485。
 * 输入参数：无。
 * 返回参数：1 表示最近 2500ms 收到过诊断请求；0 表示尚未连接或已超时，可恢复普通通信。
 */
uint8_t Diagnostic_IsLinkActive(void)
{
    return (uint8_t)((s_link_seen != 0U) && ((uint32_t)(HAL_GetTick() - s_link_tick) < DIAG_LINK_TIMEOUT_MS)); /* 用无符号时间差兼容毫秒计数回绕，不改变电机和泵请求。 */
}

/*
 * 函数功能：检查 DMA 是否已收齐一条诊断请求；完整且 CRC 正确时不再等待三个空闲周期。
 * 输入参数：data 指向 DMA 已接收的数据，length 为已收到的字节数，不得包含尚未接收的区域。
 * 返回参数：1 表示整包是一条有效诊断请求；0 表示普通帧、半帧或坏帧，继续原接收等待。
 */
uint8_t Diagnostic_IsCompleteRequest(const uint8_t *data, uint16_t length)
{
    ExternalCommFrame_t frame; /* 解析结果仅用于判断完整性，业务命令仍交给原接收流程执行。 */
    if (ExternalCommProtocol_Parse(data, length, &frame) != EXTERNAL_COMM_PARSE_OK) return 0U; /* 半帧和 CRC 错误不能提前清空 DMA，留给后续字节补齐或原流程处理。 */
    return (uint8_t)((frame.length == length) && (frame.tran_code == EXTERNAL_COMM_TRAN_DOWNLOAD) && (frame.fun_code == DIAG_FUNCTION_CODE) && (frame.info_code == EXTERNAL_COMM_INFO_NONE)); /* 只提前取出诊断请求，普通协议保持原来的空闲等待。 */
}

/*
 * 函数功能：复制一帧普通外部通信应答并排队，立即返回；队列满或参数无效时丢弃本帧并计数。
 * 输入参数：data 指向完整正式帧或简易帧，不能为空；length 为 1～160 字节。
 * 返回参数：无。
 */
void Diagnostic_SendNormal(const uint8_t *data, uint16_t length)
{
    DiagnosticTxFrame_t *frame; /* 指向普通外部通信发送队列中即将写入的位置。 */
    if (Diagnostic_IsLinkActive() != 0U) return; /* 诊断模式只发送诊断应答，普通外控数据不排队，避免与上位机请求争用半双工串口。 */
    if ((data == NULL) || (length == 0U) || (length > EXTERNAL_COMM_MAX_FRAME_SIZE) || (s_tx_count >= DIAG_TX_DEPTH)) /* 空数据、非法帧长或队列已满时，本帧无法排队发送。 */
    {
        ++s_tx_dropped; /* 累计一条未入队的普通帧，上位机读取状态时能看到此计数。 */
        return; /* 丢弃本次帧并立即返回，调用方以后重试才可能发送成功。 */
    }
    frame = &s_normal_tx[(s_tx_head + s_tx_count) % DIAG_TX_DEPTH]; /* 根据队头和已排队条数计算队尾位置，满数组时回到开头。 */
    memcpy(frame->data, data, length); /* 把整帧复制进队列；函数返回后调用方可以修改原组帧数组。 */
    frame->length = length; /* 保存本帧字节数，发送时按此长度取数据。 */
    ++s_tx_count; /* 待发帧数加 1，下一轮发送服务会优先取普通帧。 */
}

/*
 * 函数功能：UART2 发完最后停止位后，把 RS485 切回接收，并允许任务发送下一帧。
 * 输入参数：huart 为 HAL 传入的发送完成串口句柄；不是 UART2 时直接返回。
 * 返回参数：无。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return; /* 本诊断模块只处理 UART2 发送完成，其它串口不在这里切换方向。 */
    HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 最后停止位已经发出，此时把 UART2 的 RS485 切回接收。 */
    __DMB(); /* 保证先完成方向引脚写入，再允许任务改写当前发送帧。 */
    s_tx_busy = 0U; /* 清除发送忙标志；下轮任务可发送下一帧，中断不写诊断日志。 */
}

/*
 * 函数功能：每秒把串口收发、压力接收和任务耗时计数写入日志，便于查看相同数据期间通信是否仍在继续。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Diagnostic_Statistics(void)
{
    uint8_t i; /* 循环使用的串口号或任务号，范围由下面两个循环分别限定。 */
    uint8_t direction; /* 0接收、1发送。 */
    uint32_t now = HAL_GetTick(); /* 与其它日志一样使用主控毫秒数，方便比较发生先后。 */
    if ((uint32_t)(now - s_last_stats_tick) < DIAG_REFRESH_MS) return; /* 距上次统计不足 1 秒时直接返回，避免每 10ms 写一批相同统计。 */
    s_last_stats_tick = now; /* 记住本次统计时刻，下一次至少等 1 秒。 */
    for (direction = 0U; direction < 2U; ++direction)
        for (i = 1U; i <= 12U; ++i)
            if (s_wire_stats[direction][i][0] != 0U) /* 本方向、本串口从未收发过数据时，跳过全零统计。 */
                Diagnostic_Append((uint16_t)(90U + direction), i, s_wire_stats[direction][i], 4U); /* 组 90 为接收、91 为发送；保存包数、最近时刻、发送失败数和最大包长。 */
    for (i = 1U; i <= 14U; ++i)
    {
        uint32_t value[5]; /* 每个任务依次记录完成次数、最近开始时刻、两轮间隔、最大间隔和最大耗时。 */
        value[0] = s_task_stats[i].count; value[1] = s_task_stats[i].previous_begin; /* 前两项写已完成轮数和最近一轮开始时刻，未执行过的任务保持 0。 */
        value[2] = s_task_stats[i].gap; value[3] = s_task_stats[i].maximum_gap; value[4] = s_task_stats[i].maximum_duration; /* 后三项都是毫秒数，用于比较实际任务间隔和执行时间。 */
        Diagnostic_Append(92U, i, value, 5U); /* 组 92 的通道字段放任务号；这些计数不改变看门狗的健康判断。 */
    }
    for (i = 0U; i < 2U; ++i)
    {
        SimUartStats stats; /* 临时接收一份压力软件串口统计，复制期间的一致性由其查询函数保证。 */
        SimUart_GetStats((sim_uart_channel_t)i, &stats); /* 查询压力串口现有计数，不清零计数，也不移除待解析字节。 */
        Diagnostic_StateObject(93U, (uint8_t)(i + 1U), &stats, sizeof(stats)); /* 组 93 分别保存 A 泵 PE4、B 泵 PE6 的压力接收统计。 */
    }
}

/*
 * 函数功能：检查 UART2 发送是否完成或超时，再发送下一帧；普通外控应答优先于诊断应答。
 * 输入参数：无；由每 10ms 执行的外部通信任务调用，调用时持有公共业务互斥锁。
 * 返回参数：无。
 */
void Diagnostic_TransportService(void)
{
    HAL_StatusTypeDef result; /* 保存 HAL 是否接受发送；HAL_OK 只表示已启动，最终完成由 TC 中断确认。 */
    uint32_t irq_enabled; /* 记住 UART2 中断原先是否开启，超时处理后按原值恢复。 */
    Diagnostic_Statistics(); /* 到 1 秒时把统计写入 RAM 日志，不主动给上位机发送统计帧。 */
    if (s_tx_busy != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - s_tx_started) < DIAG_TX_TIMEOUT_MS) return; /* 本帧还在 40ms 等待期内，立即返回，给其它业务任务执行机会。 */
        irq_enabled = NVIC_GetEnableIRQ(USART2_IRQn); NVIC_DisableIRQ(USART2_IRQn); /* 暂时关闭 UART2 中断，随后再查忙标志，避免发送恰好完成时仍中止它。 */
        if (s_tx_busy != 0U) /* 关中断后仍为忙，说明 TC 完成处理尚未清标志，需要中止超时帧。 */
        {
            (void)HAL_UART_AbortTransmit(&huart2); /* 只停止本次 UART2 发送，保留 DMA 接收，让上位机仍能重试。 */
            HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 超时后把 RS485 切回接收，避免一直占用线路。 */
            s_tx_busy = 0U; ++s_tx_failed; /* 允许后续发送，并把本帧计入发送失败次数。 */
        }
        if (irq_enabled != 0U) NVIC_EnableIRQ(USART2_IRQn); /* 仅当 UART2 中断原来开启时才重新开启，不改变其它中断。 */
    }
    if (s_tx_count != 0U) /* 先检查普通外部通信队列，普通应答优先发送。 */
    {
        s_active_tx = s_normal_tx[s_tx_head]; /* 复制队头帧到正在发送的数组，发送中断始终读取这个副本。 */
        s_tx_head = (uint8_t)((s_tx_head + 1U) % DIAG_TX_DEPTH); --s_tx_count; /* 队头移到下一条，待发数量减 1；发送中断不修改排队数组。 */
    }
    else if (s_reply_pending != 0U) /* 普通队列为空时，才检查有没有待发诊断应答。 */
    {
        s_active_tx = s_reply; /* 把诊断应答复制到正在发送的数组，原应答继续留作重试。 */
        s_reply_pending = 0U; /* 该应答已取出，不再标记为待发送；相同请求重试时可重新置 1。 */
    }
    else return; /* 两种队列都没有数据，本轮无需切换 RS485 方向或发送。 */
    s_tx_started = HAL_GetTick(); s_tx_busy = 1U; /* 先记开始时间并置忙，再启动中断发送，避免短帧先完成后忙标志又被置 1。 */
    HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_SET); /* 准备发这帧时才把 RS485 切为发送模式。 */
    result = HAL_UART_Transmit_IT(&huart2, s_active_tx.data, s_active_tx.length); /* 由 TXE/TC 中断逐字节发送；不用发送 DMA，避免接收 DMA 重启时影响本帧。 */
    if (result != HAL_OK) /* HAL 未接受发送，不能等待正常的发送完成回调。 */
    {
        HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 本帧未能启动，把 RS485 恢复为接收。 */
        s_tx_busy = 0U; ++s_tx_failed; /* 清发送忙标志并累计失败；本帧不在此自动重发，由上位机按协议重试。 */
    }
}

/*
 * 函数功能：处理上位机的日志状态、重新开始、冻结、读取、标记和版本查询；不会取得电机外控权限。
 * 输入参数：direction/function/area/info 为 CRC 正确帧的方向、功能、操作和信息码；data 指向 length 字节请求，诊断请求为 2～10 字节，前 2 字节是请求号。
 * 返回参数：1 表示本帧属于诊断协议，已处理或丢弃，不再交给业务控制；0 表示普通帧，继续原有命令处理。
 */
uint8_t Diagnostic_Command(uint8_t direction, uint8_t function, uint8_t area, uint8_t info, const uint8_t *data, uint16_t length)
{
    uint8_t payload[EXTERNAL_COMM_MAX_INFO_SIZE] = {0U}; /* 诊断应答的数据区，最多 144 字节；整帧仍由原外部通信协议补帧头和 CRC。 */
    uint8_t key[12] = {0U}; /* 用操作码、字节数和全部请求参数判断是否是上一条命令的重试。 */
    uint16_t payload_length = 16U; /* 应答前 16 字节固定保存版本、结果、请求号、采集批次、下一序号、条数和记录开关。 */
    uint32_t cursor = 0U; /* 上位机要求从哪条日志开始读取；0 表示从当前最旧日志开始。 */
    uint32_t first = s_next_sequence - s_count; /* 用下一条序号减当前条数，算出尚未被覆盖的最旧日志序号。 */
    uint8_t records = 0U; /* 本应答已放入的日志条数，从 0 开始，每帧最多返回 2 条。 */
    uint8_t status = 0U; /* 本次处理结果：0 成功，1 参数错误，2 批次不符，3 记录已覆盖，4 记录尚未产生。 */
    uint8_t expected_length; /* 当前操作所要求的精确字节数，多收或少收都不执行。 */
    uint32_t request_session; /* 上位机指定的采集批次；冻结、读取和标记只能操作同一批次。 */
    uint32_t active_begin; /* 重新开始采集时暂存当前任务开始时间，避免清统计后把本轮耗时算错。 */
    if (function != DIAG_FUNCTION_CODE) return 0U; /* 不是 0x0E 诊断功能码，返回 0，由外部通信代码继续处理普通业务命令。 */
    if ((direction != EXTERNAL_COMM_TRAN_DOWNLOAD) || (info != EXTERNAL_COMM_INFO_NONE) || (data == NULL) || (length < 2U) || (length > 10U)) return 1U; /* 方向或信息码错误、数据为空、请求号不完整或请求过长时丢弃；属于诊断帧，不能转为外控命令。 */
    if ((area <= 5U) && (length == ((area == 2U) ? 6U : (((area == 3U) || (area == 4U)) ? 10U : 2U)))) /* 只有已知操作且参数长度正确，才延长诊断通信时间。 */
    {
        s_link_tick = HAL_GetTick(); s_link_seen = 1U; /* 重试也刷新时间，下载期间持续保持诊断通信。 */
        s_tx_count = 0U; s_tx_head = 0U; /* 丢掉尚未发送的普通帧；已在发送的帧仍等 TC 完成，不能中途截断。 */
    }
    key[0] = area; key[1] = (uint8_t)length; memcpy(&key[2], data, length); /* 拼出本次请求的比较数据；未使用的字节已清零，保证重试比较结果稳定。 */
    if ((s_request_valid != 0U) && (memcmp(key, s_request_key, sizeof(key)) == 0)) /* 仅当本次操作、长度和所有参数都与上一条相同，才认为是重试。 */
    {
        s_reply_pending = 1U; /* 重新发送上次应答，不重新执行操作，避免“重新开始”重试把刚采到的日志再次清空。 */
        return 1U; /* 相同请求已安排重发，到此结束本帧处理。 */
    }
    if (s_reply_pending != 0U)
    {
        ++s_diag_busy_requests; /* 上一条应答还没取出，新请求暂不处理，只累计繁忙次数。 */
        return 1U; /* 保留上一应答，上位机等待超时后可重试本次请求。 */
    }
    expected_length = ((area == 2U) ? 6U : (((area == 3U) || (area == 4U)) ? 10U : 2U)); /* 操作 0/1/5 只带 2 字节请求号；2 另带 4 字节批次；3/4 再带 4 字节序号或标记。 */
    if ((area > 5U) || (length != expected_length)) status = 1U; /* 操作号超出 0～5 或长度不匹配时，返回参数错误，下面不执行采集操作。 */
    request_session = (length >= 6U) ? Diagnostic_Read32(&data[2]) : s_session; /* 长度足够才读批次字段；查询状态、重新开始和查询版本不带该字段。 */
    if ((status == 0U) && (area >= 2U) && (area <= 4U) && (request_session != s_session)) status = 2U; /* 冻结、读日志或标记指定了旧批次时拒绝，防止影响已重新开始的采集。 */
    if (status == 0U) /* 只有操作号、长度和批次检查都通过，才执行下面的日志操作。 */
    {
        if (area == 1U) /* 操作 1：开始新一批采集，旧日志不再提供读取。 */
        {
            ++s_session; if (s_session == 0U) s_session = 1U; /* 生成新的非零批次号；计数溢出到 0 时从 1 重新开始。 */
            s_next_sequence = 1U; s_count = 0U; s_overwritten = 0U; s_cache_misses = 0U; /* 重置日志序号、有效条数和覆盖统计；旧字节不必擦除，因为有效条数已变为 0。 */
            memset(s_cache, 0, sizeof(s_cache)); /* 清掉各组数据的上次值，让新采集首次见到每组时都记录。 */
            active_begin = s_task_stats[s_active_task].begin; /* 先保留执行当前命令的任务开始时间，清统计后还需要算本轮耗时。 */
            memset(s_task_stats, 0, sizeof(s_task_stats)); /* 清掉上一批的任务次数、最大间隔和最大耗时。 */
            s_task_stats[s_active_task].begin = active_begin; /* 恢复本轮真实开始时间，避免用 0 算出异常长的耗时。 */
            s_task_stats[s_active_task].previous_begin = active_begin; /* 保存本轮起点，下一轮还能计算两个开始时刻的间隔。 */
            s_recording = 1U; /* 允许后续业务任务继续写日志；已有电机和泵运行请求保持原值。 */
            Diagnostic_Event(DIAG_EVENT_MARK, 0U, 1U, s_session, HAL_GetTick()); /* 写一条“开始采集”记录，保存批次号和主控时间。 */
        }
        else if (area == 2U) /* 操作 2：停止追加日志，保留已有记录供上位机分次读取。 */
        {
            Diagnostic_Event(DIAG_EVENT_MARK, 0U, 2U, s_session, HAL_GetTick()); /* 先记下冻结操作发生的时刻，再关闭日志写入。 */
            s_recording = 0U; /* 只冻结日志；脚踏、手柄和泵继续按原有请求运行。 */
        }
        else if (area == 3U) /* 操作 3：按上位机给定的序号，最多取两条日志放入应答。 */
        {
            cursor = Diagnostic_Read32(&data[6]); /* 读取请求第 6～9 字节中的日志起始序号，前面已确认请求共有 10 字节。 */
            if (cursor == 0U) cursor = first; /* 请求序号为 0 时，从当前还保留的最旧日志开始读。 */
            if (cursor < first) { status = 3U; cursor = first; } /* 请求的旧日志已被覆盖，返回状态 3 和当前最旧序号，由上位机显示记录缺失。 */
            else if (cursor > s_next_sequence) status = 4U; /* 请求序号超过下一条待写序号，返回状态 4，本次不读数组。 */
            else
            {
                while ((cursor < s_next_sequence) && (records < 2U))
                {
                    const DiagnosticRecord_t *record = &s_records[(cursor - 1U) % DIAG_RECORD_CAPACITY]; /* 按日志序号计算数组位置；读取期间仍持有公共业务互斥锁，其它任务不能改写日志。 */
                    memcpy(&payload[16U + records * sizeof(*record)], record, sizeof(*record)); /* 把完整 48 字节记录放在应答头之后；STM32 为小端，记录大小已由编译检查保证。 */
                    ++records; ++cursor; /* 本帧条数加 1，并把序号移到下一条；上位机下次从返回的序号继续读。 */
                }
                payload_length = (uint16_t)(16U + records * sizeof(DiagnosticRecord_t)); /* 应答数据长度为 16 字节头加本次日志；两条日志共 112 字节，未超过 134 字节上限。 */
            }
        }
        else if (area == 4U) Diagnostic_Event(DIAG_EVENT_MARK, 0U, 4U, Diagnostic_Read32(&data[6]), HAL_GetTick()); /* 操作 4：记录上位机给定的标记值和主控时刻，不修改电机或泵请求。 */
        else if (area == 5U) /* 操作 5：返回诊断固件编译信息、接口映射和已保存的复位原因。 */
        {
            static const char build[] = "R1 c7e8e4e + diag-v1 " __DATE__ " " __TIME__; /* 记录这版诊断代码的源码基线和编译时刻；它不是板上 EEPROM 保存的产品版本。 */
            memcpy(&payload[16], build, sizeof(build)); /* 把编译说明写入 64 字节文本区，保留末尾 0，未使用字节已清零。 */
            Diagnostic_Write32(&payload[80], HANDLE_PHYSICAL_AB_SWAP_ENABLE); /* 返回手柄 A/B 逻辑通道是否交换到另一物理接口，便于解释插拔数据。 */
            Diagnostic_Write32(&payload[84], PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE); /* 返回 A/B 泵驱动是否交换 UART5/7；压力板 A/PE4、B/PE6 不随此配置交换。 */
            Diagnostic_Write32(&payload[88], 115200U); /* 返回 UART2 当前固定波特率 115200，只作说明，不更改串口设置。 */
            Diagnostic_Write32(&payload[92], g_iwdg_last_reset.reset_flags); /* 返回启动时保存的复位标志，用于判断上次是否由看门狗复位。 */
            Diagnostic_Write32(&payload[96], g_iwdg_last_reset.fault_reason); /* 返回看门狗已保存的故障原因，不读取或清除 EEPROM。 */
            Diagnostic_Write32(&payload[100], g_iwdg_last_reset.overdue_task); /* 返回看门狗保存的上次超时任务号，编号与任务统计一致。 */
            Diagnostic_Write32(&payload[104], g_iwdg_last_reset.active_task); /* 返回看门狗记录故障时正在执行的任务号。 */
            Diagnostic_Write32(&payload[108], g_iwdg_last_reset.stage); /* 返回看门狗故障记录中的任务执行阶段原值。 */
            Diagnostic_Write32(&payload[112], HAL_GetTick()); /* 返回本次开机已运行的毫秒数；复位后 RAM 日志会丢失，不拼接成上次开机记录。 */
            payload_length = 116U; /* 版本和配置信息共占 116 字节，含最前面的 16 字节应答头。 */
        }
    }
    if ((area <= 2U) || (area == 4U)) /* 状态查询、重新开始、冻结和手工标记都返回当前日志数量及通信统计。 */
    {
        first = s_next_sequence - s_count; /* 操作可能清空或追加日志，需要按操作后的数量重算最旧序号。 */
        Diagnostic_Write32(&payload[16], first); Diagnostic_Write32(&payload[20], s_next_sequence); /* 返回最旧日志序号和下一条待写序号；可读序号包含前者，不包含后者。 */
        Diagnostic_Write32(&payload[24], s_count); Diagnostic_Write32(&payload[28], s_overwritten); /* 返回当前可读条数及本批已覆盖条数，上位机据此提示旧记录是否丢失。 */
        Diagnostic_Write32(&payload[32], (s_count != 0U) ? s_records[(first - 1U) % DIAG_RECORD_CAPACITY].tick_ms : 0U); /* 有日志时返回最旧一条的主控时刻；没有日志时填 0。 */
        Diagnostic_Write32(&payload[36], (s_count != 0U) ? s_records[(s_next_sequence - 2U) % DIAG_RECORD_CAPACITY].tick_ms : 0U); /* 有日志时返回最新一条的主控时刻；两端时间用于显示实际保留的时间范围。 */
        Diagnostic_Write32(&payload[40], s_tx_dropped); Diagnostic_Write32(&payload[44], s_tx_failed); /* 返回本次开机累计的普通帧入队失败数和 UART2 发送失败数。 */
        Diagnostic_Write32(&payload[48], s_cache_misses); Diagnostic_Write32(&payload[52], s_diag_busy_requests); /* 返回比较数组不足次数和诊断应答未取出时收到新请求的次数。 */
        Diagnostic_Write32(&payload[56], HAL_GetTick()); payload_length = 60U; /* 最后返回当前主控毫秒数，本类应答数据共 60 字节。 */
    }
    payload[0] = DIAG_PROTOCOL_VERSION; payload[1] = status; /* 写入协议版本和处理结果；状态非 0 时，上位机不能当作操作成功。 */
    payload[2] = data[0]; payload[3] = data[1]; /* 原样返回请求号，方便上位机区分本次应答与迟到的旧应答。 */
    Diagnostic_Write32(&payload[4], s_session); /* 每种应答都返回当前采集批次，防止上位机把两次采集混在一起。 */
    Diagnostic_Write32(&payload[8], (area == 3U) ? cursor : s_next_sequence); /* 读日志时返回下一次该读的序号；其它操作返回下一条待写日志序号。 */
    payload[12] = records; payload[13] = s_recording; /* 返回本帧日志条数和记录开关，0 为冻结、1 为记录中；冻结不表示设备已停止。 */
    if (ExternalCommProtocol_BuildFrame(EXTERNAL_COMM_TRAN_UPLOAD, DIAG_FUNCTION_CODE, area, EXTERNAL_COMM_INFO_NONE, payload, payload_length, s_reply.data, sizeof(s_reply.data), &s_reply.length) == EXTERNAL_COMM_BUILD_OK) /* 用原外部通信格式组成完整诊断应答，成功后才把它交给发送服务。 */
    {
        memcpy(s_request_key, key, sizeof(key)); s_request_valid = 1U; s_reply_pending = 1U; /* 记住本次请求以便重试，并标记诊断应答待发送；普通应答仍优先。 */
    }
    return 1U; /* 本帧是诊断命令，处理到此结束，不能再交给外控登录或保活处理。 */
}

/*
 * 函数功能：一条业务操作处理完后立即记录手柄和 A/B 泵请求，避免同一任务内连续启停只留下最后状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Diagnostic_Checkpoint(void)
{
    Diagnostic_CaptureWork(); Diagnostic_CaptureSignals(); /* 保存该操作后的手柄设置，以及各控制方式是否仍有运行请求。 */
    Diagnostic_CapturePump(1U, &pumpMessageA); Diagnostic_CapturePump(2U, &pumpMessageB); /* 分别保存 A/B 泵请求，避免后续操作覆盖这次启停变化。 */
}

/*
 * 函数功能：记录脚踏控制任务从队列取出的连接和定标配置，便于核对它是否已经采用解析函数收到的新配置。
 * 输入参数：connected 为 0 离线、1 在线；type 为 1 单踏板、2 双段、3 双脚踏；ll/lm/lh、rl/rm/rh 分别为左、右侧低中高定标 AD 值。
 * 返回参数：无。
 */
void Diagnostic_FootConfig(uint8_t connected, uint8_t type, uint16_t ll, uint16_t lm, uint16_t lh, uint16_t rl, uint16_t rm, uint16_t rh)
{
    uint32_t value[8]; /* 按固定顺序保存控制任务实际取出的 8 项配置，不用新收到但尚未处理的配置替代。 */
    value[0] = connected; value[1] = type; /* 保存这条脚踏消息中的连接标志和脚踏类型。 */
    value[2] = ll; value[3] = lm; value[4] = lh; /* 保存左侧松开、中点、踩到底的三个 AD 定标值。 */
    value[5] = rl; value[6] = rm; value[7] = rh; /* 保存右侧松开、中点、踩到底的三个 AD 定标值。 */
    Diagnostic_State(24U, 0U, value, 8U, 0U); /* 组 24 专门表示控制任务已取出的配置，便于和组 20～23 的解析结果比较。 */
}

/*
 * 函数功能：记录相关屏幕消息是否成功入队、何时取出；取出消息不等于屏幕已经刷新。
 * 输入参数：stage=0 入队、1 取出；area 为显示区域编号；enabled 为显示使能值；data 为 10 字节参数或 NULL；result=0 失败、1 成功。
 * 返回参数：无。
 */
void Diagnostic_Ui(uint8_t stage, uint8_t area, uint8_t enabled, const uint8_t *data, uint8_t result)
{
    uint32_t value[6] = {0U}; /* 前三项保存操作阶段、使能和结果；后三项容纳 10 字节显示参数，多出的 2 字节保持 0。 */
    if ((area > 16U) && (area != 18U)) return; /* 只记录显示区域 0～16 和 18；其它区域直接跳过，不写本条日志。 */
    value[0] = stage; value[1] = enabled; value[2] = result; /* 分别保存准备入队还是已取出、显示使能值，以及本次是否成功。 */
    if (data != NULL) memcpy(&value[3], data, 10U); /* 参数非空时复制 10 字节；为空则保持全零，不读取空地址。 */
    Diagnostic_State(118U, area, value, 6U, 0U); /* 组 118 的通道字段保存显示区域编号；保留原始字节顺序供上位机解释。 */
}

#endif /* MAINCTRL_DIAG_ENABLE */
