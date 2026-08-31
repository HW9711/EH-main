#include "motor_foot_trace.h"

#include "diagnostic_config.h"

#include "control_arbitration.h"
#include "kernel_scheduler.h"
#include "motoruartdata.h"
#include "Pubinterface.h"
#include "sscDRIVE.h"
#include "sscFOOT.h"
#include "uart2.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

/* 调试器可直接观察的诊断任务与 UART2 在线帧状态，不参与任何业务控制。 */
volatile int32_t g_motor_foot_trace_task_create_result = 0;
volatile int32_t g_motor_foot_trace_task_start_result = 0;
volatile uint32_t g_motor_foot_trace_task_run_count = 0U;
volatile uint32_t g_motor_foot_trace_alive_attempt_count = 0U;
volatile uint32_t g_motor_foot_trace_alive_success_count = 0U;
volatile uint32_t g_motor_foot_trace_alive_last_status = (uint32_t)HAL_ERROR;
volatile uint32_t g_motor_foot_trace_alive_last_tick = 0U;

#if (MOTOR_FOOT_TRACE_ENABLE == 1U)

/* 单条 CSV 最大长度；格式化只发生在故障冻结之后的低频导出任务中。 */
#define MOTOR_FOOT_TRACE_LINE_SIZE            256U
/* 相同脚踏类型下 ADC 变化不足 4 且未满 50ms 时不写环形缓冲，避免正常抖动淹没关键边沿。 */
#define MOTOR_FOOT_TRACE_ADC_CHANGE_THRESHOLD 4U
#define MOTOR_FOOT_TRACE_FRAME_LOG_PERIOD_MS  50U
/* RUN/STOP 命令不变时每 250ms 留一条保活记录，变化命令仍立即记录。 */
#define MOTOR_FOOT_TRACE_COMMAND_LOG_PERIOD_MS 250U
/* owner 阻塞状态不变时每 250ms 留一条，避免 25ms 脚踏任务快速覆盖故障前环形记录。 */
#define MOTOR_FOOT_TRACE_OWNER_BLOCK_LOG_PERIOD_MS 250U
/* 故障日志字段较长，发送超时单独放宽；正常在线帧使用更短的独立超时。 */
#define MOTOR_FOOT_TRACE_UART_TIMEOUT_MS      80U
/* 13 字节在线帧在 115200 8N1 下约 1.1ms，5ms 上限可避免异常串口长时间阻塞任务。 */
#define MOTOR_FOOT_TRACE_ALIVE_TIMEOUT_MS     5U

typedef enum
{
    MF_DUMP_WAIT_START = 0U, /* 等待 300ms 后发送 EHDBG_START。 */
    MF_DUMP_SEND_HEADER,     /* 发送字段表头。 */
    MF_DUMP_SEND_RECORDS,    /* 每周期发送一条环形记录。 */
    MF_DUMP_SEND_END,        /* 发送统计和结束标记。 */
    MF_DUMP_COMPLETE         /* 本次上电的第一份现场已经导出完成。 */
} MotorFootTraceDumpPhase_t;

static MotorFootTraceRecord_t s_trace[MOTOR_FOOT_TRACE_CAPACITY]; /* 固定 RAM 环形缓冲，禁止动态分配。 */
static volatile uint16_t s_write_index;            /* 下一条记录写入位置。 */
static volatile uint16_t s_count;                  /* 当前有效记录数。 */
static volatile uint16_t s_record_sequence;        /* 每条记录递增的序号。 */
static volatile uint32_t s_overwritten_count;      /* 环形缓冲写满后的覆盖次数。 */
static volatile uint8_t s_initialized;             /* 只有 MotorFootTrace_Init 完成后才允许埋点写入。 */
static volatile uint8_t s_frozen;                  /* 第一份异常触发后置 1，阻止后续覆盖。 */
static volatile uint8_t s_trigger;                 /* 本次冻结原因。 */
static volatile uint32_t s_freeze_tick;            /* 异常冻结的主控毫秒时刻。 */
static volatile uint16_t s_dump_index;             /* 下一条待导出记录索引。 */
static volatile uint16_t s_dump_remaining;         /* 尚未导出的记录数量。 */
static volatile uint16_t s_dump_total;             /* 冻结时保存的总记录数。 */
static volatile uint32_t s_tx_fail_count;          /* UART2 发送失败次数，不做阻塞重试。 */
static volatile uint32_t s_format_error_count;     /* CSV 行超长或格式化失败次数。 */
static MotorFootTraceDumpPhase_t s_dump_phase;     /* 故障后导出状态机。 */
static kernel_task_t s_trace_dump_task;            /* 30ms 调度任务：正常期检查在线帧周期，冻结后逐行导出。 */
static uint32_t s_last_alive_tick;                  /* 最近一次尝试发送固定在线帧的主控毫秒时刻。 */

static volatile uint8_t s_release_pending;         /* 1 表示已观察到脚踏释放且尚未获得新的合法运行授权。 */
static volatile uint32_t s_release_tick;           /* 本次释放边界时刻。 */
static volatile uint8_t s_last_command_valid;      /* 已观察到至少一份最终 UART1 命令。 */
static volatile uint8_t s_last_command_run;        /* 最近最终命令 1=RUN、0=STOP。 */
static volatile uint32_t s_last_stop_command_tick; /* RUN 转 STOP 的时刻，重复 STOP 不刷新宽限期。 */
static uint16_t s_last_logged_command_sequence;    /* 最近已写入环形缓冲的命令序号。 */
static uint32_t s_last_command_log_tick;            /* 相同命令保活记录时刻。 */
static uint8_t s_frame_log_valid;                   /* 已保存至少一份脚踏实时帧采样。 */
static uint8_t s_last_frame_pedal_type;             /* 最近记录的脚踏类型。 */
static uint16_t s_last_frame_adc_left;              /* 最近记录的左 ADC。 */
static uint16_t s_last_frame_adc_right;             /* 最近记录的右 ADC。 */
static uint32_t s_last_frame_log_tick;               /* 最近记录脚踏帧的主控毫秒时刻。 */
static uint8_t s_owner_block_log_valid;              /* 已保存至少一份脚踏 owner 阻塞记录。 */
static uint8_t s_last_block_owner;                   /* 最近记录的阻塞 owner。 */
static uint8_t s_last_block_runflag;                 /* 最近记录的阻塞周期业务运行位。 */
static uint32_t s_last_owner_block_log_tick;         /* 最近 owner 阻塞记录时刻。 */
static volatile uint32_t s_observed_trigger_mask;    /* 自动条件只置对应位并记录一次，禁止重复事件覆盖人工反应窗口。 */
static volatile uint16_t s_capture_session;          /* 每次上电或收到 ARM 命令递增，供上位机区分多轮人工测试。 */

/*
 * 函数功能：计算两个 16 位 ADC 值的绝对差，用于过滤正常采样抖动。
 * 输入参数：left、right 为待比较值。
 * 返回参数：两者绝对差。
 */
static uint16_t MotorFootTrace_AbsDiffU16(uint16_t left, uint16_t right)
{
    return (left >= right) ? (uint16_t)(left - right) : (uint16_t)(right - left); /* 先比较再相减，避免无符号下溢。 */
}

/*
 * 函数功能：从 UART2 接收包尾匹配人工诊断命令，允许命令前存在旧二进制帧或转换器回显噪声。
 * 输入参数：data、data_length 为已去除尾部空白的接收包；command、command_length 为目标 ASCII 命令。
 * 返回参数：包尾完整匹配命令返回 1，否则返回 0。
 */
static uint8_t MotorFootTrace_CommandMatchesTail(const uint8_t *data,
                                                 uint16_t data_length,
                                                 const uint8_t *command,
                                                 uint16_t command_length)
{
    if ((data == NULL) || (command == NULL) || (command_length == 0U) || (data_length < command_length))
    {
        return 0U; /* 空指针、空命令或接收长度不足均不得触发冻结或重置。 */
    }

    if (memcmp(&data[data_length - command_length], command, command_length) == 0)
    {
        return 1U; /* 只接受位于有效接收包末尾的完整命令，前置二进制噪声不影响人工按钮。 */
    }
    return 0U; /* 包尾不完全一致时保持诊断状态不变。 */
}

/*
 * 函数功能：清空当前环形记录的发布状态并重新进入人工监测，旧数组内容不再计入有效记录。
 * 输入参数：无。
 * 返回参数：无。
 */
static void MotorFootTrace_ResetCaptureState(void)
{
    uint32_t primask; /* 只保护采集开关的关闭和重新发布，批量状态清零不长时间关闭中断。 */

    primask = __get_PRIMASK(); /* 保存调用前中断状态，ARM 命令不能误开启上层已经关闭的中断。 */
    __disable_irq();
    s_initialized = 0U; /* 先拒绝其它任务的新记录，避免重置过程中形成半新半旧快照。 */
    s_frozen = 1U; /* 与 initialized 双重封闭采集入口，直到本轮全部索引和门禁重置完成。 */
    if (primask == 0U)
    {
        __enable_irq(); /* 状态发布完成后立即恢复中断，后续清零不占用控制中断时间。 */
    }

    s_write_index = 0U; /* 新一轮从数组首项写入；旧数组内容因 count=0 不会被导出。 */
    s_count = 0U; /* 清除上一轮有效记录数量，人工冻结只导出本轮数据。 */
    s_record_sequence = 0U; /* 新一轮记录序号从 1 重新开始，便于现场检查是否丢行。 */
    s_overwritten_count = 0U; /* 新一轮尚未覆盖记录。 */
    s_trigger = (uint8_t)MF_TRIGGER_NONE; /* ARM 后尚未人工冻结。 */
    s_freeze_tick = 0U; /* 清除上一轮冻结时刻。 */
    s_dump_index = 0U; /* 导出起点等待人工 DUMP 时按记录数量确定。 */
    s_dump_remaining = 0U; /* ARM 后没有待导出记录。 */
    s_dump_total = 0U; /* ARM 后没有冻结总数。 */
    s_tx_fail_count = 0U; /* 每轮独立统计 UART2 导出失败。 */
    s_format_error_count = 0U; /* 每轮独立统计 CSV 格式错误。 */
    s_dump_phase = MF_DUMP_WAIT_START; /* 人工 DUMP 后仍从同步起始行开始。 */
    s_release_pending = 0U; /* ARM 时不继承上一轮脚踏释放保护窗口。 */
    s_release_tick = 0U; /* 清除上一轮释放边沿时刻。 */
    s_last_command_valid = 0U; /* 新一轮先等待一份真实 UART1 命令再做命令/反馈对照。 */
    s_last_command_run = 0U; /* 默认保存 STOP，但 valid=0 时不得据此判断疑似续转。 */
    s_last_stop_command_tick = 0U; /* 尚无本轮 STOP 边沿。 */
    s_last_logged_command_sequence = 0U; /* 清除命令日志去重基准。 */
    s_last_command_log_tick = 0U; /* 清除命令保活记录时刻。 */
    s_frame_log_valid = 0U; /* ARM 后收到的第一份真实脚踏帧必须记录。 */
    s_last_frame_pedal_type = 0U; /* 清除脚踏类型节流基准。 */
    s_last_frame_adc_left = 0U; /* 清除左 ADC 节流基准。 */
    s_last_frame_adc_right = 0U; /* 清除右 ADC 节流基准。 */
    s_last_frame_log_tick = 0U; /* 清除脚踏帧保活记录时刻。 */
    s_owner_block_log_valid = 0U; /* ARM 后第一份 owner 阻塞状态必须记录。 */
    s_last_block_owner = CONTROL_OWNER_NONE; /* 清除上一轮阻塞 owner。 */
    s_last_block_runflag = 0U; /* 清除上一轮阻塞运行位。 */
    s_last_owner_block_log_tick = 0U; /* 清除 owner 阻塞节流时刻。 */
    s_observed_trigger_mask = 0U; /* 每类自动疑似条件在新一轮可以重新记录一次。 */
    s_capture_session = (uint16_t)(s_capture_session + 1U); /* 会话号自然回绕，只用于区分相邻测试轮次。 */

    primask = __get_PRIMASK(); /* 所有字段完成清零后再原子开放采集。 */
    __disable_irq();
    s_frozen = 0U; /* 先解除冻结，再由 initialized 最后发布完整可用状态。 */
    s_initialized = 1U; /* 从本指令开始其它任务可以写入新一轮环形记录。 */
    if (primask == 0U)
    {
        __enable_irq(); /* 恢复调用前中断状态。 */
    }
}

/*
 * 函数功能：把局部完整记录原子写入环形缓冲，满缓冲时覆盖最旧记录。
 * 输入参数：record 指向已经在临界区外组装完成的记录。
 * 返回参数：无。
 */
static void MotorFootTrace_Push(const MotorFootTraceRecord_t *record)
{
    uint32_t primask; /* 保存调用前中断状态，避免错误开启原本已经关闭的中断。 */

    if ((record == NULL) || (s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* 未初始化、已冻结或空记录都不得改写第一份故障现场。 */
    }

    primask = __get_PRIMASK(); /* 只在复制固定结构和推进索引期间关中断。 */
    __disable_irq();
    if (s_frozen != 0U)
    {
        if (primask == 0U)
        {
            __enable_irq(); /* 触发任务已经冻结现场时恢复调用前中断状态并放弃迟到记录。 */
        }
        return; /* 二次确认关闭“检查后被抢占、冻结后仍覆写一条”的竞争窗口。 */
    }
    s_record_sequence = (uint16_t)(s_record_sequence + 1U); /* 只有真正提交到环形缓冲的记录才占用序号，冻结竞争放弃的记录不会制造断号。 */
    s_trace[s_write_index] = *record; /* 单次结构体复制完成后再发布下一写入位置。 */
    s_trace[s_write_index].record_sequence = s_record_sequence; /* 在同一临界区写入连续序号，上位机可据此核验冻结会话是否完整。 */
    s_write_index = (uint16_t)(s_write_index + 1U); /* 推进环形写指针。 */
    if (s_write_index >= MOTOR_FOOT_TRACE_CAPACITY)
    {
        s_write_index = 0U; /* 到达容量后回到首条记录。 */
    }
    if (s_count < MOTOR_FOOT_TRACE_CAPACITY)
    {
        ++s_count; /* 缓冲未满时只增加有效记录数。 */
    }
    else
    {
        ++s_overwritten_count; /* 缓冲已满时本次写入覆盖了最旧记录，单独累计供现场评估。 */
    }
    if (primask == 0U)
    {
        __enable_irq(); /* 只有调用前允许中断时才恢复中断。 */
    }
}

/*
 * 函数功能：把一行诊断文本经 UART2/RS485 发送并统计失败，不做循环重试。
 * 输入参数：text 指向完整 ASCII 行；length 为实际长度。
 * 返回参数：发送成功返回 1，失败返回 0。
 */
static uint8_t MotorFootTrace_SendLine(const char *text, uint16_t length)
{
    if ((text == NULL) || (length == 0U))
    {
        ++s_format_error_count; /* 空行表示调用方格式异常，不占用 UART2。 */
        return 0U;
    }
    if (Uart2_SendPacketChecked((const uint8_t *)text, length, MOTOR_FOOT_TRACE_UART_TIMEOUT_MS) != HAL_OK)
    {
        ++s_tx_fail_count; /* 发送失败只计数，禁止故障后阻塞安全相关周期任务。 */
        return 0U;
    }
    return 1U; /* 当前行已经完整交给 HAL 发送。 */
}

/*
 * 函数功能：正常诊断期发送固定 EHDBG_ALIVE 在线帧，让上位机区分“串口已打开”和“主控已在线”。
 * 输入参数：无。
 * 返回参数：无；发送失败后等待下一个 2 秒周期，不在控制运行期重试。
 */
static void MotorFootTrace_SendAlive(void)
{
    static const uint8_t alive[] = "EHDBG_ALIVE\r\n"; /* 固定 13 字节，不做 snprintf，也不携带任何业务状态。 */
    HAL_StatusTypeDef status; /* 保存本次底层发送结果，便于脱离上位机直接在调试器中判断。 */

    ++g_motor_foot_trace_alive_attempt_count; /* 发送前计数；只增长尝试数也能证明诊断任务已经运行到 UART2 调用点。 */
    g_motor_foot_trace_alive_last_tick = HAL_GetTick(); /* 记录实际进入发送函数的主控时刻，不能用调度计划时刻代替。 */
    status = Uart2_SendPacketChecked(alive,
                                     (uint16_t)(sizeof(alive) - 1U),
                                     MOTOR_FOOT_TRACE_ALIVE_TIMEOUT_MS); /* 发送完成、超时或失败后 UART2 底层都会立即恢复接收。 */
    g_motor_foot_trace_alive_last_status = (uint32_t)status; /* 持久保存最近 HAL 结果，故障后暂停调试器也能看到。 */
    if (status == HAL_OK)
    {
        ++g_motor_foot_trace_alive_success_count; /* 只有 HAL 确认完整发送后才累计成功数。 */
    }
}

/*
 * 函数功能：按冻结原因选择专用故障事件，保证上位机可直接显示中文根因。
 * 输入参数：trigger 为第一份异常触发原因。
 * 返回参数：对应的 MotorFootTraceEvent_t。
 */
static MotorFootTraceEvent_t MotorFootTrace_EventFromTrigger(MotorFootTraceTrigger_t trigger)
{
    switch (trigger)
    {
        case MF_TRIGGER_RELEASE_BUT_RUNFLAG:
            return MF_TRACE_FAULT_RELEASE_BUT_RUNFLAG; /* 已建立释放状态但业务运行位仍为 1。 */
        case MF_TRIGGER_RELEASE_BUT_RUN_TX:
            return MF_TRACE_FAULT_RELEASE_BUT_RUN_TX; /* 已观察释放但最终 UART1 仍发送 RUN。 */
        case MF_TRIGGER_STOP_TX_BUT_DRIVER_MOVING:
            return MF_TRACE_FAULT_STOP_TX_BUT_DRIVER_MOVING; /* STOP 宽限后驱动仍反馈明显非零速度。 */
        case MF_TRIGGER_FOOT_TASK_BLOCKED_ON_RELEASE:
            return MF_TRACE_FAULT_FOOT_TASK_BLOCKED_ON_RELEASE; /* 释放区处理被其它 owner 提前返回。 */
        case MF_TRIGGER_MANUAL:
            return MF_TRACE_MANUAL_FREEZE; /* 人工冻结只保存现场，不推断自动根因。 */
        case MF_TRIGGER_NONE:
        default:
            return MF_TRACE_MANUAL_FREEZE; /* 未知触发按人工冻结记录，避免伪造具体故障类型。 */
    }
}

/*
 * 函数功能：解析上位机人工监测命令；ARM 重新开始循环记录，DUMP 冻结当前现场并准备导出。
 * 输入参数：无。
 * 返回参数：本周期收到并处理过串口数据返回 1，否则返回 0。
 */
static uint8_t MotorFootTrace_ProcessCommand(void)
{
    static const uint8_t arm_command[] = "EHDBG_ARM"; /* ARM 只重置诊断环形记录，不改变手柄、脚踏、泵或控制权。 */
    static const uint8_t dump_command[] = "EHDBG_DUMP"; /* DUMP 由测试人员确认异常后发送，主控不再自行决定冻结时刻。 */
    uint8_t rx_data[UART2_MAX_PACKET_SIZE]; /* UART2 DMA 单次最大包缓冲，局部数组只在 30ms 诊断任务栈使用。 */
    char response[MOTOR_FOOT_TRACE_LINE_SIZE]; /* 命令应答沿用诊断文本最大行长，避免额外全局 RAM。 */
    uint16_t received_length; /* 保存本次 DMA 静默成包后的真实字节数。 */
    int response_length; /* 保存 snprintf 结果，异常或超长时不发送半截应答。 */

    received_length = Uart2_DMARecvDataPeek(rx_data); /* 诊断模式独占 UART2，正式外控任务不会同时取走本命令。 */
    if (received_length == 0U)
    {
        return 0U; /* 尚未形成完整命令时继续正常心跳或故障导出。 */
    }

    while ((received_length > 0U) &&
           ((rx_data[received_length - 1U] == (uint8_t)'\r') ||
            (rx_data[received_length - 1U] == (uint8_t)'\n') ||
            (rx_data[received_length - 1U] == (uint8_t)' ') ||
            (rx_data[received_length - 1U] == (uint8_t)'\t')))
    {
        --received_length; /* 只忽略命令尾部空白，前缀和正文必须逐字匹配，防止噪声误触发冻结。 */
    }

    if (MotorFootTrace_CommandMatchesTail(rx_data,
                                          received_length,
                                          arm_command,
                                          (uint16_t)(sizeof(arm_command) - 1U)) != 0U)
    {
        if ((s_frozen != 0U) && (s_dump_phase != MF_DUMP_COMPLETE))
        {
            response_length = snprintf(response,
                                       sizeof(response),
                                       "EHDBG_CMD_REJECTED,command=ARM,state=DUMPING\r\n"); /* 正在导出时禁止清空冻结数据。 */
        }
        else
        {
            MotorFootTrace_ResetCaptureState(); /* 先重新武装主控记录，再向上位机确认本轮会话号。 */
            MotorFootTrace_Record(MF_TRACE_CAPTURE_ARMED,
                                  MOTOR_FOOT_TRACE_CAPACITY,
                                  s_capture_session); /* 首条记录明确保存容量和人工测试轮次。 */
            response_length = snprintf(response,
                                       sizeof(response),
                                       "EHDBG_ARMED,tick=%lu,session=%u,capacity=%u\r\n",
                                       (unsigned long)HAL_GetTick(),
                                       (unsigned int)s_capture_session,
                                       (unsigned int)MOTOR_FOOT_TRACE_CAPACITY); /* 上位机收到后才清除本地上一轮显示。 */
        }
    }
    else if (MotorFootTrace_CommandMatchesTail(rx_data,
                                               received_length,
                                               dump_command,
                                               (uint16_t)(sizeof(dump_command) - 1U)) != 0U)
    {
        if (s_frozen != 0U)
        {
            response_length = snprintf(response,
                                       sizeof(response),
                                       "EHDBG_CMD_REJECTED,command=DUMP,state=FROZEN\r\n"); /* 一轮只允许冻结一次，防止重复导出交叉。 */
        }
        else
        {
            MotorFootTrace_Trigger(MF_TRIGGER_MANUAL); /* 人工点击是唯一冻结入口，触发函数立即锁住当前环形窗口。 */
            response_length = snprintf(response,
                                       sizeof(response),
                                       "EHDBG_DUMP_ACCEPTED,tick=%lu,session=%u,records=%u\r\n",
                                       (unsigned long)s_freeze_tick,
                                       (unsigned int)s_capture_session,
                                       (unsigned int)s_dump_total); /* 先确认冻结成功，300ms 后再开始完整 CSV。 */
        }
    }
    else
    {
        return 1U; /* 非 ARM/DUMP 数据只清空本次 DMA 包并静默丢弃，空闲监测期不得产生拒绝回包和串口负担。 */
    }

    if ((response_length > 0) && ((uint32_t)response_length < sizeof(response)))
    {
        (void)MotorFootTrace_SendLine(response, (uint16_t)response_length); /* 本周期只发送一份命令结果，避免与心跳或 CSV 连续占线。 */
    }
    else
    {
        ++s_format_error_count; /* 固定应答异常时只计数，禁止发送未终止或截断字符串。 */
    }
    return 1U; /* 收到数据后本周期不再发送心跳或 CSV，保持 UART2 单次阻塞边界。 */
}

/*
 * 函数功能：正常期低频发送在线帧，故障冻结后按 30ms 状态机发送完整诊断现场。
 * 输入参数：event 为调度器事件值，当前任务不使用。
 * 返回参数：无。
 */
static void MotorFootTrace_DumpTask(uint32_t event)
{
    static const char header[] =
        "EHDBG_VERSION,REC,TICK,EVENT,TRIGGER,FOOT_SEQ,PEDAL_TYPE,RUNTIME_VALID,ADC_L,ADC_R,LOW_L,MID_L,HIGH_L,LOW_R,MID_R,HIGH_R,STOP_LATCH,RELEASE_READY,ACTIVE_SOURCE,OWNER,DRIVE_TYPE,RUNFLAG,JT_L,JT_R,SPEED_WORK,CMD_SEQ,CMD_RUN,CMD_TYPE,CMD_RPM,FB_SEQ,FB_VALID,FB_ERR,FB_RPM,EXTRA0,EXTRA1\r\n"; /* 固定字段顺序与上位机解析器一致。 */
    MotorFootTraceRecord_t record; /* 每周期只复制一条记录到任务栈，环形缓冲保持冻结。 */
    char line[MOTOR_FOOT_TRACE_LINE_SIZE]; /* CSV 行局部缓存，禁止动态分配。 */
    uint32_t now_tick; /* 本周期主控毫秒时钟。 */
    int line_length; /* snprintf 返回值，负数或超长时丢弃当前行并计数。 */

    (void)event; /* 调度器事件值不参与在线帧和导出状态机。 */
    ++g_motor_foot_trace_task_run_count; /* 每次真正进入回调都计数，用于区分任务未调度与 UART2 发送失败。 */
    now_tick = HAL_GetTick(); /* 正常在线帧和故障导出共用同一份无符号回绕安全时基。 */
    if (MotorFootTrace_ProcessCommand() != 0U)
    {
        return; /* 人工命令及其应答优先于本周期心跳和 CSV，避免 RS485 半双工碰撞。 */
    }
    if ((uint32_t)(now_tick - s_last_alive_tick) >= MOTOR_FOOT_TRACE_ALIVE_PERIOD_MS)
    {
        s_last_alive_tick = now_tick; /* 发送前推进周期，失败时也不能每 30ms 连续重试干扰控制。 */
        MotorFootTrace_SendAlive(); /* 冻结前、导出中和导出后都保留在线帧，晚连接上位机仍能确认主控在线。 */
        if (s_frozen != 0U)
        {
            return; /* 故障导出期本周期已经发送在线帧时不再发送 CSV，保持每 30ms 最多一次 UART2 阻塞。 */
        }
    }
    if (s_frozen == 0U)
    {
        return; /* 未触发故障时禁止进入 CSV 导出状态机。 */
    }
    if (s_dump_phase == MF_DUMP_COMPLETE)
    {
        return; /* 故障导出完成后只保留上方固定在线帧，不再重复发送已冻结现场。 */
    }

    if ((uint32_t)(now_tick - s_freeze_tick) < MOTOR_FOOT_TRACE_DUMP_DELAY_MS)
    {
        return; /* 给原控制和停机链留出 300ms，不在故障边沿执行格式化或串口发送。 */
    }

    if (s_dump_phase == MF_DUMP_WAIT_START)
    {
        line_length = snprintf(line,
                               sizeof(line),
                               "EHDBG_START,version=1,session=%u,dump_tick=%lu,freeze_tick=%lu,trigger=%u,records=%u,overwritten=%lu\r\n",
                               (unsigned int)s_capture_session,
                               (unsigned long)now_tick,
                               (unsigned long)s_freeze_tick,
                               (unsigned int)s_trigger,
                               (unsigned int)s_dump_total,
                               (unsigned long)s_overwritten_count); /* 上位机用接收该行的绝对时间和 dump_tick 反推每条事件的绝对时刻。 */
        if ((line_length > 0) && ((uint32_t)line_length < sizeof(line)))
        {
            (void)MotorFootTrace_SendLine(line, (uint16_t)line_length); /* 不论发送成功与否都只尝试一次，失败计数写入结束行。 */
        }
        else
        {
            ++s_format_error_count; /* 起始行不应超长，异常时记录并继续后续导出。 */
        }
        s_dump_phase = MF_DUMP_SEND_HEADER; /* 下一周期发送固定表头，避免连续阻塞。 */
        return;
    }

    if (s_dump_phase == MF_DUMP_SEND_HEADER)
    {
        (void)MotorFootTrace_SendLine(header, (uint16_t)(sizeof(header) - 1U)); /* 固定表头无需 snprintf，避免额外栈开销。 */
        s_dump_phase = MF_DUMP_SEND_RECORDS; /* 表头只发送一次。 */
        return;
    }

    if (s_dump_phase == MF_DUMP_SEND_RECORDS)
    {
        if (s_dump_remaining == 0U)
        {
            s_dump_phase = MF_DUMP_SEND_END; /* 所有冻结记录已经消费，下一周期发送统计。 */
            return;
        }

        record = s_trace[s_dump_index]; /* 缓冲已冻结，任务上下文可直接复制一条完整记录。 */
        s_dump_index = (uint16_t)(s_dump_index + 1U); /* 成功复制后推进到下一条时间顺序记录。 */
        if (s_dump_index >= MOTOR_FOOT_TRACE_CAPACITY)
        {
            s_dump_index = 0U; /* 环形尾部后继续从索引 0 导出。 */
        }
        --s_dump_remaining; /* 当前记录无论格式是否成功都只处理一次，禁止死循环阻塞。 */

        line_length = snprintf(line,
                               sizeof(line),
                               "EHDBG,1,"
                               "%u,%lu,%u,%u,%u,%u,%u,%u,%u,%u,"
                               "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                               "%u,%u,%u,%lu,%u,%u,%u,%lu,%u,%u,"
                               "%u,%lu,%u,%u\r\n",
                               (unsigned int)record.record_sequence,
                               (unsigned long)record.tick_ms,
                               (unsigned int)record.event,
                               (unsigned int)record.trigger,
                               (unsigned int)record.foot_frame_sequence,
                               (unsigned int)record.pedal_type,
                               (unsigned int)record.foot_runtime_valid,
                               (unsigned int)record.adc_left,
                               (unsigned int)record.adc_right,
                               (unsigned int)record.low_left,
                               (unsigned int)record.mid_left,
                               (unsigned int)record.high_left,
                               (unsigned int)record.low_right,
                               (unsigned int)record.mid_right,
                               (unsigned int)record.high_right,
                               (unsigned int)record.foot_stop_latched,
                               (unsigned int)record.foot_release_ready,
                               (unsigned int)record.foot_active_source,
                               (unsigned int)record.control_owner,
                               (unsigned int)record.drive_type,
                               (unsigned int)record.runflag_work,
                               (unsigned int)record.jt_left_flag,
                               (unsigned int)record.jt_right_flag,
                               (unsigned long)record.speed_work,
                               (unsigned int)record.motor_command_sequence,
                               (unsigned int)record.motor_command_run,
                               (unsigned int)record.motor_command_type,
                               (unsigned long)record.motor_command_rpm,
                               (unsigned int)record.motor_feedback_sequence,
                               (unsigned int)record.motor_feedback_valid,
                               (unsigned int)record.motor_feedback_error,
                               (unsigned long)record.motor_feedback_rpm,
                               (unsigned int)record.extra0,
                               (unsigned int)record.extra1); /* 只导出整数，主控不做中文和浮点格式化。 */
        if ((line_length <= 0) || ((uint32_t)line_length >= sizeof(line)))
        {
            ++s_format_error_count; /* 当前行异常时跳过，下一周期继续后续记录。 */
            return;
        }
        (void)MotorFootTrace_SendLine(line, (uint16_t)line_length); /* 每个 30ms 周期最多发送一条记录。 */
        return;
    }

    if (s_dump_phase == MF_DUMP_SEND_END)
    {
        line_length = snprintf(line,
                               sizeof(line),
                               "EHDBG_END,session=%u,trigger=%u,records=%u,overwritten=%lu,tx_fail=%lu,format_error=%lu\r\n",
                               (unsigned int)s_capture_session,
                               (unsigned int)s_trigger,
                               (unsigned int)s_dump_total,
                               (unsigned long)s_overwritten_count,
                               (unsigned long)s_tx_fail_count,
                               (unsigned long)s_format_error_count); /* 结束统计让上位机判断本次日志是否完整。 */
        if ((line_length > 0) && ((uint32_t)line_length < sizeof(line)))
        {
            (void)MotorFootTrace_SendLine(line, (uint16_t)line_length); /* 结束标记只发送一次，发送失败仍保留 tx_fail 供断点查看。 */
        }
        else
        {
            ++s_format_error_count; /* 结束行格式异常时停止本次导出，避免任务反复占 UART2。 */
        }
        s_dump_phase = MF_DUMP_COMPLETE; /* 本次上电的第一份现场导出结束。 */
    }
}

/*
 * 函数功能：初始化诊断环形缓冲、正常在线帧和故障后 UART2 导出任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorFootTrace_Init(void)
{
    memset(s_trace, 0, sizeof(s_trace)); /* 启动前清空固定记录区，不能带入上次软复位的 RAM 内容。 */
    s_capture_session = 0U; /* 上电初始化把首次监测会话固定为 1，后续每次 ARM 继续递增。 */
    MotorFootTrace_ResetCaptureState(); /* 统一使用人工 ARM 的状态复位路径，避免上电与重测行为分叉。 */
    g_motor_foot_trace_task_create_result = 0; /* 初始化阶段先清任务创建结果，防止调试器误读启动前默认值。 */
    g_motor_foot_trace_task_start_result = 0; /* 只有创建成功后才尝试启动并更新该结果。 */
    g_motor_foot_trace_task_run_count = 0U; /* 本次上电重新统计任务实际运行次数。 */
    g_motor_foot_trace_alive_attempt_count = 0U; /* 本次上电尚未尝试在线帧。 */
    g_motor_foot_trace_alive_success_count = 0U; /* 本次上电尚无在线帧发送成功。 */
    g_motor_foot_trace_alive_last_status = (uint32_t)HAL_ERROR; /* 未发送前使用明确的非成功状态。 */
    g_motor_foot_trace_alive_last_tick = 0U; /* 未发送前没有有效尝试时刻。 */
    s_last_alive_tick = HAL_GetTick() - MOTOR_FOOT_TRACE_ALIVE_PERIOD_MS; /* 预置为已到期，调度器启动后的第一个 30ms 周期立即发送在线帧。 */
    MotorFootTrace_Record(MF_TRACE_BOOT,
                          MOTOR_FOOT_TRACE_CAPACITY,
                          (uint16_t)(MOTOR_FOOT_TRACE_UART2_BAUD / 100U)); /* 首条记录extra1按100波特为单位保存，115200写为1152避免16位截断。 */
    g_motor_foot_trace_task_create_result = Kernel_TaskCreate(&s_trace_dump_task, MotorFootTrace_DumpTask); /* 保存真实创建结果，失败时不能伪装成已启动。 */
    if (g_motor_foot_trace_task_create_result != 0)
    {
        g_motor_foot_trace_task_start_result = Kernel_TaskStart(&s_trace_dump_task, KERNEL_TASK_ALWAYS, MOTOR_FOOT_TRACE_DUMP_PERIOD_MS); /* 创建成功后启动 30ms 周期，在线帧仍受 2 秒周期限制。 */
    }
}

/*
 * 函数功能：从各模块公开快照组装一条完整诊断记录并写入环形缓冲。
 * 输入参数：event 为事件类型；extra0、extra1 为事件附加值。
 * 返回参数：无。
 */
void MotorFootTrace_Record(MotorFootTraceEvent_t event, uint16_t extra0, uint16_t extra1)
{
    MotorFootTraceRecord_t record; /* 所有字段先在局部变量中组装，临界区只负责最终复制。 */
    FootTraceSnapshot_t foot_snapshot; /* 脚踏静态变量通过只读快照接口取得。 */
    MotorDriveCommandSnapshot_t command_snapshot; /* 最终 UART1 命令使用现有一致性快照。 */
    MotorUartFeedbackSnapshot_t feedback_snapshot; /* 驱动反馈使用现有一致性快照。 */

    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* 未初始化或第一份异常已经冻结时不再采集。 */
    }

    memset(&record, 0, sizeof(record)); /* 无效快照字段保持 0，避免沿用调用栈旧值。 */
    memset(&foot_snapshot, 0, sizeof(foot_snapshot));
    memset(&command_snapshot, 0, sizeof(command_snapshot));
    memset(&feedback_snapshot, 0, sizeof(feedback_snapshot));

    record.tick_ms = HAL_GetTick(); /* 所有模块快照开始前记录本条事件的主控时间。 */
    record.event = (uint8_t)event; /* 保存事件编号，中文翻译由 PC 完成。 */
    record.trigger = s_trigger; /* 普通记录为 0，冻结故障记录带第一份触发原因。 */
    record.control_owner = ControlArbitration_GetCurrentOwner(); /* 读取当前唯一仲裁 owner。 */
    record.drive_type = WorkMessage.drivetype_work; /* 保存业务控制方式，和 owner 分开观察。 */
    record.runflag_work = (WorkMessage.runflag_work != false) ? 1U : 0U; /* 保存最终业务运行请求。 */
    record.jt_left_flag = (ControlSignalMessage.jtL_control_flag != false) ? 1U : 0U; /* 保存左脚踏来源标志。 */
    record.jt_right_flag = (ControlSignalMessage.jtR_control_flag != false) ? 1U : 0U; /* 保存右脚踏来源标志。 */
    record.speed_work = WorkMessage.speed_work; /* 保存脚踏实时速度或当前业务实际速度。 */
    record.extra0 = extra0; /* 附加值含义由事件号决定。 */
    record.extra1 = extra1; /* 第二附加值同样不参与控制。 */

    if (Foot_CopyTraceSnapshot(&foot_snapshot) != 0U)
    {
        record.foot_frame_sequence = foot_snapshot.frame_sequence; /* 只对 CRC 正确实时帧递增。 */
        record.pedal_type = foot_snapshot.pedal_type; /* 保存真实帧协议类型。 */
        record.foot_runtime_valid = foot_snapshot.runtime_frame_valid; /* 保存 250ms 实时帧门禁。 */
        record.adc_left = foot_snapshot.adc_left; /* 保存本次脚踏左/单路 ADC。 */
        record.adc_right = foot_snapshot.adc_right; /* 保存双脚踏右 ADC。 */
        record.low_left = foot_snapshot.low_left; /* 保存左低点。 */
        record.mid_left = foot_snapshot.mid_left; /* 保存左中点。 */
        record.high_left = foot_snapshot.high_left; /* 保存左高点。 */
        record.low_right = foot_snapshot.low_right; /* 保存右低点。 */
        record.mid_right = foot_snapshot.mid_right; /* 保存右中点。 */
        record.high_right = foot_snapshot.high_right; /* 保存右高点。 */
        record.foot_stop_latched = foot_snapshot.stop_latched; /* 保存独立停机锁存。 */
        record.foot_release_ready = foot_snapshot.release_ready; /* 保存先松后踩门禁的释放结果。 */
        record.foot_active_source = foot_snapshot.active_source; /* 保存真正获得运行授权的踏板侧。 */
    }

    if (MotorDrive_CopyCommandSnapshot(&command_snapshot) != 0U)
    {
        record.motor_command_sequence = command_snapshot.sequence; /* 使用实际变化命令序号，不重新推测。 */
        record.motor_command_run = command_snapshot.run_state; /* 使用最终 UART1 RUN/STOP。 */
        record.motor_command_type = command_snapshot.motor_type; /* 使用最终物理电机类型。 */
        record.motor_command_rpm = command_snapshot.command_speed_rpm; /* 使用驱动帧速度字段换算后的 rpm。 */
    }

    if (MotorUart_CopyFeedbackSnapshot(&feedback_snapshot) != 0U)
    {
        record.motor_feedback_sequence = feedback_snapshot.sequence; /* 保存 CRC 正确反馈序号。 */
        record.motor_feedback_valid = feedback_snapshot.valid; /* 保存驱动反馈存在标志。 */
        record.motor_feedback_error = feedback_snapshot.raw_error; /* 保存驱动原始 Err。 */
        record.motor_feedback_rpm = feedback_snapshot.speed_rpm; /* 保存驱动反馈实际 rpm。 */
    }

    MotorFootTrace_Push(&record); /* 只有最后结构体复制进入短临界区。 */
}

/*
 * 函数功能：对真实 UART4 实时帧做 50ms/ADC 边沿节流后记录。
 * 输入参数：pedal_type 为脚踏类型；adc_left、adc_right 为本帧实时值。
 * 返回参数：无。
 */
void MotorFootTrace_OnValidFootFrame(uint8_t pedal_type, uint16_t adc_left, uint16_t adc_right)
{
    uint32_t now_tick = HAL_GetTick(); /* 本次 CRC 正确实时帧到达时刻。 */
    uint8_t should_record = 0U; /* 默认只更新帧序号，不让 10ms 抖动填满环形缓冲。 */

    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* ARM 状态重置或人工冻结期间不得更新节流基准，下一轮第一帧必须完整记录。 */
    }

    if ((s_frame_log_valid == 0U) || (s_last_frame_pedal_type != pedal_type))
    {
        should_record = 1U; /* 第一帧或脚踏类型变化必须记录。 */
    }
    else if ((MotorFootTrace_AbsDiffU16(adc_left, s_last_frame_adc_left) >= MOTOR_FOOT_TRACE_ADC_CHANGE_THRESHOLD) ||
             (MotorFootTrace_AbsDiffU16(adc_right, s_last_frame_adc_right) >= MOTOR_FOOT_TRACE_ADC_CHANGE_THRESHOLD))
    {
        should_record = 1U; /* 踩下或释放的明显 ADC 边沿立即记录。 */
    }
    else if ((uint32_t)(now_tick - s_last_frame_log_tick) >= MOTOR_FOOT_TRACE_FRAME_LOG_PERIOD_MS)
    {
        should_record = 1U; /* ADC 稳定时每 50ms 留一份，可判断帧序号是否持续前进。 */
    }

    if (should_record == 0U)
    {
        return; /* 本帧仍已更新脚踏快照序号，只是不额外写环形事件。 */
    }
    s_frame_log_valid = 1U; /* 建立后续节流比较基准。 */
    s_last_frame_pedal_type = pedal_type; /* 保存当前协议类型。 */
    s_last_frame_adc_left = adc_left; /* 保存左 ADC 基准。 */
    s_last_frame_adc_right = adc_right; /* 保存右 ADC 基准。 */
    s_last_frame_log_tick = now_tick; /* 重新开始 50ms 稳态采样间隔。 */
    MotorFootTrace_Record(MF_TRACE_FOOT_FRAME_VALID, adc_left, adc_right); /* 附加值重复保存本帧 ADC，便于脱离快照快速核对。 */
}

/*
 * 函数功能：记录脚踏进入电机释放区并开启释放保护窗口。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorFootTrace_OnFootRelease(void)
{
    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* ARM 重置或人工冻结期间不建立新的释放窗口，保证冻结现场保持不变。 */
    }
    if ((WorkMessage.drivetype_work == JTWORK) || ControlArbitration_IsOwner(CONTROL_OWNER_FOOT))
    {
        s_release_pending = 1U; /* 只有脚踏控制上下文才建立释放后 RUN 检查，不能误判屏幕/手控正常启动。 */
        s_release_tick = HAL_GetTick(); /* 保存释放边沿时刻。 */
    }
    MotorFootTrace_Record(MF_TRACE_FOOT_RELEASE_BOUNDARY,
                          WorkMessage.drivetype_work,
                          ControlArbitration_GetCurrentOwner()); /* 每次释放都保存控制方式和 owner。 */
}

/*
 * 函数功能：新踩踏通过先松后踩门禁时结束旧释放窗口。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorFootTrace_OnFootRunAuthorized(void)
{
    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* 未处于采集状态时不消费释放窗口，重新 ARM 后从完整动作周期开始观察。 */
    }
    s_release_pending = 0U; /* 新的合法启动沿消费了释放条件，后续 RUN 不属于旧释放后的异常续转。 */
}

/*
 * 函数功能：对脚踏任务被其它控制源阻塞的周期按状态变化或 250ms 做去重记录。
 * 输入参数：owner 为当前控制权；runflag 为当前业务运行位。
 * 返回参数：无。
 */
void MotorFootTrace_OnFootOwnerBlocked(uint8_t owner, uint8_t runflag)
{
    uint32_t now_tick = HAL_GetTick(); /* 保存本次脚踏任务准备提前返回的时刻。 */

    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* 人工冻结后不得继续推进 owner 去重状态，防止下一轮首份状态被遗漏。 */
    }

    if ((s_owner_block_log_valid != 0U) &&
        (s_last_block_owner == owner) &&
        (s_last_block_runflag == runflag) &&
        ((uint32_t)(now_tick - s_last_owner_block_log_tick) < MOTOR_FOOT_TRACE_OWNER_BLOCK_LOG_PERIOD_MS))
    {
        return; /* owner 和运行位均未变化时限制为每250ms一条，保留更长故障前窗口。 */
    }

    s_owner_block_log_valid = 1U; /* 建立后续阻塞去重基准。 */
    s_last_block_owner = owner; /* owner 变化时下一周期立即记录。 */
    s_last_block_runflag = runflag; /* 运行位变化时下一周期立即记录。 */
    s_last_owner_block_log_tick = now_tick; /* 重新开始250ms稳态记录间隔。 */
    MotorFootTrace_Record(MF_TRACE_FOOT_TASK_BLOCKED_BY_OWNER, owner, runflag); /* 附加字段与完整快照共同保存阻塞现场。 */
}

/*
 * 函数功能：记录最终 UART1 命令并检查释放后误发 RUN。
 * 输入参数：run_state、command_sequence、command_rpm、motor_type 均来自最终命令快照。
 * 返回参数：无。
 */
void MotorFootTrace_OnMotorCommand(uint8_t run_state,
                                   uint16_t command_sequence,
                                   uint32_t command_rpm,
                                   uint8_t motor_type)
{
    uint32_t now_tick = HAL_GetTick(); /* 本份最终命令实际准备送入 UART1 的时刻。 */
    uint8_t should_record = 0U; /* 相同 50ms 保活命令只低频记录，命令变化立即记录。 */

    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* ARM 重置或人工冻结期间不改变最近命令基准，冻结数据保持按钮时刻的一致状态。 */
    }

    if ((s_last_command_valid == 0U) || (s_last_command_run != run_state) ||
        (s_last_logged_command_sequence != command_sequence))
    {
        should_record = 1U; /* 首份命令、RUN/STOP 边沿或命令内容变化必须记录。 */
    }
    else if ((uint32_t)(now_tick - s_last_command_log_tick) >= MOTOR_FOOT_TRACE_COMMAND_LOG_PERIOD_MS)
    {
        should_record = 1U; /* 命令长期不变时每 250ms 留一份保活证据。 */
    }

    if (run_state == 0U)
    {
        if ((s_last_command_valid == 0U) || (s_last_command_run != 0U))
        {
            s_last_stop_command_tick = now_tick; /* 只在 RUN 转 STOP 或首份 STOP 时开始 300ms 反馈宽限。 */
        }
        s_last_command_run = 0U; /* 发布最近最终命令为 STOP。 */
    }
    else
    {
        s_last_command_run = 1U; /* 发布最近最终命令为 RUN。 */
    }
    s_last_command_valid = 1U; /* 从本次开始允许反馈侧判断 STOP 后仍转。 */

    if (should_record != 0U)
    {
        s_last_logged_command_sequence = command_sequence; /* 更新命令日志去重基准。 */
        s_last_command_log_tick = now_tick; /* 更新 250ms 保活起点。 */
        MotorFootTrace_Record((run_state != 0U) ? MF_TRACE_MOTOR_TX_RUN : MF_TRACE_MOTOR_TX_STOP,
                              command_sequence,
                              motor_type); /* 记录最终命令字段，速度由一致性快照保存。 */
    }

    if ((run_state != 0U) &&
        (s_release_pending != 0U) &&
        (WorkMessage.drivetype_work == JTWORK) &&
        ((uint32_t)(now_tick - s_release_tick) < MOTOR_FOOT_TRACE_RELEASE_GUARD_MS))
    {
        MotorFootTrace_Trigger(MF_TRIGGER_RELEASE_BUT_RUN_TX); /* 人工模式只记录一次疑似误发 RUN；自动模式才冻结现场。 */
    }

    (void)command_rpm; /* 最终速度已经由 MotorDriveCommandSnapshot 写入记录，保留参数强调调用者必须传真实值。 */
}

/*
 * 函数功能：记录驱动反馈并检查 STOP 宽限后仍明显转动。
 * 输入参数：反馈有效、序号、rpm、原始 Err 和 0.01A 电流均来自同一 CRC 正确回包。
 * 返回参数：无。
 */
void MotorFootTrace_OnDriverFeedback(uint8_t feedback_valid,
                                     uint16_t feedback_sequence,
                                     uint32_t speed_rpm,
                                     uint8_t raw_error,
                                     uint16_t current_x100)
{
    uint32_t now_tick = HAL_GetTick(); /* 本份驱动反馈完成 CRC 校验的时刻。 */

    if ((s_initialized == 0U) || (s_frozen != 0U))
    {
        return; /* 人工冻结后不再记录驱动回包，也不再产生新的疑似条件。 */
    }

    MotorFootTrace_Record(MF_TRACE_DRIVER_FEEDBACK, raw_error, current_x100); /* extra1 明确保存本帧电流，便于判断失控时负载。 */
    if ((feedback_valid != 0U) &&
        (s_last_command_valid != 0U) &&
        (s_last_command_run == 0U) &&
        ((uint32_t)(now_tick - s_last_stop_command_tick) >= MOTOR_FOOT_TRACE_STOP_FEEDBACK_GRACE_MS) &&
        (speed_rpm > MOTOR_FOOT_TRACE_DRIVER_MOVING_RPM))
    {
        MotorFootTrace_Trigger(MF_TRIGGER_STOP_TX_BUT_DRIVER_MOVING); /* 人工模式只记录一次疑似续转；自动模式才冻结，均不改控制状态。 */
    }

    (void)feedback_sequence; /* 序号由 MotorUartFeedbackSnapshot 写入记录，参数用于确保调用点来自真实新回包。 */
}

/*
 * 函数功能：人工触发时冻结现场；自动疑似条件在人工模式下只记录一次标记，不主动上传。
 * 输入参数：trigger 为自动或人工触发原因。
 * 返回参数：无。
 */
void MotorFootTrace_Trigger(MotorFootTraceTrigger_t trigger)
{
    uint32_t primask; /* 第一触发认领、冻结索引和数量都需要短临界区。 */
#if (MOTOR_FOOT_TRACE_MANUAL_FREEZE_ONLY == 1U)
    uint32_t observed_bit; /* 自动疑似条件按 trigger 编号映射到去重位，只记录一次但不冻结。 */
#endif

    if ((s_initialized == 0U) || (trigger == MF_TRIGGER_NONE))
    {
        return; /* 无效触发或未初始化时不处理。 */
    }

#if (MOTOR_FOOT_TRACE_MANUAL_FREEZE_ONLY == 1U)
    if (trigger != MF_TRIGGER_MANUAL)
    {
        observed_bit = (uint32_t)1UL << (uint32_t)trigger; /* 当前自动触发编号只有1~4，移位始终落在32位掩码范围内。 */
        primask = __get_PRIMASK(); /* 多任务可能同时观察到同一疑似条件，原子认领首次记录。 */
        __disable_irq();
        if ((s_frozen != 0U) || ((s_observed_trigger_mask & observed_bit) != 0U))
        {
            if (primask == 0U)
            {
                __enable_irq(); /* 已冻结或本类条件已记录时恢复调用前中断状态。 */
            }
            return; /* 人工冻结前每类疑似条件只占一条记录，不能持续淹没故障前窗口。 */
        }
        s_observed_trigger_mask |= observed_bit; /* 发布本类疑似条件已记录，后续相同条件直接忽略。 */
        if (primask == 0U)
        {
            __enable_irq(); /* 短临界区结束后再组装完整诊断快照。 */
        }
        MotorFootTrace_Record(MotorFootTrace_EventFromTrigger(trigger),
                              (uint16_t)trigger,
                              ControlArbitration_GetCurrentOwner()); /* 自动条件只留下疑似标记，冻结时刻完全由测试人员按钮决定。 */
        return;
    }
#endif

    primask = __get_PRIMASK(); /* 多任务可能在相邻周期同时发现异常，先原子认领第一触发。 */
    __disable_irq();
    if ((s_frozen != 0U) || (s_trigger != (uint8_t)MF_TRIGGER_NONE))
    {
        if (primask == 0U)
        {
            __enable_irq(); /* 已有触发时保持原中断状态并退出。 */
        }
        return; /* 只保留首个根因，后续并发触发不能覆盖原因。 */
    }
    s_trigger = (uint8_t)trigger; /* 原子发布首个原因，使故障记录直接带 trigger。 */
    if (primask == 0U)
    {
        __enable_irq(); /* 记录完整快照前恢复中断，避免长临界区影响控制任务。 */
    }

    MotorFootTrace_Record(MotorFootTrace_EventFromTrigger(trigger),
                          (uint16_t)trigger,
                          ControlArbitration_GetCurrentOwner()); /* 冻结前追加一条专用故障记录。 */

    primask = __get_PRIMASK(); /* 同时冻结写入位置、数量和导出起点。 */
    __disable_irq();
    s_frozen = 1U; /* 从本指令开始所有埋点拒绝写入。 */
    s_freeze_tick = HAL_GetTick(); /* 保存人工按钮触发的主控时间。 */
    s_dump_total = s_count; /* 冻结时记录总数，导出统计不得随运行变化。 */
    s_dump_remaining = s_count; /* 每发送一条递减。 */
    s_dump_index = (s_count < MOTOR_FOOT_TRACE_CAPACITY) ? 0U : s_write_index; /* 未写满从 0 开始，写满从最旧记录开始。 */
    s_dump_phase = MF_DUMP_WAIT_START; /* 300ms 后先发送时间同步行。 */
    if (primask == 0U)
    {
        __enable_irq(); /* 恢复调用前中断状态。 */
    }
}

/*
 * 函数功能：人工冻结当前环形缓冲，供断点或后续诊断命令调用。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorFootTrace_ForceManualFreeze(void)
{
    MotorFootTrace_Trigger(MF_TRIGGER_MANUAL); /* 人工冻结与自动触发共用同一安全导出状态机。 */
}

#else

/* 正式固件关闭开关后保留空实现，所有既有业务调用点无需使用条件编译。 */
void MotorFootTrace_Init(void) { }
void MotorFootTrace_Record(MotorFootTraceEvent_t event, uint16_t extra0, uint16_t extra1)
{
    (void)event;
    (void)extra0;
    (void)extra1;
}
void MotorFootTrace_OnValidFootFrame(uint8_t pedal_type, uint16_t adc_left, uint16_t adc_right)
{
    (void)pedal_type;
    (void)adc_left;
    (void)adc_right;
}
void MotorFootTrace_OnFootRelease(void) { }
void MotorFootTrace_OnFootRunAuthorized(void) { }
void MotorFootTrace_OnFootOwnerBlocked(uint8_t owner, uint8_t runflag)
{
    (void)owner;
    (void)runflag;
}
void MotorFootTrace_OnMotorCommand(uint8_t run_state,
                                   uint16_t command_sequence,
                                   uint32_t command_rpm,
                                   uint8_t motor_type)
{
    (void)run_state;
    (void)command_sequence;
    (void)command_rpm;
    (void)motor_type;
}
void MotorFootTrace_OnDriverFeedback(uint8_t feedback_valid,
                                     uint16_t feedback_sequence,
                                     uint32_t speed_rpm,
                                     uint8_t raw_error,
                                     uint16_t current_x100)
{
    (void)feedback_valid;
    (void)feedback_sequence;
    (void)speed_rpm;
    (void)raw_error;
    (void)current_x100;
}
void MotorFootTrace_Trigger(MotorFootTraceTrigger_t trigger) { (void)trigger; }
void MotorFootTrace_ForceManualFreeze(void) { }

#endif /* MOTOR_FOOT_TRACE_ENABLE */
