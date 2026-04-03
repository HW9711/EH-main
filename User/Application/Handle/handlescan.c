// handlescan.c

#include "stm32f4xx_hal.h"
#include "handlescan.h"
#include "board.h"
#include "data.h"
#include "common.h"
#include "OneWireI.h"
#include "OneWireII.h"

#include <stdint.h>
#include <stdio.h>
#include "screen.h"
#include "app_task.h"
#include "datahand.h"
#include "at24cs32.h"
#include "at24cs32_crc_verify.h"

task_t HANDLESCANTaskHandle;

extern UART_HandleTypeDef huart10;

/*
 * A 、B通道短接检测脚定义。
 * 当前板级连接以这两个宏为准：
 * 1. A 通道短接检测脚绑定到 `HANDLESCAN_A_SHORT_PIN`；
 * 2. B 通道短接检测脚绑定到 `HANDLESCAN_B_SHORT_PIN`；
 * 3. 当前现场验证使用“低电平表示插入成立，高电平表示拔出候选”。
 * 后续所有插拔状态机都基于这个输入脚做边沿和去抖判断，因此如果硬件接线变更，只需要修改这里的宏。
 */
#define HANDLESCAN_A_SHORT_GPIO               GPIOD
#define HANDLESCAN_A_SHORT_PIN                GPIO_PIN_1
#define HANDLESCAN_B_SHORT_GPIO               GPIOD
#define HANDLESCAN_B_SHORT_PIN                GPIO_PIN_0

/*
 * 手柄扫描任务的调度周期和状态机时间参数。
 * 说明：
 * 1. 任务本身按 10ms 周期运行；
 * 2. 插入和拔出去抖都使用 50ms；
 * 3. 插入稳定后额外等待 100ms 再做认证，给接口和 EEPROM 留出稳定时间。
 */
#define HANDLESCAN_TASK_PERIOD_MS             10U
#define HANDLESCAN_INSERT_DEBOUNCE_MS         50U
#define HANDLESCAN_REMOVE_DEBOUNCE_MS         50U
#define HANDLESCAN_VERIFY_START_DELAY_MS      100U

#define HANDLESCAN_INSERT_DEBOUNCE_TICKS      (HANDLESCAN_INSERT_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_REMOVE_DEBOUNCE_TICKS      (HANDLESCAN_REMOVE_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_START_DELAY_TICKS   (HANDLESCAN_VERIFY_START_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)

/*
 * EEPROM 中手柄信息区定义。
 * 第二页起始地址为 0x0020，因此读取这个地址开始的 16 字节即可拿到当前手柄的业务信息。
 * 其中前两个字节用于标识手柄大类和子类型。
 */
#define HANDLESCAN_INFO_ADDR                  0x0020U
#define HANDLESCAN_INFO_SIZE                  16U
#define HANDLESCAN_MODEL_MAJOR_OFFSET         0U
#define HANDLESCAN_MODEL_MINOR_OFFSET         1U

/*
 * A 通道相关报警码定义。
 * 1. 13：运行中插拔报警，沿用旧逻辑；
 * 2. 0x22：认证阶段失败报警，包含页读取失败、页和校验失败、SN 读取失败以及认证码不匹配；
 * 3. 0x23：认证通过后，信息区读取失败或手柄类型无法识别。
 */
#define HANDLESCAN_ALARM_RUNNING_PLUG         13U
#define HANDLESCAN_ALARM_A_PAGE_FAIL          0x22U
#define HANDLESCAN_ALARM_A_DATA_FAIL          0x23U

/*
 * 串口调试步骤码定义。
 * `HANDLESCAN_TRACE_ENABLE` 是当前 handlescan 驱动的总报文开关：
 * 1. 设为 `1U` 时，输出 `HS`、`HSDBG`、`HSNAME` 全部调试报文；
 * 2. 设为 `0U` 时，三个报文函数都会被静默处理，业务状态机保持不变；
 * 3. 这样可以在调试版和正式版之间通过改一个宏快速切换，而不需要修改流程代码。
 * 这些步骤码用于输出 HS、HSDBG、HSNAME 报文，便于现场快速判断当前流程卡在什么阶段。
 */
#define HANDLESCAN_TRACE_ENABLE              1U

#define HANDLESCAN_DBG_STEP_INSERT_PASS       0x02U
#define HANDLESCAN_DBG_STEP_REMOVE_PASS       0x03U
#define HANDLESCAN_DBG_STEP_VERIFY_STATUS     0x05U
#define HANDLESCAN_DBG_STEP_INFO_FAIL         0x06U
#define HANDLESCAN_DBG_STEP_MODEL_INVALID     0x07U
#define HANDLESCAN_DBG_STEP_ONLINE            0x08U
#define HANDLESCAN_DBG_STEP_OFFLINE           0x09U
#define HANDLESCAN_DBG_STEP_ALARM_SET         0x0AU
#define HANDLESCAN_DBG_STEP_I2C_DETAIL        0x0BU

/*
 * 手柄扫描状态机阶段定义。
 * A/B 两个通道都沿用这套阶段枚举，但各自维护独立的运行时变量，
 * 这样可以保证 A 通道保持原有函数结构，同时 B 通道也能直接在自己的函数里实现相同流程。
 */
typedef enum
{
    HANDLESCAN_STAGE_IDLE = 0,
    HANDLESCAN_STAGE_DEBOUNCE_IN,
    HANDLESCAN_STAGE_WAIT_VERIFY,
    HANDLESCAN_STAGE_VERIFY,
    HANDLESCAN_STAGE_READ_INFO,
    HANDLESCAN_STAGE_ONLINE,
    HANDLESCAN_STAGE_DEBOUNCE_OUT,
    HANDLESCAN_STAGE_VERIFY_FAIL
} HandlescanStage;

/*
 * 插入和拔出去抖计数器。
 * `in_debounce_ticks` 用于统计插入稳定时间，`out_debounce_ticks` 用于统计拔出稳定时间。
 */
typedef struct
{
    uint8_t in_debounce_ticks;
    uint8_t out_debounce_ticks;
} HandlescanDebounce;

/*
 * EEPROM 手柄类型映射配置表项。
 * 说明：
 * 1. `first_byte / second_byte` 对应 EEPROM 第二页前两个字节；
 * 2. `mapped_handle_type` 是系统内部使用的 `Handle_Type_xx`；
 * 3. `handle_name` 用于串口打印，便于现场直接看到手柄名称；
 * 4. 后续增加新型号时，只需要继续往配置表里追加一项即可。
 */
typedef struct
{
    uint8_t first_byte;
    uint8_t second_byte;
    uint8_t mapped_handle_type;
    const char *handle_name;
} HandlescanHandleTypeConfig;

/*
 * 当前已支持的手柄类型映射表。
 * 如果后续新增手柄，只需追加新的 `{主类型, 子类型, Handle_Type_xx, "名称"}` 表项即可。
 */
static const HandlescanHandleTypeConfig s_handle_type_config_table[] =
{
    {0x6B, 0x01, TMBB_ONLINE, "TMBB"},
    {0x6B, 0x02, TMBA_ONLINE,  "TMBA"},
    {0x6B, 0x03, EMBA_ONLINE, "EMBA"},
    {0x6B, 0x04, EMBB_ONLINE,  "EMBB"},
    {0x6B, 0x05, PXBA_ONLINE,  "PXBA"},
    {0x6B, 0x06, PXBB_ONLINE,  "PXBB"},
    {0x6B, 0x07, MX_YIM_ONLINE,  "MXYTM"},
    {0x6B, 0x08, MX_YIP_ONLINE,  "MXYTP"},
    {0x6B, 0x09, PX_YIM_ONLINE,  "PXYTM"},
    {0x6B, 0x0A, PX_YIP_ONLINE,  "PXYTP"},
	{0x6B, 0x0B, JMB_ONLINE,  "JMB"},
	{0x6B, 0x0C, MX_YIM16_ONLINE,  "MXYTM16"}
};

/* A 通道运行时状态变量。 */
static HandlescanStage s_a_stage = HANDLESCAN_STAGE_IDLE;
static HandlescanDebounce s_handleA_debounce = {0U, 0U};
static uint8_t s_a_verify_start_wait_ticks = 0U;
static uint8_t s_a_last_alarm = 0U;
static uint8_t s_a_info_buf[HANDLESCAN_INFO_SIZE] = {0U};

/* B 通道运行时状态变量。 */
static HandlescanStage s_b_stage = HANDLESCAN_STAGE_IDLE;
static HandlescanDebounce s_handleB_debounce = {0U, 0U};
static uint8_t s_b_verify_start_wait_ticks = 0U;
static uint8_t s_b_last_alarm = 0U;
static uint8_t s_b_info_buf[HANDLESCAN_INFO_SIZE] = {0U};

/*
 * 输出基础 HS 调试报文。
 * 报文格式固定为：`HS,CH=xx,ST=xx,VAL=xx`
 * 其中：
 * 1. `CH` 表示通道号；
 * 2. `ST` 表示当前阶段或事件码；
 * 3. `VAL` 表示该阶段对应的数值信息。
 */
static void Handlescan_DebugTrace(uint8_t channel, uint8_t step, uint8_t value)
{
#if (HANDLESCAN_TRACE_ENABLE == 1U)
    char tx_buf[48];
    int text_len;

    text_len = snprintf(tx_buf,
                        sizeof(tx_buf),
                        "HS,CH=%02u,ST=%02X,VAL=%02X\r\n",
                        (unsigned int)channel,
                        (unsigned int)step,
                        (unsigned int)value);
    if (text_len <= 0)
    {
        return;
    }

    if ((size_t)text_len > (sizeof(tx_buf) - 1U))
    {
        text_len = (int)(sizeof(tx_buf) - 1U);
    }

    HAL_UART_Transmit(&huart10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
    (void)step;
    (void)value;
#endif
}

/*
 * 输出最近一次 EEPROM/I2C 访问的底层细节。
 * 当认证失败或信息区读取失败时，上层可以立刻调用这个函数，把底层操作类型、
 * 器件地址、字地址、长度、HAL 返回值和错误码全部打印出来，便于现场快速定位。
 */
static void Handlescan_DebugTraceI2cDetail(uint8_t channel)
{
#if (HANDLESCAN_TRACE_ENABLE == 1U)
    AT24CS32_DebugInfo debug_info;
    char tx_buf[128];
    int text_len;

    AT24CS32_GetLastDebugInfo(&debug_info);

    text_len = snprintf(tx_buf,
                        sizeof(tx_buf),
                        "HSDBG,CH=%02u,ST=%02X,OP=%02X,DA=%04X,MA=%04X,LN=%04X,HS=%02X,ER=%08lX,SB=%08lX,SA=%08lX\r\n",
                        (unsigned int)channel,
                        (unsigned int)HANDLESCAN_DBG_STEP_I2C_DETAIL,
                        (unsigned int)debug_info.op_type,
                        (unsigned int)debug_info.dev_addr,
                        (unsigned int)debug_info.mem_addr,
                        (unsigned int)debug_info.data_len,
                        (unsigned int)debug_info.hal_status,
                        (unsigned long)debug_info.hal_error,
                        (unsigned long)debug_info.i2c_state_before,
                        (unsigned long)debug_info.i2c_state_after);
    if (text_len <= 0)
    {
        return;
    }

    if ((size_t)text_len > (sizeof(tx_buf) - 1U))
    {
        text_len = (int)(sizeof(tx_buf) - 1U);
    }

    HAL_UART_Transmit(&huart10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
#endif
}

/*
 * 根据 EEPROM 第二页前两个字节查找手柄配置。
 * 找到时返回对应表项，找不到时返回 `NULL`，由调用方按未知型号处理。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindHandleTypeConfig(uint8_t first_byte, uint8_t second_byte)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof(s_handle_type_config_table) / sizeof(s_handle_type_config_table[0])); ++index)
    {
        if ((s_handle_type_config_table[index].first_byte == first_byte) &&
            (s_handle_type_config_table[index].second_byte == second_byte))
        {
            return &s_handle_type_config_table[index];
        }
    }

    return NULL;
}

/*
 * 输出带名称的手柄识别报文。
 * 成功识别手柄类型后，额外输出一条 `HSNAME` 报文，便于现场直接看到 PXBB、JMB、PXBA 等文字名称。
 */
static void Handlescan_DebugTraceHandleName(uint8_t channel, uint8_t first_byte, uint8_t second_byte, const char *handle_name)
{
#if (HANDLESCAN_TRACE_ENABLE == 1U)
    char tx_buf[80];
    int text_len;

    if (handle_name == NULL)
    {
        return;
    }

    text_len = snprintf(tx_buf,
                        sizeof(tx_buf),
                        "HSNAME,CH=%02u,ID0=%02X,ID1=%02X,TYPE=%s\r\n",
                        (unsigned int)channel,
                        (unsigned int)first_byte,
                        (unsigned int)second_byte,
                        handle_name);
    if (text_len <= 0)
    {
        return;
    }

    if ((size_t)text_len > (sizeof(tx_buf) - 1U))
    {
        text_len = (int)(sizeof(tx_buf) - 1U);
    }

    HAL_UART_Transmit(&huart10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
    (void)first_byte;
    (void)second_byte;
    (void)handle_name;
#endif
}

/*
 * 把 EEPROM 认证返回码映射为系统报警码。
 * 当前策略如下：
 * 1. 认证阶段的失败统一映射为 `0x22`，包括页读取失败、页和校验失败、SN 读取失败以及 CRC 不匹配；
 * 2. 只有在认证已经通过后，信息区读取失败或手柄类型无法识别时，才映射为 `0x23`。
 */
static uint8_t Handlescan_MapVerifyStatusToAlarm(AT24CS32_CRC_Status verify_status)
{
    switch (verify_status)
    {
        case AT24CS32_CRC_STATUS_OK:
            return 0U;

        case AT24CS32_CRC_STATUS_DATA_READ_FAILED:
        case AT24CS32_CRC_STATUS_BAD_PARAM:
        case AT24CS32_CRC_STATUS_PAGE1_READ_FAILED:
        case AT24CS32_CRC_STATUS_PAGE1_CHECKSUM_FAILED:
        case AT24CS32_CRC_STATUS_SN_READ_FAILED:
        case AT24CS32_CRC_STATUS_CRC_MISMATCH:
        default:
            return HANDLESCAN_ALARM_A_PAGE_FAIL;
    }
}

/*
 * 处理运行中插拔报警。
 * 当电机已经处于启动状态时，如果此时检测到手柄掉线或插拔变化，按照旧逻辑需要：
 * 1. 立刻置运行中插拔报警；
 * 2. 打开蜂鸣报警标志；
 * 3. 如果板级仍保留 K1 兼容宏，则同步执行关闭动作；
 * 4. 返回 `1` 告诉调用方“本次已经触发报警”。
 */
static uint8_t Handlescan_HandleRunningPlugAlarm(uint8_t channel)
{
    if (Workvalue_s.MOTORWorking_flag == start_flag)
    {
        Workvalue_s.Alarm_value = HANDLESCAN_ALARM_RUNNING_PLUG;
        Workvalue_s.beep_Alarm_flag = 1U;
        K1_OFF();
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
        return 1U;
    }

    return 0U;
}

/*
 * A 通道 SSC 扫描函数。
 * 设计目标：
 * 1. 仅在插入稳定后做一次认证和一次信息区读取；
 * 2. 认证成功后保持在线，不再重复访问 EEPROM；
 * 3. 认证失败后保持安静，但仍持续监视拔出边沿；
 * 4. 拔出稳定后只输出一次离线报文，并清理 A 通道在线状态。
 */
void HandlescanA_Fun_SSC(void)
{
    uint8_t is_inserted;                                      /* 当前采样到的 A 通道插入状态，1 表示短接成立，0 表示短接断开。 */
    AT24CS32_CRC_Result verify_result;                       /* 保存 EEPROM 认证函数输出的中间结果，便于底层后续扩展。 */
    AT24CS32_CRC_Status verify_status;                       /* 保存当前这次 EEPROM 认证返回的状态码。 */
    uint8_t read_status;                                     /* 保存信息区读取是否成功，1 成功，0 失败。 */
    uint8_t raw_type_major;                                  /* EEPROM 信息区第 1 个字节，表示手柄主类型。 */
    uint8_t raw_type_minor;                                  /* EEPROM 信息区第 2 个字节，表示手柄子类型。 */
    uint8_t mapped_model;                                    /* 查表后的系统内部手柄型号值。 */
    const HandlescanHandleTypeConfig *handle_type_cfg;       /* 指向命中的手柄类型配置表项。 */

    /*
     * A 通道短接检测脚当前按“低电平表示插入成立”处理。
     * 因此这里读取到 `GPIO_PIN_RESET` 时，表示 A 通道已经检测到短接插入。
     */
    is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_RESET); /* 低电平代表 A 通道短接成立。 */

    if (is_inserted == 0U)
    {
        s_handleA_debounce.in_debounce_ticks = 0U;           /* 一旦检测到拔出候选，就清掉插入去抖计数。 */
        s_a_verify_start_wait_ticks = 0U;                    /* 同时清掉认证前等待计数，避免下次误续跑。 */

        /*
         * A 通道已经上线，或者已经进入认证失败保持态时，如果此时短接线被拔掉，
         * 需要切到拔出去抖阶段，并在稳定后输出离线报文。
         */
        if ((s_a_stage == HANDLESCAN_STAGE_ONLINE) ||
            (s_a_stage == HANDLESCAN_STAGE_DEBOUNCE_OUT) ||
            (s_a_stage == HANDLESCAN_STAGE_VERIFY_FAIL))
        {
            if (s_a_stage != HANDLESCAN_STAGE_DEBOUNCE_OUT)
            {
                s_handleA_debounce.out_debounce_ticks = 0U;  /* 首次进入拔出去抖时，先把拔出去抖计数清零。 */
                s_a_stage = HANDLESCAN_STAGE_DEBOUNCE_OUT;   /* 状态机切到“拔出去抖”阶段。 */
            }

            if (s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
            {
                ++s_handleA_debounce.out_debounce_ticks;     /* 拔出去抖未满阈值时，仅累计次数后返回。 */
                return;                                      /* 本轮不做其他动作，等待下一个扫描周期。 */
            }

            s_handleA_debounce.out_debounce_ticks = 0U;      /* 拔出去抖完成后，清掉计数器。 */
            s_a_stage = HANDLESCAN_STAGE_IDLE;               /* A 通道状态机回到空闲态。 */
            s_a_last_alarm = 0U;                             /* 清掉 A 通道最近一次报警缓存。 */
            Workvalue_s.Achanell_online_flag = 0U;           /* 清除 A 通道在线标志。 */
            Workvalue_s.A_ChipRecognition_FLAG = 0U;         /* 清除 A 通道认证通过标志。 */
            Workvalue_s.A_ShortCircuitRecognition_FLAG = 0U; /* 清除 A 通道短接成立标志。 */
            ChannelValue_s.A.hand_model = 0U;                /* 清空 A 通道当前记忆的手柄型号。 */
            Workvalue_s.ScreenKey_data = 26U;                /* 通知 UI：A 手柄已拔出。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出“离线完成”报文。 */
            (void)Handlescan_HandleRunningPlugAlarm(1U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            return;                                          /* A 通道离线处理完成，本轮到此结束。 */
        }

        /*
         * 其余情况下说明当前还没有形成有效上线，直接把 A 通道状态机静默复位即可，
         * 不需要通知 UI，也不需要输出离线报文。
         */
        s_handleA_debounce.out_debounce_ticks = 0U;          /* 静默复位时也要把拔出去抖计数清零。 */
        s_a_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态，等待下一次真实插入。 */
        Workvalue_s.Achanell_online_flag = 0U;               /* 保证 A 通道在线标志关闭。 */
        Workvalue_s.A_ChipRecognition_FLAG = 0U;             /* 保证 A 通道认证标志关闭。 */
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 0U;     /* 保证 A 通道短接标志关闭。 */
        return;                                              /* 当前只是插入取消，不做 UI 和报文更新。 */
    }

    /*
     * 当前已经检测到 A 通道短接成立。
     * 如果状态机还在空闲态，则说明这是一次新的插入开始，先进入插入去抖阶段。
     */
    if (s_a_stage == HANDLESCAN_STAGE_IDLE)
    {
        s_handleA_debounce.in_debounce_ticks = 0U;           /* 新一轮插入开始时，先清空插入去抖计数。 */
        s_handleA_debounce.out_debounce_ticks = 0U;          /* 同时清空拔出去抖计数，避免带入上一次状态。 */
        s_a_verify_start_wait_ticks = 0U;                    /* 清空认证前等待计数。 */
        s_a_stage = HANDLESCAN_STAGE_DEBOUNCE_IN;            /* 状态机切到“插入去抖”阶段。 */
    }

    /*
     * 插入去抖阶段。
     * 只有连续多个扫描周期都维持插入有效，才认为本次插入稳定成立。
     */
    if (s_a_stage == HANDLESCAN_STAGE_DEBOUNCE_IN)
    {
        if (s_handleA_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
        {
            ++s_handleA_debounce.in_debounce_ticks;          /* 插入信号仍稳定时，累计插入去抖计数。 */
            return;                                          /* 未到阈值前，不进入认证阶段。 */
        }

        s_handleA_debounce.in_debounce_ticks = 0U;           /* 插入去抖达标后，清掉计数器。 */
        s_a_verify_start_wait_ticks = 0U;                    /* 开始认证前等待前，先把等待计数清零。 */
        s_a_stage = HANDLESCAN_STAGE_WAIT_VERIFY;            /* 状态机切到“认证前等待”阶段。 */
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 1U;     /* 置位 A 通道短接识别成功标志。 */
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS); /* 输出“插入稳定”报文。 */
        return;                                              /* 本轮插入确认完成，等待下一轮进入认证。 */
    }

    /*
     * 插入稳定后的认证前等待阶段。
     * 这里保留 100ms 的稳定时间，避免手柄刚插入时立刻访问 EEPROM 导致首包失败。
     */
    if (s_a_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_a_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_a_verify_start_wait_ticks;                   /* 每个扫描周期把认证前等待计数加一。 */
            return;                                          /* 等待未满 100ms 前，不访问 EEPROM。 */
        }

        s_a_verify_start_wait_ticks = 0U;                    /* 等待完成后，把等待计数器清零。 */
        s_a_stage = HANDLESCAN_STAGE_VERIFY;                 /* 状态机切到“认证执行”阶段。 */
    }

    /*
     * A 通道认证阶段。
     * 这里只调用一次 I2C2 认证接口；认证失败后停在失败态，等待重新插拔。
     */
    if (s_a_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 认证前先清掉上一轮底层 I2C 调试信息。 */
        verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result); /* 调用 A 通道 I2C2 EEPROM 认证接口。 */
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status); /* 输出当前认证结果码。 */

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            s_a_last_alarm = Handlescan_MapVerifyStatusToAlarm(verify_status); /* 把认证失败码映射为系统报警码。 */
            if (s_a_last_alarm != 0U)
            {
                Workvalue_s.Alarm_value = s_a_last_alarm;    /* 写入当前系统报警值。 */
                Workvalue_s.beep_Alarm_flag = 1U;            /* 打开蜂鸣提示标志。 */
                Handlescan_DebugTraceI2cDetail(1U);          /* 输出最近一次底层 I2C 访问细节。 */
                Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, s_a_last_alarm); /* 输出报警已设置报文。 */
            }
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;        /* 认证失败后切到失败保持态。 */
            return;                                          /* 本轮不再继续读信息区。 */
        }

        s_a_stage = HANDLESCAN_STAGE_READ_INFO;              /* 认证通过后，进入信息区读取阶段。 */
    }

    /*
     * A 通道信息区读取阶段。
     * 认证通过后，再从 0x0020 读取 16 字节信息区，并解析前两个字节得到手柄类型。
     */
    if (s_a_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 读信息区前同样先清调试缓存。 */
        read_status = AT24CS32_ReadBytes_I2C2(HANDLESCAN_INFO_ADDR, s_a_info_buf, HANDLESCAN_INFO_SIZE); /* 从 A 通道 EEPROM 读出 16 字节信息区。 */
        if (read_status == 0U)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL; /* 信息区读取失败时，写入数据区失败报警。 */
            Workvalue_s.beep_Alarm_flag = 1U;               /* 打开蜂鸣提示标志。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(1U);             /* 输出本轮失败对应的底层 I2C 细节。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出报警已设置报文。 */
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;       /* 进入失败保持态，等待重新插拔。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_type_major = s_a_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 取出信息区第 1 字节作为手柄主类型。 */
        raw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 取出信息区第 2 字节作为手柄子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 按两个原始字节查找型号配置表。 */
        if (handle_type_cfg == NULL)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL; /* 查表失败时，同样按数据无效报警处理。 */
            Workvalue_s.beep_Alarm_flag = 1U;               /* 打开蜂鸣提示标志。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor); /* 输出“手柄类型无法识别”报文。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出报警已设置报文。 */
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;       /* 进入失败保持态。 */
            return;                                         /* 等待重新插拔。 */
        }

        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出查表后的系统内部手柄型号值。 */
        Workvalue_s.hand_model = mapped_model;              /* 更新当前全局工作手柄型号。 */
        ChannelValue_s.A.hand_model = mapped_model;         /* 把 A 通道记忆的手柄型号同步更新。 */
        Workvalue_s.Achanell_online_flag = 1U;              /* 置位 A 通道在线标志。 */
        Workvalue_s.A_ChipRecognition_FLAG = 1U;            /* 置位 A 通道认证通过标志。 */
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 1U;    /* 维持 A 通道短接识别成功标志。 */
        Workvalue_s.ScreenKey_data = 24U;                   /* 通知 UI：A 通道手柄上线。 */
        s_a_last_alarm = 0U;                                /* 上线成功后清掉最近一次报警缓存。 */
        s_a_stage = HANDLESCAN_STAGE_ONLINE;                /* 状态机切到在线保持态。 */
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model); /* 输出 A 通道上线报文。 */
        Handlescan_DebugTraceHandleName(1U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name); /* 输出 A 通道手柄名称报文。 */
        return;                                             /* A 通道本轮流程结束，后续等待拔出。 */
    }

    if ((s_a_stage == HANDLESCAN_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_STAGE_ONLINE))
    {
        return;                                             /* 在线保持态和失败保持态都保持静默，只等待状态变化。 */
    }
}

/*
 * B 通道 SSC 扫描函数。
 * B 通道沿用与 A 通道一致的状态机流程，但底层资源切换为：
 * 1. 短接检测脚使用 B 通道输入脚；
 * 2. EEPROM 认证和信息区读取使用 I2C3；
 * 3. 在线状态和手柄型号写回到 B 通道全局变量；
 * 4. 串口报文通道号使用 `CH=02`。
 */
void HandlescanB_Fun_SSC(void)
{
    uint8_t is_inserted;                                      /* 当前采样到的 B 通道插入状态，1 表示短接成立，0 表示短接断开。 */
    AT24CS32_CRC_Result verify_result;                       /* 保存 B 通道 EEPROM 认证输出结果。 */
    AT24CS32_CRC_Status verify_status;                       /* 保存 B 通道当前这次认证返回的状态码。 */
    uint8_t read_status;                                     /* 保存 B 通道信息区读取结果。 */
    uint8_t raw_type_major;                                  /* EEPROM 信息区第 1 个字节，表示手柄主类型。 */
    uint8_t raw_type_minor;                                  /* EEPROM 信息区第 2 个字节，表示手柄子类型。 */
    uint8_t mapped_model;                                    /* 查表映射后的系统内部型号值。 */
    const HandlescanHandleTypeConfig *handle_type_cfg;       /* 指向命中的 B 通道手柄配置表项。 */

    /*
     * B 通道短接检测脚同样按“低电平表示插入成立”处理。
     * 这里直接在 B 通道函数内实现，不再通过通用 helper 转发。
     */
    is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_B_SHORT_GPIO, HANDLESCAN_B_SHORT_PIN) == GPIO_PIN_RESET); /* 低电平代表 B 通道短接成立。 */

    if (is_inserted == 0U)
    {
        s_handleB_debounce.in_debounce_ticks = 0U;           /* 一旦检测到拔出候选，就清掉 B 通道插入去抖计数。 */
        s_b_verify_start_wait_ticks = 0U;                    /* 同时清掉 B 通道认证前等待计数。 */

        /*
         * 如果 B 通道之前已经在线，或者认证失败后仍保持插入态，
         * 此时检测到拔出，就进入拔出去抖并在稳定后输出离线报文。
         */
        if ((s_b_stage == HANDLESCAN_STAGE_ONLINE) ||
            (s_b_stage == HANDLESCAN_STAGE_DEBOUNCE_OUT) ||
            (s_b_stage == HANDLESCAN_STAGE_VERIFY_FAIL))
        {
            if (s_b_stage != HANDLESCAN_STAGE_DEBOUNCE_OUT)
            {
                s_handleB_debounce.out_debounce_ticks = 0U;  /* 首次进入 B 通道拔出去抖时，先清计数。 */
                s_b_stage = HANDLESCAN_STAGE_DEBOUNCE_OUT;   /* B 通道状态机切到“拔出去抖”阶段。 */
            }

            if (s_handleB_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
            {
                ++s_handleB_debounce.out_debounce_ticks;     /* B 通道拔出去抖未满阈值时，仅累计次数。 */
                return;                                      /* 等待下一次扫描继续判断。 */
            }

            s_handleB_debounce.out_debounce_ticks = 0U;      /* 拔出去抖完成后，清掉 B 通道计数器。 */
            s_b_stage = HANDLESCAN_STAGE_IDLE;               /* B 通道状态机回到空闲态。 */
            s_b_last_alarm = 0U;                             /* 清掉 B 通道最近一次报警缓存。 */
            Workvalue_s.Bchanell_online_flag = 0U;           /* 清除 B 通道在线标志。 */
            Workvalue_s.B_ChipRecognition_FLAG = 0U;         /* 清除 B 通道认证通过标志。 */
            Workvalue_s.B_ShortCircuitRecognition_FLAG = 0U; /* 清除 B 通道短接成立标志。 */
            ChannelValue_s.B.hand_model = 0U;                /* 清空 B 通道当前记忆的手柄型号。 */
            Workvalue_s.ScreenKey_data = 26U;                /* 通知 UI：B 手柄已拔出。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出 B 通道“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出 B 通道“离线完成”报文。 */
            (void)Handlescan_HandleRunningPlugAlarm(2U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            return;                                          /* B 通道离线处理结束。 */
        }

        /*
         * 如果还没有形成有效上线，说明只是插入过程中取消或接触抖动，
         * 此时对 B 通道做静默复位即可。
         */
        s_handleB_debounce.out_debounce_ticks = 0U;          /* 静默复位时，也把 B 通道拔出去抖计数清零。 */
        s_b_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态。 */
        Workvalue_s.Bchanell_online_flag = 0U;               /* 保证 B 通道在线标志关闭。 */
        Workvalue_s.B_ChipRecognition_FLAG = 0U;             /* 保证 B 通道认证标志关闭。 */
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 0U;     /* 保证 B 通道短接标志关闭。 */
        return;                                              /* 当前不形成有效离线事件，只做静默收尾。 */
    }

    /*
     * B 通道检测到新的插入开始，进入插入去抖阶段。
     */
    if (s_b_stage == HANDLESCAN_STAGE_IDLE)
    {
        s_handleB_debounce.in_debounce_ticks = 0U;           /* 新一轮 B 通道插入开始时，先清空插入去抖计数。 */
        s_handleB_debounce.out_debounce_ticks = 0U;          /* 清空 B 通道拔出去抖计数。 */
        s_b_verify_start_wait_ticks = 0U;                    /* 清空 B 通道认证前等待计数。 */
        s_b_stage = HANDLESCAN_STAGE_DEBOUNCE_IN;            /* 状态机切到“B 通道插入去抖”阶段。 */
    }

    /*
     * B 通道插入去抖阶段。
     * 只有连续多个扫描周期都检测到短接成立，才进入后续认证流程。
     */
    if (s_b_stage == HANDLESCAN_STAGE_DEBOUNCE_IN)
    {
        if (s_handleB_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
        {
            ++s_handleB_debounce.in_debounce_ticks;          /* B 通道插入稳定时，累计去抖次数。 */
            return;                                          /* 未达到阈值前，继续等待。 */
        }

        s_handleB_debounce.in_debounce_ticks = 0U;           /* 插入去抖达标后，清掉 B 通道计数器。 */
        s_b_verify_start_wait_ticks = 0U;                    /* 开始认证前等待前，先把等待计数清零。 */
        s_b_stage = HANDLESCAN_STAGE_WAIT_VERIFY;            /* 切到“B 通道认证前等待”阶段。 */
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 1U;     /* 置位 B 通道短接识别成功标志。 */
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS); /* 输出 B 通道“插入稳定”报文。 */
        return;                                              /* 本轮到此结束。 */
    }

    /*
     * B 通道认证前等待阶段。
     * 等待时间与 A 通道保持一致，都是 100ms。
     */
    if (s_b_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_b_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_b_verify_start_wait_ticks;                   /* 每个扫描周期把 B 通道认证前等待计数加一。 */
            return;                                          /* 等待未满 100ms 前，不访问 B 通道 EEPROM。 */
        }

        s_b_verify_start_wait_ticks = 0U;                    /* 等待完成后清零计数器。 */
        s_b_stage = HANDLESCAN_STAGE_VERIFY;                 /* 切到“B 通道认证执行”阶段。 */
    }

    /*
     * B 通道认证阶段。
     * 认证接口切换为 I2C3，对 B 通道 EEPROM 只做一次认证。
     */
    if (s_b_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 认证前清掉上一轮底层 I2C 调试信息。 */
        verify_status = AT24CS32_VerifyCrc_I2C3(&verify_result); /* 调用 B 通道 I2C3 EEPROM 认证接口。 */
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status); /* 输出 B 通道认证结果码。 */

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            s_b_last_alarm = Handlescan_MapVerifyStatusToAlarm(verify_status); /* 把 B 通道认证失败码映射为报警码。 */
            if (s_b_last_alarm != 0U)
            {
                Workvalue_s.Alarm_value = s_b_last_alarm;    /* 写入当前系统报警值。 */
                Workvalue_s.beep_Alarm_flag = 1U;            /* 打开蜂鸣提示标志。 */
                Handlescan_DebugTraceI2cDetail(2U);          /* 输出本轮 B 通道失败的 I2C 细节。 */
                Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, s_b_last_alarm); /* 输出报警设置报文。 */
            }
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;        /* B 通道切到失败保持态。 */
            return;                                          /* 停止后续信息区读取。 */
        }

        s_b_stage = HANDLESCAN_STAGE_READ_INFO;              /* 认证通过后进入 B 通道信息区读取阶段。 */
    }

    /*
     * B 通道信息区读取阶段。
     * 从 B 通道 EEPROM 的 0x0020 起始地址读取业务信息，再映射成系统手柄型号。
     */
    if (s_b_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 读信息区前先清掉底层调试缓存。 */
        read_status = AT24CS32_ReadBytes_I2C3(HANDLESCAN_INFO_ADDR, s_b_info_buf, HANDLESCAN_INFO_SIZE); /* 从 B 通道 EEPROM 读取 16 字节信息区。 */
        if (read_status == 0U)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL; /* B 通道信息区读取失败时，写入数据区失败报警。 */
            Workvalue_s.beep_Alarm_flag = 1U;               /* 打开蜂鸣提示标志。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INFO_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出 B 通道信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(2U);             /* 输出最近一次底层 I2C 访问细节。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出报警已设置报文。 */
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;       /* 切到失败保持态。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_type_major = s_b_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 取出 B 通道信息区第 1 字节作为主类型。 */
        raw_type_minor = s_b_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 取出 B 通道信息区第 2 字节作为子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 按两个原始字节查找配置表。 */
        if (handle_type_cfg == NULL)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL; /* 查表失败时，按“型号无效”处理。 */
            Workvalue_s.beep_Alarm_flag = 1U;               /* 打开蜂鸣提示标志。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor); /* 输出 B 通道“手柄类型无法识别”报文。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL); /* 输出报警设置报文。 */
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;       /* 切到失败保持态。 */
            return;                                         /* 等待重新插拔。 */
        }

        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出查表后的系统内部手柄型号值。 */
        Workvalue_s.hand_model = mapped_model;              /* 更新当前全局工作手柄型号。 */
        ChannelValue_s.B.hand_model = mapped_model;         /* 更新 B 通道记忆的手柄型号。 */
        Workvalue_s.Bchanell_online_flag = 1U;              /* 置位 B 通道在线标志。 */
        Workvalue_s.B_ChipRecognition_FLAG = 1U;            /* 置位 B 通道认证通过标志。 */
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 1U;    /* 维持 B 通道短接识别成功标志。 */
        Workvalue_s.ScreenKey_data = 25U;                   /* 通知 UI：B 通道手柄上线。 */
        s_b_last_alarm = 0U;                                /* 上线成功后清掉 B 通道最近一次报警缓存。 */
        s_b_stage = HANDLESCAN_STAGE_ONLINE;                /* 状态机切到 B 通道在线保持态。 */
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model); /* 输出 B 通道上线报文。 */
        Handlescan_DebugTraceHandleName(2U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name); /* 输出 B 通道手柄名称报文。 */
        return;                                             /* B 通道本轮处理结束。 */
    }

    if ((s_b_stage == HANDLESCAN_STAGE_VERIFY_FAIL) || (s_b_stage == HANDLESCAN_STAGE_ONLINE))
    {
        return;                                             /* 在线保持态和失败保持态都保持静默，只等待拔出或重新插入。 */
    }
}

//============================================================================
// 函数名称: Handlescan_Fun()
// 功能描述: 手柄扫描总入口
// 说明: 当前同时启用 A/B 两个通道的新型 EEPROM 认证扫描逻辑
//============================================================================
void Handlescan_Fun(void)
{
    HandlescanA_Fun_SSC();
    HandlescanB_Fun_SSC();
}

/* HANDLESCANTask 的任务执行入口 */
void HANDLESCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN HANDLESCANTaskFunc */
  /* Infinite loop */
  Handlescan_Fun();
  /* USER CODE END HANDLESCANTaskFunc */
}

//============================================================================
// 函数名称: HandlescanTaskInit()
// 功能描述: 创建并启动手柄扫描任务
// 说明: 扫描周期为 10ms
//============================================================================
void HandlescanTaskInit(void)
{
  /* definition and creation of HANDLESCANTask */
	app_task_create(&HANDLESCANTaskHandle, HANDLESCANTaskFunc);
	app_task_start(&HANDLESCANTaskHandle, APP_TASK_ALWAYS, 10);
}









