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
 * 这些步骤码用于输出 HS、HSDBG、HSNAME 报文，便于现场快速判断当前流程卡在什么阶段。
 */
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
}

/*
 * 输出最近一次 EEPROM/I2C 访问的底层细节。
 * 当认证失败或信息区读取失败时，上层可以立刻调用这个函数，把底层操作类型、
 * 器件地址、字地址、长度、HAL 返回值和错误码全部打印出来，便于现场快速定位。
 */
static void Handlescan_DebugTraceI2cDetail(uint8_t channel)
{
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
 * 3. 关闭 `K1`；
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
    uint8_t is_inserted;
    AT24CS32_CRC_Result verify_result;
    AT24CS32_CRC_Status verify_status;
    uint8_t read_status;
    uint8_t raw_type_major;
    uint8_t raw_type_minor;
    uint8_t mapped_model;
    const HandlescanHandleTypeConfig *handle_type_cfg;

    /*
     * A 通道短接检测脚当前按“低电平表示插入成立”处理。
     * 因此这里读取到 `GPIO_PIN_RESET` 时，表示 A 通道已经检测到短接插入。
     */
    is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_RESET);

    if (is_inserted == 0U)
    {
        s_handleA_debounce.in_debounce_ticks = 0U;
        s_a_verify_start_wait_ticks = 0U;

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
                s_handleA_debounce.out_debounce_ticks = 0U;
                s_a_stage = HANDLESCAN_STAGE_DEBOUNCE_OUT;
            }

            if (s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
            {
                ++s_handleA_debounce.out_debounce_ticks;
                return;
            }

            s_handleA_debounce.out_debounce_ticks = 0U;
            s_a_stage = HANDLESCAN_STAGE_IDLE;
            s_a_last_alarm = 0U;
            Workvalue_s.Achanell_online_flag = 0U;
            Workvalue_s.A_ChipRecognition_FLAG = 0U;
            Workvalue_s.A_ShortCircuitRecognition_FLAG = 0U;
            ChannelValue_s.A.hand_model = 0U;
            Workvalue_s.ScreenKey_data = 26U;
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U);
            (void)Handlescan_HandleRunningPlugAlarm(1U);
            return;
        }

        /*
         * 其余情况下说明当前还没有形成有效上线，直接把 A 通道状态机静默复位即可，
         * 不需要通知 UI，也不需要输出离线报文。
         */
        s_handleA_debounce.out_debounce_ticks = 0U;
        s_a_stage = HANDLESCAN_STAGE_IDLE;
        Workvalue_s.Achanell_online_flag = 0U;
        Workvalue_s.A_ChipRecognition_FLAG = 0U;
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 0U;
        return;
    }

    /*
     * 当前已经检测到 A 通道短接成立。
     * 如果状态机还在空闲态，则说明这是一次新的插入开始，先进入插入去抖阶段。
     */
    if (s_a_stage == HANDLESCAN_STAGE_IDLE)
    {
        s_handleA_debounce.in_debounce_ticks = 0U;
        s_handleA_debounce.out_debounce_ticks = 0U;
        s_a_verify_start_wait_ticks = 0U;
        s_a_stage = HANDLESCAN_STAGE_DEBOUNCE_IN;
    }

    /*
     * 插入去抖阶段。
     * 只有连续多个扫描周期都维持插入有效，才认为本次插入稳定成立。
     */
    if (s_a_stage == HANDLESCAN_STAGE_DEBOUNCE_IN)
    {
        if (s_handleA_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
        {
            ++s_handleA_debounce.in_debounce_ticks;
            return;
        }

        s_handleA_debounce.in_debounce_ticks = 0U;
        s_a_verify_start_wait_ticks = 0U;
        s_a_stage = HANDLESCAN_STAGE_WAIT_VERIFY;
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 1U;
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS);
        return;
    }

    /*
     * 插入稳定后的认证前等待阶段。
     * 这里保留 100ms 的稳定时间，避免手柄刚插入时立刻访问 EEPROM 导致首包失败。
     */
    if (s_a_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_a_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_a_verify_start_wait_ticks;
            return;
        }

        s_a_verify_start_wait_ticks = 0U;
        s_a_stage = HANDLESCAN_STAGE_VERIFY;
    }

    /*
     * A 通道认证阶段。
     * 这里只调用一次 I2C2 认证接口；认证失败后停在失败态，等待重新插拔。
     */
    if (s_a_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();
        verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result);
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            s_a_last_alarm = Handlescan_MapVerifyStatusToAlarm(verify_status);
            if (s_a_last_alarm != 0U)
            {
                Workvalue_s.Alarm_value = s_a_last_alarm;
                Workvalue_s.beep_Alarm_flag = 1U;
                Handlescan_DebugTraceI2cDetail(1U);
                Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, s_a_last_alarm);
            }
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        s_a_stage = HANDLESCAN_STAGE_READ_INFO;
    }

    /*
     * A 通道信息区读取阶段。
     * 认证通过后，再从 0x0020 读取 16 字节信息区，并解析前两个字节得到手柄类型。
     */
    if (s_a_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();
        read_status = AT24CS32_ReadBytes_I2C2(HANDLESCAN_INFO_ADDR, s_a_info_buf, HANDLESCAN_INFO_SIZE);
        if (read_status == 0U)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
            Workvalue_s.beep_Alarm_flag = 1U;
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL);
            Handlescan_DebugTraceI2cDetail(1U);
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL);
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        raw_type_major = s_a_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET];
        raw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET];
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor);
        if (handle_type_cfg == NULL)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
            Workvalue_s.beep_Alarm_flag = 1U;
            //Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor);
            //Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL);
            s_a_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        mapped_model = handle_type_cfg->mapped_handle_type;
        Workvalue_s.hand_model = mapped_model;
        ChannelValue_s.A.hand_model = mapped_model;
        Workvalue_s.Achanell_online_flag = 1U;
        Workvalue_s.A_ChipRecognition_FLAG = 1U;
        Workvalue_s.A_ShortCircuitRecognition_FLAG = 1U;
        Workvalue_s.ScreenKey_data = 24U;
        s_a_last_alarm = 0U;
        s_a_stage = HANDLESCAN_STAGE_ONLINE;
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model);
        Handlescan_DebugTraceHandleName(1U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name);
        return;
    }

    if ((s_a_stage == HANDLESCAN_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_STAGE_ONLINE))
    {
        return;
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
    uint8_t is_inserted;
    AT24CS32_CRC_Result verify_result;
    AT24CS32_CRC_Status verify_status;
    uint8_t read_status;
    uint8_t raw_type_major;
    uint8_t raw_type_minor;
    uint8_t mapped_model;
    const HandlescanHandleTypeConfig *handle_type_cfg;

    /*
     * B 通道短接检测脚同样按“低电平表示插入成立”处理。
     * 这里直接在 B 通道函数内实现，不再通过通用 helper 转发。
     */
    is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_B_SHORT_GPIO, HANDLESCAN_B_SHORT_PIN) == GPIO_PIN_RESET);

    if (is_inserted == 0U)
    {
        s_handleB_debounce.in_debounce_ticks = 0U;
        s_b_verify_start_wait_ticks = 0U;

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
                s_handleB_debounce.out_debounce_ticks = 0U;
                s_b_stage = HANDLESCAN_STAGE_DEBOUNCE_OUT;
            }

            if (s_handleB_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
            {
                ++s_handleB_debounce.out_debounce_ticks;
                return;
            }

            s_handleB_debounce.out_debounce_ticks = 0U;
            s_b_stage = HANDLESCAN_STAGE_IDLE;
            s_b_last_alarm = 0U;
            Workvalue_s.Bchanell_online_flag = 0U;
            Workvalue_s.B_ChipRecognition_FLAG = 0U;
            Workvalue_s.B_ShortCircuitRecognition_FLAG = 0U;
            ChannelValue_s.B.hand_model = 0U;
            Workvalue_s.ScreenKey_data = 26U;
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_OFFLINE, 0U);
            (void)Handlescan_HandleRunningPlugAlarm(2U);
            return;
        }

        /*
         * 如果还没有形成有效上线，说明只是插入过程中取消或接触抖动，
         * 此时对 B 通道做静默复位即可。
         */
        s_handleB_debounce.out_debounce_ticks = 0U;
        s_b_stage = HANDLESCAN_STAGE_IDLE;
        Workvalue_s.Bchanell_online_flag = 0U;
        Workvalue_s.B_ChipRecognition_FLAG = 0U;
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 0U;
        return;
    }

    /*
     * B 通道检测到新的插入开始，进入插入去抖阶段。
     */
    if (s_b_stage == HANDLESCAN_STAGE_IDLE)
    {
        s_handleB_debounce.in_debounce_ticks = 0U;
        s_handleB_debounce.out_debounce_ticks = 0U;
        s_b_verify_start_wait_ticks = 0U;
        s_b_stage = HANDLESCAN_STAGE_DEBOUNCE_IN;
    }

    /*
     * B 通道插入去抖阶段。
     * 只有连续多个扫描周期都检测到短接成立，才进入后续认证流程。
     */
    if (s_b_stage == HANDLESCAN_STAGE_DEBOUNCE_IN)
    {
        if (s_handleB_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
        {
            ++s_handleB_debounce.in_debounce_ticks;
            return;
        }

        s_handleB_debounce.in_debounce_ticks = 0U;
        s_b_verify_start_wait_ticks = 0U;
        s_b_stage = HANDLESCAN_STAGE_WAIT_VERIFY;
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 1U;
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS);
        return;
    }

    /*
     * B 通道认证前等待阶段。
     * 等待时间与 A 通道保持一致，都是 100ms。
     */
    if (s_b_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_b_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_b_verify_start_wait_ticks;
            return;
        }

        s_b_verify_start_wait_ticks = 0U;
        s_b_stage = HANDLESCAN_STAGE_VERIFY;
    }

    /*
     * B 通道认证阶段。
     * 认证接口切换为 I2C3，对 B 通道 EEPROM 只做一次认证。
     */
    if (s_b_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();
        verify_status = AT24CS32_VerifyCrc_I2C3(&verify_result);
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            s_b_last_alarm = Handlescan_MapVerifyStatusToAlarm(verify_status);
            if (s_b_last_alarm != 0U)
            {
                Workvalue_s.Alarm_value = s_b_last_alarm;
                Workvalue_s.beep_Alarm_flag = 1U;
                Handlescan_DebugTraceI2cDetail(2U);
                Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, s_b_last_alarm);
            }
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        s_b_stage = HANDLESCAN_STAGE_READ_INFO;
    }

    /*
     * B 通道信息区读取阶段。
     * 从 B 通道 EEPROM 的 0x0020 起始地址读取业务信息，再映射成系统手柄型号。
     */
    if (s_b_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();
        read_status = AT24CS32_ReadBytes_I2C3(HANDLESCAN_INFO_ADDR, s_b_info_buf, HANDLESCAN_INFO_SIZE);
        if (read_status == 0U)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
            Workvalue_s.beep_Alarm_flag = 1U;
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INFO_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL);
            Handlescan_DebugTraceI2cDetail(2U);
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL);
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        raw_type_major = s_b_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET];
        raw_type_minor = s_b_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET];
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor);
        if (handle_type_cfg == NULL)
        {
            Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
            Workvalue_s.beep_Alarm_flag = 1U;
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor);
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ALARM_SET, HANDLESCAN_ALARM_A_DATA_FAIL);
            s_b_stage = HANDLESCAN_STAGE_VERIFY_FAIL;
            return;
        }

        mapped_model = handle_type_cfg->mapped_handle_type;
        Workvalue_s.hand_model = mapped_model;
        ChannelValue_s.B.hand_model = mapped_model;
        Workvalue_s.Bchanell_online_flag = 1U;
        Workvalue_s.B_ChipRecognition_FLAG = 1U;
        Workvalue_s.B_ShortCircuitRecognition_FLAG = 1U;
        Workvalue_s.ScreenKey_data = 25U;
        s_b_last_alarm = 0U;
        s_b_stage = HANDLESCAN_STAGE_ONLINE;
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model);
        Handlescan_DebugTraceHandleName(2U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name);
        return;
    }

    if ((s_b_stage == HANDLESCAN_STAGE_VERIFY_FAIL) || (s_b_stage == HANDLESCAN_STAGE_ONLINE))
    {
        return;
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













