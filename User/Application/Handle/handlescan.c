// handlescan.c

#include "handlescan.h"
#include "bsp_board.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "data.h"
#include "common.h"
#include "OneWireI.h"
#include "OneWireII.h"

#include <stdint.h>
#include <stdio.h>
#include "screen.h"
#include "kernel_scheduler.h"
#include "datahand.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscKEYBH.h"
#include "at24cs32.h"
#include "at24cs32_crc_verify.h"

kernel_task_t HANDLESCANTaskHandle;

/*
 * A 、B通道短接检测脚定义。
 * 当前板级连接以这两个宏为准：
 * 1. A 通道短接检测脚绑定到 `HANDLESCAN_A_SHORT_PIN`；
 * 2. B 通道短接检测脚绑定到 `HANDLESCAN_B_SHORT_PIN`；
 * 3. 当前现场验证使用“低电平表示插入成立，高电平表示拔出候选”。
 * 后续所有插拔状态机都基于这个输入脚做边沿和去抖判断，因此如果硬件接线变更，只需要修改这里的宏。
 */
#define HANDLESCAN_A_SHORT_GPIO               BOARD_RES_HANDLESCAN_A_SHORT_PORT
#define HANDLESCAN_A_SHORT_PIN                BOARD_RES_HANDLESCAN_A_SHORT_PIN
#define HANDLESCAN_B_SHORT_GPIO               BOARD_RES_HANDLESCAN_B_SHORT_PORT
#define HANDLESCAN_B_SHORT_PIN                BOARD_RES_HANDLESCAN_B_SHORT_PIN

/*
 * 手柄扫描任务的调度周期和状态机时间参数。
 * 说明：
 * 1. 任务本身按 10ms 周期运行；
 * 2. 插入去抖使用 50ms，拔出去抖使用 500ms，避免短接线瞬断被误判为真实拔出；
 * 3. 插入稳定后额外等待 200ms 再做认证，给热插拔后的接口和 EEPROM 留出稳定时间；
 * 4. 单次认证失败后先做有限快速重试，避免上电或热插拔瞬间的 I2C 抖动把正确手柄判死；
 * 5. 多次认证仍失败后才进入最终报警保持，报警保持期间每 1000ms 慢速自恢复重试一次；
 * 6. 最终认证失败后蜂鸣器连续报警，直到坏手柄拔出或后续自恢复认证成功。
 */
#define HANDLESCAN_TASK_PERIOD_MS             10U
#define HANDLESCAN_INSERT_DEBOUNCE_MS         50U
#define HANDLESCAN_REMOVE_DEBOUNCE_MS         500U
#define HANDLESCAN_VERIFY_START_DELAY_MS      200U
#define HANDLESCAN_VERIFY_RETRY_DELAY_MS      200U
#define HANDLESCAN_VERIFY_ALARM_RETRY_MS      1000U
#define HANDLESCAN_VERIFY_RETRY_MAX           3U

#define HANDLESCAN_INSERT_DEBOUNCE_TICKS      (HANDLESCAN_INSERT_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_REMOVE_DEBOUNCE_TICKS      (HANDLESCAN_REMOVE_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_START_DELAY_TICKS   (HANDLESCAN_VERIFY_START_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_RETRY_DELAY_TICKS   (HANDLESCAN_VERIFY_RETRY_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_ALARM_RETRY_TICKS   (HANDLESCAN_VERIFY_ALARM_RETRY_MS / HANDLESCAN_TASK_PERIOD_MS)

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
 * EEPROM 中刀具信息区定义。
 * 第三页起始地址为 0x0040，本次先按旧工程联动需求，只解析：
 * 1. 前两个字节的刀具类型编码；
 * 2. 直径、长度、角度三个参数。
 * 说明：
 * 1. 参数按大端 16 位读取；
 * 2. AT24CS32 内部使用 0.1 精度存储，例如 12.5mm 会存成十进制 125，即 0x00 0x7D。
 */
#define HANDLESCAN_TOOL_INFO_ADDR             0x0040U
#define HANDLESCAN_TOOL_INFO_SIZE             16U
#define HANDLESCAN_TOOL_MAJOR_OFFSET          0U
#define HANDLESCAN_TOOL_MINOR_OFFSET          1U
#define HANDLESCAN_TOOL_DIAMETER_OFFSET       2U
#define HANDLESCAN_TOOL_LENGTH_OFFSET         4U
#define HANDLESCAN_TOOL_ANGLE_OFFSET          6U

/*
 * 手柄扫描报警码定义。
 * 1. 13：运行中插拔报警，沿用旧逻辑；
 * 2. 只有 EEPROM 最终校验失败才上报 WORK_ALARM_HANDLE_MODEL_ERROR。
 */
#define HANDLESCAN_ALARM_RUNNING_PLUG         13U

/*
 * 串口调试步骤码定义。
 * `HANDLESCAN_TRACE_ENABLE` 是当前 handlescan 驱动的总报文开关：
 * 1. 设为 `1U` 时，输出 `HS`、`HSDBG`、`HSNAME` 基础调试报文；
 * 2. 设为 `0U` 时，基础调试报文都会被静默处理，业务状态机保持不变；
 * 3. 刀具扩展报文单独受 `HANDLESCAN_TOOL_TRACE_ENABLE` 控制，方便现场按需开启。
 */
#define HANDLESCAN_TRACE_ENABLE               1U
#define HANDLESCAN_TOOL_TRACE_ENABLE          1U

#define HANDLESCAN_DBG_STEP_INSERT_PASS       0x02U
#define HANDLESCAN_DBG_STEP_REMOVE_PASS       0x03U
#define HANDLESCAN_DBG_STEP_VERIFY_STATUS     0x05U
#define HANDLESCAN_DBG_STEP_INFO_FAIL         0x06U
#define HANDLESCAN_DBG_STEP_MODEL_INVALID     0x07U
#define HANDLESCAN_DBG_STEP_ONLINE            0x08U
#define HANDLESCAN_DBG_STEP_OFFLINE           0x09U
#define HANDLESCAN_DBG_STEP_ALARM_SET         0x0AU
#define HANDLESCAN_DBG_STEP_I2C_DETAIL        0x0BU
#define HANDLESCAN_DBG_STEP_ALARM_CLEAR       0x0DU
#define HANDLESCAN_DBG_STEP_VERIFY_RETRY      0x0EU
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
    HANDLESCAN_STAGE_RETRY_WAIT,
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
 * 手柄信息来自 EEPROM 第 2 页，类型码固定以 `0x6B` 开头。
 */
static const HandlescanHandleTypeConfig s_hand_type_config_table[] =
{
    {0x6B, 0x01, TMBB_ONLINES, "TMBB"},
    {0x6B, 0x02, TMBA_ONLINES,  "TMBA"},
    {0x6B, 0x03, EMBA_ONLINES, "EMBA"},
    {0x6B, 0x04, EMBB_ONLINES,  "EMBB"},
    {0x6B, 0x05, PXBA_ONLINES,  "PXBA"},
    {0x6B, 0x06, PXBB_ONLINES,  "PXBB"}
};

/*
 * 当前已支持的刀具类型映射表。
 * 刀具信息来自 EEPROM 第 3 页，类型码固定以 `0x7C` 开头。
 * 后续如果新增刀具型号，只需追加新的 `{主类型, 子类型, Handle_Type_xx, "名称"}` 表项即可。
 */
static const HandlescanHandleTypeConfig s_tool_type_config_table[] =
{
    {0x7C, 0x01, MX_YIM_ONLINES,   "MXYTM"},
    {0x7C, 0x02, MX_YIP_ONLINES,   "MXYTP"},
    {0x7C, 0x03, PX_YIM_ONLINES,   "PXYTM"},
    {0x7C, 0x04, PX_YIP_ONLINES,   "PXYTP"},
    {0x7C, 0x05, JMB_ONLINES,      "JMB"},
    {0x7C, 0x06, MX_YIM16_ONLINES, "MXYTM16"}
};

/* A 通道运行时状态变量。 */
static HandlescanStage s_a_stage = HANDLESCAN_STAGE_IDLE;
static HandlescanDebounce s_handleA_debounce = {0U, 0U};
static uint8_t s_a_verify_start_wait_ticks = 0U;
static uint8_t s_a_last_alarm = 0U;
static uint8_t s_a_verify_retry_count = 0U;
static uint16_t s_a_verify_retry_wait_ticks = 0U;
static uint8_t s_a_info_buf[HANDLESCAN_INFO_SIZE] = {0U};
static uint8_t s_a_tool_info_buf[HANDLESCAN_TOOL_INFO_SIZE] = {0U};

/* B 通道运行时状态变量。 */
static HandlescanStage s_b_stage = HANDLESCAN_STAGE_IDLE;
static HandlescanDebounce s_handleB_debounce = {0U, 0U};
static uint8_t s_b_verify_start_wait_ticks = 0U;
static uint8_t s_b_last_alarm = 0U;
static uint8_t s_b_verify_retry_count = 0U;
static uint16_t s_b_verify_retry_wait_ticks = 0U;
static uint8_t s_b_info_buf[HANDLESCAN_INFO_SIZE] = {0U};
static uint8_t s_b_tool_info_buf[HANDLESCAN_TOOL_INFO_SIZE] = {0U};

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

    Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
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

    Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
#endif
}

/*
 * 在给定的映射表中查找类型配置。
 * 这样手柄映射和刀具映射可以共用同一套查表逻辑，只是输入的配置表不同。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindTypeConfig(const HandlescanHandleTypeConfig *config_table,
                                                                   uint32_t config_count,
                                                                   uint8_t first_byte,
                                                                   uint8_t second_byte)
{
    uint32_t index;

    for (index = 0U; index < config_count; ++index)
    {
        if ((config_table[index].first_byte == first_byte) &&
            (config_table[index].second_byte == second_byte))
        {
            return &config_table[index];
        }
    }

    return NULL;
}

/*
 * 根据 EEPROM 第 2 页前两个字节查找手柄配置。
 * 找到时返回对应表项，找不到时返回 `NULL`，由调用方按未知手柄处理。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindHandleTypeConfig(uint8_t first_byte, uint8_t second_byte)
{
    return Handlescan_FindTypeConfig(s_hand_type_config_table,
                                     (uint32_t)(sizeof(s_hand_type_config_table) / sizeof(s_hand_type_config_table[0])),
                                     first_byte,
                                     second_byte);
}

/*
 * 根据 EEPROM 第 3 页前两个字节查找刀具配置。
 * 找到时返回对应表项，找不到时返回 `NULL`，由调用方按未知刀具处理。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindToolTypeConfig(uint8_t first_byte, uint8_t second_byte)
{
    return Handlescan_FindTypeConfig(s_tool_type_config_table,
                                     (uint32_t)(sizeof(s_tool_type_config_table) / sizeof(s_tool_type_config_table[0])),
                                     first_byte,
                                     second_byte);
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

    Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
    (void)first_byte;
    (void)second_byte;
    (void)handle_name;
#endif
}

/*
 * 输出刀具名称和刀具规格扩展报文。
 * 该报文单独受 `HANDLESCAN_TOOL_TRACE_ENABLE` 控制，方便正式版只保留基础手柄报文。
 * 报文格式：
 * `HSTOOL,CH=xx,ID0=xx,ID1=xx,TYPE=xxxx,DIA=xxxx,LEN=xxxx,ANG=xxxx`
 */
static void Handlescan_DebugTraceToolInfo(uint8_t channel,
                                          uint8_t first_byte,
                                          uint8_t second_byte,
                                          const char *tool_name,
                                          uint16_t diameter_tenth,
                                          uint16_t length_tenth,
                                          uint16_t angle_tenth)
{
#if (HANDLESCAN_TRACE_ENABLE == 1U) && (HANDLESCAN_TOOL_TRACE_ENABLE == 1U)
    char tx_buf[128];
    int text_len;

    text_len = snprintf(tx_buf,
                        sizeof(tx_buf),
                        "HSTOOL,CH=%02u,ID0=%02X,ID1=%02X,TYPE=%s,DIA=%04u,LEN=%04u,ANG=%04u\r\n",
                        (unsigned int)channel,
                        (unsigned int)first_byte,
                        (unsigned int)second_byte,
                        tool_name,
                        (unsigned int)diameter_tenth,
                        (unsigned int)length_tenth,
                        (unsigned int)angle_tenth);
    if (text_len <= 0)
    {
        return;
    }

    if ((size_t)text_len > (sizeof(tx_buf) - 1U))
    {
        text_len = (int)(sizeof(tx_buf) - 1U);
    }

    Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
#else
    (void)channel;
    (void)first_byte;
    (void)second_byte;
    (void)tool_name;
    (void)diameter_tenth;
    (void)length_tenth;
    (void)angle_tenth;
#endif
}

/*
 * 按大端格式读取 EEPROM 中的 16 位参数。
 * EEPROM 里的刀具参数使用高字节在前、低字节在后的方式存储，因此这里统一封装成一个 helper。
 */
static uint16_t Handlescan_ReadUint16BE(const uint8_t *buffer, uint32_t offset)
{
    return (uint16_t)(((uint16_t)buffer[offset] << 8) | (uint16_t)buffer[offset + 1U]);
}

/*
 * 把 EEPROM 第 3 页解析到当前 UI 已使用的 `paoxueSpeciValue` 数组格式。
 * 当前 UI 适配层会按 `specidisplay(长度基值 * 5, 直径, 角度)` 来显示，因此这里保持兼容：
 * 1. `[0]` 保存长度基值，等于“0.1 精度长度值 / 5”；
 * 2. `[1]` 保存直径的 0.1 精度原始值；
 * 3. `[2]` 保存角度值；
 * 4. `[3]` 保存刀具映射后的系统类型。
 */
static void Handlescan_UpdateToolSpecValues(uint32_t *spec_values,
                                            uint16_t diameter_tenth,
                                            uint16_t length_tenth,
                                            uint16_t angle_tenth,
                                            uint8_t mapped_tool_type)
{
    spec_values[0] = (uint32_t)(length_tenth / 5U);
    spec_values[1] = (uint32_t)diameter_tenth;
    spec_values[2] = (uint32_t)angle_tenth;
    spec_values[3] = (uint32_t)mapped_tool_type;
}

/*
 * 清空当前通道缓存的刀具规格值，避免手柄拔出后界面继续显示上一次刀具数据。
 */
static void Handlescan_ClearToolSpecValues(uint32_t *spec_values)
{
    spec_values[0] = 0U;
    spec_values[1] = 0U;
    spec_values[2] = 0U;
    spec_values[3] = 0U;
}

/*
 * 新接口只保留 A/B 在线、识别结果和当前工作态三层数据：
 * 1. `ChannelrecognizeMessageA/B` 保存本通道最新识别出的手柄与刀具信息；
 * 2. `MemoryMsgA/B` 保存当前通道的记忆化手柄型号；
 * 3. `WorkMessage` 保存系统当前正在工作的通道和型号。
 * 下面这些 helper 统一负责新接口写入，避免在状态机主体中反复展开字段赋值。
 */
static uint8_t Handlescan_TenthToUint8(uint16_t value_tenth)
{
    uint16_t value = (uint16_t)(value_tenth / 10U);
    if (value > 0xFFU)
    {
        value = 0xFFU;
    }

    return (uint8_t)value;
}

static void Handlescan_ClearRecognizeMessage(ChannelrecognizeMessage_t *message)
{
    if (message == NULL)
    {
        return;
    }

    message->handle_type = 0U;
    message->tool_type = 0U;
    message->diameter = 0U;
    message->length = 0U;
    message->draw = 0U;
}

static void Handlescan_ClearActiveWorkIfNoOnlineHandle(void)
{
    if ((WorkMessage.Channel_Aonline == false) && (WorkMessage.Channel_Bonline == false))
    {
        WorkMessage.hand_model = 0U;
        WorkMessage.tool_type = 0U;
        WorkMessage.channel_work = 0U;
    }
}

static void Handlescan_ClearChannelState(uint8_t channel)
{
    if (channel == CHANNEL_A)
    {
        WorkMessage.Channel_Aonline = false;
        MemoryMsgA.hand_model = 0U;
        MemoryMsgA.hand_type_raw_major = 0U;                 /* A 通道拔出后清掉 EEPROM 原始手柄类型高字节，避免心跳误上传旧的 0x6B。 */
        MemoryMsgA.hand_type_raw_minor = 0U;                 /* A 通道拔出后清掉 EEPROM 原始手柄类型低字节，避免心跳误上传旧的子型号。 */
        Handlescan_ClearRecognizeMessage(&ChannelrecognizeMessageA);
    }
    else
    {
        WorkMessage.Channel_Bonline = false;
        MemoryMsgB.hand_model = 0U;
        MemoryMsgB.hand_type_raw_major = 0U;                 /* B 通道拔出后清掉 EEPROM 原始手柄类型高字节，避免心跳误上传旧的 0x6B。 */
        MemoryMsgB.hand_type_raw_minor = 0U;                 /* B 通道拔出后清掉 EEPROM 原始手柄类型低字节，避免心跳误上传旧的子型号。 */
        Handlescan_ClearRecognizeMessage(&ChannelrecognizeMessageB);
    }

    Handlescan_ClearActiveWorkIfNoOnlineHandle();
}

/*
 * 判断报警码是否属于手柄校验链路。
 * EEPROM 最终校验失败使用 WORK_ALARM_HANDLE_MODEL_ERROR，由 handlescan 负责在坏手柄拔出后释放。
 */
static uint8_t Handlescan_IsHandleVerifyAlarm(uint8_t alarm_value)
{
    return (uint8_t)(alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR); /* 返回 1 表示这是手柄 EEPROM 校验/型号报警，返回 0 表示不是本模块可清的报警。 */
}

/*
 * 发送一次普通提示蜂鸣。
 * 说明：sscBEEP 的普通按键蜂鸣会清掉蜂鸣任务内部报警锁存，所以只有 WorkMessage 当前无报警时才允许发单响。
 */
static void Handlescan_BeepOnceIfNoAlarm(void)
{
    if (WorkMessage.alarm_flag == false)
    {
        SendKeyBeepMessage(1U);                              /* 无报警时给用户一个 100ms 单响，用于提示插入成功或拔出确认。 */
    }
}

/*
 * 查询另一通道是否仍处于认证失败保持态。
 * 当前通道拔出并清报警时，如果另一通道仍有手柄型号错误报警，就把全局报警归还给另一通道。
 */
static uint8_t Handlescan_GetPeerActiveHandleAlarm(uint8_t channel)
{
    if (channel == CHANNEL_A)
    {
        if ((s_b_stage == HANDLESCAN_STAGE_VERIFY_FAIL) &&
            (Handlescan_IsHandleVerifyAlarm(s_b_last_alarm) != 0U))
        {
            return s_b_last_alarm;                           /* A 通道准备清报警时，B 通道仍失败保持，则返回 B 通道报警码。 */
        }

        return 0U;                                           /* B 通道没有可继承的手柄认证报警。 */
    }

    if ((s_a_stage == HANDLESCAN_STAGE_VERIFY_FAIL) &&
        (Handlescan_IsHandleVerifyAlarm(s_a_last_alarm) != 0U))
    {
        return s_a_last_alarm;                               /* B 通道准备清报警时，A 通道仍失败保持，则返回 A 通道报警码。 */
    }

    return 0U;                                               /* A 通道没有可继承的手柄认证报警。 */
}

/*
 * 清除当前通道拥有的手柄认证报警。
 * 拔出坏手柄后，如果没有另一通道接管报警，就同步清 WorkMessage 和蜂鸣锁存。
 */
static void Handlescan_ClearChannelAlarm(uint8_t channel, uint8_t alarm_value)
{
    uint8_t peer_alarm;                                      /* 保存另一通道仍在失败保持态时需要继承的报警码。 */

    if (Handlescan_IsHandleVerifyAlarm(alarm_value) == 0U)
    {
        return;                                              /* 非手柄校验报警不是本函数负责的报警，保持原报警状态不变。 */
    }

    if ((WorkMessage.alarm_flag == false) || (WorkMessage.alarm_value != alarm_value))
    {
        return;                                              /* 全局报警已被其他模块改写或清除，本通道不再覆盖它。 */
    }

    peer_alarm = Handlescan_GetPeerActiveHandleAlarm(channel); /* 当前通道清报警前，先检查另一通道是否仍需保持手柄认证报警。 */
    if (peer_alarm != 0U)
    {
        WorkMessage.alarm_flag = true;                       /* 另一通道仍失败时，全局报警继续保持有效。 */
        WorkMessage.alarm_value = peer_alarm;                /* 把全局报警值切回另一通道仍保持的手柄型号错误报警。 */
        SendAlarmMessage(peer_alarm);                        /* 重新通知蜂鸣任务当前仍存在手柄认证报警。 */
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, peer_alarm); /* 输出报警继承调试码。 */
        return;                                              /* 已由另一通道接管报警，不执行清零。 */
    }

    WorkMessage.alarm_flag = false;                          /* 没有其他手柄认证报警时，清除全局报警标志。 */
    WorkMessage.alarm_value = 0U;                             /* 清除全局报警码，UI 和外部通信后续可读到无报警状态。 */
    SendAlarmMessage(0U);                                     /* 通知蜂鸣任务退出报警蜂鸣模式。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_CLEAR, 0U); /* 输出报警清除调试码。 */
}

static void Handlescan_UpdateRecognizeMessage(ChannelrecognizeMessage_t *message,
                                              uint8_t mapped_model,
                                              uint8_t mapped_tool_model,
                                              uint16_t diameter_tenth,
                                              uint16_t length_tenth,
                                              uint16_t angle_tenth)
{
    if (message == NULL)
    {
        return;
    }

    message->handle_type = mapped_model;
    message->tool_type = mapped_tool_model;
    message->diameter = Handlescan_TenthToUint8(diameter_tenth);
    message->length = (uint16_t)(length_tenth / 10U);
    message->draw = Handlescan_TenthToUint8(angle_tenth);
}

static void Handlescan_RaiseAlarm(uint8_t channel, uint8_t alarm_value)
{
    WorkAlarm_Set(alarm_value);                              /* 所有手柄扫描报警先统一写入 WorkMessage.alarm_value。 */
    SendAlarmMessage(alarm_value);                           /* 校验失败和运行中插拔都进入报警蜂鸣，直到对应清除路径发送 0。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, alarm_value);
}

/*
 * 清空某个通道的认证重试状态。
 * 说明：成功上线、拔出复位或新一轮插入开始时都必须调用，避免上一次失败次数影响下一只手柄。
 */
static void Handlescan_ResetVerifyRetry(uint8_t *retry_count, uint16_t *retry_wait_ticks)
{
    if (retry_count != NULL)
    {
        *retry_count = 0U;                                    /* 清掉连续失败次数，下一次认证重新从第 1 次开始。 */
    }

    if (retry_wait_ticks != NULL)
    {
        *retry_wait_ticks = 0U;                               /* 清掉快速/慢速重试等待计数，避免复用旧等待时间。 */
    }
}

/*
 * 认证链路失败后的统一入口。
 * 规则：
 * 1. 前 HANDLESCAN_VERIFY_RETRY_MAX - 1 次失败只进入快速重试等待，不立刻报警；
 * 2. 达到最大失败次数后才设置最终手柄认证报警；
 * 3. 最终报警保持期间的慢速重试失败不会重复触发蜂鸣，只保持原报警。
 */
static void Handlescan_EnterRetryOrFail(uint8_t channel,
                                        HandlescanStage *stage,
                                        uint8_t *retry_count,
                                        uint16_t *retry_wait_ticks,
                                        uint8_t *last_alarm,
                                        uint8_t alarm_value)
{
    if ((stage == NULL) || (retry_count == NULL) || (retry_wait_ticks == NULL) || (last_alarm == NULL))
    {
        return;                                               /* 运行时参数异常时保持当前状态，避免空指针写入。 */
    }

    if (*retry_count < HANDLESCAN_VERIFY_RETRY_MAX)
    {
        ++(*retry_count);                                     /* 记录本次失败，达到上限后才进入最终报警保持。 */
    }

    if (*retry_count < HANDLESCAN_VERIFY_RETRY_MAX)
    {
        *retry_wait_ticks = 0U;                               /* 快速重试从当前周期重新计时。 */
        *stage = HANDLESCAN_STAGE_RETRY_WAIT;                 /* 保持插入态，稍后重新进入 VERIFY，不要求用户再次插拔。 */
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_VERIFY_RETRY, *retry_count);
        return;
    }

    if (alarm_value != 0U)
    {
        if ((Handlescan_IsHandleVerifyAlarm(*last_alarm) == 0U) || (WorkAlarm_Is(alarm_value) == false))
        {
            *last_alarm = alarm_value;                        /* 第一次达到最终失败时记录报警归属。 */
            Handlescan_RaiseAlarm(channel, *last_alarm);      /* 最终失败才写 WorkMessage.alarm_value 并触发蜂鸣提示。 */
        }
        else
        {
            *last_alarm = alarm_value;                        /* 已经处于同类报警保持时，只刷新归属，不重复触发蜂鸣。 */
        }
    }

    *retry_wait_ticks = 0U;                                   /* 最终报警保持里的慢速自恢复重试也从当前周期重新计时。 */
    *stage = HANDLESCAN_STAGE_VERIFY_FAIL;                    /* 最终失败保持态：继续监视拔出，同时慢速自恢复重试。 */
}

/*
 * 判断当前识别出的通道是否应该同步更新 `WorkMessage.hand_model`。
 * 规则尽量保持保守：
 * 1. 如果另一通道当前不在线，则允许当前通道刷新全局工作手柄型号；
 * 2. 如果另一通道也在线，则只有当前通道正好是选中通道时才允许刷新；
 * 3. 这样可以避免双通道同时在线时，后一次扫描把另一通道的工作型号覆盖掉。
 */
static uint8_t Handlescan_ShouldSyncGlobalHandModel(uint8_t channel)
{
    if (channel == CHANNEL_A)
    {
        return (uint8_t)((WorkMessage.Channel_Bonline == false) || (WorkMessage.channel_work == CHANNEL_A));
    }

    return (uint8_t)((WorkMessage.Channel_Aonline == false) || (WorkMessage.channel_work == CHANNEL_B));
}

/*
 * 把 EEPROM 认证返回码映射为系统报警码。
 * 当前策略：只有 EEPROM 最终校验失败才映射为 WORK_ALARM_HANDLE_MODEL_ERROR。
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
            return WORK_ALARM_HANDLE_MODEL_ERROR;
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
    if (WorkMessage.runflag_work == true)
    {
        Handlescan_RaiseAlarm(channel, HANDLESCAN_ALARM_RUNNING_PLUG);
        K1_OFF();
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
    uint8_t raw_tool_major;                                  /* EEPROM 刀具信息区第 1 个字节，表示刀具主类型。 */
    uint8_t raw_tool_minor;                                  /* EEPROM 刀具信息区第 2 个字节，表示刀具子类型。 */
    uint8_t mapped_model;                                    /* 查表后的系统内部手柄型号值。 */
    uint8_t mapped_tool_model;                               /* 查表后的系统内部刀具类型值。 */
    uint16_t tool_diameter_tenth;                            /* 刀具直径，单位 0.1。 */
    uint16_t tool_length_tenth;                              /* 刀具长度，单位 0.1。 */
    uint16_t tool_angle_tenth;                               /* 刀具角度，单位 0.1。 */
    const HandlescanHandleTypeConfig *handle_type_cfg;       /* 指向命中的手柄类型配置表项。 */
    const HandlescanHandleTypeConfig *tool_type_cfg;         /* 指向命中的刀具类型配置表项。 */

    /*
     * A 通道短接检测脚当前按“低电平表示插入成立”处理。
     * 因此这里读取到 `GPIO_PIN_RESET` 时，表示 A 通道已经检测到短接插入。
     */
    is_inserted = (uint8_t)(Bsp_GpioRead(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_RESET); /* 低电平代表 A 通道短接成立。 */

    if (is_inserted == 0U)
    {
        s_handleA_debounce.in_debounce_ticks = 0U;           /* 一旦检测到拔出候选，就清掉插入去抖计数。 */
        s_a_verify_start_wait_ticks = 0U;                    /* 同时清掉认证前等待计数，避免下次误续跑。 */
        Handlescan_ResetVerifyRetry(&s_a_verify_retry_count, &s_a_verify_retry_wait_ticks); /* 拔出候选出现时，下一轮认证必须重新累计失败次数。 */

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
            Handlescan_ClearChannelAlarm(CHANNEL_A, s_a_last_alarm); /* A 通道坏手柄拔出后，释放本通道手柄型号错误报警。 */
            s_a_stage = HANDLESCAN_STAGE_IDLE;               /* A 通道状态机回到空闲态。 */
            s_a_last_alarm = 0U;                             /* 清掉 A 通道最近一次报警缓存。 */
            Handlescan_ClearChannelState(CHANNEL_A);         /* 清空 A 通道新的在线与识别状态容器。 */
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_A); /* 同步清空 A 通道刀具规格缓存，避免 UI 残留旧值。 */
            SendKeyBehMessage(PLUGunPLUG, SCREENKey_UNPLUG_A); /* 通知新按键行为模块：A 手柄已拔出。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出“离线完成”报文。 */
            (void)Handlescan_HandleRunningPlugAlarm(1U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            Handlescan_BeepOnceIfNoAlarm();                  /* 没有运行中插拔或其他报警时，给 A 通道拔出确认单响。 */
            return;                                          /* A 通道离线处理完成，本轮到此结束。 */
        }

        /*
         * 其余情况下说明当前还没有形成有效上线，直接把 A 通道状态机静默复位即可，
         * 不需要通知 UI，也不需要输出离线报文。
         */
        s_handleA_debounce.out_debounce_ticks = 0U;          /* 静默复位时也要把拔出去抖计数清零。 */
        s_a_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态，等待下一次真实插入。 */
        Handlescan_ClearChannelState(CHANNEL_A);             /* 保证 A 通道新接口状态保持关闭。 */
        Handlescan_ClearToolSpecValues(paoxueSpeciValue_A);  /* 插入未完成时也清空 A 通道刀具规格缓存。 */
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
        Handlescan_ResetVerifyRetry(&s_a_verify_retry_count, &s_a_verify_retry_wait_ticks); /* 新手柄插入从 0 次失败开始认证。 */
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
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS); /* 输出“插入稳定”报文。 */
        return;                                              /* 本轮插入确认完成，等待下一轮进入认证。 */
    }

    /*
     * 插入稳定后的认证前等待阶段。
     * 这里保留 200ms 的稳定时间，避免手柄刚插入时立刻访问 EEPROM 导致首包失败。
     */
    if (s_a_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_a_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_a_verify_start_wait_ticks;                   /* 每个扫描周期把认证前等待计数加一。 */
            return;                                          /* 等待未满 200ms 前，不访问 EEPROM。 */
        }

        s_a_verify_start_wait_ticks = 0U;                    /* 等待完成后，把等待计数器清零。 */
        s_a_stage = HANDLESCAN_STAGE_VERIFY;                 /* 状态机切到“认证执行”阶段。 */
    }

    /*
     * 快速重试等待阶段。
     * 认证或信息读取失败后，先等待一小段时间再重新进 VERIFY，避免 I2C 刚恢复时连续撞总线。
     */
    if (s_a_stage == HANDLESCAN_STAGE_RETRY_WAIT)
    {
        if (s_a_verify_retry_wait_ticks < HANDLESCAN_VERIFY_RETRY_DELAY_TICKS)
        {
            ++s_a_verify_retry_wait_ticks;                   /* 每 10ms 累计一次快速重试等待时间。 */
            return;                                          /* 未到重试间隔前，不访问 EEPROM。 */
        }

        s_a_verify_retry_wait_ticks = 0U;                    /* 快速重试等待结束，清零计数。 */
        s_a_stage = HANDLESCAN_STAGE_VERIFY;                 /* 重新进入认证阶段，不需要重新插拔。 */
    }

    /*
     * A 通道认证阶段。
     * 认证失败后先做有限重试，多次失败后才进入最终报警保持。
     */
    if (s_a_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 认证前先清掉上一轮底层 I2C 调试信息。 */
        verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result); /* 调用 A 通道 I2C2 EEPROM 认证接口。 */
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status); /* 输出当前认证结果码。 */

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         Handlescan_MapVerifyStatusToAlarm(verify_status)); /* 先快速重试，最终失败才报警。 */
            Handlescan_DebugTraceI2cDetail(1U);              /* 输出最近一次底层 I2C 访问细节。 */
            return;                                          /* 本轮不再继续读信息区。 */
        }

        s_a_stage = HANDLESCAN_STAGE_READ_INFO;              /* 认证通过后，进入信息区读取阶段。 */
    }

    /*
     * A 通道信息读取阶段。
     * 认证通过后，先从第 2 页读取手柄类型，再从第 3 页读取刀具类型和规格参数。
     */
    if (s_a_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 读信息区前同样先清调试缓存。 */
        read_status = AT24CS32_ReadBytes_I2C2(HANDLESCAN_INFO_ADDR, s_a_info_buf, HANDLESCAN_INFO_SIZE); /* 从 A 通道 EEPROM 读出 16 字节信息区。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(1U);             /* 输出本轮失败对应的底层 I2C 细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U);                /* 信息区瞬时读取失败只重试，不报手柄型号错误。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_type_major = s_a_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 取出信息区第 1 字节作为手柄主类型。 */
        raw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 取出信息区第 2 字节作为手柄子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 按两个原始字节查找型号配置表。 */
        if (handle_type_cfg == NULL)
        {
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor); /* 输出“手柄类型无法识别”报文。 */
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U);                /* 型号页读到异常值时允许后续重试，避免热插拔瞬间误判。 */
            return;                                         /* 等待重新插拔。 */
        }

        AT24CS32_ClearLastDebugInfo();                       /* 读取刀具页之前，先把调试缓存切到当前这一次访问。 */
        read_status = AT24CS32_ReadBytes_I2C2(HANDLESCAN_TOOL_INFO_ADDR, s_a_tool_info_buf, HANDLESCAN_TOOL_INFO_SIZE); /* 从 A 通道 EEPROM 读出第 3 页刀具信息区。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出刀具信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(1U);             /* 输出本轮失败对应的底层 I2C 细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U);                /* 刀具页瞬时读取失败只重试，不立刻锁死。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_tool_major = s_a_tool_info_buf[HANDLESCAN_TOOL_MAJOR_OFFSET]; /* 取出刀具信息区第 1 字节作为刀具主类型。 */
        raw_tool_minor = s_a_tool_info_buf[HANDLESCAN_TOOL_MINOR_OFFSET]; /* 取出刀具信息区第 2 字节作为刀具子类型。 */
        tool_type_cfg = Handlescan_FindToolTypeConfig(raw_tool_major, raw_tool_minor); /* 按两个原始字节查找刀具配置表。 */
        if (tool_type_cfg == NULL)
        {
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_tool_minor); /* 输出“刀具类型无法识别”报文。 */
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U);                /* 刀具页读到异常值时允许后续重试。 */
            return;                                         /* 等待重新插拔。 */
        }

        tool_diameter_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 解析刀具直径，单位 0.1。 */
        tool_length_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_LENGTH_OFFSET); /* 解析刀具长度，单位 0.1。 */
        tool_angle_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_ANGLE_OFFSET); /* 解析刀具角度，单位 0.1。 */

        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出查表后的系统内部手柄型号值。 */
        mapped_tool_model = tool_type_cfg->mapped_handle_type; /* 取出查表后的系统内部刀具类型值。 */
        MemoryMsgA.hand_model = mapped_model;               /* 把 A 通道记忆的手柄型号同步更新到新接口。 */
        MemoryMsgA.hand_type_raw_major = raw_type_major;    /* 保存 A 通道 EEPROM 原始手柄类型高字节，供 UART2 心跳直接上传。 */
        MemoryMsgA.hand_type_raw_minor = raw_type_minor;    /* 保存 A 通道 EEPROM 原始手柄类型低字节，供 UART2 心跳直接上传。 */
        Handlescan_UpdateRecognizeMessage(&ChannelrecognizeMessageA,
                                          mapped_model,
                                          mapped_tool_model,
                                          tool_diameter_tenth,
                                          tool_length_tenth,
                                          tool_angle_tenth); /* 同步更新 A 通道识别结果。 */
        if (Handlescan_ShouldSyncGlobalHandModel(1U) != 0U)
        {
            WorkMessage.hand_model = mapped_model;          /* 只在 A 通道应接管当前工作态时，同步全局工作手柄型号。 */
            WorkMessage.tool_type = mapped_tool_model;      /* 当前工作通道切到 A 时，同步刀具类型。 */
            WorkMessage.channel_work = CHANNEL_A;           /* 当前工作通道切到 A。 */
        }
        Handlescan_UpdateToolSpecValues(paoxueSpeciValue_A,
                                        tool_diameter_tenth,
                                        tool_length_tenth,
                                        tool_angle_tenth,
                                        mapped_tool_model); /* 按当前 UI 使用的数组格式更新 A 通道刀具规格缓存。 */
        WorkMessage.Channel_Aonline = true;                 /* 置位 A 通道在线标志。 */
        SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A);    /* 通知新接口事件链：A 通道手柄上线。 */
        Handlescan_ClearChannelAlarm(CHANNEL_A, s_a_last_alarm); /* 如果之前已经进入最终失败报警，后续自恢复成功时清掉本通道报警。 */
        Handlescan_ResetVerifyRetry(&s_a_verify_retry_count, &s_a_verify_retry_wait_ticks); /* 上线成功后清空失败重试状态。 */
        s_a_last_alarm = 0U;                                /* 上线成功后清掉最近一次报警缓存。 */
        s_a_stage = HANDLESCAN_STAGE_ONLINE;                /* 状态机切到在线保持态。 */
        Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model); /* 输出 A 通道上线报文。 */
        Handlescan_DebugTraceHandleName(1U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name); /* 输出 A 通道手柄名称报文。 */
        Handlescan_DebugTraceToolInfo(1U,
                                      raw_tool_major,
                                      raw_tool_minor,
                                      tool_type_cfg->handle_name,
                                      tool_diameter_tenth,
                                      tool_length_tenth,
                                      tool_angle_tenth);    /* 输出 A 通道刀具名称和规格报文。 */
        Handlescan_BeepOnceIfNoAlarm();                     /* A 通道认证并上线成功后，给使用者一个确认单响。 */
        return;                                             /* A 通道本轮流程结束，后续等待拔出。 */
    }

    if (s_a_stage == HANDLESCAN_STAGE_VERIFY_FAIL)
    {
        if (s_a_verify_retry_wait_ticks < HANDLESCAN_VERIFY_ALARM_RETRY_TICKS)
        {
            ++s_a_verify_retry_wait_ticks;                   /* 最终报警保持时慢速计时，避免反复高频读 EEPROM。 */
            return;                                          /* 等待 1000ms 后再尝试自恢复认证。 */
        }

        s_a_verify_retry_wait_ticks = 0U;                    /* 慢速自恢复时间到，清掉等待计数。 */
        s_a_stage = HANDLESCAN_STAGE_VERIFY;                 /* 仍插着时重新认证一次，正确手柄可自动恢复上线。 */
        return;
    }

    if (s_a_stage == HANDLESCAN_STAGE_ONLINE)
    {
        return;                                             /* 在线保持态保持静默，只等待拔出。 */
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
    uint8_t raw_tool_major;                                  /* EEPROM 刀具信息区第 1 个字节，表示刀具主类型。 */
    uint8_t raw_tool_minor;                                  /* EEPROM 刀具信息区第 2 个字节，表示刀具子类型。 */
    uint8_t mapped_model;                                    /* 查表映射后的系统内部型号值。 */
    uint8_t mapped_tool_model;                               /* 查表映射后的系统内部刀具型号值。 */
    uint16_t tool_diameter_tenth;                            /* 刀具直径，单位 0.1。 */
    uint16_t tool_length_tenth;                              /* 刀具长度，单位 0.1。 */
    uint16_t tool_angle_tenth;                               /* 刀具角度，单位 0.1。 */
    const HandlescanHandleTypeConfig *handle_type_cfg;       /* 指向命中的 B 通道手柄配置表项。 */
    const HandlescanHandleTypeConfig *tool_type_cfg;         /* 指向命中的 B 通道刀具配置表项。 */

    /*
     * B 通道短接检测脚同样按“低电平表示插入成立”处理。
     * 这里直接在 B 通道函数内实现，不再通过通用 helper 转发。
     */
    is_inserted = (uint8_t)(Bsp_GpioRead(HANDLESCAN_B_SHORT_GPIO, HANDLESCAN_B_SHORT_PIN) == GPIO_PIN_RESET); /* 低电平代表 B 通道短接成立。 */

    if (is_inserted == 0U)
    {
        s_handleB_debounce.in_debounce_ticks = 0U;           /* 一旦检测到拔出候选，就清掉 B 通道插入去抖计数。 */
        s_b_verify_start_wait_ticks = 0U;                    /* 同时清掉 B 通道认证前等待计数。 */
        Handlescan_ResetVerifyRetry(&s_b_verify_retry_count, &s_b_verify_retry_wait_ticks); /* 拔出候选出现时，下一轮认证必须重新累计失败次数。 */

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
            Handlescan_ClearChannelAlarm(CHANNEL_B, s_b_last_alarm); /* B 通道坏手柄拔出后，释放本通道手柄型号错误报警。 */
            s_b_stage = HANDLESCAN_STAGE_IDLE;               /* B 通道状态机回到空闲态。 */
            s_b_last_alarm = 0U;                             /* 清掉 B 通道最近一次报警缓存。 */
            Handlescan_ClearChannelState(CHANNEL_B);         /* 清空 B 通道新的在线与识别状态容器。 */
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_B); /* 同步清空 B 通道刀具规格缓存。 */
            SendKeyBehMessage(PLUGunPLUG, SCREENKey_UNPLUG_B); /* 通知新按键行为模块：B 手柄已拔出。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出 B 通道“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出 B 通道“离线完成”报文。 */
            (void)Handlescan_HandleRunningPlugAlarm(2U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            Handlescan_BeepOnceIfNoAlarm();                  /* 没有运行中插拔或其他报警时，给 B 通道拔出确认单响。 */
            return;                                          /* B 通道离线处理结束。 */
        }

        /*
         * 如果还没有形成有效上线，说明只是插入过程中取消或接触抖动，
         * 此时对 B 通道做静默复位即可。
         */
        s_handleB_debounce.out_debounce_ticks = 0U;          /* 静默复位时，也把 B 通道拔出去抖计数清零。 */
        s_b_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态。 */
        Handlescan_ClearChannelState(CHANNEL_B);             /* 保证 B 通道新接口状态保持关闭。 */
        Handlescan_ClearToolSpecValues(paoxueSpeciValue_B);  /* 插入未完成时也清空 B 通道刀具规格缓存。 */
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
        Handlescan_ResetVerifyRetry(&s_b_verify_retry_count, &s_b_verify_retry_wait_ticks); /* 新手柄插入从 0 次失败开始认证。 */
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
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INSERT_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS); /* 输出 B 通道“插入稳定”报文。 */
        return;                                              /* 本轮到此结束。 */
    }

    /*
     * B 通道认证前等待阶段。
     * 等待时间与 A 通道保持一致，都是 200ms。
     */
    if (s_b_stage == HANDLESCAN_STAGE_WAIT_VERIFY)
    {
        if (s_b_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
        {
            ++s_b_verify_start_wait_ticks;                   /* 每个扫描周期把 B 通道认证前等待计数加一。 */
            return;                                          /* 等待未满 200ms 前，不访问 B 通道 EEPROM。 */
        }

        s_b_verify_start_wait_ticks = 0U;                    /* 等待完成后清零计数器。 */
        s_b_stage = HANDLESCAN_STAGE_VERIFY;                 /* 切到“B 通道认证执行”阶段。 */
    }

    /*
     * B 通道快速重试等待阶段。
     * 认证或信息读取失败后，先等待一小段时间再重新访问 EEPROM。
     */
    if (s_b_stage == HANDLESCAN_STAGE_RETRY_WAIT)
    {
        if (s_b_verify_retry_wait_ticks < HANDLESCAN_VERIFY_RETRY_DELAY_TICKS)
        {
            ++s_b_verify_retry_wait_ticks;                   /* 每 10ms 累计一次快速重试等待时间。 */
            return;                                          /* 未到重试间隔前，不访问 EEPROM。 */
        }

        s_b_verify_retry_wait_ticks = 0U;                    /* 快速重试等待结束，清零计数。 */
        s_b_stage = HANDLESCAN_STAGE_VERIFY;                 /* 重新进入认证阶段，不需要重新插拔。 */
    }

    /*
     * B 通道认证阶段。
     * 认证接口切换为 I2C3，失败后先有限重试，多次失败后才最终报警。
     */
    if (s_b_stage == HANDLESCAN_STAGE_VERIFY)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 认证前清掉上一轮底层 I2C 调试信息。 */
        verify_status = AT24CS32_VerifyCrc_I2C3(&verify_result); /* 调用 B 通道 I2C3 EEPROM 认证接口。 */
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status); /* 输出 B 通道认证结果码。 */

        if (verify_status != AT24CS32_CRC_STATUS_OK)
        {
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         Handlescan_MapVerifyStatusToAlarm(verify_status)); /* 先快速重试，最终失败才报警。 */
            Handlescan_DebugTraceI2cDetail(2U);              /* 输出本轮 B 通道失败的 I2C 细节。 */
            return;                                          /* 停止后续信息区读取。 */
        }

        s_b_stage = HANDLESCAN_STAGE_READ_INFO;              /* 认证通过后进入 B 通道信息区读取阶段。 */
    }

    /*
     * B 通道信息读取阶段。
     * 从 B 通道 EEPROM 先读第 2 页手柄信息，再读第 3 页刀具信息。
     */
    if (s_b_stage == HANDLESCAN_STAGE_READ_INFO)
    {
        AT24CS32_ClearLastDebugInfo();                       /* 读信息区前先清掉底层调试缓存。 */
        read_status = AT24CS32_ReadBytes_I2C3(HANDLESCAN_INFO_ADDR, s_b_info_buf, HANDLESCAN_INFO_SIZE); /* 从 B 通道 EEPROM 读取 16 字节信息区。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出 B 通道信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(2U);             /* 输出最近一次底层 I2C 访问细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U);                /* 信息区瞬时读取失败只重试，不报手柄型号错误。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_type_major = s_b_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 取出 B 通道信息区第 1 字节作为主类型。 */
        raw_type_minor = s_b_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 取出 B 通道信息区第 2 字节作为子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 按两个原始字节查找配置表。 */
        if (handle_type_cfg == NULL)
        {
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_minor); /* 输出 B 通道“手柄类型无法识别”报文。 */
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U);                /* 型号页读到异常值时允许后续重试，避免热插拔瞬间误判。 */
            return;                                         /* 等待重新插拔。 */
        }

        AT24CS32_ClearLastDebugInfo();                       /* 读取刀具页之前，先把调试缓存切到这一次访问。 */
        read_status = AT24CS32_ReadBytes_I2C3(HANDLESCAN_TOOL_INFO_ADDR, s_b_tool_info_buf, HANDLESCAN_TOOL_INFO_SIZE); /* 从 B 通道 EEPROM 读取第 3 页刀具信息区。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出刀具信息区读取失败报文。 */
            Handlescan_DebugTraceI2cDetail(2U);             /* 输出最近一次底层 I2C 访问细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U);                /* 刀具页瞬时读取失败只重试，不立刻锁死。 */
            return;                                         /* 本轮停止后续处理。 */
        }

        raw_tool_major = s_b_tool_info_buf[HANDLESCAN_TOOL_MAJOR_OFFSET]; /* 取出 B 通道刀具主类型。 */
        raw_tool_minor = s_b_tool_info_buf[HANDLESCAN_TOOL_MINOR_OFFSET]; /* 取出 B 通道刀具子类型。 */
        tool_type_cfg = Handlescan_FindToolTypeConfig(raw_tool_major, raw_tool_minor); /* 按原始字节查找刀具配置表。 */
        if (tool_type_cfg == NULL)
        {
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_tool_minor); /* 输出 B 通道“刀具类型无法识别”报文。 */
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U);                /* 刀具页读到异常值时允许后续重试。 */
            return;                                         /* 等待重新插拔。 */
        }

        tool_diameter_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 解析刀具直径，单位 0.1。 */
        tool_length_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_LENGTH_OFFSET); /* 解析刀具长度，单位 0.1。 */
        tool_angle_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_ANGLE_OFFSET); /* 解析刀具角度，单位 0.1。 */

        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出查表后的系统内部手柄型号值。 */
        mapped_tool_model = tool_type_cfg->mapped_handle_type; /* 取出查表后的系统内部刀具类型值。 */
        MemoryMsgB.hand_model = mapped_model;               /* 更新 B 通道记忆的手柄型号到新接口。 */
        MemoryMsgB.hand_type_raw_major = raw_type_major;    /* 保存 B 通道 EEPROM 原始手柄类型高字节，供 UART2 心跳直接上传。 */
        MemoryMsgB.hand_type_raw_minor = raw_type_minor;    /* 保存 B 通道 EEPROM 原始手柄类型低字节，供 UART2 心跳直接上传。 */
        Handlescan_UpdateRecognizeMessage(&ChannelrecognizeMessageB,
                                          mapped_model,
                                          mapped_tool_model,
                                          tool_diameter_tenth,
                                          tool_length_tenth,
                                          tool_angle_tenth); /* 同步更新 B 通道识别结果。 */
        if (Handlescan_ShouldSyncGlobalHandModel(2U) != 0U)
        {
            WorkMessage.hand_model = mapped_model;          /* 只在 B 通道应接管当前工作态时，同步全局工作手柄型号。 */
            WorkMessage.tool_type = mapped_tool_model;      /* 当前工作通道切到 B 时，同步刀具类型。 */
            WorkMessage.channel_work = CHANNEL_B;           /* 当前工作通道切到 B。 */
        }
        Handlescan_UpdateToolSpecValues(paoxueSpeciValue_B,
                                        tool_diameter_tenth,
                                        tool_length_tenth,
                                        tool_angle_tenth,
                                        mapped_tool_model); /* 按当前 UI 使用的数组格式更新 B 通道刀具规格缓存。 */
        WorkMessage.Channel_Bonline = true;                 /* 置位 B 通道在线标志。 */
        SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_B);    /* 通知新接口事件链：B 通道手柄上线。 */
        Handlescan_ClearChannelAlarm(CHANNEL_B, s_b_last_alarm); /* 如果之前已经进入最终失败报警，后续自恢复成功时清掉本通道报警。 */
        Handlescan_ResetVerifyRetry(&s_b_verify_retry_count, &s_b_verify_retry_wait_ticks); /* 上线成功后清空失败重试状态。 */
        s_b_last_alarm = 0U;                                /* 上线成功后清掉 B 通道最近一次报警缓存。 */
        s_b_stage = HANDLESCAN_STAGE_ONLINE;                /* 状态机切到 B 通道在线保持态。 */
        Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model); /* 输出 B 通道上线报文。 */
        Handlescan_DebugTraceHandleName(2U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name); /* 输出 B 通道手柄名称报文。 */
        Handlescan_DebugTraceToolInfo(2U,
                                      raw_tool_major,
                                      raw_tool_minor,
                                      tool_type_cfg->handle_name,
                                      tool_diameter_tenth,
                                      tool_length_tenth,
                                      tool_angle_tenth);    /* 输出 B 通道刀具名称和规格报文。 */
        Handlescan_BeepOnceIfNoAlarm();                     /* B 通道认证并上线成功后，给使用者一个确认单响。 */
        return;                                             /* B 通道本轮处理结束。 */
    }

    if (s_b_stage == HANDLESCAN_STAGE_VERIFY_FAIL)
    {
        if (s_b_verify_retry_wait_ticks < HANDLESCAN_VERIFY_ALARM_RETRY_TICKS)
        {
            ++s_b_verify_retry_wait_ticks;                   /* 最终报警保持时慢速计时，避免反复高频读 EEPROM。 */
            return;                                          /* 等待 1000ms 后再尝试自恢复认证。 */
        }

        s_b_verify_retry_wait_ticks = 0U;                    /* 慢速自恢复时间到，清掉等待计数。 */
        s_b_stage = HANDLESCAN_STAGE_VERIFY;                 /* 仍插着时重新认证一次，正确手柄可自动恢复上线。 */
        return;
    }

    if (s_b_stage == HANDLESCAN_STAGE_ONLINE)
    {
        return;                                             /* 在线保持态保持静默，只等待拔出。 */
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
	Kernel_TaskCreate(&HANDLESCANTaskHandle, HANDLESCANTaskFunc);
	Kernel_TaskStart(&HANDLESCANTaskHandle, KERNEL_TASK_ALWAYS, 10);
}
