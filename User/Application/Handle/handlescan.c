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
#include "kernel_scheduler.h"
#include "datahand.h"
#include "Pubinterface.h"
#include "sscUIDP.h"
#include "sscBEEP.h"
#include "sscKEYBH.h"
#include "sscRFID.h"
#include "external_comm_task.h"
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
#define HANDLESCAN_RFID_VERIFY_RETRY_MAX      2U  /* RFID 刀具头首次等待只保留 2 轮，900ms+200ms+900ms 约 2 秒后结束等待。 */

#define HANDLESCAN_INSERT_DEBOUNCE_TICKS      (HANDLESCAN_INSERT_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_REMOVE_DEBOUNCE_TICKS      (HANDLESCAN_REMOVE_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_START_DELAY_TICKS   (HANDLESCAN_VERIFY_START_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_RETRY_DELAY_TICKS   (HANDLESCAN_VERIFY_RETRY_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_ALARM_RETRY_TICKS   (HANDLESCAN_VERIFY_ALARM_RETRY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_RFID_WAIT_TIMEOUT_MS       900U
#define HANDLESCAN_RFID_MONITOR_PERIOD_MS     200U  /* RFID 在线监测按 1 秒一轮执行，发读后下一轮仍未确认即可把用户可见清除时间压到约 2 秒。 */
#define HANDLESCAN_RFID_MISS_MAX              5U
#define HANDLESCAN_RFID_WAIT_TIMEOUT_TICKS    (HANDLESCAN_RFID_WAIT_TIMEOUT_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_RFID_MONITOR_PERIOD_TICKS  (HANDLESCAN_RFID_MONITOR_PERIOD_MS / HANDLESCAN_TASK_PERIOD_MS)

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
 * EEPROM 中 Page4 初始值信息区定义。
 * Page4 对应 AT24CS32 驱动页下标 3，读取整页可以同步校验页尾，避免默认速度、流量和报警阈值读到损坏数据。
 * Page4 的 16 位字段按业务说明使用小端格式，默认注水流量按 0.1 存储，写入泵业务流量前需要除以 10。
 */
#define HANDLESCAN_INITIAL_INFO_PAGE_INDEX    3U
#define HANDLESCAN_INITIAL_DEFAULT_FLOW_OFFSET 0U
#define HANDLESCAN_INITIAL_DEFAULT_SPEED_OFFSET 2U
#define HANDLESCAN_INITIAL_MIN_SPEED_OFFSET   4U
#define HANDLESCAN_INITIAL_MAX_SPEED_OFFSET   6U
#define HANDLESCAN_INITIAL_DIRECTION_OFFSET   8U
#define HANDLESCAN_INITIAL_FREQ_OFFSET        9U
#define HANDLESCAN_INITIAL_FOR_ALARM_OFFSET   10U
#define HANDLESCAN_INITIAL_REV_ALARM_OFFSET   12U
#define HANDLESCAN_INITIAL_OSC_ALARM_OFFSET   14U
#define HANDLESCAN_INITIAL_FLOW_SCALE         10U
#define HANDLESCAN_INITIAL_FLOW_MAX           70U

/*
 * EEPROM 中 Page6 多档位速度区定义。
 * Page6 对应 AT24CS32 驱动页下标 5，当前只读取三组 16 位速度字段作为正转、反转、往复的按键慢档步进。
 * Page6 的速度字段按上位机布局使用大端格式；读不到或字段为 0 时，步进回退到 1000，避免新屏速度按键无响应。
 */
#define HANDLESCAN_SPEED_STEP_PAGE_INDEX      5U
#define HANDLESCAN_SPEED_STEP_GEAR_COUNT_OFFSET 0U
#define HANDLESCAN_SPEED_STEP_GEAR1_OFFSET    1U
#define HANDLESCAN_SPEED_STEP_GEAR2_OFFSET    5U
#define HANDLESCAN_SPEED_STEP_GEAR3_OFFSET    9U
#define HANDLESCAN_SPEED_STEP_MAX_GEAR        3U
#define HANDLESCAN_SPEED_STEP_FALLBACK        1000U

/*
 * 手柄扫描报警码定义。
 * 1. 13：运行中插拔报警，沿用旧逻辑；
 * 2. EEPROM 最终校验失败按 A/B 通道上报 WORK_ALARM_HANDLE_MODEL_ERROR_A/B/AB。
 */
#define HANDLESCAN_ALARM_RUNNING_PLUG         13U
#define HANDLESCAN_TRANSIENT_ALARM_MS         3000U
#define HANDLESCAN_TRANSIENT_SCREEN_TICKS     (HANDLESCAN_TRANSIENT_ALARM_MS / HANDLESCAN_TASK_PERIOD_MS)

/*
 * 串口调试步骤码定义。
 * `HANDLESCAN_TRACE_ENABLE` 是当前 handlescan 驱动的总报文开关：
 * 1. 设为 `1U` 时，输出 `HS`、`HSDBG`、`HSNAME` 基础调试报文；
 * 2. 设为 `0U` 时，基础调试报文都会被静默处理，业务状态机保持不变；
 * 3. 刀具扩展报文单独受 `HANDLESCAN_TOOL_TRACE_ENABLE` 控制，方便现场按需开启。
 */
#define HANDLESCAN_TRACE_ENABLE               0U
#define HANDLESCAN_TOOL_TRACE_ENABLE          0U

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
    HANDLESCAN_STAGE_IDLE = 0,//空闲/待机状态,等待插入
    HANDLESCAN_STAGE_DEBOUNCE_IN,//插入消抖状态,硬件消抖
    HANDLESCAN_STAGE_WAIT_VERIFY,//等待验证状态，身份验证
    HANDLESCAN_STAGE_VERIFY,//验证中状态,正在验证
    HANDLESCAN_STAGE_READ_INFO,//读取信息状态,手柄型号，内容信息
    HANDLESCAN_STAGE_WAIT_RFID_TOOL,//如果是RFID工具手柄，等待RFID标签识别完成
    HANDLESCAN_STAGE_ONLINE,//手柄正式接入系统，可以正常工作
    HANDLESCAN_STAGE_DEBOUNCE_OUT,//检测到拔出信号，进行硬件消抖
    HANDLESCAN_STAGE_RETRY_WAIT,//验证或读取失败，等待一段时间后重试
    HANDLESCAN_STAGE_VERIFY_FAIL//身份验证失败，手柄被拒绝接入
} HandlescanStage;

typedef enum
{
    HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3 = 0,
    HANDLESCAN_TOOL_SOURCE_RFID_EPC,
    HANDLESCAN_TOOL_SOURCE_RFID_USER
} HandlescanToolSource;

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
    {0x6B, 0x06, PXBB_ONLINES,  "PXBB"},
    {0x6B, 0x07, LGZ_I_ONLINES, "LGZ_I"},                 /* 颅骨钻一型预留，先按 Page2 顺序占位。 */
    {0x6B, 0x08, LGZ_II_ONLINES, "LGZ_II"},               /* 颅骨钻二型预留，后续如协议变更只改本表。 */
    {0x6B, 0x09, KSZ_I_ONLINES, "KSZ_I"},                 /* 克氏针一型预留，识别后供 UI 显示。 */
    {0x6B, 0x0A, KSZ_II_ONLINES, "KSZ_II"},               /* 克氏针二型预留，识别后供 UI 显示。 */
    {0x6B, 0x0B, KXZ_I_ONLINES, "KXZ_I"},                 /* 空心钻一型预留，沿用当前扫描状态机。 */
    {0x6B, 0x0C, KXZ_II_ONLINES, "KXZ_II"},               /* 空心钻二型预留，沿用当前扫描状态机。 */
    {0x6B, 0x0D, COMMON_SOCKET_ONLINES, "COMMON_SOCKET"}, /* 公共接头没有实体键，只作为在线类型和 UI 占位。 */
    {0x6B, 0x0E, EMBD_ONLINES, "EMBD"},                   /* EMBD 6万增速手柄按普通电动手柄上线，不额外开放实体键控制。 */
    {0x6B, 0x0F, EMBC_ONLINES, "EMBC"}                    /* EMBC 新增普通电动手柄，按 EEPROM 第二页 0x6B/0x0F 识别。 */
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
    {0x7C, 0x06, MX_YIM16_ONLINES, "MXYTM16"},
    {0x7C, 0x07, LGZ_I_ONLINES, "LGZ_I_TOOL"},                 /* 刀具类型预留，先按 Page3 顺序占位。 */
    {0x7C, 0x08, LGZ_II_ONLINES, "LGZ_II_TOOL"},               /* 颅骨钻二型刀具预留，后续协议变更只改本表。 */
    {0x7C, 0x09, KSZ_I_ONLINES, "KSZ_I_TOOL"},                 /* 克氏针一型刀具预留，识别后供 UI 和上位机读取。 */
    {0x7C, 0x0A, KSZ_II_ONLINES, "KSZ_II_TOOL"},               /* 克氏针二型刀具预留，识别后供 UI 和上位机读取。 */
    {0x7C, 0x0B, KXZ_I_ONLINES, "KXZ_I_TOOL"},                 /* 空心钻一型刀具预留，沿用 Page3 解析流程。 */
    {0x7C, 0x0C, KXZ_II_ONLINES, "KXZ_II_TOOL"},               /* 空心钻二型刀具预留，沿用 Page3 解析流程。 */
    {0x7C, 0x0D, COMMON_SOCKET_ONLINES, "COMMON_SOCKET_TOOL"}  /* 公共接头刀具位保留，便于完整保存类型编号。 */
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
static uint8_t s_a_initial_info_buf[AT24CS32_PAGE_SIZE] = {0U}; /* A 通道 Page4 初始值页缓存，保存默认速度、流量和阈值。 */
static uint8_t s_a_speed_step_buf[AT24CS32_PAGE_SIZE] = {0U}; /* A 通道 Page6 多档位速度页缓存，用来生成当前方向调速步进。 */
static HandlescanToolSource s_a_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* A 通道刀具信息来源，默认使用 EEPROM 第三页。 */
static uint16_t s_a_rfid_wait_ticks = 0U; /* A 通道等待 RFID 结果的 10ms 计数。 */
static uint16_t s_a_rfid_monitor_ticks = 0U; /* A 通道在线后低频监测 RFID 的 10ms 计数。 */
static uint16_t s_a_rfid_last_sequence = 0U; /* A 通道已处理的最新 RFID 结果序号。 */
static uint16_t s_a_rfid_last_presence_sequence = 0U; /* A 通道最近一次确认读到 RFID 标签的存在序号。 */
static uint8_t s_a_rfid_miss_count = 0U; /* A 通道在线监测连续未读到标签的次数。 */
static uint8_t s_a_rfid_monitor_pending = 0U; /* A 通道上一轮在线监测请求是否正在等待 presence 变化。 */
static uint8_t s_a_rfid_tool_online = 0U; /* A 通道 RFID 刀具头在线边沿标志，只在上线/离线变化时触发提示音。 */

/* B 通道运行时状态变量。 */
static HandlescanStage s_b_stage = HANDLESCAN_STAGE_IDLE;
static HandlescanDebounce s_handleB_debounce = {0U, 0U};
static uint8_t s_b_verify_start_wait_ticks = 0U;
static uint8_t s_b_last_alarm = 0U;
static uint8_t s_b_verify_retry_count = 0U;
static uint16_t s_b_verify_retry_wait_ticks = 0U;
static uint8_t s_b_info_buf[HANDLESCAN_INFO_SIZE] = {0U};
static uint8_t s_b_tool_info_buf[HANDLESCAN_TOOL_INFO_SIZE] = {0U};
static uint8_t s_b_initial_info_buf[AT24CS32_PAGE_SIZE] = {0U}; /* B 通道 Page4 初始值页缓存，保存默认速度、流量和阈值。 */
static uint8_t s_b_speed_step_buf[AT24CS32_PAGE_SIZE] = {0U}; /* B 通道 Page6 多档位速度页缓存，避免 B 通道调速误用 A 通道步进。 */
static HandlescanToolSource s_b_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* B 通道刀具信息来源，默认使用 EEPROM 第三页。 */
static uint16_t s_b_rfid_wait_ticks = 0U; /* B 通道等待 RFID 结果的 10ms 计数。 */
static uint16_t s_b_rfid_monitor_ticks = 0U; /* B 通道在线后低频监测 RFID 的 10ms 计数。 */
static uint16_t s_b_rfid_last_sequence = 0U; /* B 通道已处理的最新 RFID 结果序号。 */
static uint16_t s_b_rfid_last_presence_sequence = 0U; /* B 通道最近一次确认读到 RFID 标签的存在序号。 */
static uint8_t s_b_rfid_miss_count = 0U; /* B 通道在线监测连续未读到标签的次数。 */
static uint8_t s_b_rfid_monitor_pending = 0U; /* B 通道上一轮在线监测请求是否正在等待 presence 变化。 */
static uint8_t s_b_rfid_tool_online = 0U; /* B 通道 RFID 刀具头在线边沿标志，只在上线/离线变化时触发提示音。 */

/* 运行中另一路手柄校验失败时，屏幕提示只保持 3 秒，不写 WorkMessage，避免影响当前工作通道。 */
static uint16_t s_transient_screen_alarm_ticks = 0U;
static uint8_t s_transient_screen_alarm_value = 0U;

static void Handlescan_ClearChannelAlarm(uint8_t channel, uint8_t alarm_value);
static void Handlescan_ClearToolSpecValues(uint32_t *spec_values);
static void Handlescan_ReloadSpeedStepMessage(uint8_t channel, ChannelrecognizeMessage_t *message);
static void Handlescan_ClearOnlineRfidTool(uint8_t channel,
                                           HandlescanToolSource tool_source,
                                           ChannelrecognizeMessage_t *message,
                                           uint32_t *spec_values,
                                           uint8_t mapped_model,
                                           uint8_t raw_type_major,
                                           uint8_t raw_type_minor,
                                           uint16_t *last_sequence,
                                           uint16_t *last_presence_sequence,
                                           uint8_t *miss_count,
                                           uint8_t *monitor_pending,
                                           uint8_t *tool_online,
                                           uint8_t plug_key);
static void Handlescan_EnterRetryOrFail(uint8_t channel,
                                        HandlescanStage *stage,
                                        uint8_t *retry_count,
                                        uint16_t *retry_wait_ticks,
                                        uint8_t *last_alarm,
                                        uint8_t set_alarm);
static void Handlescan_EnterRfidRetryOrFail(uint8_t channel,
                                            HandlescanStage *stage,
                                            uint8_t *retry_count,
                                            uint16_t *retry_wait_ticks,
                                            uint8_t *last_alarm,
                                            uint8_t set_alarm);
static void Handlescan_BeepOnceIfNoAlarm(void);

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
 * 函数功能：根据 EEPROM 第二页识别出的手柄基座型号判断刀具信息来源。
 * 输入参数：mapped_handle_type 为手柄类型映射后的系统内部型号。
 * 返回参数：EEPROM 第三页、RFID EPC 或 RFID USER。
 */
static HandlescanToolSource Handlescan_GetToolSource(uint8_t mapped_handle_type)
{
    if (mapped_handle_type == COMMON_SOCKET_ONLINES)
    {
        return HANDLESCAN_TOOL_SOURCE_RFID_EPC; /* 公共接头式可拆手柄，刀具头信息来自 RFID EPC。 */
    }

    if ((mapped_handle_type == PXBA_ONLINES) || (mapped_handle_type == PXBB_ONLINES))
    {
        return HANDLESCAN_TOOL_SOURCE_RFID_USER; /* 分体式可拆手柄当前只包含 PXBA/PXBB，刀具头信息来自 RFID USER。 */
    }

    return HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 其它不可拆普通手柄继续读取 EEPROM 第三页。 */
}

/*
 * 函数功能：把 EEPROM 第三页旧刀具型号转换成业务刀具类型。
 * 输入参数：raw_tool_type 为刀具表查到的旧内部型号。
 * 返回参数：PLANER/GRINDH 或原值；PXP 归一为 PLANER，PXM 归一为 GRINDH。
 */
static uint8_t Handlescan_MapRawToolTypeToBusinessType(uint8_t raw_tool_type)
{
    if (raw_tool_type == PX_YIP_ONLINES)
    {
        return PLANER; /* PXP 一体刨按最新规则作为 PLANER，开放往复能力。 */
    }

    if (raw_tool_type == PX_YIM_ONLINES)
    {
        return GRINDH; /* PXM 一体磨按最新规则作为 GRINDH，关闭往复/开口定位。 */
    }

    return raw_tool_type; /* 其它 EEPROM 刀具暂时保持原值，避免扩大本次规则变更范围。 */
}

/*
 * 函数功能：把 PXBA/PXBB 的 RFID USER 减速比字段转换成业务刀具类型。
 * 输入参数：reduction_ratio 为 USER byte4 原始减速比。
 * 返回参数：减速比为 2 时返回 GRINDH，其它值返回 PLANER。
 */
static uint8_t Handlescan_MapUserReductionRatioToBusinessType(uint8_t reduction_ratio)
{
    if (reduction_ratio == 2U)
    {
        return GRINDH; /* 用户确认 USER 减速比 2 表示磨头类刀具。 */
    }

    return PLANER; /* USER 减速比非 2 表示刨刀类刀具，支持往复和 PXBA/PXBB 开口定位。 */
}

/*
 * 函数功能：把 handlescan 的刀具来源转换成 RFID 读取来源。
 * 输入参数：tool_source 为手柄扫描判断出的刀具来源。
 * 返回参数：RFID EPC、RFID USER 或 NONE。
 */
static RfidReadSource_t Handlescan_ToRfidSource(HandlescanToolSource tool_source)
{
    if (tool_source == HANDLESCAN_TOOL_SOURCE_RFID_EPC)
    {
        return RFID_READ_SOURCE_EPC; /* 公共接头读取 EPC。 */
    }

    if (tool_source == HANDLESCAN_TOOL_SOURCE_RFID_USER)
    {
        return RFID_READ_SOURCE_USER; /* PXBA/PXBB 分体式读取 USER。 */
    }

    return RFID_READ_SOURCE_NONE; /* EEPROM 第三页来源不需要 RFID。 */
}

/*
 * 函数功能：把 RFID 协议方向值转换为工程内部方向值。
 * 输入参数：raw_direction 为 RFID 标签方向字段，1 正转、2 反转、3 往复。
 * 返回参数：ZZDIR/FZDIR/OSCDIR。
 */
static uint8_t Handlescan_ParseRfidDirection(uint8_t raw_direction)
{
    if (raw_direction == 2U)
    {
        return FZDIR; /* 协议 2 表示反转刀具。 */
    }

    if (raw_direction == 3U)
    {
        return OSCDIR; /* 协议 3 表示往复刀具。 */
    }

    return ZZDIR; /* 协议 1 或异常值默认正转，避免无效方向影响启动。 */
}

/*
 * 函数功能：把 RFID 1 字节速度转换为当前工程速度单位。
 * 输入参数：speed_k 表示以 k rpm 保存的速度，例如 0x8C 表示 140k。
 * 返回参数：工程内部速度值，按 1000 rpm 递增。
 */
static uint16_t Handlescan_RfidSpeedToWorkSpeed(uint8_t speed_k)
{
    uint32_t speed = (uint32_t)speed_k * 1000UL; /* RFID 协议示例说明 0x8C 表示 140k，换算为 140000。 */

    if (speed > 0xFFFFUL)
    {
        speed = 0xFFFFUL; /* 通道识别结构速度字段为 16 位，超过范围时钳位。 */
    }

    return (uint16_t)speed; /* 返回可写入 speed_xxxdefault/speed_xxxmax 的速度值。 */
}

/*
 * 函数功能：把 RFID 齿轮比/减速比字段转换为工程内部 tool_reduction_ratio。
 * 输入参数：ratio_hi 和 ratio_lo 为 EPC 2 字节齿轮比，单字节模式时 ratio_lo 传 0。
 * 返回参数：高 16 位表示增速，低 16 位表示减速。
 */
static uint32_t Handlescan_BuildRfidReductionRatio(uint8_t ratio_hi, uint8_t ratio_lo)
{
    uint16_t raw_ratio = (uint16_t)(((uint16_t)ratio_hi << 8) | ratio_lo); /* 保存 EPC 原始两字节齿轮比。 */
    uint16_t ratio_value = (uint16_t)(raw_ratio & 0x0FFFU); /* 去掉高位方向标记后保留比例数值。 */

    if ((ratio_hi & 0xF0U) == 0xF0U)
    {
        return (uint32_t)ratio_value; /* 高位 F 表示减速，写入低 16 位。 */
    }

    if ((ratio_hi & 0xF0U) == 0x00U)
    {
        return ((uint32_t)ratio_value << 16); /* 高位 0 表示增速，写入高 16 位。 */
    }

    return (uint32_t)raw_ratio; /* 未知格式保留原始值到低 16 位，便于上位机和售后判断。 */
}

/*
 * 函数功能：把 RFID 读取结果解析成通道识别缓存和屏幕刀具规格缓存。
 * 输入参数：channel 为 A/B 通道；message 为通道识别缓存；spec_values 为屏幕规格缓存；mapped_model 和 raw_type_* 来自 EEPROM 第二页；rfid_result 为 RFID 原始标签结果。
 * 返回参数：true 表示已写入识别缓存，false 表示 RFID 数据不符合当前来源。
 */
static bool Handlescan_ApplyRfidToolResult(uint8_t channel,
                                           ChannelrecognizeMessage_t *message,
                                           uint32_t *spec_values,
                                           uint8_t mapped_model,
                                           uint8_t raw_type_major,
                                           uint8_t raw_type_minor,
                                           const RfidToolResult_t *rfid_result)
{
    const uint8_t *payload;      /* 指向 RFID 提取出的原始标签数据。 */
    uint8_t tool_model;          /* RFID 标签中的刀具型号或代号结构字段。 */
    uint8_t diameter;            /* RFID 标签中的直径字段。 */
    uint8_t length;              /* RFID 标签中的长度字段。 */
    uint8_t angle;               /* RFID 标签中的弯曲角度字段。 */
    uint16_t default_speed;      /* RFID 标签转换后的默认速度。 */
    uint16_t max_speed;          /* RFID 标签转换后的最大速度。 */
    uint16_t min_speed;          /* RFID 标签转换后的最小速度。 */
    uint32_t reduction_ratio = 0U; /* RFID 标签解析出的完整减速比，EPC 需要保留 32 位增/减速方向信息。 */
    uint8_t default_flow;        /* RFID 标签中的默认泵流量。 */
    uint8_t direction;           /* RFID 标签中的方向字段。 */
    uint8_t business_tool_type = 0U; /* 业务层使用的刀具能力类型，和 RFID 原始型号分开保存。 */
    uint16_t keep_speed_zzstep;  /* 保存基座阶段已从 Page6 读出的正转调速步进，防止 memset 后丢失。 */
    uint16_t keep_speed_fzstep;  /* 保存基座阶段已从 Page6 读出的反转调速步进，RFID 标签不直接携带该参数。 */
    uint16_t keep_speed_oscstep; /* 保存基座阶段已从 Page6 读出的往复调速步进，供新屏快慢调速键使用。 */

    if ((message == NULL) || (spec_values == NULL) || (rfid_result == NULL) || (rfid_result->valid == false))
    {
        return false; /* 防御空指针和无效 RFID 结果，避免错误上线。 */
    }

    if (rfid_result->channel != channel)
    {
        return false; /* RFID 结果必须属于当前通道，避免 A/B 刀具头数据串用。 */
    }

    keep_speed_zzstep = message->speed_zzstep; /* RFID 结果刷新前先暂存正转步进，避免清识别缓存时丢掉 EEPROM Page6 参数。 */
    keep_speed_fzstep = message->speed_fzstep; /* RFID 结果刷新前先暂存反转步进，保证换刀具头后仍按基座配置调速。 */
    keep_speed_oscstep = message->speed_oscstep; /* RFID 结果刷新前先暂存往复步进，PLANER 刀具切到往复后可继续用新屏调速。 */
    payload = rfid_result->payload; /* 后续按 EPC/USER 来源解释同一份原始标签数据。 */
    memset(message, 0, sizeof(*message)); /* RFID 刀具头重新识别时先清旧识别缓存，避免旧 EEPROM 字段残留。 */

    message->handle_type = mapped_model; /* 手柄基座型号仍来自 EEPROM 第二页，RFID 只更新刀具头信息。 */
    message->hand_type_raw_major = raw_type_major; /* 保存 EEPROM 第二页原始主类型，上位机心跳仍按基座身份显示。 */
    message->hand_type_raw_minor = raw_type_minor; /* 保存 EEPROM 第二页原始子类型，便于区分公共接头/PXBA/PXBB。 */
    message->freq_min = FreqMin; /* RFID 当前不提供频率下限时沿用工程默认。 */
    message->freq_max = FreqMax; /* RFID 当前不提供频率上限时沿用工程默认。 */

    if (rfid_result->source == RFID_READ_SOURCE_EPC)
    {
        if (rfid_result->payload_length != RFID_PAYLOAD_EPC_LENGTH)
        {
            return false; /* 公共接头必须是 12 字节 EPC 标签数据。 */
        }

        tool_model = payload[0]; /* EPC byte0：刀具型号。 */
        diameter = payload[1]; /* EPC byte1：刀头直径。 */
        length = payload[2]; /* EPC byte2：刀具长度。 */
        angle = payload[3]; /* EPC byte3：弯曲角度。 */
        max_speed = Handlescan_RfidSpeedToWorkSpeed(payload[6]); /* EPC byte6：最高速度，按 k rpm 转工程速度。 */
        min_speed = Handlescan_RfidSpeedToWorkSpeed(payload[7]); /* EPC byte7：最低速度，按 k rpm 转工程速度。 */
        default_speed = Handlescan_RfidSpeedToWorkSpeed(payload[8]); /* EPC byte8：上电默认速度。 */
        default_flow = payload[9]; /* EPC byte9：注水泵默认流量。 */
        direction = Handlescan_ParseRfidDirection(payload[10]); /* EPC byte10：方向能力。 */
        reduction_ratio = Handlescan_BuildRfidReductionRatio(payload[4], payload[5]); /* EPC byte4~5：齿轮比字段，解析为工程内部 32 位增/减速比。 */
        business_tool_type = GRINDH; /* 公共接头/GYJT EPC 默认按磨头类刀具处理，不开放往复和开口定位。 */
        message->meioticratio = (uint8_t)(reduction_ratio & 0xFFU); /* 旧 8 位字段继续保留低 8 位，兼容历史开口逻辑。 */
        message->overloadThresholdFor = payload[11]; /* EPC byte11：电流阈值，当前按原始值保存。 */
        message->overloadThresholdRev = payload[11]; /* 反转阈值沿用同一 RFID 电流阈值。 */
        message->overloadThresholdOSC = payload[11]; /* 往复阈值沿用同一 RFID 电流阈值。 */
        message->freq_default = 0U; /* EPC 未定义频率字段，默认 0。 */
    }
    else if (rfid_result->source == RFID_READ_SOURCE_USER)
    {
        if (rfid_result->payload_length != RFID_PAYLOAD_USER_LENGTH)
        {
            return false; /* 分体式必须是 16 字节 USER 标签数据。 */
        }

        tool_model = payload[0]; /* USER byte0：代号+结构。 */
        diameter = payload[1]; /* USER byte1：刀头直径。 */
        length = payload[2]; /* USER byte2：刀具长度。 */
        angle = payload[3]; /* USER byte3：弯曲角度。 */
        max_speed = (payload[8]*500); /* USER byte8：转速，作为上限使用。 */
        min_speed = 0U; /* USER 未提供最低速度，按 0 保存。 */
        default_speed = (payload[9]*500); /* USER byte9：默认转速。 */
        default_flow = payload[11]; /* USER byte11：泵速度。 */
        direction = Handlescan_ParseRfidDirection(payload[5]); /* USER byte5：方向。 */
        message->meioticratio = payload[4]; /* USER byte4：减速比，按原始 1 字节保存。 */
        reduction_ratio = (uint32_t)payload[4]; /* USER 只有 1 字节减速比，扩展为 32 位后统一进入通道记忆。 */
        business_tool_type = Handlescan_MapUserReductionRatioToBusinessType(payload[4]); /* PXBA/PXBB USER 按减速比归一为 GRINDH/PLANER。 */
        if(business_tool_type==PLANER)
        {
            min_speed=500;
           // max_speed=6000;
          //  default_speed=5000;
          message->meioticratio=5;
          reduction_ratio=5;
        }
        else if(business_tool_type==GRINDH)
        {
            min_speed=3000;
             message->meioticratio=2;
             reduction_ratio=2;
           // max_speed=13000;
           // default_speed=10000;
        }
        message->freq_default = payload[10]; /* USER byte10：频率。 */
        message->overloadThresholdFor = 0U; /* USER 未定义电流阈值，保持 0。 */
        message->overloadThresholdRev = 0U; /* USER 未定义反转阈值，保持 0。 */
        message->overloadThresholdOSC = 0U; /* USER 未定义往复阈值，保持 0。 */
    }
    else
    {
        return false; /* 只有 EPC/USER 两类结果能更新刀具头信息。 */
    }

    if ((max_speed != 0U) && (default_speed > max_speed))
    {
        default_speed = max_speed; /* RFID 默认速度不能超过 RFID 上限。 */
    }
    if (default_speed < min_speed)
    {
        default_speed = min_speed; /* RFID 默认速度不能低于 RFID 下限。 */
    }
    if(business_tool_type==PLANER)
    message->run_direction=OSCDIR; /* 往复刀具默认方向先按 RFID 方向字段，后续开口定位逻辑会根据业务类型调整。 */
    else
    {
        message->run_direction = ZZDIR;
    }
    message->tool_type = business_tool_type; /* 保存业务刀具类型，屏幕/方向/开口定位按 PLANER/GRINDH 判断。 */
    message->raw_tool_type = tool_model; /* 保存 RFID 原始刀具型号，供上位机扩展和售后核对标签原值。 */
    message->diameter = diameter; /* 保存 RFID 直径。 */
    message->length = length; /* 保存 RFID 长度。 */
    message->draw = angle; /* 保存 RFID 弯曲角度。 */
    message->tool_reduction_ratio = reduction_ratio; /* 保存完整 RFID 减速比，供心跳扩展和开口定位逻辑读取。 */
    message->default_injection_flow = default_flow; /* 保存 RFID 默认注水泵流量。 */
    message->speed_min = min_speed; /* 保存通用最小速度。 */
    message->speed_max = max_speed; /* 保存通用最大速度。 */
    message->speed_zzmin = min_speed; /* 正转下限使用 RFID 下限。 */
    message->speed_zzmax = max_speed; /* 正转上限使用 RFID 上限。 */
    message->speed_fzmin = min_speed; /* 反转下限使用 RFID 下限。 */
    message->speed_fzmax = max_speed; /* 反转上限使用 RFID 上限。 */
    message->speed_oscmin = min_speed; /* 往复下限使用 RFID 下限。 */
    message->speed_oscmax = max_speed; /* 往复上限使用 RFID 上限。 */
    message->speed_zzdefault = default_speed; /* 正转默认速度来自 RFID。 */
    message->speed_fzdefault = default_speed; /* 反转默认速度来自 RFID。 */
    message->speed_oscdefault = default_speed; /* 往复默认速度来自 RFID。 */
    message->speed_zzstep = keep_speed_zzstep; /* RFID 成功后恢复正转步进，优先使用基座 EEPROM Page6 的方向步进。 */
    message->speed_fzstep = keep_speed_fzstep; /* RFID 成功后恢复反转步进，避免分体式刀具头上线后退回固定默认值。 */
    message->speed_oscstep = keep_speed_oscstep; /* RFID 成功后恢复往复步进，保证 PLANER 往复调速和正反转同源。 */
    if (message->speed_zzstep == 0U)
    {
        message->speed_zzstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，正转调速按 1000 兜底。 */
    }
    if (message->speed_fzstep == 0U)
    {
        message->speed_fzstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，反转调速按 1000 兜底。 */
    }
    if (message->speed_oscstep == 0U)
    {
        message->speed_oscstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，往复调速按 1000 兜底。 */
    }
    //message->run_direction = direction; /* 默认方向来自 RFID 标签。 */

    if (rfid_result->source == RFID_READ_SOURCE_EPC)
    {
        spec_values[0] = (uint32_t)length; /* 公共接头 EPC 规格按标签原始字节保存，0x60 后续直接显示为 96mm。 */
        spec_values[1] = (uint32_t)diameter; /* 公共接头 EPC 直径不再乘 10，确保 0x10 在屏幕规格区显示为整数 16。 */
        spec_values[2] = (uint32_t)angle; /* 公共接头 EPC 角度不再乘 10，确保 0x10 在屏幕规格区显示为整数 16°。 */
    }
    else
    {
        spec_values[0] = (uint32_t)((uint16_t)length * 2U); /* PXBA/PXBB USER 继续沿用旧缓存单位，屏幕显示前再恢复到原有长度格式。 */
        spec_values[1] = (uint32_t)((uint16_t)diameter * 10U); /* PXBA/PXBB USER 继续按 x10 保存直径，保持原有 Φ2.0 类显示不变。 */
        spec_values[2] = (uint32_t)((uint16_t)angle * 10U); /* PXBA/PXBB USER 继续按 x10 保存角度，避免旧分体式显示逻辑被公共接头改动影响。 */
    }
    spec_values[3] = (uint32_t)business_tool_type; /* 保存业务刀具类型，屏幕掉线/能力判断不再直接使用 RFID 原始代号。 */

    return true; /* RFID 刀具头信息已经写入识别缓存。 */
}

/*
 * 函数功能：为可拆式手柄先准备基座上线识别缓存，不把尚未解析的 RFID 刀具头伪装成有效刀具。
 * 输入参数：message 为 A/B 识别缓存；mapped_model 为 EEPROM 第二页映射出的基座型号；raw_type_major/raw_type_minor 为 EEPROM 第二页原始类型。
 * 返回参数：无。
 */
static void Handlescan_PrepareRfidBaseRecognizeMessage(ChannelrecognizeMessage_t *message,
                                                       uint8_t mapped_model,
                                                       uint8_t raw_type_major,
                                                       uint8_t raw_type_minor)
{
    if (message == NULL)
    {
        return; /* 识别缓存为空时不写字段，避免异常路径破坏全局状态。 */
    }

    memset(message, 0, sizeof(*message)); /* 先清空刀具字段，确保上位机只显示“等待 RFID”，不会看到全 0 假刀具。 */
    message->handle_type = mapped_model; /* 保存基座型号，插拔事件链会把它搬到 MemoryMsgA/B。 */
    message->hand_type_raw_major = raw_type_major; /* 保存 EEPROM 第二页原始主类型，上位机手柄卡片按该值显示 PXBA/PXBB。 */
    message->hand_type_raw_minor = raw_type_minor; /* 保存 EEPROM 第二页原始子类型，便于区分 PXBA、PXBB 和公共接头。 */
    message->freq_min = FreqMin; /* 基座阶段仍保留工程频率下限，避免后续结构字段完全为空。 */
    message->freq_max = FreqMax; /* 基座阶段仍保留工程频率上限，真实 RFID 到达后会覆盖刀具参数。 */
    message->speed_fzdefault=60000;
    message->speed_oscdefault=60000;
    message->speed_zzdefault=60000;
    message->speed_zzmin=10000;
    message->speed_zzmax=60000;
    message->speed_fzmin=10000;
    message->speed_fzmax=60000;
    message->speed_oscmin=10000;
    message->speed_oscmax=60000;
    message->run_direction = ZZDIR;//未接刀具刚插入，默认正转
    message->speed_zzstep=1000;
    message->speed_fzstep=1000;
    message->speed_oscstep=1000;
    message->tool_reduction_ratio=1;
    message->default_injection_flow=20;
    message->freq_default=40;
}

/*
 * 函数功能：在线状态下确认可拆刀具头连续读不到时，只清刀具头信息并保留手柄基座在线。
 * 输入参数：channel 为 A/B 通道；message/spec_values 为通道识别和规格缓存；mapped_model/raw_type_* 为 EEPROM 第二页基座信息；last_sequence/last_presence_sequence/miss_count/monitor_pending 为 RFID 在线监测状态；tool_online 为刀具头在线边沿标志；plug_key 为刷新事件键值。
 * 返回参数：无。
 */
static void Handlescan_ClearOnlineRfidTool(uint8_t channel,
                                           HandlescanToolSource tool_source,
                                           ChannelrecognizeMessage_t *message,
                                           uint32_t *spec_values,
                                           uint8_t mapped_model,
                                           uint8_t raw_type_major,
                                           uint8_t raw_type_minor,
                                           uint16_t *last_sequence,
                                           uint16_t *last_presence_sequence,
                                           uint8_t *miss_count,
                                           uint8_t *monitor_pending,
                                           uint8_t *tool_online,
                                           uint8_t plug_key)
{
    if ((message == NULL) || (spec_values == NULL) || (last_sequence == NULL) ||
        (last_presence_sequence == NULL) || (miss_count == NULL) || (monitor_pending == NULL) ||
        (tool_online == NULL))
    {
        return; /* 参数不完整时不改全局状态，避免误清正在工作的另一通道。 */
    }

    if (*tool_online == 0U)
    {
        *miss_count = 0U; /* 刀具头已经处于离线态时，只清缺失计数，避免后续周期重复触发离线提示。 */
        *monitor_pending = 0U; /* 同步清掉监测等待标志，让下一轮继续安静监测是否重新上线。 */
        return; /* 已经离线时不重复清 MemoryMsg、不重复发插拔事件，也不重复蜂鸣。 */
    }

    if (tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3)
    {
        Handlescan_PrepareRfidBaseRecognizeMessage(message,
                                                   mapped_model,
                                                   raw_type_major,
                                                   raw_type_minor); /* RFID 刀具头离线时保留基座信息，只清刀具字段等待下一次标签。 */
        Handlescan_ClearToolSpecValues(spec_values); /* RFID 刀具头移开后清屏幕规格，避免继续显示旧刀具 0x4200。 */
        Rfid_ClearChannelResult(channel); /* 清当前 RFID 结果但保留历史 payload，下一次同标签重新上线也能发布业务序号。 */
        Pubinterface_ClearRfidToolMemory(channel); /* RFID 无刀具头时同步清通道记忆，屏幕显示 61/62 或等待图。 */
        Handlescan_ReloadSpeedStepMessage(channel, message); /* 清刀具头后重新装回基座 Page6 步进，等待下一次 RFID 上线继承。 */
        //SendKeyBehMessage(PLUGunPLUG, plug_key); /* 复用插入事件链刷新 MemoryMsg、屏幕和上位机心跳。 */
    }
    *tool_online = 0U; /* 先把边沿状态切到离线，后续周期继续读不到标签时不会再次进入离线提示。 */
    Handlescan_BeepOnceIfNoAlarm(); /* RFID 刀具头离线确认时单响一次，让使用者知道刀具头已移开。 */
    if (tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3)
    {
        *last_sequence = 0U; /* RFID 刀具头已被清除，后续读到任意有效标签都需要重新消费。 */
        *last_presence_sequence = 0U; /* 清掉存在序号，避免旧 presence 继续挡住下一次上线。 */
    }
    *miss_count = 0U; /* 清掉缺失次数，当前这次状态变化已经处理完成。 */
    *monitor_pending = 0U; /* 清掉未完成监测标志，等待下一轮 1 秒周期重新读取。 */
}

/*
 * 函数功能：请求 RFID 读取并进入等待刀具头结果阶段。
 * 输入参数：channel 为 A/B 通道；stage 为当前通道状态机；tool_source 为 RFID EPC/USER 来源；last_sequence 为已消费序号；wait_ticks 为等待计数器；fast_mode 表示是否快速识别。
 * 返回参数：true 表示已进入 RFID 等待阶段，false 表示请求未发起。
 */
static bool Handlescan_StartRfidWait(uint8_t channel,
                                     HandlescanStage *stage,
                                     HandlescanToolSource tool_source,
                                     uint16_t *last_sequence,
                                     uint16_t *wait_ticks,
                                     bool fast_mode)
{
    RfidReadSource_t rfid_source = Handlescan_ToRfidSource(tool_source); /* 把手柄扫描来源转换成 RFID 模块来源。 */

    if ((stage == NULL) || (last_sequence == NULL) || (wait_ticks == NULL))
    {
        return false; /* 状态或序号指针为空时不切状态，避免异常路径卡死。 */
    }

    if (rfid_source == RFID_READ_SOURCE_NONE)
    {
        return false; /* EEPROM 第三页来源不进入 RFID 等待。 */
    }

    *last_sequence = 0U; /* 新一轮 RFID 上线等待必须重新消费结果，避免同一刀具头沿用旧序号后被挡住。 */
    *wait_ticks = 0U; /* 新一轮等待从 0 计数。 */
    Rfid_ClearChannelResult(channel); /* 清掉该通道旧 RFID 缓存，让同型号刀具头重新插入时也能生成可消费结果。 */
    if (channel == CHANNEL_A)
    {
        s_a_rfid_last_presence_sequence = 0U; /* A 通道重新进入 RFID 上线等待时，存在检测也从空状态开始。 */
        s_a_rfid_miss_count = 0U; /* 清掉 A 通道在线监测缺失次数，避免上一轮刀具头拔出状态影响本轮上线。 */
        s_a_rfid_monitor_pending = 0U; /* 清掉 A 通道在线监测等待标志，上线等待由 WAIT_RFID_TOOL 独立处理。 */
    }
    else if (channel == CHANNEL_B)
    {
        s_b_rfid_last_presence_sequence = 0U; /* B 通道重新进入 RFID 上线等待时，存在检测也从空状态开始。 */
        s_b_rfid_miss_count = 0U; /* 清掉 B 通道在线监测缺失次数，避免上一轮刀具头拔出状态影响本轮上线。 */
        s_b_rfid_monitor_pending = 0U; /* 清掉 B 通道在线监测等待标志，上线等待由 WAIT_RFID_TOOL 独立处理。 */
    }
    if (Rfid_RequestToolRead(channel, rfid_source, fast_mode) == true)
    {
        *stage = HANDLESCAN_STAGE_WAIT_RFID_TOOL; /* 请求已入队后才进入 RFID 等待阶段。 */
        return true; /* RFID 请求已经交给射频任务，调用方可以同步发布基座上线。 */
    }

    return false; /* 队列未接收请求时不进入等待阶段，避免状态机空等。 */
}

/*
 * 函数功能：等待 RFID 结果并在成功后发布上线或刷新事件。
 * 输入参数：channel 为 A/B 通道；stage 为状态机；tool_source 为刀具来源；message/spec_values 为通道缓存；mapped_model/raw_type_* 为 EEPROM 第二页基座信息；last_sequence/last_presence_sequence 为该通道已处理 RFID 序号；miss_count/monitor_pending 为在线监测状态；tool_online 为刀具头在线边沿标志；wait_ticks 为等待计数；last_alarm/retry 参数沿用原有失败重试逻辑；plug_key 为插入事件键值。
 * 返回参数：true 表示本轮已处理 RFID 等待阶段，调用方应返回。
 */
static bool Handlescan_ProcessRfidWait(uint8_t channel,
                                       HandlescanStage *stage,
                                       HandlescanToolSource tool_source,
                                       ChannelrecognizeMessage_t *message,
                                       uint32_t *spec_values,
                                       uint8_t mapped_model,
                                       uint8_t raw_type_major,
                                       uint8_t raw_type_minor,
                                       uint16_t *last_sequence,
                                       uint16_t *last_presence_sequence,
                                       uint8_t *miss_count,
                                       uint8_t *monitor_pending,
                                       uint8_t *tool_online,
                                       uint16_t *wait_ticks,
                                       uint8_t *verify_retry_count,
                                       uint16_t *verify_retry_wait_ticks,
                                       uint8_t *last_alarm,
                                       uint8_t plug_key)
{
    RfidToolResult_t result; /* 保存 RFID 任务最近一次有效结果。 */

    if ((stage == NULL) || (*stage != HANDLESCAN_STAGE_WAIT_RFID_TOOL))
    {
        return false; /* 当前不在 RFID 等待阶段时，调用方继续处理其它阶段。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return true; /* 运行中不处理 RFID，不改变参数，等待电机停止后继续。 */
    }

    if (Rfid_CopyLastResult(channel, &result) == true)
    {
        if ((last_sequence != NULL) && (result.sequence != *last_sequence))
        {
            if ((Handlescan_ToRfidSource(tool_source) == result.source) &&
                (Handlescan_ApplyRfidToolResult(channel,
                                                message,
                                                spec_values,
                                                mapped_model,
                                                raw_type_major,
                                                raw_type_minor,
                                                &result) == true))
            {
                *last_sequence = result.sequence; /* 记录本次结果序号，避免下轮重复处理同一帧。 */
                if (last_presence_sequence != NULL)
                {
                    *last_presence_sequence = result.presence_sequence; /* 上线成功时同步记录存在序号，在线监测从当前标签状态继续。 */
                }
                if (miss_count != NULL)
                {
                    *miss_count = 0U; /* 上线阶段已经读到标签，连续缺失次数必须清零。 */
                }
                if (monitor_pending != NULL)
                {
                    *monitor_pending = 0U; /* 上线阶段不是低频监测请求，清掉在线监测等待标志。 */
                }
                if (tool_online != NULL)
                {
                    *tool_online = 1U; /* 等待阶段读到有效 RFID 后切到在线态，本次上线蜂鸣只对应这个状态边沿。 */
                }
                SendKeyBehMessage(PLUGunPLUG, plug_key); /* RFID 结果生效后沿用现有插入事件链装载 MemoryMsg/WorkMessage。 */
                Handlescan_BeepOnceIfNoAlarm(); /* RFID 刀具头上线确认时单响一次，同一标签重新插回也给用户插入反馈。 */
                Handlescan_ClearChannelAlarm(channel, *last_alarm); /* 成功识别后释放该通道历史校验报警。 */
                *last_alarm = 0U; /* 清掉最近报警缓存。 */
                *stage = HANDLESCAN_STAGE_ONLINE; /* 刀具头也识别成功后，通道才进入在线保持。 */
                return true; /* 本轮 RFID 阶段处理完毕。 */
            }
        }
    }

    if (wait_ticks != NULL)
    {
        (*wait_ticks)++; /* 未取到新结果时增加等待时间。 */
        if (*wait_ticks >= HANDLESCAN_RFID_WAIT_TIMEOUT_TICKS)
        {
            Handlescan_EnterRfidRetryOrFail(channel,
                                            stage,
                                            verify_retry_count,
                                            verify_retry_wait_ticks,
                                            last_alarm,
                                            0U); /* RFID 超时使用独立 2 秒窗口，避免无刀具头时长时间显示等待。 */
        }
    }

    return true; /* RFID 等待阶段已处理，调用方本轮返回。 */
}

/*
 * 函数功能：在线空闲状态下低频请求 RFID 监测可拆刀具头变化。
 * 输入参数：channel 为 A/B 通道；tool_source 为刀具来源；monitor_ticks 为低频计数器；miss_count 为连续未读到次数；monitor_pending 表示上一轮监测请求仍未被 presence 序号确认。
 * 返回参数：无。
 */
static void Handlescan_RequestOnlineRfidMonitor(uint8_t channel,
                                                HandlescanToolSource tool_source,
                                                uint16_t *monitor_ticks,
                                                uint8_t *miss_count,
                                                uint8_t *monitor_pending)
{
    RfidReadSource_t rfid_source = Handlescan_ToRfidSource(tool_source); /* 只对 RFID 来源通道做低频监测。 */

    if ((monitor_ticks == NULL) || (miss_count == NULL) || (monitor_pending == NULL) || (rfid_source == RFID_READ_SOURCE_NONE))
    {
        return; /* EEPROM 第三页来源不需要周期 RFID。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        *monitor_ticks = 0U; /* 电机运行中暂停周期读取，停止后重新计时。 */
        *monitor_pending = 0U; /* 运行中不做 RFID 识别，也不把暂停识别误判为刀具头拔出。 */
        return; /* 运行中不发送 RFID 命令。 */
    }

    (*monitor_ticks)++; /* 空闲时累计在线监测周期。 */
    if (*monitor_ticks >= HANDLESCAN_RFID_MONITOR_PERIOD_TICKS)//大于100
    {
        *monitor_ticks = 0U; /* 到周期后清零，避免连续投递请求。 */
        if (*monitor_pending != 0U)
        {
            *monitor_pending = 0U; /* 上一轮请求经过 1 秒仍未被 presence 序号确认，先结束等待。 */
            if (*miss_count < 0xFFU)
            {
                ++(*miss_count); /* 记录一次在线监测未读到标签，达到阈值后由结果处理函数清刀具头。 */
            }
            if (*miss_count >= HANDLESCAN_RFID_MISS_MAX)
            {
                return; /* 已达到约 2 秒缺失判定，本轮不再追加新请求，交给清刀具逻辑刷新状态。 */
            }
        }
        if (Rfid_RequestToolRead(channel, rfid_source, false) == true)
        {
            *monitor_pending = 1U; /* 普通在线监测请求已发出，等待 RFID 任务用 presence 序号确认标签仍在。 */
        }
    }
}

/*
 * 函数功能：在线空闲状态下消费 RFID 新结果并刷新刀具参数。
 * 输入参数：channel 为 A/B 通道；tool_source 为刀具来源；message/spec_values 为通道缓存；mapped_model/raw_type_* 为基座信息；last_sequence/last_presence_sequence 为已处理序号；miss_count/monitor_pending 为在线监测状态；tool_online 为刀具头在线边沿标志；plug_key 为刷新事件键值。
 * 返回参数：无。
 */
static void Handlescan_ProcessOnlineRfidResult(uint8_t channel,
                                               HandlescanToolSource tool_source,
                                               ChannelrecognizeMessage_t *message,
                                               uint32_t *spec_values,
                                               uint8_t mapped_model,
                                               uint8_t raw_type_major,
                                               uint8_t raw_type_minor,
                                               uint16_t *last_sequence,
                                               uint16_t *last_presence_sequence,
                                               uint8_t *miss_count,
                                               uint8_t *monitor_pending,
                                               uint8_t *tool_online,
                                               uint8_t plug_key)
{
    RfidToolResult_t result; /* 保存 RFID 任务最近一次有效结果。 */

    if ((last_sequence == NULL) || (last_presence_sequence == NULL) ||
        (miss_count == NULL) || (monitor_pending == NULL) ||
        (tool_online == NULL) ||
        (Handlescan_ToRfidSource(tool_source) == RFID_READ_SOURCE_NONE))
    {
        return; /* 非 RFID 来源或序号指针无效时不处理。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return; /* 运行中不消费新 RFID 结果，避免运行参数变化。 */
    }

    if (*miss_count >= HANDLESCAN_RFID_MISS_MAX)
    {
        Handlescan_ClearOnlineRfidTool(channel,
                                       tool_source,
                                       message,
                                       spec_values,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor,
                                       last_sequence,
                                       last_presence_sequence,
                                       miss_count,
                                       monitor_pending,
                                       tool_online,
                                       plug_key); /* 连续一次在线监测未确认标签时，清刀具头并让上位机显示等待 RFID，整体约 2 秒。 */
        return; /* 本轮已处理刀具头缺失状态。 */
    }

    if (Rfid_CopyLastResult(channel, &result) == false)
    {
        return; /* 当前通道还没有有效 RFID 结果。 */
    }

    if (result.source != Handlescan_ToRfidSource(tool_source))
    {
        return; /* 来源不匹配时不使用，避免 EPC/USER 串用。 */
    }

    if (result.presence_sequence != *last_presence_sequence)
    {
        *last_presence_sequence = result.presence_sequence; /* 无论 payload 是否变化，只要读到标签就刷新存在序号。 */
        *miss_count = 0U; /* 标签仍可读到时清掉连续缺失次数。 */
        *monitor_pending = 0U; /* 本轮在线监测请求已经被 RFID 回包确认。 */
    }

    if (result.sequence == *last_sequence)
    {
        return; /* 结果已经处理过，不重复刷新。 */
    }

    if (Handlescan_ApplyRfidToolResult(channel,
                                       message,
                                       spec_values,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor,
                                       &result) == false)
    {
        return; /* 数据不符合当前来源时不刷新。 */
    }

    *last_sequence = result.sequence; /* 记录已处理序号。 */
    SendKeyBehMessage(PLUGunPLUG, plug_key); /* 复用插入事件链刷新 MemoryMsg、WorkMessage、屏幕和上位机状态。 */
    if (*tool_online == 0U)
    {
        *tool_online = 1U; /* 刀具头从离线重新变为在线，同一标签重新插回也要形成一次上线反馈。 */
        Handlescan_BeepOnceIfNoAlarm(); /* 离线后的重新上线只响一次，后续在线监测同标签不会重复响。 */
    }
    else if (result.cache_hit == false)
    {
        Handlescan_BeepOnceIfNoAlarm(); /* 已在线时只有新刀具头或 payload 变化才单响，同一标签的在线确认不响。 */
    }
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
 * 函数功能：按小端格式读取 EEPROM Page4 中的 16 位初始值字段。
 * 输入参数：buffer 指向已通过页校验的 Page4 缓存；offset 表示字段起始偏移。
 * 返回参数：解析后的 16 位无符号值。
 */
static uint16_t Handlescan_ReadUint16LE(const uint8_t *buffer, uint32_t offset)
{
    return (uint16_t)(((uint16_t)buffer[offset + 1U] << 8) | (uint16_t)buffer[offset]); /* Page4 业务字段低字节在前，不能沿用 Page3 的大端解析。 */
}

/*
 * 函数功能：把 EEPROM Page4 默认注水流量从 0.1 单位转换为泵业务流量并限制在 0~70。
 * 输入参数：flow_x10 Page4 中按 0.1 单位保存的默认注水流量。
 * 返回参数：现有 pumpMessage.speed_work 使用的整数流量值。
 */
static uint16_t Handlescan_BuildDefaultInjectionFlow(uint16_t flow_x10)
{
    uint16_t flow = (uint16_t)(flow_x10 / HANDLESCAN_INITIAL_FLOW_SCALE); /* Page4 默认流量按 x10 存储，泵任务当前使用整数流量。 */

    if (flow > HANDLESCAN_INITIAL_FLOW_MAX)
    {
        flow = HANDLESCAN_INITIAL_FLOW_MAX; /* 业务范围明确为 0~70，超过上限时钳位，避免异常 EEPROM 值让注水泵过量输出。 */
    }

    return flow; /* 返回已经适配现有注水泵速度单位的默认流量。 */
}

/*
 * 函数功能：把 Page4 默认速度限制到同页给出的最小速度和最大速度之间。
 * 输入参数：default_speed 默认速度，min_speed 最小速度，max_speed 最大速度，三者均保持 EEPROM x10 原始单位。
 * 返回参数：限制后的默认速度。
 */
static uint16_t Handlescan_ClampDefaultSpeed(uint16_t default_speed, uint16_t min_speed, uint16_t max_speed)
{
    if (max_speed < min_speed)
    {
        max_speed = min_speed; /* EEPROM 上下限异常时按最小值收敛，避免后续速度加减出现反向边界。 */
    }

    if (default_speed < min_speed)
    {
        default_speed = min_speed; /* 默认速度低于最小值时抬到最小值，保证上线初始速度在允许范围内。 */
    }

    if (default_speed > max_speed)
    {
        default_speed = max_speed; /* 默认速度高于最大值时降到最大值，保证上线初始速度不超过手柄配置上限。 */
    }

    return default_speed; /* 返回可直接写入 WorkMessage.speed_set_work 的 x10 速度值。 */
}

/*
 * 函数功能：解析 Page4 默认运动方向。
 * 输入参数：raw_direction EEPROM Page4 方向字节，当前业务确认初始方向按正转使用。
 * 返回参数：项目内部方向值，当前固定返回 ZZDIR。
 */
static uint8_t Handlescan_ParseInitialDirection(uint8_t raw_direction)
{
    (void)raw_direction; /* 当前业务确认 Page4 默认方向为正转，保留原始字节入口便于后续扩展编码表。 */
    return ZZDIR;        /* WorkMessage/MemoryMsg 内部使用 ZZDIR 表示正转。 */
}

/*
 * 函数功能：把 EEPROM Page4 初始值信息解析到通道识别结构，供插入事件装载默认速度、频率、注水流量和阈值。
 * 输入参数：message 目标通道识别结构；initial_info_buf 已通过页尾校验的 Page4 缓存。
 * 返回参数：无。
 */
static void Handlescan_UpdateInitialInfoMessage(ChannelrecognizeMessage_t *message, const uint8_t *initial_info_buf)
{
    uint16_t default_speed;                                  /* 保存 Page4 默认速度，单位沿用 EEPROM x10，驱动下发时再 /10。 */
    uint16_t min_speed;                                      /* 保存 Page4 最小速度，单位沿用 EEPROM x10。 */
    uint16_t max_speed;                                      /* 保存 Page4 最大速度，单位沿用 EEPROM x10。 */

    if ((message == NULL) || (initial_info_buf == NULL))
    {
        return;                                              /* 防御空指针，避免异常插拔路径破坏通道识别结构。 */
    }

    default_speed = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_DEFAULT_SPEED_OFFSET); /* 读取 Page4 默认速度，小端 x10。 */
    min_speed = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_MIN_SPEED_OFFSET); /* 读取 Page4 最小速度，小端 x10。 */
    max_speed = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_MAX_SPEED_OFFSET); /* 读取 Page4 最大速度，小端 x10。 */
    if (max_speed < min_speed)
    {
        max_speed = min_speed;                               /* 上下限异常时同步修正保存值，避免后续 SpeedActive 读到反向边界。 */
    }
    default_speed = Handlescan_ClampDefaultSpeed(default_speed, min_speed, max_speed); /* 默认速度按同页上下限钳位，保证上线速度合法。 */

    message->default_injection_flow = Handlescan_BuildDefaultInjectionFlow(Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_DEFAULT_FLOW_OFFSET)); /* 解析默认注水流量并转换到泵业务单位。 */
    message->speed_min = 10000;                         /* 保存通用最小速度，供后续 UI/外控边界逻辑复用。 */
    message->speed_max = max_speed;                         /* 保存通用最大速度，供后续 UI/外控边界逻辑复用。 */
    message->speed_zzmin = 10000;                       /* Page4 当前只有一组速度上下限，正转方向使用同一最小速度。 */
    message->speed_zzmax = max_speed;                       /* Page4 当前只有一组速度上下限，正转方向使用同一最大速度。 */
    message->speed_fzmin = 10000;                       /* Page4 当前只有一组速度上下限，反转方向使用同一最小速度。 */
    message->speed_fzmax = max_speed;                       /* Page4 当前只有一组速度上下限，反转方向使用同一最大速度。 */
    message->speed_oscmin = 10000;                      /* Page4 当前只有一组速度上下限，往复方向使用同一最小速度。 */
    message->speed_oscmax = max_speed;                      /* Page4 当前只有一组速度上下限，往复方向使用同一最大速度。 */
    message->speed_zzdefault = 60000;                /* Page4 默认速度作为正转上线初始速度。 */
    message->speed_fzdefault = 60000;                /* Page4 默认速度作为反转上线初始速度。 */
    message->speed_oscdefault = 60000;               /* Page4 默认速度作为往复上线初始速度。 */

    

    message->run_direction = Handlescan_ParseInitialDirection(initial_info_buf[HANDLESCAN_INITIAL_DIRECTION_OFFSET]); /* 解析默认方向，当前按业务确认使用正转。 */
    message->freq_default = initial_info_buf[HANDLESCAN_INITIAL_FREQ_OFFSET]; /* 保存 Page4 默认频率，往复模式启动时直接装载。 */
    message->freq_min = FreqMin;                             /* Page4 未提供频率下限，沿用现有 UI 频率下限宏保持调节边界。 */
    message->freq_max = FreqMax;                             /* Page4 未提供频率上限，沿用现有 UI 频率上限宏保持调节边界。 */
    message->speed_alarm_for = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_FOR_ALARM_OFFSET); /* 保存正转速度报警阈值，小端2字节，单位与WorkMessage.speed_work一致。 */
    message->speed_alarm_rev = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_REV_ALARM_OFFSET); /* 保存反转速度报警阈值，小端2字节，单位与WorkMessage.speed_work一致。 */
    message->freq_alarm_osc = initial_info_buf[HANDLESCAN_INITIAL_OSC_ALARM_OFFSET]; /* 保存往复频率报警值，只用于蜂鸣阈值。 */
}

/*
 * 函数功能：从 Page6 多档位速度区读取指定序号的速度步进。
 * 输入参数：speed_step_buf 为 Page6 整页缓存；gear_index 为需要读取的第几组速度，1/2/3 分别对应正转、反转、往复步进槽。
 * 返回参数：可直接写入 ChannelrecognizeMessage 的 16 位步进值，读不到或为 0 时返回 1000。
 */
static uint16_t Handlescan_ReadPage6SpeedStep(const uint8_t *speed_step_buf, uint8_t gear_index)
{
    uint8_t gear_count;                                      /* 保存 Page6[0] 声明的有效档位数量。 */
    uint8_t effective_gear;                                  /* 保存本次实际读取的档位序号，避免访问未配置档位。 */
    uint8_t speed_offset;                                    /* 保存所选档位速度字段的 Page6 偏移。 */
    uint16_t speed_step;                                     /* 保存从 EEPROM Page6 读取出的调速步进。 */

    if (speed_step_buf == NULL)
    {
        return HANDLESCAN_SPEED_STEP_FALLBACK;               /* Page6 读取失败时不影响上线，调速步进使用 1000 兜底。 */
    }

    gear_count = speed_step_buf[HANDLESCAN_SPEED_STEP_GEAR_COUNT_OFFSET]; /* Page6 第 0 字节表示已配置几组速度档位。 */
    if (gear_count == 0U)
    {
        return HANDLESCAN_SPEED_STEP_FALLBACK;               /* 未配置档位时使用固定兜底，避免速度按键按下无变化。 */
    }
    if (gear_count > HANDLESCAN_SPEED_STEP_MAX_GEAR)
    {
        gear_count = HANDLESCAN_SPEED_STEP_MAX_GEAR;         /* 固件当前只解析前三组速度，超过三组先按前三组处理。 */
    }

    effective_gear = (gear_index == 0U) ? 1U : gear_index;   /* 调用方传 0 时按第一组处理，防止异常参数返回 0 步进。 */
    if (effective_gear > gear_count)
    {
        effective_gear = gear_count;                         /* 缺少对应方向速度时沿用最后一组有效档位，保持三方向都可调速。 */
    }

    switch (effective_gear)
    {
    case 1U:
        speed_offset = HANDLESCAN_SPEED_STEP_GEAR1_OFFSET;   /* 第一组速度写入正转慢档步进。 */
        break;
    case 2U:
        speed_offset = HANDLESCAN_SPEED_STEP_GEAR2_OFFSET;   /* 第二组速度写入反转慢档步进。 */
        break;
    default:
        speed_offset = HANDLESCAN_SPEED_STEP_GEAR3_OFFSET;   /* 第三组速度写入往复慢档步进。 */
        break;
    }

    speed_step = Handlescan_ReadUint16BE(speed_step_buf, speed_offset); /* Page6 速度字段按上位机布局使用大端 16 位。 */
    if (speed_step == 0U)
    {
        return HANDLESCAN_SPEED_STEP_FALLBACK;               /* 已配置档位但速度为 0 时仍使用兜底，避免 UI 操作无反馈。 */
    }

    return speed_step;                                       /* 返回当前方向可用的慢档调速步进。 */
}

/*
 * 函数功能：把 Page6 多档位速度区同步到识别缓存的正转、反转、往复调速步进字段。
 * 输入参数：message 为目标通道识别缓存；speed_step_buf 为已读取并校验过页尾的 Page6 缓存，读取失败时可传 NULL。
 * 返回参数：无。
 */
static void Handlescan_UpdateSpeedStepMessage(ChannelrecognizeMessage_t *message, const uint8_t *speed_step_buf)
{
    if (message == NULL)
    {
        return;                                              /* 识别缓存为空时不写全局状态，避免异常路径破坏另一通道参数。 */
    }
    if(message->handle_type==PXBA_ONLINE||message->handle_type==PXBB_ONLINE)
    {
         // message->speed_min=500;
        message->speed_zzstep=500;
        message->speed_fzstep=500;
        message->speed_oscstep=500;
        message->speed_zzmin=3000;
        message->speed_fzmin=3000;
        message->speed_oscmin=3000;
        message->speed_oscmax=30000;
    }
    else
    {
        message->speed_zzstep=2000;
        message->speed_fzstep=2000;
        message->speed_oscstep=2000;
    }
    // message->speed_zzstep = Handlescan_ReadPage6SpeedStep(speed_step_buf, 1U); /* 正转慢档步进来自 Page6 第一组速度，失败时回退 1000。 */
    // message->speed_fzstep = Handlescan_ReadPage6SpeedStep(speed_step_buf, 2U); /* 反转慢档步进来自 Page6 第二组速度，缺项时沿用最后有效档。 */
    // message->speed_oscstep = Handlescan_ReadPage6SpeedStep(speed_step_buf, 3U); /* 往复慢档步进来自 Page6 第三组速度，配合新屏快档翻倍。 */
}

/*
 * 函数功能：读取指定通道 EEPROM Page6 多档位速度页，并写入通道识别缓存的三方向调速步进。
 * 输入参数：channel 为 A/B 通道；message 为目标通道识别缓存。
 * 返回参数：无。
 */
static void Handlescan_LoadPage6SpeedStep(uint8_t channel, ChannelrecognizeMessage_t *message)
{
    uint8_t read_status = 0U;                                 /* 保存 Page6 读取状态，1 表示读页和页尾校验通过。 */

    if (message == NULL)
    {
        return;                                               /* 识别缓存为空时不访问 EEPROM，避免异常路径误写全局参数。 */
    }

    AT24CS32_ClearLastDebugInfo();                            /* 读取 Page6 前清掉上一次 I2C 调试信息，便于现场需要时定位失败页。 */
    if (channel == CHANNEL_A)
    {
        read_status = AT24CS32_ReadPage_I2C2(HANDLESCAN_SPEED_STEP_PAGE_INDEX, s_a_speed_step_buf); /* A 通道从 I2C2 读取 Page6 多档位速度页。 */
        if (read_status == 0U)
        {
            memset(s_a_speed_step_buf, 0, sizeof(s_a_speed_step_buf)); /* Page6 失败不影响上线，只清缓存并让步进走 1000 兜底。 */
        }
        Handlescan_UpdateSpeedStepMessage(message, (read_status != 0U) ? s_a_speed_step_buf : NULL); /* 把 A 通道 Page6 转成正反往复三组步进。 */
        return;                                               /* A 通道 Page6 处理完成，本轮不再落入 B 通道。 */
    }

    if (channel == CHANNEL_B)
    {
        read_status = AT24CS32_ReadPage_I2C3(HANDLESCAN_SPEED_STEP_PAGE_INDEX, s_b_speed_step_buf); /* B 通道从 I2C3 读取 Page6 多档位速度页。 */
        if (read_status == 0U)
        {
            memset(s_b_speed_step_buf, 0, sizeof(s_b_speed_step_buf)); /* B 通道 Page6 失败同样不阻断手柄上线，避免现场旧 EEPROM 无 Page6 时报错。 */
        }
        Handlescan_UpdateSpeedStepMessage(message, (read_status != 0U) ? s_b_speed_step_buf : NULL); /* 把 B 通道 Page6 转成正反往复三组步进。 */
    }
}

/*
 * 函数功能：不重新访问 EEPROM，仅把当前通道已经缓存的 Page6 步进重新写回识别缓存。
 * 输入参数：channel 为 A/B 通道；message 为目标通道识别缓存。
 * 返回参数：无。
 */
static void Handlescan_ReloadSpeedStepMessage(uint8_t channel, ChannelrecognizeMessage_t *message)
{
    if (message == NULL)
    {
        return;                                               /* 识别缓存为空时不写字段，避免 RFID 掉线清理路径误操作。 */
    }

    if (channel == CHANNEL_A)
    {
        Handlescan_UpdateSpeedStepMessage(message, s_a_speed_step_buf); /* A 通道复用最近一次 Page6 缓存，支持不拔基座直接换刀具头。 */
    }
    else if (channel == CHANNEL_B)
    {
        Handlescan_UpdateSpeedStepMessage(message, s_b_speed_step_buf); /* B 通道复用自己的 Page6 缓存，避免 A/B 步进串用。 */
    }
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
 * 新接口把 A/B 识别缓存和实际工作态分开维护：
 * 1. `ChannelrecognizeMessageA/B` 保存本通道最新识别出的手柄、刀具和 Page4 初始值；
 * 2. `PlugORunPLUGActive()` 负责把识别缓存装载到 `MemoryMsgA/B` 和 `WorkMessage`；
 * 3. 扫描任务只发布插拔事件，避免 I2C 扫描上下文和工作态切换上下文同时改全局状态。
 * 下面这些 helper 统一负责识别缓存写入，避免在状态机主体中反复展开字段赋值。
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

/*
 * 函数功能：清空单通道手柄扫描识别缓存，确保拔出或无效插入后不会残留旧 EEPROM 数据。
 * 输入参数：message 指向 A/B 通道的 ChannelrecognizeMessage 识别缓存。
 * 返回参数：无。
 */
static void Handlescan_ClearRecognizeMessage(ChannelrecognizeMessage_t *message)
{
    if (message == NULL)
    {
        return;
    }

    message->handle_type = 0U;
    message->hand_type_raw_major = 0U;                       /* 清掉 Page2 原始主类型，避免下一次上线前外部通信读到旧手柄编码。 */
    message->hand_type_raw_minor = 0U;                       /* 清掉 Page2 原始子类型，保持识别缓存和当前插拔状态一致。 */
    message->tool_type = 0U;
    message->raw_tool_type = 0U;                              /* 清掉刀具原始型号，避免下次上线前驱动或上位机读取旧 PXM/PXP/RFID 代号。 */
    message->diameter = 0U;
    message->length = 0U;
    message->draw = 0U;
    message->tool_reduction_ratio = 0U;                      /* 清掉 RFID 完整减速比，避免下一次普通 EEPROM 手柄沿用旧刀具头参数。 */
    message->default_injection_flow = 0U;                    /* 拔出后清掉 Page4 默认注水流量，避免下一次无效手柄沿用旧流量。 */
    message->speed_min = 0U;                                 /* 拔出后清掉 Page4 最小速度，避免旧边界继续约束新手柄。 */
    message->speed_max = 0U;                                 /* 拔出后清掉 Page4 最大速度，避免旧边界继续约束新手柄。 */
    message->speed_zzmin = 0U;                               /* 清掉正转最小速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_zzmax = 0U;                               /* 清掉正转最大速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_fzmin = 0U;                               /* 清掉反转最小速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_fzmax = 0U;                               /* 清掉反转最大速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_oscmin = 0U;                              /* 清掉往复最小速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_oscmax = 0U;                              /* 清掉往复最大速度缓存，保证下一次扫描重新由 Page4 填充。 */
    message->speed_zzdefault = 0U;                           /* 清掉正转默认速度，避免插拔后沿用旧 Page4 默认值。 */
    message->speed_fzdefault = 0U;                           /* 清掉反转默认速度，避免插拔后沿用旧 Page4 默认值。 */
    message->speed_oscdefault = 0U;                          /* 清掉往复默认速度，避免插拔后沿用旧 Page4 默认值。 */
    message->speed_zzstep = 0U;                              /* 清掉正转调速步进，下一次上线重新从 Page6 或 RFID 兜底写入。 */
    message->speed_fzstep = 0U;                              /* 清掉反转调速步进，避免换手柄后沿用旧步进。 */
    message->speed_oscstep = 0U;                             /* 清掉往复调速步进，保持离线态没有可用调速参数。 */
    message->freq_min = 0U;                                  /* 清掉频率下限缓存，避免下一次调频沿用旧手柄边界。 */
    message->freq_max = 0U;                                  /* 清掉频率上限缓存，避免下一次调频沿用旧手柄边界。 */
    message->freq_default = 0U;                              /* 清掉默认频率，避免往复模式沿用旧 Page4 默认频率。 */
    message->speed_alarm_for = 0U;                           /* 清掉正转速度报警值，避免蜂鸣阈值跨手柄残留。 */
    message->speed_alarm_rev = 0U;                           /* 清掉反转速度报警值，避免蜂鸣阈值跨手柄残留。 */
    message->freq_alarm_osc = 0U;                            /* 清掉往复频率报警值，避免蜂鸣阈值跨手柄残留。 */
    message->run_direction = 0U;                             /* 清掉默认方向，下一次上线重新按 Page4/业务规则初始化。 */
}

/*
 * 函数功能：清空扫描层的指定通道识别缓存，不直接修改通道记忆和全局工作态。
 * 输入参数：channel 需要清空的通道号，CHANNEL_A 表示 A 通道，其它值按 B 通道处理。
 * 返回参数：无。
 */
static void Handlescan_ClearChannelState(uint8_t channel)
{
    if (channel == CHANNEL_A)
    {
        Handlescan_ClearRecognizeMessage(&ChannelrecognizeMessageA); /* 扫描层只清 A 通道识别缓存，MemoryMsg/WorkMessage 由插拔事件统一处理。 */
        s_a_rfid_last_sequence = 0U;                                 /* A 通道离线或插入取消后，下一次相同 RFID 标签也必须重新消费。 */
        s_a_rfid_last_presence_sequence = 0U;                         /* A 通道离线时同步清存在序号，避免旧标签读到状态延续到下一次插入。 */
        s_a_rfid_miss_count = 0U;                                      /* A 通道离线时清掉在线缺失计数。 */
        s_a_rfid_monitor_pending = 0U;                                 /* A 通道离线时清掉在线监测等待标志。 */
        s_a_rfid_tool_online = 0U;                                     /* A 通道基座离线或重新识别时，刀具头边沿状态回到未在线。 */
        Rfid_ClearChannelResult(CHANNEL_A);                          /* 同步清除 A 通道 RFID 模块缓存，避免旧标签序号挡住新上线。 */
    }
    else
    {
        Handlescan_ClearRecognizeMessage(&ChannelrecognizeMessageB); /* 扫描层只清 B 通道识别缓存，实际离线状态由 PlugORunPLUGActive 生效。 */
        s_b_rfid_last_sequence = 0U;                                 /* B 通道离线或插入取消后，下一次相同 RFID 标签也必须重新消费。 */
        s_b_rfid_last_presence_sequence = 0U;                         /* B 通道离线时同步清存在序号，避免旧标签读到状态延续到下一次插入。 */
        s_b_rfid_miss_count = 0U;                                      /* B 通道离线时清掉在线缺失计数。 */
        s_b_rfid_monitor_pending = 0U;                                 /* B 通道离线时清掉在线监测等待标志。 */
        s_b_rfid_tool_online = 0U;                                     /* B 通道基座离线或重新识别时，刀具头边沿状态回到未在线。 */
        Rfid_ClearChannelResult(CHANNEL_B);                          /* 同步清除 B 通道 RFID 模块缓存，避免旧标签序号挡住新上线。 */
    }
}

/*
 * 判断报警码是否属于手柄校验链路。
 * EEPROM 最终校验失败按 A/B/AB 区分，由 handlescan 负责在坏手柄拔出后释放。
 */
static uint8_t Handlescan_IsHandleVerifyAlarm(uint8_t alarm_value)
{
    return Pubinterface_IsHandleVerifyAlarm(alarm_value) ? 1U : 0U; /* 返回 1 表示这是手柄 EEPROM 校验/型号报警，返回 0 表示不是本模块可清的报警。 */
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
 * 函数功能：判断当前报警是否要求暂停手柄识别流程。
 * 输入参数：channel 当前扫描通道，保留用于后续按通道扩展；当前实现只区分报警类型。
 * 返回参数：1 表示暂停认证/读 EEPROM，0 表示允许继续识别。
 */
static uint8_t Handlescan_ShouldDeferRecognition(uint8_t channel)
{
    (void)channel;                                           /* 当前暂停策略只看全局报警类型，通道参数用于保持接口语义清晰。 */

    if (WorkMessage.alarm_flag == true)
    {
        if (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) != 0U)
        {
            return 0U;                                       /* 手柄 EEPROM 报警不阻塞另一通道识别，满足 A 坏 B 仍可识别。 */
        }

        return 1U;                                           /* 普通系统报警期间不再访问 A/B EEPROM，等待报警解除。 */
    }

    return 0U;                                               /* 无报警时允许正常插入去抖和 EEPROM 识别。 */
}

/*
 * 函数功能：根据通道返回本通道 EEPROM 校验失败报警码。
 * 输入参数：channel 当前扫描通道。
 * 返回参数：A/B 通道对应的报警码。
 */
static uint8_t Handlescan_GetChannelHandleAlarm(uint8_t channel)
{
    return (channel == CHANNEL_A) ? WORK_ALARM_HANDLE_MODEL_ERROR_A : WORK_ALARM_HANDLE_MODEL_ERROR_B; /* A/B 校验失败必须让上位机能分辨来源。 */
}

/*
 * 函数功能：在 A/B 两通道都处于校验失败时，把全局报警升级为 AB。
 * 输入参数：channel_alarm 当前通道报警码；peer_alarm 另一通道仍保持的报警码。
 * 返回参数：需要写入 WorkMessage 或临时上传的报警码。
 */
static uint8_t Handlescan_CombineHandleAlarm(uint8_t channel_alarm, uint8_t peer_alarm)
{
    if ((Handlescan_IsHandleVerifyAlarm(channel_alarm) != 0U) &&
        (Handlescan_IsHandleVerifyAlarm(peer_alarm) != 0U) &&
        (channel_alarm != peer_alarm))
    {
        return WORK_ALARM_HANDLE_MODEL_ERROR_AB;             /* A/B 都坏时上传 AB，避免上位机只能看到最后一个通道。 */
    }

    return channel_alarm;                                    /* 只有单通道失败时保持 A 或 B 的独立报警码。 */
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

    if ((WorkMessage.alarm_flag == false) || (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) == 0U))
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

/*
 * 函数功能：判断本次手柄校验失败是否只能作为运行中另一路的临时提示。
 * 输入参数：channel 当前失败通道；alarm_value 当前失败报警码。
 * 返回参数：1 表示只响 3 秒并临时弹窗，0 表示写入全局报警。
 */
static uint8_t Handlescan_ShouldUseTransientHandleAlarm(uint8_t channel, uint8_t alarm_value)
{
    if (Handlescan_IsHandleVerifyAlarm(alarm_value) == 0U)
    {
        return 0U;                                           /* 非手柄校验报警仍按原全局报警处理。 */
    }

    if (WorkMessage.runflag_work == false)
    {
        return 0U;                                           /* 非运行状态下坏手柄需要持续报警，直到拔出或恢复。 */
    }

    if (((WorkMessage.channel_work == CHANNEL_A) && (channel == CHANNEL_B)) ||
        ((WorkMessage.channel_work == CHANNEL_B) && (channel == CHANNEL_A)))
    {
        return 1U;                                           /* 当前通道正在工作时，另一路坏手柄不能影响当前工作。 */
    }

    return 0U;                                               /* 当前工作通道自身异常仍按全局报警处理。 */
}

/*
 * 函数功能：显示一个 3 秒临时手柄校验报警，不写 WorkMessage.alarm_flag。
 * 输入参数：channel 当前失败通道；alarm_value A/B 手柄校验失败报警码。
 * 返回参数：无。
 */
static void Handlescan_RaiseTransientHandleAlarm(uint8_t channel, uint8_t alarm_value)
{
    uint8_t display_value[10] = {0U};                         /* 新屏报警队列只使用 Value[0] 保存报警码，其余字节清零保持消息稳定。 */
    display_value[0] = alarm_value;                           /* 把临时报警码送给 UI_AIARM_ID，避免再调用旧屏提示接口。 */
    SendAlarmMessageTimed(alarm_value, HANDLESCAN_TRANSIENT_ALARM_MS); /* 蜂鸣器只响 3 秒，避免覆盖当前工作通道。 */
    ExternalComm_SendTransientAlarm(alarm_value, HANDLESCAN_TRANSIENT_ALARM_MS); /* 上位机收到非 0 后，3 秒后会收到 0 自动关闭弹窗。 */
    SendUIDSMessage(UI_AIARM_ID, true, display_value);          /* 屏幕显示同一报警码，但不写 WorkMessage，避免阻塞其它操作。 */
    s_transient_screen_alarm_value = alarm_value;                /* 记录当前临时屏幕报警码，到期后只清本次临时显示。 */
    s_transient_screen_alarm_ticks = HANDLESCAN_TRANSIENT_SCREEN_TICKS; /* 10ms 扫描周期下保持 3 秒。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, alarm_value); /* 输出临时报警调试码，便于现场确认通道来源。 */
}

/*
 * 函数功能：维护临时屏幕报警的 3 秒自动清除。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Handlescan_UpdateTransientScreenAlarm(void)
{
    if (s_transient_screen_alarm_ticks == 0U)
    {
        return;                                              /* 当前没有临时屏幕报警需要计时。 */
    }

    --s_transient_screen_alarm_ticks;                         /* 每个 handlescan 周期扣减一次，任务周期为 10ms。 */
    if (s_transient_screen_alarm_ticks == 0U)
    {
        if ((WorkMessage.alarm_flag == false) && (s_transient_screen_alarm_value != 0U))
        {
            SendUIDSMessage(UI_AIARM_ID, false, NULL);        /* 没有真实全局报警时，3 秒到期后清掉新屏临时弹窗。 */
        }
        s_transient_screen_alarm_value = 0U;                  /* 清本次临时报警归属，下一次可重新显示。 */
    }
}

/*
 * 函数功能：把认证通过后读取到的手柄/刀具基础信息写入通道识别缓存。
 * 输入参数：message 目标通道识别缓存；mapped_model 映射后的手柄类型；raw_type_major/raw_type_minor 为 Page2 原始手柄类型字节；mapped_tool_model 为映射后的刀具类型；diameter_tenth/length_tenth/angle_tenth 为刀具规格原始 0.1 单位值。
 * 返回参数：无。
 */
static void Handlescan_UpdateRecognizeMessage(ChannelrecognizeMessage_t *message,
                                              uint8_t mapped_model,
                                              uint8_t raw_type_major,
                                              uint8_t raw_type_minor,
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
    message->hand_type_raw_major = raw_type_major;            /* 原始主类型先保存在识别缓存，等待 PlugORunPLUGActive 统一搬到 MemoryMsg。 */
    message->hand_type_raw_minor = raw_type_minor;            /* 原始子类型先保存在识别缓存，避免扫描任务直接写通道记忆。 */
    message->tool_type = Handlescan_MapRawToolTypeToBusinessType(mapped_tool_model); /* 业务刀具类型按 PXM/PXP 归一，屏幕和开口定位都看这个字段。 */
    message->raw_tool_type = mapped_tool_model;               /* 原始刀具型号保留给上位机和驱动兼容判断。 */
    message->diameter = Handlescan_TenthToUint8(diameter_tenth);
    message->length = (uint16_t)(length_tenth / 10U);
    message->draw = Handlescan_TenthToUint8(angle_tenth);
}

static void Handlescan_RaiseAlarm(uint8_t channel, uint8_t alarm_value)
{
    uint8_t report_alarm = alarm_value;                      /* 默认按当前通道报警码上报。 */

    if (Handlescan_IsHandleVerifyAlarm(alarm_value) != 0U)
    {
        report_alarm = Handlescan_CombineHandleAlarm(alarm_value,
                                                     Handlescan_GetPeerActiveHandleAlarm(channel)); /* 另一通道也失败时升级为 AB 报警。 */
    }

    WorkAlarm_Set(report_alarm);                             /* 所有持续报警统一写入 WorkMessage.alarm_value。 */
    SendAlarmMessage(report_alarm);                          /* 持续报警蜂鸣直到对应清除路径发送 0。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, report_alarm);
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
 * 函数功能：按指定最大失败次数进入快速重试或最终失败态。
 * 输入参数：channel 为 A/B 通道；stage 为状态机；retry_count/retry_wait_ticks 为失败计数；last_alarm 为最近报警；alarm_value 为最终报警值；retry_max 为本链路最大失败次数。
 * 返回参数：无。
 */
static void Handlescan_EnterRetryOrFailWithLimit(uint8_t channel,
                                                 HandlescanStage *stage,
                                                 uint8_t *retry_count,
                                                 uint16_t *retry_wait_ticks,
                                                 uint8_t *last_alarm,
                                                 uint8_t alarm_value,
                                                 uint8_t retry_max)
{
    if ((stage == NULL) || (retry_count == NULL) || (retry_wait_ticks == NULL) || (last_alarm == NULL) || (retry_max == 0U))
    {
        return;                                               /* 运行时参数异常或重试上限无效时保持当前状态，避免空指针写入。 */
    }

    if (*retry_count < retry_max)
    {
        ++(*retry_count);                                     /* 记录本次失败，达到上限后才进入最终报警保持。 */
    }

    if (*retry_count < retry_max)
    {
        *retry_wait_ticks = 0U;                               /* 快速重试从当前周期重新计时。 */
        *stage = HANDLESCAN_STAGE_RETRY_WAIT;                 /* 保持插入态，稍后重新进入 VERIFY，不要求用户再次插拔。 */
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_VERIFY_RETRY, *retry_count);
        return;
    }

    if (alarm_value != 0U)
    {
        if (Handlescan_ShouldUseTransientHandleAlarm(channel, alarm_value) != 0U)
        {
            if (*last_alarm != alarm_value)
            {
                *last_alarm = alarm_value;                    /* 记录另一路坏手柄归属，但不写 WorkMessage，避免影响当前运行通道。 */
                Handlescan_RaiseTransientHandleAlarm(channel, alarm_value); /* 第一次最终失败时只提示 3 秒。 */
            }
        }
        else if ((*last_alarm != alarm_value) || (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) == 0U))
        {
            *last_alarm = alarm_value;                        /* 第一次达到最终失败或从临时提示转为持续报警时记录报警归属。 */
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
 * 函数功能：普通 EEPROM/型号认证链路失败后的统一入口。
 * 输入参数：channel 为 A/B 通道；stage 为状态机；retry_count/retry_wait_ticks 为失败计数；last_alarm 为最近报警；alarm_value 为最终报警值。
 * 返回参数：无。
 */
static void Handlescan_EnterRetryOrFail(uint8_t channel,
                                        HandlescanStage *stage,
                                        uint8_t *retry_count,
                                        uint16_t *retry_wait_ticks,
                                        uint8_t *last_alarm,
                                        uint8_t alarm_value)
{
    Handlescan_EnterRetryOrFailWithLimit(channel,
                                         stage,
                                         retry_count,
                                         retry_wait_ticks,
                                         last_alarm,
                                         alarm_value,
                                         HANDLESCAN_VERIFY_RETRY_MAX); /* 普通手柄认证继续保留原 3 次容错。 */
}

/*
 * 函数功能：RFID 刀具头等待失败后的专用入口。
 * 输入参数：channel 为 A/B 通道；stage 为状态机；retry_count/retry_wait_ticks 为失败计数；last_alarm 为最近报警；alarm_value 为最终报警值。
 * 返回参数：无。
 */
static void Handlescan_EnterRfidRetryOrFail(uint8_t channel,
                                            HandlescanStage *stage,
                                            uint8_t *retry_count,
                                            uint16_t *retry_wait_ticks,
                                            uint8_t *last_alarm,
                                            uint8_t alarm_value)
{
    Handlescan_EnterRetryOrFailWithLimit(channel,
                                         stage,
                                         retry_count,
                                         retry_wait_ticks,
                                         last_alarm,
                                         alarm_value,
                                         HANDLESCAN_RFID_VERIFY_RETRY_MAX); /* RFID 刀具头等待最多 2 轮，整体约 2 秒后退出等待。 */
}

/*
 * 把 EEPROM 认证返回码映射为系统报警码。
 * 当前策略：只有 EEPROM 最终校验失败才按通道映射为 A/B 手柄型号错误。
 */
static uint8_t Handlescan_MapVerifyStatusToAlarm(uint8_t channel, AT24CS32_CRC_Status verify_status)
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
            return Handlescan_GetChannelHandleAlarm(channel); /* A/B 通道使用不同报警码，供上位机弹窗直接区分来源。 */
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
 * 函数功能：执行 A 通道手柄插拔去抖、EEPROM 认证和识别缓存更新，认证通过后只发布插拔事件。
 * 输入参数：无。
 * 返回参数：无。
 * 设计目标：
 * 1. 仅在插入稳定后做一次认证和一次信息区读取；
 * 2. 认证成功后保持在线，不再重复访问 EEPROM；
 * 3. 认证失败后保持安静，但仍持续监视拔出边沿；
 * 4. 拔出稳定后只输出一次离线报文，并清理 A 通道扫描识别缓存。
 */
void HandlescanA_Fun_SSC(void)
{
    static  uint8_t XUYAOrfid_flag=0;
    
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
            s_a_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 基座拔出后刀具来源回到默认，历史 RFID 缓存只作记忆不直接上线。 */
            s_a_rfid_wait_ticks = 0U;                         /* 清掉 A 通道 RFID 等待计数。 */
            s_a_rfid_monitor_ticks = 0U;                      /* 清掉 A 通道 RFID 低频监测计数。 */
            Handlescan_ClearChannelState(CHANNEL_A);         /* 清空 A 通道新的在线与识别状态容器。 */
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_A); /* 同步清空 A 通道刀具规格缓存，避免 UI 残留旧值。 */
            XUYAOrfid_flag=0;
            SendKeyBehMessage(PLUGunPLUG, SCREENKey_UNPLUG_A); /* 通知新按键行为模块：A 手柄已拔出。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出“离线完成”报文。 */
          //  (void)Handlescan_HandleRunningPlugAlarm(1U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            Handlescan_BeepOnceIfNoAlarm();                  /* 没有运行中插拔或其他报警时，给 A 通道拔出确认单响。 */
            return;                                          /* A 通道离线处理完成，本轮到此结束。 */
        }

        /*
         * 其余情况下说明当前还没有形成有效上线，直接把 A 通道状态机静默复位即可，
         * 不需要通知 UI，也不需要输出离线报文。
         */
        s_handleA_debounce.out_debounce_ticks = 0U;          /* 静默复位时也要把拔出去抖计数清零。 */
        s_a_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态，等待下一次真实插入。 */
        s_a_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 插入取消后恢复默认来源。 */
        s_a_rfid_wait_ticks = 0U;                             /* 插入取消后清掉 RFID 等待计数。 */
        s_a_rfid_monitor_ticks = 0U;                          /* 插入取消后清掉 RFID 监测计数。 */
        Handlescan_ClearChannelState(CHANNEL_A);             /* 保证 A 通道新接口状态保持关闭。 */
        Handlescan_ClearToolSpecValues(paoxueSpeciValue_A);  /* 插入未完成时也清空 A 通道刀具规格缓存。 */
        return;                                              /* 当前只是插入取消，不做 UI 和报文更新。 */
    }

    /*
     * 当前已经检测到 A 通道短接成立。
     * 如果系统存在普通阻塞报警，此时只保持短接状态，不进入 EEPROM 认证。
     */
    if (Handlescan_ShouldDeferRecognition(CHANNEL_A) != 0U)
    {
        return;                                             /* 普通报警未解除前暂停识别，坏手柄 EEPROM 报警除外。 */
    }

    /*
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
                                         Handlescan_MapVerifyStatusToAlarm(CHANNEL_A, verify_status)); /* 先快速重试，最终失败才按 A 通道报警。 */
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

        mapped_model = handle_type_cfg->mapped_handle_type; /* 先取出基座型号，后续用它判断刀具信息来源。 */
        s_a_tool_source = Handlescan_GetToolSource(mapped_model); /* EEPROM 第二页决定 A 通道刀具信息来自 EEPROM 还是 RFID。 */
        if (s_a_tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3&&XUYAOrfid_flag==0)
        {
            XUYAOrfid_flag=1;
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_A); /* 可拆刀具头等待 RFID 前先清屏幕规格缓存，避免显示旧刀具头。 */
            if (Handlescan_StartRfidWait(CHANNEL_A,
                                         &s_a_stage,
                                         s_a_tool_source,
                                         &s_a_rfid_last_sequence,
                                         &s_a_rfid_wait_ticks,
                                         true) != false) /* 基座刚上线时快速读取 RFID 标签。 */
            {
                Handlescan_PrepareRfidBaseRecognizeMessage(&ChannelrecognizeMessageA,
                                                           mapped_model,
                                                           raw_type_major,
                                                           raw_type_minor); /* 只写基座字段，刀具字段等待 RFID 真实结果。 */
                Handlescan_LoadPage6SpeedStep(CHANNEL_A, &ChannelrecognizeMessageA); /* 可拆式 A 基座先装入 Page6 步进，后续 RFID 刀具头成功时继承。 */
                SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A); /* 先上报 RFID 基座在线，刀具头数据到达后再二次刷新通道记忆。 */
                SendKeyBeepMessage(1);
            }
            return; /* RFID 结果回来前先保持基座在线，刀具区由上位机显示等待 RFID。 */
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

        AT24CS32_ClearLastDebugInfo();                       /* 读取初始值页之前，先把调试缓存切到 Page4 访问。 */
        read_status = AT24CS32_ReadPage_I2C2(HANDLESCAN_INITIAL_INFO_PAGE_INDEX, s_a_initial_info_buf); /* 从 A 通道 EEPROM 读取 Page4 初始值信息区，并校验页尾。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出 A 通道初始值页读取或页校验失败报文。 */
            Handlescan_DebugTraceI2cDetail(1U);             /* 输出本轮 Page4 失败对应的底层 I2C 细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U);                /* 初始值页瞬时读取失败只重试，不立刻锁死手柄。 */
            return;                                         /* 本轮停止后续处理，等待下一次重新读取完整业务页。 */
        }

        raw_tool_major = s_a_tool_info_buf[HANDLESCAN_TOOL_MAJOR_OFFSET]; /* 取出刀具信息区第 1 字节作为刀具主类型。 */
        raw_tool_minor = s_a_tool_info_buf[HANDLESCAN_TOOL_MINOR_OFFSET]; /* 取出刀具信息区第 2 字节作为刀具子类型。 */
        tool_type_cfg = Handlescan_FindToolTypeConfig(raw_tool_major, raw_tool_minor); /* 按两个原始字节查找刀具配置表。 */
        if (tool_type_cfg == NULL)
        {
            // Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_tool_minor); /* 输出“刀具类型无法识别”报文。 */
            // // Handlescan_EnterRetryOrFail(CHANNEL_A,
            //                              &s_a_stage,
            //                              &s_a_verify_retry_count,
            //                              &s_a_verify_retry_wait_ticks,
            //                              &s_a_last_alarm,
            //                              0U);                /* 刀具页读到异常值时允许后续重试。 */
            // return;                                         /* 等待重新插拔。 */
        }

        tool_diameter_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 解析刀具直径，单位 0.1。 */
        tool_length_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_LENGTH_OFFSET); /* 解析刀具长度，单位 0.1。 */
        tool_angle_tenth = Handlescan_ReadUint16BE(s_a_tool_info_buf, HANDLESCAN_TOOL_ANGLE_OFFSET); /* 解析刀具角度，单位 0.1。 */

        mapped_tool_model = tool_type_cfg->mapped_handle_type; /* 取出查表后的系统内部刀具类型值。 */
        Handlescan_UpdateRecognizeMessage(&ChannelrecognizeMessageA,
                                          mapped_model,
                                          raw_type_major,
                                          raw_type_minor,
                                          mapped_tool_model,
                                          tool_diameter_tenth,
                                          tool_length_tenth,
                                          tool_angle_tenth); /* 同步更新 A 通道识别结果。 */
                                          WorkMessage.auto_identify= MemoryMsgA.auto_identify=1;
        Handlescan_UpdateInitialInfoMessage(&ChannelrecognizeMessageA,
                                            s_a_initial_info_buf); /* 同步更新 A 通道 Page4 默认速度、频率、方向、注水流量和蜂鸣阈值。 */
        Handlescan_LoadPage6SpeedStep(CHANNEL_A, &ChannelrecognizeMessageA); /* A 通道普通 EEPROM 刀具上线后读取 Page6，供新屏快慢调速键使用。 */
        Handlescan_UpdateToolSpecValues(paoxueSpeciValue_A,
                                        tool_diameter_tenth,
                                        tool_length_tenth,
                                        tool_angle_tenth,
                                        Handlescan_MapRawToolTypeToBusinessType(mapped_tool_model)); /* 规格缓存保存业务刀具类型，PXM/PXP 不再直接暴露旧码给 UI。 */
                                        WorkMessage.auto_identify=0;
        SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A);    /* 通知插拔事件链：A 通道上线，实际 WorkMessage/MemoryMsg 装载在 PlugORunPLUGActive 中完成。 */
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

    if (s_a_stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL)
    {
        raw_type_major = s_a_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 等待 RFID 期间复用已读到的 EEPROM 第二页主类型。 */
        raw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 等待 RFID 期间复用已读到的 EEPROM 第二页子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 重新查表，避免局部变量跨周期丢失。 */
        if (handle_type_cfg == NULL)
        {
            Handlescan_EnterRetryOrFail(CHANNEL_A,
                                         &s_a_stage,
                                         &s_a_verify_retry_count,
                                         &s_a_verify_retry_wait_ticks,
                                         &s_a_last_alarm,
                                         0U); /* 等待期间基座信息异常时回到原有重试流程。 */
            return; /* 本轮等待处理结束。 */
        }
        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出 A 通道基座型号。 */
        s_a_tool_source = Handlescan_GetToolSource(mapped_model); /* 重新确认 A 通道 RFID 来源。 */
        if (Handlescan_ProcessRfidWait(CHANNEL_A,
                                       &s_a_stage,
                                       s_a_tool_source,
                                       &ChannelrecognizeMessageA,
                                       paoxueSpeciValue_A,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor,
                                       &s_a_rfid_last_sequence,
                                       &s_a_rfid_last_presence_sequence,
                                       &s_a_rfid_miss_count,
                                       &s_a_rfid_monitor_pending,
                                       &s_a_rfid_tool_online,
                                       &s_a_rfid_wait_ticks,
                                       &s_a_verify_retry_count,
                                       &s_a_verify_retry_wait_ticks,
                                       &s_a_last_alarm,
                                       SCREENKey_PLUG_A) != false)
        {
            return; /* RFID 等待阶段本轮已经处理。 */
        }
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
        raw_type_major = s_a_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 在线监测时复用 EEPROM 第二页基座主类型。 */
        raw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 在线监测时复用 EEPROM 第二页基座子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 在线阶段重新查表，避免局部变量跨周期丢失。 */
        if (handle_type_cfg != NULL)
        {
            mapped_model = handle_type_cfg->mapped_handle_type; /* 取出 A 通道当前基座型号。 */
            s_a_tool_source = Handlescan_GetToolSource(mapped_model); /* 判断 A 通道是否需要 RFID 低频监测。 */
            Handlescan_RequestOnlineRfidMonitor(CHANNEL_A,
                                                s_a_tool_source,
                                                &s_a_rfid_monitor_ticks,
                                                &s_a_rfid_miss_count,
                                                &s_a_rfid_monitor_pending); /* 空闲时周期请求 RFID，运行中自动暂停。 */
            Handlescan_ProcessOnlineRfidResult(CHANNEL_A,
                                               s_a_tool_source,
                                               &ChannelrecognizeMessageA,
                                               paoxueSpeciValue_A,
                                               mapped_model,
                                               raw_type_major,
                                               raw_type_minor,
                                               &s_a_rfid_last_sequence,
                                               &s_a_rfid_last_presence_sequence,
                                               &s_a_rfid_miss_count,
                                               &s_a_rfid_monitor_pending,
                                               &s_a_rfid_tool_online,
                                               SCREENKey_PLUG_A); /* 消费新的 RFID 标签结果并刷新通道记忆。 */
        }
        return;                                             /* 在线保持态保持静默，只等待拔出。 */
    }
}

/*
 * B 通道 SSC 扫描函数。
 * 函数功能：执行 B 通道手柄插拔去抖、EEPROM 认证和识别缓存更新，认证通过后只发布插拔事件。
 * 输入参数：无。
 * 返回参数：无。
 * B 通道沿用与 A 通道一致的状态机流程，但底层资源切换为：
 * 1. 短接检测脚使用 B 通道输入脚；
 * 2. EEPROM 认证和信息区读取使用 I2C3；
 * 3. 在线状态和手柄型号由 PlugORunPLUGActive 统一写回；
 * 4. 串口报文通道号使用 `CH=02`。
 */
void HandlescanB_Fun_SSC(void)
{
     static  uint8_t XUYAOrfid_flag=0;
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
            s_b_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 基座拔出后刀具来源回到默认，历史 RFID 缓存只作记忆不直接上线。 */
            s_b_rfid_wait_ticks = 0U;                         /* 清掉 B 通道 RFID 等待计数。 */
            s_b_rfid_monitor_ticks = 0U;                      /* 清掉 B 通道 RFID 低频监测计数。 */
            Handlescan_ClearChannelState(CHANNEL_B);         /* 清空 B 通道新的在线与识别状态容器。 */
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_B); /* 同步清空 B 通道刀具规格缓存。 */
            XUYAOrfid_flag=0;
            SendKeyBehMessage(PLUGunPLUG, SCREENKey_UNPLUG_B); /* 通知新按键行为模块：B 手柄已拔出。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_REMOVE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出 B 通道“拔出去抖通过”报文。 */
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_OFFLINE, 0U); /* 输出 B 通道“离线完成”报文。 */
         //   (void)Handlescan_HandleRunningPlugAlarm(2U);     /* 如果电机仍在运行，则补充触发运行中插拔报警。 */
            Handlescan_BeepOnceIfNoAlarm();                  /* 没有运行中插拔或其他报警时，给 B 通道拔出确认单响。 */
            return;                                          /* B 通道离线处理结束。 */
        }

        /*
         * 如果还没有形成有效上线，说明只是插入过程中取消或接触抖动，
         * 此时对 B 通道做静默复位即可。
         */
        s_handleB_debounce.out_debounce_ticks = 0U;          /* 静默复位时，也把 B 通道拔出去抖计数清零。 */
        s_b_stage = HANDLESCAN_STAGE_IDLE;                   /* 回到空闲态。 */
        s_b_tool_source = HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 插入取消后恢复默认来源。 */
        s_b_rfid_wait_ticks = 0U;                             /* 插入取消后清掉 RFID 等待计数。 */
        s_b_rfid_monitor_ticks = 0U;                          /* 插入取消后清掉 RFID 监测计数。 */
        Handlescan_ClearChannelState(CHANNEL_B);             /* 保证 B 通道新接口状态保持关闭。 */
        Handlescan_ClearToolSpecValues(paoxueSpeciValue_B);  /* 插入未完成时也清空 B 通道刀具规格缓存。 */
        return;                                              /* 当前不形成有效离线事件，只做静默收尾。 */
    }

    /*
     * B 通道检测到插入后，先判断当前报警是否允许继续识别。
     * 普通系统报警未解除前不访问 B 通道 EEPROM；手柄校验报警允许另一通道继续识别。
     */
    if (Handlescan_ShouldDeferRecognition(CHANNEL_B) != 0U)
    {
        return;                                             /* 保持当前阶段，等报警解除后继续本通道识别流程。 */
    }
    /* 报警允许识别后，如果 B 状态机还在空闲态，则进入插入去抖阶段。 */
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
                                         Handlescan_MapVerifyStatusToAlarm(CHANNEL_B, verify_status)); /* 先快速重试，最终失败才按 B 通道报警。 */
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

        mapped_model = handle_type_cfg->mapped_handle_type; /* 先取出基座型号，后续用它判断刀具信息来源。 */
        s_b_tool_source = Handlescan_GetToolSource(mapped_model); /* EEPROM 第二页决定 B 通道刀具信息来自 EEPROM 还是 RFID。 */
        if (s_b_tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3&&XUYAOrfid_flag==0)
        {
            XUYAOrfid_flag=1;
            Handlescan_ClearToolSpecValues(paoxueSpeciValue_B); /* 可拆刀具头等待 RFID 前先清屏幕规格缓存，避免显示旧刀具头。 */
            if (Handlescan_StartRfidWait(CHANNEL_B,
                                         &s_b_stage,
                                         s_b_tool_source,
                                         &s_b_rfid_last_sequence,
                                         &s_b_rfid_wait_ticks,
                                         true) != false) /* 基座刚上线时快速读取 RFID 标签。 */
            {
                Handlescan_PrepareRfidBaseRecognizeMessage(&ChannelrecognizeMessageB,
                                                           mapped_model,
                                                           raw_type_major,
                                                           raw_type_minor); /* 只写基座字段，刀具字段等待 RFID 真实结果。 */
                Handlescan_LoadPage6SpeedStep(CHANNEL_B, &ChannelrecognizeMessageB); /* 可拆式 B 基座先装入 Page6 步进，避免 B 通道沿用固定默认步进。 */
                SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_B); /* 先上报 RFID 基座在线，刀具头数据到达后再二次刷新通道记忆。 */
                  SendKeyBeepMessage(1);
            }
            return; /* RFID 结果回来前先保持基座在线，刀具区由上位机显示等待 RFID。 */
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

        AT24CS32_ClearLastDebugInfo();                       /* 读取初始值页之前，先把调试缓存切到 Page4 访问。 */
        read_status = AT24CS32_ReadPage_I2C3(HANDLESCAN_INITIAL_INFO_PAGE_INDEX, s_b_initial_info_buf); /* 从 B 通道 EEPROM 读取 Page4 初始值信息区，并校验页尾。 */
        if (read_status == 0U)
        {
            Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_INFO_FAIL, read_status); /* 输出 B 通道初始值页读取或页校验失败报文。 */
            Handlescan_DebugTraceI2cDetail(2U);             /* 输出本轮 Page4 失败对应的底层 I2C 细节。 */
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U);                /* 初始值页瞬时读取失败只重试，不立刻锁死手柄。 */
            return;                                         /* 本轮停止后续处理，等待下一次重新读取完整业务页。 */
        }

        raw_tool_major = s_b_tool_info_buf[HANDLESCAN_TOOL_MAJOR_OFFSET]; /* 取出 B 通道刀具主类型。 */
        raw_tool_minor = s_b_tool_info_buf[HANDLESCAN_TOOL_MINOR_OFFSET]; /* 取出 B 通道刀具子类型。 */
        tool_type_cfg = Handlescan_FindToolTypeConfig(raw_tool_major, raw_tool_minor); /* 按原始字节查找刀具配置表。 */
        if (tool_type_cfg == NULL)
        {
            // Handlescan_DebugTrace(2U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_tool_minor); /* 输出 B 通道“刀具类型无法识别”报文。 */
            // Handlescan_EnterRetryOrFail(CHANNEL_B,
            //                              &s_b_stage,
            //                              &s_b_verify_retry_count,
            //                              &s_b_verify_retry_wait_ticks,
            //                              &s_b_last_alarm,
            //                              0U);                /* 刀具页读到异常值时允许后续重试。 */
            // return;                                         /* 等待重新插拔。 */
        }

        tool_diameter_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 解析刀具直径，单位 0.1。 */
        tool_length_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_LENGTH_OFFSET); /* 解析刀具长度，单位 0.1。 */
        tool_angle_tenth = Handlescan_ReadUint16BE(s_b_tool_info_buf, HANDLESCAN_TOOL_ANGLE_OFFSET); /* 解析刀具角度，单位 0.1。 */

        mapped_tool_model = tool_type_cfg->mapped_handle_type; /* 取出查表后的系统内部刀具类型值。 */
        Handlescan_UpdateRecognizeMessage(&ChannelrecognizeMessageB,
                                          mapped_model,
                                          raw_type_major,
                                          raw_type_minor,
                                          mapped_tool_model,
                                          tool_diameter_tenth,
                                          tool_length_tenth,
                                          tool_angle_tenth); /* 同步更新 B 通道识别结果。 */
                                            WorkMessage.auto_identify= MemoryMsgB.auto_identify=1;
        Handlescan_UpdateInitialInfoMessage(&ChannelrecognizeMessageB,
                                            s_b_initial_info_buf); /* 同步更新 B 通道 Page4 默认速度、频率、方向、注水流量和蜂鸣阈值。 */
        Handlescan_LoadPage6SpeedStep(CHANNEL_B, &ChannelrecognizeMessageB); /* B 通道普通 EEPROM 刀具上线后读取 Page6，保证 A/B 调速步进独立。 */
        Handlescan_UpdateToolSpecValues(paoxueSpeciValue_B,
                                        tool_diameter_tenth,
                                        tool_length_tenth,
                                        tool_angle_tenth,
                                        Handlescan_MapRawToolTypeToBusinessType(mapped_tool_model)); /* 规格缓存保存业务刀具类型，PXM/PXP 不再直接暴露旧码给 UI。 */
        SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_B);    /* 通知插拔事件链：B 通道上线，实际 WorkMessage/MemoryMsg 装载在 PlugORunPLUGActive 中完成。 */
        WorkMessage.auto_identify=0;
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

    if (s_b_stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL)
    {
        raw_type_major = s_b_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 等待 RFID 期间复用已读到的 EEPROM 第二页主类型。 */
        raw_type_minor = s_b_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 等待 RFID 期间复用已读到的 EEPROM 第二页子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 重新查表，避免局部变量跨周期丢失。 */
        if (handle_type_cfg == NULL)
        {
            Handlescan_EnterRetryOrFail(CHANNEL_B,
                                         &s_b_stage,
                                         &s_b_verify_retry_count,
                                         &s_b_verify_retry_wait_ticks,
                                         &s_b_last_alarm,
                                         0U); /* 等待期间基座信息异常时回到原有重试流程。 */
            return; /* 本轮等待处理结束。 */
        }
        mapped_model = handle_type_cfg->mapped_handle_type; /* 取出 B 通道基座型号。 */
        s_b_tool_source = Handlescan_GetToolSource(mapped_model); /* 重新确认 B 通道 RFID 来源。 */
        if (Handlescan_ProcessRfidWait(CHANNEL_B,
                                       &s_b_stage,
                                       s_b_tool_source,
                                       &ChannelrecognizeMessageB,
                                       paoxueSpeciValue_B,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor,
                                       &s_b_rfid_last_sequence,
                                       &s_b_rfid_last_presence_sequence,
                                       &s_b_rfid_miss_count,
                                       &s_b_rfid_monitor_pending,
                                       &s_b_rfid_tool_online,
                                       &s_b_rfid_wait_ticks,
                                       &s_b_verify_retry_count,
                                       &s_b_verify_retry_wait_ticks,
                                       &s_b_last_alarm,
                                       SCREENKey_PLUG_B) != false)
        {
            return; /* RFID 等待阶段本轮已经处理。 */
        }
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
        raw_type_major = s_b_info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 在线监测时复用 EEPROM 第二页基座主类型。 */
        raw_type_minor = s_b_info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 在线监测时复用 EEPROM 第二页基座子类型。 */
        handle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor); /* 在线阶段重新查表，避免局部变量跨周期丢失。 */
        if (handle_type_cfg != NULL)
        {
            mapped_model = handle_type_cfg->mapped_handle_type; /* 取出 B 通道当前基座型号。 */
            s_b_tool_source = Handlescan_GetToolSource(mapped_model); /* 判断 B 通道是否需要 RFID 低频监测。 */
            Handlescan_RequestOnlineRfidMonitor(CHANNEL_B,
                                                s_b_tool_source,
                                                &s_b_rfid_monitor_ticks,
                                                &s_b_rfid_miss_count,
                                                &s_b_rfid_monitor_pending); /* 空闲时周期请求 RFID，运行中自动暂停。 */
            Handlescan_ProcessOnlineRfidResult(CHANNEL_B,
                                               s_b_tool_source,
                                               &ChannelrecognizeMessageB,
                                               paoxueSpeciValue_B,
                                               mapped_model,
                                               raw_type_major,
                                               raw_type_minor,
                                               &s_b_rfid_last_sequence,
                                               &s_b_rfid_last_presence_sequence,
                                               &s_b_rfid_miss_count,
                                               &s_b_rfid_monitor_pending,
                                               &s_b_rfid_tool_online,
                                               SCREENKey_PLUG_B); /* 消费新的 RFID 标签结果并刷新通道记忆。 */
        }
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
    Handlescan_UpdateTransientScreenAlarm();                 /* 维护运行中另一路坏手柄的 3 秒屏幕临时提示。 */
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
