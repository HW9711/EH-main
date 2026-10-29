// handlescan.c

#include "handlescan.h"
#include "handlekey.h" /* GZYT 临时上线许可与数字按钮测试共用同一开关，关闭后恢复型号禁用。 */
#include "diagnostic.h" /* 独立诊断开关及只读副本接口。 */
#include "board.h"
#include "board_profile.h"
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
static uint8_t s_verify_channel = CHANNEL_A; /* 每轮只允许一侧执行认证I2C访问；A/B轮换，其余插拔采样仍每10ms执行。 */

/*
 * A/B 手柄插拔检测引脚。低电平表示插着手柄，高电平表示可能已拔出。
 * 程序还会连续检查一段时间，确认不是干扰后才改变在线状态。
 * 实际端口由 BOARD_RES_* 板级配置提供；接线改变时须一起核对板级配置。
 */
#define HANDLESCAN_A_SHORT_GPIO               BOARD_RES_HANDLESCAN_A_SHORT_PORT
#define HANDLESCAN_A_SHORT_PIN                BOARD_RES_HANDLESCAN_A_SHORT_PIN
#define HANDLESCAN_B_SHORT_GPIO               BOARD_RES_HANDLESCAN_B_SHORT_PORT
#define HANDLESCAN_B_SHORT_PIN                BOARD_RES_HANDLESCAN_B_SHORT_PIN

/*
 * 手柄插拔、EEPROM 校验和 RFID 读取的等待时间，A/B 共用。
 * 时间单位为 ms；下方 *_TICKS 用整数除法换算成扫描次数，时间应为扫描周期的整数倍。
 * 插入先确认电平稳定，再等接口和 EEPROM 稳定后校验；失败时先重试，达到次数上限才报警。
 * 报警后仍会定时重试；手柄拔出或校验成功后清除相应报警。
 */
#define HANDLESCAN_TASK_PERIOD_MS             10U /* 按 10ms 换算等待次数；修改时须同步 HandlescanTaskInit 的任务周期，单改本宏不会改变扫描间隔。 */
#define HANDLESCAN_INSERT_DEBOUNCE_MS         50U /* 插入信号需连续稳定的时间，单位 ms；加大后插入确认更慢。 */
#define HANDLESCAN_ONLINE_DISCONNECT_FILTER_MS 50U /* 已在线时，高电平需连续保持这么多 ms 才开始确认拔出，滤掉短暂干扰。 */
#define HANDLESCAN_REMOVE_DEBOUNCE_MS         500U /* 拔出确认所需的累计时间，单位 ms；在线时前面的 50ms 已计入，加大后离线通知更晚。 */
#define HANDLESCAN_VERIFY_START_DELAY_MS      200U /* 插入确认后再等这么多 ms 才校验 EEPROM，给接口留出稳定时间。 */
#define HANDLESCAN_VERIFY_RETRY_DELAY_MS      200U /* 校验或读取失败后，下次快速重试前等待的 ms 数。 */
#define HANDLESCAN_VERIFY_ALARM_RETRY_MS      1000U /* 已报校验失败后，每隔这么多 ms 重试，成功后可恢复在线。 */
#define HANDLESCAN_VERIFY_RETRY_MAX           3U /* 连续失败累计达到 3 次后停止快速重试，转入报警后的定时重试。 */

#define HANDLESCAN_INSERT_DEBOUNCE_TICKS      (HANDLESCAN_INSERT_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_ONLINE_DISCONNECT_FILTER_TICKS (HANDLESCAN_ONLINE_DISCONNECT_FILTER_MS / HANDLESCAN_TASK_PERIOD_MS) /* 在线高电平需要连续出现的扫描次数，A/B 使用相同要求。 */
#define HANDLESCAN_REMOVE_DEBOUNCE_TICKS      (HANDLESCAN_REMOVE_DEBOUNCE_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_START_DELAY_TICKS   (HANDLESCAN_VERIFY_START_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_RETRY_DELAY_TICKS   (HANDLESCAN_VERIFY_RETRY_DELAY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_VERIFY_ALARM_RETRY_TICKS   (HANDLESCAN_VERIFY_ALARM_RETRY_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_RFID_WAIT_TIMEOUT_MS       6000U /* 仅限制仍未结束的首读事务；实际读取完成立即转在线检测，不再空等6秒，运行暂停不累计。 */
#define HANDLESCAN_RFID_MONITOR_PERIOD_MS     500U  /* 空闲时两轮 RFID 检查的最短启动间隔，单位 ms；上一轮没结束就继续等。 */
#define HANDLESCAN_RFID_MISS_MAX              6U    /* 连续 6 轮读取超时才确认刀具掉线；排队和取消不计入，实际时间还取决于每轮耗时。 */

#define HANDLESCAN_RFID_WAIT_TIMEOUT_TICKS    (HANDLESCAN_RFID_WAIT_TIMEOUT_MS / HANDLESCAN_TASK_PERIOD_MS)
#define HANDLESCAN_RFID_MONITOR_PERIOD_TICKS  (HANDLESCAN_RFID_MONITOR_PERIOD_MS / HANDLESCAN_TASK_PERIOD_MS)

/*
 * EEPROM 中手柄信息区定义。
 * 第二页起始地址为 0x0020，因此读取这个地址开始的 16 字节即可拿到当前手柄的业务信息。
 * 其中前两个字节用于标识手柄大类和子类型。
 */
#define HANDLESCAN_INFO_ADDR                  0x0020U /* EEPROM 第 2 页起始字节地址，保存手柄型号。 */
#define HANDLESCAN_INFO_SIZE                  16U /* 每次读取手柄信息的字节数，须覆盖型号所在的前两个字节。 */
#define HANDLESCAN_MODEL_MAJOR_OFFSET         0U /* 手柄信息第 1 字节为主型号，和子型号一起查表。 */
#define HANDLESCAN_MODEL_MINOR_OFFSET         1U /* 手柄信息第 2 字节为子型号，不能单独用它判断手柄。 */

/*
 * EEPROM 中刀具信息区定义。
 * 第三页起始地址为 0x0040，刀具规格继续沿用大端 16 位和 0.1 精度。
 * Page3[8]/[9] 保留整数倍率，Page3[11..14] 为 GR02 标记，Page3[15]/[16] 保存两位小数。
 * Page3[17..29] 保存 PX01 手动刨刀速度扩展，因此主控必须读取完整 30 字节业务区。
 * 没有 GR02 标记的历史 EEPROM 只读取整数倍率，保证旧数据继续按原倍率运行。
 */
#define HANDLESCAN_TOOL_INFO_ADDR             0x0040U /* EEPROM 第 3 页起始字节地址，保存刀具规格、倍率和 PX01 转速配置。 */
#define HANDLESCAN_TOOL_INFO_SIZE             AT24CS32_PAGE_DATA_SIZE /* 读取Page3完整30字节业务区，确保PX01及三个24位速度字段进入本通道缓存。 */
#define HANDLESCAN_TOOL_MAJOR_OFFSET          0U /* Page3[0] 为刀具主型号，和下一字节一起查表。 */
#define HANDLESCAN_TOOL_MINOR_OFFSET          1U /* Page3[1] 为刀具子型号。 */
#define HANDLESCAN_TOOL_DIAMETER_OFFSET       2U /* Page3[2..3] 为刀具直径，单位 0.1 mm，高字节在前。 */
#define HANDLESCAN_TOOL_LENGTH_OFFSET         4U /* Page3[4..5] 为刀具长度，单位 0.1 mm，高字节在前。 */
#define HANDLESCAN_TOOL_ANGLE_OFFSET          6U /* Page3[6..7] 为刀具角度，单位 0.1 度，高字节在前。 */
#define HANDLESCAN_TOOL_REDUCTION_RATIO_OFFSET 8U  /* Page3[8] 保存减速比整数部分，旧 EEPROM 仍直接使用该字节。 */
#define HANDLESCAN_TOOL_SPEED_UP_RATIO_OFFSET 9U   /* Page3[9] 保存增速比整数部分，旧 EEPROM 仍直接使用该字节。 */
#define HANDLESCAN_TOOL_RATIO_MARK_OFFSET     11U  /* Page3[11..14] 保存新倍率格式标记，Page3[10] 继续保留给夹持范围。 */
#define HANDLESCAN_TOOL_RATIO_MARK_0          0x47U /* GR02 byte0：ASCII 字符 G。 */
#define HANDLESCAN_TOOL_RATIO_MARK_1          0x52U /* GR02 byte1：ASCII 字符 R。 */
#define HANDLESCAN_TOOL_RATIO_MARK_2          0x30U /* GR02 byte2：ASCII 字符 0。 */
#define HANDLESCAN_TOOL_RATIO_MARK_3          0x32U /* GR02 byte3：ASCII 字符 2，表示两位小数格式。 */
#define HANDLESCAN_TOOL_REDUCTION_DECIMAL_OFFSET 15U /* Page3[15] 保存减速比百分位，合法范围为 0~99。 */
#define HANDLESCAN_TOOL_SPEED_UP_DECIMAL_OFFSET 16U  /* Page3[16] 保存增速比百分位，合法范围为 0~99。 */
#define HANDLESCAN_TOOL_RATIO_DECIMAL_MAX     99U  /* 倍率小数部分只允许 0~99；超过 99 时两组倍率都只使用旧整数部分。 */
#define HANDLESCAN_TOOL_RATIO_UNIT            100U /* 运行时倍率按实际值乘 100 保存；100 表示 1.00 倍，即不增速也不减速。 */
#define HANDLESCAN_TOOL_RATIO_X100_UNIT       100U /* Page3 整数部分乘 100 后进入公共倍率字段。 */
#define HANDLESCAN_TOOL_SPEED_UP_SHIFT        16U  /* tool_reduction_ratio 高 16 位表示增速比，低 16 位表示减速比。 */

/*
 * EEPROM 中 Page4 初始值信息区定义。
 * Page4 对应 AT24CS32 驱动页下标 3，读取整页可以同步校验页尾，避免默认速度、流量和报警阈值读到损坏数据。
 * Page4 默认注水流量按大端直接保存 1~300；速度和报警阈值字段仍按小端保存。
 */
#define HANDLESCAN_INITIAL_INFO_PAGE_INDEX    3U /* EEPROM 第 4 页；驱动页编号从 0 开始，不是第 3 页。 */
#define HANDLESCAN_INITIAL_DEFAULT_FLOW_OFFSET 0U /* Page4[0..1] 保存默认注水流量，单位 mL/min，高字节在前。 */
#define HANDLESCAN_INITIAL_DEFAULT_SPEED_OFFSET 2U /* Page4[2..3] 保存默认转速低 16 位，单位 rpm，低字节在前。 */
#define HANDLESCAN_INITIAL_DEFAULT_SPEED_HIGH_OFFSET 20U /* Page4[20] 保存默认速度高8位，用于和[2-3]组合成24位默认速度。 */
#define HANDLESCAN_INITIAL_MIN_SPEED_OFFSET   4U /* Page4[4..5] 保存最低转速，单位 rpm，低字节在前。 */
#define HANDLESCAN_INITIAL_MAX_SPEED_OFFSET   6U /* Page4[6..7] 保存最高转速低 16 位，单位 rpm，低字节在前。 */
#define HANDLESCAN_INITIAL_MAX_SPEED_HIGH_OFFSET 19U /* Page4[19] 保存最大速度高8位，用于和[6-7]组合成24位速度上限。 */
#define HANDLESCAN_INITIAL_DIRECTION_OFFSET   8U /* Page4[8] 保存默认方向：1 正转、2 反转、3 往复；还需检查型号是否支持。 */
#define HANDLESCAN_INITIAL_FREQ_OFFSET        9U /* Page4[9] 保存默认往复频率指令值，主控直接使用该数值；实际 Hz 含义须以驱动协议为准。 */
#define HANDLESCAN_INITIAL_FOR_ALARM_OFFSET   10U /* Page4[10..11] 保存正转速度报警阈值，单位 rpm，低字节在前。 */
#define HANDLESCAN_INITIAL_REV_ALARM_OFFSET   12U /* Page4[12..13] 保存反转速度报警阈值，单位 rpm，低字节在前。 */
#define HANDLESCAN_INITIAL_OSC_ALARM_OFFSET   14U /* 当前只读取 Page4[14] 作为往复频率蜂鸣阈值，不读取第 15 字节。 */
#define HANDLESCAN_INITIAL_CURRENT_THRESHOLD_OFFSET 21U /* Page4[21..22] 保存驱动过流保护阈值，单位 0.01A；0 表示驱动板使用自身默认保护值。 */
#define HANDLESCAN_CONTROL_PROFILE_OFFSET 23U /* Page4[23]：00旧版，01仅默认方向，10正反转，11正反往复；均为十六进制。 */
#define HANDLESCAN_MANUAL_GRIND_LOW_OFFSET 24U /* Page4[24..25]保存PX手动磨头设定转速<30000的电流，LE16、0.01A。 */
#define HANDLESCAN_MANUAL_GRIND_HIGH_OFFSET 26U /* Page4[26..27]保存PX手动磨头设定转速>=30000的电流，LE16、0.01A。 */
#define HANDLESCAN_MANUAL_PLANER_CURRENT_OFFSET 28U /* Page4[28..29]保存PX手动刨刀电流，LE16、0.01A。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MARK_OFFSET 17U /* Page3[17..20] 保存ASCII“PX01”，明确标识PX手动刨刀24位速度扩展。 */
#define HANDLESCAN_PX_PLANER_PROFILE_DEFAULT_OFFSET 21U /* Page3[21..23] 小端保存手动刨刀默认转速，单位rpm。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MIN_OFFSET 24U /* Page3[24..26] 小端保存手动刨刀最小转速，单位rpm。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MAX_OFFSET 27U /* Page3[27..29] 小端保存手动刨刀最大转速，单位rpm。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MARK_0 0x50U /* 扩展标记第1字节ASCII“P”。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MARK_1 0x58U /* 扩展标记第2字节ASCII“X”。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MARK_2 0x30U /* 扩展标记第3字节ASCII“0”。 */
#define HANDLESCAN_PX_PLANER_PROFILE_MARK_3 0x31U /* 扩展标记第4字节ASCII“1”，后续格式升级必须更换版本。 */
#define HANDLESCAN_RFID_PLANER_ZERO_MIN_SPEED_RPM 500U /* RFID协议专门约定普通刨刀最低速度字节0x00表示500rpm。 */
#define HANDLESCAN_INITIAL_FLOW_MIN           1U /* EEPROM 默认注水流量最小有效值，单位 mL/min；低于此值改用程序默认值。 */
#define HANDLESCAN_INITIAL_FLOW_MAX           300U /* EEPROM 默认注水流量最大有效值，单位 mL/min；高于此值改用程序默认值。 */
#define HANDLESCAN_INITIAL_FLOW_DEFAULT       30U /* EEPROM 默认流量不在 1~300 mL/min 内时使用 30 mL/min。 */
#define HANDLESCAN_INITIAL_DIRECTION_FORWARD  0x01U /* Page4[8]=0x01 表示默认正转。 */
#define HANDLESCAN_INITIAL_DIRECTION_REVERSE  0x02U /* Page4[8]=0x02 表示默认反转。 */
#define HANDLESCAN_INITIAL_DIRECTION_OSC      0x03U /* Page4[8]=0x03 表示默认往复，写入前必须确认型号支持。 */

/*
 * EEPROM 中 Page6 速度调节区定义。
 * Page6 对应 AT24CS32 驱动页下标 5，前 4 字节分别保存小步进和大步进。
 * 每个值的高字节在前；读不到或值为 0 时使用下面的默认增减量，避免屏幕速度键无响应。
 */
#define HANDLESCAN_SPEED_STEP_PAGE_INDEX      5U /* EEPROM 第 6 页，驱动页编号从 0 开始。 */
#define HANDLESCAN_SPEED_STEP_SMALL_OFFSET    0U   /* Page6[0..1] 保存普通加减速键每次改变的 rpm 数，高字节在前。 */
#define HANDLESCAN_SPEED_STEP_LARGE_OFFSET    2U   /* Page6[2..3] 保存快速加减速键每次改变的 rpm 数，高字节在前。 */
#define HANDLESCAN_SPEED_STEP_FALLBACK        1000U /* 缺少有效调速增减量时默认每次改变 1000 rpm。 */
#define HANDLESCAN_SPEED_STEP_SMALL_FALLBACK  1000U /* Page6 普通调速值无效时，每次增加或减少 1000 rpm。 */
#define HANDLESCAN_SPEED_STEP_LARGE_FALLBACK  2000U /* Page6 快速调速值无效时，每次增加或减少 2000 rpm。 */

/*
 * 手柄扫描报警码定义。
 * 1. 13：运行中插拔报警，沿用旧逻辑；
 * 2. EEPROM 最终校验失败按 A/B 通道上报 WORK_ALARM_HANDLE_MODEL_ERROR_A/B/AB。
 */
#define HANDLESCAN_ALARM_RUNNING_PLUG         13U
#define HANDLESCAN_TRANSIENT_SCREEN_TICKS     (ALARM_VERIFY_MS / HANDLESCAN_TASK_PERIOD_MS) /* 临时校验报警显示多久，单位为扫描次数，时长由 ALARM_VERIFY_MS 决定。 */

/*
 * 串口调试步骤码定义。
 * `HANDLESCAN_TRACE_ENABLE` 是当前 handlescan 驱动的总报文开关：
 * 1. 设为 `1U` 时，输出 `HS`、`HSDBG`、`HSNAME` 基础调试报文；
 * 2. 设为 `0U` 时，基础调试报文都会被静默处理，业务状态机保持不变；
 * 3. 刀具规格报文还要求 `HANDLESCAN_TOOL_TRACE_ENABLE` 为 1，两项开关缺一不可。
 */
#define HANDLESCAN_TRACE_ENABLE               0U /* 1 输出 HS/HSDBG/HSNAME 调试串口信息，0 不输出；不改变识别规则，但输出会占用时间。 */
#define HANDLESCAN_TOOL_TRACE_ENABLE          0U /* 与基础开关都为 1 才输出刀具规格信息；任一为 0 都不输出。 */

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
 * 手柄扫描处理到哪一步。A/B 使用相同流程，但各自保存进度和计数，互不覆盖。
 */
typedef enum
{
    HANDLESCAN_STAGE_IDLE = 0,//空闲/待机状态,等待插入
    HANDLESCAN_STAGE_DEBOUNCE_IN,//连续检查插入电平，确认不是短暂干扰
    HANDLESCAN_STAGE_WAIT_VERIFY,//插入已确认，等待接口和 EEPROM 稳定
    HANDLESCAN_STAGE_VERIFY,//校验 EEPROM 中的手柄数据
    HANDLESCAN_STAGE_READ_INFO,//读取信息状态,手柄型号，内容信息
    HANDLESCAN_STAGE_WAIT_RFID_TOOL,//如果是RFID工具手柄，等待RFID标签识别完成
    HANDLESCAN_STAGE_ONLINE,//手柄正式接入系统，可以正常工作
    HANDLESCAN_STAGE_DEBOUNCE_OUT,//连续检查拔出电平，确认是否真的拔出
    HANDLESCAN_STAGE_RETRY_WAIT,//验证或读取失败，等待一段时间后重试
    HANDLESCAN_STAGE_VERIFY_FAIL//身份验证失败，手柄被拒绝接入
} HandlescanStage;

typedef enum
{
    HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3 = 0, /* 从手柄自身 EEPROM 第 3 页取得刀具信息。 */
    HANDLESCAN_TOOL_SOURCE_RFID_EPC /* 从可拆刀具的 RFID 标签 EPC 区取得刀具信息。 */
} HandlescanToolSource;

/*
 * 插入和拔出去抖计数器。
 * `in_debounce_ticks` 用于统计插入稳定时间，`out_debounce_ticks` 用于统计拔出稳定时间。
 */
typedef struct
{
    uint8_t in_debounce_ticks; /* 连续读到插入电平的扫描次数，断开一次就清零。 */
    uint8_t out_debounce_ticks; /* 连续读到拔出电平的扫描次数，恢复插入时清零。 */
} HandlescanDebounce;

/*
 * 保存每个通道的扫描进度，下一个 10ms 周期从这里继续处理。
 * A/B 各有一份，EEPROM 数据和 RFID 计数不能混用。
 * 识别结果、通道记忆和当前工作参数仍分别保存在 ChannelrecognizeMessage、MemoryMsg、WorkMessage。
 */
typedef struct
{
    HandlescanStage stage;                                  /* 当前扫描阶段，静态零值对应空闲态。 */
    volatile uint8_t unsupported_latched;                   /* 型号不在当前模式白名单时置1，仅本通道确认拔出后清零。 */
    AT24CS32_CRC_StepContext verify_context;                  /* 保存本通道的认证进度和页数据，A/B交替读取时不会覆盖另一通道的数据。 */
    volatile uint32_t navigation_generation;                /* 进入拔出确认时加 1；外部导航读取前后比较此数，不同就放弃旧读取。 */
    HandlescanDebounce debounce;                             /* 插入和拔出去抖计数，单位都是 10ms 扫描周期。 */
    uint8_t offline_event_required;                          /* 已发布过本通道上线事件后置 1；只有离线事件成功入队才清零，防止短接毛刺或队列满造成永久假在线。 */
    uint8_t verify_start_wait_ticks;                         /* 插入稳定后、开始 EEPROM 认证前的等待计数。 */
    uint8_t last_alarm;                                     /* 本通道最近一次手柄认证报警码，用于恢复或拔出时清报警。 */
    volatile uint8_t verify_alarm_close_pending;             /* 最后一项校验报警已清除时置 1，等待插拔任务完成 UI 队列复位后补发关窗。 */
    uint8_t verify_retry_count;                              /* 当前认证连续失败次数，达到上限后进入最终失败保持态。 */
    uint16_t verify_retry_wait_ticks;                        /* 认证失败后的重试等待计数，也用于最终失败态的低频自恢复。 */
    uint8_t info_buf[HANDLESCAN_INFO_SIZE];                  /* EEPROM Page2 手柄身份缓存，保存主型号和子型号。 */
    uint8_t tool_info_buf[HANDLESCAN_TOOL_INFO_SIZE];        /* EEPROM Page3完整业务区缓存，A/B不得交叉复用，且不能截断PX01速度扩展。 */
    uint8_t initial_info_buf[AT24CS32_PAGE_SIZE];            /* EEPROM Page4 初始参数缓存，保存速度、流量和保护阈值。 */
    uint8_t speed_step_buf[AT24CS32_PAGE_SIZE];              /* EEPROM Page6 调速步进缓存，供屏幕加减速继续使用。 */
    HandlescanToolSource tool_source;                        /* 当前刀具信息来源，静态零值对应 EEPROM Page3。 */
    uint16_t rfid_wait_ticks;                                /* 手柄上线时等待 RFID 结果的 10ms 计数。 */
    uint16_t rfid_monitor_ticks;                             /* 基座在线后发起低频 RFID 监测的 10ms 计数。 */
    uint16_t rfid_last_sequence;                             /* 已经处理过的 RFID 结果编号，避免重复更新参数或蜂鸣。 */
    uint16_t rfid_last_presence_sequence;                    /* 本通道最近一次确认标签存在的结果序号。 */
    uint8_t rfid_miss_count;                                 /* 在线监测连续未读到标签的次数，用于确认刀具头离线。 */
    uint8_t rfid_monitor_pending;                            /* 1 表示上一轮 RFID 读取还没有成功、超时或取消的结果；0 表示没有等待中的读取。 */
    uint16_t rfid_monitor_ticket;                            /* 首次读取和在线监测共用的本轮请求编号，实际完成后立即交接，不能串用另一轮结果。 */
    uint8_t rfid_tool_online;                                /* 1 表示已识别到刀具，0 表示未识别到；只在有无刀具发生变化时提示。 */
    uint8_t rfid_wait_started;                               /* 首次RFID事务等待期间置1，读取结束或基座拔出后清零，不再重复认证EEPROM等待刀具。 */
} HandlescanChannelContext;

/*
 * 列出一个通道使用的引脚、数据缓存和插拔消息编号，运行中不更改。
 * 扫描函数先选 A 或 B 的这张表，再只访问该通道；会变化的进度和计数另存。
 */
typedef struct
{
    uint8_t channel;                                       /* 通道号，决定 EEPROM 总线、RFID 结果和报警归属。 */
    GPIO_TypeDef *short_gpio;                              /* 手柄短接检测 GPIO 端口，低电平表示手柄已插入。 */
    uint16_t short_pin;                                    /* 手柄短接检测引脚，只用于读取插拔状态。 */
    HandlescanChannelContext *context;                     /* 本通道跨 10ms 周期保存的扫描运行状态。 */
    ChannelrecognizeMessage_t *message;                    /* 本通道识别缓存，上线事件会把它装入通道记忆。 */
    ChannelMemoryMessage_t *memory;                        /* 本通道记忆，仅保留原有 auto_identify 更新顺序。 */
    uint32_t *spec_values;                                 /* 本通道刀具规格显示缓存，A/B 不得交叉使用。 */
    uint8_t plug_key;                                      /* 手柄或 RFID 刀具上线时发送的屏幕事件键值。 */
    uint8_t unplug_key;                                    /* 手柄确认拔出时发送的屏幕事件键值。 */
} HandlescanChannelBinding;

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
    uint8_t first_byte; /* EEPROM 型号的第一个字节。 */
    uint8_t second_byte; /* EEPROM 型号的第二个字节，两个字节都相同才算匹配。 */
    uint8_t mapped_handle_type; /* 匹配后交给主控使用的型号编号。 */
    const char *handle_name; /* 调试串口显示的型号名称，不参与功能判断。 */
} HandlescanHandleTypeConfig;

/*
 * 当前已支持的手柄类型映射表。
 * 手柄信息来自 EEPROM 第 2 页，0x6B 为普通基座码，0x7C/03..06 为一体式手柄型号码；MXYTP/MXYTM改由公共接头RFID标签识别。
 */
static const HandlescanHandleTypeConfig s_hand_type_config_table[] =
{
    {0x6B, 0x01, TMBB_ONLINES, "TMBB"},//无刷，单向
    {0x6B, 0x02, TMBA_ONLINES,  "TMBA"},//无刷，单向
    {0x6B, 0x03, EMBA_ONLINES, "EMBA"},//无刷，单向
    {0x6B, 0x04, EMBB_ONLINES,  "EMBB"},//无刷，单向
    {0x6B, 0x05, PXBA_ONLINES,  "PXBA"},//PXBA 分体式无刷手柄；刀具类型和速度由后续 EEPROM/RFID 读取决定
    {0x6B, 0x06, PXBB_ONLINES,  "PXBB"},//PXBB 分体式无刷手柄；本表只识别基座，不在这里设定往复频率或转速

    {0x7C, 0x03, PX_YIM_ONLINES,   "PXYTM"},//有刷，单向
    {0x7C, 0x04, PX_YIP_ONLINES,   "PXYTP"},//有刷，往复
    {0x7C, 0x05, JMB_ONLINES,      "JMB"},//无刷，单向
    {0x7C, 0x06, MX_YIM16_ONLINES, "MXYTM16"},//无刷，单向
    {0x6B, 0x07, LGZ_I_ONLINES, "LGZ_I"},                 /* 颅骨钻一型预留，先按 Page2 顺序占位。无刷，单向 */
    {0x6B, 0x08, LGZ_II_ONLINES, "LGZ_II"},               /* 颅骨钻二型预留，后续如协议变更只改本表。无刷，单向 */
    {0x6B, 0x09, KSZ_I_ONLINES, "KSZ_I"},                 /* 克氏针一型预留，识别后供 UI 显示。无刷，单向 */
    {0x6B, 0x0A, KSZ_II_ONLINES, "KSZ_II"},               /* 克氏针二型预留，识别后供 UI 显示。无刷，单向 */
    {0x6B, 0x0B, KXZ_I_ONLINES, "KXZ_I"},                 /* 空心钻一型预留，沿用当前扫描状态机。无刷，单向 */
    {0x6B, 0x0C, KXZ_II_ONLINES, "KXZ_II"},               /* 空心钻二型预留，沿用当前扫描状态机。无刷，单向 */
    {0x6B, 0x0D, COMMON_SOCKET_ONLINES, "COMMON_SOCKET"}, /* 公共接头没有实体键，只作为在线类型和 UI 占位。无刷，单向 */
    {0x6B, 0x0E, EMBD_ONLINES, "EMBD"},                   /* EMBD 6万增速手柄按普通电动手柄上线，不额外开放实体键控制。无刷，单向 */
    {0x6B, 0x0F, EMBC_ONLINES, "EMBC"},                   /* EMBC 新增普通电动手柄，按 EEPROM 第二页 0x6B/0x0F 识别。无刷，单向 */
    {0x6B, 0x10, DHYTM_ONLINES, "DHYTM"},                 /* DHYTM 使用独立 EEPROM 参数，不读取 RFID；屏幕固定反转且电机实际正转。 */
    {0x6B, 0x11, GZYT_ONLINES, "GZYT"}                    /* GZYT 使用独立 EEPROM 参数；识别在线后由按键任务把本通道按钮脚切为 UART 接收。 */
};

/*
 * 当前已支持的刀具类型映射表。
 * 刀具信息来自 EEPROM 第 3 页，类型码固定以 `0x7C` 开头。
 * 后续如果新增刀具型号，只需追加新的 `{主类型, 子类型, Handle_Type_xx, "名称"}` 表项即可。
 */
static const HandlescanHandleTypeConfig s_tool_type_config_table[] =
{
    {0x7C, 0x07, LGZ_I_ONLINES, "LGZ_I_TOOL"},                 /* 刀具类型预留，先按 Page3 顺序占位。 */
    {0x7C, 0x08, LGZ_II_ONLINES, "LGZ_II_TOOL"},               /* 颅骨钻二型刀具预留，后续协议变更只改本表。 */
    {0x7C, 0x09, KSZ_I_ONLINES, "KSZ_I_TOOL"},                 /* 克氏针一型刀具预留，识别后供 UI 和上位机读取。 */
    {0x7C, 0x0A, KSZ_II_ONLINES, "KSZ_II_TOOL"},               /* 克氏针二型刀具预留，识别后供 UI 和上位机读取。 */
    {0x7C, 0x0B, KXZ_I_ONLINES, "KXZ_I_TOOL"},                 /* 空心钻一型刀具预留，沿用 Page3 解析流程。 */
    {0x7C, 0x0C, KXZ_II_ONLINES, "KXZ_II_TOOL"},               /* 空心钻二型刀具预留，沿用 Page3 解析流程。 */
    {0x7C, 0x0D, COMMON_SOCKET_ONLINES, "COMMON_SOCKET_TOOL"}  /* 公共接头刀具位保留，便于完整保存类型编号。 */
};

/* A/B 通道各自保存一份扫描运行状态，禁止跨通道共用。 */
static HandlescanChannelContext s_a_context;                  /* 文件静态对象自动清零，保持 A 通道原有初始状态。 */
static HandlescanChannelContext s_b_context;                  /* 文件静态对象自动清零，保持 B 通道原有初始状态。 */

/*
 * 函数功能：读取两侧不支持手柄的插入锁存状态，供报警、蜂鸣和屏幕共用。
 * 输入参数：无。
 * 返回参数：任一拒绝的手柄仍未确认拔出时返回 true，否则返回 false。
 */
bool Handlescan_HasUnsupportedHandle(void)
{
    return ((s_a_context.unsupported_latched != 0U) ||
            (s_b_context.unsupported_latched != 0U)); /* 不直接读取瞬时GPIO，避免拔出毛刺或只拔一侧提前解除报警。 */
}

/*
 * 函数功能：按产品模式限定可上线型号，并由数字按钮测试开关临时开放 GZYT，A/B 共用此规则。
 * 输入参数：mapped_model 为 Page2 双字节映射得到的内部型号，0表示未知型号。
 * 返回参数：属于当前模式名单或已开启临时测试的 GZYT 时返回 true，其余返回 false。
 */
static bool Handlescan_IsModelSupported(uint8_t mapped_model)
{
    switch (mapped_model)
    {
    case GZYT_ONLINES: /* GZYT 临时开放独立于正式产品名单，不影响其它型号的模式限制。 */
        return (GZYT_DIGITAL_TEST_ENABLE == 1U); /* 测试开关开启时允许 A/B 上线；关闭后恢复拒绝，EEPROM 校验仍须通过。 */
    case TMBB_ONLINES:
    case TMBA_ONLINES:
    case EMBA_ONLINES:
    case EMBB_ONLINES:
    case COMMON_SOCKET_ONLINES:
    case DHYTM_ONLINES:
        return (HANDLE_FUNCTION_MODE != HANDLE_FUNCTION_SHAVING_ONLY); /* 六种磨削手柄只在双功能或磨削模式上线。 */
    case PXBA_ONLINES:
    case PXBB_ONLINES:
    case PX_YIP_ONLINES:
    case PX_YIM_ONLINES:
        return (HANDLE_FUNCTION_MODE != HANDLE_FUNCTION_GRINDING_ONLY); /* PXYTP/PXYTM均按刨削产品分类，不按刀具名称猜测。 */
    default:
        return false; /* 未列出的已知型号和完全未知的型号均不得进入运行参数装载。 */
    }
}

/* A/B 固定绑定只描述资源归属；运行时不会改写，防止泵位、I2C 总线或事件键值串通道。 */
static const HandlescanChannelBinding s_a_binding =
{
    CHANNEL_A, HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN,
    &s_a_context, &ChannelrecognizeMessageA, &MemoryMsgA, paoxueSpeciValue_A,
    SCREENKey_PLUG_A, SCREENKey_UNPLUG_A
};
static const HandlescanChannelBinding s_b_binding =
{
    CHANNEL_B, HANDLESCAN_B_SHORT_GPIO, HANDLESCAN_B_SHORT_PIN,
    &s_b_context, &ChannelrecognizeMessageB, &MemoryMsgB, paoxueSpeciValue_B,
    SCREENKey_PLUG_B, SCREENKey_UNPLUG_B
};

/*
 * 函数功能：根据 A/B 通道号取得该通道的引脚和数据缓存。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：返回该通道的配置；不是 A/B 时返回 NULL，调用方不得继续访问硬件。
 */
static const HandlescanChannelBinding *Handlescan_GetBinding(uint8_t channel)
{
    if (channel == CHANNEL_A)
    {
        return &s_a_binding; /* 逻辑A固定关联A侧业务缓存，物理接口由统一交换配置决定。 */
    }
    if (channel == CHANNEL_B)
    {
        return &s_b_binding; /* 逻辑B固定关联B侧业务缓存，物理接口由统一交换配置决定。 */
    }

    return NULL; /* 非 A/B 通道不允许回退到任意一侧，避免写错业务状态。 */
}

/*
 * 函数功能：检查已插入手柄是否仍在首次认证、参数装载或首读RFID，供启动页等待识别收尾。
 * 输入参数：无。
 * 返回参数：任一已插入通道尚未上线且未明确认证失败时返回true；空接口不参与等待。
 */
bool Handlescan_IsStartupPending(void)
{
    uint8_t channel; /* 按真实插口逐侧判断，不能只等待当前选中的手柄。 */
    for (channel = CHANNEL_A; channel <= CHANNEL_B; ++channel)
    {
        const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 使用同一套引脚和扫描状态。 */
        if (Handlescan_IsChannelPhysicallyInserted(channel) == false)
        {
            continue; /* 未插手柄不阻塞启动页，也不要求用户把所有接口接满。 */
        }
        if (binding->context->stage == HANDLESCAN_STAGE_VERIFY_FAIL)
        {
            continue; /* 已明确认证失败或型号不支持时允许显示故障，不在启动页无限等待。 */
        }
        if ((binding->context->stage != HANDLESCAN_STAGE_ONLINE) ||
            ((channel == CHANNEL_A) ? (WorkMessage.Channel_Aonline == false) : (WorkMessage.Channel_Bonline == false)))
        {
            return true; /* 扫描完成后还要等上线消息装入通道记忆，避免切页时显示旧参数。 */
        }
    }
    return false; /* 已插入的手柄均已识别完成或明确失败，空基座不要求一直等到有RFID刀具。 */
}

/*
 * 函数功能：确认指定通道最近有效RFID结果已装载，供外控区分当前有效刀具和历史锁存参数。
 * 输入参数：channel为CHANNEL_A或CHANNEL_B。
 * 返回参数：true表示该通道刀具在线且已装载最新有效结果；false表示未识别、读取失败或仍待装载。
 */
bool Handlescan_IsRfidToolReady(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 读取指定通道的扫描状态，不借用当前屏幕通道。 */
    RfidToolResult_t result; /* 复制最近一次完整结果，避免只根据历史刀具类型放行。 */
    if ((binding == NULL) || (binding->context->rfid_tool_online == 0U))
    {
        return false; /* 扫描任务尚未确认刀具上线，外控不能把历史锁存参数上报为当前有效RFID。 */
    }
    if (!Rfid_CopyLastResult(channel, &result))
    {
        return false; /* EPC和USER均失败后结果已失效，即使保留规格也不作为当前有效RFID上报。 */
    }
    return (result.sequence == binding->context->rfid_last_sequence); /* 最新结果必须已经完成参数装载，不能在换刀的两个任务之间使用旧参数。 */
}

/*
 * 函数功能：读取指定逻辑通道当前的手柄短接检测脚，供驱动报警显示前区分真实故障和物理拔柄过程。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：低电平表示手柄仍物理插入时返回 true；高电平、非法通道返回 false。
 */
bool Handlescan_IsChannelPhysicallyInserted(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 复用扫描状态机的同一套逻辑通道与GPIO绑定，避免报警模块重复硬件映射。 */

    if (binding == NULL)
    {
        return false; /* 非 A/B 通道没有实体短接检测脚，不能据此显示驱动报警图片。 */
    }

    return (Bsp_GpioRead(binding->short_gpio, binding->short_pin) == GPIO_PIN_RESET); /* 与10ms扫描任务保持同一极性：低电平为插入，高电平为拔出候选。 */
}
/*
 * 函数功能：判断指定通道的手柄 EEPROM 是否已经完成本轮识别，可供外部通信读取导航页。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：扫描状态已稳定上线返回 true；重识别、拔出消抖、离线或非法通道返回 false。
 */
bool Handlescan_IsNavigationReady(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 只读取指定逻辑通道的扫描上下文，避免 A/B 状态串用。 */

    if (binding == NULL)
    {
        return false; /* 非 A/B 通道没有实体 EEPROM，不能对外声明导航数据已就绪。 */
    }

    return (binding->context->stage == HANDLESCAN_STAGE_ONLINE); /* 只有整轮 EEPROM 认证和业务页装载完成后才允许外部导航访问。 */
}

/*
 * 函数功能：读取手柄插拔变化计数，让外部导航判断读取期间是否发生过短暂插拔。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：返回本通道的 32 位变化计数；不是 A/B 时返回 0。计数溢出后从 0 继续。
 */
uint32_t Handlescan_GetNavigationGeneration(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 读取参数指定的通道，不以屏幕当前选中的通道代替。 */

    if (binding == NULL)
    {
        return 0U; /* 不是 A/B 时没有可读计数；调用方仍须检查通道是否合法。 */
    }

    return binding->context->navigation_generation; /* 只读内存中的计数，不读 EEPROM，也不刷新屏幕。 */
}


/*
 * 函数功能：取出并清除指定通道的关报警窗请求，该请求在校验失败的手柄拔出后产生。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B，指定要检查哪个通道。
 * 返回参数：true 表示需要在无手柄 UI 队列复位后补发关窗；false 表示没有待处理请求。
 */
bool Handlescan_TakeVerifyAlarmCloseRequest(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 按事件通道读取同一侧扫描上下文，避免 A/B 关窗请求串用。 */

    if (binding == NULL)
    {
        return false;                                        /* 非 A/B 通道没有合法扫描状态，不处理A侧或B侧的请求。 */
    }

    if (binding->context->verify_alarm_close_pending == 0U)
    {
        return false;                                        /* 本通道没有被队列复位影响的校验报警关窗请求。 */
    }

    binding->context->verify_alarm_close_pending = 0U;       /* 取出后立即清零，避免后续拔出消息重复关闭其它报警窗。 */
    return true;                                             /* 通知插拔任务在全部清屏动作完成后重新发送关闭命令。 */
}

/* 运行中另一路手柄校验失败时，屏幕提示只保持 3 秒，不写 WorkMessage，避免影响当前工作通道。 */
static uint16_t s_transient_screen_alarm_ticks = 0U;
static uint8_t s_transient_screen_alarm_value = 0U;

/*
 * 函数功能：查询当前仍有效的手柄校验弹窗来源，显示任务据此丢弃旧消息并补关窗。
 * 输入参数：无。
 * 返回参数：持续或尚未到期的A/B/AB校验报警码；校验已恢复或其它持续故障接管时返回0。
 */
uint8_t Handlescan_GetVerifyAlarmForDisplay(void)
{
    if (WorkMessage.alarm_flag != false) /* 持续报警优先，临时校验提示不能覆盖驱动等真实故障。 */
    {
        if ((WorkMessage.alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_A) ||
            (WorkMessage.alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_B) ||
            (WorkMessage.alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_AB))
        {
            return WorkMessage.alarm_value; /* 另一通道仍校验失败时沿用合并后的实际报警归属。 */
        }
        return WORK_ALARM_NONE; /* 其它故障和71号不支持手柄由各自显示流程维护。 */
    }
    return (s_transient_screen_alarm_ticks != 0U) ? s_transient_screen_alarm_value : WORK_ALARM_NONE; /* 运行中另一侧的临时校验提示仅在原3秒窗口内有效。 */
}

static void Handlescan_ClearChannelAlarm(uint8_t channel, uint8_t alarm_value);
static void Handlescan_ClearToolSpecValues(uint32_t *spec_values);
static uint8_t Handlescan_ReadEepromBytes(const HandlescanChannelBinding *binding,
                                          uint16_t addr,
                                          uint8_t *data,
                                          uint16_t len);
static uint8_t Handlescan_ReadEepromPage(const HandlescanChannelBinding *binding,
                                         uint16_t page_index,
                                         uint8_t *page_buf);
static bool Handlescan_IsManualSplitMode(const HandlescanChannelBinding *binding);
static bool Handlescan_LoadSplitBaseEepromRuntime(const HandlescanChannelBinding *binding);
static void Handlescan_ReloadSpeedStepMessage(const HandlescanChannelBinding *binding);
static void Handlescan_ClearOnlineRfidTool(const HandlescanChannelBinding *binding,
                                           uint8_t mapped_model,
                                           uint8_t raw_type_major,
                                           uint8_t raw_type_minor);
static void Handlescan_EnterRetryOrFail(uint8_t channel,
                                        HandlescanChannelContext *context,
                                        uint8_t set_alarm);
static void Handlescan_BeepOnceIfNoAlarm(void);

/*
 * 函数功能：调试开关打开时输出 HS 报文，说明哪个通道处理到哪一步。
 * 输入参数：channel 为通道号；step 为步骤码；value 为该步骤的结果值。
 * 返回参数：无。
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
    /* 格式化失败时没有有效报文可发送，直接退出以免把未初始化缓冲区写到调试串口。 */
    if (text_len <= 0)
    {
        return;
    }

    /* snprintf 返回原始所需长度，超出缓冲区时按实际可用字节截断，避免串口越界读取。 */
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
 * 函数功能：调试开关打开时输出最近一次 EEPROM 访问的地址、长度和 I2C 错误，供排查读取失败。
 * 输入参数：channel 为本次访问的通道号。
 * 返回参数：无。
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
    /* 格式化失败时不发送 I2C 诊断帧，避免输出无效缓冲区内容干扰现场判断。 */
    if (text_len <= 0)
    {
        return;
    }

    /* 诊断文本超过本地缓冲区时只发送已写入部分，防止调试输出越界读取。 */
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
 * 函数功能：按两个型号字节在指定表中查找手柄或刀具配置。
 * 输入参数：config_table 为配置表；config_count 为表项数；first_byte/second_byte 为 EEPROM 的两个型号字节。
 * 返回参数：找到时返回表项地址；找不到返回 NULL。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindTypeConfig(const HandlescanHandleTypeConfig *config_table,
                                                                   uint32_t config_count,
                                                                   uint8_t first_byte,
                                                                   uint8_t second_byte)
{
    uint32_t index;

    for (index = 0U; index < config_count; ++index)
    {
        /* 两个 EEPROM 类型字节必须同时匹配，才能把该表项作为当前手柄或刀具配置。 */
        if ((config_table[index].first_byte == first_byte) &&
            (config_table[index].second_byte == second_byte))
        {
            return &config_table[index];
        }
    }

    return NULL;
}

/*
 * 函数功能：根据 EEPROM 第 2 页前两个字节查找手柄配置。
 * 输入参数：first_byte/second_byte 为手柄主型号和子型号字节。
 * 返回参数：找到时返回表项地址；找不到返回 NULL，交给调用方按未知手柄处理。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindHandleTypeConfig(uint8_t first_byte, uint8_t second_byte)
{
    return Handlescan_FindTypeConfig(s_hand_type_config_table,
                                     (uint32_t)(sizeof(s_hand_type_config_table) / sizeof(s_hand_type_config_table[0])),
                                     first_byte,
                                     second_byte);
}

/*
 * 函数功能：根据 EEPROM 第 3 页前两个字节查找刀具配置。
 * 输入参数：first_byte/second_byte 为刀具主型号和子型号字节。
 * 返回参数：找到时返回表项地址；找不到返回 NULL，交给调用方处理。
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
 * 返回参数：EEPROM 第三页或 RFID EPC。
 */
static HandlescanToolSource Handlescan_GetToolSource(uint8_t mapped_handle_type)
{
    if (mapped_handle_type == COMMON_SOCKET_ONLINES)
    {
        return HANDLESCAN_TOOL_SOURCE_RFID_EPC; /* 公共接头式可拆手柄，刀具头信息来自 RFID EPC。 */
    }

    if ((mapped_handle_type == PXBA_ONLINES) || (mapped_handle_type == PXBB_ONLINES))
    {
        return HANDLESCAN_TOOL_SOURCE_RFID_EPC; /* 分体式 PXBA/PXBB 已统一到 EPC 协议，刀具头信息只读取 EPC 区。 */
    }

    return HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 其它不可拆普通手柄继续读取 EEPROM 第三页。 */
}

/*
 * 函数功能：根据手柄/刀具能力判断 Page4 默认方向是否允许直接进入往复。
 * 输入参数：message 为已经写入手柄型号和刀具能力的通道识别缓存。
 * 返回参数：true 表示允许把 Page4[8]=0x03 装载为 OSCDIR；false 表示必须保护回正转。
 */
static bool Handlescan_SupportsOscDirection(const ChannelrecognizeMessage_t *message)
{
    bool handle_supported = false;                            /* 记录手柄本体是否具备往复硬件基础，屏幕往复入口必须先满足这一层。 */
    bool tool_supported = false;                              /* 记录当前刀具能力是否为刨刀，避免磨头或未知刀具误进入往复。 */

    if (message == NULL)
    {
        return false; /* 识别缓存不存在时不能确认往复能力，默认按单向手柄保护。 */
    }

    handle_supported = ((message->handle_type == PXBA_ONLINES) ||
                        (message->handle_type == PXBB_ONLINES) ||
                        (message->handle_type == PX_YIP_ONLINES)); /* EEPROM Page4只对真实支持电气往复的手柄生效，MXYTP改由RFID标签单独处理。 */

    tool_supported = (message->tool_type == PLANER);          /* 当前刀具能力必须已经归一为刨刀，MXYTP/PXYTP 不再作为旧刀具码兼容。 */

    return (handle_supported && tool_supported);              /* 同时满足手柄和刀具两层能力时，才把默认方向装载为 OSCDIR。 */
}

/*
 * 函数功能：读取 Page3 的减速比或增速比，乘 100 后打包保存，供电机转速换算使用。
 * 输入参数：tool_info_buf 为至少包含 Page3[0..16] 的刀具信息缓存。
 * 返回参数：低 16 位为 x100 减速比，高 16 位为 x100 增速比；异常或未配置时返回 100。
 */
static uint32_t Handlescan_BuildToolReductionRatio(const uint8_t *tool_info_buf)
{
    uint16_t reduction_ratio; /* 保存解析后的 x100 减速比，最大 25599，可完整放入低 16 位。 */
    uint16_t speed_up_ratio;  /* 保存解析后的 x100 增速比，最大 25599，可完整放入高 16 位。 */
    bool decimal_format;      /* true 表示 GR02 和两个百分位均有效，本次使用两位小数格式。 */

    if (tool_info_buf == NULL)
    {
        return HANDLESCAN_TOOL_RATIO_UNIT; /* 缓存无效时按 1.00 倍直联，禁止沿用上一把手柄倍率。 */
    }

    decimal_format = (bool)((tool_info_buf[HANDLESCAN_TOOL_RATIO_MARK_OFFSET] == HANDLESCAN_TOOL_RATIO_MARK_0) &&
                            (tool_info_buf[HANDLESCAN_TOOL_RATIO_MARK_OFFSET + 1U] == HANDLESCAN_TOOL_RATIO_MARK_1) &&
                            (tool_info_buf[HANDLESCAN_TOOL_RATIO_MARK_OFFSET + 2U] == HANDLESCAN_TOOL_RATIO_MARK_2) &&
                            (tool_info_buf[HANDLESCAN_TOOL_RATIO_MARK_OFFSET + 3U] == HANDLESCAN_TOOL_RATIO_MARK_3) &&
                            (tool_info_buf[HANDLESCAN_TOOL_REDUCTION_DECIMAL_OFFSET] <= HANDLESCAN_TOOL_RATIO_DECIMAL_MAX) &&
                            (tool_info_buf[HANDLESCAN_TOOL_SPEED_UP_DECIMAL_OFFSET] <= HANDLESCAN_TOOL_RATIO_DECIMAL_MAX)); /* 标记和两个百分位必须同时有效，避免半新半旧解析。 */

    reduction_ratio = (uint16_t)((uint16_t)tool_info_buf[HANDLESCAN_TOOL_REDUCTION_RATIO_OFFSET] * HANDLESCAN_TOOL_RATIO_X100_UNIT); /* 历史整数部分先统一换成 x100。 */
    speed_up_ratio = (uint16_t)((uint16_t)tool_info_buf[HANDLESCAN_TOOL_SPEED_UP_RATIO_OFFSET] * HANDLESCAN_TOOL_RATIO_X100_UNIT); /* 增速整数部分使用同一 x100 单位。 */
    if (decimal_format != false)
    {
        reduction_ratio = (uint16_t)(reduction_ratio + tool_info_buf[HANDLESCAN_TOOL_REDUCTION_DECIMAL_OFFSET]); /* GR02 有效时追加减速百分位，例如 5 和 25 组合为 525。 */
        speed_up_ratio = (uint16_t)(speed_up_ratio + tool_info_buf[HANDLESCAN_TOOL_SPEED_UP_DECIMAL_OFFSET]); /* GR02 有效时追加增速百分位，保留两位小数。 */
    }

    if ((reduction_ratio > HANDLESCAN_TOOL_RATIO_UNIT) &&
        (speed_up_ratio > HANDLESCAN_TOOL_RATIO_UNIT))
    {
        return HANDLESCAN_TOOL_RATIO_UNIT; /* 两个倍率同时大于 1.00 说明 EEPROM 配置矛盾，为保护电机按直联处理。 */
    }

    if (speed_up_ratio > HANDLESCAN_TOOL_RATIO_UNIT)
    {
        return ((uint32_t)speed_up_ratio << HANDLESCAN_TOOL_SPEED_UP_SHIFT); /* 增速机构把完整 x100 倍率写入高 16 位，低 16 位保持 0。 */
    }

    if (reduction_ratio > HANDLESCAN_TOOL_RATIO_UNIT)
    {
        return (uint32_t)reduction_ratio; /* 减速机构把完整 x100 倍率写入低 16 位，高 16 位保持 0。 */
    }

    return HANDLESCAN_TOOL_RATIO_UNIT; /* 0、1.00 或小于 1.00 的异常值都按 1.00 倍直联。 */
}



/*
 * 函数功能：把 EPC 标签刀具型号转换为业务刀具能力。
 * 输入参数：tool_model为EPC byte0，协议定义0x01普通刨刀、0x02普通磨刀、0x03反旋、0x04 MXYTP、0x05 MXYTM、0x06反向刨刀。
 * 返回参数：PLANER或GRINDH；普通刨刀、MXYTP和反向刨刀按刨刀外观显示，其它机械刀具按磨刀外观显示，未知值按磨刀保护。
 */
static uint8_t Handlescan_MapRfidToolType(uint8_t tool_model)
{
    if ((tool_model == RFID_TOOL_MODEL_PLANER) ||
        (tool_model == RFID_TOOL_MODEL_MXYTP) ||
        (tool_model == RFID_TOOL_MODEL_REVERSE_PLANER))
    {
        return PLANER; /* 普通刨刀、MXYTP和反向刨刀都按刨刀外观显示；各自机械方向能力继续由原始型号区分。 */
    }

    return GRINDH; /* 普通磨刀、反旋刀具、MXYTM和异常值按磨头能力处理，禁止误开放电气往复。 */
}
/*
 * 函数功能：把 Page3 的减速/增速比写入通道识别缓存。
 * 输入参数：message 为目标识别缓存；tool_info_buf 为已读出的 Page3 缓存，读取失败可传 NULL。
 * 返回参数：无。
 */
static void Handlescan_UpdateToolRatioMessage(ChannelrecognizeMessage_t *message, const uint8_t *tool_info_buf)
{
    uint32_t ratio = HANDLESCAN_TOOL_RATIO_UNIT;              /* 默认 100 表示 1.00 倍直联，避免 Page3 读取失败时沿用旧倍率。 */

    if (message == NULL)
    {
        return;                                               /* 识别缓存为空时不写全局状态，防止异常路径串改另一通道。 */
    }

    if (tool_info_buf != NULL)
    {
        ratio = Handlescan_BuildToolReductionRatio(tool_info_buf); /* 按 GR02 新格式或历史整数格式构造完整 x100 倍率。 */
    }

    message->tool_reduction_ratio = ratio;                    /* 保存完整 x100 倍率，后续通道记忆和驱动下发都读取该字段。 */
    message->meioticratio = (tool_info_buf != NULL) ? tool_info_buf[HANDLESCAN_TOOL_REDUCTION_RATIO_OFFSET] : 1U; /* 旧 8 位字段只保留 Page3 减速整数镜像，不能截取 x100 低字节。 */
}

/*
 * 函数功能：把查表后的型号值转换成业务刀具能力。
 * 输入参数：raw_tool_type为Page3刀具型号或仍保留的一体式PXY/MXYTM16/DHYTM手柄型号。
 * 返回参数：PLANER/GRINDH或原值；PXYTP归一为PLANER，PXYTM/MXYTM16/DHYTM归一为GRINDH。
 */
static uint8_t Handlescan_MapToolType(uint8_t raw_tool_type)
{
    if (raw_tool_type == PX_YIP_ONLINES)
    {
        return PLANER; /* PXYTP后缀表示刨削能力，按PLANER开放其原有电气往复和频率入口。 */
    }

    if ((raw_tool_type == PX_YIM_ONLINES) ||
        (raw_tool_type == MX_YIM16_ONLINES) ||
        (raw_tool_type == DHYTM_ONLINES))
    {
        return GRINDH; /* PXYTM、MXYTM16和DHYTM按磨削能力处理；DHYTM的反旋机械方向由驱动输出层单独转换。 */
    }

    return raw_tool_type; /* 其它EEPROM刀具保持原值，避免扩大本次RFID协议适配范围。 */
}
/*
 * 函数功能：判断 EEPROM 第二页识别出的手柄型号是否自带刀具能力。
 * 输入参数：mapped_model 为手柄表映射后的系统内部型号。
 * 返回参数：true表示PXYTM/PXYTP/JMB/MXYTM16/DHYTM由手柄型号直接给出刀具能力；false表示仍需按Page3或RFID读取。
 */
static bool Handlescan_IsSelfTypedHandleModel(uint8_t mapped_model)
{
    return ((mapped_model == PX_YIM_ONLINES) ||     /* PXYTM是一体式有刷磨削手柄，刀具能力仍由Page2型号给出。 */
            (mapped_model == PX_YIP_ONLINES) ||     /* PXYTP是一体式有刷刨削手柄，继续沿用EEPROM手柄识别。 */
            (mapped_model == JMB_ONLINES) ||        /* JMB是独立手柄型号，不能再挂在刀具Page3表中。 */
            (mapped_model == MX_YIM16_ONLINES) ||   /* MXYTM16保持现有独立手柄型号；MXYTP/MXYTM已改由RFID EPC识别。 */
            (mapped_model == DHYTM_ONLINES));       /* DHYTM由Page2型号直接确定磨削能力，Page3只提供2.00倍增速参数。 */
}
/*
 * 函数功能：判断一体式手柄是否需要从手柄自身 EEPROM Page3 显示刀具规格。
 * 输入参数：mapped_model 为 EEPROM Page2 映射后的手柄型号。
 * 返回参数：true表示PXYTM/PXYTP需要显示EEPROM Page3直径、长度和角度；MXYTP/MXYTM规格改由RFID EPC路径显示。
 */
static bool Handlescan_IsIntegratedSpecHandle(uint8_t mapped_model)
{
    return ((mapped_model == PX_YIM_ONLINES) ||  /* PXYTM规格继续来自手柄EEPROM Page3。 */
            (mapped_model == PX_YIP_ONLINES));   /* PXYTP规格继续来自手柄EEPROM Page3；MXY型号改读公共接头EPC。 */
}
/*
 * 函数功能：把 handlescan 的刀具来源转换成 RFID 读取来源。
 * 输入参数：tool_source 为手柄扫描判断出的刀具来源。
 * 返回参数：RFID EPC 或 NONE。
 */
static RfidReadSource_t Handlescan_ToRfidSource(HandlescanToolSource tool_source)
{
    if (tool_source == HANDLESCAN_TOOL_SOURCE_RFID_EPC)
    {
        return RFID_READ_SOURCE_EPC; /* 公共接头和 PXBA/PXBB 当前统一读取 EPC。 */
    }

    return RFID_READ_SOURCE_NONE; /* EEPROM 第三页来源不需要 RFID。 */
}


/*
 * 函数功能：装载 RFID 刀具参数；公共接头和分体式新 EPC 刀具按转速范围分档调速，老 USER 保留型号兼容步进。
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
    const uint8_t *payload; /* 原始标签仅用于最低转速特例。 */
    RfidToolParameters_t decoded; /* 两份协议转换成统一参数，不混用字节位置。 */
    uint8_t tool_model;          /* RFID 标签中的刀具型号或代号结构字段。 */
    uint16_t diameter;           /* 老USER直径乘5后保留完整数值。 */
    uint8_t length;              /* RFID 标签中的长度字段。 */
    uint8_t angle;               /* RFID 标签中的弯曲角度字段。 */
    uint32_t default_speed;      /* RFID 标签转换后的默认速度；EPC 最大可到 255000，必须保留32位。 */
    uint32_t max_speed;          /* RFID 标签转换后的最大速度；EPC 最大可到 255000，必须保留32位。 */
    uint32_t min_speed;          /* RFID 标签转换后的最小速度；统一用32位避免和默认速度比较时截断。 */
    uint32_t reduction_ratio = HANDLESCAN_TOOL_RATIO_UNIT; /* RFID 标签解析出的 x100 完整倍率，默认 100 表示直联。 */
    uint16_t reduction_integer; /* 保存低 16 位减速倍率的整数部分，用于兼容旧 8 位镜像字段。 */
    uint16_t current_threshold; /* 保存解析后的保护电流，单位0.01A；新版EPC换算标签值，老USER使用兼容表。 */
    uint32_t speed_range;       /* 保存公共接头和分体式新 EPC 刀具的最大/最小转速差，计算前先确认上限大于下限，避免无符号下溢。 */
    uint16_t common_speed_step_small; /* 保存公共接头和分体式新 EPC 刀具共用的分档小步进。 */
    uint16_t common_speed_step_large; /* 保存公共接头和分体式新 EPC 刀具共用的分档大步进。 */
    uint8_t default_flow;        /* RFID 标签中的默认泵流量。 */
    uint8_t direction;           /* RFID 标签中的方向字段。 */
    uint8_t business_tool_type = 0U; /* 业务层使用的刀具能力类型，和 RFID 原始型号分开保存。 */
    uint16_t keep_speed_zzstep;  /* 保存基座阶段已从 Page6 读出的正转调速步进，防止 memset 后丢失。 */
    uint16_t keep_speed_fzstep;  /* 保存基座阶段已从 Page6 读出的反转调速步进，RFID 标签不直接携带该参数。 */
    uint16_t keep_speed_oscstep; /* 保存基座阶段已从 Page6 读出的往复调速步进，供新屏快慢调速键使用。 */
    uint16_t keep_speed_zzstep_large;  /* 保存基座阶段已从 Page6 读出的大步进，避免 RFID 刷新后快键退回旧乘 2 逻辑。 */
    uint16_t keep_speed_fzstep_large;  /* 保存反转大步进，保证 A/B 通道和方向切换后仍使用 EEPROM Page6 配置。 */
    uint16_t keep_speed_oscstep_large; /* 保存往复大步进，只有进入往复方向时才由 SpeedActive 使用。 */

    if ((message == NULL) || (spec_values == NULL) || (rfid_result == NULL) || (rfid_result->valid == false))
    {
        return false; /* 防御空指针和无效 RFID 结果，避免错误上线。 */
    }

    if (rfid_result->channel != channel)
    {
        return false; /* RFID 结果必须属于当前通道，避免 A/B 刀具头数据串用。 */
    }

    if (!Rfid_DecodeToolPayload(rfid_result->source, rfid_result->payload, rfid_result->payload_length, &decoded))
    {
        return false; /* 业务CRC或参数失败，不清空也不部分改写识别缓存。 */
    }

    keep_speed_zzstep = message->speed_zzstep; /* RFID 结果刷新前先暂存正转步进，避免清识别缓存时丢掉 EEPROM Page6 参数。 */
    keep_speed_fzstep = message->speed_fzstep; /* RFID 结果刷新前先暂存反转步进，保证换刀具头后仍按基座配置调速。 */
    keep_speed_oscstep = message->speed_oscstep; /* RFID 结果刷新前先暂存往复步进，PLANER 刀具切到往复后可继续用新屏调速。 */
    keep_speed_zzstep_large = message->speed_zzstep_large; /* RFID 结果刷新前同步暂存正转大步进，避免大加/大减键丢配置。 */
    keep_speed_fzstep_large = message->speed_fzstep_large; /* 暂存反转大步进，保持 Page6 大步进和方向状态解耦。 */
    keep_speed_oscstep_large = message->speed_oscstep_large; /* 暂存往复大步进，等待刀具能力确认后继续使用。 */

    payload = rfid_result->payload; /* 原文只用于PX普通刨刀最低转速特例。 */
    {
        HandleEepromProfile_t eeprom_profile = message->eeprom_profile; /* RFID只更新刀具，先保留本通道手柄自身方向和手动电流。 */
        memset(message, 0, sizeof(*message)); /* 清理旧标签字段，防止新刀具继承上一标签参数。 */
        message->eeprom_profile = eeprom_profile; /* 恢复已通过EEPROM校验的本体配置，不被换刀或射频重识别清除。 */
    }

    message->handle_type = mapped_model; /* 手柄基座型号仍来自 EEPROM 第二页，RFID 只更新刀具头信息。 */
    message->hand_type_raw_major = raw_type_major; /* 保存 EEPROM 第二页原始主类型，上位机心跳仍按基座身份显示。 */
    message->hand_type_raw_minor = raw_type_minor; /* 保存 EEPROM 第二页原始子类型，便于区分公共接头/PXBA/PXBB。 */
    message->freq_min = FreqMin; /* RFID 当前不提供频率下限时沿用工程默认。 */
    message->freq_max = FreqMax; /* RFID 当前不提供频率上限时沿用工程默认。 */

    tool_model = decoded.model; /* B0位字段解出型号后才进入机械转向判断。 */
    diameter = decoded.diameter; /* 两种标签的直径已分别按协议转换。 */
    length = decoded.length; /* 保存刀具长度。 */
    angle = decoded.angle; /* 保存弯曲角度。 */
    max_speed = decoded.max_speed; /* 最高转速统一为rpm。 */
    min_speed = decoded.min_speed; /* 老刀下限按型号，新刀按EPC字段。 */
    if ((rfid_result->source == RFID_READ_SOURCE_EPC) &&
        ((mapped_model == PXBA_ONLINES) || (mapped_model == PXBB_ONLINES)) &&
        (tool_model == RFID_TOOL_MODEL_PLANER) && (payload[7] == 0U))
    {
        min_speed = HANDLESCAN_RFID_PLANER_ZERO_MIN_SPEED_RPM; /* 保留PX普通刨刀0表示500rpm的已有约定。 */
    }
    default_speed = decoded.default_speed; /* 错误默认速度已被拒绝，不再通过钳位掩盖标签错误。 */
    default_flow = decoded.flow; /* EPC流量来自B9，USER流量来自B11。 */
    direction = decoded.direction; /* 这里是显示方向，机械转向仍由驱动转换。 */
    reduction_ratio = decoded.ratio; /* 老USER按兼容表取190或430，新刀按EPC倍率。 */
    business_tool_type = Handlescan_MapRfidToolType(tool_model); /* 归一为刨刀或磨刀，另保留机械型号。 */
    reduction_integer = (uint16_t)((reduction_ratio & 0xFFFFU) / HANDLESCAN_TOOL_RATIO_X100_UNIT); /* 生成旧整数字段镜像，完整倍率不丢小数。 */
    message->meioticratio = (reduction_integer > 0xFFU) ? 0xFFU : (uint8_t)reduction_integer; /* 避免旧字段溢出回绕。 */
    current_threshold = decoded.current; /* 新EPC已换算为0.01A，老USER已由兼容表补齐。 */
    message->overloadThresholdFor = current_threshold; /* 为正转运行保存本次刀具的保护电流。 */
    message->overloadThresholdRev = current_threshold; /* 为反转运行保存本次刀具的保护电流。 */
    message->overloadThresholdOSC = current_threshold; /* 为往复运行保存本次刀具的保护电流。 */
    message->freq_default = decoded.frequency; /* USER取B10，新电气往复刀默认40。 */
    message->run_direction = direction; /* 默认方向已确认属于允许集合。 */
    message->rfid_result_sequence = rfid_result->sequence; /* 记录本次装载的完整标签编号，通道记忆和工作参数必须同步该编号。 */
    message->rfid_direction_mask = decoded.direction_mask; /* 保存允许方向，后续通道记忆和外控使用相同限制。 */
    message->tool_type = business_tool_type; /* 保存业务刀具类型，屏幕/方向/开口定位按 PLANER/GRINDH 判断。 */
    message->raw_tool_type = tool_model; /* 保存 RFID 原始刀具型号，供上位机扩展和售后核对标签原值。 */
    message->diameter = (diameter > 255U) ? 255U : (uint8_t)diameter; /* 旧镜像为8位，超范围饱和；完整直径仍写入屏幕规格数组。 */
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
    message->speed_zzstep = keep_speed_zzstep; /* 先恢复正转基础步进；老 USER 在下方按型号覆盖，新 EPC 按有效转速范围覆盖。 */
    message->speed_fzstep = keep_speed_fzstep; /* 先恢复反转基础步进，固定转速刀具不分档时仍保留原配置。 */
    message->speed_oscstep = keep_speed_oscstep; /* 先恢复往复基础步进，后续与正反转一起按本次标签来源覆盖。 */
    message->speed_zzstep_large = keep_speed_zzstep_large; /* 先恢复正转基础大步进，后续有效刀具参数决定屏幕快加/快减步长。 */
    message->speed_fzstep_large = keep_speed_fzstep_large; /* 先恢复反转基础大步进，未进入分档时不改变原快调配置。 */
    message->speed_oscstep_large = keep_speed_oscstep_large; /* 先恢复往复基础大步进，随后按老 USER 兼容表或新 EPC 范围统一覆盖。 */
    if (rfid_result->source == RFID_READ_SOURCE_USER)
    {
        message->speed_zzstep = decoded.speed_step_small; /* 老刀正转小步进按型号补齐，不使用基座通用值。 */
        message->speed_fzstep = decoded.speed_step_small; /* 老刀反转使用同一型号小步进，方向是否可选仍由方向掩码限制。 */
        message->speed_oscstep = decoded.speed_step_small; /* 老刨刀往复慢调使用PXYTP小步进。 */
        message->speed_zzstep_large = decoded.speed_step_large; /* 老刀正转快调使用兼容表大步进。 */
        message->speed_fzstep_large = decoded.speed_step_large; /* 老刀反转快调使用同一型号大步进。 */
        message->speed_oscstep_large = decoded.speed_step_large; /* 老刨刀往复快调使用PXYTP大步进。 */
    }
    else if ((rfid_result->source == RFID_READ_SOURCE_EPC) &&
             ((mapped_model == COMMON_SOCKET_ONLINES) ||
              (mapped_model == PXBA_ONLINES) ||
              (mapped_model == PXBB_ONLINES)) &&
             (max_speed > min_speed)) /* 三类基座的新 EPC 刀具共用分档；老 USER 已在上方处理，手动模式不进入 RFID 装载。 */
    {
        speed_range = max_speed - min_speed; /* 合法 RFID 范围先求差值；EPC 速度单位换算后最大差值为 255000rpm，不会溢出 32 位。 */
        if (speed_range >= 70000U)
        {
            common_speed_step_large = 10000U; /* 速度范围达到 7 万及以上时，大步进固定为 10000rpm。 */
            common_speed_step_small = 2000U; /* 速度范围达到 7 万及以上时，小步进固定为 2000rpm。 */
        }
        else if (speed_range <= 10000U)
        {
            common_speed_step_large = 1000U; /* 速度范围不超过 1 万时，大步进固定为 1000rpm。 */
            common_speed_step_small = 500U; /* 速度范围不超过 1 万时，小步进固定为 500rpm。 */
        }
        else
        {
            common_speed_step_large = 5000U; /* 速度范围大于 1 万且小于 7 万时，大步进固定为 5000rpm。 */
            common_speed_step_small = 1000U; /* 速度范围大于 1 万且小于 7 万时，小步进固定为 1000rpm。 */
        }
        message->speed_zzstep = common_speed_step_small; /* 公共接头和分体式正转小步进按当前新 EPC 刀具范围生成，避免高速刀具沿用基座小步长。 */
        message->speed_fzstep = common_speed_step_small; /* 反转使用本次新 EPC 分档小步进，保证切换方向后的慢调档一致。 */
        message->speed_oscstep = common_speed_step_small; /* 往复使用同一分档小步进；方向是否可选仍由手柄和刀具许可决定。 */
        message->speed_zzstep_large = common_speed_step_large; /* 公共接头和分体式正转大步进使用当前新 EPC 范围对应的分档值。 */
        message->speed_fzstep_large = common_speed_step_large; /* 反转大步进同步本次分档值，避免快调仍沿用基座 Page6 配置。 */
        message->speed_oscstep_large = common_speed_step_large; /* 往复大步进同步本次分档值，不改变独立的往复频率参数。 */
    }
    if (message->speed_zzstep == 0U)
    {
        message->speed_zzstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，正转调速步长使用默认值1000。 */
    }
    if (message->speed_fzstep == 0U)
    {
        message->speed_fzstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，反转调速步长使用默认值1000。 */
    }
    if (message->speed_oscstep == 0U)
    {
        message->speed_oscstep = HANDLESCAN_SPEED_STEP_FALLBACK; /* RFID 标签未携带 Page6 步进时，往复调速步长使用默认值1000。 */
    }
    if (message->speed_zzstep_large == 0U)
    {
        message->speed_zzstep_large = HANDLESCAN_SPEED_STEP_LARGE_FALLBACK; /* 未读到正转大步进时使用默认值，保证屏幕大加/大减仍能调整设定速度。 */
    }
    if (message->speed_fzstep_large == 0U)
    {
        message->speed_fzstep_large = HANDLESCAN_SPEED_STEP_LARGE_FALLBACK; /* 未读到反转大步进时使用默认值，避免切换方向后大加/大减按键无效。 */
    }
    if (message->speed_oscstep_large == 0U)
    {
        message->speed_oscstep_large = HANDLESCAN_SPEED_STEP_LARGE_FALLBACK; /* 未读到往复大步进时使用默认值，不改变频率显示逻辑。 */
    }
    //message->run_direction = direction; /* 默认方向来自 RFID 标签。 */

    spec_values[0] = (uint32_t)length; /* 分体式和公共接头 EPC 规格按标签原始字节保存，屏幕规格区按 EPC 模式显示。 */
    spec_values[1] = (uint32_t)diameter; /* EPC 直径不再做历史单位放大，保证标签值和屏幕规格值一致。 */
    spec_values[2] = (uint32_t)angle; /* EPC 角度不再做历史单位放大，保持最终协议的单一解释。 */
    spec_values[3] = (uint32_t)business_tool_type; /* 保存业务刀具类型，屏幕掉线/能力判断不再直接使用 RFID 原始代号。 */
    message->rfid_cache_hit = rfid_result->cache_hit ? 1U : 0U; /* 标记本次是否为同一 EPC 重识别，后续保存 MemoryMsg 时据此保留用户调节值。 */

    return true; /* RFID 刀具头信息已经写入识别缓存。 */
}

/*
 * 函数功能：先保存已识别的手柄基座信息；还没读到 RFID 时，刀具类型和规格保持为空。
 * 输入参数：message 为 A/B 识别缓存；mapped_model 为 EEPROM 第二页映射出的基座型号；raw_type_major/raw_type_minor 为 EEPROM 第二页原始类型。
 * 返回参数：无。
 */
static void Handlescan_PrepareRfidBase(ChannelrecognizeMessage_t *message,
                                                       uint8_t mapped_model,
                                                       uint8_t raw_type_major,
                                                       uint8_t raw_type_minor)
{
    if (message == NULL)
    {
        return; /* 识别缓存为空时不写字段，避免异常路径破坏全局状态。 */
    }

    memset(message, 0, sizeof(*message)); /* 先清空刀具字段，确保上位机只显示“等待 RFID”，不会看到全 0 假刀具。 */
    message->eeprom_profile.direction_mask = HANDLE_EEPROM_PROFILE_INVALID; /* 未读到本手柄Page4前禁止运行，不能把未初始化值当旧版。 */
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
    message->speed_zzstep_large=HANDLESCAN_SPEED_STEP_LARGE_FALLBACK;
    message->speed_fzstep_large=HANDLESCAN_SPEED_STEP_LARGE_FALLBACK;
    message->speed_oscstep_large=HANDLESCAN_SPEED_STEP_LARGE_FALLBACK;
    message->tool_reduction_ratio=HANDLESCAN_TOOL_RATIO_UNIT; /* 基座尚未识别刀具时使用 100 表示 1.00 倍直联，等待 Page3/RFID 刷新。 */
    message->default_injection_flow=20;
    message->freq_default=40;
    if (mapped_model == COMMON_SOCKET_ONLINES)
    {
        message->speed_fzdefault = 0U; /* 临时补丁：公共接头无 EPC 刀具头时没有可运行电机，反转默认速度显示 0。 */
        message->speed_oscdefault = 0U; /* 临时补丁：公共接头无 EPC 刀具头时没有可运行电机，往复默认速度显示 0。 */
        message->speed_zzdefault = 0U; /* 临时补丁：公共接头无 EPC 刀具头时没有可运行电机，正转默认速度显示 0。 */
        message->speed_zzmin = 0U; /* 临时补丁：公共接头基座阶段不开放正转调速范围，等待 EPC 成功后再写固定范围。 */
        message->speed_zzmax = 0U; /* 临时补丁：公共接头基座阶段不开放正转调速范围，避免无刀具时按键调出旧速度。 */
        message->speed_fzmin = 0U; /* 临时补丁：公共接头基座阶段不开放反转调速范围，等待 EPC 成功后再写固定范围。 */
        message->speed_fzmax = 0U; /* 临时补丁：公共接头基座阶段不开放反转调速范围，避免无刀具时按键调出旧速度。 */
        message->speed_oscmin = 0U; /* 临时补丁：公共接头基座阶段不开放往复调速范围，等待 EPC 成功后再写固定范围。 */
        message->speed_oscmax = 0U; /* 临时补丁：公共接头基座阶段不开放往复调速范围，避免无刀具时按键调出旧速度。 */
    }
}

/*
 * 函数功能：连续多轮 RFID 读取超时后标记刀具离线；PXBA/PXBB 和公共接头均静默锁存最后参数，等待有效标签。
 * 输入参数：binding 为目标 A/B 通道资源；mapped_model/raw_type_major/raw_type_minor 为基座型号信息。
 * 返回参数：无。
 */
static void Handlescan_ClearOnlineRfidTool(const HandlescanChannelBinding *binding,
                                           uint8_t mapped_model,
                                           uint8_t raw_type_major,
                                           uint8_t raw_type_minor)
{
    HandlescanChannelContext *context; /* 指向需要清刀具头的通道运行状态。 */

    if (binding == NULL)
    {
        return; /* 绑定无效时不清任何通道。 */
    }

    context = binding->context; /* 后续只清绑定通道。 */
    if (context->rfid_tool_online == 0U)
    {
        context->rfid_miss_count = 0U; /* 已离线时只清缺失计数。 */
        context->rfid_monitor_pending = 0U; /* 清监测等待标志。 */
        context->rfid_monitor_ticket = 0U; /* 清掉已处理的请求编号，避免离线后再查旧读取。 */
        return; /* 已经离线时不重复清 MemoryMsg、不重复发插拔事件，也不重复累计掉线。 */
    }

    if ((context->tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3) &&
        (mapped_model != PXBA_ONLINES) &&
        (mapped_model != PXBB_ONLINES) &&
        (mapped_model != COMMON_SOCKET_ONLINES)) /* PXBA/PXBB 和公共接头掉线只更新在线状态，锁存最后有效刀具参数及用户调节值。 */
    {
        Handlescan_PrepareRfidBase(binding->message,
                                   mapped_model,
                                   raw_type_major,
                                   raw_type_minor); /* RFID 刀具头离线时保留基座信息，只清刀具字段等待下一次标签。 */
        Handlescan_ClearToolSpecValues(binding->spec_values); /* 清本通道屏幕规格。 */
        Rfid_ClearChannelResult(binding->channel); /* 清本通道 RFID 结果并取消未完成事务。 */
        Pubinterface_ClearRfidToolMemory(binding->channel); /* 同步清本通道刀具记忆。 */
        Handlescan_ReloadSpeedStepMessage(binding); /* 重新装回本通道基座 Page6 步进。 */
        //SendKeyBehMessage(PLUGunPLUG, plug_key); /* 复用插入事件链刷新 MemoryMsg、屏幕和上位机心跳。 */
    }
    context->rfid_tool_online = 0U; /* 记为已离线，后面的扫描不再重复提示这次掉线。 */
    Rfid_RecordConfirmedDropout(binding->channel); /* 连续超时次数达到要求后，掉线统计增加一次。 */
    /* 射频掉线只维护在线状态和统计，锁存的参数继续使用，不触发蜂鸣。 */
    if ((context->tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3) &&
        (mapped_model != PXBA_ONLINES) &&
        (mapped_model != PXBB_ONLINES) &&
        (mapped_model != COMMON_SOCKET_ONLINES)) /* 锁存参数的型号同时保留已处理序号，同标签返回时继续沿用该参数。 */
    {
        context->rfid_last_sequence = 0U; /* 未采用锁存的型号已清刀具缓存，后续有效标签必须重新装载参数。 */
        context->rfid_last_presence_sequence = 0U; /* 未采用锁存的型号清存在序号，保证下次有效标签完整装载。 */
    }
    context->rfid_miss_count = 0U; /* 清缺失次数。 */
    context->rfid_monitor_pending = 0U; /* 清未完成监测标志。 */
    context->rfid_monitor_ticket = 0U; /* 清除已确认掉线事务票据，下一轮从新请求开始。 */
}

/*
 * 函数功能：请求 RFID 读取并进入等待刀具头结果阶段。
 * 输入参数：binding 指定 A/B 通道及其读取状态；fast_mode 为 true 时使用首次识别的快速读取方式。
 * 返回参数：true 表示已进入 RFID 等待阶段，false 表示请求未发起。
 */
static bool Handlescan_StartRfidWait(const HandlescanChannelBinding *binding,
                                     bool fast_mode)
{
    HandlescanChannelContext *context; /* 指向绑定通道的 RFID 等待状态。 */
    RfidReadSource_t rfid_source; /* 保存当前手柄对应的 RFID 数据来源。 */

    if (binding == NULL)
    {
        return false; /* 绑定无效时不发送任何通道的 RFID 请求。 */
    }

    context = binding->context; /* 后续只修改本通道 RFID 状态。 */
    rfid_source = Handlescan_ToRfidSource(context->tool_source); /* 把刀具来源转换成 RFID 模块来源。 */
    if (rfid_source == RFID_READ_SOURCE_NONE)
    {
        return false; /* EEPROM 第三页来源不进入 RFID 等待。 */
    }

    context->rfid_last_sequence = 0U; /* 新一轮等待不沿用“已处理”编号，收到结果后重新装载刀具参数。 */
    context->rfid_wait_ticks = 0U; /* 新一轮等待从 0 计数。 */
    context->rfid_last_presence_sequence = 0U; /* 存在检测从空状态开始。 */
    context->rfid_miss_count = 0U; /* 清掉上一轮在线缺失次数。 */
    context->rfid_monitor_pending = 0U; /* 上线等待独立处理，不继承在线监测请求。 */
    context->rfid_monitor_ticket = 0U; /* 清掉旧在线票据，随后保存这次首读事务的实际编号。 */
    Rfid_ClearChannelResult(binding->channel); /* 清掉本通道旧 RFID 缓存。 */
    if (Rfid_RequestToolReadTracked(binding->channel, rfid_source, fast_mode, &context->rfid_monitor_ticket) == true) /* 用实际完成结果结束首读等待，不能只依赖固定超时。 */
    {
        context->stage = HANDLESCAN_STAGE_WAIT_RFID_TOOL; /* 请求已入队后才进入 RFID 等待阶段。 */
        return true; /* RFID 请求已经交给射频任务，调用方可以同步发布基座上线。 */
    }

    return false; /* 队列未接收请求时不进入等待阶段，避免状态机空等。 */
}

/*
 * 函数功能：首读成功后装载刀具；事务结束未识别到刀具时立即转在线检测，消除固定6秒空等。
 * 输入参数：binding 为目标 A/B 通道资源；mapped_model/raw_type_major/raw_type_minor 为基座型号信息。
 * 返回参数：true 表示本轮已处理 RFID 等待阶段，调用方应返回。
 */
static bool Handlescan_ProcessRfidWait(const HandlescanChannelBinding *binding,
                                       uint8_t mapped_model,
                                       uint8_t raw_type_major,
                                       uint8_t raw_type_minor)
{
    HandlescanChannelContext *context = binding->context; /* RFID 等待只维护绑定通道的序号、计数和状态。 */
    RfidToolResult_t result; /* 保存 RFID 任务最近一次有效结果。 */
    RfidRequestResult_t request_result; /* 首读事务的实际状态，已结束时不能继续空等旧计时。 */

    if (context->stage != HANDLESCAN_STAGE_WAIT_RFID_TOOL)
    {
        return false; /* 当前不在 RFID 等待阶段时，调用方继续处理其它阶段。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return true; /* 运行中不处理 RFID，不改变参数，等待电机停止后继续。 */
    }

    if (Handlescan_IsManualSplitMode(binding) != false)
    {
        context->rfid_wait_ticks = 0U; /* 用户切手动模式后结束 RFID 等待计时。 */
        context->rfid_miss_count = 0U; /* 手动模式不累计 RFID 缺失。 */
        context->rfid_monitor_pending = 0U; /* 手动模式不再等待 RFID 回包。 */
        context->rfid_monitor_ticket = 0U; /* 手动模式不保留自动监测票据。 */
        context->rfid_wait_started = 0U; /* 手动模式结束本次首读等待，保留原手动运行规则。 */
        context->stage = HANDLESCAN_STAGE_ONLINE; /* 基座仍在线，刀具来源切回手动 EEPROM 参数。 */
        return true; /* 本轮 RFID 等待已被手动模式接管，调用方不要继续处理普通 EEPROM 链路。 */
    }

    /* RFID 任务已保存有效结果时才检查是否为新结果；没有结果就继续等待。 */
    if (Rfid_CopyLastResult(binding->channel, &result) == true)
    {
        /* 编号没有变化说明这个结果已处理过，不重复通知上线或蜂鸣。 */
        if (result.sequence != context->rfid_last_sequence)
        {
            /* 结果来源要与基座要求一致，标签内容也必须解析成功，才能更新本通道刀具参数。 */
            if ((Handlescan_ToRfidSource(context->tool_source) == RFID_READ_SOURCE_EPC) &&
                (Handlescan_ApplyRfidToolResult(binding->channel,
                                                binding->message,
                                                binding->spec_values,
                                                mapped_model,
                                                raw_type_major,
                                                raw_type_minor,
                                                &result) == true))
            {
                context->rfid_last_sequence = result.sequence; /* 记录本次结果序号，避免重复处理。 */
                context->rfid_last_presence_sequence = result.presence_sequence; /* 在线监测从当前标签存在序号继续。 */
                context->rfid_miss_count = 0U; /* 已读到标签，连续缺失次数清零。 */
                context->rfid_monitor_pending = 0U; /* 清掉在线监测等待标志。 */
                context->rfid_monitor_ticket = 0U; /* 上线等待不继承任何在线监测票据。 */
                context->rfid_tool_online = 1U; /* 标记刀具已识别，后续定期检查是否仍在；读到新标签时仍会更新参数。 */
                context->offline_event_required = 1U; /* RFID 刀具结果将触发通道上线刷新，后续物理拔出必须与之配对发布离线事件。 */
                (void)SendKeyBehMessage(PLUGunPLUG, binding->plug_key); /* 发送手柄插入消息以更新本通道参数；即使消息发送失败，也保留以后通知拔出的要求。 */
                if (result.cache_hit == false) /* 仅首次或不同完整RFID信息提示，同一标签重新识别保持静默。 */
                {
                    Handlescan_BeepOnceIfNoAlarm(); /* 识别到不同有效刀具信息时单响，同一标签恢复不能重复提示。 */
                }
                Handlescan_ClearChannelAlarm(binding->channel, context->last_alarm); /* 成功识别后释放本通道历史报警。 */
                context->last_alarm = 0U; /* 清掉最近报警缓存。 */
                context->rfid_wait_started = 0U; /* 刀具已识别，退出首次事务等待，后续由在线监测负责。 */
                context->rfid_wait_ticks = 0U; /* 不把首读耗时带入后续周期。 */
                context->stage = HANDLESCAN_STAGE_ONLINE; /* 刀具头识别成功后进入在线保持。 */
                return true; /* 本轮 RFID 阶段处理完毕。 */
            }
        }
    }

    request_result = Rfid_QueryRequestResult(binding->channel, context->rfid_monitor_ticket); /* 首次读取与在线监测一样按本通道票据查询，排队不等于读取失败。 */
    if (request_result == RFID_REQUEST_RESULT_PENDING)
    {
        ++context->rfid_wait_ticks; /* 只为尚未结束的事务计时，正常完成后不再占用剩余6秒。 */
        if (context->rfid_wait_ticks < HANDLESCAN_RFID_WAIT_TIMEOUT_TICKS)
        {
            return true; /* 当前事务仍在真实读取或排队，保持原接收预算，不重复抢占RFID。 */
        }
        Rfid_ClearChannelResult(binding->channel); /* 异常长期未完成才取消旧票据，防止迟到结果被新一轮沿用。 */
    }
    context->rfid_monitor_pending = 0U; /* 首读已完成、取消或超时；后续在线监测从新事务开始。 */
    context->rfid_monitor_ticket = 0U; /* 已结算的票据不能再次参与刀具掉线计数。 */
    context->rfid_monitor_ticks = HANDLESCAN_RFID_MONITOR_PERIOD_TICKS; /* 下一扫描周期即可申请普通读取，不再增加一段500ms等待。 */
    context->rfid_wait_ticks = 0U; /* 清掉首读等待时间，基座身份不需重新认证。 */
    context->rfid_wait_started = 0U; /* 首次等待到此结束，拔出仍由已发布上线事件的配对记录处理。 */
    context->stage = HANDLESCAN_STAGE_ONLINE; /* 保留已校验的基座和未识别刀具状态，启动沿用当前参数，不因RFID缺失单独拦截。 */

    return true; /* RFID 等待阶段已处理，调用方本轮返回。 */
}

/*
 * 函数功能：检查 PXBA/PXBB 是否已切为手动模式；手动模式不再读取 RFID，也不采用新标签参数。
 * 输入参数：binding 为目标通道的型号、刀具来源和运行状态。
 * 返回参数：true 表示 PXBA/PXBB 手动模式，应禁止 RFID 覆盖手动参数；false 表示仍允许 RFID 监测。
 */
static bool Handlescan_IsManualSplitMode(const HandlescanChannelBinding *binding)
{
    uint8_t handle_type; /* 保存当前通道识别出的基座型号，用于区分 PXBA/PXBB 和公共接头。 */

    if ((binding == NULL) || (binding->context->tool_source == HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3))
    {
        return false; /* 普通 EEPROM 手柄本身不走 RFID，不需要手动模式屏蔽。 */
    }

    handle_type = binding->message->handle_type; /* 直接读取绑定通道基座型号，不再按 A/B 分支选全局对象。 */

    if ((handle_type != PXBA_ONLINES) && (handle_type != PXBB_ONLINES))
    {
        return false; /* 公共接头没有手动磨/刨模式，即使 auto_identify 为 0 也必须继续 EPC 在线监测。 */
    }

    return (Pubinterface_IsRfidAutoIdentifyEnabled(binding->channel) == false); /* PXBA/PXBB 切到手动模式后停止 RFID 轮询。 */
}

/*
 * 函数功能：停机时定期检查 RFID 刀具；按请求编号查结果，读到标签清掉缺失次数，超时才加一次。
 * 输入参数：binding 指定 A/B 通道，内含读取来源、计时、请求编号和连续超时次数。
 * 返回参数：无。
 */
static void Handlescan_RequestOnlineRfidMonitor(const HandlescanChannelBinding *binding)
{
    HandlescanChannelContext *context = binding->context; /* 在线轮询只修改绑定通道的计数和票据。 */
    RfidReadSource_t rfid_source = Handlescan_ToRfidSource(context->tool_source); /* 只对 RFID 来源通道做低频监测。 */
    RfidRequestResult_t request_result; /* 本轮读取的结果：还在等、成功、超时、取消或编号已失效。 */

    if (rfid_source == RFID_READ_SOURCE_NONE)
    {
        return; /* EEPROM 第三页来源不需要周期 RFID。 */
    }

    if (Handlescan_IsManualSplitMode(binding) != false)
    {
        context->rfid_monitor_ticks = 0U; /* 手动模式不累计在线 RFID 轮询周期。 */
        context->rfid_monitor_pending = 0U; /* 手动模式不再等待自动识别事务。 */
        context->rfid_monitor_ticket = 0U; /* 不再查询自动读取的编号；切手动造成的取消不能算刀具缺失。 */
        context->rfid_miss_count = 0U; /* 不按 RFID 缺失清手动刀具状态。 */
        return; /* 手动模式已经关闭 RFID 自动识别，本轮不发送读取命令。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        context->rfid_monitor_ticks = 0U; /* 电机运行中暂停周期读取，停止后重新计时。 */
        context->rfid_monitor_pending = 0U; /* 电机运行就不再等这次读取，暂停识别不等于刀具被取下。 */
        context->rfid_monitor_ticket = 0U; /* 清掉旧请求编号；RFID 任务会取消运行期间未完成的读取。 */
        return; /* 运行中不发送 RFID 命令。 */
    }

    if (context->rfid_monitor_ticks < 0xFFFFU)
    {
        ++context->rfid_monitor_ticks; /* 空闲时持续累计周期，排队期间也不提前结算缺失。 */
    }

    if (context->rfid_monitor_pending != 0U)
    {
        request_result = Rfid_QueryRequestResult(binding->channel,
                                                 context->rfid_monitor_ticket); /* 只查询本通道本票据，另一通道完成结果不能串入。 */
        if (request_result == RFID_REQUEST_RESULT_PENDING)
        {
            return; /* 本轮还在排队或读取，不能用新请求替换，也不能算作没读到刀具。 */
        }

        context->rfid_monitor_pending = 0U; /* 本轮已结束，后面到时间后可以启动下一轮读取。 */
        context->rfid_monitor_ticket = 0U; /* 清掉本轮编号，避免下次扫描把同一个超时再计一次。 */
        if (request_result == RFID_REQUEST_RESULT_SUCCESS)
        {
            Rfid_RecordMonitorCompletion(binding->channel); /* 成功事务进入在线监测完成统计。 */
            context->rfid_miss_count = 0U; /* 完整 EPC 回包证明刀具仍可读，连续超时从零重新开始。 */
        }
        else if (request_result == RFID_REQUEST_RESULT_TIMEOUT)
        {
            Rfid_RecordMonitorCompletion(binding->channel); /* 明确超时也属于一次完成的在线监测事务。 */
            if (context->rfid_miss_count < 0xFFU)
            {
                ++context->rfid_miss_count; /* 本轮读取确实超时，连续没读到刀具的次数才加 1。 */
            }
        }
        else
        {
            return; /* 请求被取消或编号已失效，无法判断刀具是否在，既不计成功也不计缺失。 */
        }

        if (context->rfid_miss_count >= HANDLESCAN_RFID_MISS_MAX)
        {
            return; /* 连续 6 轮超时，交给后面的刀具离线处理；不再启动第 7 轮。 */
        }
    }

    if (context->rfid_monitor_ticks < HANDLESCAN_RFID_MONITOR_PERIOD_TICKS)
    {
        return; /* 未到500ms启动周期时保持当前状态，不重复排队。 */
    }

    context->rfid_monitor_ticks = 0U; /* 创建新事务前重新计时，排队耗时不作为缺失结论。 */
    if (Rfid_RequestToolReadTracked(binding->channel,
                                    rfid_source,
                                    false,
                                    &context->rfid_monitor_ticket) == true)
    {
        context->rfid_monitor_pending = 1U; /* 请求已被接受，接下来用返回的编号等待它完成。 */
    }
    else
    {
        context->rfid_monitor_ticket = 0U; /* 读取条件不满足或队列已满，没有实际开始读取，不能计作刀具缺失。 */
    }
}

/*
 * 函数功能：停机时装载有效RFID结果；公共接头和PX仅新刀具信息单响，掉线及同标签恢复静默。
 * 输入参数：binding 为目标 A/B 通道资源；mapped_model/raw_type_major/raw_type_minor 为基座型号信息。
 * 返回参数：无。
 */
static void Handlescan_ProcessOnlineRfidResult(const HandlescanChannelBinding *binding,
                                               uint8_t mapped_model,
                                               uint8_t raw_type_major,
                                               uint8_t raw_type_minor)
{
    HandlescanChannelContext *context = binding->context; /* 在线结果只更新绑定通道的 RFID 状态。 */
    RfidToolResult_t result; /* 保存 RFID 任务最近一次有效结果。 */

    if (Handlescan_ToRfidSource(context->tool_source) == RFID_READ_SOURCE_NONE)
    {
        return; /* 非 RFID 来源时不处理。 */
    }

    if (WorkMessage.runflag_work == true)
    {
        return; /* 手柄运行中不处理新收到的RFID结果，避免中途改变运行参数。 */
    }

    if (Handlescan_IsManualSplitMode(binding) != false)
    {
        context->rfid_miss_count = 0U; /* 手动模式不累计 RFID 缺失。 */
        context->rfid_monitor_pending = 0U; /* 手动模式不再等待 RFID 回包。 */
        context->rfid_monitor_ticket = 0U; /* 手动模式不保留自动监测票据。 */
        return; /* PXBA/PXBB 已切手动模式，禁止 EPC 晚到回包覆盖手动磨/刨选择。 */
    }

    if (context->rfid_miss_count >= HANDLESCAN_RFID_MISS_MAX)
    {
        Handlescan_ClearOnlineRfidTool(binding,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor); /* 连续已完成事务超时达到阈值后处理真实掉线边沿。 */
        return; /* 本轮已处理刀具头缺失状态。 */
    }

    if (Rfid_CopyLastResult(binding->channel, &result) == false)
    {
        return; /* 当前通道还没有有效 RFID 结果。 */
    }

    if ((Handlescan_ToRfidSource(context->tool_source) != RFID_READ_SOURCE_EPC) ||
        ((result.source != RFID_READ_SOURCE_EPC) && (result.source != RFID_READ_SOURCE_USER)))
    {
        return; /* 来源不匹配时不使用，避免非本通道 EPC 结果串用。 */
    }

    if (result.presence_sequence != context->rfid_last_presence_sequence)
    {
        context->rfid_last_presence_sequence = result.presence_sequence; /* 读到完整标签就刷新存在序号。 */
        context->rfid_miss_count = 0U; /* 有效 EPC 已证明刀具存在，连续超时清零。 */
        if ((result.sequence == context->rfid_last_sequence) &&
            (context->rfid_tool_online == 0U) &&
            ((mapped_model == PXBA_ONLINES) || (mapped_model == PXBB_ONLINES) ||
             (mapped_model == COMMON_SOCKET_ONLINES))) /* 三种型号均保留掉线前参数，同序号有效标签只需恢复在线状态。 */
        {
            context->rfid_tool_online = 1U; /* 已处理过的同一标签恢复在线，不重新装载默认参数或覆盖用户调速值。 */
        }
    }

    if (result.sequence == context->rfid_last_sequence)
    {
        return; /* 结果已经处理过，不重复刷新，也不对在线同标签重复蜂鸣。 */
    }

    if (Handlescan_ApplyRfidToolResult(binding->channel,
                                       binding->message,
                                       binding->spec_values,
                                       mapped_model,
                                       raw_type_major,
                                       raw_type_minor,
                                       &result) == false)
    {
        return; /* 数据不符合当前来源时不刷新。 */
    }

    context->rfid_last_sequence = result.sequence; /* 记录已处理序号。 */
    context->offline_event_required = 1U; /* RFID 在线监测已经请求刷新业务在线态，物理拔出时必须完成对应离线通知。 */
    (void)SendKeyBehMessage(PLUGunPLUG, binding->plug_key); /* 复用插入事件链刷新本通道状态；实际拔出由扫描锁存保证最终送达。 */
    context->rfid_tool_online = 1U; /* 完整新结果把刀具切回在线边沿。 */
    if (result.cache_hit == false) /* 原始来源、长度或完整数据改变才提示，不能按射频离线到在线边沿蜂鸣。 */
    {
        Handlescan_BeepOnceIfNoAlarm(); /* 新刀具信息装载后单响；相同RFID恢复只沿用记忆，不重复蜂鸣。 */
    }
}

/*
 * 函数功能：调试开关打开时，输出已识别手柄的型号字节和名称。
 * 输入参数：channel 为通道号；first_byte/second_byte 为型号字节；handle_name 为型号名称。
 * 返回参数：无。
 */
static void Handlescan_DebugTraceHandleName(uint8_t channel, uint8_t first_byte, uint8_t second_byte, const char *handle_name)
{
#if (HANDLESCAN_TRACE_ENABLE == 1U)
    char tx_buf[80];
    int text_len;

    /* 未知型号没有可打印名称，跳过调试报文以免 snprintf 解引用空字符串。 */
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
    /* 名称报文格式化失败时不发送，避免调试串口输出未初始化内容。 */
    if (text_len <= 0)
    {
        return;
    }

    /* 名称文本超过缓冲区时按已写入长度截断，防止串口发送越界读取。 */
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
 * 函数功能：两个调试开关都打开时，输出刀具名称、直径、长度和角度。
 * 输入参数：channel 为通道；first_byte/second_byte 为型号字节；tool_name 为名称；后三项为 0.1 mm、0.1 mm、0.1 度的规格值。
 * 返回参数：无。
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
    /* 刀具规格报文格式化失败时不发送，保持调试数据只包含完整有效帧。 */
    if (text_len <= 0)
    {
        return;
    }

    /* 规格文本超过缓冲区时限制发送长度，避免读取 tx_buf 末尾之外的数据。 */
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
 * 函数功能：把 EEPROM 中高字节在前的两个字节合成一个 16 位数值。
 * 输入参数：buffer 为数据缓存；offset 为第一个字节的位置，调用方须保证还有两个字节可读。
 * 返回参数：合成后的无符号数值。
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
 * 函数功能：按小端格式读取EEPROM业务页中的24位实际转速。
 * 输入参数：buffer指向已通过页校验的30字节业务区；offset表示字段起始偏移。
 * 返回参数：解析后的24位无符号转速，使用uint32_t承载以保留完整范围。
 */
static uint32_t Handlescan_ReadUint24LE(const uint8_t *buffer, uint32_t offset)
{
    uint32_t value = (uint32_t)buffer[offset]; /* 最低字节直接放入bit0~7，保持与Page4既有速度字段的小端语义一致。 */
    value |= ((uint32_t)buffer[offset + 1U] << 8); /* 中间字节放入bit8~15，不能按Page3传统大端刀具字段解析。 */
    value |= ((uint32_t)buffer[offset + 2U] << 16); /* 最高字节放入bit16~23，使配置可覆盖超过65535rpm的未来范围。 */
    return value; /* 返回完整转速供手动刨刀默认值及上下限共同使用。 */
}

/*
 * 函数功能：按 Page4 扩展格式读取 24 位最大速度。
 * 输入参数：buffer 指向已通过页校验的 Page4 缓存。
 * 返回参数：最大速度，单位为实际 rpm。
 */
static uint32_t Handlescan_ReadPage4MaxSpeed(const uint8_t *buffer)
{
    uint32_t max_speed = Handlescan_ReadUint16LE(buffer, HANDLESCAN_INITIAL_MAX_SPEED_OFFSET); /* 读取 Page4[6-7] 的低16位最大速度，兼容旧 EEPROM。 */
    max_speed |= ((uint32_t)buffer[HANDLESCAN_INITIAL_MAX_SPEED_HIGH_OFFSET] << 16); /* Page4[19] 是新增高8位，旧数据为0时仍保持原16位解析。 */
    return max_speed; /* 返回组合后的24位速度上限，供通道记忆、屏幕调速和默认速度钳位共用。 */
}

/*
 * 函数功能：按 Page4 扩展格式读取 24 位默认速度。
 * 输入参数：buffer 指向已通过页校验的 Page4 缓存。
 * 返回参数：默认速度，单位为实际 rpm。
 */
static uint32_t Handlescan_ReadPage4DefaultSpeed(const uint8_t *buffer)
{
    uint32_t default_speed = Handlescan_ReadUint16LE(buffer, HANDLESCAN_INITIAL_DEFAULT_SPEED_OFFSET); /* 读取 Page4[2-3] 的低16位默认速度，兼容旧 EEPROM。 */
    default_speed |= ((uint32_t)buffer[HANDLESCAN_INITIAL_DEFAULT_SPEED_HIGH_OFFSET] << 16); /* Page4[20] 是默认速度高8位，旧数据为0时仍按16位默认速度解析。 */
    return default_speed; /* 返回组合后的24位默认速度，供上线初始速度和脚踏默认最大值共用。 */
}

/*
 * 函数功能：把 EEPROM Page4 默认注水流量限制到 1~300，异常时使用程序默认值。
 * 输入参数：flow_value Page4 中直接保存的默认注水泵流量。
 * 返回参数：现有 pumpMessage.speed_work 使用的整数流量值。
 */
static uint16_t Handlescan_BuildDefaultInjectionFlow(uint16_t flow_value)
{
    if ((flow_value < HANDLESCAN_INITIAL_FLOW_MIN) ||
        (flow_value > HANDLESCAN_INITIAL_FLOW_MAX))
    {
        return HANDLESCAN_INITIAL_FLOW_DEFAULT; /* EEPROM 写 0 或超过 300 时统一回退 30，避免注水泵 0 速或异常过量输出。 */
    }

    return flow_value; /* EEPROM 写入 1~300 时直接作为注水泵业务流量使用，方便生产端按实际 ml/min 写值。 */
}

/*
 * 函数功能：把 Page4 默认速度限制到同页给出的最小速度和最大速度之间。
 * 输入参数：default_speed 默认速度，min_speed 最小速度，max_speed 最大速度，三者均为实际 rpm 值。
 * 返回参数：限制后的默认速度。
 */
static uint32_t Handlescan_ClampDefaultSpeed(uint32_t default_speed, uint32_t min_speed, uint32_t max_speed)
{
    if (max_speed < min_speed)
    {
        max_speed = min_speed; /* 最大速度小于最小速度时，将最大速度改为最小速度，避免上下限颠倒。 */
    }

    if (default_speed < min_speed)
    {
        default_speed = min_speed; /* 默认速度低于最小值时抬到最小值，保证上线初始速度在允许范围内。 */
    }

    if (default_speed > max_speed)
    {
        default_speed = max_speed; /* 默认速度高于最大值时降到最大值，保证上线初始速度不超过手柄配置上限。 */
    }

    return default_speed; /* 返回可直接写入 WorkMessage.speed_set_work 的实际 rpm 速度值。 */
}

/*
 * 函数功能：解析EEPROM默认方向；新版按本体许可，旧版保留原型号往复限制。
 * 输入参数：message为当前通道识别缓存；raw_direction为EEPROM Page4方向字节。
 * 返回参数：项目内部方向值；MXYTP/MXYTM不再进入本函数，统一由公共接头RFID EPC路径解析。
 */
static uint8_t Handlescan_ParseInitialDirection(const ChannelrecognizeMessage_t *message, uint8_t raw_direction)
{
    if ((message != NULL) && (message->eeprom_profile.direction_mask != 0U))
    {
        return ((raw_direction >= 1U) && (raw_direction <= 3U)) ? (uint8_t)(raw_direction - 1U) : ZZDIR; /* 新版默认方向已由扩展检查，非法扩展保持锁定，不再按型号改方向。 */
    }

    if (raw_direction == HANDLESCAN_INITIAL_DIRECTION_REVERSE)
    {
        return FZDIR; /* Page4[8]=0x02表示默认反转；固定方向EEPROM手柄上线后会锁住该方向。 */
    }

    if (raw_direction == HANDLESCAN_INITIAL_DIRECTION_OSC)
    {
        if (Handlescan_SupportsOscDirection(message))
        {
            return OSCDIR; /* Page4[8]=0x03且EEPROM手柄/刀具支持电气往复时，默认进入往复方向。 */
        }

        return ZZDIR; /* EEPROM写了往复但型号不支持时保护回正转，MXYTP由RFID解析路径另行固定显示往复。 */
    }

    return ZZDIR; /* Page4[8]=0x01或异常值按正转处理，保持普通EEPROM手柄安全默认。 */
}
/*
 * 函数功能：解析手柄Page4的方向和PX手动电流扩展；异常格式保持禁止运行。
 * 输入参数：message为已识别Page2型号的通道缓存；initial_info_buf为通过页校验的Page4。
 * 返回参数：无；旧版标记0保留旧规则，新版有效时发布方向许可及三档电流。
 */
static void Handlescan_UpdateEepromControlProfile(ChannelrecognizeMessage_t *message, const uint8_t *initial_info_buf)
{
    uint8_t mode = initial_info_buf[HANDLESCAN_CONTROL_PROFILE_OFFSET]; /* 直接读取01/10/11三档许可，不再混合版本标记。 */
    uint8_t mask; /* 把三档协议值转换为运行态方向位，后续入口统一检查正转、反转和往复。 */
    uint8_t default_direction = initial_info_buf[HANDLESCAN_INITIAL_DIRECTION_OFFSET]; /* Page4默认方向按1/2/3编码。 */

    message->eeprom_profile.direction_mask = HANDLE_EEPROM_PROFILE_INVALID; /* 先锁住，所有格式和电流检查完成后才开放方向。 */
    message->eeprom_profile.manual_grind_current_low = 0U; /* 清掉旧低速电流，避免换柄后沿用上一只配置。 */
    message->eeprom_profile.manual_grind_current_high = 0U; /* 清掉旧高速电流，读取失败不能使用残留值。 */
    message->eeprom_profile.manual_planer_current = 0U; /* 清掉旧刨刀电流，异常配置不回退到驱动默认保护。 */
    if (mode == 0U)
    {
        message->eeprom_profile.direction_mask = 0U; /* 明确的旧版Page4才保留原型号方向和原电流规则。 */
        return; /* 旧版剩余保留字节不解释为新电流。 */
    }
    if ((default_direction < 1U) || (default_direction > 3U))
    {
        return; /* 默认方向必须是1正转、2反转或3往复，非法值保持禁止运行。 */
    }
    if (mode == 0x01U)
    {
        mask = (uint8_t)(1U << (default_direction - 1U)); /* 单方向只开放本页默认方向，默认正转、反转或往复均可。 */
    }
    else if (mode == 0x10U)
    {
        if (default_direction == 3U)
        {
            return; /* 双方向只允许正反转，默认往复与本档冲突时禁止运行。 */
        }
        mask = 0x03U; /* 双方向开放正转和反转，不开放往复。 */
    }
    else if (mode == 0x11U)
    {
        mask = HANDLE_EEPROM_DIRECTION_BITS; /* 三方向开放正转、反转和往复，RFID仍可进一步限制。 */
    }
    else
    {
        return; /* 未定义的档位保持禁止运行，不能当作旧版或全部开放。 */
    }

    message->eeprom_profile.manual_grind_current_low = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_MANUAL_GRIND_LOW_OFFSET); /* Page4[24..25]保存低速磨头0.01A阈值。 */
    message->eeprom_profile.manual_grind_current_high = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_MANUAL_GRIND_HIGH_OFFSET); /* Page4[26..27]保存高速磨头0.01A阈值，30000归此档。 */
    message->eeprom_profile.manual_planer_current = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_MANUAL_PLANER_CURRENT_OFFSET); /* Page4[28..29]保存手动刨刀0.01A阈值。 */
    if (((message->handle_type == PXBA_ONLINES) || (message->handle_type == PXBB_ONLINES)) &&
        ((message->eeprom_profile.manual_grind_current_low == 0U) ||
         (message->eeprom_profile.manual_grind_current_high == 0U) ||
         (message->eeprom_profile.manual_planer_current == 0U)))
    {
        return; /* 新版PX必须完整配置三档电流，零值不得被驱动解释为内部默认保护。 */
    }
    message->eeprom_profile.direction_mask = mask; /* 所有检查通过后最后发布本手柄的有效方向能力。 */
}

/*
 * 函数功能：把 EEPROM Page4 初始值信息解析到通道识别结构，供插入事件装载默认速度、频率、注水流量和阈值。
 * 输入参数：message 目标通道识别结构；initial_info_buf 已通过页尾校验的 Page4 缓存。
 * 返回参数：无。
 */
static void Handlescan_UpdateInitialInfoMessage(ChannelrecognizeMessage_t *message, const uint8_t *initial_info_buf)
{
    uint32_t default_speed;                                  /* 保存 Page4 默认速度，单位为实际 rpm，驱动下发时只做倍率换算。 */
    uint32_t min_speed;                                      /* 保存 Page4 最小速度，单位为实际 rpm。 */
    uint32_t max_speed;                                      /* 保存 Page4 最大速度，单位为实际 rpm。 */
    uint16_t current_threshold;                              /* 保存 Page4 电流保护阈值，单位 0.01A，后续随当前通道下发驱动。 */

    if ((message == NULL) || (initial_info_buf == NULL))
    {
        return;                                              /* 防御空指针，避免异常插拔路径破坏通道识别结构。 */
    }

    default_speed = Handlescan_ReadPage4DefaultSpeed(initial_info_buf); /* 读取 Page4 24位默认速度，支持 120000 这类超过16位的默认值。 */
    min_speed = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_MIN_SPEED_OFFSET); /* 读取 Page4 最小速度，小端实际 rpm。 */
    max_speed = Handlescan_ReadPage4MaxSpeed(initial_info_buf); /* 读取 Page4 24位最大速度，支持 70000 这类超过16位的上限。 */
    if (max_speed < min_speed)
    {
        max_speed = min_speed;                               /* 上下限异常时同步修正保存值，避免后续 SpeedActive 读到反向边界。 */
    }
    default_speed = Handlescan_ClampDefaultSpeed(default_speed, min_speed, max_speed); /* 默认速度按同页上下限钳位，保证上线速度合法。 */

    message->default_injection_flow = Handlescan_BuildDefaultInjectionFlow(Handlescan_ReadUint16BE(initial_info_buf, HANDLESCAN_INITIAL_DEFAULT_FLOW_OFFSET)); /* Page4 默认注水流量按大端直接保存，读出后只做 1~300 范围保护。 */
    message->speed_min = min_speed;                         /* 保存 Page4 最小速度，屏幕和外控调速边界必须跟随 EEPROM 配置。 */
    message->speed_max = max_speed;                         /* 保存通用最大速度，供后续 UI/外控边界逻辑复用。 */
    message->speed_zzmin = min_speed;                       /* Page4 当前只有一组速度上下限，正转方向使用同一最小速度。 */
    message->speed_zzmax = max_speed;                       /* Page4 当前只有一组速度上下限，正转方向使用同一最大速度。 */
    message->speed_fzmin = min_speed;                       /* Page4 当前只有一组速度上下限，反转方向使用同一最小速度。 */
    message->speed_fzmax = max_speed;                       /* Page4 当前只有一组速度上下限，反转方向使用同一最大速度。 */
    message->speed_oscmin = min_speed;                      /* Page4 当前只有一组速度上下限，往复方向使用同一最小速度。 */
    message->speed_oscmax = max_speed;                      /* Page4 当前只有一组速度上下限，往复方向使用同一最大速度。 */
    message->speed_zzdefault = default_speed;                /* Page4 默认速度作为正转上线初始速度，避免 EEPROM 写 4000 却被固定显示 60000。 */
    message->speed_fzdefault = default_speed;                /* 反转上线初始速度同样来自 Page4，保证 A/B 通道切换后仍保持手柄自身配置。 */
    message->speed_oscdefault = default_speed;               /* 往复上线初始速度同样来自 Page4，后续往复调速只在该基准上变化。 */

    

    Handlescan_UpdateEepromControlProfile(message, initial_info_buf); /* 必须先解析版本、许可和PX电流，随后默认方向才可按新版装载。 */
    message->run_direction = Handlescan_ParseInitialDirection(message, initial_info_buf[HANDLESCAN_INITIAL_DIRECTION_OFFSET]); /* 新版按EEPROM默认方向；旧版继续执行型号能力保护。 */
    message->freq_default = initial_info_buf[HANDLESCAN_INITIAL_FREQ_OFFSET]; /* 原样保存 Page4 默认频率指令值，往复启动时使用，不在这里换算 Hz。 */
    message->freq_min = FreqMin;                             /* Page4 未提供频率下限，沿用现有 UI 频率下限宏保持调节边界。 */
    message->freq_max = FreqMax;                             /* Page4 未提供频率上限，沿用现有 UI 频率上限宏保持调节边界。 */
    message->speed_alarm_for = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_FOR_ALARM_OFFSET); /* 保存正转速度报警阈值，小端2字节，单位与WorkMessage.speed_work一致。 */
    message->speed_alarm_rev = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_REV_ALARM_OFFSET); /* 保存反转速度报警阈值，小端2字节，单位与WorkMessage.speed_work一致。 */
    message->freq_alarm_osc = initial_info_buf[HANDLESCAN_INITIAL_OSC_ALARM_OFFSET]; /* 只读 Page4[14]，保存与往复频率指令比较的蜂鸣阈值。 */
    current_threshold = Handlescan_ReadUint16LE(initial_info_buf, HANDLESCAN_INITIAL_CURRENT_THRESHOLD_OFFSET); /* 读取 Page4[21..22] 的电流保护阈值，0 表示不覆盖驱动默认值。 */
    message->overloadThresholdFor = current_threshold;       /* 正转保护电流来自同一 Page4 字段，选中通道后写入 WorkMessage.current_work。 */
    message->overloadThresholdRev = current_threshold;       /* 反转保护电流沿用同一阈值，保持 EEPROM 配置字段简单一致。 */
    message->overloadThresholdOSC = current_threshold;       /* 往复保护电流沿用同一阈值，驱动侧按收到的保护电流执行停机保护。 */
}

/*
 * 函数功能：为PXBA/PXBB手动刨刀装载Page3的24位速度扩展；格式缺失或数值异常时清零速度并禁止启动。
 * 输入参数：message为目标通道识别缓存；tool_info_buf为Page3业务区；pages_valid表示Page3/Page4已完整读取。
 * 返回参数：true表示专用速度配置有效；false表示配置不可用且速度字段已清零。
 */
static bool Handlescan_ApplyPxManualPlanerSpeedProfile(ChannelrecognizeMessage_t *message,
                                                       const uint8_t *tool_info_buf,
                                                       bool pages_valid)
{
    uint32_t default_speed = 0U; /* 默认保持不可运行，只有完整合法的EEPROM扩展才能赋予实际速度。 */
    uint32_t min_speed = 0U; /* 不在主控固化产品下限，后续范围完全由EEPROM配置。 */
    uint32_t max_speed = 0U; /* 不在主控固化产品上限，便于以后只改EEPROM扩展速度范围。 */
    bool profile_valid = false; /* 标记及三项速度全部通过校验后才允许手动刨刀运行。 */

    if (message == NULL)
    {
        return false; /* 目标缓存无效时不能写任意通道，也不能宣称配置有效。 */
    }

    if ((pages_valid != false) &&
        (tool_info_buf != NULL) &&
        (tool_info_buf[HANDLESCAN_PX_PLANER_PROFILE_MARK_OFFSET] == HANDLESCAN_PX_PLANER_PROFILE_MARK_0) &&
        (tool_info_buf[HANDLESCAN_PX_PLANER_PROFILE_MARK_OFFSET + 1U] == HANDLESCAN_PX_PLANER_PROFILE_MARK_1) &&
        (tool_info_buf[HANDLESCAN_PX_PLANER_PROFILE_MARK_OFFSET + 2U] == HANDLESCAN_PX_PLANER_PROFILE_MARK_2) &&
        (tool_info_buf[HANDLESCAN_PX_PLANER_PROFILE_MARK_OFFSET + 3U] == HANDLESCAN_PX_PLANER_PROFILE_MARK_3))
    {
        uint32_t configured_default = Handlescan_ReadUint24LE(tool_info_buf, HANDLESCAN_PX_PLANER_PROFILE_DEFAULT_OFFSET); /* 读取Page3[21..23]手动刨刀默认转速。 */
        uint32_t configured_min = Handlescan_ReadUint24LE(tool_info_buf, HANDLESCAN_PX_PLANER_PROFILE_MIN_OFFSET); /* 读取Page3[24..26]手动刨刀最小转速。 */
        uint32_t configured_max = Handlescan_ReadUint24LE(tool_info_buf, HANDLESCAN_PX_PLANER_PROFILE_MAX_OFFSET); /* 读取Page3[27..29]手动刨刀最大转速。 */

        if ((configured_min != 0U) &&
            (configured_min <= configured_default) &&
            (configured_default <= configured_max))
        {
            default_speed = configured_default; /* 三项关系完整合法时才采用EEPROM默认值。 */
            min_speed = configured_min; /* 手动刨刀下限完全来自EEPROM，不在主控写死产品范围。 */
            max_speed = configured_max; /* 手动刨刀上限完全来自EEPROM，后续扩展只需重写配置。 */
            profile_valid = true; /* 标记、非零值和顺序均合法，允许本次手动刨刀装载。 */
        }
    }

    message->speed_min = min_speed; /* 通用边界同步为手动刨刀范围，屏幕和外控调速使用同一限制。 */
    message->speed_max = max_speed; /* 通用最大速度来自手动刨刀EEPROM扩展。 */
    message->speed_zzmin = min_speed; /* 手动刨刀切到正转时也必须遵守EEPROM范围。 */
    message->speed_zzmax = max_speed; /* 正转上限同步手动刨刀上限。 */
    message->speed_fzmin = min_speed; /* 手动刨刀切到反转时也必须遵守刨刀范围。 */
    message->speed_fzmax = max_speed; /* 反转上限同步手动刨刀上限。 */
    message->speed_oscmin = min_speed; /* 往复调速下限使用专用配置。 */
    message->speed_oscmax = max_speed; /* 往复调速上限使用专用配置。 */
    message->speed_zzdefault = default_speed; /* 三个方向共用同一手动刨刀默认速度，避免切方向恢复通用30000rpm。 */
    message->speed_fzdefault = default_speed; /* 反转默认值保持在专用刨刀范围内。 */
    message->speed_oscdefault = default_speed; /* 手动刨刀默认往复速度只接受EEPROM配置；无效时保持0并形成停机帧。 */

    return profile_valid; /* 调用方可据此判断本次Page3扩展是否完整可用。 */
}

/*
 * 函数功能：读取 Page6 中按一次速度键增加或减少多少转速。
 * 输入参数：speed_step_buf 为 Page6 数据；speed_offset 为字段位置；fallback_step 为读不到或值为 0 时使用的默认增减量。
 * 返回参数：每次调速的增减量，单位 rpm。
 */
static uint16_t Handlescan_ReadPage6SpeedStep(const uint8_t *speed_step_buf, uint8_t speed_offset, uint16_t fallback_step)
{
    uint16_t speed_step;                                     /* 保存从 EEPROM Page6 读取出的调速步进。 */

    if (speed_step_buf == NULL)
    {
        return fallback_step;                                /* 没有Page6数据时返回调用方给出的默认步长，不因此中止手柄上线。 */
    }

    speed_step = Handlescan_ReadUint16BE(speed_step_buf, speed_offset); /* Page6 速度字段按上位机布局使用大端 16 位。 */
    if (speed_step == 0U)
    {
        return fallback_step;                                /* 步长为0时改用默认值，避免按速度加减键却不改变设定速度。 */
    }

    return speed_step;                                       /* 返回当前按钮档位可用的调速步进。 */
}

/*
 * 函数功能：把 Page6 速度调节区同步到识别缓存的小步进和大步进字段。
 * 输入参数：message 为目标通道识别缓存；speed_step_buf 为已读取并校验过页尾的 Page6 缓存，读取失败时可传 NULL。
 * 返回参数：无。
 */
static void Handlescan_UpdateSpeedStepMessage(ChannelrecognizeMessage_t *message, const uint8_t *speed_step_buf)
{
    uint16_t small_step;                                     /* 保存 Page6[0..1] 小步进，供普通加减和小步进键使用。 */
    uint16_t large_step;                                     /* 保存 Page6[2..3] 大步进，供屏幕大加/大减键使用。 */

    if (message == NULL)
    {
        return;                                              /* 识别缓存为空时不写全局状态，避免异常路径破坏另一通道参数。 */
    }

    small_step = Handlescan_ReadPage6SpeedStep(speed_step_buf,
                                               HANDLESCAN_SPEED_STEP_SMALL_OFFSET,
                                               HANDLESCAN_SPEED_STEP_SMALL_FALLBACK); /* 小步进直接来自 Page6[0..1]，不再按手柄型号硬编码。 */
    large_step = Handlescan_ReadPage6SpeedStep(speed_step_buf,
                                               HANDLESCAN_SPEED_STEP_LARGE_OFFSET,
                                               HANDLESCAN_SPEED_STEP_LARGE_FALLBACK); /* 大步进直接来自 Page6[2..3]，屏幕快键不再用小步进乘 2。 */

    message->speed_zzstep = small_step;                      /* 正转小步进使用 Page6 小步进，保证三方向按钮体感一致。 */
    message->speed_fzstep = small_step;                      /* 反转小步进同源 Page6[0..1]，避免继续读取旧三档位偏移。 */
    message->speed_oscstep = small_step;                     /* 往复小步进同源 Page6[0..1]，只有选中往复时才会使用。 */
    message->speed_zzstep_large = large_step;                /* 正转大步进使用 Page6 大步进，供 SCREENKey_SPEED_Add/Sub_Large 使用。 */
    message->speed_fzstep_large = large_step;                /* 反转大步进同源 Page6[2..3]，保持 A/B 通道独立。 */
    message->speed_oscstep_large = large_step;               /* 往复大步进同源 Page6[2..3]，不改变频率显示逻辑。 */
}

/*
 * 函数功能：读取指定通道 EEPROM Page6 多档位速度页，并写入通道识别缓存的三方向调速步进。
 * 输入参数：binding 指定 A/B 通道、该通道的 EEPROM 和识别缓存。
 * 返回参数：无。
 */
static void Handlescan_LoadPage6SpeedStep(const HandlescanChannelBinding *binding)
{
    HandlescanChannelContext *context;                       /* 指向绑定通道的 Page6 缓存。 */
    ChannelrecognizeMessage_t *message;                      /* 指向绑定通道的识别结果。 */
    uint8_t read_status = 0U;                                 /* 保存 Page6 读取状态，1 表示读页和页尾校验通过。 */

    if (binding == NULL)
    {
        return;                                               /* 通道绑定无效时不访问 EEPROM，避免误读另一侧总线。 */
    }

    context = binding->context;                              /* Page6 缓存只取当前绑定通道。 */
    message = binding->message;                              /* 步进结果只写当前绑定通道。 */
    AT24CS32_ClearLastDebugInfo();                            /* 读取 Page6 前清掉上一次 I2C 调试信息，便于现场需要时定位失败页。 */
    read_status = Handlescan_ReadEepromPage(binding, HANDLESCAN_SPEED_STEP_PAGE_INDEX, context->speed_step_buf); /* 固定绑定负责选择 I2C2 或 I2C3。 */
    if (read_status == 0U)
    {
        memset(context->speed_step_buf, 0, sizeof(context->speed_step_buf)); /* 读取失败时清空本通道页数据，随后按默认的小步长1000、大步长2000设置调速参数。 */
    }
    Handlescan_UpdateSpeedStepMessage(message, (read_status != 0U) ? context->speed_step_buf : NULL); /* 把本通道 Page6 转成三方向步进。 */
}

/*
 * 函数功能：为公共接头读取手柄本体方向许可，不覆盖尚待RFID提供的刀具速度和电流。
 * 输入参数：binding为已确认身份的通道绑定。
 * 返回参数：无；Page4读取失败时禁止本通道全部方向。
 */
static void Handlescan_LoadRfidBaseControlProfile(const HandlescanChannelBinding *binding)
{
    binding->message->eeprom_profile.direction_mask = HANDLE_EEPROM_PROFILE_INVALID; /* 读取失败不能被误当作旧版无限制。 */
    if (Handlescan_ReadEepromPage(binding, HANDLESCAN_INITIAL_INFO_PAGE_INDEX, binding->context->initial_info_buf) != 0U)
    {
        Handlescan_UpdateEepromControlProfile(binding->message, binding->context->initial_info_buf); /* 只装载手柄自身许可，RFID仍负责刀具默认参数。 */
    }
}

/*
 * 函数功能：按通道重新读取分体式手柄自身 EEPROM 的运行参数。
 * 输入参数：binding 指定 A/B 通道、该通道的 EEPROM 和识别缓存。
 * 返回参数：true 表示 Page3 和 Page4 都读取成功；false 表示至少一页失败，调用方继续使用读取失败时设置的默认参数。
 */
static bool Handlescan_LoadSplitBaseEepromRuntime(const HandlescanChannelBinding *binding)
{
    HandlescanChannelContext *context;                       /* 指向绑定通道的 Page3/Page4 缓存。 */
    ChannelrecognizeMessage_t *message;                      /* 指向绑定通道的识别结果。 */
    uint8_t tool_read_status = 0U;                            /* Page3 读取结果，成功后用于恢复手柄自身减速比/增速比。 */
    uint8_t initial_read_status = 0U;                         /* Page4 读取结果，成功后用于恢复手柄自身速度上下限、默认速度、频率和泵流量。 */

    if (binding == NULL)
    {
        return false;                                         /* 绑定无效时不读取任意 EEPROM，避免串通道。 */
    }

    context = binding->context;                              /* 后续读写只使用本通道缓存。 */
    message = binding->message;                              /* 后续参数只写本通道识别结果。 */
    AT24CS32_ClearLastDebugInfo();                            /* 读取 Page3 前清调试缓存，现场排查时可直接看到本次失败页。 */
    tool_read_status = Handlescan_ReadEepromBytes(binding, HANDLESCAN_TOOL_INFO_ADDR, context->tool_info_buf, HANDLESCAN_TOOL_INFO_SIZE); /* 从绑定通道读取Page3完整业务区，倍率和PX01速度扩展必须一次读全。 */
    AT24CS32_ClearLastDebugInfo();                            /* Page3 读取结束后切换到 Page4 调试上下文。 */
    message->eeprom_profile.direction_mask = HANDLE_EEPROM_PROFILE_INVALID; /* 本次Page4读取失败必须保持禁止运行，不能沿用旧许可或默认基座参数。 */
    initial_read_status = Handlescan_ReadEepromPage(binding, HANDLESCAN_INITIAL_INFO_PAGE_INDEX, context->initial_info_buf); /* 从绑定通道读取 Page4 运行参数。 */
    Handlescan_UpdateToolRatioMessage(message, (tool_read_status != 0U) ? context->tool_info_buf : NULL); /* Page3 失败时按 1 倍保护。 */
    if (initial_read_status != 0U)
    {
        Handlescan_UpdateInitialInfoMessage(message, context->initial_info_buf); /* Page4 成功时恢复本通道默认运行参数。 */
    }

    return (bool)((tool_read_status != 0U) && (initial_read_status != 0U)); /* 返回完整恢复结果，调用方当前只作诊断。 */
}

/*
 * 函数功能：分体式手柄从自动识别切回手动模式时，重新装载手柄 EEPROM 的倍率、速度边界和步进。
 * 输入参数：channel 为 A/B 通道；manual_tool_type 为手动选择的业务刀具类型，PLANER 或 GRINDH。
 * 返回参数：true 表示 Page3/Page4 均恢复成功；false 表示至少一项改用本地默认参数，或通道不符合处理条件。
 */
bool Handlescan_RestoreSplitHandleEepromRuntime(uint8_t channel, uint8_t manual_tool_type)
{
    const HandlescanChannelBinding *binding;                  /* 固定当前通道的 EEPROM、缓存和硬件归属。 */
    ChannelrecognizeMessage_t *message = NULL;                /* 指向当前通道识别缓存，后续把 EEPROM 运行参数写回该缓存。 */
    uint8_t handle_type = 0U;                                  /* 保存当前分体式基座型号，重建基座缓存后继续保留。 */
    uint8_t raw_type_major = 0U;                               /* 保存 EEPROM Page2 原始主类型，供上位机心跳继续显示真实基座。 */
    uint8_t raw_type_minor = 0U;                               /* 保存 EEPROM Page2 原始子类型，区分 PXBA/PXBB。 */
    bool eeprom_loaded = false;                                /* 记录 Page3/Page4 是否完整恢复成功。 */

    binding = Handlescan_GetBinding(channel);                 /* 非 A/B 通道会得到 NULL，不允许继续访问 EEPROM。 */
    if (binding == NULL)
    {
        return false;                                         /* 未选中 A/B 通道时不恢复，避免污染全局运行参数。 */
    }
    message = binding->message;                               /* 识别参数只写回所选通道。 */

    handle_type = message->handle_type;                       /* 缓存当前基座型号，后续清刀具字段时需要重新写回。 */
    raw_type_major = message->hand_type_raw_major;            /* 缓存 Page2 原始主类型，避免手动模式丢失手柄身份。 */
    raw_type_minor = message->hand_type_raw_minor;            /* 缓存 Page2 原始子类型，避免 A/B 心跳显示未知型号。 */
    if ((handle_type != PXBA_ONLINES) && (handle_type != PXBB_ONLINES))
    {
        return false;                                         /* 只有 PXBA/PXBB 分体式手柄支持手动磨/刨模式恢复 EEPROM 参数。 */
    }

    Handlescan_PrepareRfidBase(message,
                                               handle_type,
                                               raw_type_major,
                                               raw_type_minor); /* 先重建基座缓存，清掉自动识别模式下的 RFID 刀具规格和标签倍率。 */
    message->tool_type = manual_tool_type;                    /* 手动模式的刀具能力由用户选择，不能再由 RFID 标签决定。 */
    message->raw_tool_type = 0U;                              /* 手动模式没有 RFID 原始型号，清掉旧标签码避免上位机误判。 */
    message->rfid_direction_mask = 0U; /* 清刀具或转手动模式时同步撤销旧RFID方向限制。 */
    message->rfid_result_sequence = 0U; /* 清刀具后取消已装载的RFID编号，旧参数不能作为新刀具使用。 */
    message->diameter = 0U;                                   /* 手动模式没有刀具头标签规格，屏幕规格区应保持空。 */
    message->length = 0U;                                     /* 手动模式不显示 RFID 长度，防止沿用上一把刀具。 */
    message->draw = 0U;                                       /* 手动模式不显示 RFID 角度，避免旧规格残留。 */
    eeprom_loaded = Handlescan_LoadSplitBaseEepromRuntime(binding); /* 恢复本通道手柄 EEPROM Page3/Page4。 */
    if (manual_tool_type == PLANER)
    {
        if (Handlescan_ApplyPxManualPlanerSpeedProfile(message,
                                                       binding->context->tool_info_buf,
                                                       eeprom_loaded) == false)
        {
            eeprom_loaded = false; /* 手动刨刀专用字段缺失时保持0速度并报告恢复失败，绝不使用通用Page4或主控固定值。 */
        }
    }
    Handlescan_LoadPage6SpeedStep(binding);                   /* Page6 步进也跟随本通道手柄 EEPROM。 */

    return eeprom_loaded;                                     /* 返回恢复结果，调用方当前不阻断手动切换，只作为诊断依据。 */
}

/*
 * 函数功能：PXBA/PXBB 重新进入自动识别前，复位指定通道的 RFID 结果游标和在线监测状态。
 * 输入参数：channel 为 A/B 通道。
 * 返回参数：无。
 */
void Handlescan_PrepareSplitAutoIdentify(uint8_t channel)
{
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 取得指定通道绑定，确保 A/B 监测状态独立复位。 */
    HandlescanChannelContext *context; /* 指向待复位通道的扫描状态，不清识别缓存和已装载运行参数。 */

    if (binding == NULL)
    {
        return; /* 非 A/B 通道没有合法 RFID 归属，保持当前状态不变。 */
    }

    context = binding->context; /* 后续只修改这个通道已处理的 RFID 编号和计数。 */
    context->rfid_last_sequence = 0U; /* 与退出自动模式时清零的 RFID 结果序号对齐，首个新结果不会因旧序号碰撞而被忽略。 */
    context->rfid_last_presence_sequence = 0U; /* 新一轮自动识别从新的有效回包开始确认标签存在。 */
    context->rfid_monitor_ticks = 0U; /* 在线监测周期重新计时，避免刚进入自动模式就立即追加请求。 */
    context->rfid_miss_count = 0U; /* 新一轮自动等待不继承上一次射频缺失次数。 */
    context->rfid_monitor_pending = 0U; /* 清除旧请求等待标志，后续只按本轮请求结算。 */
    context->rfid_monitor_ticket = 0U; /* 本轮重新申请 RFID 读取编号，不再查询上次自动识别的请求。 */
    context->rfid_tool_online = 0U; /* 先记为未识别刀具，收到第一个有效标签后再记为在线。 */
}

/*
 * 函数功能：把内存中已保存的 Page6 调速增减量重新写回识别结果，不再次读取 EEPROM。
 * 输入参数：binding 指定 A/B 通道、该通道的 Page6 数据和识别结果。
 * 返回参数：无。
 */
static void Handlescan_ReloadSpeedStepMessage(const HandlescanChannelBinding *binding)
{
    if (binding == NULL)
    {
        return;                                               /* 绑定无效时不写步进，避免误操作另一通道。 */
    }

    Handlescan_UpdateSpeedStepMessage(binding->message, binding->context->speed_step_buf); /* 只把本通道最近 Page6 缓存写回本通道识别结果。 */
}

/*
 * 函数功能：把 EEPROM 刀具规格保存为屏幕使用的数组格式。
 * 输入参数：spec_values 为至少 4 项的输出数组；直径、长度、角度分别以 0.1 mm、0.1 mm、0.1 度为单位；mapped_tool_type 为主控刀具类型。
 * 返回参数：无；数组依次保存长度除以 5、原始直径、原始角度、刀具类型，屏幕显示长度时再乘 5。
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
 * 函数功能：清空刀具规格显示数据，手柄拔出后不再显示旧规格。
 * 输入参数：spec_values 为该通道至少 4 项的规格数组。
 * 返回参数：无。
 */
static void Handlescan_ClearToolSpecValues(uint32_t *spec_values)
{
    spec_values[0] = 0U;
    spec_values[1] = 0U;
    spec_values[2] = 0U;
    spec_values[3] = 0U;
}

/*
 * 函数功能：把按 0.1 单位保存的规格换成整数，小数舍去，最大保留 255。
 * 输入参数：value_tenth 为规格原始值，例如 125 表示 12.5。
 * 返回参数：除以 10 后的整数；超过 255 时返回 255，避免转换成 8 位时变成错误的小数值。
 */
static uint8_t Handlescan_TenthToUint8(uint16_t value_tenth)
{
    uint16_t value = (uint16_t)(value_tenth / 10U);
    /* 识别缓存字段只有 8 位，超过 255 时饱和到上限，避免强制转换回绕成小数值。 */
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
    /* 调用方未提供通道缓存时不执行清零，避免拔出流程写入空地址。 */
    if (message == NULL)
    {
        return;
    }

    memset((void *)&message->eeprom_profile, 0, sizeof(message->eeprom_profile)); /* 真实拔柄或无效插入清除本体配置，RFID掉线路径不得执行此清理。 */
    message->handle_type = 0U;
    message->hand_type_raw_major = 0U;                       /* 清掉 Page2 原始主类型，避免下一次上线前外部通信读到旧手柄编码。 */
    message->hand_type_raw_minor = 0U;                       /* 清掉 Page2 原始子类型，保持识别缓存和当前插拔状态一致。 */
    message->tool_type = 0U;
    message->raw_tool_type = 0U;                              /* 清掉刀具原始型号，避免下次上线前驱动或上位机读取旧 PXM/PXP/RFID 代号。 */
    message->rfid_direction_mask = 0U; /* 清刀具或转手动模式时同步撤销旧RFID方向限制。 */
    message->rfid_result_sequence = 0U; /* 清刀具后取消已装载的RFID编号，旧参数不能作为新刀具使用。 */
    message->rfid_cache_hit = 0U;                              /* 清掉同一 EPC 重识别标志，避免普通 EEPROM 上线误继承射频调节值。 */
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
    message->speed_zzstep = 0U;                              /* 清除正转调速步长，下次识别手柄时从Page6读取，RFID未提供时使用默认值。 */
    message->speed_fzstep = 0U;                              /* 清掉反转调速步进，避免换手柄后沿用旧步进。 */
    message->speed_oscstep = 0U;                             /* 清掉往复调速步进，保持离线态没有可用调速参数。 */
    message->speed_zzstep_large = 0U;                        /* 清掉正转大步进，避免下一支手柄误用旧 Page6 快键步进。 */
    message->speed_fzstep_large = 0U;                        /* 清掉反转大步进，保证 B/A 通道重新上线后按本通道 EEPROM 装载。 */
    message->speed_oscstep_large = 0U;                       /* 清掉往复大步进，防止不支持往复的手柄残留快键配置。 */
    message->freq_min = 0U;                                  /* 清掉频率下限缓存，避免下一次调频沿用旧手柄边界。 */
    message->freq_max = 0U;                                  /* 清掉频率上限缓存，避免下一次调频沿用旧手柄边界。 */
    message->freq_default = 0U;                              /* 清掉默认频率，避免往复模式沿用旧 Page4 默认频率。 */
    message->speed_alarm_for = 0U;                           /* 清掉正转速度报警值，避免蜂鸣阈值跨手柄残留。 */
    message->speed_alarm_rev = 0U;                           /* 清掉反转速度报警值，避免蜂鸣阈值跨手柄残留。 */
    message->freq_alarm_osc = 0U;                            /* 清掉往复频率报警值，避免蜂鸣阈值跨手柄残留。 */
    message->overloadThresholdFor = 0U;                       /* 清掉正转电流保护阈值，避免新手柄上线前沿用旧 EEPROM 电流限制。 */
    message->overloadThresholdRev = 0U;                       /* 清掉反转电流保护阈值，保证通道离线态不会下发旧保护值。 */
    message->overloadThresholdOSC = 0U;                       /* 清掉往复电流保护阈值，下一次上线重新按 Page4 或 RFID 写入。 */
    message->run_direction = 0U;                             /* 清掉默认方向，下一次上线重新按 Page4/业务规则初始化。 */
}

/*
 * 函数功能：清空本通道的识别结果和 RFID 读取状态；通道记忆和当前工作参数由插拔消息处理。
 * 输入参数：binding 指定要清理的通道；NULL 时不处理。
 * 返回参数：无。
 */
static void Handlescan_ClearChannelState(const HandlescanChannelBinding *binding)
{
    HandlescanChannelContext *context;                       /* 指向需要复位的通道运行状态。 */

    if (binding == NULL)
    {
        return;                                               /* 非 A/B 通道不清任何全局缓存。 */
    }

    context = binding->context;                              /* 只清这个通道的 RFID 状态，不影响另一通道。 */
    Handlescan_ClearRecognizeMessage(binding->message);      /* 扫描层只清识别缓存，通道记忆和工作态仍由插拔事件处理。 */
    context->rfid_last_sequence = 0U;                        /* 下次即使还是同一标签，也要重新装载刀具参数。 */
    context->rfid_last_presence_sequence = 0U;               /* 清存在序号，避免旧标签状态延续。 */
    context->rfid_miss_count = 0U;                           /* 清在线缺失计数。 */
    context->rfid_monitor_pending = 0U;                      /* 清在线监测等待标志。 */
    context->rfid_monitor_ticket = 0U;                       /* 清掉请求编号，重新插入后不能再查上一次的读取结果。 */
    context->rfid_tool_online = 0U;                          /* 标记该通道尚未识别到刀具。 */
    Rfid_ClearChannelResult(binding->channel);               /* 同步清本通道 RFID 结果，避免旧序号挡住新上线。 */
}

/*
 * 函数功能：判断报警是否属于手柄 EEPROM 校验或型号错误。
 * 输入参数：alarm_value 为待判断的报警码。
 * 返回参数：1 表示 A/B/AB 手柄校验报警，0 表示其它报警。
 */
static uint8_t Handlescan_IsHandleVerifyAlarm(uint8_t alarm_value)
{
    return Pubinterface_IsHandleVerifyAlarm(alarm_value) ? 1U : 0U; /* 返回 1 表示这是手柄 EEPROM 校验/型号报警，返回 0 表示不是本模块可清的报警。 */
}

/*
 * 函数功能：没有持续报警时蜂鸣一次，提示插入或拔出；有报警时不打断报警声。
 * 输入参数：无。
 * 返回参数：无。
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
    if ((channel_alarm == WORK_ALARM_HANDLE_UNSUPPORTED) || (peer_alarm == WORK_ALARM_HANDLE_UNSUPPORTED))
    {
        return WORK_ALARM_HANDLE_UNSUPPORTED; /* 任一侧不受支持时保留71号提示，不能合并成普通校验异常图。 */
    }
    if ((Handlescan_IsHandleVerifyAlarm(channel_alarm) != 0U) &&
        (Handlescan_IsHandleVerifyAlarm(peer_alarm) != 0U) &&
        (channel_alarm != peer_alarm))
    {
        return WORK_ALARM_HANDLE_MODEL_ERROR_AB;             /* A/B 都坏时上传 AB，避免上位机只能看到最后一个通道。 */
    }

    return channel_alarm;                                    /* 只有单通道失败时保持 A 或 B 的独立报警码。 */
}

/*
 * 函数功能：清除本通道报警前，检查另一通道是否仍有 EEPROM 校验失败，防止把它的报警一起清掉。
 * 输入参数：channel 为当前准备清报警的通道，调用方须传 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：另一通道仍在校验失败状态时返回其报警码，否则返回 0。
 */
static uint8_t Handlescan_GetPeerActiveHandleAlarm(uint8_t channel)
{
    if (channel == CHANNEL_A) /* 当前 A 准备清报警时，需要先检查 B 是否仍持有认证失败报警。 */
    {
        if ((s_b_context.stage != HANDLESCAN_STAGE_IDLE) &&
            (Handlescan_IsHandleVerifyAlarm(s_b_context.last_alarm) != 0U))
        {
            return s_b_context.last_alarm;                           /* A 通道准备清报警时，B 通道仍失败保持，则返回 B 通道报警码。 */
        }

        return 0U;                                           /* B 通道没有可继承的手柄认证报警。 */
    }

    if ((s_a_context.stage != HANDLESCAN_STAGE_IDLE) &&
        (Handlescan_IsHandleVerifyAlarm(s_a_context.last_alarm) != 0U))
    {
        return s_a_context.last_alarm;                               /* B 通道准备清报警时，A 通道仍失败保持，则返回 A 通道报警码。 */
    }

    return 0U;                                               /* A 通道没有可继承的手柄认证报警。 */
}

/*
 * 函数功能：向 UIDP 队列推送手柄 EEPROM 校验报警弹窗或关闭请求。
 * 输入参数：alarm_value 为 A/B/AB 手柄校验报警码；enable_flag 为 true 时显示报警，为 false 时关闭弹窗。
 * 返回参数：无。
 */
static void Handlescan_SendHandleVerifyAlarmUi(uint8_t alarm_value, bool enable_flag)
{
    uint8_t display_value[10] = {0U};                         /* UI_AIARM_ID 只读取 Value[0]，其余字节清零避免沿用上一条报警参数。 */

    if (enable_flag != false)
    {
        if (Handlescan_IsHandleVerifyAlarm(alarm_value) == 0U)
        {
            return;                                          /* 只处理手柄 EEPROM 校验系列报警，避免运行中插拔等其它码误占用 90 号弹窗。 */
        }

        display_value[0] = alarm_value;                      /* 把 A/B/AB 来源报警码送给 UIAIARMDP，当前屏幕统一映射到 90 号手柄校验异常图。 */
        SendUIDSMessage(UI_AIARM_ID, true, display_value);   /* 持续报警建立后立即弹窗，避免只有蜂鸣和上位机报警但屏幕无提示。 */
        return;                                              /* 显示路径已经完成，不再执行关闭弹窗分支。 */
    }

    SendUIDSMessage(UI_AIARM_ID, false, NULL);                /* 手柄校验报警解除后关闭报警弹窗，恢复主运行界面普通显示。 */
}

/*
 * 函数功能：清除已恢复或拔出通道的持续及临时校验报警，并保留另一侧和其它系统故障。
 * 输入参数：channel 当前扫描通道；alarm_value 当前通道最后一次手柄 EEPROM 校验报警码。
 * 返回参数：无。
 */
static void Handlescan_ClearChannelAlarm(uint8_t channel, uint8_t alarm_value)
{
    uint8_t peer_alarm;                                      /* 保存另一通道仍在失败保持态时需要继承的报警码。 */

    if (Handlescan_IsHandleVerifyAlarm(alarm_value) == 0U)
    {
        return;                                              /* 非手柄校验报警不是本函数负责的报警，保持原报警状态不变。 */
    }

    if ((s_transient_screen_alarm_ticks != 0U) && (s_transient_screen_alarm_value == alarm_value))
    {
        s_transient_screen_alarm_ticks = 0U; /* 本通道已恢复或拔出，不再把旧临时校验提示保持到3秒结束。 */
        s_transient_screen_alarm_value = WORK_ALARM_NONE; /* 显示任务下一周期按实际归属关闭旧90号图，即使关窗消息丢失也能收尾。 */
        if (WorkMessage.alarm_flag == false)
        {
            SendAlarmMessage(0U); /* 无其它持续故障时结束本次校验蜂鸣，不能清除另一路的真实报警。 */
        }
    }

    if ((WorkMessage.alarm_flag == false) || (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) == 0U))
    {
        return;                                              /* 全局报警已被其他模块改写或清除，本通道不再覆盖它。 */
    }

    peer_alarm = Handlescan_GetPeerActiveHandleAlarm(channel); /* 当前通道清报警前，先检查另一通道是否仍需保持手柄认证报警。 */
    if (peer_alarm != 0U)
    {
        WorkAlarm_Set(peer_alarm);                          /* 继承另一通道报警，重试或拔出消抖期间也不能清掉尚未解除的异常。 */
        SendAlarmMessage(peer_alarm);                        /* 重新通知蜂鸣任务当前仍存在手柄认证报警。 */
        Handlescan_SendHandleVerifyAlarmUi(peer_alarm, true); /* 屏幕弹窗同步切换到仍失败的通道报警，避免拔出一只坏手柄后误清提示。 */
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, peer_alarm); /* 输出报警继承调试码。 */
        return;                                              /* 已由另一通道接管报警，不执行清零。 */
    }

    WorkAlarm_Clear();                                       /* 没有其它通道报警时通过统一入口清除，仍插着的不支持手柄受锁存保护。 */
    SendAlarmMessage(0U);                                     /* 通知蜂鸣任务退出报警蜂鸣模式。 */
    Handlescan_SendHandleVerifyAlarmUi(WORK_ALARM_NONE, false); /* 最后一只坏手柄拔出或恢复后关闭手柄校验报警弹窗。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_CLEAR, 0U); /* 输出报警清除调试码。 */
}

/*
 * 函数功能：判断本次手柄校验失败是否只能作为运行中另一路的临时提示。
 * 输入参数：channel 当前失败通道；alarm_value 当前失败报警码。
 * 返回参数：1 表示只响 3 秒并临时弹窗，0 表示写入全局报警。
 */
static uint8_t Handlescan_ShouldUseTransientAlarm(uint8_t channel, uint8_t alarm_value)
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
    SendAlarmMessageTimed(alarm_value, ALARM_VERIFY_MS); /* 蜂鸣器只响 3 秒，避免覆盖当前工作通道。 */
    ExternalComm_SendTransientAlarm(alarm_value, ALARM_VERIFY_MS); /* 上位机收到非 0 后，3 秒后会收到 0 自动关闭弹窗。 */
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
    /* 目标缓存为空时不能保存识别结果，直接退出以保护扫描任务。 */
    if (message == NULL)
    {
        return;
    }

   
    message->handle_type = mapped_model;
    message->hand_type_raw_major = raw_type_major;            /* 原始主类型先保存在识别缓存，等待 PlugORunPLUGActive 统一搬到 MemoryMsg。 */
    message->hand_type_raw_minor = raw_type_minor;            /* 原始子类型先保存在识别缓存，避免扫描任务直接写通道记忆。 */
    message->tool_type = Handlescan_MapToolType(mapped_tool_model); /* 业务刀具类型按 PXM/PXP 归一，屏幕和开口定位都看这个字段。 */
    message->raw_tool_type = mapped_tool_model;               /* 原始刀具型号保留给上位机和驱动兼容判断。 */
    message->diameter = Handlescan_TenthToUint8(diameter_tenth);
    message->length = (uint16_t)(length_tenth / 10U);
    message->draw = Handlescan_TenthToUint8(angle_tenth);
}

/*
 * 函数功能：把自带刀具能力的EEPROM手柄型号写入通道识别缓存。
 * 输入参数：message 目标通道识别缓存；mapped_model 为手柄表映射后的型号；raw_type_major/raw_type_minor 为 Page2 原始手柄类型字节。
 * 返回参数：无。
 */
static void Handlescan_UpdateSelfTypedResult(ChannelrecognizeMessage_t *message,
                                                             uint8_t mapped_model,
                                                             uint8_t raw_type_major,
                                                             uint8_t raw_type_minor)
{
    Handlescan_UpdateRecognizeMessage(message,
                                      mapped_model,
                                      raw_type_major,
                                      raw_type_minor,
                                      mapped_model,
                                      0U,
                                      0U,
                                      0U); /* 这类型号自身就是手柄和刀具能力来源；DHYTM虽使用6B 10，也无需依赖Page3工具编码。 */
}

/*
 * 函数功能：建立持续报警状态，并在手柄 EEPROM 校验失败时同步推送屏幕报警弹窗。
 * 输入参数：channel 当前扫描通道；alarm_value 当前模块生成的报警码。
 * 返回参数：无。
 */
static void Handlescan_RaiseAlarm(uint8_t channel, uint8_t alarm_value)
{
    uint8_t report_alarm = alarm_value;                      /* 默认按当前通道报警码上报。 */

    if (Handlescan_HasUnsupportedHandle() && WorkMessage.alarm_flag &&
        !Pubinterface_IsHandleVerifyAlarm(WorkMessage.alarm_value))
    {
        return; /* 已有驱动等系统故障时保留其报警归属；不支持状态仍锁存，系统故障解除后继续提示。 */
    }

    if (Handlescan_IsHandleVerifyAlarm(alarm_value) != 0U)
    {
        report_alarm = Handlescan_CombineHandleAlarm(alarm_value,
                                                     Handlescan_GetPeerActiveHandleAlarm(channel)); /* 另一通道也失败时升级为 AB 报警。 */
    }

    WorkAlarm_Set(report_alarm);                             /* 所有持续报警统一写入 WorkMessage.alarm_value。 */
    SendAlarmMessage(report_alarm);                          /* 持续报警蜂鸣直到对应清除路径发送 0。 */
    Handlescan_SendHandleVerifyAlarmUi(report_alarm, true);  /* 手柄 EEPROM 校验持续报警需要主动推送屏幕弹窗，非手柄报警由 helper 自动忽略。 */
    Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_ALARM_SET, report_alarm);
}

/*
 * 函数功能：拒绝当前模式不支持或型号未知的手柄，锁存报警且不发布上线事件。
 * 输入参数：binding 为已完成认证并读取 Page2 的本通道资源。
 * 返回参数：无。
 */
static void Handlescan_RejectUnsupportedHandle(const HandlescanChannelBinding *binding)
{
    HandlescanChannelContext *context = binding->context; /* 锁存只属于本通道，另一侧插拔不能清除它。 */
    Handlescan_ClearChannelState(binding); /* 清识别缓存并取消RFID结果，禁止不支持的基座提前上线。 */
    Handlescan_ClearToolSpecValues(binding->spec_values); /* 不允许旧刀具规格残留到被拒绝的手柄。 */
    context->unsupported_latched = 1U; /* 锁存独立于重试计时和屏幕报警，只有确认拔出才可释放。 */
    context->last_alarm = WORK_ALARM_HANDLE_UNSUPPORTED; /* 保存本通道归属，拔出时保留另一侧仍有效的报警。 */
    context->stage = HANDLESCAN_STAGE_VERIFY_FAIL; /* 复用失败态的拔出消抖，锁存期间不再执行认证重试。 */
    Handlescan_RaiseAlarm(binding->channel, context->last_alarm); /* 建立持续提示，不使用限时或三声蜂鸣。 */
}

/*
 * 函数功能：清掉本通道的校验失败次数和重试等待时间，下一轮识别重新计数。
 * 输入参数：context 为该通道的扫描进度；NULL 时不处理。
 * 返回参数：无。
 */
static void Handlescan_ResetVerifyRetry(HandlescanChannelContext *context)
{
    if (context == NULL)
    {
        return;                                               /* 上下文无效时不写任何通道状态。 */
    }

    context->verify_retry_count = 0U;                         /* 清掉连续失败次数，下一次认证重新从第 1 次开始。 */
    context->verify_retry_wait_ticks = 0U;                    /* 清掉快速/慢速重试等待计数，避免复用旧等待时间。 */
}

/*
 * 函数功能：记录本次失败；未到次数上限就稍后重试，达到上限则按需要报警并改为慢速重试。
 * 输入参数：channel 为 A/B 通道；context 保存本通道进度和失败次数；alarm_value 为报警码（0 不报警）；retry_max 为连续失败上限。
 * 返回参数：无。
 */
static void Handlescan_EnterRetryOrFailWithLimit(uint8_t channel,
                                                 HandlescanChannelContext *context,
                                                 uint8_t alarm_value,
                                                 uint8_t retry_max)
{
    if ((context == NULL) || (retry_max == 0U))
    {
        return;                                               /* 运行时参数异常或重试上限无效时保持当前状态，避免空指针写入。 */
    }

    if (context->verify_retry_count < retry_max)
    {
        ++context->verify_retry_count;                        /* 记录本次失败，达到上限后才进入最终报警保持。 */
    }

    if (context->verify_retry_count < retry_max)
    {
        context->verify_retry_wait_ticks = 0U;                /* 快速重试从当前周期重新计时。 */
        context->stage = HANDLESCAN_STAGE_RETRY_WAIT;         /* 保持插入态，稍后重新进入 VERIFY，不要求用户再次插拔。 */
        Handlescan_DebugTrace(channel, HANDLESCAN_DBG_STEP_VERIFY_RETRY, context->verify_retry_count);
        return;
    }

    /* 只有非零报警码才进入提示与锁存流程；0 表示本次失败无需建立报警状态。 */
    if (alarm_value != 0U)
    {
        if (Handlescan_ShouldUseTransientAlarm(channel, alarm_value) != 0U)
        {
            if (context->last_alarm != alarm_value)
            {
                context->last_alarm = alarm_value;            /* 记录另一路坏手柄归属，但不写 WorkMessage。 */
                Handlescan_RaiseTransientHandleAlarm(channel, alarm_value); /* 第一次最终失败时只提示 3 秒。 */
            }
        }
        else if ((context->last_alarm != alarm_value) || (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) == 0U))
        {
            context->last_alarm = alarm_value;                /* 第一次达到最终失败或转为持续报警时记录归属。 */
            Handlescan_RaiseAlarm(channel, context->last_alarm); /* 最终失败才写报警并触发蜂鸣。 */
        }
        else
        {
            context->last_alarm = alarm_value;                /* 同类报警保持时只刷新归属，不重复蜂鸣。 */
        }
    }

    context->verify_retry_wait_ticks = 0U;                    /* 最终报警保持里的慢速自恢复重试从当前周期计时。 */
    context->stage = HANDLESCAN_STAGE_VERIFY_FAIL;            /* 最终失败保持态继续监视拔出并慢速重试。 */
}

/*
 * 函数功能：EEPROM 校验或型号读取失败时，按普通手柄的次数上限安排重试或报警。
 * 输入参数：channel 为 A/B 通道；context 保存本通道进度和失败次数；alarm_value 为达到上限后的报警码（0 不报警）。
 * 返回参数：无。
 */
static void Handlescan_EnterRetryOrFail(uint8_t channel,
                                        HandlescanChannelContext *context,
                                        uint8_t alarm_value)
{
    Handlescan_EnterRetryOrFailWithLimit(channel,
                                         context,
                                         alarm_value,
                                         HANDLESCAN_VERIFY_RETRY_MAX); /* 普通手柄认证继续保留原 3 次容错。 */
}

/*
 * 函数功能：把 EEPROM 校验结果换成该通道的报警码，是否已达到报警次数由调用方判断。
 * 输入参数：channel 为 A/B 通道；verify_status 为 EEPROM 校验结果。
 * 返回参数：校验成功返回 0，其它结果返回对应通道的手柄校验报警码。
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
 * 函数功能：按通道选择EEPROM总线，每轮只推进一个认证块，让泵任务及时取得业务锁。
 * 输入参数：binding 为固定通道绑定；result 保存底层认证明细。
 * 返回参数：PENDING表示下轮继续，其它值保持原AT24CS32认证结果含义。
 */
static AT24CS32_CRC_Status Handlescan_VerifyEeprom(const HandlescanChannelBinding *binding,
                                                   AT24CS32_CRC_Result *result)
{
    /* 只把逻辑通道换算成物理接口，认证结果仍写回原逻辑A/B上下文。 */
    uint8_t physical_channel = BoardProfile_MapHandlePhysicalChannel(binding->channel);

    AT24CS32_CRC_Status status = AT24CS32_VerifyCrcStep(
        (physical_channel == BOARD_PROFILE_HANDLE_CHANNEL_A) ? 0U : 1U,
        &binding->context->verify_context); /* 物理映射不变，中间进度只保存在本逻辑通道。 */
    DIAG_EVENT(DIAG_EVENT_HANDLE_VERIFY, binding->channel, binding->context->verify_context.step, status, physical_channel); /* 只记录认证进度和错误码，不导出身份认证数据。 */
    *result = binding->context->verify_context.result; /* 保留原调用方的认证明细，不再依赖共享静态缓存。 */
    return status; /* 未完成与失败分开，PENDING不能触发重试或报警。 */
}

/*
 * 函数功能：初次识别手柄时复制本轮认证已保存的参数；不满足条件时重新读取EEPROM。
 * 输入参数：binding 为固定通道绑定；addr/len 为 EEPROM 地址和长度；data 为接收缓存。
 * 返回参数：原样返回底层读取结果，1 表示成功，0 表示失败。
 */
static uint8_t Handlescan_ReadEepromBytes(const HandlescanChannelBinding *binding,
                                          uint16_t addr,
                                          uint8_t *data,
                                          uint16_t len)
{
    /* 连续读取必须与短接检测使用同一套物理映射，避免读到另一侧手柄。 */
    uint8_t physical_channel = BoardProfile_MapHandlePhysicalChannel(binding->channel);

    if (((binding->context->stage == HANDLESCAN_STAGE_READ_INFO) ||
         (binding->context->stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL)) &&
        (binding->context->verify_context.verified != 0U) && (data != NULL) && (len != 0U) &&
        (addr >= AT24CS32_AUTH_DATA_START_ADDR) &&
        ((uint32_t)addr + len <= AT24CS32_AUTH_DATA_START_ADDR + AT24CS32_AUTH_DATA_LENGTH)) {
        memcpy(data, &binding->context->verify_context.pages[addr - AT24CS32_AUTH_DATA_START_ADDR], len); /* 从本轮已通过认证的Page2~Page8复制参数，避免持锁期间重复访问EEPROM。 */
        return 1U; /* 所需参数已复制完成，返回成功，不再访问EEPROM。 */
    }

    /* 原物理A接口的EEPROM固定挂在I2C2。 */
    if (physical_channel == BOARD_PROFILE_HANDLE_CHANNEL_A)
    {
        return AT24CS32_ReadBytes_I2C2(addr, data, len);
    }

    /* 原物理B接口的EEPROM固定挂在I2C3。 */
    return AT24CS32_ReadBytes_I2C3(addr, data, len);
}

/*
 * 函数功能：初次识别手柄时复制本轮认证已保存的整页数据；不满足条件时重新读取并校验EEPROM页。
 * 输入参数：binding 指定通道；page_index 为从 0 开始的页号；page_buf 接收整页数据。
 * 返回参数：原样返回底层页读取结果，1 表示成功，0 表示失败。

 */
static uint8_t
Handlescan_ReadEepromPage(const HandlescanChannelBinding *binding,
                          uint16_t page_index, uint8_t *page_buf) {
  /* 整页读取沿用统一物理映射，逻辑消息和页面数据归属不发生交换。 */
  uint8_t physical_channel = BoardProfile_MapHandlePhysicalChannel(binding->channel);

  if (((binding->context->stage == HANDLESCAN_STAGE_READ_INFO) ||
       (binding->context->stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL)) &&
      (binding->context->verify_context.verified != 0U) && (page_buf != NULL) &&
      (page_index >= AT24CS32_AUTH_PAGE_START_INDEX) &&
      (page_index < AT24CS32_AUTH_PAGE_START_INDEX + AT24CS32_AUTH_PAGE_COUNT)) {
    memcpy(page_buf, &binding->context->verify_context.pages[(page_index - AT24CS32_AUTH_PAGE_START_INDEX) * AT24CS32_PAGE_SIZE], AT24CS32_PAGE_SIZE); /* 所需页已通过本轮认证，直接复制保存的32字节数据。 */
    return 1U; /* 整页复制完成，返回成功；不满足上述条件时才执行下面的I2C读取。 */
  }

  /* 原物理A接口的EEPROM页固定从I2C2读取。 */
  if (physical_channel == BOARD_PROFILE_HANDLE_CHANNEL_A) {
    return AT24CS32_ReadPage_I2C2(page_index, page_buf);
  }

  /* 原物理B接口的EEPROM页固定从I2C3读取。 */
  return AT24CS32_ReadPage_I2C3(page_index, page_buf);
}

/*
 * 函数功能：连续检查拔出信号，确认后通知离线并清除本通道数据；短暂断开后恢复则重新识别。
 * 输入参数：binding 指定通道；is_inserted 为本轮检测结果，0 表示高电平（可能拔出），非 0 表示低电平（插着）。
 * 返回参数：true 表示本轮已处理拔出或等待确认，调用方应结束本轮；false 表示可以继续检查插入和识别。

 */
static bool
Handlescan_ProcessDisconnect(const HandlescanChannelBinding *binding,
                             uint8_t is_inserted) {
  HandlescanChannelContext *context =
      binding->context; /* 所有拔出计数只写本通道上下文。 */
  uint32_t *spec_values =
      binding->spec_values; /* 离线时只清本通道规格显示缓存。 */
  uint8_t verify_alarm_was_active = 0U; /* 记录清除前是否存在持续校验报警，用于判断是否需要跨任务补发关窗。 */

  if (is_inserted == 0U) {
    context->debounce.in_debounce_ticks =
        0U; /* 一旦检测到拔出候选，就清掉插入去抖计数。 */
    context->verify_start_wait_ticks =
        0U; /* 同时清掉认证前等待计数，避免下次误续跑。 */
    Handlescan_ResetVerifyRetry(
        context); /* 拔出候选出现时，下一轮认证必须重新累计失败次数。 */

    if (context->stage == HANDLESCAN_STAGE_ONLINE) {
      if (context->debounce.out_debounce_ticks <
          HANDLESCAN_ONLINE_DISCONNECT_FILTER_TICKS) {
        ++context->debounce
              .out_debounce_ticks; /* 在线阶段只累计连续高电平次数，单个 10ms 毛刺不能立即退出 ONLINE。 */
      }
      if (context->debounce.out_debounce_ticks <
          HANDLESCAN_ONLINE_DISCONNECT_FILTER_TICKS) {
        return true; /* 连续高电平未满配置门槛时保持原在线状态，本轮禁止进入 EEPROM 重新认证。 */
      }

      ++context->navigation_generation; /* 高电平持续达到要求后改变计数，让外部导航放弃未读完的旧手柄数据。 */
      context->stage =
          HANDLESCAN_STAGE_DEBOUNCE_OUT; /* 达到配置门槛后进入原拔出去抖，后续仍按总计 500ms 确认真实离线。 */
      return true; /* 当前连续高电平次数已经计入 500ms 计数，下一周期继续原拔出确认流程。 */
    }

    /* 本通道曾通知过上线，或正在等待 RFID/保持校验失败时，拔出后都需要通知离线。 */
    if ((context->offline_event_required != 0U) || /* 只要本通道曾请求过上线，阶段即使被一次插入毛刺改写，也必须继续完成配对离线事件。 */
        (context->stage == HANDLESCAN_STAGE_DEBOUNCE_OUT) ||
        (context->stage == HANDLESCAN_STAGE_VERIFY_FAIL) ||
        ((context->rfid_wait_started != 0U) &&
         ((context->stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL) ||
          (context->stage == HANDLESCAN_STAGE_RETRY_WAIT)))) { /* RFID 基座已提前发布上线时，等待首包或内部重试期间拔出也必须发送离线事件；不读取异步 WorkMessage，避免 IDLE 重复进入拔出流程。 */
      if (context->stage != HANDLESCAN_STAGE_DEBOUNCE_OUT) {
        context->debounce.out_debounce_ticks =
            0U; /* 首次进入拔出去抖时，先把拔出去抖计数清零。 */
        ++context->navigation_generation; /* 开始确认拔出就让外部导航放弃旧读取，不等离线消息发出后再处理。 */
        context->stage =
            HANDLESCAN_STAGE_DEBOUNCE_OUT; /* 状态机切到“拔出去抖”阶段。 */
      }

      if (context->debounce.out_debounce_ticks <
          HANDLESCAN_REMOVE_DEBOUNCE_TICKS) {
        ++context->debounce
              .out_debounce_ticks; /* 拔出去抖未满阈值时，仅累计次数后返回。 */
        return true;               /* 本轮不做其他动作，等待下一个扫描周期。 */
      }

      if (SendKeyBehMessage(PLUGunPLUG, binding->unplug_key) == false)
      {
        context->stage = HANDLESCAN_STAGE_DEBOUNCE_OUT; /* 离线事件尚未进入队列时保持拔出态，禁止静默回到 IDLE 后永久保留旧在线图标。 */
        context->debounce.out_debounce_ticks = HANDLESCAN_REMOVE_DEBOUNCE_TICKS; /* 计数保持饱和，下一次 10ms 扫描只重试入队，不重新等待 500ms。 */
        return true; /* 本轮不清业务缓存、不蜂鸣，等待队列具备空间后再一次性完成正式离线收尾。 */
      }

      context->offline_event_required = 0U; /* 离线事件已可靠进入队列，本通道此前上线状态已经获得配对的离线通知。 */
      context->debounce.out_debounce_ticks =
          0U; /* 拔出去抖完成且离线事件入队后，清掉计数器。 */
      verify_alarm_was_active =
          (uint8_t)((Handlescan_IsHandleVerifyAlarm(context->last_alarm) != 0U) &&
                    (WorkMessage.alarm_flag == true) &&
                    (Handlescan_IsHandleVerifyAlarm(WorkMessage.alarm_value) != 0U)); /* 只有本次拔出前确有持续校验报警，才允许建立关窗交接。 */
      context->unsupported_latched = 0U; /* 仅在拔出消抖完成且离线事件成功入队后解除本通道锁存。 */
      Handlescan_ClearChannelAlarm(binding->channel, context->last_alarm); /* A
                                                                              通道坏手柄拔出后，释放本通道手柄型号错误报警。
                                                                            */
      if ((verify_alarm_was_active != 0U) && (WorkMessage.alarm_flag == false))
      {
        context->verify_alarm_close_pending = 1U; /* 最后一项校验报警已清零；插拔任务必须在可能的 xQueueReset 之后再关闭 90 号图。 */
      }
      context->stage = HANDLESCAN_STAGE_IDLE; /* 本通道状态机回到空闲态。 */
      context->last_alarm = 0U; /* 清掉本通道最近一次报警缓存。 */
      context->tool_source =
          HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 基座拔出后刀具来源回到默认，历史
                                                  RFID 缓存只作记忆不直接上线。
                                                */
      context->rfid_wait_ticks = 0U;    /* 清掉本通道 RFID 等待计数。 */
      context->rfid_monitor_ticks = 0U; /* 清掉本通道 RFID 低频监测计数。 */
      Handlescan_ClearChannelState(binding); /* 清空本通道在线与识别状态。 */
      Handlescan_ClearToolSpecValues(
          spec_values); /* 同步清空本通道刀具规格缓存，避免 UI 残留旧值。 */
      context->rfid_wait_started = 0; /* 本通道基座拔出后结束本轮 RFID
                                         等待，下次插入必须从第一轮重新开始。 */
      Handlescan_DebugTrace(
          binding->channel, HANDLESCAN_DBG_STEP_REMOVE_PASS,
          HANDLESCAN_REMOVE_DEBOUNCE_TICKS); /* 输出“拔出去抖通过”报文。 */
      Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_OFFLINE,
                            0U); /* 输出“离线完成”报文。 */
      //  (void)Handlescan_HandleRunningPlugAlarm(1U);     /*
      //  如果电机仍在运行，则补充触发运行中插拔报警。 */
      Handlescan_BeepOnceIfNoAlarm(); /* 没有运行中插拔或其他报警时，给 A
                                         通道拔出确认单响。 */
      return true;                    /* 本通道离线处理完成，本轮到此结束。 */
    }

    /* 还没通知过上线就又检测到拔出，只清本通道扫描状态，不再通知屏幕离线。 */
    context->debounce.out_debounce_ticks =
        0U; /* 静默复位时也要把拔出去抖计数清零。 */
    context->stage =
        HANDLESCAN_STAGE_IDLE; /* 回到空闲态，等待下一次真实插入。 */
    context->tool_source =
        HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3; /* 插入取消后恢复默认来源。 */
    context->rfid_wait_ticks = 0U;         /* 插入取消后清掉 RFID 等待计数。 */
    context->rfid_monitor_ticks = 0U;      /* 插入取消后清掉 RFID 监测计数。 */
    Handlescan_ClearChannelState(binding); /* 保证本通道识别状态保持关闭。 */
    Handlescan_ClearToolSpecValues(
        spec_values); /* 插入未完成时也清空本通道刀具规格缓存。 */
    return true;      /* 当前只是插入取消，不做 UI 和报文更新。 */
  }
  if ((context->stage == HANDLESCAN_STAGE_ONLINE) &&
      (context->debounce.out_debounce_ticks != 0U)) {
    context->debounce.out_debounce_ticks =
        0U; /* 在线短接在配置门槛前恢复时只清毛刺计数，保持 ONLINE 且不重新访问 EEPROM。 */
  }
  if (context->stage == HANDLESCAN_STAGE_DEBOUNCE_OUT) {
    if (context->unsupported_latched != 0U)
    {
      context->debounce.out_debounce_ticks = 0U; /* 拔出未满确认时间又接通，只撤销拔出计数，不解除报警。 */
      context->stage = HANDLESCAN_STAGE_VERIFY_FAIL; /* 被拒绝手柄回到保持态，不能因接触抖动重新识别上线。 */
      return true; /* 仍是同一次插入，继续等待真实拔出。 */
    }
    context->debounce.out_debounce_ticks =
        0U; /* 短接在 500ms 拔出确认前恢复时，旧拔出去抖计数不能继续沿用。 */
    context->debounce.in_debounce_ticks =
        0U; /* 重新插入必须重新累计 50ms 稳定时间，不能直接恢复旧在线状态。 */
    context->verify_start_wait_ticks =
        0U; /* 新目标重新执行认证前等待，避免 EEPROM 尚未稳定就读取导航首页。 */
    Handlescan_ResetVerifyRetry(context); /* 重新识别使用新的认证重试周期，不能继承拔出前的失败次数。 */
    context->stage =
        HANDLESCAN_STAGE_DEBOUNCE_IN; /* 保留界面在线标志，但扫描层回到完整插入认证流程。 */
  }

  return false; /* 短接仍成立时交给插入和认证阶段继续处理。 */
}


/*
 * 函数功能：确认插入稳定后轮换执行逐页认证；未完成时让出调度，失败仍按原间隔重试。
 * 输入参数：binding 指定 A/B 通道的引脚、EEPROM 和扫描进度。
 * 返回参数：true 表示本轮应结束；false 表示可继续处理后续阶段。校验成功时会进入读取信息阶段。
 */
static bool
Handlescan_ProcessInsertAndVerify(const HandlescanChannelBinding *binding) {
  HandlescanChannelContext *context =
      binding->context; /* 认证阶段只维护本通道计数和状态。 */
  AT24CS32_CRC_Result
      verify_result; /* 保存底层 CRC 认证明细，便于现场定位 EEPROM 失败页。 */
  AT24CS32_CRC_Status
      verify_status; /* 保存本轮认证结果，原错误码和报警映射保持不变。 */

  if ((context->stage != HANDLESCAN_STAGE_VERIFY) &&
      (context->stage != HANDLESCAN_STAGE_READ_INFO)) {
    context->verify_context.step = 0U; /* 新插入、失败等待或RFID重认证都从Page1重新开始，不能续用半轮数据。 */
    context->verify_context.verified = 0U; /* 仅完整认证后的READ_INFO阶段有权复用本轮业务页。 */
  }

  if (context->stage == HANDLESCAN_STAGE_IDLE) {
    context->debounce.in_debounce_ticks =
        0U; /* 新一轮插入开始时，先清空插入去抖计数。 */
    context->debounce.out_debounce_ticks =
        0U; /* 同时清空拔出去抖计数，避免带入上一次状态。 */
    context->verify_start_wait_ticks = 0U; /* 清空认证前等待计数。 */
    Handlescan_ResetVerifyRetry(context);  /* 新手柄插入从 0 次失败开始认证。 */
    context->stage =
        HANDLESCAN_STAGE_DEBOUNCE_IN; /* 状态机切到“插入去抖”阶段。 */
  }

  if (context->stage == HANDLESCAN_STAGE_DEBOUNCE_IN) {
    if (context->debounce.in_debounce_ticks <
        HANDLESCAN_INSERT_DEBOUNCE_TICKS) {
      ++context->debounce
            .in_debounce_ticks; /* 插入信号仍稳定时，累计插入去抖计数。 */
      return true;              /* 未到阈值前，不进入认证阶段。 */
    }

    context->debounce.in_debounce_ticks = 0U; /* 插入去抖达标后，清掉计数器。 */
    context->verify_start_wait_ticks =
        0U; /* 开始认证前等待前，先把等待计数清零。 */
    context->stage =
        HANDLESCAN_STAGE_WAIT_VERIFY; /* 状态机切到“认证前等待”阶段。 */
    Handlescan_DebugTrace(
        binding->channel, HANDLESCAN_DBG_STEP_INSERT_PASS,
        HANDLESCAN_INSERT_DEBOUNCE_TICKS); /* 输出“插入稳定”报文。 */
    return true; /* 本轮插入确认完成，等待下一轮进入认证。 */
  }

  if (context->stage == HANDLESCAN_STAGE_WAIT_VERIFY) {
    if (context->verify_start_wait_ticks <
        HANDLESCAN_VERIFY_START_DELAY_TICKS) {
      ++context
            ->verify_start_wait_ticks; /* 每个扫描周期把认证前等待计数加一。 */
      return true;                     /* 等待未满 200ms 前，不访问 EEPROM。 */
    }

    context->verify_start_wait_ticks = 0U; /* 等待完成后，把等待计数器清零。 */
    context->stage = HANDLESCAN_STAGE_VERIFY; /* 状态机切到“认证执行”阶段。 */
  }

  if (context->stage == HANDLESCAN_STAGE_RETRY_WAIT) {
    if (context->verify_retry_wait_ticks <
        HANDLESCAN_VERIFY_RETRY_DELAY_TICKS) {
      ++context
            ->verify_retry_wait_ticks; /* 每 10ms 累计一次快速重试等待时间。 */
      return true;                     /* 未到重试间隔前，不访问 EEPROM。 */
    }

    context->verify_retry_wait_ticks = 0U; /* 快速重试等待结束，清零计数。 */
    context->stage =
        HANDLESCAN_STAGE_VERIFY; /* 重新进入认证阶段，不需要重新插拔。 */
  }

  if (context->stage == HANDLESCAN_STAGE_VERIFY) {
    if (binding->channel != s_verify_channel) {
      return true; /* 本轮认证总线留给另一侧，避免双路I2C异常等待叠加；不改变插拔去抖计数。 */
    }
    AT24CS32_ClearLastDebugInfo(); /* 认证前先清掉上一轮底层 I2C 调试信息。 */
    verify_status = Handlescan_VerifyEeprom(
        binding, &verify_result); /* 按逻辑绑定和统一物理映射选择对应I2C总线完成EEPROM认证。 */
    if (verify_status == AT24CS32_CRC_STATUS_PENDING) {
      return true; /* 本次已读一页或序列号，返回等待下轮继续；不增加失败次数，也不初始化手柄参数。 */
    }
    Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_VERIFY_STATUS,
                          (uint8_t)verify_status); /* 输出当前认证结果码。 */

    if (verify_status != AT24CS32_CRC_STATUS_OK) { /* EEPROM 认证未通过时进入限次重试，不能继续读取业务信息页。 */
      Handlescan_EnterRetryOrFail(
          binding->channel, context,
          Handlescan_MapVerifyStatusToAlarm(
              binding->channel,
              verify_status)); /* 先快速重试，最终失败才按绑定通道报警。 */
      Handlescan_DebugTraceI2cDetail(
          binding->channel); /* 输出最近一次底层 I2C 访问细节。 */
      return true;           /* 本轮不再继续读信息区。 */
    }

    context->stage =
        HANDLESCAN_STAGE_READ_INFO; /* 认证通过后，进入信息区读取阶段。 */
    return true; /* 本轮认证结束即返回，下一轮再初始化手柄参数，缩短本轮扫描持锁时间。 */
  }

  return false; /* 返回false表示本函数未处理当前阶段，由调用方继续读取手柄参数或处理其它状态。 */
}

/*
 * 函数功能：Page2确认RFID基座后按EEPROM上线并解除旧校验报警；空闲时首读刀具，运行中保留原RFID暂停规则。
 * 输入参数：binding
 * 为固定通道资源；mapped_model 和 raw_type_* 为已确认的基座型号。
 *
 * 返回参数：始终返回 true，表示 RFID 基座分支已处理，本轮不得再解析普通 Page3
 * 刀具。
 */
static bool Handlescan_ProcessRfidBase(const HandlescanChannelBinding *binding,
                                       uint8_t mapped_model,
                                       uint8_t raw_type_major,
                                       uint8_t raw_type_minor) {
  HandlescanChannelContext *context =
      binding->context; /* 保存本通道 RFID 等待、重试和在线状态。 */
  ChannelrecognizeMessage_t *message =
      binding->message; /* 空基座或刀具上线结果只写本通道识别缓存。 */
  uint32_t *spec_values =
      binding->spec_values; /* 等待刀具头期间清本通道旧规格。 */

  if ((WorkMessage.runflag_work != false) &&
      ((WorkMessage.channel_work == CHANNEL_A) ||
       (WorkMessage.channel_work == CHANNEL_B)) &&
      (binding->channel != WorkMessage.channel_work)) {
    Handlescan_ClearToolSpecValues(
        spec_values); /* 另一通道运行时只按 EEPROM 发布基座身份，先清掉本通道旧 RFID 刀具规格。 */
    Rfid_ClearChannelResult(
        binding->channel); /* 作废本通道旧标签和排队事务，运行期间不发起新的 RFID 读取。 */
    Handlescan_PrepareRfidBase(
        message, mapped_model, raw_type_major,
        raw_type_minor); /* 使用 Page2 已确认的公共接头/PX 型号建立本通道基座上线缓存。 */
    if ((mapped_model == PXBA_ONLINES) || (mapped_model == PXBB_ONLINES)) {
      (void)Handlescan_LoadSplitBaseEepromRuntime(
          binding); /* PXBA/PXBB 继续从自身 EEPROM 装载 Page3/Page4，行为与运行中插入 TM 一致。 */
    }
    else {
      Handlescan_LoadRfidBaseControlProfile(binding); /* 公共接头同样读取EEPROM方向许可，保留RFID刀具参数来源。 */
    }
    Handlescan_LoadPage6SpeedStep(
        binding); /* 从新插入手柄 EEPROM 装载本通道调速步进，不修改正在运行通道的 WorkMessage。 */
    context->offline_event_required = 1U; /* 运行期间识别到另一通道基座后已经请求显示在线，后续拔出不得静默回到空闲态。 */
    (void)SendKeyBehMessage(
        PLUGunPLUG,
        binding->plug_key); /* 发布本通道上线事件；运行门禁只保存 MemoryMsg 并刷新未选中图标，不切换当前通道。 */
    Handlescan_ClearChannelAlarm(
        binding->channel,
        context->last_alarm); /* EEPROM 身份已经确认成功，释放本通道此前遗留的手柄校验报警。 */
    Handlescan_ResetVerifyRetry(
        context); /* 基座已按 EEPROM 正常上线，清掉本通道认证和 RFID 快速重试计数。 */
    context->last_alarm = 0U; /* 清掉本通道历史报警，避免在线保持阶段继续携带旧失败状态。 */
    context->rfid_wait_ticks = 0U; /* 本次没有启动 RFID 等待，等待计数必须保持为零。 */
    context->rfid_monitor_ticks = 0U; /* 运行期间暂停在线 RFID 监测，停机后由原周期重新计时。 */
    context->rfid_last_sequence = 0U; /* 当前只识别出EEPROM基座，还没有有效的RFID刀具参数。 */
    context->rfid_last_presence_sequence = 0U; /* 清空标签存在序号，后续选中该通道时重新确认刀具。 */
    context->rfid_miss_count = 0U; /* 未执行射频事务不能累计刀具缺失次数。 */
    context->rfid_monitor_pending = 0U; /* 运行期间没有监测事务，禁止保留虚假的等待状态。 */
    context->rfid_monitor_ticket = 0U; /* 没有 RFID 请求时票据必须清零，避免停机后查询旧事务。 */
    context->rfid_tool_online = 0U; /* 只把手柄基座置为在线，刀具头仍保持未识别状态。 */
    context->rfid_wait_started = 0U; /* 本轮没有进入 RFID 上线等待，后续正常选中时仍可启动原识别流程。 */
    context->stage =
        HANDLESCAN_STAGE_ONLINE; /* 进入在线保持态，防止运行期间重复读 EEPROM 或重复发送上线图标。 */
    Handlescan_BeepOnceIfNoAlarm(); /* EEPROM 手柄身份上线成功后单响，提示语义与 TM 插入一致。 */
    return true; /* 本轮已完成基座上线，运行期间不继续访问 RFID 硬件。 */
  }

  context->rfid_wait_started = 1U; /* 标记本次首读等待，实际事务结束后直接转在线检测，不再重读EEPROM等待刀具。 */
  Handlescan_ClearToolSpecValues(
      spec_values); /* 可拆刀具头等待 RFID
                       前先清屏幕规格缓存，避免显示旧刀具头。 */
  if (Handlescan_StartRfidWait(binding, true) !=
      false) /* 基座刚上线时快速读取 RFID 标签。 */
  {
    Handlescan_PrepareRfidBase(
        message, mapped_model, raw_type_major,
        raw_type_minor); /* 只写基座字段，刀具字段等待 RFID 真实结果。 */
    if ((mapped_model == PXBA_ONLINES) || (mapped_model == PXBB_ONLINES)) {
      (void)Handlescan_LoadSplitBaseEepromRuntime(
          binding); /* 分体式进入 RFID 等待时先恢复本通道 EEPROM 参数。 */
    }
    else {
      Handlescan_LoadRfidBaseControlProfile(binding); /* 公共接头首读RFID前先取得自身方向许可，后续刀具只能进一步限制。 */
    }
    Handlescan_LoadPage6SpeedStep(
        binding); /* 可拆式基座先装入本通道 Page6 步进。 */
    Handlescan_ClearChannelAlarm(binding->channel, context->last_alarm); /* EEPROM身份已经校验并完成基座参数装载，无刀具不能继续保持旧校验异常。 */
    Handlescan_ResetVerifyRetry(context); /* 本通道IIC已恢复，后续RFID未读到刀具不计为EEPROM认证失败。 */
    context->last_alarm = 0U; /* 在发布基座上线前释放旧报警归属，另一侧有效故障仍由清除入口保留。 */
    context->offline_event_required = 1U; /* 先记录以后拔出时必须发送离线消息；即使仍在等待RFID结果，拔出也不能漏报。 */
    (void)SendKeyBehMessage(
        PLUGunPLUG,
        binding->plug_key); /* 先上报 RFID
                               基座在线，刀具头数据到达后再二次刷新通道记忆。 */
    Handlescan_BeepOnceIfNoAlarm(); /* 只提示本次基座上线，后续找刀具由在线监测处理，不再反复上线蜂鸣。 */
  }
  return true; /* RFID 结果回来前先保持基座在线，刀具区由上位机显示等待 RFID。
                */
}

/*
 * 函数功能：读取一体式手柄的 Page3/Page4/Page6 参数；PXYTM/PXYTP 取得完整刀具规格后再发布本通道上线事件。
 * 输入参数：binding 为固定通道资源；handle_type_cfg 和型号参数来自已通过认证的 Page2。
 * 返回参数：始终返回 true，表示一体式手柄本轮已完成上线处理或已进入重试。
 */
static bool Handlescan_ProcessIntegratedHandle(
    const HandlescanChannelBinding *binding,
    const HandlescanHandleTypeConfig *handle_type_cfg, uint8_t mapped_model,
    uint8_t raw_type_major, uint8_t raw_type_minor) {
  HandlescanChannelContext *context =
      binding->context; /* 保存本通道 EEPROM 页缓存和扫描阶段。 */
  ChannelrecognizeMessage_t *message =
      binding->message; /* 一体式手柄参数只写本通道识别缓存。 */
  ChannelMemoryMessage_t *memory =
      binding->memory; /* 清本通道旧 RFID 自动识别记忆。 */
  uint32_t *spec_values =
      binding->spec_values;     /* 一体式刀具规格只刷新本通道。 */
  uint8_t read_status;          /* 保存 Page3 或 Page4 读取结果。 */
  bool integrated_spec_handle =
      Handlescan_IsIntegratedSpecHandle(mapped_model); /* PXYTM/PXYTP 必须取得 Page3 规格，确保上线后立即具备屏幕显示数据。 */
  uint16_t tool_diameter_tenth; /* Page3 刀具直径，单位 0.1。 */
  uint16_t tool_length_tenth;   /* Page3 刀具长度，单位 0.1。 */
  uint16_t tool_angle_tenth;    /* Page3 刀具角度，单位 0.1。 */

  AT24CS32_ClearLastDebugInfo(); /* 一体式手柄不再用 Page3
                                    识别刀具类型，但仍要读取 Page4 运行初始值。
                                  */
  read_status = Handlescan_ReadEepromPage(
      binding, HANDLESCAN_INITIAL_INFO_PAGE_INDEX,
      context->initial_info_buf); /* 从本通道读取 Page4
                                     默认速度、频率、方向和泵流量。 */
  if (read_status == 0U) {
    Handlescan_DebugTrace(
        binding->channel, HANDLESCAN_DBG_STEP_INFO_FAIL,
        read_status); /* 输出本通道 Page4 读取或页校验失败报文。 */
    Handlescan_DebugTraceI2cDetail(
        binding->channel); /* 输出本轮 Page4 失败对应的底层 I2C 细节。 */
    Handlescan_EnterRetryOrFail(
        binding->channel, context,
        0U);     /* Page4 瞬时读取失败只重试，不立刻锁死一体式手柄。 */
    return true; /* 本轮停止后续处理，等待下一次重新读取 Page4。 */
  }

  Handlescan_UpdateSelfTypedResult(
      message, mapped_model, raw_type_major,
      raw_type_minor); /* 自带刀具能力型号直接写入本通道手柄和刀具能力字段，DHYTM同样走该安全路径。 */
  read_status = Handlescan_ReadEepromBytes(
      binding, HANDLESCAN_TOOL_INFO_ADDR, context->tool_info_buf,
      HANDLESCAN_TOOL_INFO_SIZE); /* 一体式手柄仍读取本通道 Page3
                                     规格和倍率字段。 */
  if ((read_status == 0U) && (integrated_spec_handle != false)) {
    Handlescan_ClearToolSpecValues(
        spec_values); /* PXYTM/PXYTP 本轮没有读到完整规格时清掉旧缓存，禁止旧刀具参数误上屏。 */
    Handlescan_DebugTrace(
        binding->channel, HANDLESCAN_DBG_STEP_INFO_FAIL,
        read_status); /* 输出本通道 PXYTM/PXYTP Page3 规格读取失败报文。 */
    Handlescan_DebugTraceI2cDetail(
        binding->channel); /* 输出本轮 Page3 失败对应的底层 I2C 细节，便于区分线路和页数据问题。 */
    Handlescan_EnterRetryOrFail(
        binding->channel, context,
        0U); /* PXYTM/PXYTP 的规格属于上线必需数据，沿用普通 EEPROM 手柄重试机制重新读取。 */
    return true; /* 规格读取成功前不发布上线事件，避免手柄在线后规格窗口永久为空。 */
  }
  Handlescan_UpdateToolRatioMessage(
      message, (read_status != 0U)
                   ? context->tool_info_buf
                   : NULL); /* Page3 倍率无效时按 1
                               倍保护，避免机械变速字段写错导致超速。 */
  Handlescan_UpdateInitialInfoMessage(
      message,
      context
          ->initial_info_buf); /* 同步本通道 Page4
                                  默认速度、频率、方向、注水流量和蜂鸣阈值。 */
  Handlescan_LoadPage6SpeedStep(
      binding); /* 一体式手柄读取本通道 Page6 调速步进。 */
  if ((read_status != 0U) && (integrated_spec_handle != false)) {
    tool_diameter_tenth = Handlescan_ReadUint16BE(
        context->tool_info_buf,
        HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 本通道一体式手柄 Page3[2..3]
                                             保存刀具直径，单位沿用 0.1。 */
    tool_length_tenth = Handlescan_ReadUint16BE(
        context->tool_info_buf,
        HANDLESCAN_TOOL_LENGTH_OFFSET); /* 本通道一体式手柄 Page3[4..5]
                                           保存刀具长度，按现有规格窗口格式缓存。
                                         */
    tool_angle_tenth = Handlescan_ReadUint16BE(
        context->tool_info_buf,
        HANDLESCAN_TOOL_ANGLE_OFFSET); /* 本通道一体式手柄 Page3[6..7]
                                          保存刀具角度，屏幕规格区直接显示角度值。
                                        */
    Handlescan_UpdateRecognizeMessage(
        message, mapped_model, raw_type_major, raw_type_minor, mapped_model,
        tool_diameter_tenth, tool_length_tenth,
        tool_angle_tenth); /* PXYTM/PXYTP 将长、径、角同步进本通道识别缓存，使识别结果和规格窗口使用同一份 EEPROM 数据。 */
    Handlescan_UpdateToolSpecValues(
        spec_values, tool_diameter_tenth, tool_length_tenth, tool_angle_tenth,
        Handlescan_MapToolType(
            mapped_model)); /* 一体式手柄规格来自自身 EEPROM
                               Page3，不再清掉直径、长度、角度显示缓存。 */
  } else {
    Handlescan_ClearToolSpecValues(
        spec_values); /* 不需要显示规格的其它一体式手柄清空本通道缓存，避免继续显示上一支手柄的数据。 */
  }
  memory->auto_identify =
      0U; /* 一体式手柄不使用 RFID 自动识别，先清通道记忆中的旧自动识别模式。 */
  context->offline_event_required = 1U; /* 一体式手柄准备发布上线事件，后续拔出必须成功投递一次离线事件才能清锁存。 */
  (void)SendKeyBehMessage(
      PLUGunPLUG, binding->plug_key); /* 通知插拔事件链：A
                                         通道按手柄型号上线并装载到通道记忆。 */
  Handlescan_ClearChannelAlarm(
      binding->channel,
      context->last_alarm); /* 自恢复成功时清掉本通道历史手柄校验报警。 */
  Handlescan_ResetVerifyRetry(context); /* 上线成功后清空本通道失败重试状态。 */
  context->last_alarm = 0U; /* 上线成功后清掉本通道最近一次报警缓存。 */
  context->stage =
      HANDLESCAN_STAGE_ONLINE; /* 本通道切到在线保持态，后续只做拔出和 RFID
                                  在线监测判断。 */
  Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_ONLINE,
                        mapped_model); /* 输出本通道上线报文，值为手柄型号。 */
  Handlescan_DebugTraceHandleName(
      binding->channel, raw_type_major, raw_type_minor,
      handle_type_cfg->handle_name); /* 输出本通道 0x7C 手柄名称报文。 */
  Handlescan_BeepOnceIfNoAlarm();    /* A
                                        通道认证并上线成功后，给使用者一个确认单响。
                                      */
  return true; /* 自带刀具能力手柄本轮流程结束，PXYTM/PXYTP 已同步完成 Page3 刀具规格。 */
}

/*
 * 函数功能：读取普通 EEPROM 手柄的
 * Page3/Page4/Page6，并发布本通道上线事件。
 * 输入参数：binding
 * 为固定通道资源；handle_type_cfg 和型号参数来自已通过认证的 Page2。
 *
 * 返回参数：始终返回 true，表示普通 EEPROM 分支已完成上线或已进入重试。

 */
static bool Handlescan_ProcessEepromHandle(
    const HandlescanChannelBinding *binding,
    const HandlescanHandleTypeConfig *handle_type_cfg, uint8_t mapped_model,
    uint8_t raw_type_major, uint8_t raw_type_minor) {
  HandlescanChannelContext *context =
      binding->context; /* 保存本通道 EEPROM 页缓存和扫描阶段。 */
  ChannelrecognizeMessage_t *message =
      binding->message; /* 普通手柄参数只写本通道识别缓存。 */
  ChannelMemoryMessage_t *memory =
      binding->memory; /* 保留本通道原有 auto_identify 更新顺序。 */
  uint32_t *spec_values = binding->spec_values; /* 普通刀具规格只刷新本通道。 */
  uint8_t read_status;          /* 保存 Page3 或 Page4 读取结果。 */
  uint8_t raw_tool_major;       /* Page3 刀具主类型原始字节。 */
  uint8_t raw_tool_minor;       /* Page3 刀具子类型原始字节。 */
  uint8_t mapped_tool_model;    /* 配置表映射后的内部刀具型号。 */
  uint16_t tool_diameter_tenth; /* Page3 刀具直径，单位 0.1。 */
  uint16_t tool_length_tenth;   /* Page3 刀具长度，单位 0.1。 */
  uint16_t tool_angle_tenth;    /* Page3 刀具角度，单位 0.1。 */
  const HandlescanHandleTypeConfig
      *tool_type_cfg; /* 命中的刀具类型配置表项。 */

  AT24CS32_ClearLastDebugInfo(); /* 读取刀具页之前，先把调试缓存切到当前这一次访问。
                                  */
  read_status = Handlescan_ReadEepromBytes(
      binding, HANDLESCAN_TOOL_INFO_ADDR, context->tool_info_buf,
      HANDLESCAN_TOOL_INFO_SIZE); /* 从本通道 EEPROM 读出第 3 页刀具信息。 */
  if (read_status == 0U) {
    Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_INFO_FAIL,
                          read_status); /* 输出刀具信息区读取失败报文。 */
    Handlescan_DebugTraceI2cDetail(
        binding->channel); /* 输出本轮失败对应的底层 I2C 细节。 */
    Handlescan_EnterRetryOrFail(
        binding->channel, context,
        0U);     /* 刀具页瞬时读取失败只重试，不立刻锁死。 */
    return true; /* 本轮停止后续处理。 */
  }

  AT24CS32_ClearLastDebugInfo(); /* 读取初始值页之前，先把调试缓存切到 Page4
                                    访问。 */
  read_status = Handlescan_ReadEepromPage(
      binding, HANDLESCAN_INITIAL_INFO_PAGE_INDEX,
      context->initial_info_buf); /* 从本通道 EEPROM 读取 Page4
                                     初始值并校验页尾。 */
  if (read_status == 0U) {
    Handlescan_DebugTrace(
        binding->channel, HANDLESCAN_DBG_STEP_INFO_FAIL,
        read_status); /* 输出本通道初始值页读取或页校验失败报文。 */
    Handlescan_DebugTraceI2cDetail(
        binding->channel); /* 输出本轮 Page4 失败对应的底层 I2C 细节。 */
    Handlescan_EnterRetryOrFail(
        binding->channel, context,
        0U);     /* 初始值页瞬时读取失败只重试，不立刻锁死手柄。 */
    return true; /* 本轮停止后续处理，等待下一次重新读取完整业务页。 */
  }

  raw_tool_major =
      context
          ->tool_info_buf[HANDLESCAN_TOOL_MAJOR_OFFSET]; /* 取出刀具信息区第 1
                                                            字节作为刀具主类型。
                                                          */
  raw_tool_minor =
      context
          ->tool_info_buf[HANDLESCAN_TOOL_MINOR_OFFSET]; /* 取出刀具信息区第 2
                                                            字节作为刀具子类型。
                                                          */
  tool_type_cfg = Handlescan_FindToolTypeConfig(
      raw_tool_major, raw_tool_minor); /* 按两个原始字节查找刀具配置表。 */
  if (tool_type_cfg == NULL) {
    // Handlescan_DebugTrace(binding->channel,
    // HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_tool_minor); /*
    // 输出“刀具类型无法识别”报文。 */
    // // Handlescan_EnterRetryOrFail(binding->channel,
    //                              &context->stage,
    //                              &context->verify_retry_count,
    //                              &context->verify_retry_wait_ticks,
    //                              &context->last_alarm,
    //                              0U);                /*
    //                              刀具页读到异常值时允许后续重试。 */
    // return true;                                         /* 等待重新插拔。 */
  }

  tool_diameter_tenth = Handlescan_ReadUint16BE(
      context->tool_info_buf,
      HANDLESCAN_TOOL_DIAMETER_OFFSET); /* 解析刀具直径，单位 0.1。 */
  tool_length_tenth = Handlescan_ReadUint16BE(
      context->tool_info_buf,
      HANDLESCAN_TOOL_LENGTH_OFFSET); /* 解析刀具长度，单位 0.1。 */
  tool_angle_tenth = Handlescan_ReadUint16BE(
      context->tool_info_buf,
      HANDLESCAN_TOOL_ANGLE_OFFSET); /* 解析刀具角度，单位 0.1。 */

  mapped_tool_model =
      tool_type_cfg->mapped_handle_type; /* 取出查表后的系统内部刀具类型值。 */
  Handlescan_UpdateRecognizeMessage(
      message, mapped_model, raw_type_major, raw_type_minor, mapped_tool_model,
      tool_diameter_tenth, tool_length_tenth,
      tool_angle_tenth); /* 同步更新本通道识别结果。 */
  Handlescan_UpdateToolRatioMessage(
      message, context->tool_info_buf); /* 本通道按 Page3[8]/[9]
                                           装入减速/增速比，供驱动统一换算。 */
  WorkMessage.auto_identify = memory->auto_identify = 1;
  Handlescan_UpdateInitialInfoMessage(
      message,
      context
          ->initial_info_buf); /* 同步更新本通道 Page4
                                  默认速度、频率、方向、注水流量和蜂鸣阈值。 */
  Handlescan_LoadPage6SpeedStep(
      binding); /* 普通 EEPROM 手柄上线后读取本通道 Page6。 */
  Handlescan_UpdateToolSpecValues(
      spec_values, tool_diameter_tenth, tool_length_tenth, tool_angle_tenth,
      Handlescan_MapToolType(
          mapped_tool_model)); /* 规格缓存保存业务刀具类型，PXM/PXP
                                  不再直接暴露旧码给 UI。 */
  if (binding->channel == CHANNEL_A) {
    WorkMessage.auto_identify =
        0U; /* A 通道保留原顺序：先清工作态自动识别，再发送上线事件。 */
  }
  context->offline_event_required = 1U; /* 普通 EEPROM 手柄准备发布上线事件，后续物理拔出必须与该在线状态配对。 */
  (void)SendKeyBehMessage(
      PLUGunPLUG,
      binding->plug_key); /* 沿用原插入事件链装载通道记忆和工作态。 */
  if (binding->channel == CHANNEL_B) {
    WorkMessage.auto_identify =
        0U; /* B 通道保留原顺序：上线事件处理完成后再清工作态自动识别。 */
  } /* 通知插拔事件链：本通道上线，实际 WorkMessage/MemoryMsg 装载在
       PlugORunPLUGActive 中完成。 */
  Handlescan_ClearChannelAlarm(binding->channel, context->last_alarm); /* 如果之前已经进入最终失败报警，后续自恢复成功时清掉本通道报警。
                                                                        */
  Handlescan_ResetVerifyRetry(context); /* 上线成功后清空失败重试状态。 */
  context->last_alarm = 0U;             /* 上线成功后清掉最近一次报警缓存。 */
  context->stage = HANDLESCAN_STAGE_ONLINE; /* 状态机切到在线保持态。 */
  Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_ONLINE,
                        mapped_model); /* 输出本通道上线报文。 */
  Handlescan_DebugTraceHandleName(
      binding->channel, raw_type_major, raw_type_minor,
      handle_type_cfg->handle_name); /* 输出本通道手柄名称报文。 */
  Handlescan_DebugTraceToolInfo(
      binding->channel, raw_tool_major, raw_tool_minor,
      tool_type_cfg->handle_name, tool_diameter_tenth, tool_length_tenth,
      tool_angle_tenth);          /* 输出本通道刀具名称和规格报文。 */
  Handlescan_BeepOnceIfNoAlarm(); /* A
                                     通道认证并上线成功后，给使用者一个确认单响。
                                   */
  return true;                    /* 本通道本轮流程结束，后续等待拔出。 */
}

/*
 * 函数功能：读取 Page2 手柄型号，再按型号读取 RFID 刀具、一体式手柄参数或普通 EEPROM 刀具参数。
 * 输入参数：binding 指定 A/B 通道的 EEPROM、识别数据和扫描进度。
 * 返回参数：true 表示本轮信息读取已处理；false 表示当前不在读取信息阶段。
 */
static bool
Handlescan_ProcessReadInfo(const HandlescanChannelBinding *binding) {
  HandlescanChannelContext *context =
      binding->context;   /* Page2 缓存和阶段只使用本通道上下文。 */
  uint8_t read_status;    /* 保存 Page2 读取结果。 */
  uint8_t raw_type_major; /* Page2 手柄主类型原始字节。 */
  uint8_t raw_type_minor; /* Page2 手柄子类型原始字节。 */
  uint8_t mapped_model;   /* 配置表映射后的内部手柄型号。 */
  const HandlescanHandleTypeConfig
      *handle_type_cfg; /* 命中的手柄类型配置表项。 */

  if (context->stage != HANDLESCAN_STAGE_READ_INFO) {
    return false; /* 其它阶段不读取 EEPROM，交给对应阶段处理函数。 */
  }

  AT24CS32_ClearLastDebugInfo(); /* 读取 Page2
                                    前清底层调试记录，失败时可定位本轮总线访问。
                                  */
  read_status = Handlescan_ReadEepromBytes(
      binding, HANDLESCAN_INFO_ADDR, context->info_buf,
      HANDLESCAN_INFO_SIZE); /* 从绑定通道读取 Page2 身份。 */
  if (read_status == 0U) {
    Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_INFO_FAIL,
                          read_status); /* 输出本通道身份读取失败。 */
    Handlescan_DebugTraceI2cDetail(
        binding->channel); /* 输出底层 I2C 失败细节。 */
    Handlescan_EnterRetryOrFail(binding->channel, context,
                                0U); /* 瞬时失败进入原快速重试，不立即报警。 */
    return true;                     /* 本轮停止，等待下一次认证重试。 */
  }

  raw_type_major =
      context
          ->info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 取 Page2 手柄主类型。 */
  raw_type_minor =
      context
          ->info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 取 Page2 手柄子类型。 */
  handle_type_cfg = Handlescan_FindHandleTypeConfig(
      raw_type_major, raw_type_minor); /* 按原始双字节查型号表。 */
  if ((handle_type_cfg == NULL) ||
      !Handlescan_IsModelSupported(handle_type_cfg->mapped_handle_type)) {
    Handlescan_DebugTrace(binding->channel, HANDLESCAN_DBG_STEP_MODEL_INVALID,
                          raw_type_minor); /* 输出无法识别的型号字节。 */
    Handlescan_RejectUnsupportedHandle(binding); /* 认证通过后按功能白名单拒绝，未知型号也必须提示并保持到拔出。 */
    return true;                     /* 型号未确认时不读取后续业务页。 */
  }

  mapped_model =
      handle_type_cfg->mapped_handle_type; /* 取得系统内部手柄型号。 */
  context->tool_source = Handlescan_GetToolSource(
      mapped_model); /* 型号决定刀具来自 RFID 还是 EEPROM。 */
  if (context->tool_source != HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3) {
    return Handlescan_ProcessRfidBase(
        binding, mapped_model, raw_type_major,
        raw_type_minor); /* RFID 基座不得落入普通 Page3 刀具解析。 */
  }
  if (Handlescan_IsSelfTypedHandleModel(mapped_model) != false) {
    return Handlescan_ProcessIntegratedHandle(
        binding, handle_type_cfg, mapped_model, raw_type_major,
        raw_type_minor); /* 一体式手柄由自身型号直接给出刀具能力。 */
  }

  return Handlescan_ProcessEepromHandle(
      binding, handle_type_cfg, mapped_model, raw_type_major,
      raw_type_minor); /* 其余型号按原 Page3/Page4 流程上线。 */
}

/*
 * 函数功能：等待首次 RFID 结果，定期重试校验失败的手柄，或检查在线刀具是否仍能读到。
 * 输入参数：binding 指定 A/B 通道的型号、RFID 状态和扫描进度。
 * 返回参数：true 表示本轮已处理对应阶段；false 表示当前阶段不由本函数处理。

 */
static bool
Handlescan_ProcessRfidStage(const HandlescanChannelBinding *binding) {
  HandlescanChannelContext *context =
      binding->context;   /* RFID 序号和缺失计数只使用本通道上下文。 */
  uint8_t raw_type_major; /* 复用已缓存的 Page2 手柄主类型。 */
  uint8_t raw_type_minor; /* 复用已缓存的 Page2 手柄子类型。 */
  uint8_t mapped_model;   /* 当前 RFID 基座映射型号。 */
  const HandlescanHandleTypeConfig
      *handle_type_cfg; /* 当前 RFID 基座配置表项。 */

  if (context->stage == HANDLESCAN_STAGE_WAIT_RFID_TOOL) {
    raw_type_major =
        context->info_buf
            [HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 等待 RFID 期间复用已读到的
                                                EEPROM 第二页主类型。 */
    raw_type_minor =
        context->info_buf
            [HANDLESCAN_MODEL_MINOR_OFFSET]; /* 等待 RFID 期间复用已读到的
                                                EEPROM 第二页子类型。 */
    handle_type_cfg = Handlescan_FindHandleTypeConfig(
        raw_type_major,
        raw_type_minor); /* 重新查表，避免局部变量跨周期丢失。 */
    if (handle_type_cfg == NULL) {
      Handlescan_EnterRetryOrFail(
          binding->channel, context,
          0U);     /* 等待期间基座信息异常时回到原有重试流程。 */
      return true; /* 本轮等待处理结束。 */
    }
    mapped_model =
        handle_type_cfg->mapped_handle_type; /* 取出本通道基座型号。 */
    context->tool_source = Handlescan_GetToolSource(
        mapped_model); /* 重新确认本通道 RFID 来源。 */
    if (Handlescan_ProcessRfidWait(binding, mapped_model, raw_type_major,
                                   raw_type_minor) != false) {
      return true; /* RFID 等待阶段本轮已经处理。 */
    }
  }

  if (context->stage == HANDLESCAN_STAGE_VERIFY_FAIL) {
    if (context->verify_retry_wait_ticks <
        HANDLESCAN_VERIFY_ALARM_RETRY_TICKS) {
      ++context->verify_retry_wait_ticks; /* 最终报警保持时慢速计时，避免反复高频读
                                             EEPROM。 */
      return true;                        /* 等待 1000ms 后再尝试自恢复认证。 */
    }

    context->verify_retry_wait_ticks =
        0U; /* 慢速自恢复时间到，清掉等待计数。 */
    context->stage = HANDLESCAN_STAGE_VERIFY; /* 仍插着时重新认证一次，正确手柄可自动恢复上线。
                                               */
    return true;
  }

  if (context->stage == HANDLESCAN_STAGE_ONLINE) {
    raw_type_major =
        context
            ->info_buf[HANDLESCAN_MODEL_MAJOR_OFFSET]; /* 在线监测时复用 EEPROM
                                                          第二页基座主类型。 */
    raw_type_minor =
        context
            ->info_buf[HANDLESCAN_MODEL_MINOR_OFFSET]; /* 在线监测时复用 EEPROM
                                                          第二页基座子类型。 */
    handle_type_cfg = Handlescan_FindHandleTypeConfig(
        raw_type_major,
        raw_type_minor); /* 在线阶段重新查表，避免局部变量跨周期丢失。 */
    if (handle_type_cfg != NULL) {
      mapped_model =
          handle_type_cfg->mapped_handle_type; /* 取出本通道当前基座型号。 */
      context->tool_source = Handlescan_GetToolSource(
          mapped_model); /* 判断本通道是否需要 RFID 低频监测。 */
      Handlescan_RequestOnlineRfidMonitor(
          binding); /* 空闲时周期请求本通道 RFID，运行中自动暂停。 */
      Handlescan_ProcessOnlineRfidResult(
          binding, mapped_model, raw_type_major,
          raw_type_minor); /* 处理本通道新收到的RFID结果，并更新该通道保存的手柄参数。 */
    }
    return true; /* 本轮在线检查完成；后续扫描继续检查拔出或新的 RFID 结果。 */
  }

  return false; /* 其它阶段不在这里处理。 */
}

/*
 * 函数功能：每轮依次检查本通道的拔出、识别等待、EEPROM 信息和 RFID 结果；前一步需要等待时，本轮就结束。
 * 输入参数：binding 指定 A/B 通道的检测脚、数据和扫描进度。
 * 返回参数：无。
 */
static void Handlescan_RunChannel(const HandlescanChannelBinding *binding) {
  uint8_t is_inserted; /* 当前短接采样，低电平换算为 1 表示已插入。 */

  is_inserted =
      (uint8_t)(Bsp_GpioRead(binding->short_gpio, binding->short_pin) ==
                GPIO_PIN_RESET); /* 只读取绑定通道的短接脚。 */
  if (Handlescan_ProcessDisconnect(binding, is_inserted) != false) {
    return; /* 拔出候选、去抖或离线清理已经处理，本轮不再认证。 */
  }

  if (binding->context->unsupported_latched != 0U)
  {
    if ((binding->context->offline_event_required != 0U) &&
        SendKeyBehMessage(PLUGunPLUG, binding->unplug_key))
    {
      binding->context->offline_event_required = 0U; /* 快速换柄未经过完整拔出时，也必须可靠撤销上一只手柄的在线图标和通道记忆。 */
    }
    return; /* 不支持的手柄只允许走上方拔出处理，禁止定时重试、RFID或外控导航把它重新上线。 */
  }

  if (Handlescan_ShouldDeferRecognition(binding->channel) != 0U) {
    return; /* 普通阻塞报警未解除前保持当前阶段，不访问 EEPROM。 */
  }

  if (Handlescan_ProcessInsertAndVerify(binding) != false) {
    return; /* 插入消抖、等待、失败或重试要求本轮停止。 */
  }

  if (Handlescan_ProcessReadInfo(binding) != false) {
    return; /* 信息读取阶段已经完成上线、等待分流或失败处理。 */
  }

  (void)Handlescan_ProcessRfidStage(
      binding); /* RFID 等待、失败恢复和在线监测在末段串行执行。 */
}

/*
 * 函数功能：执行 A 通道一次 10ms 手柄扫描，保留该入口便于现场单独下断点。
 *
 * 输入参数：无。
 * 返回参数：无。
 */
void HandlescanA_Fun_SSC(void)
{
    Handlescan_RunChannel(&s_a_binding);
}

/*
 * 函数功能：执行 B 通道一次 10ms 手柄扫描，保留该入口便于现场单独下断点。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandlescanB_Fun_SSC(void)
{
    Handlescan_RunChannel(&s_b_binding);
}


/*
 * 函数功能：每10ms检查A/B插拔，每轮只给一侧认证页读取机会，最后处理临时报警到期关闭。
 * 输入参数：无。
 * 返回参数：无。
 */
void Handlescan_Fun(void)
{
    HandlescanA_Fun_SSC();
    HandlescanB_Fun_SSC();
    s_verify_channel = (s_verify_channel == CHANNEL_A) ? CHANNEL_B : CHANNEL_A; /* 下一轮换另一侧读取认证数据，避免同一轮连续等待A、B两侧I2C。 */
    Handlescan_UpdateTransientScreenAlarm();                 /* 维护运行中另一路坏手柄的 3 秒屏幕临时提示。 */
}

/*
 * 函数功能：每次任务调度时执行一轮 A/B 手柄扫描。
 * 输入参数：event 为调度事件，当前不使用。
 * 返回参数：无。
 */
void HANDLESCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN HANDLESCANTaskFunc */
  /* Infinite loop */
  Handlescan_Fun();
  /* USER CODE END HANDLESCANTaskFunc */
}

/*
 * 函数功能：创建手柄扫描任务，每 10ms 执行一次。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandlescanTaskInit(void)
{
  /* definition and creation of HANDLESCANTask */
	Kernel_TaskCreate(&HANDLESCANTaskHandle, HANDLESCANTaskFunc);
	Kernel_TaskStart(&HANDLESCANTaskHandle, KERNEL_TASK_ALWAYS, 10); /* 实际扫描间隔固定为 10ms；调整时须同步上面的等待次数换算宏。 */
}

#if (MAINCTRL_DIAG_ENABLE == 1U)
/*
 * 函数功能：复制指定手柄的认证和插拔记录，并读一次插入电平；不重新读取 EEPROM 或发起 RFID 查询。
 * 输入参数：channel=1 为 A 手柄、2 为 B 手柄；out 指向 DiagnosticHandlescanState_t 输出变量；调用时须持有公共业务互斥锁。
 * 返回参数：无；指针为空、通道无效或找不到通道配置时直接返回，out 保持原值。
 */
void Handlescan_CopyDiagnosticState(uint8_t channel, DiagnosticHandlescanState_t *out)
{
    if ((out == NULL) || (channel < 1U) || (channel > 2U)) return; /* 指针为空或通道不是 A/B 时不读取手柄记录，输出内容保持原值。 */
    const HandlescanChannelBinding *binding = Handlescan_GetBinding(channel); /* 查找指定逻辑通道使用的引脚和认证数据，只查询配置，不开始认证。 */
    const HandlescanChannelContext *context; /* 指向本通道已有的扫描进度，下面只读不改。 */
    if (binding == NULL) return; /* 没有对应通道配置时返回，保留调用方给出的输出内容。 */
    context = binding->context; /* 选定本通道保存的扫描数据，A/B 认证进度不能混用。 */
    out->stage = context->stage; /* 手柄识别步骤：0 等插入，1 确认插入，2 等接口稳定，3 认证，4 读参数，5 等 RFID，6 在线，7 确认拔出，8 等重试，9 认证失败。 */
    out->insert_ticks = context->debounce.in_debounce_ticks; /* 连续检测到手柄插入的次数，每轮 10ms；检测到拔出时清零。 */
    out->remove_ticks = context->debounce.out_debounce_ticks; /* 连续检测到手柄拔出的次数，每轮 10ms；恢复插入电平时清零。 */
    out->verify_step = context->verify_context.step; /* EEPROM 校验步骤：0 读第 1 页，1 读芯片序列号，2～8 读对应页。 */
    out->verified = context->verify_context.verified; /* 全部 EEPROM 认证通过后为 1；开始新一轮认证时清零，0 表示尚未全部通过。 */
    out->retry_count = context->verify_retry_count; /* 本通道连续认证失败次数；重新初始化认证记录后清零，达到上限后转为慢速重试。 */
    out->retry_ticks = context->verify_retry_wait_ticks; /* 认证失败后已等待的 10ms 周期数；进入新一轮等待或等待结束后清零。 */
    out->alarm = context->last_alarm; /* 本通道最近的手柄认证报警码；0 表示当前未保存认证报警。 */
    out->identity_generation = context->navigation_generation; /* 开始确认手柄拔出时递增的编号；EEPROM 读取前后编号不同，就不能继续使用该次读取的数据。 */
    out->tool_source = context->tool_source; /* 刀具参数从哪里取得：0 手柄 EEPROM 第 3 页，1 刀具 RFID 标签。 */
    out->rfid_ticket = context->rfid_monitor_ticket; /* 当前 RFID 监测请求的编号，用它查询本次读取结果，避免误用另一轮结果。 */
    out->rfid_sequence = context->rfid_last_sequence; /* 已经处理过的 RFID 结果编号，用于避免重复修改刀具参数和重复蜂鸣。 */
    out->rfid_presence_sequence = context->rfid_last_presence_sequence; /* 最近一次确认本通道 RFID 标签仍在的结果编号。 */
    out->rfid_pending = context->rfid_monitor_pending; /* RFID 监测请求被接受后置 1；读取结束或取消等待后清零，0 表示没有等待中的请求。 */
    out->rfid_online = context->rfid_tool_online; /* 1：已识别到 RFID 刀具；0：尚未识别到或已确认刀具离线。 */
    out->rfid_misses = context->rfid_miss_count; /* 连续未读到 RFID 标签的次数，用于判断刀具是否已取下。 */
    out->offline_event_required = context->offline_event_required; /* 请求显示手柄上线后置 1；拔出消息成功入队后清零，防止漏发离线通知。 */
    out->physical_level = HAL_GPIO_ReadPin(binding->short_gpio, binding->short_pin); /* 复制诊断数据时读取的手柄短接引脚：0 低电平表示插入，1 高电平表示拔出。 */
}
#endif
