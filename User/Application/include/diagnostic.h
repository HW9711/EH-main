#ifndef MAINCTRL_DIAGNOSTIC_H
#define MAINCTRL_DIAGNOSTIC_H

#include <stdint.h>

#ifndef MAINCTRL_DIAG_ENABLE
#define MAINCTRL_DIAG_ENABLE 0U /* 诊断编译开关：0 不编译记录和读日志功能；1 记录脚踏、手柄、泵并允许 UART2 读取。改值后须重新编译。 */
#endif
#if (MAINCTRL_DIAG_ENABLE != 0U) && (MAINCTRL_DIAG_ENABLE != 1U)
#error "MAINCTRL_DIAG_ENABLE must be 0 or 1"
#endif

#define DIAG_PROTOCOL_VERSION 1U /* 诊断数据版本为 1；修改字段顺序或命令含义时，主控和上位机必须同时更新。 */
#define DIAG_FUNCTION_CODE 0x0EU /* 外部串口正式帧的诊断功能码；收到它只处理日志，不取得外控权限，也不延长外控在线时间。 */
#define DIAG_RECORD_CAPACITY 1024U /* 最多保留 1024 条、每条 48 字节；满后覆盖最旧记录。调大可保留更多记录，但会增加 RAM 占用。 */
#define DIAG_RECORD_WORDS 9U /* 每条记录最多保存 9 个 32 位整数；串口传输时每个整数的低字节在前。 */

/* 脚踏任务处理结束后复制的 AD、连接和启停判断数据；全部用 32 位字段，按声明顺序写入诊断记录。 */
typedef struct
{
    uint32_t ad_single; /* 单踏板最近一次有效实时帧中的 AD 值，未换算成踩踏百分比。 */
    uint32_t ad_two_stage; /* 双段踏板最近一次有效实时帧中的 AD 值，未换算成踩踏百分比。 */
    uint32_t ad_left; /* 双脚踏左侧最近一次有效实时帧中的 AD 值。 */
    uint32_t ad_right; /* 双脚踏右侧最近一次有效实时帧中的 AD 值。 */
    uint32_t connected; /* 1：脚踏定标通过且上线消息已入队；0：尚未上线或已确认离线。 */
    uint32_t pedal_type; /* 脚踏类型：1 单踏板，2 双段踏板，3 双脚踏。 */
    uint32_t frame_valid; /* 收到 CRC 正确的实时帧后置 1；实时帧超时或脚踏失效时清零，禁止用旧 AD 值启动。 */
    uint32_t silent_ticks; /* 连续未收到有效实时帧的次数，每次为 10ms 解析周期；收到有效实时帧后清零。 */
    uint32_t motor_stop; /* 1：要求脚踏电机保持停止；启动检查通过后清零。该值不是电机已停转的反馈。 */
    uint32_t release_ready; /* 确认脚踏已退出电机运行区后置 1；批准新启动或启动检查失败后清零。 */
    uint32_t active_pedal; /* 获准启动电机的踏板侧：0 无，1 左，2 右；松脚或撤销脚踏启动后清零。 */
    uint32_t follow_pedal; /* 启动手柄联动注水泵的踏板侧：0 无，1 左，2 右；撤销该次联动后清零。 */
    uint32_t left_drain_pump; /* 左脚轻踩请求使用的泵：0 无，1 A 泵，2 B 泵；撤销左脚轻踩请求后清零。 */
    uint32_t right_drain_pump; /* 右脚轻踩请求使用的泵：0 无，1 A 泵，2 B 泵；撤销右脚轻踩请求后清零。 */
    uint32_t switch_wait_release; /* 切换成功后仍需先松脚的通道：0 无，1 A，2 B；该侧松脚确认后清零。 */
    uint32_t missing_tool_release; /* 保留原诊断字段位置；RFID缺刀具等待已取消，当前固定为0。 */
    uint32_t restart_release; /* 异常重启后置 1，禁止踩踏启动；连续三轮确认脚踏完全松开后清零。 */
    uint32_t restart_ticks; /* 异常重启后连续完全松脚的次数，每轮 25ms；任一侧踩下或实时帧失效时清零。 */
    uint32_t left_low; /* 脚踏报文解析函数保存的左侧松开定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t left_mid; /* 脚踏报文解析函数保存的左侧中点定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t left_high; /* 脚踏报文解析函数保存的左侧踩到底定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t right_low; /* 脚踏报文解析函数保存的右侧松开定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t right_mid; /* 脚踏报文解析函数保存的右侧中点定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t right_high; /* 脚踏报文解析函数保存的右侧踩到底定标 AD 值；控制任务已采用的值另记在组 24。 */
    uint32_t single_release_ticks; /* 单踏板连续松脚的次数，每轮 25ms；再次踩下后清零，用于延迟撤销开泵请求。 */
    uint32_t switch_a_ticks; /* 左脚深踩请求从 B 切到 A 的确认次数，每轮 25ms；达到切换次数或退出该踩踏区后清零。 */
    uint32_t switch_b_ticks; /* 右脚深踩请求从 A 切到 B 的确认次数，每轮 25ms；达到切换次数或退出该踩踏区后清零。 */
    uint32_t switch_a_ready; /* 原样记录旧字段 switchhandleA_flag；当前切换流程不更新它，不能据此判断已切到 A。 */
    uint32_t switch_b_ready; /* 原样记录旧字段 switchhandleB_flag；当前切换流程不更新它，不能据此判断已切到 B。 */
} DiagnosticFootState_t;

/* 手柄扫描任务结束后复制的认证和插拔数据；A/B 各复制一份，按字段声明顺序写入诊断记录。 */
typedef struct
{
    uint32_t stage; /* 手柄识别步骤按以下顺序执行，遇到失败或拔出时转入对应分支：
     * 0 等待插入；1 连续确认插入；2 等接口稳定；3 校验 EEPROM；4 读取手柄参数。
     * 5 等待 RFID 刀具结果；6 保持在线；7 连续确认拔出。
     * 8 等待下一次认证重试；9 认证失败，暂不允许使用该手柄。
     */
    uint32_t insert_ticks; /* 连续检测到手柄插入的次数，每轮 10ms；检测到拔出时清零。 */
    uint32_t remove_ticks; /* 连续检测到手柄拔出的次数，每轮 10ms；恢复插入电平时清零。 */
    uint32_t verify_step; /* EEPROM 校验步骤：0 读第 1 页，1 读芯片序列号，2～8 读对应页。 */
    uint32_t verified; /* 全部 EEPROM 认证通过后为 1；开始新一轮认证时清零，0 表示尚未全部通过。 */
    uint32_t retry_count; /* 本通道连续认证失败次数；重新初始化认证记录后清零，达到上限后转为慢速重试。 */
    uint32_t retry_ticks; /* 认证失败后已等待的 10ms 周期数；进入新一轮等待或等待结束后清零。 */
    uint32_t alarm; /* 本通道最近的手柄认证报警码；0 表示当前未保存认证报警。 */
    uint32_t identity_generation; /* 开始确认手柄拔出时递增的编号；EEPROM 读取前后编号不同，就不能继续使用该次读取的数据。 */
    uint32_t tool_source; /* 刀具参数从哪里取得：0 手柄 EEPROM 第 3 页，1 刀具 RFID 标签。 */
    uint32_t rfid_ticket; /* 当前 RFID 监测请求的编号，用它查询本次读取结果，避免误用另一轮结果。 */
    uint32_t rfid_sequence; /* 已经处理过的 RFID 结果编号，用于避免重复修改刀具参数和重复蜂鸣。 */
    uint32_t rfid_presence_sequence; /* 最近一次确认本通道 RFID 标签仍在的结果编号。 */
    uint32_t rfid_pending; /* RFID 监测请求被接受后置 1；读取结束或取消等待后清零，0 表示没有等待中的请求。 */
    uint32_t rfid_online; /* 1：已识别到 RFID 刀具；0：尚未识别到或已确认刀具离线。 */
    uint32_t rfid_misses; /* 连续未读到 RFID 标签的次数，用于判断刀具是否已取下。 */
    uint32_t offline_event_required; /* 请求显示手柄上线后置 1；拔出消息成功入队后清零，防止漏发离线通知。 */
    uint32_t physical_level; /* 复制诊断数据时读取的手柄短接引脚：0 低电平表示插入，1 高电平表示拔出。 */
} DiagnosticHandlescanState_t;

/* 手柄按键任务结束后复制的引脚电平和按键确认数据；A/B 各复制一份，诊断读取不改变按键计数。 */
typedef struct
{
    uint32_t pin_mode; /* 手柄按键引脚用途：0 尚未配置，1 GPIO 按键输入，2 UART 接收；用途切换时清旧按键记录。 */
    uint32_t raw_level; /* 读取诊断数据时的引脚电平：GPIO 模式下 0 为按下、1 为松开；UART 模式下不能解释为按键。 */
    uint32_t low_ticks; /* 连续读到按下低电平的次数，每轮 30ms；读到高电平后清零。 */
    uint32_t high_ticks; /* 连续读到松开高电平的次数，每轮 30ms；读到低电平后清零。 */
    uint32_t pressed; /* 按下电平持续达到确认次数后为 1；松开电平持续达到确认次数后清零。 */
    uint32_t press_event; /* 本轮刚确认按下且已满足重启后松手要求时为 1；下一轮扫描先清零，长按不会反复产生。 */
    uint32_t release_event; /* 本轮刚确认松开时为 1；下一轮扫描先清零，LGZI 用它请求松手停机。 */
    uint32_t restart_ready; /* 异常重启后为 0；确认按键稳定松开后置 1，才允许新的按下事件。普通上电初值为 1。 */
    uint32_t owner_channel; /* 由手柄实体键启动的通道：0 无，1 A，2 B；用于判断哪侧松手可以撤销运行请求。 */
} DiagnosticHandleKeyState_t;

/* 泵任务结束后复制的定位、停机限制和压力报警数据；A/B 各复制一份，诊断读取不改变开泵请求。 */
typedef struct
{
    uint32_t direction; /* 本泵本轮选用的方向值，发送帧还会按泵接口配置处理；该值不是驱动反馈方向。 */
    uint32_t driver_hold; /* 驱动未就绪或故障时置 1，要求本泵发零速；驱动恢复就绪且旧请求已清除后清零。 */
    uint32_t last_error; /* 本泵最近处理的驱动故障码；0 表示无故障，定位失败统一记为 6。 */
    uint32_t alignment_notice; /* 用于判断是否播放定位提示音的已知状态：0 未知，0x80 就绪，0x90 定位中，0xA0 失败。 */
    uint32_t realign_phase; /* 重新定位步骤：0 无请求；3、2 各发一次零速；1 发一次定位命令；4 等定位中回包，收到或取消后回到 0。 */
    uint32_t cancel_phase; /* 取消定位步骤：0 无请求；1 待发取消命令；2 持续发取消命令，回包确认已退出正常定位后回到 0。 */
    uint32_t realign_tick; /* 本次请求泵重新定位时的主控毫秒数，用于判断等待驱动确认是否超时。 */
    uint32_t pressure_condition_tick; /* 开始连续确认压力超限或恢复正常时的主控毫秒数，配合 pressure_timing 判断是否正在计时。 */
    uint32_t pressure_sample_tick; /* 上次处理新压力样本时的主控毫秒数；间隔过长时重新确认超限或恢复，不能把断帧前后合成连续时间。 */
    uint32_t pressure_alarm_tick; /* 上次发出压力报警提示时的主控毫秒数，用于限制连续提示的间隔。 */
    uint32_t pressure_target; /* 计算压力报警阈值所用的泵设定值；注水、灌注为 mL/min，吸引为内部档值。 */
    uint32_t pressure_threshold_g; /* 本泵本轮采用的压力报警阈值，单位 g，用于和压力重量比较。 */
    uint32_t pressure_sequence; /* 压力报警计算上次处理的压力帧序号，用于识别是否收到新样本。 */
    uint32_t pressure_valid; /* 收到可用于报警计算的首个压力样本后置 1；重置压力报警记录后清零。 */
    uint32_t pressure_alarm; /* 注水/抽吸已提示或灌注已因超压下发零速时置 1；恢复确认或停止请求后清零。 */
    uint32_t pressure_timing; /* 1：正在连续确认压力超限或恢复；条件中断、确认结束或重置报警后清零。 */
    uint32_t can_run; /* 1：反馈未超时、驱动就绪且无故障、停机限制已解除、无定位或取消请求；否则为 0。 */
} DiagnosticPumpBehaviorState_t;

/* 日志组号须与上位机一致：1～89 为设备数据，90～93 为统计，110～120 为操作记录，128～147 为串口分段数据。 */
#define DIAG_EVENT_FOOT_GATE 110U /* 脚踏启动检查；channel 为踏板侧（1 左、2 右），没有指定侧时为 0。
 * a=1：踏板编号无效；b 为此前启动侧，c 为是否已确认松脚。
 * a=2：电机已有启动侧；b 为该启动侧，c=1 允许同侧继续，c=0 拒绝另一侧。
 * a=3：尚未确认松脚；b 为此前启动侧，c=0，本次不启动。
 * a=4：启动检查通过；b 为当前手柄通道，c=1，随后才处理运行请求。
 * a=5：其它控制方式占用；6 实时帧失效；7 异常重启待松脚；8 驱动故障后禁止启动。
 * a=5～8 时，b 为当前手柄通道，c=0。
 */
#define DIAG_EVENT_KEY_QUEUE 111U /* 按键队列：a 为控制方式，b 为键值，channel 表示消息处理到哪一步。
 * channel=0：尝试入队，c=0 失败、1 成功。
 * channel=1：已取出；channel=2：触控消息过期被丢弃；这两种情况下 c 是等待毫秒数。
 * channel=3：其它控制方式占用，本次按键不执行，c=0。
 */
#define DIAG_EVENT_PUMP_QUEUE 112U /* 泵设定队列：channel=1 A 泵、2 B 泵；a=泵类型，b=设定值，c=0 入队失败、1 已入队、2 已取出。 */
#define DIAG_EVENT_HANDLE_VERIFY 113U /* 手柄认证：channel=逻辑 A/B 通道，a=校验步骤，b=校验返回码，c=物理通道；不保存密钥或芯片序列号。 */
#define DIAG_EVENT_FOOT_PARSE 114U /* 脚踏解析：a=该帧应有字节数，b=0 CRC 错、1 校验通过、2 帧未收齐，c=本轮尚未处理的字节数。 */
#define DIAG_EVENT_EXTERNAL 115U /* 外控命令：a=功能码，b=区域码，c 高 16 位为信息码、低 16 位为信息字节数；只记命令类别。 */
#define DIAG_EVENT_MARK 116U /* 采集标记：a=1 重新开始、2 冻结、4 手工标记；b 为采集批次或手工值，c 为主控毫秒时刻。 */
#define DIAG_EVENT_TASK_SLOW 117U /* 任务延误：channel=任务编号，a=本轮耗时，b=相邻两轮开始间隔，c=任务配置周期，单位均为 ms。 */
#define DIAG_EVENT_UI 118U /* 屏幕消息：channel=显示区域；数据依次为入队/取出、使能、成功标志和 10 字节显示参数，不代表屏幕已经执行。 */
#define DIAG_EVENT_CONTROL 119U /* 手柄联动泵请求：a=0 撤销、1 启动；b 的位 0/1 表示目标 A/B 泵；c=当前选中手柄通道。 */
#define DIAG_EVENT_DRAIN 120U /* 双脚踏轻踩排空：channel=1 为左脚、2 为右脚。
 * a=0：本次不允许开泵；b 为当前手柄通道，c 为切换后仍需松脚的通道。
 * a=1：准备提出开泵请求；b 为选中的泵（1 A、2 B），c 为当前手柄通道。
 * a=2：撤销本侧请求；b 为本侧此前请求的泵，c 为另一侧仍请求的泵，0 表示无泵。
 */

/* 压力解析任务结束后复制的收帧时间和恢复计数；只读已保存数据，不取走串口接收字节。 */
typedef struct
{
    uint32_t last_valid_tick; /* 本泵最近一帧通过格式、CRC 和数值检查的压力数据接收时刻，单位为主控毫秒。 */
    uint32_t loss_suspect_tick; /* 压力帧超时后开始等待掉线确认的主控毫秒数；收到有效帧或结束离线处理后清零。 */
    uint32_t recovery_required; /* 压力数据越界后置 1，暂停采用单个正常帧；连续三个正常帧到达后清零并恢复更新。 */
    uint32_t recovery_count; /* 数值越界后已连续收到的正常压力帧数，再次越界时清零。 */
    uint32_t parser_bytes; /* 压力接收缓存中尚未处理完的字节数，用于判断是否一直收不到完整帧。 */
    uint32_t sequence; /* 本泵最近已接受压力帧的序号；0 也是合法序号，不能仅凭它判断掉线。 */
    uint32_t offline_limit_ms; /* 距最后有效压力帧允许经过的最长时间，单位 ms；超过此时间才可确认压力板掉线。 */
} DiagnosticPressureState_t;

#if (MAINCTRL_DIAG_ENABLE == 1U)
struct task_s;
/* 记录接口只能在任务取得公共业务互斥锁后调用，启动阶段无并发任务时也可调用；不能从中断调用。 */
void Diagnostic_TaskBegin(const struct task_s *task); /* 在业务函数调用前记开始时间；task 为当前任务，不能为空。 */
void Diagnostic_TaskEnd(const struct task_s *task); /* 在业务函数返回后记录脚踏、手柄和泵数据；此时仍须持有公共业务互斥锁。 */
void Diagnostic_Event(uint16_t event, uint8_t channel, uint32_t a, uint32_t b, uint32_t c); /* 保存一个操作及 a/b/c 三个参数，编号和参数含义见上方 DIAG_EVENT_* 说明。 */
void Diagnostic_Wire(uint8_t port, uint8_t transmit, uint16_t result, const uint8_t *data, uint16_t length); /* 保存本次串口收发字节副本；不会额外读串口，也不会替业务发送命令。 */
void Diagnostic_StateObject(uint16_t kind, uint8_t channel, const void *data, uint16_t length); /* data 只含按顺序排列的 32 位字段，length 为字节数且须为 4 的倍数。 */
uint8_t Diagnostic_IsLinkActive(void); /* 1 表示诊断通信仍有效；0 表示可恢复普通外部通信。 */
uint8_t Diagnostic_IsCompleteRequest(const uint8_t *data, uint16_t length); /* 仅完整且 CRC 正确的诊断下行帧返回 1，用于减少 DMA 收帧等待。 */
void Diagnostic_TransportService(void); /* 检查 UART2 发送完成或超时，再选取下一帧；普通通信帧优先于诊断应答。 */
void Diagnostic_Checkpoint(void); /* 同一任务处理连续操作时，在操作之间记录手柄与 A/B 泵请求，保留中间的启停变化。 */
void Diagnostic_Ui(uint8_t stage, uint8_t area, uint8_t enabled, const uint8_t *data, uint8_t result); /* 保存屏幕消息入队或取出记录；data 最多读取 10 字节，为 NULL 时按全零参数记录。 */
void Diagnostic_FootConfig(uint8_t connected, uint8_t type, uint16_t ll, uint16_t lm, uint16_t lh, uint16_t rl, uint16_t rm, uint16_t rh); /* 记录脚踏控制任务已取出的配置；ll/lm/lh 为左侧低中高 AD，rl/rm/rh 为右侧低中高 AD。 */
void SimUart_CopyDiagnosticState(uint8_t channel, DiagnosticPressureState_t *out); /* channel=1 A 压力板、2 B 压力板；out 接收已保存的收帧时间和计数，不取走接收字节。 */
void Diagnostic_SendNormal(const uint8_t *data, uint16_t length); /* 复制完整普通外部通信帧并排队；data 不能为空，length 为 1～160 字节。 */
uint8_t Diagnostic_Command(uint8_t direction, uint8_t function, uint8_t area, uint8_t info, const uint8_t *data, uint16_t length); /* CRC 校验通过后调用；返回 1 表示诊断帧到此处理结束，0 表示普通帧应继续原控制流程。 */
#define DIAG_EVENT(event, channel, a, b, c) Diagnostic_Event((event), (channel), (a), (b), (c)) /* 编译开启时，把业务分支的检查结果交给诊断模块记录。 */
#define DIAG_WIRE(port, tx, result, data, length) Diagnostic_Wire((port), (tx), (result), (data), (length)) /* 编译开启时，记录业务函数已有的串口数据，tx=0 接收、1 发送。 */
#define DIAG_STATE(kind, channel, data, length) Diagnostic_StateObject((kind), (channel), (data), (length)) /* 编译开启时，按组号记录一组 32 位字段。 */
#define DIAG_TASK_BEGIN(task) Diagnostic_TaskBegin(task) /* 编译开启时，记录本轮任务开始时间。 */
#define DIAG_TASK_END(task) Diagnostic_TaskEnd(task) /* 编译开启时，记录本轮任务耗时及处理后的设备数据。 */
#define DIAG_LINK_ACTIVE() Diagnostic_IsLinkActive() /* 诊断请求间隔小于 2500ms 时暂停普通外部通信上报。 */
#define DIAG_SERVICE() Diagnostic_TransportService() /* 编译开启时，由外部通信任务处理待发帧及发送超时。 */
#define DIAG_CHECKPOINT() Diagnostic_Checkpoint() /* 编译开启时，记录同一任务内每次操作完成后的运行请求。 */
#define DIAG_UI(stage, area, enabled, data, result) Diagnostic_Ui((stage), (area), (enabled), (data), (result)) /* 编译开启时，记录屏幕消息是否入队或已取出。 */
#define DIAG_FOOT_CONFIG(msg) Diagnostic_FootConfig((msg)->connect_flag, (msg)->pedalType, (msg)->LValue_Left, (msg)->MValue_Left, (msg)->HValue_Left, (msg)->LValue_Right, (msg)->MValue_Right, (msg)->HValue_Right) /* 编译开启时，记录本条脚踏消息的连接标志、类型及定标 AD 值。 */
#define DIAG_COMMAND(direction, function, area, info, data, length) Diagnostic_Command((direction), (function), (area), (info), (data), (length)) /* 编译开启时，识别并处理 UART2 诊断命令，返回值决定是否继续普通命令处理。 */
void Foot_CopyDiagnosticState(DiagnosticFootState_t *out); /* out 接收脚踏解析和控制变量的副本；不重新解析报文，不修改踩踏请求。 */
void Handlescan_CopyDiagnosticState(uint8_t channel, DiagnosticHandlescanState_t *out); /* channel=1 A、2 B；out 接收手柄识别记录和当前插入电平，不重新读取 EEPROM。 */
void HandleKey_CopyDiagnosticState(uint8_t channel, DiagnosticHandleKeyState_t *out); /* channel=1 A、2 B；out 接收按键确认记录和当前引脚电平，不增加按键确认计数。 */
void PumpBehavior_CopyDiagnosticState(uint8_t channel, DiagnosticPumpBehaviorState_t *out); /* channel=1 A 泵、2 B 泵；out 接收定位和压力报警记录，不发送泵命令。 */
#else
/* 编译关闭时，下面的记录宏不执行任何操作，连传入参数中的函数也不会调用；诊断命令始终返回 0。 */
#define DIAG_EVENT(event, channel, a, b, c) ((void)0) /* 关闭操作记录，不读取参数。 */
#define DIAG_WIRE(port, tx, result, data, length) ((void)0) /* 关闭串口字节记录，不读取参数。 */
#define DIAG_STATE(kind, channel, data, length) ((void)0) /* 关闭设备数据记录，不读取参数。 */
#define DIAG_TASK_BEGIN(task) ((void)0) /* 关闭任务开始计时，不读取参数。 */
#define DIAG_TASK_END(task) ((void)0) /* 关闭任务结束采集，不读取参数。 */
#define DIAG_LINK_ACTIVE() 0U /* 编译关闭时不暂停普通外部通信。 */
#define DIAG_SERVICE() ((void)0) /* 关闭诊断发送处理。 */
#define DIAG_CHECKPOINT() ((void)0) /* 关闭连续操作之间的请求采集。 */
#define DIAG_UI(stage, area, enabled, data, result) ((void)0) /* 关闭屏幕消息记录，不读取参数。 */
#define DIAG_FOOT_CONFIG(msg) ((void)0) /* 关闭脚踏配置记录，不读取参数。 */
#define DIAG_COMMAND(direction, function, area, info, data, length) 0U /* 返回 0，让原有外部通信代码继续判断收到的帧。 */
#endif

#endif
