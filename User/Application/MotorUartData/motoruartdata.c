//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "uart1.h"
#include "pump.h"
#include <string.h>
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "handlescan.h"
#include "sscBEEP.h"
#include "sscUIDP.h"

kernel_task_t MOTORUARTTaskHandle;

#define MOTOR_UART_DRIVER_ERR_NONE       0U  /* 参考驱动 Err=0：无故障，dat1[7] 恢复到该值时允许释放本模块设置的报警。 */
#define MOTOR_UART_DRIVER_ERR_FAIL       1U  /* 参考驱动 Err=1：模块保护，主控按驱动板故障处理。 */
#define MOTOR_UART_DRIVER_ERR_OC1        2U  /* 参考驱动 Err=2：过流保护，主控按电机过载/刀具卡住处理。 */
#define MOTOR_UART_DRIVER_ERR_OV         3U  /* 参考驱动 Err=3：过压保护，主控按系统供电电压异常处理。 */
#define MOTOR_UART_DRIVER_ERR_UV         4U  /* 参考驱动 Err=4：欠压保护，主控按系统供电电压异常处理。 */
#define MOTOR_UART_DRIVER_ERR_RUNSTALL   5U  /* 参考驱动 Err=5：运行中堵转，主控按电机过载/刀具卡住处理。 */
#define MOTOR_UART_DRIVER_ERR_TEMP       6U  /* 参考驱动 Err=6：驱动器过温，当前报警表无独立温度项，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_SAVE       7U  /* 参考驱动 Err=7：参数保存错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_OVR        8U  /* 参考驱动 Err=8：刹车时间过长，不是电压故障，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_ENOC       9U  /* 参考驱动 Err=9：编码器错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_START      10U /* 参考驱动 Err=10：开环检测错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_NOHALL     11U /* 参考驱动 Err=11：霍尔断线，主控按霍尔错误处理。 */
#define MOTOR_UART_DRIVER_ERR_SDAHALL    12U /* 参考驱动 Err=12：霍尔学习错误，主控按霍尔错误处理。 */
#define MOTOR_UART_DRIVER_ERR_HANDSHAKE  13U /* 参考驱动 Err=13：通信握手错误，主控按驱动板故障处理。 */
#define MOTOR_UART_DRIVER_ERR_PHASELOSS  14U /* 参考驱动 Err=14：缺相，主控按电机相位错误处理。 */
#define MOTOR_UART_DRIVER_ERR_POSDRAG    15U /* 参考驱动 Err=15：有 Hall 拖动错误，归入驱动板故障。 */

/* 上方 Err 数值与驱动回包 byte7 一一对应，是协议编号，不是可调整的报警门限。 */
#define MOTOR_UART_DRIVER_PARAMETER_ADDRESS 0xFDU /* 参数读写固定地址；改动须同步驱动板，不能与 0xAA 电机控制帧混用。 */
#define MOTOR_UART_DRIVER_PARAMETER_REQUEST_LEN 8U /* 0x03 读参数、0x06 写参数请求都为 8 字节，包含 CRC；改动须同步协议。 */
#define MOTOR_UART_DRIVER_PARAMETER_TIMEOUT_MS 300U /* 等待参数响应的最长时间，单位 ms；到期生成超时结果，取走结果或保留到期后才恢复周期控制帧。 */
#define MOTOR_UART_DRIVER_PARAMETER_READY_TIMEOUT_MS 300U /* 参数结果最多保留 300ms；到期自动丢弃并恢复周期控制帧。调大可延长取结果时间，也会延长串口占用。 */
#define MOTOR_UART_TASK_PERIOD_MS 3U /* 每次任务按 3ms 累计响应超时；修改时必须同步 MotorUartData_Init 的实际任务周期。 */
#define MOTOR_UART_DRIVER_PICTURE_DELAY_MS 50U /* 报警图片等待时间，单位 ms；先确认手柄是否拔出，避免先闪驱动故障图再显示掉线图。只延后图片，不延后报警和蜂鸣。 */

#define MOTOR_UART_ALARM_HALL         WORK_ALARM_HALL_ERROR         /* 霍尔断线/学习错误使用统一报警码 9，避免误触发过载锁存。 */
#define MOTOR_UART_ALARM_OVERLOAD     WORK_ALARM_MOTOR_OVERLOAD_ALT /* 过流/堵转沿用统一报警码 5，保持脚踏松开门禁不变。 */
#define MOTOR_UART_ALARM_VOLTAGE      WORK_ALARM_MOTOR_COMM_ERROR   /* 过压/欠压沿用统一报警码 8，外控报警协议保持兼容。 */
#define MOTOR_UART_ALARM_DRIVER_BOARD WORK_ALARM_MOTOR_DRIVER_BOARD /* 缺相及未细分错误在逻辑层归入驱动板故障，避免占用控制方式报警码。 */

static uint8_t s_motor_uart_alarm_owned = 0U;        /* 记录本模块最近一次写入的报警码，驱动恢复正常时只清自己拥有的报警。 */
static uint8_t s_motor_uart_last_driver_error = 0U;  /* 记录上一帧驱动 Err，避免同一个故障每帧重复触发蜂鸣和上位机弹窗。 */
static uint32_t s_motor_uart_alarm_start_tick = 0U;  /* 记录非脚踏驱动报警的最少显示起点，脚踏来源故障不使用该时间退出。 */
static volatile uint8_t s_motor_uart_alarm_started_during_run = 0U; /* 记录首次故障时是否要求电机运行；之后确认手柄已拔出时，可据此显示运行掉线报警。 */
static volatile uint8_t s_motor_uart_driver_recovered = 0U; /* 驱动回包 Err=0 后置位，脚踏来源报警必须同时满足该条件才能解除。 */
static volatile uint8_t s_motor_uart_wait_foot_release = 0U; /* 任意驱动故障首次发生于脚踏运行时置位，禁止按固定时间自动清报警。 */
static volatile uint8_t s_motor_uart_foot_release_observed = 0U; /* 记录本次脚踏来源故障是否已观察到松脚、掉线或实时数据失效。 */
static uint8_t s_motor_uart_picture_pending = 0U; /* 驱动报警建立后等待50ms再决定是否向屏幕发送图片，安全报警状态已经立即生效。 */
static uint8_t s_motor_uart_pending_picture_value = MOTOR_ALARM_PICTURE_NONE; /* 保存本次待显示的84/86/87或91~99图片号。 */
static uint32_t s_motor_uart_picture_start_tick = 0U; /* 保存驱动报警图片延迟起点，不参与故障保护和报警退出计时。 */
static MotorUartFeedbackSnapshot_t s_motor_uart_feedback_snapshot; /* 保存最近一包 CRC 正确的速度、电流和错误码，三者必须来自同一包。 */
static volatile uint32_t s_motor_uart_feedback_version = 0U; /* 偶数表示快照稳定，奇数表示UART1接收任务正在更新。 */

typedef enum
{
	MOTOR_UART_DRIVER_PARAMETER_STATE_IDLE = 0U, /* UART1按原周期控制和反馈协议工作。 */
	MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING,   /* 维护请求已发出，正在等待0xFD响应。 */
	MOTOR_UART_DRIVER_PARAMETER_STATE_READY      /* 响应或超时结果已形成，等待外部通信任务取走。 */
} MotorUartDriverParameterState_t;

static volatile MotorUartDriverParameterState_t s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_IDLE; /* 一次只处理一条参数读写请求，记录空闲、等响应或等取结果。 */
static volatile uint16_t s_driver_parameter_elapsed_ms = 0U; /* 从维护请求发出后累计的等待时间。 */
static volatile uint16_t s_driver_parameter_ready_elapsed_ms = 0U; /* READY结果待取期间累计时间，超时后自动释放UART1。 */
static uint8_t s_driver_parameter_response[MOTOR_UART_DRIVER_PARAMETER_MAX_FRAME_SIZE]; /* 保存CRC正确的驱动原始响应。 */
static uint8_t s_driver_parameter_response_len = 0U; /* 保存维护响应实际字节数。 */
static MotorUartDriverParameterResult_t s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_NONE; /* 保存待上层取走的链路结果。 */

/*
 * 函数功能：清除本次参数读写的计时和结果，让 UART1 恢复周期电机控制。
 * 输入参数：无。
 * 返回参数：无。
 */
static void MotorUart_ResetDriverParameterTransaction(void)
{
	s_driver_parameter_elapsed_ms = 0U; /* 清除请求等待计时，下一笔事务重新累计。 */
	s_driver_parameter_ready_elapsed_ms = 0U; /* 清除结果保留计时，避免继承上一笔 READY 时间。 */
	s_driver_parameter_response_len = 0U; /* 清除已取走、过期或因停机而结束的响应长度。 */
	s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_NONE; /* 清除旧结果，下一次查询不能重复取得。 */
	s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_IDLE; /* 最后发布空闲状态，使50ms任务恢复手柄控制帧。 */
}

/*
 * 函数功能：立即设置驱动报警和蜂鸣，并把屏幕图片保存为50ms后的待确认项。
 * 输入参数：alarm_value 为主控统一报警码；picture_value 为屏幕图片号或 MOTOR_ALARM_PICTURE_NONE。
 * 返回参数：无。
 */
static void MotorUart_SetAlarm(uint8_t alarm_value, uint8_t picture_value)
{
	WorkAlarm_Set(alarm_value);       /* 统一报警状态仍由 WorkAlarm_Set 维护，避免直接改 WorkMessage。 */
	SendAlarmMessage(alarm_value);    /* 同步蜂鸣任务进入对应报警声。 */
	s_motor_uart_pending_picture_value = picture_value; /* 暂存要显示的图片号；图片号与报警码不同，不能写入对外报警字段。 */
	s_motor_uart_picture_start_tick = HAL_GetTick(); /* 从安全报警已经生效的时刻开始累计50ms显示确认窗口。 */
	s_motor_uart_picture_pending = 1U; /* 标记图片尚未显示；稍后检查手柄插入检测脚，拔出时等待掉线报警图。 */
}

/*
 * 函数功能：延后显示驱动故障图；如果检测到手柄拔出，先等掉线确认，避免掉线图前闪过其他故障图。
 * 输入参数：无。
 * 返回参数：无。
 */
static void MotorUart_ServicePendingAlarmPicture(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 的 Value[0]保存逻辑报警码，Value[1]保存屏幕专用图片覆盖值。 */

	if (s_motor_uart_picture_pending == 0U)
	{
		return; /* 当前没有待显示驱动图片，不访问GPIO或屏幕队列。 */
	}

	if ((s_motor_uart_alarm_owned == 0U) ||
		(WorkMessage.alarm_flag == false) ||
		(WorkMessage.alarm_value != s_motor_uart_alarm_owned))
	{
		s_motor_uart_picture_pending = 0U; /* 报警已清除或被80等更高层报警接管，旧驱动图片必须作废。 */
		return;
	}

	if ((uint32_t)(HAL_GetTick() - s_motor_uart_picture_start_tick) < MOTOR_UART_DRIVER_PICTURE_DELAY_MS)
	{
		return; /* 延迟未满50ms时不送屏，给10ms手柄扫描和物理触点变化留出确认时间。 */
	}

	if (((WorkMessage.channel_work == CHANNEL_A) || (WorkMessage.channel_work == CHANNEL_B)) &&
		(Handlescan_IsChannelPhysicallyInserted(WorkMessage.channel_work) == false))
	{
		return; /* 当前短接脚已经断开时继续抑制驱动图片，500ms拔出确认后由统一掉线入口直接显示80。 */
	}

	display_value[0] = s_motor_uart_alarm_owned; /* 物理连接仍稳定时按驱动真实逻辑报警码显示84/86/87等生产图片。 */
	display_value[1] = s_motor_uart_pending_picture_value; /* 调试宏开启时保留91~99覆盖图号，关闭时由原映射显示公用图片。 */
	SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 此处只补发图片；报警和蜂鸣已设置，故障首帧也已发送 A 泵零速并撤销 A/B 泵运行请求。 */
	s_motor_uart_picture_pending = 0U; /* 图片已经成功投递，本次故障不再重复刷屏。 */
}

/*
 * 函数功能：把驱动板 Err 编码转换为主控统一报警码。
 * 输入参数：driver_error 为驱动板回包 dat1[7] 的原始错误码。
 * 返回参数：主控统一报警码；Err=0 返回 WORK_ALARM_NONE。
 */
static uint8_t MotorUart_MapDriverErrorToAlarm(uint8_t driver_error)
{
	switch (driver_error)
	{
		case MOTOR_UART_DRIVER_ERR_OC1:
		case MOTOR_UART_DRIVER_ERR_RUNSTALL:
			return MOTOR_UART_ALARM_OVERLOAD; /* 过流和堵转都属于“电机过载/刀具卡住”这类现场可处理故障。 */

		case MOTOR_UART_DRIVER_ERR_OV:
		case MOTOR_UART_DRIVER_ERR_UV:
			return MOTOR_UART_ALARM_VOLTAGE; /* 过压/欠压按 UI 绑定显示系统供电电压异常。 */

		case MOTOR_UART_DRIVER_ERR_NOHALL:
		case MOTOR_UART_DRIVER_ERR_SDAHALL:
			return MOTOR_UART_ALARM_HALL; /* 霍尔断线/学习错误统一使用报警码 9，屏幕公用 86 号图且不再误判为过载。 */

		case MOTOR_UART_DRIVER_ERR_PHASELOSS:
			return MOTOR_UART_ALARM_DRIVER_BOARD; /* 缺相没有独立对外报警码，逻辑层使用驱动板故障，屏幕层按原始 Err 显示 84。 */

		case MOTOR_UART_DRIVER_ERR_NONE:
			return WORK_ALARM_NONE; /* 无故障不产生报警，调用方会负责释放本模块拥有的旧报警。 */

		default:
			return MOTOR_UART_ALARM_DRIVER_BOARD; /* 保存错误、刹车超时、握手错误等统一提示驱动板故障。 */
	}
}

/*
 * 函数功能：根据驱动原始 Err 和调试宏选择屏幕报警图片。
 * 输入参数：driver_error 为驱动板回包 dat1[7] 的原始错误码。
 * 返回参数：84~99 的有效图片号；MOTOR_ALARM_PICTURE_NONE 表示不显示误导图片。
 */
static uint8_t MotorUart_MapDriverErrorToPicture(uint8_t driver_error)
{
	switch (driver_error)
	{
		case MOTOR_UART_DRIVER_ERR_PHASELOSS:
			return 84U; /* 缺相 Err=14 始终使用 84 号“缺相保护”图片，不受内部调试宏影响。 */

		case MOTOR_UART_DRIVER_ERR_RUNSTALL:
			return 87U; /* 运行堵转 Err=5 始终使用 87 号“电机过载/刀具卡住”公用图片。 */

		case MOTOR_UART_DRIVER_ERR_OC1:
#if (MOTOR_ALARM_DETAIL_ENABLE == 1U)
			return 92U; /* 内部调试时把过流 Err=2 细分为 92 号“手柄过流保护”。 */
#else
			return 87U; /* 生产模式下过流与堵转共用 87 号过载图片。 */
#endif

		case MOTOR_UART_DRIVER_ERR_NOHALL:
#if (MOTOR_ALARM_DETAIL_ENABLE == 1U)
			return 93U; /* 内部调试时把 Err=11 显示为 93 号“手柄霍尔断线”。 */
#else
			return 86U; /* 生产模式下霍尔断线与学习错误共用 86 号 HALL 图片。 */
#endif

		case MOTOR_UART_DRIVER_ERR_SDAHALL:
#if (MOTOR_ALARM_DETAIL_ENABLE == 1U)
			return 94U; /* 内部调试时把 Err=12 显示为 94 号“手柄霍尔学习错误”。 */
#else
			return 86U; /* 生产模式下霍尔学习错误与断线共用 86 号 HALL 图片。 */
#endif

#if (MOTOR_ALARM_DETAIL_ENABLE == 1U)
		case MOTOR_UART_DRIVER_ERR_ENOC:
			return 91U; /* 编码器错误 Err=9 使用内部调试图片 91。 */
		case MOTOR_UART_DRIVER_ERR_START:
			return 95U; /* 开环启动检测错误 Err=10 使用内部调试图片 95。 */
		case MOTOR_UART_DRIVER_ERR_OV:
			return 96U; /* 母线过压 Err=3 使用内部调试图片 96。 */
		case MOTOR_UART_DRIVER_ERR_TEMP:
			return 97U; /* 驱动器过温 Err=6 使用内部调试图片 97。 */
		case MOTOR_UART_DRIVER_ERR_POSDRAG:
			return 98U; /* 有霍尔拖动错误 Err=15 使用内部调试图片 98。 */
		case MOTOR_UART_DRIVER_ERR_OVR:
			return 99U; /* 制动时间过长 Err=8 使用内部调试图片 99。 */
#endif

		case MOTOR_UART_DRIVER_ERR_FAIL:
		case MOTOR_UART_DRIVER_ERR_UV:
		case MOTOR_UART_DRIVER_ERR_SAVE:
		case MOTOR_UART_DRIVER_ERR_HANDSHAKE:
		default:
			return MOTOR_ALARM_PICTURE_NONE; /* 未配置图片的错误仍执行安全保护，但不再借用 84 号缺相图片。 */
	}
}

/*
 * 函数功能：把驱动板 Err 编码转换为主控报警，并记录本次弹窗开始时间。
 * 输入参数：driver_error 为驱动板回包 dat1[7] 的错误码。
 * 返回参数：无。
 */
static void MotorUart_SetDriverAlarm(uint8_t driver_error)
{
	uint8_t alarm_value;   /* 保存驱动原始 Err 对应的主控统一报警码。 */
	uint8_t picture_value; /* 保存本次屏幕图片号，调试宏只影响该值。 */

	/* 运行手柄掉线属于当前首要故障，驱动报警不能覆盖该报警，否则用户将失去掉线确认入口。 */
	if (WorkMessage.alarm_value == WORK_ALARM_HANDLE_NOT_CONNECTED) /* 运行手柄掉线报警必须保留用户确认入口，驱动故障不能覆盖。 */
	{
		return;
	}
	alarm_value = MotorUart_MapDriverErrorToAlarm(driver_error);     /* 把参考驱动 Err 编码转换为主控统一报警码。 */
	picture_value = MotorUart_MapDriverErrorToPicture(driver_error); /* 独立计算屏幕图号，避免图片号污染对外报警码。 */

	if (alarm_value == 0U)
	{
		return; /* 防御：无故障不应调用设置报警，直接返回避免误清其他模块状态。 */
	}

	s_motor_uart_driver_recovered = 0U; /* 每一帧非零 Err 都撤销恢复状态，重复故障去重也不能沿用旧的 Err=0 结果。 */

	if ((s_motor_uart_last_driver_error == driver_error) && /* 同一故障已由本模块持有且屏幕仍显示时，不重复上报。 */
	    (s_motor_uart_alarm_owned == alarm_value) &&
	    (WorkMessage.alarm_flag == true) &&
	    (WorkMessage.alarm_value == alarm_value))
	{
		return; /* 同一个驱动故障已上报过，本帧不重复投递蜂鸣/上位机报警。 */
	}

	/* 一次故障只在首帧记录控制来源；后续即使错误码改变，也不能因启动标志被清除而忘记它来自脚踏。 */
	if (s_motor_uart_alarm_owned == 0U)
	{
		s_motor_uart_alarm_started_during_run = (WorkMessage.runflag_work != false) ? 1U : 0U; /* 记住首次故障时是否有运行请求，供稍后的手柄拔出确认使用。 */
		s_motor_uart_wait_foot_release =
			(ControlSignalMessage.jtL_control_flag || ControlSignalMessage.jtR_control_flag) ? 1U : 0U; /* 故障首帧仍保留脚踏运行标志，可准确区分脚踏与其它控制来源。 */
		s_motor_uart_foot_release_observed = 0U; /* 每次新故障都重新等待松脚，不能沿用上一次已经松脚的记录。 */
	}

	s_motor_uart_last_driver_error = driver_error; /* 记录真实驱动 Err，便于下一帧判断是否发生了故障变化。 */
	s_motor_uart_alarm_owned = alarm_value;        /* 记录本模块拥有的主控报警码，后续驱动恢复正常时才允许自动清除。 */
	s_motor_uart_alarm_start_tick = HAL_GetTick(); /* 非脚踏驱动报警沿用最少显示时间；脚踏来源故障只用松脚和恢复条件。 */
	MotorUart_SetAlarm(alarm_value, picture_value); /* 同步安全报警状态，并按宏选择公用或内部调试图片。 */
}

/*
 * 函数功能：决定是否清除驱动报警；脚踏启动时发生的故障，要等松脚且 Err=0，其他来源还要满足最短显示时间。
 * 输入参数：skip_hold_time 非 0 表示跳过最短显示时间；脚踏故障仍必须满足“已松脚、Err=0”两个条件。
 * 返回参数：无；退出条件未满足时继续保持报警、蜂鸣和弹窗。
 */
static void MotorUart_ClearDriverAlarmIfOwned(uint8_t skip_hold_time)
{
	/* 任意驱动故障若发生于脚踏运行，必须同时观察到松脚和Err=0，两个条件到达顺序不影响最终退出。 */
	if (s_motor_uart_wait_foot_release != 0U)
	{
		if ((s_motor_uart_foot_release_observed == 0U) ||
		    (s_motor_uart_driver_recovered == 0U))
		{
			return; /* 任一条件未满足都继续保持弹窗和蜂鸣，不能按普通3秒规则提前退出。 */
		}
		skip_hold_time = 1U; /* 脚踏双条件已满足，立即退出报警，不再额外等待普通来源的3秒最少显示时间。 */
	}

	if ((s_motor_uart_alarm_owned != 0U) && /* 只清除本模块持有且当前仍显示的驱动报警，不能误清其它报警。 */
	    (WorkMessage.alarm_flag == true) &&
	    (WorkMessage.alarm_value == s_motor_uart_alarm_owned))
	{
		if ((skip_hold_time == 0U) &&
		    ((uint32_t)(HAL_GetTick() - s_motor_uart_alarm_start_tick) < ALARM_DRV_MS))
		{
			return; /* 非脚踏来源的驱动报警继续保持原3秒提示规则，避免弹窗闪一下。 */
		}

		WorkAlarm_Clear();                 /* 满足当前报警退出条件后释放本模块写入的报警码。 */
		SendAlarmMessage(WORK_ALARM_NONE); /* 报警退出后同步释放蜂鸣，避免蜂鸣锁存继续保持。 */
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 报警退出后关闭屏幕弹窗，NULL 参数由 UIDP 统一补零。 */
	}

	s_motor_uart_alarm_owned = 0U;        /* 无论当前报警是否被其他模块接管，都释放本模块报警所有权。 */
	s_motor_uart_last_driver_error = 0U;  /* 驱动恢复正常后清掉上一次真实 Err，下一次新故障可以重新上报。 */
	s_motor_uart_alarm_start_tick = 0U;   /* 本次保持周期结束或报警归属已转移，清掉旧 tick 防止下次沿用。 */
	s_motor_uart_alarm_started_during_run = 0U; /* 本次报警结束，清除故障前的运行记录，避免下次停机拔柄被误判为运行掉线。 */
	s_motor_uart_driver_recovered = 0U;   /* 本次报警结束，下次故障必须重新收到 Err=0 才算恢复。 */
	s_motor_uart_wait_foot_release = 0U;  /* 清除等待松脚状态，下次故障重新记录控制来源。 */
	s_motor_uart_foot_release_observed = 0U; /* 清除本次释放结果，避免下一次脚踏故障跳过松脚确认。 */
	s_motor_uart_picture_pending = 0U; /* 报警结束时取消尚未显示的图片，防止恢复后又弹出旧故障图。 */
	s_motor_uart_pending_picture_value = MOTOR_ALARM_PICTURE_NONE; /* 清掉旧覆盖图号，下一次故障重新按真实Err映射。 */
	s_motor_uart_picture_start_tick = 0U; /* 清掉显示延迟起点，避免毫秒计数被下一次故障沿用。 */
}

/*
 * 函数功能：查询当前是否有脚踏来源的驱动故障仍在等待脚踏释放。
 * 输入参数：无。
 * 返回参数：true表示脚踏任务必须继续检测释放；false表示不是脚踏来源或释放已经记录。
 */
bool MotorUart_IsFootDriverAlarmWaitingRelease(void)
{
	return ((s_motor_uart_wait_foot_release != 0U) &&
	        (s_motor_uart_foot_release_observed == 0U)); /* 只在尚未观察到释放时要求脚踏任务继续执行AD判断。 */
}

/*
 * 函数功能：查询驱动故障首次出现时是否有电机运行请求，供拔柄确认时改显示 80 号掉线报警图。
 * 输入参数：无。
 * 返回参数：true 表示故障发生时有运行请求且当前仍显示该驱动报警；false 表示条件不满足。
 */
bool MotorUart_DidDriverAlarmStartDuringRun(void)
{
	return ((s_motor_uart_alarm_started_during_run != 0U) &&
			(s_motor_uart_alarm_owned != 0U) &&
			(WorkMessage.alarm_flag != false) &&
			(WorkMessage.alarm_value == s_motor_uart_alarm_owned)); /* 只有驱动报警仍是当前报警时才允许物理掉线接管，不能覆盖泵压等其它报警。 */
}

/*
 * 函数功能：通知电机反馈模块，脚踏来源驱动故障已经观察到松脚、掉线或实时数据失效。
 * 输入参数：无。
 * 返回参数：无；驱动已恢复时立即清除报警，否则保存释放结果并等待后续Err=0。
 */
void MotorUart_ReleaseFootDriverAlarm(void)
{
	if (s_motor_uart_wait_foot_release == 0U)
	{
		return; /* 不是脚踏启动时发生的故障，不通过松脚来清除，仍按其显示时间处理。 */
	}

	s_motor_uart_foot_release_observed = 1U; /* 先保存释放结果，覆盖“先松脚后Err=0”和脚踏通信丢失两种时序。 */
	if (s_motor_uart_driver_recovered == 0U)
	{
		return; /* 驱动故障仍存在时继续保持弹窗和蜂鸣，后续Err=0帧会再次进入统一清除入口。 */
	}

	MotorUart_ClearDriverAlarmIfOwned(1U); /* 脚踏释放和Err=0两个条件均已满足，立即关闭报警、蜂鸣和弹窗。 */
}

/*
 * 函数功能：驱动刚从故障恢复时，清除各来源的电机运行请求，防止沿用故障前的请求自动启动。
 * 输入参数：无，函数直接清理 WorkMessage 和 ControlSignalMessage。
 * 返回参数：无。
 */
static void MotorUart_StopAllWork(void)
{
	WorkMessage.runflag_work = false; /* 撤销电机运行命令，驱动任务下一周期继续保持停机。 */

	ControlSignalMessage.handle_control_flag = false; /* 清除手柄按键运行来源，避免故障恢复后再次置位。 */
	ControlSignalMessage.HMI_control_flag = false; /* 清除外控运行来源，要求上位机重新下发启动。 */
	ControlSignalMessage.jtL_control_flag = false; /* 清除左脚踏运行来源，要求用户松开后重新踩下。 */
	ControlSignalMessage.jtR_control_flag = false; /* 清除右脚踏运行来源，保持双脚踏停机状态一致。 */
	
}

/*
 * 函数功能：保存一包 CRC 正确的驱动反馈；更新前后改变版本号，让读取方避开写入中的数据。
 * 输入参数：frame指向已经复制并通过CRC校验的驱动回包。
 * 返回参数：无。
 */
static void MotorUart_RecordFeedbackSnapshot(const uint8_t *frame)
{
	uint16_t speed_field; /* 驱动回包byte4~5保存“实际rpm/10”字段。 */

	if (frame == NULL)
	{
		return; /* 空指针不能形成有效反馈，保留上一份一致性快照。 */
	}

	speed_field = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[5]); /* 先按驱动大端字段还原16位速度。 */
	++s_motor_uart_feedback_version; /* 发布奇数版本，外部通信任务不得在字段写入中复制。 */
	__DMB(); /* 保证写入中标志先于快照字段可见。 */
	s_motor_uart_feedback_snapshot.sequence = (uint16_t)(s_motor_uart_feedback_snapshot.sequence + 1U); /* 每份CRC正确回包递增，16位自然回绕。 */
	s_motor_uart_feedback_snapshot.feedback_tick_ms = HAL_GetTick(); /* 记录本帧CRC确认完成时的主控单调毫秒时钟。 */
	s_motor_uart_feedback_snapshot.speed_rpm = (uint32_t)speed_field * 10U; /* 把驱动私有“rpm/10”换算成上位机统一使用的rpm。 */
	s_motor_uart_feedback_snapshot.current_x100 = (uint16_t)(((uint16_t)frame[8] << 8U) | frame[9]); /* 电流保留0.01A单位，避免浮点参与固件组包。 */
	s_motor_uart_feedback_snapshot.raw_error = frame[7]; /* 保留驱动原始Err，不能用主控报警码替代诊断依据。 */
	s_motor_uart_feedback_snapshot.valid = 1U; /* 所有字段写完后标记已有有效反馈。 */
	__DMB(); /* 保证字段先于最终偶数版本发布。 */
	++s_motor_uart_feedback_version; /* 写入完成，读取方可以复制整份快照。 */
}

/*
 * 函数功能：读取最近一次有效驱动反馈，读取途中发生更新时重试，避免混用两次回包。
 * 输入参数：snapshot指向调用方提供的快照缓存。
 * 返回参数：快照有效且复制成功返回1，否则返回0。
 */
uint8_t MotorUart_CopyFeedbackSnapshot(MotorUartFeedbackSnapshot_t *snapshot)
{
	uint8_t attempt; /* 瞬时并发最多重试三次，避免3ms接收任务异常时阻塞UART2通信任务。 */

	if (snapshot == NULL)
	{
		return 0U; /* 调用方未提供目标缓存时拒绝复制。 */
	}

	for (attempt = 0U; attempt < 3U; ++attempt)
	{
		uint32_t version_before = s_motor_uart_feedback_version; /* 复制前版本必须为稳定偶数。 */
		uint32_t version_after; /* 字段复制后复核版本，防止速度、电流、Err来自不同回包。 */

		if ((version_before & 1U) != 0U)
		{
			continue; /* UART1接收任务正在写入时立即重试。 */
		}

		__DMB(); /* 先完成版本判断，再读取共享字段。 */
		*snapshot = s_motor_uart_feedback_snapshot; /* 后续UART2组包只读取调用方私有副本。 */
		__DMB(); /* 字段复制结束后再读取最终版本。 */
		version_after = s_motor_uart_feedback_version;
		if ((version_before == version_after) && ((version_after & 1U) == 0U))
		{
			return (snapshot->valid != 0U) ? 1U : 0U; /* 版本稳定时返回快照有效位。 */
		}
	}

	memset(snapshot, 0, sizeof(*snapshot)); /* 多次并发冲突时返回全零无效值，避免上传混合快照。 */
	return 0U;
}

/*
 * 函数功能：判断近期有效驱动回包是否仍报告电机转动，过期回包只保留诊断值、不再作为忙状态。
 * 输入参数：无，反馈超过 MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS 毫秒后不再用于判断是否转动。
 * 返回参数：true表示近期反馈速度非零，复制冲突时按旧非零值保守保持；false表示零速、尚无反馈或反馈已过期。
 */
bool MotorUart_IsRecentFeedbackMoving(void)
{
	MotorUartFeedbackSnapshot_t feedback_snapshot; /* 同时读取同一包的速度和接收时间，不能把新速度和旧时间配在一起。 */
	uint32_t feedback_age_ms; /* 保存当前时刻距最后CRC正确回包的毫秒数，使用无符号减法兼容HAL时钟回绕。 */

	if (MotorUart_CopyFeedbackSnapshot(&feedback_snapshot) == 0U)
	{
		return (WorkMessage.driver_speed_feedback != 0U); /* 快照碰到接收任务写入时按旧非零值保持“忙”，避免瞬时并发导致提前释放控制权。 */
	}

	feedback_age_ms = (uint32_t)(HAL_GetTick() - feedback_snapshot.feedback_tick_ms); /* 计算距最后有效回包过去了多久，不修改保存的反馈值。 */
	if (feedback_age_ms >= MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS)
	{
		return false; /* 运行请求已经撤销后，旧非零回包超过250ms不再永久锁住脚踏、屏幕或手柄控制权。 */
	}

	return (feedback_snapshot.speed_rpm != 0U); /* 近期回包速度非零时继续等待真实停稳，速度为零时允许正常释放。 */
}

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 无刷
//============================================================================
/*
 * 函数功能：每次任务累计 3ms；参数响应等待到期时生成超时结果，结果保留到期时恢复周期通信。
 * 输入参数：无。
 * 返回参数：无。
 */
static void MotorUart_ServiceDriverParameterTimeout(void)
{
	if (s_driver_parameter_state == MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING)
	{
		s_driver_parameter_elapsed_ms = (uint16_t)(s_driver_parameter_elapsed_ms + MOTOR_UART_TASK_PERIOD_MS); /* 每次任务增加固定3ms，保持与调度周期一致。 */
		if (s_driver_parameter_elapsed_ms >= MOTOR_UART_DRIVER_PARAMETER_TIMEOUT_MS)
		{
			s_driver_parameter_response_len = 0U; /* 超时没有合法原始回包，长度必须清零。 */
			s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_TIMEOUT; /* 通知外部通信任务区分超时和驱动拒绝。 */
			s_driver_parameter_ready_elapsed_ms = 0U; /* 从超时结果形成时刻开始计算 READY 保留时间。 */
			s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_READY; /* 短暂保留超时结果，等待上层取走。 */
		}
		return; /* WAITING 本周期已经处理完成，不能再按 READY 重复累计。 */
	}

	if (s_driver_parameter_state == MOTOR_UART_DRIVER_PARAMETER_STATE_READY)
	{
		s_driver_parameter_ready_elapsed_ms = (uint16_t)(s_driver_parameter_ready_elapsed_ms + MOTOR_UART_TASK_PERIOD_MS); /* 结果待取时独立累计保留期限。 */
		if (s_driver_parameter_ready_elapsed_ms >= MOTOR_UART_DRIVER_PARAMETER_READY_TIMEOUT_MS)
		{
			MotorUart_ResetDriverParameterTransaction(); /* 上层长期未取结果时自动回到空闲，禁止维护状态永久阻塞手柄控制。 */
		}
	}
}

/*
 * 函数功能：从UART1空闲包中查找并保存一帧CRC正确的0xFD维护响应。
 * 输入参数：data为DMA空闲包副本；data_len为副本长度。
 * 返回参数：捕获到合法维护响应返回1；未找到、半包或CRC错误返回0。
 */
static uint8_t MotorUart_TryCaptureDriverParameterResponse(const uint8_t *data, uint16_t data_len)
{
	uint16_t offset; /* 允许空闲包前面残留最后一帧0xAA反馈，因此逐字节搜索维护地址。 */

	if ((data == NULL) || (s_driver_parameter_state != MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING))
	{
		return 0U; /* 空数据或非等待状态都不能形成新的维护结果。 */
	}

	for (offset = 0U; (uint16_t)(offset + 3U) <= data_len; ++offset)
	{
		uint16_t frame_len; /* 根据功能码计算本候选响应的完整长度。 */
		uint16_t received_crc; /* 保存响应末尾低字节在前的CRC。 */
		uint16_t calculated_crc; /* 保存主控对候选响应重新计算的CRC。 */

		if (data[offset] != MOTOR_UART_DRIVER_PARAMETER_ADDRESS)
		{
			continue; /* 不是固定维护地址时继续搜索，不能把0xAA运行反馈当成参数响应。 */
		}

		if (data[offset + 1U] == 0x03U)
		{
			if (((data[offset + 2U] & 1U) != 0U) || (data[offset + 2U] > 60U))
			{
				continue; /* 03响应数据必须是偶数字节且最多包含30个16位参数。 */
			}
			frame_len = (uint16_t)data[offset + 2U] + 5U; /* 地址、功能码、字节数、数据和2字节CRC。 */
		}
		else if (data[offset + 1U] == 0x06U)
		{
			frame_len = MOTOR_UART_DRIVER_PARAMETER_REQUEST_LEN; /* 06成功响应固定回显完整8字节请求。 */
		}
		else if ((data[offset + 1U] == 0x83U) || (data[offset + 1U] == 0x86U))
		{
			frame_len = 5U; /* Modbus异常响应为地址、异常功能码、异常原因和CRC。 */
		}
		else
		{
			continue; /* 固定维护地址下的未知功能码不交给参数界面。 */
		}

		if (((uint16_t)(offset + frame_len) > data_len) ||
			(frame_len > MOTOR_UART_DRIVER_PARAMETER_MAX_FRAME_SIZE))
		{
			continue; /* 当前DMA包没有完整候选帧时继续等待下一个空闲包。 */
		}

		received_crc = (uint16_t)data[offset + frame_len - 2U] |
							 ((uint16_t)data[offset + frame_len - 1U] << 8U); /* 驱动Modbus CRC按低字节在前返回。 */
		calculated_crc = Common_Crc16((uint8_t *)&data[offset], (uint16_t)(frame_len - 2U)); /* 公共CRC接口历史上未声明const，这里只读计算并显式转换指针。 */
		if (received_crc != calculated_crc)
		{
			continue; /* CRC错误只丢弃候选帧，事务继续等待直到合法回包或超时。 */
		}

		memcpy(s_driver_parameter_response, &data[offset], frame_len); /* CRC通过后一次性保存完整原始响应。 */
		s_driver_parameter_response_len = (uint8_t)frame_len; /* 最大65字节，可以安全收窄到uint8_t。 */
		s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_OK; /* 标记链路已经得到合法响应。 */
		s_driver_parameter_ready_elapsed_ms = 0U; /* 从合法响应形成时刻重新计算上层取结果期限。 */
		s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_READY; /* 等待外部通信任务取走，期间继续暂停0xAA周期帧。 */
		return 1U;
	}

	return 0U; /* 本DMA包没有合法维护响应，保留等待状态。 */
}

/*
 * 函数功能：启动一笔固定8字节的驱动参数维护请求。
 * 输入参数：request指向完整Modbus请求；request_len必须为8。
 * 返回参数：成功占用UART1并发送返回1；参数错误或已有事务返回0。
 */
uint8_t MotorUart_StartDriverParameterRequest(const uint8_t *request, uint8_t request_len)
{
	if ((request == NULL) || (request_len != MOTOR_UART_DRIVER_PARAMETER_REQUEST_LEN) ||
		(s_driver_parameter_state != MOTOR_UART_DRIVER_PARAMETER_STATE_IDLE))
	{
		return 0U; /* 拒绝空请求、非8字节请求和并发事务，防止UART1响应归属混乱。 */
	}

	s_driver_parameter_elapsed_ms = 0U; /* 新事务从发送时刻重新累计300ms超时。 */
	s_driver_parameter_ready_elapsed_ms = 0U; /* 新事务尚未形成结果，清除上一笔 READY 保留时间。 */
	s_driver_parameter_response_len = 0U; /* 清除上一笔响应长度，避免超时后误用旧数据。 */
	s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_NONE; /* 清除上一笔链路结果。 */
	s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING; /* 发送前先占用，避免50ms任务插入0xAA帧。 */
	Uart1_SendPacket((uint8_t *)request, request_len); /* UART1底层为阻塞发送，请求数据在返回前保持有效。 */
	return 1U;
}

/*
 * 函数功能：取走一笔已经完成的驱动参数维护结果并恢复UART1周期通信。
 * 输入参数：response接收原始响应；response_len接收长度；result接收链路结果。
 * 返回参数：有结果返回1；结果尚未形成返回0。
 */
uint8_t MotorUart_PollDriverParameterResponse(uint8_t *response,
													 uint8_t *response_len,
													 MotorUartDriverParameterResult_t *result)
{
	if ((response == NULL) || (response_len == NULL) || (result == NULL) ||
		(s_driver_parameter_state != MOTOR_UART_DRIVER_PARAMETER_STATE_READY))
	{
		return 0U; /* 输出指针无效或事务未完成时不得释放维护占用。 */
	}

	*response_len = s_driver_parameter_response_len; /* 先返回长度，超时结果固定为0。 */
	*result = s_driver_parameter_result; /* 返回成功或超时，供主控外层协议映射状态。 */
	if (s_driver_parameter_response_len > 0U)
	{
		memcpy(response, s_driver_parameter_response, s_driver_parameter_response_len); /* 只在确有回包时复制原始帧。 */
	}

	MotorUart_ResetDriverParameterTransaction(); /* 结果复制完成后统一清理状态，50ms任务下一周期恢复手柄控制帧。 */
	return 1U;
}

/*
 * 函数功能：为停机结束当前参数响应等待，并把本次读写记为超时，供外层取走。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorUart_AbortDriverParameterTransaction(void)
{
	if (s_driver_parameter_state != MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING)
	{
		return; /* 空闲或结果已形成时不改写状态，避免重复停机刷新 READY 期限。 */
	}

	s_driver_parameter_elapsed_ms = 0U; /* 停机已抢占本次等待，清除尚未到期的请求计时。 */
	s_driver_parameter_ready_elapsed_ms = 0U; /* 从抢占时刻开始给外层保留一次明确结果。 */
	s_driver_parameter_response_len = 0U; /* 被抢占事务没有可交付的驱动响应。 */
	s_driver_parameter_result = MOTOR_UART_DRIVER_PARAMETER_RESULT_TIMEOUT; /* 复用现有超时状态通知上层本次维护未完成。 */
	s_driver_parameter_state = MOTOR_UART_DRIVER_PARAMETER_STATE_READY; /* 本次改为等取结果；调用方 MOTORRUN 可继续发送本周期停止帧，后续周期仍要检查此状态。 */
}

/*
 * 函数功能：查询 UART1 是否正在等待参数响应或等待上层取走结果。
 * 输入参数：无。
 * 返回参数：等待或结果待取返回1；空闲返回0。
 */
uint8_t MotorUart_IsDriverParameterTransactionActive(void)
{
	return (s_driver_parameter_state == MOTOR_UART_DRIVER_PARAMETER_STATE_IDLE) ? 0U : 1U; /* READY状态也保持暂停，防止响应尚未封装就恢复周期帧。 */
}

/*
 * 函数功能：读取 UART1 接收包，按当前状态处理参数响应或 12 字节电机反馈。
 * 输入参数：无，直接读取 UART1 DMA 接收缓冲区。
 * 返回参数：无；更新速度、电流和报警。故障时发 A 泵零速并撤销 A/B 泵请求；故障恢复首帧再清电机请求。
 */
void BrushlessMotorUartData_ReceiveData(void)
{
	static uint8_t clean_huic=0; /* 记录上一周期是否出现驱动故障，用于故障恢复边沿再执行一次安全全停。 */
  uint8_t rlen = 0, i = 0;
  uint8_t dat[UART1_MAX_PACKET_SIZE] = { 0 }, dat1[22] = { 0 };
  uint16_t CRC_Check_Vaule=0;
	
  //读取串口数据
  rlen = Uart1_DMARecvDataPeek(dat);
	/* 参数读写期间优先处理参数响应，不按普通电机反馈解析。 */
	if (s_driver_parameter_state == MOTOR_UART_DRIVER_PARAMETER_STATE_WAITING)
	{
		(void)MotorUart_TryCaptureDriverParameterResponse(dat, rlen); /* 维护期间只解析0xFD响应，禁止落入0xAA运行反馈分支。 */
		return;
	}
	if (s_driver_parameter_state == MOTOR_UART_DRIVER_PARAMETER_STATE_READY)
	{
		return; /* 结果待取期间保持UART1业务解析静默，确保维护响应不会被普通反馈覆盖。 */
	}
  if (rlen < 11)   //数据过短时返回；下方循环还要求从当前位置能取到完整 12 字节。
	  return;

  //逐字节查找电机反馈帧头 0xAA。
  for (i = 0; i < (rlen - 11); i++)    //只检查从 i 开始至少还剩 12 字节的位置。
  {
	  if (dat[i] == 0xAA)  
	  {
		  CRC_Check_Vaule = Common_Crc16(&dat[i],10);//ssc
			/* CRC 通过后才允许修改运行状态，避免串口噪声被误当成驱动故障或速度反馈。 */
			if(CRC_Check_Vaule==dat[i+10]+(dat[i+11]<<8))
			{
				Common_CopyData(&dat[i], dat1, 12);    //复制完整 12 字节反馈，包含末尾两字节 CRC。
				MotorUart_RecordFeedbackSnapshot(dat1); /* CRC正确后先形成速度、电流、Err同源快照，供50ms遥测一致读取。 */
				WorkMessage.driver_speed_feedback = (uint16_t)(((uint16_t)dat1[4] << 8U) | dat1[5]); /* 驱动 byte4~5 是实际转速反馈，单位沿用驱动私有协议的“转速/10”，只做监测不改目标速度。 */
				WorkMessage.driver_current_x100 = (uint16_t)(((uint16_t)dat1[8] << 8U) | dat1[9]);    /* 驱动 byte8~9 是当前模式ADC滤波实际电流 * 100：无刷为AllCur、有刷为CurLPF，单位0.01A，只上传给上位机显示。 */
					/* byte7 为驱动故障码，0 表示本帧确认驱动已经恢复正常。 */
					if (dat1[7] == MOTOR_UART_DRIVER_ERR_NONE)
					{
						s_motor_uart_driver_recovered = 1U; /* 本帧确认驱动故障已经消失；脚踏来源故障仍需等待松脚条件。 */
						/* 只在上一帧有故障、本帧 Err=0 时清电机请求，避免故障恢复后沿用旧启动请求。 */
						if(clean_huic==1){
							clean_huic=0;
							MotorUart_StopAllWork();
						}
						/* 仅清理由驱动故障创建的报警，不能覆盖手柄掉线等更高层报警。 */
						MotorUart_ClearDriverAlarmIfOwned(0U); /* 普通驱动恢复路径保留原最少显示时间。 */
						
					}
					else
					{
						/* 记录故障存在，供故障恢复帧执行一次安全全停。 */
						clean_huic=1;
						/* 把驱动私有故障码转换为主工程报警，保持蜂鸣、屏幕和上位机一致。 */
						MotorUart_SetDriverAlarm(dat1[7]);
						Pump_SetSpeed_A(0);//立即下发 A 泵零速，先切断当前泵硬件输出
						//Pump_SetSpeed_B(0);//泵停止运行
						pumpMessageA.run_flag = false; /* 撤销 A 泵运行请求，防止泵任务下一周期重新拉起。 */
						//pumpMessageA.speed_work = 0U;
						pumpMessageB.run_flag = false; /* 撤销 B 泵普通运行请求；这里没有直接下发 B 泵零速，也没有清除排空标志。 */
						//pumpMessageB.speed_work = 0U;
						// MotorUart_StopAllWork();
					}	
					/* byte1 是驱动当前方向回显；现阶段仅保留协议分支，不据此改写主控方向状态。 */
					switch (dat1[1])
				{
					case 0x01 :  //正向
					{

					}
					break;
					case 0x02 :  //反向回显；当前不据此修改主控方向。
					{

					}
					break;
					case 0x03 :  //往复
					{

					}
					break;
					case 0x04 :
					case 0x05 :
					{
					}
					break;
					default : break;
				}

				memset(dat1, 0, 15);
				i += 12;
			}
	  }
  }
}

//============================================================================
//与主板进行串口通讯的任务初始化 2
//============================================================================
/* USER CODE BEGIN Header_MOTORUARTTaskFunc */
/**
* 函数功能：每周期处理参数响应超时、电机反馈和待显示的报警图片。
* 输入参数：event 为任务事件，当前不使用。
* 返回参数：无。
*/
/* USER CODE END Header_MOTORUARTTaskFunc */
void MOTORUARTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTORUARTTaskFunc */
  /*
   * UART2 已由 ExternalComm 独立任务接管，用于新的外部通信协议。
   * 本任务只保留 UART1 驱动板接收，避免两个任务同时读取 UART2 DMA 缓冲。
   */
  MotorUart_ServiceDriverParameterTimeout(); /* 先推进维护事务超时，再读取本周期可能到达的驱动响应。 */
  BrushlessMotorUartData_ReceiveData();
  MotorUart_ServicePendingAlarmPicture(); /* 驱动反馈处理后延迟确认屏幕图片，物理拔柄时让80直接成为第一张报警图。 */
  /* USER CODE END MOTORUARTTaskFunc */
}

/*
 * 函数功能：创建电机反馈任务，每 3ms 处理一次 UART1 接收及超时。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorUartData_Init(void)
{
  /* definition and creation of MOTORUARTTask */
	Kernel_TaskCreate(&MOTORUARTTaskHandle, MOTORUARTTaskFunc);
	Kernel_TaskStart(&MOTORUARTTaskHandle, KERNEL_TASK_ALWAYS, 3);//3
}





