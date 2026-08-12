//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "uart1.h"
#include "pump.h"
#include <string.h>
#include "kernel_scheduler.h"
#include "Pubinterface.h"
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

#define MOTOR_UART_ALARM_HALL         WORK_ALARM_HALL_ERROR         /* 霍尔断线/学习错误使用统一报警码 9，避免误触发过载锁存。 */
#define MOTOR_UART_ALARM_OVERLOAD     WORK_ALARM_MOTOR_OVERLOAD_ALT /* 过流/堵转沿用统一报警码 5，保持脚踏松开门禁不变。 */
#define MOTOR_UART_ALARM_VOLTAGE      WORK_ALARM_MOTOR_COMM_ERROR   /* 过压/欠压沿用统一报警码 8，外控报警协议保持兼容。 */
#define MOTOR_UART_ALARM_DRIVER_BOARD WORK_ALARM_MOTOR_DRIVER_BOARD /* 缺相及未细分错误在逻辑层归入驱动板故障，避免占用控制方式报警码。 */

static uint8_t s_motor_uart_alarm_owned = 0U;        /* 记录本模块最近一次写入的报警码，驱动恢复正常时只清自己拥有的报警。 */
static uint8_t s_motor_uart_last_driver_error = 0U;  /* 记录上一帧驱动 Err，避免同一个故障每帧重复触发蜂鸣和上位机弹窗。 */
static uint32_t s_motor_uart_alarm_start_tick = 0U;  /* 记录非脚踏驱动报警的最少显示起点，脚踏来源故障不使用该时间退出。 */
static volatile uint8_t s_motor_uart_driver_recovered = 0U; /* 驱动回包 Err=0 后置位，脚踏来源报警必须同时满足该条件才能解除。 */
static volatile uint8_t s_motor_uart_wait_foot_release = 0U; /* 任意驱动故障首次发生于脚踏运行时置位，禁止按固定时间自动清报警。 */
static volatile uint8_t s_motor_uart_foot_release_observed = 0U; /* 记录本次脚踏来源故障是否已观察到松脚、掉线或实时数据失效。 */
static MotorUartFeedbackSnapshot_t s_motor_uart_feedback_snapshot; /* 保存最近一份CRC正确驱动回包的一致性速度、电流和错误码。 */
static volatile uint32_t s_motor_uart_feedback_version = 0U; /* 偶数表示快照稳定，奇数表示UART1接收任务正在更新。 */

/*
 * 函数功能：设置驱动报警，并把逻辑报警码和屏幕图片覆盖值分别投递给各自消费者。
 * 输入参数：alarm_value 为主控统一报警码；picture_value 为屏幕图片号或 MOTOR_ALARM_PICTURE_NONE。
 * 返回参数：无。
 */
static void MotorUart_SetAlarm(uint8_t alarm_value, uint8_t picture_value)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 的 Value[0] 保存逻辑报警码，Value[1] 保存屏幕专用图片覆盖值。 */

	display_value[0] = alarm_value;   /* 逻辑报警码继续供 WorkMessage、蜂鸣和外控协议使用，不能替换成 91~99 图片号。 */
	display_value[1] = picture_value; /* 图片覆盖值只进入屏幕队列，宏开关不会改变其它安全状态。 */
	WorkAlarm_Set(alarm_value);       /* 统一报警状态仍由 WorkAlarm_Set 维护，避免直接改 WorkMessage。 */
	SendAlarmMessage(alarm_value);    /* 同步蜂鸣任务进入对应报警声。 */
	SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 同步屏幕报警弹窗，并保留原始逻辑报警语义。 */
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

	/* 整个连续驱动故障生命周期只在第一帧快照控制来源，后续Err变化不能因脚踏运行位已被停机逻辑清除而丢失来源。 */
	if (s_motor_uart_alarm_owned == 0U)
	{
		s_motor_uart_wait_foot_release =
			(ControlSignalMessage.jtL_control_flag || ControlSignalMessage.jtR_control_flag) ? 1U : 0U; /* 故障首帧仍保留脚踏运行标志，可准确区分脚踏与其它控制来源。 */
		s_motor_uart_foot_release_observed = 0U; /* 新的驱动故障生命周期必须重新等待本次脚踏真实释放，不能沿用上一次结果。 */
	}

	s_motor_uart_last_driver_error = driver_error; /* 记录真实驱动 Err，便于下一帧判断是否发生了故障变化。 */
	s_motor_uart_alarm_owned = alarm_value;        /* 记录本模块拥有的主控报警码，后续驱动恢复正常时才允许自动清除。 */
	s_motor_uart_alarm_start_tick = HAL_GetTick(); /* 非脚踏驱动报警沿用最少显示时间；脚踏来源故障只用松脚和恢复条件。 */
	MotorUart_SetAlarm(alarm_value, picture_value); /* 同步安全报警状态，并按宏选择公用或内部调试图片。 */
}

/*
 * 函数功能：按报警来源处理驱动故障退出；脚踏来源等待松脚和Err=0，其它来源沿用最少显示时间。
 * 输入参数：skip_hold_time 非 0 表示已满足脚踏双条件，可跳过普通故障显示计时；为 0 时按状态自行判断。
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
	s_motor_uart_driver_recovered = 0U;   /* 本次报警生命周期结束，下一次故障必须重新等待 Err=0。 */
	s_motor_uart_wait_foot_release = 0U;  /* 清除脚踏来源等待状态，下一次故障重新快照控制来源。 */
	s_motor_uart_foot_release_observed = 0U; /* 清除本次释放结果，避免下一次脚踏故障跳过松脚确认。 */
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
 * 函数功能：通知电机反馈模块，脚踏来源驱动故障已经观察到松脚、掉线或实时数据失效。
 * 输入参数：无。
 * 返回参数：无；驱动已恢复时立即清除报警，否则保存释放结果并等待后续Err=0。
 */
void MotorUart_ReleaseFootDriverAlarm(void)
{
	if (s_motor_uart_wait_foot_release == 0U)
	{
		return; /* 当前驱动故障不是脚踏来源，不能改变手控、触控或外控报警的定时生命周期。 */
	}

	s_motor_uart_foot_release_observed = 1U; /* 先保存释放结果，覆盖“先松脚后Err=0”和脚踏通信丢失两种时序。 */
	if (s_motor_uart_driver_recovered == 0U)
	{
		return; /* 驱动故障仍存在时继续保持弹窗和蜂鸣，后续Err=0帧会再次进入统一清除入口。 */
	}

	MotorUart_ClearDriverAlarmIfOwned(1U); /* 脚踏释放和Err=0两个条件均已满足，立即关闭报警、蜂鸣和弹窗。 */
}

/*
 * 函数功能：驱动故障恢复边沿撤销所有电机控制源的运行请求，防止旧请求自动恢复。
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
 * 函数功能：把一份CRC正确的12字节驱动回包原子更新为遥测反馈快照。
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
 * 函数功能：复制最近一次CRC正确驱动回包形成的一致性快照。
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

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 无刷
//============================================================================
/*
 * 函数功能：解析 UART1 驱动板回包，更新实际转速、电流和驱动故障状态。
 * 输入参数：无，函数直接读取 UART1 DMA 接收缓冲区。
 * 返回参数：无；合法回包会更新 WorkMessage，并在故障时撤销电机和泵运行请求。
 */
void BrushlessMotorUartData_ReceiveData(void)
{
	static uint8_t clean_huic=0; /* 记录上一周期是否出现驱动故障，用于故障恢复边沿再执行一次安全全停。 */
  uint8_t rlen = 0, i = 0;
  uint8_t dat[UART1_MAX_PACKET_SIZE] = { 0 }, dat1[22] = { 0 };
  uint16_t CRC_Check_Vaule=0;
	
  //读取串口数据
  rlen = Uart1_DMARecvDataPeek(dat);
	/* 少于一帧所需字节时保留 DMA 数据等待后续接收，不能进入 CRC 和字段解析。 */
  if (rlen < 11)   //不够一个数据包大小
	  return;

  //查询本帧数据包的帧头0x01
  for (i = 0; i < (rlen - 11); i++)    //最小帧数据包为4[0x01 0x03 0 0 0 0 0]
  {
	  if (dat[i] == 0xAA)  
	  {
		  CRC_Check_Vaule = Common_Crc16(&dat[i],10);//ssc
			/* CRC 通过后才允许修改运行状态，避免串口噪声被误当成驱动故障或速度反馈。 */
			if(CRC_Check_Vaule==dat[i+10]+(dat[i+11]<<8))
			{
				Common_CopyData(&dat[i], dat1, 12);    //截取10个数据
				MotorUart_RecordFeedbackSnapshot(dat1); /* CRC正确后先形成速度、电流、Err同源快照，供50ms遥测一致读取。 */
				WorkMessage.driver_speed_feedback = (uint16_t)(((uint16_t)dat1[4] << 8U) | dat1[5]); /* 驱动 byte4~5 是实际转速反馈，单位沿用驱动私有协议的“转速/10”，只做监测不改目标速度。 */
				WorkMessage.driver_current_x100 = (uint16_t)(((uint16_t)dat1[8] << 8U) | dat1[9]);    /* 驱动 byte8~9 是 App.FB.Prot.AllCur * 100，单位 0.01A，只上传给上位机显示。 */
					/* byte7 为驱动故障码，0 表示本帧确认驱动已经恢复正常。 */
					if (dat1[7] == MOTOR_UART_DRIVER_ERR_NONE)
					{
						s_motor_uart_driver_recovered = 1U; /* 本帧确认驱动故障已经消失；脚踏来源故障仍需等待松脚条件。 */
						/* 只在“上一帧故障、本帧恢复”的边沿再次全停，防止旧运行请求随故障解除自动恢复输出。 */
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
						pumpMessageB.run_flag = false; /* 同步撤销 B 泵请求，使驱动故障进入全泵停机状态。 */
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
					case 0x02 :  //反向HALLEErrFlag
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
* @brief Function implementing the MOTORUARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTORUARTTaskFunc */
void MOTORUARTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTORUARTTaskFunc */
  /*
   * UART2 已由 ExternalComm 独立任务接管，用于新的外部通信协议。
   * 本任务只保留 UART1 驱动板接收，避免两个任务同时读取 UART2 DMA 缓冲。
   */
  BrushlessMotorUartData_ReceiveData();
  /* USER CODE END MOTORUARTTaskFunc */
}

void MotorUartData_Init(void)
{
  /* definition and creation of MOTORUARTTask */
	Kernel_TaskCreate(&MOTORUARTTaskHandle, MOTORUARTTaskFunc);
	Kernel_TaskStart(&MOTORUARTTaskHandle, KERNEL_TASK_ALWAYS, 3);//3
}





