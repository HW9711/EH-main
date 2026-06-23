// 调用对应头文件
#include "Pubinterface.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
// #include "datahand.h"
/* 使用泵任务的统一速度上限宏，保证屏幕加减速和实际 UART 输出限幅一致。 */
#include "pump.h"
#include "sscDRIVE.h"
#include "sscUIDP.h"
#include "sscBEEP.h"
#include "sscRFID.h"
#include "external_comm_task.h"

ChannelrecognizeMessage_t ChannelrecognizeMessageA;
ChannelrecognizeMessage_t ChannelrecognizeMessageB;
ControlSigleMessage_t ControlSigleMssage;
ControlSignalMessage_t ControlSignalMessage;

WorkMessage_t WorkMessage;		   // 工作信息
ChannelMemoryMessagr_t MemoryMsgA; // 通道记忆（增对可调节参数），用于切换手柄
ChannelMemoryMessagr_t MemoryMsgB; //
pumpMessage_t pumpMessageA;
pumpMessage_t pumpMessageB;
uint32_t paoxueSpeciValue_A[4] = {0U}; /* A 通道刀具规格缓存从旧屏适配层迁出，供手柄识别和新 UI 读取。 */
uint32_t paoxueSpeciValue_B[4] = {0U}; /* B 通道刀具规格缓存从旧屏适配层迁出，旧接口删除后仍保留业务数据。 */
uint8_t paoxueSpeciValue_F[16] = {0U}; /* 分体刀具扩展缓存保留在业务层，避免旧屏文件被删除后丢失识别状态。 */
/* 当前控制权持有者，四种控制方式必须等待当前持有者结束后才能重新申请。 */
static volatile uint8_t s_control_owner = CONTROL_OWNER_NONE;
/* Page4 速度/频率阈值只驱动蜂鸣，不写 WorkMessage.alarm_flag，避免阈值提示阻塞降速操作。 */
static uint8_t s_speed_threshold_beep_active = 0U;
/* 阈值蜂鸣保持期间定时重发报警消息，避免普通按键蜂鸣覆盖后阈值提示静音。 */
static uint8_t s_speed_threshold_beep_refresh_ticks = 0U;
/* 屏幕外控退出第一次点击是否已记录，第二次点击在 1 秒窗口内才真正退出外控。 */
static uint8_t s_screen_external_exit_pending = 0U;
/* 屏幕外控退出第一次点击的 HAL tick 时间戳，用于计算 1 秒双击窗口。 */
static uint32_t s_screen_external_exit_first_tick = 0U;
/* 公共接头缺少 EPC 刀具头时的报警保持时间，和手柄校验临时报警一致使用 3 秒提示。 */
#define COMMON_SOCKET_TOOL_MISSING_ALARM_MS 3000U
/* 公共接头缺刀具提示的最小重发间隔，避免脚踏/屏幕保活连续触发导致蜂鸣队列堆积。 */
#define COMMON_SOCKET_TOOL_MISSING_REPEAT_MS 1000U
/* 公共接头缺刀具提示是否已经显示，用于 EPC 识别成功后只清理本模块产生的临时屏幕报警。 */
static uint8_t s_common_socket_tool_missing_alarm_active = 0U;
/* 公共接头缺刀具提示最近一次发送时间，用于限制重复报警频率。 */
static uint32_t s_common_socket_tool_missing_alarm_tick = 0U;
/* 手柄冷却跟随当前占用 A 泵，停止手柄时只释放本函数启动过的 A 泵输出。 */
#define HANDLE_INJECTION_FOLLOW_PUMP_A 0x01U
/* 手柄冷却跟随当前占用 B 泵，支持单个 B 注水泵跨通道给手柄降温。 */
#define HANDLE_INJECTION_FOLLOW_PUMP_B 0x02U
/* 记录手柄冷却跟随实际启动过哪些注水泵，避免停止手柄时误停非跟随来源的泵。 */
static uint8_t s_handle_injection_pump_follow_mask = 0U;

/* 电机反馈小于等于该阈值时认为机械输出已停止；0 表示必须等驱动反馈真实归零。 */
#define CONTROL_ARBITRATION_MOTOR_STOP_SPEED_THRESHOLD 0U
/* 手柄运行联动注水泵的默认冷却流量；仅在设备码已识别为注水泵时使用，避免沿用 Page4 或上位机留下的过大速度。 */
#define HANDLE_INJECTION_PUMP_DEFAULT_FLOW 70U
/* 抽吸泵业务最大显示/设定速度，和 sscPUMPA/B.c 中最终输出限幅保持一致。 */
#define PUMP_DRAWWATER_SPEED_MAX 15U
/* 灌注泵业务最大显示/设定速度，和 sscPUMPA/B.c 中最终输出限幅保持一致。 */
#define PUMP_POURWATER_SPEED_MAX 300U
/* 抽吸泵 0 速启动时的兜底速度，避免只置 run_flag 但泵任务继续输出 0。 */
#define PUMP_DRAWWATER_START_SPEED 3U
/* 灌注泵 0 速启动时的兜底速度，对应屏幕一档灌注流量。 */
#define PUMP_POURWATER_START_SPEED 50U
/* 新屏速度按键在旧通道记忆无步进时的兜底步进，避免初次插入或旧参数为空时按键无效。 */
#define SCREEN_SPEED_STEP_FALLBACK 1000U
/* 新屏速度大步进使用小步进的两倍，和 EX8 表里快减/快加的用户体感保持一致。 */
#define SCREEN_SPEED_STEP_DOUBLE_FACTOR 2U
/* 屏幕外部控制退出要求 1 秒内连续两次点击，防止主运行页小电脑图标被误触后直接释放外控。 */
#define SCREEN_EXTERNAL_EXIT_DOUBLE_CLICK_MS 1000U

/* RFID 自动识别最近一次可见刀具图标，0 表示没有可展示的识别历史。 */
static uint8_t s_rfid_last_tool_type_a = 0U;
static uint8_t s_rfid_last_tool_type_b = 0U;

/*
 * 函数功能：把 EEPROM 识别到的手柄型号转换成屏幕 UI 使用的手柄图标类别。
 * 输入参数：handle_model 当前通道记忆中的手柄型号。
 * 返回参数：屏幕 UI_HANDLE_ID 使用的图标类别。
 */
static uint8_t Pubinterface_MapHandleModelToUiType(uint8_t handle_model)
{
	switch (handle_model)
	{
	case TMBB_ONLINES:
	case TMBA_ONLINES:
	case EMBA_ONLINES:
	case EMBB_ONLINES:
	case EMBD_ONLINES:
	case EMBC_ONLINES:
	case JMB_ONLINES:
		return 1U; /* 耳膜/EMBD/磨钻类按 UI 类别 1 显示，避免直接发送原始型号导致错图。 */
	case PXBA_ONLINES:
	case PXBB_ONLINES:
		return 2U; /* 空心钻 A/B 型按 UI 类别 2 显示，保证屏幕图标稳定。 */
	case PX_YIM_ONLINES:
	case PX_YIP_ONLINES:
		return 3U; /* 一体刨类按 UI 类别 3 显示，切通道时屏幕显示一体刀图标。 */
	case MX_YIM_ONLINES:
	case MX_YIM16_ONLINES:
	case MX_YIP_ONLINES:
		return 4U; /* 一体磨类按 UI 类别 4 显示，避免原始型号 7/8/12 无匹配图标。 */
	case LGZ_I_ONLINES:
	case LGZ_II_ONLINES:
	case KXZ_I_ONLINES:
	case KXZ_II_ONLINES:
		return 5U; /* 颅骨钻/空心钻预留类复用图标类别 5。 */
	case KSZ_I_ONLINES:
	case KSZ_II_ONLINES:
		return 6U; /* 克氏针预留类复用图标类别 6。 */
	case COMMON_SOCKET_ONLINES:
		return COMMON_SOCKET_ONLINES; /* 公共接头沿用 UIDP 专用占位图标。 */
	default:
		return handle_model; /* 未知型号保持原值，方便后续新增 UI 资源时直接扩展。 */
	}
}

static void Pubinterface_SendHandleDisplay(uint8_t channel, uint8_t handle_model, bool enable_flag, bool light_flag)
{
	uint8_t display_value[10] = {0U};

	/* Value[0] 传手柄类型，sscUIDP::UIHANDLEDP() 用它决定显示哪一种手柄图标。 */
	display_value[0] = Pubinterface_MapHandleModelToUiType(handle_model);
	/* Value[1] 传 A/B 通道号，1 表示 A 通道，2 表示 B 通道。 */
	display_value[1] = channel;
	/* Value[2] 传选中高亮状态，当前工作通道亮起，非当前通道只显示在线。 */
	display_value[2] = light_flag ? 1U : 0U;
	/* 手柄插拔事件已经完成状态更新后，通过 UIDP 队列刷新屏幕手柄区域。 */
	SendUIDSMessage(UI_HANDLE_ID, enable_flag, display_value);
}

static void Pubinterface_RefreshOnlineHandleDisplay(void)
{
	/* A 通道在线时刷新 A 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Aonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_A,
									   MemoryMsgA.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_A));
	}

	/* B 通道在线时刷新 B 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Bonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_B,
									   MemoryMsgB.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_B));
	}
}

/*
 * 函数功能：判断当前手柄是否使用刀具规格窗口显示长度、直径和角度。
 * 输入参数：hand_model EEPROM 识别出的手柄型号。
 * 返回参数：true 表示优先显示规格窗口，false 表示显示普通刀具图标。
 */
static bool Pubinterface_IsSplitToolSpecDisplayModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||
			(hand_model == PXBB_ONLINES));
}

/*
 * 函数功能：判断当前手柄型号是否允许使用屏幕刀具规格窗口显示 RFID/EPC 解析出的长度、直径和角度。
 * 输入参数：hand_model EEPROM 识别出的手柄型号。
 * 返回参数：true 表示当前型号允许打开 UI_TOOLSPEC_ID；false 表示当前型号必须隐藏规格窗口。
 */
static bool Pubinterface_IsRfidToolSpecDisplayModel(uint8_t hand_model)
{
	return ((Pubinterface_IsSplitToolSpecDisplayModel(hand_model) == true) ||
			(hand_model == COMMON_SOCKET_ONLINES)); /* 公共接头按 EPC 读取刀具规格，只加入规格显示 gate，不加入 PXBA/PXBB 识别按钮 gate。 */
}

/*
 * 函数功能：判断当前刀具字段是否代表支持往复的刨刀能力。
 * 输入参数：tool_type 当前通道识别或手动选择得到的刀具类型字段。
 * 返回参数：true 表示该刀具按刨刀能力开放往复；false 表示按普通磨头/未知刀具处理。
 */
static bool Pubinterface_IsPlanerCapabilityTool(uint8_t tool_type)
{
	return ((tool_type == PLANER) ||
			(tool_type == PX_YIP_ONLINES)); /* PLANER 是新业务确认的刨刀能力标志，PXP 老型号值兼容为刨刀能力，PXM 按 GRINDH 处理。 */
}

/*
 * 函数功能：判断当前手柄型号是否允许切换到往复方向。
 * 输入参数：hand_model EEPROM 识别出的手柄型号。
 * 返回参数：true 表示该手柄支持正转/往复/反转三种方向切换，false 表示只能使用正反转。
 */
static bool Pubinterface_IsOscDirectionSupportedModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||
			(hand_model == PXBB_ONLINES) ||
			(hand_model == PX_YIP_ONLINES)); /* PXBA/PXBB 和 PXP 兼容型号具备往复硬件基础，PXM 按磨头能力不再开放往复。 */
}

/*
 * 函数功能：综合手柄型号和刀具类型判断当前通道是否支持往复方向。
 * 输入参数：hand_model 当前选中通道手柄型号；tool_type 当前选中通道刀具类型。
 * 返回参数：true 表示方向键可切入 OSCDIR；false 表示往复键保持禁用或按键无效。
 */
static bool Pubinterface_IsOscDirectionSupported(uint8_t hand_model, uint8_t tool_type)
{
	(void)hand_model; /* 最新规则把往复能力收敛到 tool_type=PLANER，手柄型号只影响开口定位范围。 */
	if ((false != false) && Pubinterface_IsOscDirectionSupportedModel(hand_model))
	{
		return true; /* 死引用旧型号判定函数，只为压住静态未引用告警，实际结果仍由 tool_type 决定。 */
	}
	if (Pubinterface_IsPlanerCapabilityTool(tool_type))
	{
		return true; /* tool_type=PLANER 或兼容 PXP 旧码时才允许往复，GRINDH/PXM 不再进入 OSCDIR。 */
	}

	return false; /* GRINDH 或未知刀具不支持往复，避免 PXBA/PXBB 基座在磨头刀具下误亮往复按钮。 */
}

/*
 * 函数功能：判断开口定位入口是否允许显示和发送。
 * 输入参数：hand_model 当前手柄基座型号；tool_type 当前刀具能力类型。
 * 返回参数：true 表示 PXBA/PXBB 且刀具为 PLANER，false 表示隐藏开口定位并拦截动作。
 */
static bool Pubinterface_IsOpenPositionEnabledTool(uint8_t hand_model, uint8_t tool_type)
{
	return ((Pubinterface_IsSplitToolSpecDisplayModel(hand_model) == true) &&
			(Pubinterface_IsPlanerCapabilityTool(tool_type) == true)); /* 开口定位只给 PXBA/PXBB 的 PLANER 刀具开放，PXP 只保留往复能力。 */
}

/*
 * 函数功能：记录当前通道最近一次 RFID 识别出的刀具类型。
 * 输入参数：channel 为 A/B 通道；tool_type 为归一后的 PLANER/GRINDH 或兼容旧型号码。
 * 返回参数：无。
 */
static void Pubinterface_SetLastRfidToolType(uint8_t channel, uint8_t tool_type)
{
	if (channel == CHANNEL_A)
	{
		s_rfid_last_tool_type_a = tool_type; /* A 通道保留最后一次 RFID 刀具类型，用于掉线后显示 61/62。 */
	}
	else if (channel == CHANNEL_B)
	{
		s_rfid_last_tool_type_b = tool_type; /* B 通道保留最后一次 RFID 刀具类型，避免 A/B 掉线图标串用。 */
	}
}

/*
 * 函数功能：读取当前通道最近一次 RFID 识别出的刀具类型。
 * 输入参数：channel 为 A/B 通道。
 * 返回参数：最近一次识别出的刀具类型；0 表示没有识别历史。
 */
static uint8_t Pubinterface_GetLastRfidToolType(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return s_rfid_last_tool_type_a; /* A 通道没有识别历史时返回 0，自动识别等待图显示 63。 */
	}
	if (channel == CHANNEL_B)
	{
		return s_rfid_last_tool_type_b; /* B 通道没有识别历史时返回 0，自动识别等待图显示 63。 */
	}
	return 0U; /* 非 A/B 通道没有 RFID 刀具历史。 */
}

/*
 * 函数功能：把刀具类型转换成 EX8 0x1404 识别结果图片编号。
 * 输入参数：tool_type 为 PLANER/GRINDH 或兼容旧 PXM/PXP 型号码。
 * 返回参数：61 表示磨头掉线图，62 表示刨刀掉线图，63 表示等待/默认图。
 */
static uint8_t Pubinterface_MapToolTypeToResultPicture(uint8_t tool_type)
{
	if ((tool_type == PLANER) || (tool_type == PX_YIP_ONLINES))
	{
		return 62U; /* PLANER/PXP 掉线后显示刨刀识别结果图。 */
	}
	if ((tool_type == GRINDH) || (tool_type == PX_YIM_ONLINES))
	{
		return 61U; /* GRINDH/PXM 掉线后显示磨头识别结果图。 */
	}
	return 63U; /* 没有有效识别历史时显示默认手柄刨标，表示等待正确 RFID 数据。 */
}

/*
 * 函数功能：读取指定通道扫描阶段缓存的刀具规格，并转换成 UIDP 使用的 4 字节显示数据。
 * 输入参数：channel 当前屏幕选中的通道，display_value UIDP 消息数据缓冲区。
 * 返回参数：true 表示规格有效，可打开 UI_TOOLSPEC_ID；false 表示没有有效规格。
 */
static bool Pubinterface_GetToolSpecForChannel(uint8_t channel, uint8_t *display_value)
{
	const uint32_t *spec_values = NULL;
	uint32_t tool_length = 0U;
	uint32_t tool_diameter = 0U;
	uint32_t tool_angle = 0U;
	bool raw_spec_display = (WorkMessage.hand_model == COMMON_SOCKET_ONLINES); /* 公共接头 EPC 标签字段按原始十六进制整数显示，不能复用 PXBA/PXBB 的倍率显示单位。 */

	if (channel == CHANNEL_A)
	{
		spec_values = paoxueSpeciValue_A;
	}
	else if (channel == CHANNEL_B)
	{
		spec_values = paoxueSpeciValue_B;
	}
	else
	{
		return false;
	}

	if (raw_spec_display)
	{
		tool_length = spec_values[0]; /* 公共接头 EPC 长度缓存就是原始整数值，0x60 直接显示为 96mm。 */
		tool_diameter = spec_values[1]; /* 公共接头 EPC 直径缓存就是原始整数值，0x10 直接显示为 16。 */
		tool_angle = spec_values[2]; /* 公共接头 EPC 角度缓存就是原始整数值，0x10 直接显示为 16°。 */
	}
	else
	{
		tool_length = spec_values[0] * 5U; /* PXBA/PXBB USER 扫描缓存 [0] 保存长度基值，屏幕显示前恢复成实际长度值。 */
		tool_diameter = spec_values[1]; /* PXBA/PXBB USER 直径仍按 x10 单位交给旧 LCD 格式化路径。 */
		tool_angle = spec_values[2]; /* PXBA/PXBB USER 角度仍按历史缓存单位交给旧屏幕格式化路径。 */
	}

	if ((tool_length == 0U) || (tool_diameter == 0U))
	{
		return false; /* 长度或直径为 0 说明本通道还没有有效刀具规格，避免残留旧数据上屏。 */
	}

	if (tool_length > 0xFFFFU)
	{
		tool_length = 0xFFFFU; /* UIDP 长度字段只有 16 位，异常数据钳位后再显示。 */
	}
	if (tool_diameter > 0xFFU)
	{
		tool_diameter = 0xFFU; /* UIDP 直径字段为 8 位，避免高位截断造成不可预期显示。 */
	}
	if (tool_angle > 0xFFU)
	{
		tool_angle = 0xFFU; /* UIDP 角度字段为 8 位，避免高位截断造成不可预期显示。 */
	}

	display_value[0] = (uint8_t)(tool_length >> 8);
	display_value[1] = (uint8_t)(tool_length & 0xFFU);
	display_value[2] = (uint8_t)tool_diameter;
	display_value[3] = (uint8_t)tool_angle;
	display_value[4] = raw_spec_display ? 1U : 0U; /* Value[4] 通知 UIDP 本次规格为公共接头 EPC 原始整数格式。 */
	return true;
}



// static void Pubinterface_RefreshToolDisplaySSC(uint8_t channel, bool planer_selected)
// {
// 	uint8_t display_value[10] = {0U};
// 	uint8_t manual_display_value[10] = {0U}; /* 单独保存手动/自动识别按钮参数，避免和 RFID 规格长度低字节共用 Value[1]。 */
// 	bool show_tool_spec = false;
// 	bool auto_identify = (WorkMessage.auto_identify != 0U);
// 	bool open_position_enabled = Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model, WorkMessage.tool_type); /* 开口定位必须同时满足 PXBA/PXBB 和 PLANER。 */
// 	uint8_t result_tool_type = WorkMessage.tool_type;


// }
/*
 * 函数功能：刷新当前通道的刀具图标、刀具规格、手动识别按钮和开口定位区域。
 * 输入参数：channel 当前选中通道，planer_selected 当前刀具是否为刨刀。
 * 返回参数：无。
 */
static void Pubinterface_RefreshToolDisplay(uint8_t channel, bool planer_selected)
{
	uint8_t display_value[10] = {0U};
	uint8_t manual_display_value[10] = {0U}; /* 单独保存手动/自动识别按钮参数，避免和 RFID 规格长度低字节共用 Value[1]。 */
	bool show_tool_spec = false;
	bool auto_identify = (WorkMessage.auto_identify != 0U); /* 自动识别模式下，未读到 RFID 规格前不显示手动磨/刨按钮。 */
	bool split_tool_spec_handle = Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model); /* 当前通道是否为 PXBA/PXBB 分体 RFID 手柄，普通 EEPROM 手柄不显示刀具识别区。 */
	bool rfid_spec_handle = Pubinterface_IsRfidToolSpecDisplayModel(WorkMessage.hand_model); /* 当前通道是否具备 RFID/EPC 规格显示能力，公共接头也要显示刀具规格。 */
	bool common_socket_spec_handle = (WorkMessage.hand_model == COMMON_SOCKET_ONLINES); /* 公共接头没有 PXBA/PXBB 识别按钮，但 EPC 规格有效时必须打开规格窗口。 */
	bool rfid_display_enabled = (auto_identify && split_tool_spec_handle); /* 当前选中通道必须是 PXBA/PXBB 才允许显示 RFID 规格和自动识别图标，避免普通手柄继承另一通道残留。 */
	bool rfid_spec_window_enabled = (rfid_display_enabled || common_socket_spec_handle); /* PXBA/PXBB 仍按自动识别状态显示规格，公共接头按 EPC 规格缓存直接显示。 */
	bool open_position_enabled = Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model, WorkMessage.tool_type); /* 开口定位必须同时满足 PXBA/PXBB 和 PLANER。 */
	uint8_t result_tool_type = WorkMessage.tool_type; /* 自动识别等待/掉线图标优先看当前刀具类型，没有当前刀具时再看历史识别。 */

	manual_display_value[0] = planer_selected ? 1U : 0U; /* 手动按钮 Value[0] 继续表示磨/刨选择，1 为刨刀、0 为磨头。 */
	manual_display_value[1] = rfid_display_enabled ? 1U : 0U; /* 只有当前通道确认为 RFID 手柄时才写自动识别图 51，普通手柄保持 0 供隐藏分支清掉识别区。 */
	if (rfid_spec_handle == false)
	{
		SendUIDSMessage(UI_TOOL_ID, false, manual_display_value); /* 普通 EEPROM 手柄不使用 EX8 刀具识别图，切通道时先关闭旧通道刀具图入口。 */
		SendUIDSMessage(UI_TOOLSPEC_ID, false, display_value); /* 普通 EEPROM 手柄没有 RFID 规格窗口，必须清掉 A 通道 PXBA/PXBB 残留规格。 */
		SendUIDSMessage(UI_MANUALBUTTON_ID, false, manual_display_value); /* enable=false 且自动识别标志为 0 时，UIDP 会隐藏 0x1404/0x1405/0x1406/0x1407 整块识别区。 */
		SendUIDSMessage(UI_ORAL_ID, false, display_value); /* 普通 EEPROM 手柄不开放 PXBA/PXBB 刨刀开口定位，避免上一通道按钮残留。 */
		return; /* 普通手柄刀具信息只供上位机和运行参数使用，屏幕主运行页不显示 RFID 识别控件。 */
	}
	if ((rfid_display_enabled == true) && (result_tool_type == 0U))
	{
		result_tool_type = Pubinterface_GetLastRfidToolType(channel); /* RFID 手柄掉线后 WorkMessage.tool_type 会清 0，此时只用本 RFID 通道最近一次识别类型决定 61/62。 */
	}
	manual_display_value[2] = rfid_display_enabled ? Pubinterface_MapToolTypeToResultPicture(result_tool_type) : 63U; /* Value[2] 只给 RFID 自动识别区驱动 61/62/63，普通手柄始终刷默认值清残留。 */

	if (rfid_spec_window_enabled)
	{
		show_tool_spec = Pubinterface_GetToolSpecForChannel(channel, display_value); /* PXBA/PXBB 自动识别和公共接头 EPC 都从通道规格缓存读取长度、直径和角度。 */
	}

	if (show_tool_spec)
	{
		manual_display_value[2] = 0U; /* 自动识别成功时 0x1404 必须隐藏，只保留 0x4200 刀具规格。 */
		SendUIDSMessage(UI_TOOL_ID, false, display_value); /* 显示规格窗口时关闭普通刀具图，避免两个区域同时显示。 */
		SendUIDSMessage(UI_MANUALBUTTON_ID, false, manual_display_value); /* 已有有效规格时关闭手动磨/刨按钮，但 0x1407 仍显示自动识别模式。 */
		SendUIDSMessage(UI_TOOLSPEC_ID, true, display_value); /* 规格数据按长度高低字节、直径、角度发送给屏幕。 */
	}
	else
	{
		SendUIDSMessage(UI_TOOLSPEC_ID, false, display_value); /* 没有有效规格时关闭规格窗口，防止显示上一次刀具数据。 */
		if (rfid_display_enabled)
		{
			SendUIDSMessage(UI_TOOL_ID, true, manual_display_value); /* 自动识别等待 RFID 时隐藏普通刀具图，避免误以为当前是手动磨/刨模式。 */
			SendUIDSMessage(UI_MANUALBUTTON_ID, true, manual_display_value); /* 自动识别模式下隐藏手动磨/刨按钮，同时把 0x1407 刷成自动识别图 51。 */
		}
		else if (common_socket_spec_handle)
		{
			SendUIDSMessage(UI_TOOL_ID, false, manual_display_value); /* 公共接头没有有效 EPC 规格时隐藏刀具图，避免误显示 PXBA/PXBB 或普通手柄残留。 */
			SendUIDSMessage(UI_MANUALBUTTON_ID, false, manual_display_value); /* 公共接头没有手动识别按钮，未读到 EPC 前整块识别按钮区保持关闭。 */
		}
		else
		{
			SendUIDSMessage(UI_TOOL_ID, true, manual_display_value); /* 手动模式或普通手柄无规格时继续显示刀具图标。 */
			SendUIDSMessage(UI_MANUALBUTTON_ID, true, manual_display_value); /* 手动识别按钮跟随当前刨/磨状态刷新，并把 0x1407 刷成手动识别图 52。 */
		}
	}

	SendUIDSMessage(UI_ORAL_ID, (rfid_display_enabled && (show_tool_spec == false)) ? false : open_position_enabled, display_value); /* 只有当前 RFID 手柄等待规格时才隐藏开口定位，普通手柄切入时立即按自身能力刷新。 */
}

/*
 * 函数功能：当前通道离线且没有可回落通道时关闭参数区显示。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Pubinterface_ClearSelectedChannelDisplay(void)
{
	uint8_t display_value[10] = {0U};

	SendUIDSMessage(UI_CONTROL_ID, false, display_value);
	if(ControlSignalMessage.jt_enable_flag==true)
	{
		display_value[0]=1;
		display_value[1]=1;
		SendUIDSMessage(UI_CONTROL_ID, true, display_value);
	}
	SendUIDSMessage(UI_DIR_ID, false, display_value);
	SendUIDSMessage(UI_FREQ_ID, false, display_value);
	SendUIDSMessage(UI_TOOL_ID, false, display_value);
	SendUIDSMessage(UI_TOOLSPEC_ID, false, display_value);
	SendUIDSMessage(UI_ORAL_ID, false, display_value);
	SendUIDSMessage(UI_SPEED_ID, false, display_value);
	SendUIDSMessage(UI_MANUALBUTTON_ID, false, display_value);
}

/*
 * 函数功能：判断当前手柄型号是否为带手控入口的预留型号。
 * 输入参数：hand_model 为 EEPROM 识别出的系统内部手柄型号。
 * 返回参数：true 表示界面允许显示手控入口，false 表示保持当前控制方式。
 */
static bool Pubinterface_IsHandleControlReservedModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||    /* EX8 表格确认 PXBA 分体手柄允许显示并进入手控。 */
			(hand_model == PXBB_ONLINES) ||    /* EX8 表格确认 PXBB 分体手柄允许显示并进入手控。 */
			(hand_model == LGZ_II_ONLINES));   /* EX8 表格确认 LGZII 双按键手柄保留手控入口，其它预留型号不再误亮手控。 */
}

/*
 * 函数功能：判断当前手柄型号是否为公共接头预留型号。
 * 输入参数：hand_model 为 EEPROM 识别出的系统内部手柄型号。
 * 返回参数：true 表示该型号没有手柄实体键，不自动切入手控。
 */
static bool Pubinterface_IsCommonSocketReservedModel(uint8_t hand_model)
{
	return (hand_model == COMMON_SOCKET_ONLINES); /* 公共接头仅用于连接/显示，不占用手柄实体键控制逻辑。 */
}

/*
 * 函数功能：判断当前选中的公共接头通道是否已经读取到有效 EPC 刀具头。
 * 输入参数：无，直接读取 WorkMessage 和当前通道 MemoryMsg。
 * 返回参数：true 表示非公共接头或公共接头刀具已就绪；false 表示公共接头基座在线但 EPC 刀具未就绪。
 */
bool Pubinterface_IsCommonSocketToolReady(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 当前通道记忆用于复核刀具类型，避免 WorkMessage 切换过程中的短暂字段不一致。 */

	if (WorkMessage.hand_model != COMMON_SOCKET_ONLINES)
	{
		return true; /* 非公共接头不受 EPC 刀具头 gate 限制，保持原有启动逻辑。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* A 通道公共接头使用 A 通道记忆判断 EPC 刀具是否已写入。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* B 通道公共接头使用 B 通道记忆判断 EPC 刀具是否已写入。 */
	}
	else
	{
		return false; /* 没有选中通道时不能运行公共接头，避免无归属的启动命令下发到驱动。 */
	}

	if ((memory == NULL) || (memory->hand_model != COMMON_SOCKET_ONLINES))
	{
		return false; /* 通道记忆尚未同步为公共接头时，认为刀具头未就绪，等待插拔事件完成。 */
	}

	return ((WorkMessage.tool_type != 0U) && (memory->tool_type != 0U)); /* EPC 识别成功后会写入刀具类型；为 0 表示仍在等待标签。 */
}

/*
 * 函数功能：公共接头未识别到 EPC 刀具头时提示“请连接手柄”报警。
 * 输入参数：无，直接读取当前 HAL tick 和 WorkMessage 报警状态。
 * 返回参数：无。
 */
static void Pubinterface_ReportCommonSocketToolMissing(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余字节补零保持消息格式稳定。 */
	uint32_t now_tick = HAL_GetTick(); /* 使用 HAL 毫秒 tick 做重发限频，避免连续控制帧让蜂鸣任务反复入队。 */

	if ((s_common_socket_tool_missing_alarm_active != 0U) &&
		((uint32_t)(now_tick - s_common_socket_tool_missing_alarm_tick) < COMMON_SOCKET_TOOL_MISSING_REPEAT_MS))
	{
		return; /* 1 秒内重复触发只保持当前提示，不再重复蜂鸣和刷屏。 */
	}

	s_common_socket_tool_missing_alarm_active = 1U; /* 记录本模块已经显示临时报警，后续 EPC 成功装载时由本模块清除。 */
	s_common_socket_tool_missing_alarm_tick = now_tick; /* 记录本次提示时间，用于下一次触发限频。 */
	display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 公共接头无刀具头时沿用“手柄未连接/请连接手柄”图片 80。 */
	SendAlarmMessageTimed(WORK_ALARM_HANDLE_NOT_CONNECTED, COMMON_SOCKET_TOOL_MISSING_ALARM_MS); /* 只做限时蜂鸣，不写 WorkMessage.alarm_flag，避免阻塞后续 RFID 识别。 */
	ExternalComm_SendTransientAlarm(WORK_ALARM_HANDLE_NOT_CONNECTED, COMMON_SOCKET_TOOL_MISSING_ALARM_MS); /* 上位机同步收到临时报警，便于外控模式下也知道刀具头未接入。 */
	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 没有更高优先级全局报警时，屏幕显示“请连接手柄”提示。 */
	}
}

/*
 * 函数功能：手柄电机启动前检查公共接头 EPC 刀具头是否就绪，缺失时触发“请连接手柄”临时报警。
 * 输入参数：无，直接读取当前通道、WorkMessage 和通道记忆中的公共接头刀具状态。
 * 返回参数：true 表示允许继续启动手柄电机；false 表示公共接头缺 EPC 刀具头，本次启动必须拒绝。
 */
bool Pubinterface_CheckCommonSocketToolReadyForRun(void)
{
	if (Pubinterface_IsCommonSocketToolReady() == false)
	{
		Pubinterface_ReportCommonSocketToolMissing(); /* 只有真正请求手柄电机运行时才报警，避免外控授权或泵控制被误判成刀具启动。 */
		return false; /* 公共接头没有有效 EPC 刀具参数时禁止运行，避免无刀具头仍下发电机启动帧。 */
	}

	return true; /* 非公共接头或 EPC 刀具已装载时，保持原有脚踏/手柄/屏幕/外控启动路径。 */
}

/*
 * 函数功能：公共接头 EPC 刀具头识别成功后清除本模块产生的临时报警显示。
 * 输入参数：无，直接读取公共接头就绪状态和 WorkMessage 报警状态。
 * 返回参数：无。
 */
static void Pubinterface_ClearCommonSocketToolMissingAlarm(void)
{
	if (s_common_socket_tool_missing_alarm_active == 0U)
	{
		return; /* 本模块没有显示过公共接头临时报警时不操作屏幕，避免误清其它报警。 */
	}

	if (Pubinterface_IsCommonSocketToolReady() == false)
	{
		return; /* EPC 刀具头仍未就绪时继续保留“请连接手柄”提示。 */
	}

	s_common_socket_tool_missing_alarm_active = 0U; /* 当前公共接头刀具已就绪，本模块的临时报警生命周期结束。 */
	s_common_socket_tool_missing_alarm_tick = 0U; /* 清时间戳，下一次刀具头移除后可立即重新提示。 */
	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 无全局报警时清掉临时屏幕报警，避免 EPC 成功后仍显示请连接手柄。 */
	}
}

/*
 * 函数功能：按 EX8 表格规则把不可用的脚控选中态回落到手控选中态。
 * 输入参数：无，直接读取当前通道、脚踏在线标志和当前手柄型号。
 * 返回参数：无。
 */
static void Pubinterface_ApplyEx8ControlModeFallback(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前通道记忆，回落手控时要同步修改，避免下次切回通道又恢复脚控。 */

	if (WorkMessage.hmiactive_work != 0U)
	{
		return; /* 外部通信占用时内部复用 TOUCHWORK，不能因为脚踏离线把外控改成手控。 */
	}

	if (WorkMessage.touchactive_work == TOUCHWORK)
	{
		return; /* 触控界面已经打开时保持触控来源，避免切通道或脚踏掉线误关闭触控控制。 */
	}

	if (ControlSignalMessage.jt_enable_flag == true)
	{
		return; /* 脚踏在线时脚控仍是有效选项，不需要按 EX8 失能规则回落。 */
	}

	if (Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model) == false)
	{
		return; /* 只有 PXBA/PXBB/LGZII 允许脚控失能后自动显示手控选中，普通手柄不能误亮手控。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* A 通道当前被选中时，同步修正 A 通道控制方式记忆。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* B 通道当前被选中时，同步修正 B 通道控制方式记忆。 */
	}
	else
	{
		return; /* 没有有效工作通道时不修改全局控制方式，避免无手柄状态凭空显示手控。 */
	}

	if (WorkMessage.drivetype_work == JTWORK)
	{
		WorkMessage.drivetype_work = HANDLEWORK; /* EX8 表格要求脚控失能时手控选中，当前工作态必须同步改成手控。 */
	}

	if ((memory != NULL) && (memory->drive_type == JTWORK))
	{
		memory->drive_type = HANDLEWORK; /* 通道记忆也同步回落，否则 A/B 切换后会再次把不可用脚控装载回来。 */
	}
}

/*
 * 函数功能：按当前控制方式立即刷新屏幕脚控、手控、触控按钮状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshControlModeDisplay(void)
{
	uint8_t display_value[10] = {0U}; /* UI_CONTROL_ID 固定使用 Value[0] 表示按钮编号，Value[1] 表示是否选中。 */
	bool foot_control_available = false; /* 脚控是否可用延后到回落逻辑之后计算，避免脚踏掉线时继续显示旧选中态。 */
	bool external_control_active = false; /* 外控占用状态延后计算，保证和回落后的 WorkMessage 保持一致。 */
	bool handle_control_available = false; /* 手控是否可用由手柄型号和外控状态共同决定。 */
	bool touch_control_available = false; /* 触控是否可用由外控状态和有效手柄共同决定。 */
	bool touch_control_selected = false; /* 触控是否选中由回落后的当前控制方式决定，前提是触控可用。 */

	Pubinterface_ApplyEx8ControlModeFallback(); /* 每次重绘控制方式前先消除“脚踏离线但脚控仍选中”的状态残留。 */

	foot_control_available = (ControlSignalMessage.jt_enable_flag == true); /* 脚踏图标按实际脚踏在线标志显示，避免未接入时仍可用。 */
	external_control_active = (WorkMessage.hmiactive_work != 0U); /* 外控内部复用 TOUCHWORK 做互斥，但显示层必须和触控按钮分开。 */
	handle_control_available = ((external_control_active == false) && Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)); /* 只有 PXBA/PXBB/LGZII 等带手控入口型号才显示手控可用，避免通用磨钻误亮手控。 */
	touch_control_available = ((external_control_active == false) && (WorkMessage.hand_model != 0U)); /* 触控入口必须已有有效手柄才显示可用，避免无手柄时误亮。 */
	touch_control_selected = (touch_control_available && (WorkMessage.drivetype_work == TOUCHWORK)); /* 外控或无手柄时不再误点亮触控按钮。 */

	display_value[0] = 1U; /* 1 表示脚踏按钮，屏幕协议和 UICONTROLDP 保持一致。 */
	display_value[1] = (foot_control_available && (WorkMessage.drivetype_work == JTWORK)) ? 1U : 0U; /* 当前工作方式为脚踏且脚踏可用时高亮脚踏。 */
	SendUIDSMessage(UI_CONTROL_ID, foot_control_available, display_value); /* 切换脚控/手控后立即刷新脚踏按钮，未接入脚踏时保持暗态。 */

	display_value[0] = 2U; /* 2 表示手控按钮，屏幕点击手控后需要马上看到高亮反馈。 */
	display_value[1] = (handle_control_available && (WorkMessage.drivetype_work == HANDLEWORK)) ? 1U : 0U; /* 当前工作方式为手控且手控可用时高亮手控。 */
	SendUIDSMessage(UI_CONTROL_ID, handle_control_available, display_value); /* 非外控状态下手控保持白色可选，选中时由 display_value[1] 切黄色。 */

	display_value[0] = 3U; /* 3 表示触控按钮，本次只同步已有触控状态，不改变触控控制权。 */
	display_value[1] = touch_control_selected ? 1U : 0U; /* 触控高亮只代表屏幕触控，不再把外部通信控制算作触控。 */
	SendUIDSMessage(UI_CONTROL_ID, touch_control_available, display_value); /* 非外控状态下触控保持白色可选，进入触控后由 display_value[1] 切黄色。 */
}

/*
 * 函数功能：刷新主运行页外部通信小电脑图标。
 * 输入参数：connected_flag 表示外部通信链路是否有合法帧；active_flag 表示外部通信是否正在控制输出。
 * 返回参数：无。
 */
void Pubinterface_RefreshExternalCommDisplay(bool connected_flag, bool active_flag)
{
	uint8_t display_value[10] = {0U}; /* UI_CONTROL_ID 的 Value[0]=4 固定代表外部通信图标，Value[1] 表示是否黄色高亮。 */

	display_value[0] = 4U; /* 4 是新屏审查表中的外部通信/小电脑图标编号。 */
	display_value[1] = active_flag ? 1U : 0U; /* 申请成功但未输出时显示 39 白色，外控正在运行泵或手柄时显示 40 黄色。 */
	SendUIDSMessage(UI_CONTROL_ID, connected_flag, display_value); /* 退出外部通信时熄灭图标，在线状态按 active_flag 切换 39/40。 */
}

/*
 * 函数功能：取得屏幕和上位机需要显示的泵速度。
 * 输入参数：pump_message 当前泵状态指针。
 * 返回参数：运行态返回压力闭环后的实际输出速度，停止态返回 speed_work 设定速度。
 */
uint16_t Pubinterface_GetPumpDisplaySpeed(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针调用，显示 0 并避免心跳或屏幕刷新越界读取。 */
	}

	if (pump_message->run_flag || pump_message->timingDrainage_flag)
	{
		return pump_message->speed_output; /* 泵正在输出时显示闭环修正后的真实业务速度，避免高压限速后仍显示旧设定值。 */
	}

	return pump_message->speed_work; /* 泵停止时保留设定速度显示，避免修改待机调速和再次启动的用户体验。 */
}

/*
 * 函数功能：把泵运行状态同步到屏幕泵区域和泵启停按钮。
 * 输入参数：pump_area_id 为泵数值区域，button_area_id 为启停按钮区域，pump_message 为当前泵状态。
 * 返回参数：无。
 */
static void Pubinterface_SendPumpDisplay(uint8_t pump_area_id, uint8_t button_area_id, const pumpMessage_t *pump_message)
{
	uint8_t display_value[10] = {0U}; /* UIDP 队列固定拷贝 10 字节，泵刷新也使用同一缓冲格式。 */
	bool pump_available = (pump_message->type != 0U); /* 未识别到泵类型时暗掉区域，防止屏幕显示可控但泵任务没有有效设备。 */
	uint16_t display_speed = Pubinterface_GetPumpDisplaySpeed(pump_message); /* 运行态显示闭环后的实际输出速度，停止态继续显示设定速度。 */

	display_value[0] = (uint8_t)pump_message->type; /* Value[0] 传业务泵类型，UIPUMPADP/UIPUMPBDP 用它选择注水、灌注或抽吸图标。 */
	display_value[1] = (uint8_t)(display_speed >> 8); /* Value[1] 传显示速度高字节，闭环限速时会跟随实际输出变化。 */
	display_value[2] = (uint8_t)(display_speed & 0xFFU); /* Value[2] 传显示速度低字节，和 UIDP 现有解析顺序一致。 */
	SendUIDSMessage(pump_area_id, pump_available, display_value); /* 屏幕泵加减或启停后立刻刷新数值和档位。 */

	display_value[0] = (uint8_t)pump_message->type; /* 按钮刷新同样携带泵类型，按钮图标按泵类型显示不同资源。 */
	display_value[1] = pump_message->run_flag ? 1U : 0U; /* Value[1] 表示运行态，启动显示运行按钮，停止显示待启动按钮。 */
	display_value[2] = 0U; /* 按钮消息不使用速度低字节，清零避免复用上一次数值消息残留。 */
	if(pump_message->type==INJECTWATER&&display_speed<100)return;
	SendUIDSMessage(button_area_id, pump_available, display_value); /* 同步按钮黄/黑状态，补齐副工程屏幕交互反馈。 */
}

/*
 * 函数功能：刷新 A 泵屏幕区域和启停按钮。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpADisplay(void)
{
	Pubinterface_SendPumpDisplay(UI_PUMPA_ID, UI_PUMPABUTTON_ID, &pumpMessageA); /* A 泵状态只读取 pumpMessageA，保持主工程 A/B 泵闭环映射不变。 */
}

/*
 * 函数功能：刷新 B 泵屏幕区域和启停按钮。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpBDisplay(void)
{
	Pubinterface_SendPumpDisplay(UI_PUMPB_ID, UI_PUMPBBUTTON_ID, &pumpMessageB); /* B 泵状态只读取 pumpMessageB，保持主工程 A/B 泵闭环映射不变。 */
}

/*
 * 函数功能：更新 A 泵压力闭环后的实际输出速度，并按需刷新左侧泵显示。
 * 输入参数：output_speed A 泵本周期最终下发前的业务速度。
 * 返回参数：无。
 */
void Pubinterface_UpdatePumpAOutputSpeed(uint16_t output_speed)
{
	if (pumpMessageA.speed_output != output_speed)
	{
		pumpMessageA.speed_output = output_speed; /* 只记录闭环后的实际输出，不覆盖 speed_work，保证压力恢复后仍按原设定恢复。 */
		Pubinterface_RefreshPumpADisplay(); /* 实际输出速度变化才刷新屏幕，避免泵任务周期性发送重复 UI 消息。 */
	}
}

/*
 * 函数功能：更新 B 泵压力闭环后的实际输出速度，并按需刷新右侧泵显示。
 * 输入参数：output_speed B 泵本周期最终下发前的业务速度。
 * 返回参数：无。
 */
void Pubinterface_UpdatePumpBOutputSpeed(uint16_t output_speed)
{
	if (pumpMessageB.speed_output != output_speed)
	{
		pumpMessageB.speed_output = output_speed; /* B 泵同样把实际输出和设定值分开，保证上位机调速值不被闭环临时改写。 */
		Pubinterface_RefreshPumpBDisplay(); /* 变化时立即刷新右侧泵区，使压力闭环限速能实时显示。 */
	}
}

/*
 * 函数功能：向屏幕刷新单个方向按钮的可用和选中状态。
 * 输入参数：ui_dir_id 屏幕方向编号，enable_flag 是否可用，selected_flag 是否选中。
 * 返回参数：无。
 */
static void Pubinterface_SendDirectionDisplay(uint8_t ui_dir_id, bool enable_flag, bool selected_flag)
{
	uint8_t display_value[10] = {0U}; /* UI_DIR_ID 只使用 Value[0] 方向编号和 Value[1] 选中状态。 */

	display_value[0] = ui_dir_id; /* 1 表示正转，2 表示反转，3 表示往复，沿用 UIDP 协议。 */
	display_value[1] = selected_flag ? 1U : 0U; /* 选中状态只在 enable_flag 为 true 时生效，禁用时由 UIDP 暗掉图标。 */
	SendUIDSMessage(UI_DIR_ID, enable_flag, display_value); /* 切换手柄后立即重绘方向区，防止保留上一个通道状态。 */
}

/*
 * 函数功能：根据当前 WorkMessage 工作快照刷新屏幕通道参数显示。
 * 输入参数：channel 本次刚被选中的通道。
 * 返回参数：无。
 */
static void Pubinterface_RefreshSelectedChannelDisplay(uint8_t channel)
{
	uint8_t display_value[10] = {0U}; /* 所有 UI 消息都使用 10 字节缓冲，保持 UIDP 队列拷贝格式稳定。 */
	bool planer_selected = Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type); /* PLANER 或兼容 PXP 旧码才按刨刀能力显示往复相关入口。 */
	bool osc_supported = Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model, WorkMessage.tool_type); /* 往复方向只看刀具能力，磨头刀具不再因为基座支持就误亮往复。 */
	bool foot_control_available = (ControlSignalMessage.jt_enable_flag == true); /* 切通道刷新时脚踏图标也按实际在线状态显示。 */
	bool external_control_active = (WorkMessage.hmiactive_work != 0U); /* 外控占用时内部驱动方式也可能是 TOUCHWORK，但屏幕不能显示为触控。 */
	bool handle_control_available = ((external_control_active == false) && Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)); /* 切换 A/B 后也按当前手柄型号决定手控是否可用，避免通道刷新把禁用手控刷白。 */
	bool touch_control_available = ((external_control_active == false) && (WorkMessage.hand_model != 0U)); /* 切通道重绘也必须有有效手柄才开放触控入口。 */
	bool touch_control_selected = (touch_control_available && (WorkMessage.drivetype_work == TOUCHWORK)); /* 外控或无手柄时不再误高亮触控按钮。 */

	if (channel != WorkMessage.channel_work)
	{
		return; /* 只刷新刚装载为当前 WorkMessage 的通道，避免异步调用把非工作通道参数刷到屏幕。 */
	}

	display_value[0] = 1U; /* 控制模式 1 表示脚踏，UICONTROLDP 使用该编号绘制脚踏按钮。 */
	display_value[1] = (foot_control_available && (WorkMessage.drivetype_work == JTWORK)) ? 1U : 0U; /* 当前通道记忆为脚踏且脚踏可用时选中脚踏。 */
	SendUIDSMessage(UI_CONTROL_ID, foot_control_available, display_value); /* 切通道后刷新脚踏按钮，未接入脚踏时不显示为可用。 */

	display_value[0] = 2U; /* 控制模式 2 表示手控，和屏幕协议控制编号一致。 */
	display_value[1] = (handle_control_available && (WorkMessage.drivetype_work == HANDLEWORK)) ? 1U : 0U; /* 当前通道记忆为手控且手控可用时选中手控。 */
	SendUIDSMessage(UI_CONTROL_ID, handle_control_available, display_value); /* 切通道时也保持手控白色可选，外控占用时才置灰。 */

	display_value[0] = 3U; /* 控制模式 3 表示触控，切换通道时同步当前触控占用状态。 */
	display_value[1] = touch_control_selected ? 1U : 0U; /* 触控高亮只代表屏幕触控，不把外部通信控制算作触控。 */
	SendUIDSMessage(UI_CONTROL_ID, touch_control_available, display_value); /* 切通道时也保持触控白色可选，外控占用时才置灰。 */
	Pubinterface_RefreshToolDisplay(channel, planer_selected); /* 所有通道都刷新刀具区；普通手柄会主动隐藏 RFID 区，避免 A/B 切换残留。 */

	if (osc_supported)
	{
		Pubinterface_SendDirectionDisplay(3U, true, (WorkMessage.dir_work == OSCDIR)); /* 当前手柄支持往复时开放往复按钮，按当前方向决定是否选中。 */
	}
	else
	{
		Pubinterface_SendDirectionDisplay(3U, false, false); /* 当前手柄不支持往复时禁用往复按钮，避免屏幕误发后进入 OSCDIR。 */
	}
	Pubinterface_SendDirectionDisplay(1U, true, (WorkMessage.dir_work == ZZDIR)); /* 正转按钮始终可显示，选中状态跟随当前通道方向。 */
	Pubinterface_SendDirectionDisplay(2U, true, (WorkMessage.dir_work == FZDIR)); /* 反转按钮始终可显示，选中状态跟随当前通道方向。 */

	if (osc_supported && (WorkMessage.dir_work == OSCDIR))
	{
		display_value[0] = (uint8_t)WorkMessage.freq_work; /* 往复方向下显示当前通道频率值，频率范围已在识别/按键逻辑限制。 */
		display_value[1] = 0U; /* 0 表示刷新静态值，运行中字体颜色仍由速度/运行消息单独控制。 */
		SendUIDSMessage(UI_FREQ_ID, true, display_value); /* 手柄支持往复且已切到往复时打开频率窗口，匹配屏幕刷新流程。 */
	}
	else
	{
		SendUIDSMessage(UI_FREQ_ID, false, display_value); /* 非往复或手柄不支持往复时关闭频率窗口，清掉上一个通道残留频率。 */
	}

	display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位沿用 WorkMessage 的 x10 速度。 */
	display_value[1] = (uint8_t)((WorkMessage.speed_set_work >>8)&0xFFU); /* 速度低字节按 UIDP 协议传输，保证 16 位速度完整显示。 */
	display_value[2] = (uint8_t)(WorkMessage.speed_set_work & 0xFFU);
	display_value[3] = 0U; /* 0 表示切通道后的静态速度刷新，不进入运行中颜色更新分支。 */
	display_value[4] = 0U; /* 切通道时电机未启动，速度字体保持非运行状态。 */
	SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 最后刷新速度值，保证切通道后的速度显示同步。 */
}

void ChannelrecognizeMessageInit(void)
{
	memset(&ChannelrecognizeMessageA, 0, sizeof(ChannelrecognizeMessageA));
	memset(&ChannelrecognizeMessageB, 0, sizeof(ChannelrecognizeMessageB));
}
void WorkMessageInit(void)
{
	memset(&WorkMessage, 0, sizeof(WorkMessage));
}
void ChannelMemoryMessageInit(void)
{
	memset(&MemoryMsgA, 0, sizeof(MemoryMsgA));
	memset(&MemoryMsgB, 0, sizeof(MemoryMsgB));
}
void pumpMessageInit(void)
{
	memset(&pumpMessageA, 0, sizeof(pumpMessageA));
	memset(&pumpMessageB, 0, sizeof(pumpMessageB));
}

/*
 * 函数功能：按照“手柄运行则冷却出口识别为注水泵时运行、手柄停止则联动泵停止”的规则刷新冷却泵状态。
 * 输入参数：enable 为 true 表示手柄已经进入运行态，需要在冷却出口识别为注水泵时启动冷却；false 表示手柄已经停止，需要同步关闭联动泵。
 * 返回参数：无。
 */
void Pubinterface_SetHandleInjectionPumpRun(bool enable)
{
	bool a_is_injection = (pumpMessageA.type == INJECTWATER); /* 读取 A 泵当前设备码识别出的业务类型，只有注水泵允许参与手柄冷却。 */
	bool b_is_injection = (pumpMessageB.type == INJECTWATER); /* 读取 B 泵当前设备码识别出的业务类型，支持 B 泵作为唯一注水泵时跨通道跟随。 */
	uint8_t target_mask = 0U;								 /* 本周期手柄冷却应该驱动的泵位，0 表示没有可用注水泵或手柄已停止。 */

	if (enable)
	{
		if (a_is_injection && b_is_injection)
		{
			if (WorkMessage.channel_work == CHANNEL_A)
			{
				target_mask = HANDLE_INJECTION_FOLLOW_PUMP_A; /* 两个泵都是注水泵且当前 A 手柄运行时，只让 A 注水泵给 A 通道手柄降温。 */
			}
			else if (WorkMessage.channel_work == CHANNEL_B)
			{
				target_mask = HANDLE_INJECTION_FOLLOW_PUMP_B; /* 两个泵都是注水泵且当前 B 手柄运行时，只让 B 注水泵给 B 通道手柄降温。 */
			}
		}
		else if (a_is_injection)
		{
			target_mask = HANDLE_INJECTION_FOLLOW_PUMP_A; /* 只有 A 泵是注水泵时，不管当前运行 A/B 手柄，都由 A 泵承担冷却供水。 */
		}
		else if (b_is_injection)
		{
			target_mask = HANDLE_INJECTION_FOLLOW_PUMP_B; /* 只有 B 泵是注水泵时，A/B 任一手柄运行都必须启动 B 泵给手柄降温。 */
		}
	}

	if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		pumpMessageA.timingDrainage_flag = false; /* 手柄运行联动不是定时排空，先清 A 排空标志，避免后台排空计时抢写输出。 */
		pumpMessageA.timingDrainage_times = 0U;   /* A 排空计数清零，保证后续真正进入排空时从完整周期开始。 */

		//pumpMessageA.speed_work = HANDLE_INJECTION_PUMP_DEFAULT_FLOW; /* A 冷却流量每次手柄启动都重装固定值，避免继承上次调泵速度。 */
		pumpMessageA.run_flag = true;			   /* 打开 A 泵任务运行门控，下一周期按注水公式和压力闭环输出。 */
		s_handle_injection_pump_follow_mask |= HANDLE_INJECTION_FOLLOW_PUMP_A; /* 记录 A 泵由手柄冷却跟随启动，停止时才允许本函数释放。 */
	}
	else if ((s_handle_injection_pump_follow_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		pumpMessageA.run_flag = false;			 /* 当前手柄不再需要 A 冷却时，只关闭本函数曾启动过的 A 跟随输出。 */
		pumpMessageA.timingDrainage_flag = false; /* 同步退出 A 排空状态，保持手柄冷却和脚踏排空互斥。 */
		pumpMessageA.timingDrainage_times = 0U;   /* A 排空计数清零，避免下一轮排空继承旧计数。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_A); /* 清掉 A 跟随占用标记，后续停止手柄不再重复改写 A 泵。 */
	}

	if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		pumpMessageB.timingDrainage_flag = false; /* 手柄运行联动不是定时排空，先清 B 排空标志，避免后台排空计时抢写输出。 */
		pumpMessageB.timingDrainage_times = 0U;   /* B 排空计数清零，保证后续真正进入排空时从完整周期开始。 */
		//pumpMessageB.speed_work = HANDLE_INJECTION_PUMP_DEFAULT_FLOW; /* B 冷却流量每次手柄启动都重装固定值，保证 B 注水泵跨通道跟随时不为 0。 */
		pumpMessageB.run_flag = true;			   /* 打开 B 泵任务运行门控，下一周期按注水公式和压力闭环输出。 */
		s_handle_injection_pump_follow_mask |= HANDLE_INJECTION_FOLLOW_PUMP_B; /* 记录 B 泵由手柄冷却跟随启动，停止时才允许本函数释放。 */
	}
	else if ((s_handle_injection_pump_follow_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		pumpMessageB.run_flag = false;			 /* 当前手柄不再需要 B 冷却时，只关闭本函数曾启动过的 B 跟随输出。 */
		pumpMessageB.timingDrainage_flag = false; /* 同步退出 B 排空状态，保持手柄冷却和脚踏排空互斥。 */
		pumpMessageB.timingDrainage_times = 0U;   /* B 排空计数清零，避免下一轮排空继承旧计数。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_B); /* 清掉 B 跟随占用标记，后续停止手柄不再重复改写 B 泵。 */
	}
}

/*
 * 函数功能：读取指定通道从 EEPROM Page4 解析出的默认注水流量，供手柄上线、脚踏和外控默认启动注水泵共用。
 * 输入参数：channel 通道号，CHANNEL_A 读取 A 通道记忆，CHANNEL_B 读取 B 通道记忆。
 * 返回参数：默认注水流量，单位为当前泵业务流量值；通道无效时返回 0。
 */
uint16_t Pubinterface_GetChannelDefaultInjectionFlow(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return MemoryMsgA.default_injection_flow; /* A 通道返回本通道 Page4 默认流量，避免 A/B 手柄默认流量串用。 */
	}

	if (channel == CHANNEL_B)
	{
		return MemoryMsgB.default_injection_flow; /* B 通道返回本通道 Page4 默认流量，避免 A/B 手柄默认流量串用。 */
	}

	return 0U; /* 当前没有有效通道时不补注水默认值，避免无手柄时误启动注水泵。 */
}

/*
 * 函数功能：读取当前工作通道的默认注水流量。
 * 输入参数：无，函数内部读取 WorkMessage.channel_work。
 * 返回参数：当前工作通道默认注水流量；当前没有 A/B 工作通道时返回 0。
 */
uint16_t Pubinterface_GetCurrentDefaultInjectionFlow(void)
{
	return Pubinterface_GetChannelDefaultInjectionFlow(WorkMessage.channel_work); /* 注水默认流量跟随当前工作手柄，而不是固定跟随某个泵口。 */
}

/*
 * 函数功能：取得注水泵启动时使用的非零流量，优先使用当前手柄 Page4，Page4 为空时回退到联调安全流量。
 * 输入参数：无。
 * 返回参数：可直接写入 pumpMessageA/B.speed_work 的非零注水泵启动流量。
 */
uint16_t Pubinterface_GetInjectionPumpStartFlow(void)
{
	uint16_t flow = Pubinterface_GetCurrentDefaultInjectionFlow(); /* 先读当前通道 Page4 默认注水流量，保证有配置时仍以 EEPROM 为准。 */

	if (flow == 0U)
	{
		flow = HANDLE_INJECTION_PUMP_DEFAULT_FLOW; /* Page4 未装载或默认流量为 0 时，启动泵必须有兜底流量，否则 run_flag 置位也不会转。 */
	}

	return flow; /* 返回最终启动流量，调用方只负责写入对应 A/B 泵速度。 */
}

/*
 * 函数功能：按泵业务类型取得 0 速启动时可直接写入 speed_work 的兜底速度。
 * 输入参数：pump_message 指向 A/B 泵运行状态，函数只读取 type 字段判断泵类型。
 * 返回参数：有效泵类型返回非零启动速度；无效类型或空指针返回 0。
 */
uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针，避免异常调用把未知泵类型误补成有效速度。 */
	}

	switch (pump_message->type)
	{
	case DRAWWATER:
		return PUMP_DRAWWATER_START_SPEED; /* 抽吸泵从一档小流量启动，避免 0 速只亮运行态。 */
	case INJECTWATER:
		return Pubinterface_GetInjectionPumpStartFlow(); /* 注水泵仍沿用当前通道 Page4 默认流量和 70ml 兜底策略。 */
	case POURWATER:
		return PUMP_POURWATER_START_SPEED; /* 灌注泵从一档 50ml 启动，解决屏幕直接启动时不转的问题。 */
	default:
		return 0U; /* 未识别泵不能补速度，避免屏幕或上位机误启动未知设备。 */
	}
}

/*
 * 函数功能：取得屏幕加减速使用的泵速度上限。
 * 输入参数：pump_message 指向 A/B 泵运行状态，优先读取已配置 speed_Max，未配置时按泵类型回退。
 * 返回参数：当前泵类型允许的最大速度；未知类型返回 0。
 */
static uint16_t Pubinterface_GetPumpSpeedMax(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针，防止调速入口异常时写出不可控速度。 */
	}

	if (pump_message->speed_Max != 0U)
	{
		return pump_message->speed_Max; /* 若后续 EEPROM 或参数表配置了上限，则优先服从配置值。 */
	}

	switch (pump_message->type)
	{
	case DRAWWATER:
		return PUMP_DRAWWATER_SPEED_MAX; /* 抽吸泵上限与泵任务 15 档限幅一致。 */
	case INJECTWATER:
		return PUMP_INJECTWATER_SPEED_MAX; /* 注水泵上限与 pump.h 中统一宏一致。 */
	case POURWATER:
		return PUMP_POURWATER_SPEED_MAX; /* 灌注泵上限与泵任务 300ml 限幅一致。 */
	default:
		return 0U; /* 未识别泵类型不允许加速，避免残留步进值改坏状态。 */
	}
}

/*
 * 函数功能：取得屏幕加减速使用的泵速度下限。
 * 输入参数：pump_message 指向 A/B 泵运行状态，读取 speed_Min 作为配置下限。
 * 返回参数：当前泵速度下限；空指针返回 0。
 */
static uint16_t Pubinterface_GetPumpSpeedMin(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针，下限按 0 处理可保证不会出现负速度。 */
	}

	return pump_message->speed_Min; /* 当前工程未初始化 speed_Min 时自然为 0，后续配置非零时可自动生效。 */
}

/*
 * 函数功能：读取当前工作通道当前方向的默认电机速度，供脚踏启动时替代旧的固定 60000。
 * 输入参数：无，函数内部读取 WorkMessage.channel_work 和 WorkMessage.dir_work。
 * 返回参数：当前方向默认速度，单位沿用 WorkMessage.speed_set_work 的 x10 单位；无有效通道时返回 0。
 */
uint16_t Pubinterface_GetCurrentDefaultMotorSpeed(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前工作通道记忆结构，后续按方向读取该通道 Page4 默认速度。 */

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 当前工作通道为 A 时，默认速度来自 A 通道 Page4 解析结果。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 当前工作通道为 B 时，默认速度来自 B 通道 Page4 解析结果。 */
	}
	else
	{
		return 0U; /* 无工作通道时不能给电机补默认速度，避免未选中手柄时误运行。 */
	}

	if (WorkMessage.dir_work == FZDIR)
	{
		return memory->fz_speed; /* 反转时返回 Page4 默认速度装载到反转记忆后的值。 */
	}

	if (WorkMessage.dir_work == OSCDIR)
	{
		return memory->osc_speed; /* 往复时返回 Page4 默认速度装载到往复记忆后的值。 */
	}

	return memory->zz_speed; /* 默认按正转处理，和 Page4 默认方向正转保持一致。 */
}

/*
 * 函数功能：手柄上线时把当前通道 Page4 默认注水流量写入仍为 0 的注水泵速度。
 * 输入参数：channel 本次上线的手柄通道。
 * 返回参数：无。
 */
static void Pubinterface_ApplyChannelDefaultInjectionFlow(uint8_t channel)
{
	uint16_t default_flow = Pubinterface_GetChannelDefaultInjectionFlow(channel); /* 读取本通道 Page4 默认注水流量，解析阶段已经完成 /10 和 0~70 钳位。 */

	if ((pumpMessageA.type == INJECTWATER) && (pumpMessageA.speed_work == 0U))
	{
		pumpMessageA.speed_work = default_flow; /* A 泵是注水泵且尚未设置流量时，用当前手柄默认流量初始化。 */
	}

	if ((pumpMessageB.type == INJECTWATER) && (pumpMessageB.speed_work == 0U))
	{
		pumpMessageB.speed_work = default_flow; /* B 泵是注水泵且尚未设置流量时，用当前手柄默认流量初始化。 */
	}
}

/*
 * 函数功能：判断报警码是否属于手柄 EEPROM 识别/校验失败系列报警。
 * 输入参数：alarm_value 当前报警码，来自 WorkMessage.alarm_value 或临时提示报警。
 * 返回参数：true 表示这是 A/B/AB 手柄校验报警，false 表示普通阻塞报警。
 */
bool Pubinterface_IsHandleVerifyAlarm(uint8_t alarm_value)
{
	return ((alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_A) ||  /* A 通道 EEPROM 校验失败时，不阻塞 B 通道继续识别。 */
			(alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_B) ||  /* B 通道 EEPROM 校验失败时，不阻塞 A 通道继续识别。 */
			(alarm_value == WORK_ALARM_HANDLE_MODEL_ERROR_AB));  /* A/B 均失败时仍属于手柄校验系列，拔出任一坏手柄后再降级报警。 */
}

/*
 * 函数功能：把指定通道的 MemoryMsg 记忆参数装载到当前 WorkMessage 工作快照。
 * 输入参数：channel 目标通道，CHANNEL_A 表示装载 MemoryMsgA，CHANNEL_B 表示装载 MemoryMsgB。
 * 返回参数：无。
 */
void Pubinterface_LoadChannelMemory(uint8_t channel)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向即将成为当前工作快照的通道记忆结构。 */

	if (channel == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 手动或自动切到 A 通道时，所有运行参数都来自 A 通道记忆。 */
	}
	else if (channel == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 手动或自动切到 B 通道时，所有运行参数都来自 B 通道记忆。 */
	}
	else
	{
		return; /* 无效通道不改 WorkMessage，避免把当前有效工作态清成不可预期状态。 */
	}

	WorkMessage.current_work = memory->current_work;			  /* 同步当前通道保护电流，后续启动时按该手柄限流。 */
	WorkMessage.tool_reduction_ratio = memory->tool_reduction_ratio; /* 同步刀具减速比，保证速度换算跟随通道。 */
	WorkMessage.dir_work = memory->dir;						  /* 同步当前方向，速度选择依赖这个方向字段。 */
	if(ControlSignalMessage.jt_enable_flag==true)
	WorkMessage.drivetype_work = memory->drive_type=JTWORK;			  /* 同步控制方式，切换通道后脚控/手控状态来自通道记忆。 */
	else
	{
		WorkMessage.drivetype_work = memory->drive_type;
	}
	WorkMessage.freq_work = memory->freq;						  /* 同步往复频率，避免 A/B 频率串用。 */
	WorkMessage.tool_type = memory->tool_type;					  /* 同步刨/磨刀具类型，界面和限速逻辑都读取这里。 */
	WorkMessage.raw_tool_type = (memory->tool_type != 0U) ? memory->raw_tool_type : 0U; /* 当前工作快照只在有效刀具存在时保留原始码，避免掉线历史误导驱动。 */
	WorkMessage.auto_identify = memory->auto_identify;			  /* 同步当前通道自动识别状态，切通道后 RFID 识别模式不丢失。 */
	WorkMessage.hand_model = memory->hand_model;				  /* 同步手柄型号，手柄按键扫描依赖当前型号判断。 */
	WorkMessage.channel_work = channel;						  /* 最后切换当前工作通道，避免中间状态被其它任务读成新通道旧参数。 */
	Pubinterface_ApplyEx8ControlModeFallback();				  /* EX8 要求脚踏离线时 PXBA/PXBB/LGZII 切通道后直接落到手控选中。 */

	if (WorkMessage.dir_work == ZZDIR)
	{
		WorkMessage.speed_set_work = memory->zz_speed;			  /* 正转时装载本通道正转速度。 */
	}
	else if (WorkMessage.dir_work == FZDIR)
	{
		WorkMessage.speed_set_work = memory->fz_speed;			  /* 反转时装载本通道反转速度。 */
	}
	else
	{
		WorkMessage.speed_set_work = memory->osc_speed;		  /* 往复或异常方向按往复速度装载，保持旧逻辑兼容。 */
	}

	WorkMessage.speed_work = WorkMessage.speed_set_work;		  /* 切换通道后实际速度同步到设置速度，避免沿用上一个通道输出值。 */
}

/*
 * 函数功能：判断新插入且校验通过的通道是否允许自动成为当前选中通道。
 * 输入参数：channel 本次插入并已写入 MemoryMsg 的通道。
 * 返回参数：true 表示可以自动选中，false 表示只更新通道记忆等待用户手动切换。
 */
static bool Pubinterface_ShouldAutoSelectPluggedChannel(uint8_t channel)
{
	if (((channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == false)) ||
		((channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == false)))
	{
		return false; /* 插拔事件还没有把在线标志置位时，不允许装载该通道，避免选中一个未上线通道。 */
	}

	if (WorkMessage.runflag_work == false)
	{
		if ((WorkMessage.alarm_flag == true) && (Pubinterface_IsHandleVerifyAlarm(WorkMessage.alarm_value) == false))
		{
			return false; /* 普通系统报警期间暂停自动选中，等待报警解除后再由用户确认。 */
		}

		return true; /* 非运行状态下，最后一个校验通过的插入通道自动成为选中通道。 */
	}

	return false; /* 运行中插入另一路有效手柄只更新 MemoryMsg，不抢占当前工作通道。 */
}

/*
 * 函数功能：把扫描层 ChannelrecognizeMessage 的识别结果写入对应 MemoryMsg 通道记忆。
 * 输入参数：channel 本次插入并认证通过的通道。
 * 返回参数：无。
 */
static void Pubinterface_SaveRecognizeToMemory(uint8_t channel)
{
	ChannelrecognizeMessage_t *recognize = NULL; /* 指向扫描层刚刚完成认证的临时识别缓存。 */
	ChannelMemoryMessagr_t *memory = NULL;		  /* 指向要更新的 A/B 通道记忆结构。 */
	uint8_t keep_auto_identify = 0U;				  /* RFID 二次刷新前保留通道自动识别状态，避免新刀具参数覆盖该模式记忆。 */

	if (channel == CHANNEL_A)
	{
		recognize = &ChannelrecognizeMessageA;	  /* A 通道上线时，只读取 A 通道扫描缓存。 */
		memory = &MemoryMsgA;					  /* A 通道上线时，只写入 A 通道记忆。 */
	}
	else if (channel == CHANNEL_B)
	{
		recognize = &ChannelrecognizeMessageB;	  /* B 通道上线时，只读取 B 通道扫描缓存。 */
		memory = &MemoryMsgB;					  /* B 通道上线时，只写入 B 通道记忆。 */
	}
	else
	{
		return;									  /* 非 A/B 通道不写通道记忆，避免破坏当前工作态。 */
	}

	keep_auto_identify = memory->auto_identify;	  /* 识别缓存只包含刀具/手柄参数，自动识别模式属于通道 UI 记忆，需要单独保留。 */
	if ((recognize->handle_type != 0U) && (recognize->tool_type == 0U) && (Pubinterface_IsSplitToolSpecDisplayModel(recognize->handle_type) == true))
	{
		keep_auto_identify = 1U; /* PXBA/PXBB 新插入基座默认打开自动识别，直到用户主动切回手动模式。 */
		Pubinterface_SetLastRfidToolType(channel, 0U); /* 新基座上线但还没有刀具结果时，0x1404 必须显示等待图 63，不沿用旧掉线图。 */
	}
	memset(memory, 0, sizeof(*memory));			  /* 新手柄上线前清掉该通道旧 EEPROM 参数，避免旧字段残留参与后续切换。 */
	memory->hand_model = recognize->handle_type;  /* 保存手柄型号，后续通道切换时再装载到 WorkMessage。 */
	memory->hand_type_raw_major = recognize->hand_type_raw_major; /* 保存 EEPROM 原始主类型，上位机心跳需要区分真实编码。 */
	memory->hand_type_raw_minor = recognize->hand_type_raw_minor; /* 保存 EEPROM 原始子类型，便于上位机显示和售后定位。 */
	memory->current_work = recognize->overloadThresholdFor;		  /* 保存保护电流，当前通道被选中后才影响 WorkMessage。 */
	memory->tool_reduction_ratio = (recognize->tool_reduction_ratio != 0U) ? recognize->tool_reduction_ratio : (uint32_t)recognize->meioticratio; /* RFID 手柄保存完整 32 位减速比，普通 EEPROM 手柄继续兼容旧 8 位减速比。 */
	memory->default_injection_flow = recognize->default_injection_flow; /* 保存 Page4 默认注水流量，选中该通道时初始化泵。 */
	memory->speed_alarm_for = recognize->speed_alarm_for;		  /* 保存正转速度报警阈值，运行阈值检查按当前通道读取。 */
	memory->speed_alarm_rev = recognize->speed_alarm_rev;		  /* 保存反转速度报警阈值，运行阈值检查按当前通道读取。 */
	memory->freq_alarm_osc = recognize->freq_alarm_osc;			  /* 保存往复频率报警阈值，运行阈值检查按当前通道读取。 */
	memory->dir = recognize->run_direction;						  /* 保存默认方向，后续装载 WorkMessage 时据此选速度。 */
	memory->freq = recognize->freq_default;						  /* 保存默认往复频率，避免插入未选中通道时改当前频率。 */
	memory->zz_speed = recognize->speed_zzdefault;				  /* 保存正转默认速度，只影响本通道记忆。 */
	memory->fz_speed = recognize->speed_fzdefault;				  /* 保存反转默认速度，只影响本通道记忆。 */
	memory->osc_speed = recognize->speed_oscdefault;			  /* 保存往复默认速度，只影响本通道记忆。 */
	memory->tool_type = recognize->tool_type;					  /* 保存刀具类型，切换到该通道后再影响当前工具显示。 */
	memory->raw_tool_type = recognize->raw_tool_type;			  /* 保存原始刀具型号，驱动和上位机仍可按旧码继续判断。 */
	memory->auto_identify = keep_auto_identify;					  /* 恢复自动识别模式记忆，避免 RFID 结果刷新把屏幕选择状态清掉。 */
	if (memory->tool_type != 0U)
	{
		Pubinterface_SetLastRfidToolType(channel, memory->tool_type); /* RFID/EEPROM 成功写入刀具后，按业务类型更新掉线图标，避免原始标签代号无法映射 61/62。 */
	}

	if (Pubinterface_IsHandleControlReservedModel(memory->hand_model))
	{
		ControlSignalMessage.HMI_enable_flag = true;			  /* 带手控入口手柄上线后打开手控可用显示，新增型号只开放 UI 预留，不改实体键扫描来源。 */
		if ((WorkMessage.drivetype_work == TOUCHWORK) || (WorkMessage.drivetype_work == JTWORK))
		{
			memory->drive_type = WorkMessage.drivetype_work;	  /* 当前处于触控或脚控时，新通道记忆跟随当前控制方式，避免自动改手控。 */
		}
		else
		{
			if(ControlSignalMessage.jt_enable_flag==true)
			{
				memory->drive_type = JTWORK;					  /* 脚控使能时，带按键手柄默认记忆为脚控，避免自动切手控后用户找不到控制入口。 */
			}
			else
			memory->drive_type = HANDLEWORK;					  /* 没有其它控制方式占用时，带按键手柄默认记忆为手控。 */
		}
	}
	else if (Pubinterface_IsCommonSocketReservedModel(memory->hand_model))
	{
		memory->drive_type = WorkMessage.drivetype_work;		  /* 公共接头没有实体键，识别上线只继承当前控制方式，不能自动切入手控。 */
	}
	else
	{
		memory->drive_type = WorkMessage.drivetype_work;		  /* 非按键手柄保留当前控制方式记忆，切换通道时不额外改变控制来源。 */
	}
}

/*
 * 函数功能：可拆刀具头射频监测确认缺失时，只清除刀具头相关参数并保留手柄基座在线。
 * 输入参数：channel 目标通道，CHANNEL_A 清 A 通道刀具头，CHANNEL_B 清 B 通道刀具头。
 * 返回参数：无。
 */
void Pubinterface_ClearRfidToolMemory(uint8_t channel)
{
	ChannelrecognizeMessage_t *recognize = NULL; /* 指向扫描层通道识别缓存，用于清掉心跳有效性判断会读取的刀具字段。 */
	ChannelMemoryMessagr_t *memory = NULL;		  /* 指向通道记忆；RFID 刀具头离线时仍要保留上次可运行参数，供切回通道继续使用。 */
	uint8_t keep_drive_type = WorkMessage.drivetype_work; /* 保留原控制方式记忆，刀具头移开不应把脚控/触控/外控显示重置。 */
	uint8_t keep_auto_identify = WorkMessage.auto_identify; /* 保留用户选择的自动/手动识别模式，刀具掉线不应改模式或清通道记忆。 */
	uint8_t keep_raw_tool_type = 0U; /* 保留最近一次原始刀具型号，避免扫描层清零后通道记忆丢失驱动兼容字段。 */
	uint8_t keep_display_tool_type = 0U; /* 保留最近一次业务刀具类型，显示 61/62 必须使用 PLANER/GRINDH 语义。 */

	if (channel == CHANNEL_A)
	{
		recognize = &ChannelrecognizeMessageA; /* A 通道 RFID 刀具头缺失时，只处理 A 通道扫描缓存。 */
		memory = &MemoryMsgA;				  /* A 通道 RFID 刀具头缺失时，只处理 A 通道记忆。 */
		memset(paoxueSpeciValue_A, 0, sizeof(paoxueSpeciValue_A)); /* 清 A 通道规格缓存，屏幕和上位机不再看到旧刀具尺寸。 */
	}
	else if (channel == CHANNEL_B)
	{
		recognize = &ChannelrecognizeMessageB; /* B 通道 RFID 刀具头缺失时，只处理 B 通道扫描缓存。 */
		memory = &MemoryMsgB;				  /* B 通道 RFID 刀具头缺失时，只处理 B 通道记忆。 */
		memset(paoxueSpeciValue_B, 0, sizeof(paoxueSpeciValue_B)); /* 清 B 通道规格缓存，避免 B 刀具头移开后规格残留。 */
	}
	else
	{
		return; /* 非 A/B 通道不改任何全局状态，避免异常调用破坏当前工作通道。 */
	}

	if ((recognize == NULL) || (memory == NULL))
	{
		return; /* 防御空指针，虽然上面分支已覆盖，仍避免后续维护误改造成异常访问。 */
	}

	if (memory->drive_type != 0U)
	{
		keep_drive_type = memory->drive_type; /* 优先保留该通道原本的控制方式记忆，防止清刀具头时改变控制模式显示。 */
	}
	keep_auto_identify = memory->auto_identify; /* 清通道记忆前先保存识别模式，避免 memset 后丢失用户选择。 */
	keep_raw_tool_type = (memory->raw_tool_type != 0U) ? memory->raw_tool_type : memory->tool_type; /* 优先保留原始标签码，没有原始码时回退当前能力值。 */
	keep_display_tool_type = memory->tool_type; /* 掉线图标只依赖业务刀具类型，原始标签码不直接参与图片映射。 */

	recognize->tool_type = 0U;				   /* 清扫描层刀具类型，心跳有效性判断会因此停止追加 RFID 刀具块。 */
	recognize->raw_tool_type = 0U;			   /* 清扫描层原始刀具型号，避免下一次自动识别前误认成旧标签。 */
	recognize->diameter = 0U;				   /* 清扫描层直径，避免后续普通刀具完整性判断误认为仍有刀具。 */
	recognize->length = 0U;					   /* 清扫描层长度，避免屏幕或上位机沿用旧尺寸。 */
	recognize->draw = 0U;					   /* 清扫描层角度，刀具头缺失时角度无效。 */
	recognize->meioticratio = 0U;			   /* 清旧 8 位减速比，避免开口定位继续按旧刀具比例判断。 */
	recognize->tool_reduction_ratio = 0U;	   /* 清完整 32 位减速比，确保 PXBA/PXBB 工具参数不再残留。 */
	recognize->speed_zzdefault = 0U;			   /* 只清扫描层正转默认速度，让心跳知道刀具头已离线；MemoryMsg 正转速度继续保留。 */
	recognize->speed_fzdefault = 0U;			   /* 只清扫描层反转默认速度，避免上位机把离线刀具头当作当前在线参数。 */
	recognize->speed_oscdefault = 0U;		   /* 只清扫描层往复默认速度；切回通道时仍从 MemoryMsg 恢复上次 RFID 速度。 */
	recognize->speed_zzstep = 0U;			   /* 清扫描层正转调速步进，下一次识别成功后再由 RFID/Page6 刷新。 */
	recognize->speed_fzstep = 0U;			   /* 清扫描层反转调速步进，避免离线刀具头继续作为在线规格上报。 */
	recognize->speed_oscstep = 0U;			   /* 清扫描层往复调速步进，通道运行记忆不在这里清。 */
	recognize->speed_min = 0U;				   /* 清扫描层速度下限，上位机无当前刀具头时不显示在线限速范围。 */
	recognize->speed_max = 0U;				   /* 清扫描层速度上限，下一次 RFID 成功后重新写入。 */
	recognize->default_injection_flow = 0U;	   /* 清扫描层刀具默认泵流量，MemoryMsg 中上次运行泵流量继续保留。 */

	memory->hand_model = recognize->handle_type; /* 只刷新基座型号，刀具头缺失不代表手柄基座离线，也不能清上次刀具运行参数。 */
	memory->hand_type_raw_major = recognize->hand_type_raw_major; /* 保留 EEPROM Page2 原始主类型，上位机仍能显示 PXBA/PXBB 基座。 */
	memory->hand_type_raw_minor = recognize->hand_type_raw_minor; /* 保留 EEPROM Page2 原始子类型，避免基座类型在心跳中消失。 */
	memory->drive_type = keep_drive_type;		   /* 恢复该通道控制方式记忆，只清扫描层刀具头在线状态，不改变用户控制来源。 */
	memory->auto_identify = keep_auto_identify;	   /* 保留原有自动识别记忆状态，掉线后继续沿用用户选中的模式。 */
	memory->raw_tool_type = keep_raw_tool_type;	   /* 通道记忆继续保留最近一次原始刀具型号，切回通道和驱动兼容判断仍可使用。 */

	if (WorkMessage.channel_work == channel)
	{
		/* 当前工作快照继续保留上次 RFID 刀具参数，保证不切通道时仍可按上次识别速度运行。 */
		Pubinterface_RefreshSelectedChannelDisplay(channel); /* 立即刷新选中通道参数区，事件队列延迟时屏幕也先清旧刀具。 */
	}

	if (channel == CHANNEL_A)
	{
		s_rfid_last_tool_type_a = keep_display_tool_type; /* A 通道掉线后保留最近一次业务刀具类型，重新进入自动识别前可显示 61/62。 */
	}
	else if (channel == CHANNEL_B)
	{
		s_rfid_last_tool_type_b = keep_display_tool_type; /* B 通道同样保留最近一次业务刀具类型，避免掉线图标丢失。 */
	}

	Pubinterface_RefreshOnlineHandleDisplay();		   /* 基座仍在线时刷新在线图标，保证只清刀具头不误暗灭手柄区域。 */
}

/*
 * 函数功能：按 Page4 的 2 字节速度报警阈值计算 WorkMessage 内部 x10 速度阈值。
 * 输入参数：alarm_value Page4 正转/反转速度报警阈值，单位与 WorkMessage.speed_work 一致，例如 55000 表示 5500.0rpm。
 * 返回参数：可与 WorkMessage.speed_work 直接比较的 x10 速度阈值；0 表示阈值关闭。
 */
static uint16_t Pubinterface_BuildSpeedAlarmThreshold(uint16_t alarm_value)
{
	return alarm_value; /* Page4 已按内部 x10 速度保存，直接返回可避免 1 字节截断导致阈值偏低。 */
}

/*
 * 函数功能：检查当前运行速度或往复频率是否触发 Page4 阈值，触发时只驱动蜂鸣器，不强制停机。
 * 输入参数：无，函数内部读取 WorkMessage 和当前通道记忆结构。
 * 返回参数：无。
 */
void Pubinterface_CheckSpeedThresholdAlarm(void)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前工作通道的 Page4 阈值记忆，用于区分 A/B 手柄。 */
	uint16_t threshold_alarm = 0U;		   /* 保存当前方向下 Page4 配置的速度/频率报警阈值，0 表示不报警。 */
	uint8_t should_beep = 0U;			   /* 保存本周期是否需要速度/频率阈值蜂鸣。 */

	if (WorkMessage.alarm_flag)
	{
		s_speed_threshold_beep_active = 0U;		   /* 真实系统报警已经接管蜂鸣时，本提示报警让位，避免清掉其它报警蜂鸣。 */
		s_speed_threshold_beep_refresh_ticks = 0U; /* 系统报警期间清掉本模块重发计数，恢复后重新判断阈值。 */
		return;
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* 当前工作通道为 A 时，使用 A 通道 Page4 阈值。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* 当前工作通道为 B 时，使用 B 通道 Page4 阈值。 */
	}
	else
	{
		memory = NULL; /* 无工作通道时没有可比较的 Page4 阈值。 */
	}

	if ((WorkMessage.runflag_work == true) && (memory != NULL))
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			threshold_alarm = memory->speed_alarm_for; /* 正转使用 Page4 正转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= Pubinterface_BuildSpeedAlarmThreshold(threshold_alarm)); /* 内部 x10 速度超过阈值时蜂鸣。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			threshold_alarm = memory->speed_alarm_rev; /* 反转使用 Page4 反转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= Pubinterface_BuildSpeedAlarmThreshold(threshold_alarm)); /* 内部 x10 速度超过阈值时蜂鸣。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			threshold_alarm = memory->freq_alarm_osc; /* 往复使用 Page4 往复转频率报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.freq_work >= threshold_alarm); /* 频率阈值与 WorkMessage.freq_work 使用同一 1 字节业务单位。 */
		}
	}

	if ((should_beep != 0U) && (s_speed_threshold_beep_active == 0U))
	{
		//SendAlarmMessage(WORK_ALARM_SPEED_THRESHOLD); /* 阈值触发时只通知蜂鸣任务报警，不写 WorkMessage.alarm_flag，因此不强制停机。 */
		s_speed_threshold_beep_active = 1U;			  /* 记录本模块已经占用蜂鸣，避免每个周期重复投递队列。 */
		s_speed_threshold_beep_refresh_ticks = 0U;	  /* 第一次触发后从 0 开始计数，后续按周期重发保持蜂鸣状态。 */
	}
	else if ((should_beep != 0U) && (s_speed_threshold_beep_active != 0U))
	{
		++s_speed_threshold_beep_refresh_ticks; /* 阈值持续触发时累计周期，避免每 50ms 重发导致蜂鸣队列堆积。 */
		if (s_speed_threshold_beep_refresh_ticks >= 10U)
		{
			//SendAlarmMessage(WORK_ALARM_SPEED_THRESHOLD); /* 约 500ms 重发一次阈值蜂鸣，防止普通按键蜂鸣覆盖报警状态。 */
			s_speed_threshold_beep_refresh_ticks = 0U;	  /* 重发后清零计数，下一轮继续按低频率保持报警。 */
		}
	}
	else if ((should_beep == 0U) && (s_speed_threshold_beep_active != 0U))
	{
		SendAlarmMessage(WORK_ALARM_NONE);		   /* 阈值恢复且没有其它系统报警时，释放本模块蜂鸣。 */
		s_speed_threshold_beep_active = 0U;		   /* 清除本模块蜂鸣占用状态，下一次超过阈值可重新触发。 */
		s_speed_threshold_beep_refresh_ticks = 0U; /* 阈值恢复后清掉重发计数，避免下一次触发立即重发。 */
	}
}

void WorkAlarm_Set(uint8_t alarm_value)
{
	/* 所有新报警统一写 WorkMessage，避免再通过旧报警字段分散传递。 */
	WorkMessage.alarm_value = alarm_value;
	WorkMessage.alarm_flag = (alarm_value != WORK_ALARM_NONE);
}

void WorkAlarm_Clear(void)
{
	WorkAlarm_Set(WORK_ALARM_NONE);
}

void WorkAlarm_ClearIf(uint8_t alarm_value)
{
	/* 只清理调用方拥有的报警，避免一个模块恢复时误清另一个模块仍存在的故障。 */
	if ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value))
	{
		WorkAlarm_Clear();
	}
}

bool WorkAlarm_Is(uint8_t alarm_value)
{
	return ((WorkMessage.alarm_flag == true) && (WorkMessage.alarm_value == alarm_value));
}

/*
 * 控制信号结构体目前仍处于新旧模块并行迁移阶段，保留 ChannelMessageInit()
 * 与 ChannelFlagMessageInit() 两个历史声明入口，统一清零到新的控制信号容器。
 * 这样 userparser 或后续模块无论调用哪个初始化名，都不会留下未初始化的控制标志。
 */
static void ControlMessageInit(void)
{
	memset(&ControlSignalMessage, 0, sizeof(ControlSignalMessage));
	memset(&ControlSigleMssage, 0, sizeof(ControlSigleMssage));
}

/*
 * 函数功能：清除当前电机、脚踏、外控和 A/B 泵输出状态，用于外控进入、退出和超时安全停机。
 * 输入参数：无。
 * 返回参数：无。
 */
/*
 * 函数功能：仲裁切换或释放控制权时统一停止电机、脚踏/外控标志和 A/B 泵输出。
 * 输入参数：无，函数直接清理 WorkMessage、ControlSignalMessage 和 pumpMessageA/B。
 * 返回参数：无。
 */
static void ControlArbitration_StopMotionOutput(void)
{
	/* 仲裁切换控制权时，先停电机，避免上一个控制源留下运行状态。 */
	WorkMessage.runflag_work = false;
	/* 实际输出速度清零，驱动任务下一周期会按停止状态下发。 */
	WorkMessage.speed_work = 0U;
	/* 清除手柄按键运行标志，防止退出外控后沿用旧的手柄启动状态。 */
	ControlSignalMessage.handle_control_flag = false;
	/* 清除外部控制运行标志，真正的控制权由 s_control_owner 统一表达。 */
	ControlSignalMessage.HMI_control_flag = false;
	/* 清除左右脚踏电机控制标志，避免外控期间脚踏状态残留。 */
	ControlSignalMessage.jtL_control_flag = false;
	ControlSignalMessage.jtR_control_flag = false;
	/* 清除三类轻排泵运行标志，使泵状态和电机状态一起回到空闲。 */
	ControlSignalMessage.jtL_gentlypump_flag = false;
	ControlSignalMessage.jtR_gentlypump_flag = false;
	ControlSignalMessage.HMI_gentlypump_flag = false;
	/* 清除外控泵启动标志，避免 A/B 泵被旧外控请求继续占用。 */
	ControlSignalMessage.HMIL_pump_flag = false;
	ControlSignalMessage.HMIR_pump_flag = false;
	/* 停止 A 泵输出，并取消排空计时，避免退出外控后继续出水。 */
	pumpMessageA.run_flag = false;
	pumpMessageA.timingDrainage_flag = false;
	pumpMessageA.timingDrainage_times = 0U;
	/* 外控申请只停止 A 泵输出，不清除 speed_work 设定值，避免屏幕在外控授权后把泵速度显示成 0。 */
	/* 停止 B 泵输出，并取消排空计时，保持两路泵的仲裁动作一致。 */
	pumpMessageB.run_flag = false;
	pumpMessageB.timingDrainage_flag = false;
	pumpMessageB.timingDrainage_times = 0U;
	/* 外控申请只停止 B 泵输出，不清除 speed_work 设定值，保证停止态仍显示原来的泵速度参数。 */
	Pubinterface_RefreshPumpADisplay(); /* 仲裁停泵后刷新 A 泵数值和按钮，避免退出外控后屏幕保留旧速度。 */
	Pubinterface_RefreshPumpBDisplay(); /* 仲裁停泵后刷新 B 泵数值和按钮，避免泵已停但屏幕仍显示运行。 */
}

/*
 * 函数功能：触控或外部控制退出后，按“脚踏优先、手控次之”的规则计算本机控制模式。
 * 输入参数：无，读取脚踏在线标志、当前手柄型号和当前通道记忆。
 * 返回参数：JTWORK/HANDLEWORK/NOWORK，表示退出占用后的本机驱动方式。
 */
static uint8_t ControlArbitration_GetLocalDriveTypeAfterExit(void)
{
	uint8_t drive_type = NOWORK; /* 默认保持无控制方式，只有明确满足脚踏或手控条件时才恢复本机入口。 */

	if (ControlSignalMessage.jt_enable_flag == true)
	{
		drive_type = JTWORK; /* 脚踏在线时优先回到脚控选中，满足触控退出后的脚控优先级要求。 */
	}
	else if (Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model))
	{
		drive_type = HANDLEWORK; /* 脚踏不在线且当前手柄支持手控时，回到手控选中。 */
	}
	else
	{
		drive_type = NOWORK; /* 没有脚踏也没有可手控手柄时，不虚亮任何控制方式。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		MemoryMsgA.drive_type = drive_type; /* A 通道同步新的回落结果，避免退出触控后仍记着 TOUCHWORK。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		MemoryMsgB.drive_type = drive_type; /* B 通道同步新的回落结果，保证后续切回通道时显示一致。 */
	}

	return drive_type;
}

/*
 * 函数功能：判断驱动板反馈的实际转速是否仍表示电机在转动。
 * 输入参数：无，直接读取 WorkMessage.driver_speed_feedback。
 * 返回参数：true 表示驱动反馈转速仍大于停止阈值；false 表示反馈已经低于停止阈值。
 */
static bool ControlArbitration_IsMotorFeedbackActive(void)
{
	/* 反馈转速来自 UART1 驱动板回包，用于避免刚下发停止命令但电机还未真实停稳时提前释放控制权。 */
	return (WorkMessage.driver_speed_feedback > CONTROL_ARBITRATION_MOTOR_STOP_SPEED_THRESHOLD);
}

/*
 * 函数功能：判断手柄电机当前是否处于忙状态。
 * 输入参数：无，读取 runflag_work 命令标志和 driver_speed_feedback 实际反馈。
 * 返回参数：true 表示电机被要求运行或反馈仍在转动；false 表示命令停止且反馈已停稳。
 */
static bool ControlArbitration_IsMotorBusy(void)
{
	/* runflag_work 是主控下发运行帧的命令源，置位时必须认为电机正在被控制。 */
	if (WorkMessage.runflag_work == true)
	{
		return true;
	}

	/* runflag_work 清零后继续看驱动反馈，保证仲裁释放跟真实停转绑定。 */
	return ControlArbitration_IsMotorFeedbackActive();
}

/*
 * 函数功能：当本地电机已经停稳时自动释放脚踏、屏幕或手柄的电机控制权。
 * 输入参数：无，读取当前 owner 和电机忙状态。
 * 返回参数：无；满足释放条件时把 s_control_owner 置为 CONTROL_OWNER_NONE。
 */
static void ControlArbitration_ReleaseLocalOwnerIfMotorIdle(void)
{
	/* 外部通信必须由主动退出、超时释放或急停释放，不能因为本地电机停稳自动退出。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 没有本地 owner 时不用处理，保持空闲状态。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return;
	}

	/* 电机命令和反馈都已经停止时，本地控制源结束，本地三种方式可以重新竞争。 */
	if (ControlArbitration_IsMotorBusy() == false)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

/*
 * 函数功能：校验传入的控制源编号是否合法。
 * 输入参数：owner 控制源编号，取 CONTROL_OWNER_EXTERNAL/FOOT/SCREEN/HANDLE。
 * 返回参数：true 表示控制源合法；false 表示非法编号，不能参与仲裁。
 */
static bool ControlArbitration_IsValidOwner(uint8_t owner)
{
	/* 只接受四种正式控制来源，防止错误参数把仲裁锁写成未知状态。 */
	return ((owner == CONTROL_OWNER_EXTERNAL) ||
			(owner == CONTROL_OWNER_FOOT) ||
			(owner == CONTROL_OWNER_SCREEN) ||
			(owner == CONTROL_OWNER_HANDLE));
}

/*
 * 函数功能：把按键队列中的控制来源类型转换为电机仲裁 owner。
 * 输入参数：control_type 按键消息来源，取 JTKey/HANDLEKey/SCREENKey/HMIkey/PLUGunPLUG 等。
 * 返回参数：CONTROL_OWNER_*；插拔和未知来源返回 CONTROL_OWNER_NONE。
 */
static uint8_t ControlArbitration_GetOwnerByKeySource(uint8_t control_type)
{
	/* 脚踏队列消息统一归属脚踏控制来源。 */
	if (control_type == JTKey)
	{
		return CONTROL_OWNER_FOOT;
	}

	/* 手柄按键队列消息统一归属手柄按键控制来源。 */
	if (control_type == HANDLEKey)
	{
		return CONTROL_OWNER_HANDLE;
	}

	/* 屏幕按钮和触控启动统一归属屏幕控制来源。 */
	if (control_type == SCREENKey)
	{
		return CONTROL_OWNER_SCREEN;
	}

	/* 历史 HMIkey 仍按外部链路处理，避免它在脚踏/屏幕/手柄占用时绕过互斥。 */
	if (control_type == HMIkey)
	{
		return CONTROL_OWNER_EXTERNAL;
	}

	/* 插拔和未知消息不参与运行控制权竞争。 */
	return CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断泵控制按键是否需要占用电机仲裁 owner。
 * 输入参数：key_value 业务按键值，包含脚踏泵键、屏幕泵键和外控 HMI 泵键。
 * 返回参数：本地泵键返回 CONTROL_OWNER_NONE；外控泵键返回 CONTROL_OWNER_EXTERNAL。
 */
static uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value)
{
	/* 脚踏泵按键和脚踏轻排按键只影响泵输出，不应该因为泵运行占用手柄电机 owner。 */
	if ((key_value == JTkey_left_short) ||
		(key_value == JTKey_left_long) ||
		(key_value == JTKey_right_short) ||
		(key_value == JTKey_right_long) ||
		(key_value == JTKey_Gently_left_start) ||
		(key_value == JTKey_Gently_left_stop) ||
		(key_value == JTKey_Gently_right_start) ||
		(key_value == JTKey_Gently_rigth_stop))
	{
		/* 脚踏泵键只控制泵，不占用手柄电机 owner；脚踏真正启动电机时由 FootControlTask 申请 FOOT owner。 */
		return CONTROL_OWNER_NONE;
	}

	/* 屏幕泵按钮只影响泵输出，不应该因为屏幕点过泵就锁住脚踏或手柄电机控制。 */
	if ((key_value == SCREENKey_APUMP_Add) ||
		(key_value == SCREENKey_APUMP_Sub) ||
		(key_value == SCREENKey_APUMP_control) ||
		(key_value == SCREENKey_BPUMP_Add) ||
		(key_value == SCREENKey_BPUMP_Sub) ||
		(key_value == SCREENKey_BPUMP_control))
	{
		/* 屏幕泵键只控制泵，不占用手柄电机 owner；屏幕触控启动电机时才申请 SCREEN owner。 */
		return CONTROL_OWNER_NONE;
	}

	/* 历史 HMI 泵按钮由外部来源占用。 */
	if ((key_value == HMIkey_APUMP_Add) ||
		(key_value == HMIkey_APUMP_Sub) ||
		(key_value == HMIkey_APUMP_control) ||
		(key_value == HMIkey_BPUMP_Add) ||
		(key_value == HMIkey_BPUMP_Sub) ||
		(key_value == HMIkey_BPUMP_control) ||
		(key_value == HMIkey_Gently_start) ||
		(key_value == HMIkey_Gently_stop))
	{
		return CONTROL_OWNER_EXTERNAL;
	}

	/* 其它泵函数入口不改变控制权。 */
	return CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断当前按键是否只是运行参数调节，不应被电机运行 owner 互斥丢弃。
 * 输入参数：control_type 为按键来源；control_key 为来源内的业务按键值。
 * 返回参数：true 表示该键只调整速度、频率或泵流量；false 表示仍按普通运行控制键仲裁。
 */
static bool ControlArbitration_IsRuntimeAdjustmentKey(uint8_t control_type, uint8_t control_key)
{
	/* 手柄实体速度加减只修改当前目标速度，不改变脚控、手控、触控或外控的运行归属。 */
	if (control_type == HANDLEKey)
	{
		return ((control_key == HANDLEKey_speed_add) ||
				(control_key == HANDLEKey_speed_sub));
	}

	/* 外部通信的速度、频率、A/B 泵加减属于参数调节，运行中也需要交给业务层刷新设定值。 */
	if (control_type == HMIkey)
	{
		return ((control_key == HMIkey_SPEED_Add) ||
				(control_key == HMIkey_SPEED_Sub) ||
				(control_key == HMIkey_FREQ_Add) ||
				(control_key == HMIkey_FREQ_Sub) ||
				(control_key == HMIkey_APUMP_Add) ||
				(control_key == HMIkey_APUMP_Sub) ||
				(control_key == HMIkey_BPUMP_Add) ||
				(control_key == HMIkey_BPUMP_Sub));
	}

	/* 新屏的速度四键、频率键和泵流量加减键都只是改参数，不能因为电机正在运行被队列层丢弃。 */
	if (control_type == SCREENKey)
	{
		return ((control_key == SCREENKey_SPEED_Add) ||
				(control_key == SCREENKey_SPEED_Sub) ||
				(control_key == SCREENKey_SPEED_Sub_Large) ||
				(control_key == SCREENKey_SPEED_Sub_Small) ||
				(control_key == SCREENKey_SPEED_Add_Small) ||
				(control_key == SCREENKey_SPEED_Add_Large) ||
				(control_key == SCREENKey_FREQ_Add) ||
				(control_key == SCREENKey_FREQ_Sub) ||
				(control_key == SCREENKey_APUMP_Add) ||
				(control_key == SCREENKey_APUMP_Sub) ||
				(control_key == SCREENKey_BPUMP_Add) ||
				(control_key == SCREENKey_BPUMP_Sub));
	}

	/* 脚踏短按由 Foot_ParseDataS 转成 A/B 泵调档键，本质也是泵流量调节。 */
	if (control_type == JTKey)
	{
		return ((control_key == JTkey_left_short) ||
				(control_key == JTKey_right_short));
	}

	/* 其它来源或其它键仍交给普通 owner 仲裁，避免误放行启动/模式切换类动作。 */
	return false;
}

/*
 * 函数功能：判断当前按键是否属于泵业务入口，泵输出不应被手柄电机 owner 直接拦截。
 * 输入参数：control_type 为按键来源；control_key 为来源内的泵业务按键值。
 * 返回参数：true 表示应进入 PUMPActive 继续按泵类型、在线状态和排空状态判断；false 表示不是泵业务键。
 */
static bool ControlArbitration_IsPumpBusinessKey(uint8_t control_type, uint8_t control_key)
{
	/* 脚踏短按调档、长按启停/排空、轻排开始/停止都通过同一个泵业务函数处理。 */
	if (control_type == JTKey)
	{
		return ((control_key == JTkey_left_short) ||
				(control_key == JTKey_left_long) ||
				(control_key == JTKey_right_short) ||
				(control_key == JTKey_right_long) ||
				(control_key == JTKey_Gently_left_start) ||
				(control_key == JTKey_Gently_left_stop) ||
				(control_key == JTKey_Gently_right_start) ||
				(control_key == JTKey_Gently_rigth_stop));
	}

	/* 外部通信泵加减、启停和轻排命令都要进入 PUMPActive，再由泵业务层决定是否真正生效。 */
	if (control_type == HMIkey)
	{
		return ((control_key == HMIkey_APUMP_Add) ||
				(control_key == HMIkey_APUMP_Sub) ||
				(control_key == HMIkey_APUMP_control) ||
				(control_key == HMIkey_BPUMP_Add) ||
				(control_key == HMIkey_BPUMP_Sub) ||
				(control_key == HMIkey_BPUMP_control) ||
				(control_key == HMIkey_Gently_start) ||
				(control_key == HMIkey_Gently_stop));
	}

	/* 屏幕 A/B 泵加减和启停也只属于泵业务，不占用手柄电机运行 owner。 */
	if (control_type == SCREENKey)
	{
		return ((control_key == SCREENKey_APUMP_Add) ||
				(control_key == SCREENKey_APUMP_Sub) ||
				(control_key == SCREENKey_APUMP_control) ||
				(control_key == SCREENKey_BPUMP_Add) ||
				(control_key == SCREENKey_BPUMP_Sub) ||
				(control_key == SCREENKey_BPUMP_control));
	}

	/* 非泵键不能绕过电机 owner 仲裁。 */
	return false;
}

/*
 * 函数功能：判断指定控制源是否为当前仲裁 owner。
 * 输入参数：owner 待查询的控制源编号。
 * 返回参数：true 表示当前 owner 与输入来源一致；false 表示不一致。
 */
bool ControlArbitration_IsOwner(uint8_t owner)
{
	/* 当前持有者和查询来源一致时，允许该来源继续控制或主动停止。 */
	return (s_control_owner == owner);
}

/*
 * 函数功能：判断指定控制源是否被其它控制源阻塞。
 * 输入参数：owner 当前准备执行动作的控制源编号。
 * 返回参数：true 表示需要阻塞当前动作；false 表示当前动作可以继续处理。
 */
bool ControlArbitration_IsBusyByOther(uint8_t owner)
{
	/* 查询前先释放已经停稳的本地 owner，避免停止后残留锁挡住下一种本地控制方式。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();

	/* 没有 owner 时表示电机仲裁空闲，任意本地来源都可以尝试控制。 */
	if (s_control_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前来源就是 owner 时允许继续控制或发送停止命令。 */
	if (s_control_owner == owner)
	{
		return false;
	}

	/* 外部通信授权期间，本地脚踏、屏幕、手柄都必须等待外控主动退出或超时释放。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return true;
	}

	/* 本地三种方式只在手柄电机忙时互斥；泵运行不参与电机控制权阻塞。 */
	return ControlArbitration_IsMotorBusy();
}

/*
 * 函数功能：尝试让指定控制源取得电机控制权。
 * 输入参数：owner 申请控制权的来源编号。
 * 返回参数：true 表示申请成功或已经持有；false 表示无效来源或被其它来源阻塞。
 */
bool ControlArbitration_TryEnter(uint8_t owner)
{
	/* 无效来源不能写入 owner，避免未知按键把仲裁状态写乱。 */
	if (ControlArbitration_IsValidOwner(owner) == false)
	{
		return false;
	}

	/* 申请前先清掉已经停稳的本地 owner，保证电机停止后其它本地模式能接管。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();

	/* 已经持有控制权时重复申请按成功处理。 */
	if (s_control_owner == owner)
	{
		return true;
	}

	/* 外部通信授权期间，只有外部通信自己能继续控制。 */
	if (s_control_owner == CONTROL_OWNER_EXTERNAL)
	{
		return false;
	}

	/* 电机命令仍在运行或驱动反馈仍在转动时，不允许不同来源抢占。 */
	if (ControlArbitration_IsMotorBusy())
	{
		return false;
	}

	/* 电机已经空闲时记录新的控制源；本地泵动作不会调用本路径占用 owner。 */
	s_control_owner = owner;
	return true;
}

/*
 * 函数功能：释放指定控制源持有的仲裁 owner，只允许当前持有者释放自己。
 * 输入参数：owner 请求释放控制权的来源编号。
 * 返回参数：无。
 */
void ControlArbitration_Exit(uint8_t owner)
{
	/* 只有当前持有者本人才能释放，避免其它来源误清正在运行的控制权。 */
	if (s_control_owner == owner)
	{
		s_control_owner = CONTROL_OWNER_NONE;
	}
}

/*
 * 函数功能：当本地控制源对应的电机已经停止时释放本地电机控制权。
 * 输入参数：owner 请求释放的本地控制源编号。
 * 返回参数：无。
 */
void ControlArbitration_ExitLocalControlIfIdle(uint8_t owner)
{
	/* 外部通信必须由外控退出、链路超时或急停释放，不能由本地空闲逻辑释放。 */
	if (owner == CONTROL_OWNER_EXTERNAL)
	{
		return;
	}

	/* 电机命令仍在运行或驱动反馈仍未归零时，本地来源还没有真正结束。 */
	if (ControlArbitration_IsMotorBusy())
	{
		return;
	}

	/* 电机停稳后释放当前本地来源；泵状态不再影响手柄电机仲裁。 */
	ControlArbitration_Exit(owner);
}

/*
 * 函数功能：周期刷新电机控制权释放状态。
 * 输入参数：无，由电机输出任务或反馈任务周期调用。
 * 返回参数：无；本地 owner 停稳后自动释放，外部 owner 不自动释放。
 */
void ControlArbitration_RefreshMotorOwner(void)
{
	/* 统一复用本地 owner 释放逻辑，保证手柄、脚踏、屏幕停止后不因反馈延迟永久占用。 */
	ControlArbitration_ReleaseLocalOwnerIfMotorIdle();
}

/*
 * 函数功能：系统级强制释放当前仲裁 owner，用于急停或全局清状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ForceRelease(void)
{
	/* 急停或系统级清状态使用强制释放，让任何来源都不能继续占用控制权。 */
	s_control_owner = CONTROL_OWNER_NONE;
}

/*
 * 函数功能：判断外部通信控制权是否处于有效占用状态。
 * 输入参数：无，读取 s_control_owner 和 WorkMessage.hmiactive_work。
 * 返回参数：true 表示外控已持有仲裁且界面状态仍有效；false 表示外控未激活。
 */
bool ControlArbitration_IsExternalActive(void)
{
	/* 外控是否有效以仲裁持有者为准，hmiactive_work 只作为界面/状态同步标志。 */
	return ((s_control_owner == CONTROL_OWNER_EXTERNAL) && (WorkMessage.hmiactive_work != 0U));
}

/*
 * 函数功能：申请进入外部通信控制模式，并初始化外控占用状态。
 * 输入参数：无。
 * 返回参数：true 表示外控取得控制权；false 表示本机电机仍忙或其它来源占用。
 */
bool ControlArbitration_EnterExternalControl(void)
{
	bool already_external = ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL);

	/* 若脚踏、屏幕或手柄正在控制，上位机申请直接失败，不能抢停当前来源。 */
	if (ControlArbitration_TryEnter(CONTROL_OWNER_EXTERNAL) == false)
	{
		return false;
	}

	/* 首次进入外控时清理空闲残留输出；重复申请外控时不打断已在进行的外控动作。 */
	if (already_external == false)
	{
		ControlArbitration_StopMotionOutput();
		ControlSignalMessage.HMI_control_flag = false; /* 首次进入外控时确认电机未启动；重复保活申请不能清运行标志，否则小电脑会从 40 退回 39。 */
	}

	/* 置位外部控制权锁，脚踏、屏幕、手柄按键会在各自入口被拦截。 */
	WorkMessage.hmiactive_work = 1U;
	/* 触控/外控占用标志同步置位，屏幕模式切换逻辑也能看到外控占用。 */
	WorkMessage.touchactive_work = TOUCHWORK;
	/* 当前驱动方式切到外部控制，心跳和驱动状态可以看到外控来源。 */
	WorkMessage.drivetype_work = TOUCHWORK;
	/* 外部控制已使能，但申请阶段不直接启动电机。 */
	ControlSignalMessage.HMI_enable_flag = true;
	/* 申请成功只表示外控 owner 已取得，不能再打开 UI_TOUCH_ID，否则屏幕会弹出 70 号触控工作区。 */
	Pubinterface_RefreshControlModeDisplay(); /* 外控内部复用 TOUCHWORK 做互斥，但显示层仍按小电脑图标表达外控状态。 */
	if (already_external == false)
	{
		Pubinterface_RefreshExternalCommDisplay(true, false); /* 首次申请外控成功但尚未控制输出时只显示 39 白色小电脑。 */
	}
	return true;
}

/*
 * 函数功能：退出外部通信控制模式，停止外控遗留输出并恢复本机可接管状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void ControlArbitration_ReleaseExternalControl(void)
{
	uint8_t data[10] = {0U}; /* 退出外控时给 UI_TOUCH_ID 发送全零参数，确保触控弹窗关闭。 */

	/* 当前不是外控时，退出外控只清理外控显示残留，不能影响脚踏、屏幕或手柄正在进行的控制。 */
	if (s_control_owner != CONTROL_OWNER_EXTERNAL)
	{
		WorkMessage.hmiactive_work = 0U; /* 非外控 owner 时只清界面外控标志，不能停本机脚踏/手柄/触控运行。 */
		ControlSignalMessage.HMI_enable_flag = false; /* 上位机退出命令到来后撤销外控可用标志，避免状态继续显示为外控占用。 */
		if (WorkMessage.touchactive_work != TOUCHWORK)
		{
			SendUIDSMessage(UI_TOUCH_ID, false, data); /* 未处于本机触控时才清理残留外控弹窗，避免误关本机触控工作区。 */
		}
		Pubinterface_RefreshControlModeDisplay(); /* UITOUCHDP(false) 会把触控主按钮置暗，这里重绘为当前本机白/黄状态。 */
		Pubinterface_RefreshExternalCommDisplay(false, false); /* 退出外部通信后不再保留 39/40 小电脑图标，避免误提示仍在线。 */
		return;
	}

	/* 释放外控时必须先停全部外控输出，防止泵或电机继续沿用上位机请求。 */
	ControlArbitration_StopMotionOutput();
	/* 清除外部控制权锁，允许后续脚踏、屏幕和手柄重新参与控制。 */
	WorkMessage.hmiactive_work = 0U;
	/* 清除触控/外控占用标志，使本机模式切换可以重新生效。 */
	WorkMessage.touchactive_work = 0U;
	/* 外控使能同步撤销，界面状态不再显示外部控制已占用。 */
	ControlSignalMessage.HMI_enable_flag = false;
	/* 恢复到本机可接管的控制方式，避免退出后仍停在 TOUCHWORK。 */
	WorkMessage.drivetype_work = ControlArbitration_GetLocalDriveTypeAfterExit();
	/* 外控完整退出后才释放公共控制权。 */
	ControlArbitration_Exit(CONTROL_OWNER_EXTERNAL);
	SendUIDSMessage(UI_TOUCH_ID, false, data); /* 外控释放完成后关闭触控/外控窗口，UI 不再显示占用状态。 */
	Pubinterface_RefreshControlModeDisplay(); /* 外控退出后按本机状态重绘脚控/手控/触控，清掉外控期间的残留高亮。 */
	Pubinterface_RefreshExternalCommDisplay(false, false); /* 主动退出外控后按需求隐藏小电脑图标，不再显示白色在线态。 */
}

/*
 * 函数功能：按键队列分发前判断当前消息是否应被仲裁阻塞。
 * 输入参数：control_type 按键来源类型；control_key 具体业务按键值。
 * 返回参数：true 表示本消息应丢弃等待当前 owner 结束；false 表示可继续分发。
 */
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type, uint8_t control_key)
{
	uint8_t key_owner = ControlArbitration_GetOwnerByKeySource(control_type);

	/* 手柄插拔事件只维护在线状态和通道记忆，不属于运行控制，互斥期间仍要接收。 */
	if (control_type == PLUGunPLUG)
	{
		return false;
	}

	/* 外控持有期间允许旧 HMI 和屏幕上的外控退出键通过，便于主动释放上位机控制。 */
	if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) &&
		(((control_type == HMIkey) && (control_key == HMIkey_HMI_EXIT)) ||
		 ((control_type == SCREENKey) && (control_key == SCREENKey_HMI_EXIT))))
	{
		return false;
	}

	/* 运行参数调节允许跨本地 owner 生效；但外控已独占时，本地脚踏/屏幕/手柄不能绕过外控锁。 */
	if (ControlArbitration_IsRuntimeAdjustmentKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			if (control_type == SCREENKey)
			{
				return false; /* 外控 owner 期间允许屏幕修改速度/频率/泵流量参数，但屏幕启停键仍走后面的业务键拦截。 */
			}
			return true; /* 外部控制占用时只允许外部来源继续调参，防止本地按键改变外控运行设定。 */
		}
		return false; /* 非外控独占时放行到 SpeedActive/FreqActive/PUMPActive，由业务函数刷新设定值。 */
	}

	/* 泵业务和手柄电机 owner 分离，先放行到 PUMPActive，再由泵类型、在线状态和排空状态判断。 */
	if (ControlArbitration_IsPumpBusinessKey(control_type, control_key))
	{
		if (ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL) && key_owner != CONTROL_OWNER_EXTERNAL)
		{
			return true; /* 外控占用期间本地泵键也不能抢改泵状态，保持外部控制一致性。 */
		}
		return false; /* 脚踏、屏幕和外部泵键都进入统一泵业务入口，不再被电机运行 owner 丢弃。 */
	}

	/* 不属于四类控制来源的消息不参与仲裁。 */
	if (key_owner == CONTROL_OWNER_NONE)
	{
		return false;
	}

	/* 当前已有其它来源占用时，丢弃该按键，必须等当前控制方式结束。 */
	return ControlArbitration_IsBusyByOther(key_owner);
}

void ChannelMessageInit(void)
{
	ControlMessageInit();
}

void ChannelFlagMessageInit(void)
{
	ControlMessageInit();
}

/*
 * 函数功能：消费屏幕外部控制退出按钮的 1 秒双击确认。
 * 输入参数：无。
 * 返回参数：true 表示本次点击是 1 秒内第二次点击，可以退出外控；false 表示本次仅记录第一次点击。
 */
static bool Pubinterface_ConsumeScreenExternalExitDoubleClick(void)
{
	uint32_t now_tick = HAL_GetTick(); /* 读取 HAL 毫秒 tick，让双击窗口不依赖具体按键任务调度周期。 */

	if (s_screen_external_exit_pending != 0U)
	{
		if ((uint32_t)(now_tick - s_screen_external_exit_first_tick) <= SCREEN_EXTERNAL_EXIT_DOUBLE_CLICK_MS)
		{
			s_screen_external_exit_pending = 0U; /* 第二次点击已确认退出，清掉待确认状态，避免第三次点击沿用旧窗口。 */
			return true; /* 1 秒窗口内第二次点击成立，允许后续释放外部控制权。 */
		}
	}

	s_screen_external_exit_first_tick = now_tick; /* 第一次点击或超时后的新第一次点击都重新记录当前时间。 */
	s_screen_external_exit_pending = 1U; /* 进入待确认状态，下一次屏幕点击在 1 秒内才会真正退出外控。 */
	return false; /* 首次点击只给蜂鸣反馈，不释放外部控制权。 */
}

/*
 * 函数功能：处理 HMI 或屏幕触发的外部控制退出。
 * 输入参数：key_value 按键值，HMIkey_HMI_EXIT 表示外部协议主动退出，SCREENKey_HMI_EXIT 表示屏幕按钮退出。
 * 返回参数：无。
 */
void HmiExitActive(uint8_t key_value)
{
	if (ControlArbitration_IsExternalActive() == false)
	{
		return; /* 只有外控真正处于占用状态时才允许退出，避免屏幕外控键在普通运行态误清状态。 */
	}
	if (key_value == SCREENKey_HMI_EXIT)
	{
		if (WorkMessage.alarm_flag == true)
		{
			s_screen_external_exit_pending = 0U; /* 报警状态下屏幕外控退出键不生效，并清掉双击窗口，避免报警解除后沿用旧点击。 */
			return; /* EX8 表格要求屏幕外控退出必须 alarm_flag=false；上位机 HMIkey_HMI_EXIT 不受这个屏幕按键门控影响。 */
		}
		if (Pubinterface_ConsumeScreenExternalExitDoubleClick() == false)
		{
			return; /* EX8 屏幕要求 1 秒内连续两次点击才退出外控，第一次点击只保留蜂鸣反馈。 */
		}
	}

	ControlArbitration_ReleaseExternalControl();
	s_screen_external_exit_pending = 0U; /* 任意来源完成外控退出后清掉屏幕双击待确认，避免下一轮外控沿用旧点击。 */
	// 通知HMI标志位界面消失
	//  判断是否为外部命令触发的HMI退出
	if (key_value == HMIkey_HMI_EXIT)
	{
		// 外部命令，说我要退出外部，全部交于动力自主
		// 如果外部控制中，则停止相关内容，比如泵和电机
	}
	// 判断是否为导航界面按钮触发的强制退出
	else if (key_value == SCREENKey_HMI_EXIT)
	{
		// 导航界面按钮强制退出，通知一下上位机，若要控制请重头校验识别
	}
}

/*
 * 函数功能：触控运行保活超时后只停止电机运行，不退出触控控制界面。
 * 输入参数：无，直接读取 WorkMessage 当前触控状态。
 * 返回参数：无。
 */
void Pubinterface_StopTouchKeepAliveRun(void)
{
	uint8_t data[10] = {0U}; /* UI_TOUCH_ID 使用 Value[0] 表示运行态，0 表示触控仍激活但电机停止。 */

	if (WorkMessage.hmiactive_work != 0U)
	{
		return; /* 外控内部复用 TOUCHWORK 做互斥时，触控保活超时不能停外控手柄，也不能弹出 70 号触控窗口。 */
	}
	if ((WorkMessage.touchactive_work != TOUCHWORK) || (WorkMessage.drivetype_work != TOUCHWORK))
	{
		return; /* 当前不是本机触控模式时不处理，避免外控或脚踏被保活超时误停。 */
	}
	Pubinterface_SetHandleInjectionPumpRun(false);
	WorkMessage.runflag_work = false; /* 保活超时只撤销运行命令，屏幕触控模式保持占用。 */
	WorkMessage.speed_work = 0U; /* 实际输出速度同步清零，驱动任务下一周期下发停止帧。 */
	data[0] = 0U; /* 触控界面仍显示，但运行态从黄色运行回到白色待运行。 */
	SendUIDSMessage(UI_TOUCH_ID, true, data); /* 不发送 enable=false，避免触控界面被隐藏。 */
	Pubinterface_RefreshControlModeDisplay(); /* 刷新主运行页触控按钮状态，保持触控模式和运行态显示同步。 */
}

/*
 * 函数功能：处理屏幕/HMI 的控制方式切换与屏幕触控启动、停止动作。
 * 输入参数：key_value 屏幕或 HMI 下发的控制方式按键值。
 * 返回参数：无。
 */
void ControlTypeActive(uint8_t key_value)
{
	uint8_t data[10] = {0U}; /* 控制方式 UI 消息的临时缓冲，触控弹窗只使用 data[0] 表示运行态。 */

	if (WorkMessage.alarm_flag == true)
		return;
	switch (key_value)
	{
	case SCREENKey_JTActi:
	case HMIkey_JTActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		// 脚踏控制
		if (ControlSignalMessage.jt_enable_flag != true)
			return; /* EX8 表格要求脚踏在线后才允许切到脚控，未接脚踏时只保留按钮蜂鸣反馈。 */
		if (WorkMessage.drivetype_work == JTWORK || WorkMessage.touchactive_work == TOUCHWORK)
			return;
		WorkMessage.drivetype_work = JTWORK;
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = JTWORK;
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = JTWORK;
		Pubinterface_RefreshControlModeDisplay(); /* 屏幕切到脚控后立刻刷新高亮，状态仍只写主工程 WorkMessage/MemoryMsg。 */
		// 注意界面更新
	}
	break;
	case SCREENKey_HandleActi:
	case HMIkey_HandleActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		// 手动控制
		if (Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model) == false)
			return; /* 只有 PXBA/PXBB/LGZII 等确认支持手控入口的手柄允许切入手控模式。 */
		if (WorkMessage.drivetype_work == HANDLEWORK || WorkMessage.touchactive_work == TOUCHWORK)
			return;
		WorkMessage.drivetype_work = HANDLEWORK;
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = HANDLEWORK;
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = HANDLEWORK;
		Pubinterface_RefreshControlModeDisplay(); /* 屏幕切到手控后立刻刷新高亮，不引入副工程旧全局状态。 */
		// 界面调整更新
	}
	break;
	case SCREENKey_TouchActi:
	{
		if (WorkMessage.runflag_work == true)
			return;
		if (WorkMessage.hand_model == 0U)
			return; /* EX8 表格要求触控入口必须已有有效手柄，未识别手柄时只保留屏幕蜂鸣反馈。 */
		if (ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN) == false)
			return;
		/* 新屏主运行页的触控入口只负责进入触控工作面，不直接起机。 */
		WorkMessage.touchactive_work = TOUCHWORK; /* 标记本机触控占用，阻止脚踏/手柄在运行中抢占。 */
		WorkMessage.drivetype_work = TOUCHWORK; /* 控制方式同步切到触控，控制图标立即由白色变黄色。 */
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = TOUCHWORK; /* A 通道记忆当前触控来源，后续切回 A 时保持显示一致。 */
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = TOUCHWORK; /* B 通道记忆当前触控来源，避免切通道后又显示为手控。 */
		data[0] = 0U; /* 点击触控图标只进入触控模式，不直接启动电机；运行由 0x5520 保活帧维持。 */
		SendUIDSMessage(UI_TOUCH_ID, true, data); /* 通知屏幕触控入口进入待运行状态，保持触控界面可见。 */
		WorkMessage.runflag_work = false; /* 未收到保活前保持停机，避免进入触控模式瞬间误启动手柄。 */
		WorkMessage.speed_work = 0U; /* 进入触控待运行态时实际输出清零，后续保活再恢复设定速度。 */
		Pubinterface_RefreshControlModeDisplay(); /* 触控模式进入后立即刷新控制方式，清掉手控/脚控旧高亮。 */
		break;
	}
	case SCREENKey_TouchKeepAlive:
		if (WorkMessage.hmiactive_work != 0U)
		{
			return; /* 外控模式下忽略屏幕 0x5520 触控保活帧，避免把外控误判成本机触控并弹出 70。 */
		}
		if ((WorkMessage.touchactive_work != TOUCHWORK) || (WorkMessage.drivetype_work != TOUCHWORK))
		{
			return; /* EX8 0x5520 运行保活必须在触控入口已打开后才有效，避免保活帧绕过 0x2404/key3 条件直接启动。 */
		}
		if (Pubinterface_CheckCommonSocketToolReadyForRun() == false)
		{
			Pubinterface_StopTouchKeepAliveRun(); /* 公共接头等待 EPC 刀具头时，保活帧只报警并保持停机。 */
			return;
		}
		if (ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN) == false)
		{
			return; /* 其它来源占用时忽略本次保活，避免触控抢占脚踏/手柄/外控。 */
		}
		WorkMessage.touchactive_work = TOUCHWORK; /* 收到 0x5520 表示触控仍在按压，保持触控占用。 */
		WorkMessage.drivetype_work = TOUCHWORK; /* 保活期间驱动方式固定为触控，屏幕图标保持触控高亮。 */
		if (WorkMessage.channel_work == CHANNEL_A)
			MemoryMsgA.drive_type = TOUCHWORK; /* A 通道保活时同步通道控制方式记忆。 */
		else if (WorkMessage.channel_work == CHANNEL_B)
			MemoryMsgB.drive_type = TOUCHWORK; /* B 通道保活时同步通道控制方式记忆。 */
		if (WorkMessage.speed_set_work == 0U)
		{
			WorkMessage.speed_set_work = Pubinterface_GetCurrentDefaultMotorSpeed(); /* 保活启动时若速度未装载，补当前通道默认速度。 */
		}
		WorkMessage.speed_work = WorkMessage.speed_set_work; /* 每次保活都恢复目标速度，超时停转后再次按下可立即恢复。 */
		WorkMessage.runflag_work = true; /* 200ms 保活窗口内持续置位运行命令。 */
		Pubinterface_SetHandleInjectionPumpRun(true);
		data[0] = 1U; /* 触控正在运行，屏幕触控图标显示黄色运行态。 */
		SendUIDSMessage(UI_TOUCH_ID, true, data); /* 保活只刷新运行态，不改变触控界面显隐。 */
		Pubinterface_RefreshControlModeDisplay(); /* 同步主运行页触控高亮，避免其它刷新把触控误刷白。 */
		break;
	case SCREENKey_TouchEXIT: // 停止
		if (WorkMessage.touchactive_work != TOUCHWORK)
		{
			return; /* 只有屏幕触控模式已经进入时才允许退出，避免未进入触控态时误清控制状态。 */
		}
		/* 屏幕触控退出时先停电机，再清触控占用，控制权释放要继续等待驱动反馈归零。 */
		WorkMessage.runflag_work = false;
			
		WorkMessage.speed_work = 0U;
		WorkMessage.touchactive_work = 0U;
		WorkMessage.drivetype_work = ControlArbitration_GetLocalDriveTypeAfterExit(); /* 触控退出后恢复本机可接管控制方式，避免图标继续停在黄色触控。 */
		SendUIDSMessage(UI_TOUCH_ID, false, data); /* 触控退出时立即隐藏弹窗，控制权释放由驱动停稳后完成。 */
		/* 这里不能直接 Exit，否则刚下发停止但电机尚未停稳时其它来源会提前接管。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN);
		Pubinterface_RefreshControlModeDisplay(); /* 停止触控后同步手控/脚控/触控白黄状态，避免残留高亮。 */
		// 退出触控界面
		break;
	}
}

/*
 * 函数功能：处理速度增减键，慢档使用当前手柄/方向识别步进，快档使用慢档两倍。
 * 输入参数：key_value 为手柄、HMI 或新屏速度键值。
 * 返回参数：无。
 */
void SpeedActive(uint8_t key_value)
{
	uint32_t speed_value = WorkMessage.speed_set_work; /* 以当前设定速度为基准，Page4 24位最大速度可能超过16位，调速过程必须保留32位。 */
	uint32_t speed_step = 0U;                          /* 本次实际步进，先取通道识别步进，新屏快档再按两倍换算。 */
	uint32_t speed_max = 0U;                           /* 当前通道、当前方向允许的最大设定速度，支持 Page4 24位上限。 */
	uint32_t speed_min = 0U;                           /* 当前通道、当前方向允许的最小设定速度。 */
	bool add_key = false;                              /* true 表示本次按键为增加速度。 */
	bool sub_key = false;                              /* true 表示本次按键为减少速度。 */
	uint8_t display_value[10] = {0U};                  /* 速度键生效后立即刷新屏幕速度数值，避免用户按键后无反馈。 */

	if (WorkMessage.alarm_flag == true)
	{
		return; /* 系统报警时不响应速度调节，避免报警未解除仍改变目标速度。 */
	}
	if (WorkMessage.hand_model == 0U)
	{
		return; /* EX8 表格要求速度键必须已有有效手柄，防止异常通道残留时写入无手柄速度记忆。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_zzstep; /* A 通道正转旧键步进来自 EEPROM/识别参数。 */
			speed_max = ChannelrecognizeMessageA.speed_zzmax;   /* A 通道正转最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_zzmin;   /* A 通道正转最小速度。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_fzstep; /* A 通道反转旧键步进来自 EEPROM/识别参数。 */
			speed_max = ChannelrecognizeMessageA.speed_fzmax;   /* A 通道反转最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_fzmin;   /* A 通道反转最小速度。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_oscstep; /* A 通道往复旧键步进来自 EEPROM/识别参数。 */
			speed_max = ChannelrecognizeMessageA.speed_oscmax;   /* A 通道往复最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_oscmin;   /* A 通道往复最小速度。 */
		}
		else
		{
			return; /* 未知方向不写速度，避免把无效方向下的参数覆盖到通道记忆。 */
		}
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_zzstep; /* B 通道正转旧键步进来自 B 通道识别参数。 */
			speed_max = ChannelrecognizeMessageB.speed_zzmax;   /* B 通道正转最大速度，避免误用 A 通道限幅。 */
			speed_min = ChannelrecognizeMessageB.speed_zzmin;   /* B 通道正转最小速度，避免误用 A 通道限幅。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_fzstep; /* B 通道反转旧键步进来自 B 通道识别参数。 */
			speed_max = ChannelrecognizeMessageB.speed_fzmax;   /* B 通道反转最大速度。 */
			speed_min = ChannelrecognizeMessageB.speed_fzmin;   /* B 通道反转最小速度。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_oscstep; /* B 通道往复旧键步进来自 B 通道识别参数。 */
			speed_max = ChannelrecognizeMessageB.speed_oscmax;   /* B 通道往复最大速度。 */
			speed_min = ChannelrecognizeMessageB.speed_oscmin;   /* B 通道往复最小速度。 */
		}
		else
		{
			return; /* 未知方向不写速度，避免把无效方向下的参数覆盖到通道记忆。 */
		}
	}
	else
	{
		return; /* 未选中 A/B 通道时不允许调速，避免写入无归属的 WorkMessage。 */
	}

	if (speed_step == 0U)
	{
		speed_step = SCREEN_SPEED_STEP_FALLBACK; /* 识别参数没有写步进时用 1000 兜底，避免新屏速度键按下无反馈。 */
	}

	switch (key_value)
	{
	case HANDLEKey_speed_add:
	case HMIkey_SPEED_Add:
	case SCREENKey_SPEED_Add:
		add_key = true; /* 旧手柄/HMI/屏幕速度加，继续使用通道识别步进。 */
		break;
	case HANDLEKey_speed_sub:
	case HMIkey_SPEED_Sub:
	case SCREENKey_SPEED_Sub:
		sub_key = true; /* 旧手柄/HMI/屏幕速度减，继续使用通道识别步进。 */
		break;
	case SCREENKey_SPEED_Add_Small:
		add_key = true; /* 新屏慢加直接使用当前方向寄存器步进。 */
		break;
	case SCREENKey_SPEED_Add_Large:
		speed_step = speed_step * SCREEN_SPEED_STEP_DOUBLE_FACTOR; /* 新屏快加为慢加步进两倍，使用32位避免24位速度上限被16位逻辑误截断。 */
		add_key = true; /* 本次按键方向为增加。 */
		break;
	case SCREENKey_SPEED_Sub_Small:
		sub_key = true; /* 新屏慢减直接使用当前方向寄存器步进。 */
		break;
	case SCREENKey_SPEED_Sub_Large:
		speed_step = speed_step * SCREEN_SPEED_STEP_DOUBLE_FACTOR; /* 新屏快减为慢减步进两倍，使用32位保持与24位速度边界一致。 */
		sub_key = true; /* 本次按键方向为减少。 */
		break;
	case HANDLEKey_greaI:
	case HANDLEKey_greaII:
	case HANDLEKey_greaIII:
	case HANDLEKey_greaIV:
	case HANDLEKey_greaV:
	default:
		return; /* 档位键当前仍保留不处理，避免误触发新屏固定步进。 */
	}

	if (add_key == true)
	{
		if (((uint32_t)speed_value + (uint32_t)speed_step) > (uint32_t)speed_max)
		{
			speed_value = speed_max; /* 增加越上限时钳到当前通道当前方向最大速度。 */
		}
		else
		{
			speed_value = speed_value + speed_step; /* 未越界时按本次步进增加，保持32位避免超过16位的目标速度回绕。 */
		}
	}
	else if (sub_key == true)
	{
		if ((speed_value <= speed_min) ||
			(((uint32_t)speed_value - (uint32_t)speed_min) <= (uint32_t)speed_step))
		{
			speed_value = speed_min; /* 减少越下限时钳到最小值，避免 uint16_t 下溢。 */
		}
		else
		{
			speed_value = speed_value - speed_step; /* 未越界时按本次步进减少，保持 32 位与 WorkMessage 速度单位一致。 */
		}
	}
	else
	{
		return; /* 理论保护分支，防止未知键值落入后写速度。 */
	}

	WorkMessage.speed_set_work = speed_value; /* 写回当前控制目标速度，驱动任务后续按该值输出。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		if (WorkMessage.dir_work == ZZDIR)
			MemoryMsgA.zz_speed = speed_value; /* A 通道正转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == FZDIR)
			MemoryMsgA.fz_speed = speed_value; /* A 通道反转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == OSCDIR)
			MemoryMsgA.osc_speed = speed_value; /* A 通道往复速度记忆同步更新。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		if (WorkMessage.dir_work == ZZDIR)
			MemoryMsgB.zz_speed = speed_value; /* B 通道正转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == FZDIR)
			MemoryMsgB.fz_speed = speed_value; /* B 通道反转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == OSCDIR)
			MemoryMsgB.osc_speed = speed_value; /* B 通道往复速度记忆同步更新。 */
	}
	display_value[0] = (uint8_t)(speed_value >> 16); /* 速度高字节按 UI_SPEED_ID 协议传输，Page4 24位最大速度会用到该高字节。 */
	display_value[1] = (uint8_t)((speed_value>>8) & 0xFFU); /* 速度低字节按 UI_SPEED_ID 协议传输，和切通道刷新保持一致。 */
	display_value[2] = (uint8_t)(speed_value & 0xFFU);
	display_value[3] = 1U; /* 1 表示只刷新速度数值，不重绘整个速度框，按键响应更快。 */
	display_value[4] = WorkMessage.runflag_work ? 1U : 0U; /* 保留当前运行态，运行中调速时字体颜色仍由 UIDP 按状态处理。 */
	SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 屏幕速度键成功改变设定值后立即显示新速度。 */

	if(WorkMessage.drivetype_work != JTWORK) /* 触控模式下速度键直接影响当前输出速度，非触控模式仅修改设定值等待后续运行命令生效。 */
	{
		WorkMessage.speed_work = WorkMessage.speed_set_work; 
	}
}
/*
 * 函数功能：处理屏幕或上位机的往复频率加减按键。
 * 输入参数：key_value 当前频率按键值，支持 HMI 和新屏频率加减。
 * 返回参数：无。
 */
void FreqActive(uint8_t key_value)
{
	uint8_t freq_value = (uint8_t)WorkMessage.freq_work;
	uint8_t freq_step = 5U;
	uint8_t freq_min = FreqMin;
	uint8_t freq_max = FreqMax;
	uint8_t display_value[10] = {0U};
	if ((WorkMessage.alarm_flag == true) ||
		(WorkMessage.hand_model == 0U) ||
		(Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model, WorkMessage.tool_type) == false))
	{
		return; /* 只有已识别且支持往复的当前手柄才允许调频，避免无手柄或普通手柄进入无效频率窗口。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		freq_min = (ChannelrecognizeMessageA.freq_min > FreqMin) ? ChannelrecognizeMessageA.freq_min : FreqMin; /* A 通道下限至少为 5Hz。 */
		freq_max = (ChannelrecognizeMessageA.freq_max != 0U) ? ChannelrecognizeMessageA.freq_max : FreqMax; /* A 通道未给上限时使用工程默认 40Hz。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		freq_min = (ChannelrecognizeMessageB.freq_min > FreqMin) ? ChannelrecognizeMessageB.freq_min : FreqMin; /* B 通道下限至少为 5Hz。 */
		freq_max = (ChannelrecognizeMessageB.freq_max != 0U) ? ChannelrecognizeMessageB.freq_max : FreqMax; /* B 通道未给上限时使用工程默认 40Hz。 */
	}
	else
	{
		return; /* 未选中 A/B 通道时没有可保存的频率记忆。 */
	}

	if (freq_max < freq_min)
	{
		freq_max = freq_min; /* 识别参数异常时钳到下限，避免后续上下限反向。 */
	}
	if (freq_value < freq_min)
	{
		freq_value = freq_min; /* 旧记忆低于 5Hz 时先抬到下限，再执行本次按键。 */
	}
	else if (freq_value > freq_max)
	{
		freq_value = freq_max; /* 旧记忆超过上限时先降到上限，避免加减计算下溢。 */
	}

	switch (key_value)
	{
	case HMIkey_FREQ_Add:
	case SCREENKey_FREQ_Add:
		if ((uint8_t)(freq_max - freq_value) <= freq_step)
		{
			freq_value = freq_max; /* 接近上限时直接钳到上限，避免 8 位加法溢出。 */
		}
		else
		{
			freq_value = (uint8_t)(freq_value + freq_step); /* 正常范围内按 5Hz 增加。 */
		}
		break;

	case HMIkey_FREQ_Sub:
	case SCREENKey_FREQ_Sub:
		if ((freq_value <= freq_min) || ((uint8_t)(freq_value - freq_min) <= freq_step))
		{
			freq_value = freq_min; /* 接近下限时直接钳到 5Hz 或通道下限，避免无符号下溢。 */
		}
		else
		{
			freq_value = (uint8_t)(freq_value - freq_step); /* 正常范围内按 5Hz 减少。 */
		}
		break;
	default:
		return; /* 非频率按键不更新记忆和 UI。 */
	}

	WorkMessage.freq_work = freq_value; /* 写回当前运行态频率。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		MemoryMsgA.freq = freq_value; /* A 通道同步频率记忆。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		MemoryMsgB.freq = freq_value; /* B 通道同步频率记忆。 */
	}
	display_value[0] = freq_value; /* UI_FREQ_ID Value[0] 传当前频率值。 */
	display_value[1] = 1U; /* 1 表示按键后只刷新数值，减少整块重绘造成的视觉闪烁。 */
	SendUIDSMessage(UI_FREQ_ID, true, display_value); /* 频率按键生效后立即刷新屏幕，避免等待下一次整页刷新。 */
}
/*
 * 函数功能：处理屏幕、手柄或上位机发起的正转、反转、往复方向切换。
 * 输入参数：key_value 当前方向按键值，支持 HANDLE/HMI/SCREEN 三类来源。
 * 返回参数：无。
 */
void DirActive(uint8_t key_value)
{
	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;
	if (WorkMessage.hand_model == 0U)
		return; /* EX8 表格要求方向键必须有有效手柄才生效，避免无手柄残留通道状态被方向键改写。 */
	if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
		return; /* 未选中 A/B 通道时没有可保存的方向记忆，直接拒绝方向键，避免屏幕显示被误刷新。 */
	switch (key_value)
	{
	case HANDLEKey_dir_Forward:
	case HMIkey_Dir_Forward:
	case SCREENKey_Dir_Forward:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = ZZDIR;
			WorkMessage.speed_set_work = MemoryMsgA.zz_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = ZZDIR;
			WorkMessage.speed_set_work = MemoryMsgB.zz_speed;
		}
		// 正向
		break;
	case HANDLEKey_dir_Reverse:
	case HMIkey_Dir_Reverse:
	case SCREENKey_Dir_Reverse:
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = FZDIR;
			WorkMessage.speed_set_work = MemoryMsgA.fz_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = FZDIR;
			WorkMessage.speed_set_work = MemoryMsgB.fz_speed;
		}
		// 反向
		break;
	case HANDLEKey_dir_OSC:
	case HMIkey_Dir_OSC:
	case SCREENKey_Dir_OSC:
		if (Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model, WorkMessage.tool_type) == false)
		{
			return; /* 当前选中手柄不支持往复时拒绝 OSCDIR，避免屏幕误发往复键后切入驱动不支持模式。 */
		}
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.dir_work = MemoryMsgA.dir = OSCDIR;
			WorkMessage.speed_set_work = MemoryMsgA.osc_speed;
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.dir_work = MemoryMsgB.dir = OSCDIR;
			WorkMessage.speed_set_work = MemoryMsgB.osc_speed;
		}
		// 往复
		break;
	}
	if ((WorkMessage.channel_work == CHANNEL_A) || (WorkMessage.channel_work == CHANNEL_B))
	{
		Pubinterface_RefreshSelectedChannelDisplay(WorkMessage.channel_work); /* 方向切换后同步刷新方向高亮、频率窗口和当前方向速度值。 */
	}
}
/*
 * 函数功能：处理屏幕或上位机发起的磨头/刨刀切换，并同步刷新当前通道显示。
 * 输入参数：key_value 为磨头或刨刀按键值。
 * 返回参数：无。
 */
void PlanerGridH(uint8_t key_value)
{
	bool tool_changed = false; /* 只在本次确实完成磨/刨类型切换后刷新界面，避免无关按键误触发重绘。 */

	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;
	if ((Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model) == false) ||
		(WorkMessage.auto_identify != 0U))
	{
		return; /* 手动磨/刨切换只开放给 PXBA/PXBB，且必须在手动识别模式下使用，自动 RFID 模式由标签结果决定刀具。 */
	}
	switch (key_value)
	{
	case SCREENKey_PlanerH: // 屏幕平面磨床水平控制按键
	case HMIkey_PlanerH:	// HMI平面磨床水平控制按键
		if (Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type))
		{
			return; /* 已经是刨刀能力时不重复刷新，避免同一按钮连续触发造成无意义 UI 消息堆积。 */
		}
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.tool_type = MemoryMsgA.tool_type = PLANER;
			WorkMessage.raw_tool_type = MemoryMsgA.raw_tool_type = 0U; /* 手动刨刀不是 RFID/EEPROM 原始型号，清掉旧标签码避免驱动误判。 */
			WorkMessage.freq_work = MemoryMsgA.freq = 40;
			WorkMessage.dir_work = OSCDIR;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.zz_speed=30000;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.fz_speed=30000;
			}
			else if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.osc_speed=30000;
			}
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.tool_type = MemoryMsgB.tool_type = PLANER;
			WorkMessage.raw_tool_type = MemoryMsgB.raw_tool_type = 0U; /* B 通道手动刨刀同样清原始码，保持 tool_type 为唯一能力来源。 */
			WorkMessage.freq_work=MemoryMsgB.freq = 40;
			WorkMessage.dir_work = OSCDIR;
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.zz_speed=30000;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.fz_speed=30000;
			}
			else if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.speed_set_work =MemoryMsgB.osc_speed=30000;
			}
		}
		tool_changed = true; /* 刨刀键已更新当前通道记忆，后续需要立即重绘刀具、方向、频率和开口定位。 */
		// 刨头
		break;
	case HMIkey_GrindH:	   // HMI磨头水平控制按键
	case SCREENKey_GrindH: // 屏幕磨头水平控制按键
		if (Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type) == false)
		{
			return; /* 当前已经不是刨刀能力时不重复切磨头，保持手动模式按钮幂等。 */
		}
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			WorkMessage.tool_type = MemoryMsgA.tool_type = GRINDH; /* EX8 磨头键必须切到磨头类型，避免继续按刨刀能力显示往复和开口定位。 */
			WorkMessage.raw_tool_type = MemoryMsgA.raw_tool_type = 0U; /* 手动磨头不使用上一次 RFID 原始码，避免有刷判断读取旧刀具。 */
			WorkMessage.freq_work = MemoryMsgA.freq;
			WorkMessage.dir_work = ZZDIR;
			if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.dir_work = MemoryMsgA.dir = ZZDIR; /* 磨头不显示往复能力，若从刨刀往复切来则回落正转，避免内部仍停在 OSCDIR。 */
			}
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work=MemoryMsgA.zz_speed = 60000; /* 磨头正转默认速度 60000，区别于刨刀的 30000，且不使用磨头记忆速度，避免切回刨刀时磨头速度过快。 */
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgA.fz_speed=60000;
			}
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			WorkMessage.tool_type = MemoryMsgB.tool_type = GRINDH; /* B 通道同样保存磨头类型，保证切回通道后界面和能力判断一致。 */
			WorkMessage.raw_tool_type = MemoryMsgB.raw_tool_type = 0U; /* B 通道手动磨头同步清原始码，保持后续运行只按手柄基座判断。 */
			WorkMessage.freq_work = MemoryMsgB.freq;
			WorkMessage.dir_work = ZZDIR;
			if (WorkMessage.dir_work == OSCDIR)
			{
				WorkMessage.dir_work = MemoryMsgB.dir = ZZDIR; /* B 通道磨头同样禁止保留往复方向，保证屏幕禁用态和运行方向一致。 */
			}
			if (WorkMessage.dir_work == ZZDIR)
			{
				WorkMessage.speed_set_work =MemoryMsgB.zz_speed = 60000;
			}
			else if (WorkMessage.dir_work == FZDIR)
			{
				WorkMessage.speed_set_work = MemoryMsgB.fz_speed=60000;;
			}
		}
		tool_changed = true; /* 磨头键已更新当前通道记忆，后续需要关闭频率和开口定位并重绘速度。 */
		// 磨头
		break;
	}

	if (tool_changed &&
		((WorkMessage.channel_work == CHANNEL_A) || (WorkMessage.channel_work == CHANNEL_B)))
	{
		Pubinterface_RefreshSelectedChannelDisplay(WorkMessage.channel_work); /* 磨/刨切换后立即刷新 EX8 主运行页，避免按钮状态和内部 tool_type 不一致。 */
	}
}

/*
 * 函数功能：处理新屏“自动识别刀具”按钮，按当前工作通道触发 RFID 读取。
 * 输入参数：key_value 为 SCREENKey_AutoIdentify；保留参数用于和其它按键处理函数保持一致。
 * 返回参数：无。
 */
void AutoIdentifyActive(uint8_t key_value)
{
	ChannelMemoryMessagr_t *memory = NULL; /* 指向当前 A/B 通道记忆，自动/手动模式要和切通道后的恢复状态保持一致。 */
	uint8_t current_channel = WorkMessage.channel_work; /* 记录当前工作通道，后续刷新界面和触发 RFID 都按这个通道执行。 */

	if (key_value != SCREENKey_AutoIdentify)
	{
		return; /* 只接收新屏自动识别键，避免其它旧键误进入 RFID 流程。 */
	}

	if (WorkMessage.runflag_work == true)
	{
		return; /* 电机运行中不启动 RFID 识别，避免识别流程改变当前刀具参数。 */
	}

	if (WorkMessage.alarm_flag == true)
	{
		return; /* 系统报警未解除时不发起自动识别，保持报警优先级。 */
	}

	if (Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model) == false)
	{
		return; /* 自动/手动识别切换只开放给 PXBA/PXBB 分体手柄，其它手柄的刀具类型由 EEPROM 或 EPC 流程决定。 */
	}

	if (current_channel == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* A 通道自动识别状态写入 A 记忆，切换到 B 再切回 A 时能恢复。 */
	}
	else if (current_channel == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* B 通道自动识别状态写入 B 记忆，避免 A/B 通道模式串用。 */
	}
	else
	{
		return; /* 未选中 A/B 通道时不触发 RFID，避免识别结果没有通道归属。 */
	}

	if (WorkMessage.auto_identify != 0U)
	{
		WorkMessage.auto_identify = 0U; /* 再次点击自动识别键时切回手动模式，屏幕重新显示磨/刨按钮。 */
		memory->auto_identify = 0U; /* 通道记忆同步保存手动模式，后续切通道不再误隐藏手动按钮。 */
		WorkMessage.tool_type = memory->tool_type = GRINDH; /* EX8 表格要求手动模式默认选择磨头，刨刀按钮保持可选。 */
		WorkMessage.raw_tool_type = memory->raw_tool_type = 0U; /* 切到手动识别后清除 RFID 原始码，避免旧刀具头影响驱动判断。 */
		if ((memory->dir != ZZDIR) && (memory->dir != FZDIR))
		{
			memory->dir = ZZDIR; /* 从自动识别或刨刀往复状态切回手动磨头时，方向回到正转，避免保留 OSCDIR。 */
		}
		WorkMessage.dir_work = memory->dir= ZZDIR; /* 当前工作方向跟随通道记忆，确保 UI 和后续驱动命令一致。 */
		WorkMessage.freq_work = memory->freq; /* 频率值保留在记忆里，但磨头模式刷新时会隐藏频率窗口。 */
		WorkMessage.speed_set_work =  memory->fz_speed = memory->zz_speed=60000; /* 手动磨头只使用正/反转速度，避免读取往复速度。 */
		Pubinterface_RefreshSelectedChannelDisplay(current_channel); /* 手动模式立即刷新磨头选中、刨刀使能、频率隐藏和开口定位隐藏。 */
		return;
	}

	WorkMessage.auto_identify = 1U; /* 从手动模式切到自动模式，当前工作态进入 RFID 自动识别。 */
	memory->auto_identify = 1U; /* 通道记忆同步保存自动模式，切换通道后仍能按自动识别界面显示。 */
	if (current_channel == CHANNEL_A)
	{
		s_rfid_last_tool_type_a = 0U; /* 用户主动重新进入自动识别时，从等待状态开始显示 63，不沿用上一次离线图标。 */
	}
	else if (current_channel == CHANNEL_B)
	{
		s_rfid_last_tool_type_b = 0U; /* B 通道同样清掉上一次离线图标，避免新一轮识别前误显示 61/62。 */
	}
	if (current_channel == CHANNEL_A)
	{
		SendKeyRFIDMessageAup(1U); /* A 通道 PXBA/PXBB 自动识别读取 USER 区，刀具头参数来自 RFID 标签。 */
	}
	else
	{
		SendKeyRFIDMessageBup(1U); /* B 通道 PXBA/PXBB 自动识别读取 USER 区，保持 A/B RFID 请求分离。 */
	}
	Pubinterface_RefreshSelectedChannelDisplay(current_channel); /* 自动模式立即隐藏手动磨/刨按钮，等待 RFID 规格刷新。 */
}

/*
 * 函数功能：处理屏幕、脚踏或上位机发起的 A/B 手柄手动切换。
 * 输入参数：key_value 切换按键值，支持 SCREENKey/HMIkey A/B 和脚踏中键长按。
 * 返回参数：无。
 */
void HandleSwitchActive(uint8_t key_value) // 2026,4,19
{
	uint8_t target_channel = CHANNEL_NONE; /* 保存本次用户确认想切到的目标通道，默认不切换。 */

	if ((WorkMessage.runflag_work == true) || (WorkMessage.alarm_flag == true))
	{
		return; /* 运行中或系统报警中不允许切换，避免两个手柄同时参与输出。 */
	}

	if ((key_value == HMIkey_HANDLE_A) || (key_value == SCREENKey_HANDLE_A))
	{
		target_channel = CHANNEL_A; /* 屏幕或上位机明确点 A 时，目标通道就是 A。 */
	}
	else if ((key_value == HMIkey_HANDLE_B) || (key_value == SCREENKey_HANDLE_B))
	{
		target_channel = CHANNEL_B; /* 屏幕或上位机明确点 B 时，目标通道就是 B。 */
	}
	else if (key_value == JTKey_middle_long)
	{
		if ((WorkMessage.channel_work == CHANNEL_A) && (WorkMessage.Channel_Bonline == true))
		{
			target_channel = CHANNEL_B; /* 当前选中 A 且 B 在线时，脚踏长按只切到 B。 */
		}
		else if ((WorkMessage.channel_work == CHANNEL_B) && (WorkMessage.Channel_Aonline == true))
		{
			target_channel = CHANNEL_A; /* 当前选中 B 且 A 在线时，脚踏长按只切到 A。 */
		}
	}

	if ((target_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == true))
	{
		Pubinterface_LoadChannelMemory(CHANNEL_A); /* 用户确认切到 A 后，把 A 通道记忆装载为当前工作快照。 */
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* 按 A 通道工作快照刷新控制模式、刀具、方向、频率和速度显示。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 A 被点亮。 */
	}
	else if ((target_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == true))
	{
		Pubinterface_LoadChannelMemory(CHANNEL_B); /* 用户确认切到 B 后，把 B 通道记忆装载为当前工作快照。 */
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* 按 B 通道工作快照刷新控制模式、刀具、方向、频率和速度显示。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 B 被点亮。 */
	}
}

/*
 * 函数功能：处理 A/B 手柄插入和拔出事件；插入先写 MemoryMsg，只有策略允许时才装载到 WorkMessage。
 * 输入参数：key_value 插拔事件按键值，SCREENKey_PLUG_A/B 表示上线，SCREENKey_UNPLUG_A/B 表示离线。
 * 返回参数：无。
 */
void PlugORunPLUGActive(uint8_t key_value)
{
	uint8_t current_channel_unplugged = 0U; /* 记录拔出的是否为当前选中通道，用于防止自动切到另一通道。 */

	switch (key_value)
	{
	case SCREENKey_PLUG_A: // 插入A
		WorkMessage.Channel_Aonline = true;					   /* A 通道校验通过后才置在线，后续心跳和显示都读取这个标志。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_A);		   /* 扫描结果只先进入 MemoryMsgA，运行中不会直接覆盖 WorkMessage。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_A))
		{
			Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非运行状态下最后插入且校验通过的 A 通道成为当前选中通道。 */
			Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* A 被选中时，才用 A 的 Page4 默认流量初始化注水泵。 */
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* A 自动成为当前通道后，同步刷新参数区和刀具规格。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 A 在线图标和当前高亮状态。 */
		Pubinterface_ClearCommonSocketToolMissingAlarm();	   /* A 通道 EPC 刀具信息装载成功后清除公共接头缺刀具临时报警。 */
		break;

	case SCREENKey_PLUG_B: // 插入B
		WorkMessage.Channel_Bonline = true;					   /* B 通道校验通过后才置在线，坏手柄不会进入在线态。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_B);		   /* 扫描结果只先进入 MemoryMsgB，避免运行中插入 B 抢占 A。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_B))
		{
			Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非运行状态下最后插入且校验通过的 B 通道成为当前选中通道。 */
			Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* B 被选中时，才用 B 的 Page4 默认流量初始化注水泵。 */
			Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* B 自动成为当前通道后，同步刷新参数区和刀具规格。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 B 在线图标和当前高亮状态。 */
		Pubinterface_ClearCommonSocketToolMissingAlarm();	   /* B 通道 EPC 刀具信息装载成功后清除公共接头缺刀具临时报警。 */
		break;

	case SCREENKey_UNPLUG_A: // 拔出A
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_A); /* 先记录拔出前 A 是否为当前选中通道。 */
		WorkMessage.Channel_Aonline = false;					   /* A 拔出后立刻离线，心跳会报告 A 不可用。 */
		memset(&MemoryMsgA, 0, sizeof(MemoryMsgA));			   /* A 离线时清空 A 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U)
		{
			if ((WorkMessage.runflag_work == false) &&
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Bonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非工作状态拔掉当前 A 时，若 B 仍在线，则回落选中 B，保持“非工作态有在线手柄即有选中通道”。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* B 成为当前选中通道后，用 B 的 Page4 默认流量补齐仍为 0 的注水泵。 */
				Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* 回落到 B 后刷新参数区，清掉 A 通道残留显示。 */
			}
			else
			{
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 B 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 B 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.raw_tool_type = 0U;				   /* 同步清当前原始刀具型号，避免拔出后驱动侧读取旧 PXM/PXP/RFID 代号。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* 工作中当前通道拔出仍不自动切到 B，等待用户手动确认。 */
				Pubinterface_ClearSelectedChannelDisplay();	   /* 没有当前通道时关闭参数区，避免屏幕保留离线通道信息。 */
			}
		}
		Pubinterface_SendHandleDisplay(CHANNEL_A, 0U, false, false); /* A 通道拔出后立即暗灭 A 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 B 仍在线，只显示 B 在线但不因 A 拔出自动高亮 B。 */
		break;

	case SCREENKey_UNPLUG_B: // 拔出B
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_B); /* 先记录拔出前 B 是否为当前选中通道。 */
		WorkMessage.Channel_Bonline = false;					   /* B 拔出后立刻离线，心跳会报告 B 不可用。 */
		memset(&MemoryMsgB, 0, sizeof(MemoryMsgB));			   /* B 离线时清空 B 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U)
		{
			if ((WorkMessage.runflag_work == false) &&
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Aonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非工作状态拔掉当前 B 时，若 A 仍在线，则回落选中 A，匹配现场先插 A 再插 B 再拔 B 的预期。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* A 成为当前选中通道后，用 A 的 Page4 默认流量补齐仍为 0 的注水泵。 */
				//Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* 回落到 A 后刷新参数区，清掉 B 通道残留显示。 */
			}
			else
			{
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 A 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 A 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.raw_tool_type = 0U;				   /* 同步清当前原始刀具型号，避免拔出后驱动侧读取旧 PXM/PXP/RFID 代号。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* 工作中当前通道拔出仍不自动切到 A，等待用户手动确认。 */
				Pubinterface_ClearSelectedChannelDisplay();	   /* 没有当前通道时关闭参数区，避免屏幕保留离线通道信息。 */
			}
		}
		Pubinterface_SendHandleDisplay(CHANNEL_B, 0U, false, false); /* B 通道拔出后立即暗灭 B 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 A 仍在线，只显示 A 在线但不因 B 拔出自动高亮 A。 */
		break;

	default:
		break;												   /* 其它按键不是插拔事件，本函数不处理。 */
	}
}

void ToolPosActive(uint8_t key_value)
{

	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true)
		return;
	if (WorkMessage.hand_model == 0U)
		return; /* 未识别有效手柄时不允许开口定位，避免屏幕残留刀具状态误触发驱动。 */
	if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
		return; /* ToolPosMay 底层非 A 会落到 B，所以上层必须先确认当前通道是 A 或 B。 */

	if (Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model, WorkMessage.tool_type) == false)
	{
		return; /* 开口定位只给 PXBA/PXBB 且刀具为 PLANER 的组合开放，磨头或非分体手柄直接拦截。 */
	}

	switch (key_value)
	{
	case HMIkey_OpenPos_ClockWise:
	case SCREENKey_OpenPos_ClockWise:
		ToolPosMay(WorkMessage.channel_work, true, 1U); /* 顺时针开口定位，角度固定 1 度。 */
		break;
	case JTKey_middle_short:
	case HMIkey_OpenPos_AntiClockWise:
	case SCREENKey_OpenPos_AntiClockWise:
		ToolPosMay(WorkMessage.channel_work, false, 1U); /* 逆时针开口定位，角度固定 1 度。 */
		break;
	default:
		break; /* 其它按键不触发开口定位，避免误发驱动命令。 */
	}
}

/*
 * 函数功能：按 EX8 屏幕交互表判断屏幕泵按键是否应被拒绝。
 * 输入参数：key_value 为屏幕解析后的 A/B 泵加、减、启停逻辑键值。
 * 返回参数：true 表示本次屏幕泵键不允许继续修改泵状态；false 表示可继续进入 PUMPActive 原有业务分支。
 */
static bool Pubinterface_ShouldRejectScreenPumpKey(uint8_t key_value)
{
	switch (key_value)
	{
	case SCREENKey_APUMP_Add:
	case SCREENKey_APUMP_Sub:
		if (pumpMessageA.online_flag == false)
		{
			Pubinterface_RefreshPumpADisplay(); /* A 泵离线时只重刷为不可用状态，不允许屏幕用旧 type 残留继续调速。 */
			return true; /* 屏幕表格要求 A 泵在线后加减流量才有效。 */
		}
		if (WorkMessage.alarm_flag == true)
		{
			return true; /* 全局报警期间屏幕 A 泵加减不生效，避免报警态继续改变输出设定。 */
		}
		if (pumpMessageA.timingDrainage_flag == true)
		{
			return true; /* A 注水泵排空期间禁止加减流量，保持排空过程稳定。 */
		}
		return false; /* A 泵在线、无报警、非排空时允许继续进入加减速分支。 */

	case SCREENKey_BPUMP_Add:
	case SCREENKey_BPUMP_Sub:
		if (pumpMessageB.online_flag == false)
		{
			Pubinterface_RefreshPumpBDisplay(); /* B 泵离线时只刷新屏幕，避免旧速度被屏幕按钮继续修改。 */
			return true; /* 屏幕表格要求 B 泵在线后加减流量才有效。 */
		}
		if (WorkMessage.alarm_flag == true)
		{
			return true; /* 报警态下 B 泵屏幕调速不生效，和 A 泵屏幕规则保持一致。 */
		}
		if (pumpMessageB.timingDrainage_flag == true)
		{
			return true; /* B 注水泵排空期间禁止加减流量，避免排空输出被现场误调。 */
		}
		return false; /* B 泵在线、无报警、非排空时允许继续进入加减速分支。 */

	case SCREENKey_APUMP_control:
		if (pumpMessageA.online_flag == false)
		{
			pumpMessageA.run_flag = false; /* A 泵离线时强制撤销屏幕侧运行状态，避免离线后按钮仍显示运行。 */
			pumpMessageA.timingDrainage_flag = false; /* 同步取消 A 排空状态，下一次上线后重新按屏幕键开始。 */
			Pubinterface_RefreshPumpADisplay(); /* 立即把 A 泵区域刷新为不可用。 */
			return true; /* 屏幕表格要求 A 泵在线后启停才有效。 */
		}
		if (WorkMessage.alarm_flag == true)
		{
			return true; /* 全局报警期间屏幕 A 泵启停不生效，保持报警处理优先级。 */
		}
		if ((pumpMessageA.type == INJECTWATER) && (WorkMessage.runflag_work == true))
		{
			return true; /* 注水泵在手柄电机运行时可能被冷却联动占用，屏幕排空/启停必须等待手柄停机。 */
		}
		return false; /* A 泵在线、无报警且未被手柄运行占用时允许启停。 */

	case SCREENKey_BPUMP_control:
		if (pumpMessageB.online_flag == false)
		{
			pumpMessageB.run_flag = false; /* B 泵离线时撤销运行标志，防止屏幕继续显示可控运行态。 */
			pumpMessageB.timingDrainage_flag = false; /* 同步取消 B 排空状态，避免离线期间计时残留。 */
			Pubinterface_RefreshPumpBDisplay(); /* 立即把 B 泵区域刷新为不可用。 */
			return true; /* 屏幕表格要求 B 泵在线后启停才有效。 */
		}
		if (WorkMessage.alarm_flag == true)
		{
			return true; /* 全局报警期间屏幕 B 泵启停不生效。 */
		}
		if ((pumpMessageB.type == INJECTWATER) && (WorkMessage.runflag_work == true))
		{
			return true; /* 注水泵在手柄运行时不能由屏幕进入排空/启停，避免和手柄冷却联动冲突。 */
		}
		return false; /* B 泵在线、无报警且未被手柄运行占用时允许启停。 */

	default:
		return false; /* 非屏幕泵键保持原有脚踏/HMI 处理路径。 */
	}
}

/*
 * 函数功能：处理脚踏、屏幕和 HMI 的 A/B 泵档位、启停、轻排和排空控制。
 * 输入参数：key_value 触发泵控制的业务按键值。
 * 返回参数：无。
 */
/*
 * 函数功能：处理脚踏、屏幕和 HMI 上位机来源的 A/B 泵档位、启停和加减速按键。
 * 输入参数：key_value 表示当前泵控制按键值，函数内部按来源申请控制权并修改 pumpMessageA/B。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value)
{
	static uint8_t PumpA_Gear = 0;
	static uint8_t PumpB_Gear = 0;
	uint8_t pump_owner = ControlArbitration_GetOwnerByPumpKey(key_value);
	bool is_timed_drainage = false; /* 仅脚踏长按排空需要置位定时排空，普通上位机/屏幕泵启动不能共用该状态。 */
	uint16_t pump_speed_max = 0U; /* 当前加减速分支使用的类型上限，避免未初始化 speed_Max 把速度夹成 0。 */
	uint16_t pump_speed_min = 0U; /* 当前加减速分支使用的类型下限，默认 0 可防止无符号减法下溢。 */

	if (Pubinterface_ShouldRejectScreenPumpKey(key_value))
	{
		return; /* EX8 屏幕泵键不满足在线/报警/排空/手柄运行条件时，蜂鸣保留但业务状态不变。 */
	}

	switch (key_value)
	{
	case JTkey_left_short:
		PumpA_Gear++;
		if (PumpA_Gear > 5)
			PumpA_Gear = 0;
		if (pumpMessageA.type == 0)
		{
			PumpA_Gear = 0; // 队列通知界面暗黑
			return;
		}
		else if (pumpMessageA.type == DRAWWATER) // 抽水
		{
			pumpMessageA.speed_work = 3 * PumpA_Gear; // 每一档位增加30ml水
		}
		else if (pumpMessageA.type == INJECTWATER) // 注水
		{
			if (pumpMessageA.timingDrainage_flag == true)
				return;
			switch (PumpA_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
				pumpMessageA.speed_work = PumpA_Gear * 10;
				break;
			case PUMPGEAR_IV:
				pumpMessageA.speed_work = 50;
				break;
			case PUMPGEAR_V:
				pumpMessageA.speed_work = 70;
				break;
			}
		}
		else if (pumpMessageA.type == POURWATER) // 灌注
		{

			switch (PumpA_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
			case PUMPGEAR_IV:
				pumpMessageA.speed_work = 50 * PumpA_Gear; // 每一档位增加50ml水
				break;
			case PUMPGEAR_V:
				pumpMessageA.speed_work = 300;
				break;
			}
		}
		Pubinterface_RefreshPumpADisplay(); /* A 泵加减速后同步刷新流量数值和按钮状态。 */
		// 队列通知ui更新界面
		break;
	case JTKey_left_long:
	case HMIkey_APUMP_control:
	case SCREENKey_APUMP_control:
		if (pumpMessageA.type == 0)
		{
			pumpMessageA.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			Pubinterface_RefreshPumpADisplay(); /* A 泵未识别时立即暗掉屏幕 A 泵区域和按钮，避免显示可启动。 */
			// 队列A停止
			return;
		}
		else
		{
			if (!pumpMessageA.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				if (pumpMessageA.speed_work == 0U)
				{
					//pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 按泵类型补启动速度，避免灌注/抽吸泵只置 run_flag 但 UART 输出 0 速。 */
				}
				//is_timed_drainage = (key_value == JTKey_left_long); /* A 泵只有脚踏长按才进入定时排空，上位机/屏幕普通启动保持连续运行。 */
				
				is_timed_drainage=true;
				pumpMessageA.run_flag = true;
				if (pumpMessageA.type == INJECTWATER)
				{
					pumpMessageA.timingDrainage_flag = is_timed_drainage;
					pumpMessageA.timingDrainage_times = 0U; /* 每次重新启动都清排空计时，避免继承上一轮剩余计数导致刚启动就停泵。 */
				}
				// 队列发送界面按钮和数字变黄，A
				// 队列发送泵运行设置数据
			}
			else
			{
				pumpMessageA.run_flag = false;
				pumpMessageA.timingDrainage_flag = false;
				pumpMessageA.timingDrainage_times = 0U;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑,A
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		//Pubinterface_RefreshPumpADisplay(); /* A 泵启停后立即刷新数值和按钮运行态，运行数据仍由 pumpMessageA 驱动。 */
		break;
	case JTKey_right_short:
		PumpB_Gear++;
		if (PumpB_Gear > 5)
			PumpB_Gear = 0;
		if (pumpMessageB.type == 0)
		{
			PumpB_Gear = 0;
			// 队列通知界面暗黑
		}
		else if (pumpMessageB.type == DRAWWATER) // 抽水
		{
			pumpMessageB.speed_work = 3 * PumpB_Gear; // 每一档位增加30ml水
		}
		else if (pumpMessageB.type == INJECTWATER) // 注水
		{
			switch (PumpB_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
				pumpMessageB.speed_work = PumpB_Gear * 10;
				break;
			case PUMPGEAR_IV:
				pumpMessageB.speed_work = 50;
				break;
			case PUMPGEAR_V:
				pumpMessageB.speed_work = 70;
				break;
			}
		}
		else if (pumpMessageB.type == POURWATER) // 灌注
		{
			switch (PumpB_Gear)
			{
			case PUMPGEAR_ZERO:
			case PUMPGEAR_I:
			case PUMPGEAR_II:
			case PUMPGEAR_III:
			case PUMPGEAR_IV:
				pumpMessageB.speed_work = 50 * PumpB_Gear; // 每一档位增加50ml水
				break;
			case PUMPGEAR_V:
				pumpMessageB.speed_work = 300; // 最大300ml水，记得宏定义
				break;
			}
		}
		Pubinterface_RefreshPumpBDisplay(); /* B 泵档位变化后同步刷新流量数值和按钮状态。 */
		break;
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		if (pumpMessageB.type == 0)
		{
			pumpMessageB.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			Pubinterface_RefreshPumpBDisplay(); /* B 泵未识别时立即暗掉屏幕 B 泵区域和按钮，避免显示可启动。 */
			// 对列通知为0;
			return;
		}
		else
		{
			if (!pumpMessageB.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* B 泵按类型补启动速度，保证屏幕直接启动灌注泵时能实际输出。 */
				}
				//is_timed_drainage = (key_value == JTKey_right_long); /* B 泵同样只有脚踏长按进入定时排空，普通外部/屏幕泵控制保持常规运行。 */
				is_timed_drainage=true;
				pumpMessageB.run_flag = true;
				if (pumpMessageB.type == INJECTWATER)
				{
					pumpMessageB.timingDrainage_flag = is_timed_drainage;
					pumpMessageB.timingDrainage_times = 0U; /* 清掉旧排空计数，防止第二次启动被历史计时立即关断。 */
				}
				// 队列发送界面按钮和数字变黄
				// 队列发送泵运行设置数据
			}
			else
			{
				pumpMessageB.timingDrainage_flag = false;
				pumpMessageB.timingDrainage_times = 0U;
				pumpMessageB.run_flag = false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		//Pubinterface_RefreshPumpBDisplay(); /* B 泵启停后立即刷新数值和按钮运行态，运行数据仍由 pumpMessageB 驱动。 */
		break;
	case HMIkey_APUMP_Add:
	case SCREENKey_APUMP_Add:
	case HMIkey_APUMP_Sub:
	case SCREENKey_APUMP_Sub:
		switch (pumpMessageA.type)
		{
		case DRAWWATER: // 抽
			pumpMessageA.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			if (pumpMessageA.timingDrainage_flag == true)
				return;
			pumpMessageA.speed_step_value = 5;
			break;
		case POURWATER: // 灌
			pumpMessageA.speed_step_value = 30;
			break;
		default:
			Pubinterface_RefreshPumpADisplay(); /* A 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pump_speed_max = Pubinterface_GetPumpSpeedMax(&pumpMessageA); /* A 泵调速上限按类型兜底，解决 speed_Max 未初始化导致加速回 0 的问题。 */
		pump_speed_min = Pubinterface_GetPumpSpeedMin(&pumpMessageA); /* A 泵调速下限来自配置，当前未配置时保持 0。 */
		if (key_value == HMIkey_APUMP_Add || key_value == SCREENKey_APUMP_Add)
		{
			if (pumpMessageA.speed_work >= pump_speed_max)
			{
				pumpMessageA.speed_work = pump_speed_max; /* 已到上限时保持上限，避免继续加速产生无效显示值。 */
			}
			else if ((uint16_t)(pump_speed_max - pumpMessageA.speed_work) <= pumpMessageA.speed_step_value)
			{
				pumpMessageA.speed_work = pump_speed_max; /* 距离上限不足一个步进时直接贴上限，避免无符号加法溢出。 */
			}
			else
			{
				pumpMessageA.speed_work += pumpMessageA.speed_step_value; /* 正常范围内按当前泵类型步进增加速度。 */
			}
		}
		else
		{
			if (pumpMessageA.speed_work <= pump_speed_min)
			{
				pumpMessageA.speed_work = pump_speed_min; /* 已到下限时保持下限，避免继续减速发生无符号下溢。 */
			}
			else if ((uint16_t)(pumpMessageA.speed_work - pump_speed_min) <= pumpMessageA.speed_step_value)
			{
				pumpMessageA.speed_work = pump_speed_min; /* 距离下限不足一个步进时直接贴下限，避免 0 减 30 变成 65506。 */
			}
			else
			{
				pumpMessageA.speed_work -= pumpMessageA.speed_step_value; /* 正常范围内按当前泵类型步进降低速度。 */
			}
		}
		if (pumpMessageA.run_flag == true)
		{
			// 队列通知A泵运行设置数据
		}
		Pubinterface_RefreshPumpADisplay(); /* A 泵加减速后同步刷新流量数值和按钮状态。 */
		break;

	case HMIkey_BPUMP_Add:
	case SCREENKey_BPUMP_Add:
	case HMIkey_BPUMP_Sub:
	case SCREENKey_BPUMP_Sub:
		switch (pumpMessageB.type)
		{
		case DRAWWATER: // 抽
			pumpMessageB.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			if (pumpMessageB.timingDrainage_flag == true)

				return;
			pumpMessageB.speed_step_value = 5;
			break;
		case POURWATER: // 灌
			pumpMessageB.speed_step_value = 30;
			break;
		default:
			Pubinterface_RefreshPumpBDisplay(); /* B 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pump_speed_max = Pubinterface_GetPumpSpeedMax(&pumpMessageB); /* B 泵调速上限按类型兜底，解决 speed_Max 未初始化导致屏幕加速仍显示 0 的问题。 */
		pump_speed_min = Pubinterface_GetPumpSpeedMin(&pumpMessageB); /* B 泵调速下限来自配置，当前未配置时保持 0。 */
		if (key_value == HMIkey_BPUMP_Add || key_value == SCREENKey_BPUMP_Add)
		{
			if (pumpMessageB.speed_work >= pump_speed_max)
			{
				pumpMessageB.speed_work = pump_speed_max; /* 已到上限时保持上限，避免继续加速产生无效显示值。 */
			}
			else if ((uint16_t)(pump_speed_max - pumpMessageB.speed_work) <= pumpMessageB.speed_step_value)
			{
				pumpMessageB.speed_work = pump_speed_max; /* 距离上限不足一个步进时直接贴上限，避免无符号加法溢出。 */
			}
			else
			{
				pumpMessageB.speed_work += pumpMessageB.speed_step_value; /* 正常范围内按当前泵类型步进增加速度。 */
			}
		}
		else
		{
			if (pumpMessageB.speed_work <= pump_speed_min)
			{
				pumpMessageB.speed_work = pump_speed_min; /* 已到下限时保持下限，避免继续减速发生无符号下溢。 */
			}
			else if ((uint16_t)(pumpMessageB.speed_work - pump_speed_min) <= pumpMessageB.speed_step_value)
			{
				pumpMessageB.speed_work = pump_speed_min; /* 距离下限不足一个步进时直接贴下限，避免 0 减 30 变成 65506。 */
			}
			else
			{
				pumpMessageB.speed_work -= pumpMessageB.speed_step_value; /* 正常范围内按当前泵类型步进降低速度。 */
			}
		}

		if (pumpMessageB.run_flag == true)
		{
			// 队列通知B泵运行设置数据
		}
		Pubinterface_RefreshPumpBDisplay(); /* B 泵加减速后同步刷新流量数值和按钮状态。 */
		break;
	case JTKey_Gently_left_start: // 其实可以判断手柄类型决定是否给与注水
		/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		if (pumpMessageA.type == INJECTWATER) // 注水
		{
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 轻排启动也要补非零注水流量，避免只置运行标志但泵不转。 */
				}
				pumpMessageA.run_flag = true;
			}
		}
		else if (pumpMessageB.type == INJECTWATER)
		{

			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* B 注水泵轻排启动时补非零流量，保持和屏幕/外控启泵一致。 */
				}
				pumpMessageB.run_flag = true;
				// 队列发送启动B
			}
		}
		Pubinterface_RefreshPumpADisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		Pubinterface_RefreshPumpBDisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		break;
	case JTKey_Gently_left_stop:
		if (pumpMessageA.type == INJECTWATER) // 注水
		{
			// 队列通知A泵停
			pumpMessageA.run_flag = false;
			;
		}
		else if (pumpMessageB.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageB.run_flag = false;
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		Pubinterface_RefreshPumpADisplay(); /* 轻排停止后同步 A 泵显示，避免按钮仍保持运行态。 */
		Pubinterface_RefreshPumpBDisplay(); /* 轻排停止后同步 B 泵显示，避免按钮仍保持运行态。 */
		break;
	case JTKey_Gently_right_start:
		/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		if (pumpMessageB.type == INJECTWATER) // 注水
		{
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* 右轻排启动 B 注水泵时补非零流量，避免 0 速不转。 */
				}
				pumpMessageB.run_flag = true;
			}
		}
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列发送
			if (!WorkMessage.channel_work) // 手柄在线，就可以运行A泵
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 右轻排回退启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.run_flag = true;
			}
		}

		Pubinterface_RefreshPumpADisplay(); /* 右轻排可能影响 A 或 B 注水泵，两个泵区一起刷新。 */
		Pubinterface_RefreshPumpBDisplay(); /* 右轻排可能影响 A 或 B 注水泵，两个泵区一起刷新。 */
		break;
	case JTKey_Gently_rigth_stop:
		if (pumpMessageB.type == INJECTWATER) // 注水
		{
			// 队列通知A泵停
			pumpMessageB.run_flag = false;
			;
		}
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageA.run_flag = false;
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		Pubinterface_RefreshPumpADisplay(); /* 右轻排停止后同步 A 泵显示，避免按钮仍保持运行态。 */
		Pubinterface_RefreshPumpBDisplay(); /* 右轻排停止后同步 B 泵显示，避免按钮仍保持运行态。 */
		break;
	case HMIkey_Gently_start:
		/* 历史 HMI 轻排属于外部来源，启动前也必须通过统一控制权仲裁。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;

		if (WorkMessage.channel_work == 1)
		{
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 轻排启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.run_flag = true;
			}
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 轻排启动 B 注水泵时补非零流量。 */
				}
				pumpMessageB.run_flag = true;
			}
		}
		else if (WorkMessage.channel_work == 2)
		{
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				if (pumpMessageB.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 当前 B 通道轻排时补 B 注水泵启动流量。 */
				}
				pumpMessageB.run_flag = true;
			}
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				if (pumpMessageA.timingDrainage_flag == true) // 如果正在排空
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 当前 B 通道但 A 是注水泵时补 A 启动流量。 */
				}
				pumpMessageA.run_flag = true;
			}
		}

		Pubinterface_RefreshPumpADisplay(); /* HMI 轻排启动后刷新 A 泵显示，保持上位机和屏幕状态一致。 */
		Pubinterface_RefreshPumpBDisplay(); /* HMI 轻排启动后刷新 B 泵显示，保持上位机和屏幕状态一致。 */
		break;
	case HMIkey_Gently_stop:
		if (WorkMessage.channel_work == 1)
		{
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送停止A
				pumpMessageA.run_flag = false;
			}
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
			}
		}
		else if (WorkMessage.channel_work == 2)
		{
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
			}
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列停止B
				pumpMessageA.run_flag = false;
			}
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		Pubinterface_RefreshPumpADisplay(); /* HMI 轻排停止后刷新 A 泵显示，避免旧运行态残留。 */
		Pubinterface_RefreshPumpBDisplay(); /* HMI 轻排停止后刷新 B 泵显示，避免旧运行态残留。 */

		break;
	default:
		break;
	}
}
