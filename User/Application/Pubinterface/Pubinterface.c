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
#include "handlescan.h"
#include "motoruartdata.h"
#include "external_comm_task.h"

ChannelrecognizeMessage_t ChannelrecognizeMessageA;
ChannelrecognizeMessage_t ChannelrecognizeMessageB;
ControlSigleMessage_t ControlSigleMssage;
ControlSignalMessage_t ControlSignalMessage;

WorkMessage_t WorkMessage;		   // 工作信息
ChannelMemoryMessage_t MemoryMsgA; // A 通道记忆，用于切换通道后恢复识别参数和用户设置
ChannelMemoryMessage_t MemoryMsgB; // B 通道记忆，字段含义与 A 通道完全一致
pumpMessage_t pumpMessageA;
pumpMessage_t pumpMessageB;
uint32_t paoxueSpeciValue_A[4] = {0U}; /* A 通道刀具规格缓存从旧屏适配层迁出，供手柄识别和新 UI 读取。 */
uint32_t paoxueSpeciValue_B[4] = {0U}; /* B 通道刀具规格缓存从旧屏适配层迁出，旧接口删除后仍保留业务数据。 */
uint8_t paoxueSpeciValue_F[16] = {0U}; /* 分体刀具扩展缓存保留在业务层，避免旧屏文件被删除后丢失识别状态。 */
/* 脚踏在线周期内的屏幕手动选择锁存；置 1 后本次脚踏在线期间不再自动抢回脚控。 */
static volatile uint8_t s_foot_priority_manual_lock = 0U;
/* Page4 速度/频率阈值只驱动蜂鸣，不写 WorkMessage.alarm_flag，避免阈值提示阻塞降速操作。 */
static uint8_t s_speed_threshold_beep_active = 0U;
/* 阈值蜂鸣保持期间定时重发报警消息，避免普通按键蜂鸣覆盖后阈值提示静音。 */
static uint8_t s_speed_threshold_beep_refresh_ticks = 0U;
/* 屏幕外控退出第一次点击是否已记录，第二次点击在 1 秒窗口内才真正退出外控。 */
static uint8_t s_screen_external_exit_pending = 0U;
/* 屏幕外控退出第一次点击的 HAL tick 时间戳，用于计算 1 秒双击窗口。 */
static uint32_t s_screen_external_exit_first_tick = 0U;
/* 压力堵塞使用独立报警码，只驱动限时蜂鸣和 89 号临时弹窗，不写 WorkMessage 全局报警锁存。 */
#define PUMP_PRESSURE_BLOCKED_BEEP_ALARM WORK_ALARM_PUMP_PRESSURE_BLOCKED
/* 公共接头缺刀具提示是否处于生命周期内，用于触发后按时间关闭并限制连续控制帧重复报警。 */
static uint8_t s_common_socket_tool_missing_alarm_active = 0U;
/* 公共接头缺刀具提示是否实际占用了屏幕报警区，只有占用过才允许到期清屏。 */
static uint8_t s_common_socket_tool_missing_alarm_displayed = 0U;
/* 公共接头缺刀具提示最近一次发送时间，用于限制重复报警频率。 */
static uint32_t s_common_socket_tool_missing_alarm_tick = 0U;
/* 公共接头缺刀具由脚踏触发时置1，弹窗和蜂鸣必须等待脚踏真实松开，不能按2秒自动退出。 */
static uint8_t s_common_socket_tool_missing_wait_foot_release = 0U;
/* 手控运行中拔手柄的临时屏幕报警码；非 0 表示 3 秒弹窗还未到期。 */
static uint8_t s_running_handle_unplug_transient_alarm_value = 0U;
/* 手控运行中拔手柄临时弹窗开始时间，用于周期服务到 3 秒后关闭屏幕报警。 */
static uint32_t s_running_handle_unplug_transient_alarm_tick = 0U;
/* 压力阈值临时弹窗归属标志，置 1 表示屏幕报警区当前由压力报警 89 图占用。 */
static uint8_t s_pump_pressure_blocked_transient_alarm_active = 0U;
/* 压力阈值临时弹窗开始时间，用于 3 秒后关闭 89 号压力报警图。 */
static uint32_t s_pump_pressure_blocked_transient_alarm_tick = 0U;
/* 手柄冷却跟随当前占用 A 泵，停止手柄时只释放本函数启动过的 A 泵输出。 */
#define HANDLE_INJECTION_FOLLOW_PUMP_A 0x01U
/* 手柄冷却跟随当前占用 B 泵，支持单个 B 注水泵跨通道给手柄降温。 */
#define HANDLE_INJECTION_FOLLOW_PUMP_B 0x02U
/* 记录手柄冷却跟随实际启动过哪些注水泵，避免停止手柄时误停非跟随来源的泵。 */
static uint8_t s_handle_injection_pump_follow_mask = 0U;
/* 注水泵压力堵塞导致手柄安全停机后的锁存位，用户明确停止/退出前禁止保活或重复启动把手柄重新拉起。 */
static uint8_t s_handle_pressure_block_stop_latched = 0U;

/* 手柄运行联动注水泵的默认冷却流量；Page4 写 0、越界或未装载时统一回退到 30。 */
#define HANDLE_INJECTION_PUMP_DEFAULT_FLOW 30U
/* 注水泵默认流量最小有效值，0 表示 EEPROM 未配置或非法，不能直接启动泵。 */
#define HANDLE_INJECTION_PUMP_FLOW_MIN 1U
/* 注水泵默认流量最大有效值，保持和灌注泵业务 0~300ml 范围一致。 */
#define HANDLE_INJECTION_PUMP_FLOW_MAX 300U
/* 新屏速度按键在旧通道记忆无步进时的兜底步进，避免初次插入或旧参数为空时按键无效。 */
#define SCREEN_SPEED_STEP_FALLBACK 1000U
/* 新屏速度大步进缺省值，只有 EEPROM Page6[2..3] 无效时才使用，正常情况直接用手柄配置。 */
#define SCREEN_SPEED_LARGE_STEP_FALLBACK 2000U
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
static uint8_t MapHandleToUiType(uint8_t handle_model)
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
	case DHYTM_ONLINES:
		return 4U; /* DHYTM复用一体磨手柄图片：A为109/110，B为139/140；不改变其独立EEPROM和非RFID业务属性。 */
	default:
		return handle_model; /* 未知型号保持原值，方便后续新增 UI 资源时直接扩展。 */
	}
}

void Pubinterface_SendHandleDisplay(uint8_t channel, uint8_t handle_model, bool enable_flag, bool light_flag)
{
	uint8_t display_value[10] = {0U};

	/* Value[0] 传手柄类型，sscUIDP::UIHANDLEDP() 用它决定显示哪一种手柄图标。 */
	display_value[0] = MapHandleToUiType(handle_model);
	/* Value[1] 传 A/B 通道号，1 表示 A 通道，2 表示 B 通道。 */
	display_value[1] = channel;
	/* Value[2] 传选中高亮状态，当前工作通道亮起，非当前通道只显示在线。 */
	display_value[2] = light_flag ? 1U : 0U;
	/* 手柄插拔事件已经完成状态更新后，通过 UIDP 队列刷新屏幕手柄区域。 */
	SendUIDSMessage(UI_HANDLE_ID, enable_flag, display_value);
}

/*
 * 函数功能：按当前 A/B 业务在线状态向屏幕发送一份完整手柄连接快照，在线通道显示型号，离线通道明确显示未连接。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshOnlineHandleDisplay(void)
{
	/* A 通道在线时刷新 A 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Aonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_A,
									   MemoryMsgA.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_A));
	}
	else
	{
		Pubinterface_SendHandleDisplay(CHANNEL_A, 0U, false, false); /* A 已离线时也必须明确发送未连接图，避免偶发 UI 消息丢失后继续保留旧手柄图标。 */
	}

	/* B 通道在线时刷新 B 手柄图标，并按当前工作通道决定是否高亮。 */
	if (WorkMessage.Channel_Bonline)
	{
		Pubinterface_SendHandleDisplay(CHANNEL_B,
									   MemoryMsgB.hand_model,
									   true,
									   (WorkMessage.channel_work == CHANNEL_B));
	}
	else
	{
		Pubinterface_SendHandleDisplay(CHANNEL_B, 0U, false, false); /* B 已离线时同步发送未连接图，使本函数成为 A/B 图标的完整权威快照。 */
	}
}

/*
 * 函数功能：判断当前手柄是否使用刀具规格窗口显示长度、直径和角度。
 * 输入参数：hand_model EEPROM 识别出的手柄型号。
 * 返回参数：true 表示优先显示规格窗口，false 表示显示普通刀具图标。
 */
bool Pubinterface_IsSplitToolSpecDisplayModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||
			(hand_model == PXBB_ONLINES));
}

/*
 * 函数功能：查询指定通道当前是否允许 PXBA/PXBB 分体式手柄继续执行 RFID 自动识别。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：true 表示该通道处于自动识别模式，false 表示手动模式或通道无效。
 */
bool Pubinterface_IsRfidAutoIdentifyEnabled(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return (MemoryMsgA.auto_identify != 0U); /* A 通道手动模式会清 0，handlescan 在线监测据此停止 RFID EPC 读取。 */
	}

	if (channel == CHANNEL_B)
	{
		return (MemoryMsgB.auto_identify != 0U); /* B 通道同样只在自动识别模式下允许继续轮询 RFID 刀具头。 */
	}

	return false; /* 非 A/B 通道没有 RFID 自动识别权限，防止异常通道触发模拟开关切换。 */
}

/*
 * 函数功能：判断当前手柄是否为规格来自手柄自身 EEPROM Page3 的一体式刀具头手柄。
 * 输入参数：hand_model EEPROM Page2 识别出的手柄型号。
 * 返回参数：true表示PXYTM/PXYTP需要显示手柄EEPROM内的直径、长度和角度；MXYTP/MXYTM改走公共接头RFID规格显示。
 */
static bool IsIntegratedToolModel(uint8_t hand_model)
{
    return ((hand_model == PX_YIM_ONLINES) ||  /* PXYTM规格来自手柄自身EEPROM Page3。 */
            (hand_model == PX_YIP_ONLINES));   /* PXYTP规格来自手柄自身EEPROM Page3；MXY型号改由公共接头RFID显示。 */
}
/*
 * 函数功能：判断当前手柄型号是否允许使用屏幕刀具规格窗口显示 RFID/EPC 解析出的长度、直径和角度。
 * 输入参数：hand_model EEPROM 识别出的手柄型号。
 * 返回参数：true 表示当前型号允许打开 UI_TOOLSPEC_ID；false 表示当前型号必须隐藏规格窗口。
 */
static bool IsRfidToolModel(uint8_t hand_model)
{
	return ((Pubinterface_IsSplitToolSpecDisplayModel(hand_model) == true) ||
			(IsIntegratedToolModel(hand_model) == true) ||
			(hand_model == COMMON_SOCKET_ONLINES)); /* PXBA/PXBB 和公共接头显示 RFID 刀具头标签规格；两类PXY一体式显示手柄EEPROM Page3规格。 */
}

/*
 * 函数功能：判断当前刀具字段是否代表支持往复的刨刀能力。
 * 输入参数：tool_type 当前通道识别或手动选择得到的刀具类型字段。
 * 返回参数：true 表示该刀具按刨刀能力开放往复；false 表示按普通磨头/未知刀具处理。
 */
bool Pubinterface_IsPlanerCapabilityTool(uint8_t tool_type)
{
	return (tool_type == PLANER); /* 普通刨刀、0x06反向刨刀、MXYTP和PXYTP均归一为PLANER，是否允许电气往复还要结合基座与raw_tool_type判断。 */
}

/*
 * 函数功能：判断EEPROM手柄或RFID机械刀具是否锁定方向，统一拦截屏幕、实体键和外控换向。
 * 输入参数：hand_model为Page2手柄/基座型号；raw_tool_type为EPC byte0原始刀具型号，非RFID来源传0或原兼容值。
 * 返回参数：true表示只能保留上线识别方向；false表示沿用原方向切换规则。
 */
bool Pubinterface_IsDirLocked(uint8_t hand_model, uint8_t raw_tool_type)
{
    bool fixed_eeprom_handle = ((hand_model == TMBA_ONLINES) || /* TMBA只服从本手柄EEPROM Page4默认方向。 */
                                (hand_model == TMBB_ONLINES) || /* TMBB只服从本手柄EEPROM Page4默认方向。 */
                                (hand_model == EMBA_ONLINES) || /* EMBA只服从本手柄EEPROM Page4默认方向。 */
                                (hand_model == EMBB_ONLINES) || /* EMBB只服从本手柄EEPROM Page4默认方向。 */
                                (hand_model == JMB_ONLINES) ||  /* JMB只服从本手柄EEPROM Page4默认方向。 */
                                (hand_model == DHYTM_ONLINES)); /* DHYTM固定显示Page4反转方向，屏幕、实体键和外控均不得换向。 */
    bool rfid_tool_handle = ((hand_model == PXBA_ONLINES) ||
                             (hand_model == PXBB_ONLINES) ||
                             (hand_model == COMMON_SOCKET_ONLINES)); /* EPC基座的0x03~0x05固定方向；0x06反向刨刀仍允许选择正转、反转和往复。 */

    if (fixed_eeprom_handle)
    {
        return true; /* EEPROM固定方向手柄继续禁止屏幕、实体键和外控切换方向。 */
    }

    if (rfid_tool_handle == false)
    {
        return false; /* 普通可换向手柄不读取RFID原始型号，保持现有方向切换能力。 */
    }

    return ((raw_tool_type == RFID_TOOL_MODEL_REVERSE_ROTATION) ||
            (raw_tool_type == RFID_TOOL_MODEL_MXYTP) ||
            (raw_tool_type == RFID_TOOL_MODEL_MXYTM)); /* 三类机械刀具方向由EPC定义；0x06反向刨刀故意不锁方向，单向取反留在驱动输出层。 */
}
/*
 * 函数功能：判断当前手柄基座是否可承载支持电气往复的普通刨刀。
 * 输入参数：hand_model为EEPROM识别出的手柄或公共接头基座型号。
 * 返回参数：true表示基座具备电气往复能力；MXYTP机械往复仍由raw_tool_type在下一层排除。
 */
static bool IsOscSupportedModel(uint8_t hand_model)
{
    return ((hand_model == PXBA_ONLINES) ||
            (hand_model == PXBB_ONLINES) ||
            (hand_model == COMMON_SOCKET_ONLINES) ||
            (hand_model == PX_YIP_ONLINES)); /* PXB和公共接头可随0x01普通刨刀或0x06反向刨刀进入电气往复；PXYTP保持原有一体式往复能力。 */
}
/*
 * 函数功能：综合基座型号、业务刀具能力和RFID原始型号判断是否支持电气往复。
 * 输入参数：hand_model为当前基座型号；tool_type为归一刀具能力；raw_tool_type为EPC byte0原始型号。
 * 返回参数：true表示允许切入OSCDIR并调频；MXYTP仅显示机械往复，因此返回false。
 */
static bool IsOscDirectionAllowed(uint8_t hand_model, uint8_t tool_type, uint8_t raw_tool_type)
{
    if (raw_tool_type == RFID_TOOL_MODEL_MXYTP)
    {
        return false; /* MXYTP仅在界面显示机械往复，不允许频率调节，也不能向驱动下发电气往复模式。 */
    }

    if (IsOscSupportedModel(hand_model) == false)
    {
        return false; /* 手柄基座不支持往复时，刨刀字段也不能单独开放OSCDIR。 */
    }

    if (Pubinterface_IsPlanerCapabilityTool(tool_type) == false)
    {
        return false; /* 当前刀具不是刨刀能力时，继续禁用往复方向和频率窗口。 */
    }

    return true; /* 普通0x01刨刀配合支持往复的基座时，允许正反转和电气往复切换。 */
}
/*
 * 函数功能：判断开口定位入口是否允许显示和发送。
 * 输入参数：hand_model 当前手柄基座型号；tool_type 当前刀具能力类型。
 * 返回参数：true 表示 PXBA/PXBB 且刀具为 PLANER，false 表示隐藏开口定位并拦截动作。
 */
bool Pubinterface_IsOpenPositionEnabledTool(uint8_t hand_model, uint8_t tool_type)
{
	return ((Pubinterface_IsSplitToolSpecDisplayModel(hand_model) == true) &&
			(Pubinterface_IsPlanerCapabilityTool(tool_type) == true)); /* 开口定位只给 PXBA/PXBB 的 PLANER 刀具开放，PXP 只保留往复能力。 */
}

/*
 * 函数功能：记录当前通道最近一次 RFID 识别出的刀具类型。
 * 输入参数：channel 为 A/B 通道；tool_type 为归一后的 PLANER/GRINDH 或兼容旧型号码。
 * 返回参数：无。
 */
void Pubinterface_SetLastRfidToolType(uint8_t channel, uint8_t tool_type)
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
static uint8_t GetLastRfidToolType(uint8_t channel)
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
static uint8_t MapToolTypeToPicture(uint8_t tool_type)
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
static bool GetToolSpecForChannel(uint8_t channel, uint8_t *display_value)
{
	const uint32_t *spec_values = NULL;
	uint32_t tool_length = 0U;
	uint32_t tool_diameter = 0U;
	uint32_t tool_angle = 0U;
	bool raw_spec_display = ((WorkMessage.hand_model == COMMON_SOCKET_ONLINES) ||
							 (((WorkMessage.hand_model == PXBA_ONLINES) || (WorkMessage.hand_model == PXBB_ONLINES)) &&
							  (WorkMessage.auto_identify != 0U))); /* 公共接头和自动识别下的 PXBA/PXBB 都按 EPC 原始整数显示规格。 */

	/* 刀具规格必须从当前 A/B 通道自己的扫描缓存读取，避免切换通道后显示另一侧的旧规格。 */
	if (channel == CHANNEL_A)
	{
		spec_values = paoxueSpeciValue_A;
	}
	/* B 通道只绑定 B 侧规格缓存，保证 A/B 手柄的长度、直径和角度不会串用。 */
	else if (channel == CHANNEL_B)
	{
		spec_values = paoxueSpeciValue_B;
	}
	else
	{
		return false; /* 未选中有效通道时没有规格归属，禁止打开刀具规格窗口。 */
	}

	if (raw_spec_display)
	{
		tool_length = spec_values[0]; /* EPC 长度缓存就是原始整数值，公共接头和 PXBA/PXBB 自动识别保持同一显示单位。 */
		tool_diameter = spec_values[1]; /* EPC 直径缓存就是原始整数值，不再做历史单位放大。 */
		tool_angle = spec_values[2]; /* EPC 角度缓存就是原始整数值，保证标签、心跳和屏幕一致。 */
	}
	else
	{
		tool_length = spec_values[0] * 5U; /* EEPROM 一体式刀具长度沿用历史缓存单位，屏幕显示前恢复成实际长度值。 */
		tool_diameter = spec_values[1]; /* EEPROM 一体式刀具直径沿用历史缓存单位，保持旧屏幕格式化路径不变。 */
		tool_angle = spec_values[2]; /* EEPROM 一体式刀具角度沿用历史缓存单位，保持旧屏幕格式化路径不变。 */
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
static void RefreshToolDisplay(uint8_t channel, bool planer_selected)
{
	uint8_t display_value[10] = {0U};
	uint8_t manual_display_value[10] = {0U}; /* 单独保存手动/自动识别按钮参数，避免和 RFID 规格长度低字节共用 Value[1]。 */
	bool show_tool_spec = false;
	bool auto_identify = (WorkMessage.auto_identify != 0U); /* 自动识别模式下，未读到 RFID 规格前不显示手动磨/刨按钮。 */
	bool split_tool_spec_handle = Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model); /* 当前通道是否为 PXBA/PXBB 分体 RFID 手柄，普通 EEPROM 手柄不显示刀具识别区。 */
	bool integrated_tool_spec_handle = IsIntegratedToolModel(WorkMessage.hand_model); /* MXYTM/MXYTP/PXYTM/PXYTP 不走 RFID，规格直接来自手柄 EEPROM Page3。 */
	bool rfid_spec_handle = IsRfidToolModel(WorkMessage.hand_model); /* 当前通道是否具备 RFID/EPC 规格显示能力，公共接头也要显示刀具规格。 */
	bool common_socket_spec_handle = (WorkMessage.hand_model == COMMON_SOCKET_ONLINES); /* 公共接头没有 PXBA/PXBB 识别按钮，但 EPC 规格有效时必须打开规格窗口。 */
	bool rfid_display_enabled = (auto_identify && split_tool_spec_handle); /* 当前选中通道必须是 PXBA/PXBB 才允许显示 RFID 规格和自动识别图标，避免普通手柄继承另一通道残留。 */
	bool rfid_spec_window_enabled = (rfid_display_enabled || common_socket_spec_handle || integrated_tool_spec_handle); /* PXBA/PXBB/公共接头用 RFID 标签规格；一体式四类用手柄 EEPROM Page3 规格。 */
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
		result_tool_type = GetLastRfidToolType(channel); /* RFID 手柄掉线后 WorkMessage.tool_type 会清 0，此时只用本 RFID 通道最近一次识别类型决定 61/62。 */
	}
	manual_display_value[2] = rfid_display_enabled ? MapToolTypeToPicture(result_tool_type) : 63U; /* Value[2] 只给 RFID 自动识别区驱动 61/62/63，普通手柄始终刷默认值清残留。 */

	if (rfid_spec_window_enabled)
	{
		show_tool_spec = GetToolSpecForChannel(channel, display_value); /* PXBA/PXBB 自动识别和公共接头 EPC 都从通道规格缓存读取长度、直径和角度。 */
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
		else if (integrated_tool_spec_handle)
		{
			SendUIDSMessage(UI_TOOL_ID, false, manual_display_value); /* 一体式手柄没有独立 RFID 刀具头，Page3 规格无效时隐藏普通刀具图，避免误显示旧规格。 */
			SendUIDSMessage(UI_MANUALBUTTON_ID, false, manual_display_value); /* 一体式手柄不显示手动/自动识别按钮，只保留有效 EEPROM 规格窗口。 */
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
 * 函数功能：判断当前工作通道是否确实对应一只在线且已识别的手柄。
 * 输入参数：无，读取 WorkMessage 当前通道、在线标志和手柄型号。
 * 返回参数：true 表示当前 A/B 通道可用于屏幕业务按键；false 表示没有有效当前手柄。
 */
static bool ScreenKey_HasCurrentHandle(void)
{
	if (WorkMessage.channel_work == CHANNEL_A) /* 当前选择 A 时必须核对 A 自己的在线标志，不能借用 B 的状态。 */
	{
		return ((WorkMessage.Channel_Aonline == true) && (WorkMessage.hand_model != 0U)); /* A 在线且当前快照已有有效型号时，参数区才真正可用。 */
	}
	if (WorkMessage.channel_work == CHANNEL_B) /* 当前选择 B 时只读取 B 在线状态，防止 A 在线误放行 B 参数区。 */
	{
		return ((WorkMessage.Channel_Bonline == true) && (WorkMessage.hand_model != 0U)); /* B 在线且已识别型号时，允许处理当前参数区按键。 */
	}

	return false; /* 没有选中 A/B 通道时，速度、方向和刀具区域都属于不可触控状态。 */
}

/*
 * 函数功能：判断当前通道是否已有可显示的刀具规格，用于同步自动识别按钮和开口定位的显隐门禁。
 * 输入参数：无，读取当前 A/B 通道规格缓存。
 * 返回参数：true 表示屏幕正在使用有效规格窗口；false 表示仍在等待 RFID 或没有有效规格。
 */
static bool ScreenKey_HasToolSpec(void)
{
	uint8_t display_value[10] = {0U}; /* 复用现有规格判定函数所需的临时缓冲，只读取结果，不发送任何 UI 消息。 */

	if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B)) /* 规格必须明确归属于 A 或 B 当前通道。 */
	{
		return false; /* 未选中通道时没有规格归属，隐藏区域必须保持不可触控。 */
	}

	return GetToolSpecForChannel(WorkMessage.channel_work, display_value); /* 与屏幕刷新复用同一判定，避免输入门禁和规格窗口不一致。 */
}

/*
 * 函数功能：按当前业务状态判断新串口屏逻辑按键是否可操作，统一拦截黑色或隐藏区域的触摸。
 * 输入参数：screen_key 为 ScreenKey_PostLegacyAction() 映射后的 SCREENKey_* 逻辑键值。
 * 返回参数：true 表示允许蜂鸣并投递业务队列；false 表示静默丢弃且不得修改业务状态。
 */
bool ScreenKey_CanUse(uint8_t screen_key)
{
	bool current_handle_available = ScreenKey_HasCurrentHandle(); /* 当前参数区只有对应在线手柄存在时才允许响应。 */
	bool split_handle_available = (current_handle_available && Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model)); /* 磨/刨和自动识别入口只属于 PXBA/PXBB 分体手柄。 */
	bool current_tool_spec_visible = false; /* 默认没有有效规格，只有分体自动识别模式需要进一步读取规格缓存。 */
	bool external_control_active = ControlArbitration_IsExternalActive(); /* 外控占用时本地控制方式按钮不可操作，只保留明确的外控退出入口。 */

	if (split_handle_available && (WorkMessage.auto_identify != 0U)) /* 自动识别时按钮显隐还取决于规格窗口是否已经显示。 */
	{
		current_tool_spec_visible = ScreenKey_HasToolSpec(); /* 规格窗口覆盖识别按钮区后，旧坐标必须同步关闭。 */
	}

	if (screen_key == SCREENKey_TouchEXIT) /* 触控退出是故障确认出口，不能被下面的普通报警门禁提前拦截。 */
	{
		return (((WorkMessage.alarm_flag == false) && (WorkMessage.touchactive_work == TOUCHWORK)) ||
				WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED)); /* 无报警的触控界面或手柄掉线确认才允许退出，普通报警不能提前清除“必须松手”锁存。 */
	}

	if (screen_key == SCREENKey_TouchKeepAlive) /* 保活只属于已经打开的本机触控工作面。 */
	{
		return ((WorkMessage.alarm_flag == false) &&
				(external_control_active == false) &&
				(WorkMessage.touchactive_work == TOUCHWORK) &&
				(WorkMessage.drivetype_work == TOUCHWORK)); /* 报警后的必须松手锁存仍由 screenkey.c 在调用本函数前维护。 */
	}

	if (screen_key == SCREENKey_HMI_EXIT) /* 外控退出图标隐藏时，原坐标不得产生蜂鸣或双击计数。 */
	{
		return ((external_control_active == true) && (WorkMessage.alarm_flag == false)); /* 外控真实占用且无报警时保留原 1 秒双击退出。 */
	}

	if (WorkMessage.alarm_flag == true) /* 报警弹窗覆盖主页面时，普通按钮全部视为不可触控。 */
	{
		return false; /* 静默拦截被弹窗覆盖的坐标，避免报警期间误改参数。 */
	}

	switch (screen_key) /* 每组逻辑键直接对应屏幕一块可见区域，未知键默认拒绝。 */
	{
	case SCREENKey_APUMP_Add:
	case SCREENKey_APUMP_Sub:
		return ((pumpMessageA.online_flag == true) &&
				(pumpMessageA.type != 0U) &&
				(pumpMessageA.timingDrainage_flag == false)); /* A 泵在线且不在定时排空时，加减按钮才允许调节。 */

	case SCREENKey_BPUMP_Add:
	case SCREENKey_BPUMP_Sub:
		return ((pumpMessageB.online_flag == true) &&
				(pumpMessageB.type != 0U) &&
				(pumpMessageB.timingDrainage_flag == false)); /* B 泵使用独立在线和排空状态，禁止串用 A 泵条件。 */

	case SCREENKey_APUMP_control:
		return ((pumpMessageA.online_flag == true) &&
				(pumpMessageA.type != 0U) &&
				!((pumpMessageA.type == INJECTWATER) && (WorkMessage.runflag_work == true))); /* 手柄运行联动注水时，A 排空键不能抢占冷却泵。 */

	case SCREENKey_BPUMP_control:
		return ((pumpMessageB.online_flag == true) &&
				(pumpMessageB.type != 0U) &&
				!((pumpMessageB.type == INJECTWATER) && (WorkMessage.runflag_work == true))); /* B 注水泵同样在手柄联动期间禁止屏幕启停。 */

	case SCREENKey_SPEED_Add:
	case SCREENKey_SPEED_Sub:
	case SCREENKey_SPEED_Sub_Large:
	case SCREENKey_SPEED_Sub_Small:
	case SCREENKey_SPEED_Add_Small:
	case SCREENKey_SPEED_Add_Large:
		return (current_handle_available &&
				((WorkMessage.dir_work == ZZDIR) || (WorkMessage.dir_work == FZDIR) || (WorkMessage.dir_work == OSCDIR))); /* 速度区域显示且方向参数有效时才允许调速。 */

	case SCREENKey_FREQ_Add:
	case SCREENKey_FREQ_Sub:
		return (current_handle_available &&
				(WorkMessage.dir_work == OSCDIR) &&
				IsOscDirectionAllowed(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type)); /* 频率区仅在当前确实处于往复方向时可见并可调。 */

	case SCREENKey_HANDLE_A:
		return ((WorkMessage.runflag_work == false) && (WorkMessage.Channel_Aonline == true)); /* A 在线且电机停止时才允许切换到 A。 */

	case SCREENKey_HANDLE_B:
		return ((WorkMessage.runflag_work == false) && (WorkMessage.Channel_Bonline == true)); /* B 在线且电机停止时才允许切换到 B。 */

	case SCREENKey_PlanerH:
	case SCREENKey_GrindH:
		return ((WorkMessage.runflag_work == false) &&
				split_handle_available &&
				(WorkMessage.auto_identify == 0U)); /* 自动识别模式会隐藏磨/刨按钮，旧坐标必须静默且不能退出 RFID。 */

	case SCREENKey_AutoIdentify:
		return ((WorkMessage.runflag_work == false) &&
				split_handle_available); /* PXBA/PXBB 停机时始终允许切换自动/手动模式，标签规格已显示也不能屏蔽该按钮。 */

	case SCREENKey_Dir_Forward:
	case SCREENKey_Dir_Reverse:
		return ((WorkMessage.runflag_work == false) &&
				current_handle_available &&
				(Pubinterface_IsDirLocked(WorkMessage.hand_model, WorkMessage.raw_tool_type) == false)); /* 固定方向手柄的已显示方向只是状态指示，任何正反转触控都必须静默丢弃。 */

	case SCREENKey_Dir_OSC:
		return ((WorkMessage.runflag_work == false) &&
				current_handle_available &&
				(Pubinterface_IsDirLocked(WorkMessage.hand_model, WorkMessage.raw_tool_type) == false) &&
				IsOscDirectionAllowed(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type)); /* 灰色往复按钮不得仅因保留旧方向而重新切入往复。 */

	case SCREENKey_OpenPos_ClockWise:
	case SCREENKey_OpenPos_AntiClockWise:
		return ((WorkMessage.runflag_work == false) &&
				current_handle_available &&
				Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model, WorkMessage.tool_type) &&
				((WorkMessage.auto_identify == 0U) || current_tool_spec_visible)); /* 自动识别等待规格时开口定位被隐藏，必须同步拦截旧坐标。 */

	case SCREENKey_JTActi:
		return ((WorkMessage.runflag_work == false) &&
				(external_control_active == false) &&
				(ControlSignalMessage.jt_enable_flag == true) &&
				(WorkMessage.touchactive_work != TOUCHWORK)); /* 脚踏在线且未被触控或外控占用时，脚控按钮才可选择。 */

	case SCREENKey_HandleActi:
		return ((WorkMessage.runflag_work == false) &&
				(external_control_active == false) &&
				current_handle_available &&
				Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model) &&
				(WorkMessage.touchactive_work != TOUCHWORK)); /* 当前型号具备手控入口且没有其它界面占用时才允许切换。 */

	case SCREENKey_TouchActi:
		return ((WorkMessage.runflag_work == false) &&
				(external_control_active == false) &&
				current_handle_available); /* 触控入口必须有当前在线手柄，外控占用时保持不可选。 */

	default:
		return false; /* 插拔事件、废弃 TouchStart 和未知编号都不是主页面可触控业务键。 */
	}
}

/*
 * 函数功能：当前通道离线且没有可回落通道时关闭参数区显示。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_ClearSelectedChannelDisplay(void)
{
	uint8_t display_value[10] = {0U};

	/* 脚踏在线时直接刷新稳定的脚控状态，避免开机补刷先写暗态再写选中态导致图标闪烁。 */
	if(ControlSignalMessage.jt_enable_flag==true)
	{
		display_value[0]=1U; /* 保持原无手柄且脚踏在线时的脚控选中显示，不改变脚控运行状态。 */
		display_value[1]=1U; /* 选中标志保持为真，使脚踏图标每次补刷都稳定显示选中图片。 */
		SendUIDSMessage(UI_CONTROL_ID, true, display_value); /* 仅写一次脚踏最终状态，不制造暗态与选中态往返。 */

		display_value[0]=2U; /* 无手柄时手控入口不可用，但不能通过整组清屏连带改写脚踏图标。 */
		display_value[1]=0U; /* 清除选中标志，确保手控图标保持灰色不可选状态。 */
		SendUIDSMessage(UI_CONTROL_ID, false, display_value); /* 单独刷新手控图标为不可用状态。 */

		display_value[0]=3U; /* 无手柄时触控入口同样不可用。 */
		SendUIDSMessage(UI_CONTROL_ID, false, display_value); /* 单独刷新触控图标，保持脚踏图标稳定。 */
	}
	else
	{
		SendUIDSMessage(UI_CONTROL_ID, false, display_value); /* 脚踏也离线时才整组恢复三种控制图标的暗态。 */
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
bool Pubinterface_IsHandleControlReservedModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||    /* EX8 表格确认 PXBA 分体手柄允许显示并进入手控。 */
			(hand_model == LGZ_I_ONLINES) ||   /* LGZI 单按键颅骨钻允许进入手控，实体键按住运行、松开停止。 */
			(hand_model == LGZ_II_ONLINES));   /* EX8 表格确认 LGZII 双按键手柄保留手控入口，其它预留型号不再误亮手控。 */
}

/*
 * 函数功能：判断当前手柄型号是否为公共接头预留型号。
 * 输入参数：hand_model 为 EEPROM 识别出的系统内部手柄型号。
 * 返回参数：true 表示该型号没有手柄实体键，不自动切入手控。
 */
static bool IsCommonSocketReserved(uint8_t hand_model)
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
	ChannelMemoryMessage_t *memory = NULL; /* 当前通道记忆用于复核刀具类型，避免 WorkMessage 切换过程中的短暂字段不一致。 */
	ChannelrecognizeMessage_t *recognize = NULL; /* 扫描层缓存表示当前 RFID 刀具头是否仍被在线监测确认，不能只看历史 MemoryMsg。 */

	if (WorkMessage.hand_model != COMMON_SOCKET_ONLINES)
	{
		return true; /* 非公共接头不受 EPC 刀具头 gate 限制，保持原有启动逻辑。 */
	}

	if (WorkMessage.channel_work == CHANNEL_A)
	{
		memory = &MemoryMsgA; /* A 通道公共接头使用 A 通道记忆判断 EPC 刀具是否已写入。 */
		recognize = &ChannelrecognizeMessageA; /* A 通道扫描缓存会在 RFID 刀具头掉线确认后清 tool_type。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		memory = &MemoryMsgB; /* B 通道公共接头使用 B 通道记忆判断 EPC 刀具是否已写入。 */
		recognize = &ChannelrecognizeMessageB; /* B 通道扫描缓存会在 RFID 刀具头掉线确认后清 tool_type。 */
	}
	else
	{
		return false; /* 没有选中通道时不能运行公共接头，避免无归属的启动命令下发到驱动。 */
	}

	if ((memory == NULL) || (memory->hand_model != COMMON_SOCKET_ONLINES))
	{
		return false; /* 通道记忆尚未同步为公共接头时，认为刀具头未就绪，等待插拔事件完成。 */
	}

	if ((recognize == NULL) || (recognize->handle_type != COMMON_SOCKET_ONLINES))
	{
		return false; /* 扫描层没有确认当前基座是公共接头时禁止启动，避免旧 WorkMessage 参数被误当成在线刀具。 */
	}

	return ((WorkMessage.tool_type != 0U) &&
			(memory->tool_type != 0U) &&
			(recognize->tool_type != 0U)); /* 三层都必须有当前 EPC 刀具类型，历史 MemoryMsg 参数不能单独放行启动。 */
}

/*
 * 函数功能：公共接头未识别到 EPC 刀具头时提示“请连接手柄”报警，并按控制来源选择退出方式。
 * 输入参数：wait_for_foot_release 为true表示脚踏来源持续到松脚；false表示其它来源保持2秒。
 * 返回参数：无。
 */
static void ReportCommonSocketToolMissing(bool wait_for_foot_release)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余字节补零保持消息格式稳定。 */
	uint32_t now_tick = HAL_GetTick(); /* 使用 HAL 毫秒 tick 做重发限频，避免连续控制帧让蜂鸣任务反复入队。 */

	if (wait_for_foot_release != false)
	{
		s_common_socket_tool_missing_wait_foot_release = 1U; /* 脚踏来源先锁住生命周期，即使此前已有2秒提示也转换为松脚退出。 */
		SendAlarmMessage(WORK_ALARM_HANDLE_NOT_CONNECTED); /* 脚踏来源使用持续蜂鸣，后续只由统一松脚出口发送NONE。 */
	}

	if ((s_common_socket_tool_missing_alarm_active != 0U) &&
		((uint32_t)(now_tick - s_common_socket_tool_missing_alarm_tick) < ALARM_SOCKET_REPEAT_MS))
	{
		return; /* 1 秒内重复触发只保持当前提示，不再重复蜂鸣和刷屏。 */
	}

	s_common_socket_tool_missing_alarm_active = 1U; /* 记录本次缺刀具提示已进入生命周期，后续按定时服务或 EPC 成功装载结束。 */
	s_common_socket_tool_missing_alarm_displayed = 0U; /* 先清显示归属，只有本次真正发出 UI 弹窗后才允许本模块到期清屏。 */
	s_common_socket_tool_missing_alarm_tick = now_tick; /* 记录本次提示时间，用于下一次触发限频。 */
	display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 公共接头无刀具头时沿用“手柄未连接/请连接手柄”图片 80。 */
	if (wait_for_foot_release == false)
	{
		SendAlarmMessageTimed(WORK_ALARM_HANDLE_NOT_CONNECTED, ALARM_SOCKET_MS); /* 非脚踏来源继续限时蜂鸣，不写WorkMessage全局报警。 */
	}
	ExternalComm_SendTransientAlarm(WORK_ALARM_HANDLE_NOT_CONNECTED, ALARM_SOCKET_MS); /* 上位机同步收到临时报警，便于外控模式下也知道刀具头未接入。 */
	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 没有更高优先级全局报警时，屏幕显示“请连接手柄”提示。 */
		s_common_socket_tool_missing_alarm_displayed = 1U; /* 记录 80 号图由本模块显示，后续到期或 EPC 成功时才具备清屏权。 */
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
		ReportCommonSocketToolMissing(false); /* 非脚踏运行来源保持2秒临时提示，不改变手控、触控和外控现有行为。 */
		return false; /* 公共接头没有有效 EPC 刀具参数时禁止运行，避免无刀具头仍下发电机启动帧。 */
	}

	return true; /* 非公共接头或 EPC 刀具已装载时，保持原有脚踏/手柄/屏幕/外控启动路径。 */
}

/*
 * 函数功能：脚踏启动手柄前检查公共接头EPC刀具，缺失时持续报警到脚踏真实松开。
 * 输入参数：无，直接读取当前公共接头刀具状态。
 * 返回参数：true表示允许脚踏继续启动；false表示缺少RFID刀具，本次启动必须拒绝。
 */
bool Pubinterface_CheckCommonSocketToolReadyForFootRun(void)
{
	if (Pubinterface_IsCommonSocketToolReady() == false)
	{
		ReportCommonSocketToolMissing(true); /* 脚踏来源改用持续弹窗和蜂鸣，不能沿用其它来源的2秒自动退出。 */
		return false; /* 缺少有效EPC参数时保持电机停机，等待用户松脚后重新触发识别和启动。 */
	}

	return true; /* 非公共接头或刀具已经就绪时，脚踏按原有比例控制流程继续。 */
}

/*
 * 函数功能：公共接头 EPC 刀具头识别成功后清除本模块产生的临时报警显示。
 * 输入参数：无，直接读取公共接头就绪状态和 WorkMessage 报警状态。
 * 返回参数：无。
 */
void Pubinterface_ClearCommonSocketToolMissingAlarm(void)
{
	if (s_common_socket_tool_missing_alarm_active == 0U)
	{
		return; /* 本模块没有显示过公共接头临时报警时不操作屏幕，避免误清其它报警。 */
	}
	if (s_common_socket_tool_missing_wait_foot_release != 0U)
	{
		return; /* 脚踏来源即使随后识别到RFID，也必须等操作者松脚后才结束本次弹窗和蜂鸣。 */
	}

	if (Pubinterface_IsCommonSocketToolReady() == false)
	{
		return; /* EPC 刀具头仍未就绪时不由识别成功路径清屏，等待周期服务按 ALARM_SOCKET_MS 到期关闭。 */
	}

	s_common_socket_tool_missing_alarm_active = 0U; /* 当前公共接头刀具已就绪，本模块的临时报警生命周期结束。 */
	s_common_socket_tool_missing_alarm_tick = 0U; /* 清时间戳，下一次刀具头移除后可立即重新提示。 */
	if ((s_common_socket_tool_missing_alarm_displayed != 0U) && /* 只有本模块显示的 80 号报警到期且没有其它报警接管时才关闭弹窗。 */
		(WorkMessage.alarm_flag == false) &&
		(s_pump_pressure_blocked_transient_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 本模块确实显示过 80 且没有其它报警占用时，清掉公共接头缺刀具弹窗。 */
	}
	s_common_socket_tool_missing_alarm_displayed = 0U; /* 无论是否实际清屏，本次公共接头缺刀具弹窗归属都已结束。 */
}

/*
 * 函数功能：脚踏真实松开或掉线后结束公共接头缺RFID产生的持续80号弹窗和蜂鸣。
 * 输入参数：无。
 * 返回参数：无；只清理由脚踏来源持有的公共接头临时报警，不改变全局真实报警。
 */
void Pubinterface_ReleaseFootCommonSocketToolMissingAlarm(void)
{
	if (s_common_socket_tool_missing_wait_foot_release == 0U)
	{
		return; /* 当前公共接头提示不是脚踏持续来源，不能提前关闭其它控制来源的2秒提示。 */
	}

	s_common_socket_tool_missing_wait_foot_release = 0U; /* 先释放脚踏来源标志，下一次重新踩下可以创建新的持续报警生命周期。 */
	if ((s_common_socket_tool_missing_alarm_displayed != 0U) &&
		(WorkMessage.alarm_flag == false) &&
		(s_pump_pressure_blocked_transient_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 报警区仍由本模块占用且没有其它报警时，松脚立即关闭80号图。 */
	}
	if ((WorkMessage.alarm_flag == false) &&
		(s_pump_pressure_blocked_transient_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendAlarmMessage(WORK_ALARM_NONE); /* 没有其它报警蜂鸣占用时，松脚立即停止公共接头缺刀具持续蜂鸣。 */
	}
	s_common_socket_tool_missing_alarm_active = 0U; /* 本次持续提示生命周期结束，避免周期服务继续按旧tick处理。 */
	s_common_socket_tool_missing_alarm_displayed = 0U; /* 清除本模块屏幕归属，后续不能误关其它报警图。 */
	s_common_socket_tool_missing_alarm_tick = 0U; /* 清除旧触发时间，下一次真实踩下可立即重新提示。 */
}

/*
 * 函数功能：周期关闭公共接头缺 EPC 刀具头产生的 80 号临时屏幕报警。
 * 输入参数：无，直接读取公共接头临时报警归属和 HAL 毫秒 tick。
 * 返回参数：无。
 */
static void ServiceCommonSocketToolAlarm(void)
{
	if (s_common_socket_tool_missing_alarm_active == 0U)
	{
		return; /* 当前没有公共接头缺刀具临时报警生命周期，本周期不处理屏幕报警区。 */
	}
	if (s_common_socket_tool_missing_wait_foot_release != 0U)
	{
		return; /* 脚踏来源由真实松脚出口关闭，禁止ALARM_SOCKET_MS到期后自动停止弹窗和蜂鸣。 */
	}

	if ((uint32_t)(HAL_GetTick() - s_common_socket_tool_missing_alarm_tick) < ALARM_SOCKET_MS)
	{
		return; /* 80 号图未达到设置保持时间，继续显示到 ALARM_SOCKET_MS 到期。 */
	}

	if ((s_common_socket_tool_missing_alarm_displayed != 0U) && /* 只有本模块显示的 80 号报警到期且没有其它报警接管时才关闭弹窗。 */
		(WorkMessage.alarm_flag == false) &&
		(s_pump_pressure_blocked_transient_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 本模块实际显示的 80 到期且没有其它报警占用时，关闭屏幕报警弹窗。 */
	}
	s_common_socket_tool_missing_alarm_active = 0U; /* 80 临时弹窗保持时间结束，允许下一次真实启动触发重新报警。 */
	s_common_socket_tool_missing_alarm_displayed = 0U; /* 清显示归属，避免下一轮误清其它模块后续显示的报警。 */
	s_common_socket_tool_missing_alarm_tick = 0U; /* 清本次开始时间，后续新报警必须重新记录 tick。 */
}

/*
 * 函数功能：按 EX8 表格规则把不可用的脚控选中态回落到手控选中态。
 * 输入参数：无，直接读取当前通道、脚踏在线标志和当前手柄型号。
 * 返回参数：无。
 */
static void ApplyEx8ModeFallback(void)
{
	ChannelMemoryMessage_t *memory = NULL; /* 指向当前通道记忆，回落手控时要同步修改，避免下次切回通道又恢复脚控。 */

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
		return; /* 只有 PXBA/LGZI/LGZII 允许脚控失能后自动显示手控选中，PXBB及普通手柄不能误亮手控。 */
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
 * 函数功能：记录用户在脚踏在线期间通过屏幕手动切到非脚控模式。
 * 输入参数：无，直接读取当前脚踏在线标志。
 * 返回参数：无。
 */
static void MarkFootManualLock(void)
{
	if (ControlSignalMessage.jt_enable_flag == true)
	{
		s_foot_priority_manual_lock = 1U; /* 只在脚踏在线周期内锁住自动脚控优先，脚踏重新插入后会重新允许脚控抢占。 */
	}
}

/*
 * 函数功能：清除脚踏在线周期内的屏幕手动选择锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_ClearFootControlManualLock(void)
{
	s_foot_priority_manual_lock = 0U; /* 脚踏离线或用户重新选择脚控后，下一次脚踏上线可继续按脚控优先处理。 */
}

/*
 * 函数功能：脚踏上线时按“脚踏优先”规则尝试把当前控制模式切到脚控。
 * 输入参数：无，读取当前运行状态、触控状态、外控状态和屏幕手动锁存状态。
 * 返回参数：true 表示已切到脚控；false 表示运行中、外控占用或本在线周期被用户手动锁住。
 */
bool Pubinterface_ApplyFootControlPriorityOnConnect(void)
{
	uint8_t data[10] = {0U}; /* 隐藏触控弹窗时使用的空 UI 数据，保持 SendUIDSMessage 调用格式一致。 */

	if (WorkMessage.runflag_work == true)
	{
		return false; /* 手柄正在运行时不抢控制模式，避免脚踏热插入改变正在执行的控制来源。 */
	}

	if (MotorUart_IsRecentFeedbackMoving())
	{
		return false; /* 停止命令刚下发且近期反馈仍非零时保持原门禁；过期旧值不再让脚踏上线永久失效。 */
	}

	if (s_foot_priority_manual_lock != 0U)
	{
		return false; /* 本次脚踏在线期间用户已经从屏幕手动切到手控/触控，保持用户选择直到脚踏重新插入。 */
	}

	if (WorkMessage.hmiactive_work != 0U)
	{
		return false; /* 外部通信授权期间仍由外控退出流程释放，脚踏上线只刷新可用状态，不直接覆盖外控。 */
	}

	if (WorkMessage.touchactive_work == TOUCHWORK)
	{
		WorkMessage.touchactive_work = 0U; /* 脚踏重新上线并取得优先级时，关闭触控占用，避免实际脚控但触控弹窗仍在。 */
		SendUIDSMessage(UI_TOUCH_ID, false, data); /* 同步隐藏触控弹窗，使屏幕显示和实际控制模式保持一致。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN); /* 电机已停稳时释放屏幕 owner，允许脚踏后续踩下取得控制权。 */
	}

	WorkMessage.drivetype_work = JTWORK; /* 脚踏在线且未被用户锁住时，当前工作快照立即切到脚控。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		MemoryMsgA.drive_type = JTWORK; /* A 通道同步脚控记忆，后续刷新或切回 A 时显示和实际控制模式一致。 */
	}
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		MemoryMsgB.drive_type = JTWORK; /* B 通道同步脚控记忆，避免脚踏上线后屏幕显示脚控但通道记忆仍是手控。 */
	}

	return true; /* 脚踏优先切换已经完成，调用方只需要统一刷新控制模式显示。 */
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

	ApplyEx8ModeFallback(); /* 每次重绘控制方式前先消除“脚踏离线但脚控仍选中”的状态残留。 */

	foot_control_available = (ControlSignalMessage.jt_enable_flag == true); /* 脚踏图标按实际脚踏在线标志显示，避免未接入时仍可用。 */
	external_control_active = (WorkMessage.hmiactive_work != 0U); /* 外控内部复用 TOUCHWORK 做互斥，但显示层必须和触控按钮分开。 */
	handle_control_available = ((external_control_active == false) && Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)); /* 只有 PXBA/LGZI/LGZII 等带按键型号才显示手控可用，PXBB及通用磨钻保持灰色。 */
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
 * 返回参数：运行/排空态返回压力闭环后的实际输出速度，停止态返回 speed_work 设定速度。
 */
uint16_t Pubinterface_GetPumpDisplaySpeed(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针调用，显示 0 并避免心跳或屏幕刷新越界读取。 */
	}

	if (pump_message->run_flag || pump_message->timingDrainage_flag)
	{
		return pump_message->speed_output; /* 泵仍处于运行或排空请求时显示闭环后的真实输出，压力限速过程可以实时显示降速结果。 */
	}

	return pump_message->speed_work; /* 泵已经停止时回到设定速度显示；压力锁止只限制再次启动，不应把待机数值长期保持为 0。 */
}

/*
 * 函数功能：只刷新指定泵的启停/排空按钮，不重画流量数值和进度条。
 * 输入参数：button_area_id 为泵按钮区域，pump_message 为当前泵状态。
 * 返回参数：无。
 */
static void SendPumpButton(uint8_t button_area_id, const pumpMessage_t *pump_message)
{
	uint8_t display_value[10] = {0U}; /* UIDP 队列固定复制 10 字节，未使用的按钮参数必须保持为 0。 */
	bool pump_available = (pump_message->type != 0U); /* 设备码尚未识别时按钮保持禁用，不能显示成可操作状态。 */
	bool button_active = pump_message->run_flag; /* 抽吸泵和灌注泵仍使用普通运行标志控制按钮高亮。 */

	if (pump_message->type == INJECTWATER)
	{
		button_active = (pump_message->timingDrainage_flag || pump_message->pedalDrainage_flag); /* 注水泵按钮只表示排空，手柄冷却联动必须显示白色。 */
	}

	display_value[0] = (uint8_t)pump_message->type; /* Value[0] 携带泵类型，屏幕据此选择启动按钮或排空按钮资源。 */
	display_value[1] = button_active ? 1U : 0U; /* Value[1] 只表示按钮是否高亮，不携带泵流量。 */
	SendUIDSMessage(button_area_id, pump_available, display_value); /* 单独投递按钮消息，切换运行来源时不会误刷旧流量进度。 */
}

/*
 * 函数功能：只刷新 A 泵启停/排空按钮，不重画启动沿尚未更新的实际流量和档位环。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpAButtonDisplay(void)
{
	SendPumpButton(UI_PUMPABUTTON_ID, &pumpMessageA); /* 启动沿先同步 A 按钮，档位环等待泵任务发布真实 speed_output 后再刷新。 */
}

/*
 * 函数功能：只刷新 B 泵启停/排空按钮，不重画启动沿尚未更新的实际流量和档位环。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpBButtonDisplay(void)
{
	SendPumpButton(UI_PUMPBBUTTON_ID, &pumpMessageB); /* 启动沿先同步 B 按钮，避免旧 speed_output 为 0 时短暂显示 249 号图。 */
}

/*
 * 函数功能：把泵运行状态同步到屏幕泵区域和泵启停按钮。
 * 输入参数：pump_area_id 为泵数值区域，button_area_id 为启停按钮区域，pump_message 为当前泵状态。
 * 返回参数：无。
 */
static void SendPumpDisplay(uint8_t pump_area_id, uint8_t button_area_id, const pumpMessage_t *pump_message)
{
	uint8_t display_value[10] = {0U}; /* UIDP 队列固定拷贝 10 字节，泵刷新也使用同一缓冲格式。 */
	bool pump_available = (pump_message->type != 0U); /* 未识别到泵类型时暗掉区域，防止屏幕显示可控但泵任务没有有效设备。 */
	uint16_t display_speed = Pubinterface_GetPumpDisplaySpeed(pump_message); /* 运行态显示闭环后的实际输出速度，停止态继续显示设定速度。 */

	display_value[0] = (uint8_t)pump_message->type; /* Value[0] 传业务泵类型，UIPUMPADP/UIPUMPBDP 用它选择注水、灌注或抽吸图标。 */
	display_value[1] = (uint8_t)(display_speed >> 8); /* Value[1] 传显示速度高字节，闭环限速时会跟随实际输出变化。 */
	display_value[2] = (uint8_t)(display_speed & 0xFFU); /* Value[2] 传显示速度低字节，和 UIDP 现有解析顺序一致。 */
	SendUIDSMessage(pump_area_id, pump_available, display_value); /* 屏幕泵加减或启停后立刻刷新数值和档位。 */
	SendPumpButton(button_area_id, pump_message); /* 完整刷新时继续同步按钮；切换来源时可单独调用按钮刷新。 */
}

/*
 * 函数功能：刷新 A 泵屏幕区域和启停按钮。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpADisplay(void)
{
	SendPumpDisplay(UI_PUMPA_ID, UI_PUMPABUTTON_ID, &pumpMessageA); /* A 泵状态只读取 pumpMessageA，保持主工程 A/B 泵闭环映射不变。 */
}

/*
 * 函数功能：刷新 B 泵屏幕区域和启停按钮。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshPumpBDisplay(void)
{
	SendPumpDisplay(UI_PUMPB_ID, UI_PUMPBBUTTON_ID, &pumpMessageB); /* B 泵状态只读取 pumpMessageB，保持主工程 A/B 泵闭环映射不变。 */
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
		pumpMessageA.speed_output = output_speed; /* 只记录闭环后的实际输出，不覆盖 speed_work，保证下一次启动仍沿用用户设定。 */
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
static void SendDirectionDisplay(uint8_t ui_dir_id, bool enable_flag, bool selected_flag)
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
void Pubinterface_RefreshSelectedChannelDisplay(uint8_t channel)
{
	uint8_t display_value[10] = {0U}; /* 所有 UI 消息都使用 10 字节缓冲，保持 UIDP 队列拷贝格式稳定。 */
	bool planer_selected = Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type); /* 只有归一后的 PLANER 才按刨刀能力显示往复相关入口。 */
	bool osc_supported = IsOscDirectionAllowed(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type); /* 往复方向同时看手柄硬件和刨刀能力，普通手柄或磨头刀具都不误亮往复。 */
	bool osc_display_retained = ((osc_supported == false) &&
								 (Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model) == true) &&
								 (WorkMessage.tool_type == 0U) &&
								 (WorkMessage.dir_work == OSCDIR)); /* PXBA/PXBB 拔掉 RFID 刀具后会清当前刀具类型，但保留上次往复方向；显示层需要继续点亮往复图标来匹配实际运行状态。 */
	bool osc_display_available = (osc_supported || osc_display_retained); /* 正常有刨刀时按能力开放往复；无刀具但方向已保留为往复时只恢复显示，不改变按键切入往复的安全门槛。 */
	bool dir_locked = Pubinterface_IsDirLocked(WorkMessage.hand_model, WorkMessage.raw_tool_type); /* 固定方向型号高亮当前方向、其余方向置灰，屏幕、实体键和外控均不得改变。 */
	bool foot_control_available = (ControlSignalMessage.jt_enable_flag == true); /* 切通道刷新时脚踏图标也按实际在线状态显示。 */
	bool external_control_active = (WorkMessage.hmiactive_work != 0U); /* 外控占用时内部驱动方式也可能是 TOUCHWORK，但屏幕不能显示为触控。 */
	bool handle_control_available = ((external_control_active == false) && Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)); /* 切换 A/B 后也按当前手柄型号决定手控是否可用，LGZI 同样允许手控入口。 */
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
	RefreshToolDisplay(channel, planer_selected); /* 所有通道都刷新刀具区；普通手柄会主动隐藏 RFID 区，避免 A/B 切换残留。 */

	if (dir_locked)
	{
		SendDirectionDisplay(1U, (WorkMessage.dir_work == ZZDIR), (WorkMessage.dir_work == ZZDIR)); /* 固定正转时高亮正转；其它固定方向仍显示灰色正转图标但不可选。 */
		SendDirectionDisplay(2U, (WorkMessage.dir_work == FZDIR), (WorkMessage.dir_work == FZDIR)); /* 固定反转时高亮反转；其它固定方向仍显示灰色反转图标但不可选。 */
		SendDirectionDisplay(3U, (WorkMessage.dir_work == OSCDIR), (WorkMessage.dir_work == OSCDIR)); /* MXYTP高亮往复；其它固定方向手柄或刀具保留灰色往复图标但不可选。 */
	}
	else
	{
		if (osc_display_available)
		{
			SendDirectionDisplay(3U, true, (WorkMessage.dir_work == OSCDIR)); /* 当前可切入往复或已保留往复显示时点亮往复按钮，避免无刀具保留 OSCDIR 时图标变黑。 */
		}
		else
		{
			SendDirectionDisplay(3U, false, false); /* 当前手柄不支持往复时禁用往复按钮，避免屏幕误发后进入 OSCDIR。 */
		}
		SendDirectionDisplay(1U, true, (WorkMessage.dir_work == ZZDIR)); /* 可切换型号继续显示正转按钮，保持原有方向选择能力。 */
		SendDirectionDisplay(2U, true, (WorkMessage.dir_work == FZDIR)); /* 可切换型号继续显示反转按钮，保持原有方向选择能力。 */
	}

	if (osc_display_available && (WorkMessage.dir_work == OSCDIR))
	{
		display_value[0] = (uint8_t)WorkMessage.freq_work; /* 往复方向下显示当前通道频率值，频率范围已在识别/按键逻辑限制。 */
		display_value[1] = 0U; /* 0 表示刷新静态值，运行中字体颜色仍由速度/运行消息单独控制。 */
		SendUIDSMessage(UI_FREQ_ID, true, display_value); /* 往复可用或拔刀后保留往复时打开频率窗口，保证图标、频率和实际 OSCDIR 状态一致。 */
	}
	else
	{
		SendUIDSMessage(UI_FREQ_ID, false, display_value); /* 非往复或手柄不支持往复时关闭频率窗口，清掉上一个通道残留频率。 */
	}

	display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位为 WorkMessage 的实际 rpm。 */
	display_value[1] = (uint8_t)((WorkMessage.speed_set_work >>8)&0xFFU); /* 速度低字节按 UIDP 协议传输，保证 16 位速度完整显示。 */
	display_value[2] = (uint8_t)(WorkMessage.speed_set_work & 0xFFU);
	display_value[3] = 0U; /* 0 表示切通道后的静态速度刷新，不进入运行中颜色更新分支。 */
	display_value[4] = 0U; /* 切通道时电机未启动，速度字体保持非运行状态。 */
	SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 最后刷新速度值，保证切通道后的速度显示同步。 */
}

/*
 * 函数功能：按当前 WorkMessage/MemoryMsg 快照补刷主运行页手柄和当前通道参数区。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_RefreshRuntimeDisplaySnapshot(void)
{
	uint8_t current_channel = WorkMessage.channel_work; /* 保存当前选中通道，补刷过程只读状态，不改变控制归属。 */

	Pubinterface_RefreshOnlineHandleDisplay(); /* 先重发 A/B 在线图标，修正上电集中刷新时后到通道图标丢失的问题。 */
	if ((current_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == true))
	{
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* A 已选中且在线时补刷 A 的刀具规格、方向、速度和控制方式。 */
	}
	else if ((current_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == true))
	{
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* B 已选中且在线时补刷 B 的刀具规格、方向、速度和控制方式。 */
	}
	else if ((WorkMessage.Channel_Aonline == false) && (WorkMessage.Channel_Bonline == false))
	{
		Pubinterface_ClearSelectedChannelDisplay(); /* 两路都不在线时保持无手柄显示，避免开机补刷恢复旧参数区。 */
	}
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
 * 函数功能：按当前 A/B 泵类型和当前工作通道计算手柄冷却注水泵目标。
 * 输入参数：无，读取 pumpMessageA/B.type 和 WorkMessage.channel_work。
 * 返回参数：HANDLE_INJECTION_FOLLOW_PUMP_A/B 位图，0 表示当前没有可用注水冷却泵。
 */
static uint8_t GetInjectionPumpMask(void)
{
	bool a_is_injection = (pumpMessageA.type == INJECTWATER); /* 读取 A 泵识别类型，只有注水泵允许给手柄提供冷却水。 */
	bool b_is_injection = (pumpMessageB.type == INJECTWATER); /* 读取 B 泵识别类型，支持双注水泵时按通道对应冷却。 */
	uint8_t target_mask = 0U;								 /* 默认没有冷却目标，避免无注水泵时误启动其它类型泵。 */

	/* 两路都可供冷却时必须继续按当前 A/B 手柄分流，不能同时返回两个泵位。 */
	if (a_is_injection && b_is_injection)
	{
		if (WorkMessage.channel_work == CHANNEL_A)
		{
			target_mask = HANDLE_INJECTION_FOLLOW_PUMP_A; /* 双注水泵场景下，A 通道手柄只由 A 注水泵冷却。 */
		}
		else if (WorkMessage.channel_work == CHANNEL_B)
		{
			target_mask = HANDLE_INJECTION_FOLLOW_PUMP_B; /* 双注水泵场景下，B 通道手柄只由 B 注水泵冷却。 */
		}
	}
	else if (a_is_injection)
	{
		target_mask = HANDLE_INJECTION_FOLLOW_PUMP_A; /* 只有 A 是注水泵时，A/B 任一手柄运行都由 A 泵提供冷却。 */
	}
	else if (b_is_injection)
	{
		target_mask = HANDLE_INJECTION_FOLLOW_PUMP_B; /* 只有 B 是注水泵时，A/B 任一手柄运行都由 B 泵提供冷却。 */
	}

	return target_mask; /* 返回位图供联动启动和压力堵塞安全停机共用，避免两处规则不一致。 */
}

/*
 * 函数功能：停掉由手柄冷却联动占用的注水泵，并同步清除跟随标记。
 * 输入参数：stop_mask 需要释放的 A/B 注水泵位图。
 * 返回参数：无。
 */
static void StopInjectionPumps(uint8_t stop_mask)
{
	if ((stop_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		pumpMessageA.run_flag = false;			 /* A 注水泵由手柄冷却触发时必须立即撤销运行请求，避免无冷却对象后继续出水。 */
		pumpMessageA.timingDrainage_flag = false; /* 压力堵塞安全停机优先级高于排空，必须退出 A 排空状态。 */
		pumpMessageA.pedalDrainage_flag = false; /* 安全停机同时退出脚踏轻踩来源，防止下次普通联动继承。 */
		pumpMessageA.timingDrainage_times = 0U;   /* 清零 A 排空计数，后续排空必须重新开始完整周期。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_A); /* 释放 A 跟随标记，后续停止手柄不再重复处理 A 泵。 */
		Pubinterface_UpdatePumpAOutputSpeed(0U);  /* A 泵实际输出已经被压到 0，立即同步给屏幕和上位机显示。 */
		Pubinterface_RefreshPumpADisplay();       /* 刷新 A 泵按钮状态，保证运行态与实际 run_flag 一致。 */
	}

	if ((stop_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		pumpMessageB.run_flag = false;			 /* B 注水泵由手柄冷却触发时必须立即撤销运行请求，避免无冷却对象后继续出水。 */
		pumpMessageB.timingDrainage_flag = false; /* 压力堵塞安全停机优先级高于排空，必须退出 B 排空状态。 */
		pumpMessageB.pedalDrainage_flag = false; /* B 泵安全停机时同步退出脚踏轻踩来源。 */
		pumpMessageB.timingDrainage_times = 0U;   /* 清零 B 排空计数，后续排空必须重新开始完整周期。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_B); /* 释放 B 跟随标记，后续停止手柄不再重复处理 B 泵。 */
		Pubinterface_UpdatePumpBOutputSpeed(0U);  /* B 泵实际输出已经被压到 0，立即同步给屏幕和上位机显示。 */
		Pubinterface_RefreshPumpBDisplay();       /* 刷新 B 泵按钮状态，保证运行态与实际 run_flag 一致。 */
	}
}

/*
 * 函数功能：压力保护触发后停掉对应泵输出，并同步刷新屏幕/上位机可见的实际输出速度。
 * 输入参数：blocked_mask 触发压力保护的 A/B 泵位图。
 * 返回参数：无。
 */
static void StopPressureBlockedPumps(uint8_t blocked_mask)
{
	if ((blocked_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		pumpMessageA.run_flag = false;            /* A 泵压力到阈值后撤销运行门控，下一周期不能再按旧控制源输出。 */
		pumpMessageA.timingDrainage_flag = false; /* 压力保护优先级高于排空，必须同步退出 A 泵排空状态。 */
		pumpMessageA.pedalDrainage_flag = false;  /* 压力停泵同时关闭脚踏轻踩来源，松脚前不能继续向堵塞水路输出。 */
		pumpMessageA.timingDrainage_times = 0U;   /* A 泵排空计数清零，下一次排空必须由新的控制源重新触发。 */
		Pubinterface_UpdatePumpAOutputSpeed(0U);  /* A 泵实际输出已停，显示值和上位机运行值必须同步为 0。 */
		Pubinterface_RefreshPumpADisplay();       /* 立即刷新 A 泵按钮和数值，避免屏幕仍显示运行态。 */
	}

	if ((blocked_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		pumpMessageB.run_flag = false;            /* B 泵压力到阈值后撤销运行门控，等待下一次明确启动。 */
		pumpMessageB.timingDrainage_flag = false; /* 压力保护触发时 B 泵排空也必须停，避免继续向堵塞水路加压。 */
		pumpMessageB.pedalDrainage_flag = false;  /* B 泵压力停机时同步清除脚踏临时排空状态。 */
		pumpMessageB.timingDrainage_times = 0U;   /* B 泵排空计数归零，保持下一次排空动作从完整周期开始。 */
		Pubinterface_UpdatePumpBOutputSpeed(0U);  /* B 泵实际输出已停，屏幕和上位机不能继续显示设定流量。 */
		Pubinterface_RefreshPumpBDisplay();       /* 立即刷新 B 泵按钮和数值，保证 UI 状态跟随真实输出。 */
	}
}

/*
 * 函数功能：上报泵压力阈值触发后的 3 秒临时屏幕报警。
 * 输入参数：无，报警码固定使用 WORK_ALARM_PUMP_PRESSURE_BLOCKED。
 * 返回参数：无。
 */
static void RaisePressureAlarm(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余字节清零避免旧报警参数残留。 */

	display_value[0] = WORK_ALARM_PUMP_PRESSURE_BLOCKED; /* 压力触发时驱动 UIAIARMDP 显示屏幕新增的 89 号报警图。 */
	s_pump_pressure_blocked_transient_alarm_tick = HAL_GetTick(); /* 记录本次压力弹窗开始时间，后续周期服务按 3 秒自动关闭。 */
	if ((WorkMessage.alarm_flag == false) &&
		(s_common_socket_tool_missing_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 没有更高优先级报警或其它临时弹窗时，立即显示压力 89 号图。 */
		s_pump_pressure_blocked_transient_alarm_active = 1U; /* 只有实际显示过 89 号图，后续才允许本模块关闭报警区。 */
	}
	else
	{
		s_pump_pressure_blocked_transient_alarm_active = 0U; /* 报警区被其它报警占用时只蜂鸣不停留关闭权，避免到期误清其它弹窗。 */
	}
}

/*
 * 函数功能：周期关闭泵压力阈值产生的 3 秒临时屏幕报警。
 * 输入参数：无，直接读取压力弹窗归属和 HAL 毫秒 tick。
 * 返回参数：无。
 */
static void ServicePressureAlarm(void)
{
	if (s_pump_pressure_blocked_transient_alarm_active == 0U)
	{
		return; /* 当前屏幕报警区不由压力弹窗占用，本周期不处理 89 号图关闭。 */
	}

	if ((uint32_t)(HAL_GetTick() - s_pump_pressure_blocked_transient_alarm_tick) < ALARM_PRESSURE_MS)
	{
		return; /* 压力报警 3 秒保持时间未到，继续显示 89 号弹窗。 */
	}

	if ((WorkMessage.alarm_flag == false) &&
		(s_common_socket_tool_missing_alarm_active == 0U) &&
		(s_running_handle_unplug_transient_alarm_value == 0U))
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 没有全局报警和其它临时弹窗时，关闭本模块显示的压力报警区。 */
	}
	s_pump_pressure_blocked_transient_alarm_active = 0U; /* 无论是否实际清屏，本次压力临时弹窗生命周期都已结束。 */
	s_pump_pressure_blocked_transient_alarm_tick = 0U; /* 清时间戳，避免下一次压力触发沿用旧 tick。 */
}

/*
 * 函数功能：处理泵压力锁止事件；新触发时蜂鸣并显示 89 号压力弹窗，保持态只负责继续停泵和停手柄。
 * 输入参数：pump_channel 触发压力停泵的泵通道；new_event 为 true 表示本次刚跨过阈值，需要 3 秒蜂鸣和弹窗。
 * 返回参数：无。
 */
static void HandlePressureStop(uint8_t pump_channel, bool new_event)
{
	uint8_t blocked_mask = 0U;						 /* 当前触发压力堵塞的泵位图，非法通道保持 0 并直接退出。 */
	uint8_t target_mask = GetInjectionPumpMask(); /* 当前手柄运行理论上需要的冷却注水泵位图。 */
	uint8_t cooling_mask;							 /* 当前压力泵是否属于手柄冷却链路，理论目标和实际跟随标记都要参与判断。 */
	bool blocked_pump_is_injection = false;			 /* 记录触发压力阈值的泵是否为注水泵，脚踏直启泵时用于安全兜底停手柄。 */

	if (pump_channel == CHANNEL_A)
	{
		blocked_mask = HANDLE_INJECTION_FOLLOW_PUMP_A; /* A 泵压力触发时先定位 A 泵，再决定是否联动停手柄。 */
		blocked_pump_is_injection = (pumpMessageA.type == INJECTWATER); /* A 泵只有识别为注水泵时，堵塞才需要按手柄冷却风险处理。 */
	}
	else if (pump_channel == CHANNEL_B)
	{
		blocked_mask = HANDLE_INJECTION_FOLLOW_PUMP_B; /* B 泵压力触发时先定位 B 泵，再决定是否联动停手柄。 */
		blocked_pump_is_injection = (pumpMessageB.type == INJECTWATER); /* B 泵只有识别为注水泵时，堵塞才需要按手柄冷却风险处理。 */
	}
	else
	{
		return; /* 非法通道不属于 A/B 泵压力保护，避免误停手柄或外控请求。 */
	}

	if (new_event != false)
	{
		SendAlarmMessageTimed(PUMP_PRESSURE_BLOCKED_BEEP_ALARM, ALARM_PRESSURE_MS); /* 压力首次触发启动 3 秒报警蜂鸣，但仍不写全局 WorkMessage 报警。 */
		RaisePressureAlarm(); /* 同步显示 89 号压力报警弹窗，到期由周期服务自动关闭。 */
	}

	StopPressureBlockedPumps(blocked_mask); /* 无论该泵是否正在冷却手柄，触发压力阈值的泵都必须先停。 */
	ExternalComm_ClearPumpRunRequest(pump_channel); /* 清除外控层旧运行请求，避免下一次刷新把保护停机泵重新拉起。 */

	cooling_mask = (uint8_t)(target_mask | s_handle_injection_pump_follow_mask); /* 现场可能已经切通道或刷新类型，实际跟随标记也必须算入冷却目标。 */
	/* 堵塞泵不在当前冷却位图时，仅在“手柄仍运行且该泵确为注水泵”这一旧路径下补入联动停机。 */
	if ((cooling_mask & blocked_mask) == 0U)
	{
		if ((WorkMessage.runflag_work != false) && (blocked_pump_is_injection != false))
		{
			cooling_mask |= blocked_mask; /* 脚踏轻排或历史直启路径可能没有写跟随标记；手柄正在运行且注水泵堵塞时必须按冷却链路停手柄。 */
		}
		else
		{
			return; /* 堵塞的泵不是当前手柄冷却目标且手柄未运行时，只停该泵，不联动停止手柄电机。 */
		}
	}

	s_handle_pressure_block_stop_latched = 1U;        /* 锁存压力堵塞停机原因，连续脚踏/触控保活不能自动重新拉起手柄。 */
	WorkMessage.runflag_work = false;                 /* 立即撤销手柄运行命令，驱动任务下一周期发送停止帧。 */
	WorkMessage.speed_work = 0U;                       /* 同步清零实际目标速度，保证停机帧和状态显示一致。 */
	/* 压力停机不清 touchactive_work，触控松开后的超时逻辑需要该状态来调用 SetHandleInjectionPumpRun(false) 清锁存。 */
	ControlSignalMessage.handle_control_flag = false; /* 手柄实体键运行标志清零，避免实体键路径认为手柄仍在运行。 */
	ControlSignalMessage.HMI_control_flag = false;    /* 外控手柄运行标志清零，避免上位机图标继续显示手柄运行。 */
	ControlSignalMessage.jtL_control_flag = false;    /* 左脚踏运行标志清零，保持踩踏时也不会自动重启手柄。 */
	ControlSignalMessage.jtR_control_flag = false;    /* 右脚踏运行标志清零，保证双脚踏两侧都退出运行态。 */
	ControlSignalMessage.jtL_gentlypump_flag = false; /* 清除左轻排联动标志，避免停手柄时遗留旧泵联动。 */
	ControlSignalMessage.jtR_gentlypump_flag = false; /* 清除右轻排联动标志，保证 B 通道脚踏场景也能停净。 */
	ControlSignalMessage.HMI_gentlypump_flag = false; /* 清除外控轻排标志，避免外控状态继续保持泵输出。 */
	StopInjectionPumps(cooling_mask); /* 停止本次冷却目标和历史跟随目标，保证注水泵实际输出归零。 */
	Pubinterface_RefreshControlModeDisplay();         /* 控制来源运行态已被清理，立即刷新主运行页脚控/手控/触控图标。 */
}

/*
 * 函数功能：A/B 泵压力堵塞首次触发时停当前泵、启动 3 秒蜂鸣并显示 89 号压力弹窗，注水泵冷却手柄时额外停手柄。
 * 输入参数：pump_channel 触发压力停泵的泵通道，CHANNEL_A 表示 A 泵，CHANNEL_B 表示 B 泵。
 * 返回参数：无。
 */
void Pubinterface_HandlePumpPressureBlocked(uint8_t pump_channel)
{
	HandlePressureStop(pump_channel, true); /* 抽吸、注水、灌注泵首次触发都停本泵并蜂鸣；只有注水冷却链路才联动停手柄。 */
}

/*
 * 函数功能：压力锁止保持期间继续停当前泵；若注水泵正在冷却手柄则继续保持手柄停机，但不重复蜂鸣。
 * 输入参数：pump_channel 处于压力锁止的泵通道，CHANNEL_A 表示 A 泵，CHANNEL_B 表示 B 泵。
 * 返回参数：无。
 */
void Pubinterface_ServicePumpPressureHold(uint8_t pump_channel)
{
	HandlePressureStop(pump_channel, false); /* 保持态只压住本泵输出，注水冷却时才阻止连续控制源自动恢复手柄。 */
}

/*
 * 函数功能：A/B 泵步进驱动首次回报非零故障时，同步停泵与相关控制源；注水冷却泵故障时联动停手柄。
 * 输入参数：pump_channel 为驱动回报故障的泵通道，CHANNEL_A 表示 A 泵，CHANNEL_B 表示 B 泵。
 * 返回参数：无。
 */
void Pubinterface_HandlePumpDriverFault(uint8_t pump_channel)
{
	HandlePressureStop(pump_channel, false); /* 复用成熟的分泵停机和注水冷却联动链，驱动故障不弹出压力 89 号图。 */
}

/*
 * 函数功能：步进驱动故障锁存期持续保持本泵和必要的手柄冷却链停机，不重复触发蜂鸣。
 * 输入参数：pump_channel 为处于驱动故障锁存的泵通道，CHANNEL_A 表示 A 泵，CHANNEL_B 表示 B 泵。
 * 返回参数：无。
 */
void Pubinterface_ServicePumpDriverFaultHold(uint8_t pump_channel)
{
	HandlePressureStop(pump_channel, false); /* 每 25ms 重申安全停机，防止外控保活或连续控制源重新拉起故障泵。 */
}

/*
 * 函数功能：新的手控/触控/外控启动沿到来时清除压力停手柄锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_ClearPressureBlockStopLatchForNewTrigger(void)
{
	s_handle_pressure_block_stop_latched = 0U; /* 用户已经重新触发控制源，允许下一次启动进入泵任务重新按实时压力判断。 */
}

/*
 * 函数功能：查询注水冷却压力停机锁存是否仍处于有效状态。
 * 输入参数：无。
 * 返回参数：true 表示压力保护后尚未释放控制源，false 表示允许新的启动沿重新尝试运行。
 */
bool Pubinterface_IsPressureBlockStopLatched(void)
{
	return (s_handle_pressure_block_stop_latched != 0U); /* 脚踏保持踩下时必须先看该锁存，避免蜂鸣结束后旧控制源自动拉起手柄和泵。 */
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
		/* 两路都是注水泵时必须按当前手柄通道选择同侧冷却泵，防止双泵同时出水。 */
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

	if (enable == false)
	{
		s_handle_pressure_block_stop_latched = 0U; /* 控制源释放/停止视为结束本次压力停机锁存，但泵 pressure_hold 保留到下一次启动沿再清。 */
	}
	else if (s_handle_pressure_block_stop_latched != 0U)
	{
		WorkMessage.runflag_work = false;                 /* 压力堵塞停机未被用户确认前，拒绝脚踏保持/触控保活/上位机重复启动。 */
		WorkMessage.speed_work = 0U;                       /* 同步保持实际目标速度为 0，避免驱动任务被重复命令拉起。 */
		ControlSignalMessage.handle_control_flag = false; /* 清除手柄实体键运行来源，保持锁存期间不能自动恢复。 */
		ControlSignalMessage.HMI_control_flag = false;    /* 清除外控运行来源，上位机重复启动后图标也回到未运行。 */
		ControlSignalMessage.jtL_control_flag = false;    /* 清除左脚踏运行来源，踩住脚踏也必须等松开再重新启动。 */
		ControlSignalMessage.jtR_control_flag = false;    /* 清除右脚踏运行来源，双脚踏两侧行为一致。 */
		if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
		{
			ExternalComm_ClearPumpRunRequest(CHANNEL_A); /* A 冷却泵压力锁存期间清掉外控 A 泵请求，防止旧请求刷新后重启。 */
		}
		if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
		{
			ExternalComm_ClearPumpRunRequest(CHANNEL_B); /* B 冷却泵压力锁存期间清掉外控 B 泵请求，等待下一次明确启动。 */
		}
		StopInjectionPumps((uint8_t)(target_mask | s_handle_injection_pump_follow_mask)); /* 压力锁存中继续保持联动注水泵停止。 */
		Pubinterface_RefreshControlModeDisplay();         /* 运行来源已经被拒绝，立即刷新控制图标显示。 */
		return;
	}

	if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		bool drainage_was_active = (pumpMessageA.timingDrainage_flag || pumpMessageA.pedalDrainage_flag); /* 记录 A 泵是否正从屏幕或脚踏排空切入手柄联动。 */

		pumpMessageA.timingDrainage_flag = false; /* 手柄运行联动不是定时排空，先清 A 排空标志，避免后台排空计时抢写输出。 */
		pumpMessageA.pedalDrainage_flag = false; /* 进入手柄联动后结束脚踏轻踩来源，下一泵周期继续使用屏幕 speed_work。 */
		pumpMessageA.timingDrainage_times = 0U;   /* A 排空计数清零，保证后续真正进入排空时从完整周期开始。 */

		pumpMessageA.run_flag = true;			   /* 打开 A 泵任务运行门控，下一周期按注水公式和压力闭环输出。 */
		s_handle_injection_pump_follow_mask |= HANDLE_INJECTION_FOLLOW_PUMP_A; /* 记录 A 泵由手柄冷却跟随启动，停止时才允许本函数释放。 */
		if (drainage_was_active)
		{
			SendPumpButton(UI_PUMPABUTTON_ID, &pumpMessageA); /* 只把 A 排空按钮恢复为白色，不能用切换前的实际速度重画进度条。 */
		}
	}
	else if ((s_handle_injection_pump_follow_mask & HANDLE_INJECTION_FOLLOW_PUMP_A) != 0U)
	{
		pumpMessageA.run_flag = false;			 /* 当前手柄不再需要 A 冷却时，只关闭本函数曾启动过的 A 跟随输出。 */
		pumpMessageA.timingDrainage_flag = false; /* 同步退出 A 排空状态，保持手柄冷却和脚踏排空互斥。 */
		pumpMessageA.pedalDrainage_flag = false; /* 停止 A 手柄冷却时同步清除脚踏临时排空状态。 */
		pumpMessageA.timingDrainage_times = 0U;   /* A 排空计数清零，避免下一轮排空继承旧计数。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_A); /* 清掉 A 跟随占用标记，后续停止手柄不再重复改写 A 泵。 */
	}

	if ((target_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		bool drainage_was_active = (pumpMessageB.timingDrainage_flag || pumpMessageB.pedalDrainage_flag); /* 记录 B 泵任一排空来源到手柄联动的切换边沿。 */

		pumpMessageB.timingDrainage_flag = false; /* 手柄运行联动不是定时排空，先清 B 排空标志，避免后台排空计时抢写输出。 */
		pumpMessageB.pedalDrainage_flag = false; /* B 泵进入手柄冷却后结束脚踏轻踩来源，继续使用屏幕设定速度。 */
		pumpMessageB.timingDrainage_times = 0U;   /* B 排空计数清零，保证后续真正进入排空时从完整周期开始。 */
		pumpMessageB.run_flag = true;			   /* 打开 B 泵任务运行门控，下一周期按注水公式和压力闭环输出。 */
		s_handle_injection_pump_follow_mask |= HANDLE_INJECTION_FOLLOW_PUMP_B; /* 记录 B 泵由手柄冷却跟随启动，停止时才允许本函数释放。 */
		if (drainage_was_active)
		{
			SendPumpButton(UI_PUMPBBUTTON_ID, &pumpMessageB); /* B 通道同样只刷新按钮，避免旧 speed_output 瞬间显示为满格。 */
		}
	}
	else if ((s_handle_injection_pump_follow_mask & HANDLE_INJECTION_FOLLOW_PUMP_B) != 0U)
	{
		pumpMessageB.run_flag = false;			 /* 当前手柄不再需要 B 冷却时，只关闭本函数曾启动过的 B 跟随输出。 */
		pumpMessageB.timingDrainage_flag = false; /* 同步退出 B 排空状态，保持手柄冷却和脚踏排空互斥。 */
		pumpMessageB.pedalDrainage_flag = false; /* 停止 B 手柄冷却时清除脚踏临时排空状态。 */
		pumpMessageB.timingDrainage_times = 0U;   /* B 排空计数清零，避免下一轮排空继承旧计数。 */
		s_handle_injection_pump_follow_mask &= (uint8_t)(~HANDLE_INJECTION_FOLLOW_PUMP_B); /* 清掉 B 跟随占用标记，后续停止手柄不再重复改写 B 泵。 */
	}

}

/*
 * 函数功能：只读取得当前由手柄冷却联动持有的 A/B 泵位图。
 * 输入参数：无。
 * 返回参数：bit0 表示 A 泵正在被手柄联动持有，bit1 表示 B 泵正在被手柄联动持有。
 */
uint8_t Pubinterface_GetHandleInjectionPumpFollowMask(void)
{
	return s_handle_injection_pump_follow_mask; /* 8 位读取在当前 MCU 上为原子操作，脚踏只据此确认是否实际持有联动泵。 */
}

/*
 * 函数功能：清除可恢复的“手柄未连接”报警，供重新插入、触控退出等用户确认动作复用。
 * 输入参数：无。
 * 返回参数：true 表示本次确实清除了手柄未连接报警；false 表示当前不是该报警。
 */
bool Pubinterface_ClearHandleNotConnectedAlarm(void)
{
	if (WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED))
	{
		s_running_handle_unplug_transient_alarm_value = 0U; /* 全局掉线报警被用户确认后，同时清掉可能残留的手控临时报警归属。 */
		s_running_handle_unplug_transient_alarm_tick = 0U; /* 清掉临时报警时间戳，避免后续周期服务误关新的报警弹窗。 */
		WorkAlarm_Clear();                 /* 用户已经通过重新插入或退出动作确认掉线故障，清除运行中拔手柄留下的报警锁存。 */
		SendAlarmMessage(WORK_ALARM_NONE); /* 报警状态清零后同步关闭蜂鸣，避免故障确认后仍持续报警。 */
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 屏幕报警弹窗随报警状态一起关闭，后续由插入事件刷新手柄和参数区。 */
		return true;					   /* 告诉调用方本次恢复动作确实关闭了手柄掉线报警。 */
	}
	return false;						   /* 当前没有手柄未连接报警，调用方不需要做额外恢复处理。 */
}

/*
 * 函数功能：上报手控模式运行中拔手柄的 3 秒临时报警。
 * 输入参数：无，报警码固定使用 WORK_ALARM_HANDLE_NOT_CONNECTED。
 * 返回参数：无。
 */
static void RaiseUnplugAlarm(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0]，其余清零避免沿用上一条报警参数。 */

	display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 手控运行中掉线同样使用“请连接手柄”报警图，保证提示含义一致。 */
	SendAlarmMessageTimed(WORK_ALARM_HANDLE_NOT_CONNECTED, ALARM_UNPLUG_MS); /* 手控没有持续按压源，蜂鸣只保持 3 秒后自动停止。 */
	ExternalComm_SendTransientAlarm(WORK_ALARM_HANDLE_NOT_CONNECTED, ALARM_UNPLUG_MS); /* 上位机同步收到临时报警，3 秒后自动回到无报警。 */
	SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 手控掉线必须弹窗提示 3 秒，弥补原来只有蜂鸣没有屏幕提示的问题。 */
	s_running_handle_unplug_transient_alarm_value = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 记录本模块拥有临时弹窗，周期到期后只清理自己的提示。 */
	s_running_handle_unplug_transient_alarm_tick = HAL_GetTick(); /* 记录弹窗开始时间，避免依赖按键任务是否继续有事件。 */
}

/*
 * 函数功能：周期关闭手控运行中拔手柄产生的 3 秒临时屏幕报警。
 * 输入参数：无，直接读取临时报警归属和 HAL 毫秒 tick。
 * 返回参数：无。
 */
static void ServiceUnplugAlarm(void)
{
	if (s_running_handle_unplug_transient_alarm_value == 0U)
	{
		return; /* 当前没有手控掉线临时弹窗，本周期不处理屏幕报警区。 */
	}

	if ((uint32_t)(HAL_GetTick() - s_running_handle_unplug_transient_alarm_tick) < ALARM_UNPLUG_MS)
	{
		return; /* 未到 3 秒保持时间，继续显示临时报警弹窗。 */
	}

	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 没有全局真实报警时才关闭弹窗，避免误清其它模块的持续报警。 */
	}
	s_running_handle_unplug_transient_alarm_value = 0U; /* 临时报警生命周期结束，允许下一次手控掉线重新显示。 */
	s_running_handle_unplug_transient_alarm_tick = 0U; /* 清时间戳，避免下次比较时使用旧 tick。 */
	Handle_SelectRemainingOnlineAfterUnplug(); /* 手控掉线提示已经结束，自动装载唯一剩余通道但保持电机停止。 */
}

/*
 * 函数功能：周期维护公共接头缺刀具、泵压力堵塞和运行中拔手柄三类限时报警。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pubinterface_ServiceTransientAlarms(void)
{
	ServiceCommonSocketToolAlarm(); /* 维护公共接头缺刀具弹窗的保持和退出时机。 */
	ServicePressureAlarm(); /* 维护泵压力堵塞弹窗的保持和退出时机。 */
	ServiceUnplugAlarm(); /* 维护运行中拔手柄弹窗的保持和退出时机。 */
	if (s_running_handle_unplug_transient_alarm_value == 0U)
	{
		Handle_SelectRemainingOnlineAfterUnplug(); /* 手控3秒提示结束后或其它来源报警清除后，周期重试待恢复通道，避免一次检查失败后永久丢失。 */
	}
}

/*
 * 函数功能：手柄拔出清屏或队列复位后，重新补发当前仍有效的掉线报警弹窗。
 * 输入参数：无，读取手控临时报警归属和全局手柄未连接报警状态。
 * 返回参数：无。
 */
void Pubinterface_RefreshHandleUnplugAlarmDisplay(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 固定只使用 Value[0] 传报警码，补发时也保持同一协议。 */

	if ((s_running_handle_unplug_transient_alarm_value == WORK_ALARM_HANDLE_NOT_CONNECTED) ||
		WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED))
	{
		display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 手控临时报警和触控持续报警都显示“请连接手柄”80号图。 */
		SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 最后一个手柄拔出会复位 UI 队列，清屏后必须补发报警弹窗。 */
	}
}

/*
 * 函数功能：触控模式运行中拔手柄后，在用户松开运行按钮时退出触控来源并清除掉线报警。
 * 输入参数：无，由屏幕 0x5520 保活释放检测调用。
 * 返回参数：无。
 */
void Pubinterface_ReleaseTouchHandleNotConnectedAlarm(void)
{
	uint8_t data[10] = {0U}; /* 关闭触控弹窗时使用全零参数，保持 UIDP 消息格式稳定。 */

	if (WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED) == false)
	{
		return; /* 只有运行中拔手柄报警允许通过松开触控运行按钮自动退出，不能误清其它报警。 */
	}

	Pubinterface_SetHandleInjectionPumpRun(false); /* 松开触控运行按钮等价于退出控制源，必须再次确保手柄联动注水泵关闭。 */
	WorkMessage.runflag_work = false; /* 触控来源退出时撤销电机运行命令，防止报警清除后旧保活状态重新起机。 */
	WorkMessage.speed_work = 0U; /* 同步清零实际输出速度，保证显示、驱动停止帧和报警恢复一致。 */
	WorkMessage.touchactive_work = 0U; /* 用户已经松开运行按钮，触控来源结束，不再占用本机控制模式。 */
	WorkMessage.drivetype_work = ControlArbitration_GetLocalDriveTypeAfterExit(); /* 退出触控后按脚踏优先、手控次之恢复本机可用控制方式。 */
	(void)Pubinterface_ClearHandleNotConnectedAlarm(); /* 控制源已退出，运行中拔手柄报警生命周期结束，关闭蜂鸣和报警弹窗。 */
	Handle_SelectRemainingOnlineAfterUnplug(); /* 触控已真实松开且报警已清，自动恢复唯一剩余通道但不启动电机。 */
	SendUIDSMessage(UI_TOUCH_ID, false, data); /* 同步隐藏触控运行弹窗，避免报警关闭后仍显示触控工作区。 */
	ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN); /* 电机已停稳时释放屏幕 owner，允许后续脚踏、手控或触控重新进入。 */
	Pubinterface_RefreshControlModeDisplay(); /* 报警和触控来源都已退出，立即刷新三种控制方式高亮状态。 */
}

/*
 * 函数功能：运行中当前工作手柄掉线时，统一停止电机和手柄联动注水泵，并锁存屏幕报警。
 * 输入参数：无，函数读取当前 WorkMessage、ControlSignalMessage 和 A/B 泵状态。
 * 返回参数：无。
 */
void Pubinterface_StopRunningHandleOnUnplug(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余字节清零避免旧值残留。 */
	uint8_t unplug_drive_type = WorkMessage.drivetype_work; /* 先保存掉线前控制方式，后面清运行标志后还要按来源决定报警生命周期。 */
	bool unplug_from_touch = ((unplug_drive_type == TOUCHWORK) &&
							  (WorkMessage.touchactive_work == TOUCHWORK) &&
							  (WorkMessage.hmiactive_work == 0U)); /* 本机触控运行掉线要等用户松开运行按钮后退出报警。 */
	bool unplug_from_handle = ((unplug_drive_type == HANDLEWORK) ||
							   (ControlSignalMessage.handle_control_flag == true)); /* 手控运行掉线没有持续控制源，只做 3 秒临时提示。 */

	WorkMessage.runflag_work = false;             /* 当前手柄物理掉线后必须撤销电机运行命令，驱动任务下一周期发送停止帧。 */
	WorkMessage.speed_work = 0U;                   /* 实际输出速度同步清零，避免停止帧前继续沿用掉线手柄的目标速度。 */
	if (unplug_from_touch == false)
	{
		WorkMessage.touchactive_work = 0U;         /* 非触控来源掉线时立即释放触控/外控占用，避免手控或脚踏被旧状态挡住。 */
	}
	WorkMessage.hmiactive_work = 0U;               /* 外控运行态随当前手柄掉线撤销，外部需要重新申请后才能再次控制。 */
	ControlSignalMessage.handle_control_flag = false; /* 手柄实体键来源掉线后不再占用运行状态，避免实体键状态残留。 */
	ControlSignalMessage.HMI_control_flag = false;    /* 外控手柄运行请求失效，避免小电脑图标继续显示运行输出。 */
	ControlSignalMessage.HMI_enable_flag = false;     /* 当前外控运行被故障打断，重新控制必须重新申请外控授权。 */
	ControlSignalMessage.jtL_control_flag = false;    /* 左脚踏运行状态随当前手柄掉线清除，防止脚踏任务继续认为电机在转。 */
	ControlSignalMessage.jtR_control_flag = false;    /* 右脚踏运行状态同样清除，保证双脚踏任一路都不会残留运行。 */
	ControlSignalMessage.jtL_gentlypump_flag = false; /* 脚踏轻排联动标志清零，避免掉线后释放脚踏时再次处理旧泵状态。 */
	ControlSignalMessage.jtR_gentlypump_flag = false; /* 右脚踏轻排联动标志清零，保证 B 通道脚踏场景也能停净。 */
	ControlSignalMessage.HMI_gentlypump_flag = false; /* 外控轻排联动标志清零，避免上位机旧状态继续保持泵输出。 */
	ExternalComm_ClearHandleInjectionPumpFollow();    /* 释放外控保存的手柄冷却跟随请求，防止后续外控刷新重新启动注水泵。 */
	Pubinterface_SetHandleInjectionPumpRun(false);     /* 关闭由手柄运行触发的 A/B 注水泵冷却跟随。 */
	if (pumpMessageA.type == INJECTWATER)
	{
		pumpMessageA.run_flag = false;           /* A 为注水泵时强制停泵，手柄已经掉线不再需要冷却供水。 */
		pumpMessageA.timingDrainage_flag = false; /* 掉线停泵优先级高于排空，必须同步退出排空计时。 */
		pumpMessageA.pedalDrainage_flag = false;  /* 手柄掉线时退出 A 泵脚踏轻踩来源，防止报警解除后沿用旧状态。 */
		pumpMessageA.timingDrainage_times = 0U;  /* 清掉 A 排空计数，避免下次启动继承掉线前的排空时间。 */
	}
	if (pumpMessageB.type == INJECTWATER)
	{
		pumpMessageB.run_flag = false;           /* B 为注水泵时同样强制停泵，覆盖 B 唯一注水泵和双注水泵场景。 */
		pumpMessageB.timingDrainage_flag = false; /* B 注水泵掉线停泵时退出排空模式，防止泵任务继续输出。 */
		pumpMessageB.pedalDrainage_flag = false;  /* B 泵掉线停机时同步清除脚踏轻踩来源。 */
		pumpMessageB.timingDrainage_times = 0U;  /* 清掉 B 排空计数，保证下次运行从干净状态开始。 */
	}
	Pubinterface_RefreshPumpADisplay();          /* A 泵实际运行状态已改变，立即刷新屏幕显示，保证显示值和输出一致。 */
	Pubinterface_RefreshPumpBDisplay();          /* B 泵实际运行状态已改变，立即刷新屏幕显示，避免用户看到泵仍在运行。 */
	ControlArbitration_ForceRelease();           /* 手柄掉线属于故障停机，释放当前 owner，避免外控/触控/脚踏占用残留。 */
	if ((unplug_from_handle != false) && (unplug_from_touch == false))
	{
		RaiseUnplugAlarm(); /* 手控来源掉线按 3 秒临时报警处理，到期自动恢复正常显示和蜂鸣。 */
	}
	else
	{
		WorkAlarm_Set(WORK_ALARM_HANDLE_NOT_CONNECTED); /* 脚踏、触控和外控掉线仍写全局报警，等待对应控制源释放或退出确认。 */
		SendAlarmMessage(WORK_ALARM_HANDLE_NOT_CONNECTED); /* 持续蜂鸣直到控制源退出或用户恢复流程清除。 */
		display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 报警码写入 UI 队列 Value[0]，驱动 UIAIARMDP 显示 80 号报警图。 */
		SendUIDSMessage(UI_AIARM_ID, true, display_value);  /* 立即弹出屏幕报警，避免运行中掉线只停机但无可见提示。 */
	}
	Pubinterface_RefreshControlModeDisplay();     /* 控制来源状态已被清除，刷新脚控/手控/触控高亮，避免界面残留运行态。 */
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
 * 函数功能：把 Page4/RFID 默认注水流量规整到可启动的泵业务范围。
 * 输入参数：flow 为通道记忆中的默认注水流量。
 * 返回参数：1~300 直接返回；0 或超过 300 时返回程序默认流量 30。
 */
static uint16_t ClampInjectionStartFlow(uint16_t flow)
{
	if ((flow < HANDLE_INJECTION_PUMP_FLOW_MIN) ||
		(flow > HANDLE_INJECTION_PUMP_FLOW_MAX))
	{
		return HANDLE_INJECTION_PUMP_DEFAULT_FLOW; /* 异常 EEPROM/RFID 流量不能写入泵速度，统一使用 30 保证启动可控。 */
	}

	return flow; /* 有效配置值直接使用，保证 EEPROM 写多少，注水泵默认就按多少运行。 */
}

/*
 * 函数功能：取得注水泵启动时使用的非零流量，优先使用当前手柄 Page4，Page4 为空或越界时回退 30。
 * 输入参数：无。
 * 返回参数：可直接写入 pumpMessageA/B.speed_work 的非零注水泵启动流量。
 */
uint16_t Pubinterface_GetInjectionPumpStartFlow(void)
{
	uint16_t flow = Pubinterface_GetCurrentDefaultInjectionFlow(); /* 先读当前通道 Page4 默认注水流量，保证有配置时仍以 EEPROM 为准。 */

	return ClampInjectionStartFlow(flow); /* 0 或越界时统一回退 30，正常 1~300 直接作为注水泵速度。 */
}

/*
 * 函数功能：读取当前工作通道当前方向的默认电机速度，供脚踏启动时替代旧的固定 60000。
 * 输入参数：无，函数内部读取 WorkMessage.channel_work 和 WorkMessage.dir_work。
 * 返回参数：当前方向默认速度，单位沿用 WorkMessage.speed_set_work 的实际 rpm；无有效通道时返回 0。
 */
uint32_t Pubinterface_GetCurrentDefaultMotorSpeed(void)
{
	ChannelMemoryMessage_t *memory = NULL; /* 指向当前工作通道记忆结构，后续按方向读取该通道 Page4 默认速度。 */

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
 * 函数功能：把指定默认流量写入所有已识别为注水泵的泵口，并立即刷新泵显示。
 * 输入参数：flow 为待装载的默认流量；0 或越界时按程序默认 30 处理。
 * 返回参数：无。
 */
void Pubinterface_ApplyInjectionPumpDefaultFlow(uint16_t flow)
{
	uint16_t default_flow = ClampInjectionStartFlow(flow); /* 统一规整默认流量，保证无手柄或 EEPROM 异常时都回到 30。 */

	if (pumpMessageA.type == INJECTWATER)
	{
		pumpMessageA.speed_work = default_flow; /* A 泵是注水泵时写入新的待机流量，使后续屏幕启动和手柄联动使用同一设定。 */
		Pubinterface_RefreshPumpADisplay(); /* 默认流量变化后立即刷新 A 泵数值，避免屏幕停留在泵先上线时的 30。 */
	}

	if (pumpMessageB.type == INJECTWATER)
	{
		pumpMessageB.speed_work = default_flow; /* B 泵是注水泵时同步写入同一默认流量，保证单注水泵或双注水泵场景一致。 */
		Pubinterface_RefreshPumpBDisplay(); /* 默认流量变化后立即刷新 B 泵数值，确保插拔手柄后屏幕显示跟随当前通道。 */
	}
}

/*
 * 函数功能：手柄上线或回落选中时把指定通道 Page4 默认注水流量写入注水泵速度。
 * 输入参数：channel 本次成为当前选中手柄的通道。
 * 返回参数：无。
 */
void Pubinterface_ApplyChannelDefaultInjectionFlow(uint8_t channel)
{
	Pubinterface_ApplyInjectionPumpDefaultFlow(Pubinterface_GetChannelDefaultInjectionFlow(channel)); /* 只在通道成为当前选中时装载 Page4 默认流量，避免普通运行周期覆盖用户手动调节。 */
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
	ChannelMemoryMessage_t *memory = NULL; /* 指向即将成为当前工作快照的通道记忆结构。 */

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

	WorkMessage.current_work = memory->current_work;			  /* 同步当前通道保护电流，单位 0.01A，后续启动时按该手柄限流。 */
	WorkMessage.tool_reduction_ratio = memory->tool_reduction_ratio; /* 同步刀具减速比，保证速度换算跟随通道。 */
	WorkMessage.dir_work = memory->dir;						  /* 同步当前方向，速度选择依赖这个方向字段。 */
	if (ControlArbitration_IsExternalActive() == true)
	{
		WorkMessage.drivetype_work = TOUCHWORK; /* 外控切换 A/B 时只装载目标通道参数，运行控制方式继续归属外控，避免脚踏在线导致图标短暂变黄。 */
	}
	else if ((ControlSignalMessage.jt_enable_flag == true) && (s_foot_priority_manual_lock == 0U))
	{
		WorkMessage.drivetype_work = memory->drive_type = JTWORK;		  /* 脚踏在线且未被屏幕手动锁住时，切通道仍按脚控优先装载。 */
	}
	else
	{
		WorkMessage.drivetype_work = memory->drive_type;				  /* 脚踏离线或本在线周期用户已手动切换时，保留通道原控制方式记忆。 */
	}
	WorkMessage.freq_work = memory->freq;						  /* 同步往复频率，避免 A/B 频率串用。 */
	WorkMessage.tool_type = memory->tool_type;					  /* 同步刨/磨刀具类型，界面和限速逻辑都读取这里。 */
	WorkMessage.raw_tool_type = (memory->tool_type != 0U) ? memory->raw_tool_type : 0U; /* 当前工作快照只在有效刀具存在时保留原始码，避免掉线历史误导驱动。 */
	WorkMessage.auto_identify = memory->auto_identify;			  /* 同步当前通道自动识别状态，切通道后 RFID 识别模式不丢失。 */
	WorkMessage.hand_model = memory->hand_model;				  /* 同步手柄型号，手柄按键扫描依赖当前型号判断。 */
	WorkMessage.channel_work = channel;						  /* 最后切换当前工作通道，避免中间状态被其它任务读成新通道旧参数。 */
	ApplyEx8ModeFallback();				  /* EX8 要求脚踏离线时只有PXBA/LGZI/LGZII切通道后落到手控；PXBB无实体按键，继续保持非手控。 */

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
bool Pubinterface_ShouldAutoSelectPluggedChannel(uint8_t channel)
{
	if (((channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == false)) ||
		((channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == false)))
	{
		return false; /* 插拔事件还没有把在线标志置位时，不允许装载该通道，避免选中一个未上线通道。 */
	}

	/* 只有电机未运行时才允许新插入通道自动成为当前通道，避免运行中抢占控制参数。 */
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
void Pubinterface_SaveRecognizeToMemory(uint8_t channel)
{
	ChannelrecognizeMessage_t *recognize = NULL; /* 指向扫描层刚刚完成认证的临时识别缓存。 */
	ChannelMemoryMessage_t *memory = NULL;		  /* 指向要更新的 A/B 通道记忆结构。 */
	uint8_t keep_auto_identify = 0U;				  /* RFID 二次刷新前保留通道自动识别状态，避免新刀具参数覆盖该模式记忆。 */
	uint32_t keep_zz_speed = 0U;				  /* 同一 EPC 重识别前暂存用户调过的正转速度，避免标签默认值覆盖屏幕调节结果。 */
	uint32_t keep_fz_speed = 0U;				  /* 同一 EPC 重识别前暂存用户调过的反转速度，保证异常掉线恢复后仍按用户设置运行。 */
	uint32_t keep_osc_speed = 0U;				  /* 同一 EPC 重识别前暂存用户调过的往复速度，避免自动识别重刷回标签默认速度。 */
	uint16_t keep_freq = 0U;					  /* 同一 EPC 重识别前暂存用户调过的往复频率，确保屏幕显示和实际运行一致。 */
	uint16_t keep_dir = 0U;					  /* 同一 EPC 重识别前暂存用户最后选择的方向，重识别后仍按该方向选择速度。 */
	bool keep_rfid_user_runtime = false;		  /* true 表示本次为同一 EPC 自动识别恢复，需要保留用户运行参数。 */

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
	keep_rfid_user_runtime = (bool)((recognize->rfid_cache_hit != 0U) &&
									(memory->auto_identify != 0U) &&
									(memory->hand_model == recognize->handle_type) &&
									(memory->tool_type != 0U) &&
									(recognize->tool_type != 0U) &&
									(memory->raw_tool_type == recognize->raw_tool_type)); /* 只有同一 EPC 在自动识别模式下恢复时才保留旧运行参数，新刀具或手动模式必须使用新来源默认值。 */
	if (keep_rfid_user_runtime != false)
	{
		keep_zz_speed = memory->zz_speed;		  /* 保存用户调过的正转速度，后面 memset 会清空整个通道记忆。 */
		keep_fz_speed = memory->fz_speed;		  /* 保存用户调过的反转速度，避免同一 EPC 回来后回到标签默认速度。 */
		keep_osc_speed = memory->osc_speed;		  /* 保存用户调过的往复速度，防止 RFID 短暂掉线造成参数跳变。 */
		keep_freq = memory->freq;				  /* 保存用户调过的频率，保证重新识别同一刨刀时频率不回默认 4Hz。 */
		keep_dir = memory->dir;					  /* 保存用户选择的方向，恢复后屏幕高亮和速度选择仍跟随用户操作。 */
	}
	if ((recognize->handle_type != 0U) && (recognize->tool_type == 0U) && (Pubinterface_IsSplitToolSpecDisplayModel(recognize->handle_type) == true))
	{
		keep_auto_identify = 1U; /* PXBA/PXBB 新插入基座默认打开自动识别，直到用户主动切回手动模式。 */
		Pubinterface_SetLastRfidToolType(channel, 0U); /* 新基座上线但还没有刀具结果时，0x1404 必须显示等待图 63，不沿用旧掉线图。 */
	}
	memset(memory, 0, sizeof(*memory));			  /* 新手柄上线前清掉该通道旧 EEPROM 参数，避免旧字段残留参与后续切换。 */
	memory->hand_model = recognize->handle_type;  /* 保存手柄型号，后续通道切换时再装载到 WorkMessage。 */
	memory->hand_type_raw_major = recognize->hand_type_raw_major; /* 保存 EEPROM 原始主类型，上位机心跳需要区分真实编码。 */
	memory->hand_type_raw_minor = recognize->hand_type_raw_minor; /* 保存 EEPROM 原始子类型，便于上位机显示和售后定位。 */
	memory->current_work = recognize->overloadThresholdFor;		  /* 保存 Page4/RFID 保护电流，单位 0.01A；当前通道被选中后才影响 WorkMessage。 */
	memory->tool_reduction_ratio = (recognize->tool_reduction_ratio != 0U) ? recognize->tool_reduction_ratio : ((recognize->meioticratio > 1U) ? ((uint32_t)recognize->meioticratio * 100U) : 100U); /* 完整x100倍率优先；旧整数镜像乘100，0/1统一归一为1.00倍。 */
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
	/* 同一 RFID 刀具重新识别时恢复用户运行参数，避免周期刷新把现场调节值重置为标签默认值。 */
	if (keep_rfid_user_runtime != false)
	{
		if (keep_zz_speed != 0U)
		{
			memory->zz_speed = keep_zz_speed;					  /* 同一 EPC 恢复时保留用户正转速度，屏幕值和下次启动目标保持一致。 */
		}
		if (keep_fz_speed != 0U)
		{
			memory->fz_speed = keep_fz_speed;					  /* 同一 EPC 恢复时保留用户反转速度，避免标签默认值覆盖用户调整。 */
		}
		if (keep_osc_speed != 0U)
		{
			memory->osc_speed = keep_osc_speed;					  /* 同一 EPC 恢复时保留用户往复速度，异常掉线不会造成速度跳回默认。 */
		}
		if (keep_freq != 0U)
		{
			memory->freq = keep_freq;							  /* 同一 EPC 恢复时保留用户往复频率，屏幕频率和实际运行频率继续一致。 */
		}
		if ((Pubinterface_IsDirLocked(memory->hand_model, memory->raw_tool_type) == false) &&
			((keep_dir == ZZDIR) ||
			 (keep_dir == FZDIR) ||
			 ((keep_dir == OSCDIR) && (IsOscDirectionAllowed(memory->hand_model, memory->tool_type, memory->raw_tool_type) == true))))
		{
			memory->dir = keep_dir;								  /* 方向仍被当前手柄和刀具支持时才恢复，避免异常方向写回运行状态。 */
		}
	}
	if (memory->tool_type != 0U)
	{
		Pubinterface_SetLastRfidToolType(channel, memory->tool_type); /* RFID/EEPROM 成功写入刀具后，按业务类型更新掉线图标，避免原始标签代号无法映射 61/62。 */
	}

	if (Pubinterface_IsHandleControlReservedModel(memory->hand_model))
	{
		ControlSignalMessage.HMI_enable_flag = true;			  /* 带手控入口手柄上线后打开手控可用显示，实体键是否能启动由 handlekey 按型号再判断。 */
		if ((WorkMessage.drivetype_work == TOUCHWORK) ||
			(WorkMessage.drivetype_work == JTWORK) ||
			(WorkMessage.drivetype_work == HANDLEWORK))
		{
			memory->drive_type = WorkMessage.drivetype_work;	  /* RFID 只刷新刀具和运行参数，当前已明确选中的触控/脚控/手控必须原样保留，避免 PXBA 手控被脚踏在线状态覆盖。 */
		}
		else
		{
			if ((ControlSignalMessage.jt_enable_flag == true) && (s_foot_priority_manual_lock == 0U))
			{
				memory->drive_type = JTWORK;					  /* 脚控使能且本在线周期没有用户手动锁定时，才按脚踏优先级写入脚控记忆。 */
			}
			else
			{
				memory->drive_type = HANDLEWORK;				  /* 没有其它控制方式占用，或用户已手动锁住非脚控时，带按键手柄保持手控记忆。 */
			}
		}
	}
	else if (IsCommonSocketReserved(memory->hand_model))
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
	ChannelMemoryMessage_t *memory = NULL;		  /* 指向通道记忆；RFID 刀具头离线时保留速度等历史参数，但当前可运行刀具必须清零。 */
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
	recognize->rfid_cache_hit = 0U;			   /* 清扫描层同一 EPC 命中标志，只有下一次真实识别到标签后才允许恢复用户调节值。 */
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
	recognize->speed_zzstep_large = 0U;		   /* 清扫描层正转大步进，下一次识别成功后再由 Page6 重新装入。 */
	recognize->speed_fzstep_large = 0U;		   /* 清扫描层反转大步进，避免离线刀具头继续影响屏幕快减/快加。 */
	recognize->speed_oscstep_large = 0U;	   /* 清扫描层往复大步进，频率显示仍由当前方向和手柄能力决定。 */
	recognize->speed_min = 0U;				   /* 清扫描层速度下限，上位机无当前刀具头时不显示在线限速范围。 */
	recognize->speed_max = 0U;				   /* 清扫描层速度上限，下一次 RFID 成功后重新写入。 */
	recognize->default_injection_flow = 0U;	   /* 清扫描层刀具默认泵流量，MemoryMsg 中上次运行泵流量继续保留。 */

	memory->hand_model = recognize->handle_type; /* 只刷新基座型号，刀具头缺失不代表手柄基座离线，也不能清上次刀具运行参数。 */
	memory->hand_type_raw_major = recognize->hand_type_raw_major; /* 保留 EEPROM Page2 原始主类型，上位机仍能显示 PXBA/PXBB 基座。 */
	memory->hand_type_raw_minor = recognize->hand_type_raw_minor; /* 保留 EEPROM Page2 原始子类型，避免基座类型在心跳中消失。 */
	memory->drive_type = keep_drive_type;		   /* 恢复该通道控制方式记忆，只清扫描层刀具头在线状态，不改变用户控制来源。 */
	memory->auto_identify = keep_auto_identify;	   /* 保留原有自动识别记忆状态，掉线后继续沿用用户选中的模式。 */
	memory->raw_tool_type = keep_raw_tool_type;	   /* 通道记忆继续保留最近一次原始刀具型号，只用于掉线图标和后续重新识别参考。 */
	memory->tool_type = 0U;					   /* 清掉通道当前可运行刀具能力，公共接头缺刀具时启动 gate 必须失败并报警 80。 */

	if (WorkMessage.channel_work == channel)
	{
		WorkMessage.tool_type = 0U;				   /* 当前选中通道的 RFID 刀具已离线，运行快照同步失效，防止脚踏继续按旧刀具放行。 */
		WorkMessage.raw_tool_type = 0U;			   /* 当前工作快照清原始刀具型号，避免驱动和上位机误读为仍有在线 RFID 标签。 */
		WorkMessage.speed_work = 0U;				   /* 刀具缺失后实际输出目标必须清零，避免下一次启动前残留脚踏比例速度。 */
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
 * 函数功能：按 Page4 的 2 字节速度报警阈值计算 WorkMessage 运行速度阈值。
 * 输入参数：alarm_value Page4 正转/反转速度报警阈值，单位与 WorkMessage.speed_work 一致，均为实际 rpm。
 * 返回参数：可与 WorkMessage.speed_work 直接比较的实际 rpm 阈值；0 表示阈值关闭。
 */
static uint16_t BuildSpeedAlarmThreshold(uint16_t alarm_value)
{
	return alarm_value; /* Page4 已按实际 rpm 保存，直接返回可避免 1 字节截断导致阈值偏低。 */
}

/*
 * 函数功能：检查当前运行速度或往复频率是否触发 Page4 阈值，触发时只驱动蜂鸣器，不强制停机。
 * 输入参数：无，函数内部读取 WorkMessage 和当前通道记忆结构。
 * 返回参数：无。
 */
void Pubinterface_CheckSpeedThresholdAlarm(void)
{
	ChannelMemoryMessage_t *memory = NULL; /* 指向当前工作通道的 Page4 阈值记忆，用于区分 A/B 手柄。 */
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

	/* 只有电机实际运行且当前通道记忆有效时才比较 Page4 阈值，停机或无通道时不触发提示蜂鸣。 */
	if ((WorkMessage.runflag_work == true) && (memory != NULL))
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			threshold_alarm = memory->speed_alarm_for; /* 正转使用 Page4 正转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= BuildSpeedAlarmThreshold(threshold_alarm)); /* 实际运行速度超过阈值时蜂鸣。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			threshold_alarm = memory->speed_alarm_rev; /* 反转使用 Page4 反转速度报警值。 */
			should_beep = (threshold_alarm != 0U) &&
						  (WorkMessage.speed_work >= BuildSpeedAlarmThreshold(threshold_alarm)); /* 实际运行速度超过阈值时蜂鸣。 */
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
static bool ConsumeExitDoubleClick(void)
{
	uint32_t now_tick = HAL_GetTick(); /* 读取 HAL 毫秒 tick，让双击窗口不依赖具体按键任务调度周期。 */

	/* 已有首次点击记录时才检查 1 秒确认窗口，超时点击会在函数末尾重新作为第一次记录。 */
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
	/* 屏幕退出键需要额外经过报警门禁和双击确认；上位机主动退出不经过这层人机交互限制。 */
	if (key_value == SCREENKey_HMI_EXIT)
	{
		if (WorkMessage.alarm_flag == true)
		{
			s_screen_external_exit_pending = 0U; /* 报警状态下屏幕外控退出键不生效，并清掉双击窗口，避免报警解除后沿用旧点击。 */
			return; /* EX8 表格要求屏幕外控退出必须 alarm_flag=false；上位机 HMIkey_HMI_EXIT 不受这个屏幕按键门控影响。 */
		}
		if (ConsumeExitDoubleClick() == false)
		{
			return; /* EX8 屏幕要求 1 秒内连续两次点击才退出外控，第一次点击只保留蜂鸣反馈。 */
		}
		ExternalComm_RequestExit(); /* 双击确认后交给通信任务真实释放 owner 并通知上位机停止申请保活。 */
	}
	else
	{
		ControlArbitration_ReleaseExternalControl(); /* 旧 HMI 退出入口保持原同步释放行为，不改变历史按键链路。 */
	}

	s_screen_external_exit_pending = 0U; /* 任意来源完成外控退出后清掉屏幕双击待确认，避免下一轮外控沿用旧点击。 */
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

	if ((WorkMessage.alarm_flag == true) &&
		((key_value != SCREENKey_TouchEXIT) || (WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED) == false)))
	{
		return; /* 报警态仍阻止启动、切换和保活；只有手柄掉线报警允许触控退出键作为用户确认入口。 */
	}
	switch (key_value)
	{
	case SCREENKey_JTActi:
	case HMIkey_JTActi:
	{
		if (WorkMessage.runflag_work == true) /* 电机运行期间禁止切换控制来源，避免脚控接管正在执行的其它控制。 */
		{
			return; /* 保持当前运行来源和通道记忆不变。 */
		}
		// 脚踏控制
		if (ControlSignalMessage.jt_enable_flag != true)
			return; /* EX8 表格要求脚踏在线后才允许切到脚控，未接脚踏时只保留按钮蜂鸣反馈。 */
		if (WorkMessage.drivetype_work == JTWORK || WorkMessage.touchactive_work == TOUCHWORK) /* 已在脚控或触控弹窗占用时不重复切换。 */
		{
			return; /* 避免重复点击重置控制状态或覆盖触控占用。 */
		}
		WorkMessage.drivetype_work = JTWORK;
		if (WorkMessage.channel_work == CHANNEL_A) /* 当前选中 A 时只更新 A 的控制方式记忆。 */
			MemoryMsgA.drive_type = JTWORK;
		else if (WorkMessage.channel_work == CHANNEL_B) /* 当前选中 B 时保持 A 记忆不变。 */
			MemoryMsgB.drive_type = JTWORK;
		Pubinterface_ClearFootControlManualLock(); /* 用户明确切回脚控后，取消本在线周期的手动锁存，脚控继续保持最高优先级。 */
		Pubinterface_RefreshControlModeDisplay(); /* 屏幕切到脚控后立刻刷新高亮，状态仍只写主工程 WorkMessage/MemoryMsg。 */
		// 注意界面更新
	}
	break;
	case SCREENKey_HandleActi:
	case HMIkey_HandleActi:
	{
		if (WorkMessage.runflag_work == true) /* 电机运行时禁止从脚控或触控切入手控，避免运行来源被中途替换。 */
		{
			return; /* 当前 owner 和运行参数继续由原控制源维护。 */
		}
		// 手动控制
		if (Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model) == false)
			return; /* 只有PXBA/LGZI/LGZII等确认具备实体按键的手柄允许切入手控，PXBB必须拒绝该入口。 */
		if (WorkMessage.drivetype_work == HANDLEWORK || WorkMessage.touchactive_work == TOUCHWORK) /* 已经是手控或触控界面仍占用时不重复写状态。 */
		{
			return; /* 防止重复点击覆盖触控退出前的控制权释放流程。 */
		}
		WorkMessage.drivetype_work = HANDLEWORK;
		if (WorkMessage.channel_work == CHANNEL_A) /* 当前选中 A 时把手控方式保存到 A 通道记忆。 */
			MemoryMsgA.drive_type = HANDLEWORK;
		else if (WorkMessage.channel_work == CHANNEL_B) /* 当前选中 B 时只更新 B 通道记忆。 */
			MemoryMsgB.drive_type = HANDLEWORK;
		MarkFootManualLock(); /* 脚踏在线时用户手动选手控，本次在线周期内不再被脚踏自动切回脚控。 */
		Pubinterface_RefreshControlModeDisplay(); /* 屏幕切到手控后立刻刷新高亮，不引入副工程旧全局状态。 */
		// 界面调整更新
	}
	break;
	case SCREENKey_TouchActi:
	{
		if (WorkMessage.runflag_work == true) /* 其它来源仍在运行时不能直接打开触控工作面。 */
		{
			return; /* 保持当前运行和 owner，不让屏幕入口抢占。 */
		}
		if (WorkMessage.hand_model == 0U)
			return; /* EX8 表格要求触控入口必须已有有效手柄，未识别手柄时只保留屏幕蜂鸣反馈。 */
		if (ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN) == false) /* 先申请屏幕 owner，避免和脚踏、手控或外控并发。 */
		{
			return; /* 申请失败时不改变触控显示、驱动方式和通道记忆。 */
		}
		Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 用户重新进入触控运行入口，视为新的控制源触发，允许重新尝试压力闭环启动。 */
		/* 新屏主运行页的触控入口只负责进入触控工作面，不直接起机。 */
		WorkMessage.touchactive_work = TOUCHWORK; /* 标记本机触控占用，阻止脚踏/手柄在运行中抢占。 */
		WorkMessage.drivetype_work = TOUCHWORK; /* 控制方式同步切到触控，控制图标立即由白色变黄色。 */
		if (WorkMessage.channel_work == CHANNEL_A) /* A 通道进入触控时同步 A 的控制方式记忆。 */
			MemoryMsgA.drive_type = TOUCHWORK; /* A 通道记忆当前触控来源，后续切回 A 时保持显示一致。 */
		else if (WorkMessage.channel_work == CHANNEL_B) /* B 通道进入触控时只更新 B 记忆，防止 A/B 串状态。 */
			MemoryMsgB.drive_type = TOUCHWORK; /* B 通道记忆当前触控来源，避免切通道后又显示为手控。 */
		MarkFootManualLock(); /* 脚踏在线时用户手动进入触控，本次在线周期保持触控选择直到脚踏重新插入。 */
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
		if (WorkMessage.runflag_work == false)
		{
			data[0] = 0U; /* 压力堵塞锁存拒绝本次触控保活后，触控弹窗必须显示停机态，避免显示运行但实际停机。 */
			SendUIDSMessage(UI_TOUCH_ID, true, data); /* 保持触控弹窗可见，但运行状态回到白色待运行。 */
			Pubinterface_RefreshControlModeDisplay(); /* 同步主运行页触控图标，避免保活被拒绝后仍显示黄色运行。 */
			break;
		}
		data[0] = 1U; /* 触控正在运行，屏幕触控图标显示黄色运行态。 */
		SendUIDSMessage(UI_TOUCH_ID, true, data); /* 保活只刷新运行态，不改变触控界面显隐。 */
		Pubinterface_RefreshControlModeDisplay(); /* 同步主运行页触控高亮，避免其它刷新把触控误刷白。 */
		break;
	case SCREENKey_TouchEXIT: // 停止
		if ((WorkMessage.touchactive_work != TOUCHWORK) &&
			(WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED) == false))
		{
			return; /* 非触控状态且不是运行手柄掉线报警时不处理，避免误清其它控制来源。 */
		}
		/* 屏幕触控退出时先停电机，再清触控占用，控制权释放要继续等待驱动反馈归零。 */
		Pubinterface_SetHandleInjectionPumpRun(false); /* 触控退出属于用户确认停机，必须再次关闭手柄联动注水泵，防止故障恢复后泵状态残留。 */
		WorkMessage.runflag_work = false;

		WorkMessage.speed_work = 0U;
		WorkMessage.touchactive_work = 0U;
		WorkMessage.drivetype_work = ControlArbitration_GetLocalDriveTypeAfterExit(); /* 触控退出后恢复本机可接管控制方式，避免图标继续停在黄色触控。 */
		(void)Pubinterface_ClearHandleNotConnectedAlarm(); /* 运行中拔手柄后的触控退出键视为用户确认故障，允许关闭报警弹窗并恢复后续操作。 */
		SendUIDSMessage(UI_TOUCH_ID, false, data); /* 触控退出时立即隐藏弹窗，控制权释放由驱动停稳后完成。 */
		/* 这里不能直接 Exit，否则刚下发停止但电机尚未停稳时其它来源会提前接管。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_SCREEN);
		Pubinterface_RefreshControlModeDisplay(); /* 停止触控后同步手控/脚控/触控白黄状态，避免残留高亮。 */
		// 退出触控界面
		break;
	}
}

/*
 * 函数功能：处理速度增减键，慢档和快档分别使用当前通道识别缓存中的小步进和大步进；公共接头步进按 RFID 转速范围分档生成。
 * 输入参数：key_value 为手柄、HMI 或新屏速度键值。
 * 返回参数：无。
 */
void SpeedActive(uint8_t key_value)
{
	uint32_t speed_value = WorkMessage.speed_set_work; /* 以当前设定速度为基准，Page4 24位最大速度可能超过16位，调速过程必须保留32位。 */
	uint32_t speed_step = 0U;                          /* 本次实际步进，普通键和小步进键使用当前识别结果的小步进。 */
	uint32_t speed_large_step = 0U;                    /* 屏幕大加/大减使用当前识别结果的大步进，不再由小步进乘 2 推导。 */
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

	/* A 通道调速只读取 A 识别参数，保证步进和上下限与当前 A 手柄一致。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_zzstep; /* A 通道正转旧键步进来自 EEPROM/识别参数。 */
			speed_large_step = ChannelrecognizeMessageA.speed_zzstep_large; /* A 通道正转大步进来自当前识别结果。 */
			speed_max = ChannelrecognizeMessageA.speed_zzmax;   /* A 通道正转最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_zzmin;   /* A 通道正转最小速度。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_fzstep; /* A 通道反转旧键步进来自 EEPROM/识别参数。 */
			speed_large_step = ChannelrecognizeMessageA.speed_fzstep_large; /* A 通道反转大步进来自当前识别结果。 */
			speed_max = ChannelrecognizeMessageA.speed_fzmax;   /* A 通道反转最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_fzmin;   /* A 通道反转最小速度。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageA.speed_oscstep; /* A 通道往复旧键步进来自 EEPROM/识别参数。 */
			speed_large_step = ChannelrecognizeMessageA.speed_oscstep_large; /* A 通道往复大步进来自当前识别结果。 */
			speed_max = ChannelrecognizeMessageA.speed_oscmax;   /* A 通道往复最大速度。 */
			speed_min = ChannelrecognizeMessageA.speed_oscmin;   /* A 通道往复最小速度。 */
		}
		else
		{
			return; /* 未知方向不写速度，避免把无效方向下的参数覆盖到通道记忆。 */
		}
	}
	/* B 通道调速改用 B 识别参数，避免沿用 A 通道的速度范围。 */
	else if (WorkMessage.channel_work == CHANNEL_B)
	{
		if (WorkMessage.dir_work == ZZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_zzstep; /* B 通道正转旧键步进来自 B 通道识别参数。 */
			speed_large_step = ChannelrecognizeMessageB.speed_zzstep_large; /* B 通道正转大步进来自当前识别结果。 */
			speed_max = ChannelrecognizeMessageB.speed_zzmax;   /* B 通道正转最大速度，避免误用 A 通道限幅。 */
			speed_min = ChannelrecognizeMessageB.speed_zzmin;   /* B 通道正转最小速度，避免误用 A 通道限幅。 */
		}
		else if (WorkMessage.dir_work == FZDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_fzstep; /* B 通道反转旧键步进来自 B 通道识别参数。 */
			speed_large_step = ChannelrecognizeMessageB.speed_fzstep_large; /* B 通道反转大步进来自当前识别结果。 */
			speed_max = ChannelrecognizeMessageB.speed_fzmax;   /* B 通道反转最大速度。 */
			speed_min = ChannelrecognizeMessageB.speed_fzmin;   /* B 通道反转最小速度。 */
		}
		else if (WorkMessage.dir_work == OSCDIR)
		{
			speed_step = ChannelrecognizeMessageB.speed_oscstep; /* B 通道往复旧键步进来自 B 通道识别参数。 */
			speed_large_step = ChannelrecognizeMessageB.speed_oscstep_large; /* B 通道往复大步进来自当前识别结果。 */
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
	if (speed_large_step == 0U)
	{
		speed_large_step = SCREEN_SPEED_LARGE_STEP_FALLBACK; /* 当前识别结果的大步进无效时单独兜底，不再复用小步进乘法。 */
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
		speed_step = speed_large_step; /* 新屏大加直接使用当前识别结果的大步进。 */
		add_key = true; /* 本次按键方向为增加。 */
		break;
	case SCREENKey_SPEED_Sub_Small:
		sub_key = true; /* 新屏慢减直接使用当前方向寄存器步进。 */
		break;
	case SCREENKey_SPEED_Sub_Large:
		speed_step = speed_large_step; /* 新屏大减直接使用当前识别结果的大步进。 */
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

	/* 增速键按当前步进向上调整并在最大速度处钳位，防止目标速度越过手柄能力。 */
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
	/* 减速键按当前步进向下调整并在最小速度处钳位，防止无符号计算下溢。 */
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
	/* 当前为 A 通道时只保存 A 的方向速度记忆，切换到 B 后仍可恢复 B 原有设定。 */
	if (WorkMessage.channel_work == CHANNEL_A)
	{
		if (WorkMessage.dir_work == ZZDIR)
			MemoryMsgA.zz_speed = speed_value; /* A 通道正转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == FZDIR)
			MemoryMsgA.fz_speed = speed_value; /* A 通道反转速度记忆同步更新。 */
		else if (WorkMessage.dir_work == OSCDIR)
			MemoryMsgA.osc_speed = speed_value; /* A 通道往复速度记忆同步更新。 */
	}
	/* 当前为 B 通道时只保存 B 的方向速度记忆，避免本次调速覆盖 A 通道参数。 */
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
		(IsOscDirectionAllowed(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == false))
	{
		return; /* 只有已识别且支持往复的当前手柄才允许调频，避免无手柄或普通手柄进入无效频率窗口。 */
	}
	if (((key_value == SCREENKey_FREQ_Add) || (key_value == SCREENKey_FREQ_Sub)) &&
		(WorkMessage.dir_work != OSCDIR)) /* 新屏只有在往复方向下才显示频率区域，非往复时该坐标属于隐藏区域。 */
	{
		return; /* 隐藏频率坐标不得修改保留频率；HMI 原调参入口保持不变。 */
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
	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true) /* 运行中切方向会造成驱动突变，报警中也必须冻结方向设置。 */
	{
		return; /* 保留当前方向和对应速度记忆，等待停机且报警解除后再切换。 */
	}
	if (WorkMessage.hand_model == 0U)
		return; /* EX8 表格要求方向键必须有有效手柄才生效，避免无手柄残留通道状态被方向键改写。 */
	if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
		return; /* 未选中 A/B 通道时没有可保存的方向记忆，直接拒绝方向键，避免屏幕显示被误刷新。 */
	if (Pubinterface_IsDirLocked(WorkMessage.hand_model, WorkMessage.raw_tool_type))
	{
		return; /* 固定方向手柄只服从上线时确定的方向，实体键、屏幕和内部 HMI 消息都不能改写方向记忆。 */
	}
	switch (key_value) /* 按目标方向同时装载该方向的记忆速度，避免沿用上一方向速度。 */
	{
	case HANDLEKey_dir_Forward:
	case HMIkey_Dir_Forward:
	case SCREENKey_Dir_Forward:
		if (WorkMessage.channel_work == CHANNEL_A) /* A 切正转时装载 A 正转速度边界内保存的设定值。 */
		{
			WorkMessage.dir_work = MemoryMsgA.dir = ZZDIR; /* 当前方向和 A 通道记忆同步，后续切回 A 仍保持正转。 */
			WorkMessage.speed_set_work = MemoryMsgA.zz_speed; /* 正转使用独立速度记忆，不继承反转或往复速度。 */
		}
		else if (WorkMessage.channel_work == CHANNEL_B) /* B 切正转时只读取 B 正转速度。 */
		{
			WorkMessage.dir_work = MemoryMsgB.dir = ZZDIR; /* 同步 B 通道方向记忆。 */
			WorkMessage.speed_set_work = MemoryMsgB.zz_speed; /* 装载 B 正转速度，避免串用 A 通道参数。 */
		}
		// 正向
		break;
	case HANDLEKey_dir_Reverse:
	case HMIkey_Dir_Reverse:
	case SCREENKey_Dir_Reverse:
		if (WorkMessage.channel_work == CHANNEL_A) /* A 切反转时装载 A 反转专用速度。 */
		{
			WorkMessage.dir_work = MemoryMsgA.dir = FZDIR; /* 当前方向和 A 记忆同步为反转。 */
			WorkMessage.speed_set_work = MemoryMsgA.fz_speed; /* 反转不沿用正转速度，避免超过反转允许范围。 */
		}
		else if (WorkMessage.channel_work == CHANNEL_B) /* B 反转只读取 B 通道反转记忆。 */
		{
			WorkMessage.dir_work = MemoryMsgB.dir = FZDIR; /* 同步 B 通道方向记忆为反转。 */
			WorkMessage.speed_set_work = MemoryMsgB.fz_speed; /* 装载 B 反转速度，保持通道隔离。 */
		}
		// 反向
		break;
	case HANDLEKey_dir_OSC:
	case HMIkey_Dir_OSC:
	case SCREENKey_Dir_OSC:
		if (IsOscDirectionAllowed(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == false)
		{
			return; /* 当前选中手柄不支持往复时拒绝 OSCDIR，避免屏幕误发往复键后切入驱动不支持模式。 */
		}
		if (WorkMessage.channel_work == CHANNEL_A) /* A 切往复时装载 A 往复速度。 */
		{
			WorkMessage.dir_work = MemoryMsgA.dir = OSCDIR; /* 当前方向和 A 记忆同步为往复。 */
			WorkMessage.speed_set_work = MemoryMsgA.osc_speed; /* 往复使用独立速度，不能沿用旋转方向速度。 */
		}
		else if (WorkMessage.channel_work == CHANNEL_B) /* B 往复只读取 B 通道往复速度。 */
		{
			WorkMessage.dir_work = MemoryMsgB.dir = OSCDIR; /* 同步 B 通道方向记忆为往复。 */
			WorkMessage.speed_set_work = MemoryMsgB.osc_speed; /* 装载 B 往复速度，防止 A/B 参数串用。 */
		}
		// 往复
		break;
	}
	if ((WorkMessage.channel_work == CHANNEL_A) || (WorkMessage.channel_work == CHANNEL_B))
	{
		Pubinterface_RefreshSelectedChannelDisplay(WorkMessage.channel_work); /* 方向切换后同步刷新方向高亮、频率窗口和当前方向速度值。 */
	}
}
