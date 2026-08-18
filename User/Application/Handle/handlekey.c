//handlekey.c

#include "bsp_gpio.h"
#include "handlekey.h"
#include "data.h"
//#include "adc.h"
#include "soft_uart.h"
#include "stm32f4xx_hal.h"

#include "kernel_scheduler.h"
#include "datahand.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscKEYBH.h"
#include "sscUIDP.h"

/*
 * 文件功能：每 30ms 按手柄型号切换 A/B 按键脚的GPIO/串口模式，并处理普通手柄实体运行键。
 * 运行入口：Userparser_Init() 调用 HandleKeyScan_Init()，任务回调只进入 HANDLEKEYTaskFunc()。
 * 关键顺序：先维护 82 号限时提示，再检查控制权，最后严格按 A 后 B 顺序处理实体键。
 * 安全约束：启动必须依次通过公共接头刀具门禁、控制权申请，再写运行状态并联动注水泵。
 */
kernel_task_t HANDLEKEYTaskHandle;

/*
 * 函数功能：把 GPIO 外部中断交给压力软串口边沿处理。
 * 输入参数：GPIO_Pin 为本次触发中断的 GPIO 引脚编号。
 * 返回参数：无。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SimUart_HandleExti(GPIO_Pin); /* 压力软串口依赖 GPIO 边沿收数，任何手柄键整理都不能跳过该转发。 */
}

#define HANDLE_KEY_DEBOUNCE_COUNT 2U /* 30ms任务连续2次确认电平，约60ms去抖，避免触点抖动误启停。 */
#define HANDLE_KEY_PRESSED_LEVEL GPIO_PIN_RESET /* 硬件默认上拉，按键按下后对应IO被拉低。 */

typedef struct
{
	uint8_t low_count;		/* 连续低电平计数，用于确认按键已经稳定按下。 */
	uint8_t high_count;		/* 连续高电平计数，用于确认按键已经稳定松开。 */
	bool stable_pressed;	/* 去抖后的按下状态，业务层只使用这个稳定结果。 */
	bool press_event;		/* 稳定按下沿事件，只在松开后再次按下并完成消抖时置位一个扫描周期。 */
	bool release_event;		/* 稳定松开沿事件，LGZI 按住运行模式依靠它在松手时立即停机。 */
} HandleRunKeyDebounce_t;

typedef enum
{
	HANDLE_RUN_KEY_PIN_MODE_UNKNOWN = 0U, /* 上电后尚未由任务确认复用状态，首次扫描必须主动配置。 */
	HANDLE_RUN_KEY_PIN_MODE_GPIO,         /* 普通手柄模式：引脚上拉输入，低电平表示按钮按下。 */
	HANDLE_RUN_KEY_PIN_MODE_UART          /* KSZ模式：引脚切换为对应UART接收复用，不再读取GPIO电平。 */
} HandleRunKeyPinMode_t;

static HandleRunKeyDebounce_t s_handle_run_key_a_filter = {0U, 0U, false, false, false}; /* A通道实体键去抖状态。 */
static HandleRunKeyDebounce_t s_handle_run_key_b_filter = {0U, 0U, false, false, false}; /* B通道实体键去抖状态。 */
static HandleRunKeyPinMode_t s_handle_run_key_a_pin_mode = HANDLE_RUN_KEY_PIN_MODE_UNKNOWN; /* A通道PE2当前复用状态。 */
static HandleRunKeyPinMode_t s_handle_run_key_b_pin_mode = HANDLE_RUN_KEY_PIN_MODE_UNKNOWN; /* B通道PE0当前复用状态。 */
static uint8_t s_handle_run_key_owner_channel = CHANNEL_NONE;			   /* 当前由实体键启动的通道，防止另一通道松开误停。 */
static uint8_t s_handle_foot_selected_timed_alarm_active = 0U; /* 记录 82 号错模式弹窗是否由手柄键模块显示，防止到期误清其它报警。 */
static uint32_t s_handle_foot_selected_timed_alarm_tick = 0U; /* 记录 82 号错模式弹窗开始时间，按 ALARM_MODE_MS（当前 2000ms）自动清除。 */

/*
 * 函数功能：脚控已选中时按手柄实体运行键，上报 82 号限时提示，保持时间由 ALARM_MODE_MS 控制（当前 2000ms）。
 * 输入参数：无，报警码固定使用 WORK_ALARM_FOOT_SELECTED。
 * 返回参数：无。
 */
static void HandleRunKey_ShowFootAlarm(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只读取 Value[0]，其余补零避免沿用上一条报警参数。 */

	display_value[0] = WORK_ALARM_FOOT_SELECTED; /* 82 号图提示当前脚控已选中，用户应使用脚踏启动。 */
	SendAlarmMessageTimed(WORK_ALARM_FOOT_SELECTED, ALARM_MODE_MS); /* 错模式提示只保持 ALARM_MODE_MS（当前 2000ms），不写全局报警锁存。 */
	s_handle_foot_selected_timed_alarm_tick = HAL_GetTick(); /* 记录本次弹窗起点，后续由手柄键任务周期自动关闭。 */
	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 没有真实报警占用报警区时显示 82 号临时弹窗。 */
		s_handle_foot_selected_timed_alarm_active = 1U; /* 只有实际显示了 82 号图，本模块才拥有后续关闭权。 */
	}
	else
	{
		s_handle_foot_selected_timed_alarm_active = 0U; /* 真实报警优先级更高，本次只蜂鸣不抢屏幕报警区。 */
	}
}

/*
 * 函数功能：周期维护 82 号错模式限时弹窗，达到 ALARM_MODE_MS（当前 2000ms）后自动关闭。
 * 输入参数：无，直接读取弹窗归属和 HAL 毫秒 tick。
 * 返回参数：无。
 */
static void HandleRunKey_ServiceFootAlarm(void)
{
	if (s_handle_foot_selected_timed_alarm_active == 0U)
	{
		return; /* 当前没有由手柄键模块显示的 82 号弹窗，不处理屏幕报警区。 */
	}

	if ((uint32_t)(HAL_GetTick() - s_handle_foot_selected_timed_alarm_tick) < ALARM_MODE_MS)
	{
		return; /* ALARM_MODE_MS（当前 2000ms）尚未到期，继续显示脚控已选中提示。 */
	}

	if (WorkMessage.alarm_flag == false)
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 没有真实报警时关闭 82 号临时弹窗。 */
	}
	s_handle_foot_selected_timed_alarm_active = 0U; /* 本次临时弹窗生命周期结束，允许下一次误按重新显示。 */
	s_handle_foot_selected_timed_alarm_tick = 0U; /* 清时间戳，避免下次比较沿用旧 tick。 */
}

/*
 * 函数功能：判断手柄型号是否支持本次实体按键启停。
 * 输入参数：hand_model 通道记忆中的手柄型号。
 * 返回参数：true 表示该手柄允许实体按键启停，false 表示忽略该通道按键。
 */
static bool HandleRunKey_IsSupportedModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||   /* 空心钻A型手柄保持原有按一次启动、再按一次停止。 */
			(hand_model == LGZ_I_ONLINES));   /* LGZI 单按键颅骨钻允许实体键控制电机，但停止由松开沿触发。 */
}

/*
 * 函数功能：判断手柄实体键是否采用按住运行、松开停止模式。
 * 输入参数：hand_model 通道记忆中的手柄型号。
 * 返回参数：true 表示松开必须停机，false 表示保持按键翻转启停。
 */
static bool HandleRunKey_IsMomentaryModel(uint8_t hand_model)
{
	return (hand_model == LGZ_I_ONLINES); /* LGZI 只有单个手控按钮，现场要求按住才运行、松手立即停止。 */
}

/*
 * 函数功能：读取指定通道当前识别出的手柄型号。
 * 输入参数：channel 目标通道，CHANNEL_A 表示A通道，CHANNEL_B 表示B通道。
 * 返回参数：通道 MemoryMsg 内的 hand_model，无效通道返回0。
 */
static uint8_t HandleRunKey_GetChannelModel(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return MemoryMsgA.hand_model; /* A实体键的控制策略必须跟随A通道识别出的真实手柄型号。 */
	}
	if (channel == CHANNEL_B)
	{
		return MemoryMsgB.hand_model; /* B实体键的控制策略必须跟随B通道识别出的真实手柄型号。 */
	}
	return 0U; /* 无效通道不具备手柄型号，后续判断会按不支持处理。 */
}

/*
 * 函数功能：判断手柄是否带内部MCU并要求使用硬件串口接收按钮数据。
 * 输入参数：hand_model 为当前通道已经识别出的手柄型号。
 * 返回参数：true 表示KSZ_I/KSZ_II串口手柄，false 表示普通GPIO按钮手柄。
 */
static bool HandleRunKey_IsUartModel(uint8_t hand_model)
{
	return ((hand_model == KSZ_I_ONLINES) ||
			(hand_model == KSZ_II_ONLINES)); /* 仅两类KSZ手柄切换串口，离线值和其它型号全部保持普通GPIO模式。 */
}

/*
 * 函数功能：清空单通道GPIO按钮去抖状态，防止复用切换前的电平被带入新模式。
 * 输入参数：filter 为待复位通道的按钮去抖对象。
 * 返回参数：无。
 */
static void HandleRunKey_ResetDebounce(HandleRunKeyDebounce_t *filter)
{
	filter->low_count = 0U;         /* 清除低电平累计，切回GPIO后必须重新连续确认按下。 */
	filter->high_count = 0U;        /* 清除高电平累计，避免串口空闲高电平被当成历史松开计数。 */
	filter->stable_pressed = false; /* 复用模式改变后不继承此前已经按下的稳定状态。 */
	filter->press_event = false;    /* 清除一次性按下沿，禁止切换当周期误启动。 */
	filter->release_event = false;  /* 清除一次性松开沿，禁止切换当周期误停止。 */
}

/*
 * 函数功能：按通道把A键PE2或B键PE0切换为上拉GPIO输入或对应UART接收复用。
 * 输入参数：channel 为CHANNEL_A/CHANNEL_B；target_mode 为目标GPIO或UART模式；filter和current_mode属于同一通道。
 * 返回参数：无；无效通道或无效目标模式不改硬件。
 */
static void HandleRunKey_ApplyPinMode(uint8_t channel,
									  HandleRunKeyPinMode_t target_mode,
									  HandleRunKeyDebounce_t *filter,
									  HandleRunKeyPinMode_t *current_mode)
{
	GPIO_InitTypeDef gpio_init = {0}; /* 每次完整填写引脚模式，避免沿用其它GPIO初始化参数。 */
	GPIO_TypeDef *gpio_port = NULL;   /* 运行时按逻辑通道选择固定端口，A/B当前都位于GPIOE。 */
	uint16_t gpio_pin = 0U;           /* 保存A通道PE2或B通道PE0的引脚位掩码。 */
	uint32_t uart_alternate = 0U;     /* 保存A通道UART10或B通道UART8的接收复用编号。 */

	if ((*current_mode == target_mode) ||
		((target_mode != HANDLE_RUN_KEY_PIN_MODE_GPIO) &&
		 (target_mode != HANDLE_RUN_KEY_PIN_MODE_UART)))
	{
		return; /* 模式未改变时不重复改寄存器；目标非法时保持当前安全配置。 */
	}

	if (channel == CHANNEL_A)
	{
		gpio_port = BOARD_RES_HANDLE_RUN_KEY_A_PORT;       /* A按钮固定使用PE2。 */
		gpio_pin = BOARD_RES_HANDLE_RUN_KEY_A_PIN;         /* 选中PE2引脚位。 */
		uart_alternate = BOARD_RES_HANDLE_RUN_KEY_A_UART_AF; /* KSZ手柄下PE2切为UART10_RX。 */
	}
	else if (channel == CHANNEL_B)
	{
		gpio_port = BOARD_RES_HANDLE_RUN_KEY_B_PORT;       /* B按钮固定使用PE0。 */
		gpio_pin = BOARD_RES_HANDLE_RUN_KEY_B_PIN;         /* 选中PE0引脚位。 */
		uart_alternate = BOARD_RES_HANDLE_RUN_KEY_B_UART_AF; /* KSZ手柄下PE0切为UART8_RX。 */
	}
	else
	{
		return; /* 非A/B通道没有复用引脚，禁止写未知GPIO寄存器。 */
	}

	gpio_init.Pin = gpio_pin; /* 本次只修改目标通道RX/按钮复用脚，不影响同端口其它引脚。 */
	if (target_mode == HANDLE_RUN_KEY_PIN_MODE_UART)
	{
		gpio_init.Mode = GPIO_MODE_AF_PP;             /* KSZ内部MCU输出串口数据，RX脚恢复硬件复用输入路径。 */
		gpio_init.Pull = GPIO_NOPULL;                  /* 串口模式沿用CubeMX现有无上下拉配置。 */
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;   /* 串口复用沿用UART MSP初始化速度。 */
		gpio_init.Alternate = uart_alternate;          /* A绑定UART10_RX，B绑定UART8_RX，禁止交叉。 */
	}
	else
	{
		gpio_init.Mode = GPIO_MODE_INPUT;              /* 非KSZ手柄使用普通数字输入读取实体按钮。 */
		gpio_init.Pull = GPIO_PULLUP;                  /* 按钮仍为低电平有效，松开时由内部上拉保持高电平。 */
		gpio_init.Speed = GPIO_SPEED_FREQ_LOW;         /* 普通按钮无需高速翻转，保持低速GPIO配置。 */
		gpio_init.Alternate = 0U;                      /* GPIO输入模式不使用复用编号，显式清零便于审查。 */
	}

	HAL_GPIO_Init(gpio_port, &gpio_init); /* 只在型号/在线状态导致模式变化时执行一次硬件切换。 */
	HandleRunKey_ResetDebounce(filter);    /* 切换后重新建立稳定电平，避免UART数据边沿误触发按钮。 */
	*current_mode = target_mode;           /* 硬件配置完成后再记录状态，下一周期无需重复初始化。 */
}

/*
 * 函数功能：根据A/B通道在线型号同步PE2/PE0的GPIO或UART复用模式。
 * 输入参数：无，直接读取通道在线标志和各自MemoryMsg手柄型号。
 * 返回参数：无。
 */
static void HandleRunKey_UpdatePinModes(void)
{
	HandleRunKeyPinMode_t a_target_mode = HANDLE_RUN_KEY_PIN_MODE_GPIO; /* A离线或普通手柄默认使用PE2上拉输入。 */
	HandleRunKeyPinMode_t b_target_mode = HANDLE_RUN_KEY_PIN_MODE_GPIO; /* B离线或普通手柄默认使用PE0上拉输入。 */

	if ((WorkMessage.Channel_Aonline == true) &&
		(HandleRunKey_IsUartModel(MemoryMsgA.hand_model) == true))
	{
		a_target_mode = HANDLE_RUN_KEY_PIN_MODE_UART; /* A识别到KSZ后把PE2切换为UART10_RX。 */
	}
	if ((WorkMessage.Channel_Bonline == true) &&
		(HandleRunKey_IsUartModel(MemoryMsgB.hand_model) == true))
	{
		b_target_mode = HANDLE_RUN_KEY_PIN_MODE_UART; /* B识别到KSZ后把PE0切换为UART8_RX。 */
	}

	HandleRunKey_ApplyPinMode(CHANNEL_A, a_target_mode, &s_handle_run_key_a_filter, &s_handle_run_key_a_pin_mode); /* 独立更新A，B型号变化不影响PE2。 */
	HandleRunKey_ApplyPinMode(CHANNEL_B, b_target_mode, &s_handle_run_key_b_filter, &s_handle_run_key_b_pin_mode); /* 独立更新B，A型号变化不影响PE0。 */
}

/*
 * 函数功能：判断指定通道是否已经在线且识别通过，可被实体按键启动。
 * 输入参数：channel 目标通道，CHANNEL_A 表示A通道，CHANNEL_B 表示B通道。
 * 返回参数：true 表示通道在线且手柄型号支持实体键，false 表示本次按键无效。
 */
static bool HandleRunKey_IsChannelReady(uint8_t channel)
{
	if (channel == CHANNEL_A)
	{
		return ((WorkMessage.Channel_Aonline == true) &&
				(HandleRunKey_IsSupportedModel(MemoryMsgA.hand_model) == true)); /* A通道必须校验通过并写入MemoryMsgA后才允许启动。 */
	}

	if (channel == CHANNEL_B)
	{
		return ((WorkMessage.Channel_Bonline == true) &&
				(HandleRunKey_IsSupportedModel(MemoryMsgB.hand_model) == true)); /* B通道必须校验通过并写入MemoryMsgB后才允许启动。 */
	}

	return false; /* 无效通道不能参与实体键控制，避免误写WorkMessage。 */
}

/*
 * 函数功能：把当前选中通道的控制方式切到手控，并同步通道记忆。
 * 输入参数：channel 当前即将由实体按键启动的通道。
 * 返回参数：无。
 */
static void HandleRunKey_ApplyHandleMode(uint8_t channel)
{
	WorkMessage.drivetype_work = HANDLEWORK; /* 实体按键属于手控来源，启动前必须把当前工作快照切到手控。 */

	if (channel == CHANNEL_A)
	{
		MemoryMsgA.drive_type = HANDLEWORK; /* A通道被实体键启动时，A通道记忆同步为手控，后续切回A仍保持一致。 */
	}
	else if (channel == CHANNEL_B)
	{
		MemoryMsgB.drive_type = HANDLEWORK; /* B通道被实体键启动时，B通道记忆同步为手控，后续切回B仍保持一致。 */
	}
	else
	{
		/* 无效通道不写通道记忆，上层启动条件已经阻止这种情况。 */
	}
}

/*
 * 函数功能：对实体按键原始电平做去抖，输出稳定按下沿并记录稳定松开沿。
 * 输入参数：filter 对应通道的去抖状态；raw_level 本周期GPIO原始电平。
 * 返回参数：true 表示本周期产生稳定按下沿事件，false 表示本周期无新按下事件。
 */
static bool HandleRunKey_DebouncePressEvent(HandleRunKeyDebounce_t *filter, GPIO_PinState raw_level)
{
	filter->press_event = false; /* 每个30ms扫描周期先清一次性事件，防止长按期间重复翻转启停。 */
	filter->release_event = false; /* 松开沿同样只允许保持一个扫描周期，避免LGZI松手后重复发送停止。 */
	if (raw_level == HANDLE_KEY_PRESSED_LEVEL)
	{
		filter->high_count = 0U; /* 本周期为低电平，清掉松开计数，避免抖动期间提前判松开。 */
		if (filter->low_count < HANDLE_KEY_DEBOUNCE_COUNT)
		{
			filter->low_count++; /* 低电平计数未达到阈值时继续累加，等待电平稳定。 */
		}
		if (filter->low_count >= HANDLE_KEY_DEBOUNCE_COUNT)
		{
			if (filter->stable_pressed == false)
			{
				filter->press_event = true; /* 只有从稳定松开进入稳定按下时才产生一次翻转事件。 */
			}
			filter->stable_pressed = true; /* 连续低电平达到阈值后，锁定为已按下，直到稳定松开才重新允许下一次事件。 */
		}
	}
	else
	{
		filter->low_count = 0U; /* 本周期为高电平，清掉按下计数，避免松开抖动误判仍按住。 */
		if (filter->high_count < HANDLE_KEY_DEBOUNCE_COUNT)
		{
			filter->high_count++; /* 高电平计数未达到阈值时继续累加，等待电平稳定。 */
		}
		if (filter->high_count >= HANDLE_KEY_DEBOUNCE_COUNT)
		{
			if (filter->stable_pressed == true)
			{
				filter->release_event = true; /* 从稳定按下进入稳定松开时输出一次松开沿，供LGZI按住运行模式停机。 */
			}
			filter->stable_pressed = false; /* 连续高电平达到阈值后复位按下状态，允许下一次按下重新触发。 */
		}
	}

	return filter->press_event; /* 返回稳定按下沿事件；松开沿保存在filter->release_event中供按住运行型号使用。 */
}

/*
 * 函数功能：通过手柄实体按键设置电机运行状态，并维护控制仲裁。
 * 输入参数：enable true 表示启动电机，false 表示停止电机。
 * 返回参数：true 表示本次状态写入成功，false 表示启动时未抢到手柄控制权。
 */
static bool HandleRunKey_SetMotorRun(bool enable)
{
	/* enable 为 true 时执行完整启动门禁；停止请求无需再次检查刀具和控制权。 */
	if (enable)
	{
		if (Pubinterface_CheckCommonSocketToolReadyForRun() == false)
		{
			return false; /* 公共接头缺 EPC 刀具头时拒绝实体运行键，避免没有刀具参数仍启动电机。 */
		}
		if (ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE) == false)
		{
			return false; /* 其它控制来源正在占用时不抢占，实体键本周期启动无效。 */
		}
		Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 实体键再次按下属于新的控制源触发，允许泵任务重新按实时压力判断。 */
	}

	ControlSignalMessage.handle_control_flag = enable; /* 通知公共控制信号当前由手柄实体键控制电机启停。 */
	WorkMessage.runflag_work = enable;				 /* sscDrive任务读取该标志后下发无刷运行或停止控制帧。 */
	Pubinterface_SetHandleInjectionPumpRun(enable);	 /* 注水泵是手柄冷却联动泵，实体键让手柄运行/停止时按泵类型和当前通道同步启停 A/B 冷却泵。 */

	if (enable == false)
	{
		WorkMessage.speed_work = 0U; /* 停止时立即清实际目标速度，防止停止帧前残留上一次速度。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE); /* 电机停稳后释放手柄控制权，允许脚踏/屏幕/上位机接管。 */
	}

	SendKeyBehMessage(HANDLEKey, enable ? HANDLEKey_motor_start : HANDLEKey_motor_stop); /* 复用原按键行为消息，保持UI和上位机状态同步链路。 */
	return true; /* 状态写入完成，调用方可以更新实体键owner通道。 */
}

/*
 * 函数功能：仅在实体键所属通道已经是当前选中通道时准备默认速度。
 * 输入参数：channel 本次按下的实体键对应通道。
 * 返回参数：true 表示启动前参数准备完成，false 表示通道不可启动或不是当前选中通道。
 */
static bool HandleRunKey_PrepareRunChannel(uint8_t channel)
{
	if (HandleRunKey_IsChannelReady(channel) == false)
	{
		return false; /* 通道未在线或校验未通过时不启动，避免坏手柄或空通道误转。 */
	}

	if (WorkMessage.channel_work != channel)
	{
		return false; /* 实体键只允许控制当前选中通道，非选中通道误触不再抢占 A/B 选择。 */
	}

	HandleRunKey_ApplyHandleMode(channel); /* 实体键启动统一视为手控，覆盖空闲时残留的脚控/触控模式记忆。 */

	if (WorkMessage.speed_set_work == 0U)
	{
		WorkMessage.speed_set_work = Pubinterface_GetCurrentDefaultMotorSpeed(); /* 通道刚上线但速度未装载时，补当前方向Page4默认速度。 */
	}

	WorkMessage.speed_work = WorkMessage.speed_set_work; /* 启动前把目标速度同步到实际运行速度，无刷控制帧读取该值。 */
	return (WorkMessage.speed_work != 0U);				  /* 默认速度为0时不启动，避免下发运行标志但目标速度为空。 */
}

/*
 * 函数功能：处理单个通道实体键的启停状态机。
 * 输入参数：channel 当前处理的通道；press_event 去抖后的稳定按下沿事件；release_event 去抖后的稳定松开沿事件。
 * 返回参数：无。
 */
static void HandleRunKey_Process(uint8_t channel, bool press_event, bool release_event)
{
	uint8_t hand_model = HandleRunKey_GetChannelModel(channel); /* 每次处理都读取通道型号，A/B 切换或重新识别后策略立即跟随当前手柄。 */
	bool momentary_model = HandleRunKey_IsMomentaryModel(hand_model); /* LGZI 使用按住运行模式，其它型号保持原翻转启停模式。 */

	/* 当前通道已经由实体键启动时，本周期只处理强制停机、松开或再次按下，不重复走启动流程。 */
	if (s_handle_run_key_owner_channel == channel)
	{
		if ((WorkMessage.alarm_flag == true) ||
			(WorkMessage.channel_work != channel) ||
			(HandleRunKey_IsChannelReady(channel) == false))
		{
			HandleRunKey_SetMotorRun(false);			  /* 报警、通道离线或被切走时仍强制停止，不能等下一次按键。 */
			s_handle_run_key_owner_channel = CHANNEL_NONE; /* 清除实体键owner，后续其它按键需要重新满足启动条件。 */
			return; /* 安全停机已经完成，本周期不再继续处理翻转事件。 */
		}
		if (momentary_model == true)
		{
			if (release_event == true)
			{
				HandleRunKey_SetMotorRun(false);			  /* LGZI 手控按钮松开后立即停止，符合按住才运行的现场动作。 */
				s_handle_run_key_owner_channel = CHANNEL_NONE; /* 松开停止后释放owner，下一次按住需要重新走启动条件。 */
			}
			return; /* LGZI 运行中不响应第二次按下翻转，只有松开沿或安全条件能停机。 */
		}
		if (press_event == true)
		{
			HandleRunKey_SetMotorRun(false);			  /* 同一通道第二次稳定按下时执行停止，实现按一次启动、再按一次停止。 */
			s_handle_run_key_owner_channel = CHANNEL_NONE; /* 停止后释放实体键owner，下一次按下可重新启动任一有效通道。 */
		}
		return; /* 本通道已经拥有控制权时，松开不处理，只有安全条件或第二次按下才停止。 */
	}

	if (s_handle_run_key_owner_channel != CHANNEL_NONE)
	{
		return; /* 另一通道实体键正在控制时，本通道无效，保证任意时刻只有一个手柄工作。 */
	}

	if ((WorkMessage.drivetype_work != HANDLEWORK) && (s_handle_run_key_owner_channel == CHANNEL_NONE))
	{
		if ((press_event == true) && (WorkMessage.drivetype_work == JTWORK))
		{
			HandleRunKey_ShowFootAlarm(); /* 当前选中脚控时误按手柄键，只提示 82 号报警，不启动手柄。 */
		}
		return; /* 手控未被选中时实体手柄键不允许启动，避免脚控或触控选中时被手柄按键绕过。 */
	}

	if (press_event == false)
	{
		return; /* 没有稳定按下沿时不做动作，松开不会停止已经运行的手柄。 */
	}

	if ((WorkMessage.runflag_work == true) ||
		(WorkMessage.alarm_flag == true) ||
		(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE) == true))
	{
		return; /* 已运行、普通报警或其它控制来源占用时不启动实体键控制。 */
	}

	if (HandleRunKey_PrepareRunChannel(channel) == false)
	{
		return; /* 通道准备失败时不占用控制权，等待下个轮询周期重新判断。 */
	}

	if (HandleRunKey_SetMotorRun(true) == true)
	{

		s_handle_run_key_owner_channel = channel; /* 启动成功后记录owner通道，后续按该手柄型号决定松开停止或二次按下停止。 */
	}
}

/*
 * 函数功能：轮询A/B手柄实体运行键，LGZI按住运行松开停止，其它支持型号保持翻转启停。
 * 输入参数：无。
 * 返回参数：无。
 */
static void HandleKey_ScanRunKeys(void)
{
	bool key_a_press_event = false; /* UART模式没有GPIO按下沿；后续接入明确的KSZ报文协议后再由串口解析结果赋值。 */
	bool key_b_press_event = false; /* UART模式没有GPIO按下沿；禁止把UART数据跳变当作低有效实体键。 */

	if (s_handle_run_key_a_pin_mode == HANDLE_RUN_KEY_PIN_MODE_GPIO)
	{
		key_a_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_a_filter, HANDLE_RUN_KEY_A_STATUS()); /* 普通A手柄读取PE2，连续低电平约60ms后产生按下沿。 */
	}
	if (s_handle_run_key_b_pin_mode == HANDLE_RUN_KEY_PIN_MODE_GPIO)
	{
		key_b_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_b_filter, HANDLE_RUN_KEY_B_STATUS()); /* 普通B手柄读取PE0，连续低电平约60ms后产生按下沿。 */
	}

	HandleRunKey_Process(CHANNEL_A, key_a_press_event, s_handle_run_key_a_filter.release_event); /* 先处理A，按型号选择翻转启停或松开停止。 */
	HandleRunKey_Process(CHANNEL_B, key_b_press_event, s_handle_run_key_b_filter.release_event); /* 再处理B，若A已取得owner则B会被忽略。 */
}

/*
 * 函数功能：手柄实体按键周期扫描任务，按统一物理映射和手柄型号处理A/B实体键启停。
 * 输入参数：event 调度器事件参数，当前任务不使用。
 * 返回参数：无。
 */
void HANDLEKEYTaskFunc(uint32_t event)
{
	(void)event; /* 当前任务只按固定 30ms 周期运行，不使用调度事件值。 */
	HandleRunKey_UpdatePinModes(); /* 无论控制权是否被占用都先跟随识别结果切换复用脚，避免KSZ串口长期停在GPIO模式。 */
	HandleRunKey_ServiceFootAlarm(); /* 维护 82 号限时弹窗，到 ALARM_MODE_MS 后关闭。 */
	if (ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))
	{
		return; /* 其它来源正在控制时不扫描启动键，但仍先完成本模块提示生命周期维护。 */
	}
	HandleKey_ScanRunKeys(); /* 严格先处理 A、再处理 B；A 已取得 owner 时 B 本周期不能抢占。 */
}

/*
 * 函数功能：创建并启动手柄实体按键 30ms 周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void)
{
	Kernel_TaskCreate(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc); /* 使用现有独立静态任务对象创建实体键任务。 */
	Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, 30U); /* 周期固定为 30ms，不改变当前实体键去抖时基。 */
}
