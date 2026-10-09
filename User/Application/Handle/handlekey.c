//handlekey.c

#include "bsp_gpio.h"
#include "diagnostic.h" /* 独立诊断开关及只读副本接口。 */
#include "handlekey.h"
#include "data.h"
//#include "adc.h"
#include "soft_uart.h"
#include "stm32f4xx_hal.h"

#include "kernel_scheduler.h"
#include "iwdg.h" /* 异常重启后实体键必须先稳定松开，不能把持续按住当成新启动。 */
#include "datahand.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscKEYBH.h"
#include "sscUIDP.h"
#include "usart.h" /* GZYT 临时测试使用 A/UART10、B/UART8 的接收寄存器。 */
#include "common.h" /* 数字模块与主控使用同一 CRC16/MODBUS 算法。 */
#include "handle_control.h" /* 插拔变化后丢弃旧手柄缓存，重新确认数字按钮已松开。 */
#include "motoruartdata.h" /* 近期反馈仍有转速时不接受新的测试启动或反向请求。 */

/*
 * 文件功能：按手柄型号切换 A/B 按键脚用途，GPIO 每 30ms 扫描；GZYT 临时测试每 10ms 解析数字按钮串口。
 * 运行入口：Userparser_Init() 调用 HandleKeyScan_Init()，任务回调只进入 HANDLEKEYTaskFunc()。
 * 处理顺序：先更新引脚用途和串口测试，再检查 82 号提示；其它控制方式没占用时，按原 30ms 周期处理 GPIO 键。
 * 启动条件：只接受当前选中通道的手控按键；有报警、已运行、刀具未准备好或其它方式正在控制时都不能启动。
 */
kernel_task_t HANDLEKEYTaskHandle;

/*
 * 函数功能：GPIO 电平变化触发中断后，交给压力传感器的软件串口接收数据。
 * 输入参数：GPIO_Pin 为本次触发中断的 GPIO 引脚编号。
 * 返回参数：无。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SimUart_HandleExti(GPIO_Pin); /* 压力传感器靠这次电平变化开始收数据，手柄按键处理不能省掉此调用。 */
}

#define HANDLE_KEY_DEBOUNCE_COUNT 8U /* 连续 8 次读到相同电平才确认按下或松开；每次间隔 30ms，增大后响应更慢。 */
#define HANDLE_KEY_PRESSED_LEVEL GPIO_PIN_RESET /* 按下时为低电平（RESET），松开时上拉为高电平（SET）；须与硬件接线一致。 */

typedef struct
{
	uint8_t low_count;		/* 连续低电平计数，用于确认按键已经稳定按下。 */
	uint8_t high_count;		/* 连续高电平计数，用于确认按键已经稳定松开。 */
	bool stable_pressed;	/* 去抖后的按下状态，业务层只使用这个稳定结果。 */
	bool press_event;		/* 本轮刚确认按下时为 true，只保持这一轮；一直按住不会反复触发。 */
	bool release_event;		/* 本轮刚确认松开时为 true，LGZI 收到它后请求停机。 */
    bool restart_release_ready; /* 异常重启后为false；稳定高电平后置true，才允许产生新的按下事件。 */
} HandleRunKeyDebounce_t;

typedef enum
{
	HANDLE_RUN_KEY_PIN_MODE_UNKNOWN = 0U, /* 上电后尚未设置引脚用途，第一次扫描必须配置。 */
	HANDLE_RUN_KEY_PIN_MODE_GPIO,         /* 普通手柄模式：引脚上拉输入，低电平表示按钮按下。 */
	HANDLE_RUN_KEY_PIN_MODE_UART          /* KSZ/GZYT模式：引脚切换为对应UART接收复用，不再读取GPIO电平。 */
} HandleRunKeyPinMode_t;

static HandleRunKeyDebounce_t s_handle_run_key_a_filter = {0U, 0U, false, false, false, false}; /* A通道实体键去抖及重启松手确认。 */
static HandleRunKeyDebounce_t s_handle_run_key_b_filter = {0U, 0U, false, false, false, false}; /* B通道实体键去抖及重启松手确认。 */
static HandleRunKeyPinMode_t s_handle_run_key_a_pin_mode = HANDLE_RUN_KEY_PIN_MODE_UNKNOWN; /* A通道PE2当前复用状态。 */
static HandleRunKeyPinMode_t s_handle_run_key_b_pin_mode = HANDLE_RUN_KEY_PIN_MODE_UNKNOWN; /* B通道PE0当前复用状态。 */
static uint8_t s_handle_run_key_owner_channel = CHANNEL_NONE;			   /* 当前由实体键启动的通道，防止另一通道松开误停。 */
static uint8_t s_handle_foot_selected_timed_alarm_active = 0U; /* 记录 82 号错模式弹窗是否由手柄键模块显示，防止到期误清其它报警。 */
static uint32_t s_handle_foot_selected_timed_alarm_tick = 0U; /* 记录 82 号错模式弹窗开始时间，按 ALARM_MODE_MS（当前 2000ms）自动清除。 */

#if (GZYT_DIGITAL_TEST_ENABLE == 1U)
#define GZYT_TEST_TASK_MS 10U          /* 串口任务周期，单位 ms；调大后按钮停机和通信超时处理会变慢。 */
#define GZYT_TEST_TIMEOUT_MS 50U       /* 完整有效新帧超过此毫秒数未到达就清运行请求；实际清除还受任务调度和业务锁等待影响。 */
#define GZYT_TEST_RELEASE_FRAMES 3U    /* 新启动前连续确认 HAL4/HAL5 松开的帧数；模块每 10ms 发帧，调大后重新启动等待更久。 */
#define GZYT_TEST_RX_SIZE 128U         /* 每通道字节缓存容量，实际最多保存 127 字节；满后停本次测试并丢弃旧数据。 */
#define GZYT_TEST_FRAME_SIZE 12U       /* 数字模块 v2 帧长，包含 A5 5A、版本、序号、按钮、状态、复位原因及 CRC。 */
#define GZYT_TEST_FORWARD_MASK 0x08U   /* TRIGGER_MASK bit3 为 HAL4/HW4，数字模块 PA1 低电平确认后置 1。 */
#define GZYT_TEST_REVERSE_MASK 0x10U   /* TRIGGER_MASK bit4 为 HAL5/HW5，数字模块 PA7 低电平确认后置 1。 */
#define GZYT_TEST_BUTTON_MASK 0x18U    /* 临时测试只使用 HAL4/HAL5，HAL1~HAL3 不参与启停或松手确认。 */

typedef struct
{
    volatile uint8_t rx[GZYT_TEST_RX_SIZE];       /* 中断接收到的字节，任务按 read_index 顺序取出后解析。 */
    volatile uint32_t rx_tick[GZYT_TEST_RX_SIZE]; /* 各字节真正到达的 HAL 毫秒时刻，防止积压旧帧被当成刚收到。 */
    volatile uint16_t write_index;               /* 中断写入位置，写完字节和时刻后才推进；到缓存末尾回到 0。 */
    volatile uint16_t read_index;                /* 任务读取位置，取出字节和时刻后才推进；配置切换时与 write_index 一起清零。 */
    volatile bool enabled;                      /* 当前选中通道是在线 GZYT 时置 true，离线或切走时清零并关闭接收中断。 */
    volatile bool rx_fault;                     /* 串口错误或缓存满时由中断置 true；任务停测试、丢旧字节后清零。 */
    uint8_t frame[GZYT_TEST_FRAME_SIZE];          /* 逐字节组装的候选帧，完整通过校验后才读取触发位。 */
    uint8_t frame_size;                          /* 已组装字节数，完整处理或接收异常后清零；帧头错位时保留后 11 字节继续查找。 */
    uint8_t release_count;                       /* 连续有效松开帧计数，到 3 才允许新按下；停机、拒绝启动或异常时清零。 */
    uint8_t trigger;                             /* 最近有效帧中的 HAL4/HAL5 触发位；0 松开、08 正转、10 反转，18 不允许启动。 */
    uint8_t run_trigger;                         /* 本次测试启动时使用的 08/10；后续触发位改变就清运行请求，禁止直接换向。 */
    uint8_t reset_flags;                         /* 最近有效帧的模块复位原因；原因改变后必须重新确认松手。 */
    uint16_t sequence;                           /* 最近有效帧序号，下一帧必须递增 1，FFFF 到 0000 属于正常递增。 */
    bool have_sequence;                          /* 已收到有效帧时置 true；重新启用、超时或异常后清零，下一帧重新记录起点。 */
    bool running;                               /* 本通道数字按钮已发出运行请求时置 true；清请求或控制方式改变时清零，不表示实测转动。 */
    uint32_t last_frame_tick;                    /* 最近有效新帧最后一个字节到达的时刻，重复序号不能刷新此时间。 */
    uint32_t frame_start_tick;                   /* 当前候选帧开始到达的时刻，拆帧跨越 50ms 时丢弃，不能继承旧半帧。 */
    uint32_t identity_generation;                /* 启用接收时记住本通道插拔次数，变化后不能继续使用旧手柄的松手记录。 */
} GzytTestState_t;

static GzytTestState_t s_gzyt_test_a; /* A 通道只接收 UART10/PE2，按钮和松手记录与 B 独立。 */
static GzytTestState_t s_gzyt_test_b; /* B 通道只接收 UART8/PE0，不能用 A 的串口数据启动。 */
static uint32_t s_handle_gpio_scan_tick; /* 保留普通 GPIO 按键两次扫描之间至少 30ms，不随测试任务加快去抖。 */
#endif

/*
 * 函数功能：保存 GZYT 数字按钮串口收到的单字节及接收时刻，串口错误和缓存满只记录异常。
 * 输入参数：channel 为 CHANNEL_A/UART10 或 CHANNEL_B/UART8。
 * 返回参数：true 表示本串口由测试接收处理；false 表示未启用或通道无效，交给 HAL 处理。
 */
bool HandleKey_UartIrqHandler(uint8_t channel)
{
#if (GZYT_DIGITAL_TEST_ENABLE == 1U)
    GzytTestState_t *state; /* 按逻辑通道选择独立缓存，禁止交叉保存 A/B 字节。 */
    UART_HandleTypeDef *uart; /* A 固定 UART10，B 固定 UART8，不跟随电机物理通道交换。 */
    uint32_t status; /* 先读 SR，再读 DR，按 STM32F4 要求清除接收错误。 */
    uint8_t byte; /* 暂存本次 DR 中的低 8 位数据。 */
    uint16_t next; /* 计算下一写入位置，等于读取位置时说明缓存满。 */

    if ((channel != CHANNEL_A) && (channel != CHANNEL_B)) return false; /* 无效通道不读取任何串口。 */
    state = (channel == CHANNEL_A) ? &s_gzyt_test_a : &s_gzyt_test_b; /* A/B 中断只修改自己的接收缓存。 */
    uart = (channel == CHANNEL_A) ? &huart10 : &huart8; /* 使用与 PE2/PE0 一致的硬件接收口。 */
    if (state->enabled == false) return false; /* 非当前在线 GZYT 不参与临时测试接收。 */
    status = uart->Instance->SR; /* 取得本次接收标志，后续 DR 读取清掉同次错误。 */
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0U) return true; /* 本次没有字节或接收错误，不推进缓存。 */
    byte = (uint8_t)uart->Instance->DR; /* 读取 DR，清 RXNE 及已读 SR 对应的错误标志。 */
    if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        state->rx_fault = true; /* 丢字节、噪声或帧错误后停止使用旧按钮记录，任务下一轮清运行请求。 */
        return true; /* 错误字节不进入候选帧，防止被当成新的有效按下。 */
    }
    if (state->rx_fault == true) return true; /* 已异常时丢弃后续字节，等待任务停止测试并清缓存。 */
    next = (uint16_t)((state->write_index + 1U) % GZYT_TEST_RX_SIZE); /* 缓存位置回绕，不覆盖尚未读取的字节。 */
    if (next == state->read_index)
    {
        state->rx_fault = true; /* 缓存满说明数据完整性已丢失，本次测试必须重新松手。 */
        return true; /* 不覆盖旧数据，避免拼出来源不完整的按钮帧。 */
    }
    state->rx[state->write_index] = byte; /* 先保存当前字节。 */
    state->rx_tick[state->write_index] = HAL_GetTick(); /* 保存字节到达时间，任务延迟不能把旧帧时间改成当前时间。 */
    state->write_index = next; /* 两项数据写完再公布新写入位置，任务不会取到只更新一半的数据。 */
    return true; /* 本次字节已交给测试缓存，HAL 不再重复读取 DR。 */
#else
    (void)channel; /* 关闭测试时不用通道参数，也不打开 RXNE 中断。 */
    return false; /* 保留原 HAL 串口处理。 */
#endif
}

/*
 * 函数功能：脚控已选中时按手柄实体运行键，上报 82 号限时提示，保持时间由 ALARM_MODE_MS 控制（当前 2000ms）。
 * 输入参数：无，报警码固定使用 WORK_ALARM_FOOT_SELECTED。
 * 返回参数：无。
 */
static void HandleRunKey_ShowFootAlarm(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只读取 Value[0]，其余补零避免沿用上一条报警参数。 */

	display_value[0] = WORK_ALARM_FOOT_SELECTED; /* 82 号图提示当前脚控已选中，用户应使用脚踏启动。 */
	SendAlarmMessageTimed(WORK_ALARM_FOOT_SELECTED, ALARM_MODE_MS); /* 只提示 ALARM_MODE_MS（当前 2000ms），不把系统设为持续报警。 */
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
 * 输入参数：无；直接检查本模块是否显示过弹窗，以及经过了多少 ms。
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
	s_handle_foot_selected_timed_alarm_active = 0U; /* 本次提示已结束；下次在脚控模式误按手柄键时可以再提示。 */
	s_handle_foot_selected_timed_alarm_tick = 0U; /* 清掉上次开始时间，下次提示重新计时。 */
}

/*
 * 函数功能：判断手柄型号是否支持本次实体按键启停。
 * 输入参数：hand_model 通道记忆中的手柄型号。
 * 返回参数：true 表示该手柄允许实体按键启停，false 表示忽略该通道按键。
 */
static bool HandleRunKey_IsSupportedModel(uint8_t hand_model)
{
	return ((hand_model == PXBA_ONLINES) ||   /* PXBA 按一次启动，再按一次停止。 */
			(hand_model == LGZ_I_ONLINES));   /* LGZI 按住运行，确认松开后停止。 */
}

/*
 * 函数功能：判断手柄实体键是否采用按住运行、松开停止模式。
 * 输入参数：hand_model 通道记忆中的手柄型号。
 * 返回参数：true 表示确认松开就停机；false 表示按一次启动、再按一次停止。
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
 * 返回参数：true 表示KSZ_I、KSZ_II或GZYT串口手柄，false 表示该型号使用普通GPIO按钮模式。
 */
static bool HandleRunKey_IsUartModel(uint8_t hand_model)
{
	return ((hand_model == KSZ_I_ONLINES) ||
			(hand_model == KSZ_II_ONLINES) ||
			(hand_model == GZYT_ONLINES)); /* GZYT与两类KSZ共用按钮串口复用；离线或其它型号由在线检查恢复GPIO。 */
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
	filter->press_event = false;    /* 清掉“刚按下”标志，不能把引脚用途切换当作一次按键。 */
	filter->release_event = false;  /* 清掉“刚松开”标志，不能把引脚用途切换当作松手停机。 */
    filter->restart_release_ready = (Iwdg_WasAbnormalReset() == 0U); /* 普通上电保持原按键行为；异常重启必须先确认本键已松开。 */
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
		uart_alternate = BOARD_RES_HANDLE_RUN_KEY_A_UART_AF; /* KSZ/GZYT手柄下PE2切为UART10_RX。 */
	}
	else if (channel == CHANNEL_B)
	{
		gpio_port = BOARD_RES_HANDLE_RUN_KEY_B_PORT;       /* B按钮固定使用PE0。 */
		gpio_pin = BOARD_RES_HANDLE_RUN_KEY_B_PIN;         /* 选中PE0引脚位。 */
		uart_alternate = BOARD_RES_HANDLE_RUN_KEY_B_UART_AF; /* KSZ/GZYT手柄下PE0切为UART8_RX。 */
	}
	else
	{
		return; /* 非A/B通道没有复用引脚，禁止写未知GPIO寄存器。 */
	}

	gpio_init.Pin = gpio_pin; /* 本次只修改目标通道RX/按钮复用脚，不影响同端口其它引脚。 */
	if (target_mode == HANDLE_RUN_KEY_PIN_MODE_UART)
	{
		gpio_init.Mode = GPIO_MODE_AF_PP;             /* KSZ/GZYT内部MCU输出串口数据，RX脚恢复硬件复用输入路径。 */
		gpio_init.Pull = GPIO_NOPULL;                  /* 串口模式沿用CubeMX现有无上下拉配置。 */
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;   /* 串口复用沿用UART MSP初始化速度。 */
		gpio_init.Alternate = uart_alternate;          /* A绑定UART10_RX，B绑定UART8_RX，禁止交叉。 */
	}
	else
	{
		gpio_init.Mode = GPIO_MODE_INPUT;              /* 非KSZ/GZYT手柄或通道离线时恢复普通数字输入。 */
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
		a_target_mode = HANDLE_RUN_KEY_PIN_MODE_UART; /* A在线且识别为KSZ/GZYT后，把PE2切换为UART10_RX。 */
	}
	if ((WorkMessage.Channel_Bonline == true) &&
		(HandleRunKey_IsUartModel(MemoryMsgB.hand_model) == true))
	{
		b_target_mode = HANDLE_RUN_KEY_PIN_MODE_UART; /* B在线且识别为KSZ/GZYT后，把PE0切换为UART8_RX。 */
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
	WorkMessage.drivetype_work = HANDLEWORK; /* 本次由手柄键启动，当前控制方式记为手控。 */

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
 * 函数功能：连续多次读到相同电平后才确认按键变化，避免触点抖动造成反复启停。
 * 输入参数：filter 保存该键的连续读数和已确认状态；raw_level 为本次读到的 GPIO 电平。
 * 返回参数：true 表示本轮刚确认按下；false 表示没有新按下。刚松开的结果另存于 release_event。
 */
static bool HandleRunKey_DebouncePressEvent(HandleRunKeyDebounce_t *filter, GPIO_PinState raw_level)
{
	filter->press_event = false; /* 每轮先清“刚按下”，长按不会每 30ms 再触发一次启停。 */
	filter->release_event = false; /* 每轮先清“刚松开”，LGZI 松手停机只处理一次。 */
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
				filter->press_event = filter->restart_release_ready; /* 异常重启后一直按住时不产生启动事件，必须先经过稳定松开。 */
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
            filter->restart_release_ready = true; /* 已连续确认高电平，下一次新按下可以按原规则处理。 */
			if (filter->stable_pressed == true)
			{
				filter->release_event = true; /* 之前按着，现在连续高电平已达标，本轮通知 LGZI 松手停机。 */
			}
			filter->stable_pressed = false; /* 连续高电平达到阈值后复位按下状态，允许下一次按下重新触发。 */
		}
	}

	return filter->press_event; /* 返回本轮是否刚按下；调用方还会读取 release_event 判断 LGZI 是否刚松开。 */
}

/*
 * 函数功能：处理手柄键的启动或停止请求，同时更新注水泵和当前控制方式。
 * 输入参数：enable true 表示启动电机，false 表示停止电机。
 * 返回参数：true 表示已写入启停状态；false 表示刀具未准备好或其它控制方式正在占用，启动被拒绝。
 */
static bool HandleRunKey_SetMotorRun(bool enable)
{
	/* 启动前检查刀具和控制方式；停止不受这些检查限制，避免无法停机。 */
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
		Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 这是一次新按下，清掉上次压力堵转停泵记录，再由泵任务检查当前压力是否允许运行。 */
	}

	ControlSignalMessage.handle_control_flag = enable; /* 通知公共控制信号当前由手柄实体键控制电机启停。 */
	WorkMessage.runflag_work = enable;				 /* sscDrive任务读取该标志后下发无刷运行或停止控制帧。 */
	Pubinterface_SetHandleInjectionPumpRun(enable);	 /* 注水泵是手柄冷却联动泵，实体键让手柄运行/停止时按泵类型和当前通道同步启停 A/B 冷却泵。 */

	if (enable == false)
	{
		WorkMessage.speed_work = 0U; /* 停止时立即清实际目标速度，防止停止帧前残留上一次速度。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE); /* 检查软件运行状态，空闲时允许其它控制方式接管；这里不等待实测转速归零。 */
	}

	SendKeyBehMessage(HANDLEKey, enable ? HANDLEKey_motor_start : HANDLEKey_motor_stop); /* 发送手柄键启停消息，让屏幕和上位机同步显示。 */
	return true; /* 启停状态已写入，调用方可以记录本次由哪个通道的实体键控制。 */
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

	WorkMessage.speed_work = WorkMessage.speed_set_work; /* 启动前把设定值复制为待下发的目标转速；这不是电机实测转速。 */
	return (WorkMessage.speed_work != 0U);				  /* 默认速度为0时不启动，避免下发运行标志但目标速度为空。 */
}

/*
 * 函数功能：按型号处理一个通道的手柄键：LGZI 按住运行，其它支持型号按两次分别启停。
 * 输入参数：channel 为通道；press_event 表示本轮刚确认按下；release_event 表示本轮刚确认松开。
 * 返回参数：无。
 */
static void HandleRunKey_Process(uint8_t channel, bool press_event, bool release_event)
{
	uint8_t hand_model = HandleRunKey_GetChannelModel(channel); /* 每次处理都读取通道型号，A/B 切换或重新识别后策略立即跟随当前手柄。 */
	bool momentary_model = HandleRunKey_IsMomentaryModel(hand_model); /* LGZI 松开就停；其它支持型号需再按一次才停。 */

	/* 当前通道已经由实体键启动时，本周期只处理强制停机、松开或再次按下，不重复走启动流程。 */
	if (s_handle_run_key_owner_channel == channel)
	{
		if ((WorkMessage.alarm_flag == true) ||
			(WorkMessage.channel_work != channel) ||
			(HandleRunKey_IsChannelReady(channel) == false))
		{
			HandleRunKey_SetMotorRun(false);			  /* 报警、通道离线或被切走时仍强制停止，不能等下一次按键。 */
			s_handle_run_key_owner_channel = CHANNEL_NONE; /* 清掉本次启动通道，下一次按键必须重新检查启动条件。 */
			return; /* 已请求停止，本轮不再尝试重新启动。 */
		}
		if (momentary_model == true)
		{
			if (release_event == true)
			{
				HandleRunKey_SetMotorRun(false);			  /* LGZI 手控按钮松开后立即停止，符合按住才运行的现场动作。 */
				s_handle_run_key_owner_channel = CHANNEL_NONE; /* 松开后清掉启动通道，下一次按下重新检查启动条件。 */
			}
			return; /* LGZI 运行时不按“再按一次停止”处理；确认松开或上面的异常条件才请求停止。 */
		}
		if (press_event == true)
		{
			HandleRunKey_SetMotorRun(false);			  /* 同一通道第二次稳定按下时执行停止，实现按一次启动、再按一次停止。 */
			s_handle_run_key_owner_channel = CHANNEL_NONE; /* 清掉启动通道；下次仍只能启动当前选中且满足条件的通道。 */
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
		return; /* 没有新的按下就不启动；已由本键启动的 LGZI 松手停机已在前面处理。 */
	}

	if ((WorkMessage.runflag_work == true) ||
		(WorkMessage.alarm_flag == true) ||
		(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE) == true))
	{
		return; /* 已运行、普通报警或其它控制来源占用时不启动实体键控制。 */
	}

	if (HandleRunKey_PrepareRunChannel(channel) == false)
	{
		return; /* 本次按下不能启动，不占用控制；条件恢复后仍需重新按键。 */
	}

	if (HandleRunKey_SetMotorRun(true) == true)
	{

		s_handle_run_key_owner_channel = channel; /* 记住本次由哪个通道启动，后续只让该通道按型号规则停止。 */
	}
}

/*
 * 函数功能：依次检查 A/B 运行键；LGZI 按住运行、松开停止，其它支持型号按一次启动、再按一次停止。
 * 输入参数：无。
 * 返回参数：无。
 */
static void HandleKey_ScanRunKeys(void)
{
	bool key_a_press_event = false; /* UART模式没有GPIO按下沿；后续按KSZ/GZYT报文协议解析后再赋值。 */
	bool key_b_press_event = false; /* UART模式没有GPIO按下沿；禁止把UART数据跳变当作低有效实体键。 */

	if (s_handle_run_key_a_pin_mode == HANDLE_RUN_KEY_PIN_MODE_GPIO)
	{
		key_a_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_a_filter, HANDLE_RUN_KEY_A_STATUS()); /* 普通A手柄读取PE2，连续低电平约120ms后产生按下沿。 */
	}
	if (s_handle_run_key_b_pin_mode == HANDLE_RUN_KEY_PIN_MODE_GPIO)
	{
		key_b_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_b_filter, HANDLE_RUN_KEY_B_STATUS()); /* 普通B手柄读取PE0，连续低电平约120ms后产生按下沿。 */
	}

	HandleRunKey_Process(CHANNEL_A, key_a_press_event, s_handle_run_key_a_filter.release_event); /* 先处理 A，根据型号判断按下启动、再次按下停止或松手停止。 */
	HandleRunKey_Process(CHANNEL_B, key_b_press_event, s_handle_run_key_b_filter.release_event); /* 再处理 B；如果 A 已经由实体键启动，B 的按键不会影响 A。 */
}

#if (GZYT_DIGITAL_TEST_ENABLE == 1U)
/*
 * 函数功能：结束本通道数字按钮发起的测试请求，并清除旧帧和松手记录。
 * 输入参数：state 为通道测试记录；channel 为 CHANNEL_A 或 CHANNEL_B；由持有业务互斥锁的任务调用。
 * 返回参数：无；其它控制方式正在控制或当前已换通道时，不修改其电机和泵请求。
 */
static void HandleGzytTest_Revoke(GzytTestState_t *state, uint8_t channel)
{
    if ((state->running == true) && (WorkMessage.channel_work == channel) &&
        (ControlArbitration_IsOwner(CONTROL_OWNER_HANDLE) == true))
    {
        WorkMessage.runflag_work = false; /* 清掉本数字按钮的运行请求，驱动任务随后发送停止帧；不代表实测已停。 */
        WorkMessage.speed_work = 0U; /* 同时清目标速度，下一驱动周期不继续使用旧转速。 */
        ControlSignalMessage.handle_control_flag = false; /* 手控运行标志随本次测试请求一起清除。 */
        Pubinterface_SetHandleInjectionPumpRun(false); /* 结束本次手柄冷却联动，不改变其它来源单独控制的泵。 */
        ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE); /* 近期反馈仍非零时保留占用，等待原有反馈检查允许退出。 */
    }
    state->running = false; /* 本模块不再持有测试运行记录，下次按下重新检查启动条件。 */
    state->run_trigger = 0U; /* 清掉上次启动使用的按钮位，不能继承上次方向。 */
    state->trigger = 0U; /* 丢弃旧按钮状态，后续只能使用新帧报告的 HAL4/HAL5。 */
    state->release_count = 0U; /* 重新收到 3 帧两键松开后，才允许下一次按下。 */
    state->have_sequence = false; /* 后续第一帧重新记录序号起点，避免模块重启后永久拒绝接收。 */
    state->frame_size = 0U; /* 丢弃正在组装的旧半帧，不能与后续字节拼接使用。 */
    taskENTER_CRITICAL(); /* 暂停接收中断修改位置，避免清缓存时只更新一半。 */
    state->read_index = state->write_index; /* 丢弃已积压字节，新的松手确认必须来自后续接收。 */
    state->rx_fault = false; /* 本次异常已经处理，接收中断可以继续保存新字节。 */
    taskEXIT_CRITICAL(); /* 接收位置更新完成，允许串口中断继续执行。 */
}

/*
 * 函数功能：在当前选中在线 GZYT 或其插拔次数改变时，重新配置本通道的字节接收。
 * 输入参数：state 为通道记录；channel 为 A/B；uart 为 A 的 huart10 或 B 的 huart8；enable 表示是否允许接收。
 * 返回参数：无；配置后仍须新收 3 帧两键松开，不能直接沿用插入时按住的按钮。
 */
static void HandleGzytTest_Configure(GzytTestState_t *state, uint8_t channel,
                                    UART_HandleTypeDef *uart, bool enable)
{
    uint32_t generation = Handle_GetIdentityGeneration(channel); /* 读取本通道的插拔次数，同型号换柄也要清旧记录。 */
    if ((state->enabled == enable) && ((enable == false) || (state->identity_generation == generation))) return; /* 通道用途和手柄未变时不重复初始化。 */
    __HAL_UART_DISABLE_IT(uart, UART_IT_RXNE); /* 先暂停该串口字节接收中断，避免清记录途中又写入字节。 */
    __HAL_UART_DISABLE_IT(uart, UART_IT_ERR); /* 配置期间不让旧串口错误再次进入测试接收。 */
    state->enabled = false; /* 中断若已经排队，看到此值后不再读写测试缓存。 */
    HandleGzytTest_Revoke(state, channel); /* 原测试停止并重新要求松手；不发异步启动/停止按键消息。 */
    state->read_index = 0U; /* 接收已关闭，可以把读取位置恢复到缓存开头。 */
    state->write_index = 0U; /* 写入位置与读取位置一起复位，缓存从空状态开始。 */
    state->identity_generation = generation; /* 记录本次使用的手柄，下一次插拔后重新配置。 */
    __HAL_UART_CLEAR_OREFLAG(uart); /* 按 SR/DR 顺序清旧 RXNE 和错误，GPIO 阶段留下的电平不进入新帧。 */
    state->enabled = enable; /* 初始化全部完成后，再允许中断访问这份接收记录。 */
    if (enable == true)
    {
        __HAL_UART_ENABLE_IT(uart, UART_IT_ERR); /* 丢字节、噪声或帧错误时记录异常，任务随后清本次测试请求。 */
        __HAL_UART_ENABLE_IT(uart, UART_IT_RXNE); /* 每收到一个字节就保存，不需要等待固定 DMA 块收满。 */
    }
}

/*
 * 函数功能：校验完整数字模块 v2 帧，更新 HAL4/HAL5 的有效状态和松手计数。
 * 输入参数：state 为通道记录；received_tick 为最后一个字节的接收时刻；now 为当前 HAL 毫秒时刻。
 * 返回参数：true 表示帧可接受或为不续时的重复帧；false 表示须停止、清旧缓存并重新松手，包括运行中松开或改变按钮。
 */
static bool HandleGzytTest_AcceptFrame(GzytTestState_t *state, uint32_t received_tick, uint32_t now)
{
    uint16_t crc = (uint16_t)state->frame[10] | ((uint16_t)state->frame[11] << 8U); /* CRC 低字节先发，与数字模块格式一致。 */
    uint16_t sequence = (uint16_t)state->frame[4] | ((uint16_t)state->frame[5] << 8U); /* 读取 16 位小端递增序号。 */
    uint8_t trigger = state->frame[7] & GZYT_TEST_BUTTON_MASK; /* 只解释 HAL4/HAL5，其它三个 Hall 不控制此次测试。 */
    if ((Common_Crc16(&state->frame[2], 8U) != crc) || (state->frame[8] != 0x01U) ||
        ((state->frame[6] & 0xE0U) != 0U) || ((state->frame[7] & 0xE0U) != 0U) ||
        ((state->frame[9] & 0xC0U) != 0U) || ((uint32_t)(now - received_tick) >= GZYT_TEST_TIMEOUT_MS))
    {
        return false; /* CRC、就绪状态、保留位或帧时效不合格，不能把异常时清零的触发位当成有效松手。 */
    }
    if (state->have_sequence == true)
    {
        if (sequence == state->sequence) return true; /* 重复帧不改变按钮、不累计松手，也不刷新通信有效时间。 */
        if (((uint16_t)(sequence - state->sequence) != 1U) || (state->frame[9] != state->reset_flags) ||
            ((uint32_t)(received_tick - state->last_frame_tick) >= GZYT_TEST_TIMEOUT_MS))
        {
            return false; /* 跳号、倒序、复位原因改变或帧间断超过 50ms 后，先停测试再重新确认松手。 */
        }
    }
    if ((trigger == GZYT_TEST_BUTTON_MASK) ||
        ((state->running == true) && (trigger != state->run_trigger)) ||
        ((state->trigger != 0U) && (trigger != 0U) && (trigger != state->trigger)))
    {
        return false; /* 双键、运行中松开或未松开就换键均清运行请求，不允许直接由正转切到反转。 */
    }
    if (trigger == 0U)
    {
        if (state->trigger != 0U) state->release_count = 0U; /* 从按下变为松开时，重新累计本次松手的有效帧。 */
        if (state->release_count < GZYT_TEST_RELEASE_FRAMES)
        {
            if ((state->frame[6] & GZYT_TEST_BUTTON_MASK) == GZYT_TEST_BUTTON_MASK)
            {
                ++state->release_count; /* 原始电平均高且稳定触发位为零，才计为一帧确实松开。 */
            }
            else
            {
                state->release_count = 0U; /* 原始电平仍低时不能用去抖延迟的零触发位解锁新启动。 */
            }
        }
    }
    state->trigger = trigger; /* 保存本帧确认的按钮状态，供任务按住运行或等待新按下。 */
    state->sequence = sequence; /* 只保存新且正常的序号，下一帧必须连续。 */
    state->reset_flags = state->frame[9]; /* 保存本次复位原因，模块重启后不能沿用旧松手记录。 */
    state->last_frame_tick = received_tick; /* 使用实际到达时刻，不能用当前处理时刻给积压帧续时。 */
    state->have_sequence = true; /* 已有正常新帧，后续开始序号和超时检查。 */
    return true; /* 帧有效，按钮状态及通信时刻已同步更新。 */
}

/*
 * 函数功能：处理一个通道的数字按钮缓存；HAL4 按住正转、HAL5 按住反转，释放、异常或超时后清请求。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B；由按键任务在公共业务互斥锁内调用。
 * 返回参数：无；仅当前在线 GZYT、手控选中且已有非零方向转速时允许启动。
 */
static void HandleGzytTest_PollChannel(uint8_t channel)
{
    GzytTestState_t *state = (channel == CHANNEL_A) ? &s_gzyt_test_a : &s_gzyt_test_b; /* 选择本通道独立记录。 */
    UART_HandleTypeDef *uart = (channel == CHANNEL_A) ? &huart10 : &huart8; /* 保持 A/UART10、B/UART8 对应关系。 */
    bool online = (channel == CHANNEL_A) ? WorkMessage.Channel_Aonline : WorkMessage.Channel_Bonline; /* 只使用本通道校验完成后的在线标志。 */
    bool enable = online && (HandleRunKey_GetChannelModel(channel) == GZYT_ONLINES) &&
                  (WorkMessage.channel_work == channel) && (WorkMessage.hand_model == GZYT_ONLINES); /* 非选中或非 GZYT 时不接收测试按钮。 */
    uint32_t now = HAL_GetTick(); /* 本轮超时检查时刻，不能用它替换字节实际接收时刻。 */
    uint16_t end; /* 本轮最多处理开始时已收字节，防止噪声连续到达让任务无法返回。 */
    uint8_t direction; /* 按触发位生成 ZZDIR/FZDIR，禁止把 Hall 电平当成方向号。 */

    HandleGzytTest_Configure(state, channel, uart, enable); /* 插拔、切通道或退出 GZYT 时清掉旧数据并重新要求松手。 */
    if (enable == false) return; /* 本通道不具备测试条件，不处理 GPIO 或 KSZ 的串口电平。 */
    if ((state->rx_fault == true) ||
        ((state->frame_size != 0U) && ((uint32_t)(now - state->frame_start_tick) >= GZYT_TEST_TIMEOUT_MS)))
    {
        HandleGzytTest_Revoke(state, channel); /* 串口错误、缓存满或旧半帧超时后先清请求，不允许后面的积压字节立即重启。 */
        return; /* 下一轮从新字节重新确认两键松开。 */
    }
    end = state->write_index; /* 固定本轮结束位置，本轮后到达的字节留给下一次任务。 */
    now = HAL_GetTick(); /* 取结束位置后再读当前时间，已保存字节的接收时刻不会晚于本次检查时刻。 */
    while (state->read_index != end)
    {
        uint8_t byte = state->rx[state->read_index]; /* 先复制字节，随后才能把该位置交回中断使用。 */
        uint32_t tick = state->rx_tick[state->read_index]; /* 同时复制该字节到达时刻，不能混入下一圈写入的新时刻。 */
        state->read_index = (uint16_t)((state->read_index + 1U) % GZYT_TEST_RX_SIZE); /* 字节和时刻复制完成后推进读取位置。 */
        if ((uint32_t)(now - tick) >= GZYT_TEST_TIMEOUT_MS)
        {
            HandleGzytTest_Revoke(state, channel); /* 即使随后还有新字节，也不能让积压旧帧维持运行或累计松手。 */
            return; /* 丢掉本轮剩余积压数据，等待新的正常松手帧。 */
        }
        if (state->frame_size == 0U) state->frame_start_tick = tick; /* 新候选帧开始时记下时间，后续拆帧不得跨越超时限制。 */
        state->frame[state->frame_size++] = byte; /* 逐字节拼候选帧，支持一帧拆成多次任务接收。 */
        if (state->frame_size < GZYT_TEST_FRAME_SIZE) continue; /* 字节不足 12 个时不解析按钮，保留到下一轮。 */
        if ((state->frame[0] != 0xA5U) || (state->frame[1] != 0x5AU) ||
            (state->frame[2] != 0x02U) || (state->frame[3] != GZYT_TEST_FRAME_SIZE))
        {
            memmove(state->frame, &state->frame[1], GZYT_TEST_FRAME_SIZE - 1U); /* 帧头错位时只移掉首字节，保留后面的可能帧头。 */
            state->frame_size = GZYT_TEST_FRAME_SIZE - 1U; /* 下一字节到达后继续查找完整 v2 帧，不把杂字节当成按钮。 */
            continue; /* 尚未找到可校验的帧，本轮通信有效时间不变。 */
        }
        state->frame_size = 0U; /* 已取得完整候选帧，校验成功后可继续处理同一缓存中的下一帧。 */
        if (HandleGzytTest_AcceptFrame(state, tick, now) == false)
        {
            HandleGzytTest_Revoke(state, channel); /* 坏帧、松开或换键均先清请求，后面的积压按下不能在本轮再次启动。 */
            return; /* 等新的三帧松手及下一次按下，避免同轮停止后又发送非零请求。 */
        }
    }
    if ((state->rx_fault == true) ||
        ((state->have_sequence == true) && ((uint32_t)(now - state->last_frame_tick) >= GZYT_TEST_TIMEOUT_MS)))
    {
        HandleGzytTest_Revoke(state, channel); /* 处理期间接收出错或最后新帧已超时，均清本次测试请求。 */
        return; /* 重复帧和坏版本帧不能延长运行许可。 */
    }
    if (state->have_sequence == false) return; /* 首帧尚未收全时保留半帧；没有正常帧不能累计松手或启动。 */
    if (state->running == true)
    {
        if ((WorkMessage.runflag_work == false) || (WorkMessage.alarm_flag == true) ||
            (WorkMessage.drivetype_work != HANDLEWORK) || (ControlSignalMessage.handle_control_flag == false) ||
            (ControlArbitration_IsOwner(CONTROL_OWNER_HANDLE) == false))
        {
            HandleGzytTest_Revoke(state, channel); /* 报警或其它处理已经结束测试时清旧按住记录，条件恢复不能自动重启。 */
        }
        return; /* 正常按住时保留原请求，不反复写方向、转速和启动消息。 */
    }
    if ((state->trigger == 0U) || (state->release_count < GZYT_TEST_RELEASE_FRAMES)) return; /* 两键松开或还未确认三帧松手时不启动。 */
    if ((WorkMessage.drivetype_work != HANDLEWORK) || (WorkMessage.runflag_work == true) ||
        (WorkMessage.alarm_flag == true) || (ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE) == true) ||
        (MotorUart_IsRecentFeedbackMoving() == true) || (s_handle_run_key_owner_channel != CHANNEL_NONE))
    {
        HandleGzytTest_Revoke(state, channel); /* 未选手控、已有请求、报警、其它来源占用、近期仍转动或旧 GPIO 按键尚未退出时拒绝本次按下；必须再松开后重试。 */
        return; /* 持续按住不能在条件后来恢复时自行启动。 */
    }
    direction = (state->trigger == GZYT_TEST_FORWARD_MASK) ? ZZDIR : FZDIR; /* HAL4 使用主控正转语义，HAL5 使用反转语义。 */
    DirActive((direction == ZZDIR) ? HANDLEKey_dir_Forward : HANDLEKey_dir_Reverse); /* 使用原方向处理，同步当前方向、该方向转速和 A/B 记忆及屏幕。 */
    if ((WorkMessage.dir_work != direction) || (WorkMessage.speed_set_work == 0U) ||
        (Pubinterface_CheckCommonSocketToolReadyForRun() == false) ||
        (ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE) == false))
    {
        HandleGzytTest_Revoke(state, channel); /* 方向被限制、该方向参数缺失或不能取得控制时拒绝启动，不套用其它手柄的测试转速。 */
        return; /* 下次需重新确认两键松开并按下，不能反复尝试旧请求。 */
    }
    HandleRunKey_ApplyHandleMode(channel); /* 保存本通道手控方式，切换通道后仍按正常界面规则选择控制方式。 */
    Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 这是新按下，冷却泵按原规则重新检查实际压力。 */
    WorkMessage.speed_work = WorkMessage.speed_set_work; /* 使用本方向当前保存的 rpm 设置，不修改电机类型、倍率或保护电流。 */
    ControlSignalMessage.handle_control_flag = true; /* 标记本次由手柄数字按钮发出运行请求。 */
    WorkMessage.runflag_work = true; /* 驱动任务按已有电机参数发送运行帧，仍由驱动反馈和原报警逻辑处理保护。 */
    Pubinterface_SetHandleInjectionPumpRun(true); /* 与正常手控相同，冷却泵继续执行原定位、压力和联动检查。 */
    state->run_trigger = state->trigger; /* 保存本次实际使用的按钮，随后换键必须先清请求并重新松手。 */
    state->running = true; /* 记录本模块已经写入运行请求；不代表驱动已回复或电机已实际转动。 */
    /* 本测试直接更新请求，不再发送 HANDLEKey_motor_start 队列消息，避免超时停机后旧启动消息再次置位。 */
}
#endif

/*
 * 函数功能：手柄实体按键周期扫描任务，按统一物理映射和手柄型号处理A/B实体键启停。
 * 输入参数：event 调度器事件参数，当前任务不使用。
 * 返回参数：无。
 */
void HANDLEKEYTaskFunc(uint32_t event)
{
	(void)event; /* 使用周期扫描，不使用调度事件值；测试开启后只加快串口处理。 */
	HandleRunKey_UpdatePinModes(); /* 无论控制权是否被占用都先跟随识别结果切换复用脚，避免KSZ/GZYT串口停在GPIO模式。 */
#if (GZYT_DIGITAL_TEST_ENABLE == 1U)
    HandleGzytTest_PollChannel(CHANNEL_A); /* 即使其它来源占用，也先维护 A 测试的超时和旧请求清除。 */
    HandleGzytTest_PollChannel(CHANNEL_B); /* B 单独接收和确认松手，不用 A 数据控制 B。 */
    if ((uint32_t)(HAL_GetTick() - s_handle_gpio_scan_tick) < 30U) return; /* GPIO 仍间隔至少 30ms，不能把原连续确认次数缩短为 10ms。 */
    s_handle_gpio_scan_tick = HAL_GetTick(); /* 记录本次 GPIO 扫描时刻，下一次至少再等待 30ms。 */
#endif
	HandleRunKey_ServiceFootAlarm(); /* 维护 82 号限时弹窗，到 ALARM_MODE_MS 后关闭。 */
	if (ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))
	{
		return; /* 脚踏、屏幕或外控正在控制时不处理手柄键；上面的引脚更新和提示到期检查仍会执行。 */
	}
	HandleKey_ScanRunKeys(); /* 先处理 A 再处理 B；A 由实体键启动后，本轮 B 不能再启动。 */
}

/*
 * 函数功能：创建手柄按键任务；开启 GZYT 测试时每 10ms 处理串口，普通 GPIO 仍按 30ms 扫描。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void)
{
	Kernel_TaskCreate(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc); /* 使用现有独立静态任务对象创建实体键任务。 */
#if (GZYT_DIGITAL_TEST_ENABLE == 1U)
    s_handle_gpio_scan_tick = HAL_GetTick(); /* 初始化 GPIO 扫描时刻，测试任务加快后仍保持原去抖间隔。 */
    Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, GZYT_TEST_TASK_MS); /* 同一任务每 10ms 检查收帧及超时，不额外增加业务线程。 */
#else
	Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, 30U); /* 周期固定为 30ms，不改变当前实体键去抖时基。 */
#endif
}

#if (MAINCTRL_DIAG_ENABLE == 1U)
/*
 * 函数功能：复制指定手柄按键的确认计数和引脚电平，供诊断记录；不增加计数，不产生新的按键事件。
 * 输入参数：channel=1 为 A 手柄、2 为 B 手柄；out 指向 DiagnosticHandleKeyState_t 输出变量；调用时须持有公共业务互斥锁。
 * 返回参数：无；out 为空或通道不是 1/2 时直接返回，out 保持原值。
 */
void HandleKey_CopyDiagnosticState(uint8_t channel, DiagnosticHandleKeyState_t *out)
{
    if ((out == NULL) || (channel < 1U) || (channel > 2U)) return; /* 指针为空或通道不是 A/B 时不读取按键数据，输出内容保持原值。 */
    const HandleRunKeyDebounce_t *filter = (channel == 1U) ? &s_handle_run_key_a_filter : &s_handle_run_key_b_filter; /* 按通道选择 A 或 B 的按键确认记录，诊断不会增加计数。 */
    out->pin_mode = (channel == 1U) ? s_handle_run_key_a_pin_mode : s_handle_run_key_b_pin_mode; /* 手柄按键引脚用途：0 尚未配置，1 GPIO 按键输入，2 UART 接收；用途切换时清旧按键记录。 */
    out->raw_level = (channel == 1U) ? HANDLE_RUN_KEY_A_STATUS() : HANDLE_RUN_KEY_B_STATUS(); /* 读取诊断数据时的引脚电平：GPIO 模式下 0 为按下、1 为松开；UART 模式下不能解释为按键。 */
    out->low_ticks = filter->low_count; /* 连续读到按下低电平的次数，每轮 30ms；读到高电平后清零。 */
    out->high_ticks = filter->high_count; /* 连续读到松开高电平的次数，每轮 30ms；读到低电平后清零。 */
    out->pressed = filter->stable_pressed; /* 按下电平持续达到确认次数后为 1；松开电平持续达到确认次数后清零。 */
    out->press_event = filter->press_event; /* 本轮刚确认按下且已满足重启后松手要求时为 1；下一轮扫描先清零，长按不会反复产生。 */
    out->release_event = filter->release_event; /* 本轮刚确认松开时为 1；下一轮扫描先清零，LGZI 用它请求松手停机。 */
    out->restart_ready = filter->restart_release_ready; /* 异常重启后为 0；确认按键稳定松开后置 1，才允许新的按下事件。普通上电初值为 1。 */
    out->owner_channel = s_handle_run_key_owner_channel; /* 由手柄实体键启动的通道：0 无，1 A，2 B；用于判断哪侧松手可以撤销运行请求。 */
}
#endif
