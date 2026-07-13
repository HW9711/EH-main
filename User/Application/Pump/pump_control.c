#include "pump_control.h"

#include "Pubinterface.h"
#include "pump.h"
#include "sscUIDP.h"

/* 抽吸泵业务最大显示/设定速度，和 A/B 泵任务中的最终输出限幅保持一致。 */
#define PUMP_DRAWWATER_SPEED_MAX 15U
/* 灌注泵业务最大显示/设定速度，和 A/B 泵任务中的最终输出限幅保持一致。 */
#define PUMP_POURWATER_SPEED_MAX 300U
/* 抽吸泵 0 速启动时使用一档小流量，避免只置运行标志但泵不转。 */
#define PUMP_DRAWWATER_START_SPEED 3U
/* 灌注泵 0 速启动时使用一档 50 ml 流量，避免只置运行标志但泵不转。 */
#define PUMP_POURWATER_START_SPEED 50U

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
		return Pubinterface_GetInjectionPumpStartFlow(); /* 注水泵沿用当前通道 Page4 默认流量，非法值回退 30。 */
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
 * 函数功能：处理左右实体脚踏的泵档位循环，只调整对应泵的工作速度并刷新对应屏幕区域。
 * 输入参数：key_value 为左侧或右侧实体脚踏短按键值。
 * 返回参数：无。
 */
static void Pubinterface_HandlePumpGearKey(uint8_t key_value)
{
	static uint8_t PumpA_Gear = 0; /* 保存 A 泵实体脚踏档位，保持与原 PUMPActive 内静态变量相同的跨周期状态。 */
	static uint8_t PumpB_Gear = 0; /* 保存 B 泵实体脚踏档位，左右泵继续各自独立循环。 */

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
	default:
		break; /* 调度入口只会传入档位键；保留默认分支防止误调用改变泵状态。 */
	}
}

/*
 * 函数功能：处理 A/B 泵启停和屏幕注水泵排空请求；只有屏幕排空按钮使用固定速度和10秒计时。
 * 输入参数：key_value 为泵启停键值，pump_owner 为该键对应的控制权来源。
 * 返回参数：无。
 */
static void Pubinterface_HandlePumpRunKey(uint8_t key_value, uint8_t pump_owner)
{
	bool is_timed_drainage = false; /* 仅屏幕排空按钮置位；实体长按和 HMI 普通启泵继续使用 speed_work。 */

	switch (key_value)
	{
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
				is_timed_drainage = (key_value == SCREENKey_APUMP_control); /* A 注水泵只有屏幕排空按钮使用固定速度和10秒计时。 */
				pumpMessageA.run_flag = true;
				if (pumpMessageA.type == INJECTWATER)
				{
					pumpMessageA.pedalDrainage_flag = false; /* 新控制请求开始前关闭脚踏轻踩来源，避免按钮状态和控制来源叠加。 */
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
				pumpMessageA.pedalDrainage_flag = false; /* 屏幕停止 A 泵时同步结束所有排空来源。 */
				pumpMessageA.timingDrainage_times = 0U;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑,A
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		Pubinterface_RefreshPumpADisplay(); /* A 泵启停后立即刷新；只有屏幕排空或脚踏轻踩来源高亮注水泵按钮。 */
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
				is_timed_drainage = (key_value == SCREENKey_BPUMP_control); /* B 注水泵只有屏幕排空按钮使用固定速度和10秒计时。 */
				pumpMessageB.run_flag = true;
				if (pumpMessageB.type == INJECTWATER)
				{
					pumpMessageB.pedalDrainage_flag = false; /* 新控制请求开始前关闭脚踏轻踩来源，避免来源叠加。 */
					pumpMessageB.timingDrainage_flag = is_timed_drainage;
					pumpMessageB.timingDrainage_times = 0U; /* 清掉旧排空计数，防止第二次启动被历史计时立即关断。 */
				}
				// 队列发送界面按钮和数字变黄
				// 队列发送泵运行设置数据
			}
			else
			{
				pumpMessageB.timingDrainage_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* 屏幕停止 B 泵时同步结束脚踏临时排空。 */
				pumpMessageB.timingDrainage_times = 0U;
				pumpMessageB.run_flag = false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 队列发送界面按钮和数字变黑
				// 队列发送泵停止设置数据（数据变为0）
			}
		}

		Pubinterface_RefreshPumpBDisplay(); /* B 泵启停后立即刷新；只有屏幕排空或脚踏轻踩来源高亮注水泵按钮。 */
		break;
	default:
		break; /* 非启停键不修改运行、排空或控制权状态。 */
	}
}

/*
 * 函数功能：处理屏幕和 HMI 的 A/B 泵加减速，按泵类型设置步进并限制在原上下限内。
 * 输入参数：key_value 为 A/B 泵加速或减速键值。
 * 返回参数：无。
 */
static void Pubinterface_HandlePumpSpeedKey(uint8_t key_value)
{
	uint16_t pump_speed_max = 0U; /* 当前泵类型允许的速度上限，防止加速越界。 */
	uint16_t pump_speed_min = 0U; /* 当前泵类型允许的速度下限，防止无符号减法下溢。 */

	switch (key_value)
	{
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
	default:
		break; /* 非加减速键不读取旧步进值，也不刷新错误的泵通道。 */
	}
}

/*
 * 函数功能：处理左右脚踏和 HMI 的轻排启停，保持注水泵选择顺序、控制权和双泵屏幕刷新不变。
 * 输入参数：key_value 为轻排启停键值，pump_owner 为该键对应的控制权来源。
 * 返回参数：无。
 */
static void Pubinterface_HandlePumpGentlyKey(uint8_t key_value, uint8_t pump_owner)
{
	switch (key_value)
	{
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
		break; /* 非轻排键不修改任何泵运行状态。 */
	}
}

/*
 * 函数功能：接收泵业务按键，先执行屏幕安全门禁，再按档位、启停、调速和轻排四类动作分派。
 * 输入参数：key_value 为脚踏、屏幕或 HMI 传入的泵控制键值。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value)
{
	uint8_t pump_owner = ControlArbitration_GetOwnerByPumpKey(key_value); /* 在业务分派前解析控制来源，保持原入口调用时机。 */

	if (Pubinterface_ShouldRejectScreenPumpKey(key_value))
	{
		return; /* 屏幕键未通过在线、报警或手柄占用门禁时，不进入任何泵动作处理。 */
	}

	switch (key_value)
	{
	case JTkey_left_short:
	case JTKey_right_short:
		Pubinterface_HandlePumpGearKey(key_value); /* 实体脚踏短按只循环对应通道档位。 */
		break;

	case JTKey_left_long:
	case HMIkey_APUMP_control:
	case SCREENKey_APUMP_control:
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		Pubinterface_HandlePumpRunKey(key_value, pump_owner); /* 启停和排空集中处理，保留 A/B 历史差异。 */
		break;

	case HMIkey_APUMP_Add:
	case SCREENKey_APUMP_Add:
	case HMIkey_APUMP_Sub:
	case SCREENKey_APUMP_Sub:
	case HMIkey_BPUMP_Add:
	case SCREENKey_BPUMP_Add:
	case HMIkey_BPUMP_Sub:
	case SCREENKey_BPUMP_Sub:
		Pubinterface_HandlePumpSpeedKey(key_value); /* 上位机和屏幕调速按对应通道统一进入限幅处理。 */
		break;

	case JTKey_Gently_left_start:
	case JTKey_Gently_left_stop:
	case JTKey_Gently_right_start:
	case JTKey_Gently_rigth_stop:
	case HMIkey_Gently_start:
	case HMIkey_Gently_stop:
		Pubinterface_HandlePumpGentlyKey(key_value, pump_owner); /* 轻排保持左右优先泵和 HMI 当前通道选择顺序。 */
		break;

	default:
		break; /* 非泵业务键保持原逻辑：直接返回且不改变任何状态。 */
	}
}
