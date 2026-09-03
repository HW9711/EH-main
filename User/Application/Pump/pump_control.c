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
static uint16_t Pump_GetSpeedMax(const pumpMessage_t *pump_message)
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
static uint16_t Pump_GetSpeedMin(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 防御空指针，下限按 0 处理可保证不会出现负速度。 */
	}

	return pump_message->speed_Min; /* 当前工程未初始化 speed_Min 时自然为 0，后续配置非零时可自动生效。 */
}

/*
 * 函数功能：按当前注水/灌注流量和本次调节方向取得分段步进值。
 * 输入参数：current_flow 为当前 speed_work 流量；increase 为 true 表示增加流量，false 表示减少流量。
 * 返回参数：0~30 区间返回 2，30~100 区间返回 5，100~200 区间返回 10，200 以上返回 20。
 */
static uint16_t Pump_GetWaterFlowStep(uint16_t current_flow, bool increase)
{
	if ((current_flow < 30U) || ((increase == false) && (current_flow == 30U)))
	{
		return 2U; /* 低于 30 始终用 2；恰好为 30 时只有减流量继续使用低区间步进。 */
	}
	if ((current_flow < 100U) || ((increase == false) && (current_flow == 100U)))
	{
		return 5U; /* 30~100 使用 5；边界 100 减流量时按即将进入的下一区间选择 5。 */
	}
	if ((current_flow < 200U) || ((increase == false) && (current_flow == 200U)))
	{
		return 10U; /* 100~200 使用 10；边界 200 减流量时仍按下方区间选择 10。 */
	}
	return 20U; /* 高于 200 使用 20；恰好为 200 时增加流量也从本档开始使用 20。 */
}


/*
 * 函数功能：按 EX8 屏幕交互表判断屏幕泵按键是否应被拒绝。
 * 输入参数：key_value 为屏幕解析后的 A/B 泵加、减、启停逻辑键值。
 * 返回参数：true 表示本次屏幕泵键不允许继续修改泵状态；false 表示可继续进入 PUMPActive 原有业务分支。
 */
static bool Pump_ShouldRejectScreenKey(uint8_t key_value)
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
static void Pump_HandleGearKey(uint8_t key_value)
{
	static uint8_t PumpA_Gear = 0; /* 保存 A 泵实体脚踏档位，保持与原 PUMPActive 内静态变量相同的跨周期状态。 */
	static uint8_t PumpB_Gear = 0; /* 保存 B 泵实体脚踏档位，左右泵继续各自独立循环。 */

	switch (key_value)
	{
	case JTkey_left_short:
		PumpA_Gear++;
		/* A 泵档位超过第五档后回到零档，使实体键保持 0~5 循环。 */
		if (PumpA_Gear > 5)
			PumpA_Gear = 0;
		/* 未识别 A 泵时清除旧档位并退出，避免把残留类型对应的速度写入未知设备。 */
		if (pumpMessageA.type == 0)
		{
			PumpA_Gear = 0; // 清除旧档位，后续刷新仍由现有界面链处理。
			return;
		}
		/* A 路识别为抽吸泵时按每档 3 单位换算工作速度。 */
		else if (pumpMessageA.type == DRAWWATER)
		{
			pumpMessageA.speed_work = 3 * PumpA_Gear; // 每一档位增加30ml水
		}
		/* A 路识别为注水泵时使用注水档位表，不能沿用抽吸泵倍率。 */
		else if (pumpMessageA.type == INJECTWATER)
		{
			/* 屏幕定时排空期间禁止实体键改速，避免固定排空输出被档位覆盖。 */
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
		/* A 路识别为灌注泵时使用 50 单位档位表，并保留第五档 300 的上限。 */
		else if (pumpMessageA.type == POURWATER)
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
		/* B 泵档位超过第五档后回到零档，使右实体键同样保持 0~5 循环。 */
		if (PumpB_Gear > 5)
			PumpB_Gear = 0;
		/* 未识别 B 泵时只清除旧档位，防止未知设备继承上一台泵的速度。 */
		if (pumpMessageB.type == 0)
		{
			PumpB_Gear = 0;
			// 队列通知界面暗黑
		}
		/* B 路识别为抽吸泵时按每档 3 单位换算工作速度。 */
		else if (pumpMessageB.type == DRAWWATER)
		{
			pumpMessageB.speed_work = 3 * PumpB_Gear; // 每一档位增加30ml水
		}
		/* B 路识别为注水泵时使用原注水档位表，保持右泵历史行为。 */
		else if (pumpMessageB.type == INJECTWATER)
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
		/* B 路识别为灌注泵时使用 50 单位档位表，并保留第五档 300 的上限。 */
		else if (pumpMessageB.type == POURWATER)
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
static void Pump_HandleRunKey(uint8_t key_value, uint8_t pump_owner)
{
	bool is_timed_drainage = false; /* 仅屏幕排空按钮置位；实体长按和 HMI 普通启泵继续使用 speed_work。 */

	switch (key_value)
	{
	case JTKey_left_long:
	case HMIkey_APUMP_control:
	case SCREENKey_APUMP_control:
		/* A 泵类型未识别时强制保持停止，并退出可能残留的本地控制权。 */
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
			/* A 泵当前停止时进入启动路径；已经运行时由对应 else 分支执行停止。 */
			if (!pumpMessageA.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				/* A 泵零速判断保留原控制结构；当前分支只有历史说明，不改写设定速度。 */
				if (pumpMessageA.speed_work == 0U)
				{
					//pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 按泵类型补启动速度，避免灌注/抽吸泵只置 run_flag 但 UART 输出 0 速。 */
				}
				is_timed_drainage = (key_value == SCREENKey_APUMP_control); /* A 注水泵只有屏幕排空按钮使用固定速度和10秒计时。 */
				pumpMessageA.run_flag = true;
				/* 只有注水泵需要区分屏幕定时排空和脚踏轻排来源。 */
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

		if (pumpMessageA.run_flag != false)
		{
			Pubinterface_RefreshPumpAButtonDisplay(); /* A 启动沿只刷新按钮，避免泵任务尚未发布实际速度时把 349 号零档图写入屏幕。 */
		}
		else
		{
			Pubinterface_RefreshPumpADisplay(); /* A 停止沿继续完整刷新，立即恢复设定流量、停止按钮和档位环。 */
		}
		break;
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		/* B 泵类型未识别时强制保持停止，并退出可能残留的本地控制权。 */
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
			/* B 泵当前停止时进入启动路径；已经运行时由对应 else 分支执行停止。 */
			if (!pumpMessageB.run_flag)
			{
				/* 只有 HMI 外控泵键会占用外控 owner；本地脚踏/屏幕泵键不占用手柄电机 owner。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				/* B 泵设定为零时补入该类型启动速度，确保运行标志对应真实非零输出。 */
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* B 泵按类型补启动速度，保证屏幕直接启动灌注泵时能实际输出。 */
				}
				is_timed_drainage = (key_value == SCREENKey_BPUMP_control); /* B 注水泵只有屏幕排空按钮使用固定速度和10秒计时。 */
				pumpMessageB.run_flag = true;
				/* 只有注水泵需要区分屏幕定时排空和脚踏轻排来源。 */
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

		if (pumpMessageB.run_flag != false)
		{
			Pubinterface_RefreshPumpBButtonDisplay(); /* B 启动沿只刷新按钮，避免泵任务尚未发布实际速度时把 249 号零档图写入屏幕。 */
		}
		else
		{
			Pubinterface_RefreshPumpBDisplay(); /* B 停止沿继续完整刷新，保持数值、按钮和档位环同步回到停止态。 */
		}
		break;
	default:
		break; /* 非启停键不修改运行、排空或控制权状态。 */
	}
}

/*
 * 函数功能：处理屏幕和 HMI 的 A/B 泵加减速，注水/灌注按当前流量分段设置步进并限制在原上下限内。
 * 输入参数：key_value 为 A/B 泵加速或减速键值。
 * 返回参数：无。
 */
static void Pump_HandleSpeedKey(uint8_t key_value)
{
	uint16_t pump_speed_max = 0U; /* 当前泵类型允许的速度上限，防止加速越界。 */
	uint16_t pump_speed_min = 0U; /* 当前泵类型允许的速度下限，防止无符号减法下溢。 */
	bool increase = false; /* 保存本次按键方向，供分段步进和最终加减运算使用同一判断。 */

	switch (key_value)
	{
	case HMIkey_APUMP_Add:
	case SCREENKey_APUMP_Add:
	case HMIkey_APUMP_Sub:
	case SCREENKey_APUMP_Sub:
		increase = (key_value == HMIkey_APUMP_Add || key_value == SCREENKey_APUMP_Add); /* A 加键为 true，减键为 false，边界步进按调节方向选择。 */
		switch (pumpMessageA.type)
		{
		case DRAWWATER: // 抽
			pumpMessageA.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			/* A 泵处于屏幕定时排空时拒绝调速，确保固定排空速度不被覆盖。 */
			if (pumpMessageA.timingDrainage_flag == true)
				return;
			pumpMessageA.speed_step_value = Pump_GetWaterFlowStep(pumpMessageA.speed_work, increase); /* A 注水泵按当前流量区间和加减方向选择 2/5/10/20 步进。 */
			break;
		case POURWATER: // 灌
			pumpMessageA.speed_step_value = Pump_GetWaterFlowStep(pumpMessageA.speed_work, increase); /* A 灌注泵复用同一分段规则，保持两类水泵调节手感一致。 */
			break;
		default:
			Pubinterface_RefreshPumpADisplay(); /* A 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pump_speed_max = Pump_GetSpeedMax(&pumpMessageA); /* A 泵调速上限按类型兜底，解决 speed_Max 未初始化导致加速回 0 的问题。 */
		pump_speed_min = Pump_GetSpeedMin(&pumpMessageA); /* A 泵调速下限来自配置，当前未配置时保持 0。 */
		/* A 泵加速键进入上限保护路径，其余同组键按减速路径处理。 */
		if (increase != false)
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
				pumpMessageA.speed_work = pump_speed_min; /* 距离下限不足一个步进时直接贴下限，避免无符号减法下溢。 */
			}
			else
			{
				pumpMessageA.speed_work -= pumpMessageA.speed_step_value; /* 正常范围内按当前泵类型步进降低速度。 */
			}
		}
		/* A 泵运行时进入历史队列通知占位；当前空分支不改变任何泵状态。 */
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
		increase = (key_value == HMIkey_BPUMP_Add || key_value == SCREENKey_BPUMP_Add); /* B 加键为 true，减键为 false，保证边界步进与 A 通道一致。 */
		switch (pumpMessageB.type)
		{
		case DRAWWATER: // 抽
			pumpMessageB.speed_step_value = 1;
			break;
		case INJECTWATER: // 注
			/* B 泵处于屏幕定时排空时拒绝调速，确保固定排空速度不被覆盖。 */
			if (pumpMessageB.timingDrainage_flag == true)

				return;
			pumpMessageB.speed_step_value = Pump_GetWaterFlowStep(pumpMessageB.speed_work, increase); /* B 注水泵按当前流量区间和加减方向选择 2/5/10/20 步进。 */
			break;
		case POURWATER: // 灌
			pumpMessageB.speed_step_value = Pump_GetWaterFlowStep(pumpMessageB.speed_work, increase); /* B 灌注泵复用同一分段规则，避免 A/B 调节行为分叉。 */
			break;
		default:
			Pubinterface_RefreshPumpBDisplay(); /* B 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pump_speed_max = Pump_GetSpeedMax(&pumpMessageB); /* B 泵调速上限按类型兜底，解决 speed_Max 未初始化导致屏幕加速仍显示 0 的问题。 */
		pump_speed_min = Pump_GetSpeedMin(&pumpMessageB); /* B 泵调速下限来自配置，当前未配置时保持 0。 */
		/* B 泵加速键进入上限保护路径，其余同组键按减速路径处理。 */
		if (increase != false)
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
				pumpMessageB.speed_work = pump_speed_min; /* 距离下限不足一个步进时直接贴下限，避免无符号减法下溢。 */
			}
			else
			{
				pumpMessageB.speed_work -= pumpMessageB.speed_step_value; /* 正常范围内按当前泵类型步进降低速度。 */
			}
		}

		/* B 泵运行时进入历史队列通知占位；当前空分支不改变任何泵状态。 */
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
static void Pump_HandleGentlyKey(uint8_t key_value, uint8_t pump_owner)
{
	switch (key_value)
	{
	case JTKey_Gently_left_start: // 其实可以判断手柄类型决定是否给与注水
		/* 本地脚踏轻排只控制泵，不占用手柄电机 owner；外控轻排仍需要外控授权。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		/* 左轻排优先选择 A 注水泵，保持左右脚踏与逻辑泵通道的首选对应关系。 */
		if (pumpMessageA.type == INJECTWATER)
		{
			/* 当前没有工作通道时才允许本地轻排，避免与手柄联动用泵重叠。 */
			if (!WorkMessage.channel_work)
			{
				// 队列发送启动A
				/* 脚踏轻排接管 A 泵前结束屏幕定时排空，避免两种停止条件同时生效。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 轻排启动也要补非零注水流量，避免只置运行标志但泵不转。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* 标记 A 当前属于轻排阶段，泵行为任务据此旁路压力报警与压力限速。 */
				pumpMessageA.run_flag = true;
			}
		}
		/* A 不是注水泵时回退选择 B 注水泵，保留左轻排的原选择顺序。 */
		else if (pumpMessageB.type == INJECTWATER)
		{

			/* 当前没有工作通道时才允许回退启动 B 泵，避免覆盖手柄联动状态。 */
			if (!WorkMessage.channel_work)
			{
				/* 脚踏轻排接管 B 泵前结束屏幕定时排空，避免计时器随后误停泵。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* B 注水泵轻排启动时补非零流量，保持和屏幕/外控启泵一致。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* 左轻排回退到 B 时记录实际排空泵，确保只旁路 B 的压力保护。 */
				pumpMessageB.run_flag = true;
				// 队列发送启动B
			}
		}
		Pubinterface_RefreshPumpADisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		Pubinterface_RefreshPumpBDisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		break;
	case JTKey_Gently_left_stop:
		/* 左轻排停止时优先停止 A 注水泵，与左侧启动选择顺序保持一致。 */
		if (pumpMessageA.type == INJECTWATER)
		{
			// 队列通知A泵停
			pumpMessageA.run_flag = false;
			pumpMessageA.pedalDrainage_flag = false; /* 左轻排停止时结束 A 排空旁路，下一次普通运行立即恢复压力保护。 */
			;
		}
		/* A 不是注水泵时停止回退使用的 B 注水泵。 */
		else if (pumpMessageB.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageB.run_flag = false;
			pumpMessageB.pedalDrainage_flag = false; /* 左轻排回退泵停止时同步恢复 B 的正常压力保护。 */
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
		/* 右轻排优先选择 B 注水泵，保持右脚踏与逻辑 B 通道的首选对应关系。 */
		if (pumpMessageB.type == INJECTWATER)
		{
			/* 当前没有工作通道时才允许本地轻排，避免与手柄联动用泵重叠。 */
			if (!WorkMessage.channel_work)
			{
				// 队列发送启动B
				/* 脚踏轻排接管 B 泵前结束屏幕定时排空，避免两种来源互相覆盖。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* 右轻排启动 B 注水泵时补非零流量，避免 0 速不转。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* 标记 B 当前属于右轻排阶段，只在该排空阶段旁路压力保护。 */
				pumpMessageB.run_flag = true;
			}
		}
		/* B 不是注水泵时回退选择 A 注水泵，保留右轻排的原选择顺序。 */
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列发送
			/* 当前没有工作通道时才允许回退启动 A 泵，避免覆盖手柄联动状态。 */
			if (!WorkMessage.channel_work)
			{
				// 队列发送启动A
				/* 脚踏轻排接管 A 泵前结束屏幕定时排空，避免计时器随后误停泵。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 右轻排回退启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* 右轻排回退到 A 时记录实际排空泵，不能误旁路 B 的压力保护。 */
				pumpMessageA.run_flag = true;
			}
		}

		Pubinterface_RefreshPumpADisplay(); /* 右轻排可能影响 A 或 B 注水泵，两个泵区一起刷新。 */
		Pubinterface_RefreshPumpBDisplay(); /* 右轻排可能影响 A 或 B 注水泵，两个泵区一起刷新。 */
		break;
	case JTKey_Gently_rigth_stop:
		/* 右轻排停止时优先停止 B 注水泵，与右侧启动选择顺序保持一致。 */
		if (pumpMessageB.type == INJECTWATER)
		{
			// 队列通知A泵停
			pumpMessageB.run_flag = false;
			pumpMessageB.pedalDrainage_flag = false; /* 右轻排停止时结束 B 排空旁路，恢复普通运行压力检测。 */
			;
		}
		/* B 不是注水泵时停止回退使用的 A 注水泵。 */
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 队列通知B泵停
			pumpMessageA.run_flag = false;
			pumpMessageA.pedalDrainage_flag = false; /* 右轻排回退泵停止时同步恢复 A 的正常压力保护。 */
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

		/* 当前工作通道为 A 时，HMI 轻排优先选择 A 注水泵。 */
		if (WorkMessage.channel_work == 1)
		{
			/* A 本身是注水泵时直接使用 A，保持 HMI 当前通道优先。 */
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				/* HMI 轻排接管 A 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 轻排启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* HMI 轻排也属于排空阶段，复用逐泵标志旁路 A 压力保护。 */
				pumpMessageA.run_flag = true;
			}
			/* A 不是注水泵时回退选择 B 注水泵，保证轻排仍有可用水泵。 */
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				/* HMI 轻排接管 B 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 轻排启动 B 注水泵时补非零流量。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* HMI 从 A 通道回退到 B 时只标记 B 为排空压力旁路。 */
				pumpMessageB.run_flag = true;
			}
		}
		/* 当前工作通道为 B 时，HMI 轻排优先选择 B 注水泵。 */
		else if (WorkMessage.channel_work == 2)
		{
			/* B 本身是注水泵时直接使用 B，保持 HMI 当前通道优先。 */
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送启动B
				/* HMI 轻排接管 B 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 当前 B 通道轻排时补 B 注水泵启动流量。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* HMI 当前 B 通道轻排时标记 B 排空，压力保护仅在退出后恢复。 */
				pumpMessageB.run_flag = true;
			}
			/* B 不是注水泵时回退选择 A 注水泵，保证轻排仍有可用水泵。 */
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送启动A
				/* HMI 轻排接管 A 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 当前 B 通道但 A 是注水泵时补 A 启动流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* HMI 从 B 通道回退到 A 时只标记 A 为排空压力旁路。 */
				pumpMessageA.run_flag = true;
			}
		}

		Pubinterface_RefreshPumpADisplay(); /* HMI 轻排启动后刷新 A 泵显示，保持上位机和屏幕状态一致。 */
		Pubinterface_RefreshPumpBDisplay(); /* HMI 轻排启动后刷新 B 泵显示，保持上位机和屏幕状态一致。 */
		break;
	case HMIkey_Gently_stop:
		/* 当前工作通道为 A 时按启动时的 A 优先顺序停止注水泵。 */
		if (WorkMessage.channel_work == 1)
		{
			/* A 为注水泵时停止 A，避免错误关闭另一通道。 */
			if (pumpMessageA.type == INJECTWATER)
			{
				// 队列发送停止A
				pumpMessageA.run_flag = false;
				pumpMessageA.pedalDrainage_flag = false; /* HMI 轻排停止时结束 A 排空旁路，恢复后续普通注水压力保护。 */
			}
			/* A 不是注水泵时停止启动阶段回退使用的 B 注水泵。 */
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* HMI 回退 B 停止时同步清除 B 排空压力旁路。 */
			}
		}
		/* 当前工作通道为 B 时按启动时的 B 优先顺序停止注水泵。 */
		else if (WorkMessage.channel_work == 2)
		{
			/* B 为注水泵时停止 B，避免错误关闭另一通道。 */
			if (pumpMessageB.type == INJECTWATER)
			{
				// 队列发送停止B
				pumpMessageB.run_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* HMI 当前 B 通道停止轻排时恢复 B 压力保护。 */
			}
			/* B 不是注水泵时停止启动阶段回退使用的 A 注水泵。 */
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 队列停止B
				pumpMessageA.run_flag = false;
				pumpMessageA.pedalDrainage_flag = false; /* HMI 回退 A 停止时同步清除 A 排空压力旁路。 */
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

	if (Pump_ShouldRejectScreenKey(key_value))
	{
		return; /* 屏幕键未通过在线、报警或手柄占用门禁时，不进入任何泵动作处理。 */
	}

	switch (key_value)
	{
	case JTkey_left_short:
	case JTKey_right_short:
		Pump_HandleGearKey(key_value); /* 实体脚踏短按只循环对应通道档位。 */
		break;

	case JTKey_left_long:
	case HMIkey_APUMP_control:
	case SCREENKey_APUMP_control:
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		Pump_HandleRunKey(key_value, pump_owner); /* 启停和排空集中处理，保留 A/B 历史差异。 */
		break;

	case HMIkey_APUMP_Add:
	case SCREENKey_APUMP_Add:
	case HMIkey_APUMP_Sub:
	case SCREENKey_APUMP_Sub:
	case HMIkey_BPUMP_Add:
	case SCREENKey_BPUMP_Add:
	case HMIkey_BPUMP_Sub:
	case SCREENKey_BPUMP_Sub:
		Pump_HandleSpeedKey(key_value); /* 上位机和屏幕调速按对应通道统一进入限幅处理。 */
		break;

	case JTKey_Gently_left_start:
	case JTKey_Gently_left_stop:
	case JTKey_Gently_right_start:
	case JTKey_Gently_rigth_stop:
	case HMIkey_Gently_start:
	case HMIkey_Gently_stop:
		Pump_HandleGentlyKey(key_value, pump_owner); /* 轻排保持左右优先泵和 HMI 当前通道选择顺序。 */
		break;

	default:
		break; /* 非泵业务键保持原逻辑：直接返回且不改变任何状态。 */
	}
}
