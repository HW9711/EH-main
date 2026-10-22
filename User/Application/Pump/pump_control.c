#include "pump_control.h"

#include "Pubinterface.h"
#include "pump.h"
#include "pump_behavior_core.h" /* 启动与失败恢复必须以驱动真实定位状态为准，不能只看设备类型。 */
#include "sscUIDP.h"

/* 抽吸泵内部设定上限，无流量单位，允许 0~15；改值还须同步核对泵任务中的 15 上限及 42 倍协议换算。 */
#define PUMP_DRAWWATER_SPEED_MAX 15U
/* 灌注泵设定流量上限，单位 mL/min，允许 0~300；改值还须核对泵任务中的 300 上限和高流量换算。 */
#define PUMP_POURWATER_SPEED_MAX 300U
/* 抽吸泵设定为 0 时使用的启动值，单位为内部设定，须在 1~15；改值会改变调用补速函数时的起始输出。 */
#define PUMP_DRAWWATER_START_SPEED 3U
/* 灌注泵设定为0时使用接入默认流量补速，单位mL/min；不改变已有非零设定。 */
#define PUMP_POURWATER_START_SPEED PUMP_POURWATER_DEFAULT_FLOW

/*
 * 函数功能：当泵设定为 0 时，按泵类型取得一个非零启动设定，供调用方写入 speed_work。
 * 输入参数：pump_message 指向 A/B 泵运行状态，函数只读取 type 字段判断泵类型。
 * 返回参数：有效泵类型返回非零启动速度；无效类型或空指针返回 0。
 */
uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message)
{
	if (pump_message == NULL)
	{
		return 0U; /* 没有泵状态就不能确定类型，返回零，避免误给未知设备设置启动速度。 */
	}

	switch (pump_message->type)
	{
	case DRAWWATER:
		return PUMP_DRAWWATER_START_SPEED; /* 抽吸泵使用内部设定 3，避免只显示运行却下发零速。 */
	case INJECTWATER:
		return Pubinterface_GetInjectionPumpStartFlow(); /* 注水泵沿用当前通道 Page4 默认流量，非法值回退 30。 */
	case POURWATER:
		return PUMP_POURWATER_START_SPEED; /* 灌注泵零流量启动时补为120mL/min，与本次开机首次识别的默认值一致。 */
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
		return 0U; /* 没有泵状态就不允许加速，返回零上限。 */
	}

	if (pump_message->speed_Max != 0U)
	{
		if ((pump_message->type == INJECTWATER) && (pump_message->speed_Max > PUMP_INJECTWATER_SPEED_MAX))
		{
			return PUMP_INJECTWATER_SPEED_MAX; /* 配置上限超过70时仍限制注水流量，防止屏幕和脚踏绕过统一上限。 */
		}
		return pump_message->speed_Max; /* 若后续 EEPROM 或参数表配置了上限，则优先服从配置值。 */
	}

	switch (pump_message->type)
	{
	case DRAWWATER:
		return PUMP_DRAWWATER_SPEED_MAX; /* 抽吸泵内部设定上限为 15，与泵任务中的限制一致。 */
	case INJECTWATER:
		return PUMP_INJECTWATER_SPEED_MAX; /* 注水泵上限与 pump.h 中统一宏一致。 */
	case POURWATER:
		return PUMP_POURWATER_SPEED_MAX; /* 灌注泵最大流量为 300 mL/min，与泵任务中的限制一致。 */
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
		return 0U; /* 没有泵状态时按最低设定为零处理。 */
	}

	return pump_message->speed_Min; /* 当前工程未初始化 speed_Min 时自然为 0，后续配置非零时可自动生效。 */
}

/*
 * 函数功能：按当前注水流量和本次调节方向取得分段步进值。
 * 输入参数：current_flow 为当前 speed_work 流量，单位 mL/min；increase 为 true 表示增加，false 表示减少。
 * 返回参数：每次调整 2、5、10 或 20 mL/min；在 30/100/200 分界值，加键用上一区间的步长，减键用下一区间的步长。
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
 * 函数功能：按灌注流量和调节方向选择屏幕/HMI加减步进，跨越140时只调整到分界值。
 * 输入参数：current_flow为当前流量，单位mL/min；increase为true表示增加，false表示减少。
 * 返回参数：本次调整量，通常为10或20mL/min；距140不足一步时返回到140的差值。
 */
static uint16_t Pump_GetPourWaterFlowStep(uint16_t current_flow, bool increase)
{
	if (increase != false)
	{
		if (current_flow < 140U) /* 低流量区每次加10，不能从135等非整档值跳过140。 */
		{
			return (current_flow > 130U) ? (uint16_t)(140U - current_flow) : 10U; /* 剩余不足10时先到140，下一次再进入高区间。 */
		}
		return 20U; /* 从140继续向上进入高流量区，每次加20，调用方限制最高300。 */
	}
	if (current_flow > 140U) /* 高流量区每次减20，接近分界时先减到140。 */
	{
		return (current_flow < 160U) ? (uint16_t)(current_flow - 140U) : 20U; /* 剩余不足20时先到140，避免一次减入低区间。 */
	}
	return 10U; /* 从140向下使用低流量区步进，调用方限制最低0。 */
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
 * 函数功能：处理泵启停和排空按键。注水泵忽略脚踏按钮长按，不执行启动或停止。驱动未就绪时拒绝启动；定位失败后按屏幕泵键只重新定位，成功后仍须再按一次才能运行。
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
		if ((key_value == JTKey_left_long) && (pumpMessageA.type == INJECTWATER))
		{
			return; /* A注水泵忽略脚踏按钮长按，不改变运行、排空和流量设定；屏幕及HMI启停继续按原逻辑处理。 */
		}
		/* A 泵类型未识别时强制保持停止，并退出可能残留的本地控制权。 */
		if (pumpMessageA.type == 0)
		{
			pumpMessageA.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			Pubinterface_RefreshPumpADisplay(); /* A 泵未识别时立即暗掉屏幕 A 泵区域和按钮，避免显示可启动。 */
			// 已清运行标志，A 泵周期任务会按停止状态发送零速。
			return;
		}
		else
		{
			/* A 泵当前停止时进入启动路径；已经运行时由对应 else 分支执行停止。 */
			if (!pumpMessageA.run_flag)
			{
				if (PumpBehavior_DriverCanRun(PUMP_BEHAVIOR_CHANNEL_A) == 0U)
				{
					if (key_value == SCREENKey_APUMP_control)
					{
						(void)PumpBehavior_RequestRealign(PUMP_BEHAVIOR_CHANNEL_A); /* 仅定位失败时申请重新定位，不启动泵；正在定位时按键无效。 */
					}
					Pubinterface_RefreshPumpADisplay(); /* 本次不启动A泵；周期任务根据驱动反馈更新青色定位或红色失败状态。 */
					return; /* 本次按键不保存任何运行/排空请求，完成后必须有新的启动动作。 */
				}
				/* HMI 启泵先申请外控权限；本地脚踏和屏幕只控制泵，不占用手柄电机的控制权限。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				/* A 泵当前零速分支为空，下面的补速语句已注释，不会自动给普通启动补入非零设定。 */
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
				// 本函数末尾更新 A 按钮，A 泵周期任务根据上述状态计算输出。
			}
			else
			{
				pumpMessageA.run_flag = false;
				pumpMessageA.timingDrainage_flag = false;
				pumpMessageA.pedalDrainage_flag = false; /* 屏幕停止 A 泵时同步结束所有排空来源。 */
				pumpMessageA.timingDrainage_times = 0U;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 本函数末尾刷新 A 停止显示，A 泵周期任务负责发送零速。
			}
		}

		if (pumpMessageA.run_flag != false)
		{
			Pubinterface_RefreshPumpAButtonDisplay(); /* 启动 A 后先只刷新按钮；泵任务尚未更新输出设定，立即刷新档位会误显示 349 号零档图。 */
		}
		else
		{
			Pubinterface_RefreshPumpADisplay(); /* 停止 A 后刷新整个泵区，恢复设定流量、停止按钮和档位环。 */
		}
		break;
	case JTKey_right_long:
	case SCREENKey_BPUMP_control:
	case HMIkey_BPUMP_control:
		if ((key_value == JTKey_right_long) && (pumpMessageB.type == INJECTWATER))
		{
			return; /* B注水泵忽略脚踏按钮长按，不改变运行、排空和流量设定；屏幕及HMI启停继续按原逻辑处理。 */
		}
		/* B 泵类型未识别时强制保持停止，并退出可能残留的本地控制权。 */
		if (pumpMessageB.type == 0)
		{
			pumpMessageB.run_flag = false;
			ControlArbitration_ExitLocalControlIfIdle(pump_owner);
			Pubinterface_RefreshPumpBDisplay(); /* B 泵未识别时立即暗掉屏幕 B 泵区域和按钮，避免显示可启动。 */
			// 已清运行标志，B 泵周期任务会按停止状态发送零速。
			return;
		}
		else
		{
			/* B 泵当前停止时进入启动路径；已经运行时由对应 else 分支执行停止。 */
			if (!pumpMessageB.run_flag)
			{
				if (PumpBehavior_DriverCanRun(PUMP_BEHAVIOR_CHANNEL_B) == 0U)
				{
					if (key_value == SCREENKey_BPUMP_control)
					{
						(void)PumpBehavior_RequestRealign(PUMP_BEHAVIOR_CHANNEL_B); /* B通道独立登记校准，不复用非零速度，也不影响A泵。 */
					}
					Pubinterface_RefreshPumpBDisplay(); /* 失败键只恢复定位，屏幕按钮仍处于停止状态。 */
					return; /* 脚踏/HMI普通启泵不能触发重定位，任何来源均不缓存提前启动。 */
				}
				/* HMI 启泵先申请外控权限；本地脚踏和屏幕只控制泵，不占用手柄电机的控制权限。 */
				if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
					return;
				/* B 泵设定为零时补入该类型启动速度，确保运行标志对应真实非零输出。 */
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* B 泵按类型补非零设定，避免运行标志已置位却仍下发零速。 */
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
				// 本函数末尾更新 B 按钮，B 泵周期任务根据上述状态计算输出。
			}
			else
			{
				pumpMessageB.timingDrainage_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* 屏幕停止 B 泵时同步结束脚踏临时排空。 */
				pumpMessageB.timingDrainage_times = 0U;
				pumpMessageB.run_flag = false;
				ControlArbitration_ExitLocalControlIfIdle(pump_owner);
				// 本函数末尾刷新 B 停止显示，B 泵周期任务负责发送零速。
			}
		}

		if (pumpMessageB.run_flag != false)
		{
			Pubinterface_RefreshPumpBButtonDisplay(); /* 启动 B 后先只刷新按钮；泵任务尚未更新输出设定，立即刷新档位会误显示 249 号零档图。 */
		}
		else
		{
			Pubinterface_RefreshPumpBDisplay(); /* 停止 B 后刷新整个泵区，让数值、按钮和档位环一起显示停止状态。 */
		}
		break;
	default:
		break; /* 非启停键不修改运行、排空或控制权状态。 */
	}
}

/*
 * 函数功能：处理A/B泵屏幕和HMI加减速；按泵类型与当前设定选择步进并限幅，同时重置对应泵的脚踏三档循环。
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
			pumpMessageA.speed_step_value = Pump_GetPourWaterFlowStep(pumpMessageA.speed_work, increase); /* 灌注泵在140以下加减10，140以上加减20，跨区时先到140。 */
			break;
		default:
			Pubinterface_RefreshPumpADisplay(); /* A 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pumpMessageA.step_value = 0U; /* 屏幕或HMI已调整流量，下次脚踏短按从首档重新开始。 */
		pump_speed_max = Pump_GetSpeedMax(&pumpMessageA); /* 优先使用 A 的 speed_Max；为 0 时按泵类型取上限，避免加速后反而被清零。 */
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
		/* 此判断内没有有效语句；A 泵周期任务直接读取上面更新的速度设定。 */
		if (pumpMessageA.run_flag == true)
		{
			// 当前不发送队列消息。
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
			pumpMessageB.speed_step_value = Pump_GetPourWaterFlowStep(pumpMessageB.speed_work, increase); /* 灌注泵在140以下加减10，140以上加减20，跨区时先到140。 */
			break;
		default:
			Pubinterface_RefreshPumpBDisplay(); /* B 泵类型无效时只刷新为不可用状态，不沿用上一次步进值误改速度。 */
			return;
		}
		pumpMessageB.step_value = 0U; /* 屏幕或HMI已调整流量，下次脚踏短按从首档重新开始。 */
		pump_speed_max = Pump_GetSpeedMax(&pumpMessageB); /* 优先使用 B 的 speed_Max；为 0 时按泵类型取上限，避免加速后反而被清零。 */
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

		/* 此判断内没有有效语句；B 泵周期任务直接读取上面更新的速度设定。 */
		if (pumpMessageB.run_flag == true)
		{
			// 当前不发送队列消息。
		}
		Pubinterface_RefreshPumpBDisplay(); /* B 泵加减速后同步刷新流量数值和按钮状态。 */
		break;
	default:
		break; /* 非加减速键不读取旧步进值，也不刷新错误的泵通道。 */
	}
}

/*
 * 函数功能：脚踏短按循环调整对应泵流量；注水保持20/25/30，仅右侧B泵灌注模式使用90/120/180，首次从首档开始。
 * 输入参数：key_value为左或右脚踏短按键值，分别调整A泵或B泵。
 * 返回参数：无；只改变流量和脚踏档位，不改变启停或排空请求。
 */
static void Pump_HandleGearKey(uint8_t key_value)
{
	pumpMessage_t *pump_message; /* 指向本次按键对应的泵，左右按钮分别保存循环档位。 */
	if (key_value == JTkey_left_short)
	{
		pump_message = &pumpMessageA; /* 左按钮只调整左侧A泵。 */
	}
	else if (key_value == JTKey_right_short)
	{
		pump_message = &pumpMessageB; /* 右按钮只调整右侧B泵。 */
	}
	else
	{
		return; /* 不是左右短按，不修改任何泵的流量或循环档位。 */
	}
	if ((pump_message->online_flag == false) ||
		((pump_message->type != INJECTWATER) && (pump_message->type != POURWATER)))
	{
		return; /* 没有识别到在线注水或灌注泵时，不记录本次按键；接入后仍从首档开始。 */
	}
	if ((pump_message->type == POURWATER) && (key_value != JTKey_right_short))
	{
		return; /* 灌注快捷档只允许右侧B泵使用；左侧A固定注水，不推进灌注档位或改写流量。 */
	}
	if ((pump_message->type == INJECTWATER) && (pump_message->timingDrainage_flag == true))
	{
		return; /* 屏幕定时排空期间不改流量或推进档位，排空结束后才能继续调节。 */
	}
	pump_message->step_value = (pump_message->step_value >= 3U) ? 1U : (uint8_t)(pump_message->step_value + 1U); /* 0首次进入1；1到2、2到3、3回1，第三档之后不回零。 */
	if (pump_message->type == INJECTWATER)
	{
		pump_message->speed_work = 20U + (uint16_t)(pump_message->step_value - 1U) * 5U; /* 注水三档固定20、25、30，不受当前屏幕流量影响。 */
		pump_message->speed_step_value = 5U; /* 诊断记录注水相邻脚踏档位相差5mL/min。 */
	}
	else
	{
		static const uint16_t pour_flow_gears[3] = {90U, 120U, 180U}; /* 右侧灌注三档间隔不同，按档位查表，第三档后回到90mL/min。 */
		uint16_t previous_flow = pump_message->speed_work; /* 保留切档前设定，诊断调节量不能继续沿用固定50mL/min。 */
		pump_message->speed_work = pour_flow_gears[pump_message->step_value - 1U]; /* 仅右侧B泵灌注分支使用该表，注水仍走20/25/30。 */
		pump_message->speed_step_value = (pump_message->speed_work >= previous_flow) ?
			(pump_message->speed_work - previous_flow) : (previous_flow - pump_message->speed_work); /* 记录本次实际调节量，兼容非等距档位及180回90。 */
	}
	if (key_value == JTkey_left_short)
	{
		Pubinterface_RefreshPumpADisplay(); /* 刷新A泵设定和流量进度，不发送启停请求。 */
	}
	else
	{
		Pubinterface_RefreshPumpBDisplay(); /* 刷新B泵设定和流量进度，不影响A泵。 */
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
	case JTKey_Gently_left_start: // 左脚踏轻排：先找 A 注水泵，没有时再找 B。
		/* 本地脚踏轻排只控制泵，不占用手柄电机控制权限；外控轻排仍需取得外控权限。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		/* 左轻排优先选择 A 注水泵，保持左右脚踏与逻辑泵通道的首选对应关系。 */
		if (pumpMessageA.type == INJECTWATER)
		{
			/* 当前没有工作通道时才允许本地轻排，避免与手柄联动用泵重叠。 */
			if (!WorkMessage.channel_work)
			{
				// 更新 A 泵状态后，由 A 泵周期任务发送运行命令。
				/* 脚踏轻排接管 A 泵前结束屏幕定时排空，避免两种停止条件同时生效。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 轻排启动也要补非零注水流量，避免只置运行标志但泵不转。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* A 进入轻排；泵任务继续用统一 800g 阈值监测压力并提示。 */
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
				pumpMessageB.pedalDrainage_flag = true; /* 左轻排实际使用 B；B 泵任务仍按统一 800g 阈值监测压力。 */
				pumpMessageB.run_flag = true;
				// B 泵周期任务会按新的轻排状态发送运行命令。
			}
		}
		Pubinterface_RefreshPumpADisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		Pubinterface_RefreshPumpBDisplay(); /* 轻排可能启动 A 或 B 注水泵，两个泵区都刷新以避免状态残留。 */
		break;
	case JTKey_Gently_left_stop:
		/* 左轻排停止时优先停止 A 注水泵，与左侧启动选择顺序保持一致。 */
		if (pumpMessageA.type == INJECTWATER)
		{
			// 清除 A 的运行和轻排标志，使 A 泵周期任务发送零速。
			pumpMessageA.run_flag = false;
			pumpMessageA.pedalDrainage_flag = false; /* 结束 A 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			;
		}
		/* A 不是注水泵时停止回退使用的 B 注水泵。 */
		else if (pumpMessageB.type == INJECTWATER)
		{
			// 清除 B 的运行和轻排标志，使 B 泵周期任务发送零速。
			pumpMessageB.run_flag = false;
			pumpMessageB.pedalDrainage_flag = false; /* 结束 B 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		Pubinterface_RefreshPumpADisplay(); /* 轻排停止后同步 A 泵显示，避免按钮仍保持运行态。 */
		Pubinterface_RefreshPumpBDisplay(); /* 轻排停止后同步 B 泵显示，避免按钮仍保持运行态。 */
		break;
	case JTKey_Gently_right_start:
		/* 本地脚踏轻排只控制泵，不占用手柄电机控制权限；外控轻排仍需取得外控权限。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;
		/* 右轻排优先选择 B 注水泵，保持右脚踏与逻辑 B 通道的首选对应关系。 */
		if (pumpMessageB.type == INJECTWATER)
		{
			/* 当前没有工作通道时才允许本地轻排，避免与手柄联动用泵重叠。 */
			if (!WorkMessage.channel_work)
			{
				// 更新 B 泵状态后，由 B 泵周期任务发送运行命令。
				/* 脚踏轻排接管 B 泵前结束屏幕定时排空，避免两种来源互相覆盖。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* 右轻排启动 B 注水泵时补非零流量，避免 0 速不转。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* B 进入右轻排，该阶段不检查超压报警和压力板是否就绪。 */
				pumpMessageB.run_flag = true;
			}
		}
		/* B 不是注水泵时回退选择 A 注水泵，保留右轻排的原选择顺序。 */
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 右侧没有 B 注水泵，改用 A 注水泵执行轻排。
			/* 当前没有工作通道时才允许回退启动 A 泵，避免覆盖手柄联动状态。 */
			if (!WorkMessage.channel_work)
			{
				// 更新 A 泵状态后，由 A 泵周期任务发送运行命令。
				/* 脚踏轻排接管 A 泵前结束屏幕定时排空，避免计时器随后误停泵。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* 右轻排回退启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* 右轻排实际使用 A；A 泵任务仍按统一 800g 阈值监测压力。 */
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
			// 本次停止的是 B 泵，清除其运行和轻排标志。
			pumpMessageB.run_flag = false;
			pumpMessageB.pedalDrainage_flag = false; /* 结束 B 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			;
		}
		/* B 不是注水泵时停止回退使用的 A 注水泵。 */
		else if (pumpMessageA.type == INJECTWATER)
		{
			// 本次停止的是备用 A 泵，清除其运行和轻排标志。
			pumpMessageA.run_flag = false;
			pumpMessageA.pedalDrainage_flag = false; /* 结束 A 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			;
		}
		ControlArbitration_ExitLocalControlIfIdle(pump_owner);
		Pubinterface_RefreshPumpADisplay(); /* 右轻排停止后同步 A 泵显示，避免按钮仍保持运行态。 */
		Pubinterface_RefreshPumpBDisplay(); /* 右轻排停止后同步 B 泵显示，避免按钮仍保持运行态。 */
		break;
	case HMIkey_Gently_start:
		/* HMI 轻排是外部控制请求，未取得控制权限就不能启动。 */
		if ((pump_owner != CONTROL_OWNER_NONE) && (ControlArbitration_TryEnter(pump_owner) == false))
			return;

		/* 当前工作通道为 A 时，HMI 轻排优先选择 A 注水泵。 */
		if (WorkMessage.channel_work == 1)
		{
			/* A 本身是注水泵时直接使用 A，保持 HMI 当前通道优先。 */
			if (pumpMessageA.type == INJECTWATER)
			{
				// 通过运行和轻排标志请求 A 泵启动，不在这里直接写串口。
				/* HMI 轻排接管 A 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 轻排启动 A 注水泵时补非零流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* HMI 让 A 进入轻排，该阶段不检查超压报警和压力板是否就绪。 */
				pumpMessageA.run_flag = true;
			}
			/* A 不是注水泵时回退选择 B 注水泵，保证轻排仍有可用水泵。 */
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 通过运行和轻排标志请求 B 泵启动，不在这里直接写串口。
				/* HMI 轻排接管 B 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 轻排启动 B 注水泵时补非零流量。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* HMI 实际使用 B 轻排；B 泵任务仍按统一 800g 阈值监测压力。 */
				pumpMessageB.run_flag = true;
			}
		}
		/* 当前工作通道为 B 时，HMI 轻排优先选择 B 注水泵。 */
		else if (WorkMessage.channel_work == 2)
		{
			/* B 本身是注水泵时直接使用 B，保持 HMI 当前通道优先。 */
			if (pumpMessageB.type == INJECTWATER)
			{
				// 通过运行和轻排标志请求 B 泵启动，不在这里直接写串口。
				/* HMI 轻排接管 B 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageB.timingDrainage_flag == true)
				{
					pumpMessageB.timingDrainage_flag = false;
				}
				if (pumpMessageB.speed_work == 0U)
				{
					pumpMessageB.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageB); /* HMI 当前 B 通道轻排时补 B 注水泵启动流量。 */
				}
				pumpMessageB.pedalDrainage_flag = true; /* HMI 让 B 进入轻排，该阶段不检查超压报警和压力板是否就绪。 */
				pumpMessageB.run_flag = true;
			}
			/* B 不是注水泵时回退选择 A 注水泵，保证轻排仍有可用水泵。 */
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 通过运行和轻排标志请求 A 泵启动，不在这里直接写串口。
				/* HMI 轻排接管 A 泵前结束屏幕定时排空，避免 10 秒计时误停。 */
				if (pumpMessageA.timingDrainage_flag == true)
				{
					pumpMessageA.timingDrainage_flag = false;
				}
				if (pumpMessageA.speed_work == 0U)
				{
					pumpMessageA.speed_work = Pubinterface_GetPumpStartSpeed(&pumpMessageA); /* HMI 当前 B 通道但 A 是注水泵时补 A 启动流量。 */
				}
				pumpMessageA.pedalDrainage_flag = true; /* HMI 实际使用 A 轻排；A 泵任务仍按统一 800g 阈值监测压力。 */
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
				// 清除 A 的运行和轻排标志，由周期任务发送零速。
				pumpMessageA.run_flag = false;
				pumpMessageA.pedalDrainage_flag = false; /* 结束 A 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			}
			/* A 不是注水泵时停止启动阶段回退使用的 B 注水泵。 */
			else if (pumpMessageB.type == INJECTWATER)
			{
				// 清除 B 的运行和轻排标志，由周期任务发送零速。
				pumpMessageB.run_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* 结束 B 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			}
		}
		/* 当前工作通道为 B 时按启动时的 B 优先顺序停止注水泵。 */
		else if (WorkMessage.channel_work == 2)
		{
			/* B 为注水泵时停止 B，避免错误关闭另一通道。 */
			if (pumpMessageB.type == INJECTWATER)
			{
				// 清除 B 的运行和轻排标志，由周期任务发送零速。
				pumpMessageB.run_flag = false;
				pumpMessageB.pedalDrainage_flag = false; /* 结束 B 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
			}
			/* B 不是注水泵时停止启动阶段回退使用的 A 注水泵。 */
			else if (pumpMessageA.type == INJECTWATER)
			{
				// 此处实际停止备用 A 泵，不改变 B 泵状态。
				pumpMessageA.run_flag = false;
				pumpMessageA.pedalDrainage_flag = false; /* 结束 A 轻排，下次普通运行重新检查超压报警和压力板是否就绪。 */
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
 * 函数功能：先判断当前状态是否允许屏幕泵按键，再分别处理档位、启停、调速或轻排动作。
 * 输入参数：key_value 为脚踏、屏幕或 HMI 传入的泵控制键值。
 * 返回参数：无。
 */
void PUMPActive(uint8_t key_value)
{
	uint8_t pump_owner = ControlArbitration_GetOwnerByPumpKey(key_value); /* 在业务分派前解析控制来源，保持原入口调用时机。 */

	if (Pump_ShouldRejectScreenKey(key_value))
	{
		return; /* 屏幕按键因泵离线、报警、排空或手柄占用而被拒绝时，不再执行泵动作。 */
	}

	switch (key_value)
	{
	case JTkey_left_short:
	case JTKey_right_short:
		Pump_HandleGearKey(key_value); /* 左右短按分别循环对应泵的三个固定流量档，不参与启停。 */
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
