#include "handle_control.h"

#include "Pubinterface.h"
#include "handlescan.h"
#include "motoruartdata.h"
#include "sscBEEP.h"
#include "sscDRIVE.h"
#include "sscRFID.h"
#include "sscUIDP.h"

static volatile uint32_t s_handle_identity_generation_a = 0U; /* A 通道每次确认插入或拔出时加 1；分多轮读取 EEPROM 的任务用它判断手柄是否已换过。 */
static volatile uint32_t s_handle_identity_generation_b = 0U; /* B 通道单独记录插拔次数；A 的插拔不会终止正在读取 B 数据的任务。 */
static uint8_t s_running_unplug_recovery_channel = CHANNEL_NONE; /* 记住运行中拔出当前手柄时，另一路在线的通道；未停机或报警未解除时继续等待，不立即切换。 */

/*
 * 函数功能：指定通道确认插入或拔出时，增加一次变化计数，让旧手柄的数据读取任务停止继续处理。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void Handle_AdvanceIdentityGeneration(uint8_t channel)
{
	if (channel == CHANNEL_A) /* A 插拔只更新 A 的计数，不影响 B 的数据读取。 */
	{
		++s_handle_identity_generation_a; /* 计数加 1；即使从最大值回到 0，也与上一次不同，调用方可发现刚发生的插拔。 */
	}
	else if (channel == CHANNEL_B) /* B 状态变化使用独立计数。 */
	{
		++s_handle_identity_generation_b; /* B 插拔后，正在读取旧 B 手柄数据的任务会在下一次检查时停止。 */
	}
}

/*
 * 函数功能：取得指定通道的插拔变化计数，调用方将它与任务开始时的计数比较，判断能否继续读取原手柄。
 * 输入参数：channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：该通道的 32 位插拔变化计数；无效通道返回 0，不能仅凭返回值判断手柄是否在线。
 */
uint32_t Handle_GetIdentityGeneration(uint8_t channel)
{
	if (channel == CHANNEL_A) /* 查询 A 时只读取 A 的当前计数。 */
	{
		return s_handle_identity_generation_a; /* 直接返回 A 的内存计数，不额外读取手柄 EEPROM。 */
	}

	if (channel == CHANNEL_B) /* 查询 B 时只读取 B 的当前计数。 */
	{
		return s_handle_identity_generation_b; /* 返回 B 的当前计数，供数据读取任务与开始时记住的值比较。 */
	}

	return 0U; /* 无效通道返回 0；调用方仍需检查通道号和在线状态，不能继续读写 EEPROM。 */
}

/*
 * 函数功能：关闭 PXBA/PXBB 当前通道的 RFID 自动识别运行状态。
 * 输入参数：channel 为当前工作通道；memory 指向该通道记忆。
 * 返回参数：无。
 */
static void Handle_DisableAutoIdentify(uint8_t channel, ChannelMemoryMessage_t *memory)
{
	if (memory == NULL) /* 通道记忆为空时无法确定应清 A 还是 B，必须保持现有状态。 */
	{
		return; /* 没有通道记忆时不能安全修改自动识别状态，避免 A/B 记忆串写。 */
	}

	WorkMessage.auto_identify = 0U; /* 把当前通道改为手动模式，屏幕不再显示自动识别的等待状态和结果区。 */
	memory->auto_identify = 0U;	   /* 通道记忆同步进入手动模式，切换通道后不会再次自动恢复 RFID。 */
	WorkMessage.raw_tool_type = 0U; /* 手动刀具不再使用 RFID 原始型号，避免驱动和上位机继续按旧标签判断。 */
	memory->raw_tool_type = 0U;	   /* 通道保存的 RFID 型号也清零，切回该通道时仍按手动刀具处理。 */

	if (channel == CHANNEL_A) /* A 通道退出自动识别时只清理 A 的标签结果和任务。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_A, 0U);							   /* A 通道手动模式不显示 RFID 掉线历史图标。 */
		memset(paoxueSpeciValue_A, 0, sizeof(paoxueSpeciValue_A)); /* A 通道手动模式不显示 RFID 刀具规格窗口。 */
		Rfid_ClearChannelResult(CHANNEL_A);						   /* 清空 A 标签结果并取消正在读取或排队的旧请求，避免旧结果覆盖手动选择。 */
		SendKeyRFIDMessageAdown();								   /* 再给 A 加入停止消息；上一步已取消旧请求，此消息由 RFID 任务空闲时处理。 */
	}
	else if (channel == CHANNEL_B) /* B 通道退出自动识别时只清理 B，避免影响 A。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_B, 0U);							   /* B 通道手动模式不显示 RFID 掉线历史图标。 */
		memset(paoxueSpeciValue_B, 0, sizeof(paoxueSpeciValue_B)); /* B 通道手动模式不显示 RFID 刀具规格窗口。 */
		Rfid_ClearChannelResult(CHANNEL_B);						   /* 清空 B 标签结果并取消正在读取或排队的旧请求，避免旧结果覆盖手动选择。 */
		SendKeyRFIDMessageBdown();								   /* 再给 B 加入停止消息；上一步已取消旧请求，此消息由 RFID 任务空闲时处理。 */
	}
}

/*
 * 函数功能：按通道取得扫描层识别缓存。
 * 输入参数：channel 为 A/B 通道。
 * 返回参数：对应通道的 ChannelrecognizeMessage；无效通道返回 NULL。
 */
static ChannelrecognizeMessage_t *Handle_GetRecognize(uint8_t channel)
{
	if (channel == CHANNEL_A) /* A 通道必须读取 A 的 EEPROM 识别缓存。 */
	{
		return &ChannelrecognizeMessageA; /* A 通道手动模式读取 A 扫描缓存，避免使用 B 手柄 EEPROM 参数。 */
	}

	if (channel == CHANNEL_B) /* B 通道必须读取 B 的独立识别缓存。 */
	{
		return &ChannelrecognizeMessageB; /* B 通道手动模式读取 B 扫描缓存，保证 A/B 参数独立。 */
	}

	return NULL; /* 非 A/B 通道没有有效识别缓存，调用方必须保持原状态。 */
}

/*
 * 函数功能：按方向取得识别缓存中的默认速度。
 * 输入参数：recognize 为扫描层识别缓存；direction 为目标运行方向。
 * 返回参数：目标方向默认速度；参数无效时返回 0。
 */
static uint32_t Handle_GetDirectionDefaultSpeed(const ChannelrecognizeMessage_t *recognize, uint8_t direction)
{
	if (recognize == NULL) /* 缺少识别缓存时没有可信的方向默认速度。 */
	{
		return 0U; /* 缺少手柄参数时返回 0，由调用方决定是否采用备用速度。 */
	}

	if (direction == FZDIR) /* 反转使用 EEPROM 中独立保存的反转默认速度。 */
	{
		return recognize->speed_fzdefault; /* 反转手动速度来自 Page4 恢复后的反转默认速度。 */
	}

	if (direction == OSCDIR) /* 往复使用 EEPROM 中独立保存的往复默认速度。 */
	{
		return recognize->speed_oscdefault; /* 往复手动速度来自 Page4 恢复后的往复默认速度。 */
	}

	return recognize->speed_zzdefault; /* 正转和异常方向按正转默认速度处理。 */
}

/*
 * 函数功能：按方向取得识别缓存中的速度下限。
 * 输入参数：recognize 为扫描层识别缓存；direction 为目标运行方向。
 * 返回参数：目标方向最小速度；参数无效时返回 0。
 */
static uint32_t Handle_GetDirectionMinSpeed(const ChannelrecognizeMessage_t *recognize, uint8_t direction)
{
	if (recognize == NULL) /* 缺少识别缓存时不能取得可靠的速度下限。 */
	{
		return 0U; /* 缺少手柄参数时返回下限 0，不凭空指定一个最小速度。 */
	}

	if (direction == FZDIR) /* 反转速度边界与正转分开保存。 */
	{
		return recognize->speed_fzmin; /* 反转最小速度来自手柄 EEPROM Page4。 */
	}

	if (direction == OSCDIR) /* 往复速度使用自己的最小值，不能沿用旋转方向。 */
	{
		return recognize->speed_oscmin; /* 往复最小速度来自手柄 EEPROM Page4。 */
	}

	return recognize->speed_zzmin; /* 正转最小速度来自手柄 EEPROM Page4。 */
}

/*
 * 函数功能：按方向取得识别缓存中的速度上限。
 * 输入参数：recognize 为扫描层识别缓存；direction 为目标运行方向。
 * 返回参数：目标方向最大速度；参数无效时返回 0。
 */
static uint32_t Handle_GetDirectionMaxSpeed(const ChannelrecognizeMessage_t *recognize, uint8_t direction)
{
	if (recognize == NULL) /* 缺少识别缓存时不能取得可靠的速度上限。 */
	{
		return 0U; /* 缺少手柄参数时返回上限 0，调用方按未提供上限处理。 */
	}

	if (direction == FZDIR) /* 反转使用 EEPROM 中的反转最大速度。 */
	{
		return recognize->speed_fzmax; /* 反转最大速度来自手柄 EEPROM Page4。 */
	}

	if (direction == OSCDIR) /* 往复使用 EEPROM 中的往复最大速度。 */
	{
		return recognize->speed_oscmax; /* 往复最大速度来自手柄 EEPROM Page4。 */
	}

	return recognize->speed_zzmax; /* 正转最大速度来自手柄 EEPROM Page4。 */
}

/*
 * 函数功能：检查手动模式速度；默认值为 0 时先用备用值，再按当前方向的 EEPROM 最小值和非零最大值调整。
 * 输入参数：recognize 为手柄参数；direction 为运行方向；speed 为待检查速度；fallback_speed 为 speed 等于 0 时使用的备用速度。
 * 返回参数：可写入 MemoryMsg/WorkMessage 的合法速度。
 */
static uint32_t Handle_ClampSpeed(const ChannelrecognizeMessage_t *recognize,
														 uint8_t direction,
														 uint32_t speed,
														 uint32_t fallback_speed)
{
	uint32_t speed_min = Handle_GetDirectionMinSpeed(recognize, direction); /* 读取当前方向 EEPROM 最小速度，手动调速不能低于此值。 */
	uint32_t speed_max = Handle_GetDirectionMaxSpeed(recognize, direction); /* 读取当前方向 EEPROM 最大速度，避免手动模式继续使用 RFID 上限。 */

	if (speed == 0U) /* 待用速度为 0 时，先采用调用方给出的备用速度。 */
	{
		speed = fallback_speed; /* 写入备用速度；如果备用值也是 0，后面仍按手柄最小值继续检查。 */
	}

	if ((speed_max != 0U) && (speed_max < speed_min)) /* 已提供非零上限，但它小于下限时，先修正用于本次检查的上限。 */
	{
		speed_max = speed_min; /* 本次按上下限相等处理，避免先提高到下限后又降到更小的上限；不修改 EEPROM。 */
	}

	if (speed < speed_min) /* 待用速度小于手柄配置的最小值时，提高到最小值。 */
	{
		speed = speed_min; /* 候选速度低于 EEPROM 下限时抬到下限，保证实际运行不会低于手柄配置。 */
	}

	if ((speed_max != 0U) && (speed > speed_max)) /* 非零上限有效时禁止候选速度越界。 */
	{
		speed = speed_max; /* 候选速度超过 EEPROM 上限时压到上限，保证显示值和可运行值一致。 */
	}

	return speed; /* 返回已经按手柄 EEPROM 上下限处理后的速度。 */
}

/*
 * 函数功能：分体手柄切到手动模式时，重新读取 EEPROM 运行参数，并同时更新通道保存值和当前使用值。
 * 输入参数：channel 为 A/B 通道；memory 为通道保存参数；tool_type 为手动刀具类型；direction 为默认方向；fallback_speed 为默认速度为 0 时的备用速度。
 * 返回参数：无。
 */
static void Handle_ApplyManualRuntime(uint8_t channel,
													   ChannelMemoryMessage_t *memory,
													   uint8_t tool_type,
													   uint8_t direction,
													   uint32_t fallback_speed)
{
	ChannelrecognizeMessage_t *recognize = Handle_GetRecognize(channel); /* 获取当前通道扫描缓存，里面保存 EEPROM 恢复后的速度上下限。 */
	uint32_t zz_speed = 0U; /* 保存手动正转速度，使用 EEPROM 默认值并检查正转上下限。 */
	uint32_t fz_speed = 0U; /* 保存手动反转速度，使用 EEPROM 默认值并检查反转上下限。 */
	uint32_t osc_speed = 0U; /* 保存手动往复速度，使用 EEPROM 默认值并检查往复上下限。 */

	if ((memory == NULL) || (recognize == NULL)) /* 任一通道对象缺失都会造成参数只更新一半。 */
	{
		return; /* 缺少任一份参数时都不继续修改，避免只更新一部分后显示和启动参数不一致。 */
	}

	(void)Handlescan_RestoreSplitHandleEepromRuntime(channel, tool_type); /* 先让扫描层重新从手柄 EEPROM 装载倍率、速度上下限和步进。 */
	recognize = Handle_GetRecognize(channel); /* 重新取得缓存指针，确保后续读取的是恢复后的识别结构。 */
	if (recognize == NULL) /* EEPROM 恢复后再次校验指针，避免继续写入无效缓存。 */
	{
		return; /* 没有取得参数就退出，避免访问空指针。 */
	}

	recognize->tool_type = tool_type; /* 手动模式刀具能力由用户选择，覆盖自动识别留下的标签类型。 */
	recognize->raw_tool_type = 0U; /* 手动模式没有 RFID 原始刀具型号，清掉旧标签码。 */
	recognize->run_direction = direction; /* 手动刨刀默认往复、手动磨头默认正转，保持旧 UI 交互习惯。 */
	zz_speed = Handle_ClampSpeed(recognize,
									 ZZDIR,
									 Handle_GetDirectionDefaultSpeed(recognize, ZZDIR),
									 fallback_speed); /* 正转速度按手柄 EEPROM 默认值和上下限生成。 */
	fz_speed = Handle_ClampSpeed(recognize,
									 FZDIR,
									 Handle_GetDirectionDefaultSpeed(recognize, FZDIR),
									 fallback_speed); /* 反转速度按手柄 EEPROM 默认值和上下限生成。 */
	osc_speed = Handle_ClampSpeed(recognize,
									  OSCDIR,
									  Handle_GetDirectionDefaultSpeed(recognize, OSCDIR),
									  fallback_speed); /* 往复速度按手柄 EEPROM 默认值和上下限生成。 */

	memory->tool_type = tool_type; /* 通道记忆同步手动刀具能力，切通道后仍保持手动磨/刨选择。 */
	memory->raw_tool_type = 0U; /* 通道记忆清 RFID 原始型号，避免上位机把手动模式当自动标签显示。 */
	memory->auto_identify = 0U; /* 通道记忆进入手动模式，后续切回该通道不自动恢复 RFID。 */
	memory->tool_reduction_ratio = (recognize->tool_reduction_ratio != 0U) ? recognize->tool_reduction_ratio : ((recognize->meioticratio > 1U) ? ((uint32_t)recognize->meioticratio * 100U) : 100U); /* 优先用完整倍率（实际倍率 × 100）；没有完整值时，把 Page3 整数倍率乘 100；两者都没有时用 100，表示 1 倍。 */
	memory->default_injection_flow = recognize->default_injection_flow; /* 手动模式仍使用手柄 Page4 默认注水流量，保证泵显示和运行一致。 */
	memory->speed_alarm_for = recognize->speed_alarm_for; /* 手动模式同步手柄 EEPROM 正转速度报警阈值。 */
	memory->speed_alarm_rev = recognize->speed_alarm_rev; /* 手动模式同步手柄 EEPROM 反转速度报警阈值。 */
	memory->freq_alarm_osc = recognize->freq_alarm_osc; /* 手动模式同步手柄 EEPROM 往复频率报警阈值。 */
	memory->freq = (recognize->freq_default != 0U) ? recognize->freq_default : 40U; /* 优先用Page4的频率指令值；值为0时改用40，主控按此数值传给驱动，不在这里换算Hz。 */
	memory->dir = direction; /* 通道记忆保存手动默认方向，切换通道后按该方向恢复速度。 */
	memory->zz_speed = zz_speed; /* 正转记忆写入 EEPROM 默认速度经上下限检查后的值。 */
	memory->fz_speed = fz_speed; /* 反转记忆写入 EEPROM 默认速度经上下限检查后的值。 */
	memory->osc_speed = osc_speed; /* 往复记忆写入 EEPROM 默认速度经上下限检查后的值。 */

	if (channel == CHANNEL_A) /* A 通道进入手动模式时只清 A 的 RFID 显示缓存。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_A, 0U); /* A 手动模式不显示 RFID 掉线历史图标。 */
		memset(paoxueSpeciValue_A, 0, sizeof(paoxueSpeciValue_A)); /* A 手动模式清刀具规格窗口，避免旧标签直径/长度/角度残留。 */
	}
	else if (channel == CHANNEL_B) /* B 通道进入手动模式时只清 B 的 RFID 显示缓存。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_B, 0U); /* B 手动模式不显示 RFID 掉线历史图标。 */
		memset(paoxueSpeciValue_B, 0, sizeof(paoxueSpeciValue_B)); /* B 手动模式清刀具规格窗口，避免旧标签直径/长度/角度残留。 */
	}

	WorkMessage.tool_type = memory->tool_type; /* 更新当前刀具类型，屏幕按钮和是否允许运行都使用此值。 */
	WorkMessage.raw_tool_type = 0U; /* 清除当前 RFID 型号，避免手动模式继续使用旧标签的类型。 */
	WorkMessage.auto_identify = 0U; /* 把当前工作模式改为手动识别刀具。 */
	WorkMessage.tool_reduction_ratio = memory->tool_reduction_ratio; /* 当前使用的倍率也改为手柄 EEPROM 参数，后续发电机速度时按此换算。 */
	WorkMessage.freq_work = memory->freq; /* 更新当前往复频率，下一次手动启动刨刀时使用此值。 */
	WorkMessage.dir_work = memory->dir; /* 更新当前手动方向，显示速度和下一次启动速度都按这个方向选择。 */
	if (direction == FZDIR) /* 当前选择反转时，屏幕和启动速度取反转记忆。 */
	{
		WorkMessage.speed_set_work = memory->fz_speed; /* 手动反转时当前设定速度来自反转记忆。 */
	}
	else if (direction == OSCDIR) /* 当前选择往复时，屏幕和启动速度取往复记忆。 */
	{
		WorkMessage.speed_set_work = memory->osc_speed; /* 手动往复时当前设定速度来自往复记忆。 */
	}
	else
	{
		WorkMessage.speed_set_work = memory->zz_speed; /* 手动正转或异常方向时当前设定速度来自正转记忆。 */
	}
	WorkMessage.speed_work = WorkMessage.speed_set_work; /* 把当前运行速度字段同步为设定值，供显示和后续启动使用；这不是实测转速。 */
}

/*
 * 函数功能：处理屏幕或上位机发起的磨头/刨刀切换，并同步刷新当前通道显示。
 * 输入参数：key_value 为磨头或刨刀按键值。
 * 返回参数：无。
 */
void PlanerGridH(uint8_t key_value)
{
	ChannelMemoryMessage_t *memory = NULL; /* 指向当前工作通道记忆，手动切换时需要同步保存自动/手动状态。 */
	uint8_t current_channel = WorkMessage.channel_work; /* 记录当前工作通道，后续关闭 RFID 和刷新界面都使用它。 */
	bool auto_identify_was_enabled = (WorkMessage.auto_identify != 0U); /* 记录进入函数前是否处于自动识别，自动转手动时即使刀具类型相同也要刷新。 */
	bool tool_changed = false; /* 标记本次是否重新装载了手动刀具参数，后面据此刷新界面。 */

	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true) /* 运行中或报警中不能切换刀具模式，避免重载方向和速度参数。 */
	{
		return; /* 保持当前 RFID、刀具类型和通道参数不变。 */
	}
	if ((key_value != SCREENKey_PlanerH) && /* 非磨/刨业务键不能关闭 RFID 或重写当前刀具参数。 */
		(key_value != HMIkey_PlanerH) &&
		(key_value != HMIkey_GrindH) &&
		(key_value != SCREENKey_GrindH))
	{
		return; /* 只有明确的手动磨/刨键才允许关闭 RFID，异常键值不能改变当前自动识别模式。 */
	}
	if (Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model) == false) /* 只有 PXBA/PXBB 分体手柄支持屏幕手动选择磨头或刨刀。 */
	{
		return; /* 手动磨/刨切换只开放给 PXBA/PXBB，普通手柄和公共接头不允许误入手动刀具模式。 */
	}
	if ((WorkMessage.auto_identify != 0U) && /* 自动识别界面已隐藏手动按钮，旧坐标必须保持无效。 */
		((key_value == SCREENKey_PlanerH) || (key_value == SCREENKey_GrindH))) /* 自动识别模式已隐藏屏幕磨/刨按钮，只拦截新屏旧坐标。 */
	{
		return; /* 禁止隐藏坐标关闭 RFID 并切换刀具；HMI 旧入口保持原有显式切换行为。 */
	}
	if (current_channel == CHANNEL_A) /* 手动选择结果必须写入当前 A 通道记忆。 */
	{
		memory = &MemoryMsgA; /* A 通道手动模式写 A 记忆，避免影响 B 通道自动识别状态。 */
	}
	else if (current_channel == CHANNEL_B) /* 当前选择 B 时只更新 B 通道记忆。 */
	{
		memory = &MemoryMsgB; /* B 通道手动模式写 B 记忆，保持 A/B 模式互相独立。 */
	}
	else
	{
		return; /* 未选中 A/B 通道时不能切手动刀具，避免把未选中通道的参数写入 WorkMessage。 */
	}
	if (auto_identify_was_enabled != false) /* 从自动切到手动前必须先终止当前 RFID 请求。 */
	{
		Handle_DisableAutoIdentify(current_channel, memory); /* 从自动识别切入手动刀具时先停止 RFID，防止旧回包覆盖手动选择。 */
	}
	switch (key_value)
	{
	case SCREENKey_PlanerH: // 屏幕上的“刨刀”选择按钮
	case HMIkey_PlanerH:	// 上位机的“刨刀”选择命令
		if ((auto_identify_was_enabled == false) && Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type)) /* 已处于手动刨刀时无需重复装载参数。 */
		{
			return; /* 已经是刨刀能力时不重复刷新，避免同一按钮连续触发造成无意义 UI 消息堆积。 */
		}
		Handle_ApplyManualRuntime(current_channel,
								  memory,
								  PLANER,
								  OSCDIR,
								  0U); /* 手动刨刀速度必须来自Page3专用扩展，配置无效时保持0并禁止电机启动。 */
		tool_changed = true; /* 刨刀键已更新当前通道记忆，后续需要立即重绘刀具、方向、频率和开口定位。 */
		// 刨头
		break;
	case HMIkey_GrindH:	   // 上位机的“磨头”选择命令
	case SCREENKey_GrindH: // 屏幕上的“磨头”选择按钮
		if ((auto_identify_was_enabled == false) && (Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type) == false)) /* 已处于手动磨头时无需重复刷新。 */
		{
			return; /* 已经是手动磨头时直接返回，重复按同一按钮不会重新装载参数。 */
		}
		Handle_ApplyManualRuntime(current_channel,
								  memory,
								  GRINDH,
								  ZZDIR,
								  60000U); /* 手动磨头默认正转；倍率和速度上下限取自手柄 EEPROM，默认速度为 0 时先用 60000，再检查手柄上下限。 */
		tool_changed = true; /* 磨头键已更新当前通道记忆，后续需要关闭频率和开口定位并重绘速度。 */
		// 磨头
		break;
	}

	if (tool_changed && /* 只有本次重新装载了刀具参数且仍选中 A/B 通道时才刷新整页。 */
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
	ChannelMemoryMessage_t *memory = NULL; /* 指向当前 A/B 通道记忆，自动/手动模式要和切通道后的恢复状态保持一致。 */
	uint8_t current_channel = WorkMessage.channel_work; /* 记录当前工作通道，后续刷新界面和触发 RFID 都按这个通道执行。 */

	if (key_value != SCREENKey_AutoIdentify) /* 该入口只处理新屏自动识别按钮。 */
	{
		return; /* 只接收新屏自动识别键，避免其它旧键误进入 RFID 流程。 */
	}

	if (WorkMessage.runflag_work == true) /* 运行中不能切换刀具来源，避免电机参数突变。 */
	{
		return; /* 电机运行中不启动 RFID 识别，避免识别流程改变当前刀具参数。 */
	}

	if (WorkMessage.alarm_flag == true) /* 报警状态必须优先处理，不能发起新的 RFID 读取。 */
	{
		return; /* 系统报警未解除时不发起自动识别，保持报警优先级。 */
	}

	if (Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model) == false) /* 非分体手柄没有自动/手动 RFID 切换入口。 */
	{
		return; /* 自动/手动识别切换只开放给 PXBA/PXBB 分体手柄，其它手柄的刀具类型由 EEPROM 或 EPC 流程决定。 */
	}

	if (current_channel == CHANNEL_A) /* A 通道自动识别状态写入 A 记忆。 */
	{
		memory = &MemoryMsgA; /* A 通道自动识别状态写入 A 记忆，切换到 B 再切回 A 时能恢复。 */
	}
	else if (current_channel == CHANNEL_B) /* B 通道自动识别状态写入 B 记忆。 */
	{
		memory = &MemoryMsgB; /* B 通道自动识别状态写入 B 记忆，避免 A/B 通道模式串用。 */
	}
	else
	{
		return; /* 未选中 A/B 通道时不触发 RFID，避免识别结果没有通道归属。 */
	}

	if (WorkMessage.auto_identify != 0U) /* 再次点击已开启的自动识别键表示切回手动模式。 */
	{
		Handle_DisableAutoIdentify(current_channel, memory); /* 再次点击自动识别键时进入手动模式，并停止当前 RFID 读取。 */
		Handle_ApplyManualRuntime(current_channel,
								  memory,
								  PLANER,
								  OSCDIR,
								  0U); /* 关闭自动识别后，从 EEPROM 的刨刀专用字段恢复速度；这里传 0，不提供固定备用速度。 */
		Pubinterface_RefreshSelectedChannelDisplay(current_channel); /* 手动模式立即刷新刨刀选中、自动识别关闭、规格窗口隐藏。 */
		return;
	}

	if (Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type) != false) /* 自动等待期间继续保留当前手动刨刀能力，避免尚无标签时失去可运行方向。 */
	{
		Handle_ApplyManualRuntime(current_channel,
								  memory,
								  PLANER,
								  OSCDIR,
								  0U); /* 进入自动等待前重新读取专用字段；无有效配置时速度为0，不能沿用通用30000rpm。 */
	}
	else
	{
		Handle_ApplyManualRuntime(current_channel,
								  memory,
								  GRINDH,
								  ZZDIR,
								  60000U); /* 当前手动模式不是刨刀时保留磨头能力，等待 RFID 结果前仍使用基座 EEPROM 参数。 */
	}

	WorkMessage.auto_identify = 1U; /* 从手动模式切到自动模式，当前工作态进入 RFID 自动识别。 */
	memory->auto_identify = 1U; /* 通道记忆同步保存自动模式，切换通道后仍能按自动识别界面显示。 */
	Handlescan_PrepareSplitAutoIdentify(current_channel); /* 清掉本通道已处理的标签编号和等待记录，重新进入自动模式后会处理第一个有效标签。 */
	if (current_channel == CHANNEL_A) /* 新一轮 A 自动识别开始前清除 A 的历史图标。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_A, 0U); /* 用户主动重新进入自动识别时，从等待状态开始显示 63，不沿用上一次离线图标。 */
	}
	else if (current_channel == CHANNEL_B) /* 新一轮 B 自动识别开始前清除 B 的历史图标。 */
	{
		Pubinterface_SetLastRfidToolType(CHANNEL_B, 0U); /* B 通道同样清掉上一次离线图标，避免新一轮识别前误显示 61/62。 */
	}
	if (current_channel == CHANNEL_A) /* A 当前被选中时只向 A RFID 队列发起请求。 */
	{
		SendKeyRFIDMessageAup(0U); /* A 通道 PXBA/PXBB 自动识别按新协议读取 EPC 区，刀具头参数来自 RFID 标签。 */
	}
	else
	{
		SendKeyRFIDMessageBup(0U); /* B 通道 PXBA/PXBB 自动识别按新协议读取 EPC 区，保持 A/B RFID 请求分离。 */
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

	if ((WorkMessage.runflag_work == true) || (WorkMessage.alarm_flag == true)) /* 运行或报警期间切通道会让当前输出失去归属。 */
	{
		return; /* 运行中或系统报警中不允许切换，避免两个手柄同时参与输出。 */
	}

	if ((key_value == HMIkey_HANDLE_A) || (key_value == SCREENKey_HANDLE_A)) /* HMI 与新屏 A 键都明确选择 A 通道。 */
	{
		target_channel = CHANNEL_A; /* 屏幕或上位机明确点 A 时，目标通道就是 A。 */
	}
	else if ((key_value == HMIkey_HANDLE_B) || (key_value == SCREENKey_HANDLE_B)) /* HMI 与新屏 B 键都明确选择 B 通道。 */
	{
		target_channel = CHANNEL_B; /* 屏幕或上位机明确点 B 时，目标通道就是 B。 */
	}
	else if (key_value == JTKey_middle_long) /* 脚踏中键长按用于在两只在线手柄之间切换。 */
	{
		if ((WorkMessage.channel_work == CHANNEL_A) && (WorkMessage.Channel_Bonline == true)) /* 当前选中 A 且 B 在线时，中键长按切到 B。 */
		{
			target_channel = CHANNEL_B; /* 当前选中 A 且 B 在线时，脚踏长按只切到 B。 */
		}
		else if ((WorkMessage.channel_work == CHANNEL_B) && (WorkMessage.Channel_Aonline == true)) /* 当前选中 B 且 A 在线时，中键长按切到 A。 */
		{
			target_channel = CHANNEL_A; /* 当前选中 B 且 A 在线时，脚踏长按只切到 A。 */
		}
	}

	if ((target_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == true)) /* 只有目标 A 仍在线时才能装载 A 参数。 */
	{
		Pubinterface_LoadChannelMemory(CHANNEL_A); /* 用户选择 A 后，把 A 保存的参数改为当前使用参数。 */
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* 按 A 当前参数刷新控制方式、刀具、方向、频率和速度显示。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 A 被点亮。 */
		s_running_unplug_recovery_channel = CHANNEL_NONE; /* 用户已明确选中 A，取消尚未执行的掉线自动恢复请求，防止周期任务覆盖用户选择。 */
	}
	else if ((target_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == true)) /* 只有目标 B 仍在线时才能装载 B 参数。 */
	{
		Pubinterface_LoadChannelMemory(CHANNEL_B); /* 用户选择 B 后，把 B 保存的参数改为当前使用参数。 */
		Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* 按 B 当前参数刷新控制方式、刀具、方向、频率和速度显示。 */
		Pubinterface_RefreshOnlineHandleDisplay(); /* 切换完成后刷新 A/B 手柄高亮，确保只有 B 被点亮。 */
		s_running_unplug_recovery_channel = CHANNEL_NONE; /* 用户已明确选中 B，取消尚未执行的掉线自动恢复请求。 */
	}
}

/*
 * 函数功能：记录运行中拔出当前手柄后需要自动恢复的另一在线通道。
 * 输入参数：channel 为待恢复的 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
void Handle_RequestRemainingOnlineAfterUnplug(uint8_t channel)
{
	if (((channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == true)) ||
		((channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == true)))
	{
		s_running_unplug_recovery_channel = channel; /* 只记住此时确实在线的另一路；未在线的通道不会在这里记作本次恢复目标。 */
	}
	else
	{
		s_running_unplug_recovery_channel = CHANNEL_NONE; /* 拔出时没有另一在线通道，本次不创建自动恢复请求。 */
	}
}

/*
 * 函数功能：停机、无报警且当前未选通道时，优先选择拔出时记住的另一路；没有记录时仅自动选择唯一在线通道，不启动电机。
 * 输入参数：无；脚踏或触控松开、外控停止或退出、手控限时提示结束，由调用方先按对应流程处理；本函数再检查停止、报警和通道状态。
 * 返回参数：无。
 */
void Handle_SelectRemainingOnlineAfterUnplug(void)
{
	uint8_t target_channel = s_running_unplug_recovery_channel; /* 先取拔出时记住的另一路，优先恢复这一路。 */

	if (target_channel == CHANNEL_NONE)
	{
		if (WorkMessage.channel_work != CHANNEL_NONE)
		{
			return; /* 已经选好通道就不再自动选择，避免覆盖用户或其它流程的选择。 */
		}
		if ((WorkMessage.Channel_Aonline == true) && (WorkMessage.Channel_Bonline == false))
		{
			target_channel = CHANNEL_A; /* 没有保存恢复目标且只有 A 在线，先把 A 作为候选，后面仍需检查停止和报警状态。 */
		}
		else if ((WorkMessage.Channel_Bonline == true) && (WorkMessage.Channel_Aonline == false))
		{
			target_channel = CHANNEL_B; /* 没有保存恢复目标且只有 B 在线，先把 B 作为候选，后面仍需检查停止和报警状态。 */
		}
		else
		{
			return; /* 没有在线手柄或 A/B 同时在线时不能猜测目标，保持停止并等待用户明确选择。 */
		}
	}

	if ((WorkMessage.runflag_work != false) ||
		(WorkMessage.alarm_flag != false))
	{
		return; /* 电机未停或报警未退出时保留待恢复请求，由周期服务稍后再次检查。 */
	}

	if (WorkMessage.channel_work != CHANNEL_NONE)
	{
		s_running_unplug_recovery_channel = CHANNEL_NONE; /* 其它处理已经选好了通道，清除自动恢复记录，不覆盖现有选择。 */
		return;
	}

	if (((target_channel == CHANNEL_A) && (WorkMessage.Channel_Aonline == false)) ||
		((target_channel == CHANNEL_B) && (WorkMessage.Channel_Bonline == false)))
	{
		return; /* 目标通道当前不在线，先保留恢复记录，等待后续任务再次检查。 */
	}

	HandleSwitchActive((target_channel == CHANNEL_A) ? SCREENKey_HANDLE_A : SCREENKey_HANDLE_B); /* 复用现有切换入口装载 EEPROM/RFID 参数并刷新手柄高亮和运行参数区。 */
	if (WorkMessage.channel_work != target_channel)
	{
		return; /* 切换后仍未选中目标，暂不清除恢复记录，下一周期再检查。 */
	}
	Pubinterface_ApplyChannelDefaultInjectionFlow(target_channel); /* 当前通道已经恢复，注水泵停止态设定值同步切到该手柄 Page4 默认流量。 */
	WorkMessage.runflag_work = false; /* 自动恢复只选择通道，绝不把剩余手柄直接启动。 */
	WorkMessage.speed_work = 0U; /* 把运行速度字段清零，只保留设定速度 speed_set_work，等用户重新发出启动操作。 */
	s_running_unplug_recovery_channel = CHANNEL_NONE; /* 恢复成功后清除记录，后续周期不再重复选择或刷新。 */
}

/*
 * 函数功能：最后一个手柄拔出后清空当前选中通道和参数显示。
 * 输入参数：无，直接清除 WorkMessage 中当前选中通道的参数。
 * 返回参数：无。
 */
static void Handle_ClearSelection(void)
{
	WorkMessage.hand_model = 0U;					   /* 没有任何手柄在线时清掉当前手柄型号，防止一体式手柄拔出后仍按旧型号刷新方向按钮。 */
	WorkMessage.tool_type = 0U;					   /* 没有任何手柄在线时清掉当前刀具能力，防止旧刨刀能力继续打开往复入口。 */
	WorkMessage.raw_tool_type = 0U;				   /* 同步清掉原始刀具型号，避免后续驱动或上位机继续看到旧 EEPROM/RFID 代号。 */
	WorkMessage.auto_identify = 0U;				   /* 无手柄状态不属于 RFID 自动识别模式，必须清掉自动识别标志以隐藏 0x1407 按钮。 */
	WorkMessage.dir_work = 0U;					   /* 无手柄时当前方向无效，方向区应回到灰色初始状态。 */
	WorkMessage.freq_work = 0U;					   /* 无手柄时往复频率无效，避免频率窗口沿用旧 PXYTP/MXYTP 参数。 */
	WorkMessage.speed_set_work = 0U;			   /* 无手柄时清掉设定速度，防止速度栏在异步刷新后恢复旧值。 */
	WorkMessage.speed_work = 0U;				   /* 无手柄时实际目标速度必须为 0，保持显示和驱动停止状态一致。 */
	WorkMessage.channel_work = CHANNEL_NONE;		   /* 最后一个手柄离线后没有选中通道，后续屏幕触控和方向按键都不能落到 A/B。 */
	s_running_unplug_recovery_channel = CHANNEL_NONE; /* A/B 都离线后不存在可恢复目标，取消运行掉线自动切换请求。 */
	UIDP_ForceNoHandleDisplay();				   /* 先清 UIDP 旧队列并直写无手柄控件，防止旧方向和自动识别消息晚到覆盖拔出状态。 */
	Pubinterface_ClearSelectedChannelDisplay();	   /* 立即关闭方向、刀具识别、刀具规格、速度和频率区，清除屏幕旧控件残留。 */
	Pubinterface_ApplyInjectionPumpDefaultFlow(0U); /* 无选中手柄时注水泵默认值回到程序默认 30，显示值和下一次运行目标保持一致。 */
}

/*
 * 函数功能：处理 A/B 手柄插入、拔出事件，维护在线标志、当前工作通道和屏幕显示。
 * 输入参数：key_value 插拔事件按键值，SCREENKey_PLUG_A/B 表示上线，SCREENKey_UNPLUG_A/B 表示离线。
 * 返回参数：无。
 */
void PlugORunPLUGActive(uint8_t key_value)
{
	uint8_t current_channel_unplugged = 0U; /* 记录拔出的是否为当前选中通道，用于区分停机报警和剩余通道恢复流程。 */
	uint8_t current_channel_run_interrupted = 0U; /* 记录拔出前是否正在运行；即使驱动先报 87 并清掉运行标志，也要按运行中拔出处理。 */
	uint8_t channel_was_online = 0U;		   /* 记录插入事件前该通道是否已经在线，用于区分首次接入和重复识别刷新。 */
	uint8_t close_idle_touch = 0U;			   /* 记录拔出时是否只是打开本机触控窗口、尚未运行；先清屏再关闭窗口，避免关闭消息被一并清掉。 */
	uint8_t close_verify_alarm = 0U;		   /* 记录扫描任务要求关闭校验报警的消息；等无手柄清屏完成后，再发送关闭 90 号报警图的命令。 */

	switch (key_value)
	{
	case SCREENKey_PLUG_A: // 插入A
		channel_was_online = (uint8_t)WorkMessage.Channel_Aonline; /* 先保存 A 原在线状态，只有从离线变成在线时才装载 Page4 默认流量。 */
		if (channel_was_online == 0U) /* 重复上报识别结果不算换柄；只有从离线变在线时增加插拔计数。 */
		{
			Handle_AdvanceIdentityGeneration(CHANNEL_A); /* 先增加 A 插拔计数，旧 A 数据读取任务下次检查时会停止，再把 A 标为在线。 */
		}
		WorkMessage.Channel_Aonline = true;					   /* A 通道校验通过后才置在线，后续心跳和显示都读取这个标志。 */
		(void)Pubinterface_ClearHandleNotConnectedAlarm();	   /* A 手柄重新接入后清除运行中拔手柄留下的未连接报警，恢复自动选中条件。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_A);		   /* 扫描结果只先进入 MemoryMsgA，运行中不会直接覆盖 WorkMessage。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_A)) /* 停机且无报警时，新接入 A 才能自动成为当前通道。 */
		{
			Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非运行状态下最后插入且校验通过的 A 通道成为当前选中通道。 */
			if (channel_was_online == 0U) /* 仅首次离线转在线时装载默认流量，重复识别不能覆盖用户设置。 */
			{
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* A 首次接入并被选中时，才用 A 的 Page4 默认流量初始化注水泵。 */
			}
			Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* A 自动成为当前通道后，同步刷新参数区和刀具规格。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 A 在线图标和当前高亮状态。 */
		Pubinterface_ClearCommonSocketToolMissingAlarm();	   /* A 通道 EPC 刀具信息装载成功后清除公共接头缺刀具临时报警。 */
		break;

	case SCREENKey_PLUG_B: // 插入B
		channel_was_online = (uint8_t)WorkMessage.Channel_Bonline; /* 先保存 B 原在线态，防止 RFID/识别重复刷新覆盖用户手动调节的泵流量。 */
		if (channel_was_online == 0U) /* B 只有从离线变在线时才记一次插入，重复识别不会增加计数。 */
		{
			Handle_AdvanceIdentityGeneration(CHANNEL_B); /* 先增加 B 插拔计数，让旧 B 数据任务下次检查时停止，再把 B 标为在线。 */
		}
		WorkMessage.Channel_Bonline = true;					   /* B 通道校验通过后才置在线，坏手柄不会进入在线态。 */
		(void)Pubinterface_ClearHandleNotConnectedAlarm();	   /* B 手柄重新接入后清除运行中拔手柄留下的未连接报警，恢复自动选中条件。 */
		Pubinterface_SaveRecognizeToMemory(CHANNEL_B);		   /* 扫描结果先保存到 MemoryMsgB，运行中插入 B 不直接替换 A 的当前参数。 */
		if (Pubinterface_ShouldAutoSelectPluggedChannel(CHANNEL_B)) /* 停机且无报警时，新接入 B 才能自动成为当前通道。 */
		{
			Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非运行状态下最后插入且校验通过的 B 通道成为当前选中通道。 */
			if (channel_was_online == 0U) /* B 首次上线才装载默认流量，防止重复扫描覆盖手工调节。 */
			{
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* B 首次接入并被选中时，才用 B 的 Page4 默认流量初始化注水泵。 */
			}
			Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* B 自动成为当前通道后，同步刷新参数区和刀具规格。 */
		}
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 无论是否自动选中，都刷新 B 在线图标和当前高亮状态。 */
		Pubinterface_ClearCommonSocketToolMissingAlarm();	   /* B 通道 EPC 刀具信息装载成功后清除公共接头缺刀具临时报警。 */
		break;

	case SCREENKey_UNPLUG_A: // 拔出A
		close_verify_alarm = (uint8_t)Handlescan_TakeVerifyAlarmCloseRequest(CHANNEL_A); /* 在任何 UI 清屏前接收 A 通道关窗请求，后续统一放到队列复位之后执行。 */
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_A); /* 先记录拔出前 A 是否为当前选中通道。 */
		current_channel_run_interrupted = (uint8_t)((current_channel_unplugged != 0U) &&
													 ((WorkMessage.runflag_work != false) ||
													  MotorUart_DidDriverAlarmStartDuringRun())); /* 拔出确认要等 500 ms，期间驱动报警可能先把运行标志清掉；仍需认定这次是在运行中拔出。 */
		channel_was_online = (uint8_t)WorkMessage.Channel_Aonline; /* 保存拔出前是否在线，重复的离线消息不重复增加插拔计数。 */
		close_idle_touch = (uint8_t)((current_channel_unplugged != 0U) &&
									 (current_channel_run_interrupted == 0U) &&
									 (WorkMessage.touchactive_work == TOUCHWORK) &&
									 (WorkMessage.drivetype_work == TOUCHWORK) &&
									 (WorkMessage.hmiactive_work == 0U)); /* 只关闭本机触控已打开但未运行的窗口；触控运行中拔出仍须停机、报警并等待松开，外控不走这条处理。 */
		if (channel_was_online != 0U) /* A 原来在线时才记一次拔出，让旧 A 数据任务失效。 */
		{
			Handle_AdvanceIdentityGeneration(CHANNEL_A); /* 先增加插拔计数，读取下一页数据前会停止原 A 请求，再把 A 标为离线。 */
		}
		WorkMessage.Channel_Aonline = false;					   /* A 拔出后立刻离线，心跳会报告 A 不可用。 */
		memset(&MemoryMsgA, 0, sizeof(MemoryMsgA));			   /* A 离线时清空 A 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U) /* 只有拔掉当前 A 才需要决定回落 B 或清空当前通道。 */
		{
			if ((current_channel_run_interrupted == 0U) && /* 确认不是运行掉线、无报警且 B 在线时才允许直接回落。 */
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Bonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_B);		   /* 非工作状态拔掉当前 A 时，若 B 仍在线，则回落选中 B，保持“非工作态有在线手柄即有选中通道”。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_B); /* 当前通道回落到 B 后，注水泵目标流量也同步改为 B 的 Page4 默认值。 */
				Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_B); /* 回落到 B 后刷新参数区，清掉 A 通道残留显示。 */
			}
			else
			{
				if (current_channel_run_interrupted != 0U) /* 当前 A 运行掉线或驱动87先到时都必须切换为80号掉线报警。 */
				{
					Handle_RequestRemainingOnlineAfterUnplug(CHANNEL_B); /* A 运行中拔出时记住仍在线的 B；等待停止操作和报警处理完成后再自动选择 B。 */
					Pubinterface_StopRunningHandleOnUnplug();  /* A 运行中拔出时立即请求停电机、停随手柄运行的注水泵，并按当前控制方式显示掉线报警。 */
				}
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 B 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 B 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.raw_tool_type = 0U;				   /* 同步清当前原始刀具型号，避免拔出后驱动侧读取旧 PXM/PXP/RFID 代号。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* A 运行中拔出先取消当前通道选择；按控制方式完成松开或退出、报警处理后，才允许恢复 B。 */
				Pubinterface_ClearSelectedChannelDisplay();	   /* 没有当前通道时关闭参数区，避免屏幕保留离线通道信息。 */
				Pubinterface_ApplyInjectionPumpDefaultFlow(0U);  /* 当前无选中手柄时，注水泵流量回到程序默认 30，显示值和下次实际运行目标保持一致。 */
			}
		}
		if ((WorkMessage.Channel_Aonline == false) && (WorkMessage.Channel_Bonline == false)) /* A/B 都离线时必须清空所有当前选择显示。 */
		{
			Handle_ClearSelection(); /* 最后一个手柄拔出时强制进入无手柄状态，避免方向按钮和自动识别按钮残留。 */
		}
		Pubinterface_SendHandleDisplay(CHANNEL_A, 0U, false, false); /* A 通道拔出后立即暗灭 A 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 B 仍在线，报警期间先显示在线未选中，控制源退出后再自动高亮 B。 */
		Pubinterface_RefreshHandleUnplugAlarmDisplay();		   /* 无手柄清屏可能复位 UI 队列，拔出事件收尾时补发仍有效的掉线报警弹窗。 */
		UIDP_RequestHandleDisplayReplay(); /* A 已标为离线，安排约 60 ms 和 120 ms 后重新发送 A/B 当前图标，减少屏幕偶尔漏收一次消息的影响。 */
		break;

	case SCREENKey_UNPLUG_B: // 拔出B
		close_verify_alarm = (uint8_t)Handlescan_TakeVerifyAlarmCloseRequest(CHANNEL_B); /* 取出并清掉 B 自己的关闭窗口请求，避免 A/B 同时异常时关闭另一侧仍有效的报警。 */
		current_channel_unplugged = (uint8_t)(WorkMessage.channel_work == CHANNEL_B); /* 先记录拔出前 B 是否为当前选中通道。 */
		current_channel_run_interrupted = (uint8_t)((current_channel_unplugged != 0U) &&
													 ((WorkMessage.runflag_work != false) ||
													  MotorUart_DidDriverAlarmStartDuringRun())); /* B 拔出前若驱动先报警并清了运行标志，仍按运行中拔出处理，统一显示 80 号掉线报警。 */
		channel_was_online = (uint8_t)WorkMessage.Channel_Bonline; /* 保存 B 拔出前在线态，过滤重复离线消息。 */
		close_idle_touch = (uint8_t)((current_channel_unplugged != 0U) &&
									 (current_channel_run_interrupted == 0U) &&
									 (WorkMessage.touchactive_work == TOUCHWORK) &&
									 (WorkMessage.drivetype_work == TOUCHWORK) &&
									 (WorkMessage.hmiactive_work == 0U)); /* B 通道使用与 A 相同的触控待运行判定，防止 A/B 插拔行为不一致。 */
		if (channel_was_online != 0U) /* B 原来在线时才增加一次插拔变化计数。 */
		{
			Handle_AdvanceIdentityGeneration(CHANNEL_B); /* 增加 B 插拔计数，旧 B 数据任务在下一次检查时停止。 */
		}
		WorkMessage.Channel_Bonline = false;					   /* B 拔出后立刻离线，心跳会报告 B 不可用。 */
		memset(&MemoryMsgB, 0, sizeof(MemoryMsgB));			   /* B 离线时清空 B 通道记忆，避免后续手动切换读到旧 EEPROM 参数。 */
		if (current_channel_unplugged != 0U) /* 只有拔掉当前 B 才需要决定回落 A 或清空当前通道。 */
		{
			if ((current_channel_run_interrupted == 0U) && /* 确认不是运行掉线、无报警且 A 在线时才允许直接回落。 */
				(WorkMessage.alarm_flag == false) &&
				(WorkMessage.Channel_Aonline == true))
			{
				Pubinterface_LoadChannelMemory(CHANNEL_A);		   /* 非工作状态拔掉当前 B 时，若 A 仍在线，则回落选中 A，匹配现场先插 A 再插 B 再拔 B 的预期。 */
				Pubinterface_ApplyChannelDefaultInjectionFlow(CHANNEL_A); /* 当前通道回落到 A 后，注水泵目标流量同步改为 A 的 Page4 默认值。 */
				Pubinterface_RefreshSelectedChannelDisplay(CHANNEL_A); /* B 拔出后回落到 A 时立即刷新参数区，避免继续显示 B 的方向和刀具状态。 */
			}
			else
			{
				if (current_channel_run_interrupted != 0U) /* 当前 B 运行掉线或驱动87先到时都必须切换为80号掉线报警。 */
				{
					Handle_RequestRemainingOnlineAfterUnplug(CHANNEL_A); /* B 运行中拔出时记住仍在线的 A；等待停止操作和报警处理完成后再自动选择 A。 */
					Pubinterface_StopRunningHandleOnUnplug();  /* B 运行中拔出时立即请求停电机、停随手柄运行的注水泵，并按当前控制方式显示掉线报警。 */
				}
				WorkMessage.hand_model = 0U;					   /* 运行中、报警中或无 A 在线时，清当前手柄型号，防止离线手柄继续被手柄键扫描。 */
				WorkMessage.tool_type = 0U;					   /* 运行中、报警中或无 A 在线时，清当前刀具类型，界面进入未选中状态。 */
				WorkMessage.raw_tool_type = 0U;				   /* 同步清当前原始刀具型号，避免拔出后驱动侧读取旧 PXM/PXP/RFID 代号。 */
				WorkMessage.channel_work = CHANNEL_NONE;		   /* B 运行中拔出先取消当前通道选择；按控制方式完成松开或退出、报警处理后，才允许恢复 A。 */
				Pubinterface_ClearSelectedChannelDisplay();	   /* 没有当前通道时关闭参数区，避免屏幕保留离线通道信息。 */
				Pubinterface_ApplyInjectionPumpDefaultFlow(0U);  /* 当前无选中手柄时，注水泵流量回到程序默认 30，避免拔掉最后手柄后残留 65。 */
			}
		}
		if ((WorkMessage.Channel_Aonline == false) && (WorkMessage.Channel_Bonline == false)) /* A/B 都离线时不能继续保留任何当前通道。 */
		{
			Handle_ClearSelection(); /* 最后一个手柄拔出时强制清空当前通道，补齐 current_channel_unplugged 为 0 时的清屏路径。 */
		}
		Pubinterface_SendHandleDisplay(CHANNEL_B, 0U, false, false); /* B 通道拔出后立即暗灭 B 手柄区域。 */
		Pubinterface_RefreshOnlineHandleDisplay();			   /* 若 A 仍在线，报警期间先显示在线未选中，控制源退出后再自动高亮 A。 */
		Pubinterface_RefreshHandleUnplugAlarmDisplay();		   /* 无手柄清屏可能复位 UI 队列，拔出事件收尾时补发仍有效的掉线报警弹窗。 */
		UIDP_RequestHandleDisplayReplay(); /* B 已标为离线，再安排两次图标发送；只补显示，不重复执行停机、报警或蜂鸣。 */
		break;

	default:
		break;												   /* 其它按键不是插拔事件，本函数不处理。 */
	}

	if (close_idle_touch != 0U)
	{
		ControlTypeActive(SCREENKey_TouchEXIT); /* 等插拔清屏和队列清理完成后再退出触控，关闭 70 号窗口、清除触控状态，让屏幕不再占用运行控制。 */
	}

	if ((close_verify_alarm != 0U) && (WorkMessage.alarm_flag == false))
	{
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 在 xQueueReset 之后重新投递关窗命令，确保屏幕90号图与已清零的报警状态一致。 */
	}
}

/*
 * 函数功能：处理屏幕、脚踏或外控发起的开口定位按键，只允许支持开口定位的刨刀手柄执行一次固定角度动作。
 * 输入参数：key_value 为顺时针或逆时针开口定位按键值。
 * 返回参数：无。
 */
void ToolPosActive(uint8_t key_value)
{

	if (WorkMessage.runflag_work == true || WorkMessage.alarm_flag == true) /* 运行或报警时禁止开口点动，避免叠加新的电机动作。 */
		return;
	if (WorkMessage.hand_model == 0U) /* 没有有效手柄时开口定位没有硬件目标。 */
		return; /* 未识别有效手柄时不允许开口定位，避免屏幕残留刀具状态误触发驱动。 */
	if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B)) /* 无 A/B 归属时底层不能安全选择驱动通道。 */
		return; /* ToolPosMay 底层非 A 会落到 B，所以上层必须先确认当前通道是 A 或 B。 */

	if (Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model, WorkMessage.tool_type) == false) /* 只有支持开口定位的分体刨刀组合允许点动。 */
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
