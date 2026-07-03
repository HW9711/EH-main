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
kernel_task_t HANDLEKEYTaskHandle;

//键值按下状态
static bool sHandleKEYValue[2] = { false };

//============================================================================
// 函数名称: HAL_GPIO_EXTI_Callback()
// 功能描述: 手柄（PXBA）按键中断回调函数
// 输　  入:
// 输    出:
// 函数说明: false抬起，true按下
//============================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SimUart_HandleExti(GPIO_Pin);

	switch (GPIO_Pin)
	{
		case BOARD_RES_HANDLE_KEY1_PIN : sHandleKEYValue[1] = (sHandleKEYValue[1] ? false : true); break; //H_KEY1
		case BOARD_RES_HANDLE_KEY0_PIN : sHandleKEYValue[0] = (sHandleKEYValue[0] ? false : true); break; //H_KEY
		default : break;
	}
}

//============================================================================
// 函数名称: HandleKey_GetKeyValue()
// 功能描述: 手柄（PXBA）按键状态
// 输　  入: 
// 输    出:
// 函数说明: 
//============================================================================
bool HandleKey_GetKeyValue(uint8_t keynum)
{
	return 0; //sHandleKEYValue[keynum];
}

/*
 * 手柄按键迁移到 V1.8 新接口后，不再直接写旧的运行/报警全局标志。
 * 这里保留原有按键去抖和长按窗口，只把输出改成 WorkMessage、ControlSignalMessage
 * 以及 SendKeyBehMessage()/SendAlarmMessage()，确保后续 sscKEYBH 统一分发。
 */
static void HandleKey_SetAlarm(uint8_t alarm_value)
{
	/* 手柄按键产生的普通报警统一交给 WorkAlarm_Set，同步 WorkMessage、蜂鸣和 sscUIDP 屏幕显示。 */
	WorkAlarm_Set(alarm_value);
}

static void HandleKey_ClearAlarm(uint8_t alarm_value)
{
	/* 只清当前按键模块自己关心的报警码，避免误清其它模块仍存在的故障。 */
	WorkAlarm_ClearIf(alarm_value);
}

static void HandleKey_SetMotorRun(bool enable)
{
	if (enable)
	{
		if (Pubinterface_CheckCommonSocketToolReadyForRun() == false)
		{
			return; /* 公共接头基座未读取到 EPC 刀具头时，实体键启动只提示“请连接手柄”，不下发运行。 */
		}
		/* 手柄按键启动电机前先占用手柄控制权，若其它方式正在控制则本次按键无效。 */
		if (ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE) == false)
		{
			return;
		}
	}
	ControlSignalMessage.handle_control_flag = enable;
	WorkMessage.runflag_work = enable;
	if (enable == false)
	{
		/* 手柄停止后释放手柄控制权，允许脚踏、屏幕或上位机重新申请。 */
		ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_HANDLE);
	}
	SendKeyBehMessage(HANDLEKey, enable ? HANDLEKey_motor_start : HANDLEKey_motor_stop);
}


//
void HandleKey_Scan0SSC()
{
	static uint8_t KEY0_ADC_time = 0;
	static uint8_t key0_up_down = 0;//按键按松开标志
	static uint8_t start_flags=0;
	static uint8_t activation_flag=0;
	
	if(WorkMessage.channel_work!=CHANNEL_A||WorkMessage.hand_model!=PXBA_ONLINES){
		return;
	}
if(WorkMessage.alarm_value==11||WorkMessage.alarm_value==12){return;}
		if(KEY0_STATUS() == 0)
			{
					key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(WorkMessage.drivetype_work==JTWORK)
						{
							//报警，请选择脚控启动
							HandleKey_SetAlarm(WORK_ALARM_FOOT_SELECTED);
							HandleKey_SetMotorRun(false);
							return;
						}
						
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
								if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_COMM_ERROR)
								{
									activation_flag=0;
									HandleKey_SetMotorRun(false);
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										HandleKey_SetMotorRun(true);
									}
									else
									{
										activation_flag=0;
										HandleKey_SetMotorRun(false);
									}
								} 
					}
					}
			}
			else
			{
				KEY0_ADC_time=0;
				key0_up_down++;
				if(key0_up_down<10)return;
				start_flags=1;
				
				if(key0_up_down>55)
				{
					if(WorkMessage.alarm_value==WORK_ALARM_FOOT_SELECTED)
					{
						HandleKey_ClearAlarm(WORK_ALARM_FOOT_SELECTED);
						key0_up_down=0;
					}
					else if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_COMM_ERROR)
					{
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
						HandleKey_ClearAlarm(WORK_ALARM_MOTOR_COMM_ERROR);
						key0_up_down=0;
						}
					}
					else
					{
						key0_up_down=0;
					}
				}
				
			}
	}
void HandleKey_Scan1SSC()
{
	static uint8_t KEY0_ADC_time = 0;
  static uint8_t key0_up_down = 0;//按键按松开标志
	static uint8_t start_flags=0;
	static uint8_t activation_flag=0;
	if(WorkMessage.channel_work!=CHANNEL_B||WorkMessage.hand_model!=PXBA_ONLINES){
		return;}
			if(WorkMessage.alarm_value==11||WorkMessage.alarm_value==12){return;}
			if(KEY1_STATUS() == 0)
			{
				key0_up_down=0;
				KEY0_ADC_time++;
				if(KEY0_ADC_time<15)return;
				KEY0_ADC_time=0;
				if(start_flags)
					{
						start_flags=0;
						if(WorkMessage.drivetype_work==JTWORK)
						{
							//报警，请选择脚控启动
							HandleKey_SetAlarm(WORK_ALARM_FOOT_SELECTED);
							HandleKey_SetMotorRun(false);
					
							return;
						}
				
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
								if(WorkMessage.alarm_value==WORK_ALARM_HALL_ERROR)
								{
									activation_flag=0;
									HandleKey_SetMotorRun(false);
								}
								else{
									if(activation_flag==0){
										activation_flag=1;
										HandleKey_SetMotorRun(true);
									}
									else
									{
										activation_flag=0;
										HandleKey_SetMotorRun(false);
									}
						}
					}
					}
			}
			else
			{
				KEY0_ADC_time=0;
				key0_up_down++;
				if(key0_up_down<10)return;
				start_flags=1;
				
				if(key0_up_down>55)
				{
					if(WorkMessage.alarm_value==WORK_ALARM_FOOT_SELECTED)
					{
						HandleKey_ClearAlarm(WORK_ALARM_FOOT_SELECTED);
						key0_up_down=0;
						activation_flag=0;
					}
					else if(WorkMessage.alarm_value==WORK_ALARM_HALL_ERROR)
					{
						if(WorkMessage.drivetype_work==HANDLEWORK)
						{
						HandleKey_ClearAlarm(WORK_ALARM_HALL_ERROR);
						key0_up_down=0;
							activation_flag=0;
						}
					}
					else
					{
						key0_up_down=0;
					}
				}
			}
}

#define HANDLE_KEY_DEBOUNCE_COUNT 2U /* 30ms任务连续2次确认电平，约60ms去抖，避免触点抖动误启停。 */
#define HANDLE_KEY_PRESSED_LEVEL GPIO_PIN_RESET /* 硬件默认上拉，按键按下后对应IO被拉低。 */

typedef struct
{
	uint8_t low_count;		/* 连续低电平计数，用于确认按键已经稳定按下。 */
	uint8_t high_count;		/* 连续高电平计数，用于确认按键已经稳定松开。 */
	bool stable_pressed;	/* 去抖后的按下状态，业务层只使用这个稳定结果。 */
	bool press_event;		/* 稳定按下沿事件，只在松开后再次按下并完成消抖时置位一个扫描周期。 */
} HandleRunKeyDebounce_t;

static HandleRunKeyDebounce_t s_handle_run_key_a_filter = {0U, 0U, false, false}; /* A通道实体键去抖状态。 */
static HandleRunKeyDebounce_t s_handle_run_key_b_filter = {0U, 0U, false, false}; /* B通道实体键去抖状态。 */
static uint8_t s_handle_run_key_owner_channel = CHANNEL_NONE;			   /* 当前由实体键启动的通道，防止另一通道松开误停。 */
#define HANDLE_MODE_MISMATCH_ALARM_MS 3000U /* 脚控已选中时误按手柄实体键，82 号弹窗保持 3 秒后自动关闭。 */
static uint8_t s_handle_foot_selected_timed_alarm_active = 0U; /* 记录 82 号错模式弹窗是否由手柄键模块显示，防止到期误清其它报警。 */
static uint32_t s_handle_foot_selected_timed_alarm_tick = 0U; /* 记录 82 号错模式弹窗开始时间，用于 3 秒自动清除。 */

/*
 * 函数功能：脚控已选中时按手柄实体运行键，上报 82 号 3 秒临时弹窗。
 * 输入参数：无，报警码固定使用 WORK_ALARM_FOOT_SELECTED。
 * 返回参数：无。
 */
static void HandleRunKey_RaiseFootSelectedTimedAlarm(void)
{
	uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只读取 Value[0]，其余补零避免沿用上一条报警参数。 */

	display_value[0] = WORK_ALARM_FOOT_SELECTED; /* 82 号图提示当前脚控已选中，用户应使用脚踏启动。 */
	SendAlarmMessageTimed(WORK_ALARM_FOOT_SELECTED, HANDLE_MODE_MISMATCH_ALARM_MS); /* 错模式提示只响 3 秒，不写全局报警锁存。 */
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
 * 函数功能：周期维护 82 号错模式临时弹窗，达到 3 秒后自动关闭。
 * 输入参数：无，直接读取弹窗归属和 HAL 毫秒 tick。
 * 返回参数：无。
 */
static void HandleRunKey_ServiceFootSelectedTimedAlarm(void)
{
	if (s_handle_foot_selected_timed_alarm_active == 0U)
	{
		return; /* 当前没有由手柄键模块显示的 82 号弹窗，不处理屏幕报警区。 */
	}

	if ((uint32_t)(HAL_GetTick() - s_handle_foot_selected_timed_alarm_tick) < HANDLE_MODE_MISMATCH_ALARM_MS)
	{
		return; /* 3 秒保持时间未到，继续显示脚控已选中提示。 */
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
	return ((hand_model == PXBA_ONLINES) || (hand_model == PXBB_ONLINES)); /* 空心钻A/B型手柄均允许实体键控制电机。 */
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
 * 函数功能：对实体按键原始电平做去抖，只在稳定按下沿输出一次翻转事件。
 * 输入参数：filter 对应通道的去抖状态；raw_level 本周期GPIO原始电平。
 * 返回参数：true 表示本周期产生稳定按下沿事件，false 表示本周期无新按下事件。
 */
static bool HandleRunKey_DebouncePressEvent(HandleRunKeyDebounce_t *filter, GPIO_PinState raw_level)
{
	filter->press_event = false; /* 每个30ms扫描周期先清一次性事件，防止长按期间重复翻转启停。 */
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
			filter->stable_pressed = false; /* 连续高电平达到阈值后只复位按下状态，不产生停止事件。 */
		}
	}

	return filter->press_event; /* 返回稳定按下沿事件，供通道控制状态机执行按一次启动、再按一次停止。 */
}

/*
 * 函数功能：通过手柄实体按键设置电机运行状态，并维护控制仲裁。
 * 输入参数：enable true 表示启动电机，false 表示停止电机。
 * 返回参数：true 表示本次状态写入成功，false 表示启动时未抢到手柄控制权。
 */
static bool HandleRunKey_SetMotorRun(bool enable)
{
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
 * 函数功能：按实体键目标通道装载通道记忆，并准备默认速度。
 * 输入参数：channel 本次按下的实体键对应通道。
 * 返回参数：true 表示启动前参数准备完成，false 表示通道不可启动。
 */
static bool HandleRunKey_PrepareRunChannel(uint8_t channel)
{
	if (HandleRunKey_IsChannelReady(channel) == false)
	{
		return false; /* 通道未在线或校验未通过时不启动，避免坏手柄或空通道误转。 */
	}

	if (WorkMessage.channel_work != channel)
	{
		HandleSwitchActive((channel == CHANNEL_A) ? SCREENKey_HANDLE_A : SCREENKey_HANDLE_B); /* 空闲时按哪个实体键就切到哪个通道，并刷新屏幕高亮。 */
	}

	if ((WorkMessage.channel_work != channel) || (HandleRunKey_IsChannelReady(channel) == false))
	{
		return false; /* 切换后再次确认，防止切换被报警、离线或其它状态拦截。 */
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
 * 函数功能：处理单个通道实体键的按一次启动、再按一次停止状态机。
 * 输入参数：channel 当前处理的通道；press_event 去抖后的稳定按下沿事件。
 * 返回参数：无。
 */
static void HandleRunKey_Process(uint8_t channel, bool press_event)
{
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
			HandleRunKey_RaiseFootSelectedTimedAlarm(); /* 当前选中脚控时误按手柄键，只提示 82 号报警，不启动手柄。 */
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
		s_handle_run_key_owner_channel = channel; /* 启动成功后记录owner通道，后续只允许该通道松开时停止。 */
	}
}

/*
 * 函数功能：轮询A/B手柄实体运行键，按住运行、松开停止。
 * 输入参数：无。
 * 返回参数：无。
 */
static void HandleKey_ScanRunKeys(void)
{
	bool key_a_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_a_filter, HANDLE_RUN_KEY_A_STATUS()); /* 读取A通道PE12实体键并生成一次稳定按下沿事件。 */
	bool key_b_press_event = HandleRunKey_DebouncePressEvent(&s_handle_run_key_b_filter, HANDLE_RUN_KEY_B_STATUS()); /* 读取B通道PE13实体键并生成一次稳定按下沿事件。 */

	HandleRunKey_Process(CHANNEL_A, key_a_press_event); /* 先处理A，两个按键同周期稳定按下时A按固定顺序优先。 */
	HandleRunKey_Process(CHANNEL_B, key_b_press_event); /* 再处理B，若A已取得owner则B会被忽略。 */
}

/*
 * 函数功能：手柄实体按键周期扫描任务，处理PE12/PE13按住运行、松开停止。
 * 输入参数：event 调度器事件参数，当前任务不使用。
 * 返回参数：无。
 */
void HANDLEKEYTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN HANDLEKEYTaskFunc */
  /* Infinite loop */
	HandleRunKey_ServiceFootSelectedTimedAlarm(); /* 手柄键任务周期维护 82 号临时弹窗，保证 3 秒后自动消失。 */
	if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))
		return;
	HandleKey_ScanRunKeys(); /* 实体键采用稳定按下沿翻转启停：按一次启动，再按一次停止，松开只复位下一次按下资格。 */
  /* USER CODE END HANDLEKEYTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 15
 */
//============================================================================
void HandleKeyScan_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	Kernel_TaskCreate(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc);
	Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
