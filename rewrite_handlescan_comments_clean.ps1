$root = 'D:\Study_HL\F413EXOsSSCH_RTOSV1.5'
$handlePath = Join-Path $root 'User\Application\Handle\handlescan.c'

$handleText = Get-Content -Path $handlePath -Raw -Encoding Default
$pattern = '(?s)void HandlescanA_Fun_SSC\(void\)\s*\{.*?\n\}\s*\n\s*/\* B通道当前不参与调度，保留空实现以兼容旧接口 \*/\s*void HandlescanB_Fun_SSC\(void\)'
$replacement = @'
void HandlescanA_Fun_SSC(void)
{
	uint8_t is_inserted;
	AT24CS32_CRC_Result verify_result;
	AT24CS32_CRC_Status verify_status;
	uint8_t read_status;
	uint8_t model;

	/*
	 * 先读取 A 通道的短接检测脚。
	 * 这里把 PD0 高电平视为“已插入候选”，把 PD0 低电平视为“已拔出候选”。
	 * 具体是否真正成立，还要经过下面的去抖状态机确认。
	 */
	is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_SET);

	/*
	 * 当前检测到低电平时，优先走“拔出链路”。
	 * 这部分逻辑覆盖两种情况：
	 * 1. 手柄原本已经在线，现在发生正常拔出；
	 * 2. 已经进入拔出去抖阶段，并且继续保持低电平。
	 */
	if (!is_inserted)
	{
		/* 进入拔出分支后，先清掉插入相关计数，避免旧状态残留到下一次插入流程。 */
		s_handleA_debounce.in_debounce_ticks = 0U;
		s_a_verify_start_wait_ticks = 0U;

		/*
		 * 只有“当前已在线”或“已经在做拔出去抖”时，才需要继续推进拔出确认。
		 * 如果本来就处于空闲态，则保持 IDLE，不重复通知 UI。
		 */
		if ((s_a_stage == HANDLESCAN_A_STAGE_ONLINE) || (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_OUT))
		{
			/*
			 * 第一次从 ONLINE 进入低电平时，先切换到拔出去抖阶段。
			 * 去抖计数必须从 0 开始，确保只有“连续稳定低电平”才算真正拔出。
			 */
			if (s_a_stage != HANDLESCAN_A_STAGE_DEBOUNCE_OUT)
			{
				s_handleA_debounce.out_debounce_ticks = 0U;
				s_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_OUT;
			}

			/* 拔出去抖未达到阈值时直接返回，避免接触抖动造成误离线。 */
			if (++s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
			{
				return;
			}

			/* 连续低电平达到阈值，确认本次拔出已经稳定成立。 */
			Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OUT_DEBOUNCE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);

			/*
			 * 如果设备正在运行，则沿用原有“运行中插拔报警”逻辑。
			 * 这种情况下优先报警并停止运行，不继续做普通离线收尾。
			 */
			if (Handlescan_HandleRunningPlugAlarm())
			{
				return;
			}

			/*
			 * 正常离线收尾：
			 * 1. 清掉 A 通道在线状态与识别标志；
			 * 2. 清掉缓存型号；
			 * 3. 通知 UI 显示“手柄已拔出”。
			 */
			s_handleA_debounce.out_debounce_ticks = 0U;
			s_a_online_latched = 0U;
			s_a_stage = HANDLESCAN_A_STAGE_IDLE;
			s_a_last_alarm = 0U;
			Workvalue_s.Achanell_online_flag = stop_flag;
			Workvalue_s.A_ChipRecognition_FLAG = stop_flag;
			Workvalue_s.A_ShortCircuitRecognition_FLAG = stop_flag;
			ChannelValue_s.A.hand_model = 0U;
			Workvalue_s.ScreenKey_data = 26U;
			K1_OFF();
			Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U);
		}
		else
		{
			/*
			 * 当前并未处于在线态，只需要保持空闲态即可。
			 * 同时把拔出去抖计数清零，保证下次重新插入时从头计数。
			 */
			s_handleA_debounce.out_debounce_ticks = 0U;
			s_a_stage = HANDLESCAN_A_STAGE_IDLE;
		}

		return;
	}

	/*
	 * 走到这里说明短接脚当前为高电平，也就是“插入候选”成立。
	 * 因为现在准备走插入链路，所以先清掉拔出去抖计数，
	 * 并同步置位 A 通道短接识别成功标志。
	 */
	s_handleA_debounce.out_debounce_ticks = 0U;
	Workvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;

	/*
	 * IDLE -> DEBOUNCE_IN：
	 * 第一次从空闲态检测到插入时，不直接访问 EEPROM，
	 * 而是先进入插入去抖阶段，滤掉连接器接触抖动和上电瞬态。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_IDLE)
	{
		s_handleA_debounce.in_debounce_ticks = 0U;
		s_a_verify_start_wait_ticks = 0U;
		s_a_last_alarm = 0U;
		s_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_IN;
	}

	/*
	 * 插入去抖阶段：
	 * 只有连续高电平达到阈值，才允许进入认证前等待阶段。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_IN)
	{
		if (++s_handleA_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
		{
			return;
		}

		s_handleA_debounce.in_debounce_ticks = HANDLESCAN_INSERT_DEBOUNCE_TICKS;
		s_a_verify_start_wait_ticks = 0U;
		s_a_stage = HANDLESCAN_A_STAGE_WAIT_VERIFY;
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_IN_DEBOUNCE_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS);
		return;
	}

	/*
	 * 认证前等待阶段：
	 * 插入去抖通过后，再额外等待一小段时间，
	 * 给手柄侧 EEPROM 上电稳定、I2C 总线空闲和连接器接触建立留出余量。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_WAIT_VERIFY)
	{
		if (++s_a_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
		{
			return;
		}

		s_a_verify_start_wait_ticks = 0U;
		s_a_stage = HANDLESCAN_A_STAGE_VERIFY;
	}

	/*
	 * 单次认证阶段：
	 * 这里只做一次 CRC 认证，不会在手柄持续插入时反复认证。
	 * 如果认证失败，就停在 VERIFY_FAIL，等待重新插拔后再触发下一轮。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_VERIFY)
	{
		/* 运行态插拔优先触发保护，不继续访问 EEPROM。 */
		if (Handlescan_HandleRunningPlugAlarm())
		{
			return;
		}

		/*
		 * 发起本轮认证前先清空最近一次 EEPROM 调试信息。
		 * 这样如果认证失败，HSDBG 报文就一定对应本轮认证中的最后一次底层访问。
		 */
		AT24CS32_ClearLastDebugInfo();
		verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result);
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

		if (verify_status != AT24CS32_CRC_STATUS_OK)
		{
			/*
			 * 认证失败时只记录一次报警并进入失败保持态，
			 * 避免手柄一直插着时每个扫描周期都反复重试、反复刷报文。
			 */
			Workvalue_s.Alarm_value = Handlescan_MapVerifyStatusToAlarm(verify_status, 0U);
			Workvalue_s.beep_Alarm_flag = 1U;
			if (Workvalue_s.Alarm_value != s_a_last_alarm)
			{
				s_a_last_alarm = Workvalue_s.Alarm_value;
				Handlescan_DebugTraceI2cDetail(1U);
				Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
			}
			s_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
			return;
		}

		/* CRC 认证通过后，继续读取 EEPROM 的业务信息区。 */
		s_a_stage = HANDLESCAN_A_STAGE_READ_INFO;
	}

	/*
	 * 信息区读取阶段：
	 * CRC 认证通过只说明 EEPROM 数据整体有效，
	 * 真正驱动 UI 切换还需要把型号等业务字段读出来。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_READ_INFO)
	{
		/*
		 * 读取信息区前同样先清空调试缓存。
		 * 如果这里失败，HSDBG 报文就会准确落在 0x0020 起始的信息区访问上。
		 */
		AT24CS32_ClearLastDebugInfo();
		read_status = (uint8_t)AT24CS32_ReadBytes_I2C2(HANDLESCAN_INFO_ADDR, s_a_info_buf, HANDLESCAN_INFO_SIZE);
		if (read_status == 0U)
		{
			Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
			Workvalue_s.beep_Alarm_flag = 1U;
			if (Workvalue_s.Alarm_value != s_a_last_alarm)
			{
				s_a_last_alarm = Workvalue_s.Alarm_value;
				Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_READ_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL);
				Handlescan_DebugTraceI2cDetail(1U);
				Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
			}
			s_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
			return;
		}

		/*
		 * 当前约定信息区第 0 字节为手柄型号。
		 * 如果型号字段为 0，则认为信息区对当前业务无效，按失败处理。
		 */
		model = s_a_info_buf[HANDLESCAN_MODEL_OFFSET];
		if (model == 0U)
		{
			Workvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
			Workvalue_s.beep_Alarm_flag = 1U;
			if (Workvalue_s.Alarm_value != s_a_last_alarm)
			{
				s_a_last_alarm = Workvalue_s.Alarm_value;
				Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, model);
				Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
			}
			s_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
			return;
		}

		/*
		 * 到这里说明：
		 * 1. 插入去抖成功；
		 * 2. CRC 认证成功；
		 * 3. 信息区读取成功；
		 * 4. 型号字段有效。
		 * 因此正式把 A 通道置为在线，并只通知一次 UI 切换界面。
		 */
		s_a_online_latched = 1U;
		s_a_last_alarm = 0U;
		ChannelValue_s.A.hand_model = model;
		Workvalue_s.Achanell_online_flag = start_flag;
		Workvalue_s.A_ChipRecognition_FLAG = start_flag;
		Workvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;
		Workvalue_s.ScreenKey_data = 24U;
		s_a_stage = HANDLESCAN_A_STAGE_ONLINE;
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, model);
		return;
	}

	/*
	 * VERIFY_FAIL 和 ONLINE 都属于“保持态”：
	 * VERIFY_FAIL 表示本轮插入失败，等待重新插拔；
	 * ONLINE 表示本轮插入成功，等待后续拔出去抖。
	 * 这两个状态都不需要在每个扫描周期重复动作，因此直接返回。
	 */
	if ((s_a_stage == HANDLESCAN_A_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_A_STAGE_ONLINE))
	{
		return;
	}
}

/* B通道当前不参与调度，保留空实现以兼容旧接口 */
void HandlescanB_Fun_SSC(void)
'@

$handleText = [regex]::Replace($handleText, $pattern, $replacement)
Set-Content -Path $handlePath -Value $handleText -Encoding Default
