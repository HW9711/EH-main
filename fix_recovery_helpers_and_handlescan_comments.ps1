$root = 'D:\Study_HL\F413EXOsSSCH_RTOSV1.5'
$at24Path = Join-Path $root 'User\Peripheral\EEPROM\at24cs32.c'
$handlePath = Join-Path $root 'User\Application\Handle\handlescan.c'

$at24Text = Get-Content -Path $at24Path -Raw -Encoding Default

$patternAt24 = '(?s)static uint8_t AT24CS32_IsI2cReady\(I2C_HandleTypeDef \*hi2c\)\s*\{\s*return \(hi2c != NULL\) \? 1U : 0U;\s*\}\s*/\*.*?计算页起始字节地址.*?\*/\s*static uint16_t AT24CS32_PageToAddr\(uint16_t page_index\)\s*\{\s*return \(uint16_t\)\(page_index \* AT24CS32_PAGE_SIZE\);\s*\}'
$replacementAt24 = @'
static uint8_t AT24CS32_IsI2cReady(I2C_HandleTypeDef *hi2c)
{
    return (hi2c != NULL) ? 1U : 0U;
}

/*
 * 判断当前 I2C 句柄是否就是手柄认证所使用的 I2C2。
 * 现场问题目前集中在 I2C2，所以后续恢复逻辑会优先对 I2C2 做定向处理。
 */
static uint8_t AT24CS32_IsI2C2Handle(I2C_HandleTypeDef *hi2c)
{
    return (hi2c == &hi2c2) ? 1U : 0U;
}

/*
 * 在真正访问 EEPROM 前，先主动确认器件是否 ready。
 * 这一步的意义是把“器件未应答/总线未空闲”和“正式读写失败”区分开，
 * 方便现场根据 HSDBG 报文判断失败到底发生在访问前还是访问中。
 */
static HAL_StatusTypeDef AT24CS32_WaitDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c,
                                 dev_addr,
                                 AT24CS32_READY_TRIALS,
                                 AT24CS32_I2C_TIMEOUT_MS);
}

/*
 * 判断当前失败是否值得执行一次 I2C 软恢复。
 * 这里重点关注 HAL_BUSY、HAL_TIMEOUT 以及 HAL_I2C_ERROR_TIMEOUT，
 * 因为这几种情况都与当前现场看到的 BUSY Flag 超时高度一致。
 */
static uint8_t AT24CS32_ShouldRecoverI2c(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef hal_status)
{
    uint32_t hal_error;

    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return 0U;
    }

    hal_error = HAL_I2C_GetError(hi2c);

    if ((hal_status == HAL_BUSY) || (hal_status == HAL_TIMEOUT)) {
        return 1U;
    }

    if ((hal_error & HAL_I2C_ERROR_TIMEOUT) != 0U) {
        return 1U;
    }

    return 0U;
}

/*
 * 对 I2C 总线执行一次最小侵入的软恢复。
 * 当前优先采用 HAL DeInit + Init 的方式，把外设状态机从 BUSY/TIMEOUT 中拉回来；
 * 这样改动面较小，也更适合先验证是不是“外设状态卡死”导致的首包读取失败。
 */
static void AT24CS32_RecoverI2cBus(I2C_HandleTypeDef *hi2c)
{
    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return;
    }

    if (AT24CS32_IsI2C2Handle(hi2c) != 0U) {
        HAL_I2C_DeInit(&hi2c2);
        MX_I2C2_Init();
    } else if (hi2c == &hi2c3) {
        HAL_I2C_DeInit(&hi2c3);
        MX_I2C3_Init();
    } else {
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);
    }

    HAL_Delay(AT24CS32_RECOVER_DELAY_MS);
}

/* 计算页起始字节地址 */
static uint16_t AT24CS32_PageToAddr(uint16_t page_index)
{
    return (uint16_t)(page_index * AT24CS32_PAGE_SIZE);
}
'@
$at24Text = [regex]::Replace($at24Text, $patternAt24, $replacementAt24)
Set-Content -Path $at24Path -Value $at24Text -Encoding Default

$handleText = Get-Content -Path $handlePath -Raw -Encoding Default
$patternHandle = '(?s)void HandlescanA_Fun_SSC\(void\)\s*\{.*?\n\}\s*\n\s*/\*.*?void HandlescanB_Fun_SSC\(void\)'
$replacementHandle = @'
void HandlescanA_Fun_SSC(void)
{
	uint8_t is_inserted;
	AT24CS32_CRC_Result verify_result;
	AT24CS32_CRC_Status verify_status;
	uint8_t read_status;
	uint8_t model;

	/*
	 * 第一步先读取 A 通道短接检测脚。
	 * 这里把 PD0 高电平视为“插入候选”，把 PD0 低电平视为“拔出候选”；
	 * 至于是否真正完成插入/拔出确认，还要交给后面的去抖状态机判断。
	 */
	is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_SET);

	/*
	 * 低电平分支专门处理“拔出链路”。
	 * 它既覆盖“原本在线后被正常拔出”，也覆盖“已经进入拔出去抖过程，继续保持低电平”的场景。
	 */
	if (!is_inserted)
	{
		/* 进入拔出链路后，插入相关计数全部清零，防止旧状态残留到下一次插入。 */
		s_handleA_debounce.in_debounce_ticks = 0U;
		s_a_verify_start_wait_ticks = 0U;

		/*
		 * 只有当前已经在线，或者已经在做拔出去抖时，才继续推进拔出确认。
		 * 如果本来就是空闲态，就保持 IDLE，不重复通知 UI。
		 */
		if ((s_a_stage == HANDLESCAN_A_STAGE_ONLINE) || (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_OUT))
		{
			/*
			 * 第一次从 ONLINE 检测到低电平时，先切到拔出去抖阶段。
			 * 去抖计数必须从 0 开始，确保是连续稳定低电平才算真正拔出。
			 */
			if (s_a_stage != HANDLESCAN_A_STAGE_DEBOUNCE_OUT)
			{
				s_handleA_debounce.out_debounce_ticks = 0U;
				s_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_OUT;
			}

			/* 拔出去抖尚未达到阈值时直接返回，避免接触抖动造成误离线。 */
			if (++s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
			{
				return;
			}

			/* 连续低电平达到阈值，确认这次拔出已经稳定成立。 */
			Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OUT_DEBOUNCE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);

			/*
			 * 运行态插拔优先走历史保护逻辑：
			 * 触发报警、停止运行，并把后续离线收尾交给系统整体流程处理。
			 */
			if (Handlescan_HandleRunningPlugAlarm())
			{
				return;
			}

			/*
			 * 正常离线收尾：
			 * 1. 清在线锁存与阶段状态；
			 * 2. 清 A 通道在线/识别标志；
			 * 3. 清掉缓存型号；
			 * 4. 通知 UI 切回“手柄已拔出”界面。
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
			 * 当前并未上线，只需要保持空闲态即可。
			 * 同时清空拔出去抖计数，保证下次真正上线后重新开始计时。
			 */
			s_handleA_debounce.out_debounce_ticks = 0U;
			s_a_stage = HANDLESCAN_A_STAGE_IDLE;
		}

		return;
	}

	/*
	 * 走到这里说明短接脚当前为高电平，说明“插入候选”成立。
	 * 因为当前要走插入链路，所以先清掉拔出去抖计数，
	 * 并同步置位 A 通道短接识别标志。
	 */
	s_handleA_debounce.out_debounce_ticks = 0U;
	Workvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;

	/*
	 * IDLE -> DEBOUNCE_IN：
	 * 第一次从空闲态看到插入时，不直接发起认证，而是先进入插入去抖阶段。
	 * 这样可以滤掉插头接触抖动和上电边沿瞬态。
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
	 * 插入去抖通过后，再额外等待一段时间，
	 * 目的是给手柄侧 EEPROM 上电稳定、I2C 总线空闲和连接器接触建立留出余量。
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
	 * 这里只做一次 CRC 认证，不再像旧调试逻辑那样在手柄持续插入时反复认证。
	 * 认证失败后会停在 VERIFY_FAIL，等待用户重新插拔，再重新走完整流程。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_VERIFY)
	{
		/* 如果设备正在运行，运行态插拔要优先触发保护，而不是继续访问 EEPROM。 */
		if (Handlescan_HandleRunningPlugAlarm())
		{
			return;
		}

		/*
		 * 发起本轮认证前先清空最近一次 EEPROM 调试信息。
		 * 这样一旦认证失败，HSDBG 报文就一定对应本轮认证中的最后一次底层访问。
		 */
		AT24CS32_ClearLastDebugInfo();
		verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result);
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

		if (verify_status != AT24CS32_CRC_STATUS_OK)
		{
			/*
			 * 认证失败时只记录一次报警并进入失败保持态。
			 * 这样做的目的就是避免“手柄一直插着时每 10ms 反复认证、反复刷报文”。
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

		/* 认证通过后，才允许继续读取 EEPROM 业务信息区。 */
		s_a_stage = HANDLESCAN_A_STAGE_READ_INFO;
	}

	/*
	 * 信息区读取阶段：
	 * CRC 认证通过只说明 EEPROM 整体数据有效，
	 * 真正让 UI 切页面，还需要读出业务信息区里的型号等字段。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_READ_INFO)
	{
		/*
		 * 与认证阶段相同，读取信息区前也先清空调试缓存。
		 * 这样如果 0x0020 信息区访问失败，HSDBG 报文就会准确落在这一步。
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
		 * 如果该字段为 0，说明信息区内容对当前业务无效，仍按失败处理。
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
		 * 因此正式把 A 通道置为在线，并只通知一次 UI 切换到对应界面。
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
	 * VERIFY_FAIL 表示本轮插入已经失败，等待重新插拔；
	 * ONLINE 表示本轮插入已经成功，等待后续拔出去抖。
	 * 这两个状态都不需要每个扫描周期重复动作，因此直接返回。
	 */
	if ((s_a_stage == HANDLESCAN_A_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_A_STAGE_ONLINE))
	{
		return;
	}
}

/* B通道当前不参与调度，保留空实现以兼容旧接口 */
void HandlescanB_Fun_SSC(void)
'@
$handleText = [regex]::Replace($handleText, $patternHandle, $replacementHandle)
Set-Content -Path $handlePath -Value $handleText -Encoding Default
