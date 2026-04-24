$root = 'D:\Study_HL\F413EXOsSSCH_RTOSV1.5'
$at24Path = Join-Path $root 'User\Peripheral\EEPROM\at24cs32.c'
$handlePath = Join-Path $root 'User\Application\Handle\handlescan.c'

$at24Text = Get-Content -Path $at24Path -Raw -Encoding Default

$at24Text = $at24Text -replace '#define AT24CS32_READ_CHUNK_SIZE       64U', "#define AT24CS32_READ_CHUNK_SIZE       64U`r`n/* EEPROM 读前设备就绪检查的轮询次数 */`r`n#define AT24CS32_READY_TRIALS          3U`r`n/* 发生 BUSY/TIMEOUT 后，I2C 软恢复后的等待时间 */`r`n#define AT24CS32_RECOVER_DELAY_MS      2U"

$pattern1 = '(?s)/\* 检查 I2C 句柄是否有效 \*/.*?/\* 计算页起始字节地址 \*/\s*static uint16_t AT24CS32_PageToAddr\(uint16_t page_index\)\s*\{\s*return \(uint16_t\)\(page_index \* AT24CS32_PAGE_SIZE\);\s*\}'
$replacement1 = @'
/* 检查 I2C 句柄是否有效 */
static uint8_t AT24CS32_IsI2cReady(I2C_HandleTypeDef *hi2c)
{
    return (hi2c != NULL) ? 1U : 0U;
}

/*
 * 判断当前访问的是否为 I2C2。
 * 之所以单独区分 I2C2，是因为你当前手柄认证链路固定走 I2C2，
 * 现场问题也集中在 I2C2 的 BUSY/TIMEOUT 上，所以恢复动作优先覆盖 I2C2。
 */
static uint8_t AT24CS32_IsI2C2Handle(I2C_HandleTypeDef *hi2c)
{
    return (hi2c == &hi2c2) ? 1U : 0U;
}

/*
 * 在真正访问 EEPROM 之前，先主动探测器件是否 ready。
 * 这样做的目的有两个：
 * 1. 如果器件刚上电、总线还没完全空闲，可以尽早在这里发现；
 * 2. 如果 HAL_I2C_Mem_Read() 失败，调试信息里也能区分“是读命令失败”还是“读前就未 ready”。
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
 * 判断当前失败是否属于“值得做一次 I2C 软恢复再重试”的场景。
 * 这里重点拦截 HAL_BUSY / HAL_TIMEOUT 以及 HAL_I2C_ERROR_TIMEOUT，
 * 因为你现场报文已经证明当前主要卡点就是 BUSY Flag 超时未释放。
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
 * 对 I2C 外设做一次最小侵入的软恢复。
 * 这里不做 GPIO 手动时钟脉冲恢复，而是先采用更稳妥的 HAL DeInit/Init，
 * 目的是在不明显扩大改动面的前提下，优先处理“外设状态机卡 BUSY”的问题。
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
$at24Text = [regex]::Replace($at24Text, $pattern1, $replacement1)

$pattern2 = '(?s)static uint8_t AT24CS32_MemRead\(I2C_HandleTypeDef \*hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t \*buf, uint16_t len\)\s*\{.*?\n\}'
$replacement2 = @'
static uint8_t AT24CS32_MemRead(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    /*
     * 每次发起 EEPROM 读取前，先记录“调用前的 I2C 状态”。
     * 如果后面现场再次出现 BUSY/TIMEOUT，就能直接从调试报文里看出：
     * 是访问过程中卡住，还是发起访问之前总线就已经不干净了。
     */
    s_at24cs32_last_debug.op_type = AT24CS32_DEBUG_OP_READ;
    s_at24cs32_last_debug.dev_addr = dev_addr;
    s_at24cs32_last_debug.mem_addr = mem_addr;
    s_at24cs32_last_debug.data_len = len;
    s_at24cs32_last_debug.hal_status = 0xFFU;
    s_at24cs32_last_debug.hal_error = 0U;
    s_at24cs32_last_debug.i2c_state_before = (hi2c != NULL) ? (uint32_t)HAL_I2C_GetState(hi2c) : 0U;
    s_at24cs32_last_debug.i2c_state_after = 0U;

    /*
     * 第一步先做设备 ready 探测，而不是直接冲进 Mem_Read。
     * 这样如果器件尚未 ready，或者总线在这里就已经 Busy，
     * 我们可以先尝试做一次恢复，避免把真正的读操作浪费掉。
     */
    hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
    if (hal_status != HAL_OK) {
        if (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U) {
            AT24CS32_RecoverI2cBus(hi2c);
            hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
        }

        AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status);
        if (hal_status != HAL_OK) {
            return 0U;
        }
    }

    /*
     * 设备 ready 后再执行真正的 EEPROM 读。
     * 如果这里仍然遇到 BUSY/TIMEOUT，再做一次软恢复和单次重试。
     * 这样既能提升现场容错，又不会因为无限重试把问题掩盖掉。
     */
    hal_status = HAL_I2C_Mem_Read(hi2c,
                                  dev_addr,
                                  mem_addr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  buf,
                                  len,
                                  AT24CS32_I2C_TIMEOUT_MS);

    if ((hal_status != HAL_OK) && (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U)) {
        AT24CS32_RecoverI2cBus(hi2c);

        if (AT24CS32_WaitDeviceReady(hi2c, dev_addr) == HAL_OK) {
            hal_status = HAL_I2C_Mem_Read(hi2c,
                                          dev_addr,
                                          mem_addr,
                                          I2C_MEMADD_SIZE_16BIT,
                                          buf,
                                          len,
                                          AT24CS32_I2C_TIMEOUT_MS);
        } else {
            hal_status = HAL_BUSY;
        }
    }

    /* 无论最终成功还是失败，都把本次访问的真实结果完整回填到调试结构里。 */
    AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status);

    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}
'@
$at24Text = [regex]::Replace($at24Text, $pattern2, $replacement2)

$pattern3 = '(?s)static uint8_t AT24CS32_MemWrite\(I2C_HandleTypeDef \*hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t \*buf, uint16_t len\)\s*\{.*?\n\}'
$replacement3 = @'
static uint8_t AT24CS32_MemWrite(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    /*
     * 写操作沿用与读操作一致的调试记录策略。
     * 这样后续如果现场转为排查“写入失败”问题，也能直接复用同一套报文分析思路。
     */
    s_at24cs32_last_debug.op_type = AT24CS32_DEBUG_OP_WRITE;
    s_at24cs32_last_debug.dev_addr = dev_addr;
    s_at24cs32_last_debug.mem_addr = mem_addr;
    s_at24cs32_last_debug.data_len = len;
    s_at24cs32_last_debug.hal_status = 0xFFU;
    s_at24cs32_last_debug.hal_error = 0U;
    s_at24cs32_last_debug.i2c_state_before = (hi2c != NULL) ? (uint32_t)HAL_I2C_GetState(hi2c) : 0U;
    s_at24cs32_last_debug.i2c_state_after = 0U;

    /*
     * 先确认器件可应答，再做真正的写入。
     * 对 EEPROM 来说，这一步也能避免“上一轮写周期尚未结束时又立即写入”带来的误判。
     */
    hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
    if (hal_status != HAL_OK) {
        if (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U) {
            AT24CS32_RecoverI2cBus(hi2c);
            hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
        }

        AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);
        if (hal_status != HAL_OK) {
            return 0U;
        }
    }

    hal_status = HAL_I2C_Mem_Write(hi2c,
                                   dev_addr,
                                   mem_addr,
                                   I2C_MEMADD_SIZE_16BIT,
                                   buf,
                                   len,
                                   AT24CS32_I2C_TIMEOUT_MS);

    if ((hal_status != HAL_OK) && (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U)) {
        AT24CS32_RecoverI2cBus(hi2c);

        if (AT24CS32_WaitDeviceReady(hi2c, dev_addr) == HAL_OK) {
            hal_status = HAL_I2C_Mem_Write(hi2c,
                                           dev_addr,
                                           mem_addr,
                                           I2C_MEMADD_SIZE_16BIT,
                                           buf,
                                           len,
                                           AT24CS32_I2C_TIMEOUT_MS);
        } else {
            hal_status = HAL_BUSY;
        }
    }

    /* 统一回填本次写操作的底层状态，便于现场直接对照 HSDBG 分析。 */
    AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);

    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}
'@
$at24Text = [regex]::Replace($at24Text, $pattern3, $replacement3)

Set-Content -Path $at24Path -Value $at24Text -Encoding Default

$handleText = Get-Content -Path $handlePath -Raw -Encoding Default
$patternHandle = '(?s)/\* A通道扫描：PD0短接高电平 \+ I2C2 EEPROM认证 \*/.*?/\* B通道当前不参与调度，保留空实现以兼容旧接口 \*/'
$replacementHandle = @'
/* A通道扫描：PD0短接高电平 + I2C2 EEPROM认证 */
void HandlescanA_Fun_SSC(void)
{
	uint8_t is_inserted;
	AT24CS32_CRC_Result verify_result;
	AT24CS32_CRC_Status verify_status;
	uint8_t read_status;
	uint8_t model;

	/*
	 * 第一步先读取 A 通道短接检测脚。
	 * 这里的设计目标很明确：
	 * 1. PD0=1 认为“手柄插入候选”；
	 * 2. PD0=0 认为“手柄拔出候选”；
	 * 3. 具体是否真正插入/拔出，还要交给后续去抖状态机判断。
	 */
	is_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_SET);

	/*
	 * 一旦检测到当前为未插入状态，就优先处理“拔出”这条链路。
	 * 这部分逻辑要覆盖两类场景：
	 * 1. 手柄原本在线，现在发生正常拔出；
	 * 2. 手柄正在拔出去抖过程中，继续维持低电平直到确认拔出稳定。
	 */
	if (!is_inserted)
	{
		/* 插入链路相关的计数在拔出分支里全部清零，避免旧状态串到下一轮插入流程。 */
		s_handleA_debounce.in_debounce_ticks = 0U;
		s_a_verify_start_wait_ticks = 0U;

		/*
		 * 只有“当前已在线”或“已经进入拔出去抖阶段”时，才需要继续走拔出去抖。
		 * 如果本来就处于空闲态，则直接保持 IDLE 即可，不需要反复通知 UI。
		 */
		if ((s_a_stage == HANDLESCAN_A_STAGE_ONLINE) || (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_OUT))
		{
			/*
			 * 第一次从 ONLINE 检测到低电平时，先切换到拔出去抖阶段，
			 * 并把去抖计数清零，确保后面是“连续稳定低电平”才会判定拔出成功。
			 */
			if (s_a_stage != HANDLESCAN_A_STAGE_DEBOUNCE_OUT)
			{
				s_handleA_debounce.out_debounce_ticks = 0U;
				s_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_OUT;
			}

			/* 拔出去抖未达到阈值时直接返回，不做离线处理，防止接触抖动造成误判。 */
			if (++s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
			{
				return;
			}

			/* 连续低电平达到阈值，确认本次为稳定拔出。 */
			Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OUT_DEBOUNCE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);

			/*
			 * 如果设备正在运行中，运行态插拔需要沿用历史保护逻辑：
			 * 触发报警 13、关闭运行标志，并暂停后续离线收尾，等待上层处理。
			 */
			if (Handlescan_HandleRunningPlugAlarm())
			{
				return;
			}

			/*
			 * 走到这里说明已经确认“非运行态稳定拔出”，
			 * 需要把 A 通道在线信息、识别标志和缓存型号全部清掉，
			 * 同时通知 UI 切回“手柄已拔出”的界面。
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
			 * 非在线态下检测到低电平，只需要维持空闲态即可。
			 * 这里顺手清掉拔出去抖计数，保证下次真正上线后重新计数。
			 */
			s_handleA_debounce.out_debounce_ticks = 0U;
			s_a_stage = HANDLESCAN_A_STAGE_IDLE;
		}

		return;
	}

	/*
	 * 走到这里说明短接脚当前为高电平，至少满足“插入候选”条件。
	 * 因为当前关注的是插入链路，所以先清掉拔出去抖计数，
	 * 并同步置位“A 通道短接识别成功”标志，告诉系统短接判断已经成立。
	 */
	s_handleA_debounce.out_debounce_ticks = 0U;
	Workvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;

	/*
	 * IDLE -> DEBOUNCE_IN：
	 * 第一次从空闲态检测到插入时，不立刻认证，而是先进入插入去抖阶段。
	 * 这样可以避免插头接触弹跳、上电瞬态等情况直接触发 EEPROM 访问。
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
	 * 只有连续检测到高电平达到阈值，才允许进入后面的认证启动等待阶段。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_IN)
	{
		if (++s_handleA_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
		{
			return;
		}

		/* 去抖通过后把计数钉在阈值，同时切到“认证前等待”阶段。 */
		s_handleA_debounce.in_debounce_ticks = HANDLESCAN_INSERT_DEBOUNCE_TICKS;
		s_a_verify_start_wait_ticks = 0U;
		s_a_stage = HANDLESCAN_A_STAGE_WAIT_VERIFY;
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_IN_DEBOUNCE_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS);
		return;
	}

	/*
	 * 认证前等待阶段：
	 * 即使插入去抖已经通过，也额外留一小段稳定时间，
	 * 给手柄侧 EEPROM 上电、总线空闲和连接器接触建立留出余量。
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
	 * 这里严格只做一次 CRC 认证，不会在手柄持续插入时每 10ms 反复认证。
	 * 这正是当前扫描任务与旧逻辑最大的区别。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_VERIFY)
	{
		/* 运行中插拔优先走保护逻辑，避免在设备运行态继续访问手柄 EEPROM。 */
		if (Handlescan_HandleRunningPlugAlarm())
		{
			return;
		}

		/*
		 * 发起本轮认证前先清空最近一次底层调试信息。
		 * 这样如果认证失败，后面打印出来的 HSDBG 一定对应这次认证中的最后一次 EEPROM 访问。
		 */
		AT24CS32_ClearLastDebugInfo();
		verify_status = AT24CS32_VerifyCrc_I2C2(&verify_result);
		Handlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

		/*
		 * 认证失败后只记录报警并进入 VERIFY_FAIL 保持态，
		 * 不做高频自动重试，等待用户重新插拔后再重新走完整流程。
		 */
		if (verify_status != AT24CS32_CRC_STATUS_OK)
		{
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

		/* CRC 认证通过后，进入业务信息区读取阶段。 */
		s_a_stage = HANDLESCAN_A_STAGE_READ_INFO;
	}

	/*
	 * 信息区读取阶段：
	 * 认证通过只能说明 EEPROM 数据整体有效，
	 * 真正驱动 UI 切换还需要把手柄型号等业务字段读出来并落到系统状态中。
	 */
	if (s_a_stage == HANDLESCAN_A_STAGE_READ_INFO)
	{
		/*
		 * 与认证阶段一样，读取业务信息区前先清空调试缓存。
		 * 如果这里失败，HSDBG 报文就会准确指向 0x0020 起始的信息区访问。
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
		 * 当前约定信息区第 0 字节为型号字段。
		 * 如果型号字段为 0，说明信息区内容对当前业务无效，同样按失败处理。
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
		 * 认证成功且信息读取成功后，正式把 A 通道置为在线：
		 * 1. 保存手柄型号；
		 * 2. 置位在线/芯片识别/短接识别标志；
		 * 3. 通知 UI 切到 A 手柄在线界面；
		 * 4. 进入 ONLINE 保持态，后续在未拔出前不再重复认证。
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
	 * 1. VERIFY_FAIL：本轮插入已经失败，等待重新插拔；
	 * 2. ONLINE：本轮插入已经成功，等待后续拔出。
	 * 这两个状态都不需要在每次扫描周期里重复做额外动作，因此直接返回。
	 */
	if ((s_a_stage == HANDLESCAN_A_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_A_STAGE_ONLINE))
	{
		return;
	}
}

/* B通道当前不参与调度，保留空实现以兼容旧接口 */
'@
$handleText = [regex]::Replace($handleText, $patternHandle, $replacementHandle)
Set-Content -Path $handlePath -Value $handleText -Encoding Default
