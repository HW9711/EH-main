from pathlib import Path
import re

path = Path(r"D:\Study_HL\F413EXOsSSCH_RTOSV1.5\User\Application\Handle\handlescan.c")
text = path.read_text(encoding="utf-8")

replacement = """
/* A通道扫描：PD0短接高电平 + I2C2 EEPROM认证 */
void HandlescanA_Fun_SSC(void)
{
\tuint8_t is_inserted;
\tAT24CS32_CRC_Result verify_result;
\tAT24CS32_CRC_Status verify_status;
\tuint8_t read_status;
\tuint8_t model;

\t/*
\t * 先读取 A 通道的短接检测脚。
\t * PD0 为高电平时认为“手柄插入候选”，PD0 为低电平时认为“手柄拔出候选”。
\t * 最终是否真正成立，还需要经过下面的去抖状态机确认。
\t */
\tis_inserted = (uint8_t)(HAL_GPIO_ReadPin(HANDLESCAN_A_SHORT_GPIO, HANDLESCAN_A_SHORT_PIN) == GPIO_PIN_SET);

\t/*
\t * 当前检测到低电平时，优先走“拔出处理链路”。
\t * 这里既处理“原本已经在线后正常拔出”，也处理“已经进入拔出去抖阶段并继续保持低电平”。
\t */
\tif (!is_inserted)
\t{
\t\t/* 进入拔出分支后，插入相关计数全部清零，避免旧状态残留到下一次插入流程。 */
\t\ts_handleA_debounce.in_debounce_ticks = 0U;
\t\ts_a_verify_start_wait_ticks = 0U;

\t\t/*
\t\t * 只有“当前已在线”或“已经在做拔出去抖”时，才继续推进拔出确认。
\t\t * 如果当前本来就是空闲态，则保持 IDLE，不重复通知 UI。
\t\t */
\t\tif ((s_a_stage == HANDLESCAN_A_STAGE_ONLINE) || (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_OUT))
\t\t{
\t\t\t/*
\t\t\t * 第一次从 ONLINE 检测到低电平时，先切到拔出去抖阶段。
\t\t\t * 去抖计数必须从 0 开始，确保只有“连续稳定低电平”才算真正拔出。
\t\t\t */
\t\t\tif (s_a_stage != HANDLESCAN_A_STAGE_DEBOUNCE_OUT)
\t\t\t{
\t\t\t\ts_handleA_debounce.out_debounce_ticks = 0U;
\t\t\t\ts_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_OUT;
\t\t\t}

\t\t\t/* 拔出去抖未达到阈值时直接返回，避免接触抖动造成误离线。 */
\t\t\tif (++s_handleA_debounce.out_debounce_ticks < HANDLESCAN_REMOVE_DEBOUNCE_TICKS)
\t\t\t{
\t\t\t\treturn;
\t\t\t}

\t\t\t/* 连续低电平达到阈值，确认本次拔出已经稳定成立。 */
\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OUT_DEBOUNCE_PASS, HANDLESCAN_REMOVE_DEBOUNCE_TICKS);

\t\t\t/*
\t\t\t * 如果设备正在运行，则优先沿用“运行中插拔报警”逻辑。
\t\t\t * 此时先报警并停止运行，不继续做普通离线收尾。
\t\t\t */
\t\t\tif (Handlescan_HandleRunningPlugAlarm())
\t\t\t{
\t\t\t\treturn;
\t\t\t}

\t\t\t/*
\t\t\t * 正常离线收尾：
\t\t\t * 1. 清掉 A 通道在线状态与识别标志；
\t\t\t * 2. 清掉缓存型号；
\t\t\t * 3. 通知 UI 显示“手柄已拔出”。
\t\t\t */
\t\t\ts_handleA_debounce.out_debounce_ticks = 0U;
\t\t\ts_a_online_latched = 0U;
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_IDLE;
\t\t\ts_a_last_alarm = 0U;
\t\t\tWorkvalue_s.Achanell_online_flag = stop_flag;
\t\t\tWorkvalue_s.A_ChipRecognition_FLAG = stop_flag;
\t\t\tWorkvalue_s.A_ShortCircuitRecognition_FLAG = stop_flag;
\t\t\tChannelValue_s.A.hand_model = 0U;
\t\t\tWorkvalue_s.ScreenKey_data = 26U;
\t\t\tK1_OFF();
\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_OFFLINE, 0U);
\t\t}
\t\telse
\t\t{
\t\t\t/*
\t\t\t * 当前并未处于在线态，只需要保持空闲态即可。
\t\t\t * 同时把拔出去抖计数清零，保证下次重新插入时从头计数。
\t\t\t */
\t\t\ts_handleA_debounce.out_debounce_ticks = 0U;
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_IDLE;
\t\t}

\t\treturn;
\t}

\t/*
\t * 走到这里说明短接脚当前为高电平，也就是“插入候选”成立。
\t * 因为现在准备走插入链路，所以先清掉拔出去抖计数，
\t * 并同步置位 A 通道短接识别成功标志。
\t */
\ts_handleA_debounce.out_debounce_ticks = 0U;
\tWorkvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;

\t/*
\t * IDLE -> DEBOUNCE_IN：
\t * 第一次从空闲态检测到插入时，不直接访问 EEPROM，
\t * 而是先进入插入去抖阶段，滤掉连接器接触抖动和上电瞬态。
\t */
\tif (s_a_stage == HANDLESCAN_A_STAGE_IDLE)
\t{
\t\ts_handleA_debounce.in_debounce_ticks = 0U;
\t\ts_a_verify_start_wait_ticks = 0U;
\t\ts_a_last_alarm = 0U;
\t\ts_a_stage = HANDLESCAN_A_STAGE_DEBOUNCE_IN;
\t}

\t/*
\t * 插入去抖阶段：
\t * 只有连续高电平达到阈值，才允许进入认证前等待阶段。
\t */
\tif (s_a_stage == HANDLESCAN_A_STAGE_DEBOUNCE_IN)
\t{
\t\tif (++s_handleA_debounce.in_debounce_ticks < HANDLESCAN_INSERT_DEBOUNCE_TICKS)
\t\t{
\t\t\treturn;
\t\t}

\t\ts_handleA_debounce.in_debounce_ticks = HANDLESCAN_INSERT_DEBOUNCE_TICKS;
\t\ts_a_verify_start_wait_ticks = 0U;
\t\ts_a_stage = HANDLESCAN_A_STAGE_WAIT_VERIFY;
\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_IN_DEBOUNCE_PASS, HANDLESCAN_INSERT_DEBOUNCE_TICKS);
\t\treturn;
\t}

\t/*
\t * 认证前等待阶段：
\t * 插入去抖通过后，再额外等待一小段时间，
\t * 给手柄侧 EEPROM 上电稳定、I2C 总线空闲和连接器接触建立留出余量。
\t */
\tif (s_a_stage == HANDLESCAN_A_STAGE_WAIT_VERIFY)
\t{
\t\tif (++s_a_verify_start_wait_ticks < HANDLESCAN_VERIFY_START_DELAY_TICKS)
\t\t{
\t\t\treturn;
\t\t}

\t\ts_a_verify_start_wait_ticks = 0U;
\t\ts_a_stage = HANDLESCAN_A_STAGE_VERIFY;
\t}

\t/*
\t * 单次认证阶段：
\t * 这里只做一次 CRC 认证，不会在手柄持续插入时反复认证。
\t * 如果认证失败，就停在 VERIFY_FAIL，等待重新插拔后再触发下一轮。
\t */
\tif (s_a_stage == HANDLESCAN_A_STAGE_VERIFY)
\t{
\t\t/* 运行态插拔优先触发保护，不继续访问 EEPROM。 */
\t\tif (Handlescan_HandleRunningPlugAlarm())
\t\t{
\t\t\treturn;
\t\t}

\t\t/*
\t\t * 发起本轮认证前先清空最近一次 EEPROM 调试信息。
\t\t * 这样如果认证失败，HSDBG 报文就一定对应本轮认证中的最后一次底层访问。
\t\t */
\t\tAT24CS32_ClearLastDebugInfo();
\t\tverify_status = AT24CS32_VerifyCrc_I2C2(&verify_result);
\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_VERIFY_STATUS, (uint8_t)verify_status);

\t\tif (verify_status != AT24CS32_CRC_STATUS_OK)
\t\t{
\t\t\t/*
\t\t\t * 认证失败时只记录一次报警并进入失败保持态，
\t\t\t * 避免手柄一直插着时每个扫描周期都反复重试、反复刷报文。
\t\t\t */
\t\t\tWorkvalue_s.Alarm_value = Handlescan_MapVerifyStatusToAlarm(verify_status, 0U);
\t\t\tWorkvalue_s.beep_Alarm_flag = 1U;
\t\t\tif (Workvalue_s.Alarm_value != s_a_last_alarm)
\t\t\t{
\t\t\t\ts_a_last_alarm = Workvalue_s.Alarm_value;
\t\t\t\tHandlescan_DebugTraceI2cDetail(1U);
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
\t\t\t}
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
\t\t\treturn;
\t\t}

\t\t/* CRC 认证通过后，继续读取 EEPROM 的业务信息区。 */
\t\ts_a_stage = HANDLESCAN_A_STAGE_READ_INFO;
\t}

\t/*
\t * 信息区读取阶段：
\t * CRC 认证通过只说明 EEPROM 数据整体有效，
\t * 真正驱动 UI 切换还需要把型号等业务字段读出来。
\t */
\tif (s_a_stage == HANDLESCAN_A_STAGE_READ_INFO)
\t{
\t\t/*
\t\t * 读取信息区前同样先清空调试缓存。
\t\t * 如果这里失败，HSDBG 报文就会准确落在 0x0020 起始的信息区访问上。
\t\t */
\t\tAT24CS32_ClearLastDebugInfo();
\t\tread_status = (uint8_t)AT24CS32_ReadBytes_I2C2(HANDLESCAN_INFO_ADDR, s_a_info_buf, HANDLESCAN_INFO_SIZE);
\t\tif (read_status == 0U)
\t\t{
\t\t\tWorkvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
\t\t\tWorkvalue_s.beep_Alarm_flag = 1U;
\t\t\tif (Workvalue_s.Alarm_value != s_a_last_alarm)
\t\t\t{
\t\t\t\ts_a_last_alarm = Workvalue_s.Alarm_value;
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_INFO_READ_FAIL, HANDLESCAN_ALARM_A_DATA_FAIL);
\t\t\t\tHandlescan_DebugTraceI2cDetail(1U);
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
\t\t\t}
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
\t\t\treturn;
\t\t}

\t\t/*
\t\t * 当前约定信息区第 0 字节为手柄型号。
\t\t * 如果型号字段为 0，则认为信息区对当前业务无效，按失败处理。
\t\t */
\t\tmodel = s_a_info_buf[HANDLESCAN_MODEL_OFFSET];
\t\tif (model == 0U)
\t\t{
\t\t\tWorkvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
\t\t\tWorkvalue_s.beep_Alarm_flag = 1U;
\t\t\tif (Workvalue_s.Alarm_value != s_a_last_alarm)
\t\t\t{
\t\t\t\ts_a_last_alarm = Workvalue_s.Alarm_value;
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, model);
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
\t\t\t}
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
\t\t\treturn;
\t\t}

\t\t/*
\t\t * 到这里说明：
\t\t * 1. 插入去抖成功；
\t\t * 2. CRC 认证成功；
\t\t * 3. 信息区读取成功；
\t\t * 4. 型号字段有效。
\t\t * 因此正式把 A 通道置为在线，并只通知一次 UI 切换界面。
\t\t */
\t\ts_a_online_latched = 1U;
\t\ts_a_last_alarm = 0U;
\t\tChannelValue_s.A.hand_model = model;
\t\tWorkvalue_s.Achanell_online_flag = start_flag;
\t\tWorkvalue_s.A_ChipRecognition_FLAG = start_flag;
\t\tWorkvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;
\t\tWorkvalue_s.ScreenKey_data = 24U;
\t\ts_a_stage = HANDLESCAN_A_STAGE_ONLINE;
\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, model);
\t\treturn;
\t}

\t/*
\t * VERIFY_FAIL 和 ONLINE 都属于“保持态”：
\t * VERIFY_FAIL 表示本轮插入失败，等待重新插拔；
\t * ONLINE 表示本轮插入成功，等待后续拔出去抖。
\t * 这两个状态都不需要在每个扫描周期重复动作，因此直接返回。
\t */
\tif ((s_a_stage == HANDLESCAN_A_STAGE_VERIFY_FAIL) || (s_a_stage == HANDLESCAN_A_STAGE_ONLINE))
\t{
\t\treturn;
\t}
}

/* B通道当前不参与调度，保留空实现以兼容旧接口 */
void HandlescanB_Fun_SSC(void)
""".strip("\n")

pattern = re.compile(r"/\* .*?void HandlescanA_Fun_SSC\(void\)\s*\{.*?\n\}\s*\n\s*/\*.*?\*/\s*void HandlescanB_Fun_SSC\(void\)", re.S)
text, count = pattern.subn(replacement, text, count=1)
if count != 1:
    raise SystemExit("replace failed")

path.write_text(text, encoding="utf-8-sig")
