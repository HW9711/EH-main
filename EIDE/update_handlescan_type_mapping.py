from pathlib import Path


TARGET = Path(r"D:\Study_HL\F413EXOsSSCH_RTOSV1.5\User\Application\Handle\handlescan.c")


INSERT_BLOCK = """extern UART_HandleTypeDef huart10;

/*
 * A 通道 EEPROM 型号映射配置表。
 * 说明：
 * 1. first_byte / second_byte 对应 EEPROM 第二页前两个字节；
 * 2. mapped_handle_type 是系统内部使用的 Handle_Type_xx；
 * 3. handle_name 用于串口调试打印，方便现场直接看到手柄名称；
 * 4. 后续如果增加新手柄类型，只需要在这个表里继续追加一项即可。
 */
typedef struct
{
\tuint8_t first_byte;
\tuint8_t second_byte;
\tuint8_t mapped_handle_type;
\tconst char *handle_name;
} HandlescanHandleTypeConfig;

static const HandlescanHandleTypeConfig s_handle_type_config_table[] =
{
\t{0x6B, 0x01, Handle_Type_23, "PXBB"},
\t{0x6B, 0x02, Handle_Type_2,  "JMB"},
\t{0x6B, 0x03, Handle_Type_22, "PXBA"},
\t{0x6B, 0x04, Handle_Type_5,  "TMBA"},
\t{0x6B, 0x05, Handle_Type_4,  "TMBC"},
\t{0x6B, 0x06, Handle_Type_3,  "TMBB"},
};

/*
 * 根据 EEPROM 第二页前两个字节查找手柄映射配置。
 * 返回值：
 * 1. 找到匹配项时，返回对应的配置指针；
 * 2. 没找到时返回 NULL，调用方按“未知型号”处理。
 */
static const HandlescanHandleTypeConfig *Handlescan_FindHandleTypeConfig(uint8_t first_byte, uint8_t second_byte)
{
\tuint32_t index;

\tfor (index = 0U; index < (uint32_t)(sizeof(s_handle_type_config_table) / sizeof(s_handle_type_config_table[0])); ++index)
\t{
\t\tif ((s_handle_type_config_table[index].first_byte == first_byte) &&
\t\t    (s_handle_type_config_table[index].second_byte == second_byte))
\t\t{
\t\t\treturn &s_handle_type_config_table[index];
\t\t}
\t}

\treturn NULL;
}

/*
 * 打印手柄名称调试报文。
 * 这里额外输出一条带名称的文本报文，便于现场调试时直接看到 PXBB / JMB / TMBA 等名称，
 * 不需要再人工对照 Handle_Type 数值表。
 */
static void Handlescan_DebugTraceHandleName(uint8_t channel, uint8_t first_byte, uint8_t second_byte, const char *handle_name)
{
\tchar tx_buf[80];
\tint text_len;

\tif (handle_name == NULL)
\t{
\t\treturn;
\t}

\ttext_len = snprintf(tx_buf,
\t                    sizeof(tx_buf),
\t                    "HSNAME,CH=%02u,ID0=%02X,ID1=%02X,TYPE=%s\\r\\n",
\t                    (unsigned int)channel,
\t                    (unsigned int)first_byte,
\t                    (unsigned int)second_byte,
\t                    handle_name);
\tif (text_len <= 0)
\t{
\t\treturn;
\t}

\tif ((size_t)text_len > (sizeof(tx_buf) - 1U))
\t{
\t\ttext_len = (int)(sizeof(tx_buf) - 1U);
\t}

\tHAL_UART_Transmit(&huart10, (uint8_t *)tx_buf, (uint16_t)text_len, 1000U);
}

/* A通道扫描：PD0短接高电平 + I2C2 EEPROM认证 */
"""


OLD_VARS = """\tuint8_t read_status;
\tuint8_t model;
"""

NEW_VARS = """\tuint8_t read_status;
\tuint8_t raw_type_major;
\tuint8_t raw_type_minor;
\tuint8_t mapped_model;
\tconst HandlescanHandleTypeConfig *handle_type_cfg;
"""


OLD_BLOCK = """\t\t/*
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
"""


NEW_BLOCK = """\t\t/*
\t\t * 当前方案把 EEPROM 第二页前两个字节作为“手柄种类编码”：
\t\t * 1. 第 1 个字节是固定头，例如当前已知为 0x6B；
\t\t * 2. 第 2 个字节是具体子类型，例如 0x01/0x02/0x03 ...；
\t\t * 3. 通过配置表把这两个字节映射成系统内部的 Handle_Type_xx。
\t\t */
\t\traw_type_major = s_a_info_buf[HANDLESCAN_MODEL_OFFSET];
\t\traw_type_minor = s_a_info_buf[HANDLESCAN_MODEL_OFFSET + 1U];
\t\thandle_type_cfg = Handlescan_FindHandleTypeConfig(raw_type_major, raw_type_minor);
\t\tif (handle_type_cfg == NULL)
\t\t{
\t\t\tWorkvalue_s.Alarm_value = HANDLESCAN_ALARM_A_DATA_FAIL;
\t\t\tWorkvalue_s.beep_Alarm_flag = 1U;
\t\t\tif (Workvalue_s.Alarm_value != s_a_last_alarm)
\t\t\t{
\t\t\t\ts_a_last_alarm = Workvalue_s.Alarm_value;
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_MODEL_INVALID, raw_type_major);
\t\t\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ALARM_SET, Workvalue_s.Alarm_value);
\t\t\t}
\t\t\ts_a_stage = HANDLESCAN_A_STAGE_VERIFY_FAIL;
\t\t\treturn;
\t\t}

\t\tmapped_model = handle_type_cfg->mapped_handle_type;

\t\t/*
\t\t * 到这里说明：
\t\t * 1. 插入去抖成功；
\t\t * 2. CRC 认证成功；
\t\t * 3. 信息区读取成功；
\t\t * 4. 第二页前两个字节已经成功映射成系统手柄类型。
\t\t * 因此正式把 A 通道置为在线，并只通知一次 UI 切换界面。
\t\t */
\t\ts_a_online_latched = 1U;
\t\ts_a_last_alarm = 0U;
\t\tChannelValue_s.A.hand_model = mapped_model;
\t\tWorkvalue_s.Achanell_online_flag = start_flag;
\t\tWorkvalue_s.A_ChipRecognition_FLAG = start_flag;
\t\tWorkvalue_s.A_ShortCircuitRecognition_FLAG = start_flag;
\t\tWorkvalue_s.ScreenKey_data = 24U;
\t\ts_a_stage = HANDLESCAN_A_STAGE_ONLINE;

\t\t/*
\t\t * 上线时同时输出两类调试信息：
\t\t * 1. 原有 HS 报文继续保留，但 VAL 改为系统内部 Handle_Type_xx；
\t\t * 2. 额外补一条带名称的 HSNAME 报文，现场可以直接看到 PXBB / JMB / PXBA / TMBA / TMBC / TMBB。
\t\t */
\t\tHandlescan_DebugTrace(1U, HANDLESCAN_DBG_STEP_ONLINE, mapped_model);
\t\tHandlescan_DebugTraceHandleName(1U, raw_type_major, raw_type_minor, handle_type_cfg->handle_name);
\t\treturn;
\t}
"""


def main() -> None:
    text = TARGET.read_text(encoding="utf-8-sig")

    if "Handlescan_FindHandleTypeConfig" not in text:
        text = text.replace("extern UART_HandleTypeDef huart10;\n\n/* A通道扫描：PD0短接高电平 + I2C2 EEPROM认证 */\n", INSERT_BLOCK, 1)

    text = text.replace(OLD_VARS, NEW_VARS, 1)
    text = text.replace(OLD_BLOCK, NEW_BLOCK, 1)

    TARGET.write_text(text, encoding="utf-8-sig")


if __name__ == "__main__":
    main()
