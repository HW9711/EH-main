from pathlib import Path

path = Path(r"D:\Study_HL\F413EXOsSSCH_RTOSV1.5\User\Application\Handle\handlescan.c")
text = path.read_text(encoding="utf-8-sig")


def find_banner_start(src: str, func_name: str) -> int:
    func_idx = src.index(f"void {func_name}(void)")
    banner_idx = src.rfind("//============================================================================", 0, func_idx)
    return banner_idx if banner_idx != -1 else func_idx


def find_func_end(src: str, func_name: str) -> int:
    start = src.index(f"void {func_name}(void)")
    brace_start = src.index("{", start)
    depth = 0
    for idx in range(brace_start, len(src)):
        ch = src[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return idx + 1
    raise RuntimeError(f"cannot find end of {func_name}")


def replace_block(src: str, func_name: str, new_block: str) -> str:
    start = find_banner_start(src, func_name)
    end = find_func_end(src, func_name)
    return src[:start] + new_block.rstrip() + "\n\n" + src[end:].lstrip("\n")


text = replace_block(
    text,
    "HandlescanA_Fun",
    """
//============================================================================
// 函数名称: HandlescanA_Fun()
// 功能描述: 旧版 A 通道型号脚扫描逻辑
// 说明: 通过 H_MD1/H_MD2/H_MD3 识别手柄型号，当前主要用于兼容旧方案
//============================================================================
void HandlescanA_Fun(void)
{
  static uint8_t MotorDtimes2 = 0, MotorOtimes2 = 0; // 电机接入识别时间
  uint8_t IDNum1 = 0;

  IDNum1 = ((H_MD2_STATUS() << 2) | (H_MD3_STATUS() << 1) | H_MD1_STATUS());

  if (IDNum1 >= 7)   // 0111，认为手柄离线
  {
      MotorDtimes2 = 0;

      // 约 100ms 去抖
      if (MotorOtimes2++ < 40)
        return;

      MotorOtimes2 = 0;

      SysInterface.HandleType[1] = 1;   // 2 号手柄类型默认值
      SysInterface.Interface[1] = 0;    // 2 号手柄在线标志：0 离线，1 在线

      SysRunData.ParamInitFlag[1] = 0;  // 手柄重新插拔后需要重新初始化
      SysRunData.DJOldInit[0] = 0;

      return;
  }

  MotorOtimes2 = 0;

  // 约 100ms 去抖
  if (MotorDtimes2++ < 40)
      return;

  MotorDtimes2 = 0;

  SysInterface.Interface[1] = 1;
  switch (IDNum1)
  {
      case 5 : SysInterface.HandleType[1] = Handle_Type_3;  break; // TMBB
      case 4 : SysInterface.HandleType[1] = Handle_Type_4;  break; // TMBC
      case 3 : SysInterface.HandleType[1] = Handle_Type_5;  break; // TMBA / EMBA / EMBB
      case 2 : SysInterface.HandleType[1] = Handle_Type_22; break; // PXBA（关节）
      case 1 : SysInterface.HandleType[1] = Handle_Type_2;  break; // JMB 无刷
      case 0 : SysInterface.HandleType[1] = Handle_Type_23; break; // PXBB（关节）
      case 6 :
      default : break;
  }
}
""",
)

text = replace_block(
    text,
    "HandlescanB_Fun",
    """
//============================================================================
// 函数名称: HandlescanB_Fun()
// 功能描述: 旧版 B 通道型号脚扫描逻辑
// 说明: 通过 M_D1/M_D2/M_D3 识别手柄型号，当前主要用于兼容旧方案
//============================================================================
void HandlescanB_Fun(void)
{
  static uint8_t MotorDtimes1 = 0, MotorOtimes1 = 0; // 电机接入识别时间
  uint8_t IDNum0 = 0;

  IDNum0 = ((M_D2_STATUS() << 2) | (M_D3_STATUS() << 1) | M_D1_STATUS());

  if (IDNum0 >= 7)   // 0111，认为手柄离线
  {
      MotorDtimes1 = 0;

      // 约 100ms 去抖
      if (MotorOtimes1++ < 40)
        return;

      MotorOtimes1 = 0;

      SysInterface.HandleType[4] = 1;   // 5 号手柄类型默认值
      SysInterface.Interface[4] = 0;    // 5 号手柄在线标志：0 离线，1 在线

      SysRunData.ParamInitFlag[4] = 0;  // 手柄重新插拔后需要重新初始化
      SysRunData.DJOldInit[1] = 0;

      return;
  }

  MotorOtimes1 = 0;

  // 约 100ms 去抖
  if (MotorDtimes1++ < 40)
      return;

  MotorDtimes1 = 0;

  SysInterface.Interface[4] = 1;
  switch (IDNum0)
  {
      case 5 : SysInterface.HandleType[4] = Handle_Type_3;  break; // TMBB
      case 4 : SysInterface.HandleType[4] = Handle_Type_4;  break; // TMBC
      case 3 : SysInterface.HandleType[4] = Handle_Type_5;  break; // TMBA / EMBA / EMBB
      case 2 : SysInterface.HandleType[4] = Handle_Type_22; break; // PXBA（关节）
      case 1 : SysInterface.HandleType[4] = Handle_Type_2;  break; // JMB 无刷
      case 0 : SysInterface.HandleType[4] = Handle_Type_23; break; // PXBB（关节）
      case 6 :
      default : break;
  }
}
""",
)

text = replace_block(
    text,
    "Handlescan0_Fun",
    """
//============================================================================
// 函数名称: Handlescan0_Fun()
// 功能描述: 旧版接口预留
// 说明: 历史调试代码已停用，当前保留空实现以兼容旧调用关系
//============================================================================
void Handlescan0_Fun(void)
{
    return;
}
""",
)

text = replace_block(
    text,
    "Handlescan1_Fun",
    """
//============================================================================
// 函数名称: Handlescan1_Fun()
// 功能描述: 旧版接口预留
// 说明: 历史调试代码已停用，当前保留空实现以兼容旧调用关系
//============================================================================
void Handlescan1_Fun(void)
{
    return;
}
""",
)

text = replace_block(
    text,
    "Handlescan2_Fun",
    """
//============================================================================
// 函数名称: Handlescan2_Fun()
// 功能描述: 旧版一体式 1-Wire 扫描入口
// 说明: 当前方案已迁移到 EEPROM 校验识别，这里保留空实现
//============================================================================
void Handlescan2_Fun(void)
{
    /* 旧版一体式 1-Wire 扫描入口，当前方案已迁移到 EEPROM 校验识别。 */
    return;
}
""",
)

text = replace_block(
    text,
    "Handlescan3_Fun",
    """
//============================================================================
// 函数名称: Handlescan3_Fun()
// 功能描述: 旧版一体式 1-Wire 扫描预留入口
// 说明: 当前方案已迁移到 EEPROM 校验识别，这里保留空实现
//============================================================================
void Handlescan3_Fun(void)
{
    /* 旧版一体式 1-Wire 扫描预留入口，当前方案已迁移到 EEPROM 校验识别。 */
    return;
}
""",
)

text = replace_block(
    text,
    "Handlescan_Fun",
    """
//============================================================================
// 函数名称: Handlescan_Fun()
// 功能描述: 手柄扫描总入口
// 说明: 当前默认只启用 A 通道 SSC 扫描逻辑，B 通道保留扩展位
//============================================================================
void Handlescan_Fun(void)
{
    HandlescanA_Fun_SSC();
    // HandlescanB_Fun_SSC();

    // AT24CS32_CRC_Status verify_status_a;
    // AT24CS32_CRC_Result verify_result_a;
    // verify_status_a = AT24CS32_VerifyCrc_I2C2(&verify_result_a);
    // HAL_UART_Transmit(&huart10, (uint8_t *)&verify_status_a, 1, 1000);
}
""",
)

# 清理 RTOS 任务函数周围的英文模板注释，改成简洁中文。
text = text.replace(
    "/* USER CODE BEGIN Header_HANDLESCANTaskFunc */\n/**\n* @brief Function implementing the HANDLESCANTask thread.\n* @param argument: Not used\n* @retval None\n*/\n/* USER CODE END Header_HANDLESCANTaskFunc */",
    "/* HANDLESCANTask 的任务执行入口 */",
)

text = text.replace(
    "/**\n * @brief Function implementing the Time thread.\n * @param argument: Not used\n * @retval None 10\n */\n//============================================================================",
    "//============================================================================\n// 函数名称: HandlescanTaskInit()\n// 功能描述: 创建并启动手柄扫描任务\n// 说明: 扫描周期为 10ms\n//============================================================================",
)

# 顺手把文件头注释改干净。
text = text.replace("//handlescan.c", "// handlescan.c", 1)

path.write_text(text, encoding="utf-8-sig")
