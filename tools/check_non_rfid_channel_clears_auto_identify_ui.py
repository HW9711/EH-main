from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释在检查脚本里误判。"""
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"无法识别文件编码：{path}")


def function_body(text: str, name: str) -> str:
    """提取指定 C 函数体，用于把检查范围限定在本次修复的刷新路径内。"""
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"找不到函数：{name}")
    depth = 0
    for index in range(match.end() - 1, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[match.end():index]
    raise AssertionError(f"函数括号不完整：{name}")


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看到普通手柄残留显示的完整风险。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """检查普通手柄通道不会继承 RFID 通道的自动识别图标和规格窗口。"""
    pub = read_text(ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c")
    body = function_body(pub, "Pubinterface_RefreshToolDisplay")
    selected_body = function_body(pub, "Pubinterface_RefreshSelectedChannelDisplay")
    compact = re.sub(r"\s+", "", body)
    selected_compact = re.sub(r"\s+", "", selected_body)
    errors: list[str] = []

    require(
        "split_tool_spec_handle=Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model);" in compact and
        "rfid_display_enabled=(auto_identify&&split_tool_spec_handle);" in compact,
        "必须用当前通道 hand_model 二次确认 RFID 显示能力，不能只看 WorkMessage.auto_identify。",
        errors,
    )
    require(
        "manual_display_value[1]=rfid_display_enabled?1U:0U;" in compact,
        "自动识别图标 51 只能由 rfid_display_enabled 驱动，普通手柄不能继承自动识别图标。",
        errors,
    )
    require(
        "if(split_tool_spec_handle==false)" in compact and
        "SendUIDSMessage(UI_MANUALBUTTON_ID,false,manual_display_value);" in compact,
        "普通 EEPROM 手柄必须主动隐藏 0x1404/0x1405/0x1406/0x1407 整块识别区。",
        errors,
    )
    require(
        "Pubinterface_RefreshToolDisplay(channel,planer_selected);" in selected_compact and
        "if(WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES)" not in selected_compact,
        "切换到普通手柄通道时也必须调用刀具区刷新函数，否则 A 通道 RFID 画面会残留。",
        errors,
    )
    require(
        "if((rfid_display_enabled==true)&&(result_tool_type==0U))" in compact,
        "RFID 掉线历史刀具类型只能给 RFID 手柄使用，普通手柄不能复用上个通道的历史图标。",
        errors,
    )
    require(
        "manual_display_value[2]=rfid_display_enabled?Pubinterface_MapToolTypeToResultPicture(result_tool_type):63U;" in compact,
        "0x1404 识别结果图只能在 RFID 手柄自动识别区域启用，普通手柄必须刷默认值。",
        errors,
    )
    require(
        "if(rfid_display_enabled)" in compact and "Pubinterface_GetToolSpecForChannel(channel,display_value)" in compact,
        "刀具规格窗口只能在当前 RFID 手柄自动识别显示允许时读取并打开。",
        errors,
    )
    require(
        "if(rfid_display_enabled)" in compact and "SendUIDSMessage(UI_MANUALBUTTON_ID,true,manual_display_value)" in compact,
        "等待自动识别时的按钮/图标刷新必须受当前通道 RFID 能力约束。",
        errors,
    )
    require(
        "SendUIDSMessage(UI_ORAL_ID,(rfid_display_enabled&&(show_tool_spec==false))?false:open_position_enabled,display_value);" in compact,
        "开口定位隐藏逻辑只能跟随当前通道 RFID 显示能力，避免普通手柄被残留自动识别状态影响。",
        errors,
    )
    require(
        "manual_display_value[1]=auto_identify?1U:0U;" not in compact,
        "不能再直接用 auto_identify 驱动自动识别图标，否则通道切换会残留。",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: 普通手柄通道会清掉 RFID 自动识别区域残留")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
