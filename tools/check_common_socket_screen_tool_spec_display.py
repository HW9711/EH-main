#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查公共接头 EPC 刀具信息必须能进入屏幕规格显示区。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
PUBINTERFACE_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释影响静态检查。"""
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"无法识别文件编码：{path}")


def function_body(text: str, name: str) -> str:
    """提取指定 C 函数体，避免检查脚本误匹配其它路径。"""
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
    """收集失败项，便于一次看到公共接头屏幕显示链路缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """检查公共接头不会被 PXBA/PXBB 专用 UI gate 误隐藏规格。"""
    pub = read_text(PUBINTERFACE_C)
    refresh = re.sub(r"\s+", "", function_body(pub, "Pubinterface_RefreshToolDisplay"))
    split_helper = re.sub(r"\s+", "", function_body(pub, "Pubinterface_IsSplitToolSpecDisplayModel"))
    errors: list[str] = []

    require(
        "Pubinterface_IsRfidToolSpecDisplayModel" in pub,
        "需要单独区分“可显示 RFID 规格”的手柄，公共接头不能复用 PXBA/PXBB 分体按钮 gate。",
        errors,
    )
    require(
        "COMMON_SOCKET_ONLINES" not in split_helper,
        "PXBA/PXBB 分体按钮 gate 不能包含公共接头，否则会误显示自动识别/手动识别按钮。",
        errors,
    )
    require(
        "rfid_spec_handle=Pubinterface_IsRfidToolSpecDisplayModel(WorkMessage.hand_model);" in refresh,
        "刷新刀具区时必须用包含公共接头的 rfid_spec_handle 判断规格显示能力。",
        errors,
    )
    require(
        "common_socket_spec_handle=(WorkMessage.hand_model==COMMON_SOCKET_ONLINES);" in refresh,
        "刷新刀具区时必须单独识别公共接头，公共接头不依赖 PXBA/PXBB 自动识别按钮状态。",
        errors,
    )
    require(
        "rfid_spec_window_enabled=(rfid_display_enabled||common_socket_spec_handle);" in refresh,
        "规格窗口允许条件必须包含公共接头 EPC，PXBA/PXBB 仍按自动识别状态显示。",
        errors,
    )
    require(
        "if(rfid_spec_handle==false)" in refresh,
        "早退隐藏分支只能拦截真正没有 RFID 规格窗口的普通手柄，不能拦截公共接头。",
        errors,
    )
    require(
        "if(rfid_spec_window_enabled)" in refresh and "Pubinterface_GetToolSpecForChannel(channel,display_value)" in refresh,
        "公共接头 EPC 规格有效时必须允许读取 paoxueSpeciValue_A/B 并打开 UI_TOOLSPEC_ID。",
        errors,
    )
    require(
        "manual_display_value[1]=rfid_display_enabled?1U:0U;" in refresh,
        "自动识别图标仍只能由 PXBA/PXBB 的 rfid_display_enabled 驱动，公共接头不显示按钮区。",
        errors,
    )
    require(
        "elseif(common_socket_spec_handle)" in refresh and
        "SendUIDSMessage(UI_MANUALBUTTON_ID,false,manual_display_value);" in refresh,
        "公共接头未读到有效 EPC 规格时必须关闭识别按钮区，不能显示普通手柄按钮。",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: 公共接头 EPC 刀具信息允许进入屏幕规格显示区")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
