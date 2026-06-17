#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 0x7C/01..06 一体式型号按手柄型号识别。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"


EXPECTED_ENTRIES = [
    "{0x7C, 0x01, MX_YIM_ONLINES,   \"MXYTM\"}",
    "{0x7C, 0x02, MX_YIP_ONLINES,   \"MXYTP\"}",
    "{0x7C, 0x03, PX_YIM_ONLINES,   \"PXYTM\"}",
    "{0x7C, 0x04, PX_YIP_ONLINES,   \"PXYTP\"}",
    "{0x7C, 0x05, JMB_ONLINES,      \"JMB\"}",
    "{0x7C, 0x06, MX_YIM16_ONLINES, \"MXYTM16\"}",
]

EXPECTED_SYMBOLS = [
    "MX_YIM_ONLINES",
    "MX_YIP_ONLINES",
    "PX_YIM_ONLINES",
    "PX_YIP_ONLINES",
    "JMB_ONLINES",
    "MX_YIM16_ONLINES",
]


def read_text(path: Path) -> str:
    """按工程源码编码读取文件，保证中文注释不会影响检查。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"无法识别文件编码: {path}")


def compact(text: str) -> str:
    """去掉空白后做结构检查，避免格式微调造成误报。"""
    return re.sub(r"\s+", "", text)


def array_body(text: str, name: str) -> str:
    """提取指定静态配置表内容。"""
    match = re.search(rf"\b{name}\[\]\s*=\s*\{{", text)
    if not match:
        raise AssertionError(f"找不到配置表: {name}")
    start = match.end()
    end = text.find("};", start)
    if end < 0:
        raise AssertionError(f"配置表未闭合: {name}")
    return text[start:end]


def function_body(text: str, name: str) -> str:
    """提取指定 C 函数体。"""
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"找不到函数: {name}")
    depth = 0
    for index in range(match.end() - 1, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[match.end():index]
    raise AssertionError(f"函数括号未闭合: {name}")


def optional_function_body(text: str, name: str, errors: list[str]) -> str:
    """函数不存在时返回空串，并把缺失原因加入失败列表。"""
    try:
        return function_body(text, name)
    except AssertionError as exc:
        errors.append(str(exc))
        return ""


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看完整改动缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """确认 0x7C/01..06 被归入手柄型号，并跳过 Page3 刀具表流程。"""
    text = read_text(HANDLESCAN_C)
    hand_table = compact(array_body(text, "s_hand_type_config_table"))
    tool_table = compact(array_body(text, "s_tool_type_config_table"))
    all_text = compact(text)
    a_scan = compact(function_body(text, "HandlescanA_Fun_SSC"))
    b_scan = compact(function_body(text, "HandlescanB_Fun_SSC"))
    errors: list[str] = []

    pxbb_pos = hand_table.find("{0x6B,0x06,PXBB_ONLINES,\"PXBB\"}")
    lgz_pos = hand_table.find("{0x6B,0x07,LGZ_I_ONLINES,\"LGZ_I\"}")
    require(pxbb_pos >= 0 and lgz_pos >= 0 and pxbb_pos < lgz_pos,
            "手柄表必须保持 PXBB 后接后续手柄型号的顺序。", errors)

    last_pos = pxbb_pos
    for entry in EXPECTED_ENTRIES:
        compact_entry = compact(entry)
        pos = hand_table.find(compact_entry)
        require(pos > last_pos and pos < lgz_pos,
                f"{entry} 必须插入手柄表 PXBB 和 LGZ_I 之间。", errors)
        last_pos = pos
        require(compact_entry not in tool_table,
                f"{entry} 不能继续保留在 EEPROM Page3 刀具表。", errors)

    require("staticboolHandlescan_IsSelfTypedHandleModel(uint8_tmapped_model)" in all_text,
            "handlescan 必须有自带刀具能力手柄判断函数。", errors)
    self_typed_body = compact(optional_function_body(text, "Handlescan_IsSelfTypedHandleModel", errors))
    for symbol in EXPECTED_SYMBOLS:
        require(symbol in self_typed_body,
                f"自带刀具能力手柄判断必须包含 {symbol}。", errors)

    require("staticvoidHandlescan_UpdateSelfTypedHandleRecognizeMessage" in all_text,
            "handlescan 必须把这类手柄型号同步写入 handle_type/tool_type/raw_tool_type。", errors)
    helper_body = compact(optional_function_body(text, "Handlescan_UpdateSelfTypedHandleRecognizeMessage", errors))
    require("Handlescan_UpdateRecognizeMessage(message,mapped_model,raw_type_major,raw_type_minor,mapped_model,0U,0U,0U)" in helper_body,
            "自带刀具能力手柄上线时必须用手柄型号本身派生 tool_type/raw_tool_type。", errors)

    for label, body, read_call, channel_msg, plug_key in [
        ("A", a_scan, "AT24CS32_ReadBytes_I2C2(HANDLESCAN_TOOL_INFO_ADDR", "ChannelrecognizeMessageA", "SCREENKey_PLUG_A"),
        ("B", b_scan, "AT24CS32_ReadBytes_I2C3(HANDLESCAN_TOOL_INFO_ADDR", "ChannelrecognizeMessageB", "SCREENKey_PLUG_B"),
    ]:
        branch = body.find("Handlescan_IsSelfTypedHandleModel(mapped_model)!=false")
        read_pos = body.find(read_call)
        require(branch >= 0 and read_pos >= 0 and branch < read_pos,
                f"{label} 通道必须在读取 Page3 刀具页前处理 0x7C 手柄型号。", errors)
        branch_tail = body[branch:read_pos]
        require(f"Handlescan_UpdateSelfTypedHandleRecognizeMessage(&{channel_msg}" in branch_tail,
                f"{label} 通道自带刀具能力分支必须写入识别缓存。", errors)
        require("Handlescan_UpdateInitialInfoMessage" in branch_tail and
                "Handlescan_LoadPage6SpeedStep" in branch_tail,
                f"{label} 通道自带刀具能力分支仍必须读取 Page4/Page6 运行参数。", errors)
        require(f"SendKeyBehMessage(PLUGunPLUG,{plug_key})" in branch_tail,
                f"{label} 通道自带刀具能力分支必须发布插入事件。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: 0x7C/01..06 已按手柄型号识别并跳过 Page3 刀具表")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
