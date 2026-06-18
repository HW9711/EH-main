#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查公共接头 EPC 规格在屏幕上按原始十六进制整数显示。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
PUBINTERFACE_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
SSCUIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
LCD_C = ROOT / "User" / "Peripheral" / "lcd" / "lcd.c"
LCD_H = ROOT / "User" / "Peripheral" / "include" / "lcd.h"


def read_text(path: Path) -> str:
    """按工程源码编码读取文件，避免中文注释影响检查结果。"""
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"无法识别文件编码：{path}")


def function_body(text: str, name: str) -> str:
    """提取 C 函数体，避免同名片段或注释造成误匹配。"""
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
    """收集所有失败项，方便一次定位显示链路缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """公共接头 EPC 的屏幕规格必须由标签原始字节直接解析。"""
    handlescan = read_text(HANDLESCAN_C)
    pubinterface = read_text(PUBINTERFACE_C)
    sscuidp = read_text(SSCUIDP_C)
    lcd_c = read_text(LCD_C)
    lcd_h = read_text(LCD_H)

    apply_rfid = re.sub(r"\s+", "", function_body(handlescan, "Handlescan_ApplyRfidToolResult"))
    pack_spec = re.sub(r"\s+", "", function_body(pubinterface, "Pubinterface_GetToolSpecForChannel"))
    ui_spec = re.sub(r"\s+", "", function_body(sscuidp, "UITOOLSPECDP"))
    lcd_raw = re.sub(r"\s+", "", lcd_c)
    errors: list[str] = []

    require(
        "if(rfid_result->source==RFID_READ_SOURCE_EPC)" in apply_rfid,
        "EPC 结果写入规格缓存时必须有独立分支，不能继续复用 PXBA/PXBB 的 x10 显示缓存。",
        errors,
    )
    require(
        "spec_values[0]=(uint32_t)length;" in apply_rfid and
        "spec_values[1]=(uint32_t)diameter;" in apply_rfid and
        "spec_values[2]=(uint32_t)angle;" in apply_rfid,
        "公共接头 EPC 长度、直径、角度必须按原始字节整数写入缓存，例如 0x10 写成 16。",
        errors,
    )
    require(
        "raw_spec_display=(WorkMessage.hand_model==COMMON_SOCKET_ONLINES);" in pack_spec,
        "Pubinterface 打包规格时必须识别公共接头原始值显示模式。",
        errors,
    )
    require(
        "if(raw_spec_display)" in pack_spec and
        "tool_length=spec_values[0];" in pack_spec and
        "tool_diameter=spec_values[1];" in pack_spec and
        "tool_angle=spec_values[2];" in pack_spec,
        "公共接头上屏前不能再把 EPC 长度乘 5、直径或角度按 x10 处理。",
        errors,
    )
    require(
        "display_value[4]=raw_spec_display?1U:0U;" in pack_spec,
        "UIDP Value[4] 必须携带原始整数显示标志，供 LCD 选择直径格式。",
        errors,
    )
    require(
        "LCD_IntegratedCutterRawData_Update" in ui_spec and "raw_display_flag" in ui_spec,
        "UITOOLSPECDP 必须在公共接头原始模式下调用原始整数显示接口。",
        errors,
    )
    require(
        "UITOOLSPECDP(msg.enable_flag,msg.Value[0]<<8|msg.Value[1],msg.Value[2],msg.Value[3],msg.Value[4])" in
        re.sub(r"\s+", "", sscuidp),
        "UI_TOOLSPEC_ID 处理必须把 Value[4] 原始显示标志传给 UITOOLSPECDP。",
        errors,
    )
    require(
        "voidLCD_IntegratedCutterRawData_Update(uint16_tAddr,uint16_tLength,uint8_tDiameter,uint8_tAngle)" in
        re.sub(r"\s+", "", lcd_h),
        "lcd.h 必须声明公共接头 EPC 原始整数规格显示接口。",
        errors,
    )
    require(
        "voidLCD_IntegratedCutterRawData_Update(uint16_tAddr,uint16_tLength,uint8_tDiameter,uint8_tAngle)" in lcd_raw and
        "s_lcd_integrated_cutter_raw_integer_mode=1U;" in lcd_raw,
        "lcd.c 必须提供原始整数显示接口，并在本次下发前打开原始直径格式。",
        errors,
    )
    require(
        "raw_integer_mode!=0U" in lcd_raw and
        "dat[15]=DiameterTemp[0]+0x30;" in lcd_raw and
        "dat[16]=0x20;" in lcd_raw,
        "原始模式下直径要显示为整数两位加空格，例如 0x10 显示 Φ16，而不是 Φ1.6。",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: 公共接头 EPC 规格按原始十六进制整数显示")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
