from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"
KEYBH_C = ROOT / "User" / "Application" / "Beep" / "sscKEYBH.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"cannot decode {path}")


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def function_body(text: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*(?://[^\n]*\n\s*)?\{{", text)
    if not match:
        raise AssertionError(f"missing function: {name}")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated function: {name}")


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []
    screen_addr = read_text(SCREEN_ADDRESS_H)
    pub_h = read_text(PUB_H)
    pub_c = read_text(PUB_C)
    uidp = read_text(UIDP_C)
    screenkey = read_text(SCREENKEY_C)
    handlescan = read_text(HANDLESCAN_C)
    rfid = read_text(RFID_C)
    keybh = read_text(KEYBH_C)

    screen_addr_c = compact(screen_addr)
    pub_h_c = compact(pub_h)
    pub_c_c = compact(pub_c)
    uidp_c = compact(uidp)
    screenkey_c = compact(screenkey)
    handlescan_c = compact(handlescan)
    rfid_c = compact(rfid)
    keybh_c = compact(keybh)

    expected_vps = {
        "UIDP_LCD_VP_PUMP_A_PLUS": "0x1421U",
        "UIDP_LCD_VP_PUMP_B_PLUS": "0x1422U",
        "UIDP_LCD_VP_PUMP_A_MINUS": "0x1423U",
        "UIDP_LCD_VP_PUMP_B_MINUS": "0x1424U",
        "UIDP_LCD_VP_PUMP_A_UNIT": "0x1425U",
        "UIDP_LCD_VP_PUMP_B_UNIT": "0x1426U",
        "UIDP_LCD_VP_PUMP_A_BUTTON": "0x1427U",
        "UIDP_LCD_VP_PUMP_B_BUTTON": "0x1428U",
        "UIDP_LCD_VP_ALARM_TIP": "0x1429U",
        "UIDP_LCD_VP_TOUCH_WORK": "0x1430U",
    }
    for name, value in expected_vps.items():
        require(f"#define{name}{value}" in screen_addr_c, f"{name} must be {value}", errors)
    require("0x1606U" not in screen_addr_c or "UIDP_LCD_LEGACY_VP_OSC_ANGLE_CACHE0x1606U" in screen_addr_c,
            "main-run pump button VP must not use old 0x1606", errors)

    uidir = compact(function_body(uidp, "UIDIRDP"))
    require("UIDP_LCD_VP_DIR_REVERSE,25U" in uidir and
            "UIDP_LCD_VP_DIR_REVERSE,23U" in uidir and
            "UIDP_LCD_VP_DIR_REVERSE,24U" in uidir,
            "reverse icon mapping must use 23/24/25", errors)
    require("UIDP_LCD_VP_DIR_OSC,28U" in uidir and
            "UIDP_LCD_VP_DIR_OSC,26U" in uidir and
            "UIDP_LCD_VP_DIR_OSC,27U" in uidir,
            "osc icon mapping must use 26/27/28", errors)

    ui_handle = compact(function_body(uidp, "UIHANDLEDP"))
    require("caseCOMMON_SOCKET_ONLINES:" in ui_handle and
            "UIDP_LCD_VP_HANDLE_A,112" in ui_handle and
            "UIDP_LCD_VP_HANDLE_A,111" in ui_handle and
            "UIDP_LCD_VP_HANDLE_B,142" in ui_handle and
            "UIDP_LCD_VP_HANDLE_B,141" in ui_handle,
            "common socket handle icons must use A 111/112 and B 141/142", errors)

    pump_a = compact(function_body(uidp, "UIPUMPADP"))
    pump_b = compact(function_body(uidp, "UIPUMPBDP"))
    for body, label in ((pump_a, "A"), (pump_b, "B")):
        require(",222U)" in body and ",223U)" in body and ",224U)" in body and ",225U)" in body,
                f"{label} pump +/- icons must use shared 222/223/224/225 resources", errors)
        require("498U" not in body and "499U" not in body and "500U" not in body and "501U" not in body,
                f"{label} pump must not use old 498-501 button resources", errors)

    require("case0x07:" in screenkey_c and "case0x02:ScreenKey_PostLegacyAction(42U);" in screenkey_c,
            "0x2407/key2 must map to touch exit", errors)
    require("case0x55:" in screenkey_c and "case0x20:" in screenkey_c and
            "ScreenKey_TouchKeepAlive" in screenkey,
            "0x5520 must map to touch keepalive, not only old 0x5510/0x5530", errors)
    require("s_touch_keepalive_ticks" in screenkey and "ScreenKey_ServiceTouchKeepAlive" in screenkey,
            "screenkey task must stop touch run when keepalive times out", errors)

    require("#defineSCREENKey_TouchKeepAlive" in pub_h_c, "missing SCREENKey_TouchKeepAlive", errors)
    require("caseSCREENKey_TouchKeepAlive:" in keybh_c, "key dispatcher must handle SCREENKey_TouchKeepAlive", errors)
    require("caseSCREENKey_TouchKeepAlive:" in compact(function_body(pub_c, "ControlTypeActive")),
            "ControlTypeActive must handle touch keepalive without hiding touch UI", errors)

    require("#defineEMBC_ONLINES" in pub_h_c, "missing EMBC_ONLINES enum", errors)
    require("{0x6B,0x0F,EMBC_ONLINES," in handlescan_c, "handlescan must recognize EMBC Page2 code", errors)
    require("caseEMBC_ONLINES:" in compact(function_body(pub_c, "Pubinterface_MapHandleModelToUiType")),
            "EMBC must map to generic drill UI type", errors)

    osc_support = compact(function_body(pub_c, "Pubinterface_IsOscDirectionSupportedModel"))
    require("hand_model==PX_YIM_ONLINES" in osc_support and "hand_model==PX_YIP_ONLINES" in osc_support,
            "PXM/PXP one-piece planer handles must support OSC direction", errors)
    toolpos = compact(function_body(pub_c, "ToolPosActive"))
    require("(WorkMessage.tool_type!=PLANER)" in toolpos or "WorkMessage.tool_type==PLANER" in toolpos,
            "ToolPosActive must gate by PLANER tool_type", errors)
    require("tool_reduction_ratio" not in toolpos and "500U" not in toolpos,
            "ToolPosActive must not keep low16 reduction-ratio 500 gate", errors)
    require("ToolPosMay(WorkMessage.channel_work,true,1U)" in toolpos and
            "ToolPosMay(WorkMessage.channel_work,false,1U)" in toolpos,
            "ToolPosActive must pass true/false directions", errors)

    freq_active = compact(function_body(pub_c, "FreqActive"))
    require("freq_step=5" in freq_active and "FreqMin5U" in pub_h_c,
            "frequency step/range must be 5Hz and 5-40Hz", errors)
    require("SendUIDSMessage(UI_FREQ_ID,true" in freq_active,
            "FreqActive must refresh UI_FREQ_ID immediately", errors)

    require("auto_identify" in pub_h and "WorkMessage.auto_identify" in pub_c,
            "WorkMessage/MemoryMsg must keep auto-identify mode", errors)
    require("HANDLESCAN_TOOL_SOURCE_RFID_USER" in handlescan and
            "COMMON_SOCKET_ONLINES" in handlescan and "Pubinterface_IsCommonSocketToolReady" in pub_c,
            "RFID source rules and common-socket run gate must exist", errors)
    require("Rfid_ClearChannelResult(channel);" not in compact(function_body(handlescan, "Handlescan_ClearOnlineRfidTool")) or
            "tool_source==HANDLESCAN_TOOL_SOURCE_RFID_EPC" in compact(function_body(handlescan, "Handlescan_ClearOnlineRfidTool")),
            "PXBA/PXBB RFID USER offline must preserve last tool parameters", errors)
    require("RFID_DEBUG_BEEP_EVERY_UART_RESPONSE0U" in rfid_c and
            "RFID_DEBUG_BEEP_EVERY_VALID_READ0U" in rfid_c,
            "RFID debug beeps must stay off; beeps should be state-change based", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - EX8 confirmed screen rules passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
