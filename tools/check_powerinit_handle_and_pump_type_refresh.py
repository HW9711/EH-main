from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SOFT_UART_C = ROOT / "User" / "Peripheral" / "uart" / "soft_uart.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def normalize(text: str) -> str:
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


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


def case_body(text: str, label: str, following_labels: tuple[str, ...]) -> str:
    start = text.find(label)
    if start < 0:
        raise AssertionError(f"missing case label: {label}")
    ends = [text.find(next_label, start + len(label)) for next_label in following_labels]
    ends = [end for end in ends if end >= 0]
    if not ends:
        raise AssertionError(f"missing end label after: {label}")
    return text[start : min(ends)]


def case_body_to_end(text: str, label: str) -> str:
    start = text.find(label)
    if start < 0:
        raise AssertionError(f"missing case label: {label}")
    return text[start:]


def main() -> int:
    errors: list[str] = []
    uidp = read_text(UIDP_C)
    soft_uart = read_text(SOFT_UART_C)

    uidp_norm = normalize(uidp)
    require("UIHANDLEDP(0,0,0,0)" not in uidp_norm,
            "power init must not call UIHANDLEDP with channel 0 because it draws neither A nor B offline handle slot.",
            errors)
    require("UIHANDLEDP(0,0,1,0)" in uidp_norm and "UIHANDLEDP(0,0,2,0)" in uidp_norm,
            "power init must draw both A and B offline handle slots before any handle has been inserted.",
            errors)

    update = function_body(soft_uart, "Cs1237_UpdatePumpMessage")
    update_norm = normalize(update)
    require("old_pump_type" in update and "old_online_flag" in update,
            "CS1237 pump update must remember old type/online state so screen refresh is only sent on identity changes.",
            errors)
    require("pump_display_changed" in update,
            "CS1237 pump update must compute whether pump type/online state changed.",
            errors)
    require("Pubinterface_RefreshPumpADisplay();" in update and
            "Pubinterface_RefreshPumpBDisplay();" in update,
            "CS1237 pump identity changes must refresh the matching A/B screen region immediately.",
            errors)
    require("channel==SIM_UART_2" in update_norm and "Pubinterface_RefreshPumpADisplay();" in update,
            "SIM_UART_2/PE6 must continue refreshing only A pump display; A remains the left pump.",
            errors)
    require("channel==SIM_UART_1" in update_norm and "Pubinterface_RefreshPumpBDisplay();" in update,
            "SIM_UART_1/PE4 must continue refreshing only B pump display; B remains the right pump.",
            errors)

    a_pump = function_body(uidp, "UIPUMPADP")
    b_pump = function_body(uidp, "UIPUMPBDP")
    a_draw = case_body(a_pump, "case DRAWWATER:", ("case POURWATER:",))
    a_pour = case_body(a_pump, "case POURWATER:", ("case INJECTWATER:",))
    a_inject = case_body_to_end(a_pump, "case INJECTWATER:")
    b_draw = case_body(b_pump, "case DRAWWATER:", ("case POURWATER:",))
    b_pour = case_body(b_pump, "case POURWATER:", ("case INJECTWATER:",))
    b_inject = case_body_to_end(b_pump, "case INJECTWATER:")
    a_draw_norm = normalize(a_draw)
    a_pour_norm = normalize(a_pour)
    a_inject_norm = normalize(a_inject)
    b_draw_norm = normalize(b_draw)
    b_pour_norm = normalize(b_pour)
    b_inject_norm = normalize(b_inject)
    require("LCD_Show_Picture(0x1601" not in normalize(a_pump),
            "left A pump title must not use old 0x1601 resources after new-screen export.",
            errors)
    require("LCD_Show_Picture(0x1415,483)" not in normalize(b_pump) and "LCD_Show_Picture(0x1415,383)" not in normalize(b_pump),
            "right B pump title must not use old 0x1415 resources after new-screen export.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(DRAWWATER,true))" in a_draw_norm and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(POURWATER,true))" in a_pour_norm and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(INJECTWATER,true))" in a_inject_norm,
            "left A pump title must use EX8 0x1417 type macro.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(DRAWWATER,true))" in b_draw_norm and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(POURWATER,true))" in b_pour_norm and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(INJECTWATER,true))" in b_inject_norm,
            "right B pump title must use EX8 0x1418 type macro.",
            errors)
    a_button = normalize(function_body(uidp, "UIPUMPABUTTONDP"))
    b_button = normalize(function_body(uidp, "UIPUMPBBUTTONDP"))
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON,UIDP_PumpButtonPicture(button_type,true,run_flag))" in a_button and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON,UIDP_PumpButtonPicture(button_type,false,false))" in a_button and
            "LCD_Show_Picture(0x1606,485)" not in a_button and
            "LCD_Show_Picture(0x1606,486)" not in a_button,
            "left A pump button must use EX8 0x1427 macro with 200-205 resources.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON,UIDP_PumpButtonPicture(button_type,true,run_flag))" in b_button and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON,UIDP_PumpButtonPicture(button_type,false,false))" in b_button and
            "LCD_Show_Picture(0x1420,385)" not in b_button and
            "LCD_Show_Picture(0x1420,386)" not in b_button,
            "right B pump button must use EX8 0x1428 macro with 200-205 resources.",
            errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - power-init handle and pump type refresh checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
