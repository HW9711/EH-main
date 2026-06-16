from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
EXT_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"


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
    depth = 0
    start = match.end() - 1
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
    pub = read_text(PUB_C)
    pub_h = read_text(PUB_H)
    uidp = read_text(UIDP_C)
    ext = read_text(EXT_C)

    pump_type_picture = normalize(function_body(uidp, "UIDP_PumpTypePicture"))
    require("caseDRAWWATER:returnenable_flag?215U:214U;" in pump_type_picture,
            "DRAWWATER title must use EX8 suction pictures 214 disabled / 215 enabled.",
            errors)
    require("caseINJECTWATER:returnenable_flag?211U:210U;" in pump_type_picture,
            "INJECTWATER title must use EX8 injection pictures 210 disabled / 211 enabled.",
            errors)
    require("casePOURWATER:returnenable_flag?213U:212U;" in pump_type_picture,
            "POURWATER title must use EX8 perfusion pictures 212 disabled / 213 enabled.",
            errors)

    pump_button_picture = normalize(function_body(uidp, "UIDP_PumpButtonPicture"))
    require("caseDRAWWATER:caseINJECTWATER:returnenable_flag?(run_flag?202U:201U):200U;" in pump_button_picture,
            "DRAWWATER/INJECTWATER buttons must use EX8 start pictures 200 disabled / 201 stopped / 202 running.",
            errors)
    require("casePOURWATER:returnenable_flag?(run_flag?205U:204U):203U;" in pump_button_picture,
            "POURWATER button must use EX8 drain pictures 203 disabled / 204 stopped / 205 running.",
            errors)
    require("default:return200U;" in pump_button_picture,
            "unknown pump buttons must fall back to disabled start picture 200.",
            errors)
    uidp_normalized = normalize(uidp)
    require("{249U,249U,249U,249U,249U}" in uidp_normalized and
            "{250U,251U,252U,253U,254U}" in uidp_normalized,
            "B pump gradient table must keep 249 as gear-0 and 250-254 as gear-1 fade frames.",
            errors)
    require("{349U,349U,349U,349U,349U}" in uidp_normalized and
            "{350U,351U,352U,353U,354U}" in uidp_normalized,
            "A pump gradient table must keep 349 as gear-0 and 350-354 as gear-1 fade frames.",
            errors)
    require("#defineUIDP_PUMP_GEAR_RUN_FRAME0U" in uidp_normalized,
            "pump display must use frame 0 as the normal running frame for each gear.",
            errors)

    a_button = normalize(function_body(uidp, "UIPUMPABUTTONDP"))
    b_button = normalize(function_body(uidp, "UIPUMPBBUTTONDP"))
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON,UIDP_PumpButtonPicture(button_type,true,run_flag))" in a_button and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON,UIDP_PumpButtonPicture(button_type,false,false))" in a_button,
            "A pump button must use EX8 VP 0x1427 macro and shared 200-205 resources.",
            errors)
    require("LCD_Show_Picture(0x1606,486)" not in a_button and
            "LCD_Show_Picture(0x1606,485)" not in a_button,
            "A pump button must not keep old 48x resources after new-screen export.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON,UIDP_PumpButtonPicture(button_type,true,run_flag))" in b_button and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON,UIDP_PumpButtonPicture(button_type,false,false))" in b_button,
            "B pump button must use EX8 VP 0x1428 macro and shared 200-205 resources.",
            errors)
    require("LCD_Show_Picture(0x1420,386)" not in b_button and
            "LCD_Show_Picture(0x1420,385)" not in b_button,
            "B pump button must not keep old 0x1420/38x resources after new-screen export.",
            errors)

    a_pump = function_body(uidp, "UIPUMPADP")
    b_pump = function_body(uidp, "UIPUMPBDP")
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON,UIDP_PumpButtonPicture(pump_type,true,false))" in normalize(a_pump),
            "A pump region refresh must draw the new-screen A stopped button as a fallback before the separate button-state message.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON,UIDP_PumpButtonPicture(pump_type,true,false))" in normalize(b_pump),
            "B pump region refresh must draw the new-screen B stopped button as a fallback before the separate button-state message.",
            errors)
    a_draw = case_body(a_pump, "case DRAWWATER:", ("case POURWATER:",))
    b_draw = case_body(b_pump, "case DRAWWATER:", ("case POURWATER:",))
    require("break;" in a_draw,
            "A pump DRAWWATER display must not fall through into POURWATER gear display.",
            errors)
    require("break;" in b_draw,
            "B pump DRAWWATER display must not fall through into POURWATER gear display.",
            errors)
    b_pour = case_body(b_pump, "case POURWATER:", ("case INJECTWATER:",))
    b_inject = case_body_to_end(b_pump, "case INJECTWATER:")
    require("UIDP_LCD_VP_PUMP_B_TYPE" in b_pour and "0x1601" not in b_pour and "0x1605" not in b_pour,
            "B pump POURWATER display must use new B-side coordinates, not old A-side coordinates.",
            errors)
    require("UIDP_LCD_VP_PUMP_B_TYPE" in b_inject and "UIDP_LCD_VP_PUMP_B_BUTTON" in b_inject and "0x1601" not in b_inject and "0x1605" not in b_inject,
            "B pump INJECTWATER display must use new B-side coordinates, not old A-side coordinates.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(DRAWWATER,true))" in normalize(a_draw) and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(POURWATER,true))" in normalize(case_body(a_pump, "case POURWATER:", ("case INJECTWATER:",))) and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE,UIDP_PumpTypePicture(INJECTWATER,true))" in normalize(case_body_to_end(a_pump, "case INJECTWATER:")),
            "A pump title must use EX8 VP 0x1417 macro and type resources 210-215.",
            errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(DRAWWATER,true))" in normalize(b_draw) and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(POURWATER,true))" in normalize(b_pour) and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE,UIDP_PumpTypePicture(INJECTWATER,true))" in normalize(b_inject),
            "B pump title must use EX8 VP 0x1418 macro and type resources 210-215.",
            errors)

    require("void Pubinterface_RefreshPumpADisplay(void);" in pub_h and
            "void Pubinterface_RefreshPumpBDisplay(void);" in pub_h,
            "pump display refresh helpers must be public so external_comm_task can refresh screen state.",
            errors)
    require("void Pubinterface_RefreshPumpADisplay(void)" in pub and
            "void Pubinterface_RefreshPumpBDisplay(void)" in pub and
            "static void Pubinterface_RefreshPumpADisplay(void)" not in pub and
            "static void Pubinterface_RefreshPumpBDisplay(void)" not in pub,
            "pump display refresh helper definitions must not be static.",
            errors)
    require("uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message);" in pub_h,
            "external pump start paths need a shared type-based nonzero start speed helper.",
            errors)

    setting = function_body(ext, "ExternalComm_ApplySetting")
    set_a_speed = case_body(setting, "case 0x03U:", ("case 0x04U:",))
    set_b_speed = case_body(setting, "case 0x04U:", ("default:",))
    require("Pubinterface_RefreshPumpADisplay();" in set_a_speed,
            "host A pump speed setting must refresh A pump value/button UI.",
            errors)
    require("Pubinterface_RefreshPumpBDisplay();" in set_b_speed,
            "host B pump speed setting must refresh B pump value/button UI.",
            errors)

    uart5_run = function_body(ext, "ExternalComm_RefreshUart5PumpRunState")
    require("Pubinterface_RefreshPumpADisplay();" in uart5_run,
            "host A pump run-state recompute must refresh A pump UI.",
            errors)

    control = function_body(ext, "ExternalComm_ApplyControlCommand")
    b_start = case_body(control, "case 0x03U:", ("case 0x04U:",))
    b_stop = case_body(control, "case 0x04U:", ("case 0x05U:",))
    require("Pubinterface_GetPumpStartSpeed(&pumpMessageB)" in b_start,
            "host B pump start must seed a nonzero type-based speed when speed_work is 0.",
            errors)
    require("Pubinterface_RefreshPumpBDisplay();" in b_start,
            "host B pump start must refresh B pump UI.",
            errors)
    require("Pubinterface_RefreshPumpBDisplay();" in b_stop,
            "host B pump stop must refresh B pump UI.",
            errors)

    pump_active = function_body(pub, "PUMPActive")
    a_control = case_body(pump_active, "case JTKey_left_long:", ("case JTKey_right_short:",))
    b_control = case_body(pump_active, "case JTKey_right_long:", ("case HMIkey_APUMP_Add:",))
    a_adjust = case_body(pump_active, "case HMIkey_APUMP_Add:", ("case HMIkey_BPUMP_Add:",))
    b_adjust = case_body(pump_active, "case HMIkey_BPUMP_Add:", ("case JTKey_Gently_left_start:",))
    gently_left_start = case_body(pump_active, "case JTKey_Gently_left_start:", ("case JTKey_Gently_left_stop:",))
    gently_left_stop = case_body(pump_active, "case JTKey_Gently_left_stop:", ("case JTKey_Gently_right_start:",))
    gently_right_start = case_body(pump_active, "case JTKey_Gently_right_start:", ("case JTKey_Gently_rigth_stop:",))
    gently_right_stop = case_body(pump_active, "case JTKey_Gently_rigth_stop:", ("case HMIkey_Gently_start:",))
    hmi_gently_start = case_body(pump_active, "case HMIkey_Gently_start:", ("case HMIkey_Gently_stop:",))
    hmi_gently_stop = case_body(pump_active, "case HMIkey_Gently_stop:", ("default:",))
    require("Pubinterface_GetPumpStartSpeed(&pumpMessageA)" in a_control,
            "screen/HMI A pump start must seed nonzero speed for valid pump types.",
            errors)
    require("Pubinterface_GetPumpStartSpeed(&pumpMessageB)" in b_control,
            "screen/HMI B pump start must seed nonzero speed for valid pump types.",
            errors)
    require("Pubinterface_GetPumpSpeedMax(&pumpMessageA)" in a_adjust and "pumpMessageA.speed_Max" not in a_adjust,
            "A pump +/- clamp must use type-based max when speed_Max is not configured.",
            errors)
    require("Pubinterface_GetPumpSpeedMax(&pumpMessageB)" in b_adjust and "pumpMessageB.speed_Max" not in b_adjust,
            "B pump +/- clamp must use type-based max when speed_Max is not configured.",
            errors)
    for label, body in (
        ("left foot gentle start", gently_left_start),
        ("right foot gentle start", gently_right_start),
        ("HMI gentle start", hmi_gently_start),
    ):
        require("Pubinterface_GetPumpStartSpeed(&pumpMessageA)" in body and
                "Pubinterface_GetPumpStartSpeed(&pumpMessageB)" in body,
                f"{label} must seed nonzero speed for whichever injection pump is selected.",
                errors)
        require("Pubinterface_RefreshPumpADisplay();" in body and
                "Pubinterface_RefreshPumpBDisplay();" in body,
                f"{label} must refresh both pump displays after changing injection pump state.",
                errors)
    for label, body in (
        ("left foot gentle stop", gently_left_stop),
        ("right foot gentle stop", gently_right_stop),
        ("HMI gentle stop", hmi_gently_stop),
    ):
        require("Pubinterface_RefreshPumpADisplay();" in body and
                "Pubinterface_RefreshPumpBDisplay();" in body,
                f"{label} must refresh both pump displays after changing injection pump state.",
                errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - pump screen UI consistency checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
