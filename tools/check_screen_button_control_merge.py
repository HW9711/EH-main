from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


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


def main() -> int:
    errors: list[str] = []
    pub = read_text(PUB_C)
    screenkey = read_text(SCREENKEY_C)

    require("void Pubinterface_RefreshControlModeDisplay(void)" in pub,
            "screen control-mode switching must have a dedicated UI refresh helper.",
            errors)
    control = function_body(pub, "ControlTypeActive")
    jt_case = case_body(control, "case SCREENKey_JTActi:", ("case SCREENKey_HandleActi:",))
    handle_case = case_body(control, "case SCREENKey_HandleActi:", ("case SCREENKey_TouchActi:",))
    require("Pubinterface_RefreshControlModeDisplay();" in jt_case,
            "foot-control screen button must refresh UI_CONTROL_ID immediately.",
            errors)
    require("Pubinterface_RefreshControlModeDisplay();" in handle_case,
            "handle-control screen button must refresh UI_CONTROL_ID immediately.",
            errors)

    direction = function_body(pub, "DirActive")
    osc_case = case_body(direction, "case HANDLEKey_dir_OSC:", ("break;",))
    require("Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model, WorkMessage.tool_type) == false" in osc_case and "return;" in osc_case,
            "OSC direction must be rejected when the selected handle/tool capability does not support reciprocation.",
            errors)
    require("Pubinterface_RefreshSelectedChannelDisplay(WorkMessage.channel_work);" in direction,
            "direction switching must refresh selected direction/speed UI after changing state.",
            errors)

    tool_pos = function_body(pub, "ToolPosActive")
    require("Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type) == false" in tool_pos and
            "tool_reduction_ratio" not in tool_pos and
            "500U" not in tool_pos,
            "open-position must be gated by PLANER/PXM/PXP tool capability and must not use the old low16 reduction-ratio 500 condition.",
            errors)

    require("void Pubinterface_RefreshPumpADisplay(void)" in pub and
            "void Pubinterface_RefreshPumpBDisplay(void)" in pub,
            "pump A/B screen actions must have UI refresh helpers.",
            errors)
    pump = function_body(pub, "PUMPActive")
    a_control = case_body(pump, "case JTKey_left_long:", ("case JTKey_right_short:",))
    b_control = case_body(pump, "case JTKey_right_long:", ("case HMIkey_APUMP_Add:",))
    a_adjust = case_body(pump, "case HMIkey_APUMP_Add:", ("case HMIkey_BPUMP_Add:",))
    b_adjust = case_body(pump, "case HMIkey_BPUMP_Add:", ("case JTKey_Gently_left_start:",))
    require("Pubinterface_RefreshPumpADisplay();" in a_control,
            "A pump start/stop must refresh pump area and button UI.",
            errors)
    require("Pubinterface_RefreshPumpBDisplay();" in b_control,
            "B pump start/stop must refresh pump area and button UI.",
            errors)
    require("Pubinterface_RefreshPumpADisplay();" in a_adjust,
            "A pump speed add/sub must refresh pump area and button UI.",
            errors)
    require("Pubinterface_RefreshPumpBDisplay();" in b_adjust,
            "B pump speed add/sub must refresh pump area and button UI.",
            errors)
    require("Pubinterface_SendPumpDisplay(UI_PUMPA_ID, UI_PUMPABUTTON_ID, &pumpMessageA)" in pub and
            "SendUIDSMessage(pump_area_id" in pub and "SendUIDSMessage(button_area_id" in pub,
            "A pump UI helper must send both value area and button state.",
            errors)
    require("Pubinterface_SendPumpDisplay(UI_PUMPB_ID, UI_PUMPBBUTTON_ID, &pumpMessageB)" in pub and
            "SendUIDSMessage(pump_area_id" in pub and "SendUIDSMessage(button_area_id" in pub,
            "B pump UI helper must send both value area and button state.",
            errors)
    require("case 50U:" in screenkey and "screen_key = SCREENKey_HMI_EXIT;" in case_body(screenkey, "case 50U:", ("default:",)),
            "screen key legacy 50U must map to SCREENKey_HMI_EXIT.",
            errors)
    require("case 0x04 : ScreenKey_PostLegacyAction(43U); break;" in screenkey,
            "screen main-run control button 0x2404/key4 must post legacy 43U for external-control exit.",
            errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - screen button control merge checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
