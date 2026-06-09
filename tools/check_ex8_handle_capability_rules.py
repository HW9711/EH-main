from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
DRIVE_C = ROOT / "User" / "Application" / "Beep" / "sscDrive.c"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"


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
    pub = read_text(PUB_C)
    drive = read_text(DRIVE_C)
    handlescan = read_text(HANDLESCAN_C)
    pub_h = read_text(PUB_H)
    pub_c = compact(pub)
    drive_c = compact(drive)
    handlescan_c = compact(handlescan)
    pub_h_c = compact(pub_h)

    planer_tool = compact(function_body(pub, "Pubinterface_IsPlanerCapabilityTool"))
    osc_supported = compact(function_body(pub, "Pubinterface_IsOscDirectionSupported"))
    selected_refresh = compact(function_body(pub, "Pubinterface_RefreshSelectedChannelDisplay"))
    dir_active = compact(function_body(pub, "DirActive"))
    freq_active = compact(function_body(pub, "FreqActive"))
    tool_pos = compact(function_body(pub, "ToolPosActive"))
    osc_model = compact(function_body(pub, "Pubinterface_IsOscDirectionSupportedModel"))
    brushed = compact(function_body(drive, "MotorDrive_IsBrushedTool"))
    motor_run = compact(function_body(drive, "MOTORRUN"))

    require("#defineEMBC_ONLINES" in pub_h_c, "Pubinterface.h must define EMBC_ONLINES.", errors)
    require("{0x6B,0x0F,EMBC_ONLINES," in handlescan_c,
            "handlescan must recognize EMBC by EEPROM Page2 0x6B/0x0F.", errors)

    require("tool_type==PLANER" in planer_tool and
            "tool_type==PX_YIM_ONLINES" in planer_tool and
            "tool_type==PX_YIP_ONLINES" in planer_tool,
            "planer-capability helper must accept PLANER and PXM/PXP tool codes.", errors)
    require("Pubinterface_IsPlanerCapabilityTool(tool_type)" in osc_supported and
            "Pubinterface_IsOscDirectionSupportedModel(hand_model)" in osc_supported and
            "hand_model==PX_YIM_ONLINES" in osc_model and
            "hand_model==PX_YIP_ONLINES" in osc_model,
            "osc-support helper must use both selected tool capability and PXM/PXP handle codes.", errors)
    require("Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model,WorkMessage.tool_type)" in selected_refresh,
            "selected-channel refresh must decide OSC UI from handle plus tool capability.", errors)
    require("Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model,WorkMessage.tool_type)==false" in dir_active,
            "DirActive must reject OSC using the combined support helper.", errors)
    require("Pubinterface_IsOscDirectionSupported(WorkMessage.hand_model,WorkMessage.tool_type)==false" in freq_active,
            "FreqActive must reject frequency changes using the combined support helper.", errors)
    require("Pubinterface_IsPlanerCapabilityTool(WorkMessage.tool_type)==false" in tool_pos,
            "ToolPosActive must gate open-position by tool capability, not reduction ratio.", errors)
    require("tool_reduction_ratio" not in tool_pos and "500U" not in tool_pos,
            "ToolPosActive must not keep the old low16 reduction-ratio 500 gate.", errors)

    require("hand_model==PX_YIM_ONLINES" in brushed and
            "hand_model==PX_YIP_ONLINES" in brushed and
            "tool_type==PX_YIM_ONLINES" in brushed and
            "tool_type==PX_YIP_ONLINES" in brushed,
            "motor brushed helper must treat PXM/PXP as brushed from either handle or tool code.", errors)
    require("MotorDrive_IsBrushedTool(WorkMessage.hand_model,WorkMessage.tool_type)" in motor_run,
            "MOTORRUN must pass both hand_model and tool_type into brushed detection.", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - EX8 handle capability rules passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
