from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"


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
                return text[start:index + 1]
    raise AssertionError(f"unterminated function: {name}")


def main() -> int:
    errors: list[str] = []
    pub = read_text(PUB_C)
    pub_c = compact(pub)

    require("ControlArbitration_IsRuntimeAdjustmentKey" in pub,
            "runtime speed/frequency/flow adjustment keys must have an arbitration bypass helper.",
            errors)
    require("ControlArbitration_IsPumpBusinessKey" in pub,
            "foot/screen/HMI pump business keys must have a source-independent arbitration helper.",
            errors)

    if "ControlArbitration_IsRuntimeAdjustmentKey" in pub:
        adjust = compact(function_body(pub, "ControlArbitration_IsRuntimeAdjustmentKey"))
        for key in (
            "HANDLEKey_speed_add", "HANDLEKey_speed_sub",
            "HMIkey_SPEED_Add", "HMIkey_SPEED_Sub",
            "SCREENKey_SPEED_Add", "SCREENKey_SPEED_Sub",
            "SCREENKey_SPEED_Sub_Large", "SCREENKey_SPEED_Sub_Small",
            "SCREENKey_SPEED_Add_Small", "SCREENKey_SPEED_Add_Large",
            "HMIkey_FREQ_Add", "HMIkey_FREQ_Sub",
            "SCREENKey_FREQ_Add", "SCREENKey_FREQ_Sub",
            "JTkey_left_short", "JTKey_right_short",
            "HMIkey_APUMP_Add", "HMIkey_APUMP_Sub",
            "HMIkey_BPUMP_Add", "HMIkey_BPUMP_Sub",
            "SCREENKey_APUMP_Add", "SCREENKey_APUMP_Sub",
            "SCREENKey_BPUMP_Add", "SCREENKey_BPUMP_Sub",
        ):
            require(key in adjust, f"runtime adjustment bypass must include {key}.", errors)

    if "ControlArbitration_IsPumpBusinessKey" in pub:
        pump = compact(function_body(pub, "ControlArbitration_IsPumpBusinessKey"))
        for key in (
            "JTkey_left_short", "JTKey_left_long",
            "JTKey_right_short", "JTKey_right_long",
            "JTKey_Gently_left_start", "JTKey_Gently_left_stop",
            "JTKey_Gently_right_start", "JTKey_Gently_rigth_stop",
            "HMIkey_APUMP_Add", "HMIkey_APUMP_Sub", "HMIkey_APUMP_control",
            "HMIkey_BPUMP_Add", "HMIkey_BPUMP_Sub", "HMIkey_BPUMP_control",
            "HMIkey_Gently_start", "HMIkey_Gently_stop",
            "SCREENKey_APUMP_Add", "SCREENKey_APUMP_Sub", "SCREENKey_APUMP_control",
            "SCREENKey_BPUMP_Add", "SCREENKey_BPUMP_Sub", "SCREENKey_BPUMP_control",
        ):
            require(key in pump, f"pump business bypass must include {key}.", errors)

    should_block = compact(function_body(pub, "ControlArbitration_ShouldBlockLocalKey"))
    require("ControlArbitration_IsRuntimeAdjustmentKey(control_type,control_key)" in should_block,
            "ControlArbitration_ShouldBlockLocalKey must allow runtime adjustment keys before generic busy-owner blocking.",
            errors)
    require("ControlArbitration_IsPumpBusinessKey(control_type,control_key)" in should_block,
            "ControlArbitration_ShouldBlockLocalKey must allow pump business keys before generic busy-owner blocking.",
            errors)
    require(should_block.find("ControlArbitration_IsRuntimeAdjustmentKey(control_type,control_key)") <
            should_block.find("ControlArbitration_IsBusyByOther(key_owner)"),
            "runtime adjustment bypass must run before ControlArbitration_IsBusyByOther.",
            errors)
    require(should_block.find("ControlArbitration_IsPumpBusinessKey(control_type,control_key)") <
            should_block.find("ControlArbitration_IsBusyByOther(key_owner)"),
            "pump business bypass must run before ControlArbitration_IsBusyByOther.",
            errors)
    require("ControlArbitration_IsOwner(CONTROL_OWNER_EXTERNAL)&&key_owner!=CONTROL_OWNER_EXTERNAL" in should_block,
            "local runtime/pump keys must still be blocked while external control owns the system.",
            errors)

    pump_active = compact(function_body(pub, "PUMPActive"))
    require("pumpMessageA.timingDrainage_flag==true" in pump_active and
            "pumpMessageB.timingDrainage_flag==true" in pump_active,
            "INJECTWATER timed-drainage flow adjustment guard must stay in PUMPActive.",
            errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - runtime adjustment arbitration checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
