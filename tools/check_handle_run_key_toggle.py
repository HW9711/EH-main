from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HANDLEKEY_C = ROOT / "User" / "Application" / "Handle" / "handlekey.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def function_body(text: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*(?://[^\n]*\n\s*)?\{{", text)
    if not match:
        return ""
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
    return ""


def main() -> int:
    errors: list[str] = []
    source = read_text(HANDLEKEY_C)
    source_c = compact(source)
    debounce_c = compact(function_body(source, "HandleRunKey_DebouncePressEvent"))
    process_c = compact(function_body(source, "HandleRunKey_Process"))
    scan_c = compact(function_body(source, "HandleKey_ScanRunKeys"))

    require("boolpress_event;" in source_c,
            "run-key debounce state must record a stable press event, not only the held level.",
            errors)
    require("filter->press_event=false;" in debounce_c,
            "debounce must clear the one-shot press event at the start of every scan.",
            errors)
    require("if(filter->stable_pressed==false)" in debounce_c and
            "filter->press_event=true;" in debounce_c and
            "filter->stable_pressed=true;" in debounce_c,
            "debounce must emit a press event only on the false-to-true stable transition.",
            errors)
    require("filter->stable_pressed=false;" in debounce_c and
            "filter->press_event=true;" not in debounce_c.split("else", 1)[-1],
            "stable release must re-arm the next press but must not emit a stop event.",
            errors)
    require("returnfilter->press_event;" in debounce_c,
            "debounce helper must return only the one-shot press event.",
            errors)

    require("HandleRunKey_Process(uint8_tchannel,boolpress_event)" in source_c,
            "run-key process must consume press events rather than held levels.",
            errors)
    require("(pressed==false)" not in process_c and "(press_event==false)" in process_c,
            "release must not stop the motor; absence of a press event should be ignored.",
            errors)
    require("if(s_handle_run_key_owner_channel==channel)" in process_c and
            "if(press_event==true)" in process_c and
            "HandleRunKey_SetMotorRun(false)" in process_c,
            "second stable press on the owner channel must stop the motor.",
            errors)
    require("WorkMessage.alarm_flag==true" in process_c and
            "WorkMessage.channel_work!=channel" in process_c and
            "HandleRunKey_IsChannelReady(channel)==false" in process_c,
            "alarm, channel switch, and offline state must still force stop.",
            errors)

    require("HandleRunKey_DebouncePressEvent(&s_handle_run_key_a_filter,HANDLE_RUN_KEY_A_STATUS())" in scan_c,
            "A run-key scan must use the press-event debounce helper.",
            errors)
    require("HandleRunKey_DebouncePressEvent(&s_handle_run_key_b_filter,HANDLE_RUN_KEY_B_STATUS())" in scan_c,
            "B run-key scan must use the press-event debounce helper.",
            errors)
    require("HandleRunKey_Process(CHANNEL_A,key_a_press_event)" in scan_c and
            "HandleRunKey_Process(CHANNEL_B,key_b_press_event)" in scan_c,
            "run-key scan must pass one-shot press events into the channel processor.",
            errors)
    require("实体键采用稳定按下沿翻转启停" in source,
            "task comment must describe press-to-toggle behavior so later maintenance does not restore hold-to-run.",
            errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - handle run key toggle checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
