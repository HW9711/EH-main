from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart")
    / "DL-EX8\u4e2d\u6587 \u62bd\u54380609"
    / "DL-EX8\u4e2d\u6587 \u62bd\u5438"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"
STARTUP_BACKGROUND = DWIN_SET / "0.jpg"
RUN_BACKGROUND = DWIN_SET / "4.jpg"

SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
ADAPTER_C = ROOT / "User" / "Application" / "Screen" / "screen_adapter.c"
SCREEN_H = ROOT / "User" / "Application" / "include" / "screen.h"
PROJECT_LISTS = [
    ROOT / "EIDE" / ".eide" / "eide.yml",
    ROOT / "EIDE" / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvprojx",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvoptx",
]

EXPECTED_PAGE4_SHOW_VPS = {
    0x1401, 0x1402, 0x1403, 0x1404, 0x1405, 0x1406, 0x1407, 0x1408,
    0x1409, 0x1410, 0x1411, 0x1412, 0x1413, 0x1414, 0x1415, 0x1416,
    0x1417, 0x1418, 0x1419, 0x1420, 0x1421, 0x1422, 0x1423, 0x1424,
    0x1425, 0x1426, 0x1427, 0x1428, 0x1429, 0x1430, 0x3420, 0x3470,
    0x3530, 0x3550, 0x4200,
}

EXPECTED_PAGE3_SHOW_VPS = {
    0x3700, 0x3710, 0x3720, 0x3730, 0x3740, 0x3750,
    0x3760, 0x3770, 0x3780, 0x3790, 0x37A0,
}

EXPECTED_PAGE4_TOUCH = {
    0x2400: {1, 2, 3, 4, 5, 6, 7},
    0x2401: {1, 2, 3, 4},
    0x2402: {1, 2, 3},
    0x2403: {1, 2},
    0x2404: {1, 2, 3, 4},
    0x2405: {1, 2, 3},
    0x2406: {1, 2, 3},
    0x2407: {2},
}


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
                return text[start:index + 1]
    raise AssertionError(f"unterminated function: {name}")


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def read_u16_be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def show_page_records(data: bytes, page: int) -> list[bytes]:
    entry = 0x10 + page * 4
    count = data[entry] | (data[entry + 1] << 8)
    offset = read_u16_be(data, entry + 2)
    return [data[offset + index * 32: offset + (index + 1) * 32] for index in range(count)]


def show_record_vp(record: bytes) -> int:
    return read_u16_be(record, 6)


def touch_records(data: bytes) -> list[bytes]:
    record_count = (len(data) - 2) // 32
    return [data[index * 32: (index + 1) * 32] for index in range(record_count)]


def touch_record_page(record: bytes) -> int:
    return read_u16_be(record, 0)


def touch_record_vp(record: bytes) -> int:
    return read_u16_be(record, 17)


def touch_record_key(record: bytes) -> int:
    return read_u16_be(record, 20)


def verify_screen_package(errors: list[str]) -> None:
    for path in (SHOW_BIN, TOUCH_BIN, STARTUP_BACKGROUND, RUN_BACKGROUND):
        require(path.exists(), f"missing screen package file: {path}", errors)
    if errors:
        return

    show_data = SHOW_BIN.read_bytes()
    page0_show = show_page_records(show_data, 0)
    page3_show = show_page_records(show_data, 3)
    page4_show = show_page_records(show_data, 4)
    page3_vps = {show_record_vp(record) for record in page3_show}
    page4_vps = {show_record_vp(record) for record in page4_show}

    require(len(page0_show) == 0, "EX8 page0 must not contain main-run display records.", errors)
    require(len(page3_show) == 11, f"EX8 calibration page should contain 11 display records, got {len(page3_show)}.", errors)
    require(len(page4_show) == 35, f"EX8 main-run page should contain 35 display records, got {len(page4_show)}.", errors)
    for vp in sorted(EXPECTED_PAGE3_SHOW_VPS):
        require(vp in page3_vps, f"EX8 page3 display missing VP 0x{vp:04X}.", errors)
    for vp in sorted(EXPECTED_PAGE4_SHOW_VPS):
        require(vp in page4_vps, f"EX8 page4 display missing VP 0x{vp:04X}.", errors)

    touch_data = TOUCH_BIN.read_bytes()
    page_touch: dict[int, dict[int, set[int]]] = {}
    for record in touch_records(touch_data):
        page = touch_record_page(record)
        vp = touch_record_vp(record)
        key = touch_record_key(record)
        page_touch.setdefault(page, {}).setdefault(vp, set()).add(key)

    require(page_touch.get(0, {}).get(0x2001, set()) == {0x2001},
            "EX8 page0 should keep exactly one 0x2001 startup touch record.", errors)
    require(page_touch.get(3, {}).get(0x2420, set()).issuperset({1, 2, 3, 4, 5, 6, 7, 8}),
            "EX8 page3 calibration 0x2420 keys 1-8 are incomplete.", errors)
    page4_touch = page_touch.get(4, {})
    for vp, expected_keys in EXPECTED_PAGE4_TOUCH.items():
        actual_keys = page4_touch.get(vp, set())
        require(actual_keys.issuperset(expected_keys),
                f"EX8 page4 touch VP 0x{vp:04X} keys {sorted(actual_keys)} do not cover {sorted(expected_keys)}.",
                errors)

    require(STARTUP_BACKGROUND.read_bytes() != RUN_BACKGROUND.read_bytes(),
            "DWIN_SET page0 background must remain separate from the page4 run background.", errors)


def verify_firmware_mapping(errors: list[str]) -> None:
    screenkey = read_text(SCREENKEY_C)
    uidp = read_text(UIDP_C)
    address_h = read_text(SCREEN_ADDRESS_H)
    scan = compact(function_body(screenkey, "ScreenKey_Scan"))
    uidp_behavior = compact(function_body(uidp, "UIDISPLAYBehavior"))
    uidp_c = compact(uidp)
    address_c = compact(address_h)

    require("KEY_CONTINUOUSCLICK" not in scan, "startup 0x2001 must not keep the old hidden-entry action.", errors)
    require("case0x00:" in scan and "case0x07:ScreenKey_PostLegacyAction(36U);" in scan,
            "0x2400 top-key group must include auto-identify key7.", errors)
    require("case0x01:" in scan and "ScreenKey_PostLegacyAction(30U)" in scan and "ScreenKey_PostLegacyAction(33U)" in scan,
            "0x2401 speed keys must map to 10000/1000 step events.", errors)
    require("case0x02:" in scan and "ScreenKey_PostLegacyAction(13U)" in scan and "ScreenKey_PostLegacyAction(15U)" in scan and "ScreenKey_PostLegacyAction(14U)" in scan,
            "0x2402 direction keys must map forward/osc/reverse.", errors)
    require("case0x04:" in scan and "ScreenKey_PostLegacyAction(16U)" in scan and "ScreenKey_PostLegacyAction(43U)" in scan,
            "0x2404 control keys must include foot and external-exit actions.", errors)
    require("case0x05:" in scan and "ScreenKey_PostLegacyAction(7U)" in scan and "ScreenKey_PostLegacyAction(12U)" in scan,
            "0x2405 A-pump keys must include plus/minus/start-stop.", errors)
    require("case0x06:" in scan and "ScreenKey_PostLegacyAction(5U)" in scan and "ScreenKey_PostLegacyAction(11U)" in scan,
            "0x2406 B-pump keys must include plus/minus/start-stop.", errors)
    require("case0x07:" in scan and "case0x02:ScreenKey_PostLegacyAction(42U);" in scan,
            "0x2407/key2 must remain mapped to touch-exit on the EX8 touch work area.", errors)
    require("case0x55:" in scan and "case0x20:ScreenKey_PostLegacyAction(44U);" in scan,
            "0x5520 touch keep-alive must be parsed for press-and-hold touch running.", errors)

    expected_address_macros = {
        "UIDP_LCD_VP_PUMP_B_PLUS": "0x1422U",
        "UIDP_LCD_VP_PUMP_B_MINUS": "0x1424U",
        "UIDP_LCD_VP_PUMP_B_BUTTON": "0x1428U",
        "UIDP_LCD_VP_ALARM_TIP": "0x1429U",
        "UIDP_LCD_VP_TOUCH_WORK": "0x1430U",
    }
    for macro, value in expected_address_macros.items():
        require(f"#define{macro}{value}" in address_c, f"screen_address.h missing {macro}={value}.", errors)

    require("LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);" in uidp_behavior,
            "power-init display must force the EX8 main-run page macro.", errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_TOUCH_WORK,70U)" in uidp_c,
            "touch active display must draw the EX8 0x1430 touch work picture.", errors)
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS,225U)" in uidp_c and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS,224U)" in uidp_c,
            "B-pump +/- display must use EX8 0x1422/0x1424 macros.", errors)

    project_lists = "\n".join(read_text(path) for path in PROJECT_LISTS if path.exists())
    require(not ADAPTER_C.exists(), "screen_adapter.c should stay deleted.", errors)
    require(not SCREEN_H.exists(), "screen.h should stay deleted.", errors)
    require("screen_adapter.c" not in project_lists and "screen.h" not in project_lists,
            "project source lists must not re-register the old screen interface.", errors)


def main() -> int:
    errors: list[str] = []
    verify_screen_package(errors)
    verify_firmware_mapping(errors)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - current EX8 screen export alignment checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
