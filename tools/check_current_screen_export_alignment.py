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
SOURCE_REFERENCE_ROOTS = [
    ROOT / "User" / "Application",
    ROOT / "User" / "UI",
]
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

PAGE4_TOUCH_ACTIONS = {
    (0x2400, 0x0001): ("ScreenKey_PostLegacyAction(24U);",),
    (0x2400, 0x0002): ("ScreenKey_PostLegacyAction(25U);",),
    (0x2400, 0x0003): ("ScreenKey_PostLegacyAction(22U);",),
    (0x2400, 0x0004): ("ScreenKey_PostLegacyAction(23U);",),
    (0x2400, 0x0005): ("ScreenKey_PostLegacyAction(20U);",),
    (0x2400, 0x0006): ("ScreenKey_PostLegacyAction(21U);",),
    (0x2400, 0x0007): ("ScreenKey_PostLegacyAction(36U);",),
    (0x2401, 0x0001): ("ScreenKey_PostLegacyAction(30U);",),
    (0x2401, 0x0002): ("ScreenKey_PostLegacyAction(31U);",),
    (0x2401, 0x0003): ("ScreenKey_PostLegacyAction(32U);",),
    (0x2401, 0x0004): ("ScreenKey_PostLegacyAction(33U);",),
    (0x2402, 0x0001): ("ScreenKey_PostLegacyAction(13U);",),
    (0x2402, 0x0002): ("ScreenKey_PostLegacyAction(15U);",),
    (0x2402, 0x0003): ("ScreenKey_PostLegacyAction(14U);",),
    (0x2403, 0x0001): ("ScreenKey_PostLegacyAction(9U);",),
    (0x2403, 0x0002): ("ScreenKey_PostLegacyAction(10U);",),
    (0x2404, 0x0001): ("ScreenKey_PostLegacyAction(16U);",),
    (0x2404, 0x0002): ("ScreenKey_PostLegacyAction(17U);",),
    (0x2404, 0x0003): ("ScreenKey_PostLegacyAction(18U);",),
    (0x2404, 0x0004): ("ScreenKey_PostLegacyAction(43U);",),
    (0x2405, 0x0001): ("ScreenKey_PostLegacyAction(7U);",),
    (0x2405, 0x0002): ("ScreenKey_PostLegacyAction(8U);",),
    (0x2405, 0x0003): ("ScreenKey_PostLegacyAction(12U);",),
    (0x2406, 0x0001): ("ScreenKey_PostLegacyAction(5U);",),
    (0x2406, 0x0002): ("ScreenKey_PostLegacyAction(6U);",),
    (0x2406, 0x0003): ("ScreenKey_PostLegacyAction(11U);",),
    (0x2407, 0x0002): ("ScreenKey_PostLegacyAction(42U);",),
    (0x0155, 0x5520): (
        "case 0x55:",
        "case 0x20 : ScreenKey_PostLegacyAction(44U);",
        "case 0x30 : ScreenKey_PostLegacyAction(42U);",
    ),
}

IGNORED_PAGE4_TOUCH = {
    (0x0155, 0x5510),  # 旧触控启动键仍可能存在于导出记录前半段，固件按新逻辑忽略。
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


def touch_record_extra_vp(record: bytes) -> int:
    return read_u16_be(record, 25)


def touch_record_extra_key(record: bytes) -> int:
    return read_u16_be(record, 28)


def touch_record_pairs(record: bytes) -> list[tuple[int, int]]:
    pairs = [(touch_record_vp(record), touch_record_key(record))]
    extra_pair = (touch_record_extra_vp(record), touch_record_extra_key(record))
    if extra_pair != (0, 0):
        pairs.append(extra_pair)
    return pairs


def screen_vp_macros(address_h: str) -> dict[int, list[str]]:
    macros: dict[int, list[str]] = {}
    for name, value in re.findall(r"#define\s+(UIDP_LCD_VP_[A-Za-z0-9_]+)\s+0x([0-9A-Fa-f]+)U?", address_h):
        macros.setdefault(int(value, 16), []).append(name)
    return macros


def firmware_source_references() -> str:
    chunks: list[str] = []
    for source_root in SOURCE_REFERENCE_ROOTS:
        for source_path in sorted(source_root.rglob("*.c")):
            # 只统计固件写屏源码，不把 screen_address.h 的宏定义误当成运行时引用。
            chunks.append(read_text(source_path))
    return "\n".join(chunks)


def verify_page4_display_firmware_coverage(show_data: bytes, address_h: str, errors: list[str]) -> None:
    macros_by_vp = screen_vp_macros(address_h)
    firmware_refs = firmware_source_references()
    page4_vps = sorted({show_record_vp(record) for record in show_page_records(show_data, 4)})

    for vp in page4_vps:
        macros = macros_by_vp.get(vp, [])
        require(bool(macros), f"EX8 page4 display VP 0x{vp:04X} is missing a UIDP_LCD_VP_* macro.", errors)
        if macros:
            require(
                any(macro in firmware_refs for macro in macros),
                f"EX8 page4 display VP 0x{vp:04X} macros {macros} have no firmware write reference.",
                errors,
            )


def verify_page4_touch_firmware_coverage(touch_data: bytes, screenkey: str, errors: list[str]) -> None:
    scan = compact(function_body(screenkey, "ScreenKey_Scan"))
    exported_pairs: set[tuple[int, int]] = set()
    for record in touch_records(touch_data):
        if touch_record_page(record) == 4:
            exported_pairs.update(touch_record_pairs(record))

    for vp, key in sorted(exported_pairs):
        if (vp, key) in IGNORED_PAGE4_TOUCH:
            continue
        action_needles = PAGE4_TOUCH_ACTIONS.get((vp, key))
        require(
            action_needles is not None,
            f"EX8 page4 touch VP 0x{vp:04X}/key 0x{key:04X} has no firmware parser expectation.",
            errors,
        )
        if action_needles is None:
            continue
        for needle in action_needles:
            require(
                compact(needle) in scan,
                f"EX8 page4 touch VP 0x{vp:04X}/key 0x{key:04X} is not covered by ScreenKey_Scan action {needle!r}.",
                errors,
            )

    # 0x0155/key 0x5520 是当前触控运行保活入口；0x5510 旧启动入口不再作为固件必需项。
    require((0x0155, 0x5520) in exported_pairs,
            "EX8 page4 touch active-area VP 0x0155/key 0x5520 is missing from 13TouchFile.bin.", errors)


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
    require("case0x01:ScreenKey_PostLegacyAction(30U);break;" in scan,
            "0x2401/key1 must map to fast speed decrease.", errors)
    require("case0x02:ScreenKey_PostLegacyAction(31U);break;" in scan,
            "0x2401/key2 must map to slow speed decrease.", errors)
    require("case0x03:ScreenKey_PostLegacyAction(32U);break;" in scan,
            "0x2401/key3 must map to slow speed increase.", errors)
    require("case0x04:ScreenKey_PostLegacyAction(33U);break;" in scan,
            "0x2401/key4 must map to fast speed increase.", errors)
    require("case0x02:" in scan and "ScreenKey_PostLegacyAction(13U)" in scan and "ScreenKey_PostLegacyAction(15U)" in scan and "ScreenKey_PostLegacyAction(14U)" in scan,
            "0x2402 direction keys must map forward/osc/reverse.", errors)
    require("case0x01:ScreenKey_PostLegacyAction(9U);break;" in scan,
            "0x2403/key1 must map to frequency increase.", errors)
    require("case0x02:ScreenKey_PostLegacyAction(10U);break;" in scan,
            "0x2403/key2 must map to frequency decrease.", errors)
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
    require("case0x51:" not in scan and "case0x52:" not in scan and
            "case0x53:" not in scan and "case0x54:" not in scan,
            "current EX8 firmware must not keep old-screen 0x51-0x54 touch parser branches.", errors)

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
    require("LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS,499U)" in uidp_c and
            "LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS,501U)" in uidp_c,
            "B-pump +/- display must use current EX8 0x1422/0x1424 icon resources 499/501.", errors)

    project_lists = "\n".join(read_text(path) for path in PROJECT_LISTS if path.exists())
    require(not ADAPTER_C.exists(), "screen_adapter.c should stay deleted.", errors)
    require(not SCREEN_H.exists(), "screen.h should stay deleted.", errors)
    require("screen_adapter.c" not in project_lists and "screen.h" not in project_lists,
            "project source lists must not re-register the old screen interface.", errors)

    if SHOW_BIN.exists():
        verify_page4_display_firmware_coverage(SHOW_BIN.read_bytes(), address_h, errors)
    if TOUCH_BIN.exists():
        verify_page4_touch_firmware_coverage(TOUCH_BIN.read_bytes(), screenkey, errors)


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
