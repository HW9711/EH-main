from __future__ import annotations

import hashlib
import sys
from pathlib import Path


SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart")
    / "DL-EX8\u4e2d\u6587 \u62bd\u54380609"
    / "DL-EX8\u4e2d\u6587 \u62bd\u5438"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
TFT_DIR = SCREEN_ROOT / "TFT"

DISPLAY_EXPORT = SCREEN_ROOT / "DisplayConfig.xls"
TOUCH_EXPORT = SCREEN_ROOT / "TouchConfig.xls"
STARTUP_BACKGROUND = DWIN_SET / "0.jpg"
CALIBRATION_BACKGROUND = DWIN_SET / "3.jpg"
RUN_BACKGROUND = DWIN_SET / "4.jpg"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"
ICON_LIB = DWIN_SET / "48.icl"
MAIN_SOURCE_TFT = TFT_DIR / "4.jpg.tft"
RUN_BACKGROUND_SOURCE_TFT = TFT_DIR / "4_\u8fd0\u884c\u754c\u9762.jpg.tft"

EXPECTED_SHOW_VPS = {
    0x1401, 0x1402, 0x1403, 0x1404, 0x1405, 0x1406, 0x1407, 0x1408,
    0x1409, 0x1410, 0x1411, 0x1412, 0x1413, 0x1414, 0x1415, 0x1416,
    0x1417, 0x1418, 0x1419, 0x1420, 0x1421, 0x1422, 0x1423, 0x1424,
    0x1425, 0x1426, 0x1427, 0x1428, 0x1429, 0x1430, 0x3420, 0x3470,
    0x3530, 0x3550, 0x4200,
}

EXPECTED_TOUCH_VPS = {0x2001, 0x2400, 0x2401, 0x2402, 0x2403, 0x2404, 0x2405, 0x2406, 0x2407, 0x2420}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def jpeg_size(path: Path) -> tuple[int, int] | None:
    data = path.read_bytes()
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        return None
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        index += 2
        if marker in (0xD8, 0xD9):
            continue
        length = int.from_bytes(data[index:index + 2], "big")
        if length < 2 or index + length > len(data):
            return None
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[index + 3:index + 5], "big")
            width = int.from_bytes(data[index + 5:index + 7], "big")
            return width, height
        index += length
    return None


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


def verify_files_exist(errors: list[str]) -> None:
    required = [
        DISPLAY_EXPORT,
        TOUCH_EXPORT,
        STARTUP_BACKGROUND,
        CALIBRATION_BACKGROUND,
        RUN_BACKGROUND,
        SHOW_BIN,
        TOUCH_BIN,
        ICON_LIB,
        MAIN_SOURCE_TFT,
        RUN_BACKGROUND_SOURCE_TFT,
    ]
    for path in required:
        require(path.exists(), f"missing required EX8 screen package file: {path}", errors)


def verify_background(errors: list[str]) -> None:
    if not STARTUP_BACKGROUND.exists() or not RUN_BACKGROUND.exists():
        return
    require(sha256(STARTUP_BACKGROUND) != sha256(RUN_BACKGROUND),
            "DWIN_SET page0 background must stay separate from the page4 run background.", errors)
    size = jpeg_size(STARTUP_BACKGROUND)
    require(size == (1280, 800), f"DWIN_SET page0 JPEG size is {size}, expected 1280x800.", errors)
    for path in (CALIBRATION_BACKGROUND, RUN_BACKGROUND):
        if path.exists():
            require(path.stat().st_size > 0, f"{path.name} must not be empty.", errors)


def verify_export_times(errors: list[str]) -> None:
    if not DISPLAY_EXPORT.exists() or not TOUCH_EXPORT.exists():
        return
    export_time = max(DISPLAY_EXPORT.stat().st_mtime, TOUCH_EXPORT.stat().st_mtime)
    for path in (SHOW_BIN, TOUCH_BIN):
        if path.exists():
            require(
                path.stat().st_mtime >= export_time - 2.0,
                f"{path.name} is older than the screen export by more than 2 seconds; regenerate DWIN_SET before downloading.",
                errors,
            )


def verify_page_layout(errors: list[str]) -> None:
    if SHOW_BIN.exists():
        show_data = SHOW_BIN.read_bytes()
        page0_records = show_page_records(show_data, 0)
        page3_records = show_page_records(show_data, 3)
        page4_records = show_page_records(show_data, 4)
        page4_vps = {show_record_vp(record) for record in page4_records}
        page3_vps = {show_record_vp(record) for record in page3_records}
        require(len(page0_records) == 0, "14ShowFile.bin page0 should not contain run-page display records.", errors)
        require(len(page4_records) == 35, f"14ShowFile.bin page4 should contain 35 main-run display records, got {len(page4_records)}.", errors)
        require(len(page3_records) == 11, f"14ShowFile.bin page3 should contain 11 calibration display records, got {len(page3_records)}.", errors)
        for vp in sorted(EXPECTED_SHOW_VPS):
            require(vp in page4_vps, f"14ShowFile.bin page4 missing main-run display VP 0x{vp:04X}.", errors)
        for vp in (0x3700, 0x3710, 0x3720, 0x3730, 0x3740, 0x3750, 0x3760, 0x3770, 0x3780, 0x3790, 0x37A0):
            require(vp in page3_vps, f"14ShowFile.bin page3 missing calibration display VP 0x{vp:04X}.", errors)

    if TOUCH_BIN.exists():
        touch_data = TOUCH_BIN.read_bytes()
        page_to_vps: dict[int, list[int]] = {}
        for record in touch_records(touch_data):
            page_to_vps.setdefault(touch_record_page(record), []).append(touch_record_vp(record))
        require(page_to_vps.get(0, []).count(0x2001) == 1, "13TouchFile.bin page0 should contain exactly one startup 0x2001 touch record.", errors)
        for vp in EXPECTED_TOUCH_VPS:
            pages = [page for page, vps in page_to_vps.items() if vp in vps]
            require(bool(pages), f"13TouchFile.bin missing touch VP 0x{vp:04X}.", errors)
        for vp in (0x2400, 0x2401, 0x2402, 0x2403, 0x2404, 0x2405, 0x2406, 0x2407):
            require(vp in page_to_vps.get(4, []), f"13TouchFile.bin page4 missing main-run touch VP 0x{vp:04X}.", errors)
        require(page_to_vps.get(3, []).count(0x2420) >= 8, "13TouchFile.bin page3 calibration 0x2420 touch records are incomplete.", errors)


def count_big_endian_word(data: bytes, value: int) -> int:
    needle = value.to_bytes(2, "big")
    count = 0
    start = 0
    while True:
        index = data.find(needle, start)
        if index < 0:
            return count
        count += 1
        start = index + 1


def verify_source_clues(errors: list[str]) -> None:
    # The EX8 .tft source file is a proprietary mixed format, so VP byte clues are optional;
    # the authoritative package check above uses DWIN_SET/13TouchFile.bin and 14ShowFile.bin.
    _ = errors


def main() -> int:
    errors: list[str] = []
    verify_files_exist(errors)
    verify_background(errors)
    verify_export_times(errors)
    verify_page_layout(errors)
    verify_source_clues(errors)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - EX8 screen DWIN_SET download package is ready")
    return 0


if __name__ == "__main__":
    sys.exit(main())
