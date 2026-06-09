from __future__ import annotations

import hashlib
import sys
from pathlib import Path


SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart\EH-main")
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
TFT_DIR = SCREEN_ROOT / "TFT"

MAIN_PAGE_BACKGROUND = DWIN_SET / "0.jpg"
CALIBRATION_PAGE_BACKGROUND = DWIN_SET / "3.jpg"
RUN_PAGE_REFERENCE_BACKGROUND = DWIN_SET / "4.jpg"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"
ICON_LIB = DWIN_SET / "48.icl"
MAIN_SOURCE_TFT = TFT_DIR / "4.jpg.tft"
RUN_BACKGROUND_SOURCE_TFT = TFT_DIR / "4_运行界面.jpg.tft"

DISPLAY_XLS = SCREEN_ROOT / "DisplayConfig.xls"
TOUCH_XLS = SCREEN_ROOT / "TouchConfig.xls"

EXPECTED_SHOW_VPS = [
    0x1401,
    0x1402,
    0x1403,
    0x1404,
    0x1405,
    0x1406,
    0x1407,
    0x1408,
    0x1409,
    0x1410,
    0x1411,
    0x1412,
    0x1413,
    0x1414,
    0x1415,
    0x1416,
    0x1417,
    0x1418,
    0x1419,
    0x1420,
    0x1421,
    0x1422,
    0x1423,
    0x1424,
    0x1425,
    0x1426,
    0x1427,
    0x1428,
    0x1606,
    0x3420,
    0x3470,
    0x3530,
    0x3550,
    0x4200,
]

EXPECTED_TOUCH_VPS = [
    0x2001,
    0x2400,
    0x2401,
    0x2402,
    0x2403,
    0x2404,
    0x2405,
    0x2406,
    0x2420,
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def jpeg_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        raise ValueError(f"not a JPEG file: {path}")
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        index += 2
        if marker in (0xD8, 0xD9):
            continue
        if index + 2 > len(data):
            break
        length = int.from_bytes(data[index : index + 2], "big")
        if length < 2 or index + length > len(data):
            break
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[index + 3 : index + 5], "big")
            width = int.from_bytes(data[index + 5 : index + 7], "big")
            return width, height
        index += length
    raise ValueError(f"cannot find JPEG size: {path}")


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


def verify_files_exist(errors: list[str]) -> None:
    required = [
        DISPLAY_XLS,
        TOUCH_XLS,
        MAIN_PAGE_BACKGROUND,
        CALIBRATION_PAGE_BACKGROUND,
        RUN_PAGE_REFERENCE_BACKGROUND,
        SHOW_BIN,
        TOUCH_BIN,
        ICON_LIB,
        MAIN_SOURCE_TFT,
        RUN_BACKGROUND_SOURCE_TFT,
    ]
    for path in required:
        require(path.exists(), f"missing required screen package file: {path}", errors)


def verify_background(errors: list[str]) -> None:
    if not MAIN_PAGE_BACKGROUND.exists() or not RUN_PAGE_REFERENCE_BACKGROUND.exists():
        return
    main_hash = sha256(MAIN_PAGE_BACKGROUND)
    reference_hash = sha256(RUN_PAGE_REFERENCE_BACKGROUND)
    require(
        main_hash != reference_hash,
        "DWIN_SET page0 background must remain the startup page; page4 is the run-page background.",
        errors,
    )
    for page_name, path in (("page0", MAIN_PAGE_BACKGROUND), ("page3", CALIBRATION_PAGE_BACKGROUND), ("page4", RUN_PAGE_REFERENCE_BACKGROUND)):
        if not path.exists():
            continue
        try:
            width, height = jpeg_size(path)
            require((width, height) == (1024, 600), f"DWIN_SET {page_name} background size is {width}x{height}, expected 1024x600.", errors)
        except ValueError as exc:
            errors.append(str(exc))


def verify_export_times(errors: list[str]) -> None:
    if not DISPLAY_XLS.exists() or not TOUCH_XLS.exists():
        return
    export_time = max(DISPLAY_XLS.stat().st_mtime, TOUCH_XLS.stat().st_mtime)
    for path in (SHOW_BIN, TOUCH_BIN):
        if path.exists():
            require(
                path.stat().st_mtime >= export_time,
                f"{path.name} is older than the Excel export; regenerate DWIN_SET before downloading.",
                errors,
            )


def verify_vp_presence(errors: list[str]) -> None:
    if SHOW_BIN.exists():
        show_data = SHOW_BIN.read_bytes()
        for vp in EXPECTED_SHOW_VPS:
            require(count_big_endian_word(show_data, vp) > 0, f"14ShowFile.bin missing display VP 0x{vp:04X}.", errors)
    if TOUCH_BIN.exists():
        touch_data = TOUCH_BIN.read_bytes()
        for vp in EXPECTED_TOUCH_VPS:
            require(count_big_endian_word(touch_data, vp) > 0, f"13TouchFile.bin missing touch VP 0x{vp:04X}.", errors)


def show_page_records(data: bytes, page: int) -> list[bytes]:
    entry = 0x10 + page * 4
    if entry + 4 > len(data):
        return []
    count = data[entry] | (data[entry + 1] << 8)
    offset = (data[entry + 2] << 8) | data[entry + 3]
    records: list[bytes] = []
    for index in range(count):
        start = offset + index * 32
        end = start + 32
        if end <= len(data):
            records.append(data[start:end])
    return records


def show_record_vp(record: bytes) -> int:
    return (record[6] << 8) | record[7]


def touch_record_page(record: bytes) -> int:
    return (record[0] << 8) | record[1]


def touch_record_vp(record: bytes) -> int:
    return (record[17] << 8) | record[18]


def verify_page_layout(errors: list[str]) -> None:
    if SHOW_BIN.exists():
        show_data = SHOW_BIN.read_bytes()
        page0_records = show_page_records(show_data, 0)
        page3_records = show_page_records(show_data, 3)
        page4_records = show_page_records(show_data, 4)
        page4_vps = {show_record_vp(record) for record in page4_records}
        page3_vps = {show_record_vp(record) for record in page3_records}
        require(len(page0_records) == 0, "14ShowFile.bin page0 should not contain run-page display records.", errors)
        require(len(page4_records) == 34, f"14ShowFile.bin page4 should contain 34 main-run display records, got {len(page4_records)}.", errors)
        require(len(page3_records) == 11, f"14ShowFile.bin page3 should contain 11 calibration display records, got {len(page3_records)}.", errors)
        for vp in EXPECTED_SHOW_VPS:
            require(vp in page4_vps, f"14ShowFile.bin page4 missing main-run display VP 0x{vp:04X}.", errors)
        for vp in (0x3700, 0x3710, 0x3720, 0x3730, 0x3740, 0x3750, 0x3760, 0x3770, 0x3780, 0x3790, 0x37A0):
            require(vp in page3_vps, f"14ShowFile.bin page3 missing calibration display VP 0x{vp:04X}.", errors)

    if TOUCH_BIN.exists():
        touch_data = TOUCH_BIN.read_bytes()
        record_count = (len(touch_data) - 2) // 32
        records = [touch_data[index * 32 : (index + 1) * 32] for index in range(record_count)]
        page_to_vps: dict[int, list[int]] = {}
        for record in records:
            page_to_vps.setdefault(touch_record_page(record), []).append(touch_record_vp(record))
        require(page_to_vps.get(0, []).count(0x2001) == 1, "13TouchFile.bin page0 should contain exactly one startup 0x2001 touch record.", errors)
        for vp in (0x2400, 0x2401, 0x2402, 0x2403, 0x2404, 0x2405, 0x2406):
            require(vp in page_to_vps.get(4, []), f"13TouchFile.bin page4 missing main-run touch VP 0x{vp:04X}.", errors)
        require(page_to_vps.get(3, []).count(0x2420) >= 8, "13TouchFile.bin page3 calibration 0x2420 touch records are incomplete.", errors)


def verify_source_clues(errors: list[str]) -> None:
    if not MAIN_SOURCE_TFT.exists():
        return
    data = MAIN_SOURCE_TFT.read_bytes()
    # DGUS 的 .tft 源文件是私有混合格式，不是每个 VP 都能用纯字节搜索稳定命中。
    # 这里只校验当前可稳定观测到的主运行页源线索，完整 VP 集合由 13/14 导出文件校验。
    required_source_vps = (
        0x1401,  # A handle display source clue.
        0x1419,  # A pump progress/source clue.
        0x1421,  # A pump plus/source clue.
        0x2401,  # Motor speed key group/source clue.
        0x2406,  # B pump key group/source clue.
        0x4200,  # Cutter recognition text/source clue.
    )
    for vp in required_source_vps:
        require(count_big_endian_word(data, vp) > 0, f"TFT/4.jpg.tft missing main-run source VP clue 0x{vp:04X}.", errors)

    if RUN_BACKGROUND_SOURCE_TFT.exists():
        background_data = RUN_BACKGROUND_SOURCE_TFT.read_bytes()
        for vp in (0x2401, 0x2405, 0x2406):
            require(
                count_big_endian_word(background_data, vp) == 0,
                f"TFT/4_运行界面.jpg.tft unexpectedly contains new page0 touch VP 0x{vp:04X}; check page source mix-up.",
                errors,
            )


def main() -> int:
    errors: list[str] = []
    verify_files_exist(errors)
    verify_background(errors)
    verify_export_times(errors)
    verify_vp_presence(errors)
    verify_page_layout(errors)
    verify_source_clues(errors)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - screen DWIN_SET download package is ready for the page4 main-run layout")
    return 0


if __name__ == "__main__":
    sys.exit(main())
