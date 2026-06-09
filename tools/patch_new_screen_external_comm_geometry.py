from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path


SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart\EH-main")
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"


def read_u16_be(data: bytes | bytearray, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def write_u16_be(data: bytearray, offset: int, value: int) -> None:
    data[offset] = (value >> 8) & 0xFF
    data[offset + 1] = value & 0xFF


def show_page_records(data: bytearray, page: int) -> list[int]:
    entry = 0x10 + page * 4
    count = data[entry] | (data[entry + 1] << 8)
    offset = read_u16_be(data, entry + 2)
    return [offset + index * 32 for index in range(count)]


def show_record_vp(data: bytearray, offset: int) -> int:
    return read_u16_be(data, offset + 6)


def touch_record_page(data: bytearray, offset: int) -> int:
    return read_u16_be(data, offset)


def touch_record_vp(data: bytearray, offset: int) -> int:
    return read_u16_be(data, offset + 17)


def touch_record_key(data: bytearray, offset: int) -> int:
    return read_u16_be(data, offset + 20)


def backup_files() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = SCREEN_ROOT / f"backup_{stamp}_before_external_comm_geometry_patch"
    backup_dir.mkdir(parents=True, exist_ok=False)
    shutil.copy2(SHOW_BIN, backup_dir / SHOW_BIN.name)
    shutil.copy2(TOUCH_BIN, backup_dir / TOUCH_BIN.name)
    return backup_dir


def patch_show_file() -> bool:
    data = bytearray(SHOW_BIN.read_bytes())
    changed = False
    for offset in show_page_records(data, 4):
        if show_record_vp(data, offset) != 0x1416:
            continue
        if read_u16_be(data, offset + 8) != 683:
            write_u16_be(data, offset + 8, 683)
            changed = True
        if read_u16_be(data, offset + 10) != 443:
            write_u16_be(data, offset + 10, 443)
            changed = True
    if changed:
        SHOW_BIN.write_bytes(data)
    return changed


def patch_touch_file() -> bool:
    data = bytearray(TOUCH_BIN.read_bytes())
    changed = False
    record_count = (len(data) - 2) // 32
    for index in range(record_count):
        offset = index * 32
        if touch_record_page(data, offset) != 4:
            continue
        if touch_record_vp(data, offset) != 0x2404:
            continue
        if touch_record_key(data, offset) != 4:
            continue
        values = ((offset + 2, 683), (offset + 4, 443), (offset + 6, 818), (offset + 8, 493))
        for value_offset, value in values:
            if read_u16_be(data, value_offset) != value:
                write_u16_be(data, value_offset, value)
                changed = True
    if changed:
        TOUCH_BIN.write_bytes(data)
    return changed


def main() -> int:
    if not SHOW_BIN.exists() or not TOUCH_BIN.exists():
        raise FileNotFoundError("DWIN_SET show/touch files are missing.")
    backup_dir = backup_files()
    show_changed = patch_show_file()
    touch_changed = patch_touch_file()
    print(f"backup={backup_dir}")
    print(f"14ShowFile.bin changed={show_changed}")
    print(f"13TouchFile.bin changed={touch_changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
