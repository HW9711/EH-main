from __future__ import annotations

import sys
from pathlib import Path


SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart")
    / "DL-EX8\u4e2d\u6587 \u62bd\u54380609"
    / "DL-EX8\u4e2d\u6587 \u62bd\u5438"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def read_u16_be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def read_four_u16(data: bytes, start: int) -> tuple[int, int, int, int]:
    return (
        read_u16_be(data, start),
        read_u16_be(data, start + 2),
        read_u16_be(data, start + 4),
        read_u16_be(data, start + 6),
    )


def show_page_records(data: bytes, page: int) -> list[bytes]:
    entry = 0x10 + page * 4
    count = data[entry] | (data[entry + 1] << 8)
    offset = read_u16_be(data, entry + 2)
    return [data[offset + index * 32 : offset + (index + 1) * 32] for index in range(count)]


def show_record_vp(record: bytes) -> int:
    return read_u16_be(record, 6)


def show_record_xy(record: bytes) -> tuple[int, int]:
    return read_u16_be(record, 8), read_u16_be(record, 10)


def show_record_icon_range(record: bytes) -> tuple[int, int, int, int]:
    return read_four_u16(record, 12)


def touch_record_page(record: bytes) -> int:
    return read_u16_be(record, 0)


def touch_record_rect(record: bytes) -> tuple[int, int, int, int]:
    return read_four_u16(record, 2)


def touch_record_vp(record: bytes) -> int:
    return read_u16_be(record, 17)


def touch_record_key(record: bytes) -> int:
    return read_u16_be(record, 20)


def rects_overlap(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> bool:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return (ax1 < bx2) and (bx1 < ax2) and (ay1 < by2) and (by1 < ay2)


def main() -> int:
    errors: list[str] = []
    require(SHOW_BIN.exists(), f"missing {SHOW_BIN}", errors)
    require(TOUCH_BIN.exists(), f"missing {TOUCH_BIN}", errors)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    show_data = SHOW_BIN.read_bytes()
    touch_data = TOUCH_BIN.read_bytes()
    page4_show = {show_record_vp(record): record for record in show_page_records(show_data, 4)}

    b_add = page4_show.get(0x1422)
    b_sub = page4_show.get(0x1424)
    external = page4_show.get(0x1416)

    require(b_add is not None, "page4 show file is missing B pump add VP 0x1422.", errors)
    require(b_sub is not None, "page4 show file is missing B pump sub VP 0x1424.", errors)
    require(external is not None, "page4 show file is missing external communication VP 0x1416.", errors)

    if b_add is not None:
        require(
            show_record_icon_range(b_add) == (498, 499, 498, 499),
            "B pump add display VP 0x1422 must be bound to icon resources 498/499.",
            errors,
        )
    if b_sub is not None:
        require(
            show_record_icon_range(b_sub) == (500, 501, 500, 501),
            "B pump sub display VP 0x1424 must be bound to icon resources 500/501.",
            errors,
        )
    if external is not None:
        require(
            show_record_xy(external) == (1130, 2),
            "External communication display VP 0x1416 must stay at x=1130,y=2 to avoid the touch icon.",
            errors,
        )

    record_count = (len(touch_data) - 2) // 32
    touch_records = [touch_data[index * 32 : (index + 1) * 32] for index in range(record_count)]
    external_touch = None
    touch_control_touch = None
    for record in touch_records:
        if touch_record_page(record) != 4 or touch_record_vp(record) != 0x2404:
            continue
        key = touch_record_key(record)
        if key == 4:
            external_touch = record
        elif key == 3:
            touch_control_touch = record

    require(external_touch is not None, "page4 touch file is missing external communication key 0x2404/key4.", errors)
    require(touch_control_touch is not None, "page4 touch file is missing touch-control key 0x2404/key3.", errors)
    if external_touch is not None:
        require(
            touch_record_rect(external_touch) == (1157, 0, 1266, 41),
            "External communication touch key must follow the top-right EX8 icon rectangle.",
            errors,
        )
    if external_touch is not None and touch_control_touch is not None:
        require(
            rects_overlap(touch_record_rect(external_touch), touch_record_rect(touch_control_touch)) is False,
            "External communication touch key must not overlap the touch-control key.",
            errors,
        )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - new screen display geometry checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
