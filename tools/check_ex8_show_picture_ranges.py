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
SHOW_BIN = SCREEN_ROOT / "DWIN_SET" / "14ShowFile.bin"
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"cannot decode {path}")


def read_u16_be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def read_u16_le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def load_picture_ranges(show_data: bytes) -> dict[int, tuple[int, int]]:
    ranges: dict[int, tuple[int, int]] = {}
    for page in range(64):
        entry = 0x10 + page * 4
        if entry + 4 > len(show_data):
            break
        count = read_u16_le(show_data, entry)
        offset = read_u16_be(show_data, entry + 2)
        if count == 0:
            continue
        if offset == 0 or offset + count * 32 > len(show_data):
            continue
        for index in range(count):
            record = show_data[offset + index * 32: offset + (index + 1) * 32]
            if len(record) < 32:
                continue
            if record[0] != 0x5A or record[1] != 0x00:
                continue
            if read_u16_be(record, 4) != 0x000A:
                continue
            vp = read_u16_be(record, 6)
            low = read_u16_be(record, 12)
            high = read_u16_be(record, 14)
            if low > high:
                low, high = high, low
            if vp in ranges:
                old_low, old_high = ranges[vp]
                ranges[vp] = (min(old_low, low), max(old_high, high))
            else:
                ranges[vp] = (low, high)
    return ranges


def parse_macros(text: str) -> dict[str, int]:
    macros: dict[str, int] = {}
    for match in re.finditer(r"#define\s+(UIDP_LCD_VP_[A-Za-z0-9_]+)\s+((?:0x)?[0-9A-Fa-f]+)U?", text):
        name = match.group(1)
        value_text = match.group(2)
        macros[name] = int(value_text, 16 if value_text.lower().startswith("0x") else 10)
    return macros


def parse_constant_picture_writes(uidp_text: str) -> list[tuple[int, str, str, int, int]]:
    writes: list[tuple[int, str, str, int, int]] = []
    pattern = re.compile(
        r"LCD_Show_Picture\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*|0x[0-9A-Fa-f]+|\d+U?)"
        r"\s*,\s*"
        r"(\d+)U?"
        r"\s*\)"
    )
    for match in pattern.finditer(uidp_text):
        line = uidp_text.count("\n", 0, match.start()) + 1
        vp_expr = match.group(1)
        picture = int(match.group(2))
        writes.append((line, match.group(0), vp_expr, picture, match.start()))
    return writes


def resolve_vp(vp_expr: str, macros: dict[str, int]) -> int | None:
    if vp_expr in macros:
        return macros[vp_expr]
    cleaned = vp_expr.rstrip("U")
    if cleaned.lower().startswith("0x"):
        return int(cleaned, 16)
    if cleaned.isdigit():
        return int(cleaned)
    return None


def main() -> int:
    errors: list[str] = []
    if not SHOW_BIN.exists():
        errors.append(f"missing EX8 show file: {SHOW_BIN}")
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    ranges = load_picture_ranges(SHOW_BIN.read_bytes())
    macros = parse_macros(read_text(SCREEN_ADDRESS_H))
    uidp = read_text(UIDP_C)
    writes = parse_constant_picture_writes(uidp)

    for line, call_text, vp_expr, picture, _ in writes:
        vp = resolve_vp(vp_expr, macros)
        if vp is None:
            errors.append(f"line {line}: cannot resolve VP expression in {call_text}")
            continue
        if vp not in ranges:
            errors.append(f"line {line}: VP 0x{vp:04X} from {vp_expr} is not a picture control in current EX8 14ShowFile.bin")
            continue
        low, high = ranges[vp]
        if not (low <= picture <= high):
            errors.append(
                f"line {line}: picture {picture} written to VP 0x{vp:04X}, "
                f"but current EX8 range is {low}-{high}"
            )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print(f"ok - EX8 LCD_Show_Picture constants fit {len(ranges)} exported picture control ranges")
    return 0


if __name__ == "__main__":
    sys.exit(main())
