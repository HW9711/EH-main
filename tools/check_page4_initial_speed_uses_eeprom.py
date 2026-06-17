#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 Page4 默认速度和最小速度必须使用 EEPROM 解析值。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释导致误判。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"无法识别文件编码: {path}")


def function_body(text: str, name: str) -> str:
    """提取指定函数体，避免匹配到注释或其它函数。"""
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"找不到函数: {name}")
    depth = 0
    for index in range(match.end() - 1, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[match.end():index]
    raise AssertionError(f"函数括号未闭合: {name}")


def compact(text: str) -> str:
    """压缩空白后检查关键赋值，避免格式调整造成误报。"""
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看到 Page4 速度装载缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """确认 Page4 解析出的速度真正写入通道识别缓存。"""
    body = compact(function_body(read_text(HANDLESCAN_C), "Handlescan_UpdateInitialInfoMessage"))
    errors: list[str] = []

    for field in ("speed_min", "speed_zzmin", "speed_fzmin", "speed_oscmin"):
        require(f"message->{field}=min_speed;" in body,
                f"{field} 必须来自 Page4 min_speed，不能写死。", errors)

    for field in ("speed_zzdefault", "speed_fzdefault", "speed_oscdefault"):
        require(f"message->{field}=default_speed;" in body,
                f"{field} 必须来自 Page4 default_speed，不能写死 60000。", errors)
        require(f"message->{field}=60000;" not in body,
                f"{field} 不能硬编码为 60000。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - Page4 initial speed uses EEPROM values")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
