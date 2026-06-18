#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 EMBD 手柄电机方向是否只在最终驱动下发前取反。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
DRIVE_C = ROOT / "User" / "Application" / "Beep" / "sscDrive.c"


def read_text(path: Path) -> str:
    """按常见编码读取源码，避免历史中文编码影响静态检查。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"无法识别文件编码: {path}")


def compact(text: str) -> str:
    """压缩空白后匹配关键逻辑，避免格式调整导致误报。"""
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看清缺失逻辑。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """确认 EMBD 只翻转正反转方向，不影响其它手柄或往复模式。"""
    drive = compact(read_text(DRIVE_C))
    errors: list[str] = []

    require("effective_dir_work=(uint8_t)WorkMessage.dir_work;" in drive,
            "MOTORRUN 必须先复制当前方向，避免直接改写 WorkMessage/UI 记忆。", errors)
    require("WorkMessage.hand_model==EMBD_ONLINES" in drive,
            "EMBD 方向补丁必须限定 hand_model == EMBD_ONLINES。", errors)
    require("effective_dir_work==ZZDIR" in drive and "effective_dir_work=FZDIR;" in drive,
            "EMBD 正转下发前必须翻成反转。", errors)
    require("effective_dir_work==FZDIR" in drive and "effective_dir_work=ZZDIR;" in drive,
            "EMBD 反转下发前必须翻成正转。", errors)
    require("switch(effective_dir_work)" in drive,
            "电机 control_mode 必须使用取反后的有效方向。", errors)
    require("WorkMessage.dir_work=FZDIR" not in drive and "WorkMessage.dir_work=ZZDIR" not in drive,
            "方向补丁不应直接改写 WorkMessage.dir_work，避免影响屏幕和通道记忆。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - EMBD motor direction is inverted only at drive output")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
