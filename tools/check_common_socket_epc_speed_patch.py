#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查公共接头 EPC 临时速度补丁是否完整落地。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
DRIVE_C = ROOT / "User" / "Application" / "Beep" / "sscDrive.c"


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释导致误判。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"无法识别文件编码: {path}")


def compact(text: str) -> str:
    """压缩空白后检查关键语句，避免格式调整造成误报。"""
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看到公共接头补丁缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """确认公共接头无刀具、EPC 成功和电机下发三段逻辑一致。"""
    handlescan = compact(read_text(HANDLESCAN_C))
    drive = compact(read_text(DRIVE_C))
    errors: list[str] = []

    require("#defineCOMMON_SOCKET_TEMP_SPEED_MIN20000U" in handlescan,
            "公共接头 EPC 最小速度必须固定为 20000。", errors)
    require("#defineCOMMON_SOCKET_TEMP_SPEED_MAX120000U" in handlescan,
            "公共接头 EPC 最大速度必须固定为 120000。", errors)
    require("#defineCOMMON_SOCKET_TEMP_SPEED_DEFAULT100000U" in handlescan,
            "公共接头 EPC 默认速度必须固定为 100000。", errors)
    require("#defineCOMMON_SOCKET_TEMP_SPEED_UP_RATIO2U" in handlescan,
            "公共接头 EPC 增速比必须固定为 2。", errors)
    require("message->speed_zzdefault=0U;" in handlescan,
            "公共接头无 RFID 刀具时正转默认速度必须为 0。", errors)
    require("message->speed_fzdefault=0U;" in handlescan,
            "公共接头无 RFID 刀具时反转默认速度必须为 0。", errors)
    require("message->speed_oscdefault=0U;" in handlescan,
            "公共接头无 RFID 刀具时往复默认速度必须为 0。", errors)
    require("max_speed=COMMON_SOCKET_TEMP_SPEED_MAX;" in handlescan,
            "公共接头 EPC 成功后最大速度必须固定为临时上限。", errors)
    require("min_speed=COMMON_SOCKET_TEMP_SPEED_MIN;" in handlescan,
            "公共接头 EPC 成功后最小速度必须固定为临时下限。", errors)
    require("default_speed=COMMON_SOCKET_TEMP_SPEED_DEFAULT;" in handlescan,
            "公共接头 EPC 成功后默认速度必须固定为临时默认值。", errors)
    require("reduction_ratio=((uint32_t)COMMON_SOCKET_TEMP_SPEED_UP_RATIO<<16);" in handlescan,
            "公共接头 EPC 成功后必须写入高 16 位增速比。", errors)
    require("MotorDrive_ApplyCommonSocketSpeedPatch" in drive,
            "电机下发侧缺少公共接头速度折算函数。", errors)
    require("WorkMessage.hand_model==COMMON_SOCKET_ONLINES" in drive,
            "电机下发侧必须只限定公共接头。", errors)
    require("speed_value/2U" in drive,
            "公共接头 2 倍增速时电机下发速度必须除以 2。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - common socket EPC temporary speed patch is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
