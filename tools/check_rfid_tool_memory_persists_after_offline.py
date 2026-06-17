from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释影响检查结果。"""
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"无法识别文件编码：{path}")


def function_body(text: str, name: str) -> str:
    """提取指定 C 函数体，让检查只覆盖 RFID 离线清理路径。"""
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"找不到函数：{name}")
    depth = 0
    for index in range(match.end() - 1, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[match.end():index]
    raise AssertionError(f"函数括号不完整：{name}")


def require(condition: bool, message: str, errors: list[str]) -> None:
    """集中收集失败项，便于一次看到 RFID 记忆保留规则是否完整。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """检查 RFID 刀具头掉线后，切换通道再切回仍能使用上次识别参数。"""
    pub = read_text(ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c")
    handlescan = read_text(ROOT / "User" / "Application" / "Handle" / "handlescan.c")
    clear_body = function_body(pub, "Pubinterface_ClearRfidToolMemory")
    load_body = function_body(pub, "Pubinterface_LoadChannelMemory")
    offline_body = function_body(handlescan, "Handlescan_ClearOnlineRfidTool")
    clear_compact = re.sub(r"\s+", "", clear_body)
    load_compact = re.sub(r"\s+", "", load_body)
    offline_compact = re.sub(r"\s+", "", offline_body)
    errors: list[str] = []

    require(
        "Pubinterface_ClearRfidToolMemory(channel);" in offline_compact,
        "RFID 离线路径必须进入 Pubinterface_ClearRfidToolMemory，才能统一处理显示和通道记忆。",
        errors,
    )
    require(
        "memset(memory,0,sizeof(*memory));" not in clear_compact,
        "RFID 刀具头离线不能清空 MemoryMsgA/B，否则切换通道再切回速度会变成 0。",
        errors,
    )
    for field in (
        "memory->tool_type",
        "memory->raw_tool_type",
        "memory->tool_reduction_ratio",
        "memory->current_work",
        "memory->zz_speed",
        "memory->fz_speed",
        "memory->osc_speed",
        "memory->freq",
        "memory->dir",
    ):
        require(
            f"{field}=0U" not in clear_compact and f"{field}=0" not in clear_compact,
            f"RFID 离线清理不能把 {field} 清零，通道切回还要使用上次识别参数。",
            errors,
        )
    require(
        "recognize->tool_type=0U;" in clear_compact and
        "recognize->speed_zzdefault=0U;" in clear_compact,
        "扫描层仍需要清掉当前在线刀具字段，让上位机能知道刀具头已离线。",
        errors,
    )
    require(
        "memory->auto_identify=keep_auto_identify;" in clear_compact and
        "memory->drive_type=keep_drive_type;" in clear_compact,
        "RFID 离线时仍要保留通道自动识别模式和控制方式记忆。",
        errors,
    )
    require(
        "WorkMessage.speed_set_work=memory->zz_speed;" in load_compact and
        "WorkMessage.speed_set_work=memory->fz_speed;" in load_compact and
        "WorkMessage.speed_set_work=memory->osc_speed;" in load_compact,
        "切回通道时速度来源仍必须是 MemoryMsgA/B，所以离线清理不能清通道记忆速度。",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: RFID 离线后通道记忆保留，切回通道不会把速度恢复为 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
