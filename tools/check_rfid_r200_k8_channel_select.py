#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 RFID 读取前必须按业务通道切换 R200-K8 模拟开关。"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
BOARD_H = ROOT / "User" / "board" / "board.h"
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释影响检查。"""
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"无法识别文件编码：{path}")


def function_body(text: str, name: str) -> str:
    """提取指定 C 函数体，避免检查脚本误匹配注释或声明。"""
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


def compact(text: str) -> str:
    """压缩空白后做结构检查，避免格式调整造成误报。"""
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    """收集所有失败项，便于一次看到通道选择链路缺口。"""
    if not condition:
        errors.append(message)


def main() -> int:
    """R200-K8 高电平必须对应 A，低电平必须对应 B，且只在允许 RFID 时切换。"""
    board_h = compact(read_text(BOARD_H))
    rfid_c = read_text(RFID_C)
    rfid_all = compact(rfid_c)
    request_body = compact(function_body(rfid_c, "Rfid_RequestToolRead"))
    receive_body = compact(function_body(rfid_c, "Rfid_ReceiveRequestMessage"))
    task_body = compact(function_body(rfid_c, "SplitType_AutoModeGetData_Task"))
    errors: list[str] = []

    require(
        "#defineR200_K8_SELECT_A()(R200_K8_Port->BSRR=R200_K8_Pin)" in board_h,
        "board.h 必须提供语义宏 R200_K8_SELECT_A()，A 通道为高电平。",
        errors,
    )
    require(
        "#defineR200_K8_SELECT_B()(R200_K8_Port->BSRR=((uint32_t)R200_K8_Pin<<16U))" in board_h,
        "board.h 必须提供语义宏 R200_K8_SELECT_B()，B 通道为低电平。",
        errors,
    )
    require(
        '#include"board.h"' in rfid_all,
        "sscRFID.c 必须包含 board.h，才能直接控制 R200-K8 模拟开关。",
        errors,
    )
    require(
        "staticvoidRfid_SelectHardwareChannel(uint8_tchannel)" in rfid_all and
        "R200_K8_SELECT_A();" in rfid_all and
        "R200_K8_SELECT_B();" in rfid_all,
        "sscRFID.c 必须封装 Rfid_SelectHardwareChannel()，按请求通道切 A/B。",
        errors,
    )
    require(
        "staticboolRfid_IsRequestAllowedForCurrentSelection(uint8_tchannel)" in rfid_all,
        "sscRFID.c 必须封装 RFID 读取 gate，统一判断单手柄、双手柄和当前选中通道。",
        errors,
    )
    require(
        "if(WorkMessage.runflag_work==true)" in rfid_all and
        "returnfalse;/*电机运行中不允许" in rfid_all,
        "RFID 读取 gate 必须运行中直接拒绝，避免运行参数被识别刷新。",
        errors,
    )
    require(
        "requested_channel_online" in rfid_all and
        "WorkMessage.Channel_Aonline" in rfid_all and
        "WorkMessage.Channel_Bonline" in rfid_all,
        "RFID 读取 gate 必须区分请求通道是否已经在线，保证后插入 RFID 通道上线前可以读取。",
        errors,
    )
    require(
        "if((WorkMessage.Channel_Aonline==true)&&(WorkMessage.Channel_Bonline==true))" in rfid_all and
        "return(WorkMessage.channel_work==channel);" in rfid_all,
        "双手柄都在线后，只允许当前选中通道继续做 RFID 读取。",
        errors,
    )
    require(
        "Rfid_IsRequestAllowedForCurrentSelection(channel)==false" in request_body,
        "Rfid_RequestToolRead() 入队前必须按当前通道规则拦截非法请求。",
        errors,
    )
    require(
        "Rfid_IsRequestAllowedForCurrentSelection(msg.channel)==false" in receive_body and
        "Rfid_SelectHardwareChannel(msg.channel);" in receive_body,
        "RFID 请求出队执行前必须重新校验并切换 R200-K8 到请求通道。",
        errors,
    )
    require(
        "Rfid_IsRequestAllowedForCurrentSelection(s_request_channel)==false" in task_body and
        "Rfid_SelectHardwareChannel(s_request_channel);" in task_body and
        "Rfid_SendReadCommand(s_request_source);" in task_body,
        "RFID 发送读命令前必须复查当前选择，并再次确保模拟开关在目标通道。",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: R200-K8 按 RFID 请求通道切换，运行中和非当前通道读取被拦截")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
