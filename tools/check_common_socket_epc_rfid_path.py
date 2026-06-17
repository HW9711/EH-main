#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查公共接头必须按协议走 EPC RFID 刀具读取。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_TXT = ROOT / "EIDE" / "刀具识别解析协议(1).txt"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"


def require(condition: bool, message: str) -> None:
    """断言协议和源码路径保持一致。"""
    if not condition:
        raise AssertionError(message)


def function_body(text: str, name: str) -> str:
    """按函数名粗略提取 C 函数体，便于做源码契约检查。"""
    marker = f"{name}("
    search_from = 0
    while True:
        pos = text.find(marker, search_from)
        if pos < 0:
            return ""
        brace = text.find("{", pos)
        if brace < 0:
            return ""
        semicolon = text.find(";", pos, brace)
        if semicolon < 0:
            break
        search_from = pos + len(marker)
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[brace:index + 1]
    return ""


def main() -> None:
    protocol = PROTOCOL_TXT.read_text(encoding="utf-8")
    handlescan = HANDLESCAN_C.read_text(encoding="utf-8")
    rfid = RFID_C.read_text(encoding="utf-8")
    request_body = function_body(rfid, "Rfid_RequestToolRead")
    request_compact = "".join(request_body.split())

    require("公共接头手柄按EPC模式" in protocol,
            "协议文件必须明确公共接头按 EPC 模式读取刀具信息")
    require("分体式手柄按USER模式" in protocol and "PXBA、PXBB" in protocol,
            "协议文件必须明确 PXBA/PXBB 分体式手柄按 USER 模式读取刀具信息")
    require("需要提取data[8-19]12个数据" in protocol,
            "协议文件必须定义 EPC 提取 data[8..19] 12 字节")

    require("{0x6B, 0x0D, COMMON_SOCKET_ONLINES" in handlescan,
            "0x6B/0x0D 必须映射为 COMMON_SOCKET_ONLINES 公共接头")
    require("mapped_handle_type == COMMON_SOCKET_ONLINES" in handlescan and
            "return HANDLESCAN_TOOL_SOURCE_RFID_EPC;" in handlescan,
            "公共接头必须在 handlescan 中选择 RFID_EPC 刀具来源")
    require("mapped_handle_type == PXBA_ONLINES" in handlescan and
            "mapped_handle_type == PXBB_ONLINES" in handlescan and
            "return HANDLESCAN_TOOL_SOURCE_RFID_USER;" in handlescan,
            "PXBA/PXBB 必须在 handlescan 中选择 RFID_USER 刀具来源")
    require("return RFID_READ_SOURCE_EPC;" in handlescan and
            "return RFID_READ_SOURCE_USER;" in handlescan,
            "handlescan 必须把 EPC/USER 刀具来源转换成 RFID 读取来源")

    require("Uart3_SendPacket(NO_MASK3_WRITE_EPC" in rfid,
            "EPC 来源必须发送 EPC 读取命令")
    require("Uart3_SendPacket(NO_MASK3_READ_USER" in rfid,
            "USER 来源必须发送 USER 读取命令")
    require("Rfid_GetPayloadOffset" in rfid and "RFID_PAYLOAD_EPC_OFFSET" in rfid,
            "RFID 解析必须按 EPC 偏移提取 payload")

    require("WorkMessage.hand_model!=PXBA_ONLINES" not in request_compact,
            "底层 RFID 请求不能再用 PXBA-only 条件拦截公共接头 EPC")
    require("WorkMessage.auto_identify==false||WorkMessage.hand_model" not in request_compact,
            "底层 RFID 请求不能依赖当前 WorkMessage，公共接头刚上线时当前通道可能尚未装载")
    require("Rfid_IsSourceValid(source)==false" in request_compact,
            "底层 RFID 请求仍必须校验 EPC/USER 来源是否合法")
    require("WorkMessage.runflag_work==true" in request_compact,
            "电机运行中仍禁止 RFID 读取，避免运行参数被新刀具改变")

    print("PASS: 公共接头 0x6B/0x0D 按协议走 EPC RFID 刀具读取")


if __name__ == "__main__":
    main()
