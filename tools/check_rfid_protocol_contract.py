#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 RFID 刀具识别协议和源码接入契约。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"
RFID_H = ROOT / "User" / "Application" / "include" / "sscRFID.h"
UART3_C = ROOT / "User" / "Peripheral" / "uart" / "uart3.c"
UART3_H = ROOT / "User" / "Peripheral" / "include" / "uart3.h"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
PUBINTERFACE_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
PUBINTERFACE_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
EXTERNAL_COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"
KEYBH_C = ROOT / "User" / "Application" / "Beep" / "sscKEYBH.c"
FOOT_C = ROOT / "User" / "Application" / "Beep" / "sscFOOT.c"


def parse_hex_frame(text: str) -> list[int]:
    """把协议示例字符串转成字节数组。"""
    return [int(item, 16) for item in text.split()]


def checksum_ok(frame: list[int]) -> bool:
    """校验 RFID 帧：除 BB 外到 checksum 前一字节累加，低 8 位等于 checksum。"""
    return (sum(frame[1:-2]) & 0xFF) == frame[-2]


def require(condition: bool, message: str) -> None:
    """契约断言失败时直接退出。"""
    if not condition:
        raise AssertionError(message)


def function_body(text: str, name: str) -> str:
    """Return a rough C function body by brace matching."""
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
    epc_frame = parse_hex_frame(
        "BB 02 22 00 11 C3 34 00 E2 00 47 0F 22 D0 60 13 04 35 E5 01 02 E5 CF 7E"
    )
    user_frame = parse_hex_frame(
        "BB 01 39 00 1F 0E 34 00 E2 00 42 12 02 30 60 12 01 CE AC C7 31 28 1A 00 01 31 00 05 0C 0A 28 00 00 00 06 EE 93 7E"
    )

    require(checksum_ok(epc_frame), "EPC 样例 checksum 应通过")
    require(checksum_ok(user_frame), "USER 样例 checksum 应通过")
    require(epc_frame[8:20] == [0xE2, 0x00, 0x47, 0x0F, 0x22, 0xD0, 0x60, 0x13, 0x04, 0x35, 0xE5, 0x01],
            "EPC 应提取 data[8..19] 共 12 字节")
    require(user_frame[20:36] == [0x31, 0x28, 0x1A, 0x00, 0x01, 0x31, 0x00, 0x05, 0x0C, 0x0A, 0x28, 0x00, 0x00, 0x00, 0x06, 0xEE],
            "USER 应提取 data[20..35] 共 16 字节")

    rfid_c = RFID_C.read_text(encoding="utf-8")
    rfid_compact = "".join(rfid_c.split())
    rfid_update_cache = function_body(rfid_c, "Rfid_UpdateParsedCache")
    rfid_clear_channel = function_body(rfid_c, "Rfid_ClearChannelResult")
    rfid_h = RFID_H.read_text(encoding="utf-8")
    uart3_c = UART3_C.read_text(encoding="utf-8")
    uart3_h = UART3_H.read_text(encoding="utf-8")
    handlescan_c = HANDLESCAN_C.read_text(encoding="utf-8")
    handlescan_compact = "".join(handlescan_c.split())
    rfid_wait_body = function_body(handlescan_c, "Handlescan_ProcessRfidWait")
    rfid_wait_compact = "".join(rfid_wait_body.split())
    online_rfid_body = function_body(handlescan_c, "Handlescan_ProcessOnlineRfidResult")
    online_rfid_compact = "".join(online_rfid_body.split())
    clear_online_rfid_body = function_body(handlescan_c, "Handlescan_ClearOnlineRfidTool")
    clear_online_rfid_compact = "".join(clear_online_rfid_body.split())
    online_reinsert_branch = online_rfid_compact[
        online_rfid_compact.find("if(*tool_online==0U)"):
        online_rfid_compact.find("elseif(result.cache_hit==false)")
    ]
    clear_already_offline_branch = clear_online_rfid_compact[
        clear_online_rfid_compact.find("if(*tool_online==0U)"):
        clear_online_rfid_compact.find("Handlescan_PrepareRfidBaseRecognizeMessage")
    ]
    pubinterface_c = PUBINTERFACE_C.read_text(encoding="utf-8")
    pubinterface_clear_rfid = function_body(pubinterface_c, "Pubinterface_ClearRfidToolMemory")
    pubinterface_h = PUBINTERFACE_H.read_text(encoding="utf-8")
    external_comm_c = EXTERNAL_COMM_C.read_text(encoding="utf-8")
    external_rfid_fallback = function_body(external_comm_c, "ExternalComm_HeartbeatGetRfidFallback")
    keybh_c = KEYBH_C.read_text(encoding="utf-8")
    foot_c = FOOT_C.read_text(encoding="utf-8")

    require("RFID_PAYLOAD_EPC_LENGTH" in rfid_h and "RFID_PAYLOAD_USER_LENGTH" in rfid_h,
            "sscRFID.h 应公开 EPC/USER 标签数据长度")
    require("Rfid_ParseReceivedFrame" in rfid_h and "Rfid_RequestToolRead" in rfid_h,
            "sscRFID.h 应公开解析结果读取和 RFID 请求接口")
    require("Rfid_CopyLastResult" in rfid_h,
            "sscRFID.h 应公开结果拷贝接口，供 handlescan 在安全边界内取结果")
    require("RFID_READ_SOURCE_EPC" in rfid_c and "RFID_READ_SOURCE_USER" in rfid_c,
            "sscRFID.c 应区分 EPC 和 USER 读取来源")
    require("RFID_PAYLOAD_EPC_OFFSET" in rfid_c and "RFID_PAYLOAD_USER_OFFSET" in rfid_c,
            "sscRFID.c 应按协议偏移提取 EPC/USER 原始标签数据")
    require("Rfid_UpdateParsedCache" in rfid_c,
            "sscRFID.c 应按完整原始标签数据维护缓存")
    require("RfidPayloadMemory_t" in rfid_c and "s_payload_memory" in rfid_c,
            "RFID must keep remembered payload separate from the currently reportable result")
    require("payload_memory->valid" in rfid_c and "memcmp(payload_memory->payload" in rfid_c,
            "same-tool detection must compare against remembered payload even after current result is cleared")
    require("memset(&s_last_result[index], 0, sizeof(s_last_result[index]));" in rfid_clear_channel and
            "s_payload_memory" not in rfid_clear_channel,
            "clearing current RFID result must not erase remembered payload used for no-repeat beep detection")
    require("result->sequence = last_result->sequence" in rfid_c and "same_payload != false" in rfid_c,
            "same RFID tag payload must not create a new sequence")
    require("if (same_payload != false)" in rfid_update_cache and
            "return false; /* 普通在线监测不刷新缓存，防止空闲周期重复刷新刀具信息。 */" in rfid_update_cache,
            "same RFID tag payload in normal online monitoring must not publish a new business result")
    require("presence_sequence" in rfid_h and "s_presence_sequence" in rfid_c,
            "RFID cache must expose a presence sequence that advances on every valid tag read")
    require("last_result->presence_sequence = result->presence_sequence" in rfid_c,
            "same RFID tag payload must still refresh presence sequence for online absence detection")
    require("s_request_fast_mode" in rfid_c and "s_request_fast_mode != false" in rfid_c,
            "fast RFID recognition must publish a consumable sequence even when the same tag is read again")
    require("Rfid_RequestToolRead(CHANNEL_A, Rfid_LegacyTypeToSource(rfid_data), false)" in rfid_c and
            "Rfid_RequestToolRead(CHANNEL_B, Rfid_LegacyTypeToSource(rfid_data), false)" in rfid_c,
            "legacy RFID screen/manual entries must use normal mode so the same tool payload does not republish as a changed tool")
    require("#define RFID_DEBUG_BEEP_EVERY_VALID_READ 0U" in rfid_c,
            "RFID valid-frame debug beep must be disabled after field diagnosis")
    require("#define RFID_DEBUG_BEEP_EVERY_UART_RESPONSE 0U" in rfid_c,
            "RFID UART-response debug beep must be disabled so unchanged tool tags do not keep beeping")
    require("void Uart3_ClearRecvData(void)" in uart3_h and
            "void Uart3_ClearRecvData(void)" in uart3_c,
            "uart3 must expose an RX clear API so a new RFID request cannot consume a stale frame")
    require("Uart3_ClearRecvData();" in rfid_c and
            rfid_c.find("Uart3_ClearRecvData();") < rfid_c.find("s_request_active = true;"),
            "RFID request start must clear UART3 RX before becoming active")
    require("s_request_active" in rfid_clear_channel and
            "s_request_channel == channel" in rfid_clear_channel and
            "Uart3_ClearRecvData();" in rfid_clear_channel,
            "clearing RFID channel result must cancel same-channel active request and drop late UART3 frames")
    require("Rfid_DiscardQueuedMessagesForChannel(channel);" in rfid_clear_channel and
            "static void Rfid_DiscardQueuedMessagesForChannel(uint8_t channel)" in rfid_c and
            "RFID_QUEUE_LENGTH" in function_body(rfid_c, "Rfid_DiscardQueuedMessagesForChannel"),
            "clearing RFID channel result must drop same-channel queued requests before they can restart stale reads")
    require("uint16_t generation;" in rfid_c and
            "s_request_generation" in rfid_c and
            "msg.generation = s_request_generation[index];" in rfid_c,
            "RFID read requests must carry a per-channel generation to reject queued stale reads")
    require("s_request_generation[index]++;" in rfid_clear_channel and
            "msg.generation != s_request_generation[index]" in rfid_c,
            "clearing RFID channel result must invalidate older queued requests by generation")
    require("Rfid_UpdateParsedCache(&parsed_result)==true)&&(parsed_result.cache_hit==false)){SendKeyBeepMessage(1U);" not in rfid_compact,
            "RFID task must not beep directly after cache update; handlescan beeps only after business state changes")
    require("Handlescan_GetToolSource" in handlescan_c,
            "handlescan.c 应按 EEPROM 第二页判断刀具信息来源")
    require("HANDLESCAN_STAGE_WAIT_RFID_TOOL" in handlescan_c,
            "handlescan.c 应有等待 RFID 刀具头结果的阶段")
    require("Handlescan_PrepareRfidBaseRecognizeMessage" in handlescan_c and
            "SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A)" in handlescan_c and
            "SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_B)" in handlescan_c,
            "PXBA/PXBB base handle must be reported online separately from RFID tool parsing")
    require("uint16_t*last_sequence" in handlescan_compact and
            "*last_sequence=0U;" in handlescan_compact and
            "Rfid_ClearChannelResult(channel);" in handlescan_c,
            "new WAIT_RFID_TOOL round must clear stale RFID result cache and consumed sequence")
    require("Handlescan_StartRfidWait(CHANNEL_A,&s_a_stage,s_a_tool_source,&s_a_rfid_last_sequence,&s_a_rfid_wait_ticks,true)" in handlescan_compact and
            "Handlescan_StartRfidWait(CHANNEL_B,&s_b_stage,s_b_tool_source,&s_b_rfid_last_sequence,&s_b_rfid_wait_ticks,true)" in handlescan_compact,
            "A/B RFID wait calls must pass their consumed sequence guard into the reset boundary")
    require("Handlescan_BeepOnceIfNoAlarm();" in rfid_wait_compact and
            "if(result.cache_hit==false){Handlescan_BeepOnceIfNoAlarm();" not in rfid_wait_compact,
            "initial RFID online/reinsert feedback must beep once even when payload is a cache hit")
    require("s_a_rfid_tool_online" in handlescan_c and "s_b_rfid_tool_online" in handlescan_c,
            "handlescan must keep a per-channel RFID tool online edge flag")
    require("uint8_t*tool_online" in handlescan_compact,
            "RFID wait/monitor/clear paths must receive the tool-online edge flag")
    require("*tool_online=1U;" in rfid_wait_compact,
            "initial RFID wait success must mark the tool head online")
    require("if(*tool_online==0U)" in online_rfid_compact and
            "*tool_online=1U;" in online_reinsert_branch and
            "Handlescan_BeepOnceIfNoAlarm();" in online_reinsert_branch,
            "online RFID monitor must beep when a previously-offline tool head comes back")
    require("elseif(result.cache_hit==false){Handlescan_BeepOnceIfNoAlarm();" in online_rfid_compact,
            "online RFID monitor must still beep when an already-online tool payload changes")
    require("if(*tool_online==0U)" in clear_online_rfid_compact and
            "*miss_count=0U;" in clear_already_offline_branch and
            "*monitor_pending=0U;" in clear_already_offline_branch and
            "return;" in clear_already_offline_branch and
            "Handlescan_BeepOnceIfNoAlarm();" not in clear_already_offline_branch and
            "SendKeyBehMessage(PLUGunPLUG,plug_key);" not in clear_already_offline_branch,
            "already-offline RFID tool head must not repeat offline clear/beep/event")
    require("*tool_online=0U;" in clear_online_rfid_compact and
            clear_online_rfid_compact.find("*tool_online=0U;") < clear_online_rfid_compact.find("Handlescan_BeepOnceIfNoAlarm();"),
            "RFID tool-head offline clear must mark offline before one-shot beep")
    require("Handlescan_BeepOnceIfNoAlarm();" in clear_online_rfid_compact,
            "RFID tool-head offline clear must beep once")
    require("HANDLESCAN_RFID_MISS_MAX" in handlescan_c and
            "s_a_rfid_miss_count" in handlescan_c and
            "s_b_rfid_miss_count" in handlescan_c,
            "online RFID monitoring must count consecutive missed tag reads")
    require("#define HANDLESCAN_RFID_WAIT_TIMEOUT_MS       900U" in handlescan_c,
            "initial RFID tool wait must use 900ms per round")
    require("#define HANDLESCAN_RFID_VERIFY_RETRY_MAX      2U" in handlescan_c,
            "initial RFID tool wait must be limited to two rounds, about 2 seconds including retry delay")
    require("Handlescan_EnterRfidRetryOrFail(channel," in handlescan_c,
            "WAIT_RFID_TOOL timeout must use the RFID-specific retry limit instead of the generic EEPROM retry limit")
    require("#define HANDLESCAN_RFID_MONITOR_PERIOD_MS     1000U" in handlescan_c,
            "online RFID monitor period must be 1s so request+miss clear is about 2 seconds")
    require("#define HANDLESCAN_RFID_MISS_MAX              1U" in handlescan_c,
            "online RFID tool absence must clear after one missed monitor confirmation")
    require("Handlescan_ClearOnlineRfidTool" in handlescan_c and
            "Rfid_ClearChannelResult(channel)" in handlescan_c and
            "Handlescan_PrepareRfidBaseRecognizeMessage" in handlescan_c,
            "online RFID misses must clear only the detachable tool head while keeping the base online")
    require("Pubinterface_ClearRfidToolMemory" in pubinterface_h and
            "Pubinterface_ClearRfidToolMemory" in pubinterface_c and
            "Pubinterface_ClearRfidToolMemory(channel);" in handlescan_c,
            "online RFID miss clear must directly clear MemoryMsg/WorkMessage tool fields before heartbeat can resend stale tool info")
    require("recognize->tool_type = 0U;" in pubinterface_clear_rfid and
            "recognize->tool_reduction_ratio = 0U;" in pubinterface_clear_rfid and
            "memset(memory, 0, sizeof(*memory));" in pubinterface_clear_rfid,
            "Pubinterface_ClearRfidToolMemory must clear scan tool fields and channel memory")
    require("memory->hand_model = recognize->handle_type;" in pubinterface_clear_rfid and
            "memory->hand_type_raw_major = recognize->hand_type_raw_major;" in pubinterface_clear_rfid and
            "memory->hand_type_raw_minor = recognize->hand_type_raw_minor;" in pubinterface_clear_rfid,
            "Pubinterface_ClearRfidToolMemory must keep the RFID base handle online while clearing only the tool head")
    require("WorkMessage.tool_type = 0U;" in pubinterface_clear_rfid and
            "WorkMessage.tool_reduction_ratio = 0U;" in pubinterface_clear_rfid and
            "WorkMessage.speed_set_work = 0U;" in pubinterface_clear_rfid,
            "Pubinterface_ClearRfidToolMemory must clear selected WorkMessage tool parameters")
    require("Pubinterface_RefreshSelectedChannelDisplay(channel);" in pubinterface_clear_rfid and
            "Pubinterface_RefreshOnlineHandleDisplay();" in pubinterface_clear_rfid,
            "Pubinterface_ClearRfidToolMemory must refresh selected display and keep online handle display coherent")
    require("WorkMessage.runflag_work" in handlescan_c and "Rfid_RequestToolRead" in handlescan_c,
            "handlescan.c 发起 RFID 前应检查电机运行状态")

    require("SendKeyRFIDMessage" not in foot_c and '#include "sscRFID.h"' not in foot_c,
            "foot pedal task must not enqueue RFID reads; RFID polling belongs to handlescan online monitoring")
    require("message->tool_reduction_ratio" in handlescan_c and "recognize->tool_reduction_ratio" in pubinterface_c,
            "RFID EPC complete reduction ratio must survive recognize-to-memory copy")
    require("EXTERNAL_COMM_HEARTBEAT_TOOL_EXT_MAGIC" in external_comm_c and "0xA5U" in external_comm_c,
            "external_comm_task.c must define heartbeat tool extension magic")
    require("#define EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS     2000U" in external_comm_c,
            "external comm release/display timeout must be 2 seconds")
    require("EXTERNAL_COMM_HEARTBEAT_TOOL_EXT_BLOCK_LEN" in external_comm_c and "20U" in external_comm_c,
            "external_comm_task.c must keep heartbeat tool block length at 20 bytes")
    require("ExternalComm_HeartbeatAppendToolInfo" in external_comm_c,
            "external_comm_task.c must append RFID/EEPROM tool info to heartbeat")
    require("is_rfid_source" in external_comm_c and "PXBA_ONLINES" in external_comm_c and "PXBB_ONLINES" in external_comm_c,
            "RFID tool heartbeat validity must tolerate zero dimension fields for PXBA/PXBB")
    require("return (recognize->tool_type != 0U)" in external_comm_c,
            "heartbeat must not send an all-zero RFID tool block before the tag payload is parsed")
    require("ExternalComm_HeartbeatAppendDWordBE" in external_comm_c,
            "external_comm_task.c must send reductionRatio as 4-byte big-endian")
    require("ExternalComm_HeartbeatResolveHandleModel" in external_comm_c and
            "ChannelrecognizeMessageA" in external_comm_c and
            "ChannelrecognizeMessageB" in external_comm_c,
            "heartbeat must fall back to scan recognize cache while plug event is still queued")
    require("ExternalComm_HeartbeatResolveOnline" in external_comm_c and
            "ExternalComm_HeartbeatResolveRawMajor" in external_comm_c and
            "ExternalComm_HeartbeatResolveRawMinor" in external_comm_c,
            "heartbeat must report recognized PXBA/PXBB base online before MemoryMsg is loaded")
    require('#include "sscRFID.h"' in external_comm_c and
            "Rfid_CopyLastResult" in external_comm_c and
            "ExternalComm_HeartbeatAppendRfidResultBlock" in external_comm_c,
            "heartbeat must fall back to the latest parsed RFID result when scan consumption is delayed")
    require("recognize->tool_type == 0U" in external_rfid_fallback and
            external_rfid_fallback.find("recognize->tool_type == 0U") < external_rfid_fallback.find("Rfid_CopyLastResult"),
            "heartbeat RFID fallback must not revive a cleared tool head after scan cache tool_type is zero")
    require("KEYBEH_PLUG_EVENT_WAIT_TICKS" in keybh_c and
            "control_type == PLUGunPLUG" in keybh_c,
            "PLUG/RFID refresh events must not be dropped by zero-tick key queue send")
    require("while(Kernel_QueueReceive(KeyBehivQueue,&msg,0)==pdTRUE)" in "".join(keybh_c.split()) and
            "continue;" in keybh_c,
            "key behavior task must drain queued messages and continue after blocked local keys")

    print("RFID protocol contract OK")


if __name__ == "__main__":
    main()
