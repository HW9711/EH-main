from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"
KEYBH_C = ROOT / "User" / "Application" / "Beep" / "sscKEYBH.c"
EXTERNAL_COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"cannot decode {path}")


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def function_body(text: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*(?://[^\n]*\n\s*)?\{{", text)
    if not match:
        raise AssertionError(f"missing function: {name}")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated function: {name}")


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def optional_function_body(text: str, name: str, errors: list[str]) -> str:
    try:
        return function_body(text, name)
    except AssertionError:
        errors.append(f"missing function: {name}")
        return ""


def require_case_icon_pair(body: str, case_label: str, a_selected: str, a_connected: str,
                           b_selected: str, b_connected: str, message: str,
                           errors: list[str]) -> None:
    case_blocks = re.findall(rf"{re.escape(case_label)}.*?break;", body)
    require_case_icon_pair_in_blocks(case_blocks, a_selected, a_connected,
                                     b_selected, b_connected, message, errors)


def require_case_icon_pair_any(body: str, case_labels: tuple[str, ...],
                               a_selected: str, a_connected: str,
                               b_selected: str, b_connected: str,
                               message: str, errors: list[str]) -> None:
    case_blocks: list[str] = []
    for case_label in case_labels:
        case_blocks.extend(re.findall(rf"{re.escape(case_label)}.*?break;", body))
    require_case_icon_pair_in_blocks(case_blocks, a_selected, a_connected,
                                     b_selected, b_connected, message, errors)


def require_case_icon_pair_in_blocks(case_blocks: list[str], a_selected: str, a_connected: str,
                                     b_selected: str, b_connected: str, message: str,
                                     errors: list[str]) -> None:
    a_pair = f"UIDP_LCD_VP_HANDLE_A,{a_selected}):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,{a_connected})"
    b_pair = f"UIDP_LCD_VP_HANDLE_B,{b_selected}):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,{b_connected})"
    require(any(a_pair in block for block in case_blocks) and
            any(b_pair in block for block in case_blocks),
            message, errors)


def require_alarm_picture(body: str, alarm_name: str, picture: str, message: str,
                          errors: list[str]) -> None:
    require(f"case{alarm_name}:LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,{picture}U)" in body,
            message, errors)


def main() -> int:
    errors: list[str] = []
    screen_addr = read_text(SCREEN_ADDRESS_H)
    pub_h = read_text(PUB_H)
    pub_c = read_text(PUB_C)
    uidp = read_text(UIDP_C)
    screenkey = read_text(SCREENKEY_C)
    handlescan = read_text(HANDLESCAN_C)
    rfid = read_text(RFID_C)
    keybh = read_text(KEYBH_C)
    external_comm = read_text(EXTERNAL_COMM_C)

    screen_addr_c = compact(screen_addr)
    pub_h_c = compact(pub_h)
    pub_c_c = compact(pub_c)
    uidp_c = compact(uidp)
    screenkey_c = compact(screenkey)
    handlescan_c = compact(handlescan)
    rfid_c = compact(rfid)
    keybh_c = compact(keybh)
    external_comm_c = compact(external_comm)

    expected_vps = {
        "UIDP_LCD_VP_PUMP_A_PLUS": "0x1421U",
        "UIDP_LCD_VP_PUMP_B_PLUS": "0x1422U",
        "UIDP_LCD_VP_PUMP_A_MINUS": "0x1423U",
        "UIDP_LCD_VP_PUMP_B_MINUS": "0x1424U",
        "UIDP_LCD_VP_PUMP_A_UNIT": "0x1425U",
        "UIDP_LCD_VP_PUMP_B_UNIT": "0x1426U",
        "UIDP_LCD_VP_PUMP_A_BUTTON": "0x1427U",
        "UIDP_LCD_VP_PUMP_B_BUTTON": "0x1428U",
        "UIDP_LCD_VP_ALARM_TIP": "0x1429U",
        "UIDP_LCD_VP_TOUCH_WORK": "0x1430U",
    }
    for name, value in expected_vps.items():
        require(f"#define{name}{value}" in screen_addr_c, f"{name} must be {value}", errors)
    require("0x1606U" not in screen_addr_c or "UIDP_LCD_LEGACY_VP_OSC_ANGLE_CACHE0x1606U" in screen_addr_c,
            "main-run pump button VP must not use old 0x1606", errors)

    uidir = compact(function_body(uidp, "UIDIRDP"))
    require("UIDP_LCD_VP_DIR_REVERSE,25U" in uidir and
            "UIDP_LCD_VP_DIR_REVERSE,23U" in uidir and
            "UIDP_LCD_VP_DIR_REVERSE,24U" in uidir,
            "reverse icon mapping must use 23/24/25", errors)
    require("UIDP_LCD_VP_DIR_OSC,28U" in uidir and
            "UIDP_LCD_VP_DIR_OSC,26U" in uidir and
            "UIDP_LCD_VP_DIR_OSC,27U" in uidir,
            "osc icon mapping must use 26/27/28", errors)

    ui_handle = compact(function_body(uidp, "UIHANDLEDP"))
    require_case_icon_pair(ui_handle, "case1:", "102", "101", "132", "131",
                           "generic drill handle icons must use A 101/102 and B 131/132 from EX8 screen table", errors)
    require_case_icon_pair(ui_handle, "case2:", "104", "103", "134", "133",
                           "split handle icons must use A 103/104 and B 133/134 from EX8 screen table", errors)
    require_case_icon_pair(ui_handle, "case3:", "106", "105", "136", "135",
                           "one-piece planer handle icons must use A 105/106 and B 135/136 from EX8 screen table", errors)
    require_case_icon_pair(ui_handle, "case4:", "110", "109", "140", "139",
                           "one-piece grinder handle icons must use A 109/110 and B 139/140 from EX8 screen table", errors)
    require_case_icon_pair_any(ui_handle, ("casePXBA_ONLINES:", "case5:", "case5U:"),
                               "108", "107", "138", "137",
                               "bone/reserved handle icons must use A 107/108 and B 137/138 from EX8 screen table", errors)
    require("caseCOMMON_SOCKET_ONLINES:" in ui_handle and
            "UIDP_LCD_VP_HANDLE_A,112" in ui_handle and
            "UIDP_LCD_VP_HANDLE_A,111" in ui_handle and
            "UIDP_LCD_VP_HANDLE_B,142" in ui_handle and
            "UIDP_LCD_VP_HANDLE_B,141" in ui_handle,
            "common socket handle icons must use A 111/112 and B 141/142", errors)

    ui_manual = compact(function_body(uidp, "UIMANUALBUTTONDP"))
    uidisplay_behavior = compact(function_body(uidp, "UIDISPLAYBehavior"))
    require("voidUIMANUALBUTTONDP(boolenable_flag,boolPAO_flag,uint8_tauto_identify_flag,uint8_ttool_result_pic)" in uidp_c,
            "manual/auto identify display must carry mode flag and tool-result picture", errors)
    require("UIDP_LCD_VP_AUTO_RECOGNIZE,52U" in ui_manual and
            "UIDP_LCD_VP_AUTO_RECOGNIZE,51U" in ui_manual,
            "0x1407 identify-mode VP must use EX8 picture 52 for manual mode and 51 for auto mode", errors)
    tool_result_map = compact(function_body(pub_c, "Pubinterface_MapToolTypeToResultPicture"))
    require("UIDP_LCD_VP_TOOL_RESULT,63U" in ui_manual and
            "tool_result_pic!=0U" in ui_manual and
            "UIDP_LCD_VP_TOOL_RESULT,tool_result_pic" in ui_manual and
            "return61U;" in tool_result_map and
            "return62U;" in tool_result_map,
            "0x1404 must support waiting picture 63 and mapped offline result pictures 61/62", errors)
    require("UIMANUALBUTTONDP(msg.enable_flag,msg.Value[0],msg.Value[1],msg.Value[2])" in uidisplay_behavior,
            "UIDISPLAYBehavior must pass UI_MANUALBUTTON_ID Value[2] as the tool-result picture", errors)

    alarm_body = compact(function_body(uidp, "UIAIARMDP"))
    require_alarm_picture(alarm_body, "WORK_ALARM_HANDLE_NOT_CONNECTED", "80",
                          "handle-not-connected alarm must use EX8 picture 80", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_MANUAL_SELECTED", "81",
                          "manual-selected alarm must use EX8 picture 81", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_FOOT_SELECTED", "82",
                          "foot-selected alarm must use EX8 picture 82", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_FOOT_VALUE_ERROR", "83",
                          "foot value error must use EX8 picture 83", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_UID_ERROR", "85",
                          "UID error must use EX8 picture 85", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_HALL_ERROR", "86",
                          "HALL error must use EX8 picture 86", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_MOTOR_OVERLOAD", "87",
                          "motor overload must use EX8 picture 87", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_MOTOR_OVERLOAD_ALT", "87",
                          "alternate motor overload must use EX8 picture 87", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_MOTOR_COMM_ERROR", "84",
                          "motor communication alarm must use EX8 generic protection picture 84 because the current table has no dedicated communication picture", errors)
    require_alarm_picture(alarm_body, "WORK_ALARM_MOTOR_DRIVER_BOARD", "84",
                          "motor driver-board alarm must use EX8 generic protection picture 84", errors)
    require("caseWORK_ALARM_HANDLE_MODEL_ERROR_A:caseWORK_ALARM_HANDLE_MODEL_ERROR_B:caseWORK_ALARM_HANDLE_MODEL_ERROR_AB:LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,84U)" in alarm_body,
            "handle model verify alarms must use EX8 generic protection picture 84, not external-control picture 88", errors)

    pump_a = compact(function_body(uidp, "UIPUMPADP"))
    pump_b = compact(function_body(uidp, "UIPUMPBDP"))
    require(",222U)" in pump_a and ",223U)" in pump_a and ",224U)" in pump_a and ",225U)" in pump_a,
            "A pump +/- icons must use current EX8 A-side resources 222/223/224/225", errors)
    require("498U" not in pump_a and "499U" not in pump_a and "500U" not in pump_a and "501U" not in pump_a,
            "A pump must not use B-side 498-501 icon resources", errors)
    require(",498U)" in pump_b and ",499U)" in pump_b and ",500U)" in pump_b and ",501U)" in pump_b,
            "B pump +/- icons must use current EX8 B-side resources 498/499/500/501", errors)
    require(",222U)" not in pump_b and ",223U)" not in pump_b and ",224U)" not in pump_b and ",225U)" not in pump_b,
            "B pump must not use A-side 222/223/224/225 icon resources", errors)

    require("case0x07:" in screenkey_c and "case0x02:ScreenKey_PostLegacyAction(42U);" in screenkey_c,
            "0x2407/key2 must map to touch exit", errors)
    require("case0x55:" in screenkey_c and "case0x20:" in screenkey_c and
            "ScreenKey_TouchKeepAlive" in screenkey,
            "0x5520 must map to touch keepalive", errors)
    require("ScreenKey_PostLegacyAction(41U)" not in screenkey,
            "0x5510 legacy touch start must not be posted by screenkey.c", errors)
    require("s_touch_keepalive_ticks" in screenkey and "ScreenKey_ServiceTouchKeepAlive" in screenkey,
            "screenkey task must stop touch run when keepalive times out", errors)
    require("case0x01:ScreenKey_PostLegacyAction(30U);break;" in screenkey_c and
            "case0x02:ScreenKey_PostLegacyAction(31U);break;" in screenkey_c and
            "case0x03:ScreenKey_PostLegacyAction(32U);break;" in screenkey_c and
            "case0x04:ScreenKey_PostLegacyAction(33U);break;" in screenkey_c,
            "0x2401 speed keys must follow EX8 sheet order: fast sub, slow sub, slow add, fast add", errors)
    require("case0x01:ScreenKey_PostLegacyAction(9U);break;" in screenkey_c and
            "case0x02:ScreenKey_PostLegacyAction(10U);break;" in screenkey_c,
            "0x2403 frequency keys must follow EX8 sheet order: add then sub", errors)
    hmi_exit = compact(function_body(pub_c, "HmiExitActive"))
    require("SCREEN_EXTERNAL_EXIT_DOUBLE_CLICK_MS1000U" in pub_c_c and
            "Pubinterface_ConsumeScreenExternalExitDoubleClick()" in pub_c_c,
            "0x2404/key4 external-control exit must use a 1 second double-click helper", errors)
    require("ControlArbitration_IsExternalActive()==false" in hmi_exit,
            "screen external-control exit must ignore the key when external control is not active", errors)
    require("key_value==SCREENKey_HMI_EXIT" in hmi_exit and
            "WorkMessage.alarm_flag==true" in hmi_exit,
            "screen external-control exit must reject the screen key while alarm_flag is true", errors)
    require("key_value==SCREENKey_HMI_EXIT" in hmi_exit and
            "Pubinterface_ConsumeScreenExternalExitDoubleClick()==false" in hmi_exit,
            "screen external-control exit must not release control on the first click", errors)
    release_external = compact(function_body(pub_c, "ControlArbitration_ReleaseExternalControl"))
    require("ControlArbitration_StopMotionOutput();" in release_external,
            "external-control release must stop motor and pump outputs before releasing the owner", errors)
    require(release_external.count("Pubinterface_RefreshControlModeDisplay();") >= 2,
            "external-control release must refresh control buttons in both active and non-owner cleanup paths", errors)
    require(release_external.count("Pubinterface_RefreshExternalCommDisplay(true,false);") >= 2,
            "external-control release must restore the external-communication icon to online-white in both release paths", errors)
    require("WorkMessage.touchactive_work!=TOUCHWORK" in release_external,
            "non-owner external-control release must not clear the local touch popup while touch mode is active", errors)

    require("#defineSCREENKey_TouchKeepAlive" in pub_h_c, "missing SCREENKey_TouchKeepAlive", errors)
    require("caseSCREENKey_TouchKeepAlive:" in keybh_c, "key dispatcher must handle SCREENKey_TouchKeepAlive", errors)
    require("caseSCREENKey_TouchKeepAlive:" in compact(function_body(pub_c, "ControlTypeActive")),
            "ControlTypeActive must handle touch keepalive without hiding touch UI", errors)
    require("s_touch_alarm_release_required" in screenkey_c and
            "s_touch_alarm_release_ticks" in screenkey_c and
            "WorkMessage.alarm_flag==true" in screenkey_c and
            "screen_key==SCREENKey_TouchKeepAlive" in screenkey_c,
            "touch keepalive must latch alarm state and require release before accepting new keepalive frames", errors)

    require("#defineEMBC_ONLINES" in pub_h_c, "missing EMBC_ONLINES enum", errors)
    require("{0x6B,0x0F,EMBC_ONLINES," in handlescan_c, "handlescan must recognize EMBC Page2 code", errors)
    require("caseEMBC_ONLINES:" in compact(function_body(pub_c, "Pubinterface_MapHandleModelToUiType")),
            "EMBC must map to generic drill UI type", errors)

    planer_tool = compact(function_body(pub_c, "Pubinterface_IsPlanerCapabilityTool"))
    osc_support = compact(function_body(pub_c, "Pubinterface_IsOscDirectionSupported"))
    require("tool_type==PLANER" in planer_tool and
            "tool_type==PX_YIP_ONLINES" in planer_tool and
            "tool_type==PX_YIM_ONLINES" not in planer_tool,
            "PLANER and PXP must support OSC capability; PXM must stay GRINDH/no-OSC", errors)
    require("Pubinterface_IsPlanerCapabilityTool(tool_type)" in osc_support,
            "OSC direction support must be driven by PLANER tool capability", errors)
    handle_control = compact(function_body(pub_c, "Pubinterface_IsHandleControlReservedModel"))
    require("hand_model==PXBA_ONLINES" in handle_control and
            "hand_model==PXBB_ONLINES" in handle_control and
            "hand_model==LGZ_II_ONLINES" in handle_control,
            "EX8 handle-control entry must allow PXBA/PXBB/LGZ_II", errors)
    require("LGZ_I_ONLINES" not in handle_control and
            "KSZ_I_ONLINES" not in handle_control and
            "KXZ_I_ONLINES" not in handle_control,
            "EX8 handle-control entry must not allow old reserved LGZ_I/KSZ_I/KXZ_I", errors)
    control_display = compact(function_body(pub_c, "Pubinterface_RefreshControlModeDisplay"))
    selected_display = compact(function_body(pub_c, "Pubinterface_RefreshSelectedChannelDisplay"))
    require("foot_control_available=(ControlSignalMessage.jt_enable_flag==true)" in control_display and
            "handle_control_available=((external_control_active==false)&&Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model))" in control_display,
            "control-mode display must only enable foot when pedal is online and handle when the EX8 handle model supports hand control", errors)
    require("foot_control_available=(ControlSignalMessage.jt_enable_flag==true)" in selected_display and
            "handle_control_available=((external_control_active==false)&&Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model))" in selected_display,
            "selected-channel display must preserve EX8 foot/handle availability rules after A/B switching", errors)
    fallback = compact(optional_function_body(pub_c, "Pubinterface_ApplyEx8ControlModeFallback", errors))
    load_channel = compact(function_body(pub_c, "Pubinterface_LoadChannelMemory"))
    require("ControlSignalMessage.jt_enable_flag==true" in fallback and
            "Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)==false" in fallback and
            "WorkMessage.drivetype_work=HANDLEWORK;" in fallback and
            "memory->drive_type=HANDLEWORK;" in fallback,
            "EX8 must fall back from unavailable foot control to hand control for PXBA/PXBB/LGZII", errors)
    require("Pubinterface_ApplyEx8ControlModeFallback();" in control_display and
            "Pubinterface_ApplyEx8ControlModeFallback();" in load_channel,
            "control-mode fallback must run before redraw and after A/B channel memory load", errors)
    control_type = compact(function_body(pub_c, "ControlTypeActive"))
    require("ControlSignalMessage.jt_enable_flag!=true" in control_type and
            "Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model)==false" in control_type,
            "ControlTypeActive must reject 0x2404 foot/handle mode keys when the EX8 availability conditions are not met", errors)
    require("WorkMessage.hand_model==0U" in control_type and
            "WorkMessage.drivetype_work==HANDLEWORK" in control_type and
            "WorkMessage.touchactive_work!=TOUCHWORK" in control_type,
            "ControlTypeActive must reject EX8 touch entry without a handle, from hand-control mode, and reject keepalive before touch entry", errors)
    touch_exit_cases = re.findall(r"caseSCREENKey_TouchEXIT:.*?break;", control_type)
    require(any("WorkMessage.touchactive_work!=TOUCHWORK" in block for block in touch_exit_cases),
            "ControlTypeActive must reject touch exit when touch mode is not active", errors)
    toolpos = compact(function_body(pub_c, "ToolPosActive"))
    require("Pubinterface_IsOpenPositionEnabledTool(WorkMessage.hand_model,WorkMessage.tool_type)==false" in toolpos,
            "ToolPosActive must gate by PXBA/PXBB plus PLANER open-position capability", errors)
    require("WorkMessage.hand_model==0U" in toolpos and
            "WorkMessage.channel_work!=CHANNEL_A" in toolpos and
            "WorkMessage.channel_work!=CHANNEL_B" in toolpos,
            "ToolPosActive must reject open-position keys when no valid handle/channel is selected", errors)
    require("tool_reduction_ratio" not in toolpos and "500U" not in toolpos,
            "ToolPosActive must not keep low16 reduction-ratio 500 gate", errors)
    require("ToolPosMay(WorkMessage.channel_work,true,1U)" in toolpos and
            "ToolPosMay(WorkMessage.channel_work,false,1U)" in toolpos,
            "ToolPosActive must pass true/false directions", errors)
    dir_active = compact(function_body(pub_c, "DirActive"))
    require("WorkMessage.hand_model==0U" in dir_active and
            "WorkMessage.channel_work!=CHANNEL_A" in dir_active and
            "WorkMessage.channel_work!=CHANNEL_B" in dir_active,
            "DirActive must reject direction keys when no valid handle/channel is selected", errors)
    speed_active = compact(function_body(pub_c, "SpeedActive"))
    require("WorkMessage.hand_model==0U" in speed_active,
            "SpeedActive must reject EX8 speed keys when no valid handle is selected", errors)
    freq_active_body = compact(function_body(pub_c, "FreqActive"))
    require("WorkMessage.hand_model==0U" in freq_active_body,
            "FreqActive must reject EX8 frequency keys when no valid handle is selected", errors)
    tool_switch = compact(function_body(pub_c, "PlanerGridH"))
    require("Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model)==false" in tool_switch and
            "WorkMessage.auto_identify!=0U" in tool_switch,
            "PlanerGridH must only allow manual grinder/planer switching for PXBA/PXBB manual mode", errors)
    require("WorkMessage.tool_type=MemoryMsgA.tool_type=PLANER;" in tool_switch and
            "WorkMessage.tool_type=MemoryMsgB.tool_type=PLANER;" in tool_switch and
            "WorkMessage.tool_type=MemoryMsgA.tool_type=GRINDH;" in tool_switch and
            "WorkMessage.tool_type=MemoryMsgB.tool_type=GRINDH;" in tool_switch,
            "PlanerGridH must switch planer button to PLANER and grinder button to GRINDH", errors)
    require("WorkMessage.dir_work=MemoryMsgA.dir=ZZDIR;" in tool_switch and
            "WorkMessage.dir_work=MemoryMsgB.dir=ZZDIR;" in tool_switch,
            "PlanerGridH must clear inherited OSC direction when switching to grinder", errors)
    require("Pubinterface_RefreshSelectedChannelDisplay(WorkMessage.channel_work);" in tool_switch,
            "PlanerGridH must immediately refresh EX8 display after tool switching", errors)

    freq_active = compact(function_body(pub_c, "FreqActive"))
    require("freq_step=5" in freq_active and "FreqMin5U" in pub_h_c,
            "frequency step/range must be 5Hz and 5-40Hz", errors)
    require("SendUIDSMessage(UI_FREQ_ID,true" in freq_active,
            "FreqActive must refresh UI_FREQ_ID immediately", errors)
    pump_screen_gate = compact(optional_function_body(pub_c, "Pubinterface_ShouldRejectScreenPumpKey", errors))
    require("caseSCREENKey_APUMP_Add:" in pump_screen_gate and
            "caseSCREENKey_APUMP_Sub:" in pump_screen_gate and
            "pumpMessageA.online_flag==false" in pump_screen_gate and
            "pumpMessageA.timingDrainage_flag==true" in pump_screen_gate,
            "EX8 A pump +/- keys must be rejected when A pump is offline or in timed drainage", errors)
    require("caseSCREENKey_BPUMP_Add:" in pump_screen_gate and
            "caseSCREENKey_BPUMP_Sub:" in pump_screen_gate and
            "pumpMessageB.online_flag==false" in pump_screen_gate and
            "pumpMessageB.timingDrainage_flag==true" in pump_screen_gate,
            "EX8 B pump +/- keys must be rejected when B pump is offline or in timed drainage", errors)
    require("caseSCREENKey_APUMP_control:" in pump_screen_gate and
            "caseSCREENKey_BPUMP_control:" in pump_screen_gate and
            "WorkMessage.alarm_flag==true" in pump_screen_gate and
            "pumpMessageA.type==INJECTWATER" in pump_screen_gate and
            "pumpMessageB.type==INJECTWATER" in pump_screen_gate and
            "WorkMessage.runflag_work==true" in pump_screen_gate,
            "EX8 pump control keys must reject alarm state and injection-pump drainage while the handle motor is running", errors)
    pump_active = compact(function_body(pub_c, "PUMPActive"))
    require("if(Pubinterface_ShouldRejectScreenPumpKey(key_value))" in pump_active and
            pump_active.find("Pubinterface_ShouldRejectScreenPumpKey(key_value)") < pump_active.find("switch(key_value)"),
            "PUMPActive must apply EX8 screen pump-key conditions before changing pump state", errors)

    require("auto_identify" in pub_h and "WorkMessage.auto_identify" in pub_c,
            "WorkMessage/MemoryMsg must keep auto-identify mode", errors)
    tool_display = compact(function_body(pub_c, "Pubinterface_RefreshToolDisplay"))
    # 当前选中通道必须二次确认 RFID 手柄型号，避免普通手柄继承另一通道自动识别残留。
    require("split_tool_spec_handle=Pubinterface_IsSplitToolSpecDisplayModel(WorkMessage.hand_model);" in tool_display and
            "rfid_display_enabled=(auto_identify&&split_tool_spec_handle);" in tool_display and
            "manual_display_value[1]=rfid_display_enabled?1U:0U;" in tool_display and
            "SendUIDSMessage(UI_MANUALBUTTON_ID,false,manual_display_value)" in tool_display and
            "SendUIDSMessage(UI_MANUALBUTTON_ID,true,manual_display_value)" in tool_display,
            "tool display must hide manual buttons only when selected channel is RFID auto-identify and show them otherwise", errors)
    auto_identify = compact(function_body(pub_c, "AutoIdentifyActive"))
    require("WorkMessage.auto_identify=0U;" in auto_identify and
            "memory->auto_identify=0U;" in auto_identify and
            "WorkMessage.tool_type=memory->tool_type=GRINDH;" in auto_identify,
            "AutoIdentifyActive must toggle back to manual grinder mode", errors)
    require("WorkMessage.auto_identify=1U;" in auto_identify and
            "memory->auto_identify=1U;" in auto_identify and
            "SendKeyRFIDMessageAup(1U)" in auto_identify and
            "SendKeyRFIDMessageBup(1U)" in auto_identify,
            "AutoIdentifyActive must enter RFID USER auto-identify mode for A/B channels", errors)
    require("HANDLESCAN_TOOL_SOURCE_RFID_USER" in handlescan and
            "COMMON_SOCKET_ONLINES" in handlescan and "Pubinterface_IsCommonSocketToolReady" in pub_c,
            "RFID source rules and common-socket run gate must exist", errors)
    common_missing_alarm = compact(optional_function_body(pub_c, "Pubinterface_ReportCommonSocketToolMissing", errors))
    require("display_value[0]=WORK_ALARM_HANDLE_NOT_CONNECTED;" in common_missing_alarm and
            "SendAlarmMessageTimed(WORK_ALARM_HANDLE_NOT_CONNECTED,COMMON_SOCKET_TOOL_MISSING_ALARM_MS)" in common_missing_alarm and
            "SendUIDSMessage(UI_AIARM_ID,true,display_value)" in common_missing_alarm,
            "common-socket missing EPC tool must raise the EX8 'handle not connected' alarm when run is requested", errors)
    common_run_gate = compact(optional_function_body(pub_c, "Pubinterface_CheckCommonSocketToolReadyForRun", errors))
    require("Pubinterface_ReportCommonSocketToolMissing();" in common_run_gate and "returnfalse;" in common_run_gate,
            "common-socket run gate must report the alarm and reject only real handle-motor run requests", errors)
    try_enter = compact(function_body(pub_c, "ControlArbitration_TryEnter"))
    require("Pubinterface_ReportCommonSocketToolMissing();" not in try_enter and
            "Pubinterface_IsCommonSocketToolReady()" not in try_enter,
            "ControlArbitration_TryEnter must stay as an owner-arbitration helper, not block external-control mode entry on missing EPC", errors)
    external_enter = compact(function_body(pub_c, "ControlArbitration_EnterExternalControl"))
    require("ControlArbitration_TryEnter(CONTROL_OWNER_EXTERNAL)" in external_enter and
            "Pubinterface_CheckCommonSocketToolReadyForRun" not in external_enter,
            "external-control authorization must be allowed to enter mode before EPC run gating", errors)
    external_control = compact(function_body(external_comm, "ExternalComm_ApplyControlCommand"))
    require("case0x05U:" in external_control and
            "Pubinterface_CheckCommonSocketToolReadyForRun()==false" in external_control,
            "external-control handle start must use the common-socket run gate before setting runflag_work", errors)
    require("Pubinterface_ClearCommonSocketToolMissingAlarm();" in compact(function_body(pub_c, "PlugORunPLUGActive")),
            "common-socket alarm must be cleared after EPC tool information is loaded", errors)
    clear_rfid_tool = compact(function_body(pub_c, "Pubinterface_ClearRfidToolMemory"))
    require("Pubinterface_ClearRfidToolMemory(channel);" in compact(function_body(handlescan, "Handlescan_ClearOnlineRfidTool")) and
            "keep_display_tool_type=memory->tool_type;" in clear_rfid_tool and
            "s_rfid_last_tool_type_a=keep_display_tool_type;" in clear_rfid_tool and
            "s_rfid_last_tool_type_b=keep_display_tool_type;" in clear_rfid_tool and
            "WorkMessage.tool_type=0U;" in clear_rfid_tool,
            "PXBA/PXBB RFID USER offline must clear live tool parameters but preserve last business type for 61/62 display", errors)
    require("RFID_DEBUG_BEEP_EVERY_UART_RESPONSE0U" in rfid_c and
            "RFID_DEBUG_BEEP_EVERY_VALID_READ0U" in rfid_c,
            "RFID debug beeps must stay off; beeps should be state-change based", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - EX8 confirmed screen rules passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
