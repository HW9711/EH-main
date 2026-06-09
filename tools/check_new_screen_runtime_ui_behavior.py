from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
PUBINTERFACE_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
PUBINTERFACE_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
SSCFOOT_C = ROOT / "User" / "Application" / "Beep" / "sscFOOT.c"
EXTERNAL_COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"


def read_text(path: Path) -> str:
    """按工程常见编码读取源码，避免中文注释导致检查脚本误判。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"cannot decode {path}")


def compact(text: str) -> str:
    """压缩空白后做结构检查，降低缩进和换行差异对结果的影响。"""
    return re.sub(r"\s+", "", text)


def function_body(text: str, name: str) -> str:
    """提取 C 函数体，用于确认关键刷新逻辑落在正确入口内。"""
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
    """把每个业务现象转成一条可读失败信息，便于硬件复测后回看。"""
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []
    uidp = read_text(UIDP_C)
    screenkey = read_text(SCREENKEY_C)
    pub = read_text(PUBINTERFACE_C)
    pub_h = read_text(PUBINTERFACE_H)
    foot = read_text(SSCFOOT_C)
    external = read_text(EXTERNAL_COMM_C)
    uidp_c = compact(uidp)
    screenkey_c = compact(screenkey)
    pub_c = compact(pub)
    foot_c = compact(foot)
    external_c = compact(external)

    pump_a = compact(function_body(uidp, "UIPUMPADP"))
    pump_b = compact(function_body(uidp, "UIPUMPBDP"))
    pump_type_picture = compact(function_body(uidp, "UIDP_PumpTypePicture"))
    screen_post = compact(function_body(screenkey, "ScreenKey_PostLegacyAction"))
    speed_active = compact(function_body(pub, "SpeedActive"))
    control_mode = compact(function_body(pub, "Pubinterface_RefreshControlModeDisplay"))
    selected_channel = compact(function_body(pub, "Pubinterface_RefreshSelectedChannelDisplay"))
    control_type = compact(function_body(pub, "ControlTypeActive"))
    ui_control = compact(function_body(uidp, "UICONTROLDP"))
    uidp_behavior = compact(function_body(uidp, "UIDISPLAYBehavior"))
    external_reset = compact(function_body(external, "ExternalComm_ResetLinkWatchdog"))
    external_timeout = compact(function_body(external, "ExternalComm_HandleLinkReleaseTimeout"))
    external_enter = compact(function_body(pub, "ControlArbitration_EnterExternalControl"))
    external_release = compact(function_body(pub, "ControlArbitration_ReleaseExternalControl"))

    require("caseINJECTWATER:returnenable_flag?211U:210U;" in pump_type_picture,
            "INJECTWATER must map to 211 yellow and 210 gray after the new pump-title export.", errors)
    require("casePOURWATER:returnenable_flag?213U:212U;" in pump_type_picture,
            "POURWATER must map to 213 yellow and 212 gray after the new pump-title export.", errors)
    require("caseDRAWWATER:returnenable_flag?214U:215U;" in pump_type_picture,
            "DRAWWATER must map to 214 yellow and 215 gray after the new pump-title export.", errors)

    require("LCD_Show_Picture(0x1423U,498U)" in pump_b and "LCD_Show_Picture(0x1425U,500U)" in pump_b,
            "B pump disabled +/- icons must match current screen export: 0x1423=498 and 0x1425=500.", errors)
    require("LCD_Show_Picture(0x1423U,499U)" in pump_b and "LCD_Show_Picture(0x1425U,501U)" in pump_b,
            "B pump enabled +/- icons must match current screen export: 0x1423=499 and 0x1425=501.", errors)
    require("LCD_Show_Picture(0x1421U,223U)" in pump_a and "LCD_Show_Picture(0x1424U,222U)" in pump_a,
            "A 泵暗态加/减按钮应写 0x1421=223、0x1424=222，避免刷新后方向反。", errors)
    require("LCD_Show_Picture(0x1421U,225U)" in pump_a and "LCD_Show_Picture(0x1424U,224U)" in pump_a,
            "A 泵亮态加/减按钮应写 0x1421=225、0x1424=224，避免刷新后方向反。", errors)

    require(pump_a.find("LCD_Show_Picture(0x1419U,UIDP_PumpGearPicture(1U,0U,UIDP_PUMP_GEAR_RUN_FRAME))") <
            pump_a.find("LCD_Show_Picture(0x1421U,223U)") <
            pump_a.find("LCD_Show_Picture(0x1424U,222U)"),
            "A pump disabled refresh must draw the gear background before +/- buttons.", errors)
    require(pump_b.find("LCD_Show_Picture(0x1420U,UIDP_PumpGearPicture(2U,0U,UIDP_PUMP_GEAR_RUN_FRAME))") <
            pump_b.find("LCD_Show_Picture(0x1423U,498U)") <
            pump_b.find("LCD_Show_Picture(0x1425U,500U)"),
            "B pump disabled refresh must draw the gear background before +/- buttons.", errors)
    last_a_gear = max(pump_a.rfind("PumpGeardisplay(1,DRAWWATER,pump_value);"),
                      pump_a.rfind("PumpGeardisplay(1,POURWATER,pump_value);"),
                      pump_a.rfind("PumpGeardisplay(1,INJECTWATER,pump_value);"))
    last_b_gear = max(pump_b.rfind("PumpGeardisplay(2,DRAWWATER,pump_value);"),
                      pump_b.rfind("PumpGeardisplay(2,POURWATER,pump_value);"),
                      pump_b.rfind("PumpGeardisplay(2,INJECTWATER,pump_value);"))
    require(last_a_gear <
            pump_a.rfind("LCD_Show_Picture(0x1421U,225U)") <
            pump_a.rfind("LCD_Show_Picture(0x1424U,224U)"),
            "A pump enabled refresh must redraw +/- buttons after the gear background.", errors)
    require(last_b_gear <
            pump_b.rfind("LCD_Show_Picture(0x1423U,499U)") <
            pump_b.rfind("LCD_Show_Picture(0x1425U,501U)"),
            "B pump enabled refresh must redraw +/- buttons after the gear background.", errors)

    dir_active = compact(function_body(pub, "DirActive"))
    require("Pubinterface_IsOscDirectionSupportedModel" in pub,
            "Pubinterface.c must use the selected handle model to decide whether OSC direction is supported.", errors)
    require("WorkMessage.tool_type!=PLANER" not in dir_active,
            "DirActive must not block the screen OSC key only by WorkMessage.tool_type; PXBA/PXBB support OSC by handle model.", errors)
    require("Pubinterface_IsOscDirectionSupportedModel(WorkMessage.hand_model)==false" in dir_active,
            "DirActive OSC branch must reject only when the selected handle model does not support OSC.", errors)

    require('#include "sscBEEP.h"' in screenkey,
            "screenkey.c 应包含 sscBEEP.h，屏幕有效触控需要蜂鸣反馈。", errors)
    require("SendKeyBeepMessage(1U);" in screen_post and
            screen_post.find("SendKeyBeepMessage(1U);") < screen_post.find("SendKeyBehMessage(SCREENKey,screen_key);"),
            "ScreenKey_PostLegacyAction 应在投递有效屏幕按键前蜂鸣一次。", errors)

    require("SendUIDSMessage(UI_SPEED_ID,true,display_value);" in speed_active,
            "SpeedActive 调整速度后应立即刷新 UI_SPEED_ID。", errors)
    require("display_value[2]=1U;" in speed_active and "display_value[3]=WorkMessage.runflag_work?1U:0U;" in speed_active,
            "SpeedActive 的速度刷新应标记为数值更新，并携带当前运行态。", errors)

    require("Pubinterface_RefreshControlModeDisplay" in pub_h and
            "voidPubinterface_RefreshControlModeDisplay(void)" in pub_c,
            "控制模式刷新函数应对脚踏任务公开，避免分散直写 UI_CONTROL_ID。", errors)
    require("boolhandle_control_available=(external_control_active==false);" in control_mode and
            "booltouch_control_available=(external_control_active==false);" in control_mode,
            "Control-mode refresh must show handle/touch as white selectable when external control is not active.", errors)
    require("boolhandle_control_available=(external_control_active==false);" in selected_channel and
            "booltouch_control_available=(external_control_active==false);" in selected_channel,
            "Channel refresh must not turn handle/touch gray after the main run page has loaded.", errors)
    require("control_type==0U" in ui_control and
            "LCD_Show_Picture(0x1414U,34U)" in ui_control and
            "LCD_Show_Picture(0x1415U,37U)" in ui_control and
            "LCD_Disappear_Picture(0x1416U)" in ui_control,
            "Power-init control icons must show handle/touch white and hide external comm until connected.", errors)
    require("caseSCREENKey_TouchActi:" in control_type and
            "ControlArbitration_TryEnter(CONTROL_OWNER_SCREEN)" in control_type and
            "WorkMessage.drivetype_work=TOUCHWORK;" in control_type and
            "WorkMessage.runflag_work=true;" in control_type,
            "The new-screen touch button must directly enter screen touch control and start motion.", errors)

    require("SendUIDSMessage(UI_CONTROL_ID" not in foot_c,
            "sscFOOT.c 不应直接写 UI_CONTROL_ID，应统一调用 Pubinterface_RefreshControlModeDisplay。", errors)
    require("Pubinterface_RefreshControlModeDisplay();" in foot,
            "sscFOOT.c 脚踏上线/掉线/切换后应调用统一控制模式刷新。", errors)

    require("Pubinterface_RefreshExternalCommDisplay" in pub_h,
            "应提供外部通信图标刷新函数，区分白色在线和黄色外控。", errors)
    require("Pubinterface_RefreshExternalCommDisplay(true,true);" in external_enter,
            "申请外控成功后应把小电脑图标刷成黄色。", errors)
    require("Pubinterface_RefreshExternalCommDisplay(true,false);" in external_release,
            "退出外控后应保留外部通信在线白色图标。", errors)
    require("Pubinterface_RefreshExternalCommDisplay(true,ControlArbitration_IsExternalActive());" in external_reset,
            "收到合法外部通信帧时应刷新小电脑图标在线状态。", errors)
    require("Pubinterface_RefreshExternalCommDisplay(false,false);" in external_timeout,
            "外部通信长时间无合法帧后应熄灭小电脑图标。", errors)

    batch_limit_match = re.search(r"#define\s+UIDP_DISPLAY_BATCH_LIMIT\s+(\d+)U", uidp)
    require(batch_limit_match is not None and int(batch_limit_match.group(1)) >= 12,
            "UIDP_DISPLAY_BATCH_LIMIT must be at least 12 to avoid visible one-by-one loading on power-up.", errors)

    require("UIDP_DISPLAY_BATCH_LIMIT" in uidp and "processed_count" in uidp_behavior and
            "while(processed_count<UIDP_DISPLAY_BATCH_LIMIT)" in uidp_behavior,
            "UIDISPLAYBehavior 应每周期批量消费 UI 消息，减少控件陆续加载。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - new screen runtime UI behavior checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
