from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
KEYBH_C = ROOT / "User" / "Application" / "Beep" / "sscKEYBH.c"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
ADAPTER_C = ROOT / "User" / "Application" / "Screen" / "screen_adapter.c"
SCREEN_H = ROOT / "User" / "Application" / "include" / "screen.h"
PROJECT_LISTS = [
    ROOT / "EIDE" / ".eide" / "eide.yml",
    ROOT / "EIDE" / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvprojx",
]


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


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


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


def switch_case_body(text: str, label: str, following_labels: tuple[str, ...]) -> str:
    start = text.find(label)
    if start < 0:
        raise AssertionError(f"missing case label: {label}")
    ends = [text.find(next_label, start + len(label)) for next_label in following_labels]
    ends = [end for end in ends if end >= 0]
    if not ends:
        raise AssertionError(f"missing end label after: {label}")
    return text[start : min(ends)]


def main() -> int:
    errors: list[str] = []
    pub_h = read_text(PUB_H)
    pub_c = read_text(PUB_C)
    keybh = read_text(KEYBH_C)
    uidp = read_text(UIDP_C)
    screenkey = read_text(SCREENKEY_C)
    project_lists = "\n".join(read_text(path) for path in PROJECT_LISTS)

    pub_h_c = compact(pub_h)
    pub_c_c = compact(pub_c)
    keybh_c = compact(keybh)
    uidp_c = compact(uidp)
    screenkey_c = compact(screenkey)
    project_lists_c = compact(project_lists)

    require("#defineSCREENKey_SPEED_Sub_Large64U" in pub_h_c, "缺少屏幕手柄速度大幅减少键 64U。", errors)
    require("#defineSCREENKey_SPEED_Sub_Small65U" in pub_h_c, "缺少屏幕手柄速度小幅减少键 65U。", errors)
    require("#defineSCREENKey_SPEED_Add_Small66U" in pub_h_c, "缺少屏幕手柄速度小幅增加键 66U。", errors)
    require("#defineSCREENKey_SPEED_Add_Large67U" in pub_h_c, "缺少屏幕手柄速度大幅增加键 67U。", errors)
    require("#defineSCREENKey_AutoIdentify68U" in pub_h_c, "缺少新屏自动识别键 68U。", errors)
    require("#defineUI_POWERINIT_ID17U//屏幕开机初始化区域，统一交给UIDP任务刷新主运行页4" in pub_h_c, "UI_POWERINIT_ID 注释应指向主运行页 page4。", errors)

    post = compact(function_body(screenkey, "ScreenKey_PostLegacyAction"))
    require("case30U:screen_key=SCREENKey_SPEED_Sub_Large;" in post, "0x2401/key1 应映射为速度大幅减少。", errors)
    require("case31U:screen_key=SCREENKey_SPEED_Sub_Small;" in post, "0x2401/key2 应映射为速度小幅减少。", errors)
    require("case32U:screen_key=SCREENKey_SPEED_Add_Small;" in post, "0x2401/key3 应映射为速度小幅增加。", errors)
    require("case33U:screen_key=SCREENKey_SPEED_Add_Large;" in post, "0x2401/key4 应映射为速度大幅增加。", errors)
    require("case36U:screen_key=SCREENKey_AutoIdentify;" in post, "自动识别旧码 36U 应映射到新屏自动识别事件。", errors)

    scan = compact(function_body(screenkey, "ScreenKey_Scan"))
    require("KEY_CONTINUOUSCLICK" not in scan, "老屏 0x2001 连点入口仍会发 KEY_CONTINUOUSCLICK。", errors)
    require("case0x00:" in scan and "case0x03:ScreenKey_PostLegacyAction(22U);" in scan and "case0x07:ScreenKey_PostLegacyAction(36U);" in scan, "0x2400 顶部键未按新屏 key 表完整映射。", errors)
    require("case0x01:" in scan and "ScreenKey_PostLegacyAction(30U)" in scan and "ScreenKey_PostLegacyAction(33U)" in scan, "0x2401 速度四键未按 +/-1000/10000 映射。", errors)
    require("case0x02:" in scan and "ScreenKey_PostLegacyAction(13U)" in scan and "ScreenKey_PostLegacyAction(15U)" in scan and "ScreenKey_PostLegacyAction(14U)" in scan, "0x2402 方向键未映射为正转/往复/反转。", errors)
    require("case0x03:" in scan and "ScreenKey_PostLegacyAction(10U)" in scan and "ScreenKey_PostLegacyAction(9U)" in scan, "0x2403 频率加减键未映射。", errors)
    require("case0x04:" in scan and "ScreenKey_PostLegacyAction(16U)" in scan and "ScreenKey_PostLegacyAction(43U)" in scan, "0x2404 控制模式/外控键未映射。", errors)
    require("case0x06:" in scan and "ScreenKey_PostLegacyAction(5U)" in scan and "ScreenKey_PostLegacyAction(11U)" in scan, "0x2406 应映射为 B 泵加/减/启停。", errors)
    require("0x2407" not in screenkey and "0x2408" not in screenkey and "0x2409" not in screenkey, "新屏主运行页不应再依赖旧 0x2407/0x2408/0x2409。", errors)

    keybh_screen = compact(function_body(keybh, "SCREENKeyBehanior"))
    require("caseSCREENKey_SPEED_Sub_Large:" in keybh_screen and "caseSCREENKey_SPEED_Add_Large:" in keybh_screen, "sscKEYBH 未把新屏大/小速度键分派给 SpeedActive。", errors)
    require("caseSCREENKey_AutoIdentify:" in keybh_screen and "AutoIdentifyActive(key_value);" in keybh_screen, "sscKEYBH 未分派自动识别键。", errors)

    speed_active = compact(function_body(pub_c, "SpeedActive"))
    require("SCREEN_SPEED_STEP_SMALL1000U" in pub_c_c and "SCREEN_SPEED_STEP_LARGE10000U" in pub_c_c, "缺少新屏速度 1000/10000 固定步进宏。", errors)
    require("caseSCREENKey_SPEED_Sub_Large:" in speed_active and "caseSCREENKey_SPEED_Add_Large:" in speed_active, "SpeedActive 未处理新屏大/小速度键。", errors)
    require("speed_step=SCREEN_SPEED_STEP_LARGE;" in speed_active and "speed_step=SCREEN_SPEED_STEP_SMALL;" in speed_active, "SpeedActive 未按 10000/1000 覆盖步进。", errors)
    require("voidAutoIdentifyActive(uint8_tkey_value)" in pub_c_c and "SendKeyRFIDMessageAup(1U)" in pub_c_c, "缺少自动识别业务入口。", errors)

    require("LCD_ForceShow_Which_Map(4);" in function_body(uidp, "UIDISPLAYBehavior"), "UI_POWERINIT_ID 应初始化到主运行页 page4。", errors)
    require(not ADAPTER_C.exists(), "旧屏 screen_adapter.c 应删除，不应继续作为兼容入口保留。", errors)
    require(not SCREEN_H.exists(), "旧屏 screen.h 适配头应删除，避免继续暴露 Screen_* 接口。", errors)
    require("screen_adapter.c" not in project_lists_c and "screen.h" not in project_lists_c, "EIDE/Keil 工程清单不应再注册旧屏接口文件。", errors)

    require("s_uidp_pump_a_gear_pic" in uidp_c and "s_uidp_pump_b_gear_pic" in uidp_c, "泵流量区必须使用 A/B 精确查表。", errors)
    require("{349U,349U,349U,349U,349U}" in uidp_c and "{249U,249U,249U,249U,249U}" in uidp_c, "A/B 泵 0 档应分别固定 349/249。", errors)
    require("{350U,351U,352U,353U,354U}" in uidp_c and "{250U,251U,252U,253U,254U}" in uidp_c, "A/B 泵 1 档渐隐组不正确。", errors)
    require("LCD_Show_Picture(0x1419U" in uidp and "LCD_Show_Picture(0x1420U" in uidp, "A/B 泵流量区 VP 应为 0x1419/0x1420。", errors)
    require("Gear_values+487" not in uidp and "Gear_values+387" not in uidp, "泵流量区仍在使用旧连续图号算法。", errors)

    expected_uidp = [
        "LCD_Show_Picture(0x1428U,80U)",
        "LCD_Show_Picture(0x1410U,",
        "LCD_Show_Picture(0x1411U,",
        "LCD_Show_Picture(0x1412U,",
        "LCD_Show_Picture(0x1413U,",
        "LCD_Show_Picture(0x1414U,",
        "LCD_Show_Picture(0x1415U,",
        "LCD_Show_Picture(0x1416U,",
        "LCD_Show_Picture(0x1408U,",
        "LCD_Show_Picture(0x1409U,",
        "LCD_Show_Picture(0x1403U,50U)",
        "LCD_Show_Picture(0x1405U,",
        "LCD_Show_Picture(0x1406U,",
        "LCD_Show_Picture(0x1407U,",
        "LCD_Show_Picture(0x1417U,",
        "LCD_Show_Picture(0x1418U,",
        "LCD_Show_Picture(0x1421U,",
        "LCD_Show_Picture(0x1423U,",
        "LCD_Show_Picture(0x1424U,",
        "LCD_Show_Picture(0x1425U,",
        "LCD_Show_Picture(0x1426U,",
        "LCD_Show_Picture(0x1427U,",
        "LCD_Show_Picture(0x1422U,",
        "LCD_Show_Picture(0x1606U,"
    ]
    for needle in expected_uidp:
        require(needle in uidp, f"sscUIDP 缺少新屏显示写法：{needle}", errors)
    require("LCD_Show_Picture(0x1421,520)" not in uidp and "LCD_Disappear_Picture(0x1421)" not in uidp, "报警仍写 0x1421，和 A 泵加号冲突。", errors)
    require("LCD_Show_Picture(0x1420,386)" not in uidp and "LCD_Show_Picture(0x1420,384)" not in uidp, "B 泵启停按钮仍写 0x1420。", errors)
    require("LCD_Show_Picture(0x1422U,UIDP_PumpButtonPicture" in uidp_c and "caseINJECTWATER:returnrun_flag?205U:204U;" in uidp_c, "B 泵启停按钮应写 0x1422/200-205。", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - new screen STM32 mapping checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
