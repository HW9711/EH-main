import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LCD_C = ROOT / "User" / "Peripheral" / "lcd" / "lcd.c"
LCD_H = ROOT / "User" / "Peripheral" / "include" / "lcd.h"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
USERPARSER_C = ROOT / "User" / "Application" / "Src" / "userparser.c"
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
UI_START_C = ROOT / "User" / "UI" / "src" / "UI_Start.c"
PROJECT_LISTS = [
    ROOT / "EIDE" / ".eide" / "eide.yml",
    ROOT / "EIDE" / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvprojx",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvoptx",
    ROOT / "build" / "MainCtrlF413MXOs" / "MainCtrlF413MXOs.lnp",
    ROOT / "build" / "MainCtrlF413MXOs" / "MainCtrlF413MXOs.objlist",
]
OLD_SCREEN_ARTIFACTS = [
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen.crf",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen.d",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen.o",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen_adapter.crf",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen_adapter.d",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs" / "screen_adapter.o",
    ROOT / "build" / "MainCtrlF413MXOs" / ".obj" / "__" / "User" / "Application" / "Screen" / "screen.d",
    ROOT / "build" / "MainCtrlF413MXOs" / ".obj" / "__" / "User" / "Application" / "Screen" / "screen.o",
    ROOT / "build" / "MainCtrlF413MXOs" / ".obj" / "__" / "User" / "Application" / "Screen" / "screen_adapter.d",
    ROOT / "build" / "MainCtrlF413MXOs" / ".obj" / "__" / "User" / "Application" / "Screen" / "screen_adapter.o",
]


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def compact(text: str) -> str:
    return "".join(text.split())


def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def function_body(text: str, name: str) -> str:
    marker = f"{name}("
    start = text.find(marker)
    if start < 0:
        return ""
    brace = text.find("{", start)
    if brace < 0:
        return ""

    depth = 0
    for index in range(brace, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[brace + 1:index]
    return ""


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []

    lcd_c = read_text(LCD_C)
    lcd_h = read_text(LCD_H)
    uidp_c = read_text(UIDP_C)
    userparser_c = read_text(USERPARSER_C)
    screen_address_h = read_text(SCREEN_ADDRESS_H)
    ui_start_c = read_text(UI_START_C)
    uidp_body = strip_c_comments(function_body(uidp_c, "UIDISPLAYBehavior"))
    uidp_body_c = compact(uidp_body)
    userparser_body_c = compact(strip_c_comments(function_body(userparser_c, "Userparser_Init")))
    ui_start_body_c = compact(strip_c_comments(function_body(ui_start_c, "UI_Start_Fun")))
    screen_address_c = compact(screen_address_h)

    require("LCD_ForceShow_Which_Map" in lcd_h, "lcd.h must expose a forced page-switch API.", errors)
    require("LCD_ForceShow_Which_Map" in lcd_c, "lcd.c must implement a forced page-switch API.", errors)
    require(
        "LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);" in uidp_body_c,
        "UI_POWERINIT_ID must force UIDP_LCD_PAGE_MAIN_RUN instead of relying on cached LCD_Show_Which_Map(0).",
        errors,
    )
    require(
        "UIPUMPADP(0,0,0);" not in uidp_body_c and "UIPUMPBDP(0,0,0);" not in uidp_body_c,
        "UI_POWERINIT_ID must not paint unknown pumps as the default dark draw-water type.",
        errors,
    )
    require(
        "UIDP_DrawPumpDisplaySnapshot(UI_PUMPA_ID,UI_PUMPABUTTON_ID,&pumpMessageA);" in uidp_body_c
        and "UIDP_DrawPumpDisplaySnapshot(UI_PUMPB_ID,UI_PUMPBBUTTON_ID,&pumpMessageB);" in uidp_body_c,
        "UI_POWERINIT_ID must draw A/B pump snapshots directly before showing the EX8 main page.",
        errors,
    )
    required_powerinit_calls = [
        "UIDP_DrawPumpDisplaySnapshot(UI_PUMPA_ID,UI_PUMPABUTTON_ID,&pumpMessageA);",
        "UIDP_DrawPumpDisplaySnapshot(UI_PUMPB_ID,UI_PUMPBBUTTON_ID,&pumpMessageB);",
        "UICONTROLDP(0,0,0);",
        "LCD_Disappear_Picture(UIDP_LCD_VP_TOUCH_WORK);",
        "UIMANUALBUTTONDP(0,0,0,0);",
        "LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);",
    ]
    for call in required_powerinit_calls:
        require(call in uidp_body_c, f"UI_POWERINIT_ID missing required call: {call}", errors)
    main_page_switch = uidp_body_c.find("LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);")
    require(
        main_page_switch > uidp_body_c.find("UIDP_DrawPumpDisplaySnapshot(UI_PUMPB_ID,UI_PUMPBBUTTON_ID,&pumpMessageB);")
        and main_page_switch > uidp_body_c.find("UICONTROLDP(0,0,0);")
        and main_page_switch > uidp_body_c.find("UIMANUALBUTTONDP(0,0,0,0);"),
        "UI_POWERINIT_ID must preload main-run controls before switching to page4, reducing visible one-by-one loading.",
        errors,
    )
    require(
        "#defineUIDP_LCD_PAGE_STARTUP0U" in screen_address_c,
        "screen_address.h must define UIDP_LCD_PAGE_STARTUP=0 for deterministic EX8 startup-page switching.",
        errors,
    )
    require(
        "LCD_Show_Which_Map(0);" not in userparser_body_c,
        "Userparser_Init must not send startup page before UART6 is initialized.",
        errors,
    )
    require(
        "Uart6_Init();" in userparser_body_c
        and "LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_STARTUP);" in userparser_body_c
        and userparser_body_c.find("Uart6_Init();") < userparser_body_c.find("LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_STARTUP);"),
        "Userparser_Init must force the EX8 startup page after Uart6_Init so the page switch frame is actually transmitted.",
        errors,
    )
    require(
        "LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_A_TYPE)" in lcd_c + uidp_c
        and "LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_B_TYPE)" in lcd_c + uidp_c,
        "Unknown pump type must hide A/B type icons instead of falling back to draw-water.",
        errors,
    )
    require(
        "ScreenKey_Scan(" not in ui_start_body_c
        and "ScreenKey_LegacyEventTake(" not in ui_start_body_c
        and "KEY_CONTINUOUSCLICK" not in ui_start_body_c
        and "UI_FootPedalCalibration_Fun(" not in ui_start_body_c
        and "LCD_Show_Which_Map(3)" not in ui_start_body_c,
        "UI_Start_Fun must not keep the old startup LOGO continuous-click calibration entry.",
        errors,
    )

    project_lists = "\n".join(read_text(path) for path in PROJECT_LISTS if path.exists())
    require("screen_adapter.c" not in project_lists, "project source lists must not reference screen_adapter.c.", errors)
    require("screen_adapter.o" not in project_lists, "project object lists must not reference screen_adapter.o.", errors)
    require("screen.h" not in project_lists, "project source lists must not reference screen.h.", errors)
    for artifact in OLD_SCREEN_ARTIFACTS:
        require(not artifact.exists(), f"old screen build artifact should be removed: {artifact.relative_to(ROOT)}", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - screen power-init resend and pump init checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
