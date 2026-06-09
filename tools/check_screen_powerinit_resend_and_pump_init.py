from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LCD_C = ROOT / "User" / "Peripheral" / "lcd" / "lcd.c"
LCD_H = ROOT / "User" / "Peripheral" / "include" / "lcd.h"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
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
    uidp_body = function_body(uidp_c, "UIDISPLAYBehavior")
    uidp_body_c = compact(uidp_body)

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
        "Pubinterface_RefreshPumpADisplay();" in uidp_body_c
        and "Pubinterface_RefreshPumpBDisplay();" in uidp_body_c,
        "UI_POWERINIT_ID must refresh A/B pumps from current pumpMessage state.",
        errors,
    )
    require(
        "LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_A_TYPE)" in lcd_c + uidp_c
        and "LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_B_TYPE)" in lcd_c + uidp_c,
        "Unknown pump type must hide A/B type icons instead of falling back to draw-water.",
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
