from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
PUB = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
UIDP = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def function_body(text: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*(?://[^\n]*\n\s*)?\{{", text)
    if not match:
        raise AssertionError(f"missing function: {name}")
    depth = 0
    start = match.end() - 1
    for index in range(start, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated function: {name}")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    text = read_text(PUB)
    uidp_text = read_text(UIDP)

    send_body = function_body(text, "Pubinterface_SendHandleDisplay")
    require(
        "Pubinterface_MapHandleModelToUiType(handle_model)" in send_body,
        "UI_HANDLE_ID must use handle icon mapping before sending",
    )

    map_body = function_body(text, "Pubinterface_MapHandleModelToUiType")
    for symbol in [
        "TMBB_ONLINES",
        "PXBA_ONLINES",
        "MX_YIM_ONLINES",
        "PX_YIM_ONLINES",
        "LGZ_I_ONLINES",
        "KSZ_I_ONLINES",
        "COMMON_SOCKET_ONLINES",
        "EMBD_ONLINES",
    ]:
        require(symbol in map_body, f"handle mapping missing {symbol}")

    refresh_body = function_body(text, "Pubinterface_RefreshSelectedChannelDisplay")
    refresh_surface = (
        refresh_body
        + function_body(text, "Pubinterface_SendDirectionDisplay")
        + function_body(text, "Pubinterface_RefreshToolDisplay")
        + function_body(text, "Pubinterface_GetToolSpecForChannel")
    )
    for area in [
        "UI_CONTROL_ID",
        "UI_DIR_ID",
        "UI_FREQ_ID",
        "UI_TOOL_ID",
        "UI_TOOLSPEC_ID",
        "UI_ORAL_ID",
        "UI_SPEED_ID",
    ]:
        require(area in refresh_surface, f"selected channel refresh missing {area}")

    require(
        "paoxueSpeciValue_A" in refresh_surface and "paoxueSpeciValue_B" in refresh_surface,
        "selected channel refresh must use A/B tool spec cache",
    )
    require(
        "SendUIDSMessage(UI_TOOLSPEC_ID, true" in refresh_surface,
        "split handle tool spec cache must be able to enable UI_TOOLSPEC_ID",
    )

    switch_body = function_body(text, "HandleSwitchActive")
    for channel in ["CHANNEL_A", "CHANNEL_B"]:
        pattern = (
            rf"Pubinterface_LoadChannelMemory\({channel}\);"
            rf"[\s\S]*?Pubinterface_RefreshSelectedChannelDisplay\({channel}\);"
            rf"[\s\S]*?Pubinterface_RefreshOnlineHandleDisplay\(\);"
        )
        require(
            re.search(pattern, switch_body) is not None,
            f"HandleSwitchActive must refresh full UI then handle highlight after switching {channel}",
        )

    plug_body = function_body(text, "PlugORunPLUGActive")
    for channel in ["CHANNEL_A", "CHANNEL_B"]:
        pattern = (
            rf"Pubinterface_LoadChannelMemory\({channel}\);"
            rf"[\s\S]*?Pubinterface_ApplyChannelDefaultInjectionFlow\({channel}\);"
            rf"[\s\S]*?Pubinterface_RefreshSelectedChannelDisplay\({channel}\);"
            rf"[\s\S]*?Pubinterface_RefreshOnlineHandleDisplay\(\);"
        )
        require(
            re.search(pattern, plug_body) is not None,
            f"PlugORunPLUGActive must refresh full UI after auto selecting {channel}",
        )

    uidp_control_body = function_body(uidp_text, "UICONTROLDP")
    require(
        "control_type == 0U" in uidp_control_body
        and "LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT, 30U)" in uidp_control_body
        and "LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE, 34U)" in uidp_control_body
        and "LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH, 37U)" in uidp_control_body
        and "LCD_Disappear_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL)" in uidp_control_body,
        "UICONTROLDP(false, 0, 0) must draw handle/touch white and hide external comm",
    )

    uidp_dir_body = function_body(uidp_text, "UIDIRDP")
    require(
        "uint8_t light_flag" in uidp_text
        and "light_flag == 2U" in uidp_dir_body
        and "dir_type == 0U" in uidp_dir_body
        and "LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD, 20U)" in uidp_dir_body
        and "LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC, 26U)" in uidp_dir_body
        and "LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE, 23U)" in uidp_dir_body,
        "UIDIRDP must support disabled direction icons and light_flag 2 dark state",
    )


if __name__ == "__main__":
    main()
