from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCREEN_ADDRESS_H = ROOT / "User" / "Application" / "include" / "screen_address.h"
SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"

MIRROR_MACRO = "UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def pump_case_body(text: str, case_id: str, next_case_id: str) -> str:
    match = re.search(
        rf"case\s+{re.escape(case_id)}\s*:\s*//\s*[AB]\s*泵.*?case\s+{re.escape(next_case_id)}\s*:",
        text,
        re.S,
    )
    if not match:
        return ""
    return match.group(0)


def main() -> int:
    errors: list[str] = []
    screen_address = read_text(SCREEN_ADDRESS_H)
    screenkey = read_text(SCREENKEY_C)
    screen_address_c = compact(screen_address)
    screenkey_c = compact(screenkey)
    pump_case_05_c = compact(pump_case_body(screenkey, "0x05", "0x06"))
    pump_case_06_c = compact(pump_case_body(screenkey, "0x06", "0x07"))

    require(f"#ifndef{MIRROR_MACRO}" in screen_address_c, "screen_address.h must expose the pump display mirror switch macro.", errors)
    require(f"#define{MIRROR_MACRO}1U" in screen_address_c, "pump display mirror switch must default to 1U for the current mirrored installation.", errors)
    require(f"#if({MIRROR_MACRO}==1U)" in screen_address_c, "pump display addresses must be guarded by the mirror switch.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_TYPE0x1418U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_TYPE0x1417U" in screen_address_c,
            "mirror-on VP pump type addresses must write logical A to screen B and logical B to screen A.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_GEAR_AREA0x1420U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_GEAR_AREA0x1419U" in screen_address_c,
            "mirror-on gear areas must be swapped.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_PLUS0x1422U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_PLUS0x1421U" in screen_address_c,
            "mirror-on plus button VPs must be swapped.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_MINUS0x1424U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_MINUS0x1423U" in screen_address_c,
            "mirror-on minus button VPs must be swapped.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_UNIT0x1426U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_UNIT0x1425U" in screen_address_c,
            "mirror-on unit VPs must be swapped.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_BUTTON0x1428U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_BUTTON0x1427U" in screen_address_c,
            "mirror-on start/stop button VPs must be swapped.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_VALUE0x3550U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_VALUE0x3530U" in screen_address_c,
            "mirror-on numeric value VPs must be swapped.", errors)
    require("#defineUIDP_LCD_SP_PUMP_A_VALUE0x9550U" in screen_address_c and "#defineUIDP_LCD_SP_PUMP_B_VALUE0x9530U" in screen_address_c,
            "mirror-on numeric SP controls must be swapped.", errors)
    require("#defineUIDP_LCD_SP_PUMP_A_OUTPUT_COLOR0x9553U" in screen_address_c and "#defineUIDP_LCD_SP_PUMP_B_OUTPUT_COLOR0x9533U" in screen_address_c,
            "mirror-on output-color SP controls must be swapped.", errors)

    require("#else" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_A_TYPE0x1417U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_TYPE0x1418U" in screen_address_c,
            "mirror-off branch must keep the original A/B VP mapping.", errors)
    require("#defineUIDP_LCD_VP_PUMP_A_VALUE0x3530U" in screen_address_c and "#defineUIDP_LCD_VP_PUMP_B_VALUE0x3550U" in screen_address_c,
            "mirror-off branch must keep the original A/B numeric VP mapping.", errors)

    require('#include"screen_address.h"' in screenkey_c, "screenkey.c must include the mirror switch from screen_address.h.", errors)
    require(f"#if({MIRROR_MACRO}==1U)" in screenkey_c, "screen touch pump mapping must be guarded by the mirror switch.", errors)
    require("case0x01:ScreenKey_PostLegacyAction(5U);break;" in pump_case_05_c and "case0x03:ScreenKey_PostLegacyAction(11U);break;" in pump_case_05_c,
            "mirror-on 0x2405 screen pump group must control logical B.", errors)
    require("case0x01:ScreenKey_PostLegacyAction(7U);break;" in pump_case_06_c and "case0x03:ScreenKey_PostLegacyAction(12U);break;" in pump_case_06_c,
            "mirror-on 0x2406 screen pump group must control logical A.", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("ok - pump display mirror swap checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
