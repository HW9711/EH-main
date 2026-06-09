from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCREEN_ROOT = Path(r"D:\EH_main\soft\Screen_uart\EH-main\DL-EX7中文 灌注\DL-EX7中文 灌注")
DISPLAY_XLS = SCREEN_ROOT / "DisplayConfig.xls"
TOUCH_XLS = SCREEN_ROOT / "TouchConfig.xls"
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
MAIN_PAGE_BACKGROUND = DWIN_SET / "0.jpg"
REFERENCE_RUN_BACKGROUND = DWIN_SET / "4.jpg"

SCREENKEY_C = ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c"
UIDP_C = ROOT / "User" / "Application" / "Beep" / "sscUIDP.c"
ADAPTER_C = ROOT / "User" / "Application" / "Screen" / "screen_adapter.c"
SCREEN_H = ROOT / "User" / "Application" / "include" / "screen.h"
PROJECT_LISTS = [
    ROOT / "EIDE" / ".eide" / "eide.yml",
    ROOT / "EIDE" / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "build" / "MainCtrlF413MXOs" / "builder.params",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvprojx",
    ROOT / "MDK-ARM" / "MainCtrlF413MXOs.uvoptx",
]


EXPECTED_DISPLAY = {
    "手柄未连接报警": "0x1428",
    "反转": "0x1412",
    "往复": "0x1411",
    "正转": "0x1410",
    "频率显示区域": "0x1409",
    "速度显示区域": "0x1408",
    "刀具识别结果区域": "0x1404",
    "自动识别": "0x1407",
    "开口定位显示区域": "0x1403",
    "磨头": "0x1405",
    "刨刀": "0x1406",
    "脚踏": "0x1413",
    "手控": "0x1414",
    "触控": "0x1415",
    "外部通信": "0x1416",
    "A手柄识别区域": "0x1401",
    "B手柄识别区域": "0x1402",
    "A泵区域": "0x1419",
    "B泵区域": "0x1420",
    "A泵类型": "0x1417",
    "B泵类型": "0x1418",
    "A泵加": "0x1421",
    "B泵加": "0x1423",
    "A泵减": "0x1424",
    "B泵减": "0x1425",
    "A泵单位": "0x1426",
    "B泵单位": "0x1427",
    "A泵启动": "0x1606",
    "B泵启动": "0x1422",
    "转速值": "0x3420",
    "频率显示": "0x3470",
    "A泵流量数值": "0x3530",
    "B泵流量数值": "0x3550",
    "刀具识别结果文本": "0x4200",
}

EXPECTED_TOUCH = {
    "A手柄/通道": "0x2400",
    "B 手柄/通道": "0x2400",
    "开口定位左": "0x2400",
    "开口定位右": "0x2400",
    "选择磨头模式": "0x2400",
    "选择刨刀模式": "0x2400",
    "自动识别": "0x2400",
    "速度大幅度减少": "0x2401",
    "速度小幅度减少": "0x2401",
    "速度小幅度增加": "0x2401",
    "速度大幅度增加": "0x2401",
    "正转": "0x2402",
    "往复": "0x2402",
    "反转": "0x2402",
    "频率减": "0x2403",
    "频率加": "0x2403",
    "脚控": "0x2404",
    "手控": "0x2404",
    "触控": "0x2404",
    "外部通信": "0x2404",
    "A泵流量增加": "0x2405",
    "A泵流量减少": "0x2405",
    "A泵启动": "0x2405",
    "B泵流量增加": "0x2406",
    "B泵流量减少": "0x2406",
    "B泵启动": "0x2406",
}


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


def normalize_vp(value: object) -> str:
    return str(value).strip().lower()


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_excel_exports() -> dict[str, list[dict[str, str]]]:
    ps_script = rf"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8
function Read-Rows($Path) {{
  $excel = $script:excel
  $wb = $excel.Workbooks.Open($Path, $null, $true)
  try {{
    $ws = $wb.Worksheets.Item(1)
    $used = $ws.UsedRange
    $rows = @()
    for ($r = 2; $r -le [int]$used.Rows.Count; $r++) {{
      $rows += [PSCustomObject]@{{
        Image = [string]$used.Cells.Item($r, 1).Text
        Name = [string]$used.Cells.Item($r, 2).Text
        VP = [string]$used.Cells.Item($r, 3).Text
        Type = [string]$used.Cells.Item($r, 4).Text
      }}
    }}
    return $rows
  }} finally {{
    $wb.Close($false)
    [System.Runtime.InteropServices.Marshal]::ReleaseComObject($wb) | Out-Null
  }}
}}

$script:excel = New-Object -ComObject Excel.Application
$script:excel.Visible = $false
$script:excel.DisplayAlerts = $false
try {{
  $result = [ordered]@{{
    display = @(Read-Rows "{DISPLAY_XLS}")
    touch = @(Read-Rows "{TOUCH_XLS}")
  }}
  $result | ConvertTo-Json -Depth 5 -Compress
}} finally {{
  $script:excel.Quit()
  [System.Runtime.InteropServices.Marshal]::ReleaseComObject($script:excel) | Out-Null
  [GC]::Collect()
  [GC]::WaitForPendingFinalizers()
}}
"""
    completed = subprocess.run(
        ["powershell", "-NoProfile", "-Command", ps_script],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())
    return json.loads(completed.stdout)


def index_rows(rows: list[dict[str, str]], image: str = "0") -> dict[str, str]:
    indexed: dict[str, str] = {}
    for row in rows:
        if str(row.get("Image", "")).strip() == image:
            indexed[str(row.get("Name", "")).strip()] = str(row.get("VP", "")).strip()
    return indexed


def verify_screen_exports(exports: dict[str, list[dict[str, str]]], errors: list[str]) -> None:
    display = index_rows(exports["display"], "4")
    touch = index_rows(exports["touch"], "4")
    for name, expected_vp in EXPECTED_DISPLAY.items():
        actual_vp = display.get(name)
        require(actual_vp is not None, f"DisplayConfig missing main-page control: {name}", errors)
        require(
            normalize_vp(actual_vp) == normalize_vp(expected_vp),
            f"DisplayConfig {name} VP {actual_vp!r} != {expected_vp}",
            errors,
        )
    for name, expected_vp in EXPECTED_TOUCH.items():
        actual_vp = touch.get(name)
        require(actual_vp is not None, f"TouchConfig missing main-page control: {name}", errors)
        require(
            normalize_vp(actual_vp) == normalize_vp(expected_vp),
            f"TouchConfig {name} VP {actual_vp!r} != {expected_vp}",
            errors,
        )
    touch_page3_count = sum(
        1
        for row in exports["touch"]
        if str(row.get("Image", "")).strip() == "3" and normalize_vp(row.get("VP", "")) == "0x2420"
    )
    require(touch_page3_count >= 8, "TouchConfig page3 calibration keys at 0x2420 are incomplete.", errors)
    touch_page0 = [
        row
        for row in exports["touch"]
        if str(row.get("Image", "")).strip() == "0" and normalize_vp(row.get("VP", "")) == "0x2001"
    ]
    require(len(touch_page0) == 1, "TouchConfig should expose exactly one page0 0x2001 startup touch record.", errors)


def verify_screen_runtime_background(errors: list[str]) -> None:
    require(MAIN_PAGE_BACKGROUND.exists(), f"missing main page background: {MAIN_PAGE_BACKGROUND}", errors)
    require(REFERENCE_RUN_BACKGROUND.exists(), f"missing reference run background: {REFERENCE_RUN_BACKGROUND}", errors)
    if not MAIN_PAGE_BACKGROUND.exists() or not REFERENCE_RUN_BACKGROUND.exists():
        return

    main_page_data = MAIN_PAGE_BACKGROUND.read_bytes()
    reference_data = REFERENCE_RUN_BACKGROUND.read_bytes()

    require(main_page_data != reference_data, "DWIN_SET page0 background should stay as startup page, not duplicate page4 run-page background.", errors)
    require(len(reference_data) > 0, "DWIN_SET page4 run-page background is empty.", errors)


def verify_firmware(errors: list[str]) -> None:
    screenkey = read_text(SCREENKEY_C)
    uidp = read_text(UIDP_C)
    scan = compact(function_body(screenkey, "ScreenKey_Scan"))
    uidp_behavior = function_body(uidp, "UIDISPLAYBehavior")
    uidp_compact = compact(uidp)

    require("KEY_CONTINUOUSCLICK" not in scan, "startup 0x2001 still posts the old hidden-entry event.", errors)
    require("case0x00:" in scan and "ScreenKey_PostLegacyAction(24U)" in scan and "ScreenKey_PostLegacyAction(36U)" in scan, "0x2400 main top keys are not fully mapped.", errors)
    require("case0x01:" in scan and "ScreenKey_PostLegacyAction(30U)" in scan and "ScreenKey_PostLegacyAction(33U)" in scan, "0x2401 speed keys are not mapped to 10000/1000 steps.", errors)
    require("case0x02:" in scan and "ScreenKey_PostLegacyAction(13U)" in scan and "ScreenKey_PostLegacyAction(15U)" in scan and "ScreenKey_PostLegacyAction(14U)" in scan, "0x2402 direction keys are not mapped.", errors)
    require("case0x03:" in scan and "ScreenKey_PostLegacyAction(10U)" in scan and "ScreenKey_PostLegacyAction(9U)" in scan, "0x2403 frequency keys are not mapped.", errors)
    require("case0x04:" in scan and "ScreenKey_PostLegacyAction(16U)" in scan and "ScreenKey_PostLegacyAction(43U)" in scan, "0x2404 control-mode keys are not mapped.", errors)
    require("case0x05:" in scan and "ScreenKey_PostLegacyAction(7U)" in scan and "ScreenKey_PostLegacyAction(12U)" in scan, "0x2405 A-pump keys are not mapped.", errors)
    require("case0x06:" in scan and "ScreenKey_PostLegacyAction(5U)" in scan and "ScreenKey_PostLegacyAction(11U)" in scan, "0x2406 B-pump keys are not mapped.", errors)
    require("0x2407" not in screenkey and "0x2408" not in screenkey and "0x2409" not in screenkey, "main-run key parsing still depends on old 0x2407/0x2408/0x2409 groups.", errors)

    expected_uidp_writes = [
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
        "LCD_Show_Picture(0x1419U,",
        "LCD_Show_Picture(0x1420U,",
        "LCD_Show_Picture(0x1421U,",
        "LCD_Show_Picture(0x1422U,",
        "LCD_Show_Picture(0x1423U,",
        "LCD_Show_Picture(0x1424U,",
        "LCD_Show_Picture(0x1425U,",
        "LCD_Show_Picture(0x1426U,",
        "LCD_Show_Picture(0x1427U,",
        "LCD_Show_Picture(0x1606U,",
        "LCD_Show_4byte_Number(0x3420",
        "LCD_Show_4byte_Number(0x3470",
        "LCD_Show_4byte_Number(0x3530",
        "LCD_Show_4byte_Number(0x3550",
    ]
    for needle in expected_uidp_writes:
        require(needle in uidp, f"sscUIDP missing current-screen write: {needle}", errors)

    require("{249U,249U,249U,249U,249U}" in uidp_compact, "B-pump 0 gear must stay fixed at picture 249.", errors)
    require("{250U,251U,252U,253U,254U}" in uidp_compact, "B-pump 1 gear fade table 250-254 is missing.", errors)
    require("{349U,349U,349U,349U,349U}" in uidp_compact, "A-pump 0 gear must stay fixed at picture 349.", errors)
    require("{350U,351U,352U,353U,354U}" in uidp_compact, "A-pump 1 gear fade table 350-354 is missing.", errors)
    require("Gear_values+487" not in uidp and "Gear_values+387" not in uidp, "pump display still uses old continuous image-id arithmetic.", errors)
    require("LCD_ForceShow_Which_Map(4);" in uidp_behavior, "power-init display should enter page4 main-run page.", errors)
    require("LCD_Show_Picture(0x1421,520)" not in uidp and "LCD_Disappear_Picture(0x1421)" not in uidp, "alarm display still writes old 0x1421 area.", errors)
    require("LCD_Show_Picture(0x1420,386)" not in uidp and "LCD_Show_Picture(0x1420,384)" not in uidp, "B-pump button still writes old 0x1420 resources.", errors)

    project_lists = "\n".join(read_text(path) for path in PROJECT_LISTS if path.exists())
    require(not ADAPTER_C.exists(), "screen_adapter.c should be deleted.", errors)
    require(not SCREEN_H.exists(), "screen.h should be deleted.", errors)
    require("screen_adapter.c" not in project_lists and "screen.h" not in project_lists, "project source lists still reference the old screen interface.", errors)


def main() -> int:
    errors: list[str] = []
    require(DISPLAY_XLS.exists(), f"missing screen export: {DISPLAY_XLS}", errors)
    require(TOUCH_XLS.exists(), f"missing screen export: {TOUCH_XLS}", errors)
    if not errors:
        exports = load_excel_exports()
        verify_screen_exports(exports, errors)
        verify_screen_runtime_background(errors)
        verify_firmware(errors)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("ok - current screen export alignment checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
