from __future__ import annotations

import argparse
import shutil
import subprocess
from datetime import datetime
from pathlib import Path


SCREEN_ROOT = (
    Path(r"D:\EH_main\soft\Screen_uart\EH-main")
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
    / "DL-EX7\u4e2d\u6587 \u704c\u6ce8"
)
DWIN_SET = SCREEN_ROOT / "DWIN_SET"
STARTUP_BACKUP = SCREEN_ROOT / "backup_20260605_screen_page0_startup" / "0_startup_original.jpg"
SHOW_BIN = DWIN_SET / "14ShowFile.bin"
TOUCH_BIN = DWIN_SET / "13TouchFile.bin"
DISPLAY_XLS = SCREEN_ROOT / "DisplayConfig.xls"
TOUCH_XLS = SCREEN_ROOT / "TouchConfig.xls"

MAIN_DISPLAY_VPS = {
    0x1401,
    0x1402,
    0x1403,
    0x1404,
    0x1405,
    0x1406,
    0x1407,
    0x1408,
    0x1409,
    0x1410,
    0x1411,
    0x1412,
    0x1413,
    0x1414,
    0x1415,
    0x1416,
    0x1417,
    0x1418,
    0x1419,
    0x1420,
    0x1421,
    0x1422,
    0x1423,
    0x1424,
    0x1425,
    0x1426,
    0x1427,
    0x1428,
    0x1606,
    0x3420,
    0x3470,
    0x3530,
    0x3550,
    0x4200,
}
CALIBRATION_DISPLAY_VPS = {0x3700, 0x3710, 0x3720, 0x3730, 0x3740, 0x3750, 0x3760, 0x3770, 0x3780, 0x3790, 0x37A0}
MAIN_TOUCH_VPS = {0x2400, 0x2401, 0x2402, 0x2403, 0x2404, 0x2405, 0x2406}


def show_entry(data: bytearray, page: int) -> tuple[int, int]:
    pos = 0x10 + page * 4
    count = data[pos] | (data[pos + 1] << 8)
    offset = (data[pos + 2] << 8) | data[pos + 3]
    return count, offset


def set_show_entry(data: bytearray, page: int, count: int, offset: int) -> None:
    pos = 0x10 + page * 4
    data[pos] = count & 0xFF
    data[pos + 1] = (count >> 8) & 0xFF
    data[pos + 2] = (offset >> 8) & 0xFF
    data[pos + 3] = offset & 0xFF


def record_vp(record: bytes) -> int:
    return (record[6] << 8) | record[7]


def backup_files() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = SCREEN_ROOT / f"backup_{stamp}_before_page4_run_layout_patch"
    backup_dir.mkdir(parents=True, exist_ok=False)
    for path in (DWIN_SET / "0.jpg", DWIN_SET / "3.jpg", DWIN_SET / "4.jpg", SHOW_BIN, TOUCH_BIN, DISPLAY_XLS, TOUCH_XLS):
        if path.exists():
            shutil.copy2(path, backup_dir / path.name)
    return backup_dir


def patch_background() -> bool:
    target = DWIN_SET / "0.jpg"
    run_page = DWIN_SET / "4.jpg"
    if not STARTUP_BACKUP.exists():
        raise FileNotFoundError(f"missing startup background backup: {STARTUP_BACKUP}")
    if target.exists() and run_page.exists() and target.read_bytes() != run_page.read_bytes():
        return False
    shutil.copy2(STARTUP_BACKUP, target)
    return True


def patch_show_bin() -> bool:
    data = bytearray(SHOW_BIN.read_bytes())
    if len(data) < 0x45A0:
        raise RuntimeError(f"14ShowFile.bin is too small: {len(data)}")

    page3_count, page3_offset = show_entry(data, 3)
    page4_count, page4_offset = show_entry(data, 4)
    page0_count, page0_offset = show_entry(data, 0)

    if (page3_count, page3_offset, page4_count, page4_offset) == (11, 0x4000, 34, 0x4160):
        return False
    if (page0_count, page0_offset, page4_count, page4_offset) != (34, 0x4000, 11, 0x4440):
        raise RuntimeError(
            f"unexpected 14ShowFile layout: page0=({page0_count},0x{page0_offset:04X}) "
            f"page3=({page3_count},0x{page3_offset:04X}) page4=({page4_count},0x{page4_offset:04X})"
        )

    run_block = bytes(data[page0_offset : page0_offset + page0_count * 32])
    calibration_block = bytes(data[page4_offset : page4_offset + page4_count * 32])
    run_vps = {record_vp(run_block[index * 32 : (index + 1) * 32]) for index in range(page0_count)}
    calibration_vps = {record_vp(calibration_block[index * 32 : (index + 1) * 32]) for index in range(page4_count)}
    if not MAIN_DISPLAY_VPS.issubset(run_vps):
        missing = sorted(MAIN_DISPLAY_VPS - run_vps)
        raise RuntimeError(f"page0 run block is missing main display VPs: {missing}")
    if not CALIBRATION_DISPLAY_VPS.issubset(calibration_vps):
        missing = sorted(CALIBRATION_DISPLAY_VPS - calibration_vps)
        raise RuntimeError(f"page4 calibration block is missing calibration VPs: {missing}")

    for index in range(0x4000, len(data)):
        data[index] = 0
    data[0x4000 : 0x4000 + len(calibration_block)] = calibration_block
    data[0x4160 : 0x4160 + len(run_block)] = run_block

    for page in range(16):
        if page <= 2:
            set_show_entry(data, page, 0, 0x4000)
        elif page == 3:
            set_show_entry(data, page, page4_count, 0x4000)
        elif page == 4:
            set_show_entry(data, page, page0_count, 0x4160)
        else:
            set_show_entry(data, page, 0, 0x45A0)

    SHOW_BIN.write_bytes(data)
    return True


def touch_page(record: bytearray) -> int:
    return (record[0] << 8) | record[1]


def set_touch_page(record: bytearray, page: int) -> None:
    record[0] = (page >> 8) & 0xFF
    record[1] = page & 0xFF


def touch_vp(record: bytes) -> int:
    return (record[17] << 8) | record[18]


def patch_touch_bin() -> bool:
    data = bytearray(TOUCH_BIN.read_bytes())
    record_count = (len(data) - 2) // 32
    changed = False
    main_count = 0
    calibration_count = 0
    startup_count = 0

    for index in range(record_count):
        start = index * 32
        record = bytearray(data[start : start + 32])
        vp = touch_vp(record)
        old_page = touch_page(record)
        if vp == 0x2001:
            new_page = 0
            startup_count += 1
        elif vp in MAIN_TOUCH_VPS:
            new_page = 4
            main_count += 1
        elif vp == 0x2420:
            new_page = 3
            calibration_count += 1
        else:
            new_page = old_page
        if old_page != new_page:
            set_touch_page(record, new_page)
            data[start : start + 32] = record
            changed = True

    if startup_count != 1 or main_count != 26 or calibration_count != 8:
        raise RuntimeError(f"unexpected touch counts: startup={startup_count}, main={main_count}, calibration={calibration_count}")
    if changed:
        TOUCH_BIN.write_bytes(data)
    return changed


def patch_excel_exports() -> bool:
    ps_script = rf"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8
$screen = '{str(SCREEN_ROOT)}'
$mainDisplay = @({','.join(str(v) for v in sorted(MAIN_DISPLAY_VPS))})
$calDisplay = @({','.join(str(v) for v in sorted(CALIBRATION_DISPLAY_VPS))})
$mainTouch = @({','.join(str(v) for v in sorted(MAIN_TOUCH_VPS))})
$changed = $false
function Parse-Vp($Text) {{
  $s = ([string]$Text).Trim()
  if ($s.StartsWith('0x') -or $s.StartsWith('0X')) {{ return [Convert]::ToInt32($s.Substring(2), 16) }}
  if ($s.Length -eq 0) {{ return -1 }}
  return [Convert]::ToInt32($s, 10)
}}
function Parse-Image($Cell) {{
  $s = ([string]$Cell.Text).Trim()
  if ($s.Length -gt 0) {{
    try {{ return [Convert]::ToInt32($s) }} catch {{ }}
  }}
  try {{ return [int][double]$Cell.Value2 }} catch {{ return -999 }}
}}
function Set-Image($Cell, [int]$Value) {{
  if ((Parse-Image $Cell) -ne $Value) {{
    $Cell.Value2 = $Value
    $script:changed = $true
  }}
}}
$excel = New-Object -ComObject Excel.Application
$excel.Visible = $false
$excel.DisplayAlerts = $false
try {{
  foreach ($file in @('DisplayConfig.xls','TouchConfig.xls')) {{
    $path = Join-Path $screen $file
    $wb = $excel.Workbooks.Open($path)
    try {{
      $ws = $wb.Worksheets.Item(1)
      $used = $ws.UsedRange
      for ($r = 2; $r -le [int]$used.Rows.Count; $r++) {{
        $vp = Parse-Vp $used.Cells.Item($r, 3).Text
        if ($file -eq 'DisplayConfig.xls') {{
          if ($mainDisplay -contains $vp) {{ Set-Image $used.Cells.Item($r, 1) 4 }}
          elseif ($calDisplay -contains $vp) {{ Set-Image $used.Cells.Item($r, 1) 3 }}
        }} else {{
          if ($vp -eq 0x2001) {{ Set-Image $used.Cells.Item($r, 1) 0 }}
          elseif ($mainTouch -contains $vp) {{ Set-Image $used.Cells.Item($r, 1) 4 }}
          elseif ($vp -eq 0x2420) {{ Set-Image $used.Cells.Item($r, 1) 3 }}
        }}
      }}
      if ($script:changed) {{ $wb.Save() }}
    }} finally {{
      $wb.Close($false)
      [System.Runtime.InteropServices.Marshal]::ReleaseComObject($wb) | Out-Null
    }}
  }}
}} finally {{
  $excel.Quit()
  [System.Runtime.InteropServices.Marshal]::ReleaseComObject($excel) | Out-Null
  [GC]::Collect()
  [GC]::WaitForPendingFinalizers()
}}
if ($script:changed) {{ 'changed' }} else {{ 'unchanged' }}
"""
    completed = subprocess.run(
        ["powershell", "-NoProfile", "-Command", ps_script],
        cwd=str(Path.cwd()),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())
    return completed.stdout.strip().splitlines()[-1:] == ["changed"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Patch exported DGUS screen files to page0 startup, page3 calibration, page4 main-run layout.")
    parser.add_argument("--no-backup", action="store_true", help="skip creating a timestamped backup before patching")
    args = parser.parse_args()

    backup_dir = None if args.no_backup else backup_files()
    changed = {
        "0.jpg": patch_background(),
        "14ShowFile.bin": patch_show_bin(),
        "13TouchFile.bin": patch_touch_bin(),
        "DisplayConfig/TouchConfig": patch_excel_exports(),
    }
    if backup_dir is not None:
        print(f"backup={backup_dir}")
    for name, did_change in changed.items():
        print(f"{name}: {'changed' if did_change else 'unchanged'}")
    print("layout=page0 startup, page3 calibration, page4 main-run")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
