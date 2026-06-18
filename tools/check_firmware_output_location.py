from __future__ import annotations

import hashlib
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EIDE_BUILD_DIR = ROOT / "EIDE" / "build" / "MainCtrlF413MXOs"
ROOT_BUILD_DIR = ROOT / "build" / "MainCtrlF413MXOs"

AUTHORITATIVE_OUTPUTS = [
    EIDE_BUILD_DIR / "MainCtrlF413MXOs.hex",
    EIDE_BUILD_DIR / "MainCtrlF413MXOs.s19",
]

ROOT_MIRROR_OUTPUTS = [
    ROOT_BUILD_DIR / "MainCtrlF413MXOs.hex",
    ROOT_BUILD_DIR / "MainCtrlF413MXOs.s19",
]

WATCHED_FIRMWARE_FILES = [
    ROOT / "User" / "Application" / "Beep" / "sscUIDP.c",
    ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c",
    ROOT / "User" / "Application" / "ScreenKey" / "screenkey.c",
    ROOT / "User" / "Application" / "Src" / "userparser.c",
    ROOT / "User" / "Application" / "include" / "screen_address.h",
    ROOT / "User" / "UI" / "src" / "UI_Start.c",
    EIDE_BUILD_DIR / "builder.params",
]

EIDE_LINK_OUTPUTS = [
    EIDE_BUILD_DIR / "MainCtrlF413MXOs.lnp",
    EIDE_BUILD_DIR / "MainCtrlF413MXOs.map",
    EIDE_BUILD_DIR / "MainCtrlF413MXOs.htm",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def newest_mtime(paths: list[Path]) -> float:
    existing = [path.stat().st_mtime for path in paths if path.exists()]
    return max(existing) if existing else 0.0


def main() -> int:
    errors: list[str] = []
    watched_mtime = newest_mtime(WATCHED_FIRMWARE_FILES)

    for output in AUTHORITATIVE_OUTPUTS:
        require(output.exists(), f"missing current EIDE firmware output: {output}", errors)
        if output.exists():
            require(
                output.stat().st_mtime >= watched_mtime - 2.0,
                f"{output} is older than the EX8 firmware mapping sources; rebuild with EIDE/build/MainCtrlF413MXOs/builder.params.",
                errors,
            )
    for link_output in EIDE_LINK_OUTPUTS:
        require(link_output.exists(), f"missing current EIDE link output: {link_output}", errors)
        if link_output.exists():
            require(
                link_output.stat().st_mtime >= watched_mtime - 2.0,
                f"{link_output} is older than the EX8 firmware mapping sources; rebuild before judging old-screen cleanup.",
                errors,
            )

    if not errors:
        link_text = "\n".join(path.read_text(encoding="utf-8", errors="ignore") for path in EIDE_LINK_OUTPUTS)
        require(
            "screen_adapter.o" not in link_text and "screen_adapter.c" not in link_text,
            "current EIDE link outputs still reference the deleted old screen_adapter module.",
            errors,
        )
        require(
            "UI_Start.o(i.UI_Start_Fun) refers to UI_FootPedalCalibration.o" not in link_text,
            "current EIDE map still shows the startup page calling the old calibration entry.",
            errors,
        )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    for eide_output, root_output in zip(AUTHORITATIVE_OUTPUTS, ROOT_MIRROR_OUTPUTS):
        if root_output.exists() and sha256(root_output) != sha256(eide_output):
            print(
                "note - root build mirror differs from current EIDE output; burn "
                f"{eide_output}"
            )

    print("ok - current firmware output is under EIDE/build/MainCtrlF413MXOs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
