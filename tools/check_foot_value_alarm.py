from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8", errors="ignore")


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []
    foot_c = read_text("User/Application/Beep/sscFOOT.c")
    uidp_c = read_text("User/Application/Beep/sscUIDP.c")

    require(
        "WORK_ALARM_FOOT_VALUE_ERROR" in uidp_c and "83U" in uidp_c,
        "sscUIDP.c must keep foot value error mapped to image 83",
        errors,
    )
    require(
        "Foot_ReportFootValueErrorAlarm" in foot_c,
        "sscFOOT.c missing foot storage value error alarm reporter",
        errors,
    )
    require(
        "display_value[0] = WORK_ALARM_FOOT_VALUE_ERROR" in foot_c,
        "sscFOOT.c must send WORK_ALARM_FOOT_VALUE_ERROR to UI_AIARM_ID",
        errors,
    )
    require(
        "WorkAlarm_Set(WORK_ALARM_FOOT_VALUE_ERROR)" in foot_c,
        "sscFOOT.c must latch WORK_ALARM_FOOT_VALUE_ERROR",
        errors,
    )
    require(
        "WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR)" in foot_c,
        "foot control path must block pump and handle control while foot value alarm is active",
        errors,
    )
    require(
        "Foot_IsPedalTwoPointStorageValid" in foot_c,
        "sscFOOT.c missing single pedal low/high validation helper",
        errors,
    )
    require(
        "Foot_IsPedalThreePointStorageValid" in foot_c,
        "sscFOOT.c missing dual pedal low/mid/high validation helper",
        errors,
    )
    require(
        foot_c.count("Foot_ReportFootValueErrorAlarm();") >= 3,
        "single/JTB/JTD parse branches must all report invalid storage values",
        errors,
    )
    require(
        "i+15 < rlen" in foot_c,
        "JTB frame bounds must cover bytes up to i+15 before reading calibration values",
        errors,
    )

    if errors:
        print("foot value alarm check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("foot value alarm check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
