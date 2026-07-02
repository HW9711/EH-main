from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []
    pub_h = read_text("User/Application/include/Pubinterface.h")
    pub_c = read_text("User/Application/Pubinterface/Pubinterface.c")
    uidp_c = read_text("User/Application/Beep/sscUIDP.c")

    require(
        "#define WORK_ALARM_PUMP_PRESSURE_BLOCKED" in pub_h,
        "Pubinterface.h missing WORK_ALARM_PUMP_PRESSURE_BLOCKED alarm code",
        errors,
    )
    require(
        "WORK_ALARM_PUMP_PRESSURE_BLOCKED" in uidp_c and "89U" in uidp_c,
        "sscUIDP.c missing pressure alarm code to image 89 mapping",
        errors,
    )
    require(
        "s_pump_pressure_blocked_transient_alarm_active" in pub_c,
        "Pubinterface.c missing pressure transient popup ownership flag",
        errors,
    )
    require(
        "Pubinterface_RaisePumpPressureBlockedTimedAlarm" in pub_c,
        "Pubinterface.c missing pressure popup raise helper",
        errors,
    )
    require(
        "Pubinterface_ServicePumpPressureBlockedTimedAlarm" in pub_c,
        "Pubinterface.c missing pressure popup timeout service",
        errors,
    )
    require(
        "display_value[0] = WORK_ALARM_PUMP_PRESSURE_BLOCKED" in pub_c,
        "Pubinterface.c does not send pressure alarm code to UI_AIARM_ID",
        errors,
    )
    require(
        "Pubinterface_ServicePumpPressureBlockedTimedAlarm();" in pub_c,
        "Pubinterface.c does not call pressure popup service periodically",
        errors,
    )
    require(
        "WorkAlarm_Set(WORK_ALARM_PUMP_PRESSURE_BLOCKED)" not in pub_c,
        "Pressure popup must stay transient and must not write global WorkMessage alarm",
        errors,
    )

    if errors:
        print("pressure alarm popup check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("pressure alarm popup check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
