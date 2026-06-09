from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
PUB = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
HDR = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
SCAN = ROOT / "User" / "Application" / "Handle" / "handlescan.c"
COMM = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"
BEEP = ROOT / "User" / "Application" / "Beep" / "sscBEEP.c"


def resolve_host_path() -> Path:
    candidates = [
        ROOT.parents[1] / "外部通信上位机" / "uart2-external-host-standalone" / "ExternalCommHost.html",
        ROOT.parents[2] / "FinalSoft_test" / "外部通信上位机" / "uart2-external-host-standalone" / "ExternalCommHost.html",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


HOST = resolve_host_path()


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def extract_function(source: str, name: str) -> str:
    marker = f"{name}("
    start = source.find(marker)
    require(start >= 0, f"missing function {name}")
    brace = source.find("{", start)
    require(brace >= 0, f"missing body for {name}")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated function {name}")


def test_handle_switch_loads_a_when_b_selected() -> None:
    func = extract_function(read(PUB), "HandleSwitchActive")
    require("SCREENKey_HANDLE_A" in func, "manual switch must accept screen A button")
    require("SCREENKey_HANDLE_B" in func, "manual switch must accept screen B button")
    require("WorkMessage.Channel_Aonline == true" in func, "manual A switch must require A online")
    require("WorkMessage.Channel_Bonline == true" in func, "manual B switch must require B online")
    require("Pubinterface_LoadChannelMemory(CHANNEL_A)" in func,
            "manual switch to A must load MemoryMsgA through shared loader")
    require("Pubinterface_LoadChannelMemory(CHANNEL_B)" in func,
            "manual switch to B must load MemoryMsgB through shared loader")


def test_pubinterface_has_shared_memory_loader() -> None:
    pub = read(PUB)
    hdr = read(HDR)
    require("Pubinterface_LoadChannelMemory" in pub, "missing shared MemoryMsg -> WorkMessage loader")
    require("Pubinterface_LoadChannelMemory" in hdr, "missing loader declaration")
    require(pub.count("Pubinterface_LoadChannelMemory(") >= 3,
            "manual switch, external switch, and plug policy should share loader")


def test_non_running_current_unplug_selects_peer_but_running_waits_manual() -> None:
    func = extract_function(read(PUB), "PlugORunPLUGActive")
    unplug_a = re.search(r"case\s+SCREENKey_UNPLUG_A:(.*?)(?:case\s+SCREENKey_UNPLUG_B:)", func, re.S)
    unplug_b = re.search(r"case\s+SCREENKey_UNPLUG_B:(.*?)(?:default:)", func, re.S)
    require(unplug_a is not None, "missing UNPLUG_A branch")
    require(unplug_b is not None, "missing UNPLUG_B branch")
    require("WorkMessage.runflag_work == false" in unplug_a.group(1),
            "UNPLUG_A must distinguish non-running from running state")
    require("WorkMessage.Channel_Bonline == true" in unplug_a.group(1),
            "UNPLUG_A non-running fallback must require B online")
    require("Pubinterface_LoadChannelMemory(CHANNEL_B)" in unplug_a.group(1),
            "UNPLUG_A while non-running should select B if B is online")
    require("WorkMessage.channel_work = CHANNEL_NONE" in unplug_a.group(1),
            "UNPLUG_A while running or no peer must wait for manual confirmation")
    require("WorkMessage.runflag_work == false" in unplug_b.group(1),
            "UNPLUG_B must distinguish non-running from running state")
    require("WorkMessage.Channel_Aonline == true" in unplug_b.group(1),
            "UNPLUG_B non-running fallback must require A online")
    require("Pubinterface_LoadChannelMemory(CHANNEL_A)" in unplug_b.group(1),
            "UNPLUG_B while non-running should select A if A is online")
    require("WorkMessage.channel_work = CHANNEL_NONE" in unplug_b.group(1),
            "UNPLUG_B while running or no peer must wait for manual confirmation")


def test_running_peer_plug_does_not_auto_select() -> None:
    func = extract_function(read(PUB), "PlugORunPLUGActive")
    require("Pubinterface_ShouldAutoSelectPluggedChannel" in func,
            "plug handling must route selection through auto-select policy")
    require("WorkMessage.runflag_work == false" in read(PUB),
            "auto-select policy must block automatic selection while motor is running")


def test_scan_defers_on_blocking_alarm() -> None:
    scan = read(SCAN)
    require("Handlescan_ShouldDeferRecognition" in scan,
            "scan layer must expose blocking-alarm defer helper")
    require("WorkMessage.alarm_flag == true" in scan,
            "defer helper must check global alarm flag")


def test_channel_specific_handle_alarm_codes_exist() -> None:
    hdr = read(HDR)
    for name in (
        "WORK_ALARM_HANDLE_MODEL_ERROR_A",
        "WORK_ALARM_HANDLE_MODEL_ERROR_B",
        "WORK_ALARM_HANDLE_MODEL_ERROR_AB",
    ):
        require(name in hdr, f"missing {name}")


def test_three_second_alarm_path_exists() -> None:
    combined = read(BEEP) + read(SCAN) + read(COMM)
    require("SendAlarmMessageTimed" in combined, "missing timed alarm beep API")
    require("3000" in combined or "30U" in combined, "missing 3 second timed alarm duration")
    require("ExternalComm_SendTransientAlarm" in read(COMM),
            "external communication must support transient alarm clear")


def test_upper_host_distinguishes_channel_verify_alarm() -> None:
    host = read(HOST)
    require("A 通道手柄 EEPROM 校验失败" in host, "upper host must name A-channel verify failure")
    require("B 通道手柄 EEPROM 校验失败" in host, "upper host must name B-channel verify failure")
    require("A/B 通道手柄 EEPROM 均校验失败" in host, "upper host must name AB verify failure")


def main() -> None:
    tests = [
        test_handle_switch_loads_a_when_b_selected,
        test_pubinterface_has_shared_memory_loader,
        test_non_running_current_unplug_selects_peer_but_running_waits_manual,
        test_running_peer_plug_does_not_auto_select,
        test_scan_defers_on_blocking_alarm,
        test_channel_specific_handle_alarm_codes_exist,
        test_three_second_alarm_path_exists,
        test_upper_host_distinguishes_channel_verify_alarm,
    ]
    for test in tests:
        test()
    print(f"ok - {len(tests)} handle channel logic checks passed")


if __name__ == "__main__":
    main()
