from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"
PUMPA_C = ROOT / "User" / "Application" / "Beep" / "sscPUMPA.c"
PUMPB_C = ROOT / "User" / "Application" / "Beep" / "sscPUMPB.c"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def extract_function(source: str, name: str) -> str:
    marker = f"{name}("
    search_from = 0
    while True:
        start = source.find(marker, search_from)
        require(start >= 0, f"missing function {name}")
        line_start = source.rfind("\n", 0, start) + 1
        line_prefix = source[line_start:start].strip()
        in_block_comment = source.rfind("/*", 0, start) > source.rfind("*/", 0, start)
        if in_block_comment or line_prefix.startswith("*") or line_prefix.startswith("//"):
            search_from = start + len(marker)
            continue
        brace = source.find("{", start)
        semicolon = source.find(";", start)
        require(brace >= 0, f"missing body for {name}")
        if semicolon < 0 or brace < semicolon:
            break
        search_from = start + len(marker)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated function {name}")


def extract_case(source: str, start_label: str, end_label: str) -> str:
    start = source.find(start_label)
    require(start >= 0, f"missing case {start_label}")
    end = source.find(end_label, start)
    require(end >= 0, f"missing next case {end_label}")
    return source[start:end]


def main() -> None:
    pub_c = read_text(PUB_C)
    pub_h = read_text(PUB_H)
    comm_c = read_text(COMM_C)
    pumpa_c = read_text(PUMPA_C)
    pumpb_c = read_text(PUMPB_C)

    require("uint16_t Pubinterface_GetInjectionPumpStartFlow(void);" in pub_h,
            "nonzero pump start-flow helper must be public for external control")
    require("uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message);" in pub_h,
            "type-based pump start-speed helper must be public for external control")
    require("volatile uint16_t speed_output;" in pub_h,
            "pump state must keep pressure-limited output speed separate from speed_work setpoint")
    require("uint16_t Pubinterface_GetPumpDisplaySpeed(const pumpMessage_t *pump_message);" in pub_h,
            "screen and host must share the pressure-limited pump display speed helper")
    require("void Pubinterface_UpdatePumpAOutputSpeed(uint16_t output_speed);" in pub_h and
            "void Pubinterface_UpdatePumpBOutputSpeed(uint16_t output_speed);" in pub_h,
            "pump tasks must expose output-speed updates that refresh A/B pump displays")
    require("static uint16_t Pubinterface_GetInjectionPumpStartFlow" not in pub_c,
            "pump start-flow helper must not be file-local")

    display_speed = extract_function(pub_c, "Pubinterface_GetPumpDisplaySpeed")
    require("pump_message->speed_output" in display_speed and
            "pump_message->speed_work" in display_speed and
            "pump_message->run_flag" in display_speed,
            "pump display helper must show closed-loop output while running and preserve speed_work as the standby setpoint")

    send_display = extract_function(pub_c, "Pubinterface_SendPumpDisplay")
    require("Pubinterface_GetPumpDisplaySpeed(pump_message)" in send_display,
            "pump screen refresh must use the shared display-speed helper")
    require("pump_message->speed_work >>" not in send_display and
            "pump_message->speed_work &" not in send_display,
            "pump screen refresh must not directly display raw speed_work during pressure closed loop")

    heartbeat_pump = extract_function(comm_c, "ExternalComm_HeartbeatAppendPump")
    require("Pubinterface_GetPumpDisplaySpeed(pump_message)" in heartbeat_pump,
            "external heartbeat must report pressure-limited pump output speed")
    require("ExternalComm_HeartbeatAppendBE16(info_area, info_len, pump_message->speed_work)" not in heartbeat_pump,
            "external heartbeat must not report raw speed_work during pressure closed loop")

    pumpa_behavior = extract_function(pumpa_c, "PUMPAehaviors")
    pumpb_behavior = extract_function(pumpb_c, "PUMPBehaviors")
    require("Pubinterface_UpdatePumpAOutputSpeed(pump_speed)" in pumpa_behavior,
            "A pump task must publish pressure-limited output speed for display refresh")
    require("Pubinterface_UpdatePumpBOutputSpeed(pump_speed)" in pumpb_behavior,
            "B pump task must publish pressure-limited output speed for display refresh")
    require("pump_speed = 0U;" in pumpa_behavior and "pump_speed = 0U;" in pumpb_behavior,
            "timed drainage expiry must clear displayed output speed before publishing it")

    refresh = extract_function(comm_c, "ExternalComm_RefreshUart5PumpRunState")
    require("Pubinterface_GetPumpStartSpeed(&pumpMessageA)" in refresh,
            "A pump manual/follow refresh must use type-based nonzero start-speed fallback")
    require("ExternalComm_ApplyFixedPumpIdentity" not in comm_c,
            "external control must not force A/B pump identity")
    require("PUMP_FIXED_TYPE" not in comm_c and "INJECT_PUMP_FIXED_ENABLE" not in comm_c,
            "external control must not keep fixed injection-pump macros")
    require("pumpMessageA.type=msg.pump_type" not in pumpa_c and "pumpMessageA.type = msg.pump_type" not in pumpa_c,
            "PUMPA queue commands must not override pump type identified by simulated UART")
    require("pumpMessageB.type=msg.pump_type" not in pumpb_c and "pumpMessageB.type = msg.pump_type" not in pumpb_c,
            "PUMPB queue commands must not override pump type identified by simulated UART")

    pump_active = extract_function(pub_c, "PUMPActive")
    a_control = extract_case(pump_active, "case JTKey_left_long:", "case JTKey_right_short:")
    b_control = extract_case(pump_active, "case JTKey_right_long:", "case HMIkey_APUMP_Add:")
    require("key_value == JTKey_left_long" in a_control,
            "A timed drainage must be limited to the foot long-press key")
    require("key_value == JTKey_right_long" in b_control,
            "B timed drainage must be limited to the foot long-press key")
    require("pumpMessageA.timingDrainage_flag = is_timed_drainage" in a_control,
            "A host/screen pump start must not always enter timed drainage")
    require("pumpMessageB.timingDrainage_flag = is_timed_drainage" in b_control,
            "B host/screen pump start must not always enter timed drainage")

    a_adjust = extract_case(pump_active, "case HMIkey_APUMP_Add:", "case HMIkey_BPUMP_Add:")
    b_adjust = extract_case(pump_active, "case HMIkey_BPUMP_Add:", "case JTKey_Gently_left_start:")
    require("key_value == HMIkey_APUMP_Add || key_value == SCREENKey_APUMP_Add" in a_adjust,
            "A pump add branch must check A pump add keys")
    require("key_value == HMIkey_BPUMP_Add || key_value == SCREENKey_BPUMP_Add" in b_adjust,
            "B pump add branch must check B pump add keys")

    print("ok - pump control runtime path check passed")


if __name__ == "__main__":
    main()
