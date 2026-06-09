from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOFT_UART_C = ROOT / "User" / "Peripheral" / "uart" / "soft_uart.c"
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"
FOOT_C = ROOT / "User" / "Application" / "Beep" / "sscFOOT.c"
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


def main() -> None:
    soft_uart = read_text(SOFT_UART_C)
    pub_c = read_text(PUB_C)
    comm_c = read_text(COMM_C)
    foot_c = read_text(FOOT_C)
    pumpa_c = read_text(PUMPA_C)
    pumpb_c = read_text(PUMPB_C)

    require("#define CS1237_DEVICE_CODE_INJECT_WATER" in soft_uart and "0x00U" in soft_uart,
            "soft_uart.c must define 0x00 as injection pump device code")
    require("#define CS1237_DEVICE_CODE_POUR_WATER" in soft_uart and "0x08U" in soft_uart,
            "soft_uart.c must define 0x08 as perfusion pump device code")
    require("#define CS1237_DEVICE_CODE_DRAW_WATER" in soft_uart and "0x09U" in soft_uart,
            "soft_uart.c must define 0x09 as draw pump device code")

    mapper = extract_function(soft_uart, "Cs1237_MapDeviceCodeToPumpType")
    require("case CS1237_DEVICE_CODE_INJECT_WATER:" in mapper and "return INJECTWATER" in mapper,
            "0x00 device code must map to INJECTWATER")
    require("case CS1237_DEVICE_CODE_POUR_WATER:" in mapper and "return POURWATER" in mapper,
            "0x08 device code must map to POURWATER")
    require("case CS1237_DEVICE_CODE_DRAW_WATER:" in mapper and "return DRAWWATER" in mapper,
            "0x09 device code must map to DRAWWATER")
    require("default:" in mapper and "return 0U" in mapper,
            "reserved pump device codes must not be forced into a known pump type")

    update = extract_function(soft_uart, "Cs1237_UpdatePumpMessage")
    require("pump_message->type = pump_type" in update,
            "valid simulated-UART frames must refresh pumpMessageA/B.type from device code")
    require("pump_message->online_flag = (pump_type != 0U)" in update,
            "only mapped pump device codes should mark the pump online")

    follow = extract_function(pub_c, "Pubinterface_SetHandleInjectionPumpRun")
    require("pumpMessageA.type = INJECTWATER" not in follow,
            "handle cooling follow must not force A pump to injection type")
    require("pumpMessageA.type == INJECTWATER" in follow,
            "handle cooling follow should run only when device code identified A as injection pump")
    require("pumpMessageA.type=INJECTWATER" not in foot_c and "pumpMessageA.type = INJECTWATER" not in foot_c,
            "foot task must not force A pump to injection type")
    require("pumpMessageA.type=msg.pump_type" not in pumpa_c and "pumpMessageA.type = msg.pump_type" not in pumpa_c,
            "PUMPA queue path must not override device-code pump type")
    require("pumpMessageB.type=msg.pump_type" not in pumpb_c and "pumpMessageB.type = msg.pump_type" not in pumpb_c,
            "PUMPB queue path must not override device-code pump type")

    forbidden = [
        "EXTERNAL_COMM_UART5_INJECT_PUMP_FIXED_ENABLE",
        "EXTERNAL_COMM_UART5_PUMP_FIXED_TYPE",
        "EXTERNAL_COMM_PUMPB_INJECT_PUMP_FIXED_ENABLE",
        "EXTERNAL_COMM_PUMPB_PUMP_FIXED_TYPE",
        "ExternalComm_ApplyFixedPumpIdentity",
    ]
    for pattern in forbidden:
        require(pattern not in comm_c, f"external_comm_task.c must remove fixed pump identity path: {pattern}")

    print("ok - pump device-code mapping check passed")


if __name__ == "__main__":
    main()
