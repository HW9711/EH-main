from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUB_C = ROOT / "User" / "Application" / "Pubinterface" / "Pubinterface.c"
PUB_H = ROOT / "User" / "Application" / "include" / "Pubinterface.h"
FOOT_C = ROOT / "User" / "Application" / "Beep" / "sscFOOT.c"
HANDLEKEY_C = ROOT / "User" / "Application" / "Handle" / "handlekey.c"
KEYBH_C = ROOT / "User" / "Application" / "Beep" / "sscKEYBH.c"
COMM_C = ROOT / "User" / "Application" / "ExternalComm" / "external_comm_task.c"


def read_text(path: Path) -> str:
    return path.read_bytes().decode("utf-8", errors="ignore")


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


def extract_case(source: str, case_label: str, next_case_label: str) -> str:
    start = source.find(case_label)
    require(start >= 0, f"missing {case_label}")
    end = source.find(next_case_label, start)
    require(end >= 0, f"missing {next_case_label}")
    return source[start:end]


def main() -> None:
    pub_c = read_text(PUB_C)
    pub_h = read_text(PUB_H)
    foot_c = read_text(FOOT_C)
    handlekey_c = read_text(HANDLEKEY_C)
    keybh_c = read_text(KEYBH_C)
    comm_c = read_text(COMM_C)

    require("Pubinterface_SetHandleInjectionPumpRun" in pub_h,
            "Pubinterface.h must declare the shared handle/injection-pump follow helper")
    follow_func = extract_function(pub_c, "Pubinterface_SetHandleInjectionPumpRun")
    require("HANDLE_INJECTION_PUMP_DEFAULT_FLOW" in pub_c,
            "shared follow helper must define the local cooling default flow")
    require("Pubinterface_GetCurrentDefaultInjectionFlow()" not in follow_func,
            "handle cooling follow must not directly use Page4 default flow as pump run speed")
    require("pumpMessageA.speed_work = HANDLE_INJECTION_PUMP_DEFAULT_FLOW" in follow_func,
            "handle cooling follow must reload the fixed cooling flow on every motor start")
    require("pumpMessageA.run_flag = true" in follow_func and
            "pumpMessageA.run_flag = false" in follow_func,
            "handle cooling follow must own A-pump start and stop")
    require("pumpMessageB.type == INJECTWATER" in follow_func,
            "handle cooling follow must support B pump when B is the identified injection pump")
    require("pumpMessageB.run_flag = true" in follow_func and
            "pumpMessageB.run_flag = false" in follow_func,
            "handle cooling follow must own B-pump start and stop")
    require("WorkMessage.channel_work" in follow_func and
            "CHANNEL_A" in follow_func and
            "CHANNEL_B" in follow_func,
            "when both pumps are injection pumps, follow target must be selected from the active handle channel")
    require("pumpMessageB.speed_work = HANDLE_INJECTION_PUMP_DEFAULT_FLOW" in follow_func,
            "B-pump handle cooling follow must reload the fixed cooling flow on every motor start")
    require("pumpMessageB.timingDrainage_flag = false" in follow_func,
            "handle cooling follow must cancel B timed drainage while the handpiece is running")
    require("pumpMessageA.timingDrainage_flag = false" in follow_func,
            "handle cooling follow must cancel timed drainage while the handpiece is running")

    foot_task = extract_function(foot_c, "FootControlTask")
    single_case = extract_case(foot_task, "case 1://jt", "case 2://jb")
    start_index = single_case.find("Pubinterface_SetHandleInjectionPumpRun(true)")
    owner_index = single_case.find("ControlArbitration_TryEnter(CONTROL_OWNER_FOOT)")
    run_index = single_case.find("WorkMessage.runflag_work=true")
    require(start_index >= 0, "single-foot motor branch must start injection pump through shared follow helper")
    require(owner_index >= 0 and owner_index < start_index,
            "single-foot branch must acquire FOOT owner before starting the linked injection pump")
    require(run_index >= 0 and run_index < start_index,
            "single-foot branch must mark handpiece running before starting the linked injection pump")
    require("Foot_StartPumpAInjection(Pubinterface_GetCurrentDefaultInjectionFlow())" not in single_case,
            "single-foot branch must not pre-start A pump from Page4 default flow")
    require("Pubinterface_SetHandleInjectionPumpRun(false)" in single_case,
            "single-foot release branch must stop the linked injection pump when the handpiece stops")

    run_key_func = extract_function(handlekey_c, "HandleRunKey_SetMotorRun")
    require("Pubinterface_SetHandleInjectionPumpRun(enable)" in run_key_func,
            "physical handle run key must keep injection pump tied to motor run state")

    handle_behavior = extract_function(keybh_c, "HANDLEKeyBehavior")
    require("Pubinterface_SetHandleInjectionPumpRun(true)" in handle_behavior and
            "Pubinterface_SetHandleInjectionPumpRun(false)" in handle_behavior,
            "queued handle key behavior must also keep injection pump tied to motor run state")

    external_follow = extract_function(comm_c, "ExternalComm_SetUart5InjectPumpFollow")
    require("Pubinterface_SetHandleInjectionPumpRun" in external_follow,
            "external host handle start/stop must use the shared A/B injection-pump follow helper")
    require("pumpMessageA.type != INJECTWATER" not in external_follow and
            "pumpMessageA.type == INJECTWATER" not in external_follow,
            "external host handle follow must not reject B-only injection-pump layouts")

    print("ok - handle injection pump follow check passed")


if __name__ == "__main__":
    main()
