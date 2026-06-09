from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FOOT_C = ROOT / "User" / "Application" / "Beep" / "sscFOOT.c"


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
    foot = read_text(FOOT_C)
    control = extract_function(foot, "FootControlTask")
    single_case = extract_case(control, "case 1://jt", "case 2://jb")

    require("FOOT_SINGLE_PEDAL_RELEASE_DEBOUNCE_TICKS" in foot,
            "single-foot release path must define a debounce window")
    require("single_release_debounce_ticks" in control,
            "single-foot release path must keep consecutive release count")
    require("single_release_debounce_ticks=0U" in single_case.replace(" ", ""),
            "pressed branch must clear the single-foot release counter")
    require("Foot_ShouldIgnoreSinglePedalReleaseGlitch" in single_case,
            "single-foot stop branch must ignore short release glitches before stopping A pump")

    stop_index = single_case.find("Foot_StopPumpAInjection()")
    guard_index = single_case.find("Foot_ShouldIgnoreSinglePedalReleaseGlitch")
    require(stop_index >= 0, "single-foot case must still stop A pump on real release")
    require(guard_index >= 0 and guard_index < stop_index,
            "single-foot debounce guard must run before Foot_StopPumpAInjection")

    print("ok - single foot pump stability check passed")


if __name__ == "__main__":
    main()
