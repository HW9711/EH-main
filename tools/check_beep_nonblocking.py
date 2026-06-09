from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BEEP_C = ROOT / "User" / "Application" / "Beep" / "sscBEEP.c"


def require(condition, message, errors):
    if not condition:
        errors.append(message)


def main():
    text = BEEP_C.read_text(encoding="utf-8")
    errors = []

    require(
        "vTaskDelay(" not in text,
        "sscBEEP.c must not call vTaskDelay() inside the beep task; it holds the shared soft-task runtime mutex.",
        errors,
    )
    require(
        "key_flag--" in text,
        "key beep handling should use a non-blocking tick countdown.",
        errors,
    )
    require(
        "key_flag = msg.keyBeepTime;" in text,
        "key beep requests should still load the requested 100 ms tick count from the queue message.",
        errors,
    )
    require(
        "BEEP_OFF();" in text and "key_flag = 0;" in text,
        "normal/no-alarm path should still force the buzzer off and clear key beep state.",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        raise SystemExit(1)

    print("beep nonblocking contract OK")


if __name__ == "__main__":
    main()
