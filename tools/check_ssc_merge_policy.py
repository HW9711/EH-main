from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    # 仅按字节转成宽松文本做静态匹配，避免历史中文编码影响检查脚本本身。
    return (ROOT / relative_path).read_bytes().decode("utf-8", errors="ignore")


def require(condition, message, errors):
    if not condition:
        errors.append(message)


def require_contains(relative_path, patterns, errors):
    text = read_text(relative_path)
    for pattern in patterns:
        require(pattern in text, f"{relative_path} 缺少: {pattern}", errors)
    return text


def main():
    errors = []

    pub_h = require_contains(
        "User/Application/include/Pubinterface.h",
        [
            "#define LGZ_I_ONLINES",
            "#define LGZ_II_ONLINES",
            "#define KSZ_I_ONLINES",
            "#define KSZ_II_ONLINES",
            "#define KXZ_I_ONLINES",
            "#define KXZ_II_ONLINES",
            "#define COMMON_SOCKET_ONLINES",
            "#define EMBD_ONLINES",
            "#define UI_POWERINIT_ID 17U",
            "#define UI_TOUCH_ID 18U",
        ],
        errors,
    )
    require("WORK_ALARM_NONE" in pub_h, "Pubinterface.h 应保留统一报警宏", errors)

    require_contains(
        "User/Application/Src/userparser.c",
        ["SendUIDSMessage(UI_POWERINIT_ID"],
        errors,
    )

    uidp_c = require_contains(
        "User/Application/Beep/sscUIDP.c",
        [
            "void UITOUCHDP",
            "case UI_TOUCH_ID:",
            "UITOUCHDP(msg.enable_flag",
            "case UI_POWERINIT_ID:",
            "LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN)",
        ],
        errors,
    )
    require("UIAIARMDP(msg.enable_flag,msg.Value[0])" in uidp_c or "UIAIARMDP(msg.enable_flag, msg.Value[0])" in uidp_c,
            "sscUIDP.c 应继续通过 UIAIARMDP 显示统一报警码", errors)

    require_contains(
        "User/Application/Pubinterface/Pubinterface.c",
        [
            "SendUIDSMessage(UI_TOUCH_ID",
            "LGZ_I_ONLINES",
            "KSZ_I_ONLINES",
            "KXZ_I_ONLINES",
            "COMMON_SOCKET_ONLINES",
            "EMBD_ONLINES",
        ],
        errors,
    )

    handlescan_c = require_contains(
        "User/Application/Handle/handlescan.c",
        [
            "LGZ_I_ONLINES",
            "LGZ_II_ONLINES",
            "KSZ_I_ONLINES",
            "KSZ_II_ONLINES",
            "KXZ_I_ONLINES",
            "KXZ_II_ONLINES",
            "COMMON_SOCKET_ONLINES",
            "EMBD_ONLINES",
        ],
        errors,
    )
    require("0x6B" in handlescan_c, "handlescan.c 应保留 Page2 手柄类型主码解析", errors)
    require("{0x6B, 0x0E, EMBD_ONLINES" in handlescan_c,
            "handlescan.c must map EMBD Page2 0x6B/0x0E", errors)

    motor_c = require_contains(
        "User/Application/MotorUartData/motoruartdata.c",
        [
            "#define MOTOR_UART_ALARM_PHASE",
            "#define MOTOR_UART_ALARM_HALL",
            "#define MOTOR_UART_ALARM_OVERLOAD",
            "#define MOTOR_UART_ALARM_VOLTAGE",
            "#define MOTOR_UART_ALARM_DRIVER_BOARD",
            "return MOTOR_UART_ALARM_OVERLOAD",
            "return MOTOR_UART_ALARM_VOLTAGE",
            "return MOTOR_UART_ALARM_HALL",
            "return MOTOR_UART_ALARM_PHASE",
            "return MOTOR_UART_ALARM_DRIVER_BOARD",
            "WorkAlarm_Set(alarm_value)",
        ],
        errors,
    )
    require("WorkMessage.alarm_flag=5" not in motor_c and "DisPlayData[0]=4U" not in motor_c,
            "motoruartdata.c 不应保留硬编码报警弹窗旁路", errors)

    soft_uart_c = read_text("User/Peripheral/uart/soft_uart.c")
    require("channel == SIM_UART_1" in soft_uart_c and "pump_message = &pumpMessageB" in soft_uart_c,
            "soft_uart.c 必须保持 SIM_UART_1/PE4 -> pumpMessageB", errors)
    require("channel == SIM_UART_2" in soft_uart_c and "pump_message = &pumpMessageA" in soft_uart_c,
            "soft_uart.c 必须保持 SIM_UART_2/PE6 -> pumpMessageA", errors)

    pump_pressure_h = require_contains(
        "User/Application/include/pump_pressure_control.h",
        [
            "#define PUMP_PRESSURE_CONTROL_A_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPA",
            "#define PUMP_PRESSURE_CONTROL_B_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPB",
        ],
        errors,
    )
    require("PUMP_PRESSURE_CONTROL_SOURCE_AUTO" in pump_pressure_h,
            "pump_pressure_control.h 可保留 AUTO 兼容宏，但默认不能使用 AUTO", errors)

    external_comm_c = read_text("User/Application/ExternalComm/external_comm_task.c")
    require("Pubinterface_GetPumpStartSpeed(&pumpMessageA)" in external_comm_c and
            "Pubinterface_GetCurrentDefaultInjectionFlow()" in read_text("User/Application/Pubinterface/Pubinterface.c"),
            "external_comm_task.c 固定注水泵启动速度必须通过公共非零兜底接口间接优先使用当前手柄 Page4", errors)
    require("ExternalComm_ApplyFixedPumpIdentity" not in external_comm_c and
            "INJECT_PUMP_FIXED_ENABLE" not in external_comm_c and
            "PUMP_FIXED_TYPE" not in external_comm_c,
            "external_comm_task.c 不得保留固定 A/B 泵为注水泵的身份覆盖路径", errors)
    require("EXTERNAL_COMM_UART5_PUMP_DEFAULT_SPEED" not in external_comm_c and
            "EXTERNAL_COMM_PUMPB_PUMP_DEFAULT_SPEED" not in external_comm_c,
            "external_comm_task.c 不应恢复 30U 固定默认速度", errors)

    for relative_path in [
        "User/Application/Beep/sscFOOT.c",
        "User/Application/Pubinterface/Pubinterface.c",
        "User/Application/Src/userparser.c",
    ]:
        text = read_text(relative_path)
        require("SysRunData.PumpDrain" not in text, f"{relative_path} 不得使用 SysRunData.PumpDrain 排空接口", errors)
        require("SysRunData.Pump5sRun" not in text, f"{relative_path} 不得使用 SysRunData.Pump5sRun 排空接口", errors)
        require("SysRunData.PumpSteping5sNum" not in text, f"{relative_path} 不得使用 SysRunData.PumpSteping5sNum 双击接口", errors)
        require("Pump_Pedal2Pump5sTask_Init" not in text, f"{relative_path} 不得启动旧 Pump_Pedal2Pump5sTask", errors)

    for relative_path in [
        "EIDE/.eide/eide.yml",
        "EIDE/build/MainCtrlF413MXOs/builder.params",
        "build/MainCtrlF413MXOs/builder.params",
        "MDK-ARM/MainCtrlF413MXOs.uvprojx",
    ]:
        text = read_text(relative_path)
        for forbidden in [
            "handledata.c",
            "param.c",
            "warn.c",
            "User/Data/data.c",
            "UI_Main.c",
            "UI_ModelConfiguration.c",
            "UI_Password.c",
        ]:
            require(forbidden not in text, f"{relative_path} 不得重新注册旧模块 {forbidden}", errors)

    if errors:
        print("SSC merge policy check failed:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(1)

    print("SSC merge policy check passed")


if __name__ == "__main__":
    main()
