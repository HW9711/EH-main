import { readFileSync } from "node:fs";
import { join } from "node:path";

const repoRoot = process.cwd();

function readSource(relativePath) {
  return readFileSync(join(repoRoot, relativePath), "utf8");
}

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

const externalComm = readSource("User/Application/ExternalComm/external_comm_task.c");
const pumpA = readSource("User/Application/Beep/sscPUMPA.c");
const pressureHeader = readSource("User/Application/include/pump_pressure_control.h");

assert(
  externalComm.includes("EXTERNAL_COMM_PUMPB_INJECT_PUMP_FIXED_ENABLE") &&
    externalComm.includes("EXTERNAL_COMM_PUMPB_PUMP_FIXED_TYPE") &&
    externalComm.includes("EXTERNAL_COMM_PUMPB_PUMP_DEFAULT_SPEED"),
  "ExternalComm 应提供 B 泵固定注水泵联调宏，方便无 CS1237 识别时测试 B 通道。"
);

assert(
  externalComm.includes("pumpMessageB.online_flag = true") &&
    externalComm.includes("pumpMessageB.type = EXTERNAL_COMM_PUMPB_PUMP_FIXED_TYPE") &&
    externalComm.includes("pumpMessageB.speed_work = EXTERNAL_COMM_PUMPB_PUMP_DEFAULT_SPEED"),
  "B 泵固定识别打开时，心跳和启动前应把 B 泵补成在线、注水泵和默认速度。"
);

assert(
  /case 0x03U:[\s\S]{0,260}ExternalComm_ApplyFixedPumpIdentity\(\)[\s\S]{0,260}pumpMessageB\.online_flag == false/.test(externalComm),
  "B 泵启动分支必须先应用固定泵身份，再检查 pumpMessageB.online_flag。"
);

assert(
  externalComm.includes("EXTERNAL_COMM_UART5_INJECT_PUMP_FIXED_ENABLE"),
  "ExternalComm 应单独提供固定 UART5 注水泵的宏，不能把固定识别和手柄跟随绑在同一个宏里。"
);

assert(
  externalComm.includes("EXTERNAL_COMM_UART5_INJECT_PUMP_FOLLOW_HANDLE_ENABLE"),
  "ExternalComm 应单独提供注水泵是否跟随手柄启停的宏。"
);

assert(
  !externalComm.includes("EXTERNAL_COMM_UART5_INJECT_PUMP_LINK_TEST_ENABLE"),
  "旧的 LINK_TEST_ENABLE 宏同时控制固定识别和跟随启停，容易误解，应拆分后移除。"
);

assert(
  externalComm.includes("s_uart5_pump_manual_run_request") &&
    externalComm.includes("s_uart5_inject_pump_follow_run_request"),
  "A 泵需要分别记录上位机独立启动请求和手柄冷却跟随请求。"
);

assert(
  externalComm.includes("ExternalComm_RefreshUart5PumpRunState"),
  "ExternalComm 应通过统一函数按“独立请求 OR 跟随请求”刷新 A 泵 run_flag。"
);

assert(
  externalComm.includes("ExternalComm_SetUart5PumpManualRun(1U)") &&
    externalComm.includes("ExternalComm_SetUart5PumpManualRun(0U)"),
  "A 泵上位机启动/停止命令应只改变独立运行请求。"
);

assert(
  externalComm.includes("ExternalComm_SetUart5InjectPumpFollow(1U)") &&
    externalComm.includes("ExternalComm_SetUart5InjectPumpFollow(0U)"),
  "手柄启动/停止命令应只改变注水泵跟随请求。"
);

assert(
  pumpA.includes("PUMPA_ApplyPressureClosedLoop") &&
    pumpA.includes("PumpPressureControl_Apply") &&
    pumpA.includes("PumpPressureControl_ShouldForceStop"),
  "A 泵最终 UART5 输出前必须继续经过压力闭环限速和 1.5 倍硬停判断。"
);

assert(
  pressureHeader.includes("#define PUMP_PRESSURE_CONTROL_ENABLE 1U"),
  "压力闭环默认必须开启，否则超过 1.5 倍阈值时 PumpPressureControl_Apply 不会把泵速压到 0。"
);

assert(
  pressureHeader.includes("#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U"),
  "压力闭环 UART10 文本诊断必须默认关闭，避免泵控制现场被测试输出占用串口时间。"
);

assert(
  !externalComm.includes("ExternalComm_ServiceUart5PumpPressureLimit") &&
    !externalComm.includes("ExternalComm_Uart5PressureForceStopActive") &&
    !externalComm.includes("PUMPA_SendStopFrameByPressureLimit") &&
    !externalComm.includes("PumpPressureControl_ShouldForceStop"),
  "ExternalComm 不能再做压力硬停抢帧；压力限速和 0 速帧必须统一由 PUMPAehaviors() 输出。"
);

assert(
  !/ExternalCommTaskFunc[\s\S]*Pump_SetSpeedS_A\(/.test(externalComm) &&
    !/ExternalCommTaskFunc[\s\S]*Uart5_SendPacket\(/.test(externalComm) &&
    !/ExternalCommTaskFunc[\s\S]*ExternalComm_ServiceUart5PumpPressureLimit\(\)/.test(externalComm),
  "ExternalCommTaskFunc 只能处理上位机协议状态，不能在 10ms 任务里直接向 UART5 发送泵控制帧。"
);

console.log("external_comm_pump_ownership.test.mjs passed");
