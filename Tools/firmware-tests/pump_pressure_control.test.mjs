import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const toolRoot = path.resolve(import.meta.dirname, "..", "..");
const headerPath = path.join(toolRoot, "User", "Application", "include", "pump_pressure_control.h");
const sourcePath = path.join(toolRoot, "User", "Application", "Pump", "pump_pressure_control.c");

function applyExpectedClosedLoop(targetSpeed, weightX10, thresholdG) {
  const thresholdX10 = thresholdG * 10;
  const stopX10 = Math.floor((thresholdX10 * 3) / 2);

  if (targetSpeed <= 0 || thresholdX10 <= 0 || weightX10 <= thresholdX10) {
    return targetSpeed;
  }

  if (weightX10 >= stopX10) {
    return 0;
  }

  return Math.floor((targetSpeed * (stopX10 - weightX10)) / (stopX10 - thresholdX10));
}

function extractFunctionBody(source, functionName) {
  const nameIndex = source.indexOf(functionName);
  assert.notEqual(nameIndex, -1, `${functionName} should exist`);
  const openIndex = source.indexOf("{", nameIndex);
  assert.notEqual(openIndex, -1, `${functionName} should have a body`);
  let depth = 0;
  for (let index = openIndex; index < source.length; index += 1) {
    if (source[index] === "{") {
      depth += 1;
    } else if (source[index] === "}") {
      depth -= 1;
      if (depth === 0) {
        return source.slice(openIndex + 1, index);
      }
    }
  }
  throw new Error(`${functionName} body was not closed`);
}

test("泵压力闭环：算法模块必须暴露统一的限速接口", () => {
  const header = readFileSync(headerPath, "utf8");
  const source = readFileSync(sourcePath, "utf8");

  assert.match(header, /PumpPressureControl_Apply/);
  assert.match(header, /PumpPressureControl_ShouldForceStop/);
  assert.match(source, /PUMP_PRESSURE_CONTROL_ENABLE/);
  assert.match(source, /threshold_x10/);
  assert.match(source, /stop_x10/);
  assert.match(source, /available_margin_x10/);
});

test("泵压力闭环：总开关默认开启，UART10 文本诊断默认关闭", () => {
  const header = readFileSync(headerPath, "utf8");

  assert.match(header, /#define PUMP_PRESSURE_CONTROL_ENABLE 1U/);
  assert.match(header, /#define PUMP_PRESSURE_CONTROL_UART10_DEBUG_ENABLE 0U/);
});

test("泵压力闭环：硬停倍率必须由宏配置，默认 1.5 倍", () => {
  const header = readFileSync(headerPath, "utf8");
  const source = readFileSync(sourcePath, "utf8");

  assert.match(header, /PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM/);
  assert.match(header, /PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN/);
  assert.match(header, /#define PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM 3U/);
  assert.match(header, /#define PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN 2U/);
  assert.match(source, /PUMP_PRESSURE_CONTROL_STOP_RATIO_NUM/);
  assert.match(source, /PUMP_PRESSURE_CONTROL_STOP_RATIO_DEN/);
  assert.doesNotMatch(source, /threshold_x10\s*\+\s*\(threshold_x10\s*\/\s*2U\)/);
});

test("泵压力闭环：1 倍到默认 1.5 倍阈值线性减速，超过停止倍率输出 0", () => {
  assert.equal(applyExpectedClosedLoop(100, 900, 100), 100);
  assert.equal(applyExpectedClosedLoop(100, 1000, 100), 100);
  assert.equal(applyExpectedClosedLoop(100, 1250, 100), 50);
  assert.equal(applyExpectedClosedLoop(100, 1490, 100), 2);
  assert.equal(applyExpectedClosedLoop(100, 1500, 100), 0);
  assert.equal(applyExpectedClosedLoop(100, 1600, 100), 0);
  assert.equal(applyExpectedClosedLoop(100, 1600, 0), 100);
});

test("泵压力闭环：超过停止倍率后只暂停输出，不能丢失恢复后的运行请求", () => {
  const source = readFileSync(sourcePath, "utf8");
  const pumpA = readFileSync(path.join(toolRoot, "User", "Application", "Beep", "sscPUMPA.c"), "utf8");
  const pumpB = readFileSync(path.join(toolRoot, "User", "Application", "Beep", "sscPUMPB.c"), "utf8");
  const legacyPump = readFileSync(path.join(toolRoot, "User", "Application", "Pump", "pump.c"), "utf8");
  const pumpABody = extractFunctionBody(pumpA, "PUMPA_PauseByPressureLimit");
  const pumpBBody = extractFunctionBody(pumpB, "PUMPB_PauseByPressureLimit");
  const legacyLimitBody = extractFunctionBody(legacyPump, "PumpLegacy_ApplyPressureLimit");

  assert.match(source, /PumpPressureControl_ShouldForceStop/);
  assert.match(pumpA, /PUMPA_PauseByPressureLimit/);
  assert.match(pumpB, /PUMPB_PauseByPressureLimit/);
  assert.doesNotMatch(pumpABody, /pumpMessageA\.run_flag\s*=\s*false/);
  assert.doesNotMatch(pumpABody, /pumpMessageA\.timingDrainage_flag\s*=\s*false/);
  assert.doesNotMatch(pumpBBody, /pumpMessageB\.run_flag\s*=\s*false/);
  assert.doesNotMatch(pumpBBody, /pumpMessageB\.timingDrainage_flag\s*=\s*false/);
  assert.match(legacyPump, /PumpLegacy_ApplyPressureLimit/);
  assert.match(legacyPump, /PumpPressureControl_ShouldForceStop/);
  assert.doesNotMatch(legacyLimitBody, /runtime_msg->run_flag\s*=\s*false/);
  assert.doesNotMatch(legacyLimitBody, /runtime_msg->timingDrainage_flag\s*=\s*false/);
  assert.match(legacyPump, /\(repeat_data!=s\) \|\| \(s == 0U\)/);
});

test("泵压力闭环：取消 UART10 测试信息，UART5 泵帧只由泵任务统一输出", () => {
  const pumpA = readFileSync(path.join(toolRoot, "User", "Application", "Beep", "sscPUMPA.c"), "utf8");
  const pumpAHeader = readFileSync(path.join(toolRoot, "User", "Application", "include", "sscPUMPA.h"), "utf8");
  const externalComm = readFileSync(path.join(toolRoot, "User", "Application", "ExternalComm", "external_comm_task.c"), "utf8");
  const uart5 = readFileSync(path.join(toolRoot, "User", "Peripheral", "uart", "uart5.c"), "utf8");
  const uart5Header = readFileSync(path.join(toolRoot, "User", "Peripheral", "include", "uart5.h"), "utf8");

  assert.doesNotMatch(pumpA, /PUMPA_DebugTraceClosedLoop|PUMPA_DebugPollUart5Reply|PUMPA_SendStopFrameByPressureLimit|BSP_UART_PORT_10/);
  assert.doesNotMatch(pumpAHeader, /PUMPA_SendStopFrameByPressureLimit/);
  assert.doesNotMatch(externalComm, /ExternalComm_DebugTraceUart5PressureStop|ExternalComm_ServiceUart5PumpPressureLimit|PUMPA_SendStopFrameByPressureLimit|PumpPressureControl_ShouldForceStop/);
  assert.match(uart5Header, /#define UART5_DEBUG_UART10_ENABLE 0U/);
  assert.doesNotMatch(uart5, /Uart5_DebugTraceTx|Uart5_DebugTraceRx|U5TX|U5RX|BSP_UART_PORT_10/);
  assert.match(pumpA, /Pump_SetSpeedS_A/);
  assert.match(pumpA, /Uart5_SendPacket/);
});

test("泵压力闭环：压力源固定为 B 泵 PE4、A 泵 PE6，不再自动回退", () => {
  const header = readFileSync(headerPath, "utf8");
  const pumpA = readFileSync(path.join(toolRoot, "User", "Application", "Beep", "sscPUMPA.c"), "utf8");
  const pumpB = readFileSync(path.join(toolRoot, "User", "Application", "Beep", "sscPUMPB.c"), "utf8");
  const legacyPump = readFileSync(path.join(toolRoot, "User", "Application", "Pump", "pump.c"), "utf8");
  const softUart = readFileSync(path.join(toolRoot, "User", "Peripheral", "uart", "soft_uart.c"), "utf8");

  assert.match(header, /#define PUMP_PRESSURE_CONTROL_A_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPA/);
  assert.match(header, /#define PUMP_PRESSURE_CONTROL_B_SOURCE PUMP_PRESSURE_CONTROL_SOURCE_PUMPB/);
  assert.match(softUart, /channel == SIM_UART_1[\s\S]{0,120}pump_message = &pumpMessageB/);
  assert.match(softUart, /channel == SIM_UART_2[\s\S]{0,120}pump_message = &pumpMessageA/);
  assert.doesNotMatch(pumpA, /PUMPA_SelectPressureSource|PUMPA_PressureSourceValid/);
  assert.doesNotMatch(pumpB, /PUMPB_GetPressureSource[\s\S]{0,120}PUMP_PRESSURE_CONTROL_SOURCE_PUMPA/);
  assert.doesNotMatch(legacyPump, /PumpLegacy_SelectPressureSourceA|PumpLegacy_PressureSourceValid/);
});
