import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");

function readSource(...parts) {
  return readFileSync(path.join(repoRoot, ...parts), "utf8");
}

test("alarm values are centralized in Pubinterface WorkMessage alarm macros", () => {
  const pubinterfaceHeader = readSource("User", "Application", "include", "Pubinterface.h");
  const pubinterfaceSource = readSource("User", "Application", "Pubinterface", "Pubinterface.c");
  const uidpSource = readSource("User", "Application", "Beep", "sscUIDP.c");
  const motorUartSource = readSource("User", "Application", "MotorUartData", "motoruartdata.c");
  const warnSource = readSource("User", "Application", "Warn", "warn.c");
  const handlescanSource = readSource("User", "Application", "Handle", "handlescan.c");

  for (const macro of [
    "WORK_ALARM_NONE",
    "WORK_ALARM_HANDLE_NOT_CONNECTED",
    "WORK_ALARM_MANUAL_SELECTED",
    "WORK_ALARM_FOOT_SELECTED",
    "WORK_ALARM_MOTOR_OVERLOAD",
    "WORK_ALARM_FOOT_VALUE_ERROR",
    "WORK_ALARM_UID_ERROR",
    "WORK_ALARM_MOTOR_COMM_ERROR",
    "WORK_ALARM_HALL_ERROR",
    "WORK_ALARM_HANDLE_MODEL_ERROR",
  ]) {
    assert.match(pubinterfaceHeader, new RegExp(`#define\\s+${macro}\\b`));
    if (macro !== "WORK_ALARM_NONE") {
      assert.match(uidpSource, new RegExp(`case\\s+${macro}\\s*:`));
    }
  }

  assert.match(pubinterfaceHeader, /void\s+WorkAlarm_Set\s*\(/);
  assert.match(pubinterfaceHeader, /void\s+WorkAlarm_ClearIf\s*\(/);
  assert.match(pubinterfaceSource, /WorkMessage\.alarm_value\s*=\s*alarm_value/);
  assert.doesNotMatch(pubinterfaceSource, /SendUIDSMessage\s*\(\s*UI_AIARM_ID/);
  assert.doesNotMatch(pubinterfaceSource, /SendAlarmMessage\s*\(\s*alarm_value\s*\)/);

  assert.doesNotMatch(motorUartSource, /SysRunData\.StuckFlag3\s*=/);
  assert.doesNotMatch(warnSource, /SysRunData\.StuckFlag3\s*=/);
  assert.doesNotMatch(motorUartSource, /MOTOR_UART_ALARM_/);
  assert.match(motorUartSource, /WORK_ALARM_MOTOR_OVERLOAD/);
  assert.match(motorUartSource, /WORK_ALARM_HALL_ERROR/);
  assert.match(motorUartSource, /WORK_ALARM_MOTOR_COMM_ERROR/);
  assert.doesNotMatch(handlescanSource, /HANDLESCAN_ALARM_A_PAGE_FAIL/);
  assert.doesNotMatch(handlescanSource, /HANDLESCAN_ALARM_A_DATA_FAIL/);
  assert.match(handlescanSource, /return\s+WORK_ALARM_HANDLE_MODEL_ERROR\s*;/);
  assert.doesNotMatch(handlescanSource, /Handlescan_RaiseAlarm\s*\([^;]+WORK_ALARM_HANDLE_MODEL_ERROR\)/);
  assert.match(handlescanSource, /Handlescan_EnterRetryOrFail\s*\([^;]+Handlescan_MapVerifyStatusToAlarm/s);
  assert.match(handlescanSource, /\*last_alarm\s*=\s*alarm_value/);
});
