import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const toolRoot = path.resolve(import.meta.dirname, "..", "..");
const footPath = path.join(toolRoot, "User", "Application", "Beep", "sscFOOT.c");

function readFootSource() {
  return readFileSync(footPath, "utf8");
}

test("脚踏轻排启动必须打开泵任务 run_flag 门控", () => {
  const source = readFootSource();

  assert.match(source, /Foot_StartPumpAInjection/);
  assert.match(source, /Foot_StartPumpBInjection/);
  assert.match(source, /Foot_StartPumpAInjection[\s\S]*pumpMessageA\.run_flag\s*=\s*true/);
  assert.match(source, /Foot_StartPumpBInjection[\s\S]*pumpMessageB\.run_flag\s*=\s*true/);
});

test("脚踏轻排停止必须关闭对应泵任务 run_flag 门控", () => {
  const source = readFootSource();

  assert.match(source, /Foot_StopPumpAInjection/);
  assert.match(source, /Foot_StopPumpBInjection/);
  assert.match(source, /Foot_StopPumpAInjection[\s\S]*pumpMessageA\.run_flag\s*=\s*false/);
  assert.match(source, /Foot_StopPumpBInjection[\s\S]*pumpMessageB\.run_flag\s*=\s*false/);
});

test("脚踏 A 泵注水分支不能误下发到 B 泵", () => {
  const source = readFootSource();

  assert.doesNotMatch(source, /if\(pumpMessageA\.type==INJECTWATER\)[\s\S]{0,600}pumpMessageB\.speed_work=70/);
  assert.doesNotMatch(source, /if\(pumpMessageA\.type==INJECTWATER\)[\s\S]{0,600}SendPumpBMessage\(INJECTWATER,0\)/);
});
