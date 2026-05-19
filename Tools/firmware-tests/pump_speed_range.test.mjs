import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");

function readSource(...parts) {
  return readFileSync(path.join(repoRoot, ...parts), "utf8");
}

test("A/B injection pump speed must not be hard-clamped at 70 when controlled by the external host", () => {
  const pumpHeader = readSource("User", "Application", "include", "pump.h");
  const pumpA = readSource("User", "Application", "Beep", "sscPUMPA.c");
  const pumpB = readSource("User", "Application", "Beep", "sscPUMPB.c");

  assert.match(pumpHeader, /#define PUMP_INJECTWATER_SPEED_MAX 300U/);
  assert.match(pumpA, /if\s*\(\s*pump_speed\s*>\s*PUMP_INJECTWATER_SPEED_MAX\s*\)/);
  assert.match(pumpB, /if\s*\(\s*pump_speed\s*>\s*PUMP_INJECTWATER_SPEED_MAX\s*\)/);
  assert.doesNotMatch(pumpA, /if\s*\(\s*pump_speed\s*>\s*70\s*\)\s*pump_speed\s*=\s*70/);
  assert.doesNotMatch(pumpB, /if\s*\(\s*pump_speed\s*>\s*70\s*\)\s*pump_speed\s*=\s*70/);
});
