import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");
const handlescanPath = path.join(repoRoot, "User", "Application", "Handle", "handlescan.c");

function extractFunctionBody(source, functionName) {
  const signaturePattern = new RegExp(`(?:static\\s+)?(?:void|uint8_t)\\s+${functionName}\\s*\\([^)]*\\)\\s*\\{`, "g");
  const matches = Array.from(source.matchAll(signaturePattern));
  assert.notEqual(matches.length, 0, `${functionName} should exist`);

  const lastMatch = matches.at(-1);
  const openIndex = lastMatch.index + lastMatch[0].lastIndexOf("{");
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

test("handlescan gives clear buzzer feedback for insert success, verify failure, and unplug", () => {
  const source = readFileSync(handlescanPath, "utf8");
  const raiseAlarm = extractFunctionBody(source, "Handlescan_RaiseAlarm");
  const beepOnce = extractFunctionBody(source, "Handlescan_BeepOnceIfNoAlarm");
  const scanA = extractFunctionBody(source, "HandlescanA_Fun_SSC");
  const scanB = extractFunctionBody(source, "HandlescanB_Fun_SSC");

  assert.match(source, /#define\s+HANDLESCAN_REMOVE_DEBOUNCE_MS\s+500U/);

  assert.match(beepOnce, /WorkMessage\.alarm_flag\s*==\s*false/);
  assert.match(beepOnce, /SendKeyBeepMessage\s*\(\s*1U\s*\)/);

  assert.match(raiseAlarm, /WorkAlarm_Set\s*\(\s*alarm_value\s*\)/);
  assert.match(raiseAlarm, /SendAlarmMessage\s*\(\s*alarm_value\s*\)/);
  assert.doesNotMatch(raiseAlarm, /Handlescan_IsHandleVerifyAlarm/);

  assert.match(scanA, /SendKeyBehMessage\s*\(\s*PLUGunPLUG\s*,\s*SCREENKey_PLUG_A\s*\)[\s\S]*s_a_stage\s*=\s*HANDLESCAN_STAGE_ONLINE[\s\S]*Handlescan_BeepOnceIfNoAlarm\s*\(\s*\)/);
  assert.match(scanB, /SendKeyBehMessage\s*\(\s*PLUGunPLUG\s*,\s*SCREENKey_PLUG_B\s*\)[\s\S]*s_b_stage\s*=\s*HANDLESCAN_STAGE_ONLINE[\s\S]*Handlescan_BeepOnceIfNoAlarm\s*\(\s*\)/);

  assert.match(scanA, /SendKeyBehMessage\s*\(\s*PLUGunPLUG\s*,\s*SCREENKey_UNPLUG_A\s*\)[\s\S]*Handlescan_HandleRunningPlugAlarm\s*\(\s*1U\s*\)[\s\S]*Handlescan_BeepOnceIfNoAlarm\s*\(\s*\)/);
  assert.match(scanB, /SendKeyBehMessage\s*\(\s*PLUGunPLUG\s*,\s*SCREENKey_UNPLUG_B\s*\)[\s\S]*Handlescan_HandleRunningPlugAlarm\s*\(\s*2U\s*\)[\s\S]*Handlescan_BeepOnceIfNoAlarm\s*\(\s*\)/);

  assert.doesNotMatch(scanA, /Handlescan_ReleaseHandleAlarmBeep\s*\(\s*CHANNEL_A\s*\)/);
  assert.doesNotMatch(scanB, /Handlescan_ReleaseHandleAlarmBeep\s*\(\s*CHANNEL_B\s*\)/);
});
