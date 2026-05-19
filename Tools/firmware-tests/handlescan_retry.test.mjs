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

test("handlescan authentication failure retries before final alarm and keeps slow self-retry", () => {
  const source = readFileSync(handlescanPath, "utf8");
  const retryHelper = extractFunctionBody(source, "Handlescan_EnterRetryOrFail");
  const scanA = extractFunctionBody(source, "HandlescanA_Fun_SSC");
  const scanB = extractFunctionBody(source, "HandlescanB_Fun_SSC");

  assert.match(source, /#define\s+HANDLESCAN_VERIFY_START_DELAY_MS\s+200U/);
  assert.match(source, /#define\s+HANDLESCAN_VERIFY_RETRY_MAX\s+3U/);
  assert.match(source, /#define\s+HANDLESCAN_VERIFY_RETRY_DELAY_MS\s+200U/);
  assert.match(source, /#define\s+HANDLESCAN_VERIFY_ALARM_RETRY_MS\s+1000U/);
  assert.match(source, /HANDLESCAN_STAGE_RETRY_WAIT/);

  for (const symbol of [
    "s_a_verify_retry_count",
    "s_b_verify_retry_count",
    "s_a_verify_retry_wait_ticks",
    "s_b_verify_retry_wait_ticks",
  ]) {
    assert.match(source, new RegExp(`static\\s+uint(?:8|16)_t\\s+${symbol}\\b`));
  }

  assert.match(retryHelper, /HANDLESCAN_STAGE_RETRY_WAIT/);
  assert.match(retryHelper, /HANDLESCAN_VERIFY_RETRY_MAX/);
  assert.match(retryHelper, /Handlescan_RaiseAlarm/);
  assert.ok(
    retryHelper.indexOf("HANDLESCAN_STAGE_RETRY_WAIT") < retryHelper.indexOf("Handlescan_RaiseAlarm"),
    "quick retry must be scheduled before raising the final handle-model alarm",
  );

  assert.match(scanA, /Handlescan_EnterRetryOrFail\s*\(\s*CHANNEL_A[\s\S]*Handlescan_MapVerifyStatusToAlarm\s*\(\s*verify_status\s*\)/);
  assert.match(scanB, /Handlescan_EnterRetryOrFail\s*\(\s*CHANNEL_B[\s\S]*Handlescan_MapVerifyStatusToAlarm\s*\(\s*verify_status\s*\)/);
  assert.match(scanA, /s_a_stage\s*==\s*HANDLESCAN_STAGE_RETRY_WAIT[\s\S]*HANDLESCAN_VERIFY_RETRY_DELAY_TICKS[\s\S]*s_a_stage\s*=\s*HANDLESCAN_STAGE_VERIFY/);
  assert.match(scanB, /s_b_stage\s*==\s*HANDLESCAN_STAGE_RETRY_WAIT[\s\S]*HANDLESCAN_VERIFY_RETRY_DELAY_TICKS[\s\S]*s_b_stage\s*=\s*HANDLESCAN_STAGE_VERIFY/);
  assert.match(scanA, /s_a_stage\s*==\s*HANDLESCAN_STAGE_VERIFY_FAIL[\s\S]*HANDLESCAN_VERIFY_ALARM_RETRY_TICKS[\s\S]*s_a_stage\s*=\s*HANDLESCAN_STAGE_VERIFY/);
  assert.match(scanB, /s_b_stage\s*==\s*HANDLESCAN_STAGE_VERIFY_FAIL[\s\S]*HANDLESCAN_VERIFY_ALARM_RETRY_TICKS[\s\S]*s_b_stage\s*=\s*HANDLESCAN_STAGE_VERIFY/);
  assert.match(scanA, /Handlescan_ClearChannelAlarm\s*\(\s*CHANNEL_A\s*,\s*s_a_last_alarm\s*\)[\s\S]*s_a_last_alarm\s*=\s*0U/);
  assert.match(scanB, /Handlescan_ClearChannelAlarm\s*\(\s*CHANNEL_B\s*,\s*s_b_last_alarm\s*\)[\s\S]*s_b_last_alarm\s*=\s*0U/);
});
