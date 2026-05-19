import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");

function readSource(...parts) {
  return readFileSync(path.join(repoRoot, ...parts), "utf8");
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

  throw new Error(`${functionName} body should close`);
}

test("logical pump A/B outputs are swapped to match the current physical left/right pump positions", () => {
  const pumpHeader = readSource("User", "Application", "include", "pump.h");
  const pumpA = readSource("User", "Application", "Beep", "sscPUMPA.c");
  const pumpB = readSource("User", "Application", "Beep", "sscPUMPB.c");
  const legacyPump = readSource("User", "Application", "Pump", "pump.c");

  const pumpABody = extractFunctionBody(pumpA, "Pump_SetSpeedS_A");
  const pumpBBody = extractFunctionBody(pumpB, "Pump_SetSpeedS_B");
  const legacyABody = extractFunctionBody(legacyPump, "Pump_SetSpeed_A");
  const legacyBBody = extractFunctionBody(legacyPump, "Pump_SetSpeed_B");

  assert.match(pumpHeader, /#define PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE 1U/);
  assert.match(pumpABody, /PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE[\s\S]*Uart7_SendPacket/);
  assert.match(pumpBBody, /PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE[\s\S]*Uart5_SendPacket/);
  assert.match(legacyABody, /PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE[\s\S]*Uart7_SendPacket/);
  assert.match(legacyBBody, /PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE[\s\S]*Uart5_SendPacket/);
});
