import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");
const softUartPath = path.join(repoRoot, "User", "Peripheral", "uart", "soft_uart.c");

function extractFunctionBody(source, functionName) {
  const signaturePattern = new RegExp(`static\\s+void\\s+${functionName}\\s*\\([^)]*\\)\\s*\\{`, "g");
  const matches = Array.from(source.matchAll(signaturePattern));
  assert.notEqual(matches.length, 0, `${functionName} should exist`);

  const lastMatch = matches.at(-1);
  const openIndex = lastMatch.index + lastMatch[0].lastIndexOf("{");
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

test("CS1237 压力帧只能更新压力识别字段，不能覆盖业务泵类型", () => {
  const source = readFileSync(softUartPath, "utf8");
  const updateBody = extractFunctionBody(source, "Cs1237_UpdatePumpMessage");

  assert.match(updateBody, /pressure_value\s*=\s*raw_cs1237/);
  assert.match(updateBody, /weight_x10\s*=\s*weight_x10/);
  assert.match(updateBody, /pressure_threshold\s*=\s*threshold_g/);
  assert.match(updateBody, /seq\s*=\s*frame\[5\]/);
  assert.match(updateBody, /online_flag\s*=\s*Cs1237_DeviceCodeValid\(device_code\)/);
  assert.match(updateBody, /losses_times/);

  assert.doesNotMatch(
    updateBody,
    /pump_message->type\s*=\s*device_code/,
    "DeviceCode 是霍尔识别码，不是 DRAWWATER/INJECTWATER/POURWATER；写入 type 会让 PUMPAehaviors() 间歇发送 0 速帧。"
  );
});
