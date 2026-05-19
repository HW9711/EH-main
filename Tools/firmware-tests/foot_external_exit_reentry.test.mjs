import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");
const footPath = path.join(repoRoot, "User", "Application", "Beep", "sscFOOT.c");

function readFootSource() {
  return readFileSync(footPath, "utf8");
}

test("退出外部控制后脚踏第一次踩下应切回脚踏模式，不能锁死为 0x02 报警", () => {
  const source = readFootSource();

  assert.match(source, /Foot_EnsureFootControlMode/);
  assert.match(source, /WorkMessage\.drivetype_work\s*=\s*JTWORK/);
  assert.match(source, /MemoryMsgA\.drive_type\s*=\s*JTWORK/);
  assert.match(source, /MemoryMsgB\.drive_type\s*=\s*JTWORK/);
  assert.match(source, /Foot_EnsureFootControlMode\(\)\s*==\s*false/);
  assert.doesNotMatch(
    source,
    /WorkMessage\.drivetype_work\s*!=\s*JTWORK[\s\S]{0,500}WorkMessage\.alarm_value\s*=\s*2U/,
    "脚踏启动时如果互斥锁已空闲，应先切回 JTWORK；直接置 0x02 会导致退出上位机控制后蜂鸣器一直响。"
  );
});
