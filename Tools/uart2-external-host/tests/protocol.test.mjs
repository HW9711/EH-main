import assert from "node:assert/strict";
import test from "node:test";

import {
  buildFrame,
  bytesToHex,
  crc16,
  hexToBytes,
  parseFrame,
  parseFrames,
  readBE16,
  toBE16
} from "../src/protocol.js";

const hex = hexToBytes;
const text = bytesToHex;

test("CRC16 与固件 Common_Crc16 样例一致", () => {
  const body = hex("02 00 18 01 FF FF 11 22 33 44 55 66 77 88");
  assert.equal(crc16(body), 0x531D);
});

test("组帧：申请外部控制", () => {
  const frame = buildFrame(0x02, 0x01, 0xFF, 0xFF, hex("11 22 33 44 55 66 77 88"));
  assert.equal(text(frame), "D7 CA F8 F1 02 00 18 01 FF FF 11 22 33 44 55 66 77 88 53 1D BF C6 BC C4");
});

test("组帧：退出外部控制", () => {
  const frame = buildFrame(0x02, 0xBB, 0xFF, 0xFF, []);
  assert.equal(text(frame), "D7 CA F8 F1 02 00 10 BB FF FF AC 74 BF C6 BC C4");
});

test("组帧：设置速度 3000", () => {
  const frame = buildFrame(0x02, 0x02, 0x01, 0xFF, toBE16(3000));
  assert.equal(text(frame), "D7 CA F8 F1 02 00 12 02 01 FF 0B B8 EE 8D BF C6 BC C4");
});

test("组帧：设置频率 20，兼容 1 字节写法", () => {
  const frame = buildFrame(0x02, 0x02, 0x02, 0xFF, [0x14]);
  assert.equal(text(frame), "D7 CA F8 F1 02 00 11 02 02 FF 14 44 25 BF C6 BC C4");
});

test("组帧：A 泵启动", () => {
  const frame = buildFrame({ funCode: 0x04, areaCode: 0x01, inforCode: 0xFF });
  assert.equal(text(frame), "D7 CA F8 F1 02 00 10 04 01 FF 28 05 BF C6 BC C4");
});

test("解析：离线心跳样例", () => {
  const heartbeat = hex("D7 CA F8 F1 01 00 17 AA FF FF FF FF FF 03 FF FF FF 5C 5D BF C6 BC C4");
  const result = parseFrame(heartbeat);
  assert.equal(result.valid, true);
  assert.equal(result.funCode, 0xAA);
  assert.equal(result.telemetry.selectedChannelText, "未选中");
  assert.equal(result.telemetry.runStatusText, "未接入");
  assert.equal(result.telemetry.pumpA.statusText, "离线");
  assert.match(result.summary, /心跳/);
});

test("解析：双手柄在线时必须保留当前选中工作手柄", () => {
  const heartbeat = buildFrame(0x01, 0xAA, 0xFF, 0xFF, hex("01 6B 05 01 6B 06 01 01 FF FF FF"));
  const result = parseFrame(heartbeat);
  assert.equal(result.valid, true);
  assert.equal(result.telemetry.handleA.online, true);
  assert.equal(result.telemetry.handleB.online, true);
  assert.equal(result.telemetry.handleA.rawType, "6B 05");
  assert.equal(result.telemetry.handleB.rawType, "6B 06");
  assert.equal(result.telemetry.handleA.typeName, "PXBA");
  assert.equal(result.telemetry.handleB.typeName, "PXBB");
  assert.equal(result.telemetry.selectedChannel, 0x01);
  assert.equal(result.telemetry.selectedChannelText, "A 通道");
  assert.doesNotMatch(result.telemetry.summary, /6B 05|6B 06/);
  assert.match(result.telemetry.summary, /PXBA/);
  assert.match(result.telemetry.summary, /当前选中：A 通道/);
});

test("解析：心跳里应透传 CS1237 原始值和最终重量", () => {
  const heartbeat = buildFrame(0x01, 0xAA, 0xFF, 0xFF, hex(
    "FF FF FF 03 FF 01 0E 00 64 C0 1D FE FF D2 04 00 00 C8 00 2A FF"
  ));
  const result = parseFrame(heartbeat);
  assert.equal(result.valid, true);
  assert.equal(result.telemetry.pumpA.online, true);
  assert.equal(result.telemetry.pumpA.type, 0x0E);
  assert.equal(result.telemetry.pumpA.speed, 100);
  assert.equal(result.telemetry.pumpA.pressureRaw, -123456);
  assert.equal(result.telemetry.pumpA.weightX10, 1234);
  assert.equal(result.telemetry.pumpA.weightText, "123.4 g");
  assert.equal(result.telemetry.pumpA.thresholdG, 200);
  assert.equal(result.telemetry.pumpA.pressureSeq, 0x2A);
  assert.equal(result.telemetry.pumpB.online, false);
  assert.match(result.telemetry.summary, /原始值 -123456/);
  assert.match(result.telemetry.summary, /重量 123.4 g/);
});

test("解析：用户曾给出的 A7 65 心跳 CRC 会标记错误", () => {
  const heartbeat = hex("D7 CA F8 F1 01 00 17 AA FF FF FF FF FF 03 FF FF FF A7 65 BF C6 BC C4");
  const result = parseFrame(heartbeat);
  assert.equal(result.valid, false);
  assert.equal(result.reason, "bad_crc");
  assert.equal(result.calculatedCrc, 0x5C5D);
});

test("解析：ACK 失败原因", () => {
  const ack = hex("D7 CA F8 F1 01 00 12 DD FF 02 01 01 22 3F BF C6 BC C4");
  const result = parseFrame(ack);
  assert.equal(result.valid, true);
  assert.equal(result.infoName, "运行值设置失败");
  assert.equal(result.decoded.reasonName, "长度错误");
  assert.match(result.payloadMeaning, /对象 01/);
});

test("解析：报警信息帧应暴露给故障弹窗", () => {
  const frame = buildFrame(0x01, 0x03, 0xFF, 0x05, [0x22]);
  const result = parseFrame(frame);
  assert.equal(result.valid, true);
  assert.equal(result.funCode, 0x03);
  assert.equal(result.infoCode, 0x05);
  assert.equal(result.alarm.active, true);
  assert.equal(result.alarm.codeText, "22");
  assert.equal(result.alarm.name, "手柄 EEPROM 认证失败");
  assert.match(result.payloadMeaning, /报警码 22/);
});

test("解析：报警码只有 0x00 表示无报警", () => {
  const clearFrame = buildFrame(0x01, 0x03, 0xFF, 0x05, [0x00]);
  const toolFrame = buildFrame(0x01, 0x03, 0xFF, 0x05, [0x02]);
  assert.equal(parseFrame(clearFrame).alarm.active, false);
  assert.equal(parseFrame(toolFrame).alarm.active, true);
});

test("解析：MCU 上传 EEPROM 单页有效数据", () => {
  const pageData = Array.from({ length: 30 }, (_, index) => index + 1);
  const frame = buildFrame(0x01, 0x01, 0x02, 0xFF, pageData);
  const result = parseFrame(frame);
  assert.equal(result.valid, true);
  assert.equal(result.tranCode, 0x01);
  assert.equal(result.funCode, 0x01);
  assert.equal(result.funName, "上传 EEPROM 单页有效数据");
  assert.equal(result.areaName, "业务区 Page3：刀具信息区");
  assert.equal(result.infoBytes.length, 30);
  assert.match(result.payloadMeaning, /EEPROM Page/);
  assert.match(result.summary, /上传 EEPROM/);
});

test("解析：CRC 错误", () => {
  const badCrc = hex("D7 CA F8 F1 02 00 12 02 01 FF 0B B8 EE 8C BF C6 BC C4");
  const result = parseFrame(badCrc);
  assert.equal(result.valid, false);
  assert.equal(result.reason, "bad_crc");
  assert.equal(result.calculatedCrc, 0xEE8D);
});

test("解析：帧尾错误", () => {
  const badTail = hex("D7 CA F8 F1 02 00 10 04 01 FF 28 05 BF C6 BC C5");
  const result = parseFrame(badTail);
  assert.equal(result.valid, false);
  assert.equal(result.reason, "bad_tail");
});

test("拆包：前导噪声和多帧粘包", () => {
  const first = buildFrame(0x02, 0x01, 0xFF, 0xFF, hex("11 22 33 44 55 66 77 88"));
  const second = buildFrame(0x02, 0x04, 0x01, 0xFF, []);
  const parsed = parseFrames([0x00, 0x55, ...first, ...second]);
  assert.equal(parsed.frames.length, 2);
  assert.equal(parsed.frames[0].offset, 2);
  assert.deepEqual(parsed.frames.map((frame) => frame.valid), [true, true]);
  assert.deepEqual(parsed.remaining, []);
});

test("HEX 输入：非法字符和半字节必须报错", () => {
  assert.deepEqual(hex("D7CAF8F1"), [0xD7, 0xCA, 0xF8, 0xF1]);
  assert.throws(() => hex("D7 CA ZZ F8 F1"), /非法|HEX/);
  assert.throws(() => hex("D7 CA F"), /完整|偶数/);
});

test("readBE16：短数据必须显式报错", () => {
  assert.equal(readBE16([0x12, 0x34]), 0x1234);
  assert.throws(() => readBE16([0x12]), RangeError);
  assert.throws(() => readBE16([0x12, 0x34], 1), RangeError);
});

test("单帧解析：完整帧后的尾随字节要给出残留提示", () => {
  const frame = buildFrame(0x02, 0x04, 0x01, 0xFF, []);
  const result = parseFrame([...frame, 0x99, 0x88]);
  assert.equal(result.valid, true);
  assert.deepEqual(result.trailingBytes, [0x99, 0x88]);
  assert.match(result.summary, /残留|粘包/);
  assert.equal(result.fields.at(-1).name, "尾随数据");
});

test("拆包：半包留在 remaining 中等待后续串口数据", () => {
  const frame = buildFrame(0x02, 0x04, 0x01, 0xFF, []);
  const halfPacket = frame.slice(0, frame.length - 3);
  const parsed = parseFrames(halfPacket);
  assert.equal(parsed.frames.length, 0);
  assert.deepEqual(parsed.remaining, halfPacket);
});
