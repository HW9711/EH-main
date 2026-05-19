import assert from "node:assert/strict";
import test from "node:test";

import { analyzeEepromFrame } from "../src/eeprom_layout.js";
import { buildFrame, parseFrame } from "../src/protocol.js";

test("EEPROM 布局：业务 Area 02 应解释为 Page3 刀具信息区", () => {
  const pageData = [
    0x7C, 0x01,
    0x00, 0x28,
    0x00, 0x96,
    0x00, 0x2D,
    0x02,
    0x04,
    0x09,
    ...Array(19).fill(0)
  ];
  const frame = parseFrame(buildFrame(0x01, 0x01, 0x02, 0xFF, pageData));
  const view = analyzeEepromFrame(frame);

  assert.ok(view);
  assert.equal(view.page, 3);
  assert.equal(view.title, "Page3 刀具信息区");
  assert.equal(view.source, "业务 AreaCode 02");
  assert.equal(view.startAddress, 64);
  assert.equal(view.dataBytes.length, 30);
  assert.equal(view.rows.length, 4);
  assert.equal(view.rows[0].pageText, "Page3：");
  assert.equal(view.fields[0].name, "刀具型号");
  assert.equal(view.fields[1].value, "40");
  assert.equal(view.fields[2].name, "长度");
  assert.equal(view.fields[2].value, "150");
  assert.match(view.note, /前 30 字节/);
});

test("EEPROM 布局：导航上传帧应按 InforCode 推导导航 Page", () => {
  const pageData = Array.from({ length: 30 }, (_, index) => index);
  const frame = parseFrame(buildFrame(0x01, 0x01, 0xFF, 0x0C, pageData));
  const view = analyzeEepromFrame(frame);

  assert.ok(view);
  assert.equal(view.page, 12);
  assert.equal(view.title, "Page12 导航数据区域");
  assert.equal(view.source, "导航页 InforCode 0C");
  assert.equal(view.rows[0].pageText, "Page12：");
  assert.equal(view.fields[0].name, "导航数据");
});

test("EEPROM 布局：导航 Page128 也应保留 Page 前缀和原始数据兜底解释", () => {
  const pageData = Array.from({ length: 30 }, (_, index) => 0x80 + index);
  const frame = parseFrame(buildFrame(0x01, 0x01, 0xFF, 0x80, pageData));
  const view = analyzeEepromFrame(frame);

  assert.ok(view);
  assert.equal(view.page, 128);
  assert.equal(view.title, "Page128 导航数据区域");
  assert.equal(view.source, "导航页 InforCode 80");
  assert.equal(view.startAddress, 4064);
  assert.equal(view.rows[0].pageText, "Page128：");
  assert.equal(view.fields[0].name, "导航数据");
});

test("EEPROM 布局：Page7/Page10 续页应按真实页号显示", () => {
  const pageData = Array.from({ length: 30 }, (_, index) => 0x40 + index);
  const page7Frame = parseFrame(buildFrame(0x01, 0x01, 0xFF, 0x07, pageData));
  const page10Frame = parseFrame(buildFrame(0x01, 0x01, 0xFF, 0x0A, pageData));
  const page7 = analyzeEepromFrame(page7Frame);
  const page10 = analyzeEepromFrame(page10Frame);

  assert.ok(page7);
  assert.equal(page7.page, 7);
  assert.match(page7.title, /续页/);
  assert.equal(page7.source, "业务续页 InforCode 07");
  assert.ok(page10);
  assert.equal(page10.page, 10);
  assert.match(page10.title, /续页/);
  assert.equal(page10.source, "业务续页 InforCode 0A");
});
