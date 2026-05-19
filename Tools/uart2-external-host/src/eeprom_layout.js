import { bytesToHex, readBE16 } from "./protocol.js";

const PAGE_SIZE = 32;
const UPLOAD_DATA_SIZE = 30;

const BUSINESS_PAGE_MAP = new Map([
  [0x01, { page: 2, title: "Page2 手柄适配信息区" }],
  [0x02, { page: 3, title: "Page3 刀具信息区" }],
  [0x03, { page: 4, title: "Page4 初始值信息区" }],
  [0x04, { page: 5, title: "Page5 按键自定义功能区" }],
  [0x05, { page: 6, title: "Page6 多档位功能调节区" }],
  [0x06, { page: 8, title: "Page8 储存信息区" }],
  [0x07, { page: 9, title: "Page9 手柄客户可编辑区域" }],
  [0x08, { page: 11, title: "Page11 出厂信息区" }]
]);

const PAGE_LAYOUTS = new Map([
  [2, [
    field(0, 2, "手柄或刀具代号", "hex"),
    field(2, 2, "厂家信息编号", "u16"),
    field(4, 2, "地域识别码", "hex"),
    field(6, 1, "刀具适配机型", "u8"),
    field(7, 1, "是否重复性使用", "reuse"),
    reserved(8, 22)
  ]],
  [3, [
    field(0, 2, "刀具型号", "hex"),
    field(2, 2, "直径", "u16"),
    field(4, 2, "长度", "u16"),
    field(6, 2, "角度", "u16"),
    field(8, 1, "增速比", "u8"),
    field(9, 1, "减速比", "u8"),
    field(10, 1, "夹持范围", "u8"),
    reserved(11, 19)
  ]],
  [4, [
    field(0, 2, "默认流量（注水）", "u16"),
    field(2, 2, "默认速度", "u16"),
    field(4, 2, "最小速度", "u16"),
    field(6, 2, "最大速度", "u16"),
    field(8, 1, "默认运动方向", "direction"),
    field(9, 1, "默认频率", "u8"),
    field(10, 1, "正转报警值", "u8"),
    field(11, 1, "反转报警值", "u8"),
    field(12, 1, "往复转报警值", "u8"),
    field(13, 2, "可运行时间", "u16"),
    field(15, 2, "可运行次数", "u16"),
    reserved(17, 13)
  ]],
  [5, [
    reserved(0, 30, "该页暂时保留")
  ]],
  [6, [
    field(0, 1, "档位总数", "u8"),
    field(1, 2, "1档位速度", "u16"),
    field(3, 2, "1档位报警值", "u16"),
    field(5, 2, "2档位速度", "u16"),
    field(7, 2, "2档位报警值", "u16"),
    field(9, 2, "3档位速度", "u16"),
    field(11, 2, "3档位报警值", "u16"),
    reserved(13, 17)
  ]],
  [7, [
    reserved(0, 30, "Page7 为 Page6 多档位功能调节区续页，Page6 写满后继续使用")
  ]],
  [8, [
    field(0, 4, "使用时长", "u32"),
    field(4, 4, "使用次数", "u32"),
    field(8, 2, "常用速度", "u16"),
    field(10, 1, "常用频率", "u8"),
    field(11, 1, "常用流量（注水）", "u8"),
    field(12, 2, "最低报警电流", "u16"),
    field(14, 2, "最高报警电流", "u16"),
    field(16, 2, "平均报警电流", "u16"),
    field(18, 2, "报警次数", "u16"),
    field(20, 2, "最后一次使用的时长", "u16"),
    reserved(22, 8)
  ]],
  [9, [
    reserved(0, 30, "手柄客户可编辑区域，当前保留")
  ]],
  [10, [
    reserved(0, 30, "Page10 为 Page9 手柄客户可编辑区域续页，Page9 写满后继续使用")
  ]],
  [11, [
    field(0, 2, "刀具/手柄批次", "u16"),
    field(2, 2, "技术状态", "u16"),
    field(4, 2, "该批次刀具/手柄总数", "u16"),
    field(6, 2, "销售区域", "u16"),
    reserved(8, 22)
  ]]
]);

export function analyzeEepromFrame(frame) {
  if (!frame?.valid || frame.tranCode !== 0x01 || frame.funCode !== 0x01) {
    return null;
  }
  const bytes = Array.from(frame.infoBytes ?? []);
  if (bytes.length < UPLOAD_DATA_SIZE) {
    return null;
  }
  const pageData = bytes.slice(0, UPLOAD_DATA_SIZE);
  const pageInfo = resolvePageInfo(frame);
  const rows = buildHexRows(pageData, pageInfo.page);
  const fields = buildLayoutRows(pageInfo.page, pageData);

  return {
    page: pageInfo.page,
    title: pageInfo.title,
    source: pageInfo.source,
    startAddress: (pageInfo.page - 1) * PAGE_SIZE,
    dataBytes: pageData,
    rawHex: bytesToHex(pageData),
    rows,
    fields,
    note: "固件上传前 30 字节有效数据；每页最后 2 字节页校验由 MCU 读页时校验，当前回包不上传。"
  };
}

function resolvePageInfo(frame) {
  if (frame.areaCode === 0xFF) {
    const pageCode = frame.infoCode;
    const page = pageCode >= 2 && pageCode <= 128 ? pageCode : pageCode + 11;
    if (page === 7 || page === 10) {
      return {
        page,
        title: `Page${page} ${page === 7 ? "多档位功能调节续页" : "手柄客户可编辑续页"}`,
        source: `业务续页 InforCode ${hexByte(pageCode)}`
      };
    }
    return {
      page,
      title: `Page${page} 导航数据区域`,
      source: `导航页 InforCode ${hexByte(pageCode)}`
    };
  }
  const mapped = BUSINESS_PAGE_MAP.get(frame.areaCode);
  if (mapped) {
    return {
      ...mapped,
      source: `业务 AreaCode ${hexByte(frame.areaCode)}`
    };
  }
  return {
    page: frame.areaCode,
    title: `未知业务 EEPROM Page ${hexByte(frame.areaCode)}`,
    source: `业务 AreaCode ${hexByte(frame.areaCode)}`
  };
}

function buildHexRows(data, page) {
  const rows = [];
  for (let offset = 0; offset < data.length; offset += 8) {
    const slice = data.slice(offset, offset + 8);
    rows.push({
      pageText: `Page${page}：`,
      offset,
      addressText: hex16(offset),
      bytes: slice.map((byte, index) => ({
        offset: offset + index,
        text: hexByte(byte)
      }))
    });
  }
  return rows;
}

function buildLayoutRows(page, data) {
  const layout = PAGE_LAYOUTS.get(page);
  if (!layout) {
    return [{
      offsetText: "00-1D",
      name: page >= 12 && page <= 128 ? "导航数据" : "未定义布局",
      hex: bytesToHex(data),
      value: page >= 12 && page <= 128 ? "导航区原始数据，布局说明未细分字段" : "当前页在布局说明中未给出字段定义"
    }];
  }
  return layout.map((item) => ({
    offsetText: formatRange(item.offset, item.length),
    name: item.name,
    hex: bytesToHex(data.slice(item.offset, item.offset + item.length)),
    value: decodeFieldValue(item, data)
  }));
}

function decodeFieldValue(item, data) {
  const slice = data.slice(item.offset, item.offset + item.length);
  if (slice.length < item.length) {
    return "数据不足";
  }
  if (item.kind === "reserved") {
    return item.note ?? "保留区域";
  }
  if (item.kind === "hex") {
    return bytesToHex(slice);
  }
  if (item.kind === "u8") {
    return String(slice[0]);
  }
  if (item.kind === "u16") {
    return String(readBE16(slice, 0));
  }
  if (item.kind === "u32") {
    return String(((slice[0] * 0x1000000) + (slice[1] << 16) + (slice[2] << 8) + slice[3]) >>> 0);
  }
  if (item.kind === "reuse") {
    return slice[0] === 0x00 ? "否" : slice[0] === 0x01 ? "是" : `未知 ${hexByte(slice[0])}`;
  }
  if (item.kind === "direction") {
    return new Map([[0x01, "正转"], [0x02, "反转"], [0x03, "往复"]]).get(slice[0]) ?? `未知 ${hexByte(slice[0])}`;
  }
  return bytesToHex(slice);
}

function field(offset, length, name, kind) {
  return { offset, length, name, kind };
}

function reserved(offset, length, note = "中间区域暂时保留") {
  return { offset, length, name: "保留区", kind: "reserved", note };
}

function formatRange(offset, length) {
  if (length === 1) {
    return hex16(offset);
  }
  return `${hex16(offset)}-${hex16(offset + length - 1)}`;
}

function hexByte(value) {
  return Number(value ?? 0).toString(16).toUpperCase().padStart(2, "0");
}

function hex16(value) {
  return `0x${Number(value ?? 0).toString(16).toUpperCase().padStart(2, "0")}`;
}
