export const FRAME_HEAD = Object.freeze([0xD7, 0xCA, 0xF8, 0xF1]);
export const FRAME_TAIL = Object.freeze([0xBF, 0xC6, 0xBC, 0xC4]);
export const FRAME_FIXED_SIZE = 16;
export const MAX_FRAME_SIZE = 150;
export const TRAN_UP = 0x01;
export const TRAN_DOWN = 0x02;

const FUN_NAMES = new Map([
  [0x01, "申请外部控制"],
  [0x02, "静态值设置"],
  [0x03, "切换值设置"],
  [0x04, "控制命令"],
  [0x05, "读取业务 EEPROM 单页"],
  [0x06, "读取业务 EEPROM 整区"],
  [0x07, "写入业务 EEPROM 单页"],
  [0x08, "读取导航 EEPROM 单页"],
  [0x09, "读取导航 EEPROM 整区"],
  [0x0A, "写入导航 EEPROM 单页"],
  [0xAA, "心跳"],
  [0xBB, "退出外部控制"],
  [0xDD, "MCU 应答"],
  [0xFA, "权限开放"]
]);

const UPLOAD_FUN_NAMES = new Map([
  [0x01, "上传 EEPROM 单页有效数据"],
  [0x02, "上传主机设置内容"],
  [0x03, "上传主机运行内容"],
  [0xAA, "心跳"],
  [0xBB, "上传主机主动退出外部控制"],
  [0xCC, "上传接插切换内容"],
  [0xDD, "MCU 应答"],
  [0xEE, "上传主机控制内容"]
]);

const ACK_NAMES = new Map([
  [0x01, "运行值设置成功"],
  [0x02, "运行值设置失败"],
  [0x03, "控制成功"],
  [0x04, "控制失败"],
  [0x05, "EEPROM 读写成功"],
  [0x06, "EEPROM 读写失败"],
  [0xAA, "外部控制申请成功"],
  [0xAB, "外部控制申请失败"],
  [0xBB, "注册码校验失败"],
  [0xFE, "权限开放成功"],
  [0xFF, "权限开放失败"]
]);

const REASON_NAMES = new Map([
  [0x01, "长度错误"],
  [0x02, "AreaCode 或参数范围错误"],
  [0x03, "当前无 A/B 通道"],
  [0x04, "设备未配置、EEPROM 读写失败或泵未识别"],
  [0x05, "V1 不支持该命令"],
  [0x06, "Busy，未申请外部控制、运行中或报警中"]
]);

const RUNNING_INFO_NAMES = new Map([
  [0x01, "手柄/刀具实时速度"],
  [0x02, "手柄/刀具实时电流"],
  [0x03, "实时 A 泵压力"],
  [0x04, "实时 B 泵压力"],
  [0x05, "报警信息"]
]);

const ALARM_NAMES = new Map([
  [0x00, "无报警"],
  [0x01, "手柄未连接，请连接手柄与刀具"],
  [0x02, "刀具未连接，请连接刀具"],
  [0x03, "电机相位错误，请联系售后"],
  [0x04, "电机霍尔错误"],
  [0x05, "电机过载，请松开脚踏后再运行，或检查刀具是否卡住"],
  [0x06, "电机霍尔信号线脱落"],
  [0x07, "脚踏存储值读取错误"],
  [0x08, "系统供电电压不稳定"],
  [0x09, "设备加密 UID 错误"],
  [0x0A, "手柄型号错误，请联系售后"],
  [0x0B, "驱动板故障，请联系售后"],
  [0x0C, "手控报警，脚踏运行"],
  [0x0D, "手控运行脚踏报警"],
  [0x22, "手柄 EEPROM 认证失败"],
  [0x23, "手柄信息区读取失败或手柄类型无法识别"]
]);

const SETTING_AREAS = new Map([
  [0x01, "设置当前手柄速度（运行中可动态生效）"],
  [0x02, "设置当前手柄频率"],
  [0x03, "设置 A 泵速度"],
  [0x04, "设置 B 泵速度"]
]);

const SWITCH_AREAS = new Map([
  [0x01, "切换到 A 通道"],
  [0x02, "切换到 B 通道"],
  [0x03, "切换运动方向"],
  [0x04, "切换控制模式"],
  [0x05, "切换工具类型"]
]);

const CONTROL_AREAS = new Map([
  [0x01, "A 泵启动"],
  [0x02, "A 泵停止"],
  [0x03, "B 泵启动"],
  [0x04, "B 泵停止"],
  [0x05, "当前手柄启动"],
  [0x06, "当前手柄停止"],
  [0x07, "开口定位左"],
  [0x08, "开口定位右"],
  [0xFF, "紧急刹车"]
]);

const BUSINESS_AREAS = new Map([
  [0x01, "业务区 Page2：识别信息区"],
  [0x02, "业务区 Page3：刀具信息区"],
  [0x03, "业务区 Page4：初始值信息区"],
  [0x04, "业务区 Page5：按键自定义区"],
  [0x05, "业务区 Page6：多档位调节区"],
  [0x06, "业务区 Page8：运行信息区"],
  [0x07, "业务区 Page9：外部编辑区"],
  [0x08, "业务区 Page11：出厂信息区"]
]);

const HAND_TYPES = new Map([
  ["6B 01", "TMBB"],
  ["6B 02", "TMBA"],
  ["6B 03", "EMBA"],
  ["6B 04", "EMBB"],
  ["6B 05", "PXBA"],
  ["6B 06", "PXBB"]
]);

export function hexToBytes(text) {
  const source = String(text ?? "").trim();
  if (source === "") {
    return [];
  }
  // 允许现场粘贴空格分隔、逗号分隔或连续 HEX，同时禁止静默吞掉非法字符。
  const normalized = source
    .replace(/0x/gi, "")
    .replace(/[，,;；\r\n\t]+/g, " ");
  if (/[^0-9a-fA-F\s]/.test(normalized)) {
    throw new Error("HEX 输入包含非法字符，只允许 00..FF 字节");
  }
  const compact = normalized.replace(/\s+/g, "");
  if (compact.length % 2 !== 0) {
    throw new Error("HEX 输入不完整，字节必须由偶数个十六进制字符组成");
  }
  const tokens = compact.match(/[0-9a-fA-F]{2}/g) ?? [];
  return tokens.map((token) => Number.parseInt(token, 16));
}

export function bytesToHex(data) {
  return normalizeBytes(data).map((byte) => hexByte(byte)).join(" ");
}

export function toBE16(value) {
  const checked = requireRange(value, 0, 0xFFFF, "16 位字段");
  return [(checked >> 8) & 0xFF, checked & 0xFF];
}

export function readBE16(data, offset = 0) {
  const bytes = normalizeBytes(data);
  // 协议字段是大端 16 位，调用方传入半包时必须显式暴露，而不是把 undefined 当成 0。
  const checkedOffset = requireRange(offset, 0, Number.MAX_SAFE_INTEGER, "readBE16 offset");
  if (bytes.length < checkedOffset + 2) {
    throw new RangeError(`readBE16 从 offset ${checkedOffset} 开始至少需要 2 字节，当前只有 ${bytes.length} 字节`);
  }
  return ((bytes[checkedOffset] << 8) | bytes[checkedOffset + 1]) >>> 0;
}

function readU16LE(data, offset = 0) {
  const bytes = normalizeBytes(data);
  // CS1237 下位机协议使用 little-endian，这里只给心跳压力扩展字段内部使用。
  const checkedOffset = requireRange(offset, 0, Number.MAX_SAFE_INTEGER, "readU16LE offset");
  if (bytes.length < checkedOffset + 2) {
    throw new RangeError(`readU16LE 从 offset ${checkedOffset} 开始至少需要 2 字节，当前只有 ${bytes.length} 字节`);
  }
  return ((bytes[checkedOffset + 1] << 8) | bytes[checkedOffset]) >>> 0;
}

function readU32LE(data, offset = 0) {
  const bytes = normalizeBytes(data);
  // RawCs1237 和 WeightX10 直接沿用 cs1237_uart_protocol.md 的 4 字节 little-endian 编码。
  const checkedOffset = requireRange(offset, 0, Number.MAX_SAFE_INTEGER, "readU32LE offset");
  if (bytes.length < checkedOffset + 4) {
    throw new RangeError(`readU32LE 从 offset ${checkedOffset} 开始至少需要 4 字节，当前只有 ${bytes.length} 字节`);
  }
  return (
    bytes[checkedOffset] |
    (bytes[checkedOffset + 1] << 8) |
    (bytes[checkedOffset + 2] << 16) |
    (bytes[checkedOffset + 3] << 24)
  ) >>> 0;
}

function readI32LE(data, offset = 0) {
  // CS1237 原始值按 int32_t 发送，最高位为 1 时需要恢复成 JS 负数。
  const value = readU32LE(data, offset);
  return value > 0x7FFFFFFF ? value - 0x100000000 : value;
}

export const bytes = Object.freeze({
  fromHex: hexToBytes,
  toHex: bytesToHex,
  readU16BE: readBE16,
  writeU16BE: toBE16
});

export function crc16(data) {
  let crc = 0xFFFF;
  for (const byte of normalizeBytes(data)) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x0001) !== 0 ? ((crc >>> 1) ^ 0xA001) : (crc >>> 1);
      crc &= 0xFFFF;
    }
  }
  return crc;
}

export function buildFrame(...args) {
  const options = normalizeBuildArgs(args);
  const tranCode = requireByte(options.tranCode ?? TRAN_DOWN, "TranCode");
  const funCode = requireByte(options.funCode, "FunCode");
  const areaCode = requireByte(options.areaCode ?? 0xFF, "AreaCode");
  const infoCode = requireByte(options.infoCode ?? options.inforCode ?? 0xFF, "InforCode");
  const infoArea = normalizeBytes(options.infoArea ?? options.infoBytes ?? []);
  const length = FRAME_FIXED_SIZE + infoArea.length;

  if (length > MAX_FRAME_SIZE) {
    throw new RangeError(`整帧长度 ${length} 超过 ${MAX_FRAME_SIZE}`);
  }

  const body = [tranCode, ...toBE16(length), funCode, areaCode, infoCode, ...infoArea];
  const crc = crc16(body);
  return [...FRAME_HEAD, ...body, ...toBE16(crc), ...FRAME_TAIL];
}

export function parseFrame(input) {
  const data = normalizeBytes(input);
  const base = {
    rawBytes: data,
    valid: false,
    checks: {
      headOk: matchAt(data, FRAME_HEAD, 0),
      lengthOk: false,
      crcOk: false,
      tailOk: false
    },
    fields: []
  };

  if (data.length < FRAME_FIXED_SIZE) {
    return finalizeFrame({
      ...base,
      reason: "incomplete",
      summary: "数据不足，未达到最短 16 字节帧"
    });
  }

  if (!base.checks.headOk) {
    return finalizeFrame({
      ...base,
      reason: "bad_head",
      summary: "帧头错误，不是 D7 CA F8 F1"
    });
  }

  const length = readBE16(data, 5);
  base.checks.lengthOk = length >= FRAME_FIXED_SIZE && length <= MAX_FRAME_SIZE && data.length >= length;

  if (length < FRAME_FIXED_SIZE || length > MAX_FRAME_SIZE) {
    return finalizeFrame({
      ...base,
      length,
      reason: "bad_length",
      summary: `Length=${length} 超出允许范围`
    });
  }

  if (data.length < length) {
    return finalizeFrame({
      ...base,
      length,
      reason: "incomplete",
      summary: `数据未收完整，声明 ${length} 字节，当前 ${data.length} 字节`
    });
  }

  const frame = data.slice(0, length);
  const infoLen = length - FRAME_FIXED_SIZE;
  const crcOffset = 10 + infoLen;
  const receivedCrc = readBE16(frame, crcOffset);
  const calculatedCrc = crc16(frame.slice(4, crcOffset));
  const tailOk = matchAt(frame, FRAME_TAIL, length - 4);
  const crcOk = receivedCrc === calculatedCrc;

  const decoded = decodeFields(frame, infoLen, receivedCrc, calculatedCrc, crcOk && tailOk);
  const trailingBytes = data.slice(length);
  let summary = makeSummary(decoded, crcOk, tailOk);
  if (trailingBytes.length > 0) {
    // parseFrame 只解释第一帧；尾随字节作为粘包/残留提示留给协议解析窗口展示。
    decoded.fields.push({
      name: "尾随数据",
      hex: bytesToHex(trailingBytes),
      meaning: `首帧后还有 ${trailingBytes.length} 字节，可能是粘包或残留，请用拆包解析继续查看`
    });
    summary = `${summary}；后续还有 ${trailingBytes.length} 字节残留`;
  }
  return finalizeFrame({
    ...decoded,
    trailingBytes,
    valid: crcOk && tailOk,
    checks: {
      headOk: true,
      lengthOk: true,
      crcOk,
      tailOk
    },
    reason: !tailOk ? "bad_tail" : !crcOk ? "bad_crc" : "ok",
    summary
  });
}

export function parseFrames(input) {
  const data = normalizeBytes(input);
  const frames = [];
  let offset = 0;

  while (offset <= data.length - FRAME_HEAD.length) {
    if (!matchAt(data, FRAME_HEAD, offset)) {
      offset += 1;
      continue;
    }

    if (data.length - offset < FRAME_FIXED_SIZE) {
      break;
    }

    const length = readBE16(data, offset + 5);
    if (length < FRAME_FIXED_SIZE || length > MAX_FRAME_SIZE) {
      const bad = parseFrame(data.slice(offset, offset + FRAME_FIXED_SIZE));
      frames.push({ ...bad, offset });
      offset += 1;
      continue;
    }

    if (data.length - offset < length) {
      break;
    }

    const parsed = parseFrame(data.slice(offset, offset + length));
    frames.push({ ...parsed, offset });
    offset += length;
  }

  const result = { frames, remaining: data.slice(offset) };
  result.length = frames.length;
  frames.forEach((frame, index) => {
    result[index] = frame;
  });
  return result;
}

function decodeFields(frame, infoLen, receivedCrc, calculatedCrc, isValid) {
  const tranCode = frame[4];
  const length = readBE16(frame, 5);
  const funCode = frame[7];
  const areaCode = frame[8];
  const infoCode = frame[9];
  const infoBytes = frame.slice(10, 10 + infoLen);
  const direction = tranCode === TRAN_UP ? "MCU 上传/应答" : tranCode === TRAN_DOWN ? "上位机下发" : `未知方向 ${hexByte(tranCode)}`;
  const funName = describeFun(tranCode, funCode);
  const areaName = describeArea(tranCode, funCode, areaCode, infoCode);
  const infoName = describeInfo(tranCode, funCode, infoCode);
  const payloadMeaning = describePayload(tranCode, funCode, areaCode, infoCode, infoBytes);
  const telemetry = funCode === 0xAA ? decodeHeartbeat(infoBytes) : null;
  const alarm = decodeAlarmInfo(tranCode, funCode, infoCode, infoBytes);

  return {
    rawBytes: frame,
    tranCode,
    length,
    funCode,
    areaCode,
    infoCode,
    infoBytes,
    infoArea: infoBytes,
    infoLength: infoLen,
    crc: receivedCrc,
    calculatedCrc,
    direction,
    funName,
    areaName,
    infoName,
    payloadMeaning,
    telemetry,
    alarm,
    decoded: {
      tranName: direction,
      funName,
      areaName,
      infoName,
      ackName: funCode === 0xDD ? ACK_NAMES.get(infoCode) ?? `未知 ACK ${hexByte(infoCode)}` : null,
      reasonName: decodeReasonName(funCode, infoCode, infoBytes),
      payloadText: payloadMeaning,
      alarmName: alarm?.name ?? null
    },
    fields: [
      { name: "帧头", hex: bytesToHex(frame.slice(0, 4)), meaning: "固定帧头" },
      { name: "TranCode", hex: hexByte(tranCode), meaning: direction },
      { name: "Length", hex: bytesToHex(frame.slice(5, 7)), meaning: `整帧 ${length} 字节` },
      { name: "FunCode", hex: hexByte(funCode), meaning: funName },
      { name: "AreaCode", hex: hexByte(areaCode), meaning: areaName },
      { name: "InforCode", hex: hexByte(infoCode), meaning: infoName },
      { name: "InforArea", hex: bytesToHex(infoBytes), meaning: payloadMeaning },
      { name: "CRC16", hex: hex16(receivedCrc), meaning: isValid ? "CRC 正确" : `重算应为 ${hex16(calculatedCrc)}` },
      { name: "帧尾", hex: bytesToHex(frame.slice(-4)), meaning: "固定帧尾" }
    ]
  };
}

function describeFun(tranCode, funCode) {
  if (tranCode === TRAN_UP) {
    return UPLOAD_FUN_NAMES.get(funCode) ?? `未知上传功能码 ${hexByte(funCode)}`;
  }
  return FUN_NAMES.get(funCode) ?? `未知功能码 ${hexByte(funCode)}`;
}

function describeArea(tranCode, funCode, areaCode, infoCode) {
  if (tranCode === TRAN_UP && funCode === 0x01) {
    if (areaCode === 0xFF) {
      return `导航 EEPROM Page ${infoCode >= 12 ? infoCode : infoCode + 11}`;
    }
    return BUSINESS_AREAS.get(areaCode) ?? `未知业务 EEPROM 区 ${hexByte(areaCode)}`;
  }
  if (funCode === 0x01 && infoCode === 0xFF) {
    return "外部控制注册码";
  }
  if (funCode === 0x02) {
    return SETTING_AREAS.get(areaCode) ?? `未知设置项 ${hexByte(areaCode)}`;
  }
  if (funCode === 0x03) {
    return SWITCH_AREAS.get(areaCode) ?? `未知切换项 ${hexByte(areaCode)}`;
  }
  if (funCode === 0x04) {
    return CONTROL_AREAS.get(areaCode) ?? `未知控制项 ${hexByte(areaCode)}`;
  }
  if (funCode === 0x05 || funCode === 0x07) {
    return BUSINESS_AREAS.get(areaCode) ?? `未知业务 EEPROM 区 ${hexByte(areaCode)}`;
  }
  if (funCode === 0x08 || funCode === 0x0A) {
    return `导航 EEPROM Page ${areaCode}`;
  }
  if (funCode === 0x01 || funCode === 0xFA || funCode === 0xAA || funCode === 0xDD || funCode === 0xBB) {
    return areaCode === 0xFF ? "无区域码" : `区域 ${hexByte(areaCode)}`;
  }
  return `区域 ${hexByte(areaCode)}`;
}

function describeInfo(tranCode, funCode, infoCode) {
  if (funCode === 0xDD) {
    return ACK_NAMES.get(infoCode) ?? `未知 ACK ${hexByte(infoCode)}`;
  }
  if (tranCode === TRAN_UP && funCode === 0x03) {
    return RUNNING_INFO_NAMES.get(infoCode) ?? `运行信息码 ${hexByte(infoCode)}`;
  }
  return infoCode === 0xFF ? "无信息码" : `信息码 ${hexByte(infoCode)}`;
}

function describePayload(tranCode, funCode, areaCode, infoCode, infoBytes) {
  if (funCode === 0xAA) {
    return decodeHeartbeat(infoBytes).summary;
  }
  if (funCode === 0xDD) {
    return describeAckPayload(infoCode, infoBytes);
  }
  if (funCode === 0x01) {
    if (infoBytes.length === 30) {
      return `上传 EEPROM Page 有效数据 30 字节，页尾 2 字节校验未上传`;
    }
    return `注册码 ${infoBytes.length} 字节`;
  }
  if (funCode === 0xFA) {
    return `权限码 ${infoBytes.length} 字节`;
  }
  if (funCode === 0x02) {
    if (areaCode === 0x02 && infoBytes.length === 1) {
      return `频率 ${infoBytes[0]}`;
    }
    if (infoBytes.length >= 2) {
      return `数值 ${readBE16(infoBytes, 0)}`;
    }
    return "无参数或参数长度不足";
  }
  if (tranCode === TRAN_UP && funCode === 0x03) {
    if (infoCode === 0x05) {
      const alarm = decodeAlarmInfo(tranCode, funCode, infoCode, infoBytes);
      return alarm ? `报警码 ${hexByte(alarm.code)}：${alarm.name}` : "报警信息字段缺失";
    }
    if (infoBytes.length >= 2) {
      return `${RUNNING_INFO_NAMES.get(infoCode) ?? "运行信息"} ${readBE16(infoBytes, 0)}`;
    }
    return infoBytes.length > 0 ? bytesToHex(infoBytes) : "运行信息无 InforArea";
  }
  if (funCode === 0x03) {
    return describeSwitchValue(areaCode, infoBytes[0]);
  }
  if (funCode === 0x04) {
    return "控制命令无 InforArea";
  }
  if (funCode === 0x07 || funCode === 0x0A) {
    return `写入 ${infoBytes.length} 字节 EEPROM 有效数据，固件要求 30 字节`;
  }
  if (funCode === 0x05 || funCode === 0x08) {
    return infoBytes.length === 0 ? "读取命令无 InforArea" : `上传 EEPROM 数据 ${infoBytes.length} 字节`;
  }
  return infoBytes.length > 0 ? bytesToHex(infoBytes) : "无 InforArea";
}

function describeSwitchValue(areaCode, value) {
  if (value === undefined) {
    return "切换命令无参数";
  }
  if (areaCode === 0x03) {
    return new Map([[0x01, "正转"], [0x02, "反转"], [0x03, "往复"]]).get(value) ?? `未知方向 ${hexByte(value)}`;
  }
  if (areaCode === 0x04) {
    return new Map([[0x01, "脚踏"], [0x02, "手控"], [0x03, "外部控制"]]).get(value) ?? `未知控制模式 ${hexByte(value)}`;
  }
  if (areaCode === 0x05) {
    return new Map([[0x01, "刨头"], [0x02, "磨头"]]).get(value) ?? `未知工具类型 ${hexByte(value)}`;
  }
  return `附加值 ${hexByte(value)}`;
}

function decodeAlarmInfo(tranCode, funCode, infoCode, infoBytes) {
  if (tranCode !== TRAN_UP || funCode !== 0x03 || infoCode !== 0x05 || infoBytes.length < 1) {
    return null;
  }
  const code = infoBytes[0];
  return {
    active: code !== 0x00,
    code,
    codeText: hexByte(code),
    name: ALARM_NAMES.get(code) ?? `未知报警 ${hexByte(code)}`,
    rawBytes: Array.from(infoBytes)
  };
}

function describeAckPayload(infoCode, infoBytes) {
  const ack = ACK_NAMES.get(infoCode) ?? `未知 ACK ${hexByte(infoCode)}`;
  if (infoBytes.length === 0) {
    return `${ack}，无附加数据`;
  }
  if ([0x02, 0x04, 0x06, 0xAB, 0xBB, 0xFF].includes(infoCode)) {
    const target = infoBytes[0];
    const reason = infoBytes[infoBytes.length - 1];
    return `${ack}，对象 ${hexByte(target)}，原因 ${REASON_NAMES.get(reason) ?? hexByte(reason)}`;
  }
  if (infoBytes.length >= 3 && infoCode === 0x01) {
    return `${ack}，对象 ${hexByte(infoBytes[0])}，回显值 ${readBE16(infoBytes, 1)}`;
  }
  return `${ack}，回显 ${bytesToHex(infoBytes)}`;
}

function decodeReasonName(funCode, infoCode, infoBytes) {
  if (funCode !== 0xDD || infoBytes.length === 0) {
    return null;
  }
  if (![0x02, 0x04, 0x06, 0xAB, 0xBB, 0xFF].includes(infoCode)) {
    return null;
  }
  return REASON_NAMES.get(infoBytes[infoBytes.length - 1]) ?? null;
}

function decodeHeartbeat(infoBytes) {
  const cursor = { index: 0 };
  const fields = [];
  const a = readStatusWithOptionalHandle(infoBytes, cursor, "A 手柄", fields);
  const b = readStatusWithOptionalHandle(infoBytes, cursor, "B 手柄", fields);
  const selected = readByte(infoBytes, cursor);
  const runStatus = readByte(infoBytes, cursor);
  const telemetry = {
    handleA: a,
    handleB: b,
    selectedChannel: selected,
    selectedChannelText: selected === 0x01 ? "A 通道" : selected === 0x02 ? "B 通道" : "未选中",
    runStatus,
    runStatusText: runStatus === 0x01 ? "待机" : runStatus === 0x02 ? "运行中" : runStatus === 0x03 ? "未接入" : "未知"
  };
  fields.push(`当前选中：${telemetry.selectedChannelText}`);
  fields.push(`运行状态：${telemetry.runStatusText}`);

  if (runStatus === 0x02) {
    telemetry.speed = readWord(infoBytes, cursor);
    telemetry.currentRaw = readWord(infoBytes, cursor);
    telemetry.current = telemetry.currentRaw / 100;
    fields.push(`速度：${telemetry.speed}`);
    fields.push(`电流：${formatMotorCurrent(telemetry.current)}`);
  }

  const foot = readByte(infoBytes, cursor);
  telemetry.footPedal = statusText(foot);
  fields.push(`脚踏：${telemetry.footPedal}`);
  telemetry.pumpA = readStatusWithOptionalPump(infoBytes, cursor, "A 泵", fields, 1);
  telemetry.pumpB = readStatusWithOptionalPump(infoBytes, cursor, "B 泵", fields, 0);
  telemetry.summary = fields.join("；");
  telemetry.remainingBytes = infoBytes.slice(cursor.index);
  return telemetry;
}

function formatMotorCurrent(value) {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return `${value.toFixed(2)} A`;
}

function readStatusWithOptionalHandle(infoBytes, cursor, label, fields) {
  const status = readByte(infoBytes, cursor);
  const result = { status, online: status === 0x01, statusText: statusText(status) };
  if (result.online && cursor.index + 1 < infoBytes.length) {
    const raw = infoBytes.slice(cursor.index, cursor.index + 2);
    cursor.index += 2;
    result.rawType = bytesToHex(raw);
    result.typeName = HAND_TYPES.get(result.rawType) ?? "未知手柄";
    fields.push(`${label}：在线 ${result.typeName}`);
  } else {
    fields.push(`${label}：${result.statusText}`);
  }
  return result;
}

function readStatusWithOptionalPump(infoBytes, cursor, label, fields, remainingPumpCount = 0) {
  const status = readByte(infoBytes, cursor);
  const result = { status, online: status === 0x01, statusText: statusText(status) };
  if (result.online && cursor.index + 2 < infoBytes.length) {
    result.type = readByte(infoBytes, cursor);
    result.speed = readWord(infoBytes, cursor);
    readOptionalPumpPressure(infoBytes, cursor, result, remainingPumpCount);
    if (result.pressureRaw !== undefined && result.weightX10 !== undefined) {
      fields.push(`${label}：在线，类型 ${hexByte(result.type)}，速度 ${result.speed}，原始值 ${result.pressureRaw}，重量 ${result.weightText}`);
    } else {
      fields.push(`${label}：在线，类型 ${hexByte(result.type)}，速度 ${result.speed}`);
    }
  } else {
    fields.push(`${label}：${result.statusText}`);
  }
  return result;
}

function readOptionalPumpPressure(infoBytes, cursor, result, remainingPumpCount) {
  // 扩展字段位于泵速度之后：RawCs1237(4LE) + WeightX10(4LE) + ThresholdG(2LE) + Seq(1)。
  const pressureFieldLength = 11;
  // A 泵后面至少还要留出 B 泵在线状态 1 字节，保证兼容旧版心跳格式。
  const reservedForFollowingPumps = remainingPumpCount;
  if (cursor.index + pressureFieldLength + reservedForFollowingPumps > infoBytes.length) {
    return;
  }
  result.pressureRaw = readI32LE(infoBytes, cursor.index);
  cursor.index += 4;
  result.weightX10 = readU32LE(infoBytes, cursor.index);
  result.weightText = formatWeightX10(result.weightX10);
  cursor.index += 4;
  result.thresholdG = readU16LE(infoBytes, cursor.index);
  cursor.index += 2;
  result.pressureSeq = readByte(infoBytes, cursor);
}

function formatWeightX10(value) {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return `${(value / 10).toFixed(1)} g`;
}

function readByte(data, cursor) {
  if (cursor.index >= data.length) {
    return undefined;
  }
  const value = data[cursor.index];
  cursor.index += 1;
  return value;
}

function readWord(data, cursor) {
  if (cursor.index + 1 >= data.length) {
    cursor.index = data.length;
    return undefined;
  }
  const value = readBE16(data, cursor.index);
  cursor.index += 2;
  return value;
}

function statusText(value) {
  if (value === 0x01) {
    return "在线";
  }
  if (value === 0xFF) {
    return "离线";
  }
  if (value === undefined) {
    return "字段缺失";
  }
  return `未知状态 ${hexByte(value)}`;
}

function makeSummary(decoded, crcOk, tailOk) {
  const prefix = decoded.direction;
  const core = `${decoded.funName} / ${decoded.areaName}`;
  if (!tailOk) {
    return `${prefix} ${core}：帧尾错误`;
  }
  if (!crcOk) {
    return `${prefix} ${core}：CRC 错误，应为 ${hex16(decoded.calculatedCrc)}`;
  }
  if (decoded.funCode === 0xAA) {
    return `${prefix} 心跳：${decoded.telemetry.summary}`;
  }
  if (decoded.funCode === 0xDD) {
    return `${prefix} ${decoded.infoName}：${decoded.payloadMeaning}`;
  }
  if (decoded.tranCode === TRAN_UP && decoded.funCode === 0x01) {
    return `${prefix} ${decoded.funName} / ${decoded.areaName}：${decoded.payloadMeaning}`;
  }
  return `${prefix} ${core}：${decoded.payloadMeaning}`;
}

function finalizeFrame(frame) {
  if (!frame.fields || frame.fields.length === 0) {
    frame.fields = [
      { name: "原始数据", hex: bytesToHex(frame.rawBytes || []), meaning: frame.summary || frame.reason || "未解析" }
    ];
  }
  if (!frame.decoded) {
    frame.decoded = {
      funName: "未解析",
      payloadText: frame.summary || "",
      reasonName: frame.reason || null
    };
  }
  return frame;
}

function normalizeBuildArgs(args) {
  if (args.length === 1 && typeof args[0] === "object" && !Array.isArray(args[0])) {
    return args[0];
  }
  const [tranCode, funCode, areaCode, infoCode, infoArea] = args;
  return { tranCode, funCode, areaCode, infoCode, infoArea };
}

function normalizeBytes(data) {
  const array = data instanceof Uint8Array ? Array.from(data) : Array.from(data ?? []);
  return array.map((value, index) => requireByte(value, `byte[${index}]`));
}

function requireByte(value, label) {
  return requireRange(value, 0, 0xFF, label);
}

function requireRange(value, min, max, label) {
  const numberValue = Number(value);
  if (!Number.isInteger(numberValue) || numberValue < min || numberValue > max) {
    throw new RangeError(`${label} 必须是 ${min}..${max} 的整数`);
  }
  return numberValue;
}

function matchAt(data, pattern, offset) {
  if (offset < 0 || data.length < offset + pattern.length) {
    return false;
  }
  return pattern.every((value, index) => data[offset + index] === value);
}

function hexByte(value) {
  return Number(value ?? 0).toString(16).toUpperCase().padStart(2, "0");
}

function hex16(value) {
  const checked = Number(value ?? 0) & 0xFFFF;
  return `${hexByte((checked >> 8) & 0xFF)} ${hexByte(checked & 0xFF)}`;
}
