export const DEFAULT_CONFIG = Object.freeze({
  version: 1,
  serial: {
    baudRate: 115200,
    dataBits: 8,
    stopBits: 1,
    parity: "none",
    flowControl: "none"
  },
  authCode: "11 22 33 44 55 66 77 88",
  presets: {
    speed: 3000,
    freq: 20,
    pumpA: 100,
    pumpB: 100
  },
  safety: {
    confirmWrites: true,
    holdEmergencyMs: 700
  },
  eepromTemplates: [
    {
      name: "空白业务页",
      areaCode: "01",
      bytes: "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
    }
  ]
});

function assertPlainObject(value, fieldName) {
  // 配置必须是普通对象，防止导入数组或字符串后 UI 读字段异常。
  if (value === null || Array.isArray(value) || typeof value !== "object") {
    throw new Error(`${fieldName} 必须是对象`);
  }
}

function normalizeHexBytes(value, expectedLength, fieldName) {
  // 字符串格式便于现场从串口助手或协议文档直接复制十六进制字节。
  if (typeof value !== "string") {
    throw new Error(`${fieldName} 必须是十六进制字符串`);
  }
  const parts = value.trim().split(/[\s,，]+/).filter(Boolean);
  if (expectedLength !== undefined && parts.length !== expectedLength) {
    throw new Error(`${fieldName} 需要 ${expectedLength} 字节，当前 ${parts.length} 字节`);
  }
  for (const part of parts) {
    // 每个字节只接受 00..FF，避免把 0x 前缀或非法字符悄悄吞掉。
    if (!/^[0-9a-fA-F]{2}$/.test(part)) {
      throw new Error(`${fieldName} 存在非法字节：${part}`);
    }
  }
  return parts.map((part) => part.toUpperCase()).join(" ");
}

function normalizeNumber(value, fieldName, min, max) {
  // 所有协议数值最终都进入 8/16 位字段，导入时先限制范围。
  const numberValue = Number(value);
  if (!Number.isInteger(numberValue) || numberValue < min || numberValue > max) {
    throw new Error(`${fieldName} 必须是 ${min}..${max} 的整数`);
  }
  return numberValue;
}

export function validateConfig(input) {
  assertPlainObject(input, "配置根节点");
  const merged = structuredClone(DEFAULT_CONFIG);

  if (input.serial !== undefined) {
    assertPlainObject(input.serial, "serial");
    if (input.serial.baudRate !== undefined) {
      merged.serial.baudRate = normalizeNumber(input.serial.baudRate, "serial.baudRate", 1200, 3000000);
    }
  }

  if (input.authCode !== undefined) {
    merged.authCode = normalizeHexBytes(input.authCode, 8, "authCode");
  }

  if (input.presets !== undefined) {
    assertPlainObject(input.presets, "presets");
    if (input.presets.speed !== undefined) {
      merged.presets.speed = normalizeNumber(input.presets.speed, "presets.speed", 0, 65535);
    }
    if (input.presets.freq !== undefined) {
      merged.presets.freq = normalizeNumber(input.presets.freq, "presets.freq", 0, 65535);
    }
    if (input.presets.pumpA !== undefined) {
      merged.presets.pumpA = normalizeNumber(input.presets.pumpA, "presets.pumpA", 0, 65535);
    }
    if (input.presets.pumpB !== undefined) {
      merged.presets.pumpB = normalizeNumber(input.presets.pumpB, "presets.pumpB", 0, 65535);
    }
  }

  if (input.safety !== undefined) {
    assertPlainObject(input.safety, "safety");
    if (input.safety.confirmWrites !== undefined) {
      merged.safety.confirmWrites = Boolean(input.safety.confirmWrites);
    }
    if (input.safety.holdEmergencyMs !== undefined) {
      merged.safety.holdEmergencyMs = normalizeNumber(input.safety.holdEmergencyMs, "safety.holdEmergencyMs", 0, 5000);
    }
  }

  if (input.eepromTemplates !== undefined) {
    if (!Array.isArray(input.eepromTemplates)) {
      throw new Error("eepromTemplates 必须是数组");
    }
    merged.eepromTemplates = input.eepromTemplates.map((template, index) => {
      assertPlainObject(template, `eepromTemplates[${index}]`);
      return {
        name: String(template.name || `模板 ${index + 1}`),
        areaCode: normalizeHexBytes(template.areaCode || "01", 1, `eepromTemplates[${index}].areaCode`),
        bytes: normalizeHexBytes(template.bytes || "", 30, `eepromTemplates[${index}].bytes`)
      };
    });
  }

  return merged;
}

export async function importConfigFile(file) {
  // File.text() 保持浏览器原生能力，不依赖任何打包器或第三方库。
  const text = await file.text();
  const parsed = JSON.parse(text);
  return validateConfig(parsed);
}
