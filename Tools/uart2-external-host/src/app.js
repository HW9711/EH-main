import { DEFAULT_CONFIG, importConfigFile, validateConfig } from "./config.js";
import { SerialConnection } from "./serial.js";
import { analyzeEepromFrame } from "./eeprom_layout.js";
import {
  buildFrame,
  bytesToHex,
  hexToBytes,
  parseFrames,
  toBE16
} from "./protocol.js";

// appState 集中保存 UI 当前状态，避免各个按钮直接修改 DOM 后彼此不同步。
const appState = {
  config: structuredClone(DEFAULT_CONFIG),
  serial: new SerialConnection(),
  rxBuffer: [],
  logs: [],
  trend: [],
  pressureTrend: [],
  showDataTimestamp: true,
  logMode: "scroll",
  pendingLogCount: 0,
  lastTelemetry: null,
  layout: {
    left: 310,
    right: 390
  },
  lastDecodedFrame: null,
  eepromView: null,
  eepromViews: [],
  eepromBatchMode: false,
  eepromTiming: null,
  eepromOperationType: null,
  lastHeartbeatAt: 0,
  serialConnectedAt: 0,
  heartbeatTimer: null,
  runtimeTimedOut: false,
  externalControlGranted: false,
  activeFault: null,
  emergencyTimer: null,
  dynamicSpeedTimer: null
};

const elements = {};

const EEPROM_MIN_PAGE = 2;
const EEPROM_MAX_PAGE = 128;
const EEPROM_NAV_START_PAGE = 12;
const EEPROM_NAV_END_PAGE = 128;
const EEPROM_PAGE_DELAY_MS = 80;
const HEARTBEAT_TIMEOUT_MS = 3500;

const EEPROM_BUSINESS_PAGES = new Map([
  [2, { areaCode: 0x01, title: "手柄适配信息区" }],
  [3, { areaCode: 0x02, title: "刀具信息区" }],
  [4, { areaCode: 0x03, title: "初始值信息区" }],
  [5, { areaCode: 0x04, title: "按键自定义功能区" }],
  [6, { areaCode: 0x05, title: "多档位功能调节区" }],
  [8, { areaCode: 0x06, title: "储存信息区" }],
  [9, { areaCode: 0x07, title: "手柄客户可编辑区域" }],
  [11, { areaCode: 0x08, title: "出厂信息区" }]
]);

const EEPROM_EXTENSION_PAGES = new Map([
  [7, { title: "多档位功能调节续页", basePage: 6 }],
  [10, { title: "手柄客户可编辑续页", basePage: 9 }]
]);

const EEPROM_ACK_OK = 0x05;
const EEPROM_ACK_FAILED = 0x06;
const DYNAMIC_SPEED_DEBOUNCE_MS = 180;
const HANDLE_START_PREFLIGHT_DELAY_MS = 80;
const SPEED_STEP_VALUE = 100;

const REQUIRED_EXTERNAL_CONTROL_ACTIONS = new Set([
  "pump-a-start",
  "pump-b-start",
  "handle-start",
  "tool-left",
  "tool-right"
]);

const OPERATION_FAIL_ACK_CODES = new Set([0x02, 0x04, 0xAB, 0xBB, 0xFF]);

const OPERATION_ACTION_TEXT = new Map([
  ["apply-control", "申请外部控制"],
  ["host-exit", "退出外部控制"],
  ["permission", "开放权限"],
  ["set-speed", "设置当前手柄速度"],
  ["set-freq", "设置当前手柄频率"],
  ["set-pump-a", "设置 A 泵速度"],
  ["set-pump-b", "设置 B 泵速度"],
  ["switch-a", "切换到 A 通道"],
  ["switch-b", "切换到 B 通道"],
  ["dir-forward", "切换正转"],
  ["dir-reverse", "切换反转"],
  ["dir-osc", "切换往复"],
  ["mode-foot", "切换脚踏控制"],
  ["mode-handle", "切换手控控制"],
  ["mode-external", "切换外部控制模式"],
  ["tool-planer", "切换刨头"],
  ["tool-grind", "切换磨头"],
  ["pump-a-start", "A 泵启动"],
  ["pump-a-stop", "A 泵停止"],
  ["pump-b-start", "B 泵启动"],
  ["pump-b-stop", "B 泵停止"],
  ["handle-start", "当前手柄启动"],
  ["handle-stop", "当前手柄停止"],
  ["tool-left", "开口定位左"],
  ["tool-right", "开口定位右"],
  ["emergency-stop", "紧急刹车"]
]);

const OPERATION_SETTING_TARGET_TEXT = new Map([
  [0x01, "当前手柄速度"],
  [0x02, "当前手柄频率"],
  [0x03, "A 泵速度"],
  [0x04, "B 泵速度"]
]);

const OPERATION_CONTROL_TARGET_TEXT = new Map([
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

const EEPROM_FAIL_REASON_TEXT = new Map([
  [0x01, "长度错误"],
  [0x02, "AreaCode 或参数范围错误"],
  [0x03, "当前无 A/B 通道"],
  [0x04, "设备未配置、EEPROM 读写失败或泵未识别"],
  [0x05, "V1 不支持该命令"],
  [0x06, "Busy，未申请外部控制、运行中或报警中"]
]);

function $(selector) {
  // DOM 查询统一走一个小函数，后续 HTML 调整时更容易定位缺失控件。
  return document.querySelector(selector);
}

function $all(selector) {
  // 按钮批量绑定时使用数组，避免 NodeList 在旧浏览器中的迭代差异。
  return Array.from(document.querySelectorAll(selector));
}

function cacheElements() {
  // 所有可选元素都允许为空，方便测试环境只挂载局部 DOM。
  elements.workspace = $("#workspace");
  elements.connectButton = $("#connectButton");
  elements.clearDisplayButton = $("#clearDisplayButton");
  elements.clearRealtimeButton = $("#clearRealtimeButton");
  elements.clearChartButton = $("#clearChartButton");
  elements.clearEepromViewButton = $("#clearEepromViewButton");
  elements.clearLogButton = $("#clearLogButton");
  elements.clearManualParseButton = $("#clearManualParseButton");
  elements.clearParserButton = $("#clearParserButton");
  elements.connectionState = $("#connectionState");
  elements.leftColumnSplitter = $("#leftColumnSplitter");
  elements.rightColumnSplitter = $("#rightColumnSplitter");
  elements.startupNotice = $("#startupNotice");
  elements.logList = $("#logList");
  elements.timestampToggle = $("#timestampToggle");
  elements.logScrollModeButton = $("#logScrollModeButton");
  elements.logManualModeButton = $("#logManualModeButton");
  elements.logPendingCount = $("#logPendingCount");
  elements.logRefreshButton = $("#logRefreshButton");
  elements.faultPopup = $("#faultPopup");
  elements.faultTime = $("#faultTime");
  elements.faultMessage = $("#faultMessage");
  elements.faultCode = $("#faultCode");
  elements.faultDetail = $("#faultDetail");
  elements.faultRaw = $("#faultRaw");
  elements.faultCloseButton = $("#faultCloseButton");
  elements.operationPopup = $("#operationPopup");
  elements.operationTime = $("#operationTime");
  elements.operationMessage = $("#operationMessage");
  elements.operationCode = $("#operationCode");
  elements.operationDetail = $("#operationDetail");
  elements.operationRaw = $("#operationRaw");
  elements.operationCloseButton = $("#operationCloseButton");
  elements.eepromErrorPopup = $("#eepromErrorPopup");
  elements.eepromErrorTime = $("#eepromErrorTime");
  elements.eepromErrorMessage = $("#eepromErrorMessage");
  elements.eepromErrorCode = $("#eepromErrorCode");
  elements.eepromErrorDetail = $("#eepromErrorDetail");
  elements.eepromErrorRaw = $("#eepromErrorRaw");
  elements.eepromErrorCloseButton = $("#eepromErrorCloseButton");
  elements.parserPanel = $("#parserPanel");
  elements.rawInput = $("#rawInput");
  elements.configInput = $("#configInput");
  elements.configFile = $("#configFile");
  elements.configStatus = $("#configStatus");
  elements.controlApplyStatus = $("#controlApplyStatus");
  elements.eepromTemplateSelect = $("#eepromTemplateSelect");
  elements.applyEepromTemplateButton = $("#applyEepromTemplateButton");
  elements.eepromPageSelect = $("#eepromPageSelect");
  elements.eepromBatchStart = $("#eepromBatchStart");
  elements.eepromBatchEnd = $("#eepromBatchEnd");
  elements.eepromPageStatus = $("#eepromPageStatus");
  elements.readEepromBatchButton = $("#readEepromBatchButton");
  elements.readAllEepromButton = $("#readAllEepromButton");
  elements.readBusinessEepromButton = $("#readBusinessEepromButton");
  elements.readNavEepromButton = $("#readNavEepromButton");
  elements.batchWriteNavTemplateButton = $("#batchWriteNavTemplateButton");
  elements.authCode = $("#authCode");
  elements.speedInput = $("#speedInput");
  elements.speedSlider = $("#speedSlider");
  elements.speedDownButton = $("#speedDownButton");
  elements.speedUpButton = $("#speedUpButton");
  elements.dynamicSpeedToggle = $("#dynamicSpeedToggle");
  elements.dynamicSpeedStatus = $("#dynamicSpeedStatus");
  elements.freqInput = $("#freqInput");
  elements.setFreqButton = $("#setFreqButton");
  elements.pumpAInput = $("#pumpAInput");
  elements.pumpBInput = $("#pumpBInput");
  elements.eepromBytes = $("#eepromBytes");
  elements.metricSpeed = $("#metricSpeed");
  elements.metricCurrent = $("#metricCurrent");
  elements.metricPumpA = $("#metricPumpA");
  elements.metricPumpB = $("#metricPumpB");
  elements.metricChannel = $("#metricChannel");
  elements.metricRun = $("#metricRun");
  elements.deviceHandleA = $("#deviceHandleA");
  elements.deviceHandleB = $("#deviceHandleB");
  elements.devicePumpA = $("#devicePumpA");
  elements.devicePumpB = $("#devicePumpB");
  elements.deviceHandleAStatus = $("#deviceHandleAStatus");
  elements.deviceHandleBStatus = $("#deviceHandleBStatus");
  elements.devicePumpAStatus = $("#devicePumpAStatus");
  elements.devicePumpBStatus = $("#devicePumpBStatus");
  elements.chartValueSpeed = $("#chartValueSpeed");
  elements.chartValueCurrent = $("#chartValueCurrent");
  elements.chartValuePumpA = $("#chartValuePumpA");
  elements.chartValuePumpB = $("#chartValuePumpB");
  elements.pressureRawA = $("#pressureRawA");
  elements.pressureWeightA = $("#pressureWeightA");
  elements.pressureSeqA = $("#pressureSeqA");
  elements.pressureThresholdA = $("#pressureThresholdA");
  elements.pressureRawB = $("#pressureRawB");
  elements.pressureWeightB = $("#pressureWeightB");
  elements.pressureSeqB = $("#pressureSeqB");
  elements.pressureThresholdB = $("#pressureThresholdB");
  elements.pressureRawTrendCanvas = $("#pressureRawTrendChart");
  elements.pressureWeightTrendCanvas = $("#pressureWeightTrendChart");
  elements.chartCanvas = $("#trendChart");
  elements.eepromPageMeta = $("#eepromPageMeta");
  elements.eepromPageTitle = $("#eepromPageTitle");
  elements.eepromPageAddress = $("#eepromPageAddress");
  elements.eepromDataLength = $("#eepromDataLength");
  elements.eepromTimingAction = $("#eepromTimingAction");
  elements.eepromTimingStart = $("#eepromTimingStart");
  elements.eepromTimingEnd = $("#eepromTimingEnd");
  elements.eepromTimingDuration = $("#eepromTimingDuration");
  elements.eepromHexGrid = $("#eepromHexGrid");
  elements.eepromExplainList = $("#eepromExplainList");
  elements.eepromPageNote = $("#eepromPageNote");
}

function setConnectionState(text, connected) {
  if (!elements.connectionState) {
    return;
  }
  // data-connected 让 CSS 可以用同一元素表达在线/离线颜色。
  elements.connectionState.textContent = text;
  elements.connectionState.dataset.connected = connected ? "true" : "false";
}

function readNumberInput(element, fallback, min = 0, max = 65535) {
  // 参数输入最终进入 16 位协议字段，空值或越界时回退到配置预设。
  const value = Number(element?.value ?? fallback);
  if (!Number.isInteger(value) || value < min || value > max) {
    throw new Error(`参数必须是 ${min}..${max} 的整数`);
  }
  return value;
}

function readSpeedInputValue() {
  // 动态调速不允许空值回退，避免用户清空输入框时误把默认速度下发到正在运行的手柄。
  const rawValue = elements.speedInput?.value;
  const value = Number(rawValue);
  if (rawValue === "" || !Number.isInteger(value) || value < 0 || value > 65535) {
    throw new Error("速度必须是 0..65535 的整数");
  }
  return value;
}

function buildSetSpeedFrame(speed) {
  // 固件的 0x02/0x01 会同时写 WorkMessage.speed_set_work 和 WorkMessage.speed_work，运行中下一周期即可动态生效。
  return buildFrame(0x02, 0x02, 0x01, 0xFF, toBE16(speed));
}

function setDynamicSpeedStatus(text, level = "idle") {
  // 状态条只反馈上位机本地动作，真正成功与否仍以通信日志里的 ACK 为准。
  if (!elements.dynamicSpeedStatus) {
    return;
  }
  elements.dynamicSpeedStatus.textContent = text;
  elements.dynamicSpeedStatus.dataset.level = level;
}

function setControlApplyStatus(text, level = "pending") {
  // 申请外部控制的 ACK 容易被高速日志刷走，因此这里提供固定状态条给现场直接查看最终结果。
  if (!elements.controlApplyStatus) {
    return;
  }
  elements.controlApplyStatus.textContent = text;
  elements.controlApplyStatus.dataset.level = level;
}

function setSpeedControlValue(value) {
  // 数字框、滑块和步进按钮共用同一个限幅逻辑，防止不同入口显示不一致。
  const speed = clamp(Math.round(Number(value)), 0, 65535);
  setValue(elements.speedInput, speed);
  setValue(elements.speedSlider, speed);
  return speed;
}

function clearDynamicSpeedTimer() {
  // 防抖计时器只保留最后一次速度变化，拖动滑块时不会向串口刷出过密帧。
  if (appState.dynamicSpeedTimer !== null) {
    window.clearTimeout(appState.dynamicSpeedTimer);
    appState.dynamicSpeedTimer = null;
  }
}

function queueDynamicSpeedSend() {
  // 未开启动态调速时，输入变化只改变表单值，不自动占用串口。
  if (!elements.dynamicSpeedToggle?.checked) {
    try {
      setDynamicSpeedStatus(`速度已调整为 ${readSpeedInputValue()}，点击设置速度下发`, "idle");
    } catch (error) {
      setDynamicSpeedStatus(error.message, "error");
    }
    return;
  }

  let speed;
  try {
    speed = readSpeedInputValue();
  } catch (error) {
    clearDynamicSpeedTimer();
    setDynamicSpeedStatus(error.message, "error");
    return;
  }

  if (!appState.serial.connected) {
    clearDynamicSpeedTimer();
    setDynamicSpeedStatus("动态调速已开启，等待串口连接", "warning");
    return;
  }

  clearDynamicSpeedTimer();
  setDynamicSpeedStatus(`准备下发 ${speed}`, "pending");
  appState.dynamicSpeedTimer = window.setTimeout(async () => {
    // 定时触发时重新读取一次输入框，确保发送的是用户最终停留的速度。
    await sendDynamicSpeedNow();
  }, DYNAMIC_SPEED_DEBOUNCE_MS);
}

async function transmitCurrentHandleSpeed() {
  // 统一下发当前速度输入框的值，动态调速和手柄启动前置都走同一帧格式，避免两条路径组帧不一致。
  const speed = readSpeedInputValue();
  // 固件收到 0x02/0x01 后会刷新 speed_set_work，随后 0x04/0x05 启动手柄才有非零目标速度。
  await transmitBytes(buildSetSpeedFrame(speed));
  // 返回实际下发值，调用方用它刷新状态条或错误提示。
  return speed;
}

async function sendDynamicSpeedNow() {
  // 动态下发复用“设置当前手柄速度”帧，不额外启动/停止手柄，也不切换通道。
  clearDynamicSpeedTimer();
  try {
    const speed = await transmitCurrentHandleSpeed();
    setDynamicSpeedStatus(`动态已下发 ${speed}`, "ok");
  } catch (error) {
    setDynamicSpeedStatus(`动态调速失败：${error.message}`, "error");
    appendTextLog("SYS", `动态调速失败：${error.message}`, "error");
  }
}

function updateDynamicSpeedModeStatus() {
  // 开关状态变化或串口状态变化时，给操作者一个明确的“自动/手动”反馈。
  clearDynamicSpeedTimer();
  if (elements.dynamicSpeedToggle?.checked) {
    setDynamicSpeedStatus(appState.serial.connected ? "动态调速已开启" : "动态调速已开启，等待串口连接", appState.serial.connected ? "ok" : "warning");
  } else {
    setDynamicSpeedStatus("手动下发", "idle");
  }
}

function syncSpeedFromInput() {
  // 数字框输入合法时同步滑块；非法时只提示，不改滑块，便于用户继续编辑。
  try {
    const speed = readSpeedInputValue();
    setValue(elements.speedSlider, speed);
    queueDynamicSpeedSend();
  } catch (error) {
    clearDynamicSpeedTimer();
    setDynamicSpeedStatus(error.message, "error");
  }
}

function adjustSpeed(delta) {
  // 步进按钮按当前有效输入计算；输入为空或非法时回到配置预设再增减。
  let baseSpeed;
  try {
    baseSpeed = readSpeedInputValue();
  } catch (error) {
    baseSpeed = appState.config.presets.speed;
  }
  setSpeedControlValue(baseSpeed + delta);
  queueDynamicSpeedSend();
}

function appendLog(direction, bytes, decoded, level = "info") {
  // 日志对象保留原始字节，点击日志时可以重新显示协议解析窗口。
  const timestamp = new Date();
  const entry = {
    id: crypto.randomUUID(),
    direction,
    bytes: Array.from(bytes),
    decoded,
    level,
    timestamp,
    timestampMs: timestamp.getTime()
  };
  appState.logs.unshift(entry);
  appState.logs = appState.logs.slice(0, 600);
  if (appState.logMode === "manual") {
    // 手动模式下新日志只计数提示，不主动抢走用户正在查看的位置。
    appState.pendingLogCount += 1;
  }
  renderLogs({ preserveManualView: appState.logMode === "manual" });
  return entry;
}

function renderLogs(options = {}) {
  if (!elements.logList) {
    return;
  }
  const previousScrollTop = elements.logList.scrollTop;
  const previousScrollHeight = elements.logList.scrollHeight;
  elements.logList.replaceChildren();
  for (const entry of appState.logs.slice(0, 180)) {
    const rawHex = bytesToHex(entry.bytes);
    const rawText = rawHex || "无原始数据帧";
    const row = document.createElement("button");
    row.type = "button";
    row.className = `log-row log-${entry.direction.toLowerCase()} log-${entry.level}`;
    row.dataset.logId = entry.id;
    row.title = `${formatDataTimestamp(entry.timestamp, true)} ${bytesToHex(entry.bytes)}`;
    row.innerHTML = `
      <span class="log-time">${appState.showDataTimestamp ? formatDataTimestamp(entry.timestamp) : entry.timestamp.toLocaleTimeString("zh-CN", { hour12: false })}</span>
      <span class="log-dir">${entry.direction}</span>
      <span class="log-main">
        <span class="log-summary">${escapeHtml(entry.decoded?.summary || rawText)}</span>
        <code class="log-raw">HEX ${escapeHtml(rawText)}</code>
      </span>
    `;
    row.addEventListener("click", () => showParser(makeTimestampDecoded(entry), entry.bytes));
    elements.logList.appendChild(row);
  }
  if (appState.logMode === "scroll") {
    // 日志按“最新在上”排列，滚动模式固定回到顶部，现场可持续盯最新 TX/RX。
    elements.logList.scrollTop = 0;
    appState.pendingLogCount = 0;
  } else if (options.preserveManualView) {
    // 新日志插入在列表顶部，补偿高度差后可尽量保持当前可视内容不跳动。
    const addedHeight = Math.max(0, elements.logList.scrollHeight - previousScrollHeight);
    elements.logList.scrollTop = previousScrollTop + addedHeight;
  } else {
    // 非新增场景只恢复原来的滚动位置，例如切换时间戳显示。
    elements.logList.scrollTop = previousScrollTop;
  }
  renderLogModeState();
}

function setLogMode(mode) {
  // scroll 表示自动跟随最新日志，manual 表示现场手动翻看历史日志。
  appState.logMode = mode === "manual" ? "manual" : "scroll";
  if (appState.logMode === "scroll") {
    // 切回滚动模式时立即显示最新日志，并清掉未查看计数。
    appState.pendingLogCount = 0;
    renderLogs();
    return;
  }
  renderLogModeState();
}

function showPendingLogs() {
  // 手动模式下点击“显示新日志”只跳到最新位置，不改变当前模式选择。
  appState.pendingLogCount = 0;
  if (elements.logList) {
    elements.logList.scrollTop = 0;
  }
  renderLogModeState();
}

function renderLogModeState() {
  const isScrollMode = appState.logMode === "scroll";
  elements.logScrollModeButton?.classList.toggle("active", isScrollMode);
  elements.logManualModeButton?.classList.toggle("active", !isScrollMode);
  elements.logScrollModeButton?.setAttribute("aria-pressed", String(isScrollMode));
  elements.logManualModeButton?.setAttribute("aria-pressed", String(!isScrollMode));
  const hasPending = !isScrollMode && appState.pendingLogCount > 0;
  if (elements.logPendingCount) {
    elements.logPendingCount.hidden = !hasPending;
    elements.logPendingCount.textContent = `${appState.pendingLogCount} 条新日志`;
  }
  if (elements.logRefreshButton) {
    elements.logRefreshButton.hidden = !hasPending;
  }
}

function formatDataTimestamp(timestamp, withDate = false) {
  // 数据时间戳用本机接收/发送时刻，毫秒用于区分 ACK 和 1s 心跳的先后顺序。
  const date = timestamp instanceof Date ? timestamp : new Date(timestamp);
  const timeText = `${date.toLocaleTimeString("zh-CN", { hour12: false })}.${String(date.getMilliseconds()).padStart(3, "0")}`;
  if (!withDate) {
    return timeText;
  }
  return `${date.toLocaleDateString("zh-CN")} ${timeText}`;
}

function formatMotorCurrent(value) {
  // MCU 心跳中的手柄电流来自驱动板反馈字段，线上单位为 0.01A，上层协议已换算成 A。
  if (!Number.isFinite(value)) {
    // 未运行或旧固件没有上传反馈电流时，保持界面占位，避免误显示为 0A。
    return "--";
  }

  // 现场调试看电流需要保留两位小数，便于区分 0.10A 和 1.00A 这类轻载变化。
  return `${value.toFixed(2)} A`;
}

function formatMotorSpeed(value) {
  // 手柄速度是转速指令/反馈值，界面直接带 rpm，避免现场误把裸数字当成比例或百分比。
  const numericValue = Number(value);
  if (value === null || value === undefined || !Number.isFinite(numericValue)) {
    return "--";
  }

  return `${Math.round(numericValue)} rpm`;
}

function formatPumpSpeed(value) {
  // 泵速对应协议中的 ml/min 设定值，状态卡片和曲线当前值保持同一单位口径。
  const numericValue = Number(value);
  if (value === null || value === undefined || !Number.isFinite(numericValue)) {
    return "--";
  }

  return `${Math.round(numericValue)} ml/min`;
}

function makeTimestampDecoded(entry) {
  if (!appState.showDataTimestamp || !entry?.decoded) {
    return entry?.decoded;
  }
  // 解析窗口使用复制后的字段数组，避免切换开关时污染协议解析原始对象。
  const fields = Array.isArray(entry.decoded.fields) ? [...entry.decoded.fields] : [];
  fields.unshift({
    name: "数据时间戳",
    hex: entry.direction,
    meaning: `${formatDataTimestamp(entry.timestamp, true)}，本机${entry.direction === "RX" ? "收到" : "记录"}该帧`
  });
  return {
    ...entry.decoded,
    timestampMs: entry.timestampMs,
    timestampText: formatDataTimestamp(entry.timestamp, true),
    fields
  };
}

function escapeHtml(value) {
  // 日志里可能包含用户粘贴的 HEX 字符串，写入 innerHTML 前必须转义。
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function showParser(decoded, bytes) {
  if (!elements.parserPanel) {
    return;
  }
  appState.lastDecodedFrame = decoded || null;
  const checks = decoded?.checks || {};
  const fieldRows = decoded?.fields || [];
  const statusText = decoded?.valid ? "通过" : "异常";
  elements.parserPanel.innerHTML = `
    <div class="parser-head">
      <div>
        <strong>${escapeHtml(decoded?.summary || "未解析帧")}</strong>
        <span class="parser-sub">${escapeHtml(bytesToHex(bytes || []))}</span>
      </div>
      <span class="parser-badge ${decoded?.valid ? "ok" : "bad"}">${statusText}</span>
    </div>
    <div class="check-grid">
      ${renderCheck("帧头", checks.headOk)}
      ${renderCheck("长度", checks.lengthOk)}
      ${renderCheck("CRC", checks.crcOk)}
      ${renderCheck("帧尾", checks.tailOk)}
    </div>
    <div class="field-list">
      ${fieldRows.map((field) => `
        <div class="field-row">
          <span>${escapeHtml(field.name)}</span>
          <code>${escapeHtml(field.hex ?? "")}</code>
          <em>${escapeHtml(field.meaning ?? "")}</em>
        </div>
      `).join("")}
    </div>
  `;
}

function makeManualParseFrame(inputBytes, parsed) {
  const firstFrame = parsed.frames[0];
  if (!firstFrame) {
    // 没有完整帧时把用户原始输入和 remaining 一起展示，便于定位半包或帧头错误。
    return {
      valid: false,
      summary: "未找到完整帧",
      checks: {},
      rawBytes: inputBytes,
      fields: [
        { name: "输入", hex: bytesToHex(inputBytes), meaning: "请检查帧头、Length 或帧尾" },
        { name: "剩余数据", hex: bytesToHex(parsed.remaining), meaning: "这些字节暂未组成完整帧" }
      ]
    };
  }

  const fields = [...firstFrame.fields, {
    name: "解析统计",
    hex: `${parsed.frames.length} 帧`,
    meaning: `当前显示第 1 帧，offset=${firstFrame.offset ?? 0}`
  }];
  if (parsed.frames.length > 1) {
    // 多帧粘包时不丢掉后续帧数量，提醒现场继续查看日志或拆开分析。
    fields.push({
      name: "后续完整帧",
      hex: `${parsed.frames.length - 1} 帧`,
      meaning: "输入中还有后续完整帧，当前窗口先展示第一帧含义"
    });
  }
  if (parsed.remaining.length > 0) {
    fields.push({
      name: "剩余数据",
      hex: bytesToHex(parsed.remaining),
      meaning: "末尾剩余字节尚未组成完整帧，可能是串口半包"
    });
  }

  return {
    ...firstFrame,
    rawBytes: inputBytes,
    fields,
    summary: parsed.frames.length > 1 || parsed.remaining.length > 0
      ? `${firstFrame.summary}；输入共 ${parsed.frames.length} 帧，剩余 ${parsed.remaining.length} 字节`
      : firstFrame.summary
  };
}

function renderCheck(name, value) {
  // undefined 表示当前无法判断，例如粘贴数据不足一个完整帧。
  const text = value === undefined ? "未判定" : value ? "OK" : "失败";
  const className = value === undefined ? "unknown" : value ? "ok" : "bad";
  return `<div class="check ${className}"><span>${name}</span><strong>${text}</strong></div>`;
}

function applyTelemetry(decoded) {
  // 只有心跳帧会更新实时状态，ACK 帧只写日志和解析窗口。
  if (!decoded?.telemetry) {
    return;
  }
  // 收到心跳才认为主机在线；串口物理打开但无心跳时只显示“等待心跳/心跳超时”。
  appState.lastHeartbeatAt = Date.now();
  appState.runtimeTimedOut = false;
  if (appState.serial.connected) {
    setConnectionState("已连接", true);
  }
  const telemetry = decoded.telemetry;
  const currentRaw = telemetry.currentRaw;
  const currentText = formatMotorCurrent(telemetry.current);
  appState.lastTelemetry = telemetry;
  updateDeviceIndicators(telemetry);
  updateFrequencyControlState(telemetry);
  setText(elements.metricSpeed, formatMotorSpeed(telemetry.speed));
  setText(elements.metricCurrent, currentText);
  if (elements.metricCurrent) {
    // 标题栏保留驱动反馈原始值，现场排查比例系数时可直接对照串口帧 byte8~9。
    elements.metricCurrent.title = Number.isFinite(currentRaw) ? `驱动反馈原始值 ${currentRaw}（0.01A）` : "";
  }
  setText(elements.metricPumpA, formatPumpSpeed(telemetry.pumpA?.speed));
  setText(elements.metricPumpB, formatPumpSpeed(telemetry.pumpB?.speed));
  setText(elements.metricChannel, telemetry.selectedChannelText ?? "未选中");
  setText(elements.metricRun, telemetry.runStatusText ?? "未知");
  updateChartValueStrip({
    speed: formatMotorSpeed(telemetry.speed),
    current: currentText,
    pumpA: formatPumpSpeed(telemetry.pumpA?.speed),
    pumpB: formatPumpSpeed(telemetry.pumpB?.speed)
  });
  updatePressureSensorPanel(telemetry);
  applyPressureThresholdAlarm(telemetry, decoded.rawBytes);
  pushPressureTrendPoint(telemetry);

  // 曲线只记录可数值化的字段，未运行时用 0 保持时间轴连续。
  appState.trend.push({
    time: Date.now(),
    speed: Number(telemetry.speed || 0),
    current: Number(telemetry.current || 0),
    pumpA: Number(telemetry.pumpA?.speed || 0),
    pumpB: Number(telemetry.pumpB?.speed || 0)
  });
  appState.trend = appState.trend.slice(-80);
  drawTrend();
  drawPressureTrend();
}

function resetRuntimeStatus(connectionText, connected = false) {
  // 系统断电、串口断开或心跳超时时，清掉最后一帧留下的在线/选中/泵速显示。
  setConnectionState(connectionText, connected);
  // 心跳超时或串口重连后不能沿用旧授权，必须重新收到 0xAA 才允许启动类操作。
  appState.externalControlGranted = false;
  setControlApplyStatus(connected ? "等待申请外部控制" : "尚未申请外部控制", "pending");
  setText(elements.metricSpeed, "--");
  setText(elements.metricCurrent, "--");
  if (elements.metricCurrent) {
    // 状态复位时同步清掉上一帧的驱动反馈原始值提示。
    elements.metricCurrent.title = "";
  }
  setText(elements.metricPumpA, "--");
  setText(elements.metricPumpB, "--");
  setText(elements.metricChannel, "当前通道：未选中");
  setText(elements.metricRun, "运行状态：未知");
  appState.lastTelemetry = null;
  resetDeviceIndicators();
  updateFrequencyControlState(null);
  updateChartValueStrip({ speed: "--", current: "--", pumpA: "--", pumpB: "--" });
  resetPressureSensorPanel();
  clearPressureAlarmIfSafe();
  appState.trend = [];
  appState.pressureTrend = [];
  drawTrend();
  drawPressureTrend();
}

function startHeartbeatWatchdog() {
  if (appState.heartbeatTimer !== null) {
    window.clearInterval(appState.heartbeatTimer);
  }
  // 心跳约 1s 一帧，留 3.5s 容错；断电或 MCU 停止上传时自动恢复 UI 状态。
  appState.heartbeatTimer = window.setInterval(handleHeartbeatTimeout, 1000);
}

function handleHeartbeatTimeout() {
  if (!appState.serial.connected) {
    return;
  }
  const now = Date.now();
  const lastActiveAt = appState.lastHeartbeatAt || appState.serialConnectedAt;
  if (lastActiveAt > 0 && now - lastActiveAt > HEARTBEAT_TIMEOUT_MS && !appState.runtimeTimedOut) {
    appState.runtimeTimedOut = true;
    resetRuntimeStatus("心跳超时", false);
    appendTextLog("SYS", "超过 3.5 秒未收到心跳，已恢复实时状态显示；请检查主机供电或串口连接", "error");
  }
}

function updateDeviceIndicators(telemetry) {
  // 心跳里 A/B 手柄既有在线位，也有“当前选中手柄插孔”字段；二者同时成立才标成当前工作手柄。
  const handleAActive = telemetry.selectedChannel === 0x01 && telemetry.handleA?.online;
  const handleBActive = telemetry.selectedChannel === 0x02 && telemetry.handleB?.online;
  setDeviceIndicator(elements.deviceHandleA, elements.deviceHandleAStatus, telemetry.handleA?.online, buildHandleDeviceText(telemetry.handleA, handleAActive), handleAActive);
  setDeviceIndicator(elements.deviceHandleB, elements.deviceHandleBStatus, telemetry.handleB?.online, buildHandleDeviceText(telemetry.handleB, handleBActive), handleBActive);
  setDeviceIndicator(elements.devicePumpA, elements.devicePumpAStatus, telemetry.pumpA?.online, buildPumpDeviceText(telemetry.pumpA));
  setDeviceIndicator(elements.devicePumpB, elements.devicePumpBStatus, telemetry.pumpB?.online, buildPumpDeviceText(telemetry.pumpB));
}

function getSelectedHandle(telemetry = appState.lastTelemetry) {
  if (!telemetry) {
    return null;
  }
  if (telemetry.selectedChannel === 0x01) {
    return telemetry.handleA;
  }
  if (telemetry.selectedChannel === 0x02) {
    return telemetry.handleB;
  }
  return null;
}

function isOscillatingHandle(handle) {
  return handle?.online && (handle.typeName === "PXBA" || handle.typeName === "PXBB");
}

function canSetFrequency() {
  return isOscillatingHandle(getSelectedHandle());
}

function updateFrequencyControlState(telemetry = appState.lastTelemetry) {
  const handle = getSelectedHandle(telemetry);
  const enabled = isOscillatingHandle(handle);
  const title = enabled ? "PXBA/PXBB 往复转手柄允许设置往复频率" : "仅 PXBA/PXBB 往复转手柄允许设置频率";
  if (elements.freqInput) {
    elements.freqInput.disabled = !enabled;
    elements.freqInput.title = title;
  }
  if (elements.setFreqButton) {
    elements.setFreqButton.disabled = !enabled;
    elements.setFreqButton.title = title;
  }
}

function resetDeviceIndicators() {
  setDeviceIndicator(elements.deviceHandleA, elements.deviceHandleAStatus, false, "未接入");
  setDeviceIndicator(elements.deviceHandleB, elements.deviceHandleBStatus, false, "未接入");
  setDeviceIndicator(elements.devicePumpA, elements.devicePumpAStatus, false, "未接入");
  setDeviceIndicator(elements.devicePumpB, elements.devicePumpBStatus, false, "未接入");
}

function setDeviceIndicator(card, statusElement, online, text, active = false) {
  if (card) {
    card.dataset.online = online ? "true" : "false";
    card.dataset.active = online && active ? "true" : "false";
    card.setAttribute("aria-current", online && active ? "true" : "false");
  }
  setText(statusElement, text);
}

function buildHandleDeviceText(handle, active = false) {
  if (!handle) {
    return "未接入";
  }
  if (!handle.online) {
    return handle.statusText === "离线" ? "未接入" : handle.statusText;
  }
  const text = handle.typeName ? `已接入 ${handle.typeName}` : "已接入";
  return active ? `${text} · 当前工作` : text;
}

function buildPumpDeviceText(pump) {
  if (!pump) {
    return "未接入";
  }
  if (!pump.online) {
    return pump.statusText === "离线" ? "未接入" : pump.statusText;
  }
  return pump.speed === undefined ? "已接入" : `已接入 ${pump.speed}`;
}

function applyEepromFrame(decoded) {
  // EEPROM 读页成功后，固件用 MCU 上传帧返回 30 字节有效数据；非该类帧不影响当前显示。
  const view = analyzeEepromFrame(decoded);
  if (!view) {
    return;
  }
  if (appState.eepromBatchMode) {
    // 批量模式下同一 Page 后到的回包覆盖旧结果，避免重复读取时可视化列表越来越长。
    appState.eepromViews = appState.eepromViews.filter((item) => item.page !== view.page).concat(view)
      .sort((left, right) => left.page - right.page);
  } else {
    // 单页读取只保留当前 Page，让窗口内容和下拉选择保持直观对应。
    appState.eepromViews = [view];
  }
  appState.eepromView = view;
  renderEepromView();
}

function applyAlarmFrame(decoded, rawBytes = []) {
  // 协议报警帧固定为 MCU 上传 FunCode=0x03、InforCode=0x05；CRC/帧尾异常时不弹窗，避免噪声误报。
  if (!decoded?.valid || !decoded.alarm) {
    return;
  }
  if (!decoded.alarm.active) {
    hideFaultPopup();
    return;
  }
  showFaultPopup(decoded.alarm, decoded, rawBytes);
}

function showFaultPopup(alarm, decoded, rawBytes) {
  const timestamp = new Date();
  // activeFault 保留最近一次报警码和时间，后续导出或扩展状态栏时可直接复用。
  appState.activeFault = {
    code: alarm.code,
    codeText: alarm.codeText,
    name: alarm.name,
    timestamp
  };
  setText(elements.faultTime, formatDataTimestamp(timestamp, true));
  setText(elements.faultMessage, alarm.name);
  setText(elements.faultCode, `报警码 0x${alarm.codeText}`);
  setText(elements.faultDetail, buildFaultDetail(decoded));
  setText(elements.faultRaw, `HEX ${bytesToHex(rawBytes.length > 0 ? rawBytes : decoded.rawBytes || [])}`);
  if (elements.faultPopup) {
    elements.faultPopup.hidden = false;
    elements.faultPopup.dataset.active = "true";
  }
}

function hideFaultPopup() {
  appState.activeFault = null;
  if (elements.faultPopup) {
    elements.faultPopup.hidden = true;
    elements.faultPopup.dataset.active = "false";
  }
}

function applyPressureThresholdAlarm(telemetry, rawBytes = []) {
  // 心跳压力扩展字段由上位机本地判断阈值，避免等待固件额外报警帧才提示现场人员。
  const alarm = buildPressureThresholdAlarm(telemetry);
  if (alarm) {
    showPressureAlarmPopup(alarm, rawBytes);
    return;
  }
  clearPressureAlarmIfSafe();
}

function buildPressureThresholdAlarm(telemetry) {
  // WeightX10 单位是 0.1g，ThresholdG 单位是 g；比较时把阈值放大 10 倍保持整数比较。
  for (const [label, pump] of [["A 泵", telemetry?.pumpA], ["B 泵", telemetry?.pumpB]]) {
    if (!pump?.online) {
      continue;
    }
    if (!Number.isFinite(pump.weightX10) || !Number.isFinite(pump.thresholdG) || pump.thresholdG <= 0) {
      continue;
    }
    if (pump.weightX10 > pump.thresholdG * 10) {
      return {
        label,
        message: `${label} 压力超过阈值`,
        weightX10: pump.weightX10,
        thresholdG: pump.thresholdG,
        pressureRaw: pump.pressureRaw,
        pressureSeq: pump.pressureSeq
      };
    }
  }
  return null;
}

function showPressureAlarmPopup(alarm, rawBytes = []) {
  const timestamp = new Date();
  // 压力超阈值属于上位机从心跳数据直接判断的安全提示，source 用于恢复正常后只关闭本类弹窗。
  appState.activeFault = {
    source: "pressure",
    code: "PRESSURE",
    codeText: alarm.label,
    name: alarm.message,
    timestamp
  };
  setText(elements.faultTime, formatDataTimestamp(timestamp, true));
  setText(elements.faultMessage, alarm.message);
  setText(elements.faultCode, `压力报警 ${alarm.label}`);
  setText(elements.faultDetail, `WeightX10=${alarm.weightX10}，最终重量 ${formatWeightX10(alarm.weightX10)}；ThresholdG=${alarm.thresholdG} g；CS1237 原始值 ${formatPressureRaw(alarm.pressureRaw)}；Seq ${Number.isFinite(alarm.pressureSeq) ? alarm.pressureSeq : "--"}`);
  setText(elements.faultRaw, rawBytes.length > 0 ? `HEX ${bytesToHex(rawBytes)}` : "HEX --");
  if (elements.faultPopup) {
    elements.faultPopup.hidden = false;
    elements.faultPopup.dataset.active = "true";
  }
}

function clearPressureAlarmIfSafe() {
  if (appState.activeFault?.source === "pressure") {
    hideFaultPopup();
  }
}

function buildFaultDetail(decoded) {
  // 文案直接对应固件的 WorkMessage.alarm_value，经 0x03/0x05 报警信息帧上传到上位机。
  const source = decoded?.direction || "MCU 上传/应答";
  const payload = decoded?.payloadMeaning || "报警信息";
  return `${source}；${payload}；来源字段：WorkMessage.alarm_value`;
}

function applyOperationAckFrame(decoded, rawBytes = []) {
  // 运行值、控制、权限类 ACK 失败不属于 EEPROM 或故障报警，需要单独弹出操作顺序提示。
  if (!decoded?.valid || decoded.tranCode !== 0x01 || decoded.funCode !== 0xDD) {
    return;
  }
  const ackInfoBytes = Array.from(decoded.infoBytes ?? []);

  // 申请外部控制成功后，后续启动手柄/泵才允许通过本地顺序检查。
  if (decoded.infoCode === 0xAA) {
    appState.externalControlGranted = true;
    setControlApplyStatus("申请成功，已取得外部控制权", "ok");
    hideOperationPopup();
    return;
  }

  // 申请外部控制失败时必须清掉本地授权状态，避免后续按钮误认为已经取得控制权。
  if (decoded.infoCode === 0xAB) {
    appState.externalControlGranted = false;
    setControlApplyStatus("申请失败，未取得外部控制权", "error");
    showOperationPopup(buildOperationAckError(decoded), rawBytes.length > 0 ? rawBytes : decoded.rawBytes || []);
    return;
  }

  // 退出外部控制成功时，固件在 ACK InforArea[0] 回显 0xBB，本地授权状态必须同步清掉。
  if ((decoded.infoCode === 0x03) && (ackInfoBytes[0] === 0xBB)) {
    appState.externalControlGranted = false;
    setControlApplyStatus("已退出外部控制", "ok");
    hideOperationPopup();
    return;
  }

  // 成功 ACK 到来时收起旧的操作提示，避免用户已经按正确顺序重试后仍看到旧弹窗。
  if ((decoded.infoCode === 0x01) || (decoded.infoCode === 0x03) || (decoded.infoCode === 0xFE)) {
    hideOperationPopup();
    return;
  }

  // EEPROM 失败由 EEPROM 专用弹窗处理，这里只处理普通操作失败。
  if (!OPERATION_FAIL_ACK_CODES.has(decoded.infoCode)) {
    return;
  }

  showOperationPopup(buildOperationAckError(decoded), rawBytes.length > 0 ? rawBytes : decoded.rawBytes || []);
}

function buildOperationAckError(decoded) {
  // 失败 ACK 的 InforArea[0] 是失败对象，最后 1 字节是固件返回的失败原因。
  const infoBytes = Array.from(decoded.infoBytes ?? []);
  const target = infoBytes[0];
  const reason = infoBytes.length > 0 ? infoBytes[infoBytes.length - 1] : undefined;
  const targetText = resolveOperationAckTarget(decoded.infoCode, target);
  const reasonText = EEPROM_FAIL_REASON_TEXT.get(reason) || decoded.decoded?.reasonName || "未知失败原因";
  const code = `ACK 0x${hexByteText(decoded.infoCode)}${reason === undefined ? "" : ` / 原因 0x${hexByteText(reason)}`}`;

  if (reason === 0x06) {
    return {
      message: "操作顺序不正确",
      code,
      detail: `${targetText} 被主机拒绝：${reasonText}。请先点击“申请外部控制”，收到申请成功 ACK 后，再启动手柄、泵或开口定位。设置泵转速只写入设定值，不等于已经取得外部控制权。`
    };
  }

  if (reason === 0x03) {
    return {
      message: "当前无有效手柄通道",
      code,
      detail: `${targetText} 被主机拒绝：${reasonText}。请先接入手柄并确认实时状态里有当前工作手柄，再执行该操作。`
    };
  }

  if (reason === 0x04) {
    return {
      message: "设备未识别或未配置",
      code,
      detail: `${targetText} 被主机拒绝：${reasonText}。请确认对应手柄、泵或 EEPROM 已识别在线，再重试。`
    };
  }

  return {
    message: decoded.infoName || "操作失败",
    code,
    detail: `${targetText} 被主机拒绝：${decoded.payloadMeaning || reasonText}。请按协议解析窗口继续核对返回原因。`
  };
}

function resolveOperationAckTarget(infoCode, target) {
  if (target === undefined) {
    return "当前操作";
  }
  if (infoCode === 0x02) {
    return OPERATION_SETTING_TARGET_TEXT.get(target) || `设置对象 0x${hexByteText(target)}`;
  }
  if (infoCode === 0x04) {
    return OPERATION_CONTROL_TARGET_TEXT.get(target) || `控制对象 0x${hexByteText(target)}`;
  }
  if (infoCode === 0xAB) {
    return "申请外部控制";
  }
  if (infoCode === 0xBB) {
    return "注册码校验";
  }
  if (infoCode === 0xFF) {
    return "权限开放";
  }
  return `对象 0x${hexByteText(target)}`;
}

function showOperationPopup(error, rawBytes = []) {
  const timestamp = new Date();
  setText(elements.operationTime, formatDataTimestamp(timestamp, true));
  setText(elements.operationMessage, error.message || "操作未执行");
  setText(elements.operationCode, error.code || "LOCAL");
  setText(elements.operationDetail, error.detail || "请检查串口连接、外部控制权限和当前设备在线状态。");
  setText(elements.operationRaw, rawBytes.length > 0 ? `HEX ${bytesToHex(rawBytes)}` : "HEX --");
  if (elements.operationPopup) {
    elements.operationPopup.hidden = false;
    elements.operationPopup.dataset.active = "true";
  }
}

function showOperationLocalError(message, action = "") {
  // 本地下发前拦截的错误也要弹窗，例如未申请外部控制、串口未连接或输入值非法。
  const actionText = OPERATION_ACTION_TEXT.get(action) || "当前操作";
  showOperationPopup({
    message: "操作未执行",
    code: "LOCAL",
    detail: `${actionText} 未发送：${message}`
  });
}

function validateOperationOrder(action) {
  // 启动类和开口定位类命令会直接驱动外设，必须先等 MCU 回 ACK 0xAA 确认外部控制权。
  if (REQUIRED_EXTERNAL_CONTROL_ACTIONS.has(action) && !appState.externalControlGranted) {
    throw new Error("请先点击“申请外部控制”，收到申请成功 ACK 后，再执行该操作");
  }
}

function hideOperationPopup() {
  if (elements.operationPopup) {
    elements.operationPopup.hidden = true;
    elements.operationPopup.dataset.active = "false";
  }
}

function applyEepromAckFrame(decoded, rawBytes = []) {
  // MCU 的 EEPROM 命令应答使用 FunCode=0xDD，其中 InforCode=0x06 表示读写失败。
  if (!decoded?.valid || decoded.tranCode !== 0x01 || decoded.funCode !== 0xDD) {
    return;
  }
  if (decoded.infoCode === EEPROM_ACK_OK) {
    // 后续读写成功时自动收起上一次错误，避免用户误以为当前 Page 仍失败。
    hideEepromErrorPopup();
    return;
  }
  if (decoded.infoCode !== EEPROM_ACK_FAILED) {
    return;
  }
  const error = buildEepromAckError(decoded);
  if (suppressEepromBatchReadErrorPopup(error)) {
    return;
  }
  showEepromErrorPopup(error, rawBytes.length > 0 ? rawBytes : decoded.rawBytes || []);
}

function suppressEepromBatchReadErrorPopup(error) {
  if (appState.eepromOperationType !== "batch-read") {
    return false;
  }
  // 批量读取时允许部分业务页/导航页返回失败 ACK，避免已成功显示的数据被居中弹窗遮挡。
  finishEepromTiming("部分页失败");
  addFailedEepromView(error);
  appendTextLog("SYS", `EEPROM 批量读取部分页失败：${error.code}，${error.targetText || error.message}`, "warning");
  return true;
}

function resolveEepromAckTargetPage(target) {
  for (const [page, info] of EEPROM_BUSINESS_PAGES) {
    if (info.areaCode === target) {
      return page;
    }
  }
  if (target >= EEPROM_NAV_START_PAGE && target <= EEPROM_NAV_END_PAGE) {
    return target;
  }
  return null;
}

function buildEepromAckError(decoded) {
  // InforArea[0] 是失败对象，最后 1 字节是固件返回的失败原因。
  const infoBytes = Array.from(decoded.infoBytes ?? []);
  const target = infoBytes[0];
  const reason = infoBytes.length > 0 ? infoBytes[infoBytes.length - 1] : undefined;
  const reasonText = EEPROM_FAIL_REASON_TEXT.get(reason) || decoded.decoded?.reasonName || "未知失败原因";
  const targetPage = resolveEepromAckTargetPage(target);
  const targetText = target === undefined ? "未知对象" : (targetPage ? `Page${targetPage} / 对象 0x${hexByteText(target)}` : `对象 0x${hexByteText(target)}`);
  if (reason === 0x03) {
    return {
      message: "未检测到当前手柄通道",
      code: `ACK 0x${hexByteText(decoded.infoCode)} / 原因 0x${hexByteText(reason)}`,
      detail: `${decoded.payloadMeaning}。当前无 A/B 通道，请先接入手柄并切换到对应通道，再读取 EEPROM 芯片数据。`,
      targetText,
      targetPage
    };
  }
  if (reason === 0x04) {
    return {
      message: "EEPROM 芯片读写未成功",
      code: `ACK 0x${hexByteText(decoded.infoCode)} / 原因 0x${hexByteText(reason)}`,
      detail: `${decoded.payloadMeaning}。请确认手柄已接入、EEPROM 已完成识别，并且当前通道对应真实接入的手柄。`,
      targetText,
      targetPage
    };
  }
  if (reason === 0x06) {
    return {
      message: "当前状态不允许 EEPROM 读写",
      code: `ACK 0x${hexByteText(decoded.infoCode)} / 原因 0x${hexByteText(reason)}`,
      detail: `${decoded.payloadMeaning}。请先申请外部控制，并确认主机未运行、未处于报警或 Busy 状态。`,
      targetText,
      targetPage
    };
  }
  return {
    message: "EEPROM 读写失败",
    code: `ACK 0x${hexByteText(decoded.infoCode)}${reason === undefined ? "" : ` / 原因 0x${hexByteText(reason)}`}`,
    detail: `${decoded.payloadMeaning || reasonText}。${targetText}，请按协议解析窗口继续核对返回原因。`,
    targetText,
    targetPage
  };
}

function showEepromErrorPopup(error, rawBytes = []) {
  const timestamp = new Date();
  // EEPROM 错误弹窗只表示本次读写命令失败，不改主故障报警 activeFault。
  setText(elements.eepromErrorTime, formatDataTimestamp(timestamp, true));
  setText(elements.eepromErrorMessage, error.message || "EEPROM 读写失败");
  setText(elements.eepromErrorCode, error.code || "ACK 0x06");
  setText(elements.eepromErrorDetail, error.detail || "请检查当前手柄接入状态、通道选择和外部控制权限。");
  setText(elements.eepromErrorRaw, rawBytes.length > 0 ? `HEX ${bytesToHex(rawBytes)}` : "HEX --");
  if (elements.eepromErrorPopup) {
    elements.eepromErrorPopup.hidden = false;
    elements.eepromErrorPopup.dataset.active = "true";
  }
}

function showEepromLocalError(message) {
  // 本地下发前拦截的错误同样弹窗，例如未连接串口或选择了固件未开放的保留页。
  showEepromErrorPopup({
    message: "EEPROM 命令未发送",
    code: "LOCAL",
    detail: message
  });
}

function hideEepromErrorPopup() {
  if (elements.eepromErrorPopup) {
    elements.eepromErrorPopup.hidden = true;
    elements.eepromErrorPopup.dataset.active = "false";
  }
}

function hexByteText(value) {
  return Number(value ?? 0).toString(16).toUpperCase().padStart(2, "0");
}

function renderEepromPageSelectors() {
  const selects = [elements.eepromPageSelect, elements.eepromBatchStart, elements.eepromBatchEnd].filter(Boolean);
  for (const select of selects) {
    select.replaceChildren();
    for (let page = EEPROM_MIN_PAGE; page <= EEPROM_MAX_PAGE; page += 1) {
      const option = document.createElement("option");
      option.value = String(page);
      option.textContent = buildEepromOptionText(page);
      select.appendChild(option);
    }
  }
  setValue(elements.eepromPageSelect, EEPROM_MIN_PAGE);
  setValue(elements.eepromBatchStart, EEPROM_MIN_PAGE);
  setValue(elements.eepromBatchEnd, 12);
  renderEepromPageStatus();
}

function buildEepromOptionText(page) {
  const title = getEepromPageTitle(page);
  return `Page${page} / ${title}`;
}

function getEepromPageTitle(page) {
  if (EEPROM_BUSINESS_PAGES.has(page)) {
    return EEPROM_BUSINESS_PAGES.get(page).title;
  }
  if (page === 7) {
    return EEPROM_EXTENSION_PAGES.get(page).title;
  }
  if (page === 10) {
    return EEPROM_EXTENSION_PAGES.get(page).title;
  }
  if (page >= 12 && page <= EEPROM_MAX_PAGE) {
    return "导航数据区域";
  }
  return "未定义区域";
}

function renderEepromPageStatus() {
  const page = readEepromSelectValue(elements.eepromPageSelect, EEPROM_MIN_PAGE);
  const batchPages = readEepromBatchPages();
  const skippedPages = batchPages.filter((item) => !isEepromPageSupported(item));
  const pageKind = EEPROM_EXTENSION_PAGES.has(page) ? "业务 EEPROM 续页" : (EEPROM_BUSINESS_PAGES.has(page) ? "业务 EEPROM 单页" : "导航 EEPROM 单页");
  const pageStatus = `Page${page} 将按${pageKind}命令读写。`;
  const batchStatus = skippedPages.length > 0 ? `批量读取会跳过 ${skippedPages.map((item) => `Page${item}`).join("、")}。` : `批量范围 ${formatPageRange(batchPages)}。`;
  setText(elements.eepromPageStatus, `${pageStatus} ${batchStatus}`);
}

function readEepromSelectValue(select, fallback) {
  const value = Number(select?.value ?? fallback);
  if (!Number.isInteger(value)) {
    return fallback;
  }
  return clamp(value, EEPROM_MIN_PAGE, EEPROM_MAX_PAGE);
}

function readSelectedEepromPage() {
  return readEepromSelectValue(elements.eepromPageSelect, EEPROM_MIN_PAGE);
}

function readEepromBatchPages() {
  const start = readEepromSelectValue(elements.eepromBatchStart, EEPROM_MIN_PAGE);
  const end = readEepromSelectValue(elements.eepromBatchEnd, start);
  return buildEepromPageRange(start, end);
}

function buildEepromPageRange(start, end) {
  // 批量动作统一通过页码数组驱动，便于全部页、导航页和自定义范围共用同一发送逻辑。
  const lower = Math.min(start, end);
  const upper = Math.max(start, end);
  const pages = [];
  for (let page = lower; page <= upper; page += 1) {
    pages.push(page);
  }
  return pages;
}

function isEepromPageSupported(page) {
  return EEPROM_BUSINESS_PAGES.has(page) || EEPROM_EXTENSION_PAGES.has(page) || (page >= 12 && page <= EEPROM_MAX_PAGE);
}

function buildEepromPageCommand(page, operation, payloadOverride = null) {
  const pageNumber = readEepromSelectValue({ value: page }, EEPROM_MIN_PAGE);
  const payload = operation === "write" ? (payloadOverride || readEepromPayloadBytes()) : [];
  const business = EEPROM_BUSINESS_PAGES.get(pageNumber);
  if (business) {
    // Page2-Page6、Page8、Page9、Page11 由固件业务 AreaCode 映射到真实 EEPROM 页。
    return buildFrame(0x02, operation === "write" ? 0x07 : 0x05, business.areaCode, 0xFF, payload);
  }
  if (EEPROM_EXTENSION_PAGES.has(pageNumber)) {
    // Page7/Page10 是业务扩展续页，按实际 EEPROM Page 号读取，避免 Page6/Page9 写满后续页看不到。
    return buildFrame(0x02, operation === "write" ? 0x0A : 0x08, pageNumber, 0xFF, payload);
  }
  if (pageNumber >= 12 && pageNumber <= EEPROM_MAX_PAGE) {
    // Page12-Page128 是导航区，固件直接用 AreaCode 表示实际页号。
    return buildFrame(0x02, operation === "write" ? 0x0A : 0x08, pageNumber, 0xFF, payload);
  }
  throw new Error(`Page${pageNumber} 当前固件不支持读写`);
}

function readEepromPayloadBytes() {
  const payload = hexToBytes(elements.eepromBytes?.value || "");
  if (payload.length !== 30) {
    throw new Error(`EEPROM 写入数据必须是 30 字节，当前 ${payload.length} 字节`);
  }
  return payload;
}

function getSelectedEepromTemplateBytes() {
  const templates = appState.config.eepromTemplates || [];
  if (templates.length === 0) {
    throw new Error("当前配置没有 EEPROM 模板，不能批量写入导航页");
  }
  const selectedIndex = Number(elements.eepromTemplateSelect?.value || 0);
  const template = templates[selectedIndex];
  if (!template) {
    throw new Error("EEPROM 模板索引无效，不能批量写入导航页");
  }
  // 批量写导航页直接读取模板字节，不依赖 textarea，避免现场误改输入框后写入错误导航数据。
  const payload = Array.isArray(template.bytes) ? template.bytes : hexToBytes(template.bytes || "");
  if (payload.length !== 30) {
    throw new Error(`EEPROM 模板必须是 30 字节，当前 ${payload.length} 字节`);
  }
  return payload;
}

function pageFromBusinessAreaText(areaCodeText) {
  const [areaCode] = hexToBytes(areaCodeText || "");
  for (const [page, info] of EEPROM_BUSINESS_PAGES) {
    if (info.areaCode === areaCode) {
      return page;
    }
  }
  return null;
}

function formatPageRange(pages) {
  if (pages.length === 0) {
    return "未选择";
  }
  if (pages.length === 1) {
    return `Page${pages[0]}`;
  }
  return `Page${pages[0]}-Page${pages[pages.length - 1]}`;
}

function formatPageList(views) {
  const pages = views.map((view) => view.page);
  if (pages.length <= 6) {
    return pages.map((page) => `Page${page}`).join("、");
  }
  return `${pages.slice(0, 3).map((page) => `Page${page}`).join("、")} ... ${pages.slice(-2).map((page) => `Page${page}`).join("、")}`;
}

function renderEepromView() {
  const views = appState.eepromViews.length > 0 ? appState.eepromViews : (appState.eepromView ? [appState.eepromView] : []);
  if (!elements.eepromHexGrid || !elements.eepromExplainList) {
    return;
  }
  if (views.length === 0) {
    setText(elements.eepromPageMeta, "等待读取选中 Page 或批量 Page");
    setText(elements.eepromPageTitle, "--");
    setText(elements.eepromPageAddress, "--");
    setText(elements.eepromDataLength, "--");
    setText(elements.eepromPageNote, "每页 32 字节，当前固件上传前 30 字节有效数据；末尾 2 字节页校验由 MCU 内部校验。");
    elements.eepromHexGrid.innerHTML = '<p class="empty">点击“读取选中 Page”或“批量读取 Page”后，这里显示该 Page 上传的全部有效数据。</p>';
    elements.eepromExplainList.innerHTML = '<p class="empty">根据 EIDE/eeprom布局说明.txt 自动解释字段。</p>';
    return;
  }

  const latest = views[views.length - 1];
  setText(elements.eepromPageMeta, views.length === 1 ? latest.source : `批量显示 ${views.length} 页：${formatPageList(views)}`);
  setText(elements.eepromPageTitle, views.length === 1 ? latest.title : `批量 Page 数据（${views.length} 页）`);
  setText(elements.eepromPageAddress, views.length === 1 ? `0x${latest.startAddress.toString(16).toUpperCase().padStart(4, "0")}` : "批量");
  setText(elements.eepromDataLength, `${views.reduce((sum, view) => sum + view.dataBytes.length, 0)} 字节`);
  setText(elements.eepromPageNote, latest.note);
  elements.eepromHexGrid.innerHTML = views.map(renderEepromHexBlock).join("");
  elements.eepromExplainList.innerHTML = views.map(renderEepromExplainBlock).join("");
}

function beginEepromTiming(action, totalPages = 1, operationType = "single") {
  const startedAt = new Date();
  // 时间统计记录本机实际下发 EEPROM 命令的耗时，批量读写会覆盖整批 Page 的发送过程。
  appState.eepromOperationType = operationType;
  appState.eepromTiming = {
    action,
    totalPages,
    startedAt,
    endedAt: null,
    durationMs: null,
    status: "进行中"
  };
  renderEepromTiming();
}

function finishEepromTiming(status = "完成") {
  if (!appState.eepromTiming) {
    return;
  }
  const endedAt = new Date();
  appState.eepromTiming = {
    ...appState.eepromTiming,
    endedAt,
    durationMs: endedAt.getTime() - appState.eepromTiming.startedAt.getTime(),
    status
  };
  renderEepromTiming();
}

function resetEepromTiming() {
  appState.eepromTiming = null;
  appState.eepromOperationType = null;
  renderEepromTiming();
}

function formatEepromTimingDuration(durationMs) {
  if (!Number.isFinite(durationMs)) {
    return "统计中...";
  }
  if (durationMs < 1000) {
    return `${durationMs} ms`;
  }
  return `${(durationMs / 1000).toFixed(2)} s`;
}

function renderEepromTiming() {
  const timing = appState.eepromTiming;
  if (!timing) {
    setText(elements.eepromTimingDuration, "--");
    setText(elements.eepromTimingAction, "最近动作：--");
    setText(elements.eepromTimingStart, "开始：--");
    setText(elements.eepromTimingEnd, "结束：--");
    return;
  }
  const pageText = timing.totalPages > 1 ? ` / ${timing.totalPages} 页` : "";
  setText(elements.eepromTimingDuration, formatEepromTimingDuration(timing.durationMs));
  setText(elements.eepromTimingAction, `最近动作：${timing.action}${pageText} / ${timing.status}`);
  setText(elements.eepromTimingStart, `开始：${formatDataTimestamp(timing.startedAt, true)}`);
  setText(elements.eepromTimingEnd, `结束：${timing.endedAt ? formatDataTimestamp(timing.endedAt, true) : "--"}`);
}

function addFailedEepromView(error) {
  const page = error.targetPage;
  if (!page) {
    return;
  }
  const view = createFailedEepromView(page, error, "该 Page 未返回 30 字节有效数据，可能为空页、未写入，或 EEPROM 芯片读页失败。");
  appState.eepromViews = appState.eepromViews.filter((item) => item.page !== page).concat(view)
    .sort((left, right) => left.page - right.page);
  appState.eepromView = view;
  renderEepromView();
}

function addSkippedEepromViews(pages) {
  if (pages.length === 0) {
    return;
  }
  // 非支持页不会产生 ACK，也要在批量结果里明确显示，避免用户误以为漏读。
  const skippedViews = pages.map((page) => createFailedEepromView(page, {
    code: "SKIP",
    targetText: `Page${page}`,
    targetPage: page
  }, "该 Page 当前未下发读取命令，请确认该页是否属于当前固件支持范围。"));
  appState.eepromViews = appState.eepromViews.concat(skippedViews)
    .sort((left, right) => left.page - right.page);
}

function createFailedEepromView(page, error, message) {
  const view = {
    page,
    title: `${getEepromPageTitle(page)} / 未写入或空页`,
    source: error.code === "SKIP" ? `${error.targetText || `Page${page}`} 未下发读取` : `${error.targetText || `Page${page}`} 返回失败 ACK`,
    startAddress: (page - 1) * 32,
    dataBytes: [],
    rawHex: "",
    failed: true,
    error,
    rows: [],
    fields: [{
      offsetText: "--",
      name: "读取结果",
      hex: error.code || "ACK 0x06",
      value: message
    }],
    note: "该 Page 本次没有有效 EEPROM 数据；已保留其他成功读取 Page 的可视化结果。"
  };
  return view;
}

function renderEepromHexBlock(view) {
  if (view.failed) {
    return `
      <section class="eeprom-page-block eeprom-page-failed">
        <div class="eeprom-page-prefix">Page${escapeHtml(view.page)}：未写入/空页</div>
        <p class="eeprom-empty-page">${escapeHtml(view.fields[0]?.value || "该 Page 未返回有效数据。")}</p>
      </section>
    `;
  }
  return `
    <section class="eeprom-page-block">
      <div class="eeprom-page-prefix">Page${escapeHtml(view.page)}：</div>
      ${view.rows.map((row) => `
        <div class="eeprom-hex-row">
          <span class="eeprom-offset">${escapeHtml(row.addressText)}</span>
          <span class="eeprom-byte-list">
            ${row.bytes.map((byte) => `<code title="Page${escapeHtml(view.page)} 页内偏移 ${byte.offset}">${escapeHtml(byte.text)}</code>`).join("")}
          </span>
        </div>
      `).join("")}
    </section>
  `;
}

function renderEepromExplainBlock(view) {
  return `
    <section class="eeprom-page-block${view.failed ? " eeprom-page-failed" : ""}">
      <div class="eeprom-page-prefix">Page${escapeHtml(view.page)}：${escapeHtml(view.title)}</div>
      ${view.fields.map((field) => `
        <div class="eeprom-field-row">
          <span>${escapeHtml(field.offsetText)}</span>
          <strong>${escapeHtml(field.name)}</strong>
          <code>${escapeHtml(field.hex)}</code>
          <em>${escapeHtml(field.value)}</em>
        </div>
      `).join("")}
    </section>
  `;
}

function setText(element, value) {
  if (element) {
    element.textContent = String(value);
  }
}

function updateChartValueStrip(values) {
  // 图表区也保留最新数值，现场看曲线时不用再回到上方状态卡片对照。
  setText(elements.chartValueSpeed, values.speed);
  setText(elements.chartValueCurrent, values.current);
  setText(elements.chartValuePumpA, values.pumpA);
  setText(elements.chartValuePumpB, values.pumpB);
}

function updatePressureSensorPanel(telemetry) {
  // 压力窗口读取心跳里由 SimUartTask 写入的 pumpMessageA/B 扩展字段，直接显示 CS1237 原始值和换算重量。
  const pumpARaw = telemetry.pumpA?.pressureRaw;
  const pumpBRaw = telemetry.pumpB?.pressureRaw;
  setText(elements.pressureRawA, formatPressureRaw(pumpARaw));
  setText(elements.pressureWeightA, formatWeightX10(telemetry.pumpA?.weightX10));
  setText(elements.pressureSeqA, formatPressureSeq(telemetry.pumpA));
  setText(elements.pressureThresholdA, formatPressureThreshold(telemetry.pumpA));
  setText(elements.pressureRawB, formatPressureRaw(pumpBRaw));
  setText(elements.pressureWeightB, formatWeightX10(telemetry.pumpB?.weightX10));
  setText(elements.pressureSeqB, formatPressureSeq(telemetry.pumpB));
  setText(elements.pressureThresholdB, formatPressureThreshold(telemetry.pumpB));
}

function resetPressureSensorPanel() {
  // 清空数据可视化或心跳超时时，压力传感器窗口也回到空状态，避免残留旧重量误导联调。
  setText(elements.pressureRawA, "--");
  setText(elements.pressureWeightA, "--");
  setText(elements.pressureSeqA, "Seq --");
  setText(elements.pressureThresholdA, "阈值 --");
  setText(elements.pressureRawB, "--");
  setText(elements.pressureWeightB, "--");
  setText(elements.pressureSeqB, "Seq --");
  setText(elements.pressureThresholdB, "阈值 --");
}

function formatPressureRaw(value) {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return String(value);
}

function formatWeightX10(value) {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return `${(Number(value) / 10).toFixed(1)} g`;
}

function formatPressureSeq(pump) {
  if (!Number.isFinite(pump?.pressureSeq)) {
    return "Seq --";
  }
  return `Seq ${pump.pressureSeq}`;
}

function formatPressureThreshold(pump) {
  if (!Number.isFinite(pump?.thresholdG)) {
    return "阈值 --";
  }
  return `阈值 ${pump.thresholdG} g`;
}

function pushPressureTrendPoint(telemetry) {
  // 压力趋势只记录真实收到的 CS1237 扩展字段；缺失字段保留为 null，绘图时自动断线。
  const point = {
    time: Date.now(),
    rawA: finiteOrNull(telemetry.pumpA?.pressureRaw),
    rawB: finiteOrNull(telemetry.pumpB?.pressureRaw),
    weightA: weightX10ToGram(telemetry.pumpA?.weightX10),
    weightB: weightX10ToGram(telemetry.pumpB?.weightX10),
    thresholdA: finiteOrNull(telemetry.pumpA?.thresholdG),
    thresholdB: finiteOrNull(telemetry.pumpB?.thresholdG)
  };
  const hasPressureValue = [point.rawA, point.rawB, point.weightA, point.weightB].some((value) => Number.isFinite(value));
  if (!hasPressureValue) {
    return;
  }
  appState.pressureTrend.push(point);
  appState.pressureTrend = appState.pressureTrend.slice(-80);
}

function finiteOrNull(value) {
  return Number.isFinite(value) ? Number(value) : null;
}

function weightX10ToGram(value) {
  return Number.isFinite(value) ? Number(value) / 10 : null;
}

function resetParserPanel() {
  if (!elements.parserPanel) {
    return;
  }
  // 清空显示后保留窗口说明，避免用户误以为解析窗口加载失败。
  elements.parserPanel.innerHTML = '<p class="empty">显示已清空，等待 TX/RX 帧或手动粘贴解析。</p>';
}

function clearDisplayData() {
  // 只清理界面层缓存，不断开串口、不改参数输入、不影响后续实时接收。
  appState.rxBuffer = [];
  appState.lastDecodedFrame = null;
  appState.activeFault = null;
  clearLogDisplay();
  clearChartDisplay();
  clearEepromDisplay();
  clearParserDisplay();
  clearManualParseDisplay();
  hideFaultPopup();
  clearRealtimeDisplay();
}

function clearLogDisplay() {
  appState.logs = [];
  appState.pendingLogCount = 0;
  renderLogs();
}

function clearChartDisplay() {
  appState.trend = [];
  appState.pressureTrend = [];
  setText(elements.chartValueSpeed, "--");
  setText(elements.chartValueCurrent, "--");
  setText(elements.chartValuePumpA, "--");
  setText(elements.chartValuePumpB, "--");
  resetPressureSensorPanel();
  drawTrend();
  drawPressureTrend();
}

function clearEepromDisplay() {
  appState.eepromView = null;
  appState.eepromViews = [];
  appState.eepromBatchMode = false;
  resetEepromTiming();
  renderEepromView();
  hideEepromErrorPopup();
}

function clearParserDisplay() {
  appState.lastDecodedFrame = null;
  resetParserPanel();
}

function clearManualParseDisplay() {
  setValue(elements.rawInput, "");
}

function clearRealtimeDisplay() {
  resetRuntimeStatus(appState.serial.connected ? "等待心跳" : "未连接", false);
}

function drawTrend() {
  const canvas = elements.chartCanvas;
  if (!(canvas instanceof HTMLCanvasElement)) {
    return;
  }
  const ctx = canvas.getContext("2d");
  const width = canvas.width = canvas.clientWidth * window.devicePixelRatio;
  const height = canvas.height = canvas.clientHeight * window.devicePixelRatio;
  ctx.clearRect(0, 0, width, height);
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

  const cssWidth = canvas.clientWidth;
  const cssHeight = canvas.clientHeight;
  const plot = {
    left: 46,
    top: 10,
    right: 10,
    bottom: 22
  };
  const plotWidth = Math.max(1, cssWidth - plot.left - plot.right);
  const plotHeight = Math.max(1, cssHeight - plot.top - plot.bottom);
  // 最大值至少为 1，避免所有字段为 0 时出现除零。
  const maxValue = Math.max(1, ...appState.trend.flatMap((point) => [point.speed, point.current, point.pumpA, point.pumpB]));

  ctx.strokeStyle = "#d6ca8d";
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i += 1) {
    const y = plot.top + (plotHeight / 5) * i;
    ctx.beginPath();
    ctx.moveTo(plot.left, y);
    ctx.lineTo(cssWidth - plot.right, y);
    ctx.stroke();
  }

  drawChartScale(ctx, maxValue, plot, plotHeight);
  drawLine(ctx, "speed", "#ffcc00", maxValue, plot, plotWidth, plotHeight);
  drawLine(ctx, "current", "#00a66a", maxValue, plot, plotWidth, plotHeight);
  drawLine(ctx, "pumpA", "#ff8a00", maxValue, plot, plotWidth, plotHeight);
  drawLine(ctx, "pumpB", "#e6002e", maxValue, plot, plotWidth, plotHeight);
}

function drawChartScale(ctx, maxValue, plot, plotHeight) {
  // 左侧刻度给曲线提供数值参照，便于脱离日志直接判断当前量级。
  const labels = [
    { value: maxValue, y: plot.top + 6 },
    { value: Math.round(maxValue / 2), y: plot.top + (plotHeight / 2) + 4 },
    { value: 0, y: plot.top + plotHeight - 2 }
  ];
  ctx.fillStyle = "#58645f";
  ctx.font = "11px Consolas, 'Courier New', monospace";
  ctx.textAlign = "right";
  for (const label of labels) {
    ctx.fillText(String(label.value), plot.left - 8, label.y);
  }
}

function drawLine(ctx, key, color, maxValue, plot, width, height) {
  const points = appState.trend;
  if (points.length < 2) {
    return;
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  points.forEach((point, index) => {
    const x = plot.left + (index / (points.length - 1)) * width;
    const y = plot.top + height - (Number(point[key] || 0) / maxValue) * height;
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();
}

function drawPressureTrend() {
  // 压力原始值和重量量级不同，拆成两个画布，避免 RawCs1237 把重量曲线压成一条直线。
  drawPressureTrendCanvas(elements.pressureRawTrendCanvas, [
    { key: "rawA", color: "#006dff" },
    { key: "rawB", color: "#ff8a00" }
  ], "");
  drawPressureTrendCanvas(elements.pressureWeightTrendCanvas, [
    { key: "weightA", color: "#00a66a" },
    { key: "weightB", color: "#e6002e" },
    { key: "thresholdA", color: "#58645f", dashed: true },
    { key: "thresholdB", color: "#7a6400", dashed: true }
  ], "g");
}

function drawPressureTrendCanvas(canvas, series, unit = "") {
  if (!(canvas instanceof HTMLCanvasElement)) {
    return;
  }
  const ctx = canvas.getContext("2d");
  const width = canvas.width = canvas.clientWidth * window.devicePixelRatio;
  const height = canvas.height = canvas.clientHeight * window.devicePixelRatio;
  ctx.clearRect(0, 0, width, height);
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

  const cssWidth = canvas.clientWidth;
  const cssHeight = canvas.clientHeight;
  const plot = {
    left: 58,
    top: 10,
    right: 10,
    bottom: 20
  };
  const plotWidth = Math.max(1, cssWidth - plot.left - plot.right);
  const plotHeight = Math.max(1, cssHeight - plot.top - plot.bottom);
  const values = appState.pressureTrend.flatMap((point) => series.map((item) => point[item.key]))
    .filter((value) => Number.isFinite(value));
  const bounds = buildPressureBounds(values);

  ctx.strokeStyle = "#d6ca8d";
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i += 1) {
    const y = plot.top + (plotHeight / 5) * i;
    ctx.beginPath();
    ctx.moveTo(plot.left, y);
    ctx.lineTo(cssWidth - plot.right, y);
    ctx.stroke();
  }

  drawPressureScale(ctx, bounds, plot, plotHeight, unit);
  for (const item of series) {
    drawPressureLine(ctx, item, bounds, plot, plotWidth, plotHeight);
  }
}

function buildPressureBounds(values) {
  if (values.length === 0) {
    return { min: 0, max: 1, span: 1 };
  }
  let min = Math.min(...values);
  let max = Math.max(...values);
  if (min === max) {
    min -= 1;
    max += 1;
  }
  const padding = Math.max(1, Math.abs(max - min) * 0.08);
  min -= padding;
  max += padding;
  return { min, max, span: max - min };
}

function drawPressureScale(ctx, bounds, plot, plotHeight, unit) {
  // 压力趋势左侧显示最大、中位、最小三个刻度，Raw 为整数，重量保留 1 位小数。
  const labels = [
    { value: bounds.max, y: plot.top + 8 },
    { value: (bounds.max + bounds.min) / 2, y: plot.top + (plotHeight / 2) + 4 },
    { value: bounds.min, y: plot.top + plotHeight - 2 }
  ];
  ctx.fillStyle = "#58645f";
  ctx.font = "10px Consolas, 'Courier New', monospace";
  ctx.textAlign = "right";
  for (const label of labels) {
    ctx.fillText(formatPressureScaleLabel(label.value, unit), plot.left - 8, label.y);
  }
}

function formatPressureScaleLabel(value, unit) {
  if (unit === "g") {
    return `${value.toFixed(1)}g`;
  }
  return String(Math.round(value));
}

function drawPressureLine(ctx, item, bounds, plot, width, height) {
  const points = appState.pressureTrend;
  if (points.length < 2) {
    return;
  }
  ctx.strokeStyle = item.color;
  ctx.lineWidth = item.dashed ? 1.5 : 2.2;
  ctx.setLineDash(item.dashed ? [5, 4] : []);
  ctx.beginPath();
  let started = false;
  points.forEach((point, index) => {
    const value = point[item.key];
    if (!Number.isFinite(value)) {
      started = false;
      return;
    }
    const x = plot.left + (index / (points.length - 1)) * width;
    const y = plot.top + height - ((value - bounds.min) / bounds.span) * height;
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function applyWorkspaceLayout() {
  if (!elements.workspace) {
    return;
  }
  // CSS 变量驱动三列宽度，拖拽时只改布局边界，不改各业务面板内容。
  elements.workspace.style.setProperty("--left-col", `${appState.layout.left}px`);
  elements.workspace.style.setProperty("--right-col", `${appState.layout.right}px`);
}

function bindLayoutResizers() {
  const minimum = {
    left: 240,
    center: 460,
    right: 320
  };
  const bindSplitter = (splitter, side) => {
    if (!splitter || !elements.workspace) {
      return;
    }
    splitter.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      const startX = event.clientX;
      const startLeft = appState.layout.left;
      const startRight = appState.layout.right;
      const move = (moveEvent) => {
        const deltaX = moveEvent.clientX - startX;
        const rect = elements.workspace.getBoundingClientRect();
        const spareWidth = Math.max(0, rect.width - minimum.center - 16);
        if (side === "left") {
          const maxLeft = Math.max(minimum.left, spareWidth - appState.layout.right);
          appState.layout.left = clamp(startLeft + deltaX, minimum.left, maxLeft);
        } else {
          const maxRight = Math.max(minimum.right, spareWidth - appState.layout.left);
          appState.layout.right = clamp(startRight - deltaX, minimum.right, maxRight);
        }
        applyWorkspaceLayout();
      };
      const stop = () => {
        document.body.classList.remove("resizing-layout");
        window.removeEventListener("pointermove", move);
        window.removeEventListener("pointerup", stop);
        window.removeEventListener("pointercancel", stop);
      };
      document.body.classList.add("resizing-layout");
      window.addEventListener("pointermove", move);
      window.addEventListener("pointerup", stop);
      window.addEventListener("pointercancel", stop);
    });
  };

  applyWorkspaceLayout();
  bindSplitter(elements.leftColumnSplitter, "left");
  bindSplitter(elements.rightColumnSplitter, "right");
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function buildCommand(action) {
  // 所有按钮动作在这里转换为协议 FunCode/AreaCode/InforArea，便于协议解析窗口同步解释。
  switch (action) {
    case "apply-control":
      return buildFrame(0x02, 0x01, 0xFF, 0xFF, hexToBytes(elements.authCode?.value || appState.config.authCode));
    case "host-exit":
      return buildFrame(0x02, 0xBB, 0xFF, 0xFF, []);
    case "permission":
      return buildFrame(0x02, 0xFA, 0xFF, 0xFF, hexToBytes(elements.authCode?.value || appState.config.authCode));
    case "set-speed":
      return buildSetSpeedFrame(readSpeedInputValue());
    case "set-freq": {
      if (!canSetFrequency()) {
        throw new Error("仅 PXBA/PXBB 往复转手柄允许设置频率");
      }
      const freq = readNumberInput(elements.freqInput, appState.config.presets.freq);
      const payload = freq <= 0xFF ? [freq] : toBE16(freq);
      return buildFrame(0x02, 0x02, 0x02, 0xFF, payload);
    }
    case "set-pump-a":
      return buildFrame(0x02, 0x02, 0x03, 0xFF, toBE16(readNumberInput(elements.pumpAInput, appState.config.presets.pumpA)));
    case "set-pump-b":
      return buildFrame(0x02, 0x02, 0x04, 0xFF, toBE16(readNumberInput(elements.pumpBInput, appState.config.presets.pumpB)));
    case "switch-a":
      return buildFrame(0x02, 0x03, 0x01, 0xFF, []);
    case "switch-b":
      return buildFrame(0x02, 0x03, 0x02, 0xFF, []);
    case "dir-forward":
      return buildFrame(0x02, 0x03, 0x03, 0xFF, [0x01]);
    case "dir-reverse":
      return buildFrame(0x02, 0x03, 0x03, 0xFF, [0x02]);
    case "dir-osc":
      return buildFrame(0x02, 0x03, 0x03, 0xFF, [0x03]);
    case "mode-foot":
      return buildFrame(0x02, 0x03, 0x04, 0xFF, [0x01]);
    case "mode-handle":
      return buildFrame(0x02, 0x03, 0x04, 0xFF, [0x02]);
    case "mode-external":
      return buildFrame(0x02, 0x03, 0x04, 0xFF, [0x03]);
    case "tool-planer":
      return buildFrame(0x02, 0x03, 0x05, 0xFF, [0x01]);
    case "tool-grind":
      return buildFrame(0x02, 0x03, 0x05, 0xFF, [0x02]);
    case "pump-a-start":
      return buildFrame(0x02, 0x04, 0x01, 0xFF, []);
    case "pump-a-stop":
      return buildFrame(0x02, 0x04, 0x02, 0xFF, []);
    case "pump-b-start":
      return buildFrame(0x02, 0x04, 0x03, 0xFF, []);
    case "pump-b-stop":
      return buildFrame(0x02, 0x04, 0x04, 0xFF, []);
    case "handle-start":
      return buildFrame(0x02, 0x04, 0x05, 0xFF, []);
    case "handle-stop":
      return buildFrame(0x02, 0x04, 0x06, 0xFF, []);
    case "tool-left":
      return buildFrame(0x02, 0x04, 0x07, 0xFF, []);
    case "tool-right":
      return buildFrame(0x02, 0x04, 0x08, 0xFF, []);
    case "emergency-stop":
      return buildFrame(0x02, 0x04, 0xFF, 0xFF, []);
    case "read-eeprom-page":
      return buildEepromPageCommand(readSelectedEepromPage(), "read");
    case "write-eeprom-page":
      return buildEepromPageCommand(readSelectedEepromPage(), "write");
    default:
      throw new Error(`未定义按钮动作：${action}`);
  }
}

async function sendCommand(action) {
  validateOperationOrder(action);
  if (action === "handle-start" && elements.dynamicSpeedToggle?.checked) {
    // 旧页面状态可能保留“动态调速”勾选，但用户本次启动前没有拖动滑块；这里强制补发一次当前速度。
    clearDynamicSpeedTimer();
    // 先写入 WorkMessage.speed_set_work，避免随后启动手柄时固件拿到 0 速度而只有注水泵转动。
    const speed = await transmitCurrentHandleSpeed();
    // 速度预下发成功后立即反馈，若失败会抛出并阻止后续手柄启动命令。
    setDynamicSpeedStatus(`启动前已下发速度 ${speed}`, "ok");
    // 两帧之间留出一个短间隔，降低 MCU UART 空闲接收把速度帧和启动帧粘成一包的概率。
    await delay(HANDLE_START_PREFLIGHT_DELAY_MS);
  }
  const bytes = buildCommand(action);
  await transmitBytes(bytes);
  if (action === "apply-control") {
    setControlApplyStatus("申请已发送，等待主机 ACK", "pending");
  }
  if (action === "host-exit") {
    setControlApplyStatus("退出外部控制已发送，等待主机 ACK", "pending");
  }
}

async function transmitBytes(bytes) {
  const [decoded] = parseFrames(bytes).frames;
  const entry = appendLog("TX", bytes, decoded);
  showParser(makeTimestampDecoded(entry), bytes);
  await appState.serial.send(bytes);
}

async function sendEepromBatchRead() {
  await sendEepromBatchReadPages(readEepromBatchPages(), "批量读取 Page");
}

async function sendEepromReadAllPages() {
  await sendEepromBatchReadPages(buildEepromPageRange(EEPROM_MIN_PAGE, EEPROM_MAX_PAGE), "批量读取全部页");
}

async function sendEepromReadBusinessPages() {
  // 业务页覆盖 Page2-Page11，其中 Page7/Page10 是 Page6/Page9 写满后的真实续页。
  await sendEepromBatchReadPages(buildEepromPageRange(2, 11), "批量读取业务页");
}

async function sendEepromReadNavPages() {
  await sendEepromBatchReadPages(buildEepromPageRange(EEPROM_NAV_START_PAGE, EEPROM_NAV_END_PAGE), "批量读取导航页");
}

async function sendEepromBatchReadPages(pages, label) {
  const supportedPages = pages.filter(isEepromPageSupported);
  const skippedPages = pages.filter((page) => !isEepromPageSupported(page));
  if (supportedPages.length === 0) {
    throw new Error("批量范围内没有当前固件可读取的 EEPROM Page");
  }
  beginEepromTiming(label, supportedPages.length, "batch-read");
  // 批量读取逐页发送单页命令，不使用固件已声明不支持的整区读取命令。
  appState.eepromBatchMode = true;
  appState.eepromView = null;
  appState.eepromViews = [];
  addSkippedEepromViews(skippedPages);
  renderEepromView();
  if (skippedPages.length > 0) {
    appendTextLog("SYS", `批量读取跳过未开放页：${skippedPages.map((page) => `Page${page}`).join("、")}`, "warning");
  }
  appendTextLog("SYS", `开始批量读取 ${formatPageRange(supportedPages)}，共 ${supportedPages.length} 页`, "info");
  try {
    for (const page of supportedPages) {
      await transmitBytes(buildEepromPageCommand(page, "read"));
      await delay(EEPROM_PAGE_DELAY_MS);
    }
    finishEepromTiming("完成");
  } catch (error) {
    finishEepromTiming("失败");
    throw error;
  }
}

async function sendEepromBatchWriteNavTemplate() {
  const payload = getSelectedEepromTemplateBytes();
  if (appState.config.safety.confirmWrites && !confirm("将把当前 EEPROM 模板批量写入 Page12-Page128 导航数据页，确认继续？")) {
    return;
  }
  const pages = buildEepromPageRange(EEPROM_NAV_START_PAGE, EEPROM_NAV_END_PAGE);
  beginEepromTiming("模板批量写导航页", pages.length, "batch-write");
  // 导航页模板批量写只写 Page12-Page128，不触碰 Page2-Page11 的业务配置页。
  appendTextLog("SYS", `开始模板批量写导航页 ${formatPageRange(pages)}，共 ${pages.length} 页`, "warning");
  try {
    for (const page of pages) {
      await transmitBytes(buildEepromPageCommand(page, "write", payload));
      await delay(EEPROM_PAGE_DELAY_MS);
    }
    finishEepromTiming("完成");
  } catch (error) {
    finishEepromTiming("失败");
    throw error;
  }
}

function delay(ms) {
  return new Promise((resolve) => {
    window.setTimeout(resolve, ms);
  });
}

function bindCommandButtons() {
  for (const button of $all("[data-action]")) {
    if (button.dataset.action === "emergency-stop") {
      // 急停由长按逻辑独立处理，避免短点击触发一次额外下发。
      continue;
    }
    button.addEventListener("click", async () => {
      const action = button.dataset.action;
      if (action === "read-eeprom-page") {
        // 单页读取重新进入单页显示模式，避免上一次批量结果继续残留。
        appState.eepromBatchMode = false;
        appState.eepromViews = [];
      }
      if (action === "write-eeprom-page") {
        // EEPROM 写会真实修改当前选中通道，默认必须二次确认。
        if (appState.config.safety.confirmWrites && !confirm("EEPROM 写入会修改当前选中通道数据，确认继续？")) {
          return;
        }
        appState.eepromBatchMode = false;
      }
      try {
        if (action === "read-eeprom-page" || action === "write-eeprom-page") {
          beginEepromTiming(action === "read-eeprom-page" ? "读取选中 Page" : "写入选中 Page", 1, action === "read-eeprom-page" ? "single-read" : "single-write");
        }
        await sendCommand(action);
        if (action === "set-speed") {
          // 手动按钮下发成功后也刷新状态，便于和动态调速反馈保持一致。
          setSpeedControlValue(readSpeedInputValue());
          setDynamicSpeedStatus(`已下发速度 ${readSpeedInputValue()}`, "ok");
        }
        if (action === "read-eeprom-page" || action === "write-eeprom-page") {
          finishEepromTiming("完成");
        }
      } catch (error) {
        if (action === "set-speed") {
          // 手动下发失败也落到速度状态条，现场不用只去日志里找原因。
          setDynamicSpeedStatus(error.message, "error");
        }
        if (action === "apply-control") {
          // 申请帧本地发送失败时，也写到按钮下方，避免现场只从高速日志里找失败原因。
          setControlApplyStatus(`申请未发送：${error.message}`, "error");
        }
        if (action === "read-eeprom-page" || action === "write-eeprom-page") {
          finishEepromTiming("失败");
          showEepromLocalError(error.message);
        } else {
          showOperationLocalError(error.message, action);
        }
        appendTextLog("SYS", error.message, "error");
      }
    });
  }
}

function bindDynamicSpeedControls() {
  // 滑块拖动时同步数字框；开启动态调速后会防抖发送 0x02/0x01 速度帧。
  elements.speedSlider?.addEventListener("input", () => {
    setSpeedControlValue(elements.speedSlider.value);
    queueDynamicSpeedSend();
  });

  // 数字框逐字符编辑时先校验，再同步滑块和动态下发状态。
  elements.speedInput?.addEventListener("input", syncSpeedFromInput);

  // 失焦时把合法速度规整回整数显示，避免后续组帧读到浏览器保留的小数文本。
  elements.speedInput?.addEventListener("change", () => {
    try {
      setSpeedControlValue(readSpeedInputValue());
      queueDynamicSpeedSend();
    } catch (error) {
      setDynamicSpeedStatus(error.message, "error");
    }
  });

  // 步进按钮方便运行中小幅加减，不需要停机后再手动输入。
  elements.speedDownButton?.addEventListener("click", () => adjustSpeed(-SPEED_STEP_VALUE));
  elements.speedUpButton?.addEventListener("click", () => adjustSpeed(SPEED_STEP_VALUE));

  // 开启后只改变后续输入行为，不主动修改当前运行速度，避免切换开关本身带来突变。
  elements.dynamicSpeedToggle?.addEventListener("change", updateDynamicSpeedModeStatus);
}

function bindEmergencyButton() {
  const button = $('[data-action="emergency-stop"]');
  if (!button) {
    return;
  }
  const startHold = () => {
    button.classList.add("holding");
    appState.emergencyTimer = window.setTimeout(async () => {
      try {
        await sendCommand("emergency-stop");
      } catch (error) {
        showOperationLocalError(error.message, "emergency-stop");
        appendTextLog("SYS", error.message, "error");
      }
    }, appState.config.safety.holdEmergencyMs);
  };
  const cancelHold = () => {
    button.classList.remove("holding");
    window.clearTimeout(appState.emergencyTimer);
    appState.emergencyTimer = null;
  };
  button.addEventListener("pointerdown", startHold);
  button.addEventListener("pointerup", cancelHold);
  button.addEventListener("pointerleave", cancelHold);
}

function appendTextLog(direction, text, level = "info") {
  // 系统日志没有协议字节，用空数组占位，点击时仍可看到说明。
  appendLog(direction, [], { valid: true, summary: text, checks: {}, fields: [] }, level);
}

function bindSerial() {
  elements.connectButton?.addEventListener("click", async () => {
    try {
      if (appState.serial.connected) {
        await appState.serial.disconnect();
      } else {
        await appState.serial.connect(appState.config.serial);
      }
    } catch (error) {
      appendTextLog("SYS", error.message, "error");
    }
  });

  appState.serial.addEventListener("status", (event) => {
    const connected = event.detail.connected;
    appState.serialConnectedAt = connected ? Date.now() : 0;
    appState.lastHeartbeatAt = 0;
    appState.runtimeTimedOut = false;
    appState.externalControlGranted = false;
    setControlApplyStatus(connected ? "串口已连接，等待申请外部控制" : "尚未申请外部控制", "pending");
    if (connected) {
      resetRuntimeStatus("等待心跳", false);
    } else {
      resetRuntimeStatus("未连接", false);
      appState.rxBuffer = [];
    }
    if (elements.connectButton) {
      elements.connectButton.textContent = connected ? "断开串口" : "连接串口";
    }
    updateDynamicSpeedModeStatus();
  });

  appState.serial.addEventListener("tx", (event) => {
    // sendCommand 已经先写 TX 日志，这里只保留串口层事件入口，避免重复刷屏。
    void event;
  });

  appState.serial.addEventListener("rx", (event) => {
    appState.rxBuffer.push(...Array.from(event.detail.bytes));
    const parsed = parseFrames(appState.rxBuffer);
    appState.rxBuffer = parsed.remaining;
    for (const frame of parsed.frames) {
      const entry = appendLog("RX", frame.rawBytes, frame, frame.valid ? "info" : "error");
      showParser(makeTimestampDecoded(entry), frame.rawBytes);
      applyTelemetry(frame);
      applyEepromFrame(frame);
      applyOperationAckFrame(frame, frame.rawBytes);
      applyEepromAckFrame(frame, frame.rawBytes);
      applyAlarmFrame(frame, frame.rawBytes);
    }
  });

  appState.serial.addEventListener("error", (event) => {
    resetRuntimeStatus("串口异常", false);
    appendTextLog("SYS", event.detail.error?.message || "串口读取异常", "error");
  });
}

function bindParserTools() {
  $("#parseRawButton")?.addEventListener("click", () => {
    try {
      const bytes = hexToBytes(elements.rawInput?.value || "");
      const parsed = parseFrames(bytes);
      const frame = makeManualParseFrame(bytes, parsed);
      showParser(frame, bytes);
      applyTelemetry(frame);
      applyEepromFrame(frame);
      applyOperationAckFrame(frame, bytes);
      applyEepromAckFrame(frame, bytes);
      applyAlarmFrame(frame, bytes);
    } catch (error) {
      appendTextLog("SYS", error.message, "error");
    }
  });

  $("#exportLogButton")?.addEventListener("click", () => exportLogs("jsonl"));
  $("#exportCsvButton")?.addEventListener("click", () => exportLogs("csv"));
  elements.clearDisplayButton?.addEventListener("click", clearDisplayData);
  elements.clearRealtimeButton?.addEventListener("click", clearRealtimeDisplay);
  elements.clearChartButton?.addEventListener("click", clearChartDisplay);
  elements.clearEepromViewButton?.addEventListener("click", clearEepromDisplay);
  elements.clearLogButton?.addEventListener("click", clearLogDisplay);
  elements.clearManualParseButton?.addEventListener("click", clearManualParseDisplay);
  elements.clearParserButton?.addEventListener("click", clearParserDisplay);
  elements.timestampToggle?.addEventListener("change", () => {
    appState.showDataTimestamp = Boolean(elements.timestampToggle.checked);
    renderLogs({ preserveManualView: appState.logMode === "manual" });
  });
  elements.logScrollModeButton?.addEventListener("click", () => setLogMode("scroll"));
  elements.logManualModeButton?.addEventListener("click", () => setLogMode("manual"));
  elements.logRefreshButton?.addEventListener("click", showPendingLogs);
}

function bindFaultPopup() {
  // 关闭只隐藏当前弹窗；如果 MCU 后续继续上传非零报警码，会再次弹出提醒现场人员。
  elements.faultCloseButton?.addEventListener("click", hideFaultPopup);
}

function bindEepromErrorPopup() {
  // EEPROM 错误弹窗允许现场手动关闭；新的失败 ACK 到来时会重新打开。
  elements.eepromErrorCloseButton?.addEventListener("click", hideEepromErrorPopup);
}

function bindOperationPopup() {
  // 操作顺序提示可手动关闭；新的本地拦截或 MCU 失败 ACK 会重新弹出。
  elements.operationCloseButton?.addEventListener("click", hideOperationPopup);
}

function bindConfigTools() {
  // 手动文本框导入适合现场复制 JSON；文件导入适合长期保存预设。
  $("#applyConfigButton")?.addEventListener("click", () => {
    try {
      appState.config = validateConfig(JSON.parse(elements.configInput?.value || "{}"));
      applyConfigToForm();
      setText(elements.configStatus, "配置已应用");
    } catch (error) {
      setText(elements.configStatus, error.message);
    }
  });

  elements.configFile?.addEventListener("change", async () => {
    const [file] = elements.configFile.files;
    if (!file) {
      return;
    }
    try {
      appState.config = await importConfigFile(file);
      applyConfigToForm();
      setText(elements.configStatus, `已导入 ${file.name}`);
    } catch (error) {
      setText(elements.configStatus, error.message);
    }
  });

  elements.applyEepromTemplateButton?.addEventListener("click", () => {
    try {
      applySelectedEepromTemplate();
      setText(elements.configStatus, "EEPROM 模板已填入");
    } catch (error) {
      setText(elements.configStatus, error.message);
    }
  });

  elements.eepromPageSelect?.addEventListener("change", renderEepromPageStatus);
  elements.eepromBatchStart?.addEventListener("change", renderEepromPageStatus);
  elements.eepromBatchEnd?.addEventListener("change", renderEepromPageStatus);
  elements.readEepromBatchButton?.addEventListener("click", async () => {
      try {
        await sendEepromBatchRead();
      } catch (error) {
        showEepromLocalError(error.message);
        appendTextLog("SYS", error.message, "error");
      }
    });
  elements.readAllEepromButton?.addEventListener("click", async () => {
    try {
      await sendEepromReadAllPages();
    } catch (error) {
      showEepromLocalError(error.message);
      appendTextLog("SYS", error.message, "error");
    }
  });
  elements.readBusinessEepromButton?.addEventListener("click", async () => {
    try {
      await sendEepromReadBusinessPages();
    } catch (error) {
      showEepromLocalError(error.message);
      appendTextLog("SYS", error.message, "error");
    }
  });
  elements.readNavEepromButton?.addEventListener("click", async () => {
    try {
      await sendEepromReadNavPages();
    } catch (error) {
      showEepromLocalError(error.message);
      appendTextLog("SYS", error.message, "error");
    }
  });
  elements.batchWriteNavTemplateButton?.addEventListener("click", async () => {
    try {
      await sendEepromBatchWriteNavTemplate();
    } catch (error) {
      showEepromLocalError(error.message);
      appendTextLog("SYS", error.message, "error");
    }
  });
}

function applyConfigToForm() {
  setValue(elements.authCode, appState.config.authCode);
  setSpeedControlValue(appState.config.presets.speed);
  setValue(elements.freqInput, appState.config.presets.freq);
  setValue(elements.pumpAInput, appState.config.presets.pumpA);
  setValue(elements.pumpBInput, appState.config.presets.pumpB);
  updateDynamicSpeedModeStatus();
  renderEepromTemplates();
  if (elements.configInput) {
    elements.configInput.value = JSON.stringify(appState.config, null, 2);
  }
}

function renderEepromTemplates() {
  const select = elements.eepromTemplateSelect;
  if (!select) {
    return;
  }
  const templates = appState.config.eepromTemplates || [];
  select.replaceChildren();
  templates.forEach((template, index) => {
    // 模板名称来自配置文件，作为 textContent 写入，避免配置文本影响 DOM 结构。
    const option = document.createElement("option");
    option.value = String(index);
    const page = pageFromBusinessAreaText(template.areaCode);
    option.textContent = page ? `${template.name} / Page${page}` : `${template.name} / Area ${template.areaCode}`;
    select.appendChild(option);
  });
  if (elements.applyEepromTemplateButton) {
    elements.applyEepromTemplateButton.disabled = templates.length === 0;
  }
}

function applySelectedEepromTemplate() {
  const templates = appState.config.eepromTemplates || [];
  if (templates.length === 0) {
    throw new Error("当前配置没有 EEPROM 模板");
  }
  const selectedIndex = Number(elements.eepromTemplateSelect?.value || 0);
  const template = templates[selectedIndex];
  if (!template) {
    throw new Error("EEPROM 模板索引无效");
  }
  // 模板只填入表单，不直接下发；现场仍需点击写业务页/写导航页确认动作。
  const page = pageFromBusinessAreaText(template.areaCode);
  if (page) {
    setValue(elements.eepromPageSelect, page);
    renderEepromPageStatus();
  }
  setValue(elements.eepromBytes, template.bytes);
}

function setValue(element, value) {
  if (element) {
    element.value = String(value);
  }
}

function exportLogs(format) {
  const filename = `externalcomm-log-${new Date().toISOString().replace(/[:.]/g, "-")}.${format}`;
  let content;
  let type;
  if (format === "csv") {
    type = "text/csv;charset=utf-8";
    content = ["time,direction,level,summary,hex"]
      .concat(appState.logs.slice().reverse().map((entry) => [
        entry.timestamp.toISOString(),
        entry.direction,
        entry.level,
        JSON.stringify(entry.decoded?.summary || ""),
        JSON.stringify(bytesToHex(entry.bytes))
      ].join(",")))
      .join("\n");
  } else {
    type = "application/x-ndjson;charset=utf-8";
    content = appState.logs.slice().reverse().map((entry) => JSON.stringify({
      time: entry.timestamp.toISOString(),
      direction: entry.direction,
      level: entry.level,
      summary: entry.decoded?.summary || "",
      hex: bytesToHex(entry.bytes),
      decoded: entry.decoded
    })).join("\n");
  }
  const blob = new Blob([content], { type });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

function bootstrap() {
  cacheElements();
  if (elements.startupNotice) {
    // 能执行到这里说明 ES module 已经加载成功；隐藏静态 HTML 的故障提示。
    elements.startupNotice.hidden = true;
  }
  renderEepromPageSelectors();
  applyConfigToForm();
  resetDeviceIndicators();
  updateFrequencyControlState(null);
  renderEepromView();
  bindLayoutResizers();
  startHeartbeatWatchdog();
  bindSerial();
  bindDynamicSpeedControls();
  bindCommandButtons();
  bindEmergencyButton();
  bindParserTools();
  bindConfigTools();
  bindFaultPopup();
  bindOperationPopup();
  bindEepromErrorPopup();
  if (elements.connectButton) {
    elements.connectButton.disabled = !appState.serial.supported;
    elements.connectButton.title = appState.serial.supported ? "" : "请使用 Chrome 或 Edge 通过 localhost 打开页面";
  }
  resetRuntimeStatus(appState.serial.supported ? "未连接" : "浏览器不支持 Web Serial", false);
  appendTextLog("SYS", "外部通信上位机已就绪，请先连接串口或粘贴 HEX 帧解析", "info");
}

window.addEventListener("resize", () => {
  drawTrend();
  drawPressureTrend();
});
document.addEventListener("DOMContentLoaded", bootstrap);
