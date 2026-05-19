import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const toolRoot = path.resolve(import.meta.dirname, "..");
const html = readFileSync(path.join(toolRoot, "index.html"), "utf8");
const appJs = readFileSync(path.join(toolRoot, "src", "app.js"), "utf8");
const protocolJs = readFileSync(path.join(toolRoot, "src", "protocol.js"), "utf8");
const stylesCss = readFileSync(path.join(toolRoot, "assets", "styles.css"), "utf8");

function collect(pattern, text) {
  // 静态测试只关心声明是否遗漏，用 Set 去重后再排序，避免 HTML 顺序影响判断。
  return [...new Set([...text.matchAll(pattern)].map((match) => match[1]))].sort();
}

test("页面按钮动作必须全部有 buildCommand 分支", () => {
  const htmlActions = collect(/data-action="([^"]+)"/g, html);
  const jsActions = collect(/case "([^"]+)":/g, appJs);
  assert.deepEqual(htmlActions, jsActions);
});

test("页面必须暴露配置导入、协议解析、日志导出和 EEPROM 模板控件", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  for (const id of [
    "applyConfigButton",
    "clearChartButton",
    "clearDisplayButton",
    "clearEepromViewButton",
    "clearLogButton",
    "clearManualParseButton",
    "clearParserButton",
    "clearRealtimeButton",
    "configFile",
    "configInput",
    "controlApplyStatus",
    "deviceHandleA",
    "deviceHandleAStatus",
    "deviceHandleB",
    "deviceHandleBStatus",
    "devicePumpA",
    "devicePumpAStatus",
    "devicePumpB",
    "devicePumpBStatus",
    "dynamicSpeedStatus",
    "dynamicSpeedToggle",
    "eepromTemplateSelect",
    "eepromBatchEnd",
    "eepromBatchStart",
    "eepromPageSelect",
    "eepromPageAddress",
    "eepromPageMeta",
    "eepromPageNote",
    "eepromPageStatus",
    "eepromPageTitle",
    "eepromTimingAction",
    "eepromTimingDuration",
    "eepromTimingEnd",
    "eepromTimingStart",
    "eepromErrorCloseButton",
    "eepromErrorCode",
    "eepromErrorDetail",
    "eepromErrorMessage",
    "eepromErrorPopup",
    "eepromErrorRaw",
    "eepromErrorTime",
    "eepromDataLength",
    "eepromExplainList",
    "eepromHexGrid",
    "applyEepromTemplateButton",
    "exportCsvButton",
    "exportLogButton",
    "faultCloseButton",
    "faultCode",
    "faultDetail",
    "faultMessage",
    "faultPopup",
    "faultRaw",
    "faultTime",
    "logList",
    "leftColumnSplitter",
    "parserPanel",
    "parseRawButton",
    "logManualModeButton",
    "logPendingCount",
    "logRefreshButton",
    "logScrollModeButton",
    "rawInput",
    "batchWriteNavTemplateButton",
    "readAllEepromButton",
    "readBusinessEepromButton",
    "readEepromBatchButton",
    "readNavEepromButton",
    "rightColumnSplitter",
    "setFreqButton",
    "speedDownButton",
    "speedSlider",
    "speedUpButton",
    "startupNotice",
    "timestampToggle",
    "trendChart",
    "workspace"
  ]) {
    assert.ok(ids.includes(id), `缺少控件 id=${id}`);
  }
});

test("通信日志必须同步显示解析摘要和原始数据帧", () => {
  assert.match(appJs, /log-summary/);
  assert.match(appJs, /log-raw/);
  assert.match(appJs, /bytesToHex\(entry\.bytes\)/);
});

test("窗口数据必须支持一键清空显示", () => {
  assert.match(html, /id="clearDisplayButton"/);
  assert.match(html, /id="clearRealtimeButton"/);
  assert.match(html, /id="clearChartButton"/);
  assert.match(html, /id="clearEepromViewButton"/);
  assert.match(html, /id="clearLogButton"/);
  assert.match(html, /id="clearManualParseButton"/);
  assert.match(html, /id="clearParserButton"/);
  assert.match(html, /清空显示/);
  assert.match(html, /清空本区/);
  assert.match(stylesCss, /\.panel-clear-button/);
  assert.match(appJs, /clearDisplayData/);
  assert.match(appJs, /clearRealtimeDisplay/);
  assert.match(appJs, /clearChartDisplay/);
  assert.match(appJs, /clearEepromDisplay/);
  assert.match(appJs, /clearLogDisplay/);
  assert.match(appJs, /clearManualParseDisplay/);
  assert.match(appJs, /clearParserDisplay/);
  assert.match(appJs, /appState\.logs\s*=\s*\[\]/);
  assert.match(appJs, /appState\.trend\s*=\s*\[\]/);
  assert.match(appJs, /rawInput/);
  assert.match(appJs, /drawTrend/);
});

test("主要窗口区域必须支持鼠标拖拽调节", () => {
  assert.match(html, /data-resize-layout="left"/);
  assert.match(html, /data-resize-layout="right"/);
  assert.match(html, /resizable-panel/);
  assert.match(appJs, /bindLayoutResizers/);
  assert.match(appJs, /pointerdown/);
  assert.match(appJs, /--left-col/);
  assert.match(appJs, /--right-col/);
});

test("通信日志必须支持数据时间戳显示选项", () => {
  assert.match(html, /id="timestampToggle"[^>]*checked/);
  assert.match(html, /显示数据时间戳/);
  assert.match(appJs, /showDataTimestamp/);
  assert.match(appJs, /formatDataTimestamp/);
  assert.match(appJs, /timestampToggle/);
  assert.match(appJs, /数据时间戳/);
});

test("通信日志必须支持滚动模式和手动模式切换", () => {
  assert.match(html, /滚动模式/);
  assert.match(html, /手动模式/);
  assert.match(html, /显示新日志/);
  assert.match(appJs, /logMode/);
  assert.match(appJs, /pendingLogCount/);
  assert.match(appJs, /setLogMode/);
  assert.match(appJs, /renderLogModeState/);
  assert.match(appJs, /preserveManualView/);
  assert.match(appJs, /previousScrollTop/);
  assert.match(appJs, /scrollTop\s*=\s*previousScrollTop\s*\+\s*addedHeight/);
  assert.match(appJs, /logList\.scrollTop\s*=\s*0/);
  assert.match(appJs, /logManualModeButton/);
  assert.match(appJs, /logScrollModeButton/);
});

test("数据可视化区域必须在曲线旁显示当前数值", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  for (const id of [
    "chartValueSpeed",
    "chartValueCurrent",
    "chartValuePumpA",
    "chartValuePumpB"
  ]) {
    assert.ok(ids.includes(id), `缺少图表数值控件 id=${id}`);
  }

  assert.match(html, /chart-value-strip/);
  assert.match(html, /chart-value-card/);
  assert.match(html, /aria-label="曲线当前数值"/);
  assert.match(stylesCss, /\.chart-value-strip/);
  assert.match(stylesCss, /\.chart-value-card/);
  assert.match(stylesCss, /\.chart-value-blue/);
  assert.match(stylesCss, /\.chart-value-green/);
  assert.match(stylesCss, /\.chart-value-amber/);
  assert.match(stylesCss, /\.chart-value-red/);
  assert.match(appJs, /elements\.chartValueSpeed/);
  assert.match(appJs, /elements\.chartValueCurrent/);
  assert.match(appJs, /elements\.chartValuePumpA/);
  assert.match(appJs, /elements\.chartValuePumpB/);
  assert.match(appJs, /updateChartValueStrip/);
  assert.match(appJs, /drawChartScale/);
});

test("数据可视化区域必须显示压力传感器原始值和最终重量", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  for (const id of [
    "pressureRawA",
    "pressureWeightA",
    "pressureRawB",
    "pressureWeightB",
    "pressureSeqA",
    "pressureSeqB",
    "pressureThresholdA",
    "pressureThresholdB"
  ]) {
    assert.ok(ids.includes(id), `缺少压力传感器显示 id=${id}`);
  }
  assert.match(html, /压力传感器/);
  assert.match(html, /CS1237 原始值/);
  assert.match(html, /最终重量/);
  assert.match(stylesCss, /\.pressure-sensor-window/);
  assert.match(stylesCss, /\.pressure-sensor-grid/);
  assert.match(stylesCss, /\.pressure-value-card/);
  assert.match(appJs, /elements\.pressureRawA/);
  assert.match(appJs, /updatePressureSensorPanel/);
  assert.match(appJs, /formatWeightX10/);
  assert.match(appJs, /telemetry\.pumpA\?\.pressureRaw/);
  assert.match(appJs, /resetPressureSensorPanel/);
});

test("压力传感器重量超过阈值时必须弹窗报警", () => {
  assert.match(appJs, /applyPressureThresholdAlarm/);
  assert.match(appJs, /buildPressureThresholdAlarm/);
  assert.match(appJs, /showPressureAlarmPopup/);
  assert.match(appJs, /clearPressureAlarmIfSafe/);
  assert.match(appJs, /压力超过阈值/);
  assert.match(appJs, /WeightX10/);
  assert.match(appJs, /ThresholdG/);
  assert.match(appJs, /pressureRaw/);
  assert.match(appJs, /pressureSeq/);
  assert.match(appJs, /appState\.activeFault\?\.source === "pressure"/);
  assert.match(appJs, /applyPressureThresholdAlarm\(telemetry, decoded\.rawBytes\)/);
});

test("压力传感器数据必须有独立实时曲线窗口", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  for (const id of [
    "pressureRawTrendChart",
    "pressureWeightTrendChart"
  ]) {
    assert.ok(ids.includes(id), `缺少压力趋势画布 id=${id}`);
  }
  assert.match(html, /pressure-trend-window/);
  assert.match(html, /CS1237 原始值曲线/);
  assert.match(html, /最终重量曲线/);
  assert.match(stylesCss, /\.pressure-trend-window/);
  assert.match(stylesCss, /\.pressure-trend-grid/);
  assert.match(stylesCss, /\.pressure-trend-canvas/);
  assert.match(appJs, /pressureTrend:\s*\[\]/);
  assert.match(appJs, /elements\.pressureRawTrendCanvas/);
  assert.match(appJs, /elements\.pressureWeightTrendCanvas/);
  assert.match(appJs, /pushPressureTrendPoint\(telemetry\)/);
  assert.match(appJs, /drawPressureTrend/);
  assert.match(appJs, /drawPressureTrendCanvas/);
  assert.match(appJs, /appState\.pressureTrend\s*=\s*\[\]/);
  assert.match(appJs, /drawPressureTrend\(\)/);
});

test("动态调速勾选时启动手柄前必须先下发当前速度", () => {
  assert.match(appJs, /HANDLE_START_PREFLIGHT_DELAY_MS/);
  assert.match(appJs, /async function transmitCurrentHandleSpeed/);
  assert.match(appJs, /action === "handle-start" && elements\.dynamicSpeedToggle\?\.checked/);
  assert.match(appJs, /clearDynamicSpeedTimer\(\);[\s\S]*transmitCurrentHandleSpeed\(\)/);
  assert.match(appJs, /delay\(HANDLE_START_PREFLIGHT_DELAY_MS\)[\s\S]*buildCommand\(action\)/);
});

test("实时状态必须用手柄和泵图标显示接入状态", () => {
  assert.match(html, /aria-label="设备接入状态"/);
  assert.match(html, /device-logo-handle/);
  assert.match(html, /device-logo-pump/);
  assert.match(html, /data-online="false"/);
  assert.match(html, /data-active="false"/);
  assert.match(html, /当前工作/);
  assert.match(html, /A 手柄/);
  assert.match(html, /B 手柄/);
  assert.match(html, /A 泵/);
  assert.match(html, /B 泵/);
  assert.match(stylesCss, /\.device-status-grid/);
  assert.match(stylesCss, /\.device-card\[data-online="true"\]/);
  assert.match(stylesCss, /\.device-card\[data-active="true"\]/);
  assert.match(stylesCss, /\.device-active-tag/);
  assert.match(stylesCss, /url\("device-handle\.png"\)/);
  assert.match(stylesCss, /url\("device-pump\.svg"\)/);
  assert.match(stylesCss, /#ffd800/);
  assert.match(stylesCss, /#d8d8d0/);
  assert.match(stylesCss, /\.device-logo-handle::before/);
  assert.match(stylesCss, /\.device-logo-pump::before/);
  assert.equal(existsSync(path.join(toolRoot, "assets", "device-handle.png")), true);
  assert.equal(existsSync(path.join(toolRoot, "assets", "device-pump.svg")), true);
  assert.match(appJs, /elements\.deviceHandleA/);
  assert.match(appJs, /elements\.devicePumpBStatus/);
  assert.match(appJs, /updateDeviceIndicators\(telemetry\)/);
  assert.match(appJs, /buildHandleDeviceText/);
  assert.match(appJs, /handle\.typeName/);
  assert.doesNotMatch(appJs, /已接入 \$\{handle\.rawType\}/);
  assert.match(appJs, /telemetry\.selectedChannel === 0x01/);
  assert.match(appJs, /telemetry\.selectedChannel === 0x02/);
  assert.match(appJs, /当前工作/);
  assert.match(appJs, /resetDeviceIndicators/);
  assert.match(appJs, /card\.dataset\.online\s*=\s*online\s*\?/);
  assert.match(appJs, /card\.dataset\.active\s*=\s*online && active\s*\?/);
});

test("频率设置必须只允许 PXBA/PXBB 往复转手柄", () => {
  assert.match(html, /id="setFreqButton"[^>]*data-action="set-freq"/);
  assert.match(appJs, /elements\.setFreqButton/);
  assert.match(appJs, /updateFrequencyControlState/);
  assert.match(appJs, /getSelectedHandle/);
  assert.match(appJs, /isOscillatingHandle/);
  assert.match(appJs, /PXBA/);
  assert.match(appJs, /PXBB/);
  assert.match(appJs, /仅 PXBA\/PXBB 往复转手柄允许设置频率/);
  assert.match(appJs, /case "set-freq"/);
  assert.match(appJs, /canSetFrequency\(\)/);
});

test("手柄速度必须支持运行中动态调节", () => {
  assert.match(html, /id="speedSlider"[^>]*type="range"/);
  assert.match(html, /id="dynamicSpeedToggle"[^>]*type="checkbox"/);
  assert.match(html, /动态调速/);
  assert.match(html, /id="speedDownButton"/);
  assert.match(html, /id="speedUpButton"/);
  assert.match(html, /id="dynamicSpeedStatus"/);
  assert.match(stylesCss, /\.speed-control/);
  assert.match(stylesCss, /\.speed-slider/);
  assert.match(stylesCss, /\.inline-status/);
  assert.match(appJs, /DYNAMIC_SPEED_DEBOUNCE_MS/);
  assert.match(appJs, /SPEED_STEP_VALUE/);
  assert.match(appJs, /buildSetSpeedFrame/);
  assert.match(appJs, /WorkMessage\.speed_set_work/);
  assert.match(appJs, /WorkMessage\.speed_work/);
  assert.match(appJs, /queueDynamicSpeedSend/);
  assert.match(appJs, /sendDynamicSpeedNow/);
  assert.match(appJs, /bindDynamicSpeedControls/);
  assert.match(appJs, /动态调速已开启/);
  assert.match(appJs, /动态已下发/);
  assert.match(appJs, /buildSetSpeedFrame\(readSpeedInputValue\(\)\)/);
  assert.match(protocolJs, /设置当前手柄速度（运行中可动态生效）/);
});

test("实时电流必须按驱动反馈 x100 换算成安培显示", () => {
  assert.match(protocolJs, /telemetry\.currentRaw\s*=\s*readWord\(infoBytes,\s*cursor\)/);
  assert.match(protocolJs, /telemetry\.current\s*=\s*telemetry\.currentRaw\s*\/\s*100/);
  assert.match(protocolJs, /电流：\$\{formatMotorCurrent\(telemetry\.current\)\}/);
  assert.match(appJs, /function formatMotorCurrent/);
  assert.match(appJs, /formatMotorCurrent\(telemetry\.current\)/);
  assert.match(appJs, /currentRaw/);
});

test("实时状态数值必须直接带单位", () => {
  assert.match(appJs, /function formatMotorSpeed/);
  assert.match(appJs, /function formatPumpSpeed/);
  assert.match(appJs, /setText\(elements\.metricSpeed,\s*formatMotorSpeed\(telemetry\.speed\)\)/);
  assert.match(appJs, /setText\(elements\.metricPumpA,\s*formatPumpSpeed\(telemetry\.pumpA\?\.speed\)\)/);
  assert.match(appJs, /setText\(elements\.metricPumpB,\s*formatPumpSpeed\(telemetry\.pumpB\?\.speed\)\)/);
  assert.match(appJs, /speed:\s*formatMotorSpeed\(telemetry\.speed\)/);
  assert.match(appJs, /pumpA:\s*formatPumpSpeed\(telemetry\.pumpA\?\.speed\)/);
  assert.match(appJs, /pumpB:\s*formatPumpSpeed\(telemetry\.pumpB\?\.speed\)/);
  assert.match(appJs, /rpm/);
  assert.match(appJs, /ml\/min/);
  assert.match(html, /驱动反馈电流 x100/);
  assert.doesNotMatch(html, /WorkMessage\.current_work/);
});

test("系统断电或心跳超时时必须恢复连接区和实时状态", () => {
  assert.match(appJs, /HEARTBEAT_TIMEOUT_MS/);
  assert.match(appJs, /lastHeartbeatAt/);
  assert.match(appJs, /startHeartbeatWatchdog/);
  assert.match(appJs, /handleHeartbeatTimeout/);
  assert.match(appJs, /resetRuntimeStatus/);
  assert.match(appJs, /等待心跳/);
  assert.match(appJs, /心跳超时/);
  assert.match(appJs, /applyTelemetry\(frame\)/);
  assert.match(appJs, /resetRuntimeStatus\("未连接"/);
  assert.match(appJs, /resetRuntimeStatus\("心跳超时"/);
  assert.match(appJs, /setText\(elements\.metricChannel, "当前通道：未选中"\)/);
  assert.match(appJs, /setText\(elements\.metricRun, "运行状态：未知"\)/);
});

test("故障报警必须有高对比弹窗并绑定协议报警帧", () => {
  assert.match(html, /id="faultPopup"/);
  assert.match(html, /role="alertdialog"/);
  assert.match(html, /故障报警/);
  assert.match(stylesCss, /\.fault-popup/);
  assert.match(stylesCss, /\.fault-dialog/);
  assert.match(stylesCss, /--red/);
  assert.match(stylesCss, /#ffd800/);
  assert.match(appJs, /elements\.faultPopup/);
  assert.match(appJs, /applyAlarmFrame/);
  assert.match(appJs, /showFaultPopup/);
  assert.match(appJs, /hideFaultPopup/);
  assert.match(appJs, /WorkMessage\.alarm_value/);
  assert.match(appJs, /faultCloseButton/);
  assert.match(appJs, /applyAlarmFrame\(frame, frame\.rawBytes\)/);
  assert.match(appJs, /applyTelemetry\(frame\)/);
  assert.match(appJs, /appState\.activeFault\s*=\s*null/);
});

test("EEPROM 可视化区域必须显示原始 Page 数据和布局解释", () => {
  assert.match(html, /EEPROM 可视化/);
  assert.match(html, /Page 原始数据/);
  assert.match(html, /布局解释/);
  assert.match(html, /读取\/写入时间统计/);
  assert.match(html, /eeprom布局说明\.txt/);
  assert.match(stylesCss, /\.eeprom-panel/);
  assert.match(stylesCss, /\.eeprom-summary/);
  assert.match(stylesCss, /\.eeprom-timing/);
  assert.match(stylesCss, /\.eeprom-viewer/);
  assert.match(stylesCss, /\.eeprom-hex-grid/);
  assert.match(stylesCss, /\.eeprom-field-row/);
  assert.match(stylesCss, /\.eeprom-page-prefix/);
  assert.match(stylesCss, /\.eeprom-page-failed/);
  assert.match(stylesCss, /\.eeprom-empty-page/);
  assert.match(appJs, /analyzeEepromFrame/);
  assert.match(appJs, /applyEepromFrame/);
  assert.match(appJs, /renderEepromView/);
  assert.match(appJs, /beginEepromTiming/);
  assert.match(appJs, /finishEepromTiming/);
  assert.match(appJs, /renderEepromTiming/);
  assert.match(appJs, /addFailedEepromView/);
  assert.match(appJs, /addSkippedEepromViews/);
  assert.match(appJs, /未下发读取/);
  assert.match(appJs, /renderEepromHexBlock/);
  assert.match(appJs, /未写入或空页/);
  assert.match(appJs, /Page\$\{escapeHtml\(view\.page\)\}：/);
  assert.match(appJs, /applyEepromFrame\(frame\)/);
  assert.match(appJs, /appState\.eepromView\s*=\s*null/);
  assert.match(appJs, /appState\.eepromViews\s*=\s*\[\]/);
});

test("EEPROM 功能区必须支持 Page2-Page128 下拉和批量读取选择", () => {
  assert.match(html, /id="eepromPageSelect"/);
  assert.match(html, /id="eepromBatchStart"/);
  assert.match(html, /id="eepromBatchEnd"/);
  assert.match(html, /id="eepromPageStatus"/);
  assert.match(html, /data-action="read-eeprom-page"/);
  assert.match(html, /data-action="write-eeprom-page"/);
  assert.match(html, /id="readEepromBatchButton"/);
  assert.match(html, /id="readAllEepromButton"/);
  assert.match(html, /id="readBusinessEepromButton"/);
  assert.match(html, /id="readNavEepromButton"/);
  assert.match(html, /id="batchWriteNavTemplateButton"/);
  assert.match(html, /批量读取 Page/);
  assert.match(html, /批量读取全部页/);
  assert.match(html, /批量读取业务页/);
  assert.match(html, /批量读取导航页/);
  assert.match(html, /模板批量写导航页/);
  assert.doesNotMatch(html, /id="businessArea"/);
  assert.doesNotMatch(html, /id="navPage"/);
  assert.match(appJs, /EEPROM_MIN_PAGE\s*=\s*2/);
  assert.match(appJs, /EEPROM_MAX_PAGE\s*=\s*128/);
  assert.match(appJs, /EEPROM_BUSINESS_PAGES/);
  assert.match(appJs, /EEPROM_EXTENSION_PAGES/);
  assert.match(appJs, /renderEepromPageSelectors/);
  assert.match(appJs, /buildEepromPageCommand/);
  assert.match(appJs, /read-eeprom-page/);
  assert.match(appJs, /write-eeprom-page/);
  assert.match(appJs, /sendEepromBatchRead/);
  assert.match(appJs, /sendEepromReadAllPages/);
  assert.match(appJs, /sendEepromReadBusinessPages/);
  assert.match(appJs, /sendEepromReadNavPages/);
  assert.match(appJs, /sendEepromBatchWriteNavTemplate/);
  assert.match(appJs, /getSelectedEepromTemplateBytes/);
  assert.match(appJs, /EEPROM_NAV_START_PAGE/);
  assert.match(appJs, /EEPROM_NAV_END_PAGE/);
  assert.match(appJs, /Page7\/Page10 是业务扩展续页/);
  assert.match(appJs, /多档位功能调节续页/);
  assert.match(appJs, /手柄客户可编辑续页/);
});

test("EEPROM 读写失败必须弹窗提示未接入手柄等原因", () => {
  assert.match(html, /id="eepromErrorPopup"/);
  assert.match(html, /role="alertdialog"/);
  assert.match(html, /EEPROM 读写失败/);
  assert.match(html, /id="eepromErrorCloseButton"/);
  assert.match(stylesCss, /\.eeprom-error-popup/);
  assert.match(stylesCss, /\.eeprom-error-dialog/);
  assert.match(appJs, /applyEepromAckFrame/);
  assert.match(appJs, /showEepromErrorPopup/);
  assert.match(appJs, /suppressEepromBatchReadErrorPopup/);
  assert.match(appJs, /resolveEepromAckTargetPage/);
  assert.match(appJs, /targetPage/);
  assert.match(appJs, /Page\$\{targetPage\} \/ 对象/);
  assert.match(appJs, /hideEepromErrorPopup/);
  assert.match(appJs, /EEPROM_ACK_FAILED/);
  assert.match(appJs, /eepromOperationType/);
  assert.match(appJs, /batch-read/);
  assert.match(appJs, /部分页失败/);
  assert.match(appJs, /当前无 A\/B 通道/);
  assert.match(appJs, /请先接入手柄并切换到对应通道/);
  assert.match(appJs, /applyEepromAckFrame\(frame, frame\.rawBytes\)/);
});

test("操作顺序错误和控制失败 ACK 必须弹窗提示", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  for (const id of [
    "operationPopup",
    "operationTitle",
    "operationTime",
    "operationMessage",
    "operationCode",
    "operationDetail",
    "operationRaw",
    "operationCloseButton"
  ]) {
    assert.ok(ids.includes(id), `缺少操作提示弹窗 id=${id}`);
  }
  assert.match(stylesCss, /\.operation-popup/);
  assert.match(stylesCss, /\.operation-dialog/);
  assert.match(stylesCss, /\.operation-code/);
  assert.match(appJs, /externalControlGranted:\s*false/);
  assert.match(appJs, /REQUIRED_EXTERNAL_CONTROL_ACTIONS/);
  assert.match(appJs, /validateOperationOrder/);
  assert.match(appJs, /showOperationPopup/);
  assert.match(appJs, /showOperationLocalError/);
  assert.match(appJs, /applyOperationAckFrame/);
  assert.match(appJs, /decoded\.infoCode === 0xAA/);
  assert.match(appJs, /decoded\.infoCode === 0xAB/);
  assert.match(html, /data-action="host-exit"/);
  assert.match(appJs, /case "host-exit"[\s\S]*buildFrame\(0x02,\s*0xBB,\s*0xFF,\s*0xFF,\s*\[\]\)/);
  assert.match(appJs, /ackInfoBytes\[0\] === 0xBB[\s\S]*externalControlGranted = false/);
  assert.match(appJs, /reason === 0x06/);
  assert.match(appJs, /请先点击“申请外部控制”/);
  assert.match(appJs, /validateOperationOrder\(action\)[\s\S]*action === "handle-start" && elements\.dynamicSpeedToggle\?\.checked/);
  assert.match(appJs, /applyOperationAckFrame\(frame, frame\.rawBytes\)/);
  assert.match(appJs, /applyOperationAckFrame\(frame, bytes\)/);
});

test("申请外部控制按钮下方必须显示申请结果", () => {
  const ids = collect(/id="([^"]+)"/g, html);
  assert.ok(ids.includes("controlApplyStatus"), "缺少申请外部控制结果状态条");
  assert.match(stylesCss, /\.control-apply-status/);
  assert.match(appJs, /setControlApplyStatus/);
  assert.match(appJs, /decoded\.infoCode === 0xAA[\s\S]*setControlApplyStatus/);
  assert.match(appJs, /decoded\.infoCode === 0xAB[\s\S]*setControlApplyStatus/);
  assert.match(appJs, /action === "apply-control"[\s\S]*setControlApplyStatus/);
});

test("顶部品牌必须显示贵州梓锐科技和 logo 主题色", () => {
  assert.match(html, /<title>贵州梓锐科技 外部通信调试上位机<\/title>/);
  assert.match(html, /<h1>贵州梓锐科技<\/h1>/);
  assert.doesNotMatch(html, /<p class="eyebrow">外部通信调试上位机<\/p>/);
  assert.match(html, /brand-mark/);
  assert.match(html, /assets\/guizhou-zirui-logo\.jpg/);
  assert.equal(existsSync(path.join(toolRoot, "assets", "guizhou-zirui-logo.jpg")), true);
  assert.match(stylesCss, /--page:\s*#fff8cf/);
  assert.match(stylesCss, /--panel:\s*#fffdf4/);
  assert.match(stylesCss, /--cyan:\s*#d7a900/);
  assert.match(stylesCss, /--green:\s*#00a66a/);
  assert.match(stylesCss, /\.brand-mark/);
  assert.match(stylesCss, /font-size:\s*42px/);
  assert.match(stylesCss, /font-weight:\s*900/);
  assert.match(stylesCss, /#ffd800/);
  assert.match(stylesCss, /#fffef8/);
  assert.match(appJs, /"#ffcc00"/);
  assert.match(appJs, /"#00a66a"/);
  assert.match(appJs, /lineWidth\s*=\s*2\.5/);
});

test("页面必须在脚本未运行时提示通过 localhost 打开", () => {
  assert.match(html, /id="startupNotice"/);
  assert.match(html, /localhost:4173/);
  assert.match(appJs, /startupNotice/);
  assert.match(appJs, /hidden\s*=\s*true/);
});

test("不支持 Web Serial 时连接按钮必须被禁用", () => {
  assert.match(appJs, /connectButton\.disabled\s*=/);
});

test("工具目录必须提供本地启动说明", () => {
  const readmePath = path.join(toolRoot, "README.md");
  assert.equal(existsSync(readmePath), true);
  const readme = readFileSync(readmePath, "utf8");
  assert.equal(existsSync(path.join(toolRoot, "启动上位机.vbs")), true);
  assert.equal(existsSync(path.join(toolRoot, "scripts", "launch.mjs")), true);
  assert.match(readme, /双击/);
  assert.match(readme, /启动上位机\.vbs/);
  assert.match(readme, /npm run serve/);
  assert.match(readme, /localhost:4173/);
  assert.match(readme, /115200/);
});
