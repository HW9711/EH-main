# 手柄实体键旧扫描链清理 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除 `handlekey.c` 中链接器已经裁掉的旧实体键扫描链，保留当前 30ms 实体键业务路径，并把任务入口和 82 号提示注释改为当前真实行为。

**Architecture:** 本阶段不新增生产模块，不改变任务、队列、控制权或硬件读取方式。只删除不可达旧函数和由它们独占的头文件接口；当前真实链 `HANDLEKEYTaskFunc()` → `HandleKey_ScanRunKeys()` → `HandleRunKey_Process()`保持不变。

**Tech Stack:** STM32F413、C99、FreeRTOS、ARM Compiler 5.06u6、EIDE unify_builder、Keil µVision 5、Python 3 临时静态校验脚本。

## Global Constraints

- 业务源码行为基线固定为 `a39c3b1`，当前 `f16850e`只在该基线上增加设计文档。
- 每个软任务独立静态线程栈、静态 TCB 和 `AppTaskRuntimeGate()`全局互斥锁保持不变。
- 手柄实体键任务周期固定为 30ms。
- 不修改 `Userparser_Init()`任务注册顺序。
- 不修改 GPIO、A/B 通道、手柄型号、报警码、UI 地址、控制权和联动泵行为。
- 不删除 `sHandleKEYValue`或 `HAL_GPIO_EXTI_Callback()`；它们不属于本阶段 map 已裁剪范围。
- 不改 `HandleRunKey_*`当前业务分支、状态写入和调用顺序。
- 临时 `verify_handlekey_cleanup.py`执行完成后必须删除，不得进入 Git。
- 本机构建通过后先交付 HEX，由用户烧录确认；用户明确确认无问题后才允许提交 Git。
- 提交信息使用中文业务标题和 `•` 项目符号正文。

---

## File Structure

### 生产文件

- Modify: `User/Application/Handle/handlekey.c`
  - 删除旧 `HandleKey_*`扫描链。
  - 保留并说明当前 `HandleRunKey_*`状态机。
  - 明确任务入口顺序和 30ms 周期。
- Modify: `User/Application/include/handlekey.h`
  - 删除已无实现的 `HandleKey_GetKeyValue()`声明。
  - 删除仅供旧扫描链使用的 `KEY0_STATUS/KEY1_STATUS`兼容宏。
  - 删除随旧查询接口一起失去用途的 `stdint.h/stdbool.h`引用。
  - 保留 A/B 当前运行键读取宏和任务初始化接口。

### 临时校验文件

- Create temporarily: `verify_handlekey_cleanup.py`
- Delete before handoff: `verify_handlekey_cleanup.py`

### 计划文件

- Keep: `docs/superpowers/plans/2026-07-10-handlekey-legacy-cleanup.md`

不新增任何生产 `.c/.h`，因此不修改 EIDE 或 Keil 源文件清单。

## Baseline Evidence

当前 EIDE map 已确认以下旧函数不进入固件镜像：

- `HandleKey_GetKeyValue()`：6B，Removing Unused。
- `HandleKey_Scan0SSC()`：300B，Removing Unused。
- `HandleKey_Scan1SSC()`：308B，Removing Unused。
- `HandleKey_SetAlarm()`：12B，Removing Unused。
- `HandleKey_ClearAlarm()`：12B，Removing Unused。
- `HandleKey_SetMotorRun()`：68B，Removing Unused。

当前必须继续存在的运行符号：

- `HANDLEKEYTaskFunc`
- `HandleKey_ScanRunKeys`
- `HandleKeyScan_Init`
- `HANDLEKEYTaskHandle`，map 中静态任务对象为 4240B。

当前全量构建基线：

- 0 error。
- 22 个已有 warning。
- RO 106692B。
- RW 130904B。
- ROM 107196B。

---

### Task 1: 建立删除范围的临时红灯校验

**Files:**
- Create temporarily: `verify_handlekey_cleanup.py`
- Read: `User/Application/Handle/handlekey.c`
- Read: `User/Application/include/handlekey.h`

**Interfaces:**
- Consumes: 当前源码中的旧符号、真实任务入口和关键调用顺序。
- Produces: 一个在旧代码存在时失败、清理完成后通过的临时静态检查。

- [ ] **Step 1: 创建临时校验脚本**

使用 `apply_patch`创建根目录 `verify_handlekey_cleanup.py`，内容必须为：

```python
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOURCE_PATH = ROOT / "User" / "Application" / "Handle" / "handlekey.c"
HEADER_PATH = ROOT / "User" / "Application" / "include" / "handlekey.h"
BASELINE_REVISION = "a39c3b1"

source_bytes = SOURCE_PATH.read_bytes()
header_bytes = HEADER_PATH.read_bytes()
source = source_bytes.decode("utf-8")
header = header_bytes.decode("utf-8")
errors: list[str] = []


def require(condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)


def require_order(text: str, tokens: list[str], label: str) -> None:
    positions: list[int] = []
    for token in tokens:
        pos = text.find(token)
        if pos < 0:
            errors.append(f"{label}: 缺少 {token}")
            return
        positions.append(pos)
    if positions != sorted(positions):
        errors.append(f"{label}: 调用顺序错误: {' -> '.join(tokens)}")


def has_mixed_line_endings(data: bytes) -> bool:
    return b"\r\n" in data and b"\n" in data.replace(b"\r\n", b"")


def extract_function(text: str, signature: str) -> str:
    start = text.index(signature)
    opening_brace = text.index("{", start)
    depth = 0
    for index in range(opening_brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise ValueError(f"函数缺少结束大括号: {signature}")


def normalize_c_code(text: str) -> str:
    without_block_comments = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    without_comments = re.sub(r"//[^\r\n]*", "", without_block_comments)
    return re.sub(r"\s+", "", without_comments)


def normalize_task_entry(text: str) -> str:
    code = normalize_c_code(text).replace("(void)event;", "")
    busy_with_braces = (
        "if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE)){return;}"
    )
    busy_without_braces = (
        "if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))return;"
    )
    return code.replace(busy_with_braces, busy_without_braces)


baseline_result = subprocess.run(
    ["git", "show", f"{BASELINE_REVISION}:User/Application/Handle/handlekey.c"],
    cwd=ROOT,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    check=False,
)
require(baseline_result.returncode == 0, f"无法读取稳定基线 {BASELINE_REVISION} 的 handlekey.c")
baseline_source = baseline_result.stdout.decode("utf-8") if baseline_result.returncode == 0 else source

require(not has_mixed_line_endings(source_bytes), "handlekey.c 同时包含 CRLF 和 LF 混合行尾")
require(not has_mixed_line_endings(header_bytes), "handlekey.h 同时包含 CRLF 和 LF 混合行尾")


forbidden_source = [
    "HandleKey_GetKeyValue",
    "HandleKey_SetAlarm",
    "HandleKey_ClearAlarm",
    "HandleKey_SetMotorRun",
    "HandleKey_Scan0SSC",
    "HandleKey_Scan1SSC",
    "KEY0_STATUS",
    "KEY1_STATUS",
]

for token in forbidden_source:
    require(token not in source, f"handlekey.c 仍包含旧符号: {token}")

for token in ["HandleKey_GetKeyValue", "KEY0_STATUS", "KEY1_STATUS"]:
    require(token not in header, f"handlekey.h 仍暴露旧接口: {token}")

for token in [
    "static bool sHandleKEYValue[2]",
    "void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)",
    "static bool HandleRunKey_SetMotorRun(bool enable)",
    "static void HandleRunKey_Process(uint8_t channel, bool press_event, bool release_event)",
    "static void HandleKey_ScanRunKeys(void)",
    "void HANDLEKEYTaskFunc(uint32_t event)",
    "void HandleKeyScan_Init(void)",
]:
    require(token in source, f"误删当前运行入口或状态: {token}")

for token in [
    "void HandleKeyScan_Init(void);",
    "HANDLE_RUN_KEY_A_STATUS()",
    "HANDLE_RUN_KEY_B_STATUS()",
]:
    require(token in header, f"handlekey.h 缺少当前接口: {token}")

require(
    re.search(
        r"#define\s+HANDLE_RUN_KEY_A_STATUS\(\)\s+"
        r"Bsp_GpioRead\(\s*BOARD_RES_HANDLE_RUN_KEY_A_PORT\s*,\s*"
        r"BOARD_RES_HANDLE_RUN_KEY_A_PIN\s*\)",
        header,
    )
    is not None,
    "A 通道实体键宏未映射到 A 通道 GPIO",
)
require(
    re.search(
        r"#define\s+HANDLE_RUN_KEY_B_STATUS\(\)\s+"
        r"Bsp_GpioRead\(\s*BOARD_RES_HANDLE_RUN_KEY_B_PORT\s*,\s*"
        r"BOARD_RES_HANDLE_RUN_KEY_B_PIN\s*\)",
        header,
    )
    is not None,
    "B 通道实体键宏未映射到 B 通道 GPIO",
)

require("3 秒" not in source and "3秒" not in source, "82号提示仍写成错误的3秒")
require("当前 2000ms" in source, "没有把82号提示说明为 ALARM_MODE_MS 当前2000ms")
require("(void)event;" in source, "HANDLEKEYTaskFunc 未明确丢弃 event 参数")
require(
    re.search(
        r"Kernel_TaskStart\s*\(\s*&HANDLEKEYTaskHandle\s*,\s*KERNEL_TASK_ALWAYS\s*,\s*30U?\s*\)\s*;",
        source,
    )
    is not None,
    "手柄实体键任务周期不是30ms",
)

exti_start = source.index("void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)")
exti_end = source.index("#define HANDLE_KEY_DEBOUNCE_COUNT")
exti_section = source[exti_start:exti_end]
require_order(
    exti_section,
    [
        "SimUart_HandleExti(GPIO_Pin);",
        "switch (GPIO_Pin)",
    ],
    "GPIO 中断软串口转发顺序",
)
require(
    normalize_c_code(extract_function(source, "void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)"))
    == normalize_c_code(extract_function(baseline_source, "void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)")),
    "HAL_GPIO_EXTI_Callback 代码相对稳定基线发生变化",
)

core_start_token = "#define HANDLE_KEY_DEBOUNCE_COUNT"
core_end_token = "void HANDLEKEYTaskFunc(uint32_t event)"
current_core = source[source.index(core_start_token) : source.index(core_end_token)]
baseline_core = baseline_source[
    baseline_source.index(core_start_token) : baseline_source.index(core_end_token)
]
require(
    normalize_c_code(current_core) == normalize_c_code(baseline_core),
    "HandleRunKey 当前业务代码相对稳定基线发生变化",
)
require(
    normalize_task_entry(extract_function(source, "void HANDLEKEYTaskFunc(uint32_t event)"))
    == normalize_task_entry(extract_function(baseline_source, "void HANDLEKEYTaskFunc(uint32_t event)")),
    "HANDLEKEYTaskFunc 控制权门禁或调用结构相对稳定基线发生变化",
)
require(
    re.sub(
        r"(?<=\d)U\b",
        "",
        normalize_c_code(extract_function(source, "void HandleKeyScan_Init(void)")),
    )
    == re.sub(
        r"(?<=\d)U\b",
        "",
        normalize_c_code(extract_function(baseline_source, "void HandleKeyScan_Init(void)")),
    ),
    "HandleKeyScan_Init 任务创建或启动结构相对稳定基线发生变化",
)

run_start = source.index("static bool HandleRunKey_SetMotorRun(bool enable)")
run_end = source.index("static bool HandleRunKey_PrepareRunChannel(uint8_t channel)")
run_section = source[run_start:run_end]
require_order(
    run_section,
    [
        "Pubinterface_CheckCommonSocketToolReadyForRun()",
        "ControlArbitration_TryEnter(CONTROL_OWNER_HANDLE)",
        "Pubinterface_ClearPressureBlockStopLatchForNewTrigger()",
        "ControlSignalMessage.handle_control_flag = enable",
        "WorkMessage.runflag_work = enable",
        "Pubinterface_SetHandleInjectionPumpRun(enable)",
    ],
    "实体键启停安全顺序",
)

scan_start = source.index("static void HandleKey_ScanRunKeys(void)")
scan_end = source.index("void HANDLEKEYTaskFunc(uint32_t event)")
scan_section = source[scan_start:scan_end]
require_order(
    scan_section,
    [
        "HandleRunKey_Process(CHANNEL_A",
        "HandleRunKey_Process(CHANNEL_B",
    ],
    "A/B实体键处理顺序",
)

task_start = source.index("void HANDLEKEYTaskFunc(uint32_t event)")
task_end = source.index("void HandleKeyScan_Init(void)")
task_section = source[task_start:task_end]
require_order(
    task_section,
    [
        "HandleRunKey_ServiceFootSelectedTimedAlarm()",
        "ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE)",
        "HandleKey_ScanRunKeys()",
    ],
    "30ms任务内部顺序",
)

if errors:
    print("FAIL: handlekey 清理校验未通过")
    for error in errors:
        print(f"- {error}")
    sys.exit(1)

print("PASS: handlekey 旧扫描链已删除，当前任务入口和关键时序保持不变")
```

- [ ] **Step 2: 在修改前运行脚本，确认红灯有效**

Run:

```powershell
py -3 .\verify_handlekey_cleanup.py
```

Expected: exit code 1，输出至少包含：

```text
FAIL: handlekey 清理校验未通过
- handlekey.c 仍包含旧符号: HandleKey_GetKeyValue
- handlekey.c 仍包含旧符号: HandleKey_Scan0SSC
- handlekey.c 仍包含旧符号: HandleKey_Scan1SSC
- handlekey.h 仍暴露旧接口: HandleKey_GetKeyValue
```

如果修改前脚本直接通过，停止实施并重新核对当前源码，不得继续删除。

---

### Task 2: 删除旧扫描链并修正任务注释

**Files:**
- Modify: `User/Application/Handle/handlekey.c:1-253`
- Modify: `User/Application/Handle/handlekey.c:269-317`
- Modify: `User/Application/Handle/handlekey.c:591-627`
- Modify: `User/Application/include/handlekey.h:1-22`
- Test: `verify_handlekey_cleanup.py`

**Interfaces:**
- Consumes: 当前 `HandleRunKey_*`实现、`SimUart_HandleExti()`、`Kernel_TaskCreate/Start()`。
- Produces: 仅保留当前实体键状态机的 `handlekey.c/.h`，公共入口仍为 `HandleKeyScan_Init()`。

- [ ] **Step 1: 在 `handlekey.c`增加当前运行链文件说明**

在 include 区之后、`HANDLEKEYTaskHandle`之前加入：

```c
/*
 * 文件功能：每 30ms 轮询 A/B 手柄实体运行键，并按手柄型号执行翻转启停或按住运行。
 * 运行入口：Userparser_Init() 调用 HandleKeyScan_Init()，任务回调只进入 HANDLEKEYTaskFunc()。
 * 关键顺序：先维护 82 号限时提示，再检查控制权，最后严格按 A 后 B 顺序处理实体键。
 * 安全约束：启动必须依次通过公共接头刀具门禁、控制权申请，再写运行状态并联动注水泵。
 */
```

不要删除或改写：

```c
kernel_task_t HANDLEKEYTaskHandle;
static bool sHandleKEYValue[2] = { false };
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
```

- [ ] **Step 2: 精确删除 map 已裁剪的连续旧代码块**

在 `handlekey.c`中删除从以下旧注释开头：

```c
// 函数名称: HandleKey_GetKeyValue()
```

一直到 `HandleKey_Scan1SSC()`结束大括号，删除后下一行必须直接是：

```c
#define HANDLE_KEY_DEBOUNCE_COUNT 2U /* 30ms任务连续2次确认电平，约60ms去抖，避免触点抖动误启停。 */
```

该连续区域删除的函数必须恰好为：

```text
HandleKey_GetKeyValue
HandleKey_SetAlarm
HandleKey_ClearAlarm
HandleKey_SetMotorRun
HandleKey_Scan0SSC
HandleKey_Scan1SSC
```

不得删除任何 `HandleRunKey_*`函数。

- [ ] **Step 3: 把 82 号提示注释改成当前真实的 2000ms**

只改注释，不改 `ALARM_MODE_MS`宏和任何判断表达式。统一使用以下表述：

```c
static uint32_t s_handle_foot_selected_timed_alarm_tick = 0U; /* 记录 82 号错模式弹窗开始时间，按 ALARM_MODE_MS（当前 2000ms）自动清除。 */

/*
 * 函数功能：脚控已选中时按手柄实体运行键，上报 82 号限时提示，保持时间由 ALARM_MODE_MS 控制（当前 2000ms）。
 * 输入参数：无，报警码固定使用 WORK_ALARM_FOOT_SELECTED。
 * 返回参数：无。
 */
```

`HandleRunKey_ServiceFootSelectedTimedAlarm()`函数头和内部提前返回注释同样写为 `ALARM_MODE_MS（当前 2000ms）`，不得再出现“3秒”或“3 秒”。

- [ ] **Step 4: 整理当前任务入口，保持原调用顺序**

把文件末尾任务入口整理为以下完整实现：

```c
/*
 * 函数功能：手柄实体按键周期扫描任务，按手柄型号处理 PE12/PE13 实体键启停。
 * 输入参数：event 为调度器事件参数，当前任务不使用。
 * 返回参数：无。
 */
void HANDLEKEYTaskFunc(uint32_t event)
{
	(void)event; /* 当前任务只按固定 30ms 周期运行，不使用调度事件值。 */
	HandleRunKey_ServiceFootSelectedTimedAlarm(); /* 维护 82 号限时弹窗，到 ALARM_MODE_MS 后关闭。 */
	if (ControlArbitration_IsBusyByOther(CONTROL_OWNER_HANDLE))
	{
		return; /* 其它来源正在控制时不扫描启动键，但仍先完成本模块提示生命周期维护。 */
	}
	HandleKey_ScanRunKeys(); /* 严格先处理 A、再处理 B；A 已取得 owner 时 B 本周期不能抢占。 */
}

/*
 * 函数功能：创建并启动手柄实体按键 30ms 周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void)
{
	Kernel_TaskCreate(&HANDLEKEYTaskHandle, HANDLEKEYTaskFunc); /* 使用现有独立静态任务对象创建实体键任务。 */
	Kernel_TaskStart(&HANDLEKEYTaskHandle, KERNEL_TASK_ALWAYS, 30U); /* 周期固定为 30ms，不改变当前实体键去抖时基。 */
}
```

- [ ] **Step 5: 将 `handlekey.h`替换为以下完整内容**

```c
//handlekey.h

#ifndef __HANDLEKEY_H
#define __HANDLEKEY_H

#include "bsp_gpio.h"

/*
 * 函数功能：创建并启动手柄实体按键 30ms 周期任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void HandleKeyScan_Init(void);

#define HANDLE_RUN_KEY_A_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_A_PORT, BOARD_RES_HANDLE_RUN_KEY_A_PIN) /* 读取 A 通道实体运行键，低电平表示按下。 */
#define HANDLE_RUN_KEY_B_STATUS()  Bsp_GpioRead(BOARD_RES_HANDLE_RUN_KEY_B_PORT, BOARD_RES_HANDLE_RUN_KEY_B_PIN) /* 读取 B 通道实体运行键，低电平表示按下。 */

#endif  //__HANDLEKEY_H
```

不调整 include 目录，不新增头文件。

---

### Task 3: 完成本机静态、EIDE、Keil 和 map 验证

**Files:**
- Test: `verify_handlekey_cleanup.py`
- Build input: `EIDE/build/MainCtrlF413MXOs/builder.params`
- Build input: `MDK-ARM/MainCtrlF413MXOs.uvprojx`
- Inspect: `EIDE/build/MainCtrlF413MXOs/MainCtrlF413MXOs.map`

**Interfaces:**
- Consumes: Task 2 修改后的 `handlekey.c/.h`。
- Produces: 可供实机烧录的 HEX，以及源码、map 和任务周期不变量证据。

- [ ] **Step 1: 运行清理校验，确认由红转绿**

Run:

```powershell
py -3 .\verify_handlekey_cleanup.py
```

Expected:

```text
PASS: handlekey 旧扫描链已删除，当前任务入口和关键时序保持不变
```

- [ ] **Step 2: 检查旧符号和异常编码**

Run:

```powershell
$old = rg -n "HandleKey_GetKeyValue|HandleKey_SetAlarm|HandleKey_ClearAlarm|HandleKey_SetMotorRun|HandleKey_Scan0SSC|HandleKey_Scan1SSC|KEY0_STATUS|KEY1_STATUS" User/Application/Handle/handlekey.c User/Application/include/handlekey.h
if ($LASTEXITCODE -ne 1) { $old; throw "旧手柄键符号仍存在" }

$bad = rg -n "�|\?\?\?" User/Application/Handle/handlekey.c User/Application/include/handlekey.h
if ($LASTEXITCODE -ne 1) { $bad; throw "手柄键文件存在异常编码字符" }
```

Expected: 两次 `rg`均无匹配，命令不抛出异常。

- [ ] **Step 3: 运行 AC5/EIDE 全量构建**

Run:

```powershell
& 'C:\Users\Dell\.vscode\extensions\cl.eide-3.27.2\res\tools\win32\unify_builder\unify_builder.exe' `
  -p .\EIDE\build\MainCtrlF413MXOs\builder.params `
  --rebuild `
  --no-color
```

Expected:

- exit code 0。
- `build successfully`。
- 0 error。
- warning 不超过基线 22。
- 生成 `EIDE/build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex`。

- [ ] **Step 4: 检查 EIDE map**

Run:

```powershell
$map = '.\EIDE\build\MainCtrlF413MXOs\MainCtrlF413MXOs.map'
$requiredSymbols = @(
  'HANDLEKEYTaskFunc',
  'HandleKey_ScanRunKeys',
  'HandleKeyScan_Init',
  'HANDLEKEYTaskHandle'
)
foreach ($symbol in $requiredSymbols) {
  $required = rg -n ("^\s+" + [regex]::Escape($symbol) + "\s+0x") $map
  if ($LASTEXITCODE -ne 0) { throw "当前手柄键运行符号缺失: $symbol" }
  $required
}

$removed = rg -n "HandleKey_GetKeyValue|HandleKey_SetAlarm|HandleKey_ClearAlarm|HandleKey_SetMotorRun|HandleKey_Scan0SSC|HandleKey_Scan1SSC" $map
if ($LASTEXITCODE -ne 1) { $removed; throw "旧手柄键符号仍进入 map" }

if (-not (Select-String -LiteralPath $map -Pattern '^\s+HANDLEKEYTaskHandle\s+0x[0-9a-fA-F]+\s+Data\s+4240\s+' -Quiet)) {
  throw 'HANDLEKEYTaskHandle 不再是 4240B'
}

$mapText = Get-Content -LiteralPath $map -Encoding Default -Raw
$sizeLimits = [ordered]@{ RO = 106692; RW = 130904; ROM = 107196 }
foreach ($label in $sizeLimits.Keys) {
  $sizeMatch = [regex]::Match($mapText, "Total $label\s+Size.*?\s+(?<bytes>\d+)\s+\(")
  if ($sizeMatch.Success -eq $false) { throw "map 缺少 Total $label Size" }
  $sizeBytes = [int]$sizeMatch.Groups['bytes'].Value
  if ($sizeBytes -gt $sizeLimits[$label]) {
    throw "$label 占用超过基线: $sizeBytes > $($sizeLimits[$label])"
  }
  Write-Output "$label=$sizeBytes"
}
```

Expected:

- 四个当前运行符号存在。
- 六个旧符号完全消失。
- `HANDLEKEYTaskHandle`仍为 4240B。
- RO 不高于 106692B。
- RW 不高于 130904B。
- ROM 不高于 107196B。

- [ ] **Step 5: 运行 Keil 工程全量构建**

Run:

```powershell
$keilGeneratedBefore = @(git status --short --untracked-files=all -- `
  MDK-ARM/MainCtrlF413MXOs `
  MDK-ARM/startup_stm32f413xx.lst)
if ($keilGeneratedBefore.Count -ne 0) {
  $keilGeneratedBefore
  throw 'Keil 输出目录在构建前已有改动，不能自动覆盖或清理'
}

$keilLog = Join-Path $env:TEMP 'MainCtrlF413MXOs-handlekey-keil.log'
Remove-Item -LiteralPath $keilLog -Force -ErrorAction SilentlyContinue
$keilProcess = Start-Process `
  -FilePath 'C:\Keil_v5\UV4\UV4.exe' `
  -ArgumentList @('-b', '.\MDK-ARM\MainCtrlF413MXOs.uvprojx', '-t', 'MainCtrlF413MXOs', '-o', $keilLog) `
  -WorkingDirectory (Resolve-Path -LiteralPath '.').Path `
  -WindowStyle Hidden `
  -PassThru
$keilProcess.WaitForExit()
Get-Content -LiteralPath $keilLog -Encoding Default
Write-Output "UV4_PROCESS_EXIT=$($keilProcess.ExitCode)"
$keilSummary = [regex]::Match(
  (Get-Content -LiteralPath $keilLog -Encoding Default -Raw),
  '(?<errors>\d+) Error\(s\), (?<warnings>\d+) Warning\(s\)'
)
if (($keilSummary.Success -eq $false) -or ([int]$keilSummary.Groups['errors'].Value -ne 0)) {
  throw 'Keil log does not report 0 Error(s)'
}
if ([int]$keilSummary.Groups['warnings'].Value -gt 22) {
  throw "Keil warning count increased: $($keilSummary.Groups['warnings'].Value)"
}
if (-not (Select-String -LiteralPath $keilLog -Pattern 'Build Time Elapsed:' -Quiet)) {
  throw 'Keil log is incomplete'
}
Write-Output 'KEIL_BUILD_OK'
Remove-Item -LiteralPath $keilLog -Force
```

Expected: 输出 `KEIL_BUILD_OK`，最终日志为 0 error、warning 不超过 22，且无残留临时日志。本机 µVision 5.24 命令行成功构建时进程返回值为 1，因此只记录该值，不把它当成编译错误数。

- [ ] **Step 6: 删除临时校验脚本并清理构建临时文件**

使用 `apply_patch`删除：

```text
verify_handlekey_cleanup.py
```

然后执行：

```powershell
$root = (Resolve-Path -LiteralPath '.').Path
$eideBuildRoot = (Resolve-Path -LiteralPath '.\EIDE\build\MainCtrlF413MXOs').Path
$temporary = Get-ChildItem -LiteralPath $eideBuildRoot -File -Recurse |
  Where-Object { ($_.Name -eq '.lock') -or (($_.Extension -eq '.d') -and ($_.Length -eq 0)) }
foreach ($file in $temporary) {
    if (-not $file.FullName.StartsWith($eideBuildRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "拒绝清理工作区外文件: $($file.FullName)"
    }
    Remove-Item -LiteralPath $file.FullName -Force
}

# Keil 全量构建会改写工程中受 Git 跟踪的输出文件；构建前已确认该目录干净，因此这里只还原本次生成物。
git restore --worktree -- MDK-ARM/MainCtrlF413MXOs
if ($LASTEXITCODE -ne 0) { throw 'Keil 受跟踪生成物还原失败' }

$keilGenerated = @(git ls-files --others --exclude-standard -- `
  MDK-ARM/MainCtrlF413MXOs `
  MDK-ARM/startup_stm32f413xx.lst)
$keilRoot = [System.IO.Path]::GetFullPath((Join-Path $root 'MDK-ARM'))
foreach ($relativePath in $keilGenerated) {
  $fullPath = [System.IO.Path]::GetFullPath((Join-Path $root $relativePath))
  if (-not $fullPath.StartsWith($keilRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "拒绝清理 MDK-ARM 外文件: $fullPath"
  }
  Remove-Item -LiteralPath $fullPath -Force
}
```

- [ ] **Step 7: 检查最终本地差异范围**

Run:

```powershell
git diff --check
git status --short --untracked-files=all
```

Expected: 仅出现以下三个路径：

```text
 M User/Application/Handle/handlekey.c
 M User/Application/include/handlekey.h
?? docs/superpowers/plans/2026-07-10-handlekey-legacy-cleanup.md
```

不得出现临时脚本、构建锁文件、空 `.d` 文件或其它业务模块。

---

### Task 4: 实机回归、用户确认和 Git 提交

**Files:**
- Test artifact: `EIDE/build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex`
- Commit: `User/Application/Handle/handlekey.c`
- Commit: `User/Application/include/handlekey.h`
- Commit: `docs/superpowers/plans/2026-07-10-handlekey-legacy-cleanup.md`

**Interfaces:**
- Consumes: Task 3 通过本机验证的 HEX。
- Produces: 用户确认无回归后的一次中文业务提交；未确认时不产生提交。

- [ ] **Step 1: 交付 HEX 并停止继续修改**

交付路径：

```text
D:\EH_main\soft\FinalSoft\EH-main-R1\EH-main-R1\EIDE\build\MainCtrlF413MXOs\MainCtrlF413MXOs.hex
```

在用户完成实机验证前：

- 不执行 `git add`。
- 不执行 `git commit`。
- 不开始 `handlescan.c`阶段。

- [ ] **Step 2: 用户执行实体键实机回归矩阵**

| 场景 | 操作 | 预期结果 |
| --- | --- | --- |
| A 通道 PXBA/PXBB | 当前选中 A，按一次实体键，再按一次 | 第一次启动，第二次停止；B 不动作 |
| B 通道 PXBA/PXBB | 当前选中 B，按一次实体键，再按一次 | 第一次启动，第二次停止；A 不动作 |
| A/B 同时在线 | 按非当前通道实体键 | 不切通道、不启动、不抢 owner |
| LGZI | 按住实体键后松开 | 按住运行，稳定松开后停止 |
| 脚控已选中 | 按实体键 | 电机不启动，显示/蜂鸣 82 号提示约 2000ms |
| 公共接头无刀具 | 按实体键 | 拒绝启动，保持原缺刀具提示行为 |
| 真实报警存在 | 按实体键 | 不启动，不覆盖真实报警 |
| 其它控制来源占用 | 按实体键 | 不抢控制权，不改变当前输出 |
| 注水泵联动 | 实体键启动和停止手柄 | 联动注水泵按原规则同时启停 |
| A/B 切换后再按 | 切换当前通道后操作对应实体键 | 只控制新选中的当前通道 |

同时观察：

- `WorkMessage.runflag_work`。
- `ControlSignalMessage.handle_control_flag`。
- `s_handle_run_key_owner_channel`。
- `WorkMessage.channel_work`。
- A/B 泵 `run_flag`。
- UART1 电机启动/停止帧。

- [ ] **Step 3: 等待用户明确确认**

只有用户明确回复该阶段烧录测试无问题，才能执行下一步。

如果任一场景有差异：

- 不提交 Git。
- 只回退 `handlekey.c/.h`本阶段修改。
- 保留问题现象和输入条件，重新定位后再生成新的测试版本。

- [ ] **Step 4: 用户确认后暂存精确文件**

Run:

```powershell
git add -- `
  User/Application/Handle/handlekey.c `
  User/Application/include/handlekey.h `
  docs/superpowers/plans/2026-07-10-handlekey-legacy-cleanup.md
git diff --cached --name-status
git diff --cached --check
```

Expected: 暂存区恰好包含三个文件，`git diff --cached --check`无输出。

- [ ] **Step 5: 提交第一阶段**

Run:

```powershell
git commit `
  -m '清理手柄实体键旧扫描链并明确运行时序' `
  -m '• 删除链接器已裁剪的旧 A/B 扫描入口、旧报警辅助函数和失效查询接口，不改变当前实体键运行链。' `
  -m '• 保持 30ms 任务周期、A 后 B 处理顺序、控制权门禁和注水泵联动，并把 82 号提示注释修正为当前 2000ms。' `
  -m '• 完成 AC5、Keil、map、编码和临时文件检查，并经实机覆盖 A/B、LGZI、脚控提示、报警门禁和联动泵场景。'
```

- [ ] **Step 6: 验证提交格式和工作区**

Run:

```powershell
git log -1 --pretty=fuller
git status --short --untracked-files=all
```

Expected:

- 中文业务标题正确。
- 正文使用 `•` 项目符号。
- 工作区为空。
- 提交完成后才能开始 `handlescan.c`第一小步。
