# 开发进度记录

## 2026-04-12

- 完成工程首轮系统阅读，梳理了启动链路、调度模型、板级层、驱动层、应用层、遗留框架关系。
- 输出四层分层方案：`Hardware / Driver / Kernel / Application`。
- 建立首批稳定骨架：
  - `hw_bootstrap`
  - `app_bootstrap`
  - `kernel_entry`
  - `kernel_scheduler`
  - `kernel_osal`
  - `bsp_i2c_bus`
- 完成 `main.c`、`freertos.c`、`usart.c` 的桥接式改造，降低 CubeMX 再生破坏架构的风险。
- 重组 EIDE 工程视图与 include 路径，使四层结构在 EIDE 中可直接定位。
- 清理 `User` 层对旧板级、旧任务调度、原生 FreeRTOS API 的直接依赖。
- 将以下模块迁到 `kernel_scheduler` / `kernel_osal` 包装接口：
  - `connectscan`
  - `warn`
  - `UI_Main`
  - `dircurrent`
  - `handlekey`
  - `iwdg`
  - `pedal`
  - `motoruartdata`
  - `footpedal`
  - `screen`
  - `screenkey`
  - `pump`
  - `splittype`
  - `drivectrl`
  - `handlescan`
- 建立板级稳定壳层：
  - `board_profile.h`
  - `board_resource_map.h`
  - `bsp_gpio.h`
  - `bsp_uart.h`
  - `bsp_uart.c`
- 完成部分 GPIO/UART 迁移：
  - `sysrunled`
  - `handlekey`
  - `handlescan`
- 修复一次 `handlescan.c` 注释起始符缺失导致的编译错误。保持中文注释与业务逻辑不变。
- 已完成增量编译验证。完整构建闭环记录待补。

## 当前状态

- 架构重构已从“方案阶段”进入“落地阶段”。
- 当前最合适的推进顺序：
  1. `uart1~uart7`
  2. `OneWire / iic / i2c`
  3. `Board_GPIOConfiguration()` 收口
  4. 再做运行态模型统一和大文件拆分

## 2026-04-13

- 已扩展 `bsp_uart.h/.c`，统一纳管 `UART1~UART7` 与 `UART10`。
- 已新增通用 UART 壳层能力：
  - `Bsp_UartInit`
  - `Bsp_UartDeInit`
  - `Bsp_UartAbort`
  - `Bsp_UartReceiveDma`
  - `Bsp_UartDmaStop`
  - `Bsp_UartRxDmaRemain`
- 已将 `uart1.c` 到 `uart7.c` 从直接操作 `huartX/HAL_UART_*` 改为通过 `bsp_uart` 壳层访问。
- `uart7.c` 的异常重发路径已保持原波特率，不再写死恢复值。
- 已做一次干净目录全量编译验证：
  - 输出目录：`build/MainCtrlF413MXOs_tmp`
  - `.hex` 与 `.s19` 已生成
- 原默认输出目录 `build/MainCtrlF413MXOs` 当前存在缓存文件锁定问题，导致直接 `--rebuild` 会报 `Permission denied`；已通过临时输出目录绕开验证。
- 已将应用层对 `Board_GPIOConfiguration()` 的直接调用收口进 `hw_bootstrap`：
  - 新增 `Hardware_BoardGpioInit()`
  - `userparser.c` 已改走壳层入口
- 已做一次新的干净目录全量编译验证：
  - 输出目录：`build/MainCtrlF413MXOs_tmp2`
  - `.hex` 与 `.s19` 已生成
- 已将 `OneWireI.c`、`OneWireII.c`、`iic.c` 的 GPIO 直连宏改为通过 `bsp_gpio + board_resource_map` 访问。
- `i2c.c` 已补 `main.h`，清掉本文件的 `Error_Handler` 隐式声明告警。
- 已做一次新的干净目录全量编译验证：
  - 输出目录：`build/MainCtrlF413MXOs_tmp3`
  - `.hex` 与 `.s19` 已生成
- 已将 `MX_I2C_Init()` 从 `main.c` 移出，改由 `Hardware_PostInit()` 统一接管。
- `main.c` 进一步减少对硬件初始化细节的直接依赖。
- 已做一次新的干净目录全量编译验证：
  - 输出目录：`build/MainCtrlF413MXOs_tmp4`
  - `.hex` 与 `.s19` 已生成
- 已将 `board.h` 中未被外部使用的旧板级初始化接口声明移除：
  - `Board_UART_Init`
  - `Board_ADC_Init`
  - `Board_I2C_Init`
  - `Board_OneWire_Init`
  - `Board_TIM_Init`
  - `Board_Hardware_Init`
- 上述函数已在 `board.c` 内部收为文件私有 `static`，减少板级头文件暴露面。
- 已做一次新的干净目录全量编译验证：
  - 输出目录：`build/MainCtrlF413MXOs_tmp5`
  - `.hex` 与 `.s19` 已生成
  - 编译链接成功；`unify_builder` 末尾仍有 SQLite 缓存写回报错，但不影响产物生成
## 2026-04-13 soft_SSC Review

- Reviewed external module bundle under `D:\EH_main\soft\soft_SSC`
- Confirmed it is an application-layer module set with task + queue + UI + protocol logic
- Mapped module responsibilities and current-project coupling points
- Identified direct merge blockers:
  - duplicate public init names
  - direct `Workvalue_s` coupling
  - direct LCD/UART/Pump calls
  - direct `app_task` + FreeRTOS queue usage
- Ready for next step: produce integration plan and migration order for selective merge into current project

## 2026-05-15 APP Task Independent Thread Refactor

- Started behavior-preserving refactor from single `AppTask` software scheduler to independent static FreeRTOS threads.
- Read existing `Src/app_task.c`, `Inc/app_task.h`, `kernel_scheduler` wrapper, and prior similar EH_test rollout summary.
- Decision: keep public APIs unchanged, allocate task TCB/stack statically inside each `task_t`, and serialize callback bodies with a shared runtime mutex to preserve old one-callback-at-a-time semantics.
- Added `Tools/firmware-tests/app_task_independent_threads.test.mjs` to lock the new architecture:
  - each registered soft task must own `StaticTask_t`, `StackType_t[]`, and `TaskHandle_t`
  - `Src/app_task.c` must use `xTaskCreateStatic`, `xSemaphoreCreateMutexStatic`, `AppTaskWorker`, and `AppTaskRuntimeGate`
  - old dynamic `xTaskCreate(AppTaskScheduler...)` entry must stay removed
- Refactored `Src/app_task.c`:
  - `app_task_create_named()` now creates one static FreeRTOS worker per `task_t`
  - `AppTaskScheduler_Init()` now only creates the shared static runtime mutex
  - callback execution remains serialized by `AppTaskRuntimeGate()` to avoid changing legacy global-state access behavior
  - Tracealyzer soft-task lifecycle markers are preserved around the new callback gate
- Verification completed:
  - `Get-ChildItem -Path Tools\firmware-tests -Filter *.test.mjs | ForEach-Object { node $_.FullName }` passed
  - EIDE `unify_builder --rebuild` passed
  - linked image memory: RAM `131.1KB/320.0KB`, ROM `86.4KB/1024.0KB`
