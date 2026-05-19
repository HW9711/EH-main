# 工程阅读与重构发现

## 工程现状

- 实际运行主线为 `main.c -> Userparser_Init() -> MX_FREERTOS_Init()`。
- `FreeRTOS` 当前主要承担宿主角色。业务周期调度核心在 `Src/app_task.c` 的 1ms 软件调度器。
- 工程存在明显历史迁移痕迹：`Cola_os` 旧框架、CubeMX 生成层、自定义 `board.c` 板级层、手写总线层长期并存。
- 当前最核心架构问题不是单个文件过大，而是两套运行态并存：
  - 旧体系：`SysRunData / SysSetParam / ...`
  - 新体系：`Workvalue_s / ChannelValue_s / ...`
- `screen.c`、`drivectrl.c`、`motoruartdata.c`、`screenkey.c` 等模块存在新旧状态交叉读写。

## 本轮已落地结果

- `main.c` 已通过 `Hardware_PostInit()` 与 `App_Bootstrap_Init()` 接入稳定层。
- `freertos.c` 已通过 `Kernel_Scheduler_Start()` 接入统一调度入口。
- `.eide/eide.yml` 已重组为 `Hardware / Driver / Kernel / Application` 四层视图。
- `User` 层已清空对以下旧直连的直接依赖：
  - `board.h`
  - `app_task.h`
  - `FreeRTOS.h`
  - `task.h`
  - `vTaskDelayUntil`
  - `xTaskGetTickCount`
- `screen.c`、`drivectrl.c`、`handlescan.c` 已迁到 `kernel_scheduler.h` 包装接口。
- 已新增稳定硬件接口：
  - `board_profile.h`
  - `board_resource_map.h`
  - `bsp_gpio.h`
  - `bsp_uart.h`
  - `bsp_uart.c`
  - `bsp_i2c_bus.h`
- 已将 `sysrunled.c`、`handlekey.h`、`handlekey.c`、`handlescan.c` 的部分 GPIO/UART 访问切到 `Bsp` 壳层。

## 已确认风险点

- `handlescan.c` 对编码和中文注释极敏感。后续仍需最小改动策略。
- `User/Peripheral/uart/uart1~uart7.c` 仍直接依赖 `huartX` 与 HAL UART API。
- `User/Peripheral/bus/iic.c`、`OneWireI.c`、`OneWireII.c`、`i2c.c` 仍存在 GPIO 或 RCC 级直连。
- `userparser.c` 仍直接调用 `Board_GPIOConfiguration()`，应用层与板级初始化尚未完全解耦。

## 已验证事实

- `handlescan.c` 曾因注释起始符缺失引发编译错误，已用最小修复补回，编译恢复。
- 已完成增量编译验证的文件：
  - `screen.c`
  - `drivectrl.c`
  - `handlescan.c`
  - `bsp_uart.c`
  - `sysrunled.c`
  - `handlekey.c`
- 当前工具链路径已固定为 `C:\Keil_v5\ARM\ARMCC`。

## 后续最高价值切入口

1. 统一 `uart1~uart7`
2. 收敛 `OneWire / iic / i2c`
3. 将 `Board_GPIOConfiguration()` 收口进 `hw_bootstrap`
4. 再进入运行态模型统一与大模块拆分

## 2026-04-13 新发现

- `uart1~uart7.c` 原实现高度模板化，主要差异只有：
  - 端口号
  - DMA 缓冲区大小
  - 发送超时
  - `uart7` 的异常恢复路径
- 当前最稳妥重构方式不是一次性合并 7 个模块，而是先把底层硬件访问统一收口到 `bsp_uart`，保留原对外接口。
- `build/MainCtrlF413MXOs` 目录存在对象缓存锁定问题，`unify_builder --rebuild` 在该目录下会因删除 `.d/.o` 失败而中断；但切换到新输出目录后，工程可完整编译并成功链接。
- `userparser.c` 原本是应用层直调 `Board_GPIOConfiguration()` 的最后热点之一，现已改为经 `hw_bootstrap` 转调，应用层与板级 GPIO 初始化边界进一步清晰。
- `OneWireI/II` 与 `soft IIC` 的主要硬件耦合点集中在 GPIO 拉高/拉低/读引脚；这类点适合优先抽成 `bsp_gpio + 语义资源映射`，无需先重写协议时序逻辑。
- `i2c.c` 当前仍承担 HAL I2C 实例与 MSP 初始化职责，更像“硬件总线实现”而不是纯设备驱动；后续若继续收口，宜将其逐步迁入硬件层或再包一层更明确的 `bsp_i2c_hal`。
## 2026-04-13 soft_SSC External Module Findings

- Source path: `D:\EH_main\soft\soft_SSC`
- File set is a small business-module bundle, not a standalone reusable BSP/driver package.
- Main files:
  - `Pubinterface.[ch]`: shared message definitions, key ids, channel/pump state structs, behavior dispatch helpers
  - `sscKEYBH.[ch]`: key behavior queue + dispatcher
  - `sscUIDP.[ch]`: LCD UI adapter and display task
  - `sscRFID.[ch]`: RFID/UART3 polling + parser + queue trigger
  - `sscDrive.[ch]`: motor frame packing + UART1 send task
  - `sscPUMPA.[ch]`, `sscPUMPB.[ch]`: pump setpoint queue + conversion + pump output task
  - `sscBEEP.[ch]`: beep queue + beep task
- Strong coupling to current project:
  - includes `screen.h`, `data.h`, `datahand.h`, `lcd.h`, `pump.h`, `uart1.h`, `uart3.h`, `Motor.h`, `board.h`
  - directly reads/writes `Workvalue_s`
  - directly calls `LCD_Show_Picture`, `Pump_SetSpeed_A/B`, `Uart1_SendPacket`, `Uart3_SendPacket`
  - directly uses `app_task` and FreeRTOS queues
- Conclusion:
  - these files overlap heavily with existing project application modules
  - they should be treated as an external application-layer feature set to be merged selectively
  - they should not be copied into Driver/Hardware as-is
- Integration risk hotspots:
  - duplicate symbol names already exist in current project: `BeepControlTask_Init`, `ScreenKeyTask_Init`, `SplitType_AutoModeGetData_Init`
  - logic overlap with current `screen.c`, `screenkey.c`, `splittype.c`, `pump.c`, `drivectrl.c`
  - queue/task model still uses `app_task` directly, not current kernel wrapper

## 2026-05-15 APP Task Independent Thread Refactor Findings

- Current scheduler entry is `Kernel_Scheduler_Start() -> AppTaskScheduler_Init()`.
- Current `Src/app_task.c` creates one FreeRTOS task named `AppTask` with `xTaskCreate()`, then scans a linked list every 1 ms.
- Business modules already call `Kernel_TaskCreate/Kernel_TaskStart`; preserving `app_task_create/start/stop` and `Kernel_Task*` signatures avoids rewriting all module call sites.
- The behavior-preserving migration pattern from the prior EH_test checkout is applicable here: create one static FreeRTOS thread per registered soft task, but guard callback execution with one shared runtime mutex so existing callbacks remain serialized instead of becoming fully concurrent.
- Final implementation keeps that pattern:
  - no dynamic `AppTask` scheduler task remains
  - registered soft tasks are created with `xTaskCreateStatic()` using memory embedded in `task_t`
  - `AppTaskRuntimeGate()` is the single serialized callback entry, preserving old one-callback-at-a-time behavior
  - `AppTaskScheduler_Init()` is retained as the public scheduler-init hook but now initializes only the shared static mutex
- Static allocation increased ZI RAM as expected but remains acceptable after full link: `131.1KB/320.0KB` RAM used.
