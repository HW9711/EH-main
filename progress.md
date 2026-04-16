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

## 2026-04-16

- 完成一次新的只读工程基线复核，目标是“重新全面了解当前工程真实状态”，未修改任何业务源码。
- 核对了仓库形态：`EIDE/` 是 Git 根目录，但其工程文件通过相对路径管理上一层的 `Src/Inc/User/Drivers/Middlewares/MDK-ARM`。
- 重新确认启动与调度链路：
  - `main.c -> Hardware_PostInit() -> App_Bootstrap_Init() -> MX_FREERTOS_Init()`
  - `freertos.c -> Kernel_Scheduler_Start()`
  - `kernel_entry.c -> AppTaskScheduler_Init()`
  - `app_task.c` 中的 1ms 软调度器负责业务周期任务
- 复核了关键分层文件：
  - `.eide/eide.yml`
  - `Src/main.c`
  - `Src/freertos.c`
  - `Src/app_task.c`
  - `User/Application/Src/userparser.c`
  - `User/Hardware/Bsp/hw_bootstrap.c`
  - `User/App/Bootstrap/app_bootstrap.c`
  - `User/Kernel/Scheduler/kernel_entry.c`
  - `User/Kernel/Scheduler/kernel_scheduler.c`
- 复核了板级与外设抽象现状：
  - `board.h/board.c`
  - `board_profile.h`
  - `board_resource_map.h`
  - `bsp_gpio.h`
  - `bsp_uart.h/.c`
  - `bsp_i2c_bus.h`
- 复核了典型驱动/设备实现：
  - `uart1.c`、`uart5.c`
  - `i2c.c`
  - `iic.c`
  - `OneWireI.c`
  - `OneWireII.c`
  - `soft_uart.h/.c`
  - `at24cs32.c`
- 复核了当前业务层体量分布，确认 `screen.c`、`handlescan.c`、`footpedal.c`、`drivectrl.c`、`splittype.c`、`param.c` 仍是后续拆分与风险控制的重点大文件。
- 复核了全局状态模型，确认 `SysRunData/SysSetParam/...` 与 `Workvalue_s/ChannelValue_s` 两套运行态结构仍在并存使用。
- 发现并记录一处新的配置漂移风险：
  - `.ioc` 与 `board.h` 中部分串口仍是 `9600`
  - `Src/usart.c` 当前默认初始化则几乎全部为 `115200`
  - `uart1~uart7.c` 保留了动态改波特率入口，但本轮未检索到明确调用点
- 读取并还原了 `eeprom布局说明.txt` 的 UTF-8 内容，确认 EEPROM 页布局、页尾校验与数据分区设计仍有工程价值。
- 本轮没有运行构建或测试命令；当前结论基于代码、工程配置、历史计划文件与文档的交叉阅读。
