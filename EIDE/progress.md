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
