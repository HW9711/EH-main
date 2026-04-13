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
