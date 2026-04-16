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

## 2026-04-16 工程基线复核

- `EIDE/` 目录是当前 Git 根目录，但它通过 `.eide/eide.yml` 中的大量 `../Src`、`../User`、`../Drivers` 相对路径管理上一层的真实 STM32 工程源码树；后续变更需要同时注意“版本库根”和“源码物理根”不在同一层。
- 顶层源码组织已经比较清晰：
  - `Src/Inc`：CubeMX 生成入口与 HAL 启动文件
  - `Drivers`：CMSIS + STM32 HAL
  - `Middlewares`：FreeRTOS
  - `User`：业务、外设、UI、数据模型、板级抽象与新分层壳层
  - `MDK-ARM`：Keil 工程与构建产物
  - `EIDE`：EIDE 工程、OMX 状态、调研记录与辅助脚本
- 现行启动链已核实为：
  - `main.c`
  - `Hardware_PostInit()`
  - `App_Bootstrap_Init()`
  - `MX_FREERTOS_Init()`
  - `Kernel_Scheduler_Start()`
  - `AppTaskScheduler_Init()`
  - FreeRTOS 任务 `AppTaskScheduler`
- `AppTaskScheduler` 是当前业务周期调度核心：它由 FreeRTOS 创建为单个宿主任务，以 1ms Tick 周期扫描任务链表；大量应用任务通过 `Kernel_TaskCreate/Start/Stop` 间接挂载到这套软调度器上。
- `User` 目录同时保留两类结构：
  - 历史业务目录：`Application`、`Peripheral`、`UI`、`Data`、`board`
  - 新分层入口目录：`Hardware`、`Driver`、`Kernel`、`App`
- 当前代码规模最集中的业务模块为：
  - `User/Application/Screen/screen.c`：181209 字节
  - `User/Application/Handle/handlescan.c`：57244 字节
  - `User/Application/FootPedal/footpedal.c`：53096 字节
  - `User/Application/DriveCtrl/drivectrl.c`：43241 字节
  - `User/Application/SplitType/splittype.c`：39161 字节
  - `User/Application/ScreenKey/param.c`：35470 字节
- `soft_uart` 已不是简单单通道 bit-bang：
  - 对外提供两路 `SimUart` 通道
  - RX 由 `TIM11` 中断采样驱动
  - 起始位捕获挂在 `HAL_GPIO_EXTI_Callback()`
  - 数据落地走 FreeRTOS 静态队列
  - 另有一个 100ms 周期后台任务负责把 ISR 环形缓冲搬运到队列
- `AT24CS32` 驱动已经体现出“新风格”演进：具备分块读写、页校验、最近一次调试快照、`HAL_BUSY/HAL_TIMEOUT` 判定与 I2C 软恢复流程，明显比大量历史业务模块更工程化。
- `i2c.c` 当前承担的是“硬件总线实现”职责而不是普通设备驱动职责：
  - 直接声明 `hi2c2/hi2c3`
  - 自行完成 `HAL_I2C_MspInit/DeInit`
  - 暴露 `MX_I2C_Init()` 与 I2C2/I2C3 的一组 HAL 包装接口
- `iic.c`、`OneWireI.c`、`OneWireII.c` 当前已经完成第一步 GPIO 收口，底层宏都改成通过 `Bsp_GpioRead/Write + board_resource_map` 访问语义资源。
- 运行态“双状态模型并存”在当前代码中仍是事实而非历史结论：
  - `SysRunData / SysSetParam / SysInterface / SysHandleData / SysFootPedalData` 定义在 `User/Data`
  - `Workvalue_s / ChannelValue_s` 定义在 `User/Application/Screen/screen.c`
  - `motoruartdata.c`、`splittype.c`、`footpedal.c`、`handlekey.c`、`handlescan.c`、`drivectrl.c` 等仍直接读写 `Workvalue_s`
- 串口配置存在一处值得后续重点盯住的配置漂移风险：
  - `.ioc` 中 `USART1/USART2/UART5/UART7` 仍记录为 `9600`
  - `User/board/board.h` 的 `BOARD_UART_LIST` 也保留了这组 `9600/115200` 混合值
  - 但 `Src/usart.c` 当前生成代码里 1/2/3/4/5/6/7/8/10 口默认初始化全部是 `115200`
  - `uart1~uart7.c` 虽然都保留了 `UartX_Configuration(uint16_t baud)`，但本轮检索未发现明确调用点
- `MainCtrlF413MXOs.ioc` 还原出的硬件画像为：
  - MCU：`STM32F413VGT6`
  - 时钟：HSE 8MHz，系统 100MHz
  - RTOS：FreeRTOS（CubeMX 侧仅保留 `defaultTask` 宿主）
  - 定时器：`TIM7`、`TIM10`、`TIM14`
  - 硬件 I2C：`I2C2`、`I2C3`
  - 硬件串口：`USART1/2/3/6`、`UART4/5/7`，另源码侧存在 `UART8/10`
  - DMA：多路 UART RX 循环接收
- `eeprom布局说明.txt` 为 UTF-8 正常文件，记录了 32 字节分页布局与页尾校验规则：
  - Page1：加密区 / CRC
  - Page2：手柄适配信息
  - Page3：刀具信息
  - Page4：初始参数
  - Page6-7：多档位参数
  - Page8：使用记录
  - Page11：出厂信息
