# V1.8 迁移发现记录

## 2026-04-23 Session 3 Findings

- `SendAlarmMessage()` 和 `SendKeyBeepMessage()` 依赖 `SscBeepControlTask_Init()` 初始化队列；如果仍启动旧 `BeepControlTask_Init()`，新接口报警和按键蜂鸣不会进入新蜂鸣队列。
- `pumpMessage_t.speed_work/speed_Max/speed_Min/speed_step_value` 必须是 `uint16_t`，否则 V1.8 里 `300ml` 灌注速度会被截断。
- `sscPUMPA/sscPUMPB` 原始 V1.8 代码只消费内部队列，不直接消费 `pumpMessageA/B`；迁移后必须让输出任务按公共泵消息结构输出，否则 `PUMPActive()` 改写的新接口不会真正驱动 UART5/UART7。
- `drivectrl.c`、`footpedal.c`、`pedal.c` 虽然任务入口已经下线，但仍提供 `warn`、`UI_FootPedalCalibration`、`UI_Main` 引用的辅助函数；删除这三个旧文件前，需要先迁移这些辅助函数或提供新的 UI/报警适配实现。
- `beep.c`、`radiofreq.c`、`splittype.c` 已无必要参与当前构建；对应职责已迁到 `sscBEEP.c` 和 `sscRFID.c`。

## 2026-04-24 Session 4 Findings

- `warn.c` 对 `drivectrl.c` 的有效外部依赖只有 `DriveCtrl_PumpFlag_A()` 和 `DriveCtrl_PumpFlag_B()`，旧电机运行任务入口已经未由 `Userparser_Init()` 启动。
- `drivectrl_adapter.c` 可以承接报警停泵职责：同步清公共泵消息状态，并直接下发 0 速，满足报警路径的即时停泵边界。
- `drivectrl.c` 移出构建后，`unify_builder --rebuild` 可链接通过，说明旧电机任务和旧 `Workvalue_s` 驱动路径已不再是当前构建必需项。
- `screen.c` 对当前构建的有效价值主要是 UI 绘图函数和少量全局刀具规格缓存；旧按键分发、蜂鸣、PUMPB 和数据仓库职责已经被 `Pubinterface/ssc*` 替代。
- `screen_adapter.c` 承接外部仍使用的 UI 绘图入口后，旧 `screen.c` 可以删除并保持链接通过。
- 当前构建仍有历史类型声明保留在 `screen.h` 中用于兼容头文件结构，但旧全局实例不再导出，业务运行态已不再通过旧全局实例访问。

## 2026-04-23 初始复核

- 当前作用域内存在两级规则文件：
  - `EIDE/AGENTS.md`
  - 上层 `..\Agents.md`
- 两级规则一致强调：
  - 源码改动必须用稳定编码。
  - 中文注释要工程化、可维护。
  - `handlescan.c` 的改动要最小化。
- `planning-with-files` 已启用，且旧 `task_plan.md/findings.md/progress.md` 内容已编码损坏，不适合继续作为本轮工作记忆，因此已准备重建。
- `git status --short` 显示当前非源码脏文件主要为：
  - `.eide/eide.yml`
  - `.omx/metrics.json`
  - `.omx/state/*`
  - `.omx/logs/*`
- 当前工程中本轮核心旧文件位于：
  - `..\User\Application\Screen\screen.c`
  - `..\User\Application\include\screen.h`
  - `..\User\Application\Handle\handlescan.c`
  - `..\User\Application\Handle\handlekey.c`
  - `..\User\Application\FootPedal\footpedal.c`
  - `..\User\Application\DriveCtrl\drivectrl.c`
  - `..\User\Application\Pump\pump.c`
  - `..\User\Application\RadioFreq\radiofreq.c`
  - `..\User\Application\SplitType\splittype.c`
  - `..\User\Application\MotorUartData\motoruartdata.c`
  - `..\User\Application\Src\userparser.c`
  - `..\User\Peripheral\uart\soft_uart.c`
  - `..\User\Peripheral\include\soft_uart.h`
- `V1.8` 参考工程中已确认存在：
  - `User\Application\Pubinterface\Pubinterface.c`
  - `User\Application\include\Pubinterface.h`
  - `User\Application\include\sscBEEP.h`
  - `User\Application\include\sscDRIVE.h`
  - `User\Application\include\sscFOOT.h`
  - `User\Application\include\sscKEYBH.h`
  - `User\Application\include\sscPUMPA.h`
  - `User\Application\include\sscPUMPB.h`
  - `User\Application\include\sscRFID.h`
  - `User\Application\include\sscUIDP.h`
- `V1.8` 的 `ssc*.c` 实际集中在 `User\Application\Beep\` 目录，不符合目录名直觉，后续必须按文件内容而非目录名迁移。

## 待确认问题

1. 当前 `.eide/eide.yml` 的未提交改动是否就是上一次记录残留，还是用户刚手动调整过。
2. 当前工程是否已有部分 `V1.8` 新接口残片，避免重复导入或命名冲突。
3. `screen.c` 内哪些函数仍需作为 UI 辅助逻辑保留，哪些能完全由 `sscUIDP` 接管。

## 2026-04-23 第一轮差异扫描

- 当前工程 `Userparser_Init()` 仍按旧路径启动以下任务和初始化：
  - `RadioFreq_Init()`
  - `BeepControlTask_Init()`
  - `HandlescanTaskInit()`
  - `FootPedalTask_Init()`
  - `ScreenKeyTask_Init()`
  - `HandleKeyScan_Init()`
  - `SplitType_AutoModeGetData_Init()`
  - `PUMPBTask_Init()`
  - `SimUartTask_Init()`
- `handlekey.c` 的 `HAL_GPIO_EXTI_Callback()` 仍直接调用 `SimUart_HandleExti(GPIO_Pin)`，这是必须保留的模拟串口入口。
- 当前工程的老接口污染面很大：
  - `screen.c` 定义 `Workvalue_s` 与 `ChannelValue_s`
  - `screenkey.c`、`footpedal.c`、`handlekey.c`、`splittype.c`、`UI_*` 等多模块直接读写 `Workvalue_s`
- `V1.8` 参考工程中的新模块仍自带旧调度模型：
  - 广泛使用 `xQueueCreate/xQueueSend/xQueueReceive`
  - 初始化函数仍调用 `app_task_create/app_task_start`
- `V1.8` 中目标迁入模块与当前工程旧模块存在明确重名冲突：
  - `BeepControlTask_Init`
  - `ScreenKeyTask_Init`
  - `PUMPBTask_Init`
  - `RadioFreq_Init`
  - `SplitType_AutoModeGetData_Init`
  - `HandleKeyScan_Init`
- `V1.8` 新模块并没有完全替代旧系统：
  - `sscRFID.c` 同名导出 `RadioFreq_Init()` 和 `SplitType_AutoModeGetData_Init()`
  - `handlekey.c` 仍保留大量旧逻辑的注释残片
  - `Pump/pump.c`、`MotorUartData/motoruartdata.c` 等旧文件在 `V1.8` 也仍存在
- 当前 `.eide/eide.yml` 的未提交差异不是本轮迁移内容，而是 Tracealyzer 重新加入、下载器切到 `JLink`、下载速度变更；迁移时必须避免误覆盖这些用户现有改动。

## 当前判断

1. 不能简单把 `V1.8` 文件整包覆盖到当前工程。
2. 更安全的路线是：
   - 先引入 `Pubinterface` 数据模型。
   - 再把 `ssc*` 模块中的业务逻辑迁入当前目录结构并替换任务创建接口。
   - 最后逐步切断旧模块对 `Workvalue_s/ChannelValue_s` 的依赖。

## 2026-04-23 新模块骨架落地

- 已把以下 `V1.8` 文件复制到当前仓：
  - `User/Application/Pubinterface/Pubinterface.c`
  - `User/Application/include/Pubinterface.h`
  - `User/Application/include/sscBEEP.h`
  - `User/Application/include/sscDRIVE.h`
  - `User/Application/include/sscFOOT.h`
  - `User/Application/include/sscKEYBH.h`
  - `User/Application/include/sscPUMPA.h`
  - `User/Application/include/sscPUMPB.h`
  - `User/Application/include/sscRFID.h`
  - `User/Application/include/sscUIDP.h`
  - `User/Application/Beep/sscBEEP.c`
  - `User/Application/Beep/sscDrive.c`
  - `User/Application/Beep/sscFOOT.c`
  - `User/Application/Beep/sscKEYBH.c`
  - `User/Application/Beep/sscPUMPA.c`
  - `User/Application/Beep/sscPUMPB.c`
  - `User/Application/Beep/sscRFID.c`
  - `User/Application/Beep/sscUIDP.c`
- 所有新 `ssc*` 任务入口已从 `app_task_create/app_task_start` 切到当前工程的 `Kernel_TaskCreate/Kernel_TaskStart`。
- `Pubinterface.c` 已修正 `pumpMessageInit()` 重复清零 `pumpMessageA` 的已知 bug，第二次清零改为 `pumpMessageB`。
- 当前新模块仍未接入构建列表，也未替换掉旧 `screen.c/radiofreq.c/splittype.c` 的同名出口，因此此时还只是“可继续集成的骨架”，不是可编译最终态。

## 2026-04-23 可编译接入与首批接口迁移

- 新 `ssc*` 初始化入口已统一加 `Ssc` 前缀，避免旧模块尚未移除时发生链接重名：
  - `SscBeepControlTask_Init()`
  - `SscDriveMotorTask_Init()`
  - `SscFootControlTask_Init()`
  - `SscKeyBehaviorTask_Init()`
  - `SscPumpATask_Init()`
  - `SscPumpBTask_Init()`
  - `SscRadioFreq_Init()`
  - `SscSplitTypeAutoModeGetData_Init()`
  - `SscUIDisplayTask_Init()`
- `Pubinterface.h/.c` 已补齐 `WorkMessageInit()`、`ChannelMemoryMessageInit()`，并为历史声明 `ChannelMessageInit()`、`ChannelFlagMessageInit()` 提供统一清零实现。
- `.eide/eide.yml` 与 Keil `.uvprojx` 已接入：
  - `User/Application/Pubinterface/Pubinterface.c`
  - `User/Application/Beep/sscBEEP.c`
  - `User/Application/Beep/sscDrive.c`
  - `User/Application/Beep/sscFOOT.c`
  - `User/Application/Beep/sscKEYBH.c`
  - `User/Application/Beep/sscPUMPA.c`
  - `User/Application/Beep/sscPUMPB.c`
  - `User/Application/Beep/sscRFID.c`
  - `User/Application/Beep/sscUIDP.c`
- `Userparser_Init()` 已在旧任务启动前初始化新接口数据，并启动 `SscKeyBehaviorTask_Init()` 用于接收新插拔/手柄按键事件。
- `handlescan.c` 已完成首批新接口迁移：
  - 不再写 `Workvalue_s/ChannelValue_s`。
  - 通道在线写入 `WorkMessage.Channel_Aonline/Channel_Bonline`。
  - 手柄型号写入 `MemoryMsgA/B.hand_model` 与 `ChannelrecognizeMessageA/B.handle_type`。
  - 报警写入 `WorkMessage.alarm_flag/alarm_value`，并通过 `SendAlarmMessage()` 触发新蜂鸣事件。
  - 插拔事件通过 `SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A/B/UNPLUG_A/B)` 发给 `sscKEYBH`。
  - 原状态机、I2C2/I2C3 认证、去抖、调试报文和 `paoxueSpeciValue_A/B` 临时 UI 规格缓存仍保留。
- `handlekey.c` 已完成首批新接口迁移：
  - `HAL_GPIO_EXTI_Callback()` 中 `SimUart_HandleExti(GPIO_Pin)` 保持不变。
  - SSC 扫描函数不再写 `Workvalue_s`，改为使用 `WorkMessage`、`ControlSignalMessage`、`SendKeyBehMessage(HANDLEKey, ...)` 和 `SendAlarmMessage()`。
- 全工程剩余 `Workvalue_s/ChannelValue_s` 依赖仍很多，静态计数约为 `1399` 行命中，主要集中在 `screen.c`、`footpedal.c`、`drivectrl.c`、`pump.c`、`radiofreq.c`、`splittype.c`、`motoruartdata.c`、`screenkey.c` 和 UI 旧路径。
- 当前构建命令已验证通过：
  - `& 'C:\Users\Dell\.vscode\extensions\cl.eide-3.26.7\res\tools\win32\unify_builder\unify_builder.exe' -p build\MainCtrlF413MXOs\builder.params --rebuild`
  - 输出 `build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex`
  - 输出 `build/MainCtrlF413MXOs/MainCtrlF413MXOs.s19`

# 2026-04-24 Session 5 Findings

- `ConnectScan`、`DirCurrent`、`Encryption`、`SerialPortPro` 在当前运行入口中没有被调用；map 中对应对象主要被链接器裁剪，不参与最终有效业务路径。
- `RadioFreq`、`SplitType` 旧目录已经为空；当前仍出现的 `SscRadioFreq_Init()`、`SscSplitTypeAutoModeGetData_Init()` 是 `sscRFID.c` 内的新接口，不是旧目录依赖。
- 清理后 `Application` 下剩余模块均仍有源码文件或头文件引用；更激进地删除 `Warn`、旧 UI 刷新、`screen_adapter` 等会牵动显示/告警兼容层，应另起一轮按行为验证处理。
- `unify_builder --rebuild` 成功，说明 EIDE 当前构建链已接受本次模块清理；Keil `.uvprojx` 与 EIDE `.yml` 均已同步去掉被删模块。
