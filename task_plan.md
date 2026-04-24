# V1.8 模块与新接口数据结构迁移计划

## 2026-04-23 Session 3 Update

- Phase 4 继续推进：`screenkey.c`、`motoruartdata.c` 和 `Userparser_Init()` 已切到新接口/新 `Ssc*` 任务入口；手柄扫描、手控按键、软串口入口保持已验证状态。
- Phase 5 已部分开始：`beep.c/radiofreq.c/splittype.c` 已从 EIDE、Keil 和当前 `builder.params` 构建列表移除；`drivectrl.c/footpedal.c/pedal.c` 仍因 UI/报警辅助函数引用暂留。
- Phase 6 已执行多次中间构建验证：当前 EIDE `unify_builder --rebuild` 可生成 `.hex/.s19`。

## 2026-04-24 Session 4 Update

- Phase 5 继续推进：`drivectrl.c` 已从 EIDE、Keil 和当前 `builder.params` 构建列表移除，改由 `drivectrl_adapter.c` 提供报警停泵兼容入口。
- `warn.c` 仍可调用 `DriveCtrl_PumpFlag_A/B()`；适配实现已切到 `pumpMessageA/B` 并立即下发 `Pump_SetSpeed_A/B(0)`。
- Phase 6 中间验证通过：`unify_builder --rebuild` 在 `2026-04-24 10:48:36` 生成新的 `.hex/.s19`。
- 当前主要剩余旧接口债务：`screen.c` 和 `screen.h` 中的数据仓库/UI 路径。

## 2026-04-24 Final Update

- Phase 4 完成：旧 `screen.c` 的按键分发、蜂鸣、PUMPB 和数据仓库职责已由 `Pubinterface/ssc*` 路径承接；外部仍需的 UI 绘图入口由 `screen_adapter.c` 提供。
- Phase 5 完成：旧 `screen.c`、`drivectrl.c` 已删除；构建清单只保留 `screen_adapter.c`、`drivectrl_adapter.c` 与新 `ssc*` 模块。
- Phase 6 完成：`rg "Workvalue_s|ChannelValue_s" ..\User\Application ..\User\UI ..\Src` 无命中；构建清单中 `screen.c/drivectrl.c/beep.c/radiofreq.c/splittype.c/footpedal.c` 无命中。
- 最终构建通过：`unify_builder --rebuild` 在 `2026-04-24 10:55:43` 生成 `MainCtrlF413MXOs.hex` 和 `MainCtrlF413MXOs.s19`。

## 目标

- 在当前 `V1.5-EH_mainReconstructV1` 架构不变的前提下，引入 `V1.8` 的 `Pubinterface` 数据模型和 `sscBEEP/sscDRIVE/sscFOOT/sscKEYBH/sscPUMPA/sscPUMPB/sscRFID/sscUIDP` 模块。
- 保留当前工程已有的 `handlescan` 状态机、I2C 识别、去抖、报警触发、调试输出和 `soft_uart` 模拟串口收发入口，只替换它们读写的业务数据接口。
- 最终彻底移除 `Workvalue_s/ChannelValue_s` 老接口和被新 `ssc*` 模块替代的旧模块文件，并保证 EIDE/Keil 可完整构建。

## 当前约束

- `EIDE/AGENTS.md` 与上层 `Agents.md` 要求：源码新增或修改时必须使用稳定编码，并补充详细、专业、可维护的中文注释。
- `handlescan.c` 和 `handlekey.c` 属于编码敏感文件，改动必须最小化，不能破坏已有中文注释和状态机流程。
- 现有工作区已存在未提交变更：`.eide/eide.yml` 和 `.omx/*`。迁移期间只能在确认差异后叠加修改，不能直接覆盖。
- `planning-with-files` 已启用，本文件、`findings.md`、`progress.md` 作为本轮迁移的唯一工作记录。

## 阶段计划

| 阶段 | 状态 | 内容 |
| --- | --- | --- |
| 1 | completed | 复核工程约束、旧记录、源码目录和 `V1.8` 参考模块位置 |
| 2 | completed | 梳理当前工程与 `V1.8` 的接口差异、任务入口、重复符号和构建清单 |
| 3 | completed | 导入并适配 `Pubinterface.[ch]` 与 `ssc*` 模块到当前调度架构 |
| 4 | completed | 将 `handlescan/handlekey/footpedal/drivectrl/pump/radiofreq/UI` 的数据读写切换到新接口 |
| 5 | completed | 删除旧接口与废弃模块，清理 `.eide/eide.yml`、`.uvprojx`、include 路径和构建列表 |
| 6 | completed | 执行静态搜索、链接检查和 EIDE/Keil 构建验证，确认 `.hex/.s19` 产物 |

## 本轮验收口径

1. 全工程搜索不再残留有效业务路径上的 `Workvalue_s`、`ChannelValue_s` 依赖。
2. `Userparser_Init()` 保持当前硬件初始化、看门狗、LCD 开机页、`HandlescanTaskInit()`、`SimUartTask_Init()` 的顺序，只改为初始化新数据并挂接新任务。
3. `HAL_GPIO_EXTI_Callback()` 中 `SimUart_HandleExti(GPIO_Pin)` 入口必须仍然存在。
4. 旧 `screen.c/screen.h` 承担的数据仓库、按键分发、蜂鸣、PUMPB 等职责迁走后再删除；若有 UI 辅助函数仍被引用，先迁入 `sscUIDP` 或专用适配文件。
5. 构建结果至少完成一次干净输出目录验证，避免默认 `build` 目录文件锁导致假失败。

## 已知风险

| 风险 | 当前判断 | 预案 |
| --- | --- | --- |
| `V1.8` 模块路径混乱，部分 `ssc*.c` 位于 `Beep/` 目录 | 高 | 以实际文件内容和 include 关系为准，不按目录名做假设 |
| 当前 `.eide/eide.yml` 已有未提交变更 | 高 | 先查看差异，再做最小叠加编辑 |
| `handlekey.c` 同时涉及按键逻辑和模拟串口 EXTI 入口 | 高 | 迁移时明确保留 `SimUart_HandleExti()` 调用链 |
| 老 `screen.c` 体量大、职责杂 | 高 | 先做职责映射，再分批搬迁，最后删除 |

## 错误记录

| 问题 | 尝试 | 结论 |
| --- | --- | --- |
| `planning-with-files` 说明中的 `session-catchup.py` 不存在 | 1 | 本机未安装该脚本，改为手工读取现有规划文件与 `git status` 恢复上下文 |
| `unify_builder.exe` 不在当前 PATH | 1 | 使用 VS Code EIDE 扩展自带路径 `C:\Users\Dell\.vscode\extensions\cl.eide-3.26.7\res\tools\win32\unify_builder\unify_builder.exe` 完成构建 |
| `sscRFID.c` 的 const 命令数组无法传给当前 `Uart3_SendPacket(uint8_t *)` | 1 | 将本文件内部读 USER 命令缓存改为普通 `static uint8_t`，发送字节不变 |
| 新 `ssc*` 与旧模块链接重名 | 1 | 新初始化入口统一改为 `Ssc*` 前缀；旧模块仍在构建时，新模块内部任务句柄、任务函数和 RFID 命令表改为文件私有 |
| `drivectrl_adapter.c` 首次构建缺少 `NULL` 定义 | 1 | 新增 `<stddef.h>`，保持适配文件只承担报警停泵桥接 |

# 2026-04-24 Session 5 Update

- Phase 5 追加完成：继续清理 `Application/Modules` 虚拟分组中的未使用模块。
- 已删除 `ConnectScan`、`DirCurrent`、`Encryption`、`SerialPortPro` 的源文件、头文件和空目录。
- 已删除旧空目录 `RadioFreq`、`SplitType`；当前射频/分体刀具逻辑由 `Beep/sscRFID.c` 承接。
- 已同步 EIDE、Keil、当前 builder 参数，并通过 `unify_builder --rebuild`。
