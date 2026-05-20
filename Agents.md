# Agents.md

## 编码与中文注释要求

1. 后续新增或修改代码时，必须要添加中文注释，且优先保证文件编码稳定，避免把中文注释写成乱码、问号或异常字符。
2. 修改现有源码时，不要通过容易破坏编码的终端拼接方式批量重写整文件；优先使用对原文件影响最小的方式进行局部修改。
3. 对已经存在中文内容的源码文件，新增注释时要保持同一文件内中文显示正常，不能因为一次修改把原有中文注释破坏。
4. 如果发现某个文件历史上存在编码问题，后续修改应尽量只改必要代码和必要注释，避免无关的大范围改写。
5. 以后生成的中文代码注释应当准确、直接、工程化，优先解释变量含义、状态机阶段、关键分支和硬件/协议动作，不写空泛注释。
6. 以后新增或重写的业务代码，注释密度要接近逐行说明：每个宏、结构体字段、关键赋值、状态切换、硬件访问、协议字段处理、异常返回都要写清楚“这行在做什么、为什么这样做、影响哪个运行状态”。旧文件只做局部修改时，也要给本次新增代码补足同等粒度注释。

## handlescan 相关说明

1. `handlescan.c` 中的 `HandlescanA_Fun_SSC()` 和 `HandlescanB_Fun_SSC()` 需要保持中文注释清晰可读。
2. 如果后续继续调整 A/B 通道扫描逻辑，新增注释时应保持与现有注释风格一致，避免出现乱码问题。

## EIDE 配置同步要求

1. 本工程主要在 EIDE 中开发和构建，调整源文件注册关系时，不能只验证根目录 `build/MainCtrlF413MXOs/builder.params`。
2. 删除、替换或新增 `.c` 文件后，必须同步检查 `EIDE/.eide/eide.yml`、`EIDE/build/MainCtrlF413MXOs/builder.params`、根目录 `build/MainCtrlF413MXOs/builder.params`、`MDK-ARM/MainCtrlF413MXOs.uvprojx`，必要时也清理 `MDK-ARM/MainCtrlF413MXOs.uvoptx` 里的旧文件记录和 watch 项。
3. EIDE 插件实际构建时优先使用 `EIDE/build/MainCtrlF413MXOs/builder.params`，因此最终验证必须覆盖这一路径，避免旧模块被 EIDE 生成清单重新带回编译。
4. 清理旧状态模块时，尤其要确认 `handledata.c`、`param.c`、`warn.c`、`User/Data/data.c`、`UI_Main.c`、`UI_ModelConfiguration.c`、`UI_Password.c` 不再出现在 EIDE 和 Keil 的源文件清单中。
5. 如果 EIDE 插件缓存或工程视图临时把上述旧模块重新加入构建，这些旧模块源文件也只能保留为空兼容文件，不能恢复任何 `SysRunData`、`SysSetParam`、`SysModelConfig`、`SysHandleData`、`SysInterface`、`SysFootPedalData`、`SysUIDisplayData` 读写。
