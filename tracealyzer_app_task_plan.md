# Tracealyzer AppTask 可视化执行记录

## 2026-04-24

- 已在 `app_task.c/app_task.h` 为每个软任务补充 Tracealyzer 生命周期事件缓存，覆盖 `CREATE`、`START`、`STOP`、`ONESHOT_STOP`、`BEGIN`、`END`。
- 已把 `Kernel_TaskCreate(task, func)` 改为宏封装，自动把 `func` 函数名传入 `Kernel_TaskCreateNamed()`，现有业务软任务无需逐个手写名称即可在 Tracealyzer 中显示可读任务名。
- 已在 `kernel_scheduler.c/h` 新增 `Kernel_QueueCreate/Send/Receive` 封装，队列创建时写入 Tracealyzer 对象名，收发时输出 `KernelQueue` 用户事件，包含队列地址、等待 tick 和返回值。
- 已迁移 `sscBEEP/sscFOOT/sscKEYBH/sscPUMPA/sscPUMPB/sscRFID/sscUIDP` 的业务队列创建、发送、接收调用；应用层剩余直接 `xQueueCreate/Send/Receive` 搜索结果仅保留 Kernel 封装层。
- 当前未改 `soft_uart.c` 的静态队列和 ISR 收发路径；该路径包含 `xQueueCreateStatic` 和中断上下文发送，后续若需要观测模拟串口字节流，应单独增加 ISR 安全封装。
- 已执行 `unify_builder --rebuild`，构建通过，生成 `build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex` 与 `.s19`。
