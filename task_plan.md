# STM32 工程重构任务计划

## 目标

- 将 `F413EXOsSSCH_RTOSV1.5` 收敛为 `Hardware / Driver / Kernel / Application` 四层。
- 保证 `CubeMX` 重新生成代码后，只影响硬件层，不打散上层软件架构。
- 在不改变现有业务行为前提下，先建立稳定入口、稳定接口、稳定 EIDE 工程视图。

## 已完成阶段

| 阶段 | 状态 | 内容 |
| --- | --- | --- |
| 1 | completed | 完成工程目录、启动路径、关键模块、运行时模型阅读 |
| 2 | completed | 输出首轮架构问题判断与四层分层方案 |
| 3 | completed | 建立 `hw_bootstrap`、`app_bootstrap`、`kernel_entry`、`kernel_scheduler`、`kernel_osal` 骨架 |
| 4 | completed | 完成 `main.c`、`freertos.c`、`usart.c` 的桥接式接入 |
| 5 | completed | 完成 `.eide/eide.yml` 四层视图重组与 include 根路径补充 |
| 6 | completed | 清理 `User` 层对 `board.h`、`app_task.h`、原生 FreeRTOS 头与 API 的直接依赖 |
| 7 | completed | 将 `screen`、`drivectrl`、`handlescan` 等业务任务切到 `kernel_scheduler`/`kernel_osal` 接口 |
| 8 | completed | 建立 `board_profile`、`board_resource_map`、`bsp_gpio`、`bsp_uart`、`bsp_i2c_bus` 稳定壳层 |
| 9 | in_progress | 持续收敛应用层、驱动层中的 GPIO/UART/Board 初始化直连 |
| 10 | in_progress | `uart1~uart7` 已切到 `bsp_uart` 壳层，后续再决定是否继续物理合并 |
| 11 | in_progress | `OneWire/iic` 已切到 `bsp_gpio` 资源映射；`i2c` 后续继续收口层次 |
| 12 | completed | 已将应用层的 `Board_GPIOConfiguration()` 调用收口进 `hw_bootstrap` |

## 下一步顺序

1. 收敛 `User/Peripheral/uart/uart1~uart7.c`
2. 收敛 `User/Peripheral/bus/iic.c`、`OneWireI.c`、`OneWireII.c`、`i2c.c`
3. 将 `userparser.c` 中的 `Board_GPIOConfiguration()` 改为通过 `hw_bootstrap` 间接调用
4. 每完成一批迁移，补一次 EIDE 编译验证

## 约束

- `handlescan.c` 编码敏感。只做最小修改。保持中文注释清晰可读。
- 仓库已有用户历史改动。不得覆盖无关内容。
- 当前验证以增量编译为主。后续需补完整链接闭环记录。
## Additional External Module Track

- Phase X1: review `D:\EH_main\soft\soft_SSC` module set and classify layer ownership
  - status: completed
- Phase X2: map overlap with current project modules and identify symbol/conflict risk
  - status: completed
- Phase X3: prepare merge strategy for selective integration into current architecture
  - status: pending
