# 手柄插拔与报警逻辑进度

## 2026-05-26
- 已读取工程内 `Agents.md`。
- 已定位屏幕按键、扫描、通道切换、蜂鸣和上位机报警链路。
- 开始执行最终方案，先加入回归检查再改业务代码。
- 已加入 `tools/check_handle_channel_logic.py`，首次运行失败在 `HandleSwitchActive()` 的 B 切 A 分支，符合预期红灯。
- 已完成 `Pubinterface_LoadChannelMemory()` 公共装载入口，屏幕/上位机/插拔自动选择都复用 MemoryMsg 通道记忆。
- 已调整插入策略：非运行状态最后插入且校验通过的通道自动选中；运行中另一路插入只更新对应 MemoryMsg。
- 已调整拔出策略：非当前通道拔出不影响当前通道；当前通道拔出不自动切到另一通道，等待用户手动确认。
- 已新增 A/B/AB 手柄 EEPROM 校验报警码，并让上位机 `ExternalCommHost.html` 区分显示。
- 已新增 3 秒限时蜂鸣、屏幕临时提示和上位机临时报警帧，用于运行中另一路坏手柄提示。
- 已运行 `python tools\check_handle_channel_logic.py`，8 项检查通过。
- 已运行 EIDE `unify_builder.exe --rebuild --no-color`，构建通过并生成 `build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex`。
- 根据现场测试补充修正：非工作状态下当前选中通道拔出时，如果另一通道仍在线，则自动回落选中另一通道；运行中当前通道拔出仍保持不自动切换。
- 已再次运行 `python tools\check_handle_channel_logic.py`，8 项检查通过。
- 已再次运行 EIDE `unify_builder.exe --rebuild --no-color`，构建通过并生成新的 `build/MainCtrlF413MXOs/MainCtrlF413MXOs.hex`。

## 2026-05-27
- 已新增 `docs/product-software-handoff.md`，以主控工程为主线整理 5 个工程的软件架构、接口协议、测试调试流程和逐文件静态风险。
- 已运行 `python tools\check_handle_channel_logic.py`，8 项手柄通道逻辑检查通过。
- 已运行文档占位符/乱码扫描，未发现常见占位标记或 Unicode 替换符；已用 UTF-8 读取前 20 行确认中文可读。
- 创建 `docs` 目录时旧版 PowerShell 不支持 `New-Item -LiteralPath`，已改用 `New-Item -Path` 完成目录创建。
- 继续扩展 `docs/product-software-handoff.md`：补入主控硬件引脚/DMA 表、公共结构体字段表、事件码表、电机命令/回包/错误映射、脚踏协议、泵方向与压力闭环、外控命令矩阵和动态心跳、EEPROM 页映射、压力软串口解析细节、主控文件接手索引。
- 已修正文档中 `Pubinterface.c` 的历史错误路径，统一为 `User\Application\Pubinterface\Pubinterface.c`。
- 已在 `task_plan.md` 和 `findings.md` 追加产品软件接手文档任务记录，和旧手柄插拔任务区分。
- 已再次运行 `python tools\check_handle_channel_logic.py`，8 项检查通过。
- 已再次运行文档占位符、乱码、旧错误路径扫描，未发现命中；已用 UTF-8 读取文档开头确认中文可读。
- 当前文档约 1147 行，覆盖主控启动、调度、硬件、公共状态、事件码、报警、电机、脚踏、泵、压力、外控、EEPROM、四个配套工程、测试调试和逐文件风险。
- 根据用户反馈继续深挖文档细节：已复核 `handlescan.c`、`at24cs32_crc_verify.c`、`at24cs32.c`、`Pubinterface.c` 中手柄 EEPROM 认证、页校验、状态机、Page2/3/4 解析、A/B 通道装载和速度/频率调节逻辑。
- 已在 `docs/product-software-handoff.md` 的 4.3 节新增“手柄 EEPROM 认证与识别算法”，补入状态机表、认证伪代码、页校验公式、CRC 多项式、Page2/Page3 映射、Page4 字段解析、识别结果流向图和调试步骤。
- 已在第 7.3 节新增手柄 EEPROM 算法专项测试用例，覆盖 Page1 页校验、Page2~8 认证数据、SN 读取、Page2/Page3 未知型号、Page4 速度钳位、默认方向和运行中另一路坏手柄。
- 已在第 8 章和第 12 章补充静态风险：B 通道速度上下限疑似串用 A 通道、B 通道减频判断疑似错误、Page2/Page3 未知型号可能不持续报警、Page4 默认方向字段当前不生效。
- 已运行 `python tools\check_handle_channel_logic.py`，8 项手柄通道逻辑检查通过。
- 已运行文档占位符、乱码和旧路径扫描，未发现异常命中；已用 UTF-8 读取文档开头确认中文可读。
- 根据用户反馈继续把手柄之外模块补到同等粒度：已复核 `sscDrive.c`、`motoruartdata.c`、`sscFOOT.c`、`pedal.c`、`sscPUMPA.c`、`sscPUMPB.c`、`pump_pressure_control.c`、`soft_uart.c`、`external_comm_protocol.c`、`external_comm_task.c`，以及无刷/有刷驱动、步进驱动、压力传感器和外部通信上位机工程的关键文件。
- 已在 `docs/product-software-handoff.md` 第 4 章新增或深化：脚踏解析/定标/行程控制算法、电机命令生成和驱动回包处理算法、泵输出与压力闭环算法、软 UART 压力帧解析算法、外控帧解析/授权/保活/安全动作。
- 已修正文档中电机主控下发帧描述：当前主控下发 11 字节 `AA ... BB AA`，驱动接收端以 `RxCRC == 0xAABB` 兼容；驱动回包才是 12 字节 CRC 小端。
- 已深化第 5 章配套工程：补充驱动板私有协议接收、驱动回包字段、步进/泵板 CRC 旁路与开环算法、压力传感器主循环/标定表/霍尔设备码算法。
- 已新增第 7.4 脚踏专项测试，并扩展电机、泵压力、外控测试用例，覆盖定标异常、短包噪声、回包 12B 边界、压力线性降速/硬停/恢复、软 UART 同相碰撞、外控半包/粘包/急停/EEPROM 长度。
- 已扩展第 8 章和第 12 章风险：脚踏除 0、JTB 长度保护、JTD 右踏在线判断、电机命令依赖 `0xAABB` 旁路、驱动回包边界、A/B 排空周期差异、压力设备码差异等。
- 已运行 `python tools\check_handle_channel_logic.py`，8 项手柄通道逻辑检查通过。
- 已运行文档占位符、乱码、旧错误路径扫描，未发现命中；已用 UTF-8 读取文档开头确认中文可读。
- 当前文档约 1643 行，阶段 9 已完成。
- 根据用户反馈修订文档表述：已将主文档中的读者称呼类字样替换为“接手阅读顺序”“接手提示”“接手时先看什么”等正式产品文档口径，并同步调整 `task_plan.md` 目标描述。
- 已在手柄 EEPROM 认证与识别章节补充 3 张 Mermaid 图：手柄识别函数调用流程图、手柄识别数据流向图、手柄运行按键调用与控制数据流图。
- 已运行读者称呼类字样扫描，未发现命中。
- 根据用户进一步反馈，已把“手柄同等深度”的要求扩展到其它复杂模块，而不是只补手柄相关内容。
- 已在 `docs/product-software-handoff.md` 补充电机函数调用流程图和电机数据流向图，覆盖 `SscDriveMotorTask_Init()`、`MOTORRUNTask()`、`MOTORRUN()`、`MotorStart()/MotorStops()`、`MotorUartData_Init()`、`MOTORUARTTaskFunc()`、`BrushlessMotorUartData_ReceiveData()` 和驱动回包报警链路。
- 已补充泵函数调用流程图和泵/压力数据流向图，覆盖 A/B 泵 25ms/100ms 周期任务、队列兼容入口、`pumpMessageA/B`、压力闭环、硬停不清 `run_flag`、UART5/UART7 物理口互换和外控心跳字段来源。
- 已补充软 UART 压力函数调用流程图和压力帧数据流向图，覆盖 `SimUartTask_Init()`、EXTI/TIM11 位采样、单 active receiver、`overlap_drop_count`、环形缓冲、CS1237 帧校验和 `pumpMessageA/B` 压力字段写入。
- 已补充外控函数调用流程图和外控数据流向图，覆盖 UART2 DMA、RX FIFO 半包/粘包重同步、协议解析、授权/owner、业务分发、链路看门狗、动态心跳和 EEPROM 读写数据走向。
- 已运行读者称呼类字样扫描，未发现命中。
- 已运行文档占位符、乱码标记、历史路径和常见错字扫描，未发现命中。
- 已运行 `python tools\check_handle_channel_logic.py`，8 项手柄通道逻辑检查通过。
- 已扫描主文档图表标题，当前复杂模块图覆盖手柄、脚踏、电机、泵/压力、软 UART 压力和外控链路。

## 2026-06-17 电机驱动风险专项分析
- 已开始分析无刷/有刷驱动工程和步进工程，目标是定位“停止命令后仍转”和“步进泵启动速度偶发异常快”的软件设计风险。
- 已确认两个目标路径存在；本轮先只读源码和记录风险，不改电机工程源码。
- 已完成无刷/有刷停止链路第一轮证据记录：停止命令进入后走刹车状态机，不是收包处直接硬停；已记录 CRC 旁路、刹车阈值依赖和 B 通道状态变量风险。
- 已补充有刷停止链路证据：串口停止只让 `Start=0/u32SetSpd=0`，实际关 PWM 依赖有刷状态机刹车计时和 Buck PWM 递减；同时记录了 A/B 共用 Buck PWM 变量的交叉影响风险。
- 已完成步进启动速度链路第一轮证据记录：UART3 IDLE DMA 长度未参与解析、`BB AA` 旁路、速度未钳位、`MCPara[25]` 影响启动斜坡，均已写入 `findings.md`。
