# 手柄插拔与报警逻辑发现

## 当前链路
- `handlescan.c` 负责 A/B 短接检测、去抖、EEPROM 认证、识别缓存更新。
- `sscKEYBH.c` 把 `SCREENKey_PLUG_A/B`、`SCREENKey_UNPLUG_A/B` 分发到 `PlugORunPLUGActive()`。
- `sscKEYBH.c` 把 `SCREENKey_HANDLE_A/B` 分发到 `HandleSwitchActive()`。
- `Pubinterface.c` 的 `MemoryMsgA/B` 是通道记忆，`WorkMessage` 是当前工作快照。
- `external_comm_task.c` 当前只上传一个全局 `WorkMessage.alarm_value`。

## 冲突点
- `PlugORunPLUGActive()` 当前插入 A/B 会直接选中该通道。
- `PlugORunPLUGActive()` 当前拔出当前通道会自动切到另一在线通道。
- `HandleSwitchActive()` 从 B 切 A 的分支把 `channel_work` 写成 `2`，应为 `CHANNEL_A`。
- `sscBEEP.c` 当前报警蜂鸣是持续翻转，不能表达 3 秒限时提示。

## 产品软件接手文档发现

- 主控真实 UART 参数以 `Src\usart.c` 为准：UART1/2/5/7/8/10 为 115200 8N1，其中 UART7 当前是 `UART_MODE_TX`；UART3/4/6 为 115200 8N2；压力软串口为 9600 8N1。
- `User\board\board.h` 的 `BOARD_UART_LIST` 仍有预留波特率和历史注释，不能单独作为串口参数依据。
- 软 UART 压力链路中，SIM_UART_1 RX=PE4 固定写 `pumpMessageA`，SIM_UART_2 RX=PE6 固定写 `pumpMessageB`。
- 压力主控白名单为 `0x00/0x0E/0x0C/0x08/0x09`，压力工程文档列 `0x0F/0x0E/0x0C/0x08/0x09`，需要实测霍尔高低电平定义。
- 外控心跳是动态长度：手柄在线才追加原始类型，泵在线才追加泵类型、速度和 11 字节压力扩展。
- 外控链路短超时 1000ms 停输出、长超时 5000ms 释放外控；源码注释仍写 30s，文档按代码值记录。
- `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE=1`，逻辑 A/B 泵最后输出到物理 UART 口时会互换。
- A/B 泵任务周期均为 25ms；A 队列深度 5、B 队列深度 2，双泵一致性仍需覆盖高频不同命令。

## 手柄 EEPROM 认证和识别细节

- `handlescan.c` 每 10ms 扫描 A/B；短接低电平是插入候选，插入去抖 50ms，认证前等待 200ms，快速重试 200ms，最终失败后慢速自恢复重试 1000ms，连续第 3 次失败进入失败保持态。
- A 通道使用 I2C2，B 通道使用 I2C3；普通系统报警期间暂停识别，但手柄 EEPROM 校验报警允许另一通道继续识别。
- AT24CS32 页大小 32B，前 30B 有效，`page[30]` 高字节、`page[31]` 低字节保存前 30B 的 16 位累加和。
- 手柄认证读取 Page1 原始页并校验页尾，再读 SN 16B（地址 `0x0800`），再读 Page2~Page8 共 224B；认证输入为 `SN + Page2~8` 共 240B。
- 认证对同一输入计算 4 组 CRC16：初值均 `0xFFFF`，多项式 `0x1021/0x8005/0x3D65/0xA097`，结果按大端拼为 8B 与 Page1 `[0..7]` 比较。
- Page2 `[0..1]` 支持 `6B 01..06` 映射 TMBB/TMBA/EMBA/EMBB/PXBA/PXBB；Page3 `[0..1]` 支持 `7C 01..06` 映射 MXYTM/MXYTP/PXYTM/PXYTP/JMB/MXYTM16。
- Page3 直径、长度、角度为大端 0.1 单位；Page4 默认流量、速度上下限、默认速度和速度报警阈值为小端，默认流量 `/10` 后最大钳到 70。
- Page4 默认方向字段当前被 `Handlescan_ParseInitialDirection()` 忽略，固定返回 `ZZDIR`。
- 信息页读取失败、Page2 未知手柄型号、Page3 未知刀具型号目前以 `alarm_value=0` 进入重试/失败态，可能出现“不上线但无持续报警”。
- `Pubinterface.c` 的 `SpeedActive()` B 通道分支疑似使用 A 通道速度上下限；`FreqActive()` B 通道减频判断疑似方向错误，需要实机或单元测试确认。

## 阶段 9：非手柄模块深挖发现

- 主控 `sscDrive.c` 当前下发电机命令为 11 字节 `AA MODE FREQ MOTOR SPD_H SPD_L RUN_TYPE CUR_H CUR_L BB AA`，没有下发 CRC；驱动回包才是 12 字节并由主控按 `Common_Crc16(&dat[i],10)` 校验，CRC 低字节在前。
- `sscDrive.c` 中往复频率只钳到 100，参考驱动 `mcuart.c` 接收后执行 `R_DATA[2] * 2`，所以主控文档不能写成“主控提前翻倍”。
- `motoruartdata.c` 回包扫描循环为 `i < (rlen - 11)`，但解析时读取 `i+11`。当 DMA 缓冲刚好只有一帧 12 字节时，循环不会进入，存在边界测试必要。
- `sscFOOT.c` 运行解析任务 10ms，底层校准接收任务 3ms，行为任务 25ms。实际掉线阈值是 `footDisconnect_times > 100`，约 1s；头文件 `FOOT_OFFLINE_THRESHOLD=20` 和注释中的 1.6s 都不是当前运行分支的真实值。
- 脚踏 JTB 运行帧分支检查 `i+13 < rlen` 后读取到 `i+15`，长度保护偏弱，噪声/短包注入时应重点验证。
- 脚踏行程速度计算多处使用 `(H-L)` 或 `(H-M)` 作除数，当前未看到运行前统一保护 `H==L`、`H==M` 的防御分支，需做定标异常测试。
- A/B 泵任务周期均为 25ms；A 队列深度 5、B 队列深度 2。二者都以 `pumpMessageA/B` 为唯一输出数据源，队列只更新类型和目标速度。
- 压力闭环阈值单位为 g，重量为 0.1g。默认硬停点是 `threshold_g * 10 * 3 / 2`；阈值以下不降速，阈值到硬停点线性降速，硬停点以上本周期输出 0 但不清 `run_flag`。
- 软 UART 压力解析是 9600 8N1；SIM_UART_1/PE4 使用 TIM11，SIM_UART_2/PE6 使用 TIM13，两路拥有独立接收状态，可同时采样；`overlap_drop_count` 仅保留为兼容诊断字段，正常应保持 0。
- CS1237 帧固定 21 字节，`AA 55 02 01 0C Seq Raw(4LE) WeightX10(4LE) ThresholdG(2LE) DeviceCode CRC16(2LE) 55 AA`，CRC16/MODBUS 覆盖 `frame[2]` 起 15 字节。
- 主控压力白名单为 `0x00/0x0E/0x0C/0x08/0x09`，压力工程 `protocol.h` 定义为 `0x0F/0x0E/0x0C/0x08/0x09`，这是跨工程协议不一致。
- 外控协议帧 Length 是整帧长度，CRC16 大端写入，覆盖 `TranCode, Length, FunCode, AreaCode, InforCode, InforArea`；接收 FIFO 会保留粘包/半包，错误候选帧只跳过帧头首字节继续重同步。
- 外控短超时 1000ms 只停输出并保留授权，长超时 5000ms 释放外控；源码注释仍写 30s。
- 步进泵板 `USER\logic.c` 支持 `BB AA` 绕过 CRC；`USER\motor.h` 实际 `PWM_FRE=8000` 但注释写 10K。
- 压力传感器工程 200ms 主循环同时处理标定协议和周期上报，标定写 Flash 时需要验证是否打断或穿插上报帧。

## 2026-06-17 电机驱动风险专项发现

- 无刷/有刷工程根路径：`D:\EH_main\soft\Temp_save\GE2433_WSYS_2026_4_24\GE2433_WSYS_2026_4_24\0`。
- 无刷/有刷工程关键目录初步定位：`UserCode\logic.c`、`UserCode\mcuart.c`、`UserCode\interrupt.c`、`UserCode\fangbo_nohall.c`、`MotorCode\mcfoc.c`、`MotorCode\mcbrush.c`、`MotorCode\hallfoc.c`、`YouShua\YSlogic.c`。
- 步进工程根路径：`D:\EH_main\reference\shima_waixie\small_2026_3_31\small_2026_3_31`。
- 步进工程关键文件初步定位：`USER\logic.c`、`USER\motor.c`、`USER\motor.h`、`Core\Src\usart.c`、`Core\Src\tim.c`。
- 无刷私有协议在 `UserCode\mcuart.c:443-446` 接受 `seruart->RxCRC == 0xAABB` 的旁路条件；停止命令在 `UserCode\mcuart.c:532-544` 只设置 `Brake_Sign=1`、`Brake_Kind=1`、必要时 `Motor_Stop_StartFlag=1`，没有在收包处直接关闭 PWM。
- 无刷停止完成依赖 `UserCode\interrupt.c:207-245` 的 1ms 刹车递减和 `UserCode\fangbo.c:1104-1113` 的 `VBusNowPWM < MCPara[45]` 判定；如果递减步进、下限或阈值配置异常，停止命令会停留在刹车链路而不是立即硬停。
- B 通道 1ms 刹车分支 `UserCode\interrupt.c:260-297` 存在明显通道变量风险：`Brake_Kind==0` 时写 `App.FB.Status = HS_STOP`，不是 `App.FB2.Status`，可能导致 B 通道刹车状态不能按预期退出。
- 无刷 A/B 共用 `App.Logic.VBusNowPWM`，`UserCode\fangbo.c:394-438` 和 `453-496` 分别按 A/B PI 更新同一个 PWM 变量，停止、开环和 B 通道刹车都复用该变量；双通道或快速切换时存在状态互相覆盖风险。
- 无刷 B 通道电压环初始化 `UserCode\fangbo.c:1715-1718` 疑似写错结构体：无 Hall 时把 `MCPara2[27/28]` 写到 `mcApp_VoltageP_PIParam`，不是 `mcApp_VoltageP_PIParam2`，会影响 B 通道刹车/闭环下限判断。
- 有刷停止命令在 `UserCode\mcuart.c:601-609` 只把 `App2.Log.u32SetSpd` 装载为 0 并清 `BreakSta`；实际停止由 `YouShua\YSlogic.c:20-25/120-125` 清 `Start`，再由 `YouShua\YSstatemachine.c:174-240` 进入 `BreakSta` 刹车计时，达到 `YSPara1/2[59]` 后才关 PWM 和 Buck。
- 有刷刹车 PWM 下降在 `YouShua\YSmcctl.c:219-256` 执行，通道 1/2 同样共用 `App2.Log.VbusNowPWM` 和 `TMR2` 的 Buck 输出；如果 `YSPara[56]` 被写成 0 或刹车计时标志异常，停止会依赖通讯/错误保护兜底。
- 步进串口接收在 `Core\Src\stm32l4xx_it.c:327-342` 通过 UART3 IDLE + DMA 写 `SerUart3.RxLen`，但解析函数 `USER\logic.c:35-73` 不检查 `RxLen >= 6`，固定读取 `R_DATA[0..5]`，短包或粘包残留有机会参与速度解析。
- 步进协议在 `USER\logic.c:49-57` 接受 `R_DATA[4]==0xBB && R_DATA[5]==0xAA` 的 CRC 旁路，速度直接由 `R_DATA[2]*256+R_DATA[3]` 写入 `App.Log.Set_Speed`，没有最大速度钳位；异常字节可形成很大的目标速度。
- 步进启动条件 `USER\logic.c:20-27` 只判断电压、`abs(Set_Speed)>MCPara[35]` 和无错误；默认 `MCPara[35]=0`，意味着任意非 0 速度都能触发启动。
- 步进初始化 `USER\logic.c:185-201` 使用 `MCPara[25]` 计算 `RampUpTime`、`PoweUpUnit` 和 `RampUpInc`；默认 `MCPara[25]=3000`，但若被写成 0 或很小，会出现除 0 或启动相位增量过大，符合“启动速度异常快”的软件风险。
- 步进 PWM 中断 `USER\bujing.c:47-82` 在开环 Stage0 每周期执行 `RampUpTemp += RampUpInc`、`Step_Temp += RampUpTemp`；Stage1 在 `USER\bujing.c:83-109` 直接按 `Step_Unit` 推进相位。`Step_Unit/RampUpInc` 均由 `App.Log.Set_Speed` 直接换算，缺少速度上限防线。
- 无刷/有刷失停建议断点：`UserCode\mcuart.c:453-545` 确认停止帧是否被接收并解析为 `Set_Spd=0`；`UserCode\fangbo.c:1104-1138` 确认 `Brake_StopPwmOK` 是否置位；`UserCode\interrupt.c:207-297` 观察 `VBusNowPWM`、`qOutMin`、`MCPara[44]`、`MCPara[45]`、`MCPara2[44]`、`MCPara2[45]`；`YouShua\YSstatemachine.c:174-240` 观察有刷 `BreakSta`、`StopTimCnt`、`AllowRun` 和 `Status`。
- 步进异常快启动建议断点：`USER\logic.c:43-57` 观察 `RxLen`、`R_DATA[0..5]`、`RxCRC/CalcCRC`、`App.Log.Set_Speed`；`USER\logic.c:185-201` 观察 `MCPara[25]`、`RampUpTime`、`Step_Unit`、`RampUpInc`；`USER\bujing.c:47-82` 观察启动前 50 个 PWM 周期内 `RampUpTemp` 和 `Step_Temp` 是否跳变过快。

## 2026-06-27 主控软件调试报告发现

- 已存在 `docs/product-software-handoff.md`，该文档覆盖产品架构、协议、测试流程和静态风险；新报告应从“调试动作”角度组织，减少重复。
- 新报告建议路径为 `docs/software-debug-report.md`，命名直接，后续测试人员能从文件名判断用途。
- 当前报告应覆盖三类入口：配置入口、断点入口、故障现象入口。
- 配置入口需要覆盖 EIDE/Keil 源文件清单、UART 参数、任务周期、手柄 EEPROM 解析、泵 A/B 物理口互换、压力白名单、外控超时、报警码和屏幕/外控命令字段。
- 断点入口需要覆盖启动链路、`Userparser_Init()`、软任务调度、`WorkMessage` 装载、手柄扫描、脚踏解析、电机命令、泵输出、压力软 UART、外控收发、EEPROM 读写、报警处理。
- 故障现象入口需要覆盖上电无响应、手柄不上线、A/B 选中异常、工作中拔插异常、电机不启停、泵不转或方向错、压力不更新、脚踏无效、外控无授权、心跳异常、参数不保存和构建清单回退旧模块。
- `Src\main.c` 启动顺序为 HAL/CubeMX 初始化、Tracealyzer、`Hardware_PostInit()`、`App_Bootstrap_Init()`、`MX_FREERTOS_Init()`、`vTaskStartScheduler()`。
- `User\Application\Src\userparser.c` 是业务初始化入口，顺序为 GPIO、EEPROM、UART1/2/3/4/5/6/7、屏幕启动页、电机紧急停止、公共状态初始化、RFID、各软任务初始化。
- `Src\app_task.c` 中旧业务回调通过同一个静态互斥锁串行执行，说明多个软任务不代表业务代码真正并发运行。
- `User\Application\include\Pubinterface.h` 中当前报警码存在复用：`WORK_ALARM_SPEED_THRESHOLD` 和 `WORK_ALARM_MOTOR_DRIVER_BOARD` 都是 11，调试时必须结合来源函数区分。
- `User\Application\Pubinterface\Pubinterface.c` 中通道切换首选断点为 `Pubinterface_LoadChannelMemory()`，插拔首选断点为 `PlugORunPLUGActive()`，控制权首选断点为 `ControlArbitration_TryEnter()`/`ControlArbitration_IsBusyByOther()`。
- `User\Application\Handle\handlescan.c` 扫描周期 10ms，插入去抖 50ms，拔出去抖 500ms，认证前等待 200ms，普通认证最多 3 次快速重试，RFID 刀具头等待约 2 秒。
- `User\Application\Beep\sscDrive.c` 电机输出任务 50ms 根据 `WorkMessage.runflag_work` 下发 UART1 启停帧，`User\Application\MotorUartData\motoruartdata.c` 3ms 解析驱动回包并更新 `driver_speed_feedback`、`driver_current_x100` 和报警。
- `User\Application\include\pump.h` 当前 `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE=0`，逻辑 A 泵走 UART5，逻辑 B 泵走 UART7。
- `User\Application\include\pump_pressure_control.h` 当前压力闭环总开关默认开启；压力停泵后锁止到下一次控制源启动沿，不再按 3000ms 自动恢复；A 压力源固定 `pumpMessageA`/SIM_UART_1/PE4，B 压力源固定 `pumpMessageB`/SIM_UART_2/PE6。
- A/B 泵任务周期均为 25ms；排空计数使用 `PUMP_TIMING_DRAINAGE_TICKS=400`，两路实际排空时长均约 10 秒。
- `docs/software-debug-report.md` 已生成，报告采用“配置速查、观察变量、模块断点、故障现象、构建清单”的顺序，适合后续测试时直接定位问题。
- 本轮没有修改业务源码；报告生成后检查了异常字符、占位符、行尾空格、关键断点函数和 `git diff --check`。

## 2026-06-27 主控原理版调试报告重写发现

- 旧调试报告偏向“在哪里打断点”，对为什么要经过公共状态、为什么不能输入直连输出解释不足；新版报告应把原理放在断点前面。
- 当前主控输入链路统一收敛到 `WorkMessage`、`MemoryMsgA/B`、`ChannelrecognizeMessageA/B`、`pumpMessageA/B`、`ControlSignalMessage`，这是理解脚踏、手柄、屏幕和外控互斥的主线。
- `AppTaskRuntimeGate()` 让旧业务回调通过同一个互斥锁串行执行，因此 FreeRTOS 多任务不是任意业务代码并发运行；调试周期卡顿时要同时看任务周期和互斥等待。
- 当前外控短超时为 2 秒停输出，长超时为 10 秒释放外控，应以 `EXTERNAL_COMM_LINK_STOP_OUTPUT_TIMEOUT_MS` 和 `EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS` 为准。
- 当前主控压力设备码白名单为 `0x00/0x08/0x09`，旧文档和压力工程历史资料出现过其它设备码组合，属于跨工程协议一致性风险。
- `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE` 改的是逻辑泵到物理 UART 的映射，`UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE` 改的是屏幕显示和触控映射，二者不能混用。
- A/B 泵任务周期均为 25ms，两路共用 `PUMP_TIMING_DRAINAGE_TICKS=400`，排空时长统一为约 10 秒。
- `WORK_ALARM_SPEED_THRESHOLD` 和 `WORK_ALARM_MOTOR_DRIVER_BOARD` 当前都为 11，报警定位时必须结合来源函数和上游链路区分。

## 2026-06-27 产品手册级报告参考文档发现

- Zephyr 文档的结构价值在于按子系统、内核服务、驱动、硬件支持、构建与配置系统分层说明，适合作为本工程“主控层、应用层、外设层、配套工程”的组织参考。参考地址：`https://docs.zephyrproject.org/latest/introduction/index.html`。
- PX4 架构文档把源码模块、消息总线、运行环境和更新频率放在同一页解释；本工程可以对应到“软任务、公共状态、UART/软串口输出、任务周期”。参考地址：`https://docs.px4.io/main/en/concept/architecture`。
- Klipper 的代码总览直接写“整体代码布局、主要代码流、典型动作路径”，并把一次动作从命令入口写到硬件输出；本工程故障排查章节也应按“输入 -> 公共状态 -> 输出 -> 反馈”的路径写。参考地址：`https://www.klipper3d.org/Code_Overview.html`。
- Marlin 配置文档明确说明配置文件和 `#define` 是最权威来源；本工程临时配置章节也应以源码宏为权威，旧文档和注释只作为辅助。参考地址：`https://marlinfw.org/docs/configuration/configuration.html`。
- 本工程主报告不应写成“源码目录说明”，而应按交付手册方式写：先解释设计边界，再解释数据结构，再给动作路径，最后给配置、断点、验证和风险。

## 2026-06-27 产品手册级报告第 15 至 16 轮发现

- AT24CS32 当前按 128 页、每页 32 字节管理，前 30 字节是业务数据，最后 2 字节是前 30 字节累加和；外控写业务页只应提供 30 字节，页和由主控写页函数生成。
- Page1 认证输入不是单页数据，而是 SN 16 字节加 Page2~Page8 共 224 字节；改 Page2~Page8 任一业务页后，如果不更新 Page1 认证结果，下次识别可能返回 CRC 不匹配。
- 外控 EEPROM 读写不直接选择 I2C2/I2C3，而是根据 `WorkMessage.channel_work` 写当前选中通道；无当前通道时读写应失败。
- 外控运行设置和 EEPROM 写页是两条链路：运行设置改 `WorkMessage`、`MemoryMsgA/B` 或 `pumpMessageA/B`，EEPROM 写页改持久化原始资料，二者生效时机不同。
- 当前主控核心业务主要依赖手柄 EEPROM 和公共状态流转；`flash.c` 存在通用驱动，但没有作为当前主控主要参数保存入口使用。
- 主控电机 11 字节下发帧仍依赖驱动工程对 `0xAABB` 兼容尾的接受；驱动侧若只接受真实 CRC，主控电机命令会被拒绝。
- 步进/泵板接收主控 6 字节泵命令，同样支持 `BB AA` 旁路；泵类型不由步进板决定，而由主控业务和压力设备码决定。
- 压力板 21 字节上报帧中的 `DeviceCode` 是主控判断在线和泵类型的重要输入；主控白名单和压力工程合法集合必须成套验证。
- 外控上位机必须按动态心跳解析手柄、泵和压力扩展字段，不能假设固定长度；EEPROM 写入还要配合主控当前通道和二次确认。

## 2026-06-27 产品手册级报告第 17 至 20 轮发现

- 当前源码确认外控短超时为 `EXTERNAL_COMM_LINK_STOP_OUTPUT_TIMEOUT_MS=2000U`，长超时为 `EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS=10000U`，主报告最终按 2 秒停输出、10 秒释放授权写入。
- 当前源码确认 `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE=0U`，主报告将泵物理口互换和屏幕镜像、压力源映射分开说明。
- 当前源码确认注水泵业务速度上限 `PUMP_INJECTWATER_SPEED_MAX=70U`，主报告将其限定为注水泵钳位入口，不扩展成所有泵类型上限。
- 脚踏真实业务文件是 `User\Application\Beep\sscFOOT.c`，掉线判断在 `Foot_ParseDataS()` 中使用 `footDisconnect_times > 100`，头文件旧阈值不能单独作为运行判断依据。
- 压力超过停泵点时建立 `pressure_hold_flag` 锁止；压力下降不自动恢复，必须释放当前请求并再次启动形成新启动沿。
- 报告最终新增了实机记录模板和串口抓包字段，后续现场问题应同时记录公共状态快照和 UART 原始帧，避免只凭 UI 现象判断。

## 2026-06-27 代码段解释增强发现

- 报告中的代码段应当按“入口条件、公共状态写入、提前返回、最终输出动作”四步解释，单独贴源码不足以支撑现场定位。
- `WorkMessage_t`、`ChannelMemoryMessagr_t`、`ChannelrecognizeMessage_t` 和 `pumpMessage_t` 需要明确分组说明，否则容易把当前工作、通道记忆、识别缓存和压力反馈混用。
- `Userparser_Init()` 和 `AppTaskRuntimeGate()` 是理解全局行为的关键代码段：前者决定业务初始化顺序，后者决定旧业务回调串行化。
- 屏幕触控保活、脚踏前置检查、实体键准备通道、电机速度倍率、泵方向互换、压力帧校验、外控解析、UI 去重和蜂鸣消息这些代码段都容易被误解为“直接输出”，因此已补充业务含义和调试读法。
- 新增的代码段解释索引可以作为后续补充文档的模板：每个代码片段都要写明解决的问题、重点变量和常见误解。

## 2026-06-27 现场使用增强发现

- 现场定位 SOP 需要比普通故障排查更“动作化”：每一步必须有断点、正常结果和异常下一步，否则测试时仍会回到凭经验猜测。
- 协议帧逐字节判读比单纯列帧格式更适合抓包现场使用；必须同时说明字段位置、单位、大小端、CRC 覆盖范围和异常含义。
- 报警体系必须区分真实报警、限时提示和蜂鸣阈值提示；否则会把蜂鸣误认为 `WorkMessage.alarm_flag=true`，或把临时外控报警误认为整机真实报警。
- 变量字典需要写“谁写、谁读、正常范围、清零条件、首选断点”，这样 Watch 窗口才能服务定位，而不是只堆变量名。
- 配置 cookbook 按“想实现什么”组织比按文件组织更适合临时修改；每个配置项都要说明影响范围、验证方法、回退点和风险。
- 测试用例表必须同时记录 UI、公共状态和原始帧；只记录 UI 现象不足以判定问题边界。

## 2026-07-01 主控工程接手理解发现

- 当前主控工程顶层包含 `Src`、`User`、`Drivers`、`Middlewares`、`EIDE`、`MDK-ARM`、`build`、`docs` 等目录；主控业务主要落在 `User/Application`，CubeMX/HAL 入口主要在 `Src`。
- 当前已有两份核心文档：`docs/software-debug-report.md` 是主调试和原理报告，约 220KB；`docs/product-software-handoff.md` 是历史接手资料，约 141KB。后续源码和旧文档冲突时，应优先按当前源码和 `software-debug-report.md` 复核。
- 当前工作区不是干净状态：`EIDE/.eide/eide.yml`、`Src/main.c`、`User/Application/Beep/sscDrive.c`、`User/Application/Beep/sscFOOT.c`、`User/Application/Pubinterface/Pubinterface.c`、`User/Application/include/Pubinterface.h` 已有未提交改动；`.omx` 下也有运行状态文件改动和新增日志。
- 未提交源码差异的业务含义：`Src/main.c` 暂时注释掉 `Tracealyzer_RecorderInit()`；`sscDrive.c` 让脚踏模式下电机输出速度和屏幕速度显示使用实时 `WorkMessage.speed_work`；`sscFOOT.c` 新增脚踏按 EEPROM 最小速度起步、压力堵塞停机锁存、松脚后解除锁存等逻辑；`Pubinterface.c/.h` 暴露压力停机锁存查询接口；`EIDE/.eide/eide.yml` 上传器从 STLink 改为 JLink。
- 旧模块回编译风险扫描未命中：`handledata.c`、`param.c`、`warn.c`、`User/Data/data.c`、`UI_Main.c`、`UI_ModelConfiguration.c`、`UI_Password.c` 以及旧 `SysRunData`、`SysSetParam`、`SysModelConfig`、`SysHandleData`、`SysInterface`、`SysFootPedalData`、`SysUIDisplayData` 在指定 EIDE/Keil 构建清单和源码范围内未检出。
- 启动主线为 `Src/main.c`：HAL 和外设初始化后执行 `Hardware_PostInit()`、`App_Bootstrap_Init()`、`MX_FREERTOS_Init()`、`vTaskStartScheduler()`；IWDG 当前保持禁用以匹配参考固件行为。
- 业务初始化主入口为 `User/Application/Src/userparser.c::Userparser_Init()`：先初始化板级 GPIO、EEPROM、UART1/2/3/4/5/6/7、启动页、电机急停、UI 启动页，再初始化 `WorkMessage`、通道识别、通道记忆、泵状态、控制信号，最后启动手柄扫描、脚踏、屏幕键、电机、外控、泵、UI、软串口压力等软任务。
- 软任务门控在 `Src/app_task.c::AppTaskRuntimeGate()`：虽然软任务拆成多个 FreeRTOS 线程，但业务回调进入前仍抢同一个 `sAppTaskRuntimeMutex`，保持旧业务串行访问全局状态的时序假设。
- 核心状态分三层：`ChannelrecognizeMessageA/B` 是 EEPROM/RFID 本次识别缓存；`MemoryMsgA/B` 是 A/B 通道记忆；`WorkMessage` 是当前工作快照。运行中另一路插入只应更新通道记忆，不应抢占当前 `WorkMessage`。
- `Pubinterface_LoadChannelMemory()` 是 A/B 通道记忆装载到当前工作快照的关键入口；它先装载电流、倍率、方向、控制方式、频率、刀具、手柄，再最后写 `channel_work`，用于避免中间状态被其它任务读到。
- `PlugORunPLUGActive()` 是 A/B 插拔事件落地入口：插入时先写在线状态和 `MemoryMsgA/B`，非运行且非普通报警时才自动选中；运行中插入另一路不抢占；非运行状态拔掉当前通道时可回落到另一在线通道；运行中拔掉当前通道停电机、停联动注水泵并等待用户确认。

## 2026-07-01 屏幕偶发点不动软件分析发现

- 屏幕输入链路为 `USART6 DMA` -> `Uart6_DMARecvDataPeek()` -> `ScreenKey_Scan()` -> `ScreenKey_PostLegacyAction()` -> `SendKeyBehMessage(SCREENKey, ...)` -> `KeyBehaviors()` -> `SCREENKeyBehanior()` -> `Pubinterface.c` 业务入口。
- `uart6.c` 没有使用 USART6 IDLE 中断判帧；`USART6_IRQHandler()` 只调用 `HAL_UART_IRQHandler(&huart6)`。当前收包依赖 `Uart6_DMARecvDataPeek()` 每 30ms 轮询 DMA 剩余长度，连续 3 次不变后才认为一包结束，再 `HAL_UART_DMAStop()`、拷贝、清 DMA 缓冲并重启。
- 上述 UART6 设计在连续触控保活、屏幕多帧连发或软任务被 UI 刷新拖住时会延迟出包；如果 DMA 剩余长度持续变化，`ScreenKey_Scan()` 长时间拿不到 `rlen`，现象就是屏幕原始帧可能到了但业务事件没生成。
- `ScreenKey_Scan()` 使用 `uint8_t dat1[16]` 接收单帧，但 `len = dat[i + 2] + 3` 后没有检查 `len <= sizeof(dat1)`，直接 `Common_CopyData(&dat[i], dat1, len)`；`Common_CopyData()` 是裸循环拷贝，没有边界保护。串口噪声、错位帧或异常长度字节可造成栈越界，属于已确认的软件设计缺陷。
- `ScreenKey_Scan()` 解析多帧时 `slen` 初始为 `rlen`，只在命中一帧后 `slen -= len`，没有扣掉前面跳过的非帧头字节；存在噪声前缀时，剩余长度判断可能偏大，增加越界或误解析风险。
- `SendKeyBehMessage()` 对普通屏幕键使用 `Kernel_QueueSend(..., 0)` 非阻塞入队，并忽略返回值；`KeyBehivQueue` 深度为 20，队列满时普通屏幕键静默丢失。插拔事件有 60ms 等待，但普通屏幕按钮没有。
- 屏幕按键被 `KeyBehaviors()` 取出后，还会先走 `ControlArbitration_ShouldBlockLocalKey()`；若外控 owner 未释放或本地其它 owner 正忙，很多屏幕控制键会被 `continue` 静默丢弃，没有 UI 错误提示。
- owner 释放依赖 `ControlArbitration_IsMotorBusy()==false`，即 `WorkMessage.runflag_work==false` 且 `WorkMessage.driver_speed_feedback <= CONTROL_ARBITRATION_MOTOR_STOP_SPEED_THRESHOLD`。如果驱动反馈回包漏解析或 `driver_speed_feedback` 卡在非零，`s_control_owner` 会保持占用，屏幕模式切换、手柄切换、启动类按键会像“点不动”。
- `motoruartdata.c` 回包扫描循环为 `for (i = 0; i < (rlen - 11); i++)`，对刚好 12 字节单帧的边界存在漏解析风险，可能导致 `driver_speed_feedback` 不及时归零，间接导致 owner 不释放。
- `UIDP` 显示队列深度也是 20，`UIDISPLAYBehavior()` 每 10ms 最多处理 12 条消息；每条消息会调用多个 `LCD_Show_*()`，底层 `Uart6_SendPacket()` 使用阻塞式 `HAL_UART_Transmit(..., timeout=3)`。所有软任务业务回调又被 `AppTaskRuntimeGate()` 串行互斥，因此大量 UI 刷新会拖住 `ScreenKey_Scan()` 和 `KeyBehaviors()` 的进入时间。
- `SendUIDSMessage()` 队列满时也只是不更新去重缓存，没有全局错误计数；现场无法直接知道 UI 队列是否已经拥堵。屏幕“有蜂鸣但业务不动”时应同时看按键队列和 UI 队列水位。

## 2026-07-10 业务功能代码简化审查发现

- 本轮基线固定为 `a39c3b1`（2026-07-09 18:29，`修正泵压力保护与分体手柄状态保持`），开始审查时工作区无未提交改动。
- 该基线已完成 AC5/EIDE 全量构建：0 error、22 个原有 warning；后续结构方案必须以不增加 warning、不改变现有行为为最低门槛。
- 用户已明确保留当前每任务独立线程栈版本，本轮不再把启动链、调度器、Kernel 包装层或任务模型列入业务重构范围。
- 历史审查资料可作为检索索引，但所有结论必须重新以 `a39c3b1` 当前源码、EIDE 实际构建清单和 map 符号为准。
- 用户确认历史记录里的重构尝试曾出现“能够编译，但烧录后 bug 很多”的情况。因此本轮需额外审查失败改动是否破坏 A/B 映射、状态清零、调用顺序、任务阻塞、队列语义和安全门禁；不能只用静态构建作为重构验收。
- 当前业务维护负担主要集中在 `Pubinterface.c`、`handlescan.c`、`external_comm_task.c`、`sscFOOT.c`、`soft_uart.c`、`sscUIDP.c`、`sscRFID.c`。前三个文件远大于其它模块，并分别混合多类状态所有权或数据流。
- EIDE 实际构建清单 `EIDE/build/MainCtrlF413MXOs/builder.params` 已确认包含上述业务模块；当前 EIDE 清单和 Keil 工程都仍注册相同的核心业务源文件，后续拆文件必须同步两套工程。
- 当前 `Src/app_task.c` 仍是每软任务独立静态线程栈，并通过 `AppTaskRuntimeGate()` 全局互斥保持旧业务串行语义。该调度结构属于明确保留边界，不纳入本轮优化。
- 工作区旁仍存在 `codex/simplify-scheduler-startup` 实验 worktree，回退前内容保存在 `stash@{0}`；它们都不是稳定版组成，审查只用于反查失败模式，绝不应用到 `R1`。
- 按物理行统计，最大业务文件分别为：`Pubinterface.c` 4712 行、`handlescan.c` 3431 行、`external_comm_task.c` 3017 行、`sscFOOT.c` 1525 行、`soft_uart.c` 1419 行、`sscUIDP.c` 1078 行、`sscRFID.c` 951 行、`handlekey.c` 627 行。
- `Userparser_Init()` 的真实启动顺序已重新核对：先完成硬件和公共状态初始化，再依次注册看门狗、LED、蜂鸣、按键行为、手柄扫描、脚踏、屏幕键、电机控制、手柄键、电机回包、外控、RFID、A/B 泵、UI 和压力软串口任务。结构重构不得调整此顺序。
- 基线 22 个 warning 中有多项来自业务文件：`sscKEYBH.c` 不可达语句、`sscPUMPA.c` 两处 LCD 函数隐式声明、`sscFOOT.c` 未使用变量、`sscRFID.c` 多个未使用数组/函数、`handlescan.c` 未使用函数。它们适合作为低风险清理候选，但必须逐项用 map 和调用检索证明无运行路径。
- map 已明确裁掉 `HandleKey_GetKeyValue()`、`HandleKey_Scan0SSC()`、`HandleKey_Scan1SSC()`、`RfidHandle()`、`Rfid_DiscardQueuedMessagesForChannel()`；这类“链接器已不进入镜像”的旧代码比直接改状态机更适合作为第一轮试点。
- map 基线为 RO 106692B、RW 130904B、ROM 107196B。后续每阶段应比较 map 和应用任务栈符号，但资源减少不能替代实机行为验证。
- 公共状态名在业务文件中的引用高度集中：`Pubinterface.c` 784 处、`external_comm_task.c` 180 处、`sscFOOT.c` 145 处、`handlescan.c` 87 处、`handlekey.c` 51 处。`Pubinterface` 当前既定义状态又包含大量业务动作，而外控、脚踏和手柄仍直接读写这些结构，说明“状态所有权不唯一”比目录层级更值得优先治理。
- `Pubinterface.c` 的函数分布可分成显示刷新、公共接头门禁、泵压力报警、通道记忆、报警生命周期、控制权仲裁和各输入来源动作七组；一次性拆完整文件会同时碰多个安全链，风险不可接受。
- `external_comm_task.c` 已经能看到相对清楚的三段函数群：业务命令、心跳组装、RX FIFO/收包；它比 `Pubinterface.c` 更适合在行为快照建立后按数据流逐段拆分。
- 当前系统的业务初始化注释仍混有旧版本说明、估算时间和占位文本；注释优化应随具体函数重构完成，不能全工程批量改写，否则容易掩盖真实代码差异。
- 排除 `==` 条件后，公共状态字段的直接赋值仍分散在多个模块：`Pubinterface.c` 258 处，`sscFOOT.c` 和 `external_comm_task.c` 各 63 处，`handlescan.c` 12 处，`sscPUMPA/B.c` 各 10 处，`handlekey.c` 8 处，`motoruartdata.c` 7 处。稳定版实际采用“多写入者共享结构”，重构时不能简单把结构搬文件，必须先明确每类状态的唯一动作入口。
- 17 个实际启动的周期执行入口重新核对为：电机回包 3ms；手柄扫描、脚踏解析、UI、外控 10ms；A/B 泵和脚踏行为 25ms；屏幕键、按键行为、手柄键 30ms；电机控制 50ms；蜂鸣、RFID、压力软串口 100ms；LED 200ms；看门狗 300ms。`PedalRecvTask_Init()`虽存在，但没有在 `Userparser_Init()` 中启动。
- `Pubinterface.c` 文件开头同时定义通道识别、当前工作、A/B 记忆、A/B 泵、控制权、脚踏优先锁、报警生命周期和显示缓存。真正需要拆的是这些“状态所有权”，而不是简单按函数数量平均切文件。
- map 进一步证明 `datahand.c`、`drivectrl_adapter.c`、`footpedal_ui_adapter.c`、`led.c` 的当前默认构建逻辑全部被裁剪；`pedal.c` 的任务、解析和定标函数也在当前镜像中被裁剪。它们属于“默认构建无运行路径”的候选，但仍需检查工厂定标等编译变体，不能只凭一个 map 直接删除。
- `motor.c` 不是整文件死代码：旧运行/位置接口被裁剪，但 `Motor_ErrorEmergencyStop_Ctrl()` 和 `BrushlessMotor_Stop()`仍在上电启动链使用。该文件只能局部清理，不能整文件删除。
- `pump.c` 也不是整文件死代码：B 路旧直接输出被裁剪，但 A 路 `Pump_SetSpeed_A()`仍被电机回包错误处理链调用，并形成 `sscPUMPA.c` 之外的第二个泵输出入口。必须先统一安全停泵入口，不能机械删除。
- 业务目录命名已失真：`Beep` 目录同时承载电机、脚踏、泵、RFID 和 UI。第一轮不建议移动文件；否则同时改目录、include 和两套工程清单，难以定位实机回归来源。
- 大文件的标准三项函数块注释覆盖并不完整，尤其 `sscFOOT.c`、`soft_uart.c` 的函数数量和函数头注释数量差距明显。注释改进应只跟随本阶段触碰的函数，解释状态切换、硬件动作和保留时序，不做独立的大范围注释提交。
- 已纠正一条历史审查结论：`motoruartdata.c` 的循环条件在 `rlen == 12` 时为 `i < 1`，会执行一次并解析完整 12 字节帧；“刚好 12 字节必然漏解析”不成立。该例证明旧报告只能作线索，最终结论必须重新算边界并读当前代码。
- `handlescan.c` 当前 A/B 两路都存在已确认的空指针风险：`Handlescan_FindToolTypeConfig()`返回 `NULL` 后，异常处理和 `return` 被注释，随后仍解引用 `tool_type_cfg->mapped_handle_type`。这属于独立缺陷，不应夹在 A/B 状态机合并中顺手修复。
- `sscRFID.c` 的 `s_request_attempts_left--`被注释，源码注释仍声称尝试次数会耗尽。当前实际行为是无有效帧时持续每周期发命令；这是行为与注释不一致，必须先确认现场预期再单独处理。
- `sscFOOT.c` 右踏板某个释放分支检查并清除 `jtL_control_flag`，另一个释放分支使用 `jtR_control_flag`；同时 A 注水泵启动使用当前默认流量，B 注水泵沿用现有 `speed_work`。这两处均为疑似 A/B 复制差异，暂不认定为 bug，需用输入状态和实机行为确认。
- `ScreenKey_Scan()` 当前已确认存在输入边界缺陷：局部缓冲 `dat1[16]`，帧长 `len = dat[i+2] + 3`，但拷贝前只检查 `slen < len`，没有检查 `len <= 16`；异常长度帧可造成栈越界。`slen`也没有扣除帧头前跳过的噪声字节。该缺陷应独立修复和注入测试，不能与按键映射重构混在同一烧录版本。
- 稳定版 A/B 不变量重新核对：泵业务任务 A/B 均为 25ms；逻辑 A 驱动 UART5、逻辑 B 驱动 UART7；压力 PE4/SIM_UART_1 写 `pumpMessageA`，PE6/SIM_UART_2 写 `pumpMessageB`；屏幕泵镜像宏默认 0。任一重构改变其中一项，都可能出现“左泵接入却显示右泵上线”。
- `a39c3b1` 与实验提交 `c35c404` 的差异横跨 42 个文件，包含脚踏、泵、压力、RFID、外控、屏幕和硬件串口，并非可单独评估的纯架构重构。它不能作为新方案模板。
- `Pubinterface.h` 当前被 18 个业务/外设源文件直接包含，并暴露约 60 个函数声明和 179 个宏；它同时承担状态结构、事件码、报警码、控制权和业务动作接口，是编译依赖和认知耦合的主要入口。后续只能按已确定所有权逐步拆窄头文件，不能一次重命名全部接口。
- 历史文档引用的 `tools/check_*.py` 在当前提交中已不存在，因此旧记录中的“脚本检查通过”无法直接复现。每阶段必须重新建立针对本阶段的行为对照；临时脚本按工程规则执行后删除，长期测试资产需先获用户确认。
- 当前两份长文档仍保留旧压力映射 `PE4→B、PE6→A`，与 `a39c3b1` 源码的 `PE4→A、PE6→B` 相反。文档不能作为当前行为真值，最终每个模块稳定后需要同步修正对应章节。
- `refs/stash` 已证实左/右泵显示反向补丁只把 `UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE` 从 0 改为 1；它同时交换显示 VP 和触摸映射，却不交换压力输入和驱动串口，属于必须废弃的局部映射补丁。
- 旧大范围提交 `16a0945` 同时改任务模型、脚踏、泵和外控，随后连续出现外控断线停机与控制权仲裁修复。能证明的是“多模块同时改导致回归不可隔离”，不能证明当前独立线程模型本身错误；当前线程栈和全局运行互斥必须冻结。
- 电机任务内部真实顺序为 `MOTORRUN()`→速度阈值蜂鸣检查→控制权刷新，停止态仍每 50ms 重复发送停止帧。`MOTORRUN()`使用静态帧状态，非法方向/通道可能沿用上周期字段；纯重构若改成本地全清零会改变隐式行为。
- `sscKEYBH.c` 的来源类型属于协议语义：不同来源可能复用相同 key 数值，不能直接合成只按键值索引的表。队列消费必须保持每 30ms 排空、逐条仲裁、被阻塞时 `continue` 而不是退出整个循环。
- 当前看门狗初始化和刷新均被注释，300ms 看门狗任务实际为空动作但仍占独立任务栈。启用或删除都会改变产品行为，应单独立项，不纳入业务代码简化。
- 所有权矩阵最终确认：`WorkMessage` 至少由 Pubinterface、脚踏、外控、handlescan、handlekey、电机回包和按键行为写；`MemoryMsgA/B`由五个模块写；`pumpMessageA/B`把硬件遥测、控制请求、闭环输出和排空计时混在同一可写结构。`s_control_owner`反而是当前边界最清楚的状态，只由 `ControlArbitration_*()`写。
- 当前识别链存在必须保留的 30ms 异步过渡态：handlescan 先写 `ChannelrecognizeMessageA/B`并投递插拔事件，按键行为任务后续再写 `MemoryMsgA/B`并按条件装载 `WorkMessage`；事件未消费前外控心跳会从识别暂存兜底。不能把该链改成同步直接写入。
- `Pubinterface.h`没有 include guard，同时混放业务枚举、报警码、UI 区域、六类结构体、可写全局变量和五十余个动作接口。正确顺序是先收敛写入者和业务动作，再拆窄头文件；反过来只会移动耦合。
- 推荐采用“行为锁定的纵向小步重构”：首个代码阶段只清理 `handlekey.c` 中 map 已裁掉的旧链并补注释；验证流程可靠后，再分三小步合并 handlescan A/B，之后逐模块处理泵、脚踏、电机、屏幕/UI、Pubinterface 所有权和外控。每个烧录版本只处理一个模块。
- 不推荐两种路线：仅补注释和删死代码无法解决共享状态；一次性按领域重写虽结构最整齐，但会重复历史上“编译通过、实机大量回归”的失败模式。
- 交付前于 2026-07-10 16:16 重新执行 AC5/EIDE 全量构建：125 个 C 文件、1 个汇编文件，0 error、22 个基线 warning；RO 106692B、RW 130904B、ROM 107196B，与回退基线一致。

## 2026-07-12 脚踏模块初始审查

- `sscFOOT.c` 约 86KB，真正复杂点集中在 874 行左右的 `FootControlTask()`；其前置辅助函数已经覆盖速度换算、控制模式、安全锁存、注水泵启停和释放去抖。
- 启动链只调用 `SscFootControlTask_Init()`；`PedalRecvTask_Init()` 仍是未接入的旧入口，不能在本轮重构时误启用。
- `FootControlTask()` 同时负责离线处理、控制方式切换、单踏板、双段踏板、双踏板左右切换、泵联动、报警清理和 owner 申请，职责混在一个大函数内。
- 当前队列长度为 5，非阻塞收发；行为任务保持 25ms 周期。两项属于稳定行为边界。
- 可复用的安全门禁已经存在，下一步应先把三类踏板各自整理成清晰的单周期处理函数，再考虑合并共同动作；不能先改状态模型或公共接口。
- 当前 map 中 `FootControlTask()` 为 2118B，`Foot_ParseDataS()` 为 1144B；前者是本轮结构简化的主要目标。
- 脚踏电机启动门禁链在单踏板、双段踏板和双踏板左右侧重复出现：控制模式检查、外控检查、公共接头检查、FOOT owner 申请、速度换算、运行标志和注水泵联动。
- 双踏板左右侧代码基本镜像，但分别使用 `switchhandle_counts`、`switchhandle_countss` 和不同控制标志；这些现有状态字段必须原样保留，不能为了合并代码擅自改成共享计数。
- 当前存在需要保持而非顺手修正的分支差异，例如 JTB 轻踩时 A 泵取默认注水流量、B 泵取当前工作速度，以及左右踏板释放时检查对侧状态。若无单独缺陷复现，本轮只做结构提取。
- 最低风险边界是先在 `sscFOOT.c` 内把连接处理、三类踏板周期处理、共用启动/停止动作提取成静态函数；暂不拆文件，也暂不同时重写 `Foot_ParseDataS()` 的 UART4 收包状态机。

## 2026-07-14 当前工程行为保持优化审查发现

- 当前审查基线为 `040cd17`，开始审查时工作区干净；历史 `a39c3b1` 文件规模、warning 和职责统计只能作为对照，不能直接作为当前结论。
- `handlescan.c`、泵行为内核、`FootControlTask()`、`PUMPActive()` 和 `ScreenKey_Scan()` 已完成多轮实机验证后提交，本轮不会把这些已治理模块再次列为首选重构对象。
- 用户唯一硬约束是业务功能不变，因此本轮把“减少代码行数”置于“保持隐式调用顺序和共享状态语义”之后；不能为追求对称而改动历史 A/B 差异。
- 规划技能会追加三份审查记录，但生产业务源码、构建清单和协议文件保持只读。
- 当前业务大文件仍集中在：`handlescan.c` 3139 行、`external_comm_task.c` 3135 行、`Pubinterface.c` 2941 行、`sscFOOT.c` 1836 行、`soft_uart.c` 1556 行、`sscUIDP.c` 1090 行；文件行数本身不能作为再次重写已验证模块的理由。
- `Pubinterface.h` 当前约 462 行，被 21 个项目业务文件直接包含，并且没有 include guard 或 `#pragma once`；补充头文件保护是可独立完成、零业务语义变化的安全修正。
- 五个受保护公共对象的直接字段写入仍分散：`Pubinterface.c` 133 处、`pump_control.c` 82 处、`sscFOOT.c` 65 处、`external_comm_task.c` 63 处、`handle_control.c` 34 处，其余模块还有少量写入。后续只能集中完整业务动作，不能搬移或改造这些数据结构。
- `ControlSigleMessage_t / ControlSigleMssage` 当前只发现定义与声明，未发现业务引用；必须再由 map 证明未进入镜像，才可作为独立死代码清理项。
- `external_comm_task.c` 仍是下一轮最值得梳理的数据流模块；它同时承担命令执行、状态回传、收包缓存和连接生命周期，且仍有大量公共状态直接写入。
- map 显示 `ControlSigleMssage` 虽无源码调用，仍作为 8 字节全局数据进入镜像；它不是“已被链接器裁掉”的代码，只能在确认没有调试器/外部符号依赖后单独删除。
- `drivectrl_adapter.c` 的三个函数、`footpedal_ui_adapter.c` 的函数与 4 字节状态全部被链接器裁掉；全工程也没有业务调用。两者属于真实的无效兼容层，连同 `userparser.c` 的旧 `drivectrl.h` include 和头文件内未实现的旧任务声明，可作为第一批纯清理对象。
- 当前 map 还裁掉 `RfidHandle()`、`ChannelMessageInit()`、`ScreenKey_LegacyEventTake()`、旧电机运行接口及 `pedal.c` 旧接收任务。后者仍被旧脚踏定标 UI 源码引用，删除前必须先确认该 UI 构建变体已永久停用，不能只凭默认 map 直接整批删除。
- 当前标准函数块注释在已重构的 `handlescan.c`、`Pubinterface.c`、`screenkey.c`、`control_arbitration.c` 等文件覆盖较好；剩余注释治理应跟随具体模块改动，不应为覆盖率单独批量重写。
- `sscRFID.c` 的 `CUTTERSCANTaskHandle` 是 `static` 私有对象，未创建、未启动，只被 `(void)` 人工引用；map 证实它仍占 4240B RW。删除该对象和无效引用不改变任何运行路径，是当前收益最高且风险最低的单点优化。
- `datahand.c/.h` 的函数、130B 状态和1B数据均被 map 裁掉，且 `handlescan.c/handlekey.c` 仅包含头文件、不使用其符号；该乱码旧状态模块与两个空适配层应一并作为“构建清单瘦身”阶段处理。
- `Pubinterface.c` 当前约89个函数；`control_arbitration` 与 `work_alarm` 已具备清晰所有权，不是冗余层，应保留。仍可把仅本文件使用的4个接口改为 `static`，并让业务调用者直接包含真实模块头文件，取消 `Pubinterface.h` 对仲裁、报警、泵和手柄接口的隐式转发。
- `Pubinterface.c` 最值得继续处理的是约195行的 `SpeedActive()` 和约157行的 `ControlTypeActive()`；只在原文件内按真实动作拆静态函数，并集中 A/B 通道指针选择，不拆 `ScreenKey_CanUse()` 这种可直接按键值查阅的线性规则。
- `handlescan.c` 仍有一处直接写报警字段，可等价改走现有 `WorkAlarm_Set/Clear`，完成报警唯一写入口；不能全面为五个公共结构生成逐字段包装函数。
- 外控当前真实数据流已经清楚，`external_comm_task.c` 不宜拆文件。优先把下行命令、失败原因、心跳字段和 AreaCode 数值集中到现有 `external_comm_protocol.h`，再修正 UART5/RxFifoFull 等误导性私有名称，最后用一个私有通道选择函数减少 `MemoryMsgA/B` 重复。
- `sscUIDP.c` 的 A/B 泵显示和手柄图标仍存在镜像重复，可用文件内通道显示配置合并；必须逐项锁定 VP、图片号和 LCD 调用顺序，泵镜像宏保持不变。
- `MOTORRUN()` 仍约140行，可按目标速度、低速补偿、方向命令、通道/闭环、显示/组帧拆为文件内静态阶段；必须保持50ms调用顺序、EMBD输出层取反和脚踏速度仅量化显示不量化电机目标。
- `screenkey.c`、`sscKEYBH.c`、LED、蜂鸣状态机及当前 RFID 主流程的职责边界已经足够清楚，不建议继续表驱动或拆文件，否则只会增加跳转层。
- 独立缺陷包括 RFID 重试次数不递减、电机回包粘包跳13字节、电机故障停机不对称、看门狗实际未启用、蜂鸣队列失败后同类报警不重试。它们都涉及行为，应单独复现和修复，不能混入“业务不变”的重构提交。
- 2026-07-14 14:18 使用 EIDE/AC5 对当前 HEAD 全量重建：127个C、1个汇编，0 error、0 warning；RO 106040B、RW 130664B、ROM 106524B。

## 2026-07-14 手柄物理 A/B 接口交换发现

- 当前逻辑 A 固定绑定 PD1 短接检测、I2C2 EEPROM、PE12 实体键、RFID A 硬件通道和电机驱动物理通道 1；逻辑 B 对应 PD0、I2C3、PE13、RFID B 和电机驱动物理通道 2。
- 不能交换 `s_a_binding/s_b_binding` 或 `CHANNEL_A/B`，否则识别缓存、记忆、报警、UI、RFID结果和事件会一起串位。
- 正确方式是保留逻辑 A/B，只在硬件访问边界把逻辑通道映射到物理通道。
- `handlescan.c` 的 EEPROM认证、字节读取和整页读取都直接按逻辑通道选择 I2C2/I2C3；需改为按物理通道选择。
- `external_comm_task.c` 也独立按 `WorkMessage.channel_work` 选择 EEPROM 总线，必须复用同一物理映射，否则扫描和上位机读写会访问不同手柄。
- RFID 缓存必须继续按逻辑通道存放，仅 R200-K8/UART3/UART9 的硬件选择按物理通道映射。
- `sscDrive.c` 的 `motor_type` 0x01/0x03 表示物理通道1，0x02/0x04 表示物理通道2；整根线束交换时必须同步映射。
- 统一配置集中在 `board_profile.h`，`board_resource_map.h` 只负责把逻辑短接脚和运行键指向对应物理资源；业务模块只在硬件访问前调用同一个通道映射函数。
- 开关 `1U` 时逻辑A使用PD0、PE13、I2C3、原物理B RFID和电机通道，逻辑B使用PD1、PE12、I2C2、原物理A RFID和电机通道；开关 `0U` 恢复原接线。
