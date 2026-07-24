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

## 2026-07-15 外控EEPROM批量协议发现

- 当前主控只真正执行0x05业务单页读、0x07业务单页写、0x08导航单页读和0x0A导航单页写；0x06/0x09整区读取明确返回不支持。
- 当前上位机批量读写只是循环构造单页命令，每页间隔80ms；循环结束即显示完成，没有等待每页上传或ACK。
- 上位机已禁止业务页写入口，但主控仍接受0x07，因此安全边界目前只在上位机，不能抵御其它外部设备直接构帧写业务页。
- 主控单帧最大150字节、信息区最大134字节，导航全区3510字节不能塞进一个帧；批量协议必须使用短请求加逐页响应，批量写需要分片或逐页确认。
- 业务页读取映射Page2/3/4/5/6/8/9/11；布局说明明确Page7/Page10当前保留。旧上位机误用0x08读取7/10时，主控会按导航序号读成实际Page18/Page21，必须改为跳过保留页。
- 批量读采用0x09：AreaCode为起始实际页12~128，InforArea首字节为结束实际页；批量写新增0x0C，后续30字节为所选范围共用模板。
- 主控批量开始时锁定当前通道对应的I2C2/I2C3，每个10ms任务周期只处理一页，避免117页循环长期占用全局任务互斥锁，也避免中途切通道跨写另一颗EEPROM。
- 批量写逐页返回原0x05成功ACK，批量读逐页返回原0x01页面数据；最后统一返回`[批量功能码, 起始页, 结束页]`完成ACK，任一页失败立即停止。
- 导航单页写和批量写均新增主控外控owner门禁；业务0x07在分发后固定返回`[0x07, 0x05]`且不调用任何写页函数。
- 上位机批量按钮已改为协议级命令并等待最终ACK；导航回包期间只累计页面，整批结束后统一重绘，避免117次全量DOM更新。
- 外控任务和其它业务任务共用全局运行互斥，单页EEPROM读写与阻塞UART发送会占用数毫秒；批量必须在电机和A/B泵停止时启动，并在运行状态改变时于下一页前中止。
- 批量启动失败不能继续回显功能码：0x0C十进制等于实际Page12，会让上位机误判失败页；最终约定启动阶段失败对象固定为0xFF，逐页失败仍回显真实页号。
- 上位机原批量状态机允许重复启动先清空结果、单页命令插入、活动批量中清空界面和手工HEX推进进度；这些路径均已增加互斥或隔离。
- 页面回包必须严格按照业务页2/3/4/5/6/8/9/11后接导航Page12..128的顺序结算；重复、延迟、范围外或乱序帧不得写入当前结果区。

## 2026-07-15 导航批量读取实机失败证据

- 实机弹窗原始帧为 `D7 CA F8 F1 01 00 12 DD FF 06 0C 04 70 BA BF C6 BC C4`，其中对象 `0x0C` 是十进制实际 Page12，原因 `0x04` 是 EEPROM 底层设备读取/页校验失败。
- 当前批量读在任一页失败时立即清空批量状态，Page12 未初始化、页和异常或底层读取失败都会让后续 Page13..Page128 完全不再读取。
- 当前主控每10ms最多上传一页，页数据帧和100ms心跳可能落在同一个任务周期连续阻塞发送；实机截图同时记录 Web Serial `Framing error`。
- 上位机对可恢复串口读取错误显示“串口读异常，保持连接”，直接覆盖顶部连接徽标，因此即使物理端口仍保持打开也会产生连接状态反复跳动的观感。
- 新增实机截图显示压力原始值被拉到约1.19亿、最终重量约13.9万克、泵速约5.7万；这些值与正常面板当前值不一致，是单帧异常写入曲线的特征。
- `applyTelemetry()` 当前只判断 `decoded.telemetry` 是否存在，没有判断 `decoded.valid`；解析器即使判定 CRC 或帧尾失败，仍会生成 telemetry，随后更新压力面板、趋势曲线、泵速曲线和导出日志。这是异常压力尖峰的直接软件入口。
- EEPROM、ACK和软件版本处理入口已经检查 `valid`；压力尖峰修复只需补齐心跳实时数据入口门禁，不需要修改压力板工程或主控压力采集算法。
- 最终主控按30ms处理一个导航页，逐页结果和最终ACK分周期发送；若心跳已到期但本周期发送了批量帧，心跳顺延到下一空闲周期，不会丢失心跳。
- 最终上位机把业务页和导航页读取失败统一结算为失败页并继续等待后续结果；最终ACK到达后，有失败页显示“部分页失败”，不再弹出整批中止错误。
- 批量读适合“记录坏页后继续并最终报告部分失败”；批量写会改变 EEPROM，必须继续保持任一页失败即停止。

## 2026-07-15 EEPROM批量进度显示发现

- 上位机已有`eepromTiming`记录开始、结束和耗时，但批量运行期间不会展示`settledPages`，使用者无法判断当前是否仍在处理页面。
- `settleEepromBatchPage()`只有在收到期望顺序的成功或失败页回包后才推进`nextPageIndex`，是进度条唯一可靠的数据源；不能用30ms任务周期推算假进度。
- 批量页面全部结算后仍需要等待主控最终ACK，进度达到100%时必须显示“正在确认完成”，不能立即显示完成。
- 批量读期间延迟整表渲染是防止117页连续DOM重绘卡顿的既有优化，进度条只能更新少量文本、ARIA属性和单个宽度样式，不能恢复整表逐页刷新。
- 最终组件在批量结束后保留完成、部分失败、超时或断线状态；使用者点击“清空本区”时才同时清除时间统计和进度终态。

## 2026-07-15 确定型进度条实机反馈

- 实机显示为0后瞬间满格，说明回包解析虽然逐页结算，但浏览器没有在同一批同步DOM更新之间完成中间绘制；继续调整百分比计算不能解决视觉问题。
- 不应为了让进度条平滑而延迟协议解析或伪造定时百分比，这会让界面显示脱离主控真实状态并增加串口积压风险。
- 更合适的表达是“不定进度工作指示+实时耗时+真实回包摘要”：旋转指示负责证明界面仍在工作，页计数只作为结果摘要，不再暗示线性速度。
- 最终活动卡只用`transform: rotate()`执行持续动画，减少主线程布局压力；250ms计时刷新不重复改写ARIA状态文本，避免辅助设备持续朗读同一句提示。
## 2026-07-15 上位机四工作台界面发现

- 当前HTML由顶部状态栏、左侧控制区、中央实时/曲线/EEPROM区和右侧日志/解析/配置区组成，所有模块垂直堆叠导致整页过长。
- 串口、外控权限、EEPROM批量状态、日志和趋势数据都保存在同一个`appState`，适合在单HTML内切换视图，不适合拆成会重建运行时的多个页面。
- 现有控件主要通过固定ID和`data-action`绑定，移动DOM父容器不会改变业务事件；必须避免复制控件造成重复ID或重复监听。
- Canvas在隐藏工作台内无法可靠取得最终尺寸，返回实时监控时需要在下一绘制帧调用现有`drawTrend()`和`drawPressureTrend()`。
- 推荐四个工作台为实时监控、设备控制、EEPROM维护和通信诊断；连接、外控状态、当前通道、运行/报警和急停保持全局可见。
- 当前三列宽度拖拽只服务整页布局，改造后应由各工作台固定网格替代，避免保留无效分隔器和旧宽度状态。
- 四工作台最终仍复用原DOM和同一个`appState`，因此切换时Web Serial连接、外控保活、日志、趋势缓存和EEPROM批量状态不会重建。
- EEPROM后台状态会同步到顶层工作台徽标；不使用百分比进度条，改为活动状态、已返回页数、最近页面、失败页数和终态摘要。
- 1920×1080与1366×768实测均无整页滚动；1024像素宽度按响应式规则降级为纵向页面滚动，避免桌面网格在小窗口中挤坏控件。
- 顶部连接、急停、清空和导出动作固定可见；导出收纳为菜单，避免多个低频按钮长期占用顶栏宽度。
- 工作台按钮支持鼠标点击以及左右方向键、Home、End键切换；返回实时监控后下一动画帧重绘趋势Canvas。

## 2026-07-15 四工作台分栏与导出入口反馈

- 四工作台改造时移除了旧三列分隔器，导致不同显示器和使用场景无法人工调整左右空间，需要按工作台恢复独立比例而不是恢复旧的全局三列宽度。
- 导出功能本身没有删除，但被收纳到一个`details`菜单；用户需要直接看到各类导出入口，因此应恢复原按钮可见性并保留原ID和事件绑定。
- 最终使用3条实际分隔线覆盖4个工作台：实时监控和通信诊断各自一条，设备控制与EEPROM维护复用同一DOM分隔线但分别保存比例。
- 分栏比例保存在`external-host-workspace-layout-v1`；方向键每次2%、Shift加方向键每次5%，Home键和双击恢复工作台默认值。
- 1366×768顶栏使用两行操作组：第一行连接/急停/清空，第二行五类导出；高度100px且整页无滚动。1024宽度隐藏分隔条并保持五类导出可见。
- 五个导出按钮已在Edge中分别触发JSONL、通用CSV、压力CSV、手柄CSV和电流CSV真实下载，继续使用原事件处理函数。
## 2026-07-15 上位机工业级视觉升级参考

- Grafana 官方仪表板最佳实践强调按信息层级组织视图、减少认知负担、只对有意义的状态使用颜色，并用阈值把正常与异常直接映射为视觉状态；本轮据此收紧青色泛用，建立成功、警告、危险、选中和遥测五类稳定语义色。参考：https://grafana.com/docs/grafana/latest/visualizations/dashboards/build-dashboards/best-practices/ 与 https://grafana.com/docs/grafana/latest/visualizations/panels-visualizations/configure-thresholds/
- Siemens WinCC Unified 官方资料把清晰、易操作和可扩展作为 HMI 布局目标；本轮保留固定全局状态与四工作台结构，通过更明确的表面层级和状态徽标降低操作人员辨认成本。参考：https://www.siemens.com/en-us/products/simatic-hmi/wincc-unified/
- Ignition Perspective 官方资料强调浏览器 HMI 的响应式布局和可配置网格；本轮继续保留每个工作台独立可调分栏，并在宽窄屏上保持信息优先级和控制入口。参考：https://www.docs.inductiveautomation.com/docs/8.3/ignition-modules/perspective 与 https://docs.inductiveautomation.com/docs/8.1/appendix/components/perspective-components/perspective-display-palette/perspective-dashboard
- 当前界面的主要视觉问题不是功能不足，而是大多数卡片、边框、激活状态和实时数值都使用相近青绿色，导致在线/运行/选中/遥测缺少语义分工；升级应优先调整设计令牌和动态状态属性，不重排已验证的业务 DOM。
- 1366x768 浏览器基线中整页无滚动，四工作台、五个独立导出按钮和分栏结构均完整；本轮必须保持这一布局基线。
- 黑曜石主题中工作台激活态、连接按钮、面板顶线、分隔线、设备激活态和绝大多数数字都使用青色，离线卡片之间也缺少 A/B 识别线；视觉升级应降低青色面积，把状态和通道信息放到边缘标记、状态点和关键值上。
- 现有 DOM 已提供 `data-connected`、`data-level`、`data-online`、`data-active` 等语义属性，可以直接驱动大部分状态色；只需为运行状态、通道选择和数据卡补充少量明确属性，不需要建立新的前端状态副本。
- 最终顶栏改为品牌、全局状态、三项主操作、五类导出的单行网格；Edge实测1366宽时高度54px且`scrollWidth == clientWidth == 1346px`，没有隐藏溢出。
- 设备控制工作台左侧把权限、参数下发作为整行，切换和控制并排；1366×768下`scrollHeight == clientHeight == 621px`，命令区不再需要内部上下滚动。
- 设备控制右侧使用408px实时状态区和203px趋势图填满621px工作区，替代原状态卡片下方空白；四个工作台在1366×768和1920×1080下均无整页滚动。
- 三条分隔线继续有效：控制工作台方向键从31%调整到33%并写入localStorage，Home恢复31%；五类导出按钮仍全部可见，157个DOM ID无重复。
- 模块脚本语法、三套主题切换、五项既有上位机回归和浏览器控制台检查全部通过，未改变协议、Web Serial状态、EEPROM安全边界或导出格式。

## 2026-07-15 前三工作台空间失衡复核

- 用户截图为 1920×约920 的实时监控页：左侧状态卡内容约在620px高度结束，但面板继续延伸到页面底部，形成约250px空白；右侧压力图和总趋势纵向堆叠，面板出现内部滚动条。
- 问题本质不是容器总高度不足，而是实时状态与图表仍分属固定左右栏，左侧不能利用图表区的剩余内容，右侧又承载全部三张曲线。
- 下一步优先复用现有面板和 Canvas，不新增图表或状态副本；通过桌面网格重排让状态摘要占顶部整行、压力和总趋势按剩余高度分配。
- 第2页和第3页必须在真实浏览器中测量空白占比与内部滚动，不能仅根据第1页截图套用同一布局。
- Edge基线测量：1920×920实时监控左状态面板底部空白210px，右图表内容比面板高229px；1366×768时左状态超高44px、右图表超高381px，证实需要把主运行趋势移到左栏空白并让右栏只承载压力视图。
- 设备控制页在1366×768下左命令区无滚动、底部仅6px，右侧状态408px加趋势203px已合理；1920×920左侧因纵向空间增加出现158px余量，但没有内容挤压，不应为了填满而放大操作按钮。
- EEPROM页在1920×920右侧底部空白95px，1366×768则超高90px；适合将右侧改成固定标题/摘要/版本区加弹性数据查看区，由数据查看区吸收不同屏高，而不是继续调整整个面板高度。
- EEPROM左命令区在1920×920余量158px、1366×768仅超高16px，需改为紧凑网格而不能简单增加卡片高度。
- 最终实时监控采用左栏“状态+运行趋势”、右栏全高压力视图：1920×920时左趋势画布274px、压力趋势区639px；1366×768时分别为119px和487px，状态面板无裁切。
- 设备控制六项摘要由两行改为一行后，右侧状态高度从408px降到320px；趋势画布在1920和1366下分别达到245px和93px，图例完整保留。
- EEPROM右栏固定标题/摘要/版本，原始数据与解释区弹性填充；左命令区取消100%子元素造成的伪滚动，桌面模式隐藏2~4px阴影溢出的无意义滚动条。
- 用户指出通道卡左侧3px色条类似“半边括号”；最终改为完整圆角细描边，A/B色、在线背景和右上状态点继续保留。
- Edge最终结构检查在两种分辨率均为4个工作台、3条分隔线、5个可见导出按钮、0个重复ID、0条控制台错误，页面高度与视口高度相等。

## 2026-07-16 重复协议解析区域复核

- 左侧通信日志已经同时显示帧方向、时间、业务摘要和完整 HEX，右侧自动协议解析面板又对每条 TX/RX 帧展开同一内容，形成持续刷新的重复信息。
- 上方手动协议解析仍有独立价值，但当前解析结果借用自动面板显示；直接删除面板会让“解析粘贴帧”失去结果，因此主动解析结果应收进手动解析卡片并按需显示。
- 自动刷新来自发送、接收两条 `showParser()` 调用；日志行点击和手动粘贴属于用户主动行为，可以继续显示详细字段，但不应常驻占用独立面板。
- 诊断右栏当前为三行网格：手动解析、自动解析、配置导入。删除中间行后应改为“手动解析自适应高度 + 配置导入弹性填充”，左侧日志和宽度分隔线保持不动。

## 2026-07-16 掉线统计需求初始判断

- 截图顶部状态徽标的主体文字明显小于连接、急停等操作按钮；需要优先放大状态值和标签，而不是整体缩放页面或增加顶部高度。
- 第二张截图显示页面存在浏览器缩放过小的可能，但用户明确指出的是顶部区域文字，因此本轮仍以 CSS 字号和间距优化为主，并在标准桌面视口验证单行容纳。
- 掉线统计必须基于有效心跳内的状态边沿；页面初始离线不能直接算一次掉线，持续离线也不能每个心跳重复累计。
- 串口整体断开或心跳超时期间无法判断具体设备真实状态，因此应暂停六类设备边沿判断，并单独保留通信状态，避免一次链路故障被误报为全部设备掉线。
- RFID 是否具备独立“模块在线”字段尚需从当前心跳解析确认；如果只有刀具/RFID识别结果，界面必须按真实语义命名，不能伪称检测到了模块物理掉线。
- 有效心跳已经直接提供 A/B 手柄在线位和 A/B 泵在线位，掉线统计不需要主控新增事件；CRC、长度或帧尾错误的心跳已在 `applyTelemetry()` 入口被拒绝。
- RFID 没有独立模块在线位。只有 COMMON_SOCKET、PXBA、PXBB 这类 RFID 手柄识别有效时，心跳才附加来源为 0x01/0x02 的 RFID 刀具结果，因此界面只能准确表述为“RFID识别结果掉线”。
- 设备掉线采用连续 3 个有效心跳确认；首次样本只建立基线，手柄整体离线时暂停同通道 RFID 统计，串口断开和心跳超时只重置观察器，不制造六条设备掉线记录。
- 原上位机估算方式只能统计“心跳是否带RFID结果”，不能对应主控实际发送了多少读取命令；现已改为主控按实际请求、有效应答、未应答和异常帧分别累计，再由A6心跳扩展上报。
- 主控丢包结算边界为：新命令发出前上一条仍未获得有效回包，或请求自然结束仍有待响应命令；运行、切通道、清刀具和停止请求造成的业务主动取消不计丢包。
- A6扩展固定放在心跳末尾：`A6 01 02`后跟A/B两个17字节大端块，上位机只接受恰好结束于心跳尾部的完整扩展，降低压力原始字节误识别为扩展头的风险。
- 实时串口接收入口曾错误调用`applyTelemetry(frame, { recordDeviceStatistics: false })`，导致日志能够解析A6而右侧统计不刷新；该参数应仅用于手工粘贴解析，实时RX必须使用默认统计路径。
- 统计开关关闭时不保留32字节A/B累计器、心跳上限恢复88字节且不发送A6；开启时心跳上限128字节，仍低于协议134字节信息区上限。
- 顶部状态区实际需要约558px才能在16px下容纳四项文字；1366宽度的单行475px会裁掉运行状态，因此最终在中等桌面拆为状态行和操作行，而不是再次缩小字体。

## 2026-07-16 RFID超时掉线与诊断页空间复核

- 截图中的请求/应答统计只占一条矮卡片，A/B数值和百分比字号明显低于同页其它状态信息；配置导入已不再是现场诊断核心功能，可把完整右栏余量交给设备健康统计。
- 主控掉线事件必须来自现有RFID请求超时边界，不能由上位机按心跳缺字段估算，否则无法区分未发请求、业务主动取消和真正等待超时。
- 当前统计结构已经包含请求、有效应答、丢失应答和异常帧；自然耗尽分支位于`SplitType_AutoModeGetData_Task()`的`s_request_attempts_left == 0U`判断，可在该唯一边界增加超时掉线计数和蜂鸣，避免新增计时器。
- 该状态机片段中的`s_request_attempts_left--`当前显示为注释，必须继续核对`Rfid_SendReadCommand()`是否在内部消耗次数；若没有，现有“自然超时”实际上不会到达，不能直接只在结束分支加计数。
- 诊断页右栏当前由手动解析、掉线统计和配置导入三行组成；移除配置导入后应把掉线统计扩展到剩余两行或整列，而不是新增第四个面板。
- `Rfid_SendReadCommand()`只发送并累计请求，不消耗`s_request_attempts_left`；任务末尾唯一递减语句被注释，因此活动请求会保持100ms周期持续轮询，`attempts_left==0`自然结束分支当前不可达。
- 现有`lost_response_count`按相邻读取命令间没有有效回包逐次累计，属于报文丢包而不是设备掉线事件；新掉线统计不能直接复用该数值，必须寻找扫描层基于`presence_sequence`和持续时间的既有离线判定。
- 蜂鸣模块已有`SendAlarmMessageTimed(flag, duration_ms)`非阻塞限时报警接口，比普通按键音更适合明确的RFID掉线提示，但最终是否复用要以现有扫描层报警语义为准。
- 真实RFID刀具掉线判定不在RFID任务的尝试次数分支，而在`handlescan.c`在线监测：200ms一次，连续10次未确认约2秒后才清刀具；该函数已经在确认边沿蜂鸣一次，持续离线会直接返回，因此无需新增蜂鸣器状态机。
- 掉线率分母采用“已完成在线监测次数”：上一轮待确认到下一监测周期仍未确认算一次完成失败，presence序号变化算一次完成成功；分子只在此前在线的刀具达到10次缺失并真正清除时增加一次。
- A6版本1每通道17字节且带通道字节；新字段若继续全部使用32位会超过心跳134字节上限。版本2改为A/B固定顺序的20字节块：请求/有效/丢失各32位，异常16位，监测完成32位，确认掉线16位，整段43字节，最大心跳恰好134字节。
- 配置导入面板删除后，诊断页右栏只保留手动解析和设备健康统计两行；掉线统计可占据剩余整行，并把标题、A/B数值、百分比和事件文字提高到12~14px。

## 2026-07-16 设备控制页视觉复核

- 左侧动作按钮全部使用相近的白色表面、阴影和字号，通道切换、方向、刀具、泵启停、手柄启停和开口定位缺少语义层级，现场操作时需要逐字阅读。
- 实时状态区的四个设备图标约36px且线条颜色接近卡片背景，右上状态点也过小；离线状态主要依赖“未接入”文字，无法快速区分手柄与泵。
- 现有DOM和业务事件已经完整，最小方案应以设备控制工作台限定CSS为主；只允许增加展示类class/data属性，不改控件ID、按钮文字和事件处理函数。
- 该页面属于高频工业控制台，动画机会只保留100~160ms按压反馈和状态色过渡；工作台切换、实时数值、状态卡和图表不应加入入场、脉冲或滚动动画。
- 大屏下第三、第四控制分组被`grid-auto-rows:minmax(32px,1fr)`纵向拉伸，按钮因此变成高而笨重的方块；应改为固定触控高度并从顶部紧凑排列，而不是继续压缩字体。
- 四张设备卡在控制页被覆盖为58px高、图标44x34px，低于同页文字和卡片尺度；应把图标容器扩大到约58x48px，并同时放大设备名称、状态文字和右上状态点。
- 当前状态卡已经具备`data-online`、`data-active`和`data-channel`，无需新增业务状态；可直接用完整描边、状态点、图标颜色和“当前工作”标签构建离线/在线/运行三级语义。
- 用户此前明确反感左侧半括号式粗竖线，因此按钮和状态卡的语义色必须使用完整圆角描边、柔和底色和小图标底座，不能再引入单边粗色条。
- 1366×768真实页面基线无整页滚动：工作区1346×574、左侧命令区417×574、实时状态区921×320；因此可以在不改变主布局高度的前提下，把设备图标从44×34扩大到约56×46。
- 1366宽度下动作按钮当前仅11px字号、36px高；1920截图中的高方块来自第三/第四分组吸收剩余高度，不是触控目标不足。修复应统一为40~42px固定行高和12px字重层级，同时停止纵向拉伸。
- 状态卡现有伪元素已经提供离线/在线状态点，但只有7px；旧活动状态还曾使用红点，容易被误认成报警。工作状态应使用通道色或主色，报警红只保留给真实告警和急停。
- 设备控制页专用样式已将1920×920下动作按钮固定为44px高、12px字体，停止了大屏余量把按钮拉成高方块；1366×768下改为34px紧凑行并保留左侧局部滚动作为安全兜底，页面仍无整页滚动。
- 启动、停止、A/B切换、方向、刀具和开口定位分别读取绿色、红色、A/B通道色、主色、琥珀色和通道色；首轮主题检查发现自定义变量被高优先级基础规则覆盖，补齐选择器优先级后极昼主题派生色已正确生效。
- 控制页设备卡在1920下为82px高，设备图标58×46px，状态点10px；1366下保持约78px卡高且图标规格不缩小，手柄和泵的轮廓均比旧44×34px明显。
- 浏览器在极昼主题、1366×768和1920×920读取到正确的尺寸、溢出和语义色；运行日志无error或warning，未改DOM ID、data-action、事件绑定和Web Serial代码。

## 2026-07-16 上位机工作台切换性能初始边界
- 四个工作台属于高频导航，不适合增加长入场动画；优化重点应是立即反馈、减少同步布局与隐藏Canvas重绘，并把必要过渡限制在160ms以内。
- 本轮只允许修改独立上位机页面及其说明；主控协议、Web Serial、EEPROM安全边界和所有运行状态必须保持不变。
- 当前主控工作区已有RFID/掉线统计等未提交修改；上位机性能修改必须与这些差异隔离，不回退或格式化相邻文件。
- 主要卡顿并非页签显隐本身，而是诊断页隐藏时仍为每条心跳重建最多180行日志、掉线统计DOM，以及隐藏Canvas反复调整像素缓冲区和重绘。
- 最终采用双`requestAnimationFrame`：当前帧更新页签和页面骨架，下一稳定帧只补画目标页图表或日志；快速连点会取消旧页面待执行任务。
- 正式Edge进行每版本80次切换与50条隐藏日志压力测试：无运行异常、无长任务；隐藏日志批次耗时由约379.8ms降为0ms，第一可见帧P95由约33.1ms降为31.9ms。
- 日志改为有界数据缓存、单次HTML解析和父容器事件委托；当前可见180条另用有界Map保存，避免手动模式长期运行后旧行失去解析能力或DOM文本节点无界增长。

## 2026-07-17 外控切换通道无蜂鸣核实

- 上位机 A/B 通道按钮正确下发 `FunCode=0x03`、`AreaCode=0x01/0x02`；主控 `ExternalComm_ApplySwitchSetting()` 完成通道状态装载、实体屏幕刷新和成功 ACK，但没有发送蜂鸣消息。
- 本地实体屏幕触摸链在业务投递前调用 `SendKeyBeepMessage(1U)`，因此本地切换有按键音；外控协议链与该入口相互独立，不是上位机切换失败。
- 普通 `BEEP_MSG_KEY` 会清除蜂鸣任务内部的持续报警和限时报警倒计时，不能直接用于外控通道提示，否则可能打断正在进行的报警声。
- 最小安全方案是增加“仅蜂鸣任务空闲时播放”的独立消息类型，并只在 `WorkMessage.channel_work` 真实发生 A/B 变化且命令成功后投递一次。
- 最终实现中 `BEEP_MSG_KEY_IF_IDLE` 只在内部 `alarm_flag==0` 时装载按键音周期，不写 `alarm_flag` 或 `alarm_limited_ticks`；外控成功 ACK、实体屏幕刷新和其它切换项保持原顺序。

## 2026-07-17 外控切通道脚踏图标黄色瞬态

- 外控 A/B 切换成功分支调用 `Pubinterface_LoadChannelMemory()`；脚踏在线且没有屏幕手动锁存时，该函数会同时把目标通道 `memory->drive_type` 和 `WorkMessage.drivetype_work` 写成 `JTWORK`。
- 随后的 `Pubinterface_RefreshRuntimeDisplaySnapshot()` 会进入 `Pubinterface_RefreshSelectedChannelDisplay()`，脚踏在线且 `drivetype_work==JTWORK` 时发送 `UI_CONTROL_ID` 选中值 1，因此实体屏幕脚踏图标立即变黄。
- 外控仲裁 owner 在整个通道切换过程中没有退出；黄色瞬态来自运行状态字段与外控 owner 不一致，不等于脚踏已经取得控制权。
- 上位机周期性重复申请/保活会再次进入 `ControlArbitration_EnterExternalControl()`，把 `drivetype_work` 写回 `TOUCHWORK` 并调用 `Pubinterface_RefreshControlModeDisplay()`，脚踏选中值随即变为 0，图标恢复白色。
- 上位机保活周期为 200ms，因此黄色持续时间取决于切换命令落在两个保活帧之间的位置，通常小于 200ms。
- `FootControlTask()` 在外控 owner 占用时只消费脚踏连接消息并立即返回，不处理踏板行程、电机、泵或通道切换；因此黄色期间脚踏不会实际取得控制权。
- 该现象仍属于真实状态不一致：`s_control_owner` 保持外控，但 `WorkMessage.drivetype_work` 短暂变成脚控。当前心跳不直接上传该字段；主要影响是实体屏幕脚踏图标瞬态，以及停机状态下依赖该字段选择显示来源的路径。
- 确认后的最小修复把外控 owner 判断放到通道记忆装载的脚踏优先判断之前；外控期间只把当前快照保持为 `TOUCHWORK`，不改目标通道 `memory->drive_type`，退出外控后的脚踏优先规则保持原样。



## 2026-07-24 固定方向手柄初始证据
- 型号表已确认：TMBB/TMBA/EMBA/EMBB为`0x6B,0x01..04`，MXYTM/MXYTP/JMB为`0x7C,0x01/02/05`。
- `handlescan.c`现有注释已经声明MXYTP靠机械结构实现往复、屏幕只显示往复而驱动仍按单向协议处理，但需要继续核对实际代码是否完整实现。
- 当前方向换入点至少包括EEPROM装载、屏幕方向键、外控方向命令和`sscDrive.c`最终下发，不能只把图标置灰而保留其它换向入口。

## 2026-07-24 当前方向实现缺口
- `Handlescan_ParseInitialDirection()`会让所有有效手柄接受Page4反转，并只对往复做能力校验；六类固定方向手柄目前没有“只服从EEPROM默认方向”的运行期锁定。
- `IsOscSupportedModel()`把MXYTP列为可切换往复型号，现有屏幕逻辑仍会把正转、反转和往复都作为可选方向，不符合“往复高亮、其它方向灰色不可选”。
- MXYTP的机械往复要求`WorkMessage.dir_work`用于屏幕显示OSCDIR，而`sscDrive.c`下发方向必须另行固定为ZZDIR；不能简单把MXYTP默认方向改成正转，否则屏幕语义会错误。

## 2026-07-24 换向入口与执行层证据
- 屏幕、旧HMI和手柄方向键最终都进入`DirActive()`，在该函数统一拒绝固定方向型号即可覆盖三类本地入口。
- 完整外控协议使用独立`ExternalComm_SetDirection()`直接写`WorkMessage/MemoryMsg.dir`，必须复用同一能力判断，否则可绕过本地门禁；简易6字节协议没有换向功能。
- 当前方向刷新始终发送正转、反转两个可用按钮，并按能力发送往复；需要改为按型号与锁定方向分别发送可用状态。
## 2026-07-20 RFID、泵队列与压力数据根因

- SplitType_AutoModeGetData_Task()发送RFID读取命令后，唯一的s_request_attempts_left--被注释；普通3次和快速10次均不会自然耗尽。
- SendPumpAMessage()和SendPumpBMessage()在Kernel_QueueSend()返回前更新静态去重消息；队列满时本次消息虽未入队，后续相同最新设定仍会被当成重复消息永久过滤。
- CS1237当前只校验帧头、版本、长度、帧尾和CRC；CRC正确但原始值不是合法24位符号扩展、重量或阈值明显越界的帧仍会刷新在线时间并进入压力闭环。
- 压力板协议明确RawCs1237是24位符号扩展；压力板现有阈值存储上限为50000g。主控采用这两个现有硬件边界，不按当前400g停泵表收紧，避免影响后续标定范围。
- 未知设备码必须继续进入原解码路径并强制泵离线，不能作为业务异常帧丢弃，否则拔出磁铁后的停泵响应会被延迟到3秒通信超时。
- 最终实现中RFID在发送后递减；泵只在Kernel_QueueSend返回pdPASS后更新去重基准；压力越界帧不刷新last_valid_tick，第3个连续合理帧才恢复业务写入。
## 2026-07-20 剩余可靠性与维护性审查结论

- 当前基线为`667bf66`；RFID次数递减、A/B泵队列满重试和CS1237业务合理性校验已经完成，不再列入待修复项。
- 驱动板非零Err当前设置报警并停A/B泵，但不立即调用`MotorUart_StopAllWork()`；电机运行请求要等后续Err=0恢复帧才统一清除，故障期间存在周期运行命令时序窗。
- UART1合法反馈没有最近有效时间和失联超时；旧速度/电流会长期保留，并参与控制权释放和上位机监测。超时阈值与停机语义需要实机联调后才能确定。
- 外控FIFO在CRC/帧尾通过后先调用`ExternalComm_ResetLinkWatchdog()`，随后`ExternalComm_DispatchFrame()`才过滤错误方向或InfoCode；合法但错向的帧能延长2秒/10秒保护。
- RFID停止消息带通道但接收端无条件清全局活动请求，A旧停止消息可能取消B新请求；停止消息当前也未初始化generation字段。
- DWIN解析只处理单次UART6 DMA快照；声明帧长超过当前剩余数据时跳过帧头，DMA随后复位，因此跨DMA半帧不能恢复。
- 脚踏掉线计时在识别到任意`FE EF`候选头时清零，持续结构错误噪声可能推迟离线；协议本身无CRC，主控只能加强结构、长度、类型和ADC边界，不能单方增加CRC。
- 17个业务线程虽然独立创建，但回调都串行经过`sAppTaskRuntimeMutex`；任一阻塞UART、Delay或长EEPROM操作会推迟其它任务。优化前应先测最长持锁时间，不应直接移除互斥锁。
- IWDG初始化与刷新均关闭，任务仍每300ms运行空刷新；FreeRTOS已包含高水位查询，但没有生产态栈/heap/互斥等待余量留证。
- `CUTTERSCANTaskHandle`未启动却在Map中占4240B RW；`ControlSigleMssage`只初始化且占8B；datahand、drivectrl_adapter、footpedal_ui_adapter已被链接器完整裁掉但仍参与构建。
- 当前大文件为external_comm_task.c 3716行、Pubinterface.c 2951行、sscFOOT.c 1835行；文件大小本身不是下一阶段首要风险。核心五个公共数据对象继续保留原位置和字段，只收口安全关键写入口。
## 2026-07-21 Page3两位小数适配初始事实

- 主控当前Page3只用`[8]`保存减速比、`[9]`保存增速比，两个字段均为单字节整数；加载时转换成内部x10倍率，不能表达EEPROM中的两位小数。
- 主控运行倍率字段低16位保存减速比、高16位保存增速比；RFID EPC当前也是x10倍率，Page3改造需要避免无关改变RFID协议。
- EEPROM写入工程当前已有用户未提交的任务配置、CSV profile和`pc_tool.zip`差异；上位机目录不是Git仓库。三处既有差异均按用户成果保护。
- 外部上位机当前把Page3 `[8]`标为增速、`[9]`标为减速，与主控和EEPROM写入工具的实际定义相反；本轮需要一并纠正只读显示。
- EEPROM写入工具`build_page3_data()`同样把减速比和增速比分别写到Page3 `[8]`、`[9]`单字节；页面表单类型是`int`、CSV加载使用`_int`、读回直接显示原始字节，现有测试明确断言单字节整数。
- 主控`tool_reduction_ratio`为32位打包字段，减速/增速各有16位空间；当前Page3整数先乘10后进入运行链，驱动计算按x10处理，RFID EPC也沿用x10。
- Page3 `[10]`由现有工具保留给夹持等后续字段的可能性需要继续核对；两位小数方案应优先复用确认未使用的后续字节并设置格式标记，而不是移动旧`[8]/[9]`字段。
- 全工程对完整倍率的实际数值运算集中在`sscDrive.c`，其它路径主要复制或上报32位打包值；因此可在每个16位倍率的高位加入`x100`格式标志，保留RFID/旧Page3的x10语义而不改公共结构体。
- 外部上位机确认Page3 `[10]`当前仅作为“夹持范围”只读展示，主控和写入工具均未使用；为降低碰撞，优先考虑在`[11..14]`写入强格式标记，再在后续字节保存两组BE16 x100真值。
- 新格式同时保留`[8]/[9]`整数镜像，可让旧主控继续得到向下取整的安全近似；新主控只有标记完整匹配才读取x100真值，否则严格回退旧整数。
- 最终兼容布局候选为：`[8]`旧减速整数镜像、`[9]`旧增速整数镜像、`[10]`夹持范围、`[11..14]="GR02"`、`[15..16]`减速x100 BE16、`[17..18]`增速x100 BE16、`[19..29]`保留。
- 主控当前只读取Page3前16字节；采用GR02布局后需把刀具信息缓存/读取长度从16增至19，仍在同一30字节业务区内且不改变页尾校验或Page1认证算法。
- 内部每个16位倍率半区可用`0x8000`标记x100，低15位保存0..25599；旧Page3/RFID x10值均小于0x1000，不会与标志碰撞。心跳A5长度和版本可保持不变，上位机按标志自适应显示。
- EEPROM写入工具已有Decimal解析基础；Page3表单需新增两位小数类型，CSV从`_int`改为数值解析，codec写GR02、整数镜像和BE16 x100，读回检测GR02后回填两位小数。
- `raw30_override`必须继续保持透明：用户给原始30字节时codec不重编码；读回仍保留原始页，但结构字段要按GR02或旧布局解析，结构字段修改后按现有逻辑清除旧raw覆盖。
- Page3缓存由`HANDLESCAN_TOOL_INFO_SIZE`统一控制，A/B扫描、手动模式恢复和一体式规格读取都复用同一缓存；把长度从16增至17即可覆盖`GR02`和两个百分位，所有调用点自动同步。
- 主控心跳A5固定传输原32位打包倍率，块长无需变化；上位机可在每个16位半区检查x100标志并分别显示减速或增速，旧x10/RFID值继续兼容。
- 方案最终确定为GR02+整数部分/百分位布局；主控活动倍率统一为x100，RFID x10在生产者入口乘10，A5刀具扩展升级到v2并由上位机同时兼容v1/v2。
- 倍率语义继续保持0或1.00为直联，只有大于1.00的减速或增速倍率生效；两者同时大于1.00时仍按矛盾配置保护为直联。
- 主控唯一软件说明手册已有Page3字段表、`tool_reduction_ratio`说明和A5协议章节，本轮代码通过后必须同步x100单位、GR02字节布局、v1/v2兼容及维护记录。
- EEPROM写入工程已有正式`pc_tool/dist/EEPROMTool.exe`和打包命令；源代码与测试通过后应确认该EXE是否受Git跟踪，再决定是否重打包，不能覆盖用户未跟踪的根目录`pc_tool.zip`。
- A5还存在直接从RFID原始EPC生成兜底块的路径；v2下该路径也必须把x10乘10，否则同一版本不同通道会混用单位。
- `pc_tool/dist/EEPROMTool.exe`是EEPROM写入仓库已跟踪正式产物；源代码与58+新增测试通过后需要按现有PyInstaller命令重建，并删除`pc_tool/build`和spec临时项。
- writer现有脏差异已精确确认：两份tasks.json是既有EIDE任务修复，CSV仅EMBA/EMBB电流阈值由0改450；本轮不得重写或回退这些行，根目录未跟踪`pc_tool.zip`也不覆盖。

## 2026-07-21 Page3两位小数适配最终结论

- 最终落地格式为：`[8]`减速整数镜像、`[9]`增速整数镜像、`[10]`夹持范围、`[11..14]="GR02"`、`[15]`减速百分位、`[16]`增速百分位；`[17..29]`继续保留。前述BE16和半区格式标志仅是探索候选，未进入产品实现。
- 只有`GR02`四字节完全匹配且两个百分位均不超过99时才使用新格式；否则减速和增速整组回退为历史整数镜像，不能半新半旧。
- 主控公共倍率字段统一使用x100：低16位减速、高16位增速、100表示1.00直联；RFID EPC的x10倍率只在装载入口乘10，驱动计算使用`uint64_t`并在收窄前钳位。
- EEPROM写入工具接受`0`或`1.00～255.99`且最多两位小数，禁止减速和增速同时大于1.00；Page3页和及Page1认证按原算法重算，固定样例认证为`5C 9E 75 05 06 A1 32 05`。
- 外控心跳A5版本2使用x100，版本1继续按x10解析；上位机仅修正只读显示与心跳解析，业务EEPROM写入口仍只允许Page12～128，`0x07`业务页写入继续固定拒绝。

## 2026-07-23 UART2简易外控协议初始事实

- 当前UART2 DMA空闲包先进入`external_comm_task.c`的环形FIFO，旧协议解析器只搜索`D7 CA F8 F1`；新协议不能并行独立扫描同一原始包，否则旧协议载荷内偶然出现`AA BB CC ... EE FF`时可能误执行。
- 最小可靠共存方式是在现有FIFO帧边界层比较两种帧头的最早偏移，固定6字节候选交给独立新模块校验和分发；旧协议CRC解析和业务分发保持原样。
- 新协议功能码表没有定义上行响应帧，因此“响应时间”暂按动作生效时限理解；本轮不自行创造ACK格式。
- 手柄增减速可复用`SpeedActive(HMIkey_SPEED_Sub/Add)`，实际步长继续来自当前通道Page6小步进及现有上下限。
- 左右泵启停可映射到现有外控控制动作，保留在线、外控owner、默认速度、手柄冷却跟随和压力保护；流量档位切换可复用`PUMPActive(JTkey_left_short/JTKey_right_short)`的0～5档类型表。
- 手柄切换使用当前通道的另一侧作为目标，继续复用`HandleSwitchActive()`的在线、运行和报警门禁。
- `0x0C`文字为“清除异常提醒”而非“清除故障”；安全实现应只关闭蜂鸣/弹窗，不清`WorkMessage.alarm_flag`，避免过载、堵转或设备故障在物理条件未恢复时重新启动。
- 现有外控具有2秒静默停输出、10秒静默释放owner保护；简易协议需要把每个完整且已知功能码视为链路活动，重复`0x01`可作为登录保活，不能因没有专用心跳码而取消安全超时。

## 2026-07-23 UART2简易外控协议最终结论

- 新协议实现全部位于`external_comm_simple_protocol.c/.h`；原`external_comm_task.c`只负责共享FIFO最早帧头仲裁、静默复用原安全分发和查询泵外控请求，不包含0x01～0x0C业务switch。
- `EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE=0U`时探测函数固定返回未找到、执行函数固定拒绝，真实全量构建比启用版减少736B ROM，证明新路径被编译裁剪。
- 简易协议没有定义应答格式，因此不发送自造ACK；动作状态继续从原心跳观察。要求可验证命令级确认时，必须由协议提供方先定义响应帧。
- `0x0C`只清蜂鸣和弹窗，不清`WorkAlarm`；这保证过载、堵转、压力或设备故障仍阻止0x03重新启动。
- 最终默认启用版全量构建为128个C、1个汇编，0 error、0 warning，RO 109848B、RW 130760B、ROM 110332B。

### EEPROM写入配置补充证据
- `EEprom_Write/pc_tool/profiles/eeprom_profiles.csv` 中 TMBA、TMBB、EMBA、EMBB、JMB、MXYTM、MXYTP 当前样例方向都写为 `1`（正转），但主控不能据此把六类固定方向型号写死为正转，仍需解析各自 Page4 的默认方向。
- MXYTP 的产品语义高于当前样例字节：业务/UI方向固定为往复，最终下发驱动时转换为正转，避免把机械往复错误地下发成电机电子换向。

### 固定方向实现结论
- 实机测试确认固定方向不应隐藏其它图标；最终复用`UIDIRDP(enable=false)`的20/23/26灰色禁用资源，当前固定方向保持高亮，上一版单方向隐藏状态已删除。
- 固定方向名单集中为 TMBA/TMBB/EMBA/EMBB/JMB/MXYTM/MXYTP；方向触控、`DirActive()`和外控方向设置都复用同一门禁，避免图标虽已置灰却仍可从其它入口改写。
- 六类单向型号仍由 `Handlescan_ParseInitialDirection()` 解析 Page4：`1` 正转、`2` 反转，非法值和不支持的往复保护为正转；锁定只发生在识别完成以后。
- MXYTP在识别层固定保存`OSCDIR`，屏幕高亮往复并把正反转置灰；`MOTORRUN()`组UART1帧前把有效方向改成`ZZDIR`，不回写屏幕和通道记忆。

### RFID标签电流阈值单位结论
- RFID EPC byte11为单字节电流阈值，标签协议单位按0.1A解释；用户现场规则确认标签100代表10A。
- 主控到驱动的电流字段单位为0.01A，因此`Handlescan_ApplyRfidToolResult()`必须乘10；0/1/100/255分别转换为0/10/1000/2550，且0继续表示使用驱动默认值。

## 2026-07-24 RFID刀具协议修正结论
- MXYTP和MXYTM是公共接头RFID刀具，不是EEPROM Page2手柄型号；两个型号均使用无刷电机。
- 0x01刨刀具保留正转、反转和电动往复，默认往复频率4Hz；0x02磨刀具禁止电动往复。
- 0x03反旋刀具屏幕显示EPC方向，UART1正反方向互换；0x04 MXYTP屏幕固定往复但UART1固定正转；0x05 MXYTM按EPC方向锁定并同向下发。
- 固定方向刀具沿用灰色禁用图标，不隐藏其它方向；屏幕、实体键/HMI和外控共用方向锁定判断。
- 只有PXY型号进入有刷判断，公共接头上的0x03/0x04/0x05均保持无刷。
- RFID电流阈值仍按EPC byte11的0.1A解释，标签100转换为驱动字段1000，即10.00A。
- EEPROM写入工具的Page2型号表与内置CSV原先仍保留7C 01/7C 02，必须移除，避免继续写出已废弃的MXY手柄身份；MXYTM16的7C 06保持不变。
- A5刀具块若只上报归一后的PLANER/GRINDH，0x03～0x05会在上位机失去型号差异；RFID基座必须保留EPC byte0原始型号，EEPROM来源继续保持原业务类型。
