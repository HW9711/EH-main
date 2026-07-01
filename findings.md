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
- 软 UART 压力链路中，SIM_UART_1 RX=PE4 固定写 `pumpMessageB`，SIM_UART_2 RX=PE6 固定写 `pumpMessageA`。
- 压力主控白名单为 `0x00/0x0E/0x0C/0x08/0x09`，压力工程文档列 `0x0F/0x0E/0x0C/0x08/0x09`，需要实测霍尔高低电平定义。
- 外控心跳是动态长度：手柄在线才追加原始类型，泵在线才追加泵类型、速度和 11 字节压力扩展。
- 外控链路短超时 1000ms 停输出、长超时 5000ms 释放外控；源码注释仍写 30s，文档按代码值记录。
- `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE=1`，逻辑 A/B 泵最后输出到物理 UART 口时会互换。
- A 泵任务周期 25ms，B 泵任务周期 100ms，双泵一致性需要做阶跃响应实测。

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
- A 泵任务周期 25ms、队列深度 5；B 泵任务周期 100ms、队列深度 2。二者都以 `pumpMessageA/B` 为唯一输出数据源，队列只更新类型和目标速度。
- 压力闭环阈值单位为 g，重量为 0.1g。默认硬停点是 `threshold_g * 10 * 3 / 2`；阈值以下不降速，阈值到硬停点线性降速，硬停点以上本周期输出 0 但不清 `run_flag`。
- 软 UART 压力解析是 9600 8N1、TIM11 采样、全局单 active receiver；一路正在接收时另一路起始位会增加 `overlap_drop_count` 并被抑制，双路同相 200ms 上报需要实测丢帧率。
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
- `User\Application\include\pump_pressure_control.h` 当前压力闭环总开关默认开启，恢复消抖 3000ms；A 压力源固定 `pumpMessageA`/SIM_UART_2/PE6，B 压力源固定 `pumpMessageB`/SIM_UART_1/PE4。
- A 泵任务周期 25ms，B 泵任务周期 100ms；同样的排空计数 `>300` 会导致 A/B 实际排空时长不同。
- `docs/software-debug-report.md` 已生成，报告采用“配置速查、观察变量、模块断点、故障现象、构建清单”的顺序，适合后续测试时直接定位问题。
- 本轮没有修改业务源码；报告生成后检查了异常字符、占位符、行尾空格、关键断点函数和 `git diff --check`。

## 2026-06-27 主控原理版调试报告重写发现

- 旧调试报告偏向“在哪里打断点”，对为什么要经过公共状态、为什么不能输入直连输出解释不足；新版报告应把原理放在断点前面。
- 当前主控输入链路统一收敛到 `WorkMessage`、`MemoryMsgA/B`、`ChannelrecognizeMessageA/B`、`pumpMessageA/B`、`ControlSignalMessage`，这是理解脚踏、手柄、屏幕和外控互斥的主线。
- `AppTaskRuntimeGate()` 让旧业务回调通过同一个互斥锁串行执行，因此 FreeRTOS 多任务不是任意业务代码并发运行；调试周期卡顿时要同时看任务周期和互斥等待。
- 当前外控短超时为 2 秒停输出，长超时为 10 秒释放外控，应以 `EXTERNAL_COMM_LINK_STOP_OUTPUT_TIMEOUT_MS` 和 `EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS` 为准。
- 当前主控压力设备码白名单为 `0x00/0x08/0x09`，旧文档和压力工程历史资料出现过其它设备码组合，属于跨工程协议一致性风险。
- `PUMP_LOGICAL_AB_PHYSICAL_SWAP_ENABLE` 改的是逻辑泵到物理 UART 的映射，`UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE` 改的是屏幕显示和触控映射，二者不能混用。
- A 泵任务周期 25ms，B 泵任务周期 100ms，两者共用 `timingDrainage_times > 300` 会造成实际排空时长不同，新报告需要明确这是实机测试点。
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
- 压力超过停泵点时不清 `run_flag` 是安全层保持停泵设计，恢复依赖压力回到安全区并满足 `PUMP_PRESSURE_CONTROL_RECOVER_DEBOUNCE_MS`。
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
