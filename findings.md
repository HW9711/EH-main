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
