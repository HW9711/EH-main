# UART2 外部通信协议串口助手测试用例

## 1. 串口助手连接设置

- UART：USART2 / UART2。
- 引脚：`PD5 = USART2_TX`，`PD6 = USART2_RX`。
- 波特率：`115200`。
- 数据位：`8`。
- 停止位：`1`。
- 校验位：`None`。
- 流控：`None`。
- 发送格式：HEX 发送。
- 注意：不要自动追加 `\r\n`，否则 CRC 会不匹配。
- 注意：MCU 每 1000ms 主动发送一次心跳，测试应答时需要按帧头和 `Length` 拆包，不要把心跳和应答混成一帧。

## 2. 帧格式和 CRC 规则

```text
D7 CA F8 F1 TranCode Length_H Length_L FunCode AreaCode InforCode InforArea CRC16_H CRC16_L BF C6 BC C4
```

- 帧头固定：`D7 CA F8 F1`。
- 帧尾固定：`BF C6 BC C4`。
- 下行帧 `TranCode = 02`。
- MCU 上传和应答 `TranCode = 01`。
- `Length` 当前固件按整帧字节数计算，包含帧头、帧尾、CRC 和全部字段。
- `CRC16` 使用 `Common_Crc16(TranCode..InforArea)`，不包含帧头、CRC 字段、帧尾。
- `CRC16_H` 先发，`CRC16_L` 后发。
- 多字节业务值按大端，例如 `0B B8 = 3000`。

## 3. 当前心跳帧解释

收到心跳：

```text
D7 CA F8 F1 01 00 17 AA FF FF FF FF FF 03 FF FF FF 5C 5D BF C6 BC C4
```

字段拆解：

| 字段 | 字节 | 含义 |
|---|---|---|
| Head | `D7 CA F8 F1` | 固定帧头 |
| TranCode | `01` | MCU 主动上传 |
| Length | `00 17` | 整帧 23 字节 |
| FunCode | `AA` | 心跳帧 |
| AreaCode | `FF` | 无区域码 |
| InforCode | `FF` | 无信息码 |
| InforArea[0] | `FF` | A 手柄离线 |
| InforArea[1] | `FF` | B 手柄离线；A 离线所以前面没有 A 手柄类型字段 |
| InforArea[2] | `FF` | 当前无选中手柄插孔；B 离线所以前面没有 B 手柄类型字段 |
| InforArea[3] | `03` | 当前手柄未接入 |
| InforArea[4] | `FF` | 脚踏离线；未运行所以前面没有速度/电流字段 |
| InforArea[5] | `FF` | A 泵离线 |
| InforArea[6] | `FF` | B 泵离线；A 泵离线所以前面没有 A 泵类型/速度字段 |
| CRC16 | `5C 5D` | 对 `01 00 17 AA FF FF FF FF FF 03 FF FF FF` 计算 |
| Tail | `BF C6 BC C4` | 固定帧尾 |

结论：板子当前外部通信任务正常发心跳；业务状态显示 A/B 手柄、脚踏、A/B 泵都离线，无当前选中通道，当前手柄状态为未接入。

新版心跳 `InforArea` 是可变长度，解析时必须按顺序读取并按条件跳字段：

- A 手柄在线状态后，只有在线才跟 2 字节 A 手柄 EEPROM 原始类型码。
- B 手柄在线状态后，只有在线才跟 2 字节 B 手柄 EEPROM 原始类型码。
- 当前手柄运行状态为运行中 `02` 时，才跟 2 字节速度和 2 字节电流。
- A/B 泵在线状态后，只有在线才跟 1 字节泵类型和 2 字节泵速度。

心跳中的手柄类型直接上传 EEPROM 原始 `0x6B xx` 两字节，不上传 `handlescan.c` 中 `s_hand_type_config_table` 映射后的 1 字节内部型号：

| 心跳手柄类型两字节 | 手柄型号 |
|---|---|
| `6B 01` | TMBB |
| `6B 02` | TMBA |
| `6B 03` | EMBA |
| `6B 04` | EMBB |
| `6B 05` | PXBA |
| `6B 06` | PXBB |

## 4. 应答码和失败原因

### 4.1 0xDD 应答 InforCode

| InforCode | 含义 |
|---|---|
| `01` | 运行值设置成功 |
| `02` | 运行值设置失败 |
| `03` | 控制成功 |
| `04` | 控制失败 |
| `05` | EEPROM 读写成功 |
| `06` | EEPROM 读写失败 |
| `AA` | 外部控制申请成功 |
| `BB` | 注册码校验失败 |
| `FE` | 权限开放成功 |
| `FF` | 权限开放失败 |

### 4.2 当前固件失败原因码

| 原因码 | 含义 |
|---|---|
| `01` | 长度错误 |
| `02` | AreaCode 或参数范围错误 |
| `03` | 当前无 A/B 通道 |
| `04` | 设备未配置、EEPROM 读写失败或泵未识别 |
| `05` | V1 不支持，例如整区读取 |
| `06` | Busy，例如未申请外部控制、运行中、报警中 |

## 5. 测试前置条件

- 基础连通测试：只接 UART2，先观察是否每 1 秒收到心跳。
- 启动类控制命令前：建议先发送“申请外部控制成功”帧。
- 设置速度/频率/泵速度前：需要当前存在 A 或 B 选中通道，否则返回“无通道”或“设备失败”。
- 切换到 A/B 通道：要求对应手柄在线，否则返回失败。
- A/B 泵启动：要求 CS1237 下位机帧中的 `DeviceCode` 属于合法霍尔设备类型码，并使 `pumpMessageA/B.online_flag == true`，否则返回设备失败。
- EEPROM 写测试会真实修改当前选中通道的 AT24CS32 数据。建议只在测试手柄或可恢复数据上执行，正式手柄先不要执行写用例。

## 6. 基础权限测试

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 1 | 申请外部控制成功 | `D7 CA F8 F1 02 00 18 01 FF FF 11 22 33 44 55 66 77 88 53 1D BF C6 BC C4` | `D7 CA F8 F1 01 00 10 DD FF AA BF 54 BF C6 BC C4` | InforArea 为 8 字节注册码；V1 只校验长度 |
| 2 | 申请外部控制失败 | `D7 CA F8 F1 02 00 17 01 FF FF 11 22 33 44 55 66 77 C9 7D BF C6 BC C4` | `D7 CA F8 F1 01 00 10 DD FF BB B3 94 BF C6 BC C4` | 注册码只有 7 字节 |
| 3 | 权限开放成功 | `D7 CA F8 F1 02 00 18 FA FF FF 01 02 03 04 05 06 07 08 FB 2A BF C6 BC C4` | `D7 CA F8 F1 01 00 10 DD FF FE 40 55 BF C6 BC C4` | InforArea 为 8 字节权限码；V1 只校验长度 |
| 4 | 权限开放失败 | `D7 CA F8 F1 02 00 17 FA FF FF 01 02 03 04 05 06 07 7F 63 BF C6 BC C4` | `D7 CA F8 F1 01 00 10 DD FF FF 80 94 BF C6 BC C4` | 权限码只有 7 字节 |

## 7. 静态值设置测试

前置：当前已有选中 A/B 通道，否则设置类命令先返回失败。

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 5 | 设置当前手柄速度 3000 | `D7 CA F8 F1 02 00 12 02 01 FF 0B B8 EE 8D BF C6 BC C4` | `D7 CA F8 F1 01 00 13 DD FF 01 01 0B B8 46 74 BF C6 BC C4` | AreaCode `01`，InforArea `0B B8` |
| 6 | 设置频率 20 | `D7 CA F8 F1 02 00 11 02 02 FF 14 44 25 BF C6 BC C4` | `D7 CA F8 F1 01 00 13 DD FF 01 02 00 14 0B 83 BF C6 BC C4` | 频率兼容 1 字节写法，返回按 2 字节回显 |
| 7 | 设置 A 泵速度 100 | `D7 CA F8 F1 02 00 12 02 03 FF 00 64 FF 8A BF C6 BC C4` | `D7 CA F8 F1 01 00 13 DD FF 01 03 00 64 2F D3 BF C6 BC C4` | 只改 A 泵速度，不启动泵 |
| 8 | 设置 B 泵速度 100 | `D7 CA F8 F1 02 00 12 02 04 FF 00 64 8B 8B BF C6 BC C4` | `D7 CA F8 F1 01 00 13 DD FF 01 04 00 64 EE 62 BF C6 BC C4` | 只改 B 泵速度，不启动泵 |
| 9 | 设置速度失败：长度不足 | `D7 CA F8 F1 02 00 11 02 01 FF 0B 8C 94 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 01 01 22 3F BF C6 BC C4` | 速度要求 2 字节；失败载荷：`01 01` |
| 10 | 设置失败：未知 AreaCode | `D7 CA F8 F1 02 00 10 02 99 FF E9 8E BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 99 02 E3 14 BF C6 BC C4` | 未定义设置项 |

无当前通道时，可用编号 5 的发送帧测试失败路径，返回通常为：

```text
D7 CA F8 F1 01 00 12 DD FF 02 01 03 E3 BE BF C6 BC C4
```

含义：设置失败，值代号 `01`，原因 `03` 当前无 A/B 通道。

## 8. 切换值测试

前置：不在运行中、无报警；切换 A/B 要求对应手柄在线。

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 11 | 切换到 A 通道 | `D7 CA F8 F1 02 00 10 03 01 FF E9 B4 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 01 01 00 E2 0E BF C6 BC C4` | A 在线时成功 |
| 12 | 切换到 B 通道 | `D7 CA F8 F1 02 00 10 03 02 FF 19 B4 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 01 02 00 12 0E BF C6 BC C4` | B 在线时成功 |
| 13 | 切换方向：往复 | `D7 CA F8 F1 02 00 11 03 03 FF 03 76 35 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 01 03 03 83 4F BF C6 BC C4` | AreaCode `03`，值 `03` |
| 14 | 切换控制模式：外部控制 | `D7 CA F8 F1 02 00 11 03 04 FF 03 B7 84 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 01 04 03 B3 4D BF C6 BC C4` | `TOUCHWORK = 03` |
| 15 | 切换工具类型：刨头 | `D7 CA F8 F1 02 00 11 03 05 FF 01 B6 54 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 01 05 01 E2 CD BF C6 BC C4` | `PLANER = 01` |
| 16 | 方向切换失败：缺少方向值 | `D7 CA F8 F1 02 00 10 03 03 FF 89 B5 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 03 01 42 3E BF C6 BC C4` | 原因 `01` 长度错误 |
| 17 | 控制模式失败：非法值 09 | `D7 CA F8 F1 02 00 11 03 04 FF 09 B0 04 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 04 02 73 7C BF C6 BC C4` | 原因 `02` 参数错误 |
| 18 | 工具类型失败：非法值 09 | `D7 CA F8 F1 02 00 11 03 05 FF 09 70 55 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 05 02 E3 7D BF C6 BC C4` | 原因 `02` 参数错误 |
| 19 | 切换失败：未知 AreaCode | `D7 CA F8 F1 02 00 10 03 99 FF 29 DF BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 02 99 02 E3 14 BF C6 BC C4` | 未定义切换项 |

运行中或报警中发送切换命令，返回类似：

```text
D7 CA F8 F1 01 00 12 DD FF 02 03 06 80 7F BF C6 BC C4
```

含义：运行值设置失败，值代号 `03`，原因 `06` Busy。

## 9. 控制命令测试

前置：启动类命令先发送编号 1 申请外部控制；当前手柄启动和开口定位还需要当前已选中 A/B 通道。

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 20 | A 泵启动 | `D7 CA F8 F1 02 00 10 04 01 FF 28 05 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 01 AF 3C BF C6 BC C4` | A 泵 type 已识别时成功 |
| 21 | A 泵停止 | `D7 CA F8 F1 02 00 10 04 02 FF D8 05 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 02 AE 7C BF C6 BC C4` | 停止类命令允许直接发 |
| 22 | B 泵启动 | `D7 CA F8 F1 02 00 10 04 03 FF 48 04 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 03 6E BD BF C6 BC C4` | B 泵 type 已识别时成功 |
| 23 | B 泵停止 | `D7 CA F8 F1 02 00 10 04 04 FF 78 06 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 04 AC FC BF C6 BC C4` | 停止类命令允许直接发 |
| 24 | 当前手柄启动 | `D7 CA F8 F1 02 00 10 04 05 FF E8 07 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 05 6C 3D BF C6 BC C4` | 需要已外部控制、已有当前通道 |
| 25 | 当前手柄停止 | `D7 CA F8 F1 02 00 10 04 06 FF 18 07 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 06 6D 7D BF C6 BC C4` | 清运行标志和速度 |
| 26 | 开口定位-左 | `D7 CA F8 F1 02 00 10 04 07 FF 88 06 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 07 AD BC BF C6 BC C4` | 需要已外部控制、已有当前通道 |
| 27 | 开口定位-右 | `D7 CA F8 F1 02 00 10 04 08 FF 78 03 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 08 A9 FC BF C6 BC C4` | 需要已外部控制、已有当前通道 |
| 28 | 紧急刹车 | `D7 CA F8 F1 02 00 10 04 FF FF 48 45 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 03 FF 2F BD BF C6 BC C4` | 清手柄运行和 A/B 泵运行 |
| 29 | 控制失败：未知 AreaCode | `D7 CA F8 F1 02 00 10 04 09 FF E8 02 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 09 02 E2 98 BF C6 BC C4` | 未定义控制项 |

常见失败路径：

| 目的 | 发送 HEX | 预期返回 HEX | 说明 |
|---|---|---|---|
| 未申请外部控制时启动当前手柄 | `D7 CA F8 F1 02 00 10 04 05 FF E8 07 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 05 06 21 9C BF C6 BC C4` | 原因 `06` Busy |
| 无当前通道时启动当前手柄 | `D7 CA F8 F1 02 00 10 04 05 FF E8 07 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 05 03 22 5C BF C6 BC C4` | 需先申请外部控制，但当前无通道 |
| A 泵未识别时启动 A 泵 | `D7 CA F8 F1 02 00 10 04 01 FF 28 05 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 01 04 20 1F BF C6 BC C4` | `pumpMessageA.type == 0` |
| B 泵未识别时启动 B 泵 | `D7 CA F8 F1 02 00 10 04 03 FF 48 04 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 03 04 40 1E BF C6 BC C4` | `pumpMessageB.type == 0` |

## 10. EEPROM 单页读写测试

危险：写测试会真实修改当前选中通道 EEPROM。正式手柄不要直接执行写入用例。

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 30 | 读取业务 EEPROM Page2 / Area01 | `D7 CA F8 F1 02 00 10 05 01 FF E8 54 BF C6 BC C4` | `D7 CA F8 F1 01 00 2E 01 01 FF <30字节EEPROM有效数据> CRC_H CRC_L BF C6 BC C4` | 返回内容随 EEPROM 实际数据变化 |
| 31 | 读取业务 EEPROM：未知 Area09 | `D7 CA F8 F1 02 00 10 05 09 FF 28 53 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 09 02 22 39 BF C6 BC C4` | 原因 `02` AreaCode 错误 |
| 32 | 写业务 EEPROM Area01，30 字节 | `D7 CA F8 F1 02 00 2E 07 01 FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E FB A1 BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 05 01 0F 3F BF C6 BC C4` | 写入 30 字节有效数据；驱动生成页尾 2 字节校验 |
| 33 | 写业务 EEPROM：长度 29 字节 | `D7 CA F8 F1 02 00 2D 07 01 FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D E0 F1 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 01 01 E3 7E BF C6 BC C4` | 原因 `01` 长度错误 |
| 34 | 写业务 EEPROM：未知 Area09 | `D7 CA F8 F1 02 00 2E 07 09 FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 9B A5 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 09 02 22 39 BF C6 BC C4` | 原因 `02` AreaCode 错误 |
| 35 | 读取导航 EEPROM Page12 | `D7 CA F8 F1 02 00 10 08 0C FF BB C1 BF C6 BC C4` | `D7 CA F8 F1 01 00 2E 01 FF 0C <30字节导航有效数据> CRC_H CRC_L BF C6 BC C4` | AreaCode 固定 `FF`，InforCode 回显页码 `0C` |
| 36 | 读取导航：页码 00 非法 | `D7 CA F8 F1 02 00 10 08 00 FF BB C4 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 00 02 72 3F BF C6 BC C4` | 原因 `02` 页码错误 |
| 37 | 写导航 EEPROM Page12，30 字节 | `D7 CA F8 F1 02 00 2E 0A 0C FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E C5 1B BF C6 BC C4` | `D7 CA F8 F1 01 00 11 DD FF 05 0C CA FE BF C6 BC C4` | 写入 Page12 的前 30 字节 |
| 38 | 写导航：长度 29 字节 | `D7 CA F8 F1 02 00 2D 0A 0C FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D DA A7 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 0C 01 73 7A BF C6 BC C4` | 原因 `01` 长度错误 |

如果当前无选中通道，或者 EEPROM 读写失败，读写类命令会返回：

```text
D7 CA F8 F1 01 00 12 DD FF 06 <命令AreaCode> 04 CRC_H CRC_L BF C6 BC C4
```

含义：EEPROM 操作失败，原因 `04` 设备失败。

## 11. 整区读取和未知命令测试

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期返回 HEX | 说明 |
|---|---|---|---|---|
| 39 | 读取业务整区：V1 不支持 | `D7 CA F8 F1 02 00 10 06 FF FF 88 E4 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 06 05 10 7D BF C6 BC C4` | 失败对象回显 FunCode `06` |
| 40 | 读取导航整区：V1 不支持 | `D7 CA F8 F1 02 00 10 09 AA FF DB EB BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 06 09 05 E0 78 BF C6 BC C4` | 失败对象回显 FunCode `09` |
| 41 | 未知 FunCode `99` | `D7 CA F8 F1 02 00 10 99 FF FF A6 D4 BF C6 BC C4` | `D7 CA F8 F1 01 00 12 DD FF 04 99 02 E2 F4 BF C6 BC C4` | 控制失败，原因 `02` |
| 42 | 外部设备下发 ACK `BB` | `D7 CA F8 F1 02 00 10 BB FF FF AC 74 BF C6 BC C4` | 无 `0xDD` 应答 | MCU 忽略外部 ACK；仍会周期发心跳 |

## 12. 异常帧测试

这些帧用于验证解析层丢弃能力。预期均无 `0xDD` 应答；串口助手仍可能看到周期心跳。

| 编号 | 目的 | 串口助手发送 HEX | MCU 预期 |
|---|---|---|---|
| 43 | CRC 错误 | `D7 CA F8 F1 02 00 18 01 FF FF 11 22 33 44 55 66 77 88 52 1D BF C6 BC C4` | 无应答 |
| 44 | 帧尾错误 | `D7 CA F8 F1 02 00 18 01 FF FF 11 22 33 44 55 66 77 88 53 1D BF C6 BC C5` | 无应答 |
| 45 | Length 小于 16 | `D7 CA F8 F1 02 00 0F 01 FF FF 11 22 33 44 55 66 77 88 67 29 BF C6 BC C4` | 无应答 |
| 46 | Length 超过 150 | `D7 CA F8 F1 02 00 97 01 FF FF 11 22 33 44 55 66 77 88 39 0B BF C6 BC C4` | 无应答 |
| 47 | TranCode 不是 `02` | `D7 CA F8 F1 01 00 18 01 FF FF 11 22 33 44 55 66 77 88 50 1E BF C6 BC C4` | 无应答 |
| 48 | InforCode 不是 `FF` | `D7 CA F8 F1 02 00 18 01 FF 00 11 22 33 44 55 66 77 88 A7 58 BF C6 BC C4` | 无应答 |

## 13. 推荐测试顺序

1. 上电后只接收，确认每 1 秒出现 `FunCode=AA` 心跳。
2. 发送编号 1，确认 MCU 返回 `DD FF AA`。
3. 插入 A 或 B 手柄，观察心跳中 A/B 在线和当前通道是否变化。
4. 发送编号 11 或 12 切换当前通道。
5. 发送编号 5、6 设置速度和频率。
6. 发送编号 24 当前手柄启动，观察心跳中的“当前手柄运行情况”字段变为 `02`，且后面追加速度和电流。
7. 发送编号 25 或 28 停止，观察心跳中的“当前手柄运行情况”字段回到 `01`；如果当前无手柄则为 `03`。
8. 只在测试 EEPROM 上执行编号 30~38。
9. 最后执行异常帧测试，确认 MCU 不乱回包。
