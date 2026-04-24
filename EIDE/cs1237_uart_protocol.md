# CS1237 串口上报协议 v2

## 1. 概述

- 物理接口：`USART1`
- 串口参数：`9600, 8N1`
- 发送方式：设备端周期主动上报，当前周期为 `200 ms`
- 帧格式：固定长度二进制帧
- 字节序：多字节字段均为 `little-endian`
- 校验方式：`CRC16/MODBUS`
- 协议版本：`0x02`

本协议用于向主控 MCU 上报以下四类业务数据：

- `CS1237` 原始值
- 最终重量，单位 `0.1 g`
- 压力传感器阈值，单位 `g`
- 设备类型码，由 `PA1~PA4` 的霍尔状态组合得到

## 2. 设备类型定义

设备类型码由消抖后的 `PA1~PA4` 稳定状态打包得到：

```c
device_code = (st1 << 3) | (st2 << 2) | (st3 << 1) | st4;
```

其中：

- `st1` 对应 `PA1`
- `st2` 对应 `PA2`
- `st3` 对应 `PA3`
- `st4` 对应 `PA4`
- `1` 表示高电平
- `0` 表示低电平

当前定义的合法设备码如下：

| 霍尔状态 | 设备码 |
| --- | --- |
| `1111` | `0x0F` |
| `1110` | `0x0E` |
| `1100` | `0x0C` |
| `1000` | `0x08` |
| `1001` | `0x09` |

其余组合仍然允许上报，主控收到后按未知或异常设备处理。

## 3. 帧结构

- 帧头：`0xAA 0x55`
- 帧尾：`0x55 0xAA`
- 帧总长度：`21 Byte`

| Offset | Len | Field | 说明 |
| --- | ---: | --- | --- |
| 0 | 2 | Header | 固定 `0xAA 0x55` |
| 2 | 1 | ProtocolVer | 固定 `0x02` |
| 3 | 1 | MsgType | 固定 `0x01`，表示数据上报帧 |
| 4 | 1 | PayloadLen | 固定 `0x0C` |
| 5 | 1 | Seq | 帧序号，`0x00~0xFF` 循环 |
| 6 | 4 | RawCs1237 | `int32_t`，CS1237 原始值，24 位符号扩展后发送 |
| 10 | 4 | WeightX10 | `uint32_t`，最终重量，单位 `0.1 g` |
| 14 | 2 | ThresholdG | `uint16_t`，压力传感器阈值，单位 `g` |
| 16 | 1 | DeviceCode | 霍尔设备类型码 |
| 17 | 2 | CRC16 | `little-endian` |
| 19 | 2 | Tail | 固定 `0x55 0xAA` |

## 4. CRC 规则

- 算法：`CRC16/MODBUS`
- 多项式：`0xA001`
- 初值：`0xFFFF`
- CRC 覆盖范围：从 `ProtocolVer` 到 `DeviceCode`
- CRC 不包含：帧头、CRC 字段本身、帧尾

也就是对以下 `15 Byte` 进行校验：

```text
ProtocolVer + MsgType + PayloadLen + Seq + RawCs1237(4) + WeightX10(4) + ThresholdG(2) + DeviceCode
```

## 5. 主控解析建议

主控 MCU 建议按以下顺序解析：

1. 在接收流中查找帧头 `0xAA 0x55`
2. 读满固定长度 `21 Byte`
3. 检查帧尾是否为 `0x55 0xAA`
4. 重新计算 CRC 并与帧内 CRC 比较
5. CRC 正确后再解析 `Seq`、`RawCs1237`、`WeightX10`、`ThresholdG`、`DeviceCode`

若帧头错位或 CRC 错误，直接丢弃当前帧并继续搜索下一个帧头。

## 6. 固件实现说明

本工程中已经按以下方式实现：

- 保留 `CS1237_ReadRawSigned()` 原始采样流程
- 保留现有重量换算流程，发送字段使用 `WeightCalibration_ApplySegmentCalibrationX10()` 输出值
- 发送当前压力阈值变量 `pressure_threshold_g`，单位 `g`
- 串口文本调试输出已替换为固定长度二进制帧输出
- 帧序号在每次发送成功后自增，`0xFF` 后自动回绕到 `0x00`

## 7. 兼容性说明

- `v2` 在 `v1` 基础上新增了 `ThresholdG` 字段
- 主控如仍按 `v1` 的 `19 Byte` 帧解析，会发生错位，因此需要同步切换到 `ProtocolVer = 0x02`
- 若后续需要查询应答、参数配置或阈值下载，建议新增 `MsgType`，不要复用当前 `0x01` 数据上报帧
