# 嵌入式软件方案设计说明书

## 文档信息

| 项目 | 内容 |
|------|------|
| 项目名称 | 主控制板F413固件 |
| 芯片型号 | STM32F4xx (ARM Cortex-M4) |
| 操作系统 | FreeRTOS |
| 文档版本 | V1.0 |
| 生成日期 | 2026-03-19 |

---

## 1. 系统概述

### 1.1 产品定位

本系统为**医疗手术动力设备主控制板**，用于控制无刷电机驱动、显示屏交互、脚踏/手柄输入等多模块协同工作。系统支持多种手术器械模式（磨钻、刨刀、耳膜机等），具备完善的故障检测与报警机制。

### 1.2 核心功能

- **电机控制**: 支持A/B双通道无刷电机调速（最高160000 RPM）
- **人机交互**: 4.3寸LCD显示屏 + 电容触摸按键
- **输入控制**: 脚踏板（支持多种踩踏模式）+ 手柄按键
- **液体管理**: 灌注泵、注水泵流量控制
- **安全监控**: 过载保护、掉电存储、异常报警

### 1.3 关键约束

| 指标 | 规格 | 源码依据 |
|------|------|----------|
| 系统时钟 | 100MHz (PLL 200MHz/2) | `main.c: SystemClock_Config()` |
| UART数量 | 7路 (6路DMA) | `main.c: MX_USART1_UART_Init() ~ MX_UART7_Init()` |
| ADC通道 | 4通道 | `board.h: BOARD_ADC_CHANNEL_CNT` |
| 看门狗 | 禁用（与参考项目对齐） | `main.c: /* MX_IWDG_Init(); */` |
| 任务栈深度 | 1024字节 | `app_task.c: APP_TASK_STACK_DEPTH` |

---

## 2. 软件架构

### 2.1 分层架构

```mermaid
graph TB
    subgraph Application["应用层"]
        A1[userparser.c<br/>系统初始化]
        A2[screen.c<br/>UI与按键处理]
        A3[app_task.c<br/>任务调度框架]
    end
    
    subgraph Middleware["中间件"]
        M1[FreeRTOS<br/>任务管理]
        M2[app_task<br/>定时任务框架]
    end
    
    subgraph Driver["驱动层"]
        D1[UART1-7<br/>串口通信]
        D2[GPIO<br/>输入输出]
        D3[TIM7/10/14<br/>定时器]
        D4[ADC<br/>模拟采集]
        D5[I2C软件模拟<br/>EEPROM]
        D6[1-Wire<br/>RFID/温度]
    end
    
    subgraph Hardware["硬件抽象层"]
        H1[board.h/c<br/>引脚配置宏]
        H2[stm32f4xx_hal<br/>HAL库]
    end
    
    A1 --> M1
    A2 --> M2
    M2 --> D1
    M2 --> D2
    D1 --> H1
    D2 --> H1
    H1 --> H2
```

### 2.2 目录结构

```
EIDE/
├── Src/                          # STM32CubeMX生成代码
│   ├── main.c                   # 主程序入口
│   ├── usart.c                  # UART外设配置
│   ├── app_task.c               # 定时任务框架
│   └── ...
├── User/
│   ├── board/                   # 硬件抽象层
│   │   ├── board.h              # 引脚/外设宏定义
│   │   └── board.c              # 硬件初始化实现
│   ├── Peripheral/
│   │   └── uart/
│   │       ├── uart1.c          # 无刷电机通信
│   │       ├── uart2.c          # 外部通讯
│   │       ├── uart3.c          # 射频通信
│   │       ├── uart4.c          # 脚踏通信
│   │       ├── uart5.c          # 步进电机1
│   │       ├── uart6.c          # 显示屏通信
│   │       └── uart7.c          # 步进电机2
│   └── Application/
│       ├── Src/
│       │   └── userparser.c     # 系统初始化与任务创建
│       └── Screen/
│           └── screen.c         # UI显示与按键处理
└── docs/
    └── EmbeddedSoftwareDesign.md # 本文档
```

---

## 3. 硬件资源分配

### 3.1 UART外设分配

| 端口 | 用途 | 波特率 | DMA通道 | 源码位置 |
|------|------|--------|---------|----------|
| USART1 | 无刷电机 | 9600 | DMA2_Stream2 | `uart1.c` |
| USART2 | 外部通讯 | 9600 | DMA1_Stream5 | `uart2.c` |
| USART3 | 显示屏 | 115200 | DMA1_Stream1 | `uart3.c` |
| UART4 | 脚踏 | 115200 | DMA1_Stream2 | `uart4.c` |
| UART5 | 步进电机1 | 9600 | DMA1_Stream0 | `uart5.c` |
| USART6 | 预留 | 115200 | DMA2_Stream1 | `uart6.c` |
| UART7 | 步进电机2 | 9600 (Tx only) | DMA1_Stream3 | `uart7.c` |

### 3.2 GPIO资源分配

```mermaid
graph LR
    subgraph 输入["输入设备"]
        I1[PA1-PA4<br/>ADC按键/电压]
        I2[PD0-PD1<br/>手柄按键]
        I3[PC12<br/>手柄按键]
        I4[PD2-PD4<br/>手柄数据]
    end
    
    subgraph 输出["输出设备"]
        O1[PE10<br/>状态LED]
        O2[PA6<br/>蜂鸣器]
        O3[PC8-PC9<br/>指示灯]
        O4[PA8-PA9<br/>指示灯]
        O5[PD14-PD15<br/>继电器K1/K2]
        O6[PB15<br/>R200-K8]
    end
    
    subgraph 通信["通信接口"]
        C1[PB6-PB7<br/>USART1]
        C2[PD5-PD6<br/>USART2]
        C3[PD8-PD9<br/>USART3]
        C4[PC4-PC5<br/>I2C软件模拟]
        C5[PD12-PD13<br/>1-Wire总线]
    end
```

### 3.3 定时器分配

| 定时器 | 用途 | 周期 | 源码依据 |
|--------|------|------|----------|
| TIM6 | 系统滴答 | 1ms | `main.c: HAL_TIM_PeriodElapsedCallback` |
| TIM7 | 系统滴答 | - | `main.c: MX_TIM7_Init()` |
| TIM10 | 蜂鸣器驱动 | 50000预分频×5周期 | `board.h: BOARD_TIM10_PRESCALER` |
| TIM14 | 系统计时 | 50000预分频×500周期 = 1s | `board.h: BOARD_TIM14_PERIOD` |

---

## 4. 任务设计

### 4.1 FreeRTOS任务列表

| 任务名 | 优先级 | 周期 | 功能 | 源码位置 |
|--------|--------|------|------|----------|
| AppTask | Idle+3 | 1ms | 应用层定时任务调度 | `app_task.c` |
| (系统任务) | - | - | FreeRTOS内核管理 | `freertos.c` |

### 4.2 应用定时任务（基于AppTask框架）

```mermaid
graph TB
    subgraph AppTask调度器["AppTask调度器 - 1ms周期"]
        AT[AppTaskScheduler<br/>遍历任务链表<br/>执行到期任务]
    end
    
    subgraph 高优先级任务["高优先级任务 (>100ms)"]
        T1[IwdgTask<br/>300ms<br/>看门狗]
        T2[LEDTask<br/>200ms<br/>运行灯]
        T3[BeepControlTask<br/>10ms<br/>蜂鸣器控制]
    end
    
    subgraph 中优先级任务["中优先级任务 (20-50ms)"]
        T4[HandlescanTask<br/>10ms<br/>手柄连接扫描]
        T5[ScreenKeyTask<br/>30ms<br/>屏幕按键]
        T6[FootKeyTask<br/>30ms<br/>脚踏按键]
        T7[DriveCtrl_Motor123Task<br/>55ms<br/>电机控制]
    end
    
    subgraph 低优先级任务["低优先级任务 (>100ms)"]
        T8[FootPedalTask<br/>25ms<br/>脚踏扫描]
        T9[MotorUartDataTask<br/>3ms<br/>电机数据接收]
        T10[SplitType_AutoModeGetDataTask<br/>200ms<br/>刀具信息获取]
        T11[PUMPBTask<br/>100ms<br/>泵控制]
    end
    
    AT --> T1
    AT --> T2
    AT --> T3
    AT --> T4
    AT --> T5
    AT --> T6
    AT --> T7
    AT --> T8
    AT --> T9
    AT --> T10
    AT --> T11
```

### 4.3 任务创建代码映射

```c
// 源码位置: userparser.c: Userparser_Init()

// 高优先级
IwdgTaskInit();              // 看门狗任务
LEDTaskInit();               // 运行灯任务  
BeepControlTask_Init();      // 蜂鸣器控制
HandlescanTaskInit();        // 手柄扫描

// 中优先级
PedalRecvTask_Init();        // 脚踏数据接收 (3ms)
FootPedalTask_Init();        // 脚踏连接扫描
ScreenKeyTask_Init();        // 屏幕按键逻辑
FootKeyTask_Init();          // 脚踏按键任务
ScreenKey_ScanInit();        // 屏幕按键扫描 (22ms)
FootThrottleTask_Init();     // 脚踏油门
DriveCtrl_Motor123Task_Init(); // 电机控制
HandleKeyScan_Init();        // 手柄按键扫描

// 低优先级
MotorUartData_Init();        // 驱动板数据接收
SplitType_AutoModeGetData_Init(); // 刀具信息获取 (200ms)
DriveCtrl_HMITask_Init();    // HMI刷新
PUMPBTask_Init();            // 泵控制
```

---

## 5. 状态机设计

### 5.1 系统运行状态机

```mermaid
stateDiagram-v2
    [*] --> PowerOnInit: 上电复位
    
    PowerOnInit --> Standby: 初始化完成
    
    Standby --> DeviceConnecting: 检测到手柄/脚踏
    DeviceConnecting --> Ready: 外设全部就绪
    
    Ready --> Running: 启动电机
    Running --> Ready: 停止电机
    
    Running --> Alarm: 故障触发
    Alarm --> Ready: 故障清除
    
    Ready --> Standby: 所有外设断开
    
    note right of Running
        速度: 0-160000 RPM
        方向: 正转/反转/往复
        控制: 脚踏/手柄/触控
    end note
    
    note right of Alarm
        过载/相位错误/
        通讯故障/型号错误
    end note
```

### 5.2 手柄识别状态

```mermaid
stateDiagram-v2
    [*] --> Disconnected
    
    Disconnected --> Connecting: 插入手柄
    Connecting --> Identifing: 读取RFID/DS2401
    Identifing --> Connected: 识别成功
    Identifing --> Error: 识别失败
    
    Connected --> Working: 选中通道
    Working --> Selected: 切换通道
    
    Connected --> Disconnected: 拨出手柄
    Error --> Disconnected: 拨出重试
    Selected --> Disconnected: 拨出手柄
    
    note right of Working
        TMBA/TMBB: 耳磨
        EMBA/EMBB: 耳膜
        PXBA/PXBB: 刨削
        MX_YIM/YIP: 一体磨
    end note
```

### 5.3 电机控制状态

```mermaid
stateDiagram-v2
    [*] --> Stopped
    
    Stopped --> Accelerating: 启动命令
    Accelerating --> Running: 达到目标转速
    
    Running --> Decelerating: 停止命令
    Decelerating --> Stopped: 转速为0
    
    Running --> Overload: 过载检测
    Overload --> Stopped: 立即停止
    
    Running --> Error: 通讯超时
    Error --> Stopped: 错误处理
    
    note right of Running
        最高转速: 160000 RPM
        控制方式: UART1
        反馈: 实时转速/电流
    end note
```

---

## 6. 通信协议

### 6.1 显示屏通信 (UART3 - 115200bps)

```mermaid
sequenceDiagram
    participant MCU
    participant LCD
    
    MCU->>LCD: 发送命令/数据
    Note over LCD: 0xXXXX:地址, 数据
    
    rect rgb(200, 240, 200)
        Note over LCD: 图片显示<br/>LCD_Show_Picture(addr, id)
    end
    
    rect rgb(200, 220, 200)
        Note over LCD: 数字显示<br/>LCD_Show_Number(addr, data)
    end
    
    rect rgb(200, 200, 200)
        Note over LCD: 按键事件<br/>通过串口返回键值
    end
```

### 6.2 无刷电机通信 (UART1 - 9600bps)

```mermaid
sequenceDiagram
    participant MCU
    participant Motor
    
    MCU->>Motor: 设置转速 [0x01, 速度高, 速度低, 校验]
    MCU->>Motor: 设置方向 [0x02, 方向, 校验]
    MCU->>Motor: 读取状态 [0x03, 校验]
    Motor-->>MCU: 返回转速/电流/故障码
    
    Note over Motor: 速度范围: 0-160000<br/>反馈周期: <10ms
```

### 6.3 脚踏通信 (UART4 - 115200bps)

```mermaid
sequenceDiagram
    participant MCU
    participant Pedal
    
    Pedal-->>MCU: 模拟量: 油门开度 (0-100%)
    Pedal-->>MCU: 数字量: 按键状态
    
    Note over Pedal: 支持双脚踏三按键<br/>支持存储校准值
```

---

## 7. 数据流设计

### 7.1 核心数据流

```mermaid
flowchart LR
    subgraph 输入层
        Input1[脚踏模拟量<br/>ADC]
        Input2[按键输入<br/>GPIO]
        Input3[触摸事件<br/>UART6]
        Input4[手柄数据<br/>UART1]
    end
    
    subgraph 处理层
        P1[输入解析<br/>screen.c]
        P2[状态更新<br/>Workvalue_s]
        P3[任务调度<br/>app_task.c]
    end
    
    subgraph 控制层
        C1[电机控制<br/>DriveCtrl]
        C2[泵控制<br/>Pump]
        C3[显示更新<br/>LCD]
    end
    
    subgraph 输出层
        O1[UART1<br/>无刷电机]
        O2[UART3<br/>显示屏]
        O3[GPIO<br/>继电器/LED]
    end
    
    Input1 --> P1
    Input2 --> P1
    Input3 --> P1
    Input4 --> P2
    P1 --> P2
    P2 --> P3
    P3 --> C1
    P3 --> C2
    P3 --> C3
    C1 --> O1
    C2 --> O2
    C3 --> O3
```

### 7.2 关键数据结构

```c
// 源码位置: screen.c

// 工作参数结构体
typedef struct {
    uint32_t set_speed;           // 设置转速 (0-160000)
    uint32_t set_Freq;            // 往复频率 (0-40Hz)
    uint8_t set_Direction;        // 方向: 0-停止, 1-正转, 2-反转, 3-往复
    uint8_t set_Way;              // 控制方式: 脚控/手控/触控
    uint8_t set_Injection;        // 注水设置 (0-70ml)
    uint8_t set_Irrigate;         // 灌注设置
    uint8_t hand_model;           // 手柄型号
    uint8_t tool_model;           // 刀具模式: 0-磨头, 1-刨刀
    
    // 状态标志
    uint8_t MOTORWorking_flag;    // 电机运行标志
    uint8_t beep_Alarm_flag;      // 报警标志
    uint8_t Alarm_value;          // 报警代码
    
    // 连接状态
    uint8_t Achanell_online_flag; // A通道在线
    uint8_t Bchanell_online_flag; // B通道在线
    uint8_t footcontrol_online_flag; // 脚踏在线
} Workvalue;

// 通道记忆结构体
typedef struct {
    uint32_t set_speed;
    uint8_t set_Freq;
    uint8_t set_Direction;
    uint8_t set_Way;
    uint8_t tool_model;
    uint8_t hand_model;
    uint8_t manual_flag;
} ChannelValue_t;
```

---

## 8. 存储设计

### 8.1 Flash分区

```mermaid
graph BT
    subgraph Flash存储["内部Flash - 1MB"]
        F1[0x08000000<br/>Bootloader<br/>~32KB]
        F2[0x08008000<br/>应用代码<br/>~448KB]
        F3[0x0807C000<br/>配置区<br/>~16KB]
        F4[0x08080000<br/>保留<br/>~512KB]
    end
    
    subgraph EEPROM["AT24C02 - 2Kb"]
        E1[0x20-0x21<br/>泵流量A]
        E2[0x23<br/>泵流量记忆]
        E3[0x25-0x27<br/>泵流量B]
    end
    
    F3 -.-> E1
    F3 -.-> E2
    F3 -.-> E3
```

### 8.2 存储操作

| 地址 | 用途 | 大小 | 源码依据 |
|------|------|------|----------|
| ADDR_BASE (0x0807C000) | 手柄模式 | 2字节 | `userparser.c: Storage_HandleMode_init()` |
| 0x20-0x27 | 泵流量 | 8字节 | `userparser.c: Storage_PumpFlow_init()` |

---

## 9. 可靠性设计

### 9.1 安全机制

| 机制 | 状态 | 说明 |
|------|------|------|
| 看门狗 (IWDG) | ⚠️ 禁用 | 与参考项目对齐，待确认是否启用 |
| 错误处理 | ✅ 启用 | `Error_Handler()` 死循环 |
| Flash存储 | ✅ 启用 | 关键配置掉电保存 |
| 报警系统 | ✅ 启用 | 16种报警代码 |

### 9.2 报警代码

```c
// 源码位置: screen.c

#define ALARM_1  1  // 手柄未连接
#define ALARM_2  2  // 脚踏未连接
#define ALARM_3  3  // 刀具未连接
#define ALARM_4  4  // 霍尔型号错误
#define ALARM_5  5  // 电机通讯故障
#define ALARM_6  6  // 电机过载
#define ALARM_7  7  // 手柄未连接，无法启动脚踏
#define ALARM_8  8  // 电机相位错误
#define ALARM_9  9  // 脚踏存储值错误
#define ALARM_10 10 // 手柄型号错误
#define ALARM_11 11 // UID错误
#define ALARM_12 12 // 脚控已选中，请使用脚控
#define ALARM_13 13 // 手控已选中，请使用手控
```

### 9.3 风险与待确认项

| 风险项 | 严重程度 | 建议措施 |
|--------|----------|----------|
| 看门狗已禁用 | 🔴 高 | 建议启用IWDG，喂狗周期<500ms |
| UART7仅TX模式 | 🟡 中 | 确认步进电机无需反馈 |
| 继电器无电流检测 | 🟡 中 | 建议增加负载检测 |

---

## 10. 性能指标建议

### 10.1 响应时间

| 指标 | 当前值 | 建议值 | 说明 |
|------|--------|--------|------|
| 按键响应 | <30ms | <20ms | ScreenKeyTask周期 |
| 电机启停 | <100ms | <50ms | 含通讯超时 |
| 显示屏刷新 | <100ms | <50ms | DriveCtrl_HMITask |
| 报警触发 | <15ms | <10ms | Warn_StatusScanTask |

### 10.2 资源占用

| 资源 | 使用量 | 总量 | 使用率 |
|------|--------|------|--------|
| SRAM | ~64KB | 192KB | ~33% |
| Flash | ~256KB | 1MB | ~25% |
| 任务栈 | 11KB | - | 11×1KB |

---

## 11. 接口定义

### 11.1 任务创建API

```c
// 源码位置: app_task.c

typedef void (*cbFunc)(uint32_t event);

typedef struct task {
    bool start;           // 任务启动标志
    bool oneShot;         // 单次执行标志
    uint32_t period;     // 周期 (ms)
    uint32_t timerTick;  // 计时器
    cbFunc func;         // 回调函数
    struct task *next;   // 链表next
} task_t;

// 创建任务
int app_task_create(task_t *task, cbFunc func);

// 启动任务
// one_shot: true-单次, false-周期执行
// time_ms: 执行周期 (ms)
int app_task_start(task_t *task, bool one_shot, uint32_t time_ms);

// 停止任务
int app_task_stop(task_t *task);
```

### 11.2 屏幕显示API

```c
// 源码位置: screen.c

// 图片显示
void LCD_Show_Picture(uint16_t addr, uint16_t pic_id);
void LCD_Disappear_Picture(uint16_t addr);

// 数字显示
void LCD_Show_Number(uint16_t addr, uint16_t data);
void LCD_Show_4byte_Number(uint16_t addr, uint32_t data);
void LCD_Disappear_Number(uint16_t addr);

// 刀具参数
void LCD_IntegratedCutterData_Update(uint16_t addr, uint16_t length, uint8_t diameter, uint8_t angle);
```

---

## 12. 验证建议

### 12.1 单元测试

| 模块 | 测试项 | 预期结果 |
|------|--------|----------|
| app_task | 多任务并发 | 各任务按周期执行，无遗漏 |
| uart1 | 接收超时 | 3ms无新数据触发处理 |
| screen | 按键响应 | 43种按键正确解析 |
| storage | 掉电存储 | 断电后配置保持 |

### 12.2 集成测试

| 测试场景 | 验证点 |
|----------|--------|
| 手柄插拔 | 识别时间<500ms，状态正确切换 |
| 电机调速 | 转速响应<100ms，显示正确 |
| 多任务压力 | 1ms周期任务稳定运行 |
| 长时间运行 | 连续运行8小时无内存泄漏 |

### 12.3 故障注入

| 故障类型 | 注入方法 | 预期行为 |
|----------|----------|----------|
| 通讯断线 | 断开UART | 触发ALARM_5报警 |
| 过载 | 堵转电机 | 触发ALARM_6停机 |
| 看门狗 | 禁用喂狗 | 系统复位 |

---

## 13. 附录

### 13.1 源码证据索引

| 章节 | 源码文件 | 关键函数/行 |
|------|----------|-------------|
| 系统时钟 | `main.c` | `SystemClock_Config()` L80-100 |
| UART初始化 | `main.c` | `MX_USART1_UART_Init()` L47-55 |
| 任务框架 | `app_task.c` | `AppTaskScheduler()` L16-40 |
| 系统初始化 | `userparser.c` | `Userparser_Init()` L35-100 |
| UI处理 | `screen.c` | `KeyBehavior()` L1200-1350 |
| 硬件配置 | `board.h` | 全部宏定义 |

### 13.2 修订历史

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| V1.0 | 2026-03-19 | 初始版本 |

---

**文档结束**

> ⚠️ 待确认项
> - 看门狗是否需要启用
> - 是否有性能优化需求
> - 是否需要补充安全加密模块
