# handlescan 通道上下文集中实施计划

> 本计划只执行已批准设计中的阶段 2.1：集中 A/B 通道私有持久状态，不合并两份主状态机，不改变产品行为。

**目标：** 把 handlescan.c 中分散的 A/B 私有持久状态分别收进两个文件私有上下文，保留现有控制流、硬件映射、异步插拔链和任务模型。

**实现边界：**

- 只修改 User/Application/Handle/handlescan.c。
- 新增一个文件私有 HandlescanChannelContext，每个通道各一份静态实例。
- 每个实例包含 19 个持久字段：18 个文件静态状态，加上原来藏在 A/B 主函数内的独立 XUYAOrfid_flag。
- 不给上下文加入 GPIO、I2C 函数指针、外部消息指针、通道号或事件码；固定绑定留到阶段 2.2 再决定。
- 不新增 getter、setter、通用服务层或运行时初始化函数。
- 不修改 handlescan.h、EIDE/Keil 源文件清单、协议、UI 地址和 EEPROM 页面布局。

**行为不变量：**

- HandlescanA_Fun_SSC()、HandlescanB_Fun_SSC()及外部链接名称不变。
- Handlescan_Fun()仍严格执行 A → B → 临时屏幕报警计时。
- 手柄扫描任务仍为独立静态线程，周期仍为 10ms。
- A 继续使用 PD1、I2C2、A 识别缓存和 A 插拔事件；B 继续使用 PD0、I2C3、B 识别缓存和 B 插拔事件。
- 插入 50ms、拔出 500ms、认证前 200ms、快速重试 200ms、失败自恢复 1000ms、RFID 等待 900ms 等时序不变。
- 连续独立 if 和全部 return 顺序不变，不能改成 else if、switch 或循环。
- A 普通 EEPROM 上线时先清 auto_identify 再入队，B 先入队再清 auto_identify；保留这一既有差异。
- s_transient_screen_alarm_ticks/value、ChannelrecognizeMessageA/B、MemoryMsgA/B、WorkMessage 和 paoxueSpeciValue_A/B 继续由原位置持有。
- Handlescan_HandleRunningPlugAlarm()及已注释调用不在本阶段处理。

## 任务 1：建立结构迁移红灯校验

**文件：**

- 新建临时文件：verify_handlescan_context.py
- 基线：提交 c5a1f7f

脚本检查：

1. 19 个字段及 A/B 两个文件静态实例存在。
2. 旧 18 对文件静态符号和 XUYAOrfid_flag 全部从源码消失。
3. 将新成员访问映射回旧符号、移除新旧状态声明区和旧函数静态声明后，函数体 token 与 c5a1f7f 完全一致。
4. handlescan.h 与基线逐字节一致。
5. A → B → 临时报警顺序、10ms 周期和 UTF-8 BOM/CRLF 不变。

先执行：

~~~powershell
py -3 .\verify_handlescan_context.py
~~~

预期：因为上下文尚不存在而失败。若此时通过，说明脚本没有覆盖目标，停止实施。

## 任务 2：集中 19 个通道持久字段

**文件：**

- 修改：User/Application/Handle/handlescan.c

在 HandlescanDebounce 之后定义私有结构：

~~~c
typedef struct
{
    HandlescanStage stage;
    HandlescanDebounce debounce;
    uint8_t verify_start_wait_ticks;
    uint8_t last_alarm;
    uint8_t verify_retry_count;
    uint16_t verify_retry_wait_ticks;
    uint8_t info_buf[HANDLESCAN_INFO_SIZE];
    uint8_t tool_info_buf[HANDLESCAN_TOOL_INFO_SIZE];
    uint8_t initial_info_buf[AT24CS32_PAGE_SIZE];
    uint8_t speed_step_buf[AT24CS32_PAGE_SIZE];
    HandlescanToolSource tool_source;
    uint16_t rfid_wait_ticks;
    uint16_t rfid_monitor_ticks;
    uint16_t rfid_last_sequence;
    uint16_t rfid_last_presence_sequence;
    uint8_t rfid_miss_count;
    uint8_t rfid_monitor_pending;
    uint8_t rfid_tool_online;
    uint8_t rfid_wait_started;
} HandlescanChannelContext;
~~~

每个字段添加直白中文注释，说明计数单位、缓存页或影响的运行阶段。

建立两份静态实例：

~~~c
static HandlescanChannelContext s_a_context;
static HandlescanChannelContext s_b_context;
~~~

不写聚合显式初始化。HANDLESCAN_STAGE_IDLE 和 HANDLESCAN_TOOL_SOURCE_EEPROM_PAGE3 当前都明确为 0，静态存储零初始化与旧行为相同，并可避免四组 EEPROM 缓冲区整体进入 RW 初始化区。

机械替换：

- s_a_stage → s_a_context.stage，其余 A 字段同理。
- s_b_stage → s_b_context.stage，其余 B 字段同理。
- s_handleA_debounce/s_handleB_debounce 分别进入 A/B 的 debounce。
- A/B 主函数内的 XUYAOrfid_flag 分别进入对应实例的 rfid_wait_started，并删除两条函数静态声明。

只改存储名称，不改函数签名、参数、语句、分支、调用顺序或业务时序。

## 任务 3：静态等价校验

执行：

~~~powershell
py -3 .\verify_handlescan_context.py
git diff --check
git diff -- User/Application/Handle/handlescan.c
~~~

预期脚本输出：

~~~text
PASS: handlescan A/B 私有持久状态已集中，基线控制流和公共接口保持不变
~~~

人工确认差异只有私有结构、两个实例、成员访问和两个函数静态声明删除。

## 任务 4：AC5/EIDE 全量构建和 map 核对

执行：

~~~powershell
& 'C:\Users\Dell\.vscode\extensions\cl.eide-3.27.2\res\tools\win32\unify_builder\unify_builder.exe' `
  -p .\EIDE\build\MainCtrlF413MXOs\builder.params `
  --rebuild `
  --no-color
~~~

验收：

- exit code 0、build successfully、0 error，warning 不超过 22。
- HANDLESCANTaskHandle 仍为 4240B，只保留原有独立任务栈。
- A/B 主函数都存在且尺寸相同；若不再是基线 1828B，先核对反汇编。
- 两个 context 尺寸相同、自然对齐；旧状态符号和两个函数静态标志消失。
- handlescan.o 的 RW+ZI 不超过基线 4481B。
- 整机基线 RO=106692、RW=130896、ROM=107196 不增加；若变化必须定位具体符号。
- Removing Unused 不新增上下文、缓冲区、A/B 入口或业务函数；允许保留既有 Handlescan_HandleRunningPlugAlarm() 32B 裁剪记录。

## 任务 5：Keil 全量构建

使用 C:\Keil_v5\UV4\UV4.exe 无界面构建 MDK-ARM\MainCtrlF413MXOs.uvprojx，日志写入系统临时目录。

验收：

- 完整日志存在 Build Time Elapsed:。
- 日志报告 0 Error(s)，warning 不超过 22。
- 本机 µVision 可能在 0 error 时返回进程码 1，以完整日志为最终判据并如实记录进程码。
- 构建后恢复 Keil 跟踪输出，只保留本阶段源码和正式计划差异。

## 任务 6：清理并交付实机测试

删除：

- verify_handlescan_context.py。
- 本轮临时构建日志、锁文件、空 .d 文件和空占位文件。
- 临时工作记录恢复到提交前内容，不进入本阶段差异。

最终执行 git status --short --untracked-files=all 和 git diff --check。

交付 EIDE 生成的 HEX，并等待用户完成 A/B 单插、同时插入、I2C 单路失败、RFID 有无标签/掉标签、双通道报警继承、运行中拔出当前/非当前通道等实机测试。只有用户明确确认无问题后，才提交本阶段 Git。

## 实机确认

- 2026-07-12：用户确认本阶段烧录测试没有问题。
- 本阶段可以提交 Git；后续辅助函数参数收敛必须作为新的独立烧录版本实施。
