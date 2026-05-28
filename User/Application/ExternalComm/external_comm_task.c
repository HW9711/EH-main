#include "external_comm_task.h"

#include "external_comm_protocol.h"

#include "at24cs32.h"
#include "bsp_uart.h"
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "sscDRIVE.h"
#include "uart2.h"

#include <string.h>

#define EXTERNAL_COMM_TASK_PERIOD_MS        10U     /* 外部通信任务 10ms 调度一次，用于接收 UART2 空闲包。 */
#define EXTERNAL_COMM_HEARTBEAT_PERIOD_MS   100U   /* 心跳 100ms 主动上传一次，可按现场需求单独改宏。 */
#define EXTERNAL_COMM_LINK_STOP_OUTPUT_TIMEOUT_MS 1000U  /* 外控链路静默 1s 后只停电机和泵输出，保留外控授权；上位机需按 300ms 周期下发保活。 */
#define EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS     5000U /* 外控链路静默 30s 后才释放外控授权，避免短暂上行/ACK 抖动把上位机踢出外控。 */
#define EXTERNAL_COMM_HEARTBEAT_USE_UART10  0U      /* 心跳发送串口开关：1 表示从 UART10 发出，0 表示从原 UART2 发出。 */
#define EXTERNAL_COMM_UART5_INJECT_PUMP_FIXED_ENABLE 1U /* UART5 A 泵固定识别开关：1 表示按交付配置识别为注水泵，0 表示使用真实识别结果。 */
#define EXTERNAL_COMM_UART5_INJECT_PUMP_FOLLOW_HANDLE_ENABLE 1U /* 注水泵跟随手柄开关：1 表示手柄转动时注水泵同步运行用于冷却，0 表示只允许上位机独立控制。 */
#define EXTERNAL_COMM_UART5_PUMP_FIXED_TYPE    INJECTWATER /* 固定识别打开时，UART5 A 泵类型固定成注水泵，PUMPA 任务按注水方向和流量公式输出。 */
#define EXTERNAL_COMM_PUMPB_INJECT_PUMP_FIXED_ENABLE 1U /* B 泵固定识别开关：1 表示按交付配置识别为注水泵，便于无 CS1237 霍尔识别时保持 B 通道可控。 */
#define EXTERNAL_COMM_PUMPB_PUMP_FIXED_TYPE    INJECTWATER /* B 泵固定识别打开时，业务类型固定成注水泵，PUMPB 任务按注水方向和流量公式输出。 */
#define EXTERNAL_COMM_RX_FIFO_SIZE       (UART2_MAX_PACKET_SIZE * 4U) /* UART2 外控软件接收 FIFO 容量，保留多包粘包和半包缓存空间。 */
#define EXTERNAL_COMM_FRAME_HEAD_SIZE    4U      /* 外部通信帧头固定 4 字节：D7 CA F8 F1。 */
#define EXTERNAL_COMM_FRAME_LENGTH_OFFSET 5U     /* Length_H 在帧内偏移 5，Length_L 在偏移 6。 */
#define EXTERNAL_COMM_RX_FIFO_MAX_STEPS  32U     /* 单个 10ms 周期最多处理 32 次 FIFO 状态，避免异常噪声长期占用任务。 */

#define EXTERNAL_COMM_STATUS_ONLINE         0x01U   /* 心跳状态值：设备在线。 */
#define EXTERNAL_COMM_STATUS_OFFLINE        0xFFU   /* 心跳状态值：设备掉线、未选中或无效。 */
#define EXTERNAL_COMM_STATUS_SELECTED_A     0x01U   /* 心跳当前通道字段：A 通道被选中。 */
#define EXTERNAL_COMM_STATUS_SELECTED_B     0x02U   /* 心跳当前通道字段：B 通道被选中。 */
#define EXTERNAL_COMM_STATUS_STANDBY        0x01U   /* 心跳运行字段：当前手柄待机。 */
#define EXTERNAL_COMM_STATUS_RUNNING        0x02U   /* 心跳运行字段：当前手柄运行中。 */
#define EXTERNAL_COMM_STATUS_UNPLUGGED      0x03U   /* 心跳运行字段：当前选中通道未接入手柄。 */
#define EXTERNAL_COMM_HANDLE_MODE_FORWARD   0x01U   /* 心跳手柄工作模式：当前手柄正转。 */
#define EXTERNAL_COMM_HANDLE_MODE_REVERSE   0x02U   /* 心跳手柄工作模式：当前手柄反转。 */
#define EXTERNAL_COMM_HANDLE_MODE_OSC       0x03U   /* 心跳手柄工作模式：当前手柄往复。 */
#define EXTERNAL_COMM_HANDLE_MODE_UNKNOWN   0xFFU   /* 心跳手柄工作模式：当前没有可识别方向。 */
#define EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN 44U    /* 新版心跳 InforArea 最大长度：43 字节旧状态 + 1 字节当前手柄工作模式。 */
#define EXTERNAL_COMM_RUNNING_INFO_ALARM    0x05U   /* 主机运行内容上传的信息码：0x05 表示报警信息，InforArea[0] 放 WorkMessage.alarm_value。 */

#define EXTERNAL_COMM_REASON_BAD_LENGTH     0x01U   /* 失败原因：InforArea 长度不符合命令要求。 */
#define EXTERNAL_COMM_REASON_BAD_AREA       0x02U   /* 失败原因：AreaCode 不在当前命令允许范围内。 */
#define EXTERNAL_COMM_REASON_NO_CHANNEL     0x03U   /* 失败原因：当前没有选中 A/B 通道。 */
#define EXTERNAL_COMM_REASON_DEVICE_FAIL    0x04U   /* 失败原因：底层设备读写失败或设备未配置。 */
#define EXTERNAL_COMM_REASON_NOT_SUPPORT    0x05U   /* 失败原因：V1 明确禁用整区读取等大包命令。 */
#define EXTERNAL_COMM_REASON_BUSY           0x06U   /* 失败原因：运行中、报警中或未取得外部控制权。 */

#define EXTERNAL_COMM_DOWN_APPLY_CONTROL    0x01U   /* 下行命令：申请外部控制。 */
#define EXTERNAL_COMM_DOWN_SET_VALUE        0x02U   /* 下行命令：设置速度、频率、泵速度等静态值。 */
#define EXTERNAL_COMM_DOWN_SWITCH_VALUE     0x03U   /* 下行命令：切换通道、方向、控制模式、工具类型。 */
#define EXTERNAL_COMM_DOWN_CONTROL_CMD      0x04U   /* 下行命令：启停泵、启停当前手柄、开口定位、急停。 */
#define EXTERNAL_COMM_DOWN_READ_PAGE        0x05U   /* 下行命令：读取手柄/刀具 EEPROM 单页业务数据。 */
#define EXTERNAL_COMM_DOWN_READ_ALL         0x06U   /* 下行命令：读取整区业务数据，V1 禁用。 */
#define EXTERNAL_COMM_DOWN_WRITE_PAGE       0x07U   /* 下行命令：写入手柄/刀具 EEPROM 单页业务数据。 */
#define EXTERNAL_COMM_DOWN_READ_NAV_PAGE    0x08U   /* 下行命令：读取导航 EEPROM 单页数据。 */
#define EXTERNAL_COMM_DOWN_READ_NAV_ALL     0x09U   /* 下行命令：读取导航整区数据，V1 禁用。 */
#define EXTERNAL_COMM_DOWN_WRITE_NAV_PAGE   0x0AU   /* 下行命令：写入导航 EEPROM 单页数据。 */
#define EXTERNAL_COMM_DOWN_HOST_EXIT        0xBBU   /* 下行命令：上位机主动退出外部控制，释放互斥控制权。 */
#define EXTERNAL_COMM_DOWN_PERMISSION       0xFAU   /* 下行命令：功能升级或权限开放。 */

typedef struct
{
    uint16_t read_index;                             /* FIFO 读指针，指向下一字节待解析数据。 */
    uint16_t write_index;                            /* FIFO 写指针，指向下一字节写入位置。 */
    uint8_t ready;                                   /* FIFO 初始化完成标志，避免任务启动前误读未准备好的缓存。 */
} ExternalCommRxFifo_t;

static kernel_task_t ExternalCommTaskHandle;         /* 外部通信任务句柄，由调度器保存任务状态。 */
static uint16_t s_heartbeat_elapsed_ms = 0U;         /* 心跳累计时间，每次任务运行增加 10ms。 */
static uint16_t s_external_link_elapsed_ms = 0U;     /* 外控保活计时，外控期间每收到一帧合法下行命令都会清零。 */
static uint8_t s_external_link_output_stopped = 0U;  /* 外控链路短超时停输出锁存，防止静默期间每 10ms 重复清运行状态。 */
static uint8_t s_last_alarm_value = 0xFFU;           /* 上一次已经上传给上位机的报警码，初始值故意设为 0xFF，确保启动后先同步一次当前报警状态。 */
static uint8_t s_alarm_report_ready = 0U;            /* 报警上传初始化标志，0 表示还没有向上位机同步过 WorkMessage 报警状态。 */
static uint8_t s_transient_alarm_value = 0U;         /* 运行中另一路手柄校验失败时临时上传的报警码，不写入 WorkMessage。 */
static uint16_t s_transient_alarm_remaining_ms = 0U; /* 临时报警剩余保持时间，递减到 0 后自动上传无报警关闭上位机弹窗。 */
static uint8_t s_uart5_pump_manual_run_request = 0U; /* 上位机独立启动 A 泵的请求锁存，停止 A 泵或急停时清零。 */
static uint8_t s_uart5_inject_pump_follow_run_request = 0U; /* 手柄运行触发的注水泵冷却跟随请求，手柄停止或急停时清零。 */

static uint8_t s_rx_buf[UART2_MAX_PACKET_SIZE];      /* UART2 DMA 空闲包复制到这里后再解析。 */
static ExternalCommRxFifo_t s_rx_fifo;               /* UART2 外控软件接收 FIFO 句柄，保存读写指针和初始化状态。 */
static uint8_t s_rx_fifo_buf[EXTERNAL_COMM_RX_FIFO_SIZE]; /* FIFO 实际存储区，使用本文件静态数组，不依赖额外工程源文件。 */
static uint8_t s_frame_buf[EXTERNAL_COMM_MAX_FRAME_SIZE]; /* 从 FIFO 中临时取出的单帧缓存，交给现有协议解析器复用。 */
static uint8_t s_tx_buf[EXTERNAL_COMM_MAX_FRAME_SIZE]; /* 所有上传帧共用发送缓存，任务内串行使用。 */
static uint8_t s_page_buf[AT24CS32_PAGE_SIZE];       /* EEPROM 页缓存，32 字节含最后 2 字节页校验。 */
static const uint8_t s_external_comm_frame_head[EXTERNAL_COMM_FRAME_HEAD_SIZE] = {0xD7U, 0xCAU, 0xF8U, 0xF1U}; /* FIFO 中搜索完整帧时使用的固定帧头。 */

static void ExternalComm_ResetLinkWatchdog(void);    /* 外控保活计时清零入口，申请外控和收到合法下行帧时复用。 */

static uint16_t ExternalComm_ReadBE16(const uint8_t *data)
{
    /* data[0] 是协议高字节，data[1] 是协议低字节。 */
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void ExternalComm_WriteBE16(uint8_t *data, uint16_t value)
{
    /* 高字节先写，保证回包里的多字节值和协议示例一致。 */
    data[0] = (uint8_t)(value >> 8);
    /* 低字节后写，只保留 value 的低 8 位。 */
    data[1] = (uint8_t)(value & 0xFFU);
}

static void ExternalComm_SendFrame(uint8_t fun_code,
                                   uint8_t area_code,
                                   uint8_t info_code,
                                   const uint8_t *info_area,
                                   uint16_t info_len)
{
    /* tx_len 接收协议层实际组出的完整帧长度。 */
    uint16_t tx_len = 0U;

    /* 所有主动上传帧的 TranCode 固定为 0x01。 */
    if (ExternalCommProtocol_BuildFrame(EXTERNAL_COMM_TRAN_UPLOAD,
                                        fun_code,
                                        area_code,
                                        info_code,
                                        info_area,
                                        info_len,
                                        s_tx_buf,
                                        sizeof(s_tx_buf),
                                        &tx_len) == EXTERNAL_COMM_BUILD_OK)
    {
        /* 组帧成功才占用 UART2 发送，避免发送半成品帧。 */
        Uart2_SendPacket(s_tx_buf, tx_len);
    }
}

static void ExternalComm_SendAck(uint8_t ack_code, const uint8_t *ack_info, uint16_t ack_info_len)
{
    /* tx_len 接收 0xDD 应答帧长度。 */
    uint16_t tx_len = 0U;

    /* ack_code 放在 InforCode，ack_info 放在 InforArea。 */
    if (ExternalCommProtocol_BuildAck(ack_code,
                                      ack_info,
                                      ack_info_len,
                                      s_tx_buf,
                                      sizeof(s_tx_buf),
                                      &tx_len) == EXTERNAL_COMM_BUILD_OK)
    {
        /* 应答帧构造成功后立即从 UART2 发回外部设备。 */
        Uart2_SendPacket(s_tx_buf, tx_len);
    }
}

static void ExternalComm_SendFailAck(uint8_t ack_code, uint8_t value_code, uint8_t reason)
{
    /* 失败载荷固定 2 字节：值代号/命令代号 + 失败原因。 */
    uint8_t info[2];

    /* 第 1 字节告诉外部设备哪个 AreaCode 或 FunCode 失败。 */
    info[0] = value_code;
    /* 第 2 字节告诉外部设备失败原因，便于现场调试。 */
    info[1] = reason;
    /* 失败也统一走 0xDD 应答帧。 */
    ExternalComm_SendAck(ack_code, info, sizeof(info));
}

static void ExternalComm_SendAlarmInfoIfChanged(void)
{
    /* alarm_value 是协议 InforArea[0]，0 表示无报警，非 0 表示固件当前 WorkMessage 报警码。 */
    uint8_t alarm_value;

    /* 全局报警优先级最高，一旦存在真实系统报警，临时报警必须让位，避免覆盖持续故障。 */
    if (WorkMessage.alarm_flag == true)
    {
        s_transient_alarm_value = 0U;                 /* 清掉临时报警值，后续只以上报真实报警为准。 */
        s_transient_alarm_remaining_ms = 0U;          /* 清掉临时报警计时，防止真实报警解除后旧弹窗再次出现。 */
        alarm_value = WorkMessage.alarm_value;        /* 有报警时直接复用 WorkMessage.alarm_value，保持蜂鸣、屏幕和上位机一致。 */
    }
    else if (s_transient_alarm_remaining_ms > 0U)
    {
        alarm_value = s_transient_alarm_value;        /* 无真实报警但临时报警未超时，继续让上位机显示本次提示。 */
        if (s_transient_alarm_remaining_ms > EXTERNAL_COMM_TASK_PERIOD_MS)
        {
            s_transient_alarm_remaining_ms = (uint16_t)(s_transient_alarm_remaining_ms - EXTERNAL_COMM_TASK_PERIOD_MS); /* 每个通信周期扣减 10ms。 */
        }
        else
        {
            s_transient_alarm_remaining_ms = 0U;      /* 最后一个周期扣到 0，下个周期会上传 0 关闭弹窗。 */
        }
    }
    else
    {
        alarm_value = 0U;                             /* 无真实报警且无临时报警时，协议报警码固定为 0。 */
    }

    /* 状态未变化时不重复刷报警帧，避免串口日志被同一个报警码持续淹没。 */
    if ((s_alarm_report_ready != 0U) && (s_last_alarm_value == alarm_value))
    {
        /* 已经同步过同一个报警值，本周期无需发送。 */
        return;
    }

    /* 记录已完成首次同步，后续只在报警码变化时再发。 */
    s_alarm_report_ready = 1U;
    /* 保存本次上传值，下一周期用来判断是否变化。 */
    s_last_alarm_value = alarm_value;
    /* 按协议上传主机运行内容：FunCode=0x03、AreaCode=0xFF、InforCode=0x05、InforArea=报警码。 */
    ExternalComm_SendFrame(EXTERNAL_COMM_FUNC_HOST_RUNNING,
                           EXTERNAL_COMM_AREA_NONE,
                           EXTERNAL_COMM_RUNNING_INFO_ALARM,
                           &alarm_value,
                           1U);
}

/*
 * 函数功能：上传一个不写入 WorkMessage 的限时报警码，用于运行中另一路手柄校验失败提示。
 * 输入参数：alarm_value 需要上位机显示的报警码；hold_ms 保持时间，单位毫秒。
 * 返回参数：无。
 */
void ExternalComm_SendTransientAlarm(uint8_t alarm_value, uint16_t hold_ms)
{
    if ((alarm_value == 0U) || (hold_ms == 0U))
    {
        return;                                      /* 空报警或 0 时长没有意义，保持当前通信报警状态不变。 */
    }

    s_transient_alarm_value = alarm_value;           /* 记录临时报警码，通信任务下一周期会上传给上位机。 */
    s_transient_alarm_remaining_ms = hold_ms;        /* 记录保持时间，到期后由 ExternalComm_SendAlarmInfoIfChanged 自动发 0。 */
    s_alarm_report_ready = 0U;                       /* 强制下一周期立即重发，避免同码临时报警被变化检测吞掉。 */
}

static uint8_t ExternalComm_IsAuthorizedCode(const uint8_t *code, uint16_t code_len)
{
    /* V1 不校验具体 8 字节内容，先显式标记参数暂未使用。 */
    (void)code;

    /*
     * V1 先按需求固定放行 8 字节注册码/权限码。
     * 后续如需改为固定码或 EEPROM Page1 校验，只替换这个函数即可。
     */
    return (code_len == 8U) ? 1U : 0U;
}

static uint8_t ExternalComm_CurrentBusIsI2C3(uint8_t *use_i2c3)
{
    /* use_i2c3 是输出参数，用来告诉读写函数选择 I2C2 还是 I2C3。 */
    if (use_i2c3 == NULL)
    {
        return 0U;
    }

    /* 当前选中 A 通道时，按硬件约定读写 I2C2 上的手柄 EEPROM。 */
    if (WorkMessage.channel_work == CHANNEL_A)
    {
        *use_i2c3 = 0U;
        return 1U;
    }

    /* 当前选中 B 通道时，按硬件约定读写 I2C3 上的手柄 EEPROM。 */
    if (WorkMessage.channel_work == CHANNEL_B)
    {
        *use_i2c3 = 1U;
        return 1U;
    }

    /* 未选中 A/B 时不能判断目标 EEPROM，调用者回失败应答。 */
    return 0U;
}

static uint8_t ExternalComm_ReadCurrentPage(uint16_t page_index, uint8_t *page_buf)
{
    /* use_i2c3 非 0 表示目标 EEPROM 在 I2C3，否则在 I2C2。 */
    uint8_t use_i2c3;

    /* 先按当前选中通道确定 EEPROM 总线，避免上位机直接指定硬件总线。 */
    if (ExternalComm_CurrentBusIsI2C3(&use_i2c3) == 0U)
    {
        return 0U;
    }

    /* B 通道走 I2C3，A 通道走 I2C2，页校验由 AT24CS32 驱动内部完成。 */
    return (use_i2c3 != 0U) ?
           AT24CS32_ReadPage_I2C3(page_index, page_buf) :
           AT24CS32_ReadPage_I2C2(page_index, page_buf);
}

static uint8_t ExternalComm_WriteCurrentPage(uint16_t page_index, const uint8_t *page_buf)
{
    /* use_i2c3 非 0 表示目标 EEPROM 在 I2C3，否则在 I2C2。 */
    uint8_t use_i2c3;

    /* EEPROM 写入同样只跟随当前选中通道。 */
    if (ExternalComm_CurrentBusIsI2C3(&use_i2c3) == 0U)
    {
        return 0U;
    }

    /* 驱动写页前会刷新最后 2 字节页校验，上位机只需要给 30 字节有效数据。 */
    return (use_i2c3 != 0U) ?
           AT24CS32_WritePage_I2C3(page_index, page_buf) :
           AT24CS32_WritePage_I2C2(page_index, page_buf);
}

static uint8_t ExternalComm_MapAreaToPageIndex(uint8_t area_code, uint16_t *page_index)
{
    /* page_index 是输出页下标，调用者传空说明无法返回结果。 */
    if (page_index == NULL)
    {
        return 0U;
    }

    /*
     * 协议 AreaCode 对应 EEPROM 说明中的业务页。
     * Page7 和 Page10 当前为保留页，V1 不单独暴露，避免上位机误写保留区。
     */
    switch (area_code)
    {
        /* 文档 Page2 对应驱动 page_index=1，因为驱动页下标从 0 开始。 */
        case 0x01U: *page_index = 1U;  return 1U; /* Page2：识别信息区 */
        /* 文档 Page3 对应驱动 page_index=2。 */
        case 0x02U: *page_index = 2U;  return 1U; /* Page3：刀具信息区 */
        /* 文档 Page4 对应驱动 page_index=3。 */
        case 0x03U: *page_index = 3U;  return 1U; /* Page4：初始值信息区 */
        /* 文档 Page5 对应驱动 page_index=4。 */
        case 0x04U: *page_index = 4U;  return 1U; /* Page5：按键自定义区 */
        /* 文档 Page6 对应驱动 page_index=5，Page7 保留不暴露。 */
        case 0x05U: *page_index = 5U;  return 1U; /* Page6：多档位调节区 */
        /* 文档 Page8 对应驱动 page_index=7。 */
        case 0x06U: *page_index = 7U;  return 1U; /* Page8：运行信息区 */
        /* 文档 Page9 对应驱动 page_index=8，Page10 保留不暴露。 */
        case 0x07U: *page_index = 8U;  return 1U; /* Page9：外部编辑区 */
        /* 文档 Page11 对应驱动 page_index=10。 */
        case 0x08U: *page_index = 10U; return 1U; /* Page11：出厂信息区 */
        /* 未列出的 AreaCode 都不允许读写。 */
        default: break;
    }

    /* AreaCode 未命中映射表。 */
    return 0U;
}

static uint8_t ExternalComm_MapNavPageToIndex(uint8_t page_code, uint16_t *page_index)
{
    /* page_no 使用 EEPROM 文档里的 1 基页号。 */
    uint16_t page_no;

    /* page_index 是驱动使用的 0 基页下标输出。 */
    if (page_index == NULL)
    {
        return 0U;
    }

    /*
     * 导航区按 Page12~Page128 读取。
     * 兼容两种常见写法：直接发实际页号 12~128，或发导航页序号 1~117。
     */
    if ((page_code >= 12U) && (page_code <= 128U))
    {
        /* 上位机直接给 Page12~Page128 时，不再二次偏移。 */
        page_no = page_code;
    }
    else if ((page_code >= 1U) && (page_code <= 117U))
    {
        /* 上位机给导航区序号 1~117 时，映射到 Page12~Page128。 */
        page_no = (uint16_t)(11U + page_code);
    }
    else
    {
        /* 导航页码超出协议允许范围。 */
        return 0U;
    }

    /* 驱动接口按 0 基页下标访问，所以文档页号减 1。 */
    *page_index = (uint16_t)(page_no - 1U);
    /* 导航页码映射成功。 */
    return 1U;
}

static void ExternalComm_SaveCurrentSpeed(uint16_t speed)
{
    /* 更新当前设置速度，供 UI、心跳和运行任务读取。 */
    WorkMessage.speed_set_work = speed;
    /* 更新当前实际速度，保证外部设置后立即生效。 */
    WorkMessage.speed_work = speed;

    /* 当前通道是 A 时，把速度写回 A 通道记忆结构。 */
    if (WorkMessage.channel_work == CHANNEL_A)
    {
        /* 正转时修改 A 通道正转速度。 */
        if (WorkMessage.dir_work == ZZDIR)
        {
            MemoryMsgA.zz_speed = speed;
        }
        /* 反转时修改 A 通道反转速度。 */
        else if (WorkMessage.dir_work == FZDIR)
        {
            MemoryMsgA.fz_speed = speed;
        }
        /* 往复或未知方向按 A 通道往复速度保存。 */
        else
        {
            MemoryMsgA.osc_speed = speed;
        }
    }
    /* 当前通道是 B 时，把速度写回 B 通道记忆结构。 */
    else if (WorkMessage.channel_work == CHANNEL_B)
    {
        /* 正转时修改 B 通道正转速度。 */
        if (WorkMessage.dir_work == ZZDIR)
        {
            MemoryMsgB.zz_speed = speed;
        }
        /* 反转时修改 B 通道反转速度。 */
        else if (WorkMessage.dir_work == FZDIR)
        {
            MemoryMsgB.fz_speed = speed;
        }
        /* 往复或未知方向按 B 通道往复速度保存。 */
        else
        {
            MemoryMsgB.osc_speed = speed;
        }
    }
}

static void ExternalComm_SaveCurrentFreq(uint16_t freq)
{
    /* 当前工作频率立即更新。 */
    WorkMessage.freq_work = freq;
    /* A 通道选中时同步保存到 A 通道记忆结构。 */
    if (WorkMessage.channel_work == CHANNEL_A)
    {
        MemoryMsgA.freq = freq;
    }
    /* B 通道选中时同步保存到 B 通道记忆结构。 */
    else if (WorkMessage.channel_work == CHANNEL_B)
    {
        MemoryMsgB.freq = freq;
    }
}

static uint8_t ExternalComm_SetDirection(uint8_t dir_value)
{
    /* dir 是项目内部方向值：ZZDIR/FZDIR/OSCDIR。 */
    uint8_t dir;

    /* 兼容外部设备发送 0 表示正转。 */
    if (dir_value == 0U)
    {
        dir = ZZDIR;
    }
    /* 也兼容外部设备按 1/2/3 表示正转/反转/往复。 */
    else if (dir_value <= 3U)
    {
        dir = (uint8_t)(dir_value - 1U);
    }
    else
    {
        /* 其他值不是合法方向。 */
        return 0U;
    }

    /* 更新当前工作方向。 */
    WorkMessage.dir_work = dir;
    /* A 通道选中时更新 A 通道方向和对应速度。 */
    if (WorkMessage.channel_work == CHANNEL_A)
    {
        /* A 通道记忆结构保存方向。 */
        MemoryMsgA.dir = dir;
        /* 正转方向加载 A 正转速度。 */
        if (dir == ZZDIR)
        {
            WorkMessage.speed_set_work = MemoryMsgA.zz_speed;
        }
        /* 反转方向加载 A 反转速度。 */
        else if (dir == FZDIR)
        {
            WorkMessage.speed_set_work = MemoryMsgA.fz_speed;
        }
        /* 往复方向加载 A 往复速度。 */
        else
        {
            WorkMessage.speed_set_work = MemoryMsgA.osc_speed;
        }
    }
    /* B 通道选中时更新 B 通道方向和对应速度。 */
    else if (WorkMessage.channel_work == CHANNEL_B)
    {
        /* B 通道记忆结构保存方向。 */
        MemoryMsgB.dir = dir;
        /* 正转方向加载 B 正转速度。 */
        if (dir == ZZDIR)
        {
            WorkMessage.speed_set_work = MemoryMsgB.zz_speed;
        }
        /* 反转方向加载 B 反转速度。 */
        else if (dir == FZDIR)
        {
            WorkMessage.speed_set_work = MemoryMsgB.fz_speed;
        }
        /* 往复方向加载 B 往复速度。 */
        else
        {
            WorkMessage.speed_set_work = MemoryMsgB.osc_speed;
        }
    }
    else
    {
        /* 当前没有 A/B 通道，方向不能写入任何通道记忆。 */
        return 0U;
    }

    /* 实际输出速度跟随方向切换后的设置速度。 */
    WorkMessage.speed_work = WorkMessage.speed_set_work;
    /* 方向切换成功。 */
    return 1U;
}

static uint8_t ExternalComm_ApplyExternalAuth(const ExternalCommFrame_t *frame)
{
    /* 外部控制申请要求 8 字节注册码，V1 只校验长度。 */
    if (ExternalComm_IsAuthorizedCode(frame->info_area, frame->info_len) == 0U)
    {
        ExternalComm_SendAck(EXTERNAL_COMM_ACK_AUTH_FAILED, NULL, 0U);
        return 0U;
    }

    /* 外控申请必须等待当前脚踏/屏幕/手柄控制结束，不能抢停正在运行的本机来源。 */
    if (ControlArbitration_EnterExternalControl() == false)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_EXTERNAL_FAILED,
                                 EXTERNAL_COMM_DOWN_APPLY_CONTROL,
                                 EXTERNAL_COMM_REASON_BUSY);
        return 0U;
    }
    /* 申请成功后从“已经持有外控锁”的时刻重新计算保活窗口，避免边界时序误释放。 */
    ExternalComm_ResetLinkWatchdog();
    /* 返回外部控制开启成功。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_EXTERNAL_OK, NULL, 0U);
    /* 告诉调用方认证通过。 */
    return 1U;
}

static uint8_t ExternalComm_ApplyPermission(const ExternalCommFrame_t *frame)
{
    /* 权限开放命令同样要求 8 字节权限码，V1 只校验长度。 */
    if (ExternalComm_IsAuthorizedCode(frame->info_area, frame->info_len) == 0U)
    {
        ExternalComm_SendAck(EXTERNAL_COMM_ACK_PERMISSION_FAIL, NULL, 0U);
        return 0U;
    }

    /* V1 暂时全通过，返回权限已开放。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_PERMISSION_OK, NULL, 0U);
    /* 告诉调用方权限校验通过。 */
    return 1U;
}

/* A/B 泵固定注水泵识别逻辑同时服务设置帧、控制帧和心跳帧，提前声明用于后面的入口调用。 */
static void ExternalComm_ApplyFixedPumpIdentity(void);
/* 当前选中手柄在线状态在心跳和启动前置检查中复用，提前声明避免启动逻辑只看通道号。 */
static uint8_t ExternalComm_SelectedHandleOnline(void);

static void ExternalComm_ApplySetting(const ExternalCommFrame_t *frame)
{
    /* value 保存外部设备下发的速度、频率或泵速度。 */
    uint16_t value;
    /* 成功应答载荷固定为 AreaCode + 2 字节值。 */
    uint8_t info[3];

    if (ControlArbitration_IsExternalActive() == false)
    {
        /* 未取得外部控制权时不能写运行值，脚踏/屏幕/手柄占用期间也必须等待。 */
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BUSY);
        return;
    }

    /*
     * 只有“当前手柄转速”和“当前手柄往复频率”依赖 WorkMessage.channel_work。
     * A/B 泵速度写入 pumpMessageA/B.speed_work，属于独立泵状态，不能因为未选中手柄通道而拒绝。
     */
    if (((frame->area_code == 0x01U) || (frame->area_code == 0x02U)) &&
        ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B)))
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_NO_CHANNEL);
        return;
    }

    /* 手柄速度按 WorkMessage.speed_work 的内部 x10 单位解析；A/B 泵速度仍按泵业务流量值解析，三者协议字段都是 2 字节大端。 */
    if ((frame->area_code == 0x01U) || (frame->area_code == 0x03U) || (frame->area_code == 0x04U))
    {
        /* 2 字节参数不足时不能解析。 */
        if (frame->info_len < 2U)
        {
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                     frame->area_code,
                                     EXTERNAL_COMM_REASON_BAD_LENGTH);
            return;
        }
        /* 读取 16 位参数值。 */
        value = ExternalComm_ReadBE16(frame->info_area);
    }
    /* 频率文档宽度不完全明确，V1 兼容 1 字节和 2 字节。 */
    else if (frame->area_code == 0x02U)
    {
        /* 1 字节频率直接扩展为 16 位值。 */
        if (frame->info_len == 1U)
        {
            value = frame->info_area[0];
        }
        /* 2 字节及以上按前两个字节大端解析。 */
        else if (frame->info_len >= 2U)
        {
            value = ExternalComm_ReadBE16(frame->info_area);
        }
        else
        {
            /* 0 字节无法得到频率值。 */
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                     frame->area_code,
                                     EXTERNAL_COMM_REASON_BAD_LENGTH);
            return;
        }
    }
    else
    {
        /* 未定义的静态设置 AreaCode 返回区域错误。 */
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_AREA);
        return;
    }

    /* 按 AreaCode 把解析后的 value 写入对应业务状态。 */
    switch (frame->area_code)
    {
        case 0x01U:
            /* 设置当前手柄速度。 */
            ExternalComm_SaveCurrentSpeed(value);
            break;
        case 0x02U:
            /* 设置当前手柄往复频率。 */
            ExternalComm_SaveCurrentFreq(value);
            break;
        case 0x03U:
            /* 固定识别打开时，设置 A 泵速度会同步补齐 UART5 注水泵在线状态和业务类型。 */
            ExternalComm_ApplyFixedPumpIdentity();
            /* 设置 A 泵速度，不切换泵启停状态；PUMPAehaviors() 后续按该值换算 UART5 驱动数据。 */
            pumpMessageA.speed_work = value;
            break;
        case 0x04U:
            /* 固定识别打开时，设置 B 泵速度会同步补齐 B 泵在线状态和注水泵业务类型。 */
            ExternalComm_ApplyFixedPumpIdentity();
            /* 设置 B 泵速度，不切换泵启停状态。 */
            pumpMessageB.speed_work = value;
            break;
        default:
            /* 前面已过滤非法 AreaCode，这里只保留防御分支。 */
            break;
    }

    /* 应答第 1 字节回显本次设置的 AreaCode。 */
    info[0] = frame->area_code;
    /* 应答后 2 字节回显最终写入值，便于外部设备确认大端顺序。 */
    ExternalComm_WriteBE16(&info[1], value);
    /* 返回运行值设置成功。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_RUN_SET_OK, info, sizeof(info));
}

static void ExternalComm_ApplySwitchSetting(const ExternalCommFrame_t *frame)
{
    /* 成功应答载荷为 AreaCode + 设置值。 */
    uint8_t info[2];
    /* value 保存 InforArea 第 1 字节切换值。 */
    uint8_t value;

    if (ControlArbitration_IsExternalActive() == false)
    {
        /* 上位机没有拿到控制权时不允许切换通道/方向/模式，避免抢占当前控制来源。 */
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BUSY);
        return;
    }

    /* 运行中或报警中禁止切换通道/方向/模式，避免运行链路跳变。 */
    if (WorkMessage.runflag_work || WorkMessage.alarm_flag)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BUSY);
        return;
    }

    /* 没有载荷的切换项按 0 处理，通道切换命令本身不依赖 value。 */
    value = (frame->info_len > 0U) ? frame->info_area[0] : 0U;
    /* AreaCode 决定具体切换项。 */
    switch (frame->area_code)
    {
        case 0x01U:
            /* 切换到 A 通道前必须确认 A 通道在线。 */
            if (WorkMessage.Channel_Aonline == false)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_NO_CHANNEL);
                return;
            }
            /* 把 A 通道记忆状态装载到当前工作状态。 */
            Pubinterface_LoadChannelMemory(CHANNEL_A);
            break;
        case 0x02U:
            /* 切换到 B 通道前必须确认 B 通道在线。 */
            if (WorkMessage.Channel_Bonline == false)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_NO_CHANNEL);
                return;
            }
            /* 把 B 通道记忆状态装载到当前工作状态。 */
            Pubinterface_LoadChannelMemory(CHANNEL_B);
            break;
        case 0x03U:
            /* 方向切换需要至少 1 字节方向值，并且必须能映射到内部方向。 */
            if ((frame->info_len < 1U) || (ExternalComm_SetDirection(value) == 0U))
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_BAD_LENGTH);
                return;
            }
            break;
        case 0x04U:
            /* 控制模式只允许脚踏、手控、外部控制三类内部值。 */
            if ((value != JTWORK) && (value != HANDLEWORK) && (value != TOUCHWORK))
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_BAD_AREA);
                return;
            }
            /* 更新当前控制模式。 */
            WorkMessage.drivetype_work = value;
            /* 当前通道是 A 时保存到 A 通道记忆结构。 */
            if (WorkMessage.channel_work == CHANNEL_A)
            {
                MemoryMsgA.drive_type = value;
            }
            /* 当前通道是 B 时保存到 B 通道记忆结构。 */
            else if (WorkMessage.channel_work == CHANNEL_B)
            {
                MemoryMsgB.drive_type = value;
            }
            break;
        case 0x05U:
            /* 工具类型只接受刨头或磨头。 */
            if ((value != PLANER) && (value != GRINDH))
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_BAD_AREA);
                return;
            }
            /* 更新当前工具类型。 */
            WorkMessage.tool_type = value;
            /* 当前通道是 A 时保存到 A 通道记忆结构。 */
            if (WorkMessage.channel_work == CHANNEL_A)
            {
                MemoryMsgA.tool_type = value;
            }
            /* 当前通道是 B 时保存到 B 通道记忆结构。 */
            else if (WorkMessage.channel_work == CHANNEL_B)
            {
                MemoryMsgB.tool_type = value;
            }
            break;
        default:
            /* 未定义切换项返回 AreaCode 错误。 */
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                     frame->area_code,
                                     EXTERNAL_COMM_REASON_BAD_AREA);
            return;
    }

    /* 应答第 1 字节回显切换项 AreaCode。 */
    info[0] = frame->area_code;
    /* 应答第 2 字节回显切换值；通道切换无载荷时为 0。 */
    info[1] = value;
    /* 返回运行值设置成功。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_RUN_SET_OK, info, sizeof(info));
}

static uint8_t ExternalComm_EnsureActiveForRun(uint8_t allow_emergency_stop)
{
    /* 急停是安全例外，报警中也必须允许执行全停和控制权释放。 */
    if (allow_emergency_stop != 0U)
    {
        return 1U;
    }

    /* 报警状态下禁止外部启动或继续动作，停止类命令也让上层走急停分支。 */
    if (WorkMessage.alarm_flag)
    {
        return 0U;
    }

    /* 非急停命令必须由外控持有者发起，脚踏/屏幕/手柄占用时上位机只能等待。 */
    if (ControlArbitration_IsExternalActive() == false)
    {
        return 0U;
    }

    /* 当前允许执行控制命令。 */
    return 1U;
}

/*
 * 函数功能：刷新 UART5 A 注水泵最终运行状态，并在速度为 0 时使用当前手柄 Page4 默认流量补初始值。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ExternalComm_RefreshUart5PumpRunState(void)
{
    /*
	 * A 泵现在有两个运行来源：
	 * 1. 上位机独立启动 A 泵，用于单独调试注水泵；
	 * 2. 手柄运行时，如果 A 泵是注水泵，则跟随手柄运行给手柄降温。
	 * 本函数只维护请求并集，不直接发送 UART5 泵帧；压力限速和 0 速硬停统一在 PUMPAehaviors() 内输出。
	 */
    uint8_t manual_request = s_uart5_pump_manual_run_request; /* 保存上位机独立控制请求，避免后续表达式重复读全局变量。 */
    uint8_t follow_request = 0U;                              /* 保存手柄冷却跟随请求，只有注水泵类型才允许置 1。 */

    if ((s_uart5_inject_pump_follow_run_request != 0U) &&
        (pumpMessageA.type == INJECTWATER))
    {
        follow_request = 1U; /* 当前 A 泵确认为注水泵，手柄运行时才允许它进入冷却跟随运行。 */
    }

	if ((manual_request != 0U) || (follow_request != 0U))
	{
#if (EXTERNAL_COMM_UART5_INJECT_PUMP_FIXED_ENABLE == 1U)
		if ((pumpMessageA.type == INJECTWATER) && (pumpMessageA.speed_work == 0U))
		{
            pumpMessageA.speed_work = Pubinterface_GetCurrentDefaultInjectionFlow(); /* 固定注水泵识别时补当前手柄 Page4 默认流量，避免继续使用程序固定值。 */
        }
#endif
        pumpMessageA.run_flag = true;         /* 任一来源请求运行时，A 泵最终运行标志置位。 */
        pumpMessageA.timingDrainage_flag = false; /* 上位机启动和手柄跟随都不是排空模式，必须清掉排空计时。 */
    }
    else
    {
        pumpMessageA.run_flag = false;        /* 两个来源都不请求运行时，才真正停止 A 泵。 */
        pumpMessageA.timingDrainage_flag = false; /* 停止时同步取消排空状态，保证下一周期发送停泵帧。 */
    }
}

static void ExternalComm_SetUart5PumpManualRun(uint8_t enable)
{
    s_uart5_pump_manual_run_request = (enable != 0U) ? 1U : 0U; /* 只改上位机独立运行请求，不直接覆盖手柄冷却跟随请求。 */
    ExternalComm_RefreshUart5PumpRunState();                    /* 按“独立请求 OR 跟随请求”重新计算 A 泵最终 run_flag。 */
}

static void ExternalComm_ClearUart5PumpRunRequests(void)
{
    s_uart5_pump_manual_run_request = 0U;        /* 急停/全停时清除上位机独立运行请求。 */
    s_uart5_inject_pump_follow_run_request = 0U; /* 急停/全停时清除手柄冷却跟随请求。 */
}

static void ExternalComm_ApplyHostExit(void)
{
    uint8_t info[1];

    /* 退出外控时先清除外部通信层自己的 A 泵锁存请求，避免后续刷新又把泵拉起。 */
    ExternalComm_ClearUart5PumpRunRequests();
    /* 释放公共仲裁锁，并停止外控遗留的电机、脚踏标志和 A/B 泵输出。 */
    ControlArbitration_ReleaseExternalControl();
    /* ACK 回显 0xBB，Tools 可据此把“已取得外部控制权”状态清掉。 */
    info[0] = EXTERNAL_COMM_DOWN_HOST_EXIT;
    /* 退出动作本身按控制成功返回，表示 MCU 已经释放外部控制权。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_CONTROL_OK, info, sizeof(info));
}

static void ExternalComm_ResetLinkWatchdog(void)
{
    /* 每收到一帧合法上位机下行命令，都认为 RS485 外控链路仍然存在。 */
    s_external_link_elapsed_ms = 0U;
    /* 链路恢复后允许下一次静默重新触发停输出动作。 */
    s_external_link_output_stopped = 0U;
}

static void ExternalComm_StopOutputForLinkSilent(void)
{
    /* 短超时只处理安全输出，不改变 WorkMessage.hmiactive_work，避免上位机外控状态被误释放。 */
    WorkMessage.runflag_work = false;
    /* 清零当前电机实际输出速度，驱动任务下一周期会按停止状态下发。 */
    WorkMessage.speed_work = 0U;
    /* 清除手柄按键运行标志，避免外控恢复后沿用上一次运行沿。 */
    ControlSignalMessage.handle_control_flag = false;
    /* 清除外控电机运行标志，但保留 HMI_enable_flag 和外控授权状态。 */
    ControlSignalMessage.HMI_control_flag = false;
    /* 清除外控 A 泵运行标志，避免泵任务继续认为外控在请求 A 泵输出。 */
    ControlSignalMessage.HMIL_pump_flag = false;
    /* 清除外控 B 泵运行标志，避免泵任务继续认为外控在请求 B 泵输出。 */
    ControlSignalMessage.HMIR_pump_flag = false;
    /* 清除外部通信层的 A 泵手动/跟随锁存请求，防止后续状态刷新再次拉起注水泵。 */
    ExternalComm_ClearUart5PumpRunRequests();
    /* 停止 A 泵运行，泵任务下一周期会发送停止帧。 */
    pumpMessageA.run_flag = false;
    /* 取消 A 泵排空计时，断线静默时不允许排空动作继续累计。 */
    pumpMessageA.timingDrainage_flag = false;
    /* 清零 A 泵排空累计时间，恢复后必须由新的上位机命令重新开始。 */
    pumpMessageA.timingDrainage_times = 0U;
    /* 清零 A 泵输出速度，避免运行标志恢复前仍保留旧速度。 */
    pumpMessageA.speed_work = 0U;
    /* 停止 B 泵运行，保持两路泵的断线停机动作一致。 */
    pumpMessageB.run_flag = false;
    /* 取消 B 泵排空计时，断线静默时不允许排空动作继续累计。 */
    pumpMessageB.timingDrainage_flag = false;
    /* 清零 B 泵排空累计时间，恢复后必须由新的上位机命令重新开始。 */
    pumpMessageB.timingDrainage_times = 0U;
    /* 清零 B 泵输出速度，避免运行标志恢复前仍保留旧速度。 */
    pumpMessageB.speed_work = 0U;
}

static void ExternalComm_HandleLinkReleaseTimeout(void)
{
    /* 长超时确认上位机或 RS485 已长时间离线，先清外部通信层自己的泵运行请求。 */
    ExternalComm_ClearUart5PumpRunRequests();
    /* 释放外控仲裁锁，并由公共释放函数统一停止电机、A/B 泵和外控显示标志。 */
    ControlArbitration_ReleaseExternalControl();
    /* 超时处理完成后清零计时，避免释放后的空闲状态继续重复进入本函数。 */
    s_external_link_elapsed_ms = 0U;
    /* 外控已经释放，短超时停输出锁存也同步复位。 */
    s_external_link_output_stopped = 0U;
}

static void ExternalComm_CheckLinkWatchdog(void)
{
    /* 只有外部控制权有效时才监控链路；本机脚踏/屏幕/手柄控制不受该超时影响。 */
    if (ControlArbitration_IsExternalActive() == false)
    {
        /* 非外控状态下保持计时清零，下一次申请外控从完整超时时间重新开始。 */
        s_external_link_elapsed_ms = 0U;
        /* 非外控状态下不保留短超时停输出锁存。 */
        s_external_link_output_stopped = 0U;
        return;
    }

    /* 防止计数超过长释放阈值太多后溢出，达到阈值前按 10ms 任务周期累计。 */
    if (s_external_link_elapsed_ms < EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS)
    {
        /* 外控已激活且本周期没有合法下行帧时，累计保活静默时间。 */
        s_external_link_elapsed_ms = (uint16_t)(s_external_link_elapsed_ms + EXTERNAL_COMM_TASK_PERIOD_MS);
    }

    /* 短超时只停输出，不释放外控授权；这一步用于 RS485 真拔线后的安全停机。 */
    if ((s_external_link_elapsed_ms >= EXTERNAL_COMM_LINK_STOP_OUTPUT_TIMEOUT_MS) &&
        (s_external_link_output_stopped == 0U))
    {
        /* 停止电机和 A/B 泵输出，但仍允许后续合法下行帧继续使用当前外控授权。 */
        ExternalComm_StopOutputForLinkSilent();
        /* 锁存本次短超时动作，避免静默期间反复清状态造成调试观察困难。 */
        s_external_link_output_stopped = 1U;
    }

    /* 长超时才认为上位机已经离线，释放外控授权让本机控制可以重新接管。 */
    if (s_external_link_elapsed_ms >= EXTERNAL_COMM_LINK_RELEASE_TIMEOUT_MS)
    {
        ExternalComm_HandleLinkReleaseTimeout();
    }
}

static void ExternalComm_StopAllWork(void)
{
    /* 清除当前手柄运行标志。 */
    WorkMessage.runflag_work = false;
    /* 清零实际输出速度，驱动任务下一周期会发送停止。 */
    WorkMessage.speed_work = 0U;
    /* 清除手柄按键控制标志。 */
    ControlSignalMessage.handle_control_flag = false;
    /* 清除外部控制启动标志。 */
    ControlSignalMessage.HMI_control_flag = false;
    /* 急停后清除外控激活显示，系统回到无控制来源状态。 */
    WorkMessage.hmiactive_work = 0U;
    /* 急停后清除触控占用显示，避免屏幕触控残留继续挡住其它来源。 */
    WorkMessage.touchactive_work = 0U;
    /* 清除左脚踏控制标志。 */
    ControlSignalMessage.jtL_control_flag = false;
    /* 清除右脚踏控制标志。 */
    ControlSignalMessage.jtR_control_flag = false;
    /* 急停属于最高优先级停机，需要同时清除 A 泵的独立控制请求和手柄跟随请求。 */
    ExternalComm_ClearUart5PumpRunRequests();
    /* 停止 A 泵运行。 */
    pumpMessageA.run_flag = false;
    /* 取消 A 泵排空计时。 */
    pumpMessageA.timingDrainage_flag = false;
    /* 清零 A 泵速度输出。 */
    pumpMessageA.speed_work = 0U;
    /* 停止 B 泵运行。 */
    pumpMessageB.run_flag = false;
    /* 取消 B 泵排空计时。 */
    pumpMessageB.timingDrainage_flag = false;
    /* 清零 B 泵速度输出。 */
    pumpMessageB.speed_work = 0U;
    /* 急停是安全例外，允许强制释放当前任意控制来源。 */
    ControlArbitration_ForceRelease();
}

/*
 * 函数功能：按交付配置补齐固定注水泵身份，并在泵速度为 0 时使用当前手柄 Page4 默认流量初始化。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ExternalComm_ApplyFixedPumpIdentity(void)
{
#if (EXTERNAL_COMM_UART5_INJECT_PUMP_FIXED_ENABLE == 1U)
    /* 固定识别打开时，固定 UART5 对应的 A 泵在线状态，避免单独测泵或手柄联调被 CS1237 霍尔设备码卡住。 */
    pumpMessageA.online_flag = true;
    /* 固定识别打开时固定 A 泵为注水泵，PUMPAehaviors() 后续按注水泵方向和公式换算 UART5 驱动数据。 */
    pumpMessageA.type = EXTERNAL_COMM_UART5_PUMP_FIXED_TYPE;
    /* 如果上位机还没有单独下发 A 泵速度，则补当前手柄 Page4 默认流量，避免手柄跟随或单独启动时 UART5 输出仍为 0。 */
    if (pumpMessageA.speed_work == 0U)
    {
        /* 默认流量只在 0 时写入；已经通过 AreaCode=03 设置过速度时，不覆盖用户下发值。 */
        pumpMessageA.speed_work = Pubinterface_GetCurrentDefaultInjectionFlow();
    }
    /* 清零识别丢失计数，让心跳和后续控制都看到 A 泵处于稳定识别状态。 */
    pumpMessageA.losses_times = 0U;
#endif

#if (EXTERNAL_COMM_PUMPB_INJECT_PUMP_FIXED_ENABLE == 1U)
    /* 固定识别打开时，固定 B 泵在线状态，便于没有接入 B 路 CS1237 霍尔识别时保持 B 泵通道可控。 */
    pumpMessageB.online_flag = true;
    /* 固定识别打开时固定 B 泵为注水泵，PUMPBBehaviors() 后续按注水泵方向和公式换算输出。 */
    pumpMessageB.type = EXTERNAL_COMM_PUMPB_PUMP_FIXED_TYPE;
    /* 如果上位机还没有单独下发 B 泵速度，则补当前手柄 Page4 默认流量，避免已启动但输出仍为 0。 */
    if (pumpMessageB.speed_work == 0U)
    {
        /* 默认流量只在 0 时写入；已经通过 AreaCode=04 设置过速度时，不覆盖用户下发值。 */
        pumpMessageB.speed_work = Pubinterface_GetCurrentDefaultInjectionFlow();
    }
    /* 清零识别丢失计数，让心跳和后续控制都看到 B 泵处于稳定识别状态。 */
    pumpMessageB.losses_times = 0U;
#endif
}

static void ExternalComm_SetUart5InjectPumpFollow(uint8_t enable)
{
#if (EXTERNAL_COMM_UART5_INJECT_PUMP_FOLLOW_HANDLE_ENABLE == 1U)
    /* 跟随手柄开关打开时，先按固定识别开关补齐 A 泵身份；固定识别关闭时该函数不改真实识别结果。 */
    ExternalComm_ApplyFixedPumpIdentity();

    /* 只有注水泵需要跟随手柄；抽水泵和灌注泵即使在线，也不参与当前手柄启停联动。 */
    if (pumpMessageA.type != INJECTWATER)
    {
        s_uart5_inject_pump_follow_run_request = 0U; /* 当前不是注水泵时清掉跟随请求，避免后续类型变化后误启动。 */
        ExternalComm_RefreshUart5PumpRunState();     /* 刷新最终运行状态；若上位机独立启动仍有效，则 A 泵继续按独立请求运行。 */
        return;
    }

    s_uart5_inject_pump_follow_run_request = (enable != 0U) ? 1U : 0U; /* 只改手柄冷却跟随请求，不直接覆盖上位机独立启动请求。 */
    ExternalComm_RefreshUart5PumpRunState();                          /* 按两个来源的并集刷新最终运行状态。 */
#else
    s_uart5_inject_pump_follow_run_request = 0U; /* 跟随开关关闭时强制清掉手柄跟随请求，A 泵是否运行只看独立控制。 */
    ExternalComm_RefreshUart5PumpRunState();     /* 刷新后不会影响仍然存在的上位机独立启动请求。 */
    /* 跟随开关关闭时保留入参消耗，避免编译器因未使用参数产生告警。 */
    (void)enable;
#endif
}

static void ExternalComm_ApplyControlCommand(const ExternalCommFrame_t *frame)
{
    /* 控制成功应答只回显 1 字节控制代号。 */
    uint8_t info[1];

    /* 只有急停允许跨来源强制全停，其它启停命令都必须先取得外部控制权。 */
    if (ExternalComm_EnsureActiveForRun(frame->area_code == 0xFFU) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BUSY);
        return;
    }

    /* AreaCode 决定具体控制动作，V1 直接改业务状态，不模拟 HMI 切换按键。 */
    switch (frame->area_code)
    {
        case 0x01U:
            /* UART5 是 A 泵输出口；固定识别打开时，先补齐 A 泵在线和注水类型，再执行启动判断。 */
            ExternalComm_ApplyFixedPumpIdentity();
            /* A 泵启动前要求 CS1237 上报过合法霍尔设备类型码。 */
            if (pumpMessageA.online_flag == false)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_DEVICE_FAIL);
                return;
            }
            /* 上位机单独启动 A 泵时，只置位“独立运行请求”，不改变手柄冷却跟随请求。 */
            ExternalComm_SetUart5PumpManualRun(1U);
            break;
        case 0x02U:
            /* 上位机单独停止 A 泵时，只清除“独立运行请求”；如果手柄仍在转动且 A 泵是注水泵，冷却跟随会继续保持。 */
            ExternalComm_SetUart5PumpManualRun(0U);
            break;
        case 0x03U:
            /* 固定识别打开时，先补齐 B 泵在线和注水类型，再执行启动判断。 */
            ExternalComm_ApplyFixedPumpIdentity();
            /* B 泵启动前要求 CS1237 上报过合法霍尔设备类型码。 */
            if (pumpMessageB.online_flag == false)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_DEVICE_FAIL);
                return;
            }
            /* 置位 B 泵运行标志。 */
            pumpMessageB.run_flag = true;
            /* 外部普通启动不进入排空计时模式。 */
            pumpMessageB.timingDrainage_flag = false;
            break;
        case 0x04U:
            /* 清除 B 泵运行标志。 */
            pumpMessageB.run_flag = false;
            /* 同时清除 B 泵排空计时。 */
            pumpMessageB.timingDrainage_flag = false;
            break;
        case 0x05U:
            /* 当前手柄启动必须有选中通道且该通道已经识别在线，避免无手柄时误启动电机和联动注水泵。 */
            if (ExternalComm_SelectedHandleOnline() == 0U)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_NO_CHANNEL);
                return;
            }
            /* 恢复实际速度为已设置速度。 */
            WorkMessage.speed_work = WorkMessage.speed_set_work;
            /* 置位运行标志，驱动任务会发送启动帧。 */
            WorkMessage.runflag_work = true;
            /* 置位外部控制启动标志，UI/状态层可区分外部启动来源。 */
            ControlSignalMessage.HMI_control_flag = true;
            /* 当前手柄启动成功后，按调试开关同步启动 UART5 对应的注水泵。 */
            ExternalComm_SetUart5InjectPumpFollow(1U);
            break;
        case 0x06U:
            /* 停止当前手柄运行。 */
            WorkMessage.runflag_work = false;
            /* 停止时清零实际输出速度。 */
            WorkMessage.speed_work = 0U;
            /* 清除外部控制启动标志。 */
            ControlSignalMessage.HMI_control_flag = false;
            /* 当前手柄停止时，同步停止由手柄带动的 UART5 注水泵。 */
            ExternalComm_SetUart5InjectPumpFollow(0U);
            break;
        case 0x07U:
            /* 开口定位左动作必须有当前通道。 */
            if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_NO_CHANNEL);
                return;
            }
            /* 开口定位必须依赖当前选中通道，避免未选通道时误按 B 通道下发。 */
            ToolPosMay(WorkMessage.channel_work, true, 1U);
            break;
        case 0x08U:
            /* 开口定位右动作必须有当前通道。 */
            if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_NO_CHANNEL);
                return;
            }
            /* 开口定位必须依赖当前选中通道，避免未选通道时误按 B 通道下发。 */
            ToolPosMay(WorkMessage.channel_work, false, 1U);
            break;
        case 0xFFU:
            /* 急停统一清除手柄和泵运行状态。 */
            ExternalComm_StopAllWork();
            break;
        default:
            /* 未定义控制码返回控制失败。 */
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                     frame->area_code,
                                     EXTERNAL_COMM_REASON_BAD_AREA);
            return;
    }

    /* 应答载荷回显控制 AreaCode。 */
    info[0] = frame->area_code;
    /* 返回控制成功。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_CONTROL_OK, info, sizeof(info));
}

static void ExternalComm_ReadBusinessPage(const ExternalCommFrame_t *frame)
{
    /* page_index 是 AT24CS32 驱动使用的 0 基页下标。 */
    uint16_t page_index;

    /* 先把协议业务 AreaCode 映射到 EEPROM 页。 */
    if (ExternalComm_MapAreaToPageIndex(frame->area_code, &page_index) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_AREA);
        return;
    }

    /* 从当前选中通道对应的 EEPROM 读取 32 字节页并校验页尾。 */
    if (ExternalComm_ReadCurrentPage(page_index, s_page_buf) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_DEVICE_FAIL);
        return;
    }

    /* 上传时只发前 30 字节有效数据，最后 2 字节页校验不暴露给外部设备。 */
    ExternalComm_SendFrame(EXTERNAL_COMM_FUNC_EEPROM_UPLOAD,
                           frame->area_code,
                           EXTERNAL_COMM_INFO_NONE,
                           s_page_buf,
                           AT24CS32_PAGE_DATA_SIZE);
}

static void ExternalComm_ReadNavPage(const ExternalCommFrame_t *frame)
{
    /* page_index 是 AT24CS32 驱动使用的 0 基页下标。 */
    uint16_t page_index;

    /* 导航区支持实际页号 12~128 或导航序号 1~117。 */
    if (ExternalComm_MapNavPageToIndex(frame->area_code, &page_index) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_AREA);
        return;
    }

    /* 导航数据仍从当前选中通道 EEPROM 读取。 */
    if (ExternalComm_ReadCurrentPage(page_index, s_page_buf) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_DEVICE_FAIL);
        return;
    }

    /* 导航数据上传 AreaCode 固定 0xFF，InforCode 回显页码。 */
    ExternalComm_SendFrame(EXTERNAL_COMM_FUNC_EEPROM_UPLOAD,
                           EXTERNAL_COMM_AREA_NONE,
                           frame->area_code,
                           s_page_buf,
                           AT24CS32_PAGE_DATA_SIZE);
}

static void ExternalComm_WriteBusinessPage(const ExternalCommFrame_t *frame)
{
    /* page_index 是 AT24CS32 驱动使用的 0 基页下标。 */
    uint16_t page_index;

    /* 写页只接受 30 字节有效数据，页尾 2 字节校验由驱动生成。 */
    if (frame->info_len != AT24CS32_PAGE_DATA_SIZE)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_LENGTH);
        return;
    }

    /* 把业务 AreaCode 转为 EEPROM 页下标。 */
    if (ExternalComm_MapAreaToPageIndex(frame->area_code, &page_index) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_AREA);
        return;
    }

    /* 清空 32 字节页缓存，确保未写入的尾部字节是确定值。 */
    memset(s_page_buf, 0, sizeof(s_page_buf));
    /* 复制外部设备下发的前 30 字节有效数据。 */
    memcpy(s_page_buf, frame->info_area, AT24CS32_PAGE_DATA_SIZE);
    /* 写入当前通道 EEPROM，驱动会刷新最后 2 字节页校验。 */
    if (ExternalComm_WriteCurrentPage(page_index, s_page_buf) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_DEVICE_FAIL);
        return;
    }

    /* 写成功后回显被修改的业务 AreaCode。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_MEMORY_OK, &frame->area_code, 1U);
}

static void ExternalComm_WriteNavPage(const ExternalCommFrame_t *frame)
{
    /* page_index 是 AT24CS32 驱动使用的 0 基页下标。 */
    uint16_t page_index;

    /* 导航写页同样只接受 30 字节有效数据。 */
    if (frame->info_len != AT24CS32_PAGE_DATA_SIZE)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_LENGTH);
        return;
    }

    /* 把导航页码转为 EEPROM 页下标。 */
    if (ExternalComm_MapNavPageToIndex(frame->area_code, &page_index) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_BAD_AREA);
        return;
    }

    /* 清空 32 字节页缓存，避免残留数据影响页校验。 */
    memset(s_page_buf, 0, sizeof(s_page_buf));
    /* 复制 30 字节导航有效数据。 */
    memcpy(s_page_buf, frame->info_area, AT24CS32_PAGE_DATA_SIZE);
    /* 写入当前通道 EEPROM，最后 2 字节页校验由驱动生成。 */
    if (ExternalComm_WriteCurrentPage(page_index, s_page_buf) == 0U)
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_DEVICE_FAIL);
        return;
    }

    /* 写成功后回显导航页码。 */
    ExternalComm_SendAck(EXTERNAL_COMM_ACK_MEMORY_OK, &frame->area_code, 1U);
}

static void ExternalComm_DispatchFrame(const ExternalCommFrame_t *frame)
{
    /* V1 只处理外部设备下发帧，忽略本机上传帧或其他方向码。 */
    if ((frame->tran_code != EXTERNAL_COMM_TRAN_DOWNLOAD) ||
        (frame->info_code != EXTERNAL_COMM_INFO_NONE))
    {
        return;
    }

    /* FunCode 决定下行命令大类。 */
    switch (frame->fun_code)
    {
        case EXTERNAL_COMM_DOWN_APPLY_CONTROL:
            /* 申请外部控制，InforArea 必须是 8 字节注册码。 */
            (void)ExternalComm_ApplyExternalAuth(frame);
            break;
        case EXTERNAL_COMM_DOWN_SET_VALUE:
            /* 设置速度、频率、A 泵速度、B 泵速度。 */
            ExternalComm_ApplySetting(frame);
            break;
        case EXTERNAL_COMM_DOWN_SWITCH_VALUE:
            /* 切换通道、方向、控制模式或工具类型。 */
            ExternalComm_ApplySwitchSetting(frame);
            break;
        case EXTERNAL_COMM_DOWN_CONTROL_CMD:
            /* 执行泵启停、手柄启停、开口定位、急停。 */
            ExternalComm_ApplyControlCommand(frame);
            break;
        case EXTERNAL_COMM_DOWN_READ_PAGE:
            /* 读取业务 EEPROM 单页。 */
            ExternalComm_ReadBusinessPage(frame);
            break;
        case EXTERNAL_COMM_DOWN_WRITE_PAGE:
            /* 写入业务 EEPROM 单页。 */
            ExternalComm_WriteBusinessPage(frame);
            break;
        case EXTERNAL_COMM_DOWN_READ_NAV_PAGE:
            /* 读取导航 EEPROM 单页。 */
            ExternalComm_ReadNavPage(frame);
            break;
        case EXTERNAL_COMM_DOWN_WRITE_NAV_PAGE:
            /* 写入导航 EEPROM 单页。 */
            ExternalComm_WriteNavPage(frame);
            break;
        case EXTERNAL_COMM_DOWN_READ_ALL:
        case EXTERNAL_COMM_DOWN_READ_NAV_ALL:
            /* 整区读取接近 4KB，超过当前单帧策略，V1 明确返回不支持。 */
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_MEMORY_FAILED,
                                     frame->fun_code,
                                     EXTERNAL_COMM_REASON_NOT_SUPPORT);
            break;
        case EXTERNAL_COMM_DOWN_PERMISSION:
            /* 功能升级/权限开放命令，V1 只校验 8 字节权限码长度。 */
            (void)ExternalComm_ApplyPermission(frame);
            break;
        case EXTERNAL_COMM_DOWN_HOST_EXIT:
            /* 上位机主动退出外部控制，释放互斥控制权并停止外控输出。 */
            ExternalComm_ApplyHostExit();
            break;
        default:
            /* 未定义 FunCode 返回控制失败，失败对象用 FunCode 标识。 */
            ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                     frame->fun_code,
                                     EXTERNAL_COMM_REASON_BAD_AREA);
            break;
    }
}

static void ExternalComm_HeartbeatAppendU8(uint8_t *info_area, uint16_t *info_len, uint8_t value)
{
    /* info_area 指向本次心跳 InforArea 缓冲，由调用方保证容量为 EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN。 */
    if ((info_area == NULL) || (info_len == NULL))
    {
        /* 防御空指针，避免异常调用时写坏内存。 */
        return;
    }

    /* 当前写入位置由 *info_len 表示，写完后长度同步后移 1 字节。 */
    if (*info_len < EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN)
    {
        /* 写入一个单字节状态、类型或代号字段。 */
        info_area[*info_len] = value;
        /* 记录心跳载荷已经增加 1 字节。 */
        *info_len = (uint16_t)(*info_len + 1U);
    }
}

static void ExternalComm_HeartbeatAppendBE16(uint8_t *info_area, uint16_t *info_len, uint16_t value)
{
    /* info_area 指向心跳载荷缓冲，info_len 指向当前已经写入的字节数。 */
    if ((info_area == NULL) || (info_len == NULL))
    {
        /* 防御空指针，保持心跳任务不会因异常参数越界。 */
        return;
    }

    /* 16 位字段需要 2 字节空间，不足时直接跳过本字段。 */
    if ((uint16_t)(*info_len + 2U) <= EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN)
    {
        /* 按协议大端顺序写入速度、电流或泵速度。 */
        ExternalComm_WriteBE16(&info_area[*info_len], value);
        /* 记录心跳载荷已经增加 2 字节。 */
        *info_len = (uint16_t)(*info_len + 2U);
    }
}

static void ExternalComm_HeartbeatAppendPumpPressure(uint8_t *info_area,
                                                     uint16_t *info_len,
                                                     const pumpMessage_t *pump_message)
{
    /* raw_bits 保存 int32_t 原始采样的二进制位，不做符号变换，确保上位机可恢复负数。 */
    uint32_t raw_bits;
    /* weight_x10 保存下位机换算后的重量，单位 0.1g，与 cs1237_uart_protocol.md 保持一致。 */
    uint32_t weight_x10;
    /* threshold_g 保存压力阈值，单位 g，便于上位机判断当前重量接近哪个保护阈值。 */
    uint16_t threshold_g;

    /* 泵消息为空时无法追加压力扩展字段，直接返回保持前面在线/速度字段有效。 */
    if ((info_area == NULL) || (info_len == NULL) || (pump_message == NULL))
    {
        return;
    }

    /* 压力扩展固定 11 字节：RawCs1237(4LE) + WeightX10(4LE) + ThresholdG(2LE) + Seq(1)。 */
    if ((uint16_t)(*info_len + 11U) > EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN)
    {
        /* 剩余空间不足时不写半截压力字段，避免上位机把残缺数据误解析成 B 泵状态。 */
        return;
    }

    /* 将 volatile 字段先读到局部变量，保证后续多字节拆分时同一字段来自同一次读取。 */
    raw_bits = (uint32_t)pump_message->pressure_value;
    /* WeightX10 已在 SimUartTaskFunc 中由 CS1237 下位机帧写入，单位为 0.1g。 */
    weight_x10 = pump_message->weight_x10;
    /* pressure_threshold 由压力模块上报，单位为 g。 */
    threshold_g = pump_message->pressure_threshold;

    /* CS1237 下位机协议多字节字段是 little-endian，这里保持同样顺序，方便上位机按原协议解释。 */
    info_area[*info_len] = (uint8_t)(raw_bits & 0xFFU);
    info_area[(uint16_t)(*info_len + 1U)] = (uint8_t)((raw_bits >> 8U) & 0xFFU);
    info_area[(uint16_t)(*info_len + 2U)] = (uint8_t)((raw_bits >> 16U) & 0xFFU);
    info_area[(uint16_t)(*info_len + 3U)] = (uint8_t)((raw_bits >> 24U) & 0xFFU);
    /* 记录 RawCs1237 已经追加 4 字节。 */
    *info_len = (uint16_t)(*info_len + 4U);

    /* WeightX10 同样采用 little-endian，数值除以 10 后就是上位机显示的 g。 */
    info_area[*info_len] = (uint8_t)(weight_x10 & 0xFFU);
    info_area[(uint16_t)(*info_len + 1U)] = (uint8_t)((weight_x10 >> 8U) & 0xFFU);
    info_area[(uint16_t)(*info_len + 2U)] = (uint8_t)((weight_x10 >> 16U) & 0xFFU);
    info_area[(uint16_t)(*info_len + 3U)] = (uint8_t)((weight_x10 >> 24U) & 0xFFU);
    /* 记录 WeightX10 已经追加 4 字节。 */
    *info_len = (uint16_t)(*info_len + 4U);

    /* ThresholdG 是 16 位 little-endian，保持和 CS1237 上报帧一致。 */
    info_area[*info_len] = (uint8_t)(threshold_g & 0xFFU);
    info_area[(uint16_t)(*info_len + 1U)] = (uint8_t)((threshold_g >> 8U) & 0xFFU);
    /* 记录 ThresholdG 已经追加 2 字节。 */
    *info_len = (uint16_t)(*info_len + 2U);

    /* seq 是压力模块帧序号，单字节追加，用于上位机判断压力数据是否在刷新。 */
    info_area[*info_len] = pump_message->seq;
    /* 记录 Seq 已经追加 1 字节。 */
    *info_len = (uint16_t)(*info_len + 1U);
}

static uint8_t ExternalComm_FootPedalOnlineStatus(void)
{
    /* 脚踏在线状态以 sscFOOT 写入的新控制使能为准，不再读取旧脚踏连接字段。 */
    if (ControlSignalMessage.jt_enable_flag == true)
    {
        /* 返回协议定义的在线值 0x01。 */
        return EXTERNAL_COMM_STATUS_ONLINE;
    }

    /* 两个连接字段都不是 Connect 时，心跳上报脚踏掉线。 */
    return EXTERNAL_COMM_STATUS_OFFLINE;
}

static uint8_t ExternalComm_SelectedHandleStatus(void)
{
    /* 当前工作通道为 A 且 A 插孔在线时，上报选中 A。 */
    if ((WorkMessage.channel_work == CHANNEL_A) && WorkMessage.Channel_Aonline)
    {
        /* 协议定义选中 A 手柄为 0x01。 */
        return EXTERNAL_COMM_STATUS_SELECTED_A;
    }

    /* 当前工作通道为 B 且 B 插孔在线时，上报选中 B。 */
    if ((WorkMessage.channel_work == CHANNEL_B) && WorkMessage.Channel_Bonline)
    {
        /* 协议定义选中 B 手柄为 0x02。 */
        return EXTERNAL_COMM_STATUS_SELECTED_B;
    }

    /* 没有有效选中通道时，上报 0xFF。 */
    return EXTERNAL_COMM_STATUS_OFFLINE;
}

static uint8_t ExternalComm_SelectedHandleOnline(void)
{
    /* 当前通道指向 A 时，只看 A 插孔在线标志。 */
    if (WorkMessage.channel_work == CHANNEL_A)
    {
        /* 把 bool 转成 0/1，便于运行状态判断。 */
        return WorkMessage.Channel_Aonline ? 1U : 0U;
    }

    /* 当前通道指向 B 时，只看 B 插孔在线标志。 */
    if (WorkMessage.channel_work == CHANNEL_B)
    {
        /* 把 bool 转成 0/1，便于运行状态判断。 */
        return WorkMessage.Channel_Bonline ? 1U : 0U;
    }

    /* 当前通道不是 A/B，按未接入处理。 */
    return 0U;
}

static uint8_t ExternalComm_CurrentHandleRunStatus(void)
{
    /* 没有有效选中手柄时，协议要求运行状态上报未接入 0x03。 */
    if (ExternalComm_SelectedHandleOnline() == 0U)
    {
        /* 当前手柄未接入。 */
        return EXTERNAL_COMM_STATUS_UNPLUGGED;
    }

    /* 有选中手柄时，runflag_work 为 true 表示当前手柄正在运行。 */
    if (WorkMessage.runflag_work)
    {
        /* 当前手柄运行中。 */
        return EXTERNAL_COMM_STATUS_RUNNING;
    }

    /* 有选中手柄但未运行时，上报待机。 */
    return EXTERNAL_COMM_STATUS_STANDBY;
}

static uint8_t ExternalComm_CurrentHandleMode(void)
{
    /* 未选中 A/B 或选中插孔已经离线时，方向字段没有现场意义，上报未知避免上位机误判。 */
    if (ExternalComm_SelectedHandleOnline() == 0U)
    {
        return EXTERNAL_COMM_HANDLE_MODE_UNKNOWN;
    }

    /* 项目内部 ZZDIR 表示正转，协议心跳使用 0x01 明确给上位机显示。 */
    if (WorkMessage.dir_work == ZZDIR)
    {
        return EXTERNAL_COMM_HANDLE_MODE_FORWARD;
    }

    /* 项目内部 FZDIR 表示反转，协议心跳使用 0x02 明确给上位机显示。 */
    if (WorkMessage.dir_work == FZDIR)
    {
        return EXTERNAL_COMM_HANDLE_MODE_REVERSE;
    }

    /* 项目内部 OSCDIR 表示往复，PX 系列手柄切到往复时实时状态需要显示这个模式。 */
    if (WorkMessage.dir_work == OSCDIR)
    {
        return EXTERNAL_COMM_HANDLE_MODE_OSC;
    }

    /* 其它历史值不强行解释，交给上位机显示“未知”。 */
    return EXTERNAL_COMM_HANDLE_MODE_UNKNOWN;
}

static void ExternalComm_HeartbeatAppendHandle(uint8_t *info_area,
                                               uint16_t *info_len,
                                               uint8_t online,
                                               uint8_t raw_type_major,
                                               uint8_t raw_type_minor)
{
    /* 先追加插孔在线状态，这是协议固定出现的字段。 */
    ExternalComm_HeartbeatAppendU8(info_area,
                                   info_len,
                                   online ? EXTERNAL_COMM_STATUS_ONLINE : EXTERNAL_COMM_STATUS_OFFLINE);

    /* 插孔在线时才继续追加对应手柄 EEPROM 原始类型码，离线时省略类型字段。 */
    if (online)
    {
        /* 第 1 个类型字节直接使用 EEPROM Page2 第 0 字节，例如 0x6B。 */
        ExternalComm_HeartbeatAppendU8(info_area, info_len, raw_type_major);
        /* 第 2 个类型字节直接使用 EEPROM Page2 第 1 字节，例如 0x01。 */
        ExternalComm_HeartbeatAppendU8(info_area, info_len, raw_type_minor);
    }
}

static void ExternalComm_HeartbeatAppendPump(uint8_t *info_area,
                                             uint16_t *info_len,
                                             const pumpMessage_t *pump_message)
{
    /* pump_message 为空时无法读取泵状态，按离线上报。 */
    if (pump_message == NULL)
    {
        /* 追加一个离线状态，保持心跳字段顺序可解析。 */
        ExternalComm_HeartbeatAppendU8(info_area, info_len, EXTERNAL_COMM_STATUS_OFFLINE);
        return;
    }

    /* 先追加泵在线状态，这是协议固定出现的字段。 */
    ExternalComm_HeartbeatAppendU8(info_area,
                                   info_len,
                                   pump_message->online_flag ? EXTERNAL_COMM_STATUS_ONLINE : EXTERNAL_COMM_STATUS_OFFLINE);

    /* 泵在线时才继续追加泵类型、泵速度和 CS1237 压力扩展字段，离线时省略这些字段。 */
    if (pump_message->online_flag)
    {
        /* 泵类型当前来自 CS1237 DeviceCode，协议线上按 1 字节设备类型码上传。 */
        ExternalComm_HeartbeatAppendU8(info_area, info_len, (uint8_t)(pump_message->type & 0xFFU));
        /* 泵速度是业务数值，按 2 字节大端上传。 */
        ExternalComm_HeartbeatAppendBE16(info_area, info_len, pump_message->speed_work);
        /* 压力原始值和最终重量来自 SimUartTaskFunc 解析的 CS1237 21 字节下位机帧。 */
        ExternalComm_HeartbeatAppendPumpPressure(info_area, info_len, pump_message);
    }
}

static void ExternalComm_SendHeartbeat(void)
{
    /* heartbeat_info 保存新版心跳 InforArea，字段会随在线/运行状态动态增减。 */
    uint8_t heartbeat_info[EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN];
    /* heartbeat_len 记录当前已经写入的 InforArea 字节数。 */
    uint16_t heartbeat_len = 0U;
    /* run_status 保存当前手柄运行状态，后续决定是否追加速度和电流。 */
    uint8_t run_status;
    /* tx_len 接收心跳完整帧长度。 */
    uint16_t tx_len = 0U;

    /* 固定识别打开时，心跳也显示 UART5 A 泵在线且类型为注水泵，方便上位机确认当前可控状态。 */
    ExternalComm_ApplyFixedPumpIdentity();

    /* 追加 A 手柄插孔在线状态；若在线，紧跟 A 手柄类型。 */
    ExternalComm_HeartbeatAppendHandle(heartbeat_info,
                                       &heartbeat_len,
                                       WorkMessage.Channel_Aonline ? 1U : 0U,
                                       MemoryMsgA.hand_type_raw_major,
                                       MemoryMsgA.hand_type_raw_minor);
    /* 追加 B 手柄插孔在线状态；若在线，紧跟 B 手柄类型。 */
    ExternalComm_HeartbeatAppendHandle(heartbeat_info,
                                       &heartbeat_len,
                                       WorkMessage.Channel_Bonline ? 1U : 0U,
                                       MemoryMsgB.hand_type_raw_major,
                                       MemoryMsgB.hand_type_raw_minor);
    /* 追加当前选中的手柄插孔，未选中或选中通道离线时为 0xFF。 */
    ExternalComm_HeartbeatAppendU8(heartbeat_info, &heartbeat_len, ExternalComm_SelectedHandleStatus());
    /* 读取当前手柄运行情况：待机 0x01、运行中 0x02、未接入 0x03。 */
    run_status = ExternalComm_CurrentHandleRunStatus();
    /* 追加当前手柄运行情况。 */
    ExternalComm_HeartbeatAppendU8(heartbeat_info, &heartbeat_len, run_status);
    /* 追加当前手柄方向/往复模式，PXBA/PXBB 上位机会用该字段显示“手柄工作模式”。 */
    ExternalComm_HeartbeatAppendU8(heartbeat_info, &heartbeat_len, ExternalComm_CurrentHandleMode());
    /* 当前手柄运行中时，按协议继续追加当前通道工作速度和工作电流。 */
    if (run_status == EXTERNAL_COMM_STATUS_RUNNING)
    {
        /* 当前通道手柄工作速度，单位为 WorkMessage.speed_work 内部 x10，2 字节大端；上位机显示 rpm 时需要除以 10。 */
        ExternalComm_HeartbeatAppendBE16(heartbeat_info, &heartbeat_len, WorkMessage.speed_work);
        /* 驱动板反馈实时电流，单位 0.01A，2 字节大端；current_work 保留为下发给驱动板的保护电流阈值。 */
        ExternalComm_HeartbeatAppendBE16(heartbeat_info, &heartbeat_len, WorkMessage.driver_current_x100);
    }
    /* 追加脚踏在线状态。 */
    ExternalComm_HeartbeatAppendU8(heartbeat_info, &heartbeat_len, ExternalComm_FootPedalOnlineStatus());
    /* 追加 A 泵在线状态；若在线，紧跟 A 泵类型和 A 泵速度。 */
    ExternalComm_HeartbeatAppendPump(heartbeat_info, &heartbeat_len, &pumpMessageA);
    /* 追加 B 泵在线状态；若在线，紧跟 B 泵类型和 B 泵速度。 */
    ExternalComm_HeartbeatAppendPump(heartbeat_info, &heartbeat_len, &pumpMessageB);

    /* 按 0xAA 功能码构造心跳帧。 */
    if (ExternalCommProtocol_BuildHeartbeat(heartbeat_info,
                                            heartbeat_len,
                                            s_tx_buf,
                                            sizeof(s_tx_buf),
                                            &tx_len) == EXTERNAL_COMM_BUILD_OK)
    {
#if (EXTERNAL_COMM_HEARTBEAT_USE_UART10 == 1U)
        /* 开关置 1 时，心跳帧从 UART10 主动上传，用于保留现场确认后的心跳输出口。 */
        Bsp_UartTransmit(BSP_UART_PORT_10, s_tx_buf, tx_len, 100U);
#else
        /* 开关置 0 时，心跳帧回到原 UART2 发送路径，便于和旧外部通信链路对比。 */
        Uart2_SendPacket(s_tx_buf, tx_len);
#endif
    }
}

static void ExternalComm_RxFifoInit(void)
{
    /* 只初始化一次 FIFO，保持读写指针从 0 开始，避免运行中误清已经缓存的半帧。 */
    if (s_rx_fifo.ready != 0U)
    {
        /* 已经初始化过时直接返回，本函数可被 Init 和接收路径重复安全调用。 */
        return;
    }

    /* 读写指针同时清零表示 FIFO 为空，最后 1 字节预留给满/空状态区分。 */
    s_rx_fifo.read_index = 0U;
    /* 写指针也从 0 开始，第一包 DMA 数据会从缓冲区头部写入。 */
    s_rx_fifo.write_index = 0U;
    /* 初始化完成后才允许接收路径读写 FIFO。 */
    s_rx_fifo.ready = 1U;
}

static uint16_t ExternalComm_RxFifoFull(void)
{
    /* 写指针在读指针后方时，已用长度就是两者差值。 */
    if (s_rx_fifo.write_index >= s_rx_fifo.read_index)
    {
        return (uint16_t)(s_rx_fifo.write_index - s_rx_fifo.read_index);
    }

    /* 写指针绕回到缓冲区前部时，已用长度由尾段和头段两部分组成。 */
    return (uint16_t)((EXTERNAL_COMM_RX_FIFO_SIZE - s_rx_fifo.read_index) + s_rx_fifo.write_index);
}

static uint16_t ExternalComm_RxFifoFree(void)
{
    /* 环形 FIFO 预留 1 字节区分满/空，因此最大可用容量是总长度减 1。 */
    return (uint16_t)((EXTERNAL_COMM_RX_FIFO_SIZE - 1U) - ExternalComm_RxFifoFull());
}

static void ExternalComm_RxFifoReset(void)
{
    /* 清空 FIFO 只需要把读写指针重新对齐，不需要擦除数据区内容。 */
    s_rx_fifo.read_index = 0U;
    /* 写指针同步清零，下一包会从头部重新开始写。 */
    s_rx_fifo.write_index = 0U;
}

static uint16_t ExternalComm_RxFifoWrite(const uint8_t *data, uint16_t data_len)
{
    /* written 记录实际写入字节数，用来判断 FIFO 是否发生拥塞。 */
    uint16_t written = 0U;

    /* FIFO 未初始化、空指针或 0 长度都没有可写入内容。 */
    if ((s_rx_fifo.ready == 0U) || (data == NULL) || (data_len == 0U))
    {
        return 0U;
    }

    /* 逐字节写入，确保写指针跨尾部时能正确回绕到缓冲区头部。 */
    while ((written < data_len) && (ExternalComm_RxFifoFree() > 0U))
    {
        /* 当前 DMA 字节写入写指针位置。 */
        s_rx_fifo_buf[s_rx_fifo.write_index] = data[written];
        /* 写入计数先增加，供调用者判断是否完整接收本包。 */
        ++written;
        /* 写指针前进一个字节。 */
        ++s_rx_fifo.write_index;
        /* 写指针到达数组尾部时回绕到 0。 */
        if (s_rx_fifo.write_index >= EXTERNAL_COMM_RX_FIFO_SIZE)
        {
            s_rx_fifo.write_index = 0U;
        }
    }

    return written;
}

static uint16_t ExternalComm_RxFifoSkip(uint16_t skip_len)
{
    /* full 是当前可跳过的最大字节数，避免读指针越过写指针。 */
    uint16_t full = ExternalComm_RxFifoFull();
    /* actual_len 是本次真正跳过的字节数。 */
    uint16_t actual_len = skip_len;

    /* 请求跳过长度超过已有数据时，只跳过当前 FIFO 内全部数据。 */
    if (actual_len > full)
    {
        actual_len = full;
    }

    /* 读指针前移 actual_len，超出数组尾部时按环形缓存回绕。 */
    s_rx_fifo.read_index = (uint16_t)(s_rx_fifo.read_index + actual_len);
    /* 可能一次跨过尾部，减去 FIFO 长度即可回到有效索引范围。 */
    if (s_rx_fifo.read_index >= EXTERNAL_COMM_RX_FIFO_SIZE)
    {
        s_rx_fifo.read_index = (uint16_t)(s_rx_fifo.read_index - EXTERNAL_COMM_RX_FIFO_SIZE);
    }

    return actual_len;
}

static uint8_t ExternalComm_RxFifoPeek(uint16_t skip_count, uint8_t *data, uint16_t data_len)
{
    /* full 是 FIFO 当前已有数据，必须覆盖 skip_count 和目标读取长度。 */
    uint16_t full = ExternalComm_RxFifoFull();
    /* index 是本次窥探使用的临时读索引，不会改变真实读指针。 */
    uint16_t index;
    /* copied 是已经复制到目标缓冲的字节数。 */
    uint16_t copied = 0U;

    /* 空目标或目标长度为 0 时没有可复制内容。 */
    if ((data == NULL) || (data_len == 0U))
    {
        return 0U;
    }

    /* FIFO 中数据不足时不能窥探，调用者应等待下一包补齐。 */
    if (full < (uint16_t)(skip_count + data_len))
    {
        return 0U;
    }

    /* 从真实读指针加偏移位置开始读，支持前面已经有噪声但暂不消费的场景。 */
    index = (uint16_t)(s_rx_fifo.read_index + skip_count);
    /* 偏移跨越尾部时回绕。 */
    if (index >= EXTERNAL_COMM_RX_FIFO_SIZE)
    {
        index = (uint16_t)(index - EXTERNAL_COMM_RX_FIFO_SIZE);
    }

    /* 逐字节复制，保证跨尾部的帧也能取成线性缓存。 */
    while (copied < data_len)
    {
        /* 复制当前 FIFO 字节到目标缓存。 */
        data[copied] = s_rx_fifo_buf[index];
        /* 已复制长度递增。 */
        ++copied;
        /* 临时读索引前进。 */
        ++index;
        /* 到尾部后回绕，继续读取头部残留数据。 */
        if (index >= EXTERNAL_COMM_RX_FIFO_SIZE)
        {
            index = 0U;
        }
    }

    return 1U;
}

static uint8_t ExternalComm_RxFifoFind(const uint8_t *pattern,
                                       uint16_t pattern_len,
                                       uint16_t start_offset,
                                       uint16_t *found_offset)
{
    /* full 是可搜索数据长度，搜索范围不能超过当前 FIFO 已有数据。 */
    uint16_t full = ExternalComm_RxFifoFull();
    /* offset 是当前候选匹配起点。 */
    uint16_t offset;
    /* idx 是模式串内部比较下标。 */
    uint16_t idx;
    /* byte 保存从 FIFO 中窥探出的单字节。 */
    uint8_t byte;

    /* 参数不完整时不能搜索。 */
    if ((pattern == NULL) || (pattern_len == 0U) || (found_offset == NULL))
    {
        return 0U;
    }

    /* 已有数据不够容纳起始偏移加完整模式串时，说明还不能判定。 */
    if (full < (uint16_t)(start_offset + pattern_len))
    {
        return 0U;
    }

    /* 默认输出 0，避免调用者在失败路径读到旧值。 */
    *found_offset = 0U;
    /* 逐个候选偏移查找帧头 D7 CA F8 F1。 */
    for (offset = start_offset; offset <= (uint16_t)(full - pattern_len); ++offset)
    {
        /* 先假设当前偏移匹配，遇到任何字节不等就跳出。 */
        for (idx = 0U; idx < pattern_len; ++idx)
        {
            /* 从 FIFO 当前候选位置窥探一个字节，不移动真实读指针。 */
            if (ExternalComm_RxFifoPeek((uint16_t)(offset + idx), &byte, 1U) == 0U)
            {
                return 0U;
            }
            /* 只要有一个字节不一致，当前 offset 就不是帧头位置。 */
            if (byte != pattern[idx])
            {
                break;
            }
        }

        /* idx 走完整个模式串，说明找到了完整帧头。 */
        if (idx == pattern_len)
        {
            *found_offset = offset;
            return 1U;
        }
    }

    return 0U;
}

static void ExternalComm_WriteRxChunk(const uint8_t *data, uint16_t data_len)
{
    /* written 保存本次实际写入 FIFO 的字节数，用于判断是否发生接收拥塞。 */
    uint16_t written;

    /* 空指针或 0 长度没有任何可缓存数据，直接返回。 */
    if ((data == NULL) || (data_len == 0U))
    {
        return;
    }

    /* 确保 FIFO 已初始化；如果初始化失败，本次数据不能安全保存。 */
    ExternalComm_RxFifoInit();
    if (s_rx_fifo.ready == 0U)
    {
        return;
    }

    /* 先尝试把整个 DMA 空闲包追加到软件 FIFO，保留粘包和半包。 */
    written = ExternalComm_RxFifoWrite(data, data_len);
    /* FIFO 空间足够时直接返回，后续由 ExternalComm_ProcessRxFifo() 按帧消费。 */
    if (written == data_len)
    {
        return;
    }

    /*
     * FIFO 满通常说明上位机连续高速发送或前面有异常噪声。
     * 这里清空旧残留后只保留最新一包，优先让现场重新同步到最新帧，而不是卡在旧半帧里等待超时。
     */
    ExternalComm_RxFifoReset();
    /* 清空后再次写入当前空闲包；若当前包本身仍放不下，本地 FIFO 会截断，解析层会按坏帧重新同步。 */
    (void)ExternalComm_RxFifoWrite(data, data_len);
}

static uint8_t ExternalComm_ProcessRxFifoFrame(void)
{
    /* full 保存 FIFO 当前已有字节数，读取前先做长度判断，避免 peek 越界。 */
    uint16_t full;
    /* head_offset 保存帧头在 FIFO 中的偏移，非 0 时会先丢弃前导噪声。 */
    uint16_t head_offset;
    /* length_bytes 保存 Length_H/Length_L 两个字节。 */
    uint8_t length_bytes[2];
    /* frame_len 是协议 Length 字段声明的完整帧长。 */
    uint16_t frame_len;
    /* frame 保存协议层解析出的业务字段。 */
    ExternalCommFrame_t frame;
    /* parse_result 保存现有协议解析器的校验结果。 */
    ExternalCommParseResult_t parse_result;

    /* FIFO 尚未初始化时没有可处理数据。 */
    if (s_rx_fifo.ready == 0U)
    {
        return 0U;
    }

    /* 少于帧头长度时不能判断是否有完整帧，等待下一次 DMA 空闲包补齐。 */
    full = ExternalComm_RxFifoFull();
    if (full < EXTERNAL_COMM_FRAME_HEAD_SIZE)
    {
        return 0U;
    }

    /* 在 FIFO 中搜索 D7 CA F8 F1，支持前面带噪声或上一次残留半帧。 */
    if (ExternalComm_RxFifoFind(s_external_comm_frame_head,
                                EXTERNAL_COMM_FRAME_HEAD_SIZE,
                                0U,
                                &head_offset) == 0U)
    {
        /* 没找到完整帧头时保留最后 3 字节，防止帧头被拆成前后两包。 */
        if (full > (EXTERNAL_COMM_FRAME_HEAD_SIZE - 1U))
        {
            /* 其余前导噪声没有继续解析价值，跳过后让下一包继续拼帧头。 */
            (void)ExternalComm_RxFifoSkip((uint16_t)(full - (EXTERNAL_COMM_FRAME_HEAD_SIZE - 1U)));
        }
        return 0U;
    }

    /* 帧头前有噪声或旧坏帧残留时先跳过，保证 FIFO 读指针正对候选帧头。 */
    if (head_offset > 0U)
    {
        (void)ExternalComm_RxFifoSkip(head_offset);
        /* 跳过后重新读取 FIFO 有效字节数，后续判断都基于新读指针。 */
        full = ExternalComm_RxFifoFull();
    }

    /* 帧头后不足最短 16 字节时说明半帧还没收完，保留缓存等待下一包。 */
    if (full < EXTERNAL_COMM_FRAME_FIXED_SIZE)
    {
        return 0U;
    }

    /* 只窥探 Length 两个字节，不移动读指针，避免半帧被提前消费。 */
    if (ExternalComm_RxFifoPeek(EXTERNAL_COMM_FRAME_LENGTH_OFFSET,
                                length_bytes,
                                sizeof(length_bytes)) == 0U)
    {
        return 0U;
    }

    /* Length 字段按协议高字节在前，表示整帧长度。 */
    frame_len = ExternalComm_ReadBE16(length_bytes);
    /* 非法长度通常是噪声或坏帧，只跳过 1 字节重新找帧头，避免误删后续合法帧。 */
    if ((frame_len < EXTERNAL_COMM_FRAME_FIXED_SIZE) || (frame_len > EXTERNAL_COMM_MAX_FRAME_SIZE))
    {
        (void)ExternalComm_RxFifoSkip(1U);
        return 1U;
    }

    /* FIFO 中还没有凑齐 Length 声明的整帧时保留当前数据，等待下一包补齐。 */
    if (full < frame_len)
    {
        return 0U;
    }

    /* 把候选整帧取到线性缓存中，继续复用已经验证过的协议解析和 CRC 校验逻辑。 */
    if (ExternalComm_RxFifoPeek(0U, s_frame_buf, frame_len) == 0U)
    {
        return 0U;
    }

    /* 使用原协议解析器做帧尾、CRC、字段拆解，避免在 FIFO 层重复实现业务解析。 */
    parse_result = ExternalCommProtocol_Parse(s_frame_buf, frame_len, &frame);
    if (parse_result == EXTERNAL_COMM_PARSE_OK)
    {
        /* 合法下行帧到达说明 RS485 链路仍存在，先喂外控保活计时。 */
        ExternalComm_ResetLinkWatchdog();
        /* 消费当前完整帧，后续循环会继续处理同一 FIFO 里的下一帧。 */
        (void)ExternalComm_RxFifoSkip(frame_len);
        /* 按 FunCode 分发下行命令，业务层仍然只看到一帧完整协议数据。 */
        ExternalComm_DispatchFrame(&frame);
        return 1U;
    }

    /* 候选帧 CRC 或帧尾不合法时只跳过帧头首字节，尽量保住后续可能粘连的合法帧。 */
    (void)ExternalComm_RxFifoSkip(1U);
    return 1U;
}

static void ExternalComm_ProcessRxFifo(void)
{
    /* step_count 限制单次任务处理次数，避免异常噪声导致 10ms 任务被长期占用。 */
    uint16_t step_count = 0U;

    /* 只要本轮处理有推进，就继续尝试吐出下一帧，实现一包多帧连续分发。 */
    while (step_count < EXTERNAL_COMM_RX_FIFO_MAX_STEPS)
    {
        /* 返回 0 表示当前 FIFO 暂无完整帧或只剩半帧，本周期处理结束。 */
        if (ExternalComm_ProcessRxFifoFrame() == 0U)
        {
            break;
        }
        /* 每成功消费或跳过一段异常数据，都累计一次循环次数。 */
        ++step_count;
    }
}

static void ExternalComm_ProcessReceive(void)
{
    /* recv_len 保存 UART2 DMA 空闲包长度。 */
    uint16_t recv_len;

    /* 从 UART2 DMA 缓存取出一包已经静默稳定的数据。 */
    recv_len = Uart2_DMARecvDataPeek(s_rx_buf);
    /* 有新空闲包时先追加到软件 FIFO，不在 DMA 临时包里直接只解析第一帧。 */
    if (recv_len > 0U)
    {
        /* FIFO 会保留粘包和半包，解决一包多帧或一帧跨包时被 DMAReset 清掉的问题。 */
        ExternalComm_WriteRxChunk(s_rx_buf, recv_len);
    }

    /* 无论本周期有没有新包，都继续尝试处理 FIFO 中上一轮剩下的完整帧。 */
    ExternalComm_ProcessRxFifo();
}

static void ExternalCommTaskFunc(uint32_t event)
{
    /* 当前调度器未使用 event，显式丢弃避免编译器告警。 */
    (void)event;

    /* 每 10ms 检查一次 UART2 是否收到完整空闲包。 */
    ExternalComm_ProcessReceive();

    /* 外控有效时监控上位机保活；RS485 拔线后收不到下行帧，超时会释放外控并停止电机/泵。 */
    ExternalComm_CheckLinkWatchdog();

    /* 监视 WorkMessage 报警码变化，变化时立即上传 0x03/0x05 报警信息帧给上位机弹窗。 */
    ExternalComm_SendAlarmInfoIfChanged();

    /* 累加心跳计时，任务周期由 EXTERNAL_COMM_TASK_PERIOD_MS 定义。 */
    s_heartbeat_elapsed_ms = (uint16_t)(s_heartbeat_elapsed_ms + EXTERNAL_COMM_TASK_PERIOD_MS);
    /* 到达心跳周期后主动上传 0xAA 心跳帧。 */
    if (s_heartbeat_elapsed_ms >= EXTERNAL_COMM_HEARTBEAT_PERIOD_MS)
    {
        /* 先清零计时，避免发送阻塞时重复触发。 */
        s_heartbeat_elapsed_ms = 0U;
        /* 采集当前工作状态并发送心跳。 */
        ExternalComm_SendHeartbeat();
    }
}

void ExternalComm_Init(void)
{
    /* 初始化外控 RX FIFO，保证任务第一次运行前已经准备好接收粘包/半包数据。 */
    ExternalComm_RxFifoInit();
    /* 创建 UART2 外部通信任务。 */
    Kernel_TaskCreate(&ExternalCommTaskHandle, ExternalCommTaskFunc);
    /* 任务常驻运行，每 10ms 执行一次接收和心跳调度。 */
    Kernel_TaskStart(&ExternalCommTaskHandle, KERNEL_TASK_ALWAYS, EXTERNAL_COMM_TASK_PERIOD_MS);
}
