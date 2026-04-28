#include "external_comm_task.h"

#include "external_comm_protocol.h"

#include "at24cs32.h"
#include "data.h"
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "sscDRIVE.h"
#include "uart2.h"

#include <string.h>

#define EXTERNAL_COMM_TASK_PERIOD_MS        10U     /* 外部通信任务 10ms 调度一次，用于接收 UART2 空闲包。 */
#define EXTERNAL_COMM_HEARTBEAT_PERIOD_MS   1000U   /* 心跳 1000ms 主动上传一次，可按现场需求单独改宏。 */

#define EXTERNAL_COMM_STATUS_ONLINE         0x01U   /* 心跳状态值：设备在线。 */
#define EXTERNAL_COMM_STATUS_OFFLINE        0xFFU   /* 心跳状态值：设备掉线、未选中或无效。 */
#define EXTERNAL_COMM_STATUS_SELECTED_A     0x01U   /* 心跳当前通道字段：A 通道被选中。 */
#define EXTERNAL_COMM_STATUS_SELECTED_B     0x02U   /* 心跳当前通道字段：B 通道被选中。 */
#define EXTERNAL_COMM_STATUS_STANDBY        0x01U   /* 心跳运行字段：当前手柄待机。 */
#define EXTERNAL_COMM_STATUS_RUNNING        0x02U   /* 心跳运行字段：当前手柄运行中。 */
#define EXTERNAL_COMM_STATUS_UNPLUGGED      0x03U   /* 心跳运行字段：当前选中通道未接入手柄。 */
#define EXTERNAL_COMM_HEARTBEAT_INFO_MAX_LEN 21U    /* 新版心跳 InforArea 最大长度：A/B 手柄各上传2字节EEPROM原始类型，且运行速度/电流、A/B泵信息全部存在。 */

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
#define EXTERNAL_COMM_DOWN_ACK              0xBBU   /* 下行命令：外部设备应答，本机 V1 不处理。 */
#define EXTERNAL_COMM_DOWN_PERMISSION       0xFAU   /* 下行命令：功能升级或权限开放。 */

static kernel_task_t ExternalCommTaskHandle;         /* 外部通信任务句柄，由调度器保存任务状态。 */
static uint16_t s_heartbeat_elapsed_ms = 0U;         /* 心跳累计时间，每次任务运行增加 10ms。 */

static uint8_t s_rx_buf[UART2_MAX_PACKET_SIZE];      /* UART2 DMA 空闲包复制到这里后再解析。 */
static uint8_t s_tx_buf[EXTERNAL_COMM_MAX_FRAME_SIZE]; /* 所有上传帧共用发送缓存，任务内串行使用。 */
static uint8_t s_page_buf[AT24CS32_PAGE_SIZE];       /* EEPROM 页缓存，32 字节含最后 2 字节页校验。 */

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

static void ExternalComm_LoadChannelMemory(uint8_t channel)
{
    /* memory 指向当前通道保存的 EEPROM/识别结果缓存。 */
    ChannelMemoryMessagr_t *memory;

    /* A 通道取 MemoryMsgA，B 通道取 MemoryMsgB。 */
    memory = (channel == CHANNEL_A) ? &MemoryMsgA : &MemoryMsgB;
    /* 同步保护电流。 */
    WorkMessage.current_work = memory->current_work;
    /* 同步工具减速比。 */
    WorkMessage.tool_reduction_ratio = memory->tool_reduction_ratio;
    /* 同步运行方向。 */
    WorkMessage.dir_work = memory->dir;
    /* 同步控制模式。 */
    WorkMessage.drivetype_work = memory->drive_type;
    /* 同步往复频率。 */
    WorkMessage.freq_work = memory->freq;
    /* 同步刨/磨工具类型。 */
    WorkMessage.tool_type = memory->tool_type;
    /* 同步手柄模型。 */
    WorkMessage.hand_model = memory->hand_model;
    /* 最后切换当前工作通道，后续启动命令会使用这个通道。 */
    WorkMessage.channel_work = channel;

    /* 正转方向使用该通道保存的正转速度。 */
    if (WorkMessage.dir_work == ZZDIR)
    {
        WorkMessage.speed_set_work = memory->zz_speed;
    }
    /* 反转方向使用该通道保存的反转速度。 */
    else if (WorkMessage.dir_work == FZDIR)
    {
        WorkMessage.speed_set_work = memory->fz_speed;
    }
    /* 其他方向当前按往复处理，使用往复速度。 */
    else
    {
        WorkMessage.speed_set_work = memory->osc_speed;
    }

    /* 实际输出速度同步为当前设置速度，避免切换通道后保留旧速度。 */
    WorkMessage.speed_work = WorkMessage.speed_set_work;
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

    /* 置位外部控制激活标志。 */
    WorkMessage.hmiactive_work = 1U;
    /* 驱动方式切到外部控制。 */
    WorkMessage.drivetype_work = TOUCHWORK;
    /* 允许外部控制链路参与电机控制。 */
    ControlSignalMessage.HMI_enable_flag = true;
    /* 申请成功只拿控制权，不直接启动运行。 */
    ControlSignalMessage.HMI_control_flag = false;
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

static void ExternalComm_ApplySetting(const ExternalCommFrame_t *frame)
{
    /* value 保存外部设备下发的速度、频率或泵速度。 */
    uint16_t value;
    /* 成功应答载荷固定为 AreaCode + 2 字节值。 */
    uint8_t info[3];

    /* 设置运行值必须依附当前 A/B 通道，否则不知道该写哪套通道状态。 */
    if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
    {
        ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_RUN_SET_FAILED,
                                 frame->area_code,
                                 EXTERNAL_COMM_REASON_NO_CHANNEL);
        return;
    }

    /* 速度、A 泵速度、B 泵速度按 2 字节大端值解析。 */
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
            /* 设置 A 泵速度，不切换泵启停状态。 */
            pumpMessageA.speed_work = value;
            break;
        case 0x04U:
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
            ExternalComm_LoadChannelMemory(CHANNEL_A);
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
            ExternalComm_LoadChannelMemory(CHANNEL_B);
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

static uint8_t ExternalComm_EnsureActiveForRun(uint8_t allow_stop)
{
    /* 报警状态下禁止外部启动或继续动作，停止类命令也让上层走急停分支。 */
    if (WorkMessage.alarm_flag)
    {
        return 0U;
    }

    /* 非停止类命令必须先申请外部控制，防止外部设备未授权直接启动。 */
    if ((allow_stop == 0U) && (WorkMessage.hmiactive_work == 0U))
    {
        return 0U;
    }

    /* 当前允许执行控制命令。 */
    return 1U;
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
    /* 清除左脚踏控制标志。 */
    ControlSignalMessage.jtL_control_flag = false;
    /* 清除右脚踏控制标志。 */
    ControlSignalMessage.jtR_control_flag = false;
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
}

static void ExternalComm_ApplyControlCommand(const ExternalCommFrame_t *frame)
{
    /* 控制成功应答只回显 1 字节控制代号。 */
    uint8_t info[1];

    /* A/B 泵停止、当前手柄停止、急停允许在未激活时执行，其余动作必须先申请外部控制。 */
    if (ExternalComm_EnsureActiveForRun((frame->area_code == 0x02U) ||
                                        (frame->area_code == 0x04U) ||
                                        (frame->area_code == 0x06U) ||
                                        (frame->area_code == 0xFFU)) == 0U)
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
            /* A 泵启动前要求 CS1237 上报过合法霍尔设备类型码。 */
            if (pumpMessageA.online_flag == false)
            {
                ExternalComm_SendFailAck(EXTERNAL_COMM_ACK_CONTROL_FAILED,
                                         frame->area_code,
                                         EXTERNAL_COMM_REASON_DEVICE_FAIL);
                return;
            }
            /* 置位 A 泵运行标志。 */
            pumpMessageA.run_flag = true;
            /* 外部普通启动不进入排空计时模式。 */
            pumpMessageA.timingDrainage_flag = false;
            break;
        case 0x02U:
            /* 清除 A 泵运行标志。 */
            pumpMessageA.run_flag = false;
            /* 同时清除 A 泵排空计时。 */
            pumpMessageA.timingDrainage_flag = false;
            break;
        case 0x03U:
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
            /* 当前手柄启动必须有 A/B 当前通道。 */
            if ((WorkMessage.channel_work != CHANNEL_A) && (WorkMessage.channel_work != CHANNEL_B))
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
            break;
        case 0x06U:
            /* 停止当前手柄运行。 */
            WorkMessage.runflag_work = false;
            /* 停止时清零实际输出速度。 */
            WorkMessage.speed_work = 0U;
            /* 清除外部控制启动标志。 */
            ControlSignalMessage.HMI_control_flag = false;
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
        case EXTERNAL_COMM_DOWN_ACK:
            /* 外部设备对应答的应答不需要本机再回包，直接忽略。 */
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

static uint8_t ExternalComm_FootPedalOnlineStatus(void)
{
    /* 脚踏老逻辑有两个连接字段，任一字段为 Connect 就认为脚踏在线。 */
    if ((SysFootPedalData.FootPedalConnectOkNo == Connect) ||
        (SysFootPedalData.FootPedalConnectFlag == Connect))
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

    /* 泵在线时才继续追加泵类型和泵速度，离线时省略这两个字段。 */
    if (pump_message->online_flag)
    {
        /* 泵类型当前来自 CS1237 DeviceCode，协议线上按 1 字节设备类型码上传。 */
        ExternalComm_HeartbeatAppendU8(info_area, info_len, (uint8_t)(pump_message->type & 0xFFU));
        /* 泵速度是业务数值，按 2 字节大端上传。 */
        ExternalComm_HeartbeatAppendBE16(info_area, info_len, pump_message->speed_work);
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
    /* 当前手柄运行中时，按协议继续追加当前通道工作速度和工作电流。 */
    if (run_status == EXTERNAL_COMM_STATUS_RUNNING)
    {
        /* 当前通道手柄工作速度，2 字节大端。 */
        ExternalComm_HeartbeatAppendBE16(heartbeat_info, &heartbeat_len, WorkMessage.speed_work);
        /* 当前通道手柄工作电流，2 字节大端。 */
        ExternalComm_HeartbeatAppendBE16(heartbeat_info, &heartbeat_len, WorkMessage.current_work);
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
        /* 心跳组帧成功后从 UART2 主动上传。 */
        Uart2_SendPacket(s_tx_buf, tx_len);
    }
}

static void ExternalComm_ProcessReceive(void)
{
    /* recv_len 保存 UART2 DMA 空闲包长度。 */
    uint16_t recv_len;
    /* frame 保存协议层解析出的字段和 InforArea。 */
    ExternalCommFrame_t frame;

    /* 从 UART2 DMA 缓存取出一包已经静默稳定的数据。 */
    recv_len = Uart2_DMARecvDataPeek(s_rx_buf);
    /* 没有完整空闲包时本周期不处理。 */
    if (recv_len == 0U)
    {
        return;
    }

    /* 只有帧头、长度、帧尾、CRC 全部通过时才进入业务调度。 */
    if (ExternalCommProtocol_Parse(s_rx_buf, recv_len, &frame) == EXTERNAL_COMM_PARSE_OK)
    {
        /* 按 FunCode 分发下行命令。 */
        ExternalComm_DispatchFrame(&frame);
    }
}

static void ExternalCommTaskFunc(uint32_t event)
{
    /* 当前调度器未使用 event，显式丢弃避免编译器告警。 */
    (void)event;

    /* 每 10ms 检查一次 UART2 是否收到完整空闲包。 */
    ExternalComm_ProcessReceive();

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
    /* 创建 UART2 外部通信任务。 */
    Kernel_TaskCreate(&ExternalCommTaskHandle, ExternalCommTaskFunc);
    /* 任务常驻运行，每 10ms 执行一次接收和心跳调度。 */
    Kernel_TaskStart(&ExternalCommTaskHandle, KERNEL_TASK_ALWAYS, EXTERNAL_COMM_TASK_PERIOD_MS);
}
