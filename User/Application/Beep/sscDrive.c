#include "sscDRIVE.h"
#include "data.h"
#include "board_profile.h"


#include "lcd.h"
#include "common.h"
#include "iwdg.h"

#include <stdint.h>
#include "kernel_scheduler.h"
#include "lcd.h"
#include "uart1.h"
#include "motoruartdata.h"
#include "Pubinterface.h"
#include "sscUIDP.h"
#include "sscRFID.h"
#include "sscFOOT.h"
#include "delay.h"

#include <string.h>

#define motor_frem_length  11 /* 电机控制帧共 11 字节，含末尾 2 字节 CRC；修改长度必须同步驱动协议。 */
#define MOTOR_DRIVE_CMD_FREQ_MAX 100U /* 往复频率字段允许 0~100，超过时按 100 发送；这是协议值，不直接表示 Hz。 */
#define MOTOR_DRIVE_RATIO_X100_UNIT 100U /* 倍率放大 100 倍保存：100=1.00 倍、525=5.25 倍；须与 EEPROM/RFID 定义一致。 */
#define MOTOR_DRIVE_CMD_SPEED_UNIT_RPM 10U /* 每个速度单位为 10rpm：3000rpm 发 300；修改会改变所有手柄的下发转速。 */
#define MOTOR_DRIVE_CMD_SPEED_MAX 0xFFFFU /* 速度字段最多 65535（16 位）；这是协议容量，不是手柄允许的工作转速。 */
#define MOTOR_DRIVE_RPM_MAX (MOTOR_DRIVE_CMD_SPEED_MAX * MOTOR_DRIVE_CMD_SPEED_UNIT_RPM) /* 协议最多表示 655350rpm，仅防止数值溢出；实际工作上限由手柄参数限制。 */
#define MOTOR_DRIVE_SPEED_UP_SHIFT 16U /* WorkMessage.tool_reduction_ratio 高 16 位表示增速比。 */
#define MOTOR_DRIVE_REDUCTION_MASK 0xFFFFU /* WorkMessage.tool_reduction_ratio 低 16 位表示减速比。 */
#define MOTOR_DRIVE_DISPLAY_SPEED_INVALID 0xFFFFFFFFUL /* 速度显示缓存的无效值，用于强制下一次运行刷新屏幕速度。 */
#define MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM 100U /* 脚踏速度显示步长，单位 rpm；向下取整到 100 的倍数，只影响显示，不改变电机速度。 */
#define MOTOR_TOOL_POSITION_GUARD_MS 150U /* 定位命令后暂停周期空闲停止帧的时间，单位 ms；过短可能打断定位，过长会延后空闲停止帧。启动或报警会提前结束等待。 */
#define MOTOR_DRIVE_ZERO_REARM_GAP_MS 5U /* 从停止到启动时，先发同通道零速帧，再间隔 5ms 发运行帧；过短可能被驱动当成同一包，调大增加启动等待。 */

kernel_task_t MOTORRUNTaskHandle;
static const uint8_t s_motor_stop_fallback[motor_frem_length]={0xAA ,0x01 ,0x00 ,0x01 ,0x00 ,0x00 ,0x02 ,0x00 ,0x00  ,0x00 ,0x00}; /* 上电尚未形成运行模板时只发送历史兼容零速帧，此状态下主控从未授权本次电机运行。 */
static uint8_t s_motor_stop_template[motor_frem_length]; /* 保存最近一份真实 RUN 帧除速度外的全部协议字段，STOP 必须沿用同一物理通道、方向和运行方式。 */
static uint8_t s_motor_stop_template_valid = 0U; /* 首次组装真实 RUN 帧后置 1，防止未初始化模板被周期停机任务使用。 */

typedef struct {
  
    uint8_t control_mode; //控制模式[0x01]正转；[0x02]:反转；[0x03]:往复正反转;[0x04]:正向拖动模式；[0x05]:反向拖动模式
    uint8_t frequency; //往复频率协议值，原协议 20 对应 1Hz、80 对应 4Hz；此处不再乘 2。
    uint8_t motor_type; //电机选择：0x01：无刷通道1  0x02:无刷通道2 0x03:有刷通道1 0x04 有刷通道2
    uint8_t speed_h; //转速高
    uint8_t speed_l; //转速低
    uint8_t run_type; //闭环运行方式：0x01无霍尔 0x02 有霍尔 0x03:有刷刀头1  0x04:有刷刀头2  
    uint8_t pro_current_h; //保护电流高
    uint8_t pro_current_l; //保护电流低
    
}
RunInformMessage_t;
static RunInformMessage_t msg;
static MotorDriveCommandSnapshot_t s_motor_drive_command_snapshot; /* 保存最近一次实际改变并送入UART1的周期电机命令。 */
static uint8_t s_motor_drive_last_command[motor_frem_length]; /* 保存上一份11字节周期命令，用于排除每50ms重复保活帧。 */
static uint8_t s_motor_drive_last_command_valid = 0U; /* 0表示尚未保存命令；首次生成启动或停止帧时必须保存一份记录。 */
static volatile uint32_t s_motor_drive_snapshot_version = 0U; /* 偶数表示数据可读，奇数表示正在更新；读取前后比较此值，避免混用两次命令。 */
static volatile uint8_t s_tool_position_guard_active = 0U; /* 1 表示已发出定位命令，暂不重发空闲停止帧，给驱动留出完成定位的时间。 */
static volatile uint32_t s_tool_position_guard_until_ms = 0U; /* 保存定位保护截止时刻，配合有符号差值兼容HAL毫秒计数回绕。 */

/*
 * 函数功能：把往复频率限制在驱动允许的 0~100 范围内。
 * 输入参数：freq_work 为当前频率协议值，不是直接以 Hz 表示的数值。
 * 返回参数：可直接写入控制帧 byte2 的频率值。
 */
static uint8_t MotorDrive_BuildCommandFrequency(uint16_t freq_work)
{
    if (freq_work > MOTOR_DRIVE_CMD_FREQ_MAX)
    {
        return (uint8_t)MOTOR_DRIVE_CMD_FREQ_MAX; /* EEPROM 或上位机给出的频率超过协议范围时，按驱动允许的最大值下发。 */
    }

    return (uint8_t)freq_work; /* 参考驱动接收端会再执行 `R_DATA[2] * 2`，主控这里保持原始命令值，不再提前翻倍。 */
}

/*
 * 函数功能：计算 11 字节电机控制帧的 CRC16，并写到最后两个字节。
 * 输入参数：command 指向待发送的完整控制帧，前 9 字节必须已经组装完成。
 * 返回参数：无；command 为空时不处理。
 */
static void MotorDrive_UpdateCommandCrc(uint8_t *command)
{
    uint16_t crc; /* 保存前 9 字节的 CRC16/MODBUS 结果，驱动端按低字节在前校验。 */

    if (command == NULL)
    {
        return; /* 内部命令缓存无效时不得写 CRC 字段，调用方也不会发送该帧。 */
    }

    crc = Common_Crc16(command, (uint16_t)(motor_frem_length - 2U)); /* CRC 覆盖地址、模式、通道、速度和保护电流。 */
    command[motor_frem_length - 2U] = (uint8_t)(crc & 0x00FFU); /* 协议第 10 字节发送 CRC 低字节。 */
    command[motor_frem_length - 1U] = (uint8_t)((crc >> 8U) & 0x00FFU); /* 协议第 11 字节发送 CRC 高字节。 */
}

/*
 * 函数功能：从所有类型手柄的本次 RUN 帧生成后续 STOP 模板，只清零速度并重新计算 CRC。
 * 输入参数：run_command 指向已经完成方向、物理通道、运行方式、电流和 CRC 组装的 11 字节 RUN 帧。
 * 返回参数：无；空指针时保留上一份可信模板。
 */
static void MotorDrive_UpdateStopTemplate(const uint8_t *run_command)
{
    if (run_command == NULL)
    {
        return; /* 内部组帧异常时不能破坏上一份可用于安全停机的真实模板。 */
    }

    memcpy(s_motor_stop_template, run_command, motor_frem_length); /* 无刷/有刷、物理A/B、霍尔/无霍尔及各方向都保留各自 RUN 配置。 */
    s_motor_stop_template[4] = 0U; /* 协议 byte4 为速度高字节，STOP 模板固定清零。 */
    s_motor_stop_template[5] = 0U; /* 协议 byte5 为速度低字节，与高字节共同形成零速命令。 */
    MotorDrive_UpdateCommandCrc(s_motor_stop_template); /* 速度改变后重新生成 CRC，驱动端才能接受该停止帧。 */
    s_motor_stop_template_valid = 1U; /* 所有 11 字节完成后再开放模板，周期任务不会读取半更新数据。 */
}

/*
 * 函数功能：判断当前通道是否需要按有刷一体刨/一体磨协议下发驱动帧。
 * 输入参数：hand_model 为 EEPROM 第二页手柄型号；tool_type 为主控使用的刀具类型；raw_tool_type 为 EEPROM/RFID 原始刀具型号。
 * 返回参数：1 表示按有刷电机通道下发；0 表示按无刷/霍尔通道下发。
 */
static uint8_t MotorDrive_IsBrushedTool(uint8_t hand_model, uint8_t tool_type, uint8_t raw_tool_type)
{
    return (uint8_t)((hand_model == PX_YIP_ONLINES) ||
                     (hand_model == PX_YIM_ONLINES) ||
                     (tool_type == PX_YIP_ONLINES) ||
                     (tool_type == PX_YIM_ONLINES) ||
                     (raw_tool_type == PX_YIP_ONLINES) ||
                     (raw_tool_type == PX_YIM_ONLINES)); /* 只有PXYTM/PXYTP进入有刷类型；RFID 0x04/0x05明确保持公共接头无刷通道。 */
}

/*
 * 函数功能：按分体手柄手动刀具模式或RFID机械刀具规则，把屏幕方向转换为实际电机方向。
 * 输入参数：hand_model为当前Page2基座型号；display_direction为屏幕方向；raw_tool_type为EPC byte0原始刀具型号；auto_identify为0表示手动选刀，非0表示RFID自动识别。
 * 返回参数：写入驱动控制模式的ZZDIR/FZDIR/OSCDIR；不修改屏幕和通道记忆。
 */
static uint8_t MotorDrive_BuildActualDirection(uint8_t hand_model, uint8_t display_direction, uint8_t raw_tool_type, uint8_t auto_identify)
{
    bool rfid_tool_handle = ((hand_model == PXBA_ONLINES) ||
                             (hand_model == PXBB_ONLINES) ||
                             (hand_model == COMMON_SOCKET_ONLINES)); /* 只有公共接头/PXB 手柄读取的 RFID 刀具码，才按 0x03~0x06 的方向规则处理。 */

    if(((hand_model == PXBA_ONLINES) || (hand_model == PXBB_ONLINES)) && (auto_identify == 0U))
    {
        if(display_direction == ZZDIR)
        {
            return FZDIR; /* 分体手柄手动选刀时，屏幕正转对应实际电机反转，不改显示和通道记忆。 */
        }
        if(display_direction == FZDIR)
        {
            return ZZDIR; /* 分体手柄手动选刀时，屏幕反转对应实际电机正转，不叠加旧RFID型号的反向规则。 */
        }
        return display_direction; /* 手动刨刀往复仍执行往复，只互换单向正反转。 */
    }

    if (rfid_tool_handle == false)
    {
        return display_direction; /* EEPROM Page3历史刀具码可能与0x03~0x06重号，非RFID基座必须保持原方向。 */
    }

    if (raw_tool_type == RFID_TOOL_MODEL_MXYTP)
    {
        return ZZDIR; /* MXYTP由机械结构形成往复，电机必须始终单向正转。 */
    }

    if ((raw_tool_type == RFID_TOOL_MODEL_REVERSE_ROTATION) ||
        (raw_tool_type == RFID_TOOL_MODEL_REVERSE_PLANER))
    {
        if (display_direction == ZZDIR)
        {
            return FZDIR; /* 屏幕选择正向单向运行时，反旋刀具和反向刨刀都需要电机实际反转。 */
        }

        if (display_direction == FZDIR)
        {
            return ZZDIR; /* 屏幕选择反向单向运行时，反旋刀具和反向刨刀都需要电机实际正转。 */
        }

        if ((raw_tool_type == RFID_TOOL_MODEL_REVERSE_PLANER) && (display_direction == OSCDIR))
        {
            return OSCDIR; /* 0x06反向刨刀的默认往复保持电气往复，只对用户选择的正转或反转单向模式取反。 */
        }

        return ZZDIR; /* 0x03 反旋刀具不使用电机正反往复；其余未处理方向返回正转，是否允许启动仍由上层检查。 */
    }

    return display_direction; /* 普通刨磨刀具和MXYTM的屏幕方向就是实际电机方向。 */
}

/*
 * 函数功能：按PX分体式手柄的实际电机方向选择无刷有霍尔或无霍尔运行方式。
 * 输入参数：hand_model 为当前基座型号；actual_direction 为机械刀具方向换算后的实际电机方向。
 * 返回参数：0x01表示无刷无霍尔，0x02表示无刷有霍尔。
 */
static uint8_t MotorDrive_BuildBrushlessRunType(uint8_t hand_model, uint8_t actual_direction)
{
    if ((hand_model == PXBB_ONLINES) || (hand_model == PXBA_ONLINES))
    {
        return (actual_direction == OSCDIR) ? 0x02U : 0x01U; /* 只有电机真实执行电气往复才使用霍尔；正反单向及MXYTP机械往复均使用无霍尔。 */
    }

    return 0x01U; /* 其他手柄默认按无霍尔方式下发，保持旧工程的兼容行为。 */
}

/*
 * 函数功能：把已经完成机械倍率换算的电机实际 rpm 转成 GE2433 启动帧速度字段。
 * 输入参数：motor_speed_rpm 为最终希望电机达到的实际转速，单位 rpm。
 * 返回参数：写入启动帧 byte4~5 的 16 位速度字段，单位 10rpm。
 */
static uint16_t MotorDrive_BuildCommandSpeed(uint32_t motor_speed_rpm)
{
    uint32_t command_speed = motor_speed_rpm / MOTOR_DRIVE_CMD_SPEED_UNIT_RPM; /* GE2433 协议规定速度字段等于实际 rpm/10，例如 3000rpm 写 300。 */

    if (command_speed > MOTOR_DRIVE_CMD_SPEED_MAX)
    {
        command_speed = MOTOR_DRIVE_CMD_SPEED_MAX; /* 实际 rpm 超过协议字段可表达范围时钳到 0xFFFF，避免高低字节回绕。 */
    }

    return (uint16_t)command_speed; /* 返回组帧可直接拆高低字节的协议速度值。 */
}

/*
 * 函数功能：按 x100 减速倍率换算机械减速机构需要的电机速度。
 * 输入参数：motor_speed 为倍率换算前的目标 rpm；ratio 为低 16 位 x100 减速比。
 * 返回参数：换算后的电机 rpm，最多为协议允许的上限；倍率不大于 1.00 时返回原速度。
 */
static uint32_t MotorDrive_ApplyReductionRatioValue(uint32_t motor_speed, uint32_t ratio)
{
    uint64_t converted_speed; /* 使用 64 位保存乘法结果，避免最大速度乘 x100 倍率时发生 32 位回绕。 */

    if (ratio <= MOTOR_DRIVE_RATIO_X100_UNIT)
    {
        return motor_speed; /* 0~100 都表示未配置或不大于 1.00，运行链路统一按直联处理。 */
    }

    converted_speed = ((uint64_t)motor_speed * ratio) / MOTOR_DRIVE_RATIO_X100_UNIT; /* 例如 525 表示 5.25 倍减速，电机端速度需要乘 5.25。 */
    if (converted_speed > MOTOR_DRIVE_RPM_MAX)
    {
        converted_speed = MOTOR_DRIVE_RPM_MAX; /* 转回 32 位前先限制上限，避免过大数值被截断后变成低速。 */
    }

    return (uint32_t)converted_speed; /* 返回驱动协议可表达范围内的实际 rpm。 */
}

/*
 * 函数功能：按 x100 增速倍率换算机械增速机构需要的电机速度。
 * 输入参数：motor_speed 为倍率换算前的目标 rpm；ratio 为高 16 位 x100 增速比。
 * 返回参数：换算后的电机 rpm；倍率不大于 1.00 时返回原速度。
 */
static uint32_t MotorDrive_ApplySpeedUpRatioValue(uint32_t motor_speed, uint32_t ratio)
{
    uint64_t converted_speed; /* 增速换算分子使用 64 位，避免 motor_speed×100 在异常输入下溢出 32 位。 */

    if (ratio <= MOTOR_DRIVE_RATIO_X100_UNIT)
    {
        return motor_speed; /* 0~100 都表示未配置或不大于 1.00，保持屏幕目标速度不变。 */
    }

    converted_speed = ((uint64_t)motor_speed * MOTOR_DRIVE_RATIO_X100_UNIT) / ratio; /* 例如 525 表示 5.25 倍增速，电机端速度需要除以 5.25。 */
    if (converted_speed > MOTOR_DRIVE_RPM_MAX)
    {
        converted_speed = MOTOR_DRIVE_RPM_MAX; /* 转回 32 位前先限制上限，防止输入速度异常时溢出。 */
    }

    return (uint32_t)converted_speed; /* 返回本次增速机构换算后的实际 rpm。 */
}

/*
 * 函数功能：按 EEPROM/RFID 解析出的机械减速比或增速比，把屏幕手柄速度换算成电机输出速度。
 * 输入参数：display_speed 为本次控制源目标速度，脚踏模式取行程比例后的 speed_work，其它模式取设定最大速度 speed_set_work。
 * 返回参数：倍率换算后的电机 rpm，已限制到协议上限；写入 16 位速度字段前还要除以 10。
 */
static uint32_t MotorDrive_ApplyToolReductionRatio(uint32_t display_speed)
{
    uint32_t ratio = WorkMessage.tool_reduction_ratio;        /* 当前通道记忆装载的 x100 完整倍率，高 16 位增速、低 16 位减速。 */
    uint32_t reduction_ratio = ratio & MOTOR_DRIVE_REDUCTION_MASK; /* 低 16 位减速比，减速机构需要放大电机速度。 */
    uint32_t speed_up_ratio = ratio >> MOTOR_DRIVE_SPEED_UP_SHIFT; /* 高 16 位增速比，增速机构需要降低电机速度。 */
    uint32_t motor_speed = display_speed;                     /* 设定速度已经是实际 rpm，例如 6000 表示 6000rpm，倍率换算前不能再除以 10。 */

    if ((reduction_ratio > MOTOR_DRIVE_RATIO_X100_UNIT) &&
        (speed_up_ratio > MOTOR_DRIVE_RATIO_X100_UNIT))
    {
        reduction_ratio = MOTOR_DRIVE_RATIO_X100_UNIT;        /* 双倍率同时大于 1.00 属于 EEPROM 写入错误，保护为 x100 直联值。 */
        speed_up_ratio = 0U;                                  /* 同时清增速分支，避免错误标签让电机速度不可预测。 */
    }

    if (reduction_ratio > MOTOR_DRIVE_RATIO_X100_UNIT)
    {
        motor_speed = MotorDrive_ApplyReductionRatioValue(motor_speed, reduction_ratio); /* 低 16 位大于 100 时按 x100 减速倍率换算。 */
    }
    else if (speed_up_ratio > MOTOR_DRIVE_RATIO_X100_UNIT)
    {
        motor_speed = MotorDrive_ApplySpeedUpRatioValue(motor_speed, speed_up_ratio); /* 高 16 位大于 100 时按 x100 增速倍率换算。 */
    }

    if (motor_speed > MOTOR_DRIVE_RPM_MAX)
    {
        motor_speed = MOTOR_DRIVE_RPM_MAX;                    /* 实际 rpm 超过 GE2433 协议可表达上限时先钳位，后续再按 /10 写入速度字段。 */
    }

    return motor_speed;                                       /* 返回本次启动帧希望电机达到的实际 rpm，组帧前还要按协议 /10。 */
}

/*
 * 函数功能：把脚踏实时速度向下取整到 100rpm 的倍数，只用于显示，并减少重复刷屏。
 * 输入参数：actual_speed 为脚踏任务计算出的完整精度实际目标速度，单位为 rpm。
 * 返回参数：不大于实际目标速度的 100rpm 整数倍；输入小于 100rpm 时返回 0。
 */
static uint32_t MotorDrive_QuantizeFootDisplaySpeed(uint32_t actual_speed)
{
    return (actual_speed / MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM) * MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM; /* 只截掉百位以下数值，不回写 WorkMessage，也不影响电机驱动帧。 */
}

/**
 * 函数功能：按逻辑手柄通道生成开口定位命令，并发送给对应的物理电机通道。
 * 输入参数：channel_number 为逻辑通道；direction 为定位方向；angel 为定位角度。
 * 返回参数：无。
 */
void ToolPosMay(uint8_t channel_number,bool direction,uint8_t angel)
{
 uint8_t cmd[11] ={0xAA ,0x04 ,0x00 ,0x01 ,0x00 ,0x01 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00 };
 uint8_t physical_channel = BoardProfile_MapHandlePhysicalChannel(channel_number); /* 定位命令只交换物理电机路，调用方仍传逻辑A/B。 */

 cmd[3]=(physical_channel==BOARD_PROFILE_HANDLE_CHANNEL_A)?BOARD_PROFILE_HANDLE_CHANNEL_A:BOARD_PROFILE_HANDLE_CHANNEL_B; /* 驱动帧第3字节选择原物理A或B电机通道。 */
 direction==true?(cmd[1]=4):(cmd[1]=5); /* 定位方向沿用原协议4/5，不受通道交换影响。 */
 cmd[5]=angel; /* 定位角度继续写入协议第5字节，保持原单位和范围。 */
 MotorDrive_UpdateCommandCrc(cmd); /* 定位命令也必须携带真实 CRC，否则升级后的手柄驱动会按无效帧拒绝。 */
 s_tool_position_guard_until_ms=HAL_GetTick()+MOTOR_TOOL_POSITION_GUARD_MS; /* 从本次点击开始保留足够时间，使驱动完成准备、单步拖动和末端保持。 */
 s_tool_position_guard_active=1U; /* 先发布保护状态再发送定位帧，避免任务切换时停机保活帧插到定位帧之后。 */
 Uart1_SendPacket(cmd,motor_frem_length); /* 保护窗口已经建立，再通过UART1发送完整定位帧。 */
}


/*
 * 函数功能：记录本次50ms输出判定，并在周期电机命令内容变化时同步记录真实UART1命令序号和时刻。
 * 输入参数：command为11字节驱动帧；logical_channel、run_state和command_speed_rpm描述实际帧；其余参数描述原始运行请求。
 * 返回参数：无；重复帧仍更新请求诊断字段，但不增加命令序号和sent_tick_ms。
 */
static void MotorDrive_RecordCommandSnapshot(const uint8_t *command,
                                             uint8_t logical_channel,
                                             uint8_t run_state,
                                             uint32_t command_speed_rpm,
                                             uint8_t requested_run_state,
                                             uint8_t drive_type,
                                             uint32_t source_speed_rpm,
                                             uint8_t zero_speed_blocked)
{
    uint8_t command_changed = 0U; /* 只有完整11字节帧变化才递增序号，50ms重复保活只刷新请求诊断。 */
    uint32_t evaluated_tick_ms; /* 一次读取HAL时钟同时供本次判定时刻和新命令发送时刻使用。 */

    if (command == NULL)
    {
        return; /* 没有提供命令数据时保留上次记录，避免上位机读到只更新一部分的字段。 */
    }

    evaluated_tick_ms = HAL_GetTick(); /* 记录本周期完成启停判断的毫秒时刻。 */
    if ((s_motor_drive_last_command_valid == 0U) ||
        (memcmp(s_motor_drive_last_command, command, motor_frem_length) != 0))
    {
        command_changed = 1U; /* 首次发命令，或命令任一字节改变时，记为一条新命令。 */
    }

    ++s_motor_drive_snapshot_version; /* 先把版本改成奇数，外部通信任务会等待本次写入结束。 */
    __DMB(); /* 确保先写入奇数版本，再更新命令字段，让其它任务知道数据正在修改。 */
    s_motor_drive_command_snapshot.evaluated_tick_ms = evaluated_tick_ms; /* 每个周期都刷新，断点可确认输出任务仍在调度。 */
    s_motor_drive_command_snapshot.source_speed_rpm = source_speed_rpm; /* 保存倍率和补偿前的原始速度，直接定位零速RUN来源。 */
    s_motor_drive_command_snapshot.requested_run_state = requested_run_state; /* 保存业务层提出的RUN/STOP，不用反推实际UART帧。 */
    s_motor_drive_command_snapshot.drive_type = drive_type; /* 保存脚踏、手控、触控或外控来源，定位是哪条控制路径留下请求。 */
    s_motor_drive_command_snapshot.zero_speed_blocked = zero_speed_blocked; /* 要求启动但速度为 0、实际改发停止帧时记为 1。 */
    if (command_changed != 0U)
    {
        memcpy(s_motor_drive_last_command, command, motor_frem_length); /* 保存完整实际帧，方向、电机类型或电流变化也会触发新序号。 */
        s_motor_drive_last_command_valid = 1U; /* 首份命令记录完成后开放后续逐字节判重。 */
        s_motor_drive_command_snapshot.sequence = (uint16_t)(s_motor_drive_command_snapshot.sequence + 1U); /* 不同命令序号按16位自然回绕。 */
        s_motor_drive_command_snapshot.sent_tick_ms = evaluated_tick_ms; /* 记录本次命令改变的时刻；具体在发送前还是发送后记录，由调用位置决定。 */
        s_motor_drive_command_snapshot.command_speed_rpm = command_speed_rpm; /* 保存机械倍率和协议量化后的实际命令rpm。 */
        s_motor_drive_command_snapshot.channel = logical_channel; /* 保存形成本帧时的逻辑通道，避免仅凭物理电机类型反推A/B。 */
        s_motor_drive_command_snapshot.run_state = run_state; /* 启动帧写1，周期停止帧和零速重装帧写0。 */
        s_motor_drive_command_snapshot.direction = command[1]; /* 直接保存实际帧byte1，包含机械方向换算后的控制模式。 */
        s_motor_drive_command_snapshot.motor_type = command[3]; /* 直接保存实际帧byte3，反映板级A/B映射和有刷/无刷类型。 */
        s_motor_drive_command_snapshot.valid = 1U; /* 已生成并保存一份UART1命令，之后允许其它任务读取这份记录。 */
    }
    __DMB(); /* 保证所有字段完成后才发布最终偶数版本。 */
    ++s_motor_drive_snapshot_version; /* 写完后将版本恢复为偶数，其它任务才可复制这份命令记录。 */
}

/*
 * 函数功能：读取电机请求和命令记录，读取途中发生更新时重试，避免混用新旧数据。
 * 输入参数：snapshot指向接收命令记录的结构体，复制后调用方只读取自己的这份数据。
 * 返回参数：已有有效命令记录且复制期间未被修改时返回1，否则返回0。
 */
uint8_t MotorDrive_CopyCommandSnapshot(MotorDriveCommandSnapshot_t *snapshot)
{
    uint8_t attempt; /* 限制瞬时并发时的重试次数，避免通信任务在异常状态下长期占用CPU。 */

    if (snapshot == NULL)
    {
        return 0U; /* 调用方未提供缓存时不能复制。 */
    }

    for (attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t version_before = s_motor_drive_snapshot_version; /* 复制前读取版本，奇数表示驱动任务仍在写。 */
        uint32_t version_after; /* 复制后再次读取版本，只有前后一致才表示字段属于同一命令。 */

        if ((version_before & 1U) != 0U)
        {
            continue; /* 写入窗口通常只有数个指令周期，下一次循环直接重试。 */
        }

        __DMB(); /* 先检查版本是否允许读取，再读取命令字段，不能交换这两个操作的顺序。 */
        *snapshot = s_motor_drive_command_snapshot; /* 结构体一次复制到调用方私有缓存，后续组包不再读取共享状态。 */
        __DMB(); /* 字段复制结束后再复核版本。 */
        version_after = s_motor_drive_snapshot_version;
        if ((version_before == version_after) && ((version_after & 1U) == 0U))
        {
            return (snapshot->valid != 0U) ? 1U : 0U; /* 复制期间未发生更新，再用valid判断是否已有有效命令记录。 */
        }
    }

    memset(snapshot, 0, sizeof(*snapshot)); /* 连续三次都遇到数据更新时清空输出并返回失败，避免上传混用新旧值的数据。 */
    return 0U;
}

/*
 * 函数功能：发送电机停止帧，并记录原请求是否为启动、是否因为零速而改发停止。
 * 输入参数：requested_run_state、drive_type、source_speed_rpm和zero_speed_blocked描述本次输出判定来源。
 * 返回参数：无。
 */
static void MotorStops(uint8_t requested_run_state,
                       uint8_t drive_type,
                       uint32_t source_speed_rpm,
                       uint8_t zero_speed_blocked)
{
 uint8_t motor_stopcode[motor_frem_length]; /* 本周期私有 STOP 帧，避免公共模板被后续 RUN 更新时影响当前发送。 */

 if (s_motor_stop_template_valid != 0U)
 {
     memcpy(motor_stopcode, s_motor_stop_template, motor_frem_length); /* 正常停机沿用最近 RUN 的通道、方向、run_type、电流和频率。 */
 }
 else
 {
     memcpy(motor_stopcode, s_motor_stop_fallback, motor_frem_length); /* 上电尚无 RUN 时使用零速兼容帧，不读取未初始化模板。 */
 }
 motor_stopcode[4] = 0U; /* 发送前再次强制速度高字节为 0，避免任何模板异常产生非零停止帧。 */
 motor_stopcode[5] = 0U; /* 发送前再次强制速度低字节为 0，与高字节共同形成零速命令。 */
 MotorDrive_UpdateCommandCrc(motor_stopcode); /* 最终零速字段确定后重新生成 CRC。 */
 MotorDrive_RecordCommandSnapshot(motor_stopcode, WorkMessage.channel_work, 0U, 0U,
                                  requested_run_state, drive_type, source_speed_rpm, zero_speed_blocked); /* 实际STOP和原始请求同源记录后再送入UART1。 */
 (void)Uart1_SendPacket(motor_stopcode, motor_frem_length); /* 周期停止帧按最近 RUN 的完整配置下发，所有手柄类型共用同一路径。 */
}

/*
 * 函数功能：把 msg 组装成启动帧发送，并记录控制来源、原始速度和实际命令速度。
 * 输入参数：command_speed_rpm 为命令速度，单位 rpm；drive_type 为控制方式；source_speed_rpm 为换算前的非零目标速度。
 * 返回参数：无。
 */
static void MotorStart(uint32_t command_speed_rpm,
                       uint8_t drive_type,
                       uint32_t source_speed_rpm)
{
    /* msg.pro_current_h/l 已在 MOTORRUN() 中由 WorkMessage.current_work 拆分，发送前不能再改写，否则会覆盖 EEPROM/上位机设置的保护电流。 */
    uint8_t motor_startcode[motor_frem_length]={0xAA ,msg.control_mode ,msg.frequency ,msg.motor_type\
      ,msg.speed_h ,msg.speed_l ,msg.run_type ,msg.pro_current_h ,msg.pro_current_l ,0x00 ,0x00};
    uint8_t motor_rearmcode[motor_frem_length]; /* 启动前先发同通道、同模式的零速帧，让驱动解除“必须先收到停止命令才能再启动”的限制。 */
    HAL_StatusTypeDef transmit_status; /* 保存安全零速帧和 RUN 帧的 UART1 实际发送结果。 */
    bool zero_rearm_required = ((s_motor_drive_command_snapshot.valid == 0U) ||
                                (s_motor_drive_command_snapshot.run_state == 0U)); /* 仅从未发送或停止态进入运行时重装，运行中的速度更新不能插入零速。 */ 
    MotorDrive_UpdateCommandCrc(motor_startcode); /* 启动帧只在全部业务字段确定后计算 CRC，避免速度或电流更新使校验失效。 */
    MotorDrive_UpdateStopTemplate(motor_startcode); /* 所有手柄 RUN 字段确定后生成同配置 STOP 模板，松脚时只允许速度归零。 */
    if (zero_rearm_required)
    {
        memcpy(motor_rearmcode, motor_startcode, sizeof(motor_rearmcode)); /* 复制本次目标帧，保证零速重装作用于即将启动的真实物理通道。 */
        motor_rearmcode[4] = 0U; /* 驱动协议 byte4 为速度高字节，清零后明确表示本帧不得产生运动。 */
        motor_rearmcode[5] = 0U; /* 驱动协议 byte5 为速度低字节，与高字节共同形成安全零速命令。 */
        MotorDrive_UpdateCommandCrc(motor_rearmcode); /* 速度字段改变后必须重新计算 CRC，否则驱动会拒绝本次重装帧。 */
        transmit_status = Uart1_SendPacket(motor_rearmcode, motor_frem_length); /* 新启动沿先下发同配置零速帧，并取得底层真实发送结果。 */
        if (transmit_status != HAL_OK)
        {
            return; /* 安全零速帧发送失败时禁止紧接着发送非零 RUN，下一 50ms 周期重新尝试。 */
        }
        MotorDrive_RecordCommandSnapshot(motor_rearmcode, WorkMessage.channel_work, 0U, 0U,
                                         1U, drive_type, source_speed_rpm, 0U); /* 发送成功后才发布重装 STOP，失败时保持原状态以便下周期继续重装。 */
        Delay_ms(MOTOR_DRIVE_ZERO_REARM_GAP_MS); /* 保留完整串口静默间隔，避免零速帧与后续启动帧被驱动拼成一包。 */
    }
    transmit_status = Uart1_SendPacket(motor_startcode, motor_frem_length); /* 非零速度的RUN命令发送后，再保存供上位机读取的命令记录。 */
    if (transmit_status == HAL_OK)
    {
        MotorDrive_RecordCommandSnapshot(motor_startcode, WorkMessage.channel_work, 1U, command_speed_rpm,
                                         1U, drive_type, source_speed_rpm, 0U); /* 只有成功下发的 RUN 才成为后续重装和状态上传依据。 */
    }
}
/*
 * 函数功能：根据当前 WorkMessage 运行态组装电机驱动帧，向 UART1 电机驱动板下发启动或停止命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void MOTORRUN(void)
{
    static uint8_t huci=0;
    static uint32_t last_display_speed=MOTOR_DRIVE_DISPLAY_SPEED_INVALID; /* 记录脚踏运行时上一次发给屏幕的实时速度，避免每 50ms 无变化也刷屏。 */
    uint8_t display_value[10]={0};
    uint8_t effective_dir_work=0U; /* 保存真正下发给驱动板的方向，RFID机械刀具、DHYTM和EMBD只在输出层转换，不改屏幕和通道记忆。 */
    uint8_t physical_channel=BoardProfile_MapHandlePhysicalChannel(WorkMessage.channel_work); /* 仅把当前逻辑手柄通道换算成物理电机通道。 */
    uint32_t motor_source_speed=WorkMessage.speed_set_work; /* 非脚踏控制时，屏幕/EEPROM 当前设定速度就是电机运行目标速度。 */
    uint32_t display_speed_value=WorkMessage.speed_set_work; /* 非脚踏控制时，屏幕继续显示用户设定的目标速度。 */
    uint32_t ssc_speed_value=0U; /* 保存倍率换算后的电机实际 rpm，后续再按 GE2433 协议除以 10 下发。 */
    uint16_t command_speed_value=0U; /* 保存写入 GE2433 启动帧 byte4~5 的协议速度字段，单位为 10rpm。 */
    uint8_t requested_run_state=0U; /* 先处理脚踏停止要求，再保存最终的原始启停请求，供本周期发送和调试使用。 */
    uint8_t motor_output_run_state=0U; /* 只有原始RUN且原始速度非零时置1，实际UART1才允许发送启动帧。 */
    uint8_t zero_speed_blocked=0U; /* 要求启动但原始速度或协议速度为 0 时置 1，表示本周期改发停止帧。 */
    if((WorkMessage.drivetype_work==JTWORK) && (Foot_IsMotorStopLatched()!=0U))
    {
        WorkMessage.runflag_work=false; /* 脚踏松开或失效锁存优先覆盖普通运行标志，防止其它业务分支把旧 RUN 重新写回。 */
        WorkMessage.speed_work=0U; /* 同步清除脚踏实时目标速度，停机周期不得继续沿用上一帧高 AD 对应的转速。 */
    }
    if(MotorUart_IsDriverParameterTransactionActive()!=0U)
    {
        if((WorkMessage.runflag_work==false) &&
           ((s_motor_drive_command_snapshot.valid==0U) || (s_motor_drive_command_snapshot.run_state!=0U)))
        {
            MotorUart_AbortDriverParameterTransaction(); /* 只有上一份真实UART1命令仍是RUN时，新的停机请求才抢占维护等待。 */
        }
        else
        {
            return; /* 电机早已收到STOP时允许静止调参；运行请求则继续等待事务结束，禁止控制帧与维护响应交叉。 */
        }
    }
    if(s_tool_position_guard_active!=0U)
    {
        if((WorkMessage.runflag_work!=false)||(WorkMessage.alarm_flag!=false)||
           ((int32_t)(s_tool_position_guard_until_ms-HAL_GetTick())<=0))
        {
            s_tool_position_guard_active=0U; /* 收到运行请求、存在报警或等待到期时，结束定位等待，继续处理本周期命令。 */
        }
        else
        {
            return; /* 仅在设备仍处于安全停止态时跳过空闲停机保活帧，防止第一下定位命令被提前覆盖。 */
        }
    }
    /* 脚踏控制使用实时行程速度；其它控制方式继续使用屏幕或 EEPROM 设定速度。 */
    if(WorkMessage.drivetype_work==JTWORK)
    {
        motor_source_speed=WorkMessage.speed_work; /* 脚踏带行程霍尔，speed_work 已由脚踏任务按踩踏比例实时换算。 */
        display_speed_value=MotorDrive_QuantizeFootDisplaySpeed(WorkMessage.speed_work); /* 屏幕只显示 100rpm 整数倍；电机仍使用上方未量化的完整速度。 */
    }
    requested_run_state=(WorkMessage.runflag_work!=false)?1U:0U; /* 在任何倍率和低速补偿前冻结本周期原始启停请求。 */
    if((requested_run_state!=0U) && (motor_source_speed==0U))
    {
        zero_speed_blocked=1U; /* 虽然要求启动，但目标为 0，必须发停止帧，不能被后面的最低 500rpm 补偿改成运行。 */
    }
    if((requested_run_state!=0U) && (motor_source_speed!=0U))
    {
        motor_output_run_state=1U; /* 只有RUN和非零原始速度同时成立，才允许进行机械倍率与启动补偿。 */
        ssc_speed_value=MotorDrive_ApplyToolReductionRatio(motor_source_speed); /* 合法非零目标统一按EEPROM/RFID倍率换算成电机实际rpm。 */
    }
    // if(WorkMessage.hand_model==PX_YIP_ONLINES) /* 仅 PXYTP 临时启用 5 倍减速验证，避免影响其它手柄和后续 EEPROM 正式方案。 */
    // {
    //     ssc_speed_value=WorkMessage.speed_set_work*5U; /* PXYTP 机械端自带 5 倍减速，屏幕仍显示刀具端目标速度，电机端下发速度需要放大 5 倍。 */
    //     if(ssc_speed_value>0xFFFFU) /* 电机驱动协议速度只有高低 2 字节，临时放大后必须避免回绕成异常低速。 */
    //     {
    //         ssc_speed_value=0xFFFFU; /* 超过驱动帧可表达范围时按最大值下发，保证验证过程不会因溢出误判。 */
    //     }
    // }
    /* PXBA/PXBB 分体手柄需要保留原刨刀低速补偿，其它手柄直接使用倍率换算后的速度。 */
    if((motor_output_run_state!=0U) &&
       (WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES))
    {
        /* 只有刨刀在低速下需要补偿启动扭矩，磨头不得进入该速度修正。 */
        if(WorkMessage.tool_type==PLANER)
        {
            /* 低于 1500rpm 时按原规则提高 30%，改善刨刀低速启动能力。 */
            if(ssc_speed_value<1500)
          ssc_speed_value=ssc_speed_value*1.3;
          /* 补偿后仍低于 500rpm 时提高到 500rpm，避免刨刀电机只响不转。 */
          if(ssc_speed_value<500)
          ssc_speed_value=500;
        }
    }

    if(motor_output_run_state!=0U) /* 原始速度与机械换算都完成后，再检查驱动协议最终可表达值。 */
    {
        command_speed_value=MotorDrive_BuildCommandSpeed(ssc_speed_value); /* GE2433字段按rpm/10量化，最终字段为0时仍属于零速命令。 */
        if(command_speed_value==0U)
        {
            motor_output_run_state=0U; /* 禁止用RUN状态发送byte4~5全零帧，统一转入下方周期STOP分支。 */
            zero_speed_blocked=1U; /* source_speed_rpm仍保留原值，可区分原始零速和倍率/协议量化归零。 */
        }
    }
   
    /* 必须同时满足：要求启动、原始速度非零、换算后的协议速度非零；否则发送停止帧。 */
    if(motor_output_run_state!=0U)
    {
        /* 首次起转必须刷新；脚踏运行中只有量化显示速度变化时才再次写屏。 */
        if((huci==0) || ((WorkMessage.drivetype_work==JTWORK) && (last_display_speed!=display_speed_value)))
        {
            display_value[0] = (uint8_t)(display_speed_value >> 16); /* 运行速度高字节按 UIDP 协议传输，脚踏时来自实时行程速度。 */
            display_value[1] = (uint8_t)((display_speed_value >> 8)&0xFFU); /* 运行速度中字节按 UIDP 协议传输，保证 24 位速度完整显示。 */
         display_value[2] = (uint8_t)(display_speed_value & 0xFFU);
            display_value[3] = 1U;             
            display_value[4] = 1U; 
            huci=1;
            last_display_speed=display_speed_value; /* 缓存本次运行显示速度，脚踏行程变化后下一周期才再次刷新屏幕。 */
	SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 刷新运行速度，脚踏模式下随行程实时变化，非脚踏模式仍只在起转时刷新。 */
     LCD_Show_2byte_Number(0x9473,0xffE0);
        }
        //msg的数据填充
        effective_dir_work=MotorDrive_BuildActualDirection(WorkMessage.hand_model, (uint8_t)WorkMessage.dir_work, WorkMessage.raw_tool_type, WorkMessage.auto_identify); /* 分体手柄手动选刀互换实际正反转，自动识别继续按RFID刀具换算，屏幕方向保持原值。 */
        if(WorkMessage.hand_model==DHYTM_ONLINES) /* DHYTM屏幕固定显示反转，但内部反旋机械结构要求电机始终正转。 */
        {
            effective_dir_work=ZZDIR; /* 只覆盖本次驱动帧方向，不回写屏幕和通道记忆，确保界面仍固定显示反转。 */
        }
        else if(WorkMessage.hand_model==EMBD_ONLINES) /* EMBD 现场电机实际方向与协议方向相反，只针对该手柄在驱动帧前取反。 */
        {
            if(effective_dir_work==ZZDIR) /* 屏幕/记忆认为正转时，EMBD 实际需要向驱动板发送反转。 */
            {
                effective_dir_work=FZDIR; /* 只改本次局部下发方向，不回写 WorkMessage.dir_work。 */
            }
            else if(effective_dir_work==FZDIR) /* 屏幕/记忆认为反转时，EMBD 实际需要向驱动板发送正转。 */
            {
                effective_dir_work=ZZDIR; /* 只互换正反转，往复方向不在 EMBD 正常能力范围内，保持原值。 */
            }
        }
        switch(effective_dir_work)
        {
             case ZZDIR: 
             msg.control_mode=0x01;
             msg.frequency=0;
             break;
             case FZDIR: 
             msg.control_mode=0x02;
             msg.frequency=0;
              break;
             case OSCDIR: 
             msg.control_mode=0x03;
             msg.frequency=MotorDrive_BuildCommandFrequency(WorkMessage.freq_work);//参考驱动内部会再乘2，这里只下发协议原值
             break;
             default:
             break;
        }
        if(physical_channel==BOARD_PROFILE_HANDLE_CHANNEL_A) /* 原物理A电机通道。 */
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x01; /* 原物理A无刷电机使用协议类型0x01；逻辑状态不随物理交换改变。 */
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model, effective_dir_work); /* PX分体式按实际电机方向选择有/无霍尔，机械方向换算必须先于此处。 */
                
            }
            else{
             msg.motor_type=0x03; /* 原物理A有刷电机固定使用协议类型0x03。 */
              msg.run_type=0x03; /* 有刷物理A路的运行类型与电机类型保持一致。 */
            }
        }
        else if(physical_channel==BOARD_PROFILE_HANDLE_CHANNEL_B) /* 原物理B电机通道。 */
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x02; /* 原物理B无刷电机使用协议类型0x02；逻辑状态仍归原A/B业务通道。 */
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model, effective_dir_work); /* B物理路使用同一实际方向规则，避免A/B通道运行方式不一致。 */
            }
            else 
            { msg.motor_type=0x04; /* 原物理B有刷电机固定使用协议类型0x04。 */
                 msg.run_type=0x04; /* 有刷物理B路的运行类型与电机类型保持一致。 */
            }
        }
      msg.speed_h=command_speed_value/256;
      msg.speed_l=(command_speed_value)%256;//速度
      msg.pro_current_h=WorkMessage.current_work/256; /* 保护电流来自手柄 EEPROM Page4[21..22]，单位 0.01A；0 表示驱动板使用内部默认保护。 */
      msg.pro_current_l=WorkMessage.current_work%256;//电流
      MotorStart((uint32_t)command_speed_value * MOTOR_DRIVE_CMD_SPEED_UNIT_RPM,
                 WorkMessage.drivetype_work,
                 motor_source_speed); /* 同时记录控制来源、原始速度和驱动16位字段可表达的最终rpm。 */
    }
    else
    {
         /* 只有上一周期确实处于运行显示态时才恢复停止颜色，避免每 50ms 重复刷屏。 */
         if(huci==1){
            huci=0;
            last_display_speed=MOTOR_DRIVE_DISPLAY_SPEED_INVALID; /* 停机后清显示缓存，下次起转必须重新同步运行速度。 */
            display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位为 WorkMessage 的实际 rpm。 */
            display_value[1] = (uint8_t)((WorkMessage.speed_set_work >>8)& 0xFFU); /* 发送 24 位速度的中间字节，最低字节在下方填写。 */
             display_value[2] = (uint8_t)(WorkMessage.speed_set_work & 0xFFU);  
            display_value[3] = 1U; 
            display_value[4] = 0U; 
            SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 最后刷新速度值，保证切通道后的速度显示同步。 */
       
	        LCD_Show_2byte_Number(0x9473,0xffff);

         }
       MotorStops(requested_run_state,
                  WorkMessage.drivetype_work,
                  motor_source_speed,
                  zero_speed_blocked); /* 保留原请求，便于区分用户要求停止和“要求启动但速度为 0”。 */
    }

}

/*
 * 函数功能：周期发送电机启停命令，并检查是否可以把控制权交给其他本地方式。
 * 输入参数：event 调度器传入的任务事件值，当前任务不使用该参数。
 * 返回参数：无。
 */
void MOTORRUNTask(uint32_t event) 
{ 
    /* 当前任务不按 event 分支处理，显式丢弃参数避免后续误解。 */
    (void)event;
    /* 根据 runflag_work、方向、通道、速度和保护电流组帧，向 UART1 电机驱动板下发命令。 */
    MOTORRUN();
    /* 每周期检查 Page4 的速度/频率阈值；本函数调用不改变这里已经发出的电机命令。 */
    Pubinterface_CheckSpeedThresholdAlarm();
    /* 每周期检查控制权：无运行请求，且反馈为零或已超时后，允许其他本地方式接管。 */
    ControlArbitration_RefreshMotorOwner();
}
/*
 * 函数功能：创建电机输出任务，每 50ms 处理一次启停命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscDriveMotorTask_Init(void)
{
   
  /* definition and creation of MOTOR123Task */
	Kernel_TaskCreate(&MOTORRUNTaskHandle, MOTORRUNTask);
	Kernel_TaskStart(&MOTORRUNTaskHandle, KERNEL_TASK_ALWAYS, 50);
}
