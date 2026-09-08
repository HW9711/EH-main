#include "external_comm_simple_protocol.h"

#include "board_profile.h"
#include "control_arbitration.h"
#include "external_comm_task.h"
#include "Pubinterface.h"
#include "pump_control.h"
#include "sscBEEP.h"
#include "sscUIDP.h"

#include <stddef.h>

#define EXT_SIMPLE_HEAD_SIZE              3U    /* 固定帧头 AA BB CC 的字节数。 */
#define EXT_SIMPLE_FUN_LOGIN              0x01U /* 启动简易外部控制。 */
#define EXT_SIMPLE_FUN_LOGOUT             0x02U /* 退出简易外部控制。 */
#define EXT_SIMPLE_FUN_HANDLE_START       0x03U /* 启动当前手柄电机。 */
#define EXT_SIMPLE_FUN_HANDLE_STOP        0x04U /* 停止当前手柄电机。 */
#define EXT_SIMPLE_FUN_SPEED_DOWN         0x05U /* 按当前 Page6 小步进减小手柄转速。 */
#define EXT_SIMPLE_FUN_SPEED_UP           0x06U /* 按当前 Page6 小步进增加手柄转速。 */
#define EXT_SIMPLE_FUN_HANDLE_SWITCH      0x07U /* 在当前 A/B 手柄之间切换。 */
#define EXT_SIMPLE_FUN_PUMP_A_TOGGLE      0x08U /* 切换 A 泵外控独立运行请求。 */
#define EXT_SIMPLE_FUN_PUMP_B_TOGGLE      0x09U /* 切换 B 泵外控独立运行请求。 */
#define EXT_SIMPLE_FUN_PUMP_A_GEAR        0x0AU /* 循环切换 A 泵 0～5 流量档。 */
#define EXT_SIMPLE_FUN_PUMP_B_GEAR        0x0BU /* 循环切换 B 泵 0～5 流量档。 */
#define EXT_SIMPLE_FUN_CLEAR_REMINDER     0x0CU /* 关闭蜂鸣和报警弹窗，不清除用于禁止启动的故障状态。 */

#define EXT_SIMPLE_LEGACY_LOGIN           0x01U /* 复用原外控申请功能码，不把授权逻辑复制到本模块。 */
#define EXT_SIMPLE_LEGACY_SWITCH          0x03U /* 复用原外控通道切换功能码。 */
#define EXT_SIMPLE_LEGACY_CONTROL         0x04U /* 复用原外控泵和手柄控制功能码。 */
#define EXT_SIMPLE_LEGACY_LOGOUT          0xBBU /* 复用原外控安全退出功能码。 */
#define EXT_SIMPLE_AREA_PUMP_A_START      0x01U /* 原外控 A 泵启动动作。 */
#define EXT_SIMPLE_AREA_PUMP_A_STOP       0x02U /* 原外控 A 泵停止动作。 */
#define EXT_SIMPLE_AREA_PUMP_B_START      0x03U /* 原外控 B 泵启动动作。 */
#define EXT_SIMPLE_AREA_PUMP_B_STOP       0x04U /* 原外控 B 泵停止动作。 */
#define EXT_SIMPLE_AREA_HANDLE_START      0x05U /* 原外控当前手柄启动动作。 */
#define EXT_SIMPLE_AREA_HANDLE_STOP       0x06U /* 原外控当前手柄停止动作。 */
#define EXT_SIMPLE_AUTH_SIZE              8U    /* 原外控申请函数要求 8 字节授权数据；仅在主控内部补齐，简易串口帧仍为 6 字节。 */

#if (EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE == 1U)

static const uint8_t s_simple_head[EXT_SIMPLE_HEAD_SIZE] = {0xAAU, 0xBBU, 0xCCU}; /* 共用 FIFO 只搜索前三字节帧头。 */
static uint8_t s_simple_session_active = 0U; /* 0x01 登录并取得外控控制权后置 1，才允许执行 0x03～0x0C。 */

/*
 * 函数功能：判断功能码是否属于用户定义的 0x01～0x0C 范围。
 * 输入参数：fun_code 为简易协议第四字节。
 * 返回参数：已定义返回 1，未知功能码返回 0。
 */
static uint8_t ExtSimple_IsKnown(uint8_t fun_code)
{
    /* 功能码连续定义为 0x01～0x0C，范围判断可避免维护重复列表。 */
    return ((fun_code >= EXT_SIMPLE_FUN_LOGIN) && (fun_code <= EXT_SIMPLE_FUN_CLEAR_REMINDER)) ? 1U : 0U;
}

/*
 * 函数功能：通过原外控安全入口申请控制权，并记录简易协议登录状态。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ExtSimple_Login(void)
{
    uint8_t auth_code[EXT_SIMPLE_AUTH_SIZE] = {0U}; /* 给原申请函数补足所需的 8 字节数据，不通过串口发送。 */

    ExternalComm_NotifyLink(); /* 第三份登录帧确认简易会话后才点亮图标并刷新2秒/10秒链路保护。 */
    ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_LOGIN,
                           0xFFU,
                           auth_code,
                           sizeof(auth_code)); /* 沿用控制权检查、屏幕退出保护和首次登录停机处理，但不发送原协议应答。 */
    s_simple_session_active = ControlArbitration_IsExternalActive() ? 1U : 0U; /* 确实取得外控控制权才记为登录成功，申请被拒绝时保持未登录。 */
}

/*
 * 函数功能：退出由简易协议建立的外控会话，并安全停止外控电机和泵。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ExtSimple_Logout(void)
{
    if (s_simple_session_active == 0U)
    {
        return; /* 简易协议未登录时直接返回，不能退出正式协议建立的外控连接。 */
    }

    ExternalComm_NotifyLink(); /* 退出帧也是合法链路活动，先刷新在线状态再执行安全释放。 */
    ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_LOGOUT,
                           0xFFU,
                           NULL,
                           0U); /* 沿用原退出处理，停止输出、清除泵运行请求并交还控制权。 */
    s_simple_session_active = 0U; /* 退出后本协议必须重新收到 0x01 才能执行控制动作。 */
}

/*
 * 函数功能：在 A/B 手柄之间切换；是否允许切换仍由原外控函数检查。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ExtSimple_SwitchHandle(void)
{
    uint8_t target_area = 0U; /* 原外控 AreaCode 1 表示 A、2 表示 B；0 表示当前不可切换。 */

    if (WorkMessage.channel_work == CHANNEL_A)
    {
        target_area = 0x02U; /* 当前为 A 时，简易“切换手柄”目标固定为 B。 */
    }
    else if (WorkMessage.channel_work == CHANNEL_B)
    {
        target_area = 0x01U; /* 当前为 B 时，简易“切换手柄”目标固定为 A。 */
    }

    if (target_area == 0U)
    {
        return; /* 没有当前 A/B 通道时不猜测目标，等待手柄识别流程先建立选中通道。 */
    }

    ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_SWITCH,
                           target_area,
                           NULL,
                           0U); /* 原切换入口继续检查目标在线、运行状态、报警状态并刷新 UI。 */
}

/*
 * 函数功能：切换指定泵的外控独立运行请求。
 * 输入参数：pump_channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：无。
 */
static void ExtSimple_TogglePump(uint8_t pump_channel)
{
    uint8_t area_code; /* 保存映射到原外控控制命令的启动或停止 AreaCode。 */

    if (pump_channel == CHANNEL_A)
    {
        area_code = (ExternalComm_PumpRunRequested(CHANNEL_A) != 0U) ?
                    EXT_SIMPLE_AREA_PUMP_A_STOP :
                    EXT_SIMPLE_AREA_PUMP_A_START; /* 已有独立运行请求就停止，否则发起独立启动；不按实测转速判断。 */
    }
    else if (pump_channel == CHANNEL_B)
    {
        area_code = (ExternalComm_PumpRunRequested(CHANNEL_B) != 0U) ?
                    EXT_SIMPLE_AREA_PUMP_B_STOP :
                    EXT_SIMPLE_AREA_PUMP_B_START; /* 已有独立运行请求就停止，否则发起独立启动；不按实测转速判断。 */
    }
    else
    {
        return; /* 非 A/B 通道没有泵控制含义，防御异常内部调用。 */
    }

    ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_CONTROL,
                           area_code,
                           NULL,
                           0U); /* 原泵控制函数继续检查泵配置、压力保护、默认速度和外控控制权。 */
}

/*
 * 函数功能：在共用 UART2 FIFO 中查找简易协议帧头，并判断固定 6 字节是否已经收全。
 * 输入参数：available 为接收缓存已有字节数；reader 读取缓存但不移除数据；head_offset 返回帧头位置。
 * 返回参数：返回未找到帧头、帧未收全或 6 字节已收全；已收全不代表帧尾正确。
 */
ExtSimpleProbeResult_t ExtSimple_Probe(uint16_t available,
                                      ExtSimpleReadFn reader,
                                      uint16_t *head_offset)
{
    uint16_t offset; /* 本次检查的缓存位置，每次向后移动 1 字节查找帧头。 */
    uint8_t candidate[EXT_SIMPLE_HEAD_SIZE]; /* 暂存读取的 3 字节，用来比较 AA BB CC 帧头。 */

    if ((reader == NULL) || (head_offset == NULL))
    {
        return EXT_SIMPLE_PROBE_NOT_FOUND; /* 回调或输出指针无效时不能安全访问共用 FIFO。 */
    }

    *head_offset = 0U; /* 失败路径先清输出，避免调用方沿用上一次偏移。 */
    if (available < EXT_SIMPLE_HEAD_SIZE)
    {
        return EXT_SIMPLE_PROBE_NOT_FOUND; /* 连完整三字节帧头都不足时等待下一包。 */
    }

    for (offset = 0U; offset <= (uint16_t)(available - EXT_SIMPLE_HEAD_SIZE); ++offset)
    {
        if (reader(offset, candidate, sizeof(candidate)) == 0U)
        {
            return EXT_SIMPLE_PROBE_NOT_FOUND; /* FIFO 数据量与回调结果不一致时停止探测，避免越界。 */
        }

        if ((candidate[0] == s_simple_head[0]) &&
            (candidate[1] == s_simple_head[1]) &&
            (candidate[2] == s_simple_head[2]))
        {
            *head_offset = offset; /* 返回最早帧头，接收层会与旧协议帧头偏移比较。 */
            if ((uint16_t)(available - offset) < EXTERNAL_COMM_SIMPLE_FRAME_SIZE)
            {
                return EXT_SIMPLE_PROBE_INCOMPLETE; /* 保留半帧，等待下一次 DMA 数据补齐尾部。 */
            }
            return EXT_SIMPLE_PROBE_READY; /* 固定 6 字节已经可供完整校验和执行。 */
        }
    }

    return EXT_SIMPLE_PROBE_NOT_FOUND; /* 当前 FIFO 没有简易协议帧头。 */
}

/*
 * 函数功能：校验并执行一帧 AA BB CC FunCode EE FF 简易外控指令。
 * 输入参数：frame 指向待检查的帧；frame_len 为帧的字节数。
 * 返回参数：帧结构完整时返回 1；帧头、帧尾或长度错误时返回 0。
 */
uint8_t ExtSimple_HandleFrame(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t fun_code; /* 保存第四字节功能码，固定结构校验通过后才读取。 */

    if ((frame == NULL) || (frame_len != EXTERNAL_COMM_SIMPLE_FRAME_SIZE))
    {
        return 0U; /* 长度不是 6 或空指针都不是合法简易协议帧。 */
    }

    if ((frame[0] != 0xAAU) || (frame[1] != 0xBBU) || (frame[2] != 0xCCU) ||
        (frame[4] != 0xEEU) || (frame[5] != 0xFFU))
    {
        return 0U; /* 头尾任一字节错误时不执行动作，也不刷新外控链路。 */
    }

    fun_code = frame[3]; /* 固定头尾合法后读取功能码，避免坏帧影响会话状态。 */
    if (ExtSimple_IsKnown(fun_code) == 0U)
    {
        return 1U; /* 帧格式完整，可从接收缓存移除；未知功能码不执行，也不重置断线计时。 */
    }

    if (fun_code == EXT_SIMPLE_FUN_LOGIN)
    {
        if (ExternalComm_TryConfirmProtocol(EXTERNAL_COMM_PROTOCOL_SOURCE_SIMPLE,
                                            NULL,
                                            0U) != 0U)
        {
            ExtSimple_Login(); /* 收齐三份登录帧，或简易协议已连接时，才重置断线计时并申请控制权。 */
        }
        return 1U;
    }

    if (fun_code == EXT_SIMPLE_FUN_LOGOUT)
    {
        ExtSimple_Logout(); /* 0x02 只退出简易协议建立的连接，不能退出正式协议的连接。 */
        return 1U;
    }

    if ((s_simple_session_active == 0U) || (ControlArbitration_IsExternalActive() == false))
    {
        s_simple_session_active = 0U; /* 断线超时或屏幕退出导致外控失效时，同时清除简易协议登录状态。 */
        return 1U; /* 未登录时忽略命令，不执行、不应答，必须先通过 0x01 登录。 */
    }

    ExternalComm_NotifyLink(); /* 已登录且已知的每条控制帧都刷新原有 2s/10s 链路保护。 */

    switch (fun_code)
    {
        case EXT_SIMPLE_FUN_HANDLE_START:
            ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_CONTROL,
                                   EXT_SIMPLE_AREA_HANDLE_START,
                                   NULL,
                                   0U); /* 沿用原手柄启动检查，报警、掉线或未选通道时均不启动。 */
            break;
        case EXT_SIMPLE_FUN_HANDLE_STOP:
            ExternalComm_RunSilent(EXT_SIMPLE_LEGACY_CONTROL,
                                   EXT_SIMPLE_AREA_HANDLE_STOP,
                                   NULL,
                                   0U); /* 复用原手柄停止及联动泵清理路径。 */
            break;
        case EXT_SIMPLE_FUN_SPEED_DOWN:
            SpeedActive(HMIkey_SPEED_Sub); /* 使用当前 Page6 小步进和原方向上下限，不在协议层写死转速。 */
            break;
        case EXT_SIMPLE_FUN_SPEED_UP:
            SpeedActive(HMIkey_SPEED_Add); /* 使用当前 Page6 小步进和原方向上下限，不在协议层写死转速。 */
            break;
        case EXT_SIMPLE_FUN_HANDLE_SWITCH:
            ExtSimple_SwitchHandle(); /* 只在原通道切换安全条件满足时真正切换。 */
            break;
        case EXT_SIMPLE_FUN_PUMP_A_TOGGLE:
            ExtSimple_TogglePump(CHANNEL_A); /* A 泵未配置或被压力保护阻止时由原控制入口保持停机。 */
            break;
        case EXT_SIMPLE_FUN_PUMP_B_TOGGLE:
            ExtSimple_TogglePump(CHANNEL_B); /* B 泵未配置或被压力保护阻止时由原控制入口保持停机。 */
            break;
        case EXT_SIMPLE_FUN_PUMP_A_GEAR:
            PUMPActive(JTkey_left_short); /* 复用 A 泵类型对应的 0～5 档循环和显示刷新。 */
            break;
        case EXT_SIMPLE_FUN_PUMP_B_GEAR:
            PUMPActive(JTKey_right_short); /* 复用 B 泵类型对应的 0～5 档循环和显示刷新。 */
            break;
        case EXT_SIMPLE_FUN_CLEAR_REMINDER:
            SendAlarmMessage(WORK_ALARM_NONE); /* 只关闭蜂鸣提醒，WorkMessage 中阻止再次启动的故障状态仍保留。 */
            SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 只关闭报警弹窗；故障状态仍保留，电机和泵仍不能启动。 */
            break;
        default:
            break; /* 0x01、0x02 已在前面处理，其余未知值已被范围检查拦截。 */
    }

    return 1U; /* 本帧处理结束，接收函数可从缓存移除这 6 字节。 */
}

#else

/*
 * 函数功能：编译开关关闭时保持相同接口，但不在 UART2 FIFO 中识别简易协议。
 * 输入参数：available、reader、head_offset 均不使用。
 * 返回参数：固定返回未找到。
 */
ExtSimpleProbeResult_t ExtSimple_Probe(uint16_t available,
                                      ExtSimpleReadFn reader,
                                      uint16_t *head_offset)
{
    (void)available; /* 关闭功能后不读取 FIFO 长度。 */
    (void)reader; /* 关闭功能后不调用任何 FIFO 回调。 */
    if (head_offset != NULL)
    {
        *head_offset = 0U; /* 输出仍初始化为 0，保证调用方没有未定义数据。 */
    }
    return EXT_SIMPLE_PROBE_NOT_FOUND; /* 宏关闭时所有简易帧都按普通噪声处理。 */
}

/*
 * 函数功能：简易协议编译开关关闭时，不执行收到的简易协议帧。
 * 输入参数：frame 和 frame_len 均不使用。
 * 返回参数：固定返回 0，接收函数按无效帧处理，逐字节重新查找帧头。
 */
uint8_t ExtSimple_HandleFrame(const uint8_t *frame, uint16_t frame_len)
{
    (void)frame; /* 功能已关闭，不读取帧内容。 */
    (void)frame_len; /* 功能已关闭，不检查帧长度。 */
    return 0U; /* 不允许任何简易协议动作生效。 */
}

#endif
