/**
 ******************************************************************************
 * @file    sscFOOT.c
 * @brief   脚踏板数据解析驱动
 *          用于解析脚踏板传感器传来的串口数据
 ******************************************************************************
 * @note
 * 下表为旧 AA/BB 协议说明，仅供查旧接口。当前 UART4 实际按 FE EF 帧头解析，
 * 单踏板/按键帧为 10 字节、双段踏板帧为 18 字节、双脚踏帧为 24 字节。
 * 旧通讯协议格式：
 * +------+------+-------+--------+--------+--------+--------+------+----------+
 * | 帧头  | 类型  | 数据1  | 数据2  | 数据3  | 数据4  | 数据5  | 校验和 | 帧尾   |
 * | 0xAA  | 0xXX  | 0xXX  | 0xXX  | 0xXX  | 0xXX  | 0xXX  | 0xXX  | 0xBB   |
 * +------+------+-------+--------+--------+--------+--------+------+----------+
 * 数据字段说明：
 * - 数据1: 脚踏板AD值高8位
 * - 数据2: 脚踏板AD值低8位
 * - 数据3: 按键状态
 * - 数据4: 连接状态
 * - 数据5: 脚踏板类型 (0x01单踏板/0x02双踏板)
 *
 * 版本: V1.0
 * 日期: 2026-04-15
 ******************************************************************************
 */

#include "sscFOOT.h"
#include "data.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "board.h"
#include "uart4.h"
#include "common.h"
#include "Pubinterface.h"
#include "sscKEYBH.h"
#include "sscUIDP.h"
#include "sscBEEP.h"
#include "sscPUMPA.h"
#include "sscPUMPB.h"
#include "pump.h"
#include "motoruartdata.h"



#define JT_threshold  20U /* 踩下判断需超过定标起点 20 个 AD 计数；调大需踩得更深，也会提高定标行程的最小要求。 */
/* 单踏板连续前 3 次低值先不停止泵，第 4 次才确认松脚；周期 25ms，即首个低值后再等约 75ms。调大可减少抖动停泵，但松脚停泵更慢；电机不等待此计数。 */
#define FOOT_SINGLE_PEDAL_RELEASE_DEBOUNCE_TICKS 3U
#define FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS 10U /* 双脚踏深踩切通道需累计 10 次控制检查，每次 25ms，约 250ms；调大切换更慢，调小更容易误切。 */
#define FOOT_PEDAL_SPEED_HIGH_MARGIN 30U /* 双脚踏控制对应通道时，从定标高点减去 30 个 AD 计数作为满速位置；调大将更早达到设定速度，不改变最高速度。 */
#define FOOT_SINGLE_FRAME_LENGTH 10U /* 单踏板实时值、Flash读回和脚踏实体按键帧均为10字节，最后2字节为CRC。 */
#define FOOT_TWO_STAGE_FRAME_LENGTH 18U /* 双段踏板周期帧为18字节，包含实时AD、三点定标值和CRC。 */
#define FOOT_DOUBLE_FRAME_LENGTH 24U /* 双脚踏周期帧为24字节，包含左右实时AD、两组三点定标值和CRC。 */
#define FOOT_RUNTIME_TIMEOUT_TICKS 25U /* 每次计数 10ms，连续 25 次无有效实时帧就停脚踏电机和泵；调大延后失联停机，须小于掉线计数。 */
#define FOOT_OFFLINE_TIMEOUT_TICKS 100U /* 每次计数 10ms，超过 100 次才通知界面掉线，首次为 101 次约 1.01s；调大延后掉线提示，不改变前面的 250ms 停机。 */
#define FOOT_MOTOR_SOURCE_NONE 0U /* 内部编号 0：没有踏板启动电机；不是可调的运行参数。 */
#define FOOT_MOTOR_SOURCE_LEFT 1U /* 内部编号 1：电机由单踏板、双段踏板或双脚踏左侧启动。 */
#define FOOT_MOTOR_SOURCE_RIGHT 2U /* 内部编号 2：电机由双脚踏右侧启动；用于区分哪侧松脚应停机。 */

static uint8_t get_jtHvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB8,0Xdf,0x3e,0x84};//读高值

static uint8_t get_jtLvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB5,0XCD,0xA3,0x00};//读低值

static uint16_t jt_adcvalue = 0;
static uint16_t jtb_adcvalue = 0;
static uint16_t jtd_adcvalue_l = 0;
static uint16_t jtd_adcvalue_r = 0;
static uint8_t s_double_pedal_release_before_run_channel = CHANNEL_NONE;
static uint8_t s_double_left_gently_pump_channel = CHANNEL_NONE; /* 记录左脚踏轻踩实际启动的泵，松脚时不能再按泵类型重新猜测。 */
static uint8_t s_double_right_gently_pump_channel = CHANNEL_NONE; /* 记录右脚踏轻踩实际启动的泵，A/B 都是注水泵时确保停止 B。 */
static bool s_common_socket_missing_wait_release = false; /* 公共接头缺刀具触发后等待脚踏真实释放，防止同一次长踩反复弹 80。 */
static volatile bool s_foot_runtime_frame_valid = false; /* 只有收到完整且 CRC 正确的实时帧才为 true；为 false 时不能沿用旧踩踏值启动。 */
static volatile bool s_foot_motor_stop_latched = true; /* 上电默认禁止脚踏起机，必须先观察到真实松脚再允许新的踩下周期。 */
static bool s_foot_motor_release_ready = false; /* 停机后已检测到松脚时为 true；必须先满足此条件，下次踩下才能启动。 */
static uint8_t s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 记录真正启动电机的踏板侧，未踩的另一侧释放不能误停当前运行。 */
static uint8_t s_foot_handle_pump_source = FOOT_MOTOR_SOURCE_NONE; /* 记住哪侧脚踏启动了手柄联动泵；即使异常处理先清了 jtL/jtR，松脚时仍能找到要停的泵。 */

/*
 * 函数功能：让注水泵跟随手柄运行，并记住是左脚还是右脚启动的。
 * 输入参数：source 为 FOOT_MOTOR_SOURCE_LEFT/RIGHT，表示本次启动电机的踏板侧。
 * 返回参数：无；泵驱动故障后的停机要求尚未清除，或没有注水泵时，不记录启动侧。
 */
static void Foot_StartHandleInjectionPumpFollow(uint8_t source)
{
    uint8_t active_follow_mask; /* bit0/bit1 分别记录 A/B 泵已被手柄联动请求启动；这是软件记录，不是泵转动反馈。 */

    if ((source != FOOT_MOTOR_SOURCE_LEFT) && (source != FOOT_MOTOR_SOURCE_RIGHT))
    {
        return; /* 无法确定是哪侧脚踏时不启动泵，以免松脚时停错泵。 */
    }

    Pubinterface_SetHandleInjectionPumpRun(true); /* 由公共接口选择 A/B 注水泵；泵驱动故障后的停机要求尚未清除时，会拒绝运行。 */
    active_follow_mask = Pubinterface_GetHandleInjectionPumpFollowMask(); /* 查看刚才是否至少有一台泵获准跟随手柄。 */
    s_foot_handle_pump_source = ((WorkMessage.runflag_work != false) && (active_follow_mask != 0U)) ? source : FOOT_MOTOR_SOURCE_NONE; /* 只有手柄有运行请求且泵获准跟随时，才记住本次启动侧。 */
}

/*
 * 函数功能：停止指定踏板启动的手柄联动泵，不处理另一侧脚踏的泵请求。
 * 输入参数：source 为 LEFT/RIGHT；NONE 用于启动检查失败或通信超时，表示不区分左右侧。
 * 返回参数：无。
 */
static void Foot_StopHandleInjectionPumpFollow(uint8_t source)
{
    uint8_t active_source = s_foot_handle_pump_source; /* 先取出启动泵的踏板侧，下面据此判断本次是否该停泵。 */

    if (active_source == FOOT_MOTOR_SOURCE_NONE)
    {
        return; /* 没有记录脚踏启动过联动泵，不改动屏幕或外控的泵请求。 */
    }
    if ((source != FOOT_MOTOR_SOURCE_NONE) && (source != active_source))
    {
        return; /* 松开的不是启动泵的那侧，另一侧的泵继续运行。 */
    }

    s_foot_handle_pump_source = FOOT_MOTOR_SOURCE_NONE; /* 清掉启动侧记录，后续松脚或异常处理不再重复停止这次联动。 */
    Pubinterface_SetHandleInjectionPumpRun(false); /* 停止公共接口记录过的联动泵；没有联动标记的泵不在这里改动。 */
}

/*
 * 函数功能：禁止脚踏再次启动电机，直到检测到松脚后重新踩下。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_LatchMotorStopUntilRelease(void)
{
    s_foot_motor_stop_latched = true; /* 先要求保持停机，50ms 手柄命令任务下一次执行时改发零速帧。 */
    s_foot_motor_release_ready = false; /* 通信或启动检查失败后，仍踩着脚踏不能算作一次新的启动。 */
    s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 清掉此前启动电机的踏板侧，下次允许启动时重新记录。 */
}

/*
 * 函数功能：记录踏板已经退出电机运行区，电机保持停止，允许下次重新踩下启动。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_MarkMotorReleased(void)
{
    s_foot_motor_stop_latched = true; /* 踏板一退出电机运行区就要求停机，不等待后面的松脚确认计数。 */
    s_foot_motor_release_ready = true; /* 已检测到退出电机运行区，下一次有效深踩可以重新申请启动。 */
    s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 本次踩踏结束，清掉启动侧记录。 */
}

/*
 * 函数功能：脚踏回到非电机运行区时立即停止脚踏来源的手柄电机，同时保留原泵释放去抖流程。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_StopMotorAtReleaseBoundary(void)
{
    Foot_MarkMotorReleased(); /* 任何踏板类型一旦退出电机运行区，就先锁住停止并记录本次真实释放。 */
    if((WorkMessage.drivetype_work==JTWORK) || ControlArbitration_IsOwner(CONTROL_OWNER_FOOT))
    {
        WorkMessage.runflag_work=false; /* 只撤销脚踏来源的运行请求，不能因脚踏处于低位误停手控、触控或外控。 */
        WorkMessage.speed_work=0U;      /* 同步清除脚踏实际目标速度，确保50ms驱动任务下一周期发送零速帧。 */
    }
}

/*
 * 函数功能：双脚踏一侧退出电机运行区时，只有电机由这一侧启动才要求停机。
 * 输入参数：source 为 FOOT_MOTOR_SOURCE_LEFT 或 FOOT_MOTOR_SOURCE_RIGHT。
 * 返回参数：无。
 */
static void Foot_StopDoubleMotorAtReleaseBoundary(uint8_t source)
{
    if(s_foot_motor_active_source != source)
    {
        return; /* 电机由另一侧启动或本来已停，本侧松脚不改手柄运行状态。 */
    }

    Foot_LatchMotorStopUntilRelease(); /* 启动电机的那侧退出运行区后先停机，两侧都退出运行区才允许再次启动。 */
    if((WorkMessage.drivetype_work==JTWORK) || ControlArbitration_IsOwner(CONTROL_OWNER_FOOT))
    {
        WorkMessage.runflag_work=false; /* 只撤销实际活动踏板产生的电机请求，保证50ms任务发送STOP。 */
        WorkMessage.speed_work=0U;      /* 清实际目标速度，另一侧若要运行必须先完成双侧释放再重新踩下。 */
    }
}

/*
 * 函数功能：在脚踏准备写入运行标志前验证“先松开、后重新踩下”的完整动作周期。
 * 输入参数：source 为左侧或右侧踏板编号，取 FOOT_MOTOR_SOURCE_LEFT/RIGHT。
 * 返回参数：true 表示本侧允许启动或继续运行；false 表示本侧必须保持停机。
 */
static bool Foot_TryAuthorizeMotorRun(uint8_t source)
{
    if((source != FOOT_MOTOR_SOURCE_LEFT) && (source != FOOT_MOTOR_SOURCE_RIGHT))
    {
        return false; /* 不是左侧或右侧编号时，不能启动手柄。 */
    }
    if(s_foot_motor_stop_latched == false)
    {
        return (s_foot_motor_active_source == source); /* 持续踩踏时只允许原启动侧调速；切到另一侧之前必须先松脚。 */
    }
    if(s_foot_motor_release_ready == false)
    {
        return false; /* 上电、通信中断或启动检查失败后还没松脚，不能直接按旧踩踏值重启。 */
    }

    s_foot_motor_release_ready = false; /* 本次松脚记录只用一次，下次停机后仍需再次松脚。 */
    s_foot_motor_stop_latched = false; /* 取消强制停机，手柄驱动任务可以执行本次运行请求。 */
    s_foot_motor_active_source = source; /* 保存实际启动侧，双脚踏未活动一侧的释放分支不得误停本侧。 */
    return true;
}

/*
 * 函数功能：查询脚踏是否仍要求电机保持停止。
 * 输入参数：无。
 * 返回参数：1 表示必须保持停止；0 表示当前踩踏已允许运行。
 */
uint8_t Foot_IsMotorStopLatched(void)
{
    return (s_foot_motor_stop_latched != false) ? 1U : 0U; /* 电机发送任务只需要知道是否必须停机，松脚判断仍由本模块处理。 */
}



//任务句柄
kernel_task_t FOOTTaskHandle;
kernel_task_t FOOTBHHandle;

//消息队列
static QueueHandle_t FootMsgQueue = NULL;

//脚踏板消息类型
typedef struct {
    bool connect_flag;//链接状态
    uint8_t  pedalType;           //脚踏类型
    uint16_t LValue_Left;   //脚踏板AD值-左踏板低值
    uint16_t MValue_Left;   //脚踏板AD值-左踏板中值
    uint16_t HValue_Left;   //脚踏板AD值-左踏板高值

    uint16_t LValue_Right;   // 右踏板低点定标 AD 值，用来判断是否松脚。
    uint16_t MValue_Right;   // 右踏板中点定标 AD 值，用来区分轻踩泵区和电机区。
    uint16_t HValue_Right;   // 右踏板高点定标 AD 值，用来计算踩到底时的速度。


} FootMessage_t;

FootMessage_t footmessage;

/*
 * 函数功能：按当前通道和当前方向读取手柄 EEPROM/RFID 已装载的最小运行速度。
 * 输入参数：无。
 * 返回参数：当前方向最小速度，单位为实际 rpm；无有效通道或方向参数缺失时返回 0。
 */
static uint32_t Foot_GetCurrentDirectionMinSpeed(void)
{
    ChannelrecognizeMessage_t *recognize = NULL; /* 指向当前工作通道识别缓存，里面保存 Page4/RFID 解析出的速度下限。 */

    if(WorkMessage.channel_work==CHANNEL_A)
    {
        recognize=&ChannelrecognizeMessageA; /* A 通道运行时，脚踏下限必须读取 A 通道当前刀具/手柄参数。 */
    }
    else if(WorkMessage.channel_work==CHANNEL_B)
    {
        recognize=&ChannelrecognizeMessageB; /* B 通道运行时，脚踏下限必须读取 B 通道当前刀具/手柄参数。 */
    }
    else
    {
        return 0U; /* 未选中 A/B 通道时不施加下限，避免无手柄状态误写运行速度。 */
    }

    if(WorkMessage.dir_work==FZDIR)
    {
        return recognize->speed_fzmin; /* 反转脚踏比例速度从反转最小速度开始，避免低速触发驱动报警。 */
    }
    if(WorkMessage.dir_work==OSCDIR)
    {
        return recognize->speed_oscmin; /* 往复脚踏比例速度从往复最小速度开始，保持和屏幕调速下限一致。 */
    }

    return recognize->speed_zzmin; /* 正转或异常方向默认按正转下限处理，和当前工程默认方向逻辑一致。 */
}

/*
 * 函数功能：把脚踏霍尔 ADC 行程换算为本周期实际电机目标速度。
 * 输入参数：ad_value 为当前 AD 值；low_adc/high_adc 为运行段低点/高点；high_margin 为从高点扣除的 AD 计数。
 * 返回参数：电机目标速度，单位 rpm，限制在当前方向最小速度与设定速度之间；设定值更小时以设定值为准。
 */
static uint32_t Foot_BuildTravelMotorSpeed(uint16_t ad_value,uint16_t low_adc,uint16_t high_adc,uint16_t high_margin)
{
    uint16_t effective_high_adc=high_adc; /* 达到设定速度的 AD 位置，双脚踏部分分支会比定标高点提前 30 个计数。 */
    uint16_t clamped_adc=ad_value; /* 将 AD 值限制在运行段内，防止踩到底时速度超过设定值。 */
    uint32_t min_speed=Foot_GetCurrentDirectionMinSpeed(); /* 当前方向 EEPROM/RFID 最小速度，脚踏比例不能再从 0 开始。 */
    uint32_t max_speed=WorkMessage.speed_set_work; /* 当前屏幕/EEPROM 设定速度在脚踏模式下作为最大速度。 */
    uint32_t adc_range=0U; /* 运行段 ADC 总行程，用于线性比例计算。 */
    uint32_t adc_offset=0U; /* 当前 ADC 相对运行段起点的有效行程。 */
    uint32_t speed_range=0U; /* 允许脚踏调节的速度区间，等于最大速度减最小速度。 */
    uint32_t speed=0U; /* 本周期换算出的实际运行速度，最终写入 WorkMessage.speed_work。 */

    if(max_speed==0U)
    {
        max_speed=Pubinterface_GetCurrentDefaultMotorSpeed(); /* 通道刚上线但速度未装载时，用当前方向默认速度作为脚踏最大速度。 */
        WorkMessage.speed_set_work=max_speed; /* 同步回设定速度，保证屏幕显示、脚踏换算和驱动输出使用同一最大值。 */
    }

    if((high_margin!=0U)&&((uint32_t)effective_high_adc>((uint32_t)low_adc+(uint32_t)high_margin)))
    {
        effective_high_adc=(uint16_t)(effective_high_adc-high_margin); /* 提前满速位置，但必须保证扣除后仍高于运行起点。 */
    }

    if(min_speed>max_speed)
    {
        min_speed=max_speed; /* EEPROM 上下限或当前设定异常时，最小速度不能超过本次允许的最大速度。 */
    }

    if(effective_high_adc<=low_adc)
    {
        return min_speed; /* 脚踏校准区间无效时不做除法，安全地保持最小运行速度。 */
    }

    if(clamped_adc<low_adc)
    {
        clamped_adc=low_adc; /* 低于运行段起点时按 0% 行程处理，输出 EEPROM 最小速度。 */
    }
    if(clamped_adc>effective_high_adc)
    {
        clamped_adc=effective_high_adc; /* 高于运行段终点时按 100% 行程处理，输出不超过设定速度。 */
    }

    adc_range=(uint32_t)(effective_high_adc-low_adc); /* 计算有效行程总宽度，前面已保证不为 0。 */
    adc_offset=(uint32_t)(clamped_adc-low_adc); /* 计算当前踩踏位置在运行段内的偏移。 */
    speed_range=max_speed-min_speed; /* 前面已保证最大速度不小于最小速度，这里可以直接相减。 */
    speed=min_speed+(uint32_t)(((uint64_t)speed_range*(uint64_t)adc_offset)/(uint64_t)adc_range); /* 用 64 位乘法避免高转速和大 ADC 行程相乘溢出。 */

    if(speed>max_speed)
    {
        speed=max_speed; /* 最终保护：任何舍入或异常输入都不能让实际速度超过屏幕设定最大速度。 */
    }
    if(speed<min_speed)
    {
        speed=min_speed; /* 最终保护：脚踏进入运行段后实际速度不能低于 EEPROM 最小速度。 */
    }

    return speed; /* 返回可直接写入 WorkMessage.speed_work 的脚踏实时目标速度。 */
}











/*
 * 函数功能：把脚踏连接状态发给控制任务；相同消息已发送成功时不重复发送，失败时允许重试。
 * 输入参数：msg 包含本次连接状态、脚踏类型和定标参数。
 * 返回参数：true 表示消息已经成功排队或与最近一次成功消息相同；false 表示队列不可用或本次发送失败。
 */
static bool Foot_SendMessage(FootMessage_t msg)
{
    static FootMessage_t last_queued_msg; /* 只保存最近一次发送成功的消息，发送失败的消息下次仍要重试。 */
    static bool last_queued_valid = false; /* 上电后尚无成功消息时不比较全零静态缓存，避免误判首次消息。 */

    if(FootMsgQueue == NULL)
    {
        return false; /* 队列还没创建，通知调用方本次未发送，下次需要重试。 */
    }
    if((last_queued_valid != false) &&
       (memcmp(&last_queued_msg, &msg, sizeof(FootMessage_t)) == 0))
    {
        return true; /* 完全相同的消息已经成功排队，不重复占用深度为5的连接队列。 */
    }
    if(Kernel_QueueSend(FootMsgQueue, &msg, 0) != pdPASS)
    {
        return false; /* 队列临时已满时不更新缓存，下一次解析周期仍允许重试同一消息。 */
    }

    memcpy(&last_queued_msg, &msg, sizeof(FootMessage_t)); /* 发送成功后才保存本次状态，供下次判断是否重复。 */
    last_queued_valid = true; /* 之后可以和这条已发送的消息比较，跳过重复消息。 */
    return true;
}

/*
 * 函数功能：创建保存脚踏连接状态和定标值的队列。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_Queue_Init(void)
{
    FootMsgQueue = Kernel_QueueCreate(5, sizeof(FootMessage_t), "FootMsgQueue"); /* 最多缓存 5 条消息，供串口解析任务与脚踏控制任务交接数据。 */
}

/*
 * 函数功能：按指定脚踏侧启动 A 注水泵；双脚踏轻踩继续使用屏幕保存的速度。
 * 输入参数：speed_work 为普通联动速度；right_pedal 为 false 表示左侧、true 表示右侧；pedal_drainage 为 true 表示双脚踏轻踩来源。
 * 返回参数：无。
 */
static void Foot_StartPumpAInjection(uint16_t speed_work, bool right_pedal, bool pedal_drainage)
{
    /* 脚踏启动 A 注水泵时，定时排空必须让位，避免排空计时同时改写泵输出。 */
    pumpMessageA.timingDrainage_flag=false;
    /* 排空时间清零，保证下次排空从完整周期重新开始计时。 */
    pumpMessageA.timingDrainage_times=0U;
    /* 记住是哪侧脚踏轻踩开的泵，松脚时才能执行对应的停泵步骤。 */
    if(right_pedal)
    {
        ControlSignalMessage.jtR_gentlypump_flag=true; /* 右脚启动 A 泵时标记右侧，松右脚才能结束本次请求。 */
    }
    else
    {
        ControlSignalMessage.jtL_gentlypump_flag=true; /* 标记 A 泵由左脚轻踩启动，松左脚时需要停止。 */
    }
    /* 双脚踏轻踩只记录控制来源，泵任务仍读取 speed_work，不进入屏幕固定速度排空。 */
    pumpMessageA.pedalDrainage_flag=pedal_drainage;
    if(pedal_drainage==false)
    {
        pumpMessageA.speed_work=speed_work; /* 普通两段脚踏联动仍按原规则更新设定速度。 */
    }
    /* run_flag 为 true 时 A 泵任务才允许发送非零速度，这里发出运行请求。 */
    pumpMessageA.run_flag=true;
    if(pedal_drainage==false)
    {
        SendPumpAMessage(INJECTWATER,pumpMessageA.speed_work); /* 普通联动继续通过旧队列同步速度。 */
    }
}

/*
 * 函数功能：按指定脚踏侧启动 B 注水泵；双脚踏轻踩继续使用屏幕保存的速度。
 * 输入参数：speed_work 为普通联动速度；right_pedal 为 false 表示左侧、true 表示右侧；pedal_drainage 为 true 表示双脚踏轻踩来源。
 * 返回参数：无。
 */
static void Foot_StartPumpBInjection(uint16_t speed_work, bool right_pedal, bool pedal_drainage)
{
    /* 脚踏启动 B 注水泵时，先关闭定时排空，避免两种泵动作同时抢占 B 泵输出。 */
    pumpMessageB.timingDrainage_flag=false;
    /* 清掉历史排空计数，下一次排空动作不能沿用脚踏前的剩余时间。 */
    pumpMessageB.timingDrainage_times=0U;
    /* 记住是哪侧脚踏轻踩开的泵，松脚时按对应标志停泵。 */
    if(right_pedal)
    {
        ControlSignalMessage.jtR_gentlypump_flag=true; /* 右脚启动 B 泵时必须置右侧标记，修正松右脚无法停泵。 */
    }
    else
    {
        ControlSignalMessage.jtL_gentlypump_flag=true; /* 标记 B 泵由左脚轻踩启动，松左脚时需要停止。 */
    }
    /* 双脚踏轻踩只记录控制来源，实际输出继续使用屏幕保存的 speed_work。 */
    pumpMessageB.pedalDrainage_flag=pedal_drainage;
    if(pedal_drainage==false)
    {
        pumpMessageB.speed_work=speed_work; /* 普通两段脚踏联动继续保存本次要求的速度。 */
    }
    /* run_flag 为 true 时 B 泵任务才允许发送非零速度，这里发出运行请求。 */
    pumpMessageB.run_flag=true;
    if(pedal_drainage==false)
    {
        SendPumpBMessage(INJECTWATER,pumpMessageB.speed_work); /* 普通联动继续通过旧队列同步速度。 */
    }
}

/*
 * 函数功能：取消 A 泵的脚踏运行和排空状态，使泵任务下次发送零速。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_StopPumpAInjection(void)
{
    /* 撤销 A 泵运行请求，sscPUMPA 下一周期按 run_flag=false 下发 0 速。 */
    pumpMessageA.run_flag=false;
    pumpMessageA.pedalDrainage_flag=false; /* 松脚结束 A 泵脚踏轻踩来源，后续屏幕排空不受旧状态影响。 */
    /* 脚踏停泵不进入排空模式，必须同步清除排空标志。 */
    pumpMessageA.timingDrainage_flag=false;
    /* 排空计数清零，避免下一次排空或脚踏启动继承旧计数。 */
    pumpMessageA.timingDrainage_times=0U;
}

/*
 * 函数功能：取消 B 泵的脚踏运行和排空状态，使泵任务下次发送零速。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_StopPumpBInjection(void)
{
    /* 撤销 B 泵运行请求，sscPUMPB 下一周期按 run_flag=false 下发 0 速。 */
    pumpMessageB.run_flag=false;
    pumpMessageB.pedalDrainage_flag=false; /* 松脚结束 B 泵脚踏轻踩来源，不能把该状态带到下一次联动。 */
    /* 脚踏停泵不进入排空模式，必须同步清除排空标志。 */
    pumpMessageB.timingDrainage_flag=false;
    /* 排空计数清零，避免下一次排空或脚踏启动继承旧计数。 */
    pumpMessageB.timingDrainage_times=0U;
}

/*
 * 函数功能：脚踏不满足启动条件时停止电机，只撤销脚踏实际控制的注水泵，保留独立灌注请求。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearRunRequestAfterGateFail(void)
{
    bool stop_pump_a=(s_double_left_gently_pump_channel==CHANNEL_A||s_double_right_gently_pump_channel==CHANNEL_A); /* 清除记录前，先记住左脚或右脚是否选中过A泵；后面据此停泵，不依赖轻踩标志。 */
    bool stop_pump_b=(s_double_left_gently_pump_channel==CHANNEL_B||s_double_right_gently_pump_channel==CHANNEL_B); /* 保存左右任一侧使用的 B 泵，两侧共用同一泵时只停止一次。 */

    if(s_double_left_gently_pump_channel==CHANNEL_NONE&&s_double_right_gently_pump_channel==CHANNEL_NONE&&
       (ControlSignalMessage.jtL_gentlypump_flag||ControlSignalMessage.jtR_gentlypump_flag)) /* 双段踏板没有双脚踏通道记录，必须有轻踩开泵标志才按其原选择顺序清理。 */
    {
        stop_pump_a=(pumpMessageA.type==INJECTWATER); /* 双段踏板轻踩原本优先开启 A 注水泵，不把独立灌注泵纳入停止范围。 */
        stop_pump_b=(!stop_pump_a&&pumpMessageB.type==INJECTWATER); /* 只有 A 不是注水泵时才选择 B，避免误停未被轻踩选中的另一注水泵。 */
    }

    Foot_LatchMotorStopUntilRelease();              /* 本次启动被拒绝后必须先松脚，不能踩住等待自动启动。 */
    Foot_StopHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_NONE); /* 撤销左右脚踏此前发出的手柄联动开泵请求，不依赖 jt 标志是否还在。 */
    WorkMessage.runflag_work=false;                 /* 撤销电机运行请求，防止缺刀具等检查失败后仍发出启动帧。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免屏幕或驱动继续沿用本周期脚踏比例速度。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 本次启动检查未通过，清除左脚运行请求，避免松脚时按已经启动处理。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行标志，双脚踏任一侧失败都不能留下运行来源。 */
    s_double_left_gently_pump_channel=CHANNEL_NONE; /* 本次启动被拒绝，清掉左脚轻踩使用的泵记录。 */
    s_double_right_gently_pump_channel=CHANNEL_NONE; /* 清右侧实际泵记录，避免松脚时误停后续其它来源启动的泵。 */
    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT); /* 电机已停止时取消脚踏对电机的占用，让其他控制方式可以使用。 */
    ControlSignalMessage.jtL_gentlypump_flag=false; /* 启动被拒绝后取消左脚轻踩状态，实际停止对象使用清理前保存的选择。 */
    ControlSignalMessage.jtR_gentlypump_flag=false; /* 同步取消右脚轻踩状态，保持原先两侧都必须重新建立请求的保护。 */
    if(stop_pump_a&&pumpMessageA.type==INJECTWATER) /* 只停止脚踏选中过且当前仍为注水泵的 A，类型已变化时不撤销新的灌注请求。 */
    {
        Foot_StopPumpAInjection(); /* 撤销 A 的脚踏开泵及排空状态，泵任务下周期发送零速。 */
    }
    if(stop_pump_b&&pumpMessageB.type==INJECTWATER) /* 只有脚踏选中过B泵且B仍是注水泵时才停止，不能误停独立运行的灌注泵。 */
    {
        Foot_StopPumpBInjection(); /* 撤销实际由脚踏启动的 B，不影响独立运行的另一台泵。 */
    }
}

/*
 * 函数功能：脚踏实时数据超时后，停止由脚踏启动的电机和泵，禁止继续使用旧踩踏值。
 * 输入参数：无，直接检查当前控制方式及脚踏运行记录。
 * 返回参数：无。
 */
static void Foot_StopRunOnRealtimeTimeout(void)
{
    bool foot_output_active; /* 记录当前电机或轻踩泵是否确实由脚踏来源占用，避免误停其它控制方式。 */

    s_foot_runtime_frame_valid = false; /* 标记实时数据不可用，必须收到新的完整且 CRC 正确的实时帧后才能继续判断踩踏。 */
    Foot_LatchMotorStopUntilRelease(); /* 实时AD失效后即使恢复为旧高值也不能自动起机，必须先收到真实松脚。 */
    MotorUart_ReleaseFootDriverAlarm(); /* 通信超时按脚踏已松开处理；驱动错误 Err 仍需恢复为 0 才能清除驱动报警。 */
    foot_output_active = (s_foot_handle_pump_source != FOOT_MOTOR_SOURCE_NONE) ||
                         ControlArbitration_IsOwner(CONTROL_OWNER_FOOT) ||
                         ((WorkMessage.drivetype_work == JTWORK) &&
                          (ControlSignalMessage.jtL_control_flag ||
                           ControlSignalMessage.jtR_control_flag ||
                           ControlSignalMessage.jtL_gentlypump_flag ||
                           ControlSignalMessage.jtR_gentlypump_flag)); /* 任一记录表明脚踏仍在控制电机或泵，就执行下面的停机处理。 */
    if(foot_output_active == false)
    {
        return; /* 手控、触控或外控正在运行时，脚踏通信异常不能清除其它来源的运行请求。 */
    }

    Foot_ClearRunRequestAfterGateFail(); /* 清掉电机和脚踏开泵请求，驱动任务下次发送停止帧。 */
}

/*
 * 函数功能：公共接头缺 EPC 刀具头报警后锁住当前脚踏触发，要求操作者松脚后才能再次启动报警。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_LatchSocketMissing(void)
{
    s_common_socket_missing_wait_release=true;       /* 当前脚踏动作已经触发过缺刀具报警，同一次长踩不再反复请求启动和刷 80。 */
}

/*
 * 函数功能：缺刀具已报过警而脚踏还没松开时，保持停止且不重复报警。
 * 输入参数：无。
 * 返回参数：true 表示必须等待松脚；false 表示可以继续检查是否允许启动。
 */
static bool Foot_BlockIfSocketMissing(void)
{
    if(s_common_socket_missing_wait_release==false)
    {
        return false;                                /* 没有需要等待松脚的缺刀具报警，继续正常启动检查。 */
    }

    Foot_ClearRunRequestAfterGateFail();             /* 脚踏未松开就保持电机和联动泵停止，不能因报警显示结束而重启。 */
    return true;                                     /* 通知调用处退出本周期启动分支，必须等待真实松脚。 */
}

/*
 * 函数功能：检测到松脚后，结束本次缺刀具报警，允许下次踩踏重新检查刀具。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearSocketLatchOnRelease(void)
{
    if(s_common_socket_missing_wait_release==false)
    {
        return;                                      /* 当前没有因缺刀具而等待松脚，不改动其他报警状态。 */
    }

    Pubinterface_ReleaseFootCommonSocketToolMissingAlarm(); /* 脚踏真实松开后关闭本次持续80号弹窗和蜂鸣，不能等待固定2秒退出。 */
    s_common_socket_missing_wait_release=false;      /* 操作者已经松开脚踏，下一次重新踩下才允许重新触发 80 报警。 */
}

/*
 * 函数功能：脚踏启动失败时统一上报码“手柄未连接”，并同步屏幕报警弹窗。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ReportHandleNotConnectedAlarm(void)
{
    uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只读取 Value[0]，其余补零避免旧值残留。 */

    display_value[0] = WORK_ALARM_HANDLE_NOT_CONNECTED; /* 屏幕使用 80 号图显示“手柄未连接，请连接手柄”。 */
    WorkAlarm_Set(WORK_ALARM_HANDLE_NOT_CONNECTED);      /* 把当前报警写入统一报警容器，便于蜂鸣和弹窗共用同一状态。 */
    SendAlarmMessage(WORK_ALARM_HANDLE_NOT_CONNECTED);   /* 同步蜂鸣任务发出报警提示，提醒用户脚踏起机失败。 */
    SendUIDSMessage(UI_AIARM_ID, true, display_value);   /* 同步屏幕报警弹窗，让用户直接看到未连接提示。 */
}

/*
 * 函数功能：判断单踏板保存的低点和高点是否满足有效行程要求。
 * 输入参数：low_adc 为脚踏保存低点；high_adc 为脚踏保存高点。
 * 返回参数：true 表示低点到高点的行程有效；false 表示脚踏存储值错误。
 */
static bool Foot_IsPedalTwoPointStorageValid(uint16_t low_adc, uint16_t high_adc)
{
    if(high_adc <= low_adc)
    {
        return false; /* 高点必须大于低点，否则脚踏行程方向错误，不能允许控制泵或手柄。 */
    }

    if(((uint32_t)high_adc - (uint32_t)low_adc) <= JT_threshold)
    {
        return false; /* 有效行程必须大于启动阈值，避免一点抖动就触发泵或手柄运行。 */
    }

    return true; /* 低点和高点顺序、行程都有效，允许后续接入脚踏。 */
}

/*
 * 函数功能：判断双段脚踏保存的低点、中点和高点是否满足有效行程要求。
 * 输入参数：low_adc 为低点；mid_adc 为中点；high_adc 为高点。
 * 返回参数：true 表示低/中/高值有效；false 表示脚踏存储值错误。
 */
static bool Foot_IsPedalThreePointStorageValid(uint16_t low_adc, uint16_t mid_adc, uint16_t high_adc)
{
    if(Foot_IsPedalTwoPointStorageValid(low_adc, mid_adc) == false)
    {
        return false; /* 低点到中点必须留出足够的轻踩范围，否则不能用脚踏开泵。 */
    }

    if(Foot_IsPedalTwoPointStorageValid(mid_adc, high_adc) == false)
    {
        return false; /* 电机段中点到高点无有效行程时，脚踏不能安全控制手柄。 */
    }

    return true; /* 低/中/高三点递增且两段行程都有效，允许脚踏接入。 */
}

/*
 * 函数功能：上报脚踏存储值错误报警，并锁住后续脚踏泵/手柄控制入口。
 * 输入参数：无，报警码固定为 WORK_ALARM_FOOT_VALUE_ERROR。
 * 返回参数：无。
 */
static void Foot_ReportFootValueErrorAlarm(void)
{
    uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余字节清零避免旧参数残留。 */

    if(WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR))
    {
        return; /* 当前已经是脚踏值错误报警，避免同一坏数据帧重复塞蜂鸣和屏幕队列。 */
    }

    display_value[0] = WORK_ALARM_FOOT_VALUE_ERROR; /* 报警码 6 驱动 UIAIARMDP 显示 EX8 83 号脚踏存储值错误图。 */
    WorkAlarm_Set(WORK_ALARM_FOOT_VALUE_ERROR);     /* 保存脚踏定标值错误报警，启动检查会据此禁止泵和手柄运行。 */
    SendAlarmMessage(WORK_ALARM_FOOT_VALUE_ERROR);  /* 同步蜂鸣报警，提示当前脚踏定标值不可用。 */
    SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 同步屏幕显示 83 号报警弹窗。 */
}

/*
 * 函数功能：手控已选中时踩脚踏，上报 81 号错模式报警并锁住脚踏控制入口。
 * 输入参数：无，报警码固定为 WORK_ALARM_MANUAL_SELECTED。
 * 返回参数：无。
 */
static void Foot_ReportManualSelectedAlarm(void)
{
    uint8_t display_value[10] = {0U}; /* UI_AIARM_ID 只使用 Value[0] 保存报警码，其余清零避免旧弹窗参数残留。 */

    if(WorkAlarm_Is(WORK_ALARM_MANUAL_SELECTED))
    {
        return; /* 脚踏一直踩着时不重复发送 81 号报警，避免屏幕和蜂鸣任务收到大量相同消息。 */
    }

    display_value[0] = WORK_ALARM_MANUAL_SELECTED; /* 81 号图提示当前手控已选中，用户应使用手柄按键启动。 */
    WorkAlarm_Set(WORK_ALARM_MANUAL_SELECTED);     /* 锁住错模式报警状态，松开脚踏后由本模块清零。 */
    SendAlarmMessage(WORK_ALARM_MANUAL_SELECTED);  /* 同步蜂鸣报警，提示当前脚踏启动来源不匹配。 */
    SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 同步屏幕显示 81 号报警弹窗。 */
}

/*
 * 函数功能：脚踏释放后清除“手控已选中，请用手控”错模式报警。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearManualAlarmOnRelease(void)
{
    if(WorkAlarm_Is(WORK_ALARM_MANUAL_SELECTED) == false)
    {
        return; /* 当前不是脚踏错模式报警，不能误清其它报警。 */
    }

    WorkAlarm_Clear();                 /* 用户已经松开脚踏，本次错模式启动尝试结束，释放报警状态。 */
    SendAlarmMessage(WORK_ALARM_NONE); /* 报警状态清零后同步关闭蜂鸣。 */
    SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 关闭 81 号报警弹窗，恢复普通运行页显示。 */
}

/*
 * 函数功能：脚踏存储值恢复有效或错误脚踏拔出后，只清除本模块产生的脚踏值错误报警。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearFootValueErrorAlarm(void)
{
    if(WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR) == false)
    {
        return; /* 当前不是脚踏值错误报警，不能误清其它真实报警。 */
    }

    WorkAlarm_Clear();                 /* 新定标值已有效，或错误脚踏已拔出，清除脚踏值错误报警。 */
    SendAlarmMessage(WORK_ALARM_NONE); /* 停止本模块触发的报警蜂鸣。 */
    SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 关闭 83 号报警弹窗，后续由脚踏接入刷新控制状态。 */
}

/*
 * 函数功能：松脚时清除“手柄未连接”提示；名称保留了 Overload，但过载清除代码当前已停用。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearHandleOrOverloadAlarm(void)
{
    /* 当前只清手柄未连接报警；过载、UID、通信、HALL 等故障不能在这里顺带清除。 */
    if(WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED))
    {
           WorkAlarm_Clear();                 /* 清掉统一报警状态，让后续控制方式可以重新响应。 */
          SendAlarmMessage(WORK_ALARM_NONE); /* 释放脚踏或拔掉脚踏时同步停止报警蜂鸣。 */
          SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 关闭本次“手柄未连接”弹窗。 */
          Handle_SelectRemainingOnlineAfterUnplug(); /* 脚踏真实松开并清除掉线报警后，统一恢复唯一剩余通道且保持电机停止。 */
    }
    // if(WorkAlarm_Is(WORK_ALARM_MOTOR_OVERLOAD))
    // {
    //     times++;
    //     WorkAlarm_Clear();                 /* 清掉统一报警状态，让后续控制方式可以重新响应。 */
    //     SendAlarmMessage(WORK_ALARM_NONE); /* 释放脚踏或拔掉脚踏时同步停止报警蜂鸣。 */
    //     SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 同步关闭屏幕报警弹窗，避免“手柄未连接/过载”残留。 */
    // }
}

/*
 * 函数功能：检查是否选中脚控、手柄是否可用，以及其他控制方式或报警是否禁止启动。
 * 输入参数：无。
 * 返回参数：true 表示本次可继续检查脚踏启动；false 表示不能启动泵和电机。
 */
static bool Foot_EnsureFootControlMode(void)
{
    /* 只有脚控已经被选中时，脚踏动作才允许继续进入运行分支。 */
    if(WorkMessage.drivetype_work!=JTWORK)
    {
        if(WorkMessage.drivetype_work==HANDLEWORK)
        {
            Foot_ReportManualSelectedAlarm(); /* 当前选中手控时踩脚踏，只显示 81 号报警并拒绝泵和手柄运行。 */
        }
        return false;
    }

    /* 没有有效工作通道或没有手柄型号时，踩脚踏必须提示手柄未连接，不能下发运行。 */
    if((WorkMessage.channel_work==CHANNEL_NONE) || (WorkMessage.hand_model==0U))
    {
        Foot_ReportHandleNotConnectedAlarm();
        return false;
    }

    /* 上位机正在控制或触控仍有效时，不允许脚踏插入控制。 */
    if(ControlArbitration_IsExternalActive() || ControlSignalMessage.HMI_control_flag || (WorkMessage.touchactive_work==TOUCHWORK))
    {
        return false;
    }

    /* 其他本地方式仍在控制电机时，脚踏不能抢过来启动。 */
    if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_FOOT))
    {
        return false;
    }

    if(WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR))
    {
        return false; /* 脚踏存储值错误属于输入安全故障，未恢复有效值前禁止控制泵和手柄。 */
    }

    /* 已通过前面的脚控模式检查，清掉此前“手控已选中”的提示，避免旧提示继续挡住启动。 */
    if(WorkAlarm_Is(WORK_ALARM_MANUAL_SELECTED))
    {
        WorkAlarm_ClearIf(WORK_ALARM_MANUAL_SELECTED);
    }
    /* 其它真实报警仍然禁止脚踏启动，避免用脚踏切换掩盖过载、未接手柄等故障。 */
    else if(WorkMessage.alarm_flag==true)
    {
        return false;
    }

    /* 模式选中由 0x2404/key1 或默认回落完成，脚踏启动入口不再自行改写控制方式。 */
    return true;
}

/*
 * 函数功能：判断单踏板低于启动阈值是否只是瞬时抖动，需要暂时忽略本次停泵动作。
 * 输入参数：release_ticks 连续低于阈值的计数指针，由 FootControlTask 保存单踏板释放确认次数。
 * 返回参数：true 表示暂不执行后续停泵步骤；false 表示可以执行停泵。本函数不延迟电机停机。
 */
static bool Foot_IgnoreSingleReleaseGlitch(uint8_t *release_ticks)
{
    /* 防御空指针：如果调用方没有传入计数器，就不能做去抖，直接允许停泵保持旧安全行为。 */
    if(release_ticks == NULL)
    {
        return false;
    }

    /* 未达到连续松脚确认次数前只累计，不清 run_flag，避免踩住脚踏时 AD 抖动让 A 泵一停一启。 */
    if(*release_ticks < FOOT_SINGLE_PEDAL_RELEASE_DEBOUNCE_TICKS)
    {
        (*release_ticks)++;
        return true;
    }

    /* 达到确认次数后清零计数，本次按真实松脚处理，后续下一次踩下再重新统计。 */
    *release_ticks = 0U;
    return false;
}

/*
 * 函数功能：确认松脚后，清除注水泵驱动故障导致的手柄停机记录，允许下次踩踏重试。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearPressureLatchOnRelease(void)
{
    Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 保留旧 Pressure 接口名，实际清泵驱动故障的联动停机记录；泵任务仍检查驱动故障，超压提示本身不停机。 */
}

/*
 * 函数功能：注水泵驱动故障联动停止手柄后，脚踏还踩着就继续禁止电机和泵自动启动。
 * 输入参数：无。
 * 返回参数：true 表示本周期必须保持停机，false 表示允许脚踏按正常路径继续判断。
 */
static bool Foot_BlockRunIfPressureStopLatched(void)
{
    if(Pubinterface_IsPressureBlockStopLatched() == false)
    {
        return false; /* 没有泵驱动故障留下的等待松脚要求，继续正常启动检查。 */
    }

    Foot_LatchMotorStopUntilRelease();              /* 泵驱动故障已触发联动停机，必须先松脚再允许手柄重试启动。 */
    WorkMessage.runflag_work=false;                 /* 故障停机后脚踏仍踩着，手柄运行请求继续保持为停止。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免驱动任务看到旧速度重新输出。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 清左脚踏运行来源，必须等左脚释放后重新踩下才允许再置位。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行来源，双脚踏两侧都不能靠保持踩踏自动恢复。 */
    ControlSignalMessage.jtL_gentlypump_flag=false; /* 清左脚轻踩开泵标志，防止故障停机后保持轻踩就重新开泵。 */
    ControlSignalMessage.jtR_gentlypump_flag=false; /* 清右侧轻踩泵联动来源，防止右脚保持踩下时泵自动恢复。 */
    s_double_left_gently_pump_channel=CHANNEL_NONE; /* 泵驱动故障处理已撤销联动开泵请求，清掉左侧使用的泵记录。 */
    s_double_right_gently_pump_channel=CHANNEL_NONE; /* 清右侧实际泵记录，必须重新踩下才能建立新请求。 */
    return true;                                    /* 通知调用处退出本次脚踏启动分支，等待真实松脚。 */
}

/*
 * 函数功能：判断某一路脚踏 AD 是否已经回到释放区间。
 * 输入参数：ad_value 为当前脚踏 AD 原始值，low_value 为该路脚踏低位基准值。
 * 返回参数：true 表示已经低于启动阈值，false 表示该路脚踏仍处于踩下状态。
 */
static bool Foot_IsPedalReleased(uint16_t ad_value, uint16_t low_value)
{
    return ((uint32_t)ad_value <= ((uint32_t)low_value + (uint32_t)JT_threshold)); /* AD 不超过基准值加 20 才算已退出该区间，供松脚和退出电机区的检查共用。 */
}

/*
 * 函数功能：任意脚踏来源驱动故障后阻止保持踩下自动重启，并检测松脚条件。
 * 输入参数：msg 为最近一次有效脚踏连接和定标消息。
 * 返回参数：当前必须保持停机返回 true；允许继续正常脚踏处理返回 false。
 */
static bool Foot_BlockDriverAlarmRun(const FootMessage_t *msg)
{
    bool is_released = false; /* 保存当前脚踏是否已真实回到释放区，双脚踏必须左右均释放。 */
    if (MotorUart_IsFootDriverAlarmWaitingRelease() == false)
    {
        return false; /* 当前没有脚踏来源驱动故障等待松脚，保持原有脚踏启动流程。 */
    }
    if ((msg == NULL) || (msg->connect_flag == false))
    {
        is_released = true; /* 没有连接消息或脚踏已掉线时，按已松脚处理，驱动报警不再只因缺少松脚数据而一直保留。 */
    }
    else
    {
        switch (msg->pedalType)
        {
            case 1U:
                is_released = Foot_IsPedalReleased(jt_adcvalue, msg->LValue_Left); /* 单踏板只检查单路 AD。 */
                break;
            case 2U:
                is_released = Foot_IsPedalReleased(jtb_adcvalue, msg->LValue_Left); /* 双段踏板只检查共用踏板 AD。 */
                break;
            case 3U:
                is_released = Foot_IsPedalReleased(jtd_adcvalue_l, msg->LValue_Left) &&
                              Foot_IsPedalReleased(jtd_adcvalue_r, msg->LValue_Right); /* 双脚踏必须左右都释放，不能由另一侧持续踩住恢复运行。 */
                break;
            default:
                break; /* 不认识脚踏类型就无法判断是否松脚，继续禁止启动。 */
        }
    }
    if (is_released)
    {
        MotorUart_ReleaseFootDriverAlarm(); /* 保存脚踏释放结果；只有驱动Err同时恢复后才关闭弹窗和蜂鸣。 */
        return false;
    }
    Foot_ClearRunRequestAfterGateFail(); /* 长踩期间持续撤销电机、脚踏来源和联动泵请求，防止任意驱动报警恢复后自动运行。 */
    return true;
}
/*
 * 函数功能：双脚踏切换手柄后，要求对应侧先松脚，防止切换完成立即运行。
 * 输入参数：channel 为切换后的 A/B 手柄通道。
 * 返回参数：无。
 */
static void Foot_RequireDoublePedalRelease(uint8_t channel)
{
    s_double_pedal_release_before_run_channel = channel; /* 记住需要先松脚的通道，后续深踩不能直接启动它。 */
}

/*
 * 函数功能：查询指定手柄通道是否仍在等待切换后的松脚动作。
 * 输入参数：channel 为 A/B 手柄通道。
 * 返回参数：true 表示必须先松脚；false 表示没有此限制。
 */
static bool Foot_DoublePedalIsWaitingRelease(uint8_t channel)
{
    return (s_double_pedal_release_before_run_channel == channel); /* 只限制记录的通道，不把另一通道误判为等待松脚。 */
}

/*
 * 函数功能：对应侧松脚后，取消手柄切换后的等待松脚要求。
 * 输入参数：channel 为本次松脚对应的 A/B 手柄通道。
 * 返回参数：无。
 */
static void Foot_DoublePedalMarkReleased(uint8_t channel)
{
    /* 必须是切换后要求松脚的那一侧，另一侧松脚不算。 */
    if(s_double_pedal_release_before_run_channel == channel)
    {
        s_double_pedal_release_before_run_channel = CHANNEL_NONE; /* 已完成松脚，下次踩下可以重新检查启动。 */
    }
}

/**
 * @brief 获取解析后的数据
 * @return 解析数据指针
 */

/**
 * @brief 清除按键状态
 */


/* 踏板处理结果决定是否继续处理另一侧，以及是否执行任务末尾的电机空闲检查。 */
typedef enum
{
    FOOT_CONTROL_FLOW_CONTINUE = 0, /* 左侧处理完毕，继续处理右侧。 */
    FOOT_CONTROL_FLOW_FINISH_CYCLE, /* 结束本次踏板处理，仍执行任务末尾的空闲检查。 */
    FOOT_CONTROL_FLOW_RETURN_TASK /* 立即退出任务，不再执行末尾的空闲检查。 */
} FootControlFlow_t;

static uint8_t s_single_release_debounce_ticks = 0U; /* 记录单踏板连续处于松脚区的次数；再次踩下时清零，供延迟停泵使用。 */

/*
 * 函数功能：收到脚踏接入或掉线消息后，更新可选控制方式、运行状态和屏幕高亮。
 * 输入参数：msg 指向本次从队列收到的脚踏连接状态和定标参数。
 * 返回参数：无。
 */
static void Foot_HandleConnectionUpdate(const FootMessage_t *msg)
{
    if (msg == NULL)
    {
        return; /* 队列消息无效时不改任何脚踏、电机或界面状态。 */
    }

    if(msg->connect_flag==false)
    {
        Foot_LatchMotorStopUntilRelease(); /* 脚踏掉线后禁止重连旧高AD直接起机，必须先收到一帧真实松脚位置。 */
        MotorUart_ReleaseFootDriverAlarm(); /* 对驱动报警恢复来说，脚踏掉线按已松脚处理，仍需等待驱动错误消失。 */
        if( WorkMessage.drivetype_work==JTWORK)// 掉线前选中脚控时，需要重新选择还能使用的控制方式。
        {
            if(ControlSignalMessage.jtL_control_flag||ControlSignalMessage.jtR_control_flag)//如果当前正是脚踏控制电机过程中
            {
                ControlSignalMessage.jtL_control_flag=false;// 清除左脚踏运行标志，掉线后不再接受原来的踩踏请求。
                ControlSignalMessage.jtR_control_flag=false;
                WorkMessage.runflag_work=false;//电机停止运行
                if(WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED||WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD)// 进入旧报警处理入口；该函数当前只清手柄未连接，不清过载。
                 {
                    Foot_ClearHandleOrOverloadAlarm();
                 }

            }
            if(Pubinterface_IsHandleControlReservedModel(WorkMessage.hand_model))//只允许真实具备手柄按键能力的型号在脚踏掉线后回到手控，PXBB必须保持非手控状态
            {
                 ControlSignalMessage.handle_enable_flag=true;//控制信号，手控使能开启
                 WorkMessage.drivetype_work=HANDLEWORK;//工作标志手控
                 if(WorkMessage.channel_work==CHANNEL_A)
                 {
                    MemoryMsgA.drive_type=HANDLEWORK;// 保存 A 通道改用手控，下次切回 A 时恢复此选择。
                 }
                 else if(WorkMessage.channel_work==CHANNEL_B) /* 掉线前工作通道为 B 时，只把 B 通道记忆恢复为手控。 */
                 {
                    MemoryMsgB.drive_type=HANDLEWORK;
                 }
            }
            else
            {
                 WorkMessage.drivetype_work=NOWORK;//无控制
            }

        }
        else
        {
            //工作状态什么都不变，只是脚踏掉线了，
        }
         //通知界面，如果因为脚踏行为报警，则恢复
         ControlSignalMessage.jt_enable_flag=false;// 脚踏掉线后不可再选择脚控，界面根据此标志更新可选项。
         Pubinterface_ClearFootControlManualLock(); /* 脚踏已拔出，清掉本次连接期间用户手动选择手控/触控的记录。 */
         Foot_ClearManualAlarmOnRelease(); /* 脚踏掉线等价于释放脚踏，清除手控已选中的错模式报警。 */
         Foot_ClearSocketLatchOnRelease(); /* 脚踏掉线等价于释放脚踏，公共接头缺刀具等待松脚状态也必须结束。 */

         if(ControlSignalMessage.jtL_control_flag||ControlSignalMessage.jtR_control_flag)//如果当前正是脚踏控制电机过程中
         {
            ControlSignalMessage.jtL_control_flag=false;// 清除左脚踏运行标志，掉线后不再接受原来的踩踏请求。
            ControlSignalMessage.jtR_control_flag=false;
            WorkMessage.runflag_work=false;//电机停止运行
            if(WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED||WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD)// 掉线时检查旧报警入口；实际只清手柄未连接提示。
             {
                Foot_ClearHandleOrOverloadAlarm();
             }

         }
         Pubinterface_RefreshControlModeDisplay();//脚踏掉线后统一重绘脚控/手控/触控，清掉旧高亮残留
     }
    else
    {
        ControlSignalMessage.jt_enable_flag=true;
       (void)Pubinterface_ApplyFootControlPriorityOnConnect(); /* 脚踏上线时统一按脚控优先处理，未运行且未被屏幕手动锁住时会覆盖手控/触控。 */
       Pubinterface_RefreshControlModeDisplay();//脚踏上线后统一刷新三种控制方式，避免其它图标残留高亮
    }
}

/*
 * 函数功能：按双脚踏左右侧的固定优先级启动实际注水泵，并保存本侧泵通道。
 * 输入参数：right_pedal 为 false 时处理左脚 A 优先，为 true 时处理右脚 B 优先。
 * 返回参数：无。
 */
static void Foot_StartDoublePedalGentlyPump(bool right_pedal)
{
    uint8_t pump_channel = CHANNEL_NONE; /* 本次真正启动的泵通道，无注水泵时保持 NONE。 */

    if(right_pedal)
    {
        if(pumpMessageB.type==INJECTWATER)
        {
            Foot_StartPumpBInjection(pumpMessageB.speed_work, true, true); /* 右脚优先启动 B 泵，轻踩输出直接使用 B 泵屏幕设定速度。 */
            pump_channel = CHANNEL_B; /* 保存实际启动 B，松右脚时只处理 B。 */
        }
        else if(pumpMessageA.type==INJECTWATER)
        {
            Foot_StartPumpAInjection(pumpMessageA.speed_work, true, true); /* B 不是注水泵时回退 A，轻踩输出使用 A 泵屏幕设定速度。 */
            pump_channel = CHANNEL_A; /* 保存回退启动的 A，停止时不能再按泵类型优先级判断。 */
        }
        s_double_right_gently_pump_channel = pump_channel; /* 每周期刷新右侧实际泵记录，松脚依据该记录停止。 */
        if(pump_channel==CHANNEL_NONE)
        {
            ControlSignalMessage.jtR_gentlypump_flag=false; /* A/B 都不是注水泵，清掉右脚轻踩开泵标志。 */
        }
    }
    else
    {
        if(pumpMessageA.type==INJECTWATER)
        {
            Foot_StartPumpAInjection(pumpMessageA.speed_work, false, true); /* 左脚优先启动 A 泵，轻踩输出直接使用 A 泵屏幕设定速度。 */
            pump_channel = CHANNEL_A; /* 保存实际启动 A，松左脚时只处理 A。 */
        }
        else if(pumpMessageB.type==INJECTWATER)
        {
            Foot_StartPumpBInjection(pumpMessageB.speed_work, false, true); /* A 不是注水泵时回退 B，轻踩输出使用 B 泵屏幕设定速度。 */
            pump_channel = CHANNEL_B; /* 保存回退启动的 B，松脚时准确停止 B。 */
        }
        s_double_left_gently_pump_channel = pump_channel; /* 每周期刷新左侧实际泵记录。 */
        if(pump_channel==CHANNEL_NONE)
        {
            ControlSignalMessage.jtL_gentlypump_flag=false; /* A/B 都不是注水泵，清掉左脚轻踩开泵标志。 */
        }
    }
}

/*
 * 函数功能：释放指定双脚踏侧记录的注水泵；另一侧仍使用同一泵时保持运行。
 * 输入参数：right_pedal 为 false 表示松开左脚，为 true 表示松开右脚。
 * 返回参数：无。
 */
static void Foot_StopDoublePedalGentlyPump(bool right_pedal)
{
    uint8_t released_channel; /* 保存本次松脚需要释放的实际泵通道。 */
    uint8_t other_channel; /* 保存另一侧仍在保持的泵通道，避免提前停掉共享泵。 */

    if(right_pedal)
    {
        released_channel = s_double_right_gently_pump_channel; /* 读取右脚实际启动的泵。 */
        s_double_right_gently_pump_channel = CHANNEL_NONE; /* 右脚已松开，先清掉右侧使用哪台泵的记录。 */
        ControlSignalMessage.jtR_gentlypump_flag=false; /* 右脚已松开，清掉本次轻踩开泵标志。 */
        other_channel = s_double_left_gently_pump_channel; /* 左脚若仍踩住同一泵，本次不能停。 */
    }
    else
    {
        released_channel = s_double_left_gently_pump_channel; /* 读取左脚实际启动的泵。 */
        s_double_left_gently_pump_channel = CHANNEL_NONE; /* 左脚已松开，清掉左侧使用哪台泵的记录。 */
        ControlSignalMessage.jtL_gentlypump_flag=false; /* 左脚已松开，清掉本次轻踩开泵标志，避免重复停泵。 */
        other_channel = s_double_right_gently_pump_channel; /* 右脚若仍使用同一泵则继续保持输出。 */
    }

    if(released_channel==other_channel)
    {
        return; /* 两侧共同使用同一台注水泵时，只释放本侧记录，等待另一侧也松开。 */
    }
    if(released_channel==CHANNEL_A)
    {
        Foot_StopPumpAInjection(); /* 本侧实际启动 A 且另一侧未使用 A，准确停止 A。 */
    }
    else if(released_channel==CHANNEL_B)
    {
        Foot_StopPumpBInjection(); /* 本侧实际启动 B 且另一侧未使用 B，准确停止 B。 */
    }
}

/*
 * 函数功能：每 25ms 按单踏板踩下深度调速，检查能否启动，并在松脚后停止电机和泵。
 * 输入参数：msg 指向当前保存的单踏板在线状态及低点、高点定标值。
 * 返回参数：FINISH_CYCLE 表示继续执行任务末尾的空闲检查；RETURN_TASK 表示立即退出任务。
 */
static FootControlFlow_t Foot_ProcessSinglePedal(const FootMessage_t *msg)
{
    uint16_t adValue; /* 当前单踏板 AD 值，计算前将低于定标低点的值提高到低点。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 没有脚踏参数就不启动，任务末尾仍检查是否可以取消脚踏对电机的占用。 */
    }

    adValue = jt_adcvalue; /* 读取单踏板当前 AD，后续按本次定标低点计算有效行程。 */
    if (adValue < msg->LValue_Left) /* 低于定标低点的采样按低点处理，避免无符号减法回绕。 */
    {
        adValue = msg->LValue_Left;
    }
    if ((adValue - msg->LValue_Left) > JT_threshold) /* 比低点高出 20 个 AD 计数以上才算踩下，随后检查启动条件并计算速度。 */
        {
            if(Foot_BlockRunIfPressureStopLatched())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 泵驱动故障停机后脚踏仍踩着，本周期不允许重新写运行请求，必须先松脚。 */
            }
            if(Foot_BlockIfSocketMissing())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 公共接头缺刀具后脚踏仍踩住时，只保持停机，等待松脚后才允许重新报警。 */
            }
                ControlSignalMessage.jtL_control_flag=true;
            /* 单踏板重新确认踩下后清掉松脚去抖计数，防止上一轮释放残留影响本次泵保持运行。 */
            s_single_release_debounce_ticks=0U;
            if(Foot_EnsureFootControlMode() == false)
            {
                Foot_ClearRunRequestAfterGateFail(); /* 模式或手柄状态不允许脚踏运行时，撤销前面提前置位的脚踏请求。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            }
            // 上位机正在控制时，脚踏不能启动。
            if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
            {
                // 以下按键蜂鸣未启用，当前这里只取消脚踏请求。
               // SendKeyBeepMessage(1);// 若启用会在每次检查时蜂鸣一次，不能直接用作一次性提示。
                Foot_ClearRunRequestAfterGateFail(); /* 上位机正在控制，清掉前面写入的脚踏运行请求。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }
           /* 单踏板要等手柄获准运行才开注水泵，避免电机未启动而泵单独转。 */
            if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
            /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
            if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
            {
                Foot_LatchSocketMissing(); /* 本次脚踏已经触发公共接头缺刀具报警，后续长踩必须等松脚再触发。 */
                Foot_ClearRunRequestAfterGateFail(); /* 公共接头缺刀具头时只报警 80，并保证脚踏不会留下运行状态。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            } /* 脚踏启动手柄电机前检查公共接头 EPC 刀具头，缺失时只报警不运行。 */
            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
            {
                Foot_ClearRunRequestAfterGateFail(); /* 其他方式仍在控制电机，撤销本次脚踏请求。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            }

            if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_LEFT) == false)
            {
                Foot_ClearRunRequestAfterGateFail(); /* 上电或异常恢复后脚踏尚未真实松开，本次高AD不能直接恢复手柄运行。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            }
            WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(jt_adcvalue,msg->LValue_Left,msg->HValue_Left,0U); /* 单踏板按低值到高值线性映射，输出限制在 EEPROM 最小速度到设定速度之间。 */
            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
            Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 手柄获准运行后启动联动泵，并记住是单踏板启动，方便松脚时停泵。 */
        }
        else
        {
            Foot_StopMotorAtReleaseBoundary(); /* 单踏板首次回到低阈值就立即停手柄；下面75ms去抖只继续保护泵输出不抖动。 */
            /* 单踏板踩住时 AD 可能短暂跌回阈值以下，先去抖，避免 A/B 冷却泵被一个采样毛刺立刻停掉。 */
            if(Foot_IgnoreSingleReleaseGlitch(&s_single_release_debounce_ticks))
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE;
            }
            Foot_StopHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 确认松脚后按保存的启动侧停止联动泵，即使 jtL 已被异常处理清零也能停泵。 */
            if(ControlSignalMessage.jtL_control_flag)
            {
                WorkMessage.runflag_work=false;

                // 本次曾由单踏板开泵时，按 A 优先、B 备用的顺序停止注水泵。
                if(ControlSignalMessage.jtL_gentlypump_flag)
                {
                 ControlSignalMessage.jtL_gentlypump_flag=false;
                 if(pumpMessageA.type==INJECTWATER) /* 有单踏板开泵记录且 A 为注水泵时，松脚停止 A。 */
                 {
                   Foot_StopPumpAInjection();
                 }
                 else if(pumpMessageB.type==INJECTWATER) /* A 不是注水泵时，按原选择顺序停止实际使用的 B 注水泵。 */
                 {
                   Foot_StopPumpBInjection();
                 }
                }
                ControlSignalMessage.jtL_control_flag=false;
                WorkMessage.speed_work=0;

               if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 松脚后检查报警；调用的函数当前只清手柄未连接。
               {
                Foot_ClearHandleOrOverloadAlarm();
               }
                Foot_ClearManualAlarmOnRelease(); /* 单踏板确认松开后清除 81 号错模式报警。 */
                Foot_ClearSocketLatchOnRelease(); /* 单踏板确认松开后允许下一次缺刀具启动重新弹 80。 */
            }
            else
            {
                Foot_ClearPressureLatchOnRelease(); /* 单踏板已松开，清除泵驱动故障留下的联动停机记录，下次踩下可以重试。 */
                Foot_ClearManualAlarmOnRelease(); /* 单踏板无运行标志但已释放时，也要关闭手控已选中报警。 */
                Foot_ClearSocketLatchOnRelease(); /* 即使本次没启动成功，松脚后也结束缺刀具报警的等待状态。 */
                //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
            }
            Foot_ClearHandleOrOverloadAlarm(); /* 即使启动失败时已经清掉运行标志，松脚后也要关闭“手柄未连接”提示，下次踩下重新检查。 */
        }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 处理完毕，回到任务末尾检查电机空闲时能否取消脚踏占用。 */
}

/*
 * 函数功能：处理双段踏板一个 25ms 周期的轻踩注水、深踩电机运行和完全释放。
 * 输入参数：msg 指向当前保存的双段踏板在线状态及低点、中点、高点定标值。
 * 返回参数：FINISH_CYCLE 表示继续执行任务末尾的空闲检查；RETURN_TASK 表示立即退出任务。
 */
static FootControlFlow_t Foot_ProcessTwoStagePedal(const FootMessage_t *msg)
{
    uint16_t adValue; /* 本周期双段踏板 AD，分别用于轻踩段和电机段判断。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无有效定标参数时不启动泵和电机。 */
    }

    adValue=jtb_adcvalue;
    if(adValue<msg->LValue_Left)adValue=msg->LValue_Left;
    if(adValue-msg->LValue_Left>JT_threshold)// 超过低点启动范围后可以开注水泵，A 泵优先，A 不是注水泵时再选 B。
    {
        if(Foot_BlockRunIfPressureStopLatched())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 泵驱动故障联动停机后还没松脚，轻踩也不能重新开泵。 */
        }
        if(Foot_BlockIfSocketMissing())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 缺刀具报警后踏板还踩着，不再启动轻踩泵，也不重复弹 80。 */
        }
        ControlSignalMessage.jtL_control_flag=true;
        //   if(WorkMessage.alarm_flag!=0)
        //     {
        //         //报警状态不允许脚踏任何动作
        //          return;
        //     }
           if(Foot_EnsureFootControlMode() == false)
           {
               Foot_ClearRunRequestAfterGateFail(); /* 当前模式不允许脚踏启动，清掉轻踩时提前写入的运行标志。 */
               return FOOT_CONTROL_FLOW_RETURN_TASK;
           }
              // 再检查上位机控制标志；有效时本次脚踏动作不执行。
            if(ControlSignalMessage.HMI_control_flag)
            {
                // 以下按键蜂鸣未启用，当前这里只取消脚踏请求。
                //SendKeyBeepMessage(1);// 若启用会在每次检查时蜂鸣一次，不能直接用作一次性提示。
                Foot_ClearRunRequestAfterGateFail(); /* 外控仍在占用时，脚踏轻踩请求不能保留到下一周期。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }

            /* 轻踩只启动注水泵；进入深踩区后才检查脚踏能否使用手柄电机。 */

            if(pumpMessageA.type==INJECTWATER)// A 配置为注水泵时优先使用 A。
            {
                 /* A 泵轻踩阶段沿用屏幕当前设定流量，快速踩入电机段时不再短暂写入 Page4 默认流量。 */
                 Foot_StartPumpAInjection(pumpMessageA.speed_work, false, false);
            }
            else if(pumpMessageB.type==INJECTWATER)
            {
                 /* A 不是注水泵而 B 是时，给 B 泵发送运行请求。 */
                 Foot_StartPumpBInjection(pumpMessageB.speed_work, false, false);
            }
            ControlSignalMessage.jtL_control_flag=true;
    }
    else
    {
         Foot_StopMotorAtReleaseBoundary(); /* 双段踏板完全退出轻踩区时再次确认手柄停止，并允许下一次重新踩下。 */
         Foot_StopHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 双段踏板完全松开时释放深踩阶段建立的联动泵；退回轻踩区时仍保持泵运行。 */
         if(ControlSignalMessage.jtL_control_flag)
            {

                if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 完全松脚后检查报警；调用的函数当前只清手柄未连接。
                    {
                            Foot_ClearHandleOrOverloadAlarm();
                    }
                WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                // 结束本次双段踏板电机运行记录；注水泵在下面单独停止。
                ControlSignalMessage.jtL_control_flag=false;


                 ControlSignalMessage.jtL_control_flag=false;
            }
            // 本次轻踩开过注水泵时，完全松脚后停止该泵。
            if(ControlSignalMessage.jtL_gentlypump_flag)
                 {
                   ControlSignalMessage.jtL_gentlypump_flag=false;
                  if(pumpMessageA.type==INJECTWATER) /* 双段踏板轻踩使用 A 注水泵时，完全松脚后停止 A。 */
                  {
                    Foot_StopPumpAInjection();
                  }
                  else if(pumpMessageB.type==INJECTWATER) /* A 不是注水泵时，完全松脚后停止实际使用的 B 泵。 */
                  {
                     Foot_StopPumpBInjection();
                   }
                 }
             Foot_ClearHandleOrOverloadAlarm(); /* 即使本次启动失败，完全松脚后仍需关闭“手柄未连接”提示，下次踩下重新检查。 */
             Foot_ClearPressureLatchOnRelease(); /* 踏板已退到低点附近，清除泵驱动故障后的联动停机记录，下次踩下重新检查能否运行。 */
             Foot_ClearManualAlarmOnRelease(); /* 双段脚踏完全松开后清除 81 号错模式报警。 */
             Foot_ClearSocketLatchOnRelease(); /* 双段脚踏完全释放后，下一次踩下才允许公共接头缺刀具重新报警。 */
     }
    if(adValue<msg->MValue_Left)adValue=msg->MValue_Left;
    if(adValue-msg->MValue_Left>JT_threshold)
    {
         //手柄运行
           if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
        /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
        if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
        {
            Foot_LatchSocketMissing(); /* 双段脚踏电机段已触发公共接头缺刀具报警，保持踩下时不再重复弹 80。 */
            Foot_ClearRunRequestAfterGateFail(); /* 双段脚踏进入电机段前发现公共接头无刀具，停泵并清运行请求。 */
            return FOOT_CONTROL_FLOW_RETURN_TASK;
        } /* 脚踏比例启动前先确认刀具头参数有效，避免公共接头空刀具运行。 */
        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
        {
            Foot_ClearRunRequestAfterGateFail(); /* 脚踏不能使用电机时，停掉轻踩开的泵并清除脚踏运行请求。 */
            return FOOT_CONTROL_FLOW_RETURN_TASK;
        }
        ControlSignalMessage.jtL_control_flag=true;
        if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_LEFT) == false)
        {
            Foot_ClearRunRequestAfterGateFail(); /* 双段脚踏在异常恢复后仍深踩时保持停机，必须先回到松脚区。 */
            return FOOT_CONTROL_FLOW_RETURN_TASK;
        }
        WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg->MValue_Left,msg->HValue_Left,0U); /* 双段脚踏电机段从中值开始算比例，不再从 0rpm 起步。 */
        WorkMessage.runflag_work=true;//通知SSCdrive电机运行
        Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 深踩获准启动手柄后，记住该踏板启动了联动泵，完全松脚时据此停泵。 */
    }
    else
    {
         Foot_StopMotorAtReleaseBoundary(); /* 双段踏板退回中点以下即退出电机段，不能等待轻踩泵释放才停止手柄。 */
         if(ControlSignalMessage.jtL_control_flag)
            {
                 if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 退出深踩区后检查报警；当前只清手柄未连接，不清过载。
                    {
                        Foot_ClearHandleOrOverloadAlarm();
                    }
                WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                // 这里只清电机运行记录，退回轻踩区仍可继续注水。
                ControlSignalMessage.jtL_control_flag=false;
            }
            else
            {
                //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
            }
    }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 处理完毕，任务末尾检查电机空闲时能否取消脚踏占用。 */
}

/*
 * 函数功能：处理双踏板左侧一个 25ms 周期的轻踩注水、A 通道运行、跨通道切换和释放复位。
 * 输入参数：msg 指向当前保存的双踏板左侧低点、中点和高点定标值。
 * 返回参数：CONTINUE 表示继续右侧；FINISH_CYCLE 表示只做末尾空闲检查；RETURN_TASK 表示立即退出任务。
 */
static FootControlFlow_t Foot_ProcessDoublePedalLeft(const FootMessage_t *msg)
{
    uint16_t adValue; /* 左踏板本次 AD 值；计算轻踩和深踩距离前，先保证它不低于对应的定标起点。 */
    uint8_t switch_before_channel = CHANNEL_NONE; /* 保存切换前的通道，确认确实切到 A 后才蜂鸣一次。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无左侧定标参数时结束本周期，不继续处理右侧。 */
    }

    adValue = jtd_adcvalue_l; /* 读取双脚踏左侧 AD，按左侧独立定标区间处理。 */
    if (adValue < msg->LValue_Left) /* 低于左侧低点时按低点处理，避免无符号相减得到很大的正数。 */
    {
        adValue = msg->LValue_Left;
    }
        if ((adValue - msg->LValue_Left) > JT_threshold) /* 左侧超过踩下阈值后才进入轻踩泵和深踩电机流程。 */
        {
            if(Foot_BlockRunIfPressureStopLatched())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 泵驱动故障联动停机后左脚还踩着，本周期不再启动泵或电机。 */
            }
            if(Foot_BlockIfSocketMissing())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏左侧缺刀具后保持踩下时，本周期只维持停机并等待松脚。 */
            }
            ControlSignalMessage.jtL_control_flag=true;

           if(Foot_EnsureFootControlMode() == false)
           {
               Foot_ClearRunRequestAfterGateFail(); /* 左脚踏不满足启动条件，清掉轻踩时写入的运行请求。 */
               return FOOT_CONTROL_FLOW_RETURN_TASK;
           }

             // 上位机正在控制时，左脚踏不能启动。
            if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
            {
                // 以下按键蜂鸣未启用，当前这里只取消左脚踏请求。
                //SendKeyBeepMessage(1);// 若启用会在每次检查时蜂鸣一次，不能直接用作一次性提示。
                Foot_ClearRunRequestAfterGateFail(); /* 外部控制中左脚踏不参与，清掉本周期脚踏请求。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }

            /* 左脚轻踩只开泵，进入深踩区后才检查能否使用手柄电机。 */

            Foot_StartDoublePedalGentlyPump(false); /* 左脚按 A 优先规则启动并记录实际泵，使用屏幕设定速度且不进入10秒定时排空。 */

            if (adValue < msg->MValue_Left) /* 低于中点时按中点处理，避免相减得到很大的正数而误认为深踩。 */
            {
                adValue = msg->MValue_Left;
            }
          if ((adValue - msg->MValue_Left) > JT_threshold) /* 比中点高出 20 个 AD 计数以上才算深踩，才允许检查电机启动。 */
            {
                if(WorkMessage.channel_work==CHANNEL_A)
                {
                    /* 双踏板左侧触发当前通道启动前先占用脚踏控制权。 */
                    if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
                    {
                        Foot_LatchSocketMissing(); /* 左脚踏当前通道已触发缺刀具报警，长踩期间不再重复弹窗。 */
                        Foot_ClearRunRequestAfterGateFail(); /* 左脚踏当前通道启动前发现缺刀具，停泵并保持电机停机。 */
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    } /* 启动当前通道前必须确认公共接头已识别到刀具头。 */
                    if(Foot_DoublePedalIsWaitingRelease(CHANNEL_A))
                    {
                        ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    }
                    if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                    {
                        Foot_ClearRunRequestAfterGateFail(); /* 左脚踏不能使用当前电机时，取消泵和电机运行请求。 */
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    }
                    ControlSignalMessage.jtL_control_flag=true;
                    if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_LEFT) == false)
                    {
                        Foot_ClearRunRequestAfterGateFail(); /* 左踏板异常恢复后仍处于运行段时保持停机，禁止旧行程直接重启。 */
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    }
                    WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg->MValue_Left,msg->HValue_Left,FOOT_PEDAL_SPEED_HIGH_MARGIN); /* 双脚踏左侧当前通道保留高位死区，并限制最大不超过设定速度。 */
                    WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                    Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 记住联动泵由左脚启动，右脚松开不能撤销这次开泵请求。 */
                }
                else if(WorkMessage.channel_work==CHANNEL_B) /* 左脚对应 A 通道；当前在 B 时需判断能否切回 A。 */
                {
                        if(WorkMessage.Channel_Aonline==true) /* A 手柄在线时先按去抖次数切到 A，避免一次采样抖动切通道。 */
                        {
                            WorkMessage.switchhandle_counts++;
                            if(WorkMessage.switchhandle_counts<FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS)
                            {
                                ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                return FOOT_CONTROL_FLOW_RETURN_TASK;//未达到切换去抖计数，不执行
                            }
                             WorkMessage.switchhandle_counts=0;
                            switch_before_channel = WorkMessage.channel_work; /* 记录切换前通道，后续用实际 channel_work 判断是否成功切到 A。 */
                            HandleSwitchActive(SCREENKey_HANDLE_A);//切换
                            if((switch_before_channel != CHANNEL_A) && (WorkMessage.channel_work == CHANNEL_A))
                            {
                                SendKeyBeepMessage(1U); /* 双脚踏左踏板跨通道切换 A 手柄成功，蜂鸣一次给操作者确认。 */
                            }
                            Foot_RequireDoublePedalRelease(CHANNEL_A);
                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                            return FOOT_CONTROL_FLOW_RETURN_TASK;

                        }
                        else
                        {
                            if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
                            {
                                Foot_LatchSocketMissing(); /* 左脚踏跨通道已触发缺刀具报警，必须松脚后才允许再次触发。 */
                                Foot_ClearRunRequestAfterGateFail(); /* 左脚踏跨通道直接运行前发现公共接头无刀具，立即撤销脚踏请求。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            } /* A 手柄不在线而左脚准备控制 B 时，也必须检查公共接头是否有刀具头。 */
                            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                            {
                                Foot_ClearRunRequestAfterGateFail(); /* 左脚不能使用 B 通道电机时，也要停掉此前轻踩开启的泵。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            }
                            ControlSignalMessage.jtL_control_flag=true;
                            if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_LEFT) == false)
                            {
                                Foot_ClearRunRequestAfterGateFail(); /* 左踏板跨通道前仍需满足先松后踩，不能由旧高AD完成切换起机。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            }
                            WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg->MValue_Left,msg->HValue_Left,0U); /* 双脚踏左侧跨通道运行同样按 EEPROM 最小速度起步。 */
                            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                            Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 左脚控制 B 通道时仍记为左脚启动，松左脚时据此停联动泵。 */
                        }
                }
            }
            else
            {
                Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_LEFT); /* 左脚退回轻踩区；电机若由左脚启动就要求停机。 */
                  WorkMessage.switchhandle_counts=0;
                if(ControlSignalMessage.jtL_control_flag)
                {
                     WorkMessage.runflag_work=false;
                    WorkMessage.speed_work=0;
                    // 左脚退回轻踩区后取消电机请求，轻踩注水泵仍可继续运行。
                    ControlSignalMessage.jtL_control_flag=false; /* 左踏板退回轻踩区时只清左侧电机来源，不能误清右侧。 */

                    if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 左脚退出电机区后检查报警；当前只清手柄未连接。
                    {
                            Foot_ClearHandleOrOverloadAlarm();
                    }
                 }
            }
        }
        else
        {

            Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_LEFT); /* 左侧完全松开只停止左侧实际启动的电机请求，不能误停右侧。 */
            Foot_StopHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_LEFT); /* 只撤销左脚发出的联动开泵请求，不撤销右脚的请求。 */
             WorkMessage.switchhandle_counts=0;
            Foot_DoublePedalMarkReleased(CHANNEL_A);
            if(ControlSignalMessage.jtL_control_flag)
            {
               WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                // 左脚已完全松开，清电机运行记录；下面再按保存的泵通道停止注水。
                ControlSignalMessage.jtL_control_flag=false;

              if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 左脚完全松开后检查报警；当前只清手柄未连接。
               {
                    Foot_ClearHandleOrOverloadAlarm();
               }
            }
             if(ControlSignalMessage.jtL_gentlypump_flag)
             {
                 Foot_StopDoublePedalGentlyPump(false); /* 松左脚只释放左侧实际启动的泵，不能按当前泵类型重新猜。 */
             }
             if(Foot_IsPedalReleased(jtd_adcvalue_r, msg->LValue_Right))
             {
                 Foot_ClearHandleOrOverloadAlarm(); /* 左右都松开后，即使运行标志早已清掉，也要关闭“手柄未连接”提示。 */
                 Foot_ClearPressureLatchOnRelease(); /* 左右脚都已松开，才清除泵驱动故障留下的联动停机记录。 */
                 Foot_ClearManualAlarmOnRelease(); /* 双脚踏左右都释放后清除手控已选中的错模式报警。 */
                 Foot_ClearSocketLatchOnRelease(); /* 双脚踏左右都释放后，下一次左/右脚踏启动才允许重新弹 80。 */
             }
            //左脚停止
         }

    return FOOT_CONTROL_FLOW_CONTINUE; /* 左侧正常处理结束后，保持原顺序继续处理右踏板。 */
}

/*
 * 函数功能：处理双踏板右侧一个 25ms 周期的轻踩注水、B 通道运行、跨通道切换和释放复位。
 * 输入参数：msg 指向当前保存的双踏板右侧低点、中点和高点定标值。
 * 返回参数：FINISH_CYCLE 表示继续执行任务末尾的空闲检查；RETURN_TASK 表示立即退出任务。
 */
static FootControlFlow_t Foot_ProcessDoublePedalRight(const FootMessage_t *msg)
{
    uint16_t adValue_r; /* 右踏板本次 AD 值；计算轻踩和深踩距离前，先保证它不低于对应的定标起点。 */
    uint8_t switch_before_channel = CHANNEL_NONE; /* 保存切换前的通道，确认确实切到 B 后才蜂鸣一次。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 没有右侧定标参数就不启动，仍执行任务末尾的空闲检查。 */
    }

    adValue_r=jtd_adcvalue_r;
    if(adValue_r<msg->LValue_Right)adValue_r=msg->LValue_Right;
    if(adValue_r-msg->LValue_Right>JT_threshold)//右踏板
    {
        if(Foot_BlockRunIfPressureStopLatched())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 泵驱动故障联动停机后右脚仍踩着，不能因蜂鸣结束而自动重启手柄和泵。 */
        }
        if(Foot_BlockIfSocketMissing())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏右侧缺刀具后保持踩下时，只保持停机，不再重复触发 80 弹窗。 */
        }
        ControlSignalMessage.jtR_control_flag=true;
       if(Foot_EnsureFootControlMode() == false)
       {
           Foot_ClearRunRequestAfterGateFail(); /* 右脚踏不满足启动条件，清掉右侧请求，避免松脚时误以为已启动。 */
           return FOOT_CONTROL_FLOW_RETURN_TASK;
       }

         // 上位机正在控制时，右脚踏不能启动。
        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
        {
            // 以下按键蜂鸣未启用，当前这里只取消右脚踏请求。
            //SendKeyBeepMessage(1);// 若启用会在每次检查时蜂鸣一次，不能直接用作一次性提示。
            Foot_ClearRunRequestAfterGateFail(); /* 外部控制中右脚踏不参与，清掉本周期脚踏请求。 */
            return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
        }

       /* 右脚轻踩只开泵，进入深踩区后才检查能否使用手柄电机。 */

       Foot_StartDoublePedalGentlyPump(true); /* 右脚按 B 优先规则启动并记录实际泵，使用屏幕设定速度且不进入10秒定时排空。 */
     if(adValue_r<msg->MValue_Right)adValue_r=msg->MValue_Right;
      if(adValue_r-msg->MValue_Right>JT_threshold)
        {
            if(WorkMessage.channel_work==CHANNEL_B)
            {
               /* 双踏板右侧触发当前通道启动前先占用脚踏控制权。 */
               if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
               {
                   Foot_LatchSocketMissing(); /* 右脚踏当前通道已触发缺刀具报警，等待松脚后才能再报警。 */
                   Foot_ClearRunRequestAfterGateFail(); /* 右脚启动前发现缺刀具，报警后清掉本次运行请求。 */
                   return FOOT_CONTROL_FLOW_RETURN_TASK;
               } /* 右踏板启动当前通道前检查公共接头刀具头是否已识别。 */
               if(Foot_DoublePedalIsWaitingRelease(CHANNEL_B))
               {
                   ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                   return FOOT_CONTROL_FLOW_RETURN_TASK;
               }
               if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
               {
                   Foot_ClearRunRequestAfterGateFail(); /* 右脚不能使用当前电机时，取消泵和电机运行请求。 */
                   return FOOT_CONTROL_FLOW_RETURN_TASK;
               }
               ControlSignalMessage.jtR_control_flag=true;
                if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_RIGHT) == false)
                {
                    Foot_ClearRunRequestAfterGateFail(); /* 右踏板异常恢复后仍处于运行段时保持停机，禁止旧行程直接重启。 */
                    return FOOT_CONTROL_FLOW_RETURN_TASK;
                }
                WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue_r,msg->MValue_Right,msg->HValue_Right,FOOT_PEDAL_SPEED_HIGH_MARGIN); /* 双脚踏右侧当前通道保留高位死区，并限制最大不超过设定速度。 */
                 WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_RIGHT); /* 记住联动泵由右脚启动，左脚松开不能撤销这次开泵请求。 */
            }
            else if(WorkMessage.channel_work==CHANNEL_A) /* 右脚对应 B 通道；当前在 A 时需判断能否切到 B。 */
            {
                    if(WorkMessage.Channel_Aonline==true) /* 当前 A 仍在线时，继续判断 B 是否可作为右脚目标通道。 */
                    {
                      if(WorkMessage.Channel_Bonline==true) /* A/B 都在线时按去抖次数切到 B，防止误触造成通道跳变。 */
                      {
                            WorkMessage.switchhandle_countss++;
                            if(WorkMessage.switchhandle_countss<FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS)
                            {
                                ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                return FOOT_CONTROL_FLOW_RETURN_TASK;//未达到切换去抖计数，不执行
                            }
                            WorkMessage.switchhandle_countss=0;
                            switch_before_channel = WorkMessage.channel_work; /* 记录切换前通道，后续用实际 channel_work 判断是否成功切到 B。 */
                            HandleSwitchActive(SCREENKey_HANDLE_B);//切换
                            if((switch_before_channel != CHANNEL_B) && (WorkMessage.channel_work == CHANNEL_B))
                            {
                                SendKeyBeepMessage(1U); /* 双脚踏右踏板跨通道切换 B 手柄成功，蜂鸣一次给操作者确认。 */
                            }
                            Foot_RequireDoublePedalRelease(CHANNEL_B);
                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                            return FOOT_CONTROL_FLOW_RETURN_TASK;
                        }
                        else
                        {
                            if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
                            {
                                Foot_LatchSocketMissing(); /* 右脚踏跨通道已触发缺刀具报警，长踩期间不再重复请求启动。 */
                                Foot_ClearRunRequestAfterGateFail(); /* 右脚踏跨通道直接运行前发现公共接头无刀具，立即撤销脚踏请求。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            } /* 双踏板右侧跨通道直接运行前也要检查公共接头刀具头，避免空刀具下发运行。 */
                            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                            {
                                Foot_ClearRunRequestAfterGateFail(); /* 右脚不能使用 A 通道电机时，清掉本次脚踏运行请求。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            }
                            ControlSignalMessage.jtR_control_flag=true;
                            if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_RIGHT) == false)
                            {
                                Foot_ClearRunRequestAfterGateFail(); /* 右踏板跨通道前仍需满足先松后踩，不能由旧高AD完成切换起机。 */
                                return FOOT_CONTROL_FLOW_RETURN_TASK;
                            }
                            WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue_r,msg->MValue_Right,msg->HValue_Right,0U); /* 双脚踏右侧跨通道运行同样按 EEPROM 最小速度起步。 */
                            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                            Foot_StartHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_RIGHT); /* 右脚控制 A 通道时仍记为右脚启动，松右脚时据此停联动泵。 */
                        }

                    }
            }
        }
        else
        {
            Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_RIGHT); /* 右脚退回轻踩区；电机若由右脚启动就要求停机。 */
             WorkMessage.switchhandle_countss=0;
            if(ControlSignalMessage.jtR_control_flag)
            {
                 WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                // 右脚退回轻踩区后取消电机请求，轻踩注水泵仍可继续运行。
                ControlSignalMessage.jtR_control_flag=false; /* 右踏板退回轻踩区时只清右侧电机来源，保持左右状态对称。 */

                if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 右脚退出电机区后检查报警；当前只清手柄未连接。
                {
                        Foot_ClearHandleOrOverloadAlarm();
                }
             }
        }
    }
    else
    {
        Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_RIGHT); /* 右侧完全松开只停止右侧实际启动的电机请求，不能误停左侧。 */
        Foot_StopHandleInjectionPumpFollow(FOOT_MOTOR_SOURCE_RIGHT); /* 只撤销右脚发出的联动开泵请求，不撤销左脚的请求。 */
        WorkMessage.switchhandle_countss=0;
        Foot_DoublePedalMarkReleased(CHANNEL_B);
        if(ControlSignalMessage.jtR_control_flag)
        {
           WorkMessage.runflag_work=false;
            WorkMessage.speed_work=0;
            // 右脚已完全松开，清电机运行记录；下面再按保存的泵通道停止注水。
            ControlSignalMessage.jtR_control_flag=false;

          if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)// 右脚完全松开后检查报警；当前只清手柄未连接。
           {
                Foot_ClearHandleOrOverloadAlarm();
           }
        }
        if(ControlSignalMessage.jtR_gentlypump_flag)
        {
            Foot_StopDoublePedalGentlyPump(true); /* 松右脚只释放右侧实际启动的泵；A/B 都是注水泵时准确停止 B。 */
        }
        if(Foot_IsPedalReleased(jtd_adcvalue_l, msg->LValue_Left))
        {
            Foot_ClearHandleOrOverloadAlarm(); /* 左右都松开后，即使运行标志早已清掉，也要关闭“手柄未连接”提示。 */
            Foot_ClearPressureLatchOnRelease(); /* 两侧都松开才清故障联动停机记录，避免另一侧一直踩着就自动重启。 */
            Foot_ClearManualAlarmOnRelease(); /* 双脚踏左右都释放后清除手控已选中的错模式报警。 */
            Foot_ClearSocketLatchOnRelease(); /* 两侧都已松开，结束缺刀具报警的等待松脚状态。 */
        }
     }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 右侧处理完毕，任务末尾检查电机空闲时能否取消脚踏占用。 */
}

/*
 * 函数功能：先处理左踏板，再处理右踏板；左侧要求退出时，不再处理右侧。
 * 输入参数：msg 指向当前保存的双踏板左右定标参数。
 * 返回参数：结束本周期或立即退出任务；只有左侧正常完成时才进入右侧。
 */
static FootControlFlow_t Foot_ProcessDoublePedal(const FootMessage_t *msg)
{
    FootControlFlow_t flow; /* 保存左侧处理结果，用于决定是否还能进入右侧。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无双踏板参数时保持停机并结束本周期。 */
    }

    if(Foot_IsPedalReleased(jtd_adcvalue_l, msg->MValue_Left) &&
       Foot_IsPedalReleased(jtd_adcvalue_r, msg->MValue_Right))
    {
        Foot_StopMotorAtReleaseBoundary(); /* 两侧都退出电机区后才允许下一次深踩启动；可以停在轻踩泵区，不必完全松到底。 */
    }

    flow = Foot_ProcessDoublePedalLeft(msg); /* 原大函数固定先处理左踏板。 */
    if (flow != FOOT_CONTROL_FLOW_CONTINUE)
    {
        return flow; /* 左侧要求结束本周期或立即退出时，跳过右侧处理。 */
    }

    return Foot_ProcessDoublePedalRight(msg); /* 左侧无提前结束时再处理右踏板。 */
}

/*
 * 函数功能：每 25ms 读取脚踏连接消息，并按踏板类型处理本次泵和电机动作。
 * 输入参数：event 为调度器事件值，当前任务不使用。
 * 返回参数：无。
 */
void FootControlTask(uint32_t event)
{
    static FootMessage_t msg; /* 保存最近一次脚踏连接状态和定标值，无新消息时继续使用。 */
    FootControlFlow_t flow = FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 默认处理后还要检查电机是否空闲，以便取消脚踏占用。 */

    (void)event;
    if (ControlArbitration_IsBusyByOther(CONTROL_OWNER_FOOT))
    {
        if ((FootMsgQueue != NULL) && (Kernel_QueueReceive(FootMsgQueue, &msg, 0) == pdTRUE))
        {
            Foot_HandleConnectionUpdate(&msg); /* 外控占用时仍更新脚踏在线状态和图标，但下方行程控制继续被直接跳过。 */
        }
        return; /* 连接状态更新后立即退出，脚踏不得在外控期间启动电机、泵或切换通道。 */
    }

    if ((FootMsgQueue != NULL) && (Kernel_QueueReceive(FootMsgQueue, &msg, 0) == pdTRUE))
    {
        Foot_HandleConnectionUpdate(&msg); /* 收到新连接消息才更新接入或掉线状态。 */
    }

    if(s_foot_runtime_frame_valid == false)
    {
        Foot_StopRunOnRealtimeTimeout(); /* 没有可信实时AD时持续保持脚踏输出为停止，防止旧高值在下一周期重新起机。 */
        return; /* 等到新的完整且 CRC 正确的实时帧后再处理踩踏，本周期不能按旧值调速。 */
    }

    if (Foot_BlockDriverAlarmRun(&msg))
    {
        return; /* 驱动故障后还没松脚，前面已要求停机，本周期不能再启动。 */
    }
    if (msg.connect_flag == true) /* 脚踏在线时才按踏板类型处理动作；离线消息只更新连接状态。 */
    {
        switch (msg.pedalType)
        {
            case 1U:
                flow = Foot_ProcessSinglePedal(&msg); /* 单踏板按低点到高点执行比例运行。 */
                break;
            case 2U:
                flow = Foot_ProcessTwoStagePedal(&msg); /* 双段踏板按轻踩泵、深踩电机执行。 */
                break;
            case 3U:
                flow = Foot_ProcessDoublePedal(&msg); /* 双踏板保持先左后右的处理顺序。 */
                break;
            default:
                break; /* 未知类型不修改泵、电机或通道状态。 */
        }
    }

    if (flow == FOOT_CONTROL_FLOW_RETURN_TASK)
    {
        return; /* 踏板处理要求立即退出，本周期不再执行下面的空闲检查。 */
    }

    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT); /* 电机已空闲时取消脚踏占用，使其他控制方式可以接着使用。 */
}

/**
 * @brief 脚踏板任务初始化
 */



typedef struct
{
    uint8_t connected;             /* 1 表示定标值检查通过，且已把接入消息发给脚踏控制任务。 */
    volatile uint8_t silent_ticks; /* 连续无完整帧的 10ms 周期数，超过 100 后确认掉线。 */
    uint8_t single_low_ready;      /* 单踏板低值已经读回，下一步等待高值。 */
    uint8_t single_high_requested; /* 实际在收到高点回包后置 1；置 1 后收到低点回包不再请求高点。 */
} FootParserState_t;

/* UART4 解析状态只由 10ms 解析任务维护，集中存放便于插拔时一次清理。 */
static FootParserState_t s_foot_parser_state = {0U};

/*
 * 函数功能：处理 UART4 本周期没有完整脚踏帧的掉线确认。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_HandleMissingUartFrame(void)
{
    FootMessage_t disconnect_msg; /* 先在局部变量中准备掉线消息，发送失败时保留原来的全局连接记录。 */

    if ((s_foot_parser_state.connected == 0U) &&
        (WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR) == false))
    {
        return; /* 从未上线且没有定标报警时不累计掉线，避免空串口周期产生无意义状态变化。 */
    }

    if(s_foot_parser_state.silent_ticks <= FOOT_OFFLINE_TIMEOUT_TICKS)
    {
        ++s_foot_parser_state.silent_ticks; /* 无有效实时帧每10ms累计一次，到101后饱和并持续重试掉线队列。 */
    }
    if((s_foot_parser_state.silent_ticks >= FOOT_RUNTIME_TIMEOUT_TICKS) &&
       (s_foot_runtime_frame_valid != false))
    {
        Foot_StopRunOnRealtimeTimeout(); /* 连续250ms没有可信实时AD时先停电机和联动泵，不等待1秒UI掉线。 */
    }
    if (s_foot_parser_state.silent_ticks <= FOOT_OFFLINE_TIMEOUT_TICKS)
    {
        return; /* 尚未超过掉线阈值时保留当前脚踏状态。 */
    }

    if (s_foot_parser_state.connected != 0U)
    {
        disconnect_msg = footmessage; /* 保留最近一次脚踏类型和定标值，只在候选消息中改连接状态。 */
        disconnect_msg.connect_flag = false; /* 行为任务收到该消息后停止脚踏控制并释放控制权。 */
        if(Foot_SendMessage(disconnect_msg) == false)
        {
            return; /* 队列暂满时保持connected和101计数，下个10ms周期继续发送同一掉线消息。 */
        }
        footmessage.connect_flag = false; /* 掉线消息发送成功后才更新全局连接状态，失败时仍可下次重试。 */
        SendKeyBeepMessage(1U); /* 脚踏真实掉线时保留一次按键蜂鸣提示。 */
    }

    s_foot_parser_state.silent_ticks = 0U; /* 掉线消息已发出，或只需结束定标报警；清零计数，供下次接入使用。 */
    s_foot_parser_state.connected = 0U; /* 回到未连接状态，下次插入必须重新校验定标值。 */
    s_foot_parser_state.single_low_ready = 0U; /* 清除单踏板低值读取阶段。 */
    s_foot_parser_state.single_high_requested = 0U; /* 清除单踏板高值读取阶段。 */
    Foot_ClearFootValueErrorAlarm(); /* 错误值脚踏拔出后清除 83 号报警。 */
}

/*
 * 函数功能：根据脚踏协议头和类型字段确定一帧完整数据的固定长度。
 * 输入参数：frame 指向 FE EF 帧头；remaining 为本次 DMA 接收数据中从此帧头开始的剩余字节数。
 * 返回参数：返回10、18或24字节；字段不足或协议不支持时返回0。
 */
static uint16_t Foot_GetFrameLength(const uint8_t *frame, uint16_t remaining)
{
    if((frame == NULL) || (remaining < 6U))
    {
        return 0U; /* 读取命令和类型字段前先保证最小长度，防止噪声尾部越界。 */
    }
    if(((frame[2] == 0xB6U) && (frame[3] == 0xC1U)) ||
       ((frame[2] == 0xD0U) && (frame[3] == 0xB4U)))
    {
        return FOOT_SINGLE_FRAME_LENGTH; /* 单踏板实时值及Flash读回均使用10字节完整CRC帧。 */
    }
    if((frame[2] != 0xBBU) || (frame[3] != 0xAAU))
    {
        return 0U; /* 不是支持的脚踏命令，不能把它算作脚踏仍在线的证据。 */
    }
    if((frame[4] == 0xDDU) && (frame[5] == 0x01U))
    {
        return FOOT_TWO_STAGE_FRAME_LENGTH; /* 双段周期帧固定携带一组三点定标值。 */
    }
    if((frame[4] == 0xDDU) && (frame[5] == 0x02U))
    {
        return FOOT_DOUBLE_FRAME_LENGTH; /* 双脚踏周期帧固定携带左右两组三点定标值。 */
    }
    if(frame[4] == 0xCCU)
    {
        return FOOT_SINGLE_FRAME_LENGTH; /* 按键帧为 10 字节，但收到按键不能代替实时 AD 数据，不能重置实时帧超时计数。 */
    }
    return 0U; /* 未定义功能码继续向后寻找下一组合法帧头。 */
}

/*
 * 函数功能：校验脚踏完整帧尾部的CRC16/MODBUS，阻止坏帧更新实时AD和在线计数。
 * 输入参数：frame 指向完整帧；frame_length 为包含末尾2字节CRC的总长度。
 * 返回参数：true 表示CRC正确；false 表示指针、长度或CRC不正确。
 */
static bool Foot_IsFrameCrcValid(uint8_t *frame, uint16_t frame_length)
{
    uint16_t received_crc; /* 保存脚踏板按高字节在前发送的CRC值。 */

    if((frame == NULL) || (frame_length < 2U))
    {
        return false; /* 无法同时容纳数据和CRC时禁止进入任何业务解析。 */
    }
    received_crc = ((uint16_t)frame[frame_length - 2U] << 8) |
                   frame[frame_length - 1U]; /* 组合脚踏板尾部CRC高低字节。 */
    return (Common_Crc16(frame, (uint16_t)(frame_length - 2U)) == received_crc); /* 只接受与脚踏板同一算法计算出的完整帧。 */
}

/*
 * 函数功能：处理单踏板实时值、低值和高值三类 UART4 帧。
 * 输入参数：frame 指向 FE EF 帧头；remaining 为从帧头开始的剩余字节数。
 * 返回参数：true 表示发现无效定标值并要求本周期立即退出；false 表示继续扫描后续数据。
 */
static bool Foot_ParseSinglePedalFrame(const uint8_t *frame, uint16_t remaining)
{
    /* 长度足够且命令头为 B6 C1 或 D0 B4 时按单踏板协议解析，避免误读其它帧。 */
    if ((remaining >= 8U) &&
        (frame[2] == 0xB6U) && (frame[3] == 0xC1U) &&
        (frame[4] == 0x01U) && (frame[5] == 0x01U))
    {
        jt_adcvalue = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存单踏板当前 AD 值供 25ms 行为任务换算速度。 */
        if ((s_foot_parser_state.connected == 0U) &&
            (s_foot_parser_state.single_low_ready == 0U))
        {
            Uart4_SendPacket(get_jtLvalue, 8U); /* 单踏板尚未完成识别且未读到低点时，请求板上保存的低点值。 */
        }
        return false;
    }

    if ((remaining >= 8U) &&
        (frame[2] == 0xD0U) && (frame[3] == 0xB4U) &&
        (frame[4] == 0xB5U) && (frame[5] == 0xCDU))
    {
        s_foot_parser_state.single_low_ready = 1U; /* 标记低值已经读回。 */
        footmessage.LValue_Left = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存单踏板低位定标值。 */
        if (s_foot_parser_state.single_high_requested == 0U)
        {
            Uart4_SendPacket(get_jtHvalue, 8U); /* 已收到低点但还没收到高点时，请求板上保存的高点值。 */
        }
        return false;
    }

    if ((remaining >= 8U) &&
        (frame[2] == 0xD0U) && (frame[3] == 0xB4U) &&
        (frame[4] == 0xB8U) && (frame[5] == 0xDFU))
    {
        s_foot_parser_state.single_high_requested = 1U; /* 标记高值已经读回。 */
        footmessage.HValue_Left = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存单踏板高位定标值。 */
        if (Foot_IsPedalTwoPointStorageValid(footmessage.LValue_Left,
                                             footmessage.HValue_Left) == false)
        {
            s_foot_parser_state.single_low_ready = 0U; /* 无效定标值要求下次重新读取完整低/高值。 */
            s_foot_parser_state.single_high_requested = 0U;
            Foot_ReportFootValueErrorAlarm(); /* 显示 83 号报警并阻止脚踏控制。 */
            return true;
        }

        Foot_ClearFootValueErrorAlarm(); /* 新定标值有效时清除历史 83 号报警。 */
        footmessage.connect_flag = true;
        footmessage.pedalType = 1U; /* 行为任务按单踏板比例运行流程处理。 */
        if(Foot_SendMessage(footmessage) != false)
        {
            s_foot_parser_state.connected = 1U; /* 上线消息成功排队后才进入在线态，行为任务不会遗漏脚踏类型和定标值。 */
            SendKeyBeepMessage(1U); /* 脚踏真实上线且行为队列已接收后提示一次。 */
        }
        else
        {
            s_foot_parser_state.single_low_ready = 0U; /* 队列满时重新发起低/高值读取，让后续周期可以重试上线消息。 */
            s_foot_parser_state.single_high_requested = 0U; /* 解除高值已请求状态，避免单踏板卡在未上线但不再请求的阶段。 */
        }
    }

    return false;
}

/*
 * 函数功能：把双段/双脚踏按键码转换成现有业务按键消息。
 * 输入参数：key_code 为 UART4 帧中的脚踏按键编号。
 * 返回参数：无。
 */
static void Foot_DispatchPedalKey(uint8_t key_code)
{
    switch (key_code)
    {
        case 0x01U:
            SendKeyBehMessage(1U, JTKey_left_long); /* 左键长按。 */
            break;
        case 0x02U:
            SendKeyBehMessage(1U, JTKey_right_long); /* 右键长按。 */
            break;
        case 0x03U:
            SendKeyBehMessage(1U, JTKey_middle_long); /* 中键长按。 */
            break;
        case 0x04U:
            SendKeyBehMessage(1U, JTKey_right_short); /* 右键短按。 */
            break;
        case 0x05U:
            SendKeyBehMessage(1U, JTkey_left_short); /* 左键短按。 */
            break;
        case 0x06U:
            SendKeyBehMessage(1U, JTKey_middle_short); /* 中键短按。 */
            break;
        default:
            return; /* 不认识的按键码不交给按键任务，也不发蜂鸣提示。 */
    }

    SendKeyBeepMessage(1U); /* 合法脚踏按键保持一次按键蜂鸣。 */
}

/*
 * 函数功能：处理双段踏板和双脚踏的实时值、定标值及实体按键。
 * 输入参数：frame 指向 FE EF BB AA 帧；remaining 为从帧头开始的剩余字节数。
 * 返回参数：true 表示结束本次扫描；false 表示继续扫描本批数据中的后续帧。
 */
static bool Foot_ParseMultiPedalFrame(const uint8_t *frame, uint16_t remaining)
{
    if ((remaining >= 10U) && (frame[4] == 0xDDU) && (frame[5] == 0x01U))
    {
        jtb_adcvalue = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存双段踏板当前 AD 值。 */
        if (s_foot_parser_state.connected != 0U)
        {
            return true; /* 双段踏板已识别完成，更新实时 AD 后结束本次扫描，不再读取定标值。 */
        }
        if (remaining < 16U)
        {
            return false; /* 定标字段尚未收完整时等待下一包，不访问越界数据。 */
        }

        footmessage.HValue_Left = ((uint16_t)frame[10] << 8) | frame[11];
        footmessage.MValue_Left = ((uint16_t)frame[12] << 8) | frame[13];
        footmessage.LValue_Left = ((uint16_t)frame[14] << 8) | frame[15];
        if (Foot_IsPedalThreePointStorageValid(footmessage.LValue_Left,
                                               footmessage.MValue_Left,
                                               footmessage.HValue_Left) == false)
        {
            Foot_ReportFootValueErrorAlarm(); /* 双段踏板任一校准点无效时显示 83 号报警。 */
            return true;
        }

        Foot_ClearFootValueErrorAlarm(); /* 三点定标恢复有效后释放报警。 */
        footmessage.connect_flag = true;
        footmessage.pedalType = 2U; /* 行为任务按轻踩泵、深踩电机流程处理。 */
        if(Foot_SendMessage(footmessage) != false)
        {
            SendKeyBeepMessage(1U); /* 双段上线消息成功排队后才提示，避免队列失败产生伪上线蜂鸣。 */
            s_foot_parser_state.connected = 1U; /* 消息发送成功后才记为在线；发送失败则仍为未连接，下帧继续重试。 */
        }
        return false;
    }

    if ((remaining >= 10U) && (frame[4] == 0xDDU) && (frame[5] == 0x02U))
    {
        jtd_adcvalue_l = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存双脚踏左侧 AD 值。 */
        jtd_adcvalue_r = ((uint16_t)frame[8] << 8) | frame[9]; /* 保存双脚踏右侧 AD 值。 */
        if (s_foot_parser_state.connected != 0U)
        {
            return false; /* 双脚踏在线后继续扫描同一 DMA 包中的其它帧，保持原行为。 */
        }
        if (remaining < 22U)
        {
            return true; /* 双脚踏尚未识别完成且定标字节不足，结束本次扫描，不读超出范围的数据。 */
        }

        footmessage.HValue_Left = ((uint16_t)frame[10] << 8) | frame[11];
        footmessage.MValue_Left = ((uint16_t)frame[12] << 8) | frame[13];
        footmessage.LValue_Left = ((uint16_t)frame[14] << 8) | frame[15];
        footmessage.HValue_Right = ((uint16_t)frame[16] << 8) | frame[17];
        footmessage.MValue_Right = ((uint16_t)frame[18] << 8) | frame[19];
        footmessage.LValue_Right = ((uint16_t)frame[20] << 8) | frame[21];
        /* 双脚踏左右任一路三点标定无效都禁止上线，避免异常区间参与电机速度换算。 */
        if ((Foot_IsPedalThreePointStorageValid(footmessage.LValue_Left,
                                                footmessage.MValue_Left,
                                                footmessage.HValue_Left) == false) ||
            (Foot_IsPedalThreePointStorageValid(footmessage.LValue_Right,
                                                footmessage.MValue_Right,
                                                footmessage.HValue_Right) == false))
        {
            Foot_ReportFootValueErrorAlarm(); /* 左右任一路定标无效都禁止双脚踏上线。 */
            return true;
        }

        Foot_ClearFootValueErrorAlarm(); /* 左右定标均有效后释放历史报警。 */
        footmessage.connect_flag = true;
        footmessage.pedalType = 3U; /* 行为任务按双脚踏左右独立流程处理。 */
        if(Foot_SendMessage(footmessage) != false)
        {
            s_foot_parser_state.connected = 1U; /* 接入消息和完整定标值已放入队列，才记为双脚踏在线。 */
            SendKeyBeepMessage(1U); /* 成功上线后保留一次连接提示。 */
        }
        return false;
    }

    /* 剩余长度足够且功能码为 0xCC 时才读取第 7 字节按键码，避免短帧越界。 */
    if ((remaining >= 8U) && (frame[4] == 0xCCU))
    {
        Foot_DispatchPedalKey(frame[7]); /* 按键帧只读取既有第 7 字节业务码。 */
    }

    return false;
}

/*
 * 函数功能：解析一个已经找到 FE EF 帧头的脚踏数据片段。
 * 输入参数：frame 指向帧头；remaining 为当前 DMA 数据中从帧头开始的剩余长度。
 * 返回参数：true 表示调用方应立即结束本周期；false 表示继续寻找后续帧。
 */
static bool Foot_ParseUartFrame(const uint8_t *frame, uint16_t remaining)
{
    /* 调用方已检查长度和 CRC；这里只按命令解析字段，是否重置实时帧超时计数由调用方判断。 */
    if ((remaining >= 8U) &&
        (((frame[2] == 0xB6U) && (frame[3] == 0xC1U)) ||
         ((frame[2] == 0xD0U) && (frame[3] == 0xB4U))))
    {
        return Foot_ParseSinglePedalFrame(frame, remaining);
    }

    /* 命令头为 BB AA 时进入双段/双脚踏协议解析，6 字节是读取类型字段的最小长度。 */
    if ((remaining >= 6U) && (frame[2] == 0xBBU) && (frame[3] == 0xAAU))
    {
        return Foot_ParseMultiPedalFrame(frame, remaining);
    }

    return false; /* 未识别帧保持静默，不改变脚踏业务状态。 */
}

/*
 * 函数功能：每 10ms 读取 UART4 DMA 数据，解析脚踏上线、掉线、定标值、实时值和按键。
 * 输入参数：event 为调度器传入的任务事件，本函数当前不使用。
 * 返回参数：无。
 */
void Foot_ParseDataS(uint32_t event)
{
    uint16_t offset = 0U; /* 当前候选帧头在 DMA 数据中的偏移。 */
    uint16_t received_len; /* 本周期从 UART4 DMA 取得的字节数。 */
    uint16_t frame_length; /* 当前协议头对应的完整帧长度，只允许10、18或24。 */
    bool realtime_frame_valid = false; /* 本次收到至少一帧完整且 CRC 正确的实时 AD 时置 true，用来清除超时计数。 */
    bool stop_scan = false; /* 解析函数要求提前结束时置 true；结束后仍需更新实时帧超时状态。 */
    uint8_t data[255] = {0U}; /* UART4 单周期接收缓存，容量保持原 255 字节。 */

    (void)event;
    received_len = Uart4_DMARecvDataPeek(data); /* 取出本次 UART4 DMA 接收的数据，供下面逐帧检查。 */
    while((uint16_t)(offset + 1U) < received_len)
    {
        if ((data[offset] != 0xFEU) || (data[offset + 1U] != 0xEFU))
        {
            ++offset; /* 当前字节不是帧头时只前移一字节，允许从噪声后恢复。 */
            continue; /* 跳过噪声字节，继续寻找下一个 FE EF 帧头。 */
        }

        frame_length = Foot_GetFrameLength(&data[offset], (uint16_t)(received_len - offset)); /* 按已识别协议选择完整帧长度。 */
        if(frame_length == 0U)
        {
            ++offset; /* 当前 FE EF 后不是支持的命令，向后一字节继续找，不能漏掉后面的合法帧。 */
            continue;
        }
        if((uint16_t)(received_len - offset) < frame_length)
        {
            break; /* DMA尾部只有半帧时不读取字段；本周期按无有效实时帧推进安全计时。 */
        }
        if(Foot_IsFrameCrcValid(&data[offset], frame_length) == false)
        {
            ++offset; /* CRC 错误时向后一字节继续找帧头，后面仍可能有正确的数据帧。 */
            continue;
        }

        if(((data[offset + 2U] == 0xB6U) &&
            (data[offset + 3U] == 0xC1U) &&
            (data[offset + 4U] == 0x01U) &&
            (data[offset + 5U] == 0x01U)) ||
           ((data[offset + 2U] == 0xBBU) &&
            (data[offset + 3U] == 0xAAU) &&
            (data[offset + 4U] == 0xDDU) &&
            ((data[offset + 5U] == 0x01U) || (data[offset + 5U] == 0x02U))))
        {
            realtime_frame_valid = true; /* 只有这三类帧带有当前踩踏 AD，读取定标值或按键回包不能重置实时帧超时计数。 */
        }

        stop_scan = Foot_ParseUartFrame(&data[offset], frame_length); /* 完整且CRC正确后才允许更新AD、定标值或按键事件。 */
        offset = (uint16_t)(offset + frame_length); /* 当前帧已处理，跳到帧尾之后，避免把帧内数据当成另一个帧头。 */
        if(stop_scan != false)
        {
            break; /* 本次扫描结束，仍要执行下面的实时帧有效标志和超时计数更新。 */
        }
    }

    if(realtime_frame_valid != false)
    {
        s_foot_parser_state.silent_ticks = 0U; /* 完整CRC实时帧到达后重新开始250ms安全和1秒掉线计时。 */
        s_foot_runtime_frame_valid = true; /* 行为任务下一周期可以使用本帧刚更新的实时AD。 */
    }
    else
    {
        Foot_HandleMissingUartFrame(); /* 空数据、噪声、截断、CRC错误或只有按键/读回帧都按无实时帧累计。 */
    }
}

/*
 * 函数功能：创建脚踏消息队列，并启动串口解析任务和脚踏控制任务。
 * 输入参数：无。
 * 返回参数：无。
 */
void SscFootControlTask_Init(void)
{
    //初始化消息队列
    Foot_Queue_Init();
     //创建任务
    Kernel_TaskCreate(&FOOTTaskHandle, Foot_ParseDataS);
    Kernel_TaskStart(&FOOTTaskHandle, KERNEL_TASK_ALWAYS, 10);// 每 10ms 处理接收数据；修改周期时需重新计算实时帧超时和掉线时间。

    Kernel_TaskCreate(&FOOTBHHandle, FootControlTask);
    Kernel_TaskStart(&FOOTBHHandle, KERNEL_TASK_ALWAYS, 25); /* 每 25ms 处理踩踏动作；修改周期会影响松脚确认和双脚踏切通道等待时间。 */
}
