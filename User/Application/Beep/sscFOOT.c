/**
 ******************************************************************************
 * @file    sscFOOT.c
 * @brief   脚踏板数据解析驱动
 *          用于解析脚踏板传感器传来的串口数据
 ******************************************************************************
 * @note
 * 通讯协议格式：
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



#define JT_threshold  20U
/* 单踏板松脚去抖周期数；FootControlTask 为 25ms，3 个周期约 75ms，用于过滤踩住时 AD 瞬时跌落导致的泵反复启停。 */
#define FOOT_SINGLE_PEDAL_RELEASE_DEBOUNCE_TICKS 3U
#define FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS 10U
#define FOOT_PEDAL_SPEED_HIGH_MARGIN 30U /* 双脚踏当前通道运行段预留的高位死区，沿用旧公式 HValue-MValue-30 的行程范围。 */
#define FOOT_SINGLE_FRAME_LENGTH 10U /* 单踏板实时值、Flash读回和脚踏实体按键帧均为10字节，最后2字节为CRC。 */
#define FOOT_TWO_STAGE_FRAME_LENGTH 18U /* 双段踏板周期帧为18字节，包含实时AD、三点定标值和CRC。 */
#define FOOT_DOUBLE_FRAME_LENGTH 24U /* 双脚踏周期帧为24字节，包含左右实时AD、两组三点定标值和CRC。 */
#define FOOT_RUNTIME_TIMEOUT_TICKS 25U /* UART4解析任务每10ms执行一次，连续250ms无有效实时帧先安全停止脚踏输出。 */
#define FOOT_OFFLINE_TIMEOUT_TICKS 100U /* 连续约1秒无有效实时帧后再发布脚踏掉线，保持原界面防抖时间。 */
#define FOOT_MOTOR_SOURCE_NONE 0U /* 当前没有踏板持有手柄电机运行授权。 */
#define FOOT_MOTOR_SOURCE_LEFT 1U /* 单踏板、双段踏板或双脚踏左侧持有运行授权。 */
#define FOOT_MOTOR_SOURCE_RIGHT 2U /* 双脚踏右侧持有运行授权。 */

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
static volatile bool s_foot_runtime_frame_valid = false; /* 只有完整且CRC正确的实时帧才能置位，失效时行为任务禁止沿用旧高AD。 */
static volatile bool s_foot_motor_stop_latched = true; /* 上电默认禁止脚踏起机，必须先观察到真实松脚再允许新的踩下周期。 */
static bool s_foot_motor_release_ready = false; /* 记录停机锁存后是否已经观察到松脚，只有该条件成立才能清锁存。 */
static uint8_t s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 记录真正启动电机的踏板侧，未踩的另一侧释放不能误停当前运行。 */

/*
 * 函数功能：锁住脚踏手柄电机停机，并要求后续先观察到真实松脚才能再次起机。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_LatchMotorStopUntilRelease(void)
{
    s_foot_motor_stop_latched = true; /* 先发布停机锁存，50ms手柄命令任务下一周期必须改发零速帧。 */
    s_foot_motor_release_ready = false; /* 通信失效或门禁失败不能把持续踩踏当成新的启动沿。 */
    s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 停机锁存生效后撤销旧踏板所有权，恢复必须重新授权。 */
}

/*
 * 函数功能：记录脚踏当前已经回到非电机运行区，并维持手柄电机停机锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_MarkMotorReleased(void)
{
    s_foot_motor_stop_latched = true; /* 原始行程回落时立即锁住停机，不等待普通运行标志的后续去抖。 */
    s_foot_motor_release_ready = true; /* 本次松脚已经满足重新武装条件，下一次有效深踩才可起机。 */
    s_foot_motor_active_source = FOOT_MOTOR_SOURCE_NONE; /* 松脚结束当前电机踏板所有权，下一次踩下重新登记来源。 */
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
 * 函数功能：双脚踏某一侧退出电机区时，仅在该侧确实持有运行授权时锁存停止，避免未踩侧误停另一侧。
 * 输入参数：source 为 FOOT_MOTOR_SOURCE_LEFT 或 FOOT_MOTOR_SOURCE_RIGHT。
 * 返回参数：无。
 */
static void Foot_StopDoubleMotorAtReleaseBoundary(uint8_t source)
{
    if(s_foot_motor_active_source != source)
    {
        return; /* 当前运行由另一侧持有或本来已停机，本侧释放不能改写全局手柄运行状态。 */
    }

    Foot_LatchMotorStopUntilRelease(); /* 活动侧退出电机区后先保持停止，双侧都退出电机区才允许重新武装。 */
    if((WorkMessage.drivetype_work==JTWORK) || ControlArbitration_IsOwner(CONTROL_OWNER_FOOT))
    {
        WorkMessage.runflag_work=false; /* 只撤销实际活动踏板产生的电机请求，保证50ms任务发送STOP。 */
        WorkMessage.speed_work=0U;      /* 清实际目标速度，另一侧若要运行必须先完成双侧释放再重新踩下。 */
    }
}

/*
 * 函数功能：在脚踏准备写入运行标志前验证“先松开、后重新踩下”的完整动作周期。
 * 输入参数：无。
 * 返回参数：true表示允许本次及本次持续踩踏运行；false表示仍须保持停机。
 */
static bool Foot_TryAuthorizeMotorRun(uint8_t source)
{
    if((source != FOOT_MOTOR_SOURCE_LEFT) && (source != FOOT_MOTOR_SOURCE_RIGHT))
    {
        return false; /* 未定义踏板来源不能取得手柄运行授权。 */
    }
    if(s_foot_motor_stop_latched == false)
    {
        return (s_foot_motor_active_source == source); /* 同一次持续踩踏只允许原授权侧更新速度，另一侧不能无释放切换。 */
    }
    if(s_foot_motor_release_ready == false)
    {
        return false; /* 上电、通信失效或门禁失败后尚未松脚，禁止旧高AD直接恢复运行。 */
    }

    s_foot_motor_release_ready = false; /* 消耗本次松脚授权，下一轮起机必须再次经历真实释放。 */
    s_foot_motor_stop_latched = false; /* 最后解除锁存，使手柄驱动任务可以接受本次新的运行请求。 */
    s_foot_motor_active_source = source; /* 保存实际启动侧，双脚踏未活动一侧的释放分支不得误停本侧。 */
    return true;
}

/*
 * 函数功能：查询脚踏手柄电机是否处于独立停机锁存状态。
 * 输入参数：无。
 * 返回参数：锁存有效返回1；当前踩踏周期已经授权运行返回0。
 */
uint8_t Foot_IsMotorStopLatched(void)
{
    return (s_foot_motor_stop_latched != false) ? 1U : 0U; /* 只向电机发送任务发布单字节快照，不暴露内部重启条件。 */
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

    uint16_t LValue_Right;   //脚踏板AD值-左踏板低值
    uint16_t MValue_Right;   //脚踏板AD值-左踏板中值
    uint16_t HValue_Right;   //脚踏板AD值-左踏板高值


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
 * 输入参数：ad_value 为当前脚踏 ADC；low_adc 为运行段起点；high_adc 为运行段终点；high_margin 为高位死区。
 * 返回参数：钳位后的实际运行速度，范围为 EEPROM 最小速度到当前屏幕/EEPROM 设定速度。
 */
static uint32_t Foot_BuildTravelMotorSpeed(uint16_t ad_value,uint16_t low_adc,uint16_t high_adc,uint16_t high_margin)
{
    uint16_t effective_high_adc=high_adc; /* 运行段实际高点，双脚踏部分分支需要扣除旧逻辑保留的 30 点死区。 */
    uint16_t clamped_adc=ad_value; /* 钳位后的 ADC，避免猛踩或采样过冲让比例超过 100%。 */
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
        effective_high_adc=(uint16_t)(effective_high_adc-high_margin); /* 仅在高点足够大时扣除死区，防止无符号下溢。 */
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
    speed_range=max_speed-min_speed; /* 最小速度已经钳到不大于最大速度，此处不会下溢。 */
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
 * 函数功能：把脚踏上线或掉线消息可靠投递到行为队列，并只对已经成功入队的相同消息去重。
 * 输入参数：msg 为本次连接状态、脚踏类型和定标参数快照。
 * 返回参数：true 表示消息已经成功排队或与最近一次成功消息相同；false 表示队列不可用或本次发送失败。
 */
static bool Foot_SendMessage(FootMessage_t msg)
{
    static FootMessage_t last_queued_msg; /* 保存最近一次已经成功进入队列的消息，失败消息不能成为去重基准。 */
    static bool last_queued_valid = false; /* 上电后尚无成功消息时不比较全零静态缓存，避免误判首次消息。 */

    if(FootMsgQueue == NULL)
    {
        return false; /* 脚踏队列尚未创建时不能投递连接状态，调用方必须保留重试条件。 */
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

    memcpy(&last_queued_msg, &msg, sizeof(FootMessage_t)); /* 只有发送成功后才记住本次连接快照。 */
    last_queued_valid = true; /* 标记去重基准已经有效，后续相同消息可以安全过滤。 */
    return true;
}

/**
 * @brief 初始化脚踏板消息队列
 */
static void Foot_Queue_Init(void)
{
    FootMsgQueue = Kernel_QueueCreate(5, sizeof(FootMessage_t), "FootMsgQueue");
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
    /* 记录本次脚踏正在联动轻排泵，松开脚踏时才会进入对应停泵分支。 */
    if(right_pedal)
    {
        ControlSignalMessage.jtR_gentlypump_flag=true; /* 右脚启动 A 泵时标记右侧，松右脚才能结束本次请求。 */
    }
    else
    {
        ControlSignalMessage.jtL_gentlypump_flag=true; /* 左脚启动 A 泵时保持原左侧轻排标记。 */
    }
    /* 双脚踏轻踩只记录控制来源，泵任务仍读取 speed_work，不进入屏幕固定速度排空。 */
    pumpMessageA.pedalDrainage_flag=pedal_drainage;
    if(pedal_drainage==false)
    {
        pumpMessageA.speed_work=speed_work; /* 普通两段脚踏联动仍按原规则更新设定速度。 */
    }
    /* run_flag 是 sscPUMPA 任务真正允许输出非零速度的门控，脚踏启动必须置位。 */
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
    /* 记录脚踏轻排已经启动，松开脚踏时按同一标志关闭联动泵。 */
    if(right_pedal)
    {
        ControlSignalMessage.jtR_gentlypump_flag=true; /* 右脚启动 B 泵时必须置右侧标记，修正松右脚无法停泵。 */
    }
    else
    {
        ControlSignalMessage.jtL_gentlypump_flag=true; /* 左脚启动 B 泵时保持原左侧轻排标记。 */
    }
    /* 双脚踏轻踩只记录控制来源，实际输出继续使用屏幕保存的 speed_work。 */
    pumpMessageB.pedalDrainage_flag=pedal_drainage;
    if(pedal_drainage==false)
    {
        pumpMessageB.speed_work=speed_work; /* 普通两段脚踏联动继续保存本次要求的速度。 */
    }
    /* run_flag 是 sscPUMPB 任务真正允许输出非零速度的门控，脚踏启动必须置位。 */
    pumpMessageB.run_flag=true;
    if(pedal_drainage==false)
    {
        SendPumpBMessage(INJECTWATER,pumpMessageB.speed_work); /* 普通联动继续通过旧队列同步速度。 */
    }
}

static void Foot_StopPumpAInjection(void)
{
    /* 松开脚踏后关闭 A 泵运行门控，sscPUMPA 下一周期会按 run_flag=false 下发 0 速。 */
    pumpMessageA.run_flag=false;
    pumpMessageA.pedalDrainage_flag=false; /* 松脚结束 A 泵脚踏轻踩来源，后续屏幕排空不受旧状态影响。 */
    /* 脚踏停泵不进入排空模式，必须同步清除排空标志。 */
    pumpMessageA.timingDrainage_flag=false;
    /* 排空计数清零，避免下一次排空或脚踏启动继承旧计数。 */
    pumpMessageA.timingDrainage_times=0U;
}

static void Foot_StopPumpBInjection(void)
{
    /* 松开脚踏后关闭 B 泵运行门控，sscPUMPB 下一周期会按 run_flag=false 下发 0 速。 */
    pumpMessageB.run_flag=false;
    pumpMessageB.pedalDrainage_flag=false; /* 松脚结束 B 泵脚踏轻踩来源，不能把该状态带到下一次联动。 */
    /* 脚踏停泵不进入排空模式，必须同步清除排空标志。 */
    pumpMessageB.timingDrainage_flag=false;
    /* 排空计数清零，避免下一次排空或脚踏启动继承旧计数。 */
    pumpMessageB.timingDrainage_times=0U;
}

/*
 * 函数功能：脚踏启动门禁失败后撤销本周期已经置位的脚踏运行请求。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearRunRequestAfterGateFail(void)
{
    Foot_LatchMotorStopUntilRelease();              /* 门禁失败发生在踩踏期间，必须先松脚再允许下一次手柄起机。 */
    WorkMessage.runflag_work=false;                 /* 门禁失败时禁止驱动任务继续看到运行命令，公共接头缺刀具不能下发启动帧。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免屏幕或驱动继续沿用本周期脚踏比例速度。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 清左脚踏运行标志，避免 gate 失败后释放分支误认为左脚已经启动。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行标志，双脚踏任一侧失败都不能留下运行来源。 */
    s_double_left_gently_pump_channel=CHANNEL_NONE; /* 门禁失败会停掉两台泵，左侧实际泵记录必须同步失效。 */
    s_double_right_gently_pump_channel=CHANNEL_NONE; /* 清右侧实际泵记录，避免松脚时误停后续其它来源启动的泵。 */
    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT); /* 如果本周期已临时占用脚踏 owner，电机未运行时立即释放给其它控制源。 */
    if(ControlSignalMessage.jtL_gentlypump_flag||ControlSignalMessage.jtR_gentlypump_flag)
    {
        ControlSignalMessage.jtL_gentlypump_flag=false; /* 清脚踏左侧轻排标志，防止缺刀具报警后注水泵继续按脚踏保持。 */
        ControlSignalMessage.jtR_gentlypump_flag=false; /* 清脚踏右侧轻排标志，保证双脚踏两侧失败行为一致。 */
        Foot_StopPumpAInjection();                      /* A 注水泵若已被轻踩阶段打开，门禁失败时必须立即关闭。 */
        Foot_StopPumpBInjection();                      /* B 注水泵若已被轻踩阶段打开，也必须和手柄运行一起撤销。 */
    }
}

/*
 * 函数功能：实时脚踏帧超时后保持脚踏来源安全停机，防止25ms行为任务继续使用旧高AD重复置运行位。
 * 输入参数：无，函数读取当前控制所有权和脚踏运行标志，只处理脚踏来源。
 * 返回参数：无。
 */
static void Foot_StopRunOnRealtimeTimeout(void)
{
    bool foot_output_active; /* 记录当前电机或轻踩泵是否确实由脚踏来源占用，避免误停其它控制方式。 */

    s_foot_runtime_frame_valid = false; /* 超时后先关闭实时数据有效门禁，只有下一帧完整CRC数据才能恢复。 */
    Foot_LatchMotorStopUntilRelease(); /* 实时AD失效后即使恢复为旧高值也不能自动起机，必须先收到真实松脚。 */
    MotorUart_ReleaseFootDriverAlarm(); /* 连续无可信脚踏数据等价于控制源已释放，记录该条件并继续等待驱动Err=0。 */
    foot_output_active = ControlArbitration_IsOwner(CONTROL_OWNER_FOOT) ||
                         ((WorkMessage.drivetype_work == JTWORK) &&
                          (ControlSignalMessage.jtL_control_flag ||
                           ControlSignalMessage.jtR_control_flag ||
                           ControlSignalMessage.jtL_gentlypump_flag ||
                           ControlSignalMessage.jtR_gentlypump_flag)); /* 仅脚踏owner或脚控模式的脚踏标志允许触发本次安全停机。 */
    if(foot_output_active == false)
    {
        return; /* 手控、触控或外控正在运行时，脚踏通信异常不能清除其它来源的运行请求。 */
    }

    Pubinterface_SetHandleInjectionPumpRun(false); /* 脚踏电机失去可信实时输入时同步关闭手柄联动注水泵。 */
    Foot_ClearRunRequestAfterGateFail(); /* 清电机速度、运行位、左右脚踏标志和轻踩泵，下一驱动周期发送停止帧。 */
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
 * 函数功能：公共接头缺刀具等待松脚锁存有效时拦截脚踏保持踩下造成的重复报警。
 * 输入参数：无。
 * 返回参数：true 表示本周期已被等待松脚锁存拦截；false 表示允许继续判断脚踏启动。
 */
static bool Foot_BlockIfSocketMissing(void)
{
    if(s_common_socket_missing_wait_release==false)
    {
        return false;                                /* 没有缺刀具等待松脚锁存时，脚踏按原有启动流程继续。 */
    }

    Foot_ClearRunRequestAfterGateFail();             /* 锁存期间保持电机和联动泵停止，避免长踩脚踏到期后又写入运行请求。 */
    return true;                                     /* 通知调用处退出本周期启动分支，必须等待真实松脚。 */
}

/*
 * 函数功能：脚踏确认释放后解除公共接头缺刀具等待松脚锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearSocketLatchOnRelease(void)
{
    if(s_common_socket_missing_wait_release==false)
    {
        return;                                      /* 当前没有缺刀具等待松脚锁存时不改状态，避免影响其它门禁失败。 */
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
        return false; /* 轻排段低点到中点无有效行程时，脚踏不能安全控制泵。 */
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
    WorkAlarm_Set(WORK_ALARM_FOOT_VALUE_ERROR);     /* 写入全局报警锁存，Foot_EnsureFootControlMode 会据此禁止泵和手柄运行。 */
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
        return; /* 脚踏保持踩下期间不重复投递同一个 81 号报警，避免屏幕和蜂鸣队列堆积。 */
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

    WorkAlarm_Clear();                 /* 新收到的脚踏存储值已经有效，或错误脚踏已拔出，释放脚踏值错误锁存。 */
    SendAlarmMessage(WORK_ALARM_NONE); /* 停止本模块触发的报警蜂鸣。 */
    SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 关闭 83 号报警弹窗，后续由脚踏接入刷新控制状态。 */
}

static void Foot_ClearHandleOrOverloadAlarm(void)
{
    /* 脚踏释放动作只清手柄未连接/电机过载这两类可恢复提示，不能误清 UID、通讯、HALL 等故障。 */
    if(WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED))
    {
           WorkAlarm_Clear();                 /* 清掉统一报警状态，让后续控制方式可以重新响应。 */
          SendAlarmMessage(WORK_ALARM_NONE); /* 释放脚踏或拔掉脚踏时同步停止报警蜂鸣。 */
          SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 同步关闭屏幕报警弹窗，避免“手柄未连接/过载”残留。 */
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

    /* 外部控制仍有效时不能切入脚踏，必须等上位机退出并释放互斥控制权。 */
    if(ControlArbitration_IsExternalActive() || ControlSignalMessage.HMI_control_flag || (WorkMessage.touchactive_work==TOUCHWORK))
    {
        return false;
    }

    /* 其它本地方式正在持有控制权时不抢占，必须等待当前来源真正停稳后再响应脚踏。 */
    if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_FOOT))
    {
        return false;
    }

    if(WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR))
    {
        return false; /* 脚踏存储值错误属于输入安全故障，未恢复有效值前禁止控制泵和手柄。 */
    }

    /* 历史脚踏分支会把“当前不是脚踏模式”误报成 0x02；现在接管前先清掉这个旧误报。 */
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
 * 返回参数：true 表示本周期仍按抖动处理并保持泵/电机运行；false 表示已经确认松脚，可以进入停泵分支。
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
 * 函数功能：脚踏已经确认松开后，结束本次压力堵塞造成的手柄停机锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_ClearPressureLatchOnRelease(void)
{
    Pubinterface_ClearPressureBlockStopLatchForNewTrigger(); /* 脚踏松开只解除压力停机锁存，不走停泵接口，避免误清手柄联动注水泵状态。 */
}

/*
 * 函数功能：压力堵塞停机锁存有效时拦截脚踏保持踩下造成的重复启动。
 * 输入参数：无。
 * 返回参数：true 表示本周期必须保持停机，false 表示允许脚踏按正常路径继续判断。
 */
static bool Foot_BlockRunIfPressureStopLatched(void)
{
    if(Pubinterface_IsPressureBlockStopLatched() == false)
    {
        return false; /* 没有压力停机锁存时，脚踏可以按普通启动沿继续运行。 */
    }

    Foot_LatchMotorStopUntilRelease();              /* 压力保护发生时锁住本次持续踩踏，等待真实松脚后再允许起机。 */
    WorkMessage.runflag_work=false;                 /* 压力保护后脚踏仍踩住时，继续强制手柄运行命令为停止。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免驱动任务看到旧速度重新输出。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 清左脚踏运行来源，必须等左脚释放后重新踩下才允许再置位。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行来源，双脚踏两侧都不能靠保持踩踏自动恢复。 */
    ControlSignalMessage.jtL_gentlypump_flag=false; /* 清左侧轻踩泵联动来源，防止压力保护后轻踩段重新开泵。 */
    ControlSignalMessage.jtR_gentlypump_flag=false; /* 清右侧轻踩泵联动来源，防止右脚保持踩下时泵自动恢复。 */
    s_double_left_gently_pump_channel=CHANNEL_NONE; /* 压力停机已撤销泵输出，左侧实际泵记录同步清除。 */
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
    return ((uint32_t)ad_value <= ((uint32_t)low_value + (uint32_t)JT_threshold)); /* 用和运行入口一致的低阈值判断释放，避免双脚踏一边未松就清压力锁存。 */
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
        is_released = true; /* 脚踏掉线等价于用户已经释放，不能让旧过载锁存永久保留。 */
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
                break; /* 未知脚踏类型无法可靠确认释放，保持锁存比误启动更安全。 */
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
static void Foot_RequireDoublePedalRelease(uint8_t channel)
{
    s_double_pedal_release_before_run_channel = channel;
}

static bool Foot_DoublePedalIsWaitingRelease(uint8_t channel)
{
    return (s_double_pedal_release_before_run_channel == channel);
}

static void Foot_DoublePedalMarkReleased(uint8_t channel)
{
    /* 只有当前等待释放的通道真正松脚时才解除门禁，另一侧松脚不能提前放行。 */
    if(s_double_pedal_release_before_run_channel == channel)
    {
        s_double_pedal_release_before_run_channel = CHANNEL_NONE;
    }
}

/**
 * @brief 获取解析后的数据
 * @return 解析数据指针
 */

/**
 * @brief 清除按键状态
 */


/*
 * 函数功能：脚踏业务控制任务，消费 UART4 解析出的脚踏消息，并根据踏板行程控制注水泵预启动和手柄电机运行。
 * 输入参数：event 调度器传入的任务事件值，当前任务不使用该参数。
 * 返回参数：无。
 */
typedef enum
{
    FOOT_CONTROL_FLOW_CONTINUE = 0,
    FOOT_CONTROL_FLOW_FINISH_CYCLE,
    FOOT_CONTROL_FLOW_RETURN_TASK
} FootControlFlow_t;

static uint8_t s_single_release_debounce_ticks = 0U; /* 单踏板连续低值确认次数，生命周期与原函数内静态变量一致。 */

/*
 * 函数功能：处理脚踏上线或掉线边沿，更新脚踏使能、控制方式、运行状态和屏幕高亮。
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
        MotorUart_ReleaseFootDriverAlarm(); /* 脚踏掉线等价于物理释放，修复过载或其它驱动故障永久等待松脚的边界。 */
        if( WorkMessage.drivetype_work==JTWORK)//如果，前面用的是脚踏使能，现在跳出脚踏自动轮训到什么控制
        {
            if(ControlSignalMessage.jtL_control_flag||ControlSignalMessage.jtR_control_flag)//如果当前正是脚踏控制电机过程中
            {
                ControlSignalMessage.jtL_control_flag=false;//脚踏所以参数职为false
                ControlSignalMessage.jtR_control_flag=false;
                WorkMessage.runflag_work=false;//电机停止运行
                if(WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED||WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD)//过载报警，相关报警清除
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
                    MemoryMsgA.drive_type=HANDLEWORK;//添加记忆功能
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
         ControlSignalMessage.jt_enable_flag=false;//脚踏掉线，禁止脚踏控制界面选择少了一个。后续界面按钮需要判断脚踏是否使能进行判断
         Pubinterface_ClearFootControlManualLock(); /* 脚踏离线代表本在线周期结束，用户手动切手控/触控的锁存到此失效。 */
         Foot_ClearManualAlarmOnRelease(); /* 脚踏掉线等价于释放脚踏，清除手控已选中的错模式报警。 */
         Foot_ClearSocketLatchOnRelease(); /* 脚踏掉线等价于释放脚踏，公共接头缺刀具等待松脚状态也必须结束。 */

         if(ControlSignalMessage.jtL_control_flag||ControlSignalMessage.jtR_control_flag)//如果当前正是脚踏控制电机过程中
         {
            ControlSignalMessage.jtL_control_flag=false;//脚踏所以参数职为false
            ControlSignalMessage.jtR_control_flag=false;
            WorkMessage.runflag_work=false;//电机停止运行
            if(WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED||WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD)//过载报警，相关报警清除
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
            ControlSignalMessage.jtR_gentlypump_flag=false; /* 没有可启动注水泵时不保留右侧轻排运行标志。 */
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
            ControlSignalMessage.jtL_gentlypump_flag=false; /* 没有可启动注水泵时不保留左侧轻排运行标志。 */
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
        s_double_right_gently_pump_channel = CHANNEL_NONE; /* 右脚已经松开，先结束右侧泵所有权。 */
        ControlSignalMessage.jtR_gentlypump_flag=false; /* 清右侧轻排标志，下一次踩下重新建立记录。 */
        other_channel = s_double_left_gently_pump_channel; /* 左脚若仍踩住同一泵，本次不能停。 */
    }
    else
    {
        released_channel = s_double_left_gently_pump_channel; /* 读取左脚实际启动的泵。 */
        s_double_left_gently_pump_channel = CHANNEL_NONE; /* 左脚已经松开，结束左侧泵所有权。 */
        ControlSignalMessage.jtL_gentlypump_flag=false; /* 清左侧轻排标志，防止释放分支重复停泵。 */
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
 * 函数功能：处理单踏板一个 25ms 周期的比例调速、运行门禁、松脚去抖和停止复位。
 * 输入参数：msg 指向当前保存的单踏板在线状态及低点、高点定标值。
 * 返回参数：返回结束本周期或立即退出任务，保持原 break/return 对 owner 释放的区别。
 */
static FootControlFlow_t Foot_ProcessSinglePedal(const FootMessage_t *msg)
{
    uint16_t adValue; /* 本周期单踏板 AD，进入比例计算前按低点钳位。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无有效脚踏参数时保持停机，并允许周期末释放 owner。 */
    }

    adValue = jt_adcvalue; /* 读取单踏板当前 AD，后续按本次定标低点计算有效行程。 */
    if (adValue < msg->LValue_Left) /* 低于定标低点的采样按低点处理，避免无符号减法回绕。 */
    {
        adValue = msg->LValue_Left;
    }
    if ((adValue - msg->LValue_Left) > JT_threshold) /* 超过踩下去抖阈值后才进入运行门禁和速度计算。 */
        {
            if(Foot_BlockRunIfPressureStopLatched())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 压力保护后脚踏仍踩住时，本周期不允许重新写运行标志，必须等待真实松脚。 */
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
            //其他控制中不允许执行改动作
            if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
            {
                //可以滴一下声音
               // SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                Foot_ClearRunRequestAfterGateFail(); /* 外部控制占用时脚踏本周期无效，必须清掉运行边沿标志。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }
           /* 单踏板按“手柄运行才联动注水泵”处理，先不预启动泵，避免 owner 获取失败时泵单独转。 */
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
                Foot_ClearRunRequestAfterGateFail(); /* 未抢到脚踏 owner 时撤销本周期启动请求，不能留下伪运行状态。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            }

            if(Foot_TryAuthorizeMotorRun(FOOT_MOTOR_SOURCE_LEFT) == false)
            {
                Foot_ClearRunRequestAfterGateFail(); /* 上电或异常恢复后脚踏尚未真实松开，本次高AD不能直接恢复手柄运行。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;
            }
            WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(jt_adcvalue,msg->LValue_Left,msg->HValue_Left,0U); /* 单踏板按低值到高值线性映射，输出限制在 EEPROM 最小速度到设定速度之间。 */
            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
            Pubinterface_SetHandleInjectionPumpRun(true); /* 电机确认进入运行态后再按泵类型和当前通道启动 A/B 注水冷却泵，保证冷却泵只跟随手柄运行。 */
        }
        else
        {
            Foot_StopMotorAtReleaseBoundary(); /* 单踏板首次回到低阈值就立即停手柄；下面75ms去抖只继续保护泵输出不抖动。 */
            if(ControlSignalMessage.jtL_control_flag)
            {
                /* 单踏板踩住时 AD 可能短暂跌回阈值以下，先去抖，避免 A 泵被一个采样毛刺立刻停掉。 */
                if(Foot_IgnoreSingleReleaseGlitch(&s_single_release_debounce_ticks))
                {
                    return FOOT_CONTROL_FLOW_FINISH_CYCLE;
                }
                WorkMessage.runflag_work=false;
                Pubinterface_SetHandleInjectionPumpRun(false); /* 单踏板确认松开后，手柄停止的同一周期同步关闭联动注水泵。 */

                //如果泵以注水泵运行-泵停止
                if(ControlSignalMessage.jtL_gentlypump_flag)
                {
                 ControlSignalMessage.jtL_gentlypump_flag=false;
                 if(pumpMessageA.type==INJECTWATER) /* 单踏板轻排实际使用 A 注水泵时，松脚只停止 A 泵。 */
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

               if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
               {
                Foot_ClearHandleOrOverloadAlarm();
               }
                Foot_ClearManualAlarmOnRelease(); /* 单踏板确认松开后清除 81 号错模式报警。 */
                Foot_ClearSocketLatchOnRelease(); /* 单踏板确认松开后允许下一次缺刀具启动重新弹 80。 */
            }
            else
            {
                if(Foot_IgnoreSingleReleaseGlitch(&s_single_release_debounce_ticks))
                {
                    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 压力停机后 jtL_control_flag 已被清零，仍要按松脚去抖确认，防止踩住脚踏时 AD 抖动误清锁存。 */
                }
                Foot_ClearPressureLatchOnRelease(); /* 单踏板确认松开后结束压力停机锁存，下一次重新踩下才允许启动。 */
                Foot_ClearManualAlarmOnRelease(); /* 单踏板无运行标志但已释放时，也要关闭手控已选中报警。 */
                Foot_ClearSocketLatchOnRelease(); /* 单踏板无运行标志但已释放时，同步解除缺刀具等待松脚锁存。 */
                //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
            }
            Foot_ClearHandleOrOverloadAlarm(); /* 门禁失败已清运行标志时，真实松脚仍要关闭手柄未连接报警，让下一次踩下可以重新报警。 */
        }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 单踏板正常处理结束，回到任务入口执行 owner 释放检查。 */
}

/*
 * 函数功能：处理双段踏板一个 25ms 周期的轻踩注水、深踩电机运行和完全释放。
 * 输入参数：msg 指向当前保存的双段踏板在线状态及低点、中点、高点定标值。
 * 返回参数：返回结束本周期或立即退出任务，保持原 break/return 对 owner 释放的区别。
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
    if(adValue-msg->LValue_Left>JT_threshold)//注水标志进行，至于如何让那个泵运行，则要看泵的状态，以及泵的行为
    {
        if(Foot_BlockRunIfPressureStopLatched())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双段脚踏保持踩下时如果仍在压力锁存内，不允许轻踩段重新启动注水泵。 */
        }
        if(Foot_BlockIfSocketMissing())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双段脚踏缺刀具后保持踩下时，不再重新启动轻排泵或重复弹 80。 */
        }
        ControlSignalMessage.jtL_control_flag=true;
        //   if(WorkMessage.alarm_flag!=0)
        //     {
        //         //报警状态不允许脚踏任何动作
        //          return;
        //     }
           if(Foot_EnsureFootControlMode() == false)
           {
               Foot_ClearRunRequestAfterGateFail(); /* 脚踏模式门禁失败时撤销轻踩前置运行标志，防止后续显示像已运行。 */
               return FOOT_CONTROL_FLOW_RETURN_TASK;
           }
              //其他控制中不允许执行改动作(外部控制中，触摸控制中，手控控制中，脚踏右键控制中)
            if(ControlSignalMessage.HMI_control_flag)
            {
                //可以滴一下声音
                //SendKeyBeepMessage(1);//滴一声，一直执行一直响，响声短。（是否界面提示）
                Foot_ClearRunRequestAfterGateFail(); /* 外控仍在占用时，脚踏轻踩请求不能保留到下一周期。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }

            /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

            if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
            {
                 /* A 泵轻踩阶段沿用屏幕当前设定流量，快速踩入电机段时不再短暂写入 Page4 默认流量。 */
                 Foot_StartPumpAInjection(pumpMessageA.speed_work, false, false);
            }
            else if(pumpMessageB.type==INJECTWATER)
            {
                 /* B 泵是注水泵时启动 B 泵，并同步打开 B 泵任务门控。 */
                 Foot_StartPumpBInjection(pumpMessageB.speed_work, false, false);
            }
            ControlSignalMessage.jtL_control_flag=true;
    }
    else
    {
         Foot_StopMotorAtReleaseBoundary(); /* 双段踏板完全退出轻踩区时再次确认手柄停止，并允许下一次重新踩下。 */
         if(ControlSignalMessage.jtL_control_flag)
            {

                if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
                    {
                            Foot_ClearHandleOrOverloadAlarm();
                    }
                WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                //如果泵以注水泵运行-泵停止
                ControlSignalMessage.jtL_control_flag=false;


                 ControlSignalMessage.jtL_control_flag=false;
            }
            //如果泵以注水泵运行-泵停止
            if(ControlSignalMessage.jtL_gentlypump_flag)
                 {
                   ControlSignalMessage.jtL_gentlypump_flag=false;
                  if(pumpMessageA.type==INJECTWATER) /* 双段轻排实际使用 A 注水泵时，完全松脚后停止 A 泵。 */
                  {
                    Foot_StopPumpAInjection();
                  }
                  else if(pumpMessageB.type==INJECTWATER) /* A 不是注水泵时，完全松脚后停止实际使用的 B 泵。 */
                  {
                     Foot_StopPumpBInjection();
                   }
                 }
             Foot_ClearHandleOrOverloadAlarm(); /* 门禁失败已清运行标志时，真实松脚仍要关闭手柄未连接报警，让下一次踩下可以重新报警。 */
             Foot_ClearPressureLatchOnRelease(); /* 双段脚踏完全松到低阈值以下后，释放压力停机锁存，避免再踩脚踏被旧锁存拒绝。 */
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
            Foot_ClearRunRequestAfterGateFail(); /* 电机段未抢到脚踏 owner 时停掉轻踩泵和脚踏运行标志。 */
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
        Pubinterface_SetHandleInjectionPumpRun(true); /* 脚踏二段启动电机后统一经过联动接口，压力锁存时会立即拒绝连续踩踏重新起机。 */
    }
    else
    {
         Foot_StopMotorAtReleaseBoundary(); /* 双段踏板退回中点以下即退出电机段，不能等待轻踩泵释放才停止手柄。 */
         if(ControlSignalMessage.jtL_control_flag)
            {
                 if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
                    {
                        Foot_ClearHandleOrOverloadAlarm();
                    }
                WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                //如果泵以注水泵运行-泵停止
                ControlSignalMessage.jtL_control_flag=false;
            }
            else
            {
                //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
            }
    }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双段踏板正常处理结束，回到任务入口释放空闲 owner。 */
}

/*
 * 函数功能：处理双踏板左右两侧一个 25ms 周期的轻踩注水、通道切换、比例运行和释放。
 * 输入参数：msg 指向当前保存的双踏板左右低点、中点和高点定标值。
 * 返回参数：返回结束本周期或立即退出任务，保持原 break/return 对 owner 释放的区别。
 */
/*
 * 函数功能：处理双踏板左侧一个 25ms 周期的轻踩注水、A 通道运行、跨通道切换和释放复位。
 * 输入参数：msg 指向当前保存的双踏板左侧低点、中点和高点定标值。
 * 返回参数：继续右踏板、结束本周期或立即退出任务，保持原左侧 break/return 语义。
 */
static FootControlFlow_t Foot_ProcessDoublePedalLeft(const FootMessage_t *msg)
{
    uint16_t adValue; /* 左踏板本周期 AD，进入轻踩段和电机段前分别按定标值钳位。 */
    uint8_t switch_before_channel = CHANNEL_NONE; /* 左踏板跨通道切换前快照，用于成功切到 A 后蜂鸣一次。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无左侧定标参数时结束本周期，不继续处理右侧。 */
    }

    adValue = jtd_adcvalue_l; /* 读取双脚踏左侧 AD，按左侧独立定标区间处理。 */
    if (adValue < msg->LValue_Left) /* 低于左踏板低点时钳位，防止后续差值回绕。 */
    {
        adValue = msg->LValue_Left;
    }
        if ((adValue - msg->LValue_Left) > JT_threshold) /* 左侧超过踩下阈值后才进入轻踩泵和深踩电机流程。 */
        {
            if(Foot_BlockRunIfPressureStopLatched())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏左侧保持踩下时如果压力锁存未释放，禁止继续进入轻踩泵和电机启动路径。 */
            }
            if(Foot_BlockIfSocketMissing())
            {
                return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏左侧缺刀具后保持踩下时，本周期只维持停机并等待松脚。 */
            }
            ControlSignalMessage.jtL_control_flag=true;

           if(Foot_EnsureFootControlMode() == false)
           {
               Foot_ClearRunRequestAfterGateFail(); /* 左脚踏门禁失败时撤销运行请求，避免轻踩阶段状态残留。 */
               return FOOT_CONTROL_FLOW_RETURN_TASK;
           }

             //其他控制中不允许执行改动作
            if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
            {
                //可以滴一下声音
                //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                Foot_ClearRunRequestAfterGateFail(); /* 外部控制中左脚踏不参与，清掉本周期脚踏请求。 */
                return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
            }

            /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

            Foot_StartDoublePedalGentlyPump(false); /* 左脚按 A 优先规则启动并记录实际泵，使用屏幕设定速度且不进入10秒定时排空。 */

            if (adValue < msg->MValue_Left) /* 进入深踩段前按中点钳位，避免中点差值发生无符号回绕。 */
            {
                adValue = msg->MValue_Left;
            }
          if ((adValue - msg->MValue_Left) > JT_threshold) /* 超过中点阈值才允许申请电机 owner，轻踩只控制注水泵。 */
            {
                if(WorkMessage.channel_work==CHANNEL_A)
                {
                    /* 双踏板左侧触发当前通道启动前先占用脚踏控制权。 */
                    if(Pubinterface_CheckCommonSocketToolReadyForFootRun() == false)
                    {
                        Foot_LatchSocketMissing(); /* 左脚踏当前通道已触发缺刀具报警，长踩期间不再重复弹窗。 */
                        Foot_ClearRunRequestAfterGateFail(); /* 左脚踏当前通道启动前发现缺刀具，停泵并保持电机停机。 */
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    } /* 双踏板当前通道启动前执行公共接头刀具头 gate。 */
                    if(Foot_DoublePedalIsWaitingRelease(CHANNEL_A))
                    {
                        ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                        return FOOT_CONTROL_FLOW_RETURN_TASK;
                    }
                    if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                    {
                        Foot_ClearRunRequestAfterGateFail(); /* 左脚踏当前通道未取得 owner 时，不允许保持泵或运行标志。 */
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
                    Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板左侧启动 A 通道后进入冷却联动和压力锁存门禁，堵管后保持踩踏不能重启手柄。 */
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
                            } /* 双踏板左侧跨通道直接运行前也要检查公共接头刀具头，避免绕过当前通道 gate。 */
                            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                            {
                                Foot_ClearRunRequestAfterGateFail(); /* 左脚踏跨通道未取得 owner 时，禁止保留上一次轻踩状态。 */
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
                            Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板左侧跨通道启动后同样走压力锁存门禁，避免旧分支绕过停手柄保护。 */
                        }
                }
            }
            else
            {
                Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_LEFT); /* 只有左侧实际持有电机授权时，退回轻踩区才锁存STOP。 */
                  WorkMessage.switchhandle_counts=0;
                if(ControlSignalMessage.jtL_control_flag)
                {
                     WorkMessage.runflag_work=false;
                    WorkMessage.speed_work=0;
                    //如果泵以注水泵运行-泵停止
                    ControlSignalMessage.jtL_control_flag=false; /* 左踏板退回轻踩区时只清左侧电机来源，不能误清右侧。 */

                    if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
                    {
                            Foot_ClearHandleOrOverloadAlarm();
                    }
                 }
            }
        }
        else
        {

            Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_LEFT); /* 左侧完全松开只停止左侧实际启动的电机请求，不能误停右侧。 */
             WorkMessage.switchhandle_counts=0;
            Foot_DoublePedalMarkReleased(CHANNEL_A);
            if(ControlSignalMessage.jtL_control_flag)
            {
               WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                //如果泵以注水泵运行-泵停止
                ControlSignalMessage.jtL_control_flag=false;

              if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
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
                 Foot_ClearHandleOrOverloadAlarm(); /* 门禁失败已清运行标志时，真实松脚仍要关闭手柄未连接报警；双踏板必须确认右侧也已释放。 */
                 Foot_ClearPressureLatchOnRelease(); /* 双脚踏必须左右两侧都回到释放区，才算退出本次压力停机控制源。 */
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
 * 返回参数：结束本周期或立即退出任务，保持原右侧 break/return 语义。
 */
static FootControlFlow_t Foot_ProcessDoublePedalRight(const FootMessage_t *msg)
{
    uint16_t adValue_r; /* 右踏板本周期 AD，进入轻踩段和电机段前分别按定标值钳位。 */
    uint8_t switch_before_channel = CHANNEL_NONE; /* 右踏板跨通道切换前快照，用于成功切到 B 后蜂鸣一次。 */

    if (msg == NULL)
    {
        return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 无右侧定标参数时结束本周期并允许 owner 释放。 */
    }

    adValue_r=jtd_adcvalue_r;
    if(adValue_r<msg->LValue_Right)adValue_r=msg->LValue_Right;
    if(adValue_r-msg->LValue_Right>JT_threshold)//右踏板
    {
        if(Foot_BlockRunIfPressureStopLatched())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏右侧保持踩下时如果压力锁存未释放，禁止蜂鸣结束后自动拉起手柄和泵。 */
        }
        if(Foot_BlockIfSocketMissing())
        {
            return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 双脚踏右侧缺刀具后保持踩下时，只保持停机，不再重复触发 80 弹窗。 */
        }
        ControlSignalMessage.jtR_control_flag=true;
       if(Foot_EnsureFootControlMode() == false)
       {
           Foot_ClearRunRequestAfterGateFail(); /* 右脚踏门禁失败时清掉右侧运行请求，防止释放分支误判。 */
           return FOOT_CONTROL_FLOW_RETURN_TASK;
       }

         //其他控制中不允许执行改动作
        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
        {
            //可以滴一下声音
            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
            Foot_ClearRunRequestAfterGateFail(); /* 外部控制中右脚踏不参与，清掉本周期脚踏请求。 */
            return FOOT_CONTROL_FLOW_RETURN_TASK;//不参与
        }

       /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

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
                   Foot_ClearRunRequestAfterGateFail(); /* 右脚踏当前通道启动前发现缺刀具，报警后不保留运行边沿。 */
                   return FOOT_CONTROL_FLOW_RETURN_TASK;
               } /* 右踏板启动当前通道前检查公共接头刀具头是否已识别。 */
               if(Foot_DoublePedalIsWaitingRelease(CHANNEL_B))
               {
                   ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                   return FOOT_CONTROL_FLOW_RETURN_TASK;
               }
               if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
               {
                   Foot_ClearRunRequestAfterGateFail(); /* 右脚踏当前通道未取得 owner 时，不允许保持泵或运行标志。 */
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
                Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板右侧启动 B 通道后统一刷新冷却泵跟随，并让压力堵塞锁存能够清回 runflag。 */
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
                                Foot_ClearRunRequestAfterGateFail(); /* 右脚踏跨通道未取得 owner 时，禁止残留脚踏运行标志。 */
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
                            Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板右侧跨通道启动后不能绕过联动接口，否则压力停机后脚踏保持会重新置运行。 */
                        }

                    }
            }
        }
        else
        {
            Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_RIGHT); /* 只有右侧实际持有电机授权时，退回轻踩区才锁存STOP。 */
             WorkMessage.switchhandle_countss=0;
            if(ControlSignalMessage.jtR_control_flag)
            {
                 WorkMessage.runflag_work=false;
                WorkMessage.speed_work=0;
                //如果泵以注水泵运行-泵停止
                ControlSignalMessage.jtR_control_flag=false; /* 右踏板退回轻踩区时只清右侧电机来源，保持左右状态对称。 */

                if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
                {
                        Foot_ClearHandleOrOverloadAlarm();
                }
             }
        }
    }
    else
    {
        Foot_StopDoubleMotorAtReleaseBoundary(FOOT_MOTOR_SOURCE_RIGHT); /* 右侧完全松开只停止右侧实际启动的电机请求，不能误停左侧。 */
        WorkMessage.switchhandle_countss=0;
        Foot_DoublePedalMarkReleased(CHANNEL_B);
        if(ControlSignalMessage.jtR_control_flag)
        {
           WorkMessage.runflag_work=false;
            WorkMessage.speed_work=0;
            //如果泵以注水泵运行-泵停止
            ControlSignalMessage.jtR_control_flag=false;

          if(WorkMessage.alarm_value==WORK_ALARM_MOTOR_OVERLOAD||WorkMessage.alarm_value==WORK_ALARM_HANDLE_NOT_CONNECTED)//过载或者手柄未连接的情况，清除报警
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
            Foot_ClearHandleOrOverloadAlarm(); /* 门禁失败已清运行标志时，真实松脚仍要关闭手柄未连接报警；双踏板必须确认左侧也已释放。 */
            Foot_ClearPressureLatchOnRelease(); /* 双脚踏必须左右两侧都松开后才清压力锁存，防止另一侧仍踩住时自动恢复。 */
            Foot_ClearManualAlarmOnRelease(); /* 双脚踏左右都释放后清除手控已选中的错模式报警。 */
            Foot_ClearSocketLatchOnRelease(); /* 双脚踏左右都释放后，解除公共接头缺刀具等待松脚锁存。 */
        }
     }

    return FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 右侧正常处理结束，回到任务入口执行 owner 释放检查。 */
}

/*
 * 函数功能：按固定的先左后右顺序处理双踏板，并把左侧提前结束语义传回主任务。
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
        Foot_StopMotorAtReleaseBoundary(); /* 双侧都退出各自电机区后才完成全局重新武装，轻踩泵区仍可作为安全释放区。 */
    }

    flow = Foot_ProcessDoublePedalLeft(msg); /* 原大函数固定先处理左踏板。 */
    if (flow != FOOT_CONTROL_FLOW_CONTINUE)
    {
        return flow; /* 左侧原 break/return 都必须跳过右侧，直接交回主任务。 */
    }

    return Foot_ProcessDoublePedalRight(msg); /* 左侧无提前结束时再处理右踏板。 */
}

/*
 * 函数功能：脚踏业务控制任务，读取最新连接消息并按踏板类型分派一个 25ms 行为周期。
 * 输入参数：event 为调度器事件值，当前任务不使用。
 * 返回参数：无。
 */
void FootControlTask(uint32_t event)
{
    static FootMessage_t msg; /* 保存最近一次脚踏连接状态和定标值，无新消息时继续使用。 */
    FootControlFlow_t flow = FOOT_CONTROL_FLOW_FINISH_CYCLE; /* 默认完成本周期并执行 owner 释放检查。 */

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
        Foot_HandleConnectionUpdate(&msg); /* 只有收到新消息时才处理上线或掉线边沿。 */
    }

    if(s_foot_runtime_frame_valid == false)
    {
        Foot_StopRunOnRealtimeTimeout(); /* 没有可信实时AD时持续保持脚踏输出为停止，防止旧高值在下一周期重新起机。 */
        return; /* 等待新的完整CRC实时帧恢复门禁，本周期不得进入任何脚踏比例控制。 */
    }

    if (Foot_BlockDriverAlarmRun(&msg))
    {
        return; /* 驱动故障后的持续踩踏已由统一门禁停机，不能再进入任一种脚踏启动分支。 */
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
        return; /* 对应原分支中的 return，不能额外执行周期末 owner 释放。 */
    }

    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT); /* 对应原 case break 或正常结束后的统一释放。 */
}

/**
 * @brief 脚踏板任务初始化
 */



typedef struct
{
    uint8_t connected;             /* 1 表示脚踏已经完成定标校验并向行为队列发布上线。 */
    volatile uint8_t silent_ticks; /* 连续无完整帧的 10ms 周期数，超过 100 后确认掉线。 */
    uint8_t single_low_ready;      /* 单踏板低值已经读回，下一步等待高值。 */
    uint8_t single_high_requested; /* 单踏板高值读取命令已经发送，防止重复发送。 */
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
    FootMessage_t disconnect_msg; /* 掉线消息使用局部副本，入队失败时不能提前污染全局在线快照。 */

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
        footmessage.connect_flag = false; /* 掉线消息成功入队后才更新全局连接快照，避免发送失败后无法恢复。 */
        SendKeyBeepMessage(1U); /* 脚踏真实掉线时保留一次按键蜂鸣提示。 */
    }

    s_foot_parser_state.silent_ticks = 0U; /* 掉线消息已成功发布或当前仅处理定标报警，清零供下次上线使用。 */
    s_foot_parser_state.connected = 0U; /* 回到未连接状态，下次插入必须重新校验定标值。 */
    s_foot_parser_state.single_low_ready = 0U; /* 清除单踏板低值读取阶段。 */
    s_foot_parser_state.single_high_requested = 0U; /* 清除单踏板高值读取阶段。 */
    Foot_ClearFootValueErrorAlarm(); /* 错误值脚踏拔出后清除 83 号报警。 */
}

/*
 * 函数功能：根据脚踏协议头和类型字段确定一帧完整数据的固定长度。
 * 输入参数：frame 指向FE EF帧头；remaining 为当前DMA快照从帧头开始的剩余字节数。
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
        return 0U; /* 非当前脚踏协议的FE EF数据不能续期在线状态。 */
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
        return FOOT_SINGLE_FRAME_LENGTH; /* 脚踏实体按键帧保持10字节，但不作为实时AD保活。 */
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
            Uart4_SendPacket(get_jtLvalue, 8U); /* 首次识别单踏板时请求 EEPROM 低值。 */
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
            Uart4_SendPacket(get_jtHvalue, 8U); /* 低值到达后只请求一次高值。 */
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
            return; /* 未定义按键码不投递消息，也不产生蜂鸣。 */
    }

    SendKeyBeepMessage(1U); /* 合法脚踏按键保持一次按键蜂鸣。 */
}

/*
 * 函数功能：处理双段踏板和双脚踏的实时值、定标值及实体按键。
 * 输入参数：frame 指向 FE EF BB AA 帧；remaining 为从帧头开始的剩余字节数。
 * 返回参数：true 表示保持原提前退出语义；false 表示继续扫描后续数据。
 */
static bool Foot_ParseMultiPedalFrame(const uint8_t *frame, uint16_t remaining)
{
    if ((remaining >= 10U) && (frame[4] == 0xDDU) && (frame[5] == 0x01U))
    {
        jtb_adcvalue = ((uint16_t)frame[6] << 8) | frame[7]; /* 保存双段踏板当前 AD 值。 */
        if (s_foot_parser_state.connected != 0U)
        {
            return true; /* 保持旧逻辑：双段踏板已在线时收到实时帧后立即结束本周期。 */
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
            s_foot_parser_state.connected = 1U; /* 保持未上线时每帧都可重试同一连接消息。 */
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
            return true; /* 双脚踏未上线且定标字段不完整时保持原立即退出语义。 */
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
            s_foot_parser_state.connected = 1U; /* 双脚踏连接状态只有在行为队列收到完整定标快照后才生效。 */
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
    /* 调用方已经完成固定长度和CRC校验；此处只按命令头分派业务字段，不能单独续期在线状态。 */
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
    bool realtime_frame_valid = false; /* 本周期至少有一帧完整CRC实时AD时置位，用于复位失效安全计时。 */
    bool stop_scan = false; /* 保留原解析函数的提前结束语义，但结束前仍要统一维护实时帧状态。 */
    uint8_t data[255] = {0U}; /* UART4 单周期接收缓存，容量保持原 255 字节。 */

    (void)event;
    received_len = Uart4_DMARecvDataPeek(data); /* 读取并消费本周期 UART4 DMA 数据。 */
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
            ++offset; /* 未支持的FE EF组合不能阻塞同一DMA快照中的后续合法帧。 */
            continue;
        }
        if((uint16_t)(received_len - offset) < frame_length)
        {
            break; /* DMA尾部只有半帧时不读取字段；本周期按无有效实时帧推进安全计时。 */
        }
        if(Foot_IsFrameCrcValid(&data[offset], frame_length) == false)
        {
            ++offset; /* CRC错误只丢弃当前候选帧头，继续搜索同一快照中的下一帧。 */
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
            realtime_frame_valid = true; /* 只有三类实时AD帧可以证明当前踩踏值可信，读回值和按键帧不能续期。 */
        }

        stop_scan = Foot_ParseUartFrame(&data[offset], frame_length); /* 完整且CRC正确后才允许更新AD、定标值或按键事件。 */
        offset = (uint16_t)(offset + frame_length); /* 已消费一帧时直接跨过完整长度，避免把帧内数据误识别为新帧头。 */
        if(stop_scan != false)
        {
            break; /* 无效定标或原有提前退出分支结束本次扫描，但仍执行下方实时保活维护。 */
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

void SscFootControlTask_Init(void)
{
    //初始化消息队列
    Foot_Queue_Init();
     //创建任务
    Kernel_TaskCreate(&FOOTTaskHandle, Foot_ParseDataS);
    Kernel_TaskStart(&FOOTTaskHandle, KERNEL_TASK_ALWAYS, 10);//这里时间尤为重要，要保证每一帧能接受到，（数据解析需要修改）

    Kernel_TaskCreate(&FOOTBHHandle, FootControlTask);
    Kernel_TaskStart(&FOOTBHHandle, KERNEL_TASK_ALWAYS, 25);
}
