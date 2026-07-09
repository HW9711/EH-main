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
#include "Pubinterface.h"
#include "sscKEYBH.h"
#include "sscUIDP.h"
#include "sscBEEP.h"
#include "sscPUMPA.h"
#include "sscPUMPB.h"



#define JT_threshold  20U
/* 单踏板松脚去抖周期数；FootControlTask 为 25ms，3 个周期约 75ms，用于过滤踩住时 AD 瞬时跌落导致的泵反复启停。 */
#define FOOT_SINGLE_PEDAL_RELEASE_DEBOUNCE_TICKS 3U
#define FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS 10U
#define FOOT_PEDAL_SPEED_HIGH_MARGIN 30U /* 双脚踏当前通道运行段预留的高位死区，沿用旧公式 HValue-MValue-30 的行程范围。 */

static uint8_t get_jtHvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB8,0Xdf,0x3e,0x84};//读高值

static uint8_t get_jtLvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB5,0XCD,0xA3,0x00};//读低值

static uint16_t jt_adcvalue = 0;
static uint16_t jtb_adcvalue = 0;
static uint16_t jtd_adcvalue_l = 0;
static uint16_t jtd_adcvalue_r = 0;
static uint8_t s_double_pedal_release_before_run_channel = CHANNEL_NONE;
static bool s_common_socket_missing_wait_release = false; /* 公共接头缺刀具触发后等待脚踏真实释放，防止同一次长踩反复弹 80。 */



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











/**
 * @brief 发送脚踏板消息到队列
 * @param msgType 消息类型
 * @param adValue AD值
 * @param keyStatus 按键状态
 */
static void Foot_SendMessage(FootMessage_t msg)
{
    //初始化一个静态FootMessage_t bj_msg结构体，并且判断bj_msg和msg的值是否相等，如果相等返回，如果不相等msg赋值给bj_msg
    static FootMessage_t bj_msg;
    if(FootMsgQueue == NULL) return;
    if(memcmp(&bj_msg, &msg, sizeof(FootMessage_t)) == 0) return;
       memcpy(&bj_msg, &msg, sizeof(FootMessage_t));
    (void)Kernel_QueueSend(FootMsgQueue, &msg, 0);
}

/**
 * @brief 初始化脚踏板消息队列
 */
static void Foot_Queue_Init(void)
{
    FootMsgQueue = Kernel_QueueCreate(5, sizeof(FootMessage_t), "FootMsgQueue");
}

static void Foot_StartPumpAInjection(uint16_t speed_work)
{
    /* 脚踏启动 A 注水泵时，定时排空必须让位，避免排空计时同时改写泵输出。 */
    pumpMessageA.timingDrainage_flag=false;
    /* 排空时间清零，保证下次排空从完整周期重新开始计时。 */
    pumpMessageA.timingDrainage_times=0U;
    /* 记录本次脚踏正在联动轻排泵，松开脚踏时才会进入对应停泵分支。 */
    ControlSignalMessage.jtL_gentlypump_flag=true;
    /* 保存脚踏本次要求的 A 泵速度，泵任务下一周期从公共 pumpMessageA 读取。 */
    pumpMessageA.speed_work=speed_work;
    /* run_flag 是 sscPUMPA 任务真正允许输出非零速度的门控，脚踏启动必须置位。 */
    pumpMessageA.run_flag=true;
    /* 队列消息只兼容旧接口同步速度；泵类型仍以模拟串口设备码写入的 pumpMessageA.type 为准。 */
    SendPumpAMessage(INJECTWATER,pumpMessageA.speed_work);
}

static void Foot_StartPumpBInjection(uint16_t speed_work)
{
    /* 脚踏启动 B 注水泵时，先关闭定时排空，避免两种泵动作同时抢占 B 泵输出。 */
    pumpMessageB.timingDrainage_flag=false;
    /* 清掉历史排空计数，下一次排空动作不能沿用脚踏前的剩余时间。 */
    pumpMessageB.timingDrainage_times=0U;
    /* 记录脚踏轻排已经启动，松开脚踏时按同一标志关闭联动泵。 */
    ControlSignalMessage.jtL_gentlypump_flag=true;
    /* 保存脚踏本次要求的 B 泵速度，泵任务下一周期从公共 pumpMessageB 读取。 */
    pumpMessageB.speed_work=speed_work;
    /* run_flag 是 sscPUMPB 任务真正允许输出非零速度的门控，脚踏启动必须置位。 */
    pumpMessageB.run_flag=true;
    /* 队列消息只兼容旧接口同步速度；泵类型仍以模拟串口设备码写入的 pumpMessageB.type 为准。 */
    SendPumpBMessage(INJECTWATER,pumpMessageB.speed_work);
}

static void Foot_StopPumpAInjection(void)
{
    /* 松开脚踏后关闭 A 泵运行门控，sscPUMPA 下一周期会按 run_flag=false 下发 0 速。 */
    pumpMessageA.run_flag=false;
    /* 脚踏停泵不进入排空模式，必须同步清除排空标志。 */
    pumpMessageA.timingDrainage_flag=false;
    /* 排空计数清零，避免下一次排空或脚踏启动继承旧计数。 */
    pumpMessageA.timingDrainage_times=0U;
}

static void Foot_StopPumpBInjection(void)
{
    /* 松开脚踏后关闭 B 泵运行门控，sscPUMPB 下一周期会按 run_flag=false 下发 0 速。 */
    pumpMessageB.run_flag=false;
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
    WorkMessage.runflag_work=false;                 /* 门禁失败时禁止驱动任务继续看到运行命令，公共接头缺刀具不能下发启动帧。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免屏幕或驱动继续沿用本周期脚踏比例速度。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 清左脚踏运行标志，避免 gate 失败后释放分支误认为左脚已经启动。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行标志，双脚踏任一侧失败都不能留下运行来源。 */
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
 * 函数功能：公共接头缺 EPC 刀具头报警后锁住当前脚踏触发，要求操作者松脚后才能再次启动报警。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Foot_LatchCommonSocketMissingUntilRelease(void)
{
    s_common_socket_missing_wait_release=true;       /* 当前脚踏动作已经触发过缺刀具报警，同一次长踩不再反复请求启动和刷 80。 */
}

/*
 * 函数功能：公共接头缺刀具等待松脚锁存有效时拦截脚踏保持踩下造成的重复报警。
 * 输入参数：无。
 * 返回参数：true 表示本周期已被等待松脚锁存拦截；false 表示允许继续判断脚踏启动。
 */
static bool Foot_BlockRunIfCommonSocketMissingLatched(void)
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
static void Foot_ClearCommonSocketMissingLatchAfterRelease(void)
{
    if(s_common_socket_missing_wait_release==false)
    {
        return;                                      /* 当前没有缺刀具等待松脚锁存时不改状态，避免影响其它门禁失败。 */
    }

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
static void Foot_ClearManualSelectedAlarmOnRelease(void)
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
    static uint8_t times=0;
    /* 脚踏释放动作只清手柄未连接/电机过载这两类可恢复提示，不能误清 UID、通讯、HALL 等故障。 */
    if(WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED))
    {
           WorkAlarm_Clear();                 /* 清掉统一报警状态，让后续控制方式可以重新响应。 */
          SendAlarmMessage(WORK_ALARM_NONE); /* 释放脚踏或拔掉脚踏时同步停止报警蜂鸣。 */
          SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 同步关闭屏幕报警弹窗，避免“手柄未连接/过载”残留。 */
            if(WorkMessage.Channel_Aonline)
            {
              //  WorkMessage.channel_work = CHANNEL_A;
               //拔掉B手柄
              //  SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_A); 
                HandleSwitchActive(SCREENKey_HANDLE_A);
            }
            else if(WorkMessage.Channel_Bonline)
            {
               // WorkMessage.channel_work = CHANNEL_B;
                //拔掉A手柄
               //  SendKeyBehMessage(PLUGunPLUG, SCREENKey_PLUG_B); 
                 HandleSwitchActive(SCREENKey_HANDLE_B);
            }
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
static bool Foot_ShouldIgnoreSinglePedalReleaseGlitch(uint8_t *release_ticks)
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
static void Foot_ClearPressureStopLatchAfterRelease(void)
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

    WorkMessage.runflag_work=false;                 /* 压力保护后脚踏仍踩住时，继续强制手柄运行命令为停止。 */
    WorkMessage.speed_work=0U;                      /* 同步清实际目标速度，避免驱动任务看到旧速度重新输出。 */
    ControlSignalMessage.jtL_control_flag=false;    /* 清左脚踏运行来源，必须等左脚释放后重新踩下才允许再置位。 */
    ControlSignalMessage.jtR_control_flag=false;    /* 清右脚踏运行来源，双脚踏两侧都不能靠保持踩踏自动恢复。 */
    ControlSignalMessage.jtL_gentlypump_flag=false; /* 清左侧轻踩泵联动来源，防止压力保护后轻踩段重新开泵。 */
    ControlSignalMessage.jtR_gentlypump_flag=false; /* 清右侧轻踩泵联动来源，防止右脚保持踩下时泵自动恢复。 */
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

static void Foot_DoublePedalRequireReleaseBeforeRun(uint8_t channel)
{
    s_double_pedal_release_before_run_channel = channel;
}

static bool Foot_DoublePedalIsWaitingRelease(uint8_t channel)
{
    return (s_double_pedal_release_before_run_channel == channel);
}

static void Foot_DoublePedalMarkReleased(uint8_t channel)
{
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
void FootControlTask(uint32_t event)
{
    uint16_t adValue;
    uint16_t adValue_r;
    uint8_t switch_before_channel = CHANNEL_NONE; /* 双脚踏跨通道切换前的工作通道快照，用于确认切换成功后只蜂鸣一次。 */
    static uint8_t single_release_debounce_ticks=0U;
   (void)event;
    if(ControlArbitration_IsBusyByOther(CONTROL_OWNER_FOOT))
    {
        FootMessage_t discard_msg;
        /* 其它控制方式占用期间脚踏不能改 WorkMessage，同时丢弃旧踏板消息，避免对方结束后误触发。 */
        if(FootMsgQueue != NULL)
        {
            (void)Kernel_QueueReceive(FootMsgQueue, &discard_msg, 0);
        }
        return;
    }
    /* 泵类型只由模拟串口设备码刷新，脚踏任务只按当前已识别类型决定是否联动注水泵。 */
    //从消息队列获取消息
    if(FootMsgQueue != NULL)
    {
         static FootMessage_t msg;
        if(Kernel_QueueReceive(FootMsgQueue, &msg, 0) == pdTRUE)
        {
            if(msg.connect_flag==false)
            {
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
                    if(WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES)//或者空心转
                    {
                         ControlSignalMessage.handle_enable_flag=true;//控制信号，手控使能开启
                         WorkMessage.drivetype_work=HANDLEWORK;//工作标志手控
                         if(WorkMessage.channel_work==CHANNEL_A)
                         {
                            MemoryMsgA.drive_type=HANDLEWORK;//添加记忆功能
                         }
                         else if(WorkMessage.channel_work==CHANNEL_B)
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
                 Foot_ClearManualSelectedAlarmOnRelease(); /* 脚踏掉线等价于释放脚踏，清除手控已选中的错模式报警。 */
                 Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 脚踏掉线等价于释放脚踏，公共接头缺刀具等待松脚状态也必须结束。 */

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
       if(msg.connect_flag==true)
         {
             //手柄运行
            switch(msg.pedalType)
            {
                case 1://jt
                adValue=jt_adcvalue;
                if(adValue<msg.LValue_Left)adValue=msg.LValue_Left;
                if(adValue-msg.LValue_Left>JT_threshold)
                    {
                        if(Foot_BlockRunIfPressureStopLatched())
                        {
                            break; /* 压力保护后脚踏仍踩住时，本周期不允许重新写运行标志，必须等待真实松脚。 */
                        }
                        if(Foot_BlockRunIfCommonSocketMissingLatched())
                        {
                            break; /* 公共接头缺刀具后脚踏仍踩住时，只保持停机，等待松脚后才允许重新报警。 */
                        }
                            ControlSignalMessage.jtL_control_flag=true;
                        /* 单踏板重新确认踩下后清掉松脚去抖计数，防止上一轮释放残留影响本次泵保持运行。 */
                        single_release_debounce_ticks=0U;
                        if(Foot_EnsureFootControlMode() == false)
                        {
                            Foot_ClearRunRequestAfterGateFail(); /* 模式或手柄状态不允许脚踏运行时，撤销前面提前置位的脚踏请求。 */
                            return;
                        }
                        //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                           // SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            Foot_ClearRunRequestAfterGateFail(); /* 外部控制占用时脚踏本周期无效，必须清掉运行边沿标志。 */
                            return;//不参与
                        }
                       /* 单踏板按“手柄运行才联动注水泵”处理，先不预启动泵，避免 owner 获取失败时泵单独转。 */
                        if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
                        /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
                        if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                        {
                            Foot_LatchCommonSocketMissingUntilRelease(); /* 本次脚踏已经触发公共接头缺刀具报警，后续长踩必须等松脚再触发。 */
                            Foot_ClearRunRequestAfterGateFail(); /* 公共接头缺刀具头时只报警 80，并保证脚踏不会留下运行状态。 */
                            return;
                        } /* 脚踏启动手柄电机前检查公共接头 EPC 刀具头，缺失时只报警不运行。 */
                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                        {
                            Foot_ClearRunRequestAfterGateFail(); /* 未抢到脚踏 owner 时撤销本周期启动请求，不能留下伪运行状态。 */
                            return;
                        }
                    
                        WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(jt_adcvalue,msg.LValue_Left,msg.HValue_Left,0U); /* 单踏板按低值到高值线性映射，输出限制在 EEPROM 最小速度到设定速度之间。 */
                        WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                        Pubinterface_SetHandleInjectionPumpRun(true); /* 电机确认进入运行态后再按泵类型和当前通道启动 A/B 注水冷却泵，保证冷却泵只跟随手柄运行。 */
                    }
                    else
                    {
                        if(ControlSignalMessage.jtL_control_flag)
                        {
                            /* 单踏板踩住时 AD 可能短暂跌回阈值以下，先去抖，避免 A 泵被一个采样毛刺立刻停掉。 */
                            if(Foot_ShouldIgnoreSinglePedalReleaseGlitch(&single_release_debounce_ticks))
                            {
                                break;
                            }
                            WorkMessage.runflag_work=false;
                            Pubinterface_SetHandleInjectionPumpRun(false); /* 单踏板确认松开后，手柄停止的同一周期同步关闭联动注水泵。 */

                            //如果泵以注水泵运行-泵停止
                            if(ControlSignalMessage.jtL_gentlypump_flag)
                            {
                             ControlSignalMessage.jtL_gentlypump_flag=false;
                             if(pumpMessageA.type==INJECTWATER)
                             {
                               Foot_StopPumpAInjection();
                             }
                             else if(pumpMessageB.type==INJECTWATER)
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
                            Foot_ClearManualSelectedAlarmOnRelease(); /* 单踏板确认松开后清除 81 号错模式报警。 */
                            Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 单踏板确认松开后允许下一次缺刀具启动重新弹 80。 */
                        }
                        else
                        {
                            if(Foot_ShouldIgnoreSinglePedalReleaseGlitch(&single_release_debounce_ticks))
                            {
                                break; /* 压力停机后 jtL_control_flag 已被清零，仍要按松脚去抖确认，防止踩住脚踏时 AD 抖动误清锁存。 */
                            }
                            Foot_ClearPressureStopLatchAfterRelease(); /* 单踏板确认松开后结束压力停机锁存，下一次重新踩下才允许启动。 */
                            Foot_ClearManualSelectedAlarmOnRelease(); /* 单踏板无运行标志但已释放时，也要关闭手控已选中报警。 */
                            Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 单踏板无运行标志但已释放时，同步解除缺刀具等待松脚锁存。 */
                            //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
                        }
                    }
                break;
                case 2://jb
                adValue=jtb_adcvalue;
                if(adValue<msg.LValue_Left)adValue=msg.LValue_Left;
                if(adValue-msg.LValue_Left>JT_threshold)//注水标志进行，至于如何让那个泵运行，则要看泵的状态，以及泵的行为
                {
                    if(Foot_BlockRunIfPressureStopLatched())
                    {
                        break; /* 双段脚踏保持踩下时如果仍在压力锁存内，不允许轻踩段重新启动注水泵。 */
                    }
                    if(Foot_BlockRunIfCommonSocketMissingLatched())
                    {
                        break; /* 双段脚踏缺刀具后保持踩下时，不再重新启动轻排泵或重复弹 80。 */
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
                           return;
                       }
                          //其他控制中不允许执行改动作(外部控制中，触摸控制中，手控控制中，脚踏右键控制中)
                        if(ControlSignalMessage.HMI_control_flag)
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声，一直执行一直响，响声短。（是否界面提示）
                            Foot_ClearRunRequestAfterGateFail(); /* 外控仍在占用时，脚踏轻踩请求不能保留到下一周期。 */
                            return;//不参与
                        }

                        /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

                        if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
                        {
                             /* A 泵是注水泵时按当前手柄 Page4 默认流量启动；旧代码误写 B 泵会导致脚踏踩下后目标泵不转。 */
                             Foot_StartPumpAInjection(Pubinterface_GetCurrentDefaultInjectionFlow());
                        }
                        else if(pumpMessageB.type==INJECTWATER)
                        {
                             /* B 泵是注水泵时启动 B 泵，并同步打开 B 泵任务门控。 */
                             Foot_StartPumpBInjection(pumpMessageB.speed_work);
                        }
                        ControlSignalMessage.jtL_control_flag=true;
                }
                else
                {
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
                              if(pumpMessageA.type==INJECTWATER)
                              {
                                Foot_StopPumpAInjection();
                              }
                              else if(pumpMessageB.type==INJECTWATER)
                              {
                                 Foot_StopPumpBInjection();
                               }
                             }
                         Foot_ClearPressureStopLatchAfterRelease(); /* 双段脚踏完全松到低阈值以下后，释放压力停机锁存，避免再踩脚踏被旧锁存拒绝。 */
                         Foot_ClearManualSelectedAlarmOnRelease(); /* 双段脚踏完全松开后清除 81 号错模式报警。 */
                         Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 双段脚踏完全释放后，下一次踩下才允许公共接头缺刀具重新报警。 */
                 }
                if(adValue<msg.MValue_Left)adValue=msg.MValue_Left;
                if(adValue-msg.MValue_Left>JT_threshold)
                {
                     //手柄运行
                       if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
                    /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
                    if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                    {
                        Foot_LatchCommonSocketMissingUntilRelease(); /* 双段脚踏电机段已触发公共接头缺刀具报警，保持踩下时不再重复弹 80。 */
                        Foot_ClearRunRequestAfterGateFail(); /* 双段脚踏进入电机段前发现公共接头无刀具，停泵并清运行请求。 */
                        return;
                    } /* 脚踏比例启动前先确认刀具头参数有效，避免公共接头空刀具运行。 */
                    if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                    {
                        Foot_ClearRunRequestAfterGateFail(); /* 电机段未抢到脚踏 owner 时停掉轻踩泵和脚踏运行标志。 */
                        return;
                    }
                    ControlSignalMessage.jtL_control_flag=true;
                    WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg.MValue_Left,msg.HValue_Left,0U); /* 双段脚踏电机段从中值开始算比例，不再从 0rpm 起步。 */
                    WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                    Pubinterface_SetHandleInjectionPumpRun(true); /* 脚踏二段启动电机后统一经过联动接口，压力锁存时会立即拒绝连续踩踏重新起机。 */
                }
                else
                {
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
               break;
                case 3://jtd
                adValue=jtd_adcvalue_l;
                if(adValue<msg.LValue_Left)adValue=msg.LValue_Left;
                    if(adValue-msg.LValue_Left>JT_threshold)
                    {
                        if(Foot_BlockRunIfPressureStopLatched())
                        {
                            break; /* 双脚踏左侧保持踩下时如果压力锁存未释放，禁止继续进入轻踩泵和电机启动路径。 */
                        }
                        if(Foot_BlockRunIfCommonSocketMissingLatched())
                        {
                            break; /* 双脚踏左侧缺刀具后保持踩下时，本周期只维持停机并等待松脚。 */
                        }
                        ControlSignalMessage.jtL_control_flag=true;

                       if(Foot_EnsureFootControlMode() == false)
                       {
                           Foot_ClearRunRequestAfterGateFail(); /* 左脚踏门禁失败时撤销运行请求，避免轻踩阶段状态残留。 */
                           return;
                       }

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            Foot_ClearRunRequestAfterGateFail(); /* 外部控制中左脚踏不参与，清掉本周期脚踏请求。 */
                            return;//不参与
                        }

                        /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

                        if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
                        {
                             /* 双踏板左侧轻排启动 A 注水泵，并确保 A 泵 run_flag 已打开。 */
                             Foot_StartPumpAInjection(pumpMessageA.speed_work);
                        }
                        else if(pumpMessageB.type==INJECTWATER)
                        {
                             /* 双踏板左侧轻排启动 B 注水泵，并确保 B 泵 run_flag 已打开。 */
                             Foot_StartPumpBInjection(pumpMessageB.speed_work);
                        }

                        if(adValue<msg.MValue_Left)adValue=msg.MValue_Left;
                      if(adValue-msg.MValue_Left>JT_threshold)
                        {
                            if(WorkMessage.channel_work==CHANNEL_A)
                            {
                                /* 双踏板左侧触发当前通道启动前先占用脚踏控制权。 */
                                if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                                {
                                    Foot_LatchCommonSocketMissingUntilRelease(); /* 左脚踏当前通道已触发缺刀具报警，长踩期间不再重复弹窗。 */
                                    Foot_ClearRunRequestAfterGateFail(); /* 左脚踏当前通道启动前发现缺刀具，停泵并保持电机停机。 */
                                    return;
                                } /* 双踏板当前通道启动前执行公共接头刀具头 gate。 */
                                if(Foot_DoublePedalIsWaitingRelease(CHANNEL_A))
                                {
                                    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                    return;
                                }
                                if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                                {
                                    Foot_ClearRunRequestAfterGateFail(); /* 左脚踏当前通道未取得 owner 时，不允许保持泵或运行标志。 */
                                    return;
                                }
                                ControlSignalMessage.jtL_control_flag=true;
                                WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg.MValue_Left,msg.HValue_Left,FOOT_PEDAL_SPEED_HIGH_MARGIN); /* 双脚踏左侧当前通道保留高位死区，并限制最大不超过设定速度。 */
                                WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                                Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板左侧启动 A 通道后进入冷却联动和压力锁存门禁，堵管后保持踩踏不能重启手柄。 */
                            }
                            else if(WorkMessage.channel_work==CHANNEL_B)
                            {
                                    if(WorkMessage.Channel_Aonline==true)
                                    {
                                        WorkMessage.switchhandle_counts++;
                                        if(WorkMessage.switchhandle_counts<FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS)
                                        {
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;//未达到切换去抖计数，不执行
                                        }
                                         WorkMessage.switchhandle_counts=0;
                                        switch_before_channel = WorkMessage.channel_work; /* 记录切换前通道，后续用实际 channel_work 判断是否成功切到 A。 */
                                        HandleSwitchActive(SCREENKey_HANDLE_A);//切换
                                        if((switch_before_channel != CHANNEL_A) && (WorkMessage.channel_work == CHANNEL_A))
                                        {
                                            SendKeyBeepMessage(1U); /* 双脚踏左踏板跨通道切换 A 手柄成功，蜂鸣一次给操作者确认。 */
                                        }
                                        Foot_DoublePedalRequireReleaseBeforeRun(CHANNEL_A);
                                        ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                        return;

                                    }
                                    else
                                    {
                                        if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                                        {
                                            Foot_LatchCommonSocketMissingUntilRelease(); /* 左脚踏跨通道已触发缺刀具报警，必须松脚后才允许再次触发。 */
                                            Foot_ClearRunRequestAfterGateFail(); /* 左脚踏跨通道直接运行前发现公共接头无刀具，立即撤销脚踏请求。 */
                                            return;
                                        } /* 双踏板左侧跨通道直接运行前也要检查公共接头刀具头，避免绕过当前通道 gate。 */
                                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                                        {
                                            Foot_ClearRunRequestAfterGateFail(); /* 左脚踏跨通道未取得 owner 时，禁止保留上一次轻踩状态。 */
                                            return;
                                        }
                                        ControlSignalMessage.jtL_control_flag=true;
                                        WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue,msg.MValue_Left,msg.HValue_Left,0U); /* 双脚踏左侧跨通道运行同样按 EEPROM 最小速度起步。 */
                                        WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                                        Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板左侧跨通道启动后同样走压力锁存门禁，避免旧分支绕过停手柄保护。 */
                                    }
                            }
                        }
                        else
                        {
                              WorkMessage.switchhandle_counts=0;
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
                        }
                    }
                    else
                    {

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
                               ControlSignalMessage.jtL_gentlypump_flag=false;
                              if(pumpMessageA.type==INJECTWATER)
                              {
                               Foot_StopPumpAInjection();
                              }
                              else if(pumpMessageB.type==INJECTWATER)
                              {
                                Foot_StopPumpBInjection();
                               }
                             }
                         if(Foot_IsPedalReleased(jtd_adcvalue_r, msg.LValue_Right))
                         {
                             Foot_ClearPressureStopLatchAfterRelease(); /* 双脚踏必须左右两侧都回到释放区，才算退出本次压力停机控制源。 */
                             Foot_ClearManualSelectedAlarmOnRelease(); /* 双脚踏左右都释放后清除手控已选中的错模式报警。 */
                             Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 双脚踏左右都释放后，下一次左/右脚踏启动才允许重新弹 80。 */
                         }
                        //左脚停止
                     }
                    adValue_r=jtd_adcvalue_r;
                    if(adValue_r<msg.LValue_Right)adValue_r=msg.LValue_Right;
                    if(adValue_r-msg.LValue_Right>JT_threshold)//右踏板
                    {
                        if(Foot_BlockRunIfPressureStopLatched())
                        {
                            break; /* 双脚踏右侧保持踩下时如果压力锁存未释放，禁止蜂鸣结束后自动拉起手柄和泵。 */
                        }
                        if(Foot_BlockRunIfCommonSocketMissingLatched())
                        {
                            break; /* 双脚踏右侧缺刀具后保持踩下时，只保持停机，不再重复触发 80 弹窗。 */
                        }
                        ControlSignalMessage.jtR_control_flag=true;
                       if(Foot_EnsureFootControlMode() == false)
                       {
                           Foot_ClearRunRequestAfterGateFail(); /* 右脚踏门禁失败时清掉右侧运行请求，防止释放分支误判。 */
                           return;
                       }

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            Foot_ClearRunRequestAfterGateFail(); /* 外部控制中右脚踏不参与，清掉本周期脚踏请求。 */
                            return;//不参与
                        }

                       /* 轻踩阶段只预启动注水泵，不占用手柄电机 owner；真正启动电机前再申请 FOOT owner。 */

                       if(pumpMessageB.type==INJECTWATER)
                        {
                            /* 双踏板右侧轻排启动 B 注水泵，并确保 B 泵 run_flag 已打开。 */
                            Foot_StartPumpBInjection(pumpMessageB.speed_work);
                        }
                        else  if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
                        {
                             /* A 泵是注水泵时启动 A 泵，并确保 A 泵 run_flag 已打开。 */
                             Foot_StartPumpAInjection(pumpMessageA.speed_work);
                        }
                     if(adValue_r<msg.MValue_Right)adValue_r=msg.MValue_Right;
                      if(adValue_r-msg.MValue_Right>JT_threshold)
                        {
                            if(WorkMessage.channel_work==CHANNEL_B)
                            {
                               /* 双踏板右侧触发当前通道启动前先占用脚踏控制权。 */
                               if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                               {
                                   Foot_LatchCommonSocketMissingUntilRelease(); /* 右脚踏当前通道已触发缺刀具报警，等待松脚后才能再报警。 */
                                   Foot_ClearRunRequestAfterGateFail(); /* 右脚踏当前通道启动前发现缺刀具，报警后不保留运行边沿。 */
                                   return;
                               } /* 右踏板启动当前通道前检查公共接头刀具头是否已识别。 */
                               if(Foot_DoublePedalIsWaitingRelease(CHANNEL_B))
                               {
                                   ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                   return;
                               }
                               if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                               {
                                   Foot_ClearRunRequestAfterGateFail(); /* 右脚踏当前通道未取得 owner 时，不允许保持泵或运行标志。 */
                                   return;
                               }
                               ControlSignalMessage.jtR_control_flag=true;
                                WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue_r,msg.MValue_Right,msg.HValue_Right,FOOT_PEDAL_SPEED_HIGH_MARGIN); /* 双脚踏右侧当前通道保留高位死区，并限制最大不超过设定速度。 */
                                 WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                                Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板右侧启动 B 通道后统一刷新冷却泵跟随，并让压力堵塞锁存能够清回 runflag。 */
                            }
                            else if(WorkMessage.channel_work==CHANNEL_A)
                            {
                                    if(WorkMessage.Channel_Aonline==true)
                                    {
                                      if(WorkMessage.Channel_Bonline==true)
                                      {
                                            WorkMessage.switchhandle_countss++;
                                            if(WorkMessage.switchhandle_countss<FOOT_DOUBLE_PEDAL_SWITCH_DEBOUNCE_TICKS)
                                            {
                                                ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                                return;//未达到切换去抖计数，不执行
                                            }
                                            WorkMessage.switchhandle_countss=0;
                                            switch_before_channel = WorkMessage.channel_work; /* 记录切换前通道，后续用实际 channel_work 判断是否成功切到 B。 */
                                            HandleSwitchActive(SCREENKey_HANDLE_B);//切换
                                            if((switch_before_channel != CHANNEL_B) && (WorkMessage.channel_work == CHANNEL_B))
                                            {
                                                SendKeyBeepMessage(1U); /* 双脚踏右踏板跨通道切换 B 手柄成功，蜂鸣一次给操作者确认。 */
                                            }
                                            Foot_DoublePedalRequireReleaseBeforeRun(CHANNEL_B);
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;
                                        }
                                        else
                                        {
                                            if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)
                                            {
                                                Foot_LatchCommonSocketMissingUntilRelease(); /* 右脚踏跨通道已触发缺刀具报警，长踩期间不再重复请求启动。 */
                                                Foot_ClearRunRequestAfterGateFail(); /* 右脚踏跨通道直接运行前发现公共接头无刀具，立即撤销脚踏请求。 */
                                                return;
                                            } /* 双踏板右侧跨通道直接运行前也要检查公共接头刀具头，避免空刀具下发运行。 */
                                            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)
                                            {
                                                Foot_ClearRunRequestAfterGateFail(); /* 右脚踏跨通道未取得 owner 时，禁止残留脚踏运行标志。 */
                                                return;
                                            }
                                            ControlSignalMessage.jtR_control_flag=true;
                                            WorkMessage.speed_work=Foot_BuildTravelMotorSpeed(adValue_r,msg.MValue_Right,msg.HValue_Right,0U); /* 双脚踏右侧跨通道运行同样按 EEPROM 最小速度起步。 */
                                            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                                            Pubinterface_SetHandleInjectionPumpRun(true); /* 双踏板右侧跨通道启动后不能绕过联动接口，否则压力停机后脚踏保持会重新置运行。 */
                                        }

                                    }
                            }
                        }
                        else
                        {
                             WorkMessage.switchhandle_countss=0;
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
                        }
                    }
                    else
                    {
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
                              ControlSignalMessage.jtR_gentlypump_flag=false;
                              if(pumpMessageA.type==INJECTWATER)
                              {
                               Foot_StopPumpAInjection();
                              }
                              else if(pumpMessageB.type==INJECTWATER)
                              {
                               Foot_StopPumpBInjection();
                              }
                            }
                        if(Foot_IsPedalReleased(jtd_adcvalue_l, msg.LValue_Left))
                        {
                            Foot_ClearPressureStopLatchAfterRelease(); /* 双脚踏必须左右两侧都松开后才清压力锁存，防止另一侧仍踩住时自动恢复。 */
                            Foot_ClearManualSelectedAlarmOnRelease(); /* 双脚踏左右都释放后清除手控已选中的错模式报警。 */
                            Foot_ClearCommonSocketMissingLatchAfterRelease(); /* 双脚踏左右都释放后，解除公共接头缺刀具等待松脚锁存。 */
                        }
                     }

                break;
                default:
                break;;

            }
         }

    }


    /* 本周期如果脚踏相关电机/轻排输出都已经停下，就释放脚踏控制权给其它方式。 */
    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
}

/**
 * @brief 脚踏板任务初始化
 */



/*
 * 函数功能：周期读取脚踏串口数据，完成脚踏上线、掉线、定标值校验和脚踏状态解析。
 * 输入参数：event 为调度器传入的任务事件，本函数当前不依赖该值。
 * 返回参数：无。
 */
void Foot_ParseDataS(uint32_t event)//开个任务扫描预计10ms扫描一次
{

    uint8_t i;
    uint16_t rlen = 0;
    uint8_t dat[255] = { 0 };
    static uint8_t footconnect_flag = 0;
    //static uint8_t footconnect_times = 0;
    static volatile uint8_t footDisconnect_times = 0;


   static uint8_t first_connect_flag=0;
     static uint8_t double_connect_flag=0;

    //读取串口数据
    rlen = Uart4_DMARecvDataPeek(dat);//脚踏链接和退出200ms表示，这里循环20次
    if (rlen < 10)
    {   //不够一个数据包大小
               /* RFID 由 handlescan 在线监测统一触发，脚踏无数据时不能周期塞队列，避免刀具头拔掉后旧刀具信息无法清除。 */
        if((footconnect_flag != 0U) || (WorkAlarm_Is(WORK_ALARM_FOOT_VALUE_ERROR) != false))
        {
          footDisconnect_times++; /* 已上线脚踏或脚踏值错误报警都需要累计无数据周期，用于确认脚踏已经真正拔出。 */
          if(footDisconnect_times > 100) // 160*10ms=1600ms，超过1.6秒没有数据，认为脚踏掉线,这个地方判断一下，难道1秒6都不清零的么
          {
            footDisconnect_times=0;
            if(footconnect_flag != 0U)
            {
              footmessage.connect_flag=false;
              Foot_SendMessage(footmessage);//队列通知掉线
              //队列消息通知脚踏掉线
              SendKeyBeepMessage(1U);
            }
            footconnect_flag=0; /* 断开确认后统一回到未连接状态，下一次插入必须重新读取并校验脚踏存储值。 */
            first_connect_flag=0; /* 清掉单脚踏低值读取阶段，避免下次插入沿用上一次不完整的定标读取进度。 */
            double_connect_flag=0; /* 清掉双脚踏读取阶段，保证重新插入后从高/中/低值流程重新开始。 */
            Foot_ClearFootValueErrorAlarm(); /* 错误值脚踏拔出后没有新的有效帧，断开确认完成时清掉 83 号报警。 */
          }
        }
        return;
    }
    else
    {
        // 修改循环条件，防止数组越界
        for(i=0; i <= rlen - 10; i++)  // 使用加法形式的比较，避免下溢
        {
            if(i+9 >= rlen) break; // 额外保护，确保不会数组越界

            if(dat[i]==0xfE && dat[i+1]==0xEF)
            {
                footDisconnect_times=0;
                if(i+7 < rlen && dat[i+2]==0xb6 && dat[i+3]==0xc1 && dat[i+4]==0x01 && dat[i+5]==0x01){
                    jt_adcvalue=dat[i+6]<<8 | dat[i+7];
                    if(!footconnect_flag)
                    {
                       //发送获取低值的命令
                       if(!first_connect_flag){
                       Uart4_SendPacket(get_jtLvalue, 8);
                   }

                    }
                }
                else if(i+7 < rlen && dat[i+2]==0xd0 && dat[i+3]==0xb4 && dat[i+4]==0xb5 && dat[i+5]==0xcd)//脚踏低值
                {
                        first_connect_flag=1;
                    footmessage.LValue_Left=dat[i+6]<<8 | dat[i+7];

                    if(!double_connect_flag){
                    //判断脚踏高低值是否正确，若正确发送获取低值，不正确则通知队列报警
                     Uart4_SendPacket(get_jtHvalue, 8);//获取高
                    }

                 // return;
                }
                else if(i+7 < rlen && dat[i+2]==0xd0 && dat[i+3]==0xb4 && dat[i+4]==0xb8 && dat[i+5]==0xdf)//脚踏高值
                {
                     double_connect_flag=1;

                    footmessage.HValue_Left=dat[i+6]<<8 | dat[i+7];
                    if(Foot_IsPedalTwoPointStorageValid(footmessage.LValue_Left, footmessage.HValue_Left) == false)
                    {
                      first_connect_flag=0; /* 本次读到的低/高值无效，清掉低值阶段，等待下次重新读取完整定标值。 */
                      double_connect_flag=0; /* 高值阶段同样回到未完成状态，避免坏高值让脚踏误上线。 */
                      Foot_ReportFootValueErrorAlarm(); /* 脚踏存储值错误时弹出 83 号报警图，并锁住泵和手柄控制。 */
                      return; /* 坏定标值不能发送脚踏上线消息，避免后续任务使用错误行程计算速度。 */
                    }
                    Foot_ClearFootValueErrorAlarm(); /* 新读到的单踏板低/高值有效，允许清除历史脚踏值错误报警。 */
                    //判断脚踏高低值正确后，队列通知脚踏上线
                      footconnect_flag=1;
                      footmessage.connect_flag=true;
                      footmessage.pedalType=1;//JT
                      Foot_SendMessage(footmessage);
                        SendKeyBeepMessage(1U);
                     //通知相关队列脚踏已连接，脚踏型号
                }
                else if(i+5 < rlen && dat[i+2]==0xbb && dat[i+3]==0xaa)
                {
                   if(i+9 < rlen && dat[i+4]==0xdd)//AD值
                   {
                      if(dat[i+5]==0x01)//jtb
                      {
                         jtb_adcvalue=dat[i+6]<<8 | dat[i+7];//ad值
                        if(!footconnect_flag)
                        {
                            if(i+15 < rlen) // 确保读取 H/M/L 三组定标值时不会越界
                            {
                                footmessage.HValue_Left=dat[i+10]<<8  | dat[i+11];
                                footmessage.MValue_Left=dat[i+12]<<8 | dat[i+13];
                                footmessage.LValue_Left=dat[i+14]<<8 | dat[i+15];
                                if(Foot_IsPedalThreePointStorageValid(footmessage.LValue_Left, footmessage.MValue_Left, footmessage.HValue_Left) == false)
                                {
                                  Foot_ReportFootValueErrorAlarm(); /* JTB 低/中/高任一段无效时显示 83 号报警，并禁止控制泵和手柄。 */
                                  return; /* 不发送脚踏上线消息，避免错误中点把轻排段或电机段误触发。 */
                                }
                                Foot_ClearFootValueErrorAlarm(); /* JTB 三点定标恢复有效后，释放脚踏值错误报警锁存。 */
                                //判断脚踏值正确后，通知队列脚踏上线
                                //footconnect_flag=2;
                                footmessage.connect_flag=true;
                                footmessage.pedalType=2;//JTB
                                Foot_SendMessage(footmessage);
                                  SendKeyBeepMessage(1U);
                                  footconnect_flag=1;
                                //通知相关队列脚踏已连接，脚踏型号jtb
                            }
                        }
                        else return;
                      }
                      else if(i+9 < rlen && dat[i+5]==0x02)//jtd
                      {
                        jtd_adcvalue_l=dat[i+6]<<8 | dat[i+7];
                        jtd_adcvalue_r=dat[i+8]<<8 | dat[i+9];

                        if(!footconnect_flag)
                        {
                            if(i+21 < rlen) // 确保不会越界
                            {
                                footmessage.HValue_Left=dat[i+10]<<8 | dat[i+11];
                                footmessage.MValue_Left=dat[i+12]<<8 | dat[i+13];
                                footmessage.LValue_Left=dat[i+14]<<8 | dat[i+15];
                                footmessage.HValue_Right=dat[i+16]<<8 | dat[i+17];
                                footmessage.MValue_Right=dat[i+18]<<8 | dat[i+19];
                                footmessage.LValue_Right=dat[i+20]<<8 | dat[i+21];
                                if((Foot_IsPedalThreePointStorageValid(footmessage.LValue_Left, footmessage.MValue_Left, footmessage.HValue_Left) == false) ||
                                   (Foot_IsPedalThreePointStorageValid(footmessage.LValue_Right, footmessage.MValue_Right, footmessage.HValue_Right) == false))
                                {
                                  Foot_ReportFootValueErrorAlarm(); /* JTD 左/右任一路三点定标无效时显示 83 号报警，并禁止双脚踏控制。 */
                                  return; /* 不发送脚踏上线消息，避免错误通道继续控制泵或手柄。 */
                                }
                                Foot_ClearFootValueErrorAlarm(); /* JTD 左右两路三点定标都有效后，释放历史脚踏值错误报警。 */
                                //判断脚踏值正确后，队列通知脚踏上线
                                footmessage.connect_flag=true;
                                footmessage.pedalType=3;//JTd
                                Foot_SendMessage(footmessage);
                                  footconnect_flag=1;
                                  SendKeyBeepMessage(1U);
                                //通知相关队列脚踏已连接，脚踏型号jtd
                            }
                            else return;
                        }

                      }
                   }
                   else if(i+7 < rlen && dat[i+4]==0xcc)//按键
                   {
                      switch(dat[i+7])//队列通知行为
                      {
                         case 0x01://左键长按
                         SendKeyBehMessage(1,JTKey_left_long);
                           SendKeyBeepMessage(1U);
                         break;
                         case 0x02://右键长按
                          SendKeyBehMessage(1,JTKey_right_long);
                            SendKeyBeepMessage(1U);
                         break;
                         case 0x03://中键长按
                           SendKeyBehMessage(1,JTKey_middle_long);
                             SendKeyBeepMessage(1U);
                         break;
                         case 0x04://右键短按
                          SendKeyBehMessage(1,JTKey_right_short);
                            SendKeyBeepMessage(1U);
                         break;
                         case 0x05://左键短按
                             SendKeyBehMessage(1,JTkey_left_short);
                               SendKeyBeepMessage(1U);
                         break;
                         case 0x06://中键短按
                           SendKeyBehMessage(1,JTKey_middle_short);
                             SendKeyBeepMessage(1U);
                         break;
                         default:
                         break;
                      }
                   }
                }
               // memset(dat,0,sizeof(dat));

            }
            else
            {

            }
        }
       //  memset(dat,0,sizeof(dat));

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
