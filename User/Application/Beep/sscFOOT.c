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

static uint8_t get_jtHvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB8,0Xdf,0x3e,0x84};//读高值

static uint8_t get_jtLvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB5,0XCD,0xA3,0x00};//读低值

static uint16_t jt_adcvalue = 0;
static uint16_t jtb_adcvalue = 0;
static uint16_t jtd_adcvalue_l = 0;
static uint16_t jtd_adcvalue_r = 0;
static uint8_t s_double_pedal_release_before_run_channel = CHANNEL_NONE;



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
               if(WorkMessage.drivetype_work!=HANDLEWORK&&WorkMessage.drivetype_work!=TOUCHWORK)//如果没有手控，没有点开触控，外控除外，即便外控在控制运行中，也可以显示脚踏选中
               {
                 WorkMessage.drivetype_work=JTWORK;//是否要更新记忆值
                  if(WorkMessage.channel_work==CHANNEL_A)
                    {
                    MemoryMsgA.drive_type=JTWORK;//添加记忆功能
                    }
                    else if(WorkMessage.channel_work==CHANNEL_B)
                    {
                    MemoryMsgB.drive_type=JTWORK;
                    }
               }
               else
               {
                    /* 脚踏在线但未取得控制权时，统一刷新在分支结束处执行，避免重复入队。 */
               }
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
                            ControlSignalMessage.jtL_control_flag=true;
                        /* 单踏板重新确认踩下后清掉松脚去抖计数，防止上一轮释放残留影响本次泵保持运行。 */
                        single_release_debounce_ticks=0U;
                        if(Foot_EnsureFootControlMode() == false)return;
                        //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                           // SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            return;//不参与
                        }
                       /* 单踏板按“手柄运行才联动注水泵”处理，先不预启动泵，避免 owner 获取失败时泵单独转。 */
                        if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
                        /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
                       // if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)return; /* 脚踏启动手柄电机前检查公共接头 EPC 刀具头，缺失时只报警不运行。 */
                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                    
                        WorkMessage.speed_work=(float)(jt_adcvalue-msg.LValue_Left)/(float)(msg.HValue_Left-msg.LValue_Left)*WorkMessage.speed_set_work;
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
                        }
                        else
                        {
                            /* 未处于脚踏运行态时不需要保留释放计数，避免下次启动前误认为已经连续松脚。 */
                            single_release_debounce_ticks=0U;
                            //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
                        }
                    }
                break;
                case 2://jb
                adValue=jtb_adcvalue;
                if(adValue<msg.LValue_Left)adValue=msg.LValue_Left;
                if(adValue-msg.LValue_Left>JT_threshold)//注水标志进行，至于如何让那个泵运行，则要看泵的状态，以及泵的行为
                {
                    ControlSignalMessage.jtL_control_flag=true;
                    //   if(WorkMessage.alarm_flag!=0)
                    //     {
                    //         //报警状态不允许脚踏任何动作
                    //          return;
                    //     }
                       if(Foot_EnsureFootControlMode() == false)return;
                          //其他控制中不允许执行改动作(外部控制中，触摸控制中，手控控制中，脚踏右键控制中)
                        if(ControlSignalMessage.HMI_control_flag)
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声，一直执行一直响，响声短。（是否界面提示）
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
                }
                if(adValue<msg.MValue_Left)adValue=msg.MValue_Left;
                if(adValue-msg.MValue_Left>JT_threshold)
                {
                     //手柄运行
                       if(WorkMessage.speed_set_work==0U)WorkMessage.speed_set_work=Pubinterface_GetCurrentDefaultMotorSpeed();//当前手柄还未装载速度时，使用 Page4 默认速度替代旧固定 60000
                    /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
                    // if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)return; /* 脚踏比例启动前先确认刀具头参数有效，避免公共接头空刀具运行。 */
                    if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                    ControlSignalMessage.jtL_control_flag=true;
                    WorkMessage.speed_work=(float)(adValue-msg.MValue_Left)/(float)(msg.HValue_Left-msg.MValue_Left)* WorkMessage.speed_set_work;
                    WorkMessage.runflag_work=true;//通知SSCdrive电机运行
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
                        ControlSignalMessage.jtL_control_flag=true;

                       if(Foot_EnsureFootControlMode() == false)return;

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
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
                                //if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)return; /* 双踏板当前通道启动前执行公共接头刀具头 gate。 */
                                if(Foot_DoublePedalIsWaitingRelease(CHANNEL_A))
                                {
                                    ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                    return;
                                }
                                if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                                ControlSignalMessage.jtL_control_flag=true;
                                WorkMessage.speed_work=(float)(adValue-msg.MValue_Left)/(float)(msg.HValue_Left-msg.MValue_Left-30)*WorkMessage.speed_set_work;
                                WorkMessage.runflag_work=true;//通知SSCdrive电机运行
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
                                        HandleSwitchActive(SCREENKey_HANDLE_A);//切换
                                        Foot_DoublePedalRequireReleaseBeforeRun(CHANNEL_A);
                                        ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                        return;

                                    }
                                    else
                                    {
                                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                                        ControlSignalMessage.jtL_control_flag=true;
                                        WorkMessage.speed_work=(float)(adValue-msg.MValue_Left)/(float)(msg.HValue_Left-msg.MValue_Left)*WorkMessage.speed_set_work;
                                        WorkMessage.runflag_work=true;//通知SSCdrive电机运行
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
                       //左脚停止
                    }
                    adValue_r=jtd_adcvalue_r;
                    if(adValue_r<msg.LValue_Right)adValue_r=msg.LValue_Right;
                    if(adValue_r-msg.LValue_Right>JT_threshold)//右踏板
                    {
                        ControlSignalMessage.jtR_control_flag=true;
                       if(Foot_EnsureFootControlMode() == false)return;

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
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
                               //if(Pubinterface_CheckCommonSocketToolReadyForRun() == false)return; /* 右踏板启动当前通道前检查公共接头刀具头是否已识别。 */
                               if(Foot_DoublePedalIsWaitingRelease(CHANNEL_B))
                               {
                                   ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                   return;
                               }
                               if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                               ControlSignalMessage.jtR_control_flag=true;
                                WorkMessage.speed_work=(float)(adValue_r-msg.MValue_Right)/(float)(msg.HValue_Right-msg.MValue_Right-30)*WorkMessage.speed_set_work;
                                WorkMessage.runflag_work=true;//通知SSCdrive电机运行
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
                                            HandleSwitchActive(SCREENKey_HANDLE_B);//切换
                                            Foot_DoublePedalRequireReleaseBeforeRun(CHANNEL_B);
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;
                                        }
                                        else
                                        {
                                            if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                                             ControlSignalMessage.jtR_control_flag=true;
                                            WorkMessage.speed_work=(float)(adValue_r-msg.MValue_Right)/(float)(msg.HValue_Right-msg.MValue_Right)*WorkMessage.speed_set_work;
                                            WorkMessage.runflag_work=true;//通知SSCdrive电机运行
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



//数据解析
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
        if(footconnect_flag)
        {
          footDisconnect_times++;
          if(footDisconnect_times > 100) // 160*10ms=1600ms，超过1.6秒没有数据，认为脚踏掉线,这个地方判断一下，难道1秒6都不清零的么
          {
            footDisconnect_times=0;
            footconnect_flag=0;
            first_connect_flag=0;
            double_connect_flag=0;
            footmessage.connect_flag=false;
            Foot_SendMessage(footmessage);//队列通知掉线
            //队列消息通知脚踏掉线
              SendKeyBeepMessage(1U);
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
                    //判断脚踏高低值是否正确，正确队列通知
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
                            if(i+13 < rlen) // 确保不会越界
                            {
                                footmessage.HValue_Left=dat[i+10]<<8  | dat[i+11];
                                footmessage.MValue_Left=dat[i+12]<<8 | dat[i+13];
                                footmessage.LValue_Left=dat[i+14]<<8 | dat[i+15];
                                //判断脚踏值是否正确，不正确请通知队列报警
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
                                //判断一下，脚踏值是否错误，如果有错误，队列通知报警
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
