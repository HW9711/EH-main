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
#include "sscRFID.h"


#define JT_threshold  20U

static uint8_t get_jtHvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB8,0Xdf,0x3e,0x84};//读高值

static uint8_t get_jtLvalue[10]={0XFE,0XEF,0XB6,0XC1,0XB5,0XCD,0xA3,0x00};//读低值

static uint16_t jt_adcvalue = 0;
static uint16_t jtb_adcvalue = 0;
static uint16_t jtd_adcvalue_l = 0;
static uint16_t jtd_adcvalue_r = 0;



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
    /* 队列消息只同步类型和速度；真正是否输出仍由 run_flag 统一控制。 */
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
    /* 队列消息只同步类型和速度；真正是否输出仍由 run_flag 统一控制。 */
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

static void Foot_SaveFootControlMode(void)
{
    /* 脚踏准备接管时，先把当前运行方式切到 JTWORK，后续旧逻辑才能继续走脚踏启动分支。 */
    WorkMessage.drivetype_work=JTWORK;
    /* A 通道正在工作时同步通道记忆，避免下一次通道切换后又恢复成外控或手控。 */
    if(WorkMessage.channel_work==CHANNEL_A)
    {
        MemoryMsgA.drive_type=JTWORK;
    }
    /* B 通道正在工作时同步通道记忆，保持两路手柄的控制方式记忆一致。 */
    else if(WorkMessage.channel_work==CHANNEL_B)
    {
        MemoryMsgB.drive_type=JTWORK;
    }
}

static void Foot_ClearHandleOrOverloadAlarm(void)
{
    /* 脚踏释放动作只清手柄未连接/电机过载这两类可恢复提示，不能误清 UID、通讯、HALL 等故障。 */
    if(WorkAlarm_Is(WORK_ALARM_HANDLE_NOT_CONNECTED) || WorkAlarm_Is(WORK_ALARM_MOTOR_OVERLOAD))
    {
        WorkAlarm_Clear();
    }
}

static bool Foot_EnsureFootControlMode(void)
{
    /* 已经处于脚踏模式时直接放行，不重复刷新 UI 和记忆值。 */
    if(WorkMessage.drivetype_work==JTWORK)
    {
        return true;
    }

    /* 没有选中任何有效通道时，脚踏不能盲目启动电机或泵，仍按手柄未连接处理。 */
    if(WorkMessage.channel_work==CHANNEL_NONE)
    {
        WorkAlarm_Set(WORK_ALARM_HANDLE_NOT_CONNECTED);
        return false;
    }

    /* 外部控制仍有效时不能切入脚踏，必须等上位机退出并释放互斥控制权。 */
    if(ControlArbitration_IsExternalActive() || ControlSignalMessage.HMI_control_flag || (WorkMessage.touchactive_work==TOUCHWORK))
    {
        return false;
    }

    /* 其它本地方式正在持有控制权时不抢占；只有空闲后第一次踩脚踏才允许切换为脚踏模式。 */
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

    /* 互斥锁空闲且外控已退出时，脚踏第一次动作就是新的控制来源，直接切到脚踏模式。 */
    Foot_SaveFootControlMode();
    /* 刷新脚踏选中显示，让屏幕状态和实际控制来源一致。 */
    DisPlayData[0]=1U;
    DisPlayData[1]=1U;
    SendUIDSMessage(UI_CONTROL_ID, 1U, DisPlayData);
    return true;
}

/**
 * @brief 获取解析后的数据
 * @return 解析数据指针
 */

/**
 * @brief 清除按键状态
 */


/**
 * @brief 脚踏板控制任务
 */
void FootControlTask(uint32_t event)
{
    uint16_t adValue;
    uint16_t adValue_r;
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
    pumpMessageA.type=INJECTWATER;
    //从消息队列获取消息
    if(FootMsgQueue != NULL)
    {
         static FootMessage_t msg;
        if(Kernel_QueueReceive(FootMsgQueue, &msg, 0) == pdTRUE)
        {
            if(msg.connect_flag==false)
            {
                 SendUIDSMessage(UI_CONTROL_ID, 0U, DisPlayData);//脚踏暗黑显示
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
                         DisPlayData[0]=2U;//手控显示
                         DisPlayData[1]=1U;//手控在线选中显示
                         SendUIDSMessage(UI_CONTROL_ID, 1U, DisPlayData);//脚踏暗黑显示
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
             }
            else
            {
                ControlSignalMessage.jt_enable_flag=true;
               if(WorkMessage.drivetype_work!=HANDLEWORK&&WorkMessage.drivetype_work!=TOUCHWORK)//如果没有手控，没有点开触控，外控除外，即便外控在控制运行中，也可以显示脚踏选中
               {
                 DisPlayData[0]=1U;//脚控ID
                  DisPlayData[1]=1U;//手控在线选中显示
                 SendUIDSMessage(UI_CONTROL_ID, 1U, DisPlayData);//脚踏选中显示
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
                    DisPlayData[0]=0U;//脚踏连接显示，保持上次控制功能不变，只刷新脚踏在线状态。
                    SendUIDSMessage(UI_CONTROL_ID, 1U, DisPlayData);//显示任务会拷贝 10 字节数据，这里必须传有效缓冲区。
               }
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
                        if(Foot_EnsureFootControlMode() == false)return;
                        //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                           // SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            return;//不参与
                        }
                       /* 脚踏即将启动联动泵或电机，先占用脚踏控制权，避免其它来源同时下发控制。 */
                       if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                       if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
                        {
                            /* 单踏板左侧脚踏默认以 70 启动 A 注水泵，并同步打开 A 泵 run_flag 门控。 */
                            Foot_StartPumpAInjection(70U);
                        }
                        else if(pumpMessageB.type==INJECTWATER)
                        {
                            /* B 泵作为注水泵时沿用当前设置速度启动，同时打开 B 泵 run_flag 门控。 */
                            Foot_StartPumpBInjection(pumpMessageB.speed_work);
                        }
                        WorkMessage.speed_set_work=60000;
                        /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                        ControlSignalMessage.jtL_control_flag=true;
                        WorkMessage.speed_work=(float)(jt_adcvalue-msg.LValue_Left)/(float)(msg.HValue_Left-msg.LValue_Left)*WorkMessage.speed_set_work;
                        WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                    }
                    else
                    {
                        if(ControlSignalMessage.jtL_control_flag)
                        {
                            WorkMessage.runflag_work=false;

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
                            //本来就是停止，如果不是脚踏控制的那么不需要任何处理和操作
                        }
                    }
                break;
                case 2://jb
                adValue=jtb_adcvalue;
                if(adValue<msg.LValue_Left)adValue=msg.LValue_Left;
                if(adValue-msg.LValue_Left>JT_threshold)//注水标志进行，至于如何让那个泵运行，则要看泵的状态，以及泵的行为
                {
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

                        /* 脚踏即将启动联动泵或电机，先占用脚踏控制权，避免其它来源同时下发控制。 */
                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;

                        if(pumpMessageA.type==INJECTWATER)//事实上不准备给外部控制提供改轻排按钮
                        {
                             /* A 泵是注水泵时只能启动 A 泵；旧代码误写 B 泵会导致脚踏踩下后目标泵不转。 */
                             Foot_StartPumpAInjection(70U);
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
                       WorkMessage.speed_set_work=60000;
                    /* 脚踏真正启动电机前占用脚踏控制权，当前来源结束前其它方式不能接管。 */
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

                       if(Foot_EnsureFootControlMode() == false)return;

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            return;//不参与
                        }

                        /* 脚踏即将启动联动泵或电机，先占用脚踏控制权，避免其它来源同时下发控制。 */
                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;

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
                                if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                                ControlSignalMessage.jtL_control_flag=true;
                                WorkMessage.speed_work=(float)(adValue-msg.MValue_Left)/(float)(msg.HValue_Left-msg.MValue_Left)*WorkMessage.speed_set_work;
                                WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                            }
                            else if(WorkMessage.channel_work==CHANNEL_B)
                            {
                                    if(WorkMessage.Channel_Aonline==true)
                                    {
                                        WorkMessage.switchhandle_counts++;
                                        if(WorkMessage.switchhandle_counts<10)
                                        {
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;//小于500ms，不执行，该功能记得清零
                                        }
                                         WorkMessage.switchhandle_counts=0;
                                        SendKeyBehMessage(JTKey,JTKey_middle_long);//切换
                                        if(!WorkMessage.switchhandleA_flag)
                                        {
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;
                                        }
                                        /* 双踏板自动切换完成后再占用脚踏控制权，避免切换等待期锁住其它来源。 */
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

                         WorkMessage.switchhandle_counts=0;
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
                       if(Foot_EnsureFootControlMode() == false)return;

                         //其他控制中不允许执行改动作
                        if(ControlSignalMessage.HMI_control_flag)//外部控制中-无效 解除控制才往下执行
                        {
                            //可以滴一下声音
                            //SendKeyBeepMessage(1);//滴一声,（这里带考虑，显示请停止其他控制，再控制-代码逻辑不强制关闭运行，只是提示应该有提示）
                            return;//不参与
                        }

                       /* 脚踏即将启动联动泵或电机，先占用脚踏控制权，避免其它来源同时下发控制。 */
                       if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;

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
                               if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                               ControlSignalMessage.jtL_control_flag=true;
                                WorkMessage.speed_work=(float)(adValue_r-msg.MValue_Right)/(float)(msg.HValue_Right-msg.MValue_Right)*WorkMessage.speed_set_work;
                                WorkMessage.runflag_work=true;//通知SSCdrive电机运行
                            }
                            else if(WorkMessage.channel_work==CHANNEL_A)
                            {
                                    if(WorkMessage.Channel_Aonline==true)
                                    {
                                        WorkMessage.switchhandle_counts++;
                                        if(WorkMessage.switchhandle_counts<10)
                                        {
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;//小于500ms，不执行，该功能记得清零
                                        }
                                         WorkMessage.switchhandle_counts=0;
                                        SendKeyBehMessage(JTKey,JTKey_middle_long);//切换
                                        if(!WorkMessage.switchhandleA_flag)
                                        {
                                            ControlArbitration_ExitLocalControlIfIdle(CONTROL_OWNER_FOOT);
                                            return;
                                        }
                                        /* 双踏板自动切换完成后再占用脚踏控制权，避免切换等待期锁住其它来源。 */
                                        if(ControlArbitration_TryEnter(CONTROL_OWNER_FOOT) == false)return;
                                        ControlSignalMessage.jtL_control_flag=true;
                                        WorkMessage.speed_work=(adValue_r-msg.MValue_Right)/(msg.HValue_Right-msg.MValue_Right)*WorkMessage.speed_set_work;
                                        WorkMessage.runflag_work=true;//通知SSCdrive电机运行

                                    }
                            }
                        }
                        else
                        {
                             WorkMessage.switchhandle_counts=0;
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
                        WorkMessage.switchhandle_counts=0;
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
               SendKeyRFIDMessageAup(1U);
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
           // SendKeyRFIDMessageAup(1U);
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
              //  SendKeyRFIDMessageAdown();
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
