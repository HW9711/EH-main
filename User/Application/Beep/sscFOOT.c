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
    if(FootMsgQueue == NULL) return;
    (void)Kernel_QueueSend(FootMsgQueue, &msg, 0);
}

/**
 * @brief 初始化脚踏板消息队列
 */
static void Foot_Queue_Init(void)
{
    FootMsgQueue = Kernel_QueueCreate(5, sizeof(FootMessage_t), "FootMsgQueue");
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
    
   (void)event;
    //从消息队列获取消息
    if(FootMsgQueue != NULL)
    {
        FootMessage_t msg;
        if(Kernel_QueueReceive(FootMsgQueue, &msg, 0) == pdTRUE)
        {
            if(msg.connect_flag==false)
            {
                //通知界面，如果因为脚踏行为报警，则恢复
                 ControlSignalMessage.jt_enable_flag=false;
            }
        }
       if(msg.connect_flag==true)
         {
            ControlSignalMessage.jt_enable_flag=true;
            switch(msg.pedalType)
            {
                case 1://jt
                if(jt_adcvalue-msg.LValue_Left>JT_threshold)
                    {
                        //手柄运行
                        ControlSignalMessage.jtL_gentlypump_flag=true;
                        ControlSignalMessage.jtL_control_flag=true;
                        WorkMessage.speed_work=(jt_adcvalue-msg.LValue_Left)/(msg.HValue_Left-msg.LValue_Left)*WorkMessage.speed_set_work;
                    }
                    else
                    {
                        
                        if(ControlSignalMessage.jtL_control_flag)
                        {
                            //手柄停止//
                            ControlSignalMessage.jtL_gentlypump_flag=false;
                            ControlSignalMessage.jtL_control_flag=false;
                            WorkMessage.speed_work=0;
                        }
                    }
                break;
                case 2://jb
                if(jt_adcvalue-msg.LValue_Left>JT_threshold)//灌注标志进行，至于如何让那个泵运行，则要看泵的状态，以及泵的行为
                {
                    //通知泵运行
                    ControlSignalMessage.jtL_gentlypump_flag=true;//队列通知泵A，如果泵A是注水泵，优先泵A，如果泵B是注水泵则泵B，两边都不是则不运行
                }
                else
                {
                    if( ControlSignalMessage.jtL_gentlypump_flag)
                    {
                         ControlSignalMessage.jtL_gentlypump_flag=false;
                        //通知泵停止
                    }
                }
                if(jt_adcvalue-msg.MValue_Left>JT_threshold)
                {
                    //手柄运行
                    ControlSignalMessage.jtL_control_flag=true;
                  //  jtB_control_flag=1;
                     WorkMessage.speed_work=(jt_adcvalue-msg.MValue_Left)/(msg.HValue_Left-msg.MValue_Left)* WorkMessage.speed_set_work;
                }
                else
                {
                    if(ControlSignalMessage.jtL_control_flag==true)
                    {
                        //停止运行
                        ControlSignalMessage.jtL_control_flag=false;
                    }
                }

                break;
                case 3://jd
                break;
                default:
                break;;
            
            }
         }
         
    }
    
    
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
    static uint8_t footDisconnect_times = 0;

   
   static uint8_t first_connect_flag=0;

    //读取串口数据
    rlen = Uart4_DMARecvDataPeek(dat);//脚踏链接和退出200ms表示，这里循环20次
    if (rlen < 10) 
    {   //不够一个数据包大小
        if(footconnect_flag)
        {
          footDisconnect_times++;
          if(footDisconnect_times > 50)
          {
            footDisconnect_times=0;
            footconnect_flag=0;
            first_connect_flag=0;
            footmessage.connect_flag=false;
            Foot_SendMessage(footmessage);//队列通知掉线
            //队列消息通知脚踏掉线
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
                       first_connect_flag=1;
                       }
                      
                    }
                }
                else if(i+7 < rlen && dat[i+2]==0xd0 && dat[i+3]==0xb4 && dat[i+4]==0xb5 && dat[i+5]==0xcd)//脚踏低值
                {
                    footmessage.LValue_Left=dat[i+6]<<8 | dat[i+7];
                    
                    //判断脚踏高低值是否正确，若正确发送获取低值，不正确则通知队列报警
                     Uart4_SendPacket(get_jtHvalue, 8);//获取高
                  return;
                }
                else if(i+7 < rlen && dat[i+2]==0xd0 && dat[i+3]==0xb4 && dat[i+4]==0xb8 && dat[i+5]==0xdf)//脚踏高值
                {
                    footmessage.HValue_Left=dat[i+6]<<8 | dat[i+7];
                    //判断脚踏高低值是否正确，正确队列通知
                      footconnect_flag=1;
                      footmessage.connect_flag=true;
                      footmessage.pedalType=1;//JT
                      Foot_SendMessage(footmessage);
                     //通知相关队列脚踏已连接，脚踏型号
                }
                else if(i+5 < rlen && dat[i+2]==0xbb && dat[i+3]==0xaa)
                {
                   if(i+9 < rlen && dat[i+4]==0xdd)//AD值
                   {
                      if(dat[i+5]==0x01)//jtb
                      {
                        if(!footconnect_flag)
                        {
                            if(i+13 < rlen) // 确保不会越界
                            {
                                
                                footmessage.HValue_Left=dat[i+8]<<8  | dat[i+9];
                                footmessage.MValue_Left=dat[i+10]<<8 | dat[i+11];
                                footmessage.LValue_Left=dat[i+12]<<8 | dat[i+13];
                                //判断脚踏值是否正确，不正确请通知队列报警
                                  //footconnect_flag=2;
                                footmessage.connect_flag=true;
                                footmessage.pedalType=2;//JTB
                                Foot_SendMessage(footmessage);
                                //通知相关队列脚踏已连接，脚踏型号jtb
                            }
                        }
                        if(i+7 < rlen) // 确保不会越界
                        {
                            jtb_adcvalue=dat[i+6]<<8 | dat[i+7];//ad值
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
                                footmessage.pedalType=3;//JTB
                                Foot_SendMessage(footmessage);
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
                         break;
                         case 0x02://右键长按
                          SendKeyBehMessage(1,JTKey_right_long);
                         break;
                         case 0x03://中键长按
                           SendKeyBehMessage(1,JTKey_middle_long);
                         break;
                         case 0x04://右键短按
                          SendKeyBehMessage(1,JTKey_right_short);
                         break;
                         case 0x05://左键短按
                             SendKeyBehMessage(1,JTkey_left_short);
                         break;
                         case 0x06://中键短按
                           SendKeyBehMessage(1,JTKey_middle_short);
                         break;
                         default:
                         break;
                      }
                   }
                }
               // memset(dat,0,sizeof(dat));

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
    Kernel_TaskStart(&FOOTTaskHandle, KERNEL_TASK_ALWAYS, 10);

    Kernel_TaskCreate(&FOOTBHHandle, FootControlTask);
    Kernel_TaskStart(&FOOTBHHandle, KERNEL_TASK_ALWAYS, 50);
}
