#include "sscDRIVE.h"
#include "data.h"


#include "lcd.h"
#include "common.h"
#include "iwdg.h"

#include <stdint.h>
#include "kernel_scheduler.h"
#include "lcd.h"
#include "uart1.h"
#include "Pubinterface.h"

#define motor_frem_length  11

kernel_task_t MOTORRUNTaskHandle;
static uint8_t motor_stopcode[motor_frem_length]={0xAA ,0x01 ,0x00 ,0x01 ,0x00 ,0x00 ,0x02 ,0x00 ,0x00  ,0xBB ,0xAA};

typedef struct {
  
    uint8_t control_mode; //控制模式[0x01]正转；[0x02]:反转；[0x03]:往复正反转;[0x04]:正向拖动模式；[0x05]:反向拖动模式
    uint8_t frequency; //20对应1hz,80对于4hz
    uint8_t motor_type; //电机选择：0x01：无刷通道1  0x02:无刷通道2 0x03:有刷通道1 0x04 有刷通道2
    uint8_t speed_h; //转速高
    uint8_t speed_l; //转速低
    uint8_t run_type; //闭环运行方式：0x01无霍尔 0x02 有霍尔 0x03:有刷刀头1  0x04:有刷刀头2  
    uint8_t pro_current_h; //保护电流高
    uint8_t pro_current_l; //保护电流低
    
}
RunInformMessage_t;
static RunInformMessage_t msg;

/// 开口定位
void ToolPosMay(uint8_t channel_number,bool direction,uint8_t angel)//通道，方向，角度
{
 uint8_t cmd[11] ={0xAA ,0x04 ,0x00 ,0x01 ,0x00 ,0x01 ,0x02 ,0x00 ,0x00 ,0xBB ,0xAA };
 channel_number==1?(cmd[3]=1):(cmd[3]=2);
 direction==true?(cmd[1]=4):(cmd[1]=5);
 cmd[5]=angel;
 Uart1_SendPacket(cmd,motor_frem_length);
}


void MotorStops(void)
{
 Uart1_SendPacket(motor_stopcode, motor_frem_length);
}
void MotorStart()
{
    msg.pro_current_l=0x64;
    uint8_t motor_startcode[motor_frem_length]={0xAA ,msg.control_mode ,msg.frequency ,msg.motor_type\
         ,msg.speed_h ,msg.speed_l ,msg.run_type ,msg.pro_current_h ,msg.pro_current_l ,0xBB ,0xAA};
    Uart1_SendPacket(motor_startcode, motor_frem_length);
}
void MOTORRUN(void)
{
    
    if(WorkMessage.runflag_work)
    {
        //msg的数据填充
        switch(WorkMessage.dir_work)
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
             msg.frequency=WorkMessage.freq_work*2;//2倍值
             break;
             default:
             break;
        }
        if(WorkMessage.channel_work==1)//通道1
        {
            if(WorkMessage.hand_model!=PX_YIP_ONLINES&&WorkMessage.hand_model!=PX_YIM_ONLINES)
            {
                msg.motor_type=0x01;
                if(WorkMessage.hand_model==PXBB_ONLINES||WorkMessage.hand_model==PXBA_ONLINES)
                msg.run_type=0x02;
                else
                msg.run_type=0x01;
                
            }
            else{
             msg.motor_type=0x03;
              msg.run_type=0x03;
            }
        }
        else if(WorkMessage.channel_work==2)//通道2
        {
            if(WorkMessage.hand_model!=PX_YIP_ONLINES&&WorkMessage.hand_model!=PX_YIM_ONLINES)
            {
                msg.motor_type=0x02;
                if(WorkMessage.hand_model==PXBB_ONLINES||WorkMessage.hand_model==PXBA_ONLINES)
                msg.run_type=0x02;
                else
                msg.run_type=0x01;
            }
            else 
            { msg.motor_type=0x04;
                 msg.run_type=0x04;
            }
        }
      msg.speed_h=WorkMessage.speed_work/2560;
      msg.speed_l=(WorkMessage.speed_work/10)%256;//速度
      msg.pro_current_h=WorkMessage.current_work/256;
      msg.pro_current_l=WorkMessage.current_work%256;//电流
      MotorStart();
    }
    else
    {
       MotorStops();
    }

}

void MOTORRUNTask(uint32_t event) 
{ 
    MOTORRUN();
}
void SscDriveMotorTask_Init(void)
{
   
  /* definition and creation of MOTOR123Task */
	Kernel_TaskCreate(&MOTORRUNTaskHandle, MOTORRUNTask);
	Kernel_TaskStart(&MOTORRUNTaskHandle, KERNEL_TASK_ALWAYS, 50);
}
