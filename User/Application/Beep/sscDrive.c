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
#define MOTOR_DRIVE_CMD_FREQ_MAX 100U /* 驱动私有协议第 2 字节允许 0~100，超过上限时必须钳位，避免异常频率触发驱动保护。 */

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

static uint8_t MotorDrive_BuildCommandFrequency(uint16_t freq_work)
{
    if (freq_work > MOTOR_DRIVE_CMD_FREQ_MAX)
    {
        return (uint8_t)MOTOR_DRIVE_CMD_FREQ_MAX; /* EEPROM 或上位机给出的频率超过协议范围时，按驱动允许的最大值下发。 */
    }

    return (uint8_t)freq_work; /* 参考驱动接收端会再执行 `R_DATA[2] * 2`，主控这里保持原始命令值，不再提前翻倍。 */
}

/*
 * 函数功能：判断当前通道是否需要按有刷一体刨/一体磨协议下发驱动帧。
 * 输入参数：hand_model 为 EEPROM 第二页识别出的手柄型号；tool_type 为归一后的业务刀具类型；raw_tool_type 为 EEPROM/RFID 原始刀具型号。
 * 返回参数：1 表示按有刷电机通道下发；0 表示按无刷/霍尔通道下发。
 */
static uint8_t MotorDrive_IsBrushedTool(uint8_t hand_model, uint8_t tool_type, uint8_t raw_tool_type)
{
    return (uint8_t)((hand_model == PX_YIP_ONLINES) ||
                     (hand_model == PX_YIM_ONLINES) ||
                     (tool_type == PX_YIP_ONLINES) ||
                     (tool_type == PX_YIM_ONLINES) ||
                     (raw_tool_type == PX_YIP_ONLINES) ||
                     (raw_tool_type == PX_YIM_ONLINES)); /* tool_type 归一为 PLANER/GRINDH 后，仍用 raw_tool_type 保留 PXM/PXP 有刷判定。 */
}

static uint8_t MotorDrive_BuildBrushlessRunType(uint8_t hand_model)
{
    if ((hand_model == PXBB_ONLINES) || (hand_model == PXBA_ONLINES))
    {
        return 0x02U; /* PXBA/PXBB 是带霍尔往复手柄，驱动闭环方式固定走方波霍尔。 */
    }

    return 0x01U; /* 其他手柄默认按无霍尔方式下发，保持旧工程的兼容行为。 */
}

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
    /* msg.pro_current_h/l 已在 MOTORRUN() 中由 WorkMessage.current_work 拆分，发送前不能再改写，否则会覆盖 EEPROM/上位机设置的保护电流。 */
    uint8_t motor_startcode[motor_frem_length]={0xAA ,msg.control_mode ,msg.frequency ,msg.motor_type\
         ,msg.speed_h ,msg.speed_l ,msg.run_type ,msg.pro_current_h ,msg.pro_current_l ,0xBB ,0xAA};
    Uart1_SendPacket(motor_startcode, motor_frem_length);
}
/*
 * 函数功能：根据当前 WorkMessage 运行态组装电机驱动帧，向 UART1 电机驱动板下发启动或停止命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void MOTORRUN(void)
{
    static uint8_t huci=0;
    uint8_t display_value[10]={0};
    if(WorkMessage.tool_reduction_ratio==0)WorkMessage.tool_reduction_ratio=1;
    if(WorkMessage.auto_identify==0)WorkMessage.tool_reduction_ratio=1;
    uint32_t ssc_speed_value=WorkMessage.speed_set_work*WorkMessage.tool_reduction_ratio/10;//显示速度使用设定速度，保持与切通道时一致，避免运行中调速显示跳变；实际下发驱动的速度仍使用 WorkMessage.speed_work，保持控制和反馈的分离，以及与驱动协议的兼容。
    if(WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES)
    {
        if(WorkMessage.tool_type==PLANER)
        {
            if(ssc_speed_value<1500)
          ssc_speed_value=ssc_speed_value*1.3;
          if(ssc_speed_value<500)
          ssc_speed_value=500;
        }
    }
   
    if(WorkMessage.runflag_work)
    {
        if(huci==0)
        {
            display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位沿用 WorkMessage 的 x10 速度。 */
            display_value[1] = (uint8_t)((WorkMessage.speed_set_work >> 8)&0xFFU); /* 速度低字节按 UIDP 协议传输，保证 16 位速度完整显示。 */
         display_value[2] = (uint8_t)(WorkMessage.speed_set_work & 0xFFU);  
            display_value[3] = 1U;             
            display_value[4] = 1U; 
            huci=1;
	SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 最后刷新速度值，保证切通道后的速度显示同步。 */
     LCD_Show_2byte_Number(0x9473,0xffE0);
        }
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
             msg.frequency=MotorDrive_BuildCommandFrequency(WorkMessage.freq_work);//参考驱动内部会再乘2，这里只下发协议原值
             break;
             default:
             break;
        }
        if(WorkMessage.channel_work==1)//通道1
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x01;
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model);
                
            }
            else{
             msg.motor_type=0x03;
              msg.run_type=0x03;
            }
        }
        else if(WorkMessage.channel_work==2)//通道2
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x02;
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model);
            }
            else 
            { msg.motor_type=0x04;
                 msg.run_type=0x04;
            }
        }
      WorkMessage.current_work=0xffff;
      msg.speed_h=ssc_speed_value/256;
      msg.speed_l=(ssc_speed_value)%256;//速度
      msg.pro_current_h=WorkMessage.current_work/256;
      msg.pro_current_l=WorkMessage.current_work%256;//电流
      MotorStart();
    }
    else
    {
         if(huci==1){
            huci=0;
            display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位沿用 WorkMessage 的 x10 速度。 */
            display_value[1] = (uint8_t)((WorkMessage.speed_set_work >>8)& 0xFFU); /* 速度低字节按 UIDP 协议传输，保证 16 位速度完整显示。 */
             display_value[2] = (uint8_t)(WorkMessage.speed_set_work & 0xFFU);  
            display_value[3] = 1U; 
            display_value[4] = 0U; 
            SendUIDSMessage(UI_SPEED_ID, true, display_value); /* 最后刷新速度值，保证切通道后的速度显示同步。 */
       
	        LCD_Show_2byte_Number(0x9473,0xffff);

         }
       MotorStops();
    }

}

/*
 * 函数功能：电机输出周期任务，按当前 WorkMessage 下发启动/停止帧，并刷新本地电机仲裁释放。
 * 输入参数：event 调度器传入的任务事件值，当前任务不使用该参数。
 * 返回参数：无。
 */
void MOTORRUNTask(uint32_t event) 
{ 
    /* 当前任务不按 event 分支处理，显式丢弃参数避免后续误解。 */
    (void)event;
    /* 根据 runflag_work、方向、通道、速度和保护电流组帧，向 UART1 电机驱动板下发命令。 */
    MOTORRUN();
    /* 每个电机输出周期检查 Page4 速度/频率阈值，只触发蜂鸣提示，不强制停止电机输出。 */
    Pubinterface_CheckSpeedThresholdAlarm();
    /* 每个电机输出周期刷新本地控制权，确保停止命令和驱动反馈都归零后再允许其它模式接管。 */
    ControlArbitration_RefreshMotorOwner();
}
void SscDriveMotorTask_Init(void)
{
   
  /* definition and creation of MOTOR123Task */
	Kernel_TaskCreate(&MOTORRUNTaskHandle, MOTORRUNTask);
	Kernel_TaskStart(&MOTORRUNTaskHandle, KERNEL_TASK_ALWAYS, 50);
}
