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
#include "sscUIDP.h"

#define motor_frem_length  11
#define MOTOR_DRIVE_CMD_FREQ_MAX 100U /* 驱动私有协议第 2 字节允许 0~100，超过上限时必须钳位，避免异常频率触发驱动保护。 */
#define MOTOR_DRIVE_RATIO_UNIT 1U /* 机械变速倍率为 1 时表示屏幕速度和电机速度一致。 */
#define MOTOR_DRIVE_RATIO_X10_UNIT 10U /* 内部完整齿轮比按 x10 保存，50 表示 5.0 倍。 */
#define MOTOR_DRIVE_CMD_SPEED_UNIT_RPM 10U /* GE2433 启动帧速度字段单位为 10rpm，3000rpm 需要下发 300。 */
#define MOTOR_DRIVE_CMD_SPEED_MAX 0xFFFFU /* GE2433 启动帧速度字段只有 16 位，超过时必须钳位。 */
#define MOTOR_DRIVE_RPM_MAX (MOTOR_DRIVE_CMD_SPEED_MAX * MOTOR_DRIVE_CMD_SPEED_UNIT_RPM) /* 电机实际 rpm 的协议可表达上限。 */
#define MOTOR_DRIVE_SPEED_UP_SHIFT 16U /* WorkMessage.tool_reduction_ratio 高 16 位表示增速比。 */
#define MOTOR_DRIVE_REDUCTION_MASK 0xFFFFU /* WorkMessage.tool_reduction_ratio 低 16 位表示减速比。 */
#define MOTOR_DRIVE_DISPLAY_SPEED_INVALID 0xFFFFFFFFUL /* 速度显示缓存的无效值，用于强制下一次运行刷新屏幕速度。 */
#define MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM 100U /* 脚踏实时速度只按 100rpm 整数档刷新屏幕，实际电机速度仍保留完整精度。 */

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
 * 函数功能：把内部 x10 倍率或历史整数倍率换算成减速机构需要的电机速度。
 * 输入参数：motor_speed 为已从屏幕 x10 速度恢复后的 rpm；ratio 为低 16 位减速比。
 * 返回参数：换算后的电机 rpm，未做 16 位钳位。
 */
static uint32_t MotorDrive_ApplyReductionRatioValue(uint32_t motor_speed, uint32_t ratio)
{
    if (ratio >= MOTOR_DRIVE_RATIO_X10_UNIT)
    {
        return (motor_speed * ratio) / MOTOR_DRIVE_RATIO_X10_UNIT; /* 内部倍率按 x10 保存，50 表示 5.0 倍减速，电机端速度需要乘 5.0。 */
    }

    return motor_speed * ratio; /* 兼容历史直接写入的整数倍率，2 表示 2 倍减速。 */
}

/*
 * 函数功能：把内部 x10 倍率或历史整数倍率换算成增速机构需要的电机速度。
 * 输入参数：motor_speed 为已从屏幕 x10 速度恢复后的 rpm；ratio 为高 16 位增速比。
 * 返回参数：换算后的电机 rpm，倍率异常时返回原速度。
 */
static uint32_t MotorDrive_ApplySpeedUpRatioValue(uint32_t motor_speed, uint32_t ratio)
{
    if (ratio >= MOTOR_DRIVE_RATIO_X10_UNIT)
    {
        return (motor_speed * MOTOR_DRIVE_RATIO_X10_UNIT) / ratio; /* 内部倍率按 x10 保存，50 表示 5.0 倍增速，电机端速度需要除以 5.0。 */
    }

    if (ratio > MOTOR_DRIVE_RATIO_UNIT)
    {
        return motor_speed / ratio; /* 兼容历史直接写入的整数倍率，2 表示 2 倍增速。 */
    }

    return motor_speed; /* 0 或 1 表示无有效增速，保护为原速度。 */
}

/*
 * 函数功能：按 EEPROM/RFID 解析出的机械减速比或增速比，把屏幕手柄速度换算成电机输出速度。
 * 输入参数：display_speed 为本次控制源目标速度，脚踏模式取行程比例后的 speed_work，其它模式取设定最大速度 speed_set_work。
 * 返回参数：需要写入 UART1 电机启动帧的速度值，已完成倍率换算和 16 位钳位。
 */
static uint32_t MotorDrive_ApplyToolReductionRatio(uint32_t display_speed)
{
    uint32_t ratio = WorkMessage.tool_reduction_ratio;        /* 当前通道记忆装载的完整倍率，高 16 位增速、低 16 位减速。 */
    uint32_t reduction_ratio = ratio & MOTOR_DRIVE_REDUCTION_MASK; /* 低 16 位减速比，减速机构需要放大电机速度。 */
    uint32_t speed_up_ratio = ratio >> MOTOR_DRIVE_SPEED_UP_SHIFT; /* 高 16 位增速比，增速机构需要降低电机速度。 */
    uint32_t motor_speed = display_speed;                     /* 设定速度已经是实际 rpm，例如 6000 表示 6000rpm，倍率换算前不能再除以 10。 */

    if ((reduction_ratio > MOTOR_DRIVE_RATIO_UNIT) &&
        (speed_up_ratio > MOTOR_DRIVE_RATIO_UNIT))
    {
        reduction_ratio = MOTOR_DRIVE_RATIO_UNIT;             /* 双倍率同时有效属于 EEPROM 写入错误，保护为无减速。 */
        speed_up_ratio = 0U;                                  /* 同时清增速分支，避免错误标签让电机速度不可预测。 */
    }

    if (reduction_ratio > MOTOR_DRIVE_RATIO_UNIT)
    {
        motor_speed = MotorDrive_ApplyReductionRatioValue(motor_speed, reduction_ratio); /* 减速机构：正常内部 50 表示 5.0 倍，历史值 2 仍兼容为 2 倍。 */
    }
    else if (speed_up_ratio > MOTOR_DRIVE_RATIO_UNIT)
    {
        motor_speed = MotorDrive_ApplySpeedUpRatioValue(motor_speed, speed_up_ratio); /* 增速机构：正常内部 50 表示 5.0 倍，历史值 2 仍兼容为 2 倍。 */
    }

    if (motor_speed > MOTOR_DRIVE_RPM_MAX)
    {
        motor_speed = MOTOR_DRIVE_RPM_MAX;                    /* 实际 rpm 超过 GE2433 协议可表达上限时先钳位，后续再按 /10 写入速度字段。 */
    }

    return motor_speed;                                       /* 返回本次启动帧希望电机达到的实际 rpm，组帧前还要按协议 /10。 */
}

/*
 * 函数功能：把脚踏实时速度向下量化为 100rpm 整数倍，仅用于屏幕显示和刷新判重。
 * 输入参数：actual_speed 为脚踏任务计算出的完整精度实际目标速度，单位为 rpm。
 * 返回参数：不大于实际目标速度的 100rpm 整数倍；输入小于 100rpm 时返回 0。
 */
static uint32_t MotorDrive_QuantizeFootDisplaySpeed(uint32_t actual_speed)
{
    return (actual_speed / MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM) * MOTOR_DRIVE_FOOT_DISPLAY_STEP_RPM; /* 只截掉百位以下数值，不回写 WorkMessage，也不影响电机驱动帧。 */
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
    // uint8_t motor_startcode[motor_frem_length]={0xAA ,0x03 ,0x28 ,0x03\
    //      ,0x01 ,0x90 ,0x03 ,0x00 ,0x2D ,0xBB ,0xAA};    
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
    static uint32_t last_display_speed=MOTOR_DRIVE_DISPLAY_SPEED_INVALID; /* 记录脚踏运行时上一次发给屏幕的实时速度，避免每 50ms 无变化也刷屏。 */
    uint8_t display_value[10]={0};
    uint8_t effective_dir_work=0U; /* 保存本次真正下发给驱动板的方向，EMBD 只在输出层取反，避免改写屏幕和通道记忆。 */
    uint32_t motor_source_speed=WorkMessage.speed_set_work; /* 非脚踏控制时，屏幕/EEPROM 当前设定速度就是电机运行目标速度。 */
    uint32_t display_speed_value=WorkMessage.speed_set_work; /* 非脚踏控制时，屏幕继续显示用户设定的目标速度。 */
    uint32_t ssc_speed_value=0U; /* 保存倍率换算后的电机实际 rpm，后续再按 GE2433 协议除以 10 下发。 */
    uint16_t command_speed_value=0U; /* 保存写入 GE2433 启动帧 byte4~5 的协议速度字段，单位为 10rpm。 */
    /* 脚踏控制使用实时行程速度；其它控制方式继续使用屏幕或 EEPROM 设定速度。 */
    if(WorkMessage.drivetype_work==JTWORK)
    {
        motor_source_speed=WorkMessage.speed_work; /* 脚踏带行程霍尔，speed_work 已由脚踏任务按踩踏比例实时换算。 */
        display_speed_value=MotorDrive_QuantizeFootDisplaySpeed(WorkMessage.speed_work); /* 屏幕只显示 100rpm 整数倍；电机仍使用上方未量化的完整速度。 */
    }
    ssc_speed_value=MotorDrive_ApplyToolReductionRatio(motor_source_speed); /* 本次控制源速度统一按 EEPROM/RFID 倍率换算成电机实际 rpm。 */
    // if(WorkMessage.hand_model==PX_YIP_ONLINES) /* 仅 PXYTP 临时启用 5 倍减速验证，避免影响其它手柄和后续 EEPROM 正式方案。 */
    // {
    //     ssc_speed_value=WorkMessage.speed_set_work*5U; /* PXYTP 机械端自带 5 倍减速，屏幕仍显示刀具端目标速度，电机端下发速度需要放大 5 倍。 */
    //     if(ssc_speed_value>0xFFFFU) /* 电机驱动协议速度只有高低 2 字节，临时放大后必须避免回绕成异常低速。 */
    //     {
    //         ssc_speed_value=0xFFFFU; /* 超过驱动帧可表达范围时按最大值下发，保证验证过程不会因溢出误判。 */
    //     }
    // }
    /* PXBA/PXBB 分体手柄需要保留原刨刀低速补偿，其它手柄直接使用倍率换算后的速度。 */
    if(WorkMessage.hand_model==PXBA_ONLINES||WorkMessage.hand_model==PXBB_ONLINES)
    {
        /* 只有刨刀在低速下需要补偿启动扭矩，磨头不得进入该速度修正。 */
        if(WorkMessage.tool_type==PLANER)
        {
            /* 低于 1500rpm 时按原规则提高 30%，改善刨刀低速启动能力。 */
            if(ssc_speed_value<1500)
          ssc_speed_value=ssc_speed_value*1.3;
          /* 补偿后仍低于 500rpm 时钳位到最小可驱动速度，避免电机只响不转。 */
          if(ssc_speed_value<500)
          ssc_speed_value=500;
        }
    }
   
    /* 电机运行标志有效时组装并发送启动帧；无效时进入下方停止帧分支。 */
    if(WorkMessage.runflag_work)
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
        effective_dir_work=(uint8_t)WorkMessage.dir_work; /* 默认按当前工作方向下发，保证其它手柄完全沿用原方向逻辑。 */
        if(WorkMessage.hand_model==EMBD_ONLINES) /* EMBD 现场电机实际方向与协议方向相反，只针对该手柄在驱动帧前取反。 */
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
        if(WorkMessage.channel_work==1)//通道1
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x01; /* A 通道无刷手柄使用驱动协议的 0x01 电机类型，保证命令送到 A 路无刷控制分支。 */
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model); /* 无刷运行类型由手柄型号换算，保留各型号原有驱动方式。 */
                
            }
            else{
             msg.motor_type=0x03; /* A 通道有刷手柄固定使用协议类型 0x03，不能与 B 通道的 0x04 混用。 */
              msg.run_type=0x03; /* 有刷 A 路的运行类型与电机类型保持一致，驱动板据此进入 A 路有刷控制。 */
            }
        }
        else if(WorkMessage.channel_work==2)//通道2
        {
            if(MotorDrive_IsBrushedTool(WorkMessage.hand_model, WorkMessage.tool_type, WorkMessage.raw_tool_type) == 0U)
            {
                msg.motor_type=0x02; /* B 通道无刷手柄使用驱动协议的 0x02 电机类型，保证命令送到 B 路无刷控制分支。 */
                msg.run_type=MotorDrive_BuildBrushlessRunType(WorkMessage.hand_model); /* 无刷运行类型仍按手柄型号换算，通道选择只影响物理驱动路。 */
            }
            else 
            { msg.motor_type=0x04; /* B 通道有刷手柄固定使用协议类型 0x04，不能与 A 通道的 0x03 混用。 */
                 msg.run_type=0x04; /* 有刷 B 路的运行类型与电机类型保持一致，驱动板据此进入 B 路有刷控制。 */
            }
        }
      command_speed_value=MotorDrive_BuildCommandSpeed(ssc_speed_value); /* 最终输出给电机前按 GE2433 协议把实际 rpm 转为 rpm/10 字段。 */
      msg.speed_h=command_speed_value/256;
      msg.speed_l=(command_speed_value)%256;//速度
      msg.pro_current_h=WorkMessage.current_work/256; /* 保护电流来自手柄 EEPROM Page4[21..22]，单位 0.01A；0 表示驱动板使用内部默认保护。 */
      msg.pro_current_l=WorkMessage.current_work%256;//电流
      MotorStart();
    }
    else
    {
         /* 只有上一周期确实处于运行显示态时才恢复停止颜色，避免每 50ms 重复刷屏。 */
         if(huci==1){
            huci=0;
            last_display_speed=MOTOR_DRIVE_DISPLAY_SPEED_INVALID; /* 停机后清显示缓存，下次起转必须重新同步运行速度。 */
            display_value[0] = (uint8_t)(WorkMessage.speed_set_work >> 16); /* 速度高字节按 UIDP 协议传输，单位为 WorkMessage 的实际 rpm。 */
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
