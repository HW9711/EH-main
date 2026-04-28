#include <stdint.h>
#include <stdbool.h>

#define TMBB_ONLINES 1
#define TMBA_ONLINES 2
#define EMBA_ONLINES 3
#define EMBB_ONLINES 4
#define PXBA_ONLINES 5
#define PXBB_ONLINES 6
#define MX_YIM_ONLINES 7  //磨削 一体磨
#define MX_YIP_ONLINES 8  //磨削 一体刨
#define PX_YIM_ONLINES 9  //刨削 一体磨
#define PX_YIP_ONLINES 10 //刨削 一体刨
#define JMB_ONLINES    11 //
#define MX_YIM16_ONLINES    12 //


#define CHANNEL_A 1U
#define CHANNEL_B 2U

#define JTkey_left_short 1U //左键短按
#define JTKey_left_long 2U  //左键长按
#define JTKey_right_short 3U///右键短按
#define JTKey_right_long 4U//右键长按
#define JTKey_middle_short 5U//中间短按
#define JTKey_middle_long 6U//中间长按 ，总计预留3个按钮，分长按短按,该长按只针对单踏板，按钮，双踏板自动切换（通过脚踏值）
#define JTKey_Gently_left_start 7U//轻排开始
#define JTKey_Gently_left_stop 8U//轻排停止
#define JTKey_Gently_right_start 9U//轻排开始
#define JTKey_Gently_rigth_stop 10U//轻排停止

#define DRAWWATER    1U//抽水
#define INJECTWATER  2U//注水
#define POURWATER    3U//灌注

#define JTWORK      1U//脚踏工作
#define HANDLEWORK  2U//手控工作
#define TOUCHWORK     3U//外部工作


#define PLANER      1U//刨头
#define GRINDH      2U//磨头

#define ZZDIR 0U//正转
#define FZDIR 1U//反转
#define OSCDIR 2U//往复


#define JTKey        1U//脚踏调节按键
#define HANDLEKey    2U//手控调节按键
#define HMIkey       3U//外部调节按键
#define SCREENKey    4U//显示屏调节按键
#define PLUGunPLUG   5U//手柄插拔

// 手柄型号定义


#define PUMPGEAR_ZERO 0U//0档
#define PUMPGEAR_I    1U//一档
#define PUMPGEAR_II   2U//二档
#define PUMPGEAR_III  3U//三档
#define PUMPGEAR_IV   4U//四档
#define PUMPGEAR_V    5U//五档
#define PUMPGEAR_VI   6U//六档

#define HANDLEKey_speed_add 7U//速度加
#define HANDLEKey_speed_sub 8U//速度减
#define HANDLEKey_motor_start 9U//启动运行
#define HANDLEKey_motor_stop 10U//停止运行
#define HANDLEKey_dir_Forward 11U//正转
#define HANDLEKey_dir_Reverse 12U//手柄中间长按
#define HANDLEKey_dir_OSC 13U//往复转动
#define HANDLEKey_greaI	 14U//手柄一档
#define HANDLEKey_greaII 15U//手柄二挡
#define HANDLEKey_greaIII 16U//手柄三挡
#define HANDLEKey_greaIV 17U///手柄四档
#define HANDLEKey_greaV 18U//手柄五档，总计预留5个档位

#define HMIkey_APUMP_Add 19U//A泵加
#define HMIkey_APUMP_Sub 20U//A泵减
#define HMIkey_APUMP_control 21U//A泵控制按钮
#define HMIkey_BPUMP_Add 22U//B泵加
#define HMIkey_BPUMP_Sub 23U//B泵减
#define HMIkey_BPUMP_control 24U//B泵控制按钮

#define HMIkey_SPEED_Add 25U//速度加
#define HMIkey_SPEED_Sub 26U//速度减
#define HMIkey_FREQ_Add 27U//频率加
#define HMIkey_FREQ_Sub 28U//频率减

#define HMIkey_HANDLE_A 29U//手柄A
#define HMIkey_HANDLE_B 30U//手柄B

#define HMIkey_PlanerH 31U//刨头按钮
#define HMIkey_GrindH 32U//磨头按钮

#define HMIkey_Dir_Forward 33U//正转
#define HMIkey_Dir_Reverse 34U//反转
#define HMIkey_Dir_OSC 35U//往复

#define HMIkey_OpenPos_ClockWise 36U//开口定位顺时针
#define HMIkey_OpenPos_AntiClockWise 37U//开口定位逆时针
#define HMIkey_HMI_EXIT 38U//外部控制 20个按键值如上
#define  HMIkey_JTActi 64U///脚控激活
#define  HMIkey_HandleActi 65U///手控激活
// #define  HMIkey_TouchActi 66U///触控激活
// #define  HMIkey_TouchStart 67U///触控开始

// #define  HMIkey_Gently_left_start 68//轻排开始
// #define  HMIkey_Gently_left_stop 69//轻排停止
// #define  HMIkey_Gently_right_start 70//轻排开始
// #define  HMIkey_Gently_rigth_stop 71//轻排停止
#define  HMIkey_Gently_start 68U
#define  HMIkey_Gently_stop 69U
#define  HMIkey_TouchEXIT 72U///触控退出

#define SCREENKey_APUMP_Add 39U//A泵加
#define SCREENKey_APUMP_Sub 40U//A泵减
#define SCREENKey_APUMP_control 41U//A泵控制按钮
#define SCREENKey_BPUMP_Add 42U//B泵加
#define SCREENKey_BPUMP_Sub 43U//B泵减
#define SCREENKey_BPUMP_control 44U//B泵控制按钮

#define SCREENKey_SPEED_Add 45U//速度加
#define SCREENKey_SPEED_Sub 46U//速度减
#define SCREENKey_FREQ_Add 47U//频率加
#define SCREENKey_FREQ_Sub 48U//频率减

#define SCREENKey_HANDLE_A  49U//手柄A
#define SCREENKey_HANDLE_B  50U//手柄B
#define  SCREENKey_PLUG_A   70U//插入A手柄
#define  SCREENKey_PLUG_B   71U//插入B手柄
#define  SCREENKey_UNPLUG_A 72U//拔掉A手柄
#define  SCREENKey_UNPLUG_B 73U//拔掉B手柄

#define SCREENKey_PlanerH 51U//刨头按钮
#define SCREENKey_GrindH  52U//磨头按钮

#define SCREENKey_Dir_Forward 53U//正转
#define SCREENKey_Dir_Reverse 54U//反转
#define SCREENKey_Dir_OSC 55U//往复

#define SCREENKey_OpenPos_ClockWise 56U//开口定位顺时针
#define SCREENKey_OpenPos_AntiClockWise 57U//开口定位逆时针
#define SCREENKey_JTActi 58U//脚控激活
#define SCREENKey_HandleActi 59U//手控激活
#define SCREENKey_TouchActi 60U//触控激活
#define SCREENKey_TouchStart 61U///触控启动
#define SCREENKey_TouchEXIT 62U///触控退出
#define SCREENKey_HMI_EXIT 63U//外部控制退出，显示器共计25个按钮指令



#define UI_PUMPA_ID 1U//泵A区域
#define UI_PUMPB_ID 2U//泵B区域
#define UI_CONTROL_ID 3U//控制模式，脚踏，手控，外控，触控
#define UI_DIR_ID 4U  //方向 正向反向往复
#define UI_HANDLE_ID 5U//手柄显示
#define UI_TOOL_ID 6U//图片
#define UI_ORAL_ID 7U//开口定位
#define UI_FREQ_ID 8U//频率
#define UI_SPEED_ID 9U//速度
#define UI_AIARM_ID 10U//报警提示
#define UI_TOOLSPEC_ID 11U//刀具规格
#define UI_MANUALBUTTON_ID 12U//识别按钮(手动)
#define UI_PUMPAGEAR_ID 13U//A泵档位显示
#define UI_PUMPBGEAR_ID 14U//B泵档位显示
#define UI_PUMPABUTTON_ID 15U//A泵按钮显示
#define UI_PUMPBBUTTON_ID 16U//B泵按钮显示


typedef struct 
{
  volatile bool      runflag_work;//电机运行标志位，这里考虑
  volatile bool      alarm_flag;
  volatile uint8_t   alarm_value;//报警码，供蜂鸣、UI 和上位机统一读取
  volatile bool      Channel_Aonline;
  volatile bool      Channel_Bonline;
  volatile uint8_t   hand_model;
  volatile uint8_t   channel_work;//工作通道//1为a通道，2为b通道,3为双通道（基于配合使用前提下）
  volatile uint8_t   drivetype_work;//驱动方式(脚控，手控，触控)
  volatile uint8_t   hmiactive_work;//外部控制激活标志位
   volatile uint8_t   touchactive_work;//外部控制激活标志位
  volatile uint8_t   tool_type;
  volatile uint16_t  speed_work;//工作速度
  volatile uint16_t  speed_set_work;//设置速度
  volatile uint16_t  freq_work;//工作频率
  volatile uint16_t  dir_work;///工作方向
  volatile uint16_t  current_work;//工作电流
  volatile uint32_t  tool_reduction_ratio;//减速比 高16位表示增速16位表示减速
}
WorkMessage_t;
extern WorkMessage_t WorkMessage;//工作信息


typedef struct 
{
  volatile bool  jt_enable_flag;//脚踏启动flag-电机
  volatile bool  handle_enable_flag;//脚踏启动flag-电机
  volatile bool  HMI_enable_flag;//脚踏启动flag-电机
  volatile bool  jtL_control_flag;//脚踏启动flag-电机,左
  volatile bool  jtR_control_flag;//脚踏启动flag-电机，右
  volatile bool  handle_control_flag;//手控启动flag-电机
  volatile bool  HMI_control_flag;//外部控制-电机 控制左右，增对当前的手柄通道

  volatile bool  jtL_gentlypump_flag;//脚踏轻踩运行泵-左
  volatile bool  jtR_gentlypump_flag;//脚踏轻踩运行泵-右
  volatile bool  HMI_gentlypump_flag;//外控轻踩运行泵-外部控制

  volatile bool  JTSCREENL_pump_flag;//脚控启动泵-左
  volatile bool  JTSCREENR_pump_flag;//脚控启动泵-右
  volatile bool  HMIL_pump_flag;//外控启动泵
  volatile bool  HMIR_pump_flag;//外控启动泵

}ControlSignalMessage_t;
extern ControlSignalMessage_t ControlSignalMessage;//控制信号量


typedef struct {

  volatile uint8_t   hand_model;//手柄类型
  volatile uint8_t   hand_type_raw_major;//手柄EEPROM原始类型高字节，来自Page2第0字节，外部通信心跳在线时直接上传
  volatile uint8_t   hand_type_raw_minor;//手柄EEPROM原始类型低字节，来自Page2第1字节，外部通信心跳在线时直接上传
  volatile uint8_t  tool_type;//刨还是磨
  volatile uint8_t   drive_type;//驱动方式(脚控，手控，外控，触控)
  volatile uint16_t  zz_speed;//正传速度
  volatile uint16_t  fz_speed;//反传速度
  volatile uint16_t  osc_speed;//往复速度
  volatile uint16_t  freq;//工作频率
  volatile uint16_t  dir;///工作方向
  
  volatile uint16_t  current_work;
  volatile uint32_t  tool_reduction_ratio;//刀具减数比
}
ChannelMemoryMessagr_t;
extern ChannelMemoryMessagr_t MemoryMsgA;//通道记忆（增对可调节参数），用于切换手柄
extern ChannelMemoryMessagr_t MemoryMsgB;//
typedef struct 
{
  volatile bool      digital_enable;
	volatile bool      Pubadapter;//共用转接头0表示不是公共头1表示一体刀钻接头2表示其他的，接口留上
	volatile bool      dualDrive_Flag;//双驱协作flag
	volatile bool      dualDrive_Matchcode;//双驱协作匹配码，为后续双驱动留接口，当前板子用不上

	volatile uint8_t   freq_max;
	volatile uint8_t   freq_min;
	volatile uint8_t   freq_default;
  
	volatile uint8_t   draw;//角度
	volatile uint8_t   diameter;//直径
	volatile uint8_t   meioticratio;//减数比
  volatile uint16_t  speed_zzmax;
    volatile uint16_t  speed_zzmin;
    volatile uint16_t  speed_fzmax;
    volatile uint16_t  speed_fzmin;
      volatile uint16_t  speed_oscmax;
    volatile uint16_t  speed_oscmin;
   volatile uint16_t speed_zzstep;//正向步进速度
    volatile uint16_t speed_fzstep;//正向步进速度
   volatile uint16_t speed_oscstep;//正向步进速度
  volatile uint16_t speed_zzdefault;//速度
  volatile uint16_t speed_fzdefault;//速度
  volatile uint16_t speed_oscdefault;//速度
	volatile uint16_t  speed_min;//速度
	volatile uint16_t  speed_max;//速度
	volatile uint16_t  length;///长度
	volatile uint16_t  overloadThresholdFor;//过载阀值（正）
	volatile uint16_t  overloadThresholdRev;//过载阀值（反）
	volatile uint16_t  overloadThresholdOSC;//过载阀值（往复）
	volatile uint8_t   handle_type;//手柄类型
    volatile uint8_t   run_direction;//运行方向
    volatile uint8_t   control_mode;//控制模式
    volatile uint8_t   tool_type;//刀具类型	
}
ChannelrecognizeMessage_t;//通道数据结构体(手柄，刀具识别内容)
extern ChannelrecognizeMessage_t ChannelrecognizeMessageA;
extern ChannelrecognizeMessage_t ChannelrecognizeMessageB;



typedef struct 
{
  volatile bool     online_flag;//泵在线
  volatile bool     run_flag;//泵开始
   volatile uint16_t timingDrainage_times;//定时排空时间，100ms任务中累计时需要大于1位
  volatile bool     timingDrainage_flag;//定时排空，优先级在run_flag运行后，遇到run_flag=true则切为false
  volatile uint8_t  step_value;//步进值
  volatile uint8_t  associated_channel;//关联通道，如果
  volatile uint16_t  type;//泵类型设备码
  volatile uint8_t  direction;//方向
  volatile uint16_t speed_work;//泵速度，灌注最大到300ml，必须使用16位避免截断
  volatile uint16_t speed_Max;
  volatile uint16_t speed_Min;
  volatile uint16_t speed_step_value;//调节步进值一次多少ml，快速档位先不管，后续在判断最大值最小值之间分为6档进行分配
  volatile uint8_t  losses_times;//识别丢失次数
  volatile int32_t  pressure_value;//压力传感器原始值，对应下位机 RawCs1237
  volatile uint16_t pressure_threshold;//压力阀值，由压力模块提供
  volatile uint32_t weight_x10;//重量值，单位 0.1g，对应下位机 WeightX10
  volatile uint8_t  seq;//压力模块上报帧序号

}
pumpMessage_t;
extern pumpMessage_t pumpMessageA;//泵A结构体
extern pumpMessage_t pumpMessageB;//泵B结构体



typedef struct 
{
  bool JTing_FLAG;//jt控制
  bool JTBing_FLAG;//jtb控制
  bool JTDLing_FLAG;//jtd左踏板控制
  bool JTDRing_FLAG;//jtd右踏板控制

  bool Handleing_FLAG;//手柄控制
  bool HMIing_FLAG;//外部控制控制

  bool JTApumpGently_FLAG;//A泵轻排控制
  bool JTBpumpGently_FLAG;//B泵轻排控制
}
ControlSigleMessage_t;
extern ControlSigleMessage_t ControlSigleMssage;

void ChannelMessageInit(void);
void WorkMessageInit(void);
void ChannelMemoryMessageInit(void);
void PUMPActive(uint8_t key_value);
void ToolPosActive(uint8_t key_value);//刀具开孔位置行为
void HandleSwitchActive(uint8_t key_value);
void SpeedActive(uint8_t key_value);
void DirActive(uint8_t key_value);
void FreqActive(uint8_t key_value);
void HmiExitActive(uint8_t key_value);
void ControlTypeActive(uint8_t key_value);
void PlanerGridH(uint8_t key_value);
 void ChannelrecognizeMessageInit(void);
void ChannelFlagMessageInit(void);
void pumpMessageInit(void);
void PlugORunPLUGActive(uint8_t key_value);
