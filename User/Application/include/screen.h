//screen.h

#ifndef __SCREEN_H
#define __SCREEN_H

#include <stdint.h>
#include "stm32f4xx_hal.h"



#define   Sports_mode_NO 1
#define   Sports_mode_forward_select_p_P   1  //刨模式（刨） 正向
#define   Sports_mode_reverse_select_p_P    1 //刨模式 反向
#define   Sports_mode_osc_select_p_P       1  //刨模式 往复

#define   Sports_mode_forward_select_p_M_M  1   //刨 、磨模式 （磨） 正向
#define   Sports_mode_reverse_select_p_M_M  1    //刨、磨模式 （磨） 反向
#define   Sports_mode_reverse_select_M_W    1  //磨模式  往复

#define TmbbMaxSpeed   60000
#define TmbbMinSpeed   10000
#define TmbbSlowlyStepSpeed  2000
#define TmbbFastlyStepSpeed  5000

#define TmbaMaxSpeed   70000
#define TmbaMinSpeed   10000
#define TmbaSlowlyStepSpeed  2000
#define TmbaFastlyStepSpeed  5000

#define EmbaMaxSpeed   70000
#define EmbaMinSpeed   10000
#define EmbaSlowlyStepSpeed  2000
#define EmbaFastlyStepSpeed  5000

#define EmbbMaxSpeed   70000
#define EmbbMinSpeed   10000
#define EmbbSlowlyStepSpeed  2000
#define EmbbFastlyStepSpeed  5000

#define PXDXMaxSpeed   13000
#define PXDXMinSpeed   3000

#define PXDXMaxSpeed_P   6000
#define PXDXMinSpeed_P   500


#define PXDXSlowlyStepSpeed  1000
#define PXDXFastlyStepSpeed  2000

#define PXWFMaxSpeed   6000
#define PXWFMinSpeed   500
#define PXWFSlowlyStepSpeed  500
#define PXWFFastlyStepSpeed  1000


#define IrrigateMax       300  
#define IrrigateMin       0   
#define IrrigateStep  		30//步进值30

#define InjectionMax      70  
#define InjectionMin       0   
#define InjectionStep  		10//步进值10

#define FreqMax      40   //4.0HZ  
#define FreqMin       0   
#define FreqStep  		5//步进值0.5hz

#define Direction_forward     2      //正转
#define Direction_reverse     1      //反转
#define Direction_osc         3      //往复

#define footcontrol           1   	//脚控
#define handelcontrol         2     //手控
#define touchcontrol          3     //触控
#define noControl             4     //无控制

#define  start_flag     1
#define  stop_flag      0


 



#define beep_time 8 



//one region
#define A_Handel_Key  1         //A手柄按键返回值

#define A_Handel_adrr 1              //A手柄地址
#define A_Handel_NO   1              //无手柄图标
#define A_Handel_M_selected 1        //磨钻手柄已选中图标
#define A_Handel_M_connect 2         //磨钻手柄已连接图标
#define A_Handel_F_selected 1        //分体手柄已选中图标
#define A_Handel_F_connect 2         //分体手柄已连接图标
#define A_Handel_Y_selected 1 			 //一体手柄已选中图标
#define A_Handel_Y_connect 2         //一体手柄已连接图标



#define B_Handel_Key  1          //B手柄按键返回值

#define B_Handel_adrr 1               //B区地址
#define B_Handel_NO   1               //无手柄图标
#define B_Handel_M_selected 1         //磨钻手柄已选中图标
#define B_Handel_M_connect 2          //磨钻手柄已连接图标
#define B_Handel_F_selected 1         //分体手柄已选中图标
#define B_Handel_F_connect 2          //分体手柄已连接图标
#define B_Handel_Y_selected 1         //一体手柄已选中图标
#define B_Handel_Y_connect 2          //一体手柄已连接图标

#define Text_Spec_addr   1             //刀具规格地址

#define ManualRecognition_key   1       //手动识别按键值
#define ManualRecognition_addr  2       //手动识别地址
#define ManualRecognition_start 1      //手动识别启动
#define ManualRecognition_close 1      //手动识别关闭

#define OpeningPosition_addr 1//开口定位  
#define OpeningPosition_icon 1

#define ModeSlect_M_P_addr 3                //模式选择地址  

#define ModeSlect_M_key   1    					//磨头按键返回值
#define ModeSlect_M_select 3            //模式选择(磨头)

#define ModeSlect_P_key   1             //刨刀按键返回值
#define ModeSlect_P_select 3            //模式选择（刨刀）

//one region


//two region
#define WaterInjection_data_addr 1        //注水数据地址

#define WaterInjection_addr 1             //注水图标地址
#define WaterInjection_start 1            //注水启用
#define WaterInjection_close 1            //注水关闭
#define WaterInjection_add_key 1          //+按键
#define WaterInjection_minus_key 2        //-按键

#define WaterInjection_drain_addr 1       //注水 排空 地址
#define WaterInjection_drain_start 1 			//注水 排空 开 图标
#define WaterInjection_drain_stop  1      //注水 排空 关 图标
#define WaterInjection_drain_close  1      //注水 排空 暗黑
#define WaterInjection_drain_key 1        //注水 排空 关 按键值
//two region       



//three region

#define WaterIrrigate_data_addr 1        //灌水数据地址

#define WaterIrrigate_addr  1             //灌水图标地址
#define WaterIrrigate_start 1             //灌水图标启用
#define WaterIrrigate_close 1             //灌水图标关闭

#define WaterIrrigate_add_key 1          //+按键
#define WaterIrrigate_minus_key 2        //-按键

#define WaterIrrigate_drain_addr 1       //灌水 排空 地址
#define WaterIrrigate_drain_start 1 		 //灌水 排空 开 图标
#define WaterIrrigate_drain_stop  1      //灌水 排空 关 图标
#define WaterIrrigate_drain_close  1      //灌水 排空暗黑图标

#define WaterIrrigate_drain_key 1        //灌水 排空 关 按键值
//three region


//four  region
#define   Speed_data_addr 1               //数据地址
#define   Speed_addr  1         					//速度地址
#define   Speed_activation        1        //速度激活图标
#define   Speed_lazy              1  			//速度未激活图标
#define   Speed_Quickly_add_key   1       //快加
#define   Speed_Slowly_add_key    1       //慢加
#define   Speed_Quickly_minus_key  1      //快减
#define   Speed_Slowly_minus_key  1       //慢减

#define   Sports_mode_addr 1               //运动模式图标地址
#define   Sports_mode_forward_key        1  //正转按键
#define   Sports_mode_reverse_key       1    //反转按键
#define   Sports_mode_osc_key      			1	 //往复按键

#define   Sports_mode_NO 1
#define   Sports_mode_forward_select_p_P   1  //刨模式（刨） 正向
#define   Sports_mode_reverse_select_p_P    1 //刨模式 反向
#define   Sports_mode_osc_select_p_P       1  //刨模式 往复

#define   Sports_mode_forward_select_p_M_M  1   //刨 、磨模式 （磨） 正向
#define   Sports_mode_reverse_select_p_M_M  1    //刨、磨模式 （磨） 反向
#define   Sports_mode_reverse_select_M_W    1  //磨模式  往复



#define   Frequency_data_addr  1  //频率数据地址
#define   Frequency_addr       1  //频率图标地址
#define   Frequency_activation 1 //频率激活图标
#define   Frequency_lazy       1 //频率未激活图标
#define   Frequency_add_key    1 //频率 加 键值
#define   Frequency_minus_key  1 //频率 减 键值
//four  region



//five  region


#define Control_model_JSC_sclect_J 1 //脚控手控触控，选择脚控
#define Control_model_JSC_sclect_S 1 //脚控手控触控，选择手控
#define Control_model_JSC_sclect_C 1 //脚控手控触控，选择触控
#define Control_model_JC_sclect_J 1 //脚控触控，选择脚控
#define Control_model_JC_sclect_C 1 //脚控触控，选择C控
#define Control_model_SC_sclect_C 1 //脚控触控，选择C控
#define Control_model_SC_sclect_S 1 //脚控触控，选择S控
#define Control_model_C_sclect_C 1 //脚控触控，选择S控
#define Control_model_JSC_sclect_W 1 //脚控触控，选择无控制

#define Control_model_addr  1       //控制模式图标

#define Control_model_F_sclect 1    //脚控已选中
#define Control_model_F_key   1    //脚控按键值

#define Control_model_H_sclect 1    //手控已选中
#define Control_model_H_key    1    //手控按键值

#define Control_model_C_sclect 1    //触控已选中
#define Control_model_C_key    1    //触控按键值

#define Control_model_ZF_sclect 1    //只有脚控触控选择手控
#define Control_model_ZC_sclect 1    //只有脚控触控选择触控

#define Control_model_SH_sclect 1    //只有手控触控选择手控
#define Control_model_SC_sclect 1    //只有手控触控选择触控

#define Control_model_NO      1    //无选择模式

//five  region







//uint16_t storageInjectionValue[4]={0};//TMBB,TMBA,EMBA,EMBB
//uint32_t storageSpeedValue[4]={0};//TMBB,TMBA,EMBA,EMBB
extern uint32_t paoxueSpeciValue_A[4];//直径，长度，角度
extern uint32_t paoxueSpeciValue_B[4];//直径，长度，角度
extern uint8_t paoxueSpeciValue_F[16];

extern volatile uint8_t KeyBeep_flag;


typedef struct
{
	volatile uint8_t Handel_icon;
//	volatile uint8_t speed_icon;				//速度栏图标
//	volatile uint8_t pump_icon;					//泵流量图标
//	volatile uint8_t pump_button_icon; 	//泵操作按钮图标
	volatile uint8_t sports_icon;				//运动图标
	volatile uint8_t frequency_icon;		//频率图标
	volatile uint8_t manualRecognition_icon; //手动识别是否开启识别flag
	volatile uint8_t mode_M_P_Slect_icon; //磨头还是刨刀
	volatile uint8_t control_model_icon;//控制模式图片
}UIShow;
	
typedef struct
{
	volatile uint8_t Irrigatepump_icon;
	volatile uint8_t Irrigatepump_button_icon;
	volatile uint8_t Injectionpump_icon;
	volatile uint8_t Injectionpump_button_icon;
	volatile uint8_t speed_icon;
	volatile uint8_t frequency_icon;
	volatile uint8_t control_model_icon;
	UIShow A_Show;
	UIShow B_Show;
}UIShow_S;
	
extern UIShow_S UIShow_s;

typedef struct
{
	volatile uint8_t manualRecognition_flag;//手动模式是否开启
	volatile uint8_t mode_M_P_Slect_mode;//分体模式选择（磨头、刨刀）
	volatile uint8_t control_Slect_mode;//控制模式（手控、脚控，触控）
	volatile uint8_t sports_Slect_mode;// 运动模式（正转，反转，往复）
	
	
	volatile uint8_t waterIrrigate_flag;//灌注模式开关
	volatile uint8_t waterIrrigate_drain_flag;//灌水按钮暗黑
	volatile uint8_t waterInjection_flag;//注水模式开关
	volatile uint8_t waterInjection_drain_flag;//注水按钮暗黑
	
	
	volatile uint8_t A_Handel_mode;//A区手柄模式
	volatile uint8_t B_Handel_mode;//B区手柄模式
	volatile uint8_t HandleselectionChannel;//手柄选中当前通道,0无通道，1A通道，2B通道
	volatile uint8_t Handel_switch_flag;//手柄切换信号
	
}UIStateControl;

extern UIStateControl UIStateControl_s;



typedef struct
{
	volatile uint8_t waterIrrigate_data;	//灌注流量
	volatile uint8_t waterInjection_data;  //注水流量
	//灌注默认值(开机第一次eeprom读取)
//	volatile uint8_t waterInjection_data_TMBA;					//注水默认值TMBA（开机第一次eeprom读取）
//	volatile uint8_t waterInjection_data_TMBB;					//TMBB（eeprom读取）
//	volatile uint8_t waterInjection_data_EMBA;					//EMBA（eeprom读取）
//	volatile uint8_t waterInjection_data_EMBB;					//EMBB（eeprom读取）
//	volatile uint8_t waterInjection_data_YIMX_M;				// 一体磨削-磨
//	volatile uint8_t waterInjection_data_YIMX_P;				//一体磨
//	volatile uint8_t waterInjection_data_PX;            //分体刨(射频读取)
//	volatile uint8_t waterInjection_data_YIPX_P;        //一体刨(芯片读取)
	volatile uint8_t Frequency_data ;             		 //接口频率设置（第一次射频读取）
//	volatile uint8_t Frequency_data_YI ;              //接口频率设置（第一次射频读取）
	volatile uint8_t Speed_data;
	
}
UIDATA;

typedef struct
{
	UIDATA MemoryDATA;                                 //第一次读取（仅仅适用于磨削功能）
	UIDATA SetDATA_A;                                  //数据设置A接口
	UIDATA SetDATA_B;																	 //数据设置B接口
	volatile uint8_t RFID_flag;                         //自动识别开启
}
UIdata;

extern UIdata UIdata_s;
 









typedef struct
{
	volatile uint32_t speed;
	volatile uint8_t freq;
	volatile uint8_t Direc;
	volatile uint8_t Injection;
}
InsertHand;
typedef struct
{
	volatile uint16_t Init_RFID_CRC;//用于比较两次刀具信息（如果信息想同，不更新数据界面，如果不想同则更新界面）
	volatile uint16_t Init_tool_mode;
	InsertHand TMBB;
	InsertHand TMBA;
	InsertHand EMBA;
	InsertHand EMBB;
	InsertHand PXBX;
	InsertHand YIMX;
	InsertHand YIPX;
}
Defaultvalue;
extern Defaultvalue Defaultvalue_s;


typedef struct 
{
	volatile uint16_t	R_Hvalue;//右高
	volatile uint16_t	R_Lvalue;//右低
	volatile uint16_t	R_Mvalue;//右中
	volatile uint16_t	L_Hvalue;//左高
	volatile uint16_t	L_Lvalue;//左低
	volatile uint16_t	L_Mvalue;//左中
}
foot_memory_value;
extern foot_memory_value Foot_memory_value_s;


typedef struct 
{
	volatile uint32_t M_speed;//速度
	volatile uint8_t  M_freq;//频率
	volatile uint8_t  M_dir;//方向
	volatile uint8_t  M_irrigate;//灌注流量
	volatile uint8_t  M_injection;//注水流量
	//volatile uint8_t  M_way;//控制模式
}
workmemory_value;
typedef struct 
{
	workmemory_value A;
	workmemory_value B;
}
Workmemory_value;
extern Workmemory_value Workmemory_value_S;


typedef struct 
{
	volatile uint8_t   select_channel;//工作通道
	volatile uint8_t 	 hand_model;//手柄型号
	volatile uint32_t  set_speed;//设置速度
	volatile uint16_t  set_Irrigate;//设置灌注流量
	volatile uint8_t   Irrigate_start_flag;//灌注启动标识
  volatile uint8_t   set_Injection;//设置注水流量
	volatile uint8_t   Injection_start_flag;//注水启动标识（按照实时流量启动）
	volatile uint8_t   Injection_drain_flag;//注水排空标识（按照最大流量启动）
	volatile uint8_t   set_Freq;//设置频率
	volatile uint8_t   tool_model;//刀具模式（刨刀还是磨头）
	volatile uint8_t   set_Direction;//运行方向（正，反，往复）
	volatile  uint8_t  set_Way;//脚控，手控，触控
	volatile  uint8_t  footcontrol_online_flag;//脚控在线flag
	volatile	uint8_t  Achanell_online_flag;		//A通道是否在线
	volatile 	uint8_t  Bchanell_online_flag;   //B通道是否在线
	volatile  uint8_t  HandControlStart_flag;  //手控启动flag
	volatile  uint8_t  beep_Alarm_flag;//报警falg
	volatile  uint8_t  Alarm_value;//报警值
	volatile	uint8_t	 Foot_Key_value;//脚踏键值
	volatile	uint8_t	 Foot_L_value;//脚踏左值
	volatile	uint8_t	 Foot_R_value;//脚踏右值
	volatile	uint8_t	 Foot_model;//脚踏模式
	volatile	uint8_t	 Foot_start_flag;//脚踏启动flag
	volatile	uint8_t  FastGear_flag;//快速档位flag
	volatile 	uint8_t  MOTORWorking_flag;//电机工作flag
	volatile  uint8_t  Handle_MOTORWorking_flag;//手控工作flag
	volatile  uint8_t  ScreenKey_data;//触摸屏下发数据
  volatile  uint8_t  Footmemory_reads_signal;//脚踏内存读取信号
	volatile  uint8_t  Foot_type;//脚踏型号
	volatile  uint8_t  foot_count_M_value;//脚踏中间值
	volatile  uint8_t  FootThrottletask_flag;//油门启动
	volatile  uint32_t MotorRealSpeed;//电机真实速度
	volatile  uint8_t  Handle_mutual_flag;//手柄手动模式
	volatile  uint8_t  Handle_type;//手柄类型
	volatile  uint8_t  A_ShortCircuitRecognition_FLAG;//A短路识别flag
	volatile  uint8_t  B_ShortCircuitRecognition_FLAG;//B短路识别flag
	volatile  uint8_t  A_ChipRecognition_FLAG;//A芯片识别flag
	volatile  uint8_t  B_ChipRecognition_FLAG;//B芯片识别flag
	volatile  uint8_t  leftfoot_share_flag;//左踏板共享flag
	volatile  uint8_t  Rightfoot_share_flag;//右踏板共享flag
	volatile  uint8_t   TouchActivation_flag;//触控模式flag
	volatile  uint8_t   fenti_jiyi_flag;//分体记忆flag
	volatile  uint8_t   fenti_switch_flag;//分体开关flag
	volatile  uint8_t   HMI_Control_flag;//外部控制flag
	volatile  uint8_t   HMI_Working_flag;//外部工作flag
	volatile  uint8_t   HMI_Injection_stop_flag;//停止
}
Workvalue;
extern Workvalue Workvalue_s;

typedef struct 
{
	volatile uint8_t manual_flag;
	volatile uint8_t tool_model;
	
	
}
Memorytool;//分体记录

typedef struct
{
	Memorytool A;
	Memorytool B;
} 
Memorytools  ;
extern Memorytools Memorytools_S;
	


typedef struct 
{
	volatile	uint8_t set_speed; 
	volatile	uint8_t hand_model; 
	volatile	uint8_t manual_flag;
	volatile	uint8_t set_Injection;
	volatile	uint8_t set_Freq;
	volatile	uint8_t tool_model;
	volatile	uint8_t set_Direction;
	volatile	uint8_t set_Way;
}
Memorycontent;

typedef struct 
{
	Memorycontent A;
	Memorycontent B;
}
ChannelValue;
 extern ChannelValue ChannelValue_s;

typedef struct 
{
	volatile uint8_t Speed;
	volatile uint8_t Direction;
	//volatile uint8_t Way;
	volatile uint8_t Injection;
	volatile uint8_t Freq;
	volatile uint8_t Tool_mode;
}
PXMemory;
typedef struct 
{
	PXMemory A_Auto;
	PXMemory B_Auto;
	PXMemory A_Manual_PAODAO;
	PXMemory B_Manual_PAODAO;
	PXMemory A_Manual_MOTOU;
	PXMemory B_Manual_MOTOU;
	uint8_t  A_Manual_tool_mode;
	uint8_t  B_Manual_tool_mode;
}
FTValue;
extern FTValue FTValue_s;


void ssc_Connectfootpedal(uint8_t hannel_online);

void ssc_Connecthandel(uint8_t ui_data);
void ssc_Connecttouch(uint8_t ui_data);
void Ahandeldisplay(uint8_t ui_data);
void Bhandeldisplay(uint8_t ui_data);
void speeddisplay(uint8_t ui_data,uint32_t speed_data);
void freqdisplay(uint8_t ui_hide_flag,uint8_t fre_data);
void draindisplay(uint8_t  drain_flag);
void Irrndisplay(uint8_t  Irrnd_flag);

void clockwisedisplay(uint8_t clockwise_flag);
void anticlockwisedisplay(uint8_t anticlockwise_flag);
void OSCdiplay(uint8_t OSC_flag);
void Tooldisplay(uint8_t tool_data);
void OPENpostionDisplay(uint8_t openpostion_flag);
void PAOORMODisplay(uint8_t PAOORMO_VALUE);
void specidisplay(uint8_t speci_flag,uint16_t Length, uint8_t Diameter, uint8_t Angle);
void injectiondisplay(uint8_t activation_flag,uint16_t inject_data);
void Infusiondisplay(uint8_t activation_flag,uint16_t inject_data);
void ALARMdisplay();
void specidisplay(uint8_t speci_flag,uint16_t Length, uint8_t Diameter, uint8_t Angle);
void AutomaticDisplay(uint8_t activation_flag);
void PoweronInit();//上电初始化
void ScreenKeyTask_Init(void);
void FootKeyTask_Init(void);
void  BeepControlTask_Init(void);
void AutomaticAxtion(uint8_t paodao_flag);
uint8_t  APump(uint8_t pump_data);

void InsertHandControl(uint8_t key_value);

void PUMPBTask_Init(void);


//============================================================================
// 函数名称: SscDisplayInit()
// 功能描述: 主控界面初始化
// 输　  入: Position：
// 输    出: 无
// 函数说明: 开机延时初始化赋值
//============================================================================
void SscDisplayInit();
	


//============================================================================
// 函数名称: SscDisplayManage()
// 功能描述: 显示管理
// 输　  入: Position：
// 输    出: 无
// 函数说明: 1_管理员模式界面
//============================================================================
void SscDisplayManage();






//============================================================================
// 函数名称: LCD_Show_Password_Input()
// 功能描述: 密码输入显示
// 输　  入: Position：当前输入的密码位置
// 输    出: 无
// 函数说明: 1_管理员模式界面
//============================================================================
void Screen_Password_Input(uint8_t Position);

//============================================================================
// 函数名称: Info_B()
// 功能描述: 信息栏图片更新
// 输　  入:  
//           Info1：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量2
 
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_B(uint8_t Info1);

//============================================================================
// 函数名称: Info_A()
// 功能描述: 信息栏图片更新
// 输　  入:  
//           Info1：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量2
 
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_A(uint8_t Info1);


//============================================================================
// 函数名称: Info_HZ()
// 功能描述: 信息栏图片更新
//           Info3：右下栏 0隐藏，1往复的频率信息（高亮），2往复的频率（灰色），
//                         3速度挡位信息（灰色），4Ⅰ挡位（高亮），5Ⅱ挡位（高亮），6Ⅲ挡位（高亮）
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_HZ(uint8_t Info3);

//============================================================================
// 函数名称: LCD_InformationBarImage_Update()
// 功能描述: 信息栏图片更新
// 输　  入: Info1：上方栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）
//           Info2：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）
//           Info3：右下栏 0隐藏，1往复的频率信息（高亮），2往复的频率（灰色），
//                         3速度挡位信息（灰色），4Ⅰ挡位（高亮），5Ⅱ挡位（高亮），6Ⅲ挡位（高亮）
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_InformationBarImage_Update(uint8_t Info1, uint8_t Info2, uint8_t Info3, uint8_t Info4);

//============================================================================
// 函数名称: LCD_Show_IntegratedCutterPic_Update()
// 功能描述: 一体式刀具信息图片显示
// 输　  入: IntegratedCutter 1号刀具， 2 2号刀具，3 其它刀具
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面 10ms ~ 30ms
//============================================================================
void Screen_IntegratedCutterPic_Update(uint8_t IntegratedCutter);

//============================================================================
// 函数名称: LCD_Show_SeparatingCutterPic_Update()
// 功能描述: 分离式刀具信息图片显示
// 输　  入: SeparatingCutter：0隐，1 刀具设别已开启，未设别到刀具， 2 刀具设别已开启，设别到1号刀具，
//                             3 刀具设别已开启，设别到2号刀具，4 刀具设别已关闭 选择刀具 选中“刨刀”，
//                             5 刀具设别已关闭 选择刀具 选中“磨头”
//           WFFlag：往复标志 0 不具备往复 1 具备往复
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_SeparatingCutterPic_Update(uint8_t SeparatingCutter, uint8_t WFFlag);

//============================================================================
// 函数名称: LCD_Show_SeparatingCutterPic_Update()
// 功能描述: 手柄连接状态栏图片显示
// 输　  入: *HandleConn：HandleConn[0 ~ 3] 1 ~ 4号接口，1 接入未选中，2 接入且选中
//           *HandleType：HandleType[0 ~ 3] 1 ~ 4号接口手柄类型，1 ~ ... 对应手柄的类型
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面  显示位置 2[A] 3[B] 3[B] 1[C]
//============================================================================
uint8_t *Screen_HandleConnectState_Update(uint8_t *HandleConn, uint8_t *HandleType);

//============================================================================
// 函数名称: LCD_FootPedalConnectState_Update()
// 功能描述: 脚踏连接状态图片显示
// 输　  入: Type:0 不显示脚控、手控图片，1 脚控未连接，2 脚控 已选中，
//                3 手控 已选中，4 脚控已连接 手控已选中，5 脚控已选中 手控已连接
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_FootPedalConnectState_Update(uint8_t Type);

//============================================================================
// 函数名称: LCD_ElectricalMachineryDirectionState_Update()
// 功能描述: 电机转动方向图片显示
// 输　  入: Dir1 0 正向图片隐，1 正向 未选中 长，2 正向 选中 长，3 正向 未选中 短，4 正向 选中 短
//           Dir2 0 往复图片隐，1 正向隐 往复 未选中 长，2 正向隐 往复 选中 长，3 往复 未选中 长，4 往复 选中 长
//           Dir3 0 反转图片隐，1 反向 未选中 短，2 反向 选中 短
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_ElectricalMachineryDirectionState_Update(uint8_t Dir1, uint8_t Dir2, uint8_t Dir3);

//============================================================================
// 函数名称: LCD_ElectricalMachineryDirectionState_Update()
// 功能描述: 电机转动方向图片显示
// 输　  入: Dir1 0 正向图片隐，1 正向 未选中 长，2 正向 选中 长，3 正向 未选中 短，4 正向 选中 短
//           Dir2 0 往复图片隐，1 正向隐 往复 未选中 长，2 正向隐 往复 选中 长，3 往复 未选中 长，4 往复 选中 长
//           Dir3 0 反转图片隐，1 反向 未选中 短，2 反向 选中 短
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_ElectricalMachineryDirectionState_Update2(uint8_t Dir1);
//============================================================================
// 函数名称: LCD_TipInfo_Update()
// 功能描述: 提示信息刷新
// 输　  入: TipID 提示信息ID
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_TipInfo_Update(uint8_t TipID);

//============================================================================
// 函数名称: LCD_WindowSwitch_Update()
// 功能描述: 脚踏控制的窗口切换
// 输　  入: Num1 0 ~ 5 隐藏的框 Num2 0 ~ 5 显示的框
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_WindowSwitch_Update(int8_t Num1, uint8_t Num2);


#endif  //__SCREEN_H


