#include <stdint.h>
#include <stdbool.h>

#include "control_arbitration.h"
#include "work_alarm.h"
#include "pump_control.h"
#include "handle_control.h"

/* 手柄型号固定编号：识别、控制和屏幕显示共用，不能当作可调参数修改。 */
#define TMBB_ONLINES 1 //TMBB 手柄编号
#define TMBA_ONLINES 2 //TMBA 手柄编号
#define EMBA_ONLINES 3 //EMBA 手柄编号
#define EMBB_ONLINES 4 //EMBB 手柄编号
#define PXBA_ONLINES 5 //PXBA 分体刨削手柄编号
#define PXBB_ONLINES 6 //PXBB 分体刨削手柄编号
#define MX_YIM_ONLINES 7  //旧 MXYTM 编号；现在用 RFID EPC 中的 0x05 识别，不再从手柄 Page2 识别
#define MX_YIP_ONLINES 8  //旧 MXYTP 编号；现在用 RFID EPC 中的 0x04 识别，不再从手柄 Page2 识别
#define PX_YIM_ONLINES 9  //一体磨刀具编号
#define PX_YIP_ONLINES 10 //一体刨刀具编号
#define JMB_ONLINES    11 //JMB 手柄编号
#define MX_YIM16_ONLINES    12 //MXYTM16 手柄编号
#define LGZ_I_ONLINES        13U//单按键颅骨钻手柄编号，与屏幕型号表保持一致
#define LGZ_II_ONLINES       14U//双按键颅骨钻手柄编号，与屏幕型号表保持一致
#define KSZ_I_ONLINES        15U//克氏针一型手柄编号
#define KSZ_II_ONLINES       16U//克氏针二型手柄编号
#define KXZ_I_ONLINES        17U//空心钻一型手柄编号
#define KXZ_II_ONLINES       18U//空心钻二型手柄编号
#define COMMON_SOCKET_ONLINES 19U//公共接头编号；接头没有实体按键，运行前还要识别 RFID 刀具
#define EMBD_ONLINES         20U//EMBD 6 万增速手柄编号，用于 EEPROM 识别和屏幕显示
#define EMBC_ONLINES         21U//EMBC 普通电动手柄编号，用于 EEPROM 识别和屏幕显示
#define DHYTM_ONLINES        22U//DHYTM 反旋增速手柄编号；屏幕固定反转，电机实际正转，使用无霍尔无刷驱动


/* 通道固定编号；0 表示未选择通道，不代表 A 或 B。 */
#define CHANNEL_A 1U //A 通道
#define CHANNEL_B 2U //B 通道
#define CHANNEL_NONE 0U //没有选中通道

#define JTkey_left_short 1U //左键短按
#define JTKey_left_long 2U  //左键长按
#define JTKey_right_short 3U///右键短按
#define JTKey_right_long 4U//右键长按
#define JTKey_middle_short 5U//中间短按
#define JTKey_middle_long 6U//中键长按事件；双踏板切换通道另外按踩踏值判断
#define JTKey_Gently_left_start 7U//左踏板轻踩，开始排空
#define JTKey_Gently_left_stop 8U//左踏板松开，停止轻踩排空
#define JTKey_Gently_right_start 9U//右踏板轻踩，开始排空
#define JTKey_Gently_rigth_stop 10U//右踏板松开，停止轻踩排空

/* 泵用途固定编号，决定流量范围和流量转驱动速度的计算方式。 */
#define DRAWWATER    1U//吸引泵，设置值使用内部速度档值，不是 mL/min
#define INJECTWATER  2U//注水泵，设置流量单位为 mL/min
#define POURWATER    3U//灌注泵，设置流量单位为 mL/min

#define NOWORK      0U//无控制方式
#define JTWORK      1U//脚踏工作
#define HANDLEWORK  2U//手控工作
#define TOUCHWORK     3U//触控/外控使用的控制方式编号；具体来源另看激活标志

/*
 * 报警时间配置，单位均为毫秒：1000 表示 1 秒。
 * 调大后提示保持更久，调小后更快消失；这里只改提示时间，不改变故障判定和停机条件。
 * 每类报警仍由对应模块决定何时显示、何时清除。
 */
#define ALARM_DRV_MS           3000U //驱动板故障提示至少保持 3 秒，故障很快恢复也能看清提示
#define ALARM_MODE_MS          2000U //已选脚控时按手柄键，82 号提示保持 2 秒
#define ALARM_VERIFY_MS        2000U //运行时另一手柄校验失败，屏幕、蜂鸣和上位机临时提示保持 2 秒
#define ALARM_SOCKET_MS        2000U //公共接头未识别到 RFID 刀具，限时蜂鸣和上位机报警保持 2 秒
#define ALARM_SOCKET_REPEAT_MS 1000U //缺刀具提示两次触发至少间隔 1 秒；调小会增加重复提示次数
#define ALARM_UNPLUG_MS        2000U //手控运行时拔出手柄，屏幕、蜂鸣和上位机临时提示保持 2 秒
#define ALARM_PRESSURE_MS      2000U //压力超限时，89 号弹窗和蜂鸣保持 2 秒；此报警不停止泵或手柄


#define PLANER      1U//刨头
#define GRINDH      2U//磨头

#define ZZDIR 0U//正转
#define FZDIR 1U//反转
#define OSCDIR 2U//往复

/* 往复频率调节范围；这里保存屏幕和驱动共用的频率指令值，不要再乘除 10。 */
#define FreqMax 40U//未读到刀具频率上限时使用 40；调大允许用户选到更高的往复频率
#define FreqMin 5U//最低允许设置 5；刀具要求的下限更高时用刀具下限，不允许用户调到 0


#define JTKey        1U//脚踏调节按键
#define HANDLEKey    2U//手控调节按键
#define HMIkey       3U//外部调节按键
#define SCREENKey    4U//显示屏调节按键
#define PLUGunPLUG   5U//手柄插拔

// 以下是泵档位和按键固定编号，不是速度、流量或可调配置值。


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
#define HANDLEKey_dir_Reverse 12U//选择反转的手柄按键事件
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
#define HMIkey_HMI_EXIT 38U//退出外部控制
#define  HMIkey_JTActi 64U///脚控激活
#define  HMIkey_HandleActi 65U///手控激活

#define  HMIkey_Gently_start 68U//外部控制请求开始轻排
#define  HMIkey_Gently_stop 69U//外部控制请求停止轻排
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
#define SCREENKey_TouchStart 61U///触控启动（已废弃，0x5510 不再参与新屏触控运行）
#define SCREENKey_TouchEXIT 62U///触控退出
#define SCREENKey_HMI_EXIT 63U//在屏幕上退出外部控制

#define SCREENKey_SPEED_Sub_Large 64U//速度快减键，按当前方向的大步进减少，不是固定减 10000
#define SCREENKey_SPEED_Sub_Small 65U//速度慢减键，按当前方向的小步进减少，不是固定减 1000
#define SCREENKey_SPEED_Add_Small 66U//速度慢加键，按当前方向的小步进增加，不是固定加 1000
#define SCREENKey_SPEED_Add_Large 67U//速度快加键，按当前方向的大步进增加，不是固定加 10000
#define SCREENKey_AutoIdentify    68U//自动识别刀具按钮，开始 RFID 识别
#define SCREENKey_TouchKeepAlive  69U//屏幕按住运行按钮时反复发送的事件，超时收不到就结束触控运行



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
#define UI_POWERINIT_ID 17U//屏幕开机刷新请求，由 UIDP 任务刷新主运行页 4
#define UI_TOUCH_ID 18U//触控/外部控制弹窗显示区域，由屏幕 UI 模块负责开关


typedef struct 
{
  /* 32 位字段放在一起，减少编译器为内存对齐补入的空字节。 */
  volatile uint32_t  speed_work;//当前请求的电机速度，单位 rpm；不是驱动板实测转速
  volatile uint32_t  speed_set_work;//用户设置速度，停止后继续保留供下次启动使用
  volatile uint32_t  tool_reduction_ratio;//刀具倍率：高 16 位存增速比、低 16 位存减速比，两者均放大 100 倍，100 表示 1 倍

  /* 16 位运行参数。 */
  volatile uint16_t  freq_work;//当前往复频率指令值，主控原样发给驱动板和屏幕
  volatile uint16_t  dir_work;//当前方向：ZZDIR 正转、FZDIR 反转、OSCDIR 往复
  volatile uint16_t  current_work;//驱动过流保护设置值，来自 EEPROM Page4[21..22] 或 RFID；单位 0.01A，0 使用驱动板默认值
  volatile uint16_t  driver_speed_feedback;//驱动板实测转速，取 0xAA 回包 byte4~5，数值乘 10 才是 rpm；不改 speed_work
  volatile uint16_t  driver_current_x100;//驱动板实测电流，取 0xAA 回包 byte8~9，单位 0.01A；只供监测，不能当成过流保护设置值

  /* 单字节参数和是/否状态。 */
  volatile uint8_t   switchhandle_counts;//双踏板长按切换通道的连续计数，供 V2.1 脚踏逻辑判断切换时机
  volatile uint8_t   switchhandle_countss;//双踏板另一侧长按切换通道的连续计数
  volatile uint8_t   switchhandleA_flag;//脚踏切到 A 通道后的确认标志，未确认前不允许误启动 A 通道
  volatile uint8_t   switchhandleB_flag;//脚踏切到 B 通道后的确认标志，未确认前不允许误启动 B 通道
  volatile uint8_t   alarm_value;//报警码，供蜂鸣、UI 和上位机统一读取
  volatile uint8_t   hand_model;//当前选中通道的手柄型号
  volatile uint8_t   channel_work;//工作通道：1 为 A，2 为 B，3 为双通道
  volatile uint8_t   drivetype_work;//驱动方式：脚控、手控、触控或外控
  volatile uint8_t   hmiactive_work;//外部控制激活标志
  volatile uint8_t   touchactive_work;//屏幕触控激活标志
  volatile uint8_t   tool_type;//控制时使用的刀具类别，例如刨刀 PLANER、磨刀 GRINDH
  volatile uint8_t   raw_tool_type;//识别到的原始刀具型号；刨刀/磨刀分类后仍保留它，供驱动和上位机区分具体刀具
  volatile uint8_t   auto_identify;//当前通道是否采用 RFID 自动识别；切走后记住，切回来时恢复
  volatile bool      runflag_work;//电机运行请求标志
  volatile bool      alarm_flag;//全局持续报警是否有效
  volatile bool      Channel_Aonline;//A 通道手柄是否在线
  volatile bool      Channel_Bonline;//B 通道手柄是否在线
}
WorkMessage_t;
extern WorkMessage_t WorkMessage;//当前选中手柄的设置、运行请求和反馈信息


typedef struct 
{
  volatile bool  jt_enable_flag;//是否允许脚踏控制；不等于电机正在转
  volatile bool  handle_enable_flag;//是否允许手柄按键控制；不等于电机正在转
  volatile bool  HMI_enable_flag;//外部控制使能状态；实际启动还要检查当前控制权限
  volatile bool  jtL_control_flag;//左踏板是否正在请求电机运行
  volatile bool  jtR_control_flag;//右踏板是否正在请求电机运行
  volatile bool  handle_control_flag;//手柄按键是否正在请求电机运行
  volatile bool  HMI_control_flag;//外部控制是否正在请求当前通道电机运行

  volatile bool  jtL_gentlypump_flag;//左踏板轻踩排空请求
  volatile bool  jtR_gentlypump_flag;//右踏板轻踩排空请求
  volatile bool  HMI_gentlypump_flag;//外部控制的轻排请求

  volatile bool  JTSCREENL_pump_flag;//左侧脚控启动泵状态，历史保留字段
  volatile bool  JTSCREENR_pump_flag;//右侧脚控启动泵状态，历史保留字段
  volatile bool  HMIL_pump_flag;//外部控制左侧泵状态
  volatile bool  HMIR_pump_flag;//外部控制右侧泵状态

}ControlSignalMessage_t;
extern ControlSignalMessage_t ControlSignalMessage;//各控制来源的允许状态和运行请求，不是 RTOS 信号量


typedef struct {
  /* 按通道记住设置，切回该通道时恢复；速度单位均为 rpm。 */
  volatile uint32_t  zz_speed;//本通道记住的正转设置速度
  volatile uint32_t  fz_speed;//本通道记住的反转设置速度
  volatile uint32_t  osc_speed;//本通道记住的往复设置速度
  volatile uint32_t  speed_alarm_for;//Page4 正转速度提示阈值，与 speed_work 比较，单位 rpm
  volatile uint32_t  speed_alarm_rev;//Page4 反转速度提示阈值，与 speed_work 比较，单位 rpm
  volatile uint32_t  tool_reduction_ratio;//本通道刀具倍率；高 16 位增速比、低 16 位减速比，均放大 100 倍

  /* 本通道的频率、方向、电流保护值和默认注水量。 */
  volatile uint16_t  freq;//本通道记住的往复频率指令值，与 freq_work 使用同一单位
  volatile uint16_t  dir;//本通道记住的工作方向
  volatile uint16_t  current_work;//本通道过流保护值，单位 0.01A；切到本通道时复制到 WorkMessage.current_work
  volatile uint16_t  default_injection_flow;//Page4 默认注水量，单位 mL/min；有效范围 1~300，0 或越界改用 30；EEPROM 高字节在前

  /* 本通道的识别结果和控制方式。 */
  volatile uint8_t   hand_model;//手柄类型
  volatile uint8_t   hand_type_raw_major;//手柄EEPROM原始类型高字节，来自Page2第0字节，外部通信心跳在线时直接上传
  volatile uint8_t   hand_type_raw_minor;//手柄EEPROM原始类型低字节，来自Page2第1字节，外部通信心跳在线时直接上传
  volatile uint8_t   tool_type;//控制用刀具类别：刨刀 PLANER 或磨刀 GRINDH
  volatile uint8_t   raw_tool_type;//原始刀具型号；tool_type 只分刨刀/磨刀，这里仍保留具体型号
  volatile uint8_t   drive_type;//驱动方式：脚控、手控、外控或触控
  volatile uint8_t   freq_alarm_osc;//Page4往复转频率报警值，单位沿用频率工作值
  volatile uint8_t   auto_identify;//本通道是否采用 RFID 自动识别，切回本通道时恢复
}
ChannelMemoryMessage_t;
extern ChannelMemoryMessage_t MemoryMsgA;//A 通道保存的识别结果和用户设置
extern ChannelMemoryMessage_t MemoryMsgB;//B 通道保存的识别结果和用户设置
typedef struct 
{
  volatile bool      digital_enable;//历史预留的数字使能字段，当前业务未读取
	volatile bool      Pubadapter;//历史预留的公共转接头标志；类型为 bool，只能保存是/否，当前业务未读取
	volatile bool      dualDrive_Flag;//历史预留的双驱动标志，当前业务未使用
	volatile bool      dualDrive_Matchcode;//历史预留的双驱动匹配标志，当前业务未使用

	volatile uint8_t   freq_max;//本刀具允许的往复频率指令上限
	volatile uint8_t   freq_min;//本刀具允许的往复频率指令下限
	volatile uint8_t   freq_default;//识别本刀具后采用的默认往复频率指令值
  
	volatile uint8_t   draw;//角度
	volatile uint8_t   diameter;//直径
	volatile uint8_t   meioticratio;//旧版减速比字段，只保存整数部分；精确倍率用 tool_reduction_ratio
	volatile uint32_t  tool_reduction_ratio;//RFID/EEPROM 读出的倍率；高 16 位增速比、低 16 位减速比，均放大 100 倍
  volatile uint32_t  speed_zzmax;//本刀具正转速度上限，单位 rpm
    volatile uint32_t  speed_zzmin;//本刀具正转速度下限，单位 rpm
    volatile uint32_t  speed_fzmax;//本刀具反转速度上限，单位 rpm
    volatile uint32_t  speed_fzmin;//本刀具反转速度下限，单位 rpm
      volatile uint32_t  speed_oscmax;//本刀具往复速度上限，单位 rpm
    volatile uint32_t  speed_oscmin;//本刀具往复速度下限，单位 rpm
   volatile uint16_t speed_zzstep;//正转慢加/慢减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
    volatile uint16_t speed_fzstep;//反转慢加/慢减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
   volatile uint16_t speed_oscstep;//往复慢加/慢减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
   volatile uint16_t speed_zzstep_large;//正转快加/快减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
   volatile uint16_t speed_fzstep_large;//反转快加/快减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
   volatile uint16_t speed_oscstep_large;//往复快加/快减每次改变的 rpm；普通手柄读 Page6，公共接头按 RFID 速度范围选择
  volatile uint32_t speed_zzdefault;//识别后默认正转速度，单位 rpm
  volatile uint32_t speed_fzdefault;//识别后默认反转速度，单位 rpm
  volatile uint32_t speed_oscdefault;//识别后默认往复速度，单位 rpm
	volatile uint32_t  speed_min;//刀具总体速度下限，单位 rpm
	volatile uint32_t  speed_max;//刀具总体速度上限，单位 rpm
	volatile uint16_t  length;///长度
	volatile uint16_t  overloadThresholdFor;//正转过流保护阈值，Page4[21..22] 小端存储，单位 0.01A
	volatile uint16_t  overloadThresholdRev;//反转过流保护阈值，当前与正转共用 Page4[21..22]，单位 0.01A
	volatile uint16_t  overloadThresholdOSC;//往复过流保护阈值，当前与正转共用 Page4[21..22]，单位 0.01A
	volatile uint16_t  default_injection_flow;//Page4 默认注水量，单位 mL/min；有效范围 1~300，0 或越界改用 30；EEPROM 高字节在前
	volatile uint16_t  speed_alarm_for;//Page4 正转速度提示阈值，单位 rpm；EEPROM 低字节在前，与 speed_work 比较
	volatile uint16_t  speed_alarm_rev;//Page4 反转速度提示阈值，单位 rpm；EEPROM 低字节在前，与 speed_work 比较
	volatile uint8_t   freq_alarm_osc;//Page4 往复频率提示阈值，与 freq_work 使用同一指令单位，只用于蜂鸣提醒
	volatile uint8_t   handle_type;//手柄类型
	volatile uint8_t   hand_type_raw_major;//EEPROM 原始手柄类型高字节；识别成功后复制到本通道 MemoryMsg
	volatile uint8_t   hand_type_raw_minor;//EEPROM 原始手柄类型低字节；识别成功后复制到本通道 MemoryMsg
    volatile uint8_t   run_direction;//运行方向
    volatile uint8_t   control_mode;//控制模式
    volatile uint8_t   tool_type;//刀具类型	
    volatile uint8_t   raw_tool_type;//RFID/EEPROM 读出的原始刀具型号，保留标签编号或旧型号编号
    volatile uint8_t   rfid_cache_hit;//是否再次识别到同一 RFID 刀具；是则保留用户已调好的速度和频率
}
ChannelrecognizeMessage_t;//本通道刚读出的手柄、刀具信息
extern ChannelrecognizeMessage_t ChannelrecognizeMessageA;
extern ChannelrecognizeMessage_t ChannelrecognizeMessageB;



typedef struct 
{
  /* 压力板读数；压力报警使用 weight_x10，不直接用原始采样值判断。 */
  volatile int32_t  pressure_value;//压力板上报的原始采样值 RawCs1237，尚未换算成重量
  volatile uint32_t weight_x10;//压力板换算后的重量 WeightX10，单位 0.1g；6000 是未就绪标记，不按普通重量处理

  /* 注水/灌注的 speed 字段表示 mL/min，吸引泵表示内部设置值，都不是驱动板转速。 */
  volatile uint16_t timingDrainage_times;//屏幕排空已累计的 25ms 周期数，400 次约 10 秒，须用 16 位避免溢出
  volatile uint16_t type;//泵用途编号：吸引、注水或灌注；不是压力板发来的原始霍尔设备码
  volatile uint16_t speed_work;//用户设置流量或吸引速度值，普通运行和手柄联动都使用它
  volatile uint16_t speed_output;//本周期最终输出设置值，运行时屏幕和上位机显示它；压力超限只报警，不清零
  volatile uint16_t speed_Max;//本泵允许的设置上限，单位与 speed_work 相同
  volatile uint16_t speed_Min;//本泵允许的设置下限，单位与 speed_work 相同
  volatile uint16_t speed_step_value;//加减键每次改变的量，单位与 speed_work 相同
  volatile uint16_t pressure_threshold;//压力板上报的报警阈值，单位 g；比较前需与 weight_x10 统一单位

  /* 本泵档位、识别结果和运行请求。 */
  volatile uint8_t  step_value;//当前泵档位
  volatile uint8_t  associated_channel;//泵与手柄通道的关联标志
  volatile uint8_t  direction;//泵业务方向
  volatile uint8_t  losses_times;//压力模块连续识别丢失次数
  volatile uint8_t  seq;//压力模块上报帧序号
  volatile bool     online_flag;//已识别到压力板及泵类型；不表示步进电机一定在转
  volatile bool     run_flag;//普通运行或手柄联动的开泵请求；不等于驱动板运行反馈
  volatile bool     timingDrainage_flag;//屏幕请求进行 10 秒定时排空
  volatile bool     pedalDrainage_flag;//脚踏/外控轻排请求：按 speed_work 排空，不检查压力，不累计屏幕 10 秒排空时间
}
pumpMessage_t;
extern pumpMessage_t pumpMessageA;//A 泵的设置、压力读数和运行请求
extern pumpMessage_t pumpMessageB;//B 泵的设置、压力读数和运行请求
extern uint32_t paoxueSpeciValue_A[4];//A 通道刀具规格，识别任务写入，屏幕任务读取显示
extern uint32_t paoxueSpeciValue_B[4];//B 通道刀具规格，识别任务写入，屏幕任务读取显示
extern uint8_t paoxueSpeciValue_F[16];//历史保留的分体刀具扩展数据区



typedef struct 
{
  bool JTing_FLAG;//历史保留的 JT 脚踏控制标志，当前业务未使用
  bool JTBing_FLAG;//历史保留的 JTB 脚踏控制标志，当前业务未使用
  bool JTDLing_FLAG;//历史保留的 JTD 左踏板控制标志，当前业务未使用
  bool JTDRing_FLAG;//历史保留的 JTD 右踏板控制标志，当前业务未使用

  bool Handleing_FLAG;//历史保留的手柄控制标志，当前业务未使用
  bool HMIing_FLAG;//历史保留的外部控制标志，当前业务未使用

  bool JTApumpGently_FLAG;//历史保留的 A 泵轻排标志，当前业务未使用
  bool JTBpumpGently_FLAG;//历史保留的 B 泵轻排标志，当前业务未使用
}
ControlSigleMessage_t;
extern ControlSigleMessage_t ControlSigleMssage;

void ChannelMessageInit(void); /* 清空两组控制标志，不负责清除手柄识别结果。 */
void WorkMessageInit(void); /* 清除当前手柄运行状态并设置初始值。 */
void ChannelMemoryMessageInit(void); /* 初始化 A/B 通道保存的参数。 */
void HandleSwitchActive(uint8_t key_value); /* 按按键事件切换当前手柄通道。 */
void Handle_RequestRemainingOnlineAfterUnplug(uint8_t channel); /* 记住运行中拔柄后要切换到的另一在线通道，等停止处理完成再切换。 */
void Handle_SelectRemainingOnlineAfterUnplug(void); /* 运行中拔出当前手柄并完成控制源退出后，自动选择唯一剩余的在线通道。 */
void SpeedActive(uint8_t key_value); /* 按加减键和本刀具允许范围调整速度。 */
void DirActive(uint8_t key_value); /* 按按键事件切换方向，方向固定的刀具不允许切换。 */
void FreqActive(uint8_t key_value); /* 按加减键和本刀具允许范围调整往复频率指令值。 */
bool ScreenKey_CanUse(uint8_t screen_key); /* 判断新屏逻辑按键当前是否真正可用；黑色、隐藏或被业务状态禁用时返回 false。 */
void HmiExitActive(uint8_t key_value); /* 处理退出外部控制的按键事件。 */
void Pubinterface_RefreshControlModeDisplay(void); /* 一起刷新脚控、手控、触控图标，防止旧控制方式仍亮着。 */
void Pubinterface_RefreshRuntimeDisplaySnapshot(void); /* 开机后按当前保存的状态刷新 A/B 手柄图标和当前通道参数。 */
bool Pubinterface_ApplyFootControlPriorityOnConnect(void); /* 脚踏上线且手柄未运行时，按脚踏优先规则尝试切到脚控。 */
void Pubinterface_ClearFootControlManualLock(void); /* 脚踏掉线或用户重新选脚控后，清除“用户已手动选其他方式”的记录。 */
void Pubinterface_RefreshExternalCommDisplay(bool connected_flag, bool active_flag); /* 刷新外部通信小电脑图标：在线白色、外控黄色、离线熄灭。 */
bool Pubinterface_IsHandleVerifyAlarm(uint8_t alarm_value); /* 判断报警码是否属于手柄校验失败。 */
bool Pubinterface_IsHandleControlReservedModel(uint8_t hand_model); /* 判断此型号是否允许用手柄实体按键控制。 */
void Pubinterface_ServiceTransientAlarms(void); /* 周期检查公共接头、压力超限和拔手柄的限时提示，到时清除对应提示。 */
void Pubinterface_SendHandleDisplay(uint8_t channel, uint8_t handle_model, bool enable_flag, bool light_flag); /* 刷新单个手柄图标。 */
void Pubinterface_RefreshOnlineHandleDisplay(void); /* 按当前在线状态刷新 A/B 手柄图标，已拔出的通道显示为未连接。 */
bool Pubinterface_IsSplitToolSpecDisplayModel(uint8_t hand_model); /* 判断是否为 PXBA/PXBB 分体识别手柄。 */
bool Pubinterface_IsPlanerCapabilityTool(uint8_t tool_type); /* 判断此刀具是否按刨刀方式控制。 */
bool Pubinterface_IsDirLocked(uint8_t hand_model, uint8_t raw_tool_type); /* 判断此手柄或 RFID 刀具是否只能用固定方向，固定后屏幕和外控都不能改。 */
bool Pubinterface_IsOpenPositionEnabledTool(uint8_t hand_model, uint8_t tool_type); /* 判断当前组合是否允许开口定位。 */
void Pubinterface_SetLastRfidToolType(uint8_t channel, uint8_t tool_type); /* 保存指定通道最近一次 RFID 刀具类型。 */
void Pubinterface_ClearSelectedChannelDisplay(void); /* 无当前手柄时关闭运行参数区。 */
void Pubinterface_ClearCommonSocketToolMissingAlarm(void); /* 刀具重新识别后清除公共接头缺刀报警。 */
void Pubinterface_RefreshSelectedChannelDisplay(uint8_t channel); /* 用指定通道保存的参数刷新主运行页。 */
bool Pubinterface_ClearHandleNotConnectedAlarm(void); /* 清除当前手柄未连接报警。 */
void Pubinterface_RefreshHandleUnplugAlarmDisplay(void); /* 清屏后补发仍有效的拔手柄报警。 */
void Pubinterface_StopRunningHandleOnUnplug(void); /* 运行中拔当前手柄时停止电机和联动泵。 */
void Pubinterface_ApplyInjectionPumpDefaultFlow(uint16_t flow); /* 为在线注水泵设置默认流量；输入无效时改用备用值，单位 mL/min。 */
void Pubinterface_ApplyChannelDefaultInjectionFlow(uint8_t channel); /* 把指定通道 Page4 的默认注水量设给在线注水泵。 */
bool Pubinterface_ShouldAutoSelectPluggedChannel(uint8_t channel); /* 判断新插入通道是否允许自动选中。 */
void Pubinterface_SaveRecognizeToMemory(uint8_t channel); /* 把指定通道扫描结果保存到 MemoryMsgA/B。 */
bool Pubinterface_IsCommonSocketToolReady(void); /* 公共接头需已识别到可用刀具才返回true，非公共接头直接返回true；只查询，不发报警。 */
bool Pubinterface_CheckCommonSocketToolReadyForRun(void); /* 手柄电机启动前检查公共接头 EPC 刀具头，缺失时负责报警并拒绝运行。 */
bool Pubinterface_CheckCommonSocketToolReadyForFootRun(void); /* 脚踏启动前检查公共接头刀具，缺失时持续报警到松脚。 */
void Pubinterface_ReleaseFootCommonSocketToolMissingAlarm(void); /* 脚踏松开或掉线后，关闭未识别到 RFID 刀具的持续弹窗和蜂鸣。 */
void Pubinterface_StopTouchKeepAliveRun(void); /* 结束屏幕按住运行按钮发起的电机运行。 */
void Pubinterface_ReleaseTouchHandleNotConnectedAlarm(void); /* 触控运行时拔柄，等用户松开按钮后退出本次触控并清除报警。 */
void Pubinterface_LoadChannelMemory(uint8_t channel); /* 把指定通道保存的参数恢复到当前工作信息。 */
uint16_t Pubinterface_GetChannelDefaultInjectionFlow(uint8_t channel); /* 取得指定通道默认注水量，单位 mL/min。 */
uint16_t Pubinterface_GetCurrentDefaultInjectionFlow(void); /* 取得当前通道默认注水量，单位 mL/min。 */
uint16_t Pubinterface_GetInjectionPumpStartFlow(void); /* 取得非零启动流量，避免注水泵收到开泵请求却因流量为 0 不转。 */
uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message); /* 按泵用途取得非零启动设置值，避免只设置 run_flag 但泵不转。 */
uint16_t Pubinterface_GetPumpDisplaySpeed(const pumpMessage_t *pump_message); /* 运行时显示最终输出设置值，停止时显示用户设置值；不是实测泵转速。 */
void Pubinterface_UpdatePumpAOutputSpeed(uint16_t output_speed); /* 保存 A 泵本周期最终输出设置值，变化时刷新左侧泵显示。 */
void Pubinterface_UpdatePumpBOutputSpeed(uint16_t output_speed); /* 保存 B 泵本周期最终输出设置值，变化时刷新右侧泵显示。 */
void Pubinterface_RefreshPumpAButtonDisplay(void); /* A 泵刚启动时先刷新按钮，等输出设置值更新后再刷新档位环，避免闪出零档图。 */
void Pubinterface_RefreshPumpBButtonDisplay(void); /* B 泵刚启动时先刷新按钮，等输出设置值更新后再刷新档位环，避免闪出零档图。 */
void Pubinterface_RefreshPumpADisplay(void); /* 刷新 A 泵数值和启停按钮，屏幕操作和上位机操作共用。 */
void Pubinterface_RefreshPumpBDisplay(void); /* 刷新 B 泵数值和启停按钮，屏幕操作和上位机操作共用。 */
uint32_t Pubinterface_GetCurrentDefaultMotorSpeed(void); /* 读取当前通道、当前方向在MemoryMsg中保存的速度，单位rpm；调速后保存值会更新，不一定仍是出厂默认值。 */
void Pubinterface_SetHandleInjectionPumpRun(bool enable); /* 随手柄启动或停止注水冷却泵，只释放本次联动接管的泵。 */
uint8_t Pubinterface_GetHandleInjectionPumpFollowMask(void); /* 查询哪些泵正由手柄冷却联动接管，脚踏据此只停止自己联动的泵。 */
void Pubinterface_HandlePumpPressureBlocked(uint8_t pump_channel); /* A/B 泵持续超压确认后仅蜂鸣并显示 89 号弹窗，不停止泵或联动手柄。 */
void Pubinterface_HandlePumpDriverFault(uint8_t pump_channel); /* 步进驱动首次回报非零故障时停本泵和相关控制源，注水冷却时联动停手柄。 */
void Pubinterface_ServicePumpDriverFaultHold(uint8_t pump_channel); /* 驱动故障停机记录未清除时继续保持停止，不重复蜂鸣或弹窗。 */
void Pubinterface_ClearPressureBlockStopLatchForNewTrigger(void); /* 保留旧名称；再次收到新的启动操作时，清除注水泵驱动故障造成的联动停机记录。 */
bool Pubinterface_IsPressureBlockStopLatched(void); /* 保留旧名称；查询注水泵驱动故障是否要求联动停机，压力超限报警不会设置此记录。 */
void Pubinterface_CheckSpeedThresholdAlarm(void); /* 检查当前速度或往复频率是否达到刀具提示阈值。 */
bool Pubinterface_IsRfidAutoIdentifyEnabled(uint8_t channel); /* 查询指定 PXBA/PXBB 通道是否仍允许自动识别 RFID，识别任务据此决定是否继续读取。 */
void ControlTypeActive(uint8_t key_value); /* 按按键事件选择脚控、手控或触控。 */
 void ChannelrecognizeMessageInit(void); /* 初始化 A/B 手柄、刀具识别信息。 */
void ChannelFlagMessageInit(void); /* 初始化通道相关状态标志。 */
void pumpMessageInit(void); /* 清除 A/B 泵运行信息并设置初始值。 */
void Pubinterface_ClearRfidToolMemory(uint8_t channel); /* 清除指定通道保存的 RFID 刀具信息。 */
