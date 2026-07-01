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
#define LGZ_I_ONLINES        13U//颅骨钻单按键手柄预留，按 UI 类型编号保持一致
#define LGZ_II_ONLINES       14U//颅骨钻双按键手柄预留，后续实物协议确认后直接复用该类型
#define KSZ_I_ONLINES        15U//克氏针一型手柄预留，用于屏幕显示和识别表占位
#define KSZ_II_ONLINES       16U//克氏针二型手柄预留，用于屏幕显示和识别表占位
#define KXZ_I_ONLINES        17U//空心钻一型手柄预留，保留 UI 绑定编号
#define KXZ_II_ONLINES       18U//空心钻二型手柄预留，保留 UI 绑定编号
#define COMMON_SOCKET_ONLINES 19U//公共接头预留，没有手柄实体键，仅用于识别和屏幕显示
#define EMBD_ONLINES         20U//EMBD 6万增速手柄，用于 EEPROM 识别和普通电动手柄 UI 显示
#define EMBC_ONLINES         21U//EMBC 新增普通电动手柄，用于 EEPROM 识别和通用手柄 UI 显示


#define CHANNEL_A 1U
#define CHANNEL_B 2U
#define CHANNEL_NONE 0U

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

#define NOWORK      0U//无控制方式
#define JTWORK      1U//脚踏工作
#define HANDLEWORK  2U//手控工作
#define TOUCHWORK     3U//外部工作

#define CONTROL_OWNER_NONE      0U//当前没有任何控制方式占用运行控制权
#define CONTROL_OWNER_EXTERNAL  1U//上位机外部通信正在占用控制权
#define CONTROL_OWNER_FOOT      2U//脚踏正在占用控制权
#define CONTROL_OWNER_SCREEN    3U//屏幕按钮/触控正在占用控制权
#define CONTROL_OWNER_HANDLE    4U//手柄按键正在占用控制权

/*
 * WorkMessage.alarm_value 统一报警码。
 * 数值必须和 sscUIDP.c::UIAIARMDP() 的图片映射保持一致，蜂鸣、屏幕和上位机都读取同一份报警状态。
 */
#define WORK_ALARM_NONE                 0U//无报警
#define WORK_ALARM_HANDLE_NOT_CONNECTED 1U//手柄未连接，请连接手柄
#define WORK_ALARM_MANUAL_SELECTED      2U//手控已选中，请用手控
#define WORK_ALARM_FOOT_SELECTED        3U//脚控已选中，请用脚控
#define WORK_ALARM_MOTOR_OVERLOAD       4U//电机过载，请松开脚踏
#define WORK_ALARM_FOOT_VALUE_ERROR     6U//脚踏值错误，请联系售后
#define WORK_ALARM_MOTOR_OVERLOAD_ALT   5U//电机过载，请松开脚踏，兼容旧 UI 图片位
#define WORK_ALARM_UID_ERROR            7U//UID 错误
#define WORK_ALARM_MOTOR_COMM_ERROR     8U//电机通讯异常
#define WORK_ALARM_HALL_ERROR           9U//HALL 值错误
#define WORK_ALARM_HANDLE_MODEL_ERROR_A 10U//A通道EEprom校验失败，手柄型号错误
#define WORK_ALARM_HANDLE_MODEL_ERROR   WORK_ALARM_HANDLE_MODEL_ERROR_A//兼容旧代码：未区分通道时默认沿用A通道手柄校验报警
#define WORK_ALARM_SPEED_THRESHOLD      11U//Page4速度/频率阈值蜂鸣报警，只驱动蜂鸣，不强制停机
#define WORK_ALARM_MOTOR_DRIVER_BOARD   11U//电机驱动板故障报警码，UI 绑定 0x0B；真实报警写 WorkMessage，阈值报警只蜂鸣
#define WORK_ALARM_HANDLE_MODEL_ERROR_B 12U//B通道EEprom校验失败，手柄型号错误
#define WORK_ALARM_HANDLE_MODEL_ERROR_AB 14U//A/B通道EEprom均校验失败，上位机需要显示双通道来源


#define PLANER      1U//刨头
#define GRINDH      2U//磨头

#define ZZDIR 0U//正转
#define FZDIR 1U//反转
#define OSCDIR 2U//往复

#define FreqMax 40U//新屏未单独下发频率上限时沿用 4.0Hz 上限
#define FreqMin 5U//8寸屏往复频率按 5Hz 起调，避免 0Hz 被显示为可用频率


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
#define SCREENKey_TouchStart 61U///触控启动（已废弃，0x5510 不再参与新屏触控运行）
#define SCREENKey_TouchEXIT 62U///触控退出
#define SCREENKey_HMI_EXIT 63U//外部控制退出，显示器共计25个按钮指令

#define SCREENKey_SPEED_Sub_Large 64U//新屏速度大幅减少，固定减少 10000
#define SCREENKey_SPEED_Sub_Small 65U//新屏速度小幅减少，固定减少 1000
#define SCREENKey_SPEED_Add_Small 66U//新屏速度小幅增加，固定增加 1000
#define SCREENKey_SPEED_Add_Large 67U//新屏速度大幅增加，固定增加 10000
#define SCREENKey_AutoIdentify    68U//新屏自动识别刀具按钮，进入 RFID 自动识别入口
#define SCREENKey_TouchKeepAlive  69U//8寸屏触控运行保活键，持续收到才保持触控运行



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
#define UI_POWERINIT_ID 17U//屏幕开机初始化区域，统一交给 UIDP 任务刷新主运行页 4
#define UI_TOUCH_ID 18U//触控/外部控制弹窗显示区域，由屏幕 UI 模块负责开关


typedef struct 
{
  volatile bool      runflag_work;//电机运行标志位，这里考虑
  volatile uint8_t   switchhandle_counts;//双踏板长按切换通道的连续计数，供 V2.1 脚踏逻辑判断切换时机。
  volatile uint8_t   switchhandle_countss;//双踏板长按切换通道的连续计数，供 V2.1 脚踏逻辑判断切换时机。
  volatile uint8_t   switchhandleA_flag;//脚踏切到 A 通道后的确认标志，未确认前不允许误启动 A 通道。
  volatile uint8_t   switchhandleB_flag;//脚踏切到 B 通道后的确认标志，未确认前不允许误启动 B 通道。
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
  volatile uint8_t   raw_tool_type;//原始刀具型号，保留 PXM/PXP/RFID 标签代号，驱动和上位机扩展可按原始来源判断
  volatile uint32_t  speed_work;//工作速度
  volatile uint32_t  speed_set_work;//设置速度
  volatile uint16_t  freq_work;//工作频率
  volatile uint16_t  dir_work;///工作方向
  volatile uint16_t  current_work;//工作电流
  volatile uint16_t  driver_speed_feedback;//驱动板反馈实际转速，来自 0xAA 回包 byte4~5，保持驱动协议中的“实际转速/10”单位，不覆盖控制目标速度
  volatile uint16_t  driver_current_x100;//驱动板反馈实时电流，来自 0xAA 回包 byte8~9，单位 0.01A，只用于监测上传，不能覆盖 current_work 保护电流
  volatile uint32_t  tool_reduction_ratio;//减速比 高16位表示增速16位表示减速
  volatile uint8_t   auto_identify;//当前通道是否由屏幕自动识别/RFID 流程触发，切通道时同步记忆
}
WorkMessage_t;
extern WorkMessage_t WorkMessage;//工作信息


typedef struct 
{
  volatile bool  jt_enable_flag;//脚踏启动flag-电机
  volatile bool  handle_enable_flag;//手控启动flag-电机
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
  volatile uint8_t  raw_tool_type;//原始刀具型号，tool_type 归一为 PLANER/GRINDH 后仍保留旧型号码
  volatile uint8_t   drive_type;//驱动方式(脚控，手控，外控，触控)
  volatile uint32_t  zz_speed;//正传速度
  volatile uint32_t  fz_speed;//反传速度
  volatile uint32_t  osc_speed;//往复速度
  volatile uint16_t  freq;//工作频率
  volatile uint16_t  dir;///工作方向
  
  volatile uint16_t  current_work;
  volatile uint16_t  default_injection_flow;//Page4默认注水流量，大端直接写1~70，0或越界由业务层回退30
  volatile uint32_t  speed_alarm_for;//Page4正转速度报警阈值，单位与WorkMessage.speed_work一致为实际rpm
  volatile uint32_t  speed_alarm_rev;//Page4反转速度报警阈值，单位与WorkMessage.speed_work一致为实际rpm
  volatile uint8_t   freq_alarm_osc;//Page4往复转频率报警值，单位沿用频率工作值
  volatile uint32_t  tool_reduction_ratio;//刀具减数比
  volatile uint8_t   auto_identify;//通道自动识别状态记忆，避免切换通道后丢失 RFID 识别模式
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
	volatile uint32_t  tool_reduction_ratio;//RFID/EEPROM 解析出的完整刀具减速比，高16位表示增速、低16位表示减速
  volatile uint32_t  speed_zzmax;
    volatile uint32_t  speed_zzmin;
    volatile uint32_t  speed_fzmax;
    volatile uint32_t  speed_fzmin;
      volatile uint32_t  speed_oscmax;
    volatile uint32_t  speed_oscmin;
   volatile uint16_t speed_zzstep;//正向小步进速度，来自 EEPROM Page6[0..1]
    volatile uint16_t speed_fzstep;//反向小步进速度，来自 EEPROM Page6[0..1]
   volatile uint16_t speed_oscstep;//往复小步进速度，来自 EEPROM Page6[0..1]
   volatile uint16_t speed_zzstep_large;//正向大步进速度，来自 EEPROM Page6[2..3]，供屏幕大加/大减键使用
   volatile uint16_t speed_fzstep_large;//反向大步进速度，来自 EEPROM Page6[2..3]，避免继续用小步进乘 2
   volatile uint16_t speed_oscstep_large;//往复大步进速度，来自 EEPROM Page6[2..3]，只影响往复方向调速
  volatile uint32_t speed_zzdefault;//速度
  volatile uint32_t speed_fzdefault;//速度
  volatile uint32_t speed_oscdefault;//速度
	volatile uint32_t  speed_min;//速度
	volatile uint32_t  speed_max;//速度
	volatile uint16_t  length;///长度
	volatile uint16_t  overloadThresholdFor;//过载阀值（正）
	volatile uint16_t  overloadThresholdRev;//过载阀值（反）
	volatile uint16_t  overloadThresholdOSC;//过载阀值（往复）
	volatile uint16_t  default_injection_flow;//Page4默认注水流量，EEPROM大端直接写1~70，0或越界由业务层回退30
	volatile uint16_t  speed_alarm_for;//Page4正转速度报警阈值，EEPROM小端2字节，单位与WorkMessage.speed_work一致为实际rpm
	volatile uint16_t  speed_alarm_rev;//Page4反转速度报警阈值，EEPROM小端2字节，单位与WorkMessage.speed_work一致为实际rpm
	volatile uint8_t   freq_alarm_osc;//Page4往复转频率报警值，单位沿用频率工作值，只用于蜂鸣阈值
	volatile uint8_t   handle_type;//手柄类型
	volatile uint8_t   hand_type_raw_major;//手柄EEPROM原始类型高字节，扫描认证后先暂存在识别结构，后续由PlugORunPLUGActive统一写入MemoryMsg
	volatile uint8_t   hand_type_raw_minor;//手柄EEPROM原始类型低字节，扫描认证后先暂存在识别结构，后续由PlugORunPLUGActive统一写入MemoryMsg
    volatile uint8_t   run_direction;//运行方向
    volatile uint8_t   control_mode;//控制模式
    volatile uint8_t   tool_type;//刀具类型	
    volatile uint8_t   raw_tool_type;//原始刀具型号，RFID/EEPROM 解析后用于保留标签或旧表码
    volatile uint8_t   rfid_cache_hit;//同一 EPC 标签重识别标志，保存到 MemoryMsg 前用于保留用户调节过的速度和频率
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
  volatile uint16_t  type;//业务泵类型：DRAWWATER/INJECTWATER/POURWATER，不能保存 CS1237 霍尔设备码
  volatile uint8_t  direction;//方向
  volatile uint16_t speed_work;//泵速度，灌注最大到300ml，必须使用16位避免截断
  volatile uint16_t speed_output;//压力闭环修正后的实际输出速度，屏幕和上位机运行态显示读这里，speed_work 保持设定值
  volatile uint16_t speed_Max;
  volatile uint16_t speed_Min;
  volatile uint16_t speed_step_value;//调节步进值一次多少ml，快速档位先不管，后续在判断最大值最小值之间分为6档进行分配
  volatile uint8_t  losses_times;//识别丢失次数
  volatile int32_t  pressure_value;//压力传感器原始值，对应下位机 RawCs1237
  volatile uint16_t pressure_threshold;//压力阀值，由压力模块提供
  volatile uint32_t weight_x10;//重量值，单位 0.1g，对应下位机 WeightX10
  volatile bool     pressure_hold_flag;//压力触发停泵后的锁止标志，保持期间实际输出为0，只能由下一次控制源启动沿清除。
  volatile uint32_t pressure_recover_ms;//压力锁止辅助计数保留字段，当前策略不再按压力恢复时间自动恢复泵输出。
  volatile uint8_t  seq;//压力模块上报帧序号

}
pumpMessage_t;
extern pumpMessage_t pumpMessageA;//泵A结构体
extern pumpMessage_t pumpMessageB;//泵B结构体
extern uint32_t paoxueSpeciValue_A[4];//A 通道刀具规格缓存，供手柄识别写入并由 UI 刷新读取
extern uint32_t paoxueSpeciValue_B[4];//B 通道刀具规格缓存，供手柄识别写入并由 UI 刷新读取
extern uint8_t paoxueSpeciValue_F[16];//分体刀具扩展缓存，保留给识别流程和后续显示逻辑使用



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
void AutoIdentifyActive(uint8_t key_value);//新屏自动识别刀具入口，按当前通道触发 RFID 识别
void HmiExitActive(uint8_t key_value);
bool ControlArbitration_IsExternalActive(void);
bool ControlArbitration_IsOwner(uint8_t owner);
bool ControlArbitration_IsBusyByOther(uint8_t owner);
bool ControlArbitration_TryEnter(uint8_t owner);
void ControlArbitration_Exit(uint8_t owner);
void ControlArbitration_ExitLocalControlIfIdle(uint8_t owner);
/* 周期刷新本地电机 owner 释放，电机命令停止且驱动反馈归零后允许其它本地模式接管。 */
void ControlArbitration_RefreshMotorOwner(void);
void ControlArbitration_ForceRelease(void);
bool ControlArbitration_EnterExternalControl(void);
void ControlArbitration_ReleaseExternalControl(void);
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type,uint8_t control_key);
void Pubinterface_RefreshControlModeDisplay(void); /* 统一刷新脚控、手控、触控三个控制方式图标，避免脚踏任务分散直写残留高亮。 */
bool Pubinterface_ApplyFootControlPriorityOnConnect(void); /* 脚踏上线且手柄未运行时，按脚踏优先规则尝试切到脚控。 */
void Pubinterface_ClearFootControlManualLock(void); /* 脚踏离线或用户重新选择脚控时清除本在线周期的屏幕手动锁存。 */
void Pubinterface_RefreshExternalCommDisplay(bool connected_flag, bool active_flag); /* 刷新外部通信小电脑图标：在线白色、外控黄色、离线熄灭。 */
void WorkAlarm_Set(uint8_t alarm_value);
void WorkAlarm_Clear(void);
void WorkAlarm_ClearIf(uint8_t alarm_value);
bool WorkAlarm_Is(uint8_t alarm_value);
bool Pubinterface_IsHandleVerifyAlarm(uint8_t alarm_value);
bool Pubinterface_IsCommonSocketToolReady(void);
bool Pubinterface_CheckCommonSocketToolReadyForRun(void); /* 手柄电机启动前检查公共接头 EPC 刀具头，缺失时负责报警并拒绝运行。 */
void Pubinterface_StopTouchKeepAliveRun(void);
void Pubinterface_ReleaseTouchHandleNotConnectedAlarm(void); /* 触控运行中拔手柄后，用户松开运行按钮时退出触控来源并清报警。 */
void Pubinterface_LoadChannelMemory(uint8_t channel);
uint16_t Pubinterface_GetChannelDefaultInjectionFlow(uint8_t channel);
uint16_t Pubinterface_GetCurrentDefaultInjectionFlow(void);
uint16_t Pubinterface_GetInjectionPumpStartFlow(void); /* 取得注水泵启动兜底流量，保证上位机/屏幕/脚踏启动泵时 speed_work 不为 0。 */
uint16_t Pubinterface_GetPumpStartSpeed(const pumpMessage_t *pump_message); /* 按泵业务类型取得启动兜底速度，避免灌注/抽吸泵只置 run_flag 但 0 速不转。 */
uint16_t Pubinterface_GetPumpDisplaySpeed(const pumpMessage_t *pump_message); /* 取得泵显示速度：运行态显示闭环后的实际输出，停止态保留设定速度。 */
void Pubinterface_UpdatePumpAOutputSpeed(uint16_t output_speed); /* A 泵任务发布压力闭环后的实际输出速度，并在变化时刷新左侧泵显示。 */
void Pubinterface_UpdatePumpBOutputSpeed(uint16_t output_speed); /* B 泵任务发布压力闭环后的实际输出速度，并在变化时刷新右侧泵显示。 */
void Pubinterface_RefreshPumpADisplay(void); /* 对外刷新 A 泵数值区和启停按钮，供屏幕路径和上位机路径共用。 */
void Pubinterface_RefreshPumpBDisplay(void); /* 对外刷新 B 泵数值区和启停按钮，供屏幕路径和上位机路径共用。 */
uint16_t Pubinterface_GetCurrentDefaultMotorSpeed(void);
void Pubinterface_SetHandleInjectionPumpRun(bool enable);
void Pubinterface_HandlePumpPressureBlocked(uint8_t pump_channel); /* 抽吸/注水/灌注泵压力堵塞首次触发时由泵任务调用，负责停本泵并蜂鸣；注水冷却时再停手柄。 */
void Pubinterface_ServicePumpPressureHold(uint8_t pump_channel); /* 压力锁止保持期间由泵任务调用，继续停本泵；注水冷却时防止连续控制源把手柄重新拉起。 */
void Pubinterface_ClearPressureBlockStopLatchForNewTrigger(void); /* 手控/触控/外控新的启动沿到来时清除压力停机锁存。 */
bool Pubinterface_IsPressureBlockStopLatched(void); /* 查询注水冷却压力停机锁存是否仍有效，脚踏保持踩下时用它拦截自动重启。 */
void Pubinterface_CheckSpeedThresholdAlarm(void);
bool Pubinterface_IsRfidAutoIdentifyEnabled(uint8_t channel); /* 查询 PXBA/PXBB 指定通道是否仍允许 RFID 自动识别，供 handlescan 在线监测门禁使用。 */
void ControlTypeActive(uint8_t key_value);
void PlanerGridH(uint8_t key_value);
 void ChannelrecognizeMessageInit(void);
void ChannelFlagMessageInit(void);
void pumpMessageInit(void);
void Pubinterface_ClearRfidToolMemory(uint8_t channel);
void PlugORunPLUGActive(uint8_t key_value);
