//data.h

#ifndef __DATA_H
#define __DATA_H

#include <stdint.h>

#define ADDR_BASE 0x080E0000 	// FLASH 存储手柄型号的起始地址 (128K : 0x080E0000 ~ 0x080FFFFF)

#define OFF  0
#define ON   1

#define No_Connect  0
#define Connect     1

#define No_Error    0
#define Error       1

#define No_Lift     0
#define Lift        1
#define LiftS       2

#define Normal_stop    0  //正常停止
#define Emergency_stop 1  //紧急制动
#define Free_stop      2  //自由停止

#define Pedal_Stop  0   //停止
#define Pedal_Run   1   //运行

#define Pedal_HandleNO   0  //无手柄
#define Pedal_HandleOK   1  //有手柄

#define All_Clear_JT   0x02 //解除报警
#define Warning1_JT    0x03 //手柄未连接！
#define Warning2_JT    0x04 //电机过载！
#define Warning3_JT    0x05 //脚踏值错误！
#define Warning4_JT    0x06 //UID错误！
#define Warning5_JT    0x07 //手柄错误！
#define Warning6_JT    0x08 //驱动故障！

#define MOTORTYPE_EC13   13 	// 13电机 电流小
#define MOTORTYPE_EC16   16 	// 16电机 电流大
#define MOTORTYPE_EC100  100  //有刷/无刷分割

#define MotorNone  0
#define MotorNum1  1
#define MotorNum2  2
#define MotorNum3  3

#define MotorStop      0
#define MotorWorking   1

#define FootCtrl      0  //脚控
#define ManualCtrl    1  //手控
#define FootPedalValueOffset   20

#define NO_Press  0       //抬起
#define Press     1       //按下

#define RECIPROCATING  1   //往复
#define FORWARD        2    //正向
#define REVERSE        3    //反向

#define Handle_Type_NONE  0  //NONE

#define Handle_Type_2     2  //JMB 无霍尔  PXBA
#define Handle_Type_3     3  //TMBB 无霍尔 进口  PXBB
#define Handle_Type_4     4  //TMBC（未生产） 无霍尔 进口
#define Handle_Type_5     5  //TMBA 无霍尔 进口
#define Handle_Type_6     6  //EMBD 6万  增速
#define Handle_Type_7     7  //EMBC 7万

#define Handle_Type_8     8  //进口 无霍尔 TMBA
#define Handle_Type_9     9   //国产 有霍尔JMB-EC16-3W
#define Handle_Type_10    10  //国产 有霍尔TMBB-EC16-6W
#define Handle_Type_11    11  //国产 有霍尔TMBC-EC16-6W
#define Handle_Type_12    12  //国产 有霍尔TMBA-EC13-6W

#define Handle_Type_13    13  //国产 无霍尔JMB-EC16-3W  (进口国产通用一个码)
#define Handle_Type_14    14  //国产 无霍尔TMBB-EC16-6W (进口国产通用一个码)
#define Handle_Type_15    15  //国产 无霍尔TMBC-EC16-6W
#define Handle_Type_16    16  //国产 无霍尔TMBA-EC13-6W

#define Handle_Type_17    17  //EMBC
#define Handle_Type_18    18  //EMBD
#define Handle_Type_19    19  //
#define Handle_Type_20    20  //

#define Handle_Type_22    22  //PXBA
#define Handle_Type_23    23  //PXBB

#ifndef KEY_NONE
#define KEY_NONE          0xFF  // 屏幕键值空状态；若 screenkey.h 已定义，则沿用桥接定义。
#endif
#define KEY_ZERO          0
#define KEY_ONE           1
#define KEY_TWO           2
#define KEY_THREE         3
#define KEY_FOUR          4
#define KEY_FIVE          5
#define KEY_SIX           6
#define KEY_SEVEN         7
#define KEY_EIGHT         8
#define KEY_NINE          9

#define KEY_RETURN        10    //返回
#define KEY_ENTER         11    //确认
#define KEY_NEXTPAGE      12    //下一页
#define KEY_LASTPAGE      13    //上一页
#define KEY_REINPUT       14    //重新输入

#ifndef KEY_STORAGEMIN
#define KEY_STORAGEMIN    15    //存储最小值
#endif
#ifndef KEY_STORAGEMAX
#define KEY_STORAGEMAX    16    //存储最大值
#endif

#ifndef KEY_STORAGEMIN2
#define KEY_STORAGEMIN2    80    //存储最小值2
#endif
#ifndef KEY_STORAGEMAX2
#define KEY_STORAGEMAX2    81    //存储最大值2
#endif

#ifndef KEY_STORAMEDIAN
#define KEY_STORAMEDIAN     82    //存储中间值
#endif
#ifndef KEY_STORAMEDIAN2
#define KEY_STORAMEDIAN2    83    //存储中间值2
#endif
#ifndef M_KEY_FOOT
#define M_KEY_FOOT            85   //脚踏中键桥接到定标页事件。
#endif
#ifndef L_KEY_FOOT
#define L_KEY_FOOT            86   //脚踏左键桥接到定标页事件。
#endif
#ifndef R_KEY_FOOT
#define R_KEY_FOOT            87   //脚踏右键桥接到定标页事件。
#endif


#define KEY_JMB           17    //手柄1
#define KEY_TMBA          18    //‘’
#define KEY_TMBB          19    //‘’
#define KEY_TMBC          20    //‘’

#ifndef KEY_CONTINUOUSCLICK
#define KEY_CONTINUOUSCLICK  21  //LOGO的连续点击
#endif

#define KEY_HANDLEONE     22    //手柄1
#define KEY_HANDLETWO     23    //手柄2
#define KEY_HANDLETHREE   24    //手柄3
#define KEY_HANDLEFOUR    25    //手柄4

#define KEY_SECONDGEAR    26    //Ⅱ档
#define KEY_FLOWRATESWITCH_B  27  //流量开关_B

#define KEY_FORWARD       28    //正向
#define KEY_REVERSE       29    //反向
#define KEY_RECIPROCATING  30   //往复

#define KEY_FOOTCTRL      31    //脚控
#define KEY_MANUALCTRL    32    //手控

#define KEY_EMPTY_B         33    //排空B

#define KEY_AUTOMANUAL    34    //自动 手动
#define KEY_PLANING       35    //刨削
#define KEY_GRINDINGHEAD  36    //磨头
#define KEY_OPENLEFT      37    //左开口
#define KEY_OPENRIGHT     38    //右开口

#define KEY_SPEEDREDUCE   39    //速度减
#define KEY_SPEEDLONGPRESSREDUCE  40  //速度减长按
#define KEY_SPEEDENDLONGPRESSREDUCE  41  //速度减长按停止

#define KEY_SPEEDPLUS   42    //速度加
#define KEY_SPEEDLONGPRESSPLUS  43  //速度加长按
#define KEY_SPEEDENDLONGPRESSPLUS  44  //速度加长按停止

#define KEY_FREQREDUCE   45   //频率减
#define KEY_FREQPLUS     46   //频率加

#define KEY_FLOWRATEREDUCE_B   47    //B流量减
#define KEY_FLOWRATELONGPRESSREDUCE_B  48  //B流量减长按
#define KEY_FLOWRATEENDLONGPRESSREDUCE_B  49  //B流量减长按停止

#define KEY_FLOWRATEPLUS_B   50    //B流量加
#define KEY_FLOWRATELONGPRESSPLUS_B  51  //B流量加长按
#define KEY_FLOWRATEENDLONGPRESSPLUS_B  52  //B流量加长按停止


#define KEY_FLOWRATEREDUCE_A   53    //A流量减
#define KEY_FLOWRATELONGPRESSREDUCE_A  54  //A流量减长按
#define KEY_FLOWRATEENDLONGPRESSREDUCE_A  55  //A流量减长按停止

#define KEY_FLOWRATEPLUS_A   56    //A流量加
#define KEY_FLOWRATELONGPRESSPLUS_A  57  //A流量加长按
#define KEY_FLOWRATEENDLONGPRESSPLUS_A  58  //A流量加长按停止
#define KEY_FLOWRATESWITCH_A  59  //流量开关_A
#define KEY_EMPTY_A         60    //排空A

#define KEY_SWITCH_A 61
#define KEY_SWITCH_B 62



#define PUMPMLUNITMAX  70  //ML

#define FREQUENCYMAX  40  //最大频率

/**************操作指令码定义******************************/
/*DS2431 ROM功能命令*/
#define Rom_Read_Cmd                0x33    //Read ROM
#define Rom_Match_Cmd               0x55    //Match ROM
#define Rom_Skip_Cmd                0xCC    //Skip ROM
#define Rom_Search_Cmd              0xF0    //Search ROM

/*DS2431 存储器功能命令*/
#define Memory_Read_Cmd             0xF0    //Read Memory
#define Scratchpad_Read_Cmd         0xAA    //Read Scratchpad
#define Scratchpad_Write_Cmd        0x0F    //Write Scratchpad
#define Scratchpad_Copy_Cmd         0x55    //Copy Scratchpad

//1_管理员模式界面 密码输入*填入位置
#define Addr_Pic_Page1_1  0x1001
#define Addr_Pic_Page1_2  0x1002
#define Addr_Pic_Page1_3  0x1003
#define Addr_Pic_Page1_4  0x1004
#define Addr_Pic_Page1_5  0x1005
#define Addr_Pic_Page1_6  0x1006
#define Addr_Pic_Page1_7  0x1007

#define Addr_ICL_Page1_1  100  //‘*’
#define Addr_ICL_Page1_2  101  //密码输入错误提示图片

#define Address_ICL_Page4_1 102    //圈中不选中
#define Address_ICL_Page4_2 103    //圈中选中


//手柄类型
typedef struct ModelConfigTag
{
  uint8_t type[16];
	uint8_t HandlePortA;  //A接口  1有连接  0无连接
	uint8_t HandlePortB;  //B接口  1有连接  0无连接	
} ModelConfig;

//接口(AABB)
typedef struct HandleDataTag
{
  uint8_t EPCBuffAA[2][20];

  uint8_t Ds2431BuffBB[2][20]; //一体式单总线读取数据
  uint8_t IntegratedOffTiming[2];    //一体式离线扫描计时
} HandleData;

//接口连接状态，对应手柄类型
typedef struct InterfaceTag
{
  uint8_t Interface[5];   //1连接 2连接并选中 【下标：0 --1号手柄，1 --2号手柄，2 --3号手柄，3 --4号手柄, 4 --5号手柄】

  uint8_t HandleType[5];  //对应接口的手柄类型

  uint8_t BeSelectNum;  //当前选中的手柄号	0xff未选中 1选中1号 2... 3... 4...  ----1

  uint8_t LcdKeyValue; //当前UI的按键选择值

  uint8_t DisplayUpdateFlag;  //接口更新表标志

  uint8_t InterfaceSwitchNo1;  //1号接口选择 1主、2副
  uint8_t InterfaceSwitchNo2;  //2号接口选择 1主、2副
  uint8_t InterfaceSwitchNo3;  //2号接口选择 1主、2副

  uint8_t BackgroundPag;   //当前显示的背景页号

  uint8_t BeSelectIndexUI; //显示手柄被选中的下标（脚踏切换）----2

} InterfaceData;

//脚踏数据
typedef struct FootPedalDataTag
{
  uint16_t FootPedalADValue;  //脚踏AD值
//	uint16_t FootPedalADValue_R;  //脚踏R值
//  uint16_t FootPedalADValue_L;  //脚踏L值
  uint16_t FootPedalMemoryLValue;  //脚踏存储低值
  uint16_t FootPedalMemoryHValue;  //脚踏存储高值

  uint16_t FootPedalADValue_Right;
  uint16_t FootPedalADValue_Left;	
	
  uint16_t FootPedalMemoryLValue_Right;  //脚踏存储低值右
  uint16_t FootPedalMemoryLValue_Left;   //脚踏存储低值左	
  uint16_t FootPedalMemoryHValue_Right;  //脚踏存储高值右
  uint16_t FootPedalMemoryHValue_Left;   //脚踏存储高值左	
  uint16_t FootPedalMemoryMValue_Right;  //脚踏存储中值右
  uint16_t FootPedalMemoryMValue_Left;   //脚踏存储中值左		
	
	uint8_t FootPedalType ;  // //脚踏连接型号   0是单踏板   1是双踏板
  uint8_t FootPedalConnectOkNo;  //JT_Connect_kono;  //脚踏连接与否

  uint8_t FootPedalConnectFlag;  //脚踏连接标志  ----3
  uint8_t FootPedalOffTimes;  //脚踏离线计时器

  uint8_t FootPedalReadFlag;  //JT_Read_Flag 脚踏值读取OK标志  ----2
  uint8_t FootPedalLiftFlag;  //OFF_JT1_RUN 脚踏抬起标志

  uint8_t FootPedalKeyValue;   //脚踏键值 1手柄切换、2窗口切换、3参数+、4参数-

  uint8_t FootPedalLiftTimeCnt;  //手柄切换，脚踏抬起标志置位计时

} FootPedalData;

//UI显示数据
typedef struct UIDisplayDataTag
{
  uint8_t UIBeSelectWindow;    //被脚踏选中的框

  uint8_t UIDisplay0x1403;     //往复角度图片 0隐 1显

  uint8_t UIDisplay0x1303;  //转速栏

  uint8_t UIDisplay0x1304;  //频率 or 挡位

  uint8_t UIDisplay0x1305;  //泵2

  uint8_t UIDisplay0x1311;  //往复
  uint8_t UIDisplay0x1310;  //正向
  uint8_t UIDisplay0x1316;  //反向

  uint8_t UIDisplay0x1312;  //脚踏
  uint8_t UIDisplay0x1313;  //手控
	
  uint8_t UIDisplay0x1500;  //A手柄
  uint8_t UIDisplay0x1501;  //B手柄	
	
  uint8_t UIDisplay0x1318;  //泵1
	
} UIDisplayData;

//设备运行参数
typedef struct SystemRunParamTag
{
  uint32_t MaxSetMotorSpeed;  //connect/lcdkey/display = 0
  uint32_t MinSetMotorSpeed;  //connect/lcdkey/display = 0
  uint32_t StartSetMotorSpeed;  //connect/lcdkey/display = 0

  uint32_t SetISpeed;  //connect/lcdkey/display = 0
  uint32_t SetIISpeed;  //connect/lcdkey/display = 0
  uint32_t SetIIISpeed;  //connect/lcdkey/display = 0

  uint32_t MotorSetSpeed;  //main/Connect/Pump/WARN/Drive/LcdKey/Display = 0  ----1

  uint32_t MotorRealSpeed;  //Motor_Real_Speed

  uint32_t MotorReadSpeed;  //Motor_Read_Speed  //驱动返回的实际速度

  uint32_t PumpMotorSetSpeed_B;  //泵电机速度B STMotor_Set_Speed = 0

  uint32_t SystemTime;  //记录系统时钟ms

  uint32_t SustainedBuzzerFlag;  //报警蜂鸣器持续响标志

  uint16_t BeepTimeMS;  //蜂鸣器响时长 Beep_TimeMS = 0

  uint16_t DirCurrent;  //驱动电流

  uint8_t EncryptionCheckFlag;  //UID加密检查标志  ----2

  uint8_t DriveBoardConnectFlag;     //驱动板连接标志
  uint8_t DriveBoardOffTimes;  //驱动板离线计时器

  uint8_t MotorType;  //电机类型 EC13 EC16
  uint8_t MotorRun;         //Motor_rut 电机正工作 1运行 0停止

  uint8_t WarningBeep;    //Warning_Beep 报警类型的Beep声
  uint8_t WarningBeepPwErrF;  //Warning_Beep3

  uint8_t MotorNum;     //当前运行电机 1 2 3 0无选中电机  ----3

  uint8_t PumpFlowEEPOM;  //（耳磨）存储的流量
  float FlowRateB;        //流量B NumberFluidSetB

  uint8_t Frequency;       //频率 NumberHZSet

  uint8_t PumpONOFF_B;  //B泵的开关标志 1开启  2关闭
  uint8_t PumpDrain_B;  //B泵的排空标志 0未开启排空  1排空
  uint8_t Pump5sRun_B;   //B泵运行5s标志
  uint8_t PumpSteping5sNum_B;   //B泵运行5s计次标志  ----4

  uint8_t Motor2StopTime;    //电机2停止时间计数器  Motor2_stop_Time = 0;

  uint8_t HandleAllowSwitchFlag;  // 禁止UI上切换手柄操作标志 Display_KEY_Flag = 0;

  uint8_t StartingMethod;   //启动方式 JTHandleCtrlFlag; //1手控/0脚控 JT_Handle_ControlFlag = 0

  uint8_t StuckFlag;        //电机卡死标志 Stuck_Flag = 0;
  uint8_t StuckFlag3;        //u8 Stuck_Flag3 = 0;

  uint8_t CommunicatFlag;   //驱动板连接标志

  uint8_t HALLErrFlag;  //霍尔错误标志  ----5

  uint8_t HandleKeyValue[2];  //手柄键值 电机2

  uint8_t WarnID;     //报警ID

  uint8_t KeyValue;     //屏幕键值
  uint8_t KeyLongPressValue;   //屏幕键值“长按”

  uint8_t DJManualRefreshFlag;  //刀具信息刷新标志 DJ_Manual_Refresh_Flag = 0
  uint8_t ReciveOKTime;  //时间计数器（2号手柄 分离试手柄 检测刀具是否连接） Recive_oktime = 0
  uint8_t CutterON;  // = 0
  uint8_t CutterOFF; // = 0  ----6

  uint8_t DJOldValueFlag[2];  // 0 刨  1 磨  默认0 DJ_old_Value_Flag = 0 该接口上一次的刀具类型

  uint8_t ParamInitFlag[5];   //接口数据是否需初始化标志判断

  uint8_t DJOldInit[2];  //2号（分体式手柄）刀具是否已有数据标志 DJ_old_init = 0;  ----7

  uint8_t CompileTime[6];  //程序编译时间  ----8
	
  float FlowRateA;        //流量A NumberFluidSetA
  uint8_t PumpONOFF_A;  //泵的开关标志 1开启  2关闭
  uint8_t PumpDrain_A;  //泵的排空标志 0未开启排空  1排空
  uint8_t Pump5sRun_A;   //泵运行5s标志
  uint8_t PumpSteping5sNum_A;   //泵运行5s计次标志  ----4	
  uint32_t PumpMotorSetSpeed_A;  //泵电机速度A STMotor_Set_Speed = 0 
	
  uint8_t PumpModel_A;   //A泵模式
  uint8_t PumpModel_B;   //B泵模式	 
	
  uint8_t PumpPourIntoONOFF_A;   //A泵灌注开关
  uint8_t PumpPourIntoONOFF_B;   //B泵灌注开关	
  uint8_t PumpPourIntoVelocityA;	 //A泵灌加减值
  uint16_t PumpPourIntoVelocityB;  //B泵灌加减值
 
} SystemRunParam;

//设备初始及设置参数
typedef struct SystemSetParamTag
{
  uint32_t MaxMotorSpeed;  //最大转速  MaxSetMotorSpeed
  uint32_t StartMotorSpeed;  //起始转速  StartSetMotorSpeed
  uint32_t MinMotorSpeed;	  //最小转速  MinSetMotorSpeed

  uint32_t ISpeed;  //Ⅰ档转速  SetISpeed
  uint32_t IISpeed;  //Ⅱ档转速  SetIISpeed
  uint32_t IIISpeed;	  //Ⅲ档转速  SetIIISpeed

  uint32_t RunMotorSpeed;  //运行转速  ----1  MotorSetSpeed

  uint8_t PumpVelocitySetB;   //B流速0 ~ 70mL/min 或 0 ~ 1.2L/min  NumberFluidSet

  uint8_t HzSet;      //频率0.5Hz ~ 4.0Hz  NumberHZSet

  uint8_t PumpOffOnB;  //泵开关0：关 1：开  SteppingMotorOFFON

  uint8_t DJAutoGetFlag;  //刀具"自动获取"标志 0：自动设别 1：手动选择 
  uint8_t DJSetPDMT;  //0刨刀  1磨头 注：仅刀具“自动获取”标志设置为“1：手动选择”时，有用

  uint8_t ReciprocatingFlag;  //往复标志 0：单向 1：往复  DXWFFlag
  uint8_t MotorModel; //电机运行模式  ----2  1：往复 2：正向 3：反向 注：是否存在“往复”模式由刀具信息决定，仅刨刀有“往复”模式 MotorModel2 MotorModel3
//  uint8_t ForwardReverseFlag;  //正向\反向 在往复标志为0时该标志有效 CWCCW

  uint8_t FootAndHandCtrl; //1：手控 0：脚控 注：仅PxBA支持手控模式 JTHandleCtrlFlag

  uint8_t GearPositionHz;  //挡位 频率 1：频率 4：Ⅰ档 5：Ⅱ档 6：Ⅲ档

  uint8_t ModeReciprocating; //往复  ---- UI显示的选中标志 StateWF
  uint8_t ModeForward;  //正向 StateDX
  uint8_t ModeReverse;  //反向
  uint8_t PumpVelocitySetA;   //A流速0 ~ 70mL/min 或 0 ~ 1.2L/min  NumberFluidSet
	uint8_t PumpOffOnA;   //A泵开关0：关 1：开  SteppingMotorOFFON
	
} SystemSetParam;

 






//按接口存参数


#endif  //__DATA_H


