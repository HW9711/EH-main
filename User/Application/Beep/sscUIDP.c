#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscUIDP.h"
#include "lcd.h"
#include "Pubinterface.h"
#include "screen_address.h"
#include "sscBEEP.h"
#include "motoruartdata.h"

uint8_t DisPlayData[10] = {0};

QueueHandle_t UIDPMsgQueue = NULL;
kernel_task_t UIDISPLAYBehaviorHandle;

typedef struct
{
	uint8_t areaId;//a泵区域，B泵区域，速度区域，等等

	bool enable_flag;//激活
	uint8_t Value[10];
}UIDPMessage_t ;

static UIDPMessage_t s_uidp_last_msg = {0};//记录上一次成功投递的 UI 消息，用于复用副工程的重复刷新过滤效果。
static uint8_t s_uidp_last_valid = 0U;//上一次 UI 消息是否有效，避免上电第一帧被误判为重复消息。
/* UIDP 队列需要承接上电 A/B 手柄和 RFID 二次刷新连续消息，容量过小会导致后到的 B 通道或刀具规格消息丢失。 */
#define UIDP_QUEUE_LENGTH 48U
/* 主运行页开机后 3 秒内允许补刷，覆盖手柄 EEPROM 和 RFID 首次识别完成的窗口。 */
#define UIDP_STARTUP_REPLAY_TOTAL_TICKS 300U
/* 每 200ms 尝试补刷一次，避免每个 10ms 周期都重复发送整页 UI。 */
#define UIDP_STARTUP_REPLAY_PERIOD_TICKS 20U
/* 手柄拔出业务状态落地后补发两次 A/B 权威连接快照，覆盖屏幕串口或显示队列的低概率单帧丢失。 */
#define UIDP_HANDLE_REPLAY_COUNT 2U
/* 显示任务周期为 10ms，6 个周期对应 60ms；两次补发约发生在拔出后的 60ms 和 120ms。 */
#define UIDP_HANDLE_REPLAY_PERIOD_TICKS 6U
/* 泵档位正常运行态固定使用每档渐隐组的第 0 帧，取消动画后续如需播放再传入 1~4 帧。 */
#define UIDP_PUMP_GEAR_RUN_FRAME 0U
/* 每个显示任务周期最多连续处理 12 条 UI 消息，压缩上电主运行页控件逐个加载的可见时间。 */
#define UIDP_DISPLAY_BATCH_LIMIT 12U
/* 泵档位表每档包含 5 张渐隐资源，0 档没有渐隐时 5 个槽位都保持同一张图。 */
#define UIDP_PUMP_GEAR_FRAME_COUNT 5U
/* 泵档位最多显示 0~10 共 11 档，和屏幕工程 249/349 起始资源保持一致。 */
#define UIDP_PUMP_GEAR_COUNT 11U
static uint16_t s_uidp_startup_replay_ticks = 0U; /* 开机状态补刷剩余调度次数，倒计时结束后不再重发。 */
static uint16_t s_uidp_startup_replay_period = 0U; /* 补刷间隔计数，确保 UI 队列有时间处理上一批消息。 */
static volatile uint8_t s_uidp_handle_replay_remaining = 0U; /* 拔出后尚未发送的 A/B 连接快照次数，由插拔业务任务启动、显示任务消费。 */
static volatile uint8_t s_uidp_handle_replay_period = 0U; /* 拔出连接快照的 10ms 周期倒计时，避免连续消息紧挨发送。 */

/* B 泵档位资源：249 为 0 档；250~254 为 1 档取消渐隐 5 帧；后续每 10 递增一档。 */
static const uint16_t s_uidp_pump_b_gear_pic[UIDP_PUMP_GEAR_COUNT][UIDP_PUMP_GEAR_FRAME_COUNT] =
{
	{249U, 249U, 249U, 249U, 249U},
	{250U, 251U, 252U, 253U, 254U},
	{260U, 261U, 262U, 263U, 264U},
	{270U, 271U, 272U, 273U, 274U},
	{280U, 281U, 282U, 283U, 284U},
	{290U, 291U, 292U, 293U, 294U},
	{300U, 301U, 302U, 303U, 304U},
	{310U, 311U, 312U, 313U, 314U},
	{320U, 321U, 322U, 323U, 324U},
	{330U, 331U, 332U, 333U, 334U},
	{340U, 341U, 342U, 343U, 344U}
};

/* A 泵档位资源：349 为 0 档；350~354 为 1 档取消渐隐 5 帧；后续每 10 递增一档。 */
static const uint16_t s_uidp_pump_a_gear_pic[UIDP_PUMP_GEAR_COUNT][UIDP_PUMP_GEAR_FRAME_COUNT] =
{
	{349U, 349U, 349U, 349U, 349U},
	{350U, 351U, 352U, 353U, 354U},
	{360U, 361U, 362U, 363U, 364U},
	{370U, 371U, 372U, 373U, 374U},
	{380U, 381U, 382U, 383U, 384U},
	{390U, 391U, 392U, 393U, 394U},
	{400U, 401U, 402U, 403U, 404U},
	{410U, 411U, 412U, 413U, 414U},
	{420U, 421U, 422U, 423U, 424U},
	{430U, 431U, 432U, 433U, 434U},
	{440U, 441U, 442U, 443U, 444U}
};

static void UIDPQueue_Init(void)
{
    UIDPMsgQueue = Kernel_QueueCreate(UIDP_QUEUE_LENGTH, sizeof(UIDPMessage_t), "UIDPMsgQueue");
}

/*
 * 函数功能：按泵类型和流量值换算屏幕 0~10 档档位。
 * 输入参数：pump_type 为 DRAWWATER/INJECTWATER/POURWATER；value 为当前泵流量/速度值。
 * 返回参数：0~10 档位，超出范围时按 10 档钳位。
 */
static uint8_t UIDP_PumpGearFromValue(uint8_t pump_type, uint16_t value)
{
	uint8_t gear_value = 0U; /* 默认 0 档，保证未知类型或 0 流量时不误显示蓝色进度。 */

	if(pump_type==INJECTWATER)//注水
	{
		if(value==0U)gear_value=0U; /* 注水流量为 0 时显示停止档，不点亮进度。 */
		else if(value<=5U)gear_value=1U; /* 1~5 档流量归入第 1 格，保持低流量可见。 */
		else if(value<=10U)gear_value=2U; /* 6~10 档流量映射到第 2 格。 */
		else if(value<=15U)gear_value=3U; /* 11~15 档流量映射到第 3 格。 */
		else if(value<=20U)gear_value=4U; /* 16~20 档流量映射到第 4 格。 */
		else if(value<=25U)gear_value=5U; /* 21~25 档流量映射到第 5 格。 */
		else if(value<=30U)gear_value=6U; /* 26~30 档流量映射到第 6 格。 */
		else if(value<=40U)gear_value=7U; /* 31~40 档流量映射到第 7 格。 */
		else if(value<=50U)gear_value=8U; /* 41~50 档流量映射到第 8 格。 */
		else if(value<=60U)gear_value=9U; /* 51~60 档流量映射到第 9 格。 */
		else gear_value=10U;
	}
	else if(pump_type==DRAWWATER)//抽吸
	{
		if(value==8U)gear_value=7U; /* 抽吸泵 8 档对应屏幕第 7 格。 */
		else if(value==10U)gear_value=8U; /* 抽吸泵 10 档对应屏幕第 8 格。 */
		else if(value==12U)gear_value=9U; /* 抽吸泵 12 档对应屏幕第 9 格。 */
		else if(value>=15U)gear_value=10U; /* 抽吸泵 15 档及以上统一显示满格，避免图片索引越界。 */
		else gear_value=(uint8_t)value;
	}
	else if(pump_type==POURWATER)//灌注
	{
		if(value==0U)gear_value=0U; /* 灌注流量为 0 时显示停止档。 */
		else if(value<=30U)gear_value=1U; /* 1~30ml 映射到第 1 格。 */
		else if(value<=60U)gear_value=2U; /* 31~60ml 映射到第 2 格。 */
		else if(value<=90U)gear_value=3U; /* 61~90ml 映射到第 3 格。 */
		else if(value<=120U)gear_value=4U; /* 91~120ml 映射到第 4 格。 */
		else if(value<=150U)gear_value=5U; /* 121~150ml 映射到第 5 格。 */
		else if(value<=180U)gear_value=6U; /* 151~180ml 映射到第 6 格。 */
		else if(value<=210U)gear_value=7U; /* 181~210ml 映射到第 7 格。 */
		else if(value<=240U)gear_value=8U; /* 211~240ml 映射到第 8 格。 */
		else if(value<=270U)gear_value=9U; /* 241~270ml 映射到第 9 格。 */
		else gear_value=10U;
	}

	if(gear_value >= UIDP_PUMP_GEAR_COUNT)
	{
		gear_value = (uint8_t)(UIDP_PUMP_GEAR_COUNT - 1U); /* 防止抽吸泵直接传入异常大值越过图片表。 */
	}

	return gear_value;
}

/*
 * 函数功能：按 A/B 泵、档位和渐隐帧号查找新屏泵区域图片资源。
 * 输入参数：pump_id 为 1 表示 A 泵、2 表示 B 泵；gear_value 为 0~10 档；fade_frame 为 0~4 渐隐帧。
 * 返回参数：屏幕图片资源号，异常参数返回对应泵 0 档图。
 */
static uint16_t UIDP_PumpGearPicture(uint8_t pump_id, uint8_t gear_value, uint8_t fade_frame)
{
	if(gear_value >= UIDP_PUMP_GEAR_COUNT)
	{
		gear_value = 0U; /* 异常档位回到 0 档，避免访问越界导致屏幕显示错图。 */
	}
	if(fade_frame >= UIDP_PUMP_GEAR_FRAME_COUNT)
	{
		fade_frame = UIDP_PUMP_GEAR_RUN_FRAME; /* 当前固件只刷新运行态，异常帧号回到第 0 帧。 */
	}
	if(pump_id == 1U)
	{
		return s_uidp_pump_a_gear_pic[gear_value][fade_frame]; /* A 泵使用 349~444 资源表。 */
	}
	if(pump_id == 2U)
	{
		return s_uidp_pump_b_gear_pic[gear_value][fade_frame]; /* B 泵使用 249~344 资源表。 */
	}
	return s_uidp_pump_b_gear_pic[0U][UIDP_PUMP_GEAR_RUN_FRAME]; /* 未知泵号按 B 泵 0 档兜底，避免返回 0 号图片。 */
}

/*
 * 函数功能：按泵类型和启用状态选择泵类型标题图片。
 * 输入参数：pump_type 为泵业务类型；enable_flag 表示该泵是否可用。
 * 返回参数：210~215 范围内的新屏泵类型图片号。
 */
static uint16_t UIDP_PumpTypePicture(uint8_t pump_type, bool enable_flag)
{
	switch(pump_type)
	{
		case DRAWWATER:
			return enable_flag ? 215U : 214U; /* 抽吸泵标题使用 214 灰色、215 黄色，和 EX8 屏幕表格保持一致。 */
		case POURWATER:
			return enable_flag ? 213U : 212U; /* 灌注泵标题使用 212/213 一组。 */
		case INJECTWATER:
			return enable_flag ? 211U : 210U; /* 注水泵识别后必须显示 211 黄色图标，停用态显示 210 灰色。 */
		default:
			return 0U; /* 未识别类型不再兜底为抽吸泵，调用侧负责隐藏类型标题。 */
	}
}

/*
 * 函数功能：判断泵类型是否已经由 CS1237 设备码识别为有效业务类型。
 * 输入参数：pump_type 为 DRAWWATER、INJECTWATER、POURWATER 或 0。
 * 返回参数：true 表示可显示泵类型标题，false 表示未知类型需要隐藏标题。
 */
static bool UIDP_IsPumpTypeKnown(uint8_t pump_type)
{
	switch(pump_type)
	{
		case DRAWWATER:
		case POURWATER:
		case INJECTWATER:
			return true; /* 三种已确认泵类型都有对应标题资源，允许写入 A/B 类型 VP。 */
		default:
			return false; /* 未识别或备用设备码不能显示成抽吸泵，避免误导现场判断。 */
	}
}

/*
 * 函数功能：按泵类型、启用状态和运行状态选择 A/B 共用的启停按钮图片。
 * 输入参数：button_type 为泵业务类型；enable_flag 表示按钮是否可点；run_flag 表示泵是否运行。
 * 返回参数：200~205 范围内的新屏泵启停按钮图片号。
 */
static uint16_t UIDP_PumpButtonPicture(uint8_t button_type, bool enable_flag, bool run_flag)
{
	switch(button_type)
	{
		case DRAWWATER:
		case POURWATER:
			return enable_flag ? (run_flag ? 202U : 201U) : 200U; /* 抽吸泵和注水泵使用启动按钮组：200 禁用、201 停止、202 运行。 */
		case INJECTWATER:
			return enable_flag ? (run_flag ? 205U : 204U) : 203U; /* 灌注泵使用排空按钮组：203 禁用、204 停止、205 运行。 */
		default:
			return 200U; /* 未识别泵类型统一回到启动按钮禁用图，避免误显示为可操作状态。 */
	}
}

/*
 * 函数功能：向屏幕显示任务投递一个区域刷新消息。
 * 输入参数：areaId 为 UI 区域编号；enable_flag 为显示开关；Value 为最多 10 字节的区域参数，允许为空。
 * 返回参数：无。
 */
void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value)
{
	if(UIDPMsgQueue == NULL) return; /* 显示队列尚未创建时不能投递刷新消息，直接返回避免访问空句柄。 */
    UIDPMessage_t msg;
    msg.areaId = areaId;
	msg.enable_flag = enable_flag;
	if(Value != NULL)
	{
		memcpy(msg.Value, Value, 10);//调用方传入有效参数时完整复制，保证显示任务读取到稳定快照
	}
	else
	{
		memset(msg.Value, 0, sizeof(msg.Value));//开机初始化和清屏类消息不需要参数，统一补零避免空指针访问
	}
	if((s_uidp_last_valid != 0U) && /* 已有相同区域、状态和参数的成功消息时，无需重复占用屏幕队列。 */
	   (s_uidp_last_msg.areaId == msg.areaId) &&
	   (s_uidp_last_msg.enable_flag == msg.enable_flag) &&
	   (memcmp(s_uidp_last_msg.Value, msg.Value, sizeof(msg.Value)) == 0))
	{
		return;//同一区域、同一开关状态和同一参数不重复入队，降低屏幕串口刷新压力。
	}
    if(Kernel_QueueSend(UIDPMsgQueue, &msg, 10) == pdPASS)
	{
		s_uidp_last_msg = msg;//只在投递成功后更新去重缓存，避免队列满时吞掉下一次有效刷新。
		s_uidp_last_valid = 1U;//标记去重缓存已建立，后续重复 UI 消息才允许被过滤。
	}
}

/*
 * 函数功能：启动一次手柄拔出后的短时显示补发，在不重置 UI 队列的前提下重新发送两次 A/B 完整连接快照。
 * 输入参数：无。
 * 返回参数：无。
 */
void UIDP_RequestHandleDisplayReplay(void)
{
	s_uidp_handle_replay_remaining = UIDP_HANDLE_REPLAY_COUNT; /* 每次真实拔出都重新装载两次补发额度，覆盖最新 A/B 业务在线状态。 */
	s_uidp_handle_replay_period = UIDP_HANDLE_REPLAY_PERIOD_TICKS; /* 首次补发延迟约 60ms，先让本次掉线、报警和参数清屏消息完成发送。 */
}

/*
 * 函数功能：刷新 A/B 泵流量可视化区域，按新屏 5 帧渐隐资源表查图。
 * 输入参数：pump_id 为 1 表示 A 泵、2 表示 B 泵；pump_type 为泵业务类型；value 为当前流量值。
 * 返回参数：无。
 */
void PumpGeardisplay(uint8_t pump_id ,uint8_t pump_type,uint16_t value)
{
	uint8_t gear_value = UIDP_PumpGearFromValue(pump_type, value); /* 先把不同泵类型的流量换算成统一 0~10 档。 */
	uint16_t pic_value = UIDP_PumpGearPicture(pump_id, gear_value, UIDP_PUMP_GEAR_RUN_FRAME); /* 当前正常刷新使用每档第 0 帧运行态。 */

	if(pump_id==1U)//A泵
	{
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_GEAR_AREA, pic_value); /* A 泵区域 VP 使用 UIDP_LCD_VP_PUMP_A_GEAR_AREA，资源号为 349~444。 */
	}
	else if(pump_id==2U)
	{
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_GEAR_AREA, pic_value); /* B 泵区域 VP 使用 UIDP_LCD_VP_PUMP_B_GEAR_AREA，资源号为 249~344。 */
	}
}

//A泵区域显示，参数（是否激活，流量值，单位ml或者l）
/*
 * 函数功能：刷新 A 泵区域的可用状态、泵类型图标、档位和流量数值。
 * 输入参数：enable_flag 表示 A 泵区域是否可用；pump_type 表示 DRAWWATER/INJECTWATER/POURWATER 业务类型；pump_value 表示当前显示流量。
 * 返回参数：无。
 */
void UIPUMPADP(bool enable_flag,uint8_t pump_type,uint16_t pump_value )
{

	if(!enable_flag)//A区域暗灭
	{
		if(UIDP_IsPumpTypeKnown(pump_type))
		{
			LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE, UIDP_PumpTypePicture(pump_type, false));//A 泵类型已识别时才显示对应暗态标题
		}
		else
		{
			LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_A_TYPE);//A 泵未知类型隐藏标题，避免开机默认显示为抽吸泵
		}
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_GEAR_AREA, UIDP_PumpGearPicture(1U, 0U, UIDP_PUMP_GEAR_RUN_FRAME));//A 泵区域回到 0 档 349
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_UNIT, 220U);//A 泵单位暗态资源，当前导出资源中 220/221 为单位图
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON, UIDP_PumpButtonPicture(pump_type, false, false));//A 泵启动按钮暗态/停止图
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_PLUS, 222U);//A 泵加按钮暗态，使用 8 寸屏统一加号失能资源
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_MINUS, 223U);//A 泵减按钮暗态，使用 8 寸屏统一减号失能资源
		LCD_Disappear_Number(UIDP_LCD_SP_PUMP_A_VALUE);//流量值
	}
	else
	{
		LCD_Show_Number (UIDP_LCD_SP_PUMP_A_VALUE, UIDP_LCD_VP_PUMP_A_VALUE);//显示
		LCD_Show_4byte_Number(UIDP_LCD_VP_PUMP_A_VALUE,pump_value);
		switch(pump_type)
		{
			case DRAWWATER://抽吸
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE, UIDP_PumpTypePicture(DRAWWATER, true));//A 泵类型 VP 使用 UIDP_LCD_VP_PUMP_A_TYPE 显示抽吸亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_UNIT, 221U);//A 泵单位亮态资源
				PumpGeardisplay(1,DRAWWATER,pump_value);
			break;
			case POURWATER://灌注
			    LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE, UIDP_PumpTypePicture(POURWATER, true));//A 泵类型 VP 使用 UIDP_LCD_VP_PUMP_A_TYPE 显示灌注亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_UNIT, 221U);//A 泵单位亮态资源
				PumpGeardisplay(1,POURWATER,pump_value);
			break;
			case INJECTWATER://注水
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_TYPE, UIDP_PumpTypePicture(INJECTWATER, true));//A 泵类型 VP 使用 UIDP_LCD_VP_PUMP_A_TYPE 显示注水亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_UNIT, 221U);//A 泵单位亮态资源
				PumpGeardisplay(1,INJECTWATER,pump_value);
			//排空按钮黑色
			break;
		}
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON, UIDP_PumpButtonPicture(pump_type, true, false));//档位图写完后再补画 A 侧停止态按钮，防止按钮区域被泵区刷新盖住
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_PLUS, 225U);//A 泵加按钮亮态，泵区底图刷新后最后补画避免被覆盖
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_MINUS, 224U);//A 泵减按钮亮态，泵区底图刷新后最后补画避免方向反
	}
}
//B泵区域显示，参数（是否激活，流量值，单位ml或者l）
/*
 * 函数功能：刷新 B 泵区域的可用状态、泵类型图标、档位和流量数值。
 * 输入参数：enable_flag 表示 B 泵区域是否可用；pump_type 表示 DRAWWATER/INJECTWATER/POURWATER 业务类型；pump_value 表示当前显示流量。
 * 返回参数：无。
 */
void UIPUMPBDP(bool enable_flag,uint8_t pump_type,uint16_t pump_value )
{
if(!enable_flag)//B区域暗灭
	{
		if(UIDP_IsPumpTypeKnown(pump_type ))
		{
			LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE, UIDP_PumpTypePicture(pump_type, false));//B 泵类型已识别时才显示对应暗态标题
		}
		else
		{
			LCD_Disappear_Picture(UIDP_LCD_VP_PUMP_B_TYPE);//B 泵未知类型隐藏标题，避免开机默认显示为抽吸泵
		}
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_UNIT, 220U);//B 泵单位暗态资源
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_GEAR_AREA, UIDP_PumpGearPicture(2U, 0U, UIDP_PUMP_GEAR_RUN_FRAME));//B 泵区域回到 0 档 249
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON, UIDP_PumpButtonPicture(pump_type, false, false));//B 泵启动按钮暗态/停止图
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS, 222);//B 泵加按钮暗态，当前 EX8 导出 0x1422 绑定 498/499，不能再写 A 泵 222/225 资源
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS, 223);//B 泵减按钮暗态，当前 EX8 导出 0x1424 绑定 500/501，避免按钮触控有效但图标不显示
		LCD_Disappear_Number(UIDP_LCD_SP_PUMP_B_VALUE);
	}
	else
	{

		LCD_Show_Number (UIDP_LCD_SP_PUMP_B_VALUE, UIDP_LCD_VP_PUMP_B_VALUE);
		LCD_Show_4byte_Number(UIDP_LCD_VP_PUMP_B_VALUE,pump_value);
		switch(pump_type)
		{
			case DRAWWATER://抽吸
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE, UIDP_PumpTypePicture(DRAWWATER, true));//B 泵类型 VP 使用 UIDP_LCD_VP_PUMP_B_TYPE 显示抽吸亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_UNIT, 221U);//B 泵单位亮态资源
				PumpGeardisplay(2,DRAWWATER,pump_value);
			break;
			case POURWATER://灌注
			    LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE, UIDP_PumpTypePicture(POURWATER, true));//B 泵类型 VP 使用 UIDP_LCD_VP_PUMP_B_TYPE 显示灌注亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_UNIT, 221U);//B 泵单位亮态资源

				PumpGeardisplay(2,POURWATER,pump_value);
			break;
			case INJECTWATER://注水
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_TYPE, UIDP_PumpTypePicture(INJECTWATER, true));//B 泵类型 VP 使用 UIDP_LCD_VP_PUMP_B_TYPE 显示注水亮态
				LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_UNIT, 221U);//B 泵单位亮态资源
				PumpGeardisplay(2,INJECTWATER,pump_value);
			//排空按钮黑色
			break;
		}
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON, UIDP_PumpButtonPicture(pump_type, true, false));//档位图写完后再补画 B 侧停止态按钮，防止按钮区域被泵区刷新盖住
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS, 225);//B 泵加按钮亮态，对应 0x1422 当前导出的 499 号资源
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS, 224);//B 泵减按钮亮态，对应 0x1424 当前导出的 501 号资源
	}
}
/*
 * 函数功能：刷新主运行页控制方式图标。
 * 输入参数：enable_flag 表示该控制方式是否可用；control_type 为 1 脚控、2 手控、3 触控、4 外控；light_flag 表示是否高亮。
 * 返回参数：无。
 */
void UICONTROLDP(bool enable_flag,uint8_t control_type, bool light_flag)
{
	if(enable_flag)
	{
		//传入控制模式，是否需要重写代码
		if(control_type==1)//脚踏
		{
		   if(light_flag) /* 脚控被选中时显示黄色高亮，否则保持白色可选状态。 */
		   LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT,32U);
		   else
		   LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT,31U);
		}
		else if(control_type==2)//手控
		{
			if(light_flag) /* 手控被选中时显示黄色高亮，否则保持白色可选状态。 */
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE,35U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE,34U);
		}
		else if(control_type==3)//触控，记得宏定义
		{
			light_flag?LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,38U):LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,37U);//触控入口使用 36~38 资源组
		}
		else if(control_type==4)///外部控制
		{
			if(light_flag) /* 外控已接管时显示在线高亮，否则显示普通在线图标。 */
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL,40U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL,39U);

		}
	}
	else
	{
		// 暗灭；control_type 为 0 时用于开机或无当前通道的全量初始化。
		if(control_type == 0U)
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT, 30U);//脚控按钮暗态
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE, 33U);//手控按钮默认白色可选态，未选中时不能显示黄色高亮
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH, 36U);//触控按钮默认白色可选态，按下进入触控后才显示黄色
			LCD_Disappear_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL);//外部通信未接入时隐藏小电脑图标，避免误显示为在线
		}
		else if(control_type == 1U) /* 只禁用脚控入口时，不改写其它控制方式图标。 */
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT, 30U);
		}
		else if(control_type == 2U) /* 只禁用手控入口时，把手控按钮恢复为未选状态。 */
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE, 33U);
		}
		else if(control_type == 3U) /* 只禁用触控入口时，把触控按钮恢复为暗态。 */
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH, 36U);
		}
		else if(control_type == 4U) /* 外控离线时隐藏图标，避免误显示上位机仍连接。 */
		{
			LCD_Disappear_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL);
		}
	}
}
/*
 * 函数功能：刷新主运行页方向按钮。
 * 输入参数：enable_flag 表示方向区是否可用；dir_type 为 1 正转、2 反转、3 往复；light_flag 为 1 高亮、2 暗态、其它普通态。
 * 返回参数：无。
 */
void UIDIRDP(bool enable_flag,uint8_t dir_type, uint8_t light_flag)
{
	if(enable_flag)
	{
		//传入方向，是否需要重写代码
		switch(dir_type)
		{
			case 1://顺时针
			if(light_flag == 1U) /* 1 表示正转已选中，显示黄色高亮资源。 */
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,22U);
			else if(light_flag == 2U) /* 2 表示正转不可用，显示灰色禁用资源。 */
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,20U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,21U);
			break;
			case 2://逆时针
			if(light_flag == 1U) /* 1 表示反转已选中，显示黄色高亮资源。 */
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,25U);
			else if(light_flag == 2U) /* 2 表示反转不可用，显示灰色禁用资源。 */
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,23U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,24U);
			break;
			case 3://往复
			if(light_flag == 1U) /* 1 表示往复已选中，显示黄色高亮资源。 */
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,28U);
			else if(light_flag == 2U) /* 2 表示往复不可用，显示灰色禁用资源。 */
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,26U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,27U);
			break;
		}
	}
	else
	{
		if(dir_type == 0U) /* 方向类型为 0 表示整组不可用，三个方向同时回到灰色禁用态。 */
		{
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD, 20U);
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC, 26U);
			LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE, 23U);
		}
		else
		{
			switch(dir_type)
			{
				case 1://顺时针
				LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,20U);
				break;
				case 2://逆时针
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,23U);
				break;
				case 3://往复
				LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,26U);
				break;
			}
		}
		//暗灭
	}
}
/*
 * 函数功能：刷新 A/B 手柄连接和选中状态。
 * 输入参数：enable_flag 表示手柄是否在线；handle_type 为 UI 图标类别；handle_channel 为 1/A 或 2/B；light_flag 表示当前通道是否选中。
 * 返回参数：无。
 */
void UIHANDLEDP(bool enable_flag,uint16_t handle_type,uint8_t handle_channel,uint8_t light_flag)
{

	if(enable_flag)
	{
		//传入工具，是否需要重写代码//AB通道分开显示，不互斥，防止未来双通道同时进行
		if(handle_channel==1)//A通道
		{
          		switch(handle_type)
				{
					case 1://通用磨钻
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,102):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,101);//通用磨钻手柄，101为已连接，102为已选中
					break;
					case 2://分体手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,104):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,103);//分体手柄，103为已连接，104为已选中
					break;
					case 3://一体刨手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,106):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,105);//一体刨手柄，105为已连接，106为已选中
					break;
					case 4://一体磨手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,110):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,109);//一体磨手柄，109为已连接，110为已选中
					break;
					case 5U://UI 类别 5，骨钻/空心钻等预留手柄共用骨钻资源
					case 6U://UI 类别 6，克氏针等预留手柄暂时共用骨钻资源
					case LGZ_I_ONLINES://颅骨钻一型预留，先复用新增手柄图标资源
					case LGZ_II_ONLINES://颅骨钻二型预留，先复用新增手柄图标资源
					case KSZ_I_ONLINES://克氏针一型预留，先复用新增手柄图标资源
					case KSZ_II_ONLINES://克氏针二型预留，先复用新增手柄图标资源
					case KXZ_I_ONLINES://空心钻一型预留，沿用空心钻图标资源
					case KXZ_II_ONLINES://空心钻二型预留，沿用空心钻图标资源
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,108):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,107);//骨钻/预留手柄，107为已连接，108为已选中
					break;
					case COMMON_SOCKET_ONLINES:
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,112):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,111);//公共接头 A 通道使用 8 寸屏专用图标
					break;
				}

		}
		else if(handle_channel==2)//B通道
		{
				switch(handle_type)
				{
					case 1://通用磨钻
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,132):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,131);//通用磨钻手柄，131为已连接，132为已选中
					break;
					case 2://分体手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,134):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,133);//分体手柄，133为已连接，134为已选中
					break;
					case 3://一体刨手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,136):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,135);//一体刨手柄，135为已连接，136为已选中
					break;
					case 4://一体磨手柄
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,140):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,139);//一体磨手柄，139为已连接，140为已选中
					break;
					case 5U://UI 类别 5，骨钻/空心钻等预留手柄共用骨钻资源
					case 6U://UI 类别 6，克氏针等预留手柄暂时共用骨钻资源
					case LGZ_I_ONLINES://颅骨钻一型预留，先复用新增手柄图标资源
					case LGZ_II_ONLINES://颅骨钻二型预留，先复用新增手柄图标资源
					case KSZ_I_ONLINES://克氏针一型预留，先复用新增手柄图标资源
					case KSZ_II_ONLINES://克氏针二型预留，先复用新增手柄图标资源
					case KXZ_I_ONLINES://空心钻一型预留，沿用空心钻图标资源
					case KXZ_II_ONLINES://空心钻二型预留，沿用空心钻图标资源
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,138):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,137);//骨钻/预留手柄，137为已连接，138为已选中
					break;
					case COMMON_SOCKET_ONLINES:
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,142):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,141);//公共接头 B 通道使用 8 寸屏专用图标
					break;
				}
		}
	}
	else
	{
		//暗灭
		if(handle_channel==1)//A通道
		LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,100);//无
		else if(handle_channel==2)//B通道
		LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,130);//无手柄
	}
}
/*
 * 函数功能：刷新磨头/刨刀类型显示。
 * 输入参数：enable_flag 表示刀具区是否可用；tool_type 为 1 表示刨刀、0 表示磨头。
 * 返回参数：无。
 */
void UITOOLDP(bool enable_flag,bool tool_type)
{
	// if(enable_flag)
	// {
    //     if(tool_type==1)//wanpao
	// 	{
	// 		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刨刀选中时磨头按钮显示未选中态
	// 		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,56U);//刨刀按钮显示选中态
	// 	}
	// 	else
	// 	{
	// 		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,54U);//磨头按钮显示选中态
	// 		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//磨头选中时刨刀按钮显示未选中态
	// 	}
	// }
	// else
	// {
	// 	LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刀具区禁用时磨头回到暗态
	// 	LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//刀具区禁用时刨刀回到暗态
	// }
}
/*
 * 函数功能：刷新主界面刀具规格文本。
 * 输入参数：enable_flag 表示规格区是否显示；tool_length/tool_Diameter/tool_angle 为屏幕规格值；raw_display_flag 为公共接头 EPC 原始整数显示标志。
 * 返回参数：无。
 */
void UITOOLSPECDP(bool enable_flag,uint16_t tool_length,uint8_t tool_Diameter,uint8_t tool_angle,uint8_t raw_display_flag)
{
	if(enable_flag)
	{
		LCD_Show_2byte_Number(UIDP_LCD_SP_TOOL_SPEC_COLOR,34);
		//LCD_IntegratedCutterData_Update(UIDP_LCD_VP_TOOL_SPEC_TEXT, 30, 20, 10); 
		if(raw_display_flag != 0U)
		{
			LCD_IntegratedCutterRawData_Update(UIDP_LCD_VP_TOOL_SPEC_TEXT, tool_length, tool_Diameter, tool_angle); //公共接头 EPC 按原始整数显示，避免 0x10 被格式化成 1.6。
		}
		else
		{
			LCD_IntegratedCutterData_Update(UIDP_LCD_VP_TOOL_SPEC_TEXT, tool_length, tool_Diameter, tool_angle); //PXBA/PXBB 保持原有 x10 小数直径格式。
		}
	}
	else
	{
		//消失
		LCD_Show_2byte_Number(UIDP_LCD_SP_TOOL_SPEC_COLOR,0);
	}
}

/*
 * 函数功能：刷新手动刀具选择和自动识别按钮区域。
 * 输入参数：enable_flag 表示手动磨/刨或等待结果区域是否显示；PAO_flag 为 true 表示刨刀选中、false 表示磨头选中；auto_identify_flag 为 true 表示当前处于 RFID 自动识别模式，自动识别按钮必须保持显示；tool_result_pic 为 0x1404 图片编号。
 * 返回参数：无。
 */
void UIMANUALBUTTONDP(bool enable_flag,bool PAO_flag,uint8_t auto_identify_flag,uint8_t tool_result_pic)
{
	/* 只有识别区域启用时才显示自动或手动选择内容；禁用状态下对应触控位置也应保持不可操作。 */
	if(enable_flag)
	{
		/* 自动识别模式隐藏手动磨头、刨刀按钮，避免画面无按钮时仍产生手动选择歧义。 */
		if(auto_identify_flag)
		{
			//LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_RESULT);
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_RESULT,61U);//自动识别等待或掉线时显示“自动识别”图，63/61/62 由业务层决定
			LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,51U);//自动按钮识别显示
			LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_BURR);
			LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_BLADE);
		}
		else
		{
			/* 手动识别模式显示磨头、刨刀按钮，输入层此时才允许对应触控事件进入业务处理。 */
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_RESULT,63U);
			LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,52U);//手动模式下 0x1407 显示“手动识别”
			if(PAO_flag)
			{
					LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刨刀选中时磨头按钮回未选中态
					LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,56U);//刨刀按钮显示选中态
			}
			else
			{
				LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,54U);//磨头按钮显示选中态
				LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//磨头选中时刨刀按钮回未选中态
			}
		}
	}
	else
	{
		if(auto_identify_flag)
		{
			LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,51U);//PXBA/PXBB识别成功后规格区接管显示，仍保留自动识别按钮供用户切回手动模式
		}
		else
		{
			LCD_Disappear_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE);//非自动识别状态禁用识别区时隐藏0x1407，避免普通手柄继承旧按钮
		}
		/* 规格窗口或无识别能力状态下不显示等待结果和手动磨/刨图片，避免与规格数据重叠。 */
		LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_RESULT);
		LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_BURR);
		LCD_Disappear_Picture(UIDP_LCD_VP_TOOL_BLADE);
	 }
}

/*
 * 函数功能：刷新开口定位入口。
 * 输入参数：enable_flag 表示当前刀具是否需要显示开口定位。
 * 返回参数：无。
 */
void UIORALDP(bool enable_flag)
{
	enable_flag?LCD_Show_Picture(UIDP_LCD_VP_OPEN_POSITION,50U):LCD_Disappear_Picture(UIDP_LCD_VP_OPEN_POSITION);//开口定位只使用 UIDP_LCD_VP_OPEN_POSITION 和 50 号资源
}

/*
 * 函数功能：最后一个手柄拔出后强制刷新主运行页无手柄状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void UIDP_ForceNoHandleDisplay(void)
{
	if(UIDPMsgQueue != NULL)
	{
		xQueueReset(UIDPMsgQueue); /* 清掉旧 UI 队列，防止拔出后旧方向或自动识别消息晚到覆盖无手柄状态。 */
		s_uidp_last_valid = 0U;	 /* 队列已被清空时同步释放去重缓存，保证后续无手柄刷新消息不会被误判为重复帧。 */
	}

	UIDIRDP(false, 0U, 0U);				   /* 无手柄时三个方向按钮必须全部回到灰色初始状态。 */
	UIHANDLEDP(false, 0U, 1U, 0U);		   /* A 通道写无手柄图标，避免拔出 PXYTM 后仍保留旧选中图标。 */
	UIHANDLEDP(false, 0U, 2U, 0U);		   /* B 通道写无手柄图标，保证最后一个手柄拔出后两侧状态一致。 */
	UITOOLSPECDP(false, 0U, 0U, 0U, 0U);	   /* 隐藏刀具规格，避免 EEPROM 规格在无手柄状态下残留。 */
	UIMANUALBUTTONDP(false, false, 0U, 0U); /* 隐藏自动识别、手动磨/刨和识别结果区域。 */
	UIORALDP(false);						   /* 隐藏开口定位入口，避免无手柄时保留旧刀具能力入口。 */
}

/*
 * 函数功能：刷新频率显示区域。
 * 输入参数：enable_flag 表示频率区是否可见；freq_value 为当前频率；update_value 表示是否只刷新数值。
 * 返回参数：无。
 */
void UIFREQDP(bool enable_flag,uint8_t freq_value,bool update_value)
{
	
	/* 仅支持频率调节的方向才启用此区域，输入层应只在该状态下接受频率加减事件。 */
	if(enable_flag)
	{
       //显示频率值
	   if(!update_value)//只更新数据
	   {
		LCD_Show_Picture(UIDP_LCD_VP_FREQ_AREA,12U);//显示频率
		LCD_Show_Number (UIDP_LCD_SP_FREQ_VALUE, UIDP_LCD_VP_FREQ_VALUE);
	   }
	   LCD_Show_4byte_Number(UIDP_LCD_VP_FREQ_VALUE,freq_value);

	}
	else
	{
	   /* 黑色暗态表示当前方向不支持频率调节，对应触控位置必须静默且不得分发业务事件。 */
	   //消失或者暗黑
	   LCD_Show_Picture(UIDP_LCD_VP_FREQ_AREA,11U);//频率区暗态
	   LCD_Disappear_Number(UIDP_LCD_SP_FREQ_VALUE);///隐藏数字
	}
}

/*
 * 函数功能：刷新主界面速度显示区，并在运行状态下切换速度字体颜色。
 * 输入参数：enable_flag 为速度区显示开关；speed_value 为当前速度值；update_value 表示是否只刷新数值；run_flag 表示电机是否处于运行态。
 * 返回参数：无。
 */
void UISPEEDDP(bool enable_flag,uint32_t speed_value,bool update_value,bool run_flag)
{
	static uint8_t huchi=0U;//记录运行态黄色字体是否已经下发，避免副工程同类 UI 场景反复写屏。
	if(enable_flag)
	{
       //显示速度值
	   if(!update_value)
	   {
			huchi=0U;//重新显示速度框时清掉颜色缓存，保证下一次运行态会重新下发字体颜色。
			LCD_Show_Picture(UIDP_LCD_VP_SPEED_AREA,2U);//速度激活
			LCD_Show_Number (UIDP_LCD_SP_SPEED_VALUE, UIDP_LCD_VP_SPEED_VALUE);
	   }
	   else//运行中更新数据
	   {
		 if(run_flag)
		 {
			//黄色字体
			if(huchi==0U)
			{
				LCD_Show_2byte_Number(UIDP_LCD_SP_SPEED_UNIT_COLOR,0xffE0);
				huchi=1U;//黄色字体已经写入，后续运行中只刷新速度值。
			}
		 }
		 else
		 {
			//黑色字体
			huchi=0U;//退出运行态后释放颜色缓存，下次启动仍会重新写黄色字体。
			LCD_Show_2byte_Number(UIDP_LCD_SP_SPEED_UNIT_COLOR,0xffFF);
		 }
	   }
	   LCD_Show_4byte_Number(UIDP_LCD_VP_SPEED_VALUE,speed_value);
	}
	else
	{
	   //消失或者暗黑
	    huchi=0U;//速度区隐藏后清掉颜色状态，避免再次显示时沿用旧运行态。
		LCD_Show_Picture(UIDP_LCD_VP_SPEED_AREA,1U);//速度暗黑
			LCD_Disappear_Number(UIDP_LCD_SP_SPEED_VALUE);
	}
}
/*
 * 函数功能：刷新报警提示图片，驱动报警可使用独立图片覆盖值。
 * 输入参数：enable_flag 表示是否显示报警；arm_value 为 WorkMessage 统一报警码；picture_value 为驱动报警图片覆盖值。
 * 返回参数：无。
 */
void UIAIARMDP(bool enable_flag,uint8_t arm_value,uint8_t picture_value)
{
	if(enable_flag)
	{
	  if(picture_value == MOTOR_ALARM_PICTURE_NONE)
	  {
		LCD_Disappear_Picture(UIDP_LCD_VP_ALARM_TIP); /* 该驱动错误没有对应图片时清除旧图，但停机、蜂鸣和外控报警继续有效。 */
		return; /* 图片处理已经完成，不能再按逻辑报警码误落到 84 号缺相图片。 */
	  }
	  if(picture_value != 0U)
	  {
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,picture_value); /* 驱动报警使用上游选定的84~99图片，逻辑报警码不参与图号重映射。 */
		return; /* 已显示驱动专用图片，避免后续公用报警分支覆盖。 */
	  }
      //根据报警值显示图片
	  /* 非驱动报警按 EX8 屏幕表格 80~90 绑定，驱动报警已在上方使用独立图片覆盖值处理。 */
	  switch(arm_value)
	  {
		case WORK_ALARM_HANDLE_NOT_CONNECTED:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,80U);//手柄未连接，请链接手柄
		break;
		case WORK_ALARM_MANUAL_SELECTED:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,81U);//手控已选中，请用手控
		break;
		case WORK_ALARM_FOOT_SELECTED:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,82U);//脚控已选中，请用脚控
		break;
		case WORK_ALARM_MOTOR_OVERLOAD:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,87U);//电机过载使用 EX8 87 号报警图
		break;
		case WORK_ALARM_FOOT_VALUE_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,83U);//脚踏存储值错误使用 EX8 83 号报警图
		break;
		case WORK_ALARM_MOTOR_OVERLOAD_ALT:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,87U);//兼容旧过载报警码，同样显示 EX8 87 号过载图
		break;
		case WORK_ALARM_UID_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,85U);//UID错误
		break;
		case WORK_ALARM_HALL_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,86U);//HALL 值错误使用 EX8 86 号报警图
		break;
		case WORK_ALARM_HANDLE_MODEL_ERROR_A:
		case WORK_ALARM_HANDLE_MODEL_ERROR_B:
		case WORK_ALARM_HANDLE_MODEL_ERROR_AB:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,90U);//手柄校验失败统一显示 EX8 90 号“手柄校验异常”报警图
		break;
		case WORK_ALARM_PUMP_PRESSURE_BLOCKED:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,89U);//泵压力达到阈值时显示屏幕新增 89 号压力报警图
		break;
	  }

	}
	else
	{
		//消失
		LCD_Disappear_Picture(UIDP_LCD_VP_ALARM_TIP);//无错隐藏
	}
}
/*
 * 函数功能：刷新 A 泵启停按钮图标，按泵业务类型选择注水按钮或灌注/抽吸按钮资源。
 * 输入参数：enable_flag 表示按钮是否可用；button_type 表示当前泵业务类型；run_flag 表示泵是否处于运行态。
 * 返回参数：无。
 */
void UIPUMPABUTTONDP(bool enable_flag,uint8_t button_type,bool run_flag)
{
	if(enable_flag)
	{
       //显示泵A按钮
	  LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON, UIDP_PumpButtonPicture(button_type, true, run_flag));//A 泵启动按钮使用 UIDP_LCD_VP_PUMP_A_BUTTON，资源使用 200~205
	}
	else
	{
		//消失
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_BUTTON, UIDP_PumpButtonPicture(button_type, false, false));//A 泵禁用时回停止底图
	}
}
//b泵显示按钮
/*
 * 函数功能：刷新 B 泵启停按钮图标，按泵业务类型选择注水按钮或灌注/抽吸按钮资源。
 * 输入参数：enable_flag 表示按钮是否可用；button_type 表示当前泵业务类型；run_flag 表示泵是否处于运行态。
 * 返回参数：无。
 */
void UIPUMPBBUTTONDP(bool enable_flag,uint8_t button_type,bool run_flag)//自动识别按钮，欠缺字体变黄，或者变黑
{
	if(enable_flag)
	{
       //显示泵B按钮
	  LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON, UIDP_PumpButtonPicture(button_type, true, run_flag));//B 泵启动按钮使用 UIDP_LCD_VP_PUMP_B_BUTTON，避免和 UIDP_LCD_VP_PUMP_B_GEAR_AREA 重叠
	}
	else
	{
		//消失
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_BUTTON, UIDP_PumpButtonPicture(button_type, false, false));//B 泵禁用时回停止底图
	}
}

/*
 * 函数功能：按当前 pumpMessage 快照直接预绘 A/B 泵显示，不经过 UI 队列二次排队。
 * 输入参数：pump_area_id 为 A/B 泵区域 UI 编号，button_area_id 为 A/B 泵启停按钮 UI 编号，pump_message 为当前泵状态快照。
 * 返回参数：无。
 */
static void UIDP_DrawPumpDisplaySnapshot(uint8_t pump_area_id, uint8_t button_area_id, const pumpMessage_t *pump_message)
{
	uint8_t display_value[10] = {0U}; /* 复用 UIDP 队列协议的 10 字节参数格式，保证直接预绘和运行期刷新解释一致。 */
	bool pump_available = false; /* 泵类型为 0 时表示尚未识别，开机预绘要显示禁用态而不是误显示抽吸泵。 */
	bool button_active = false; /* 注水泵按钮只显示排空来源，不能根据普通冷却运行状态高亮。 */
	uint16_t display_speed = 0U; /* 显示速度使用 Pubinterface 的统一换算，运行态显示实际输出，停止态显示设定值。 */

	if(pump_message == NULL)
	{
		return; /* 防御空指针，避免开机预绘阶段异常访问导致 UI 任务中断。 */
	}

	pump_available = (pump_message->type != 0U); /* 只有 CS1237 已识别出业务泵类型时，泵区才按可用态显示。 */
	display_speed = Pubinterface_GetPumpDisplaySpeed(pump_message); /* 保持与 Pubinterface_RefreshPumpADisplay/BDisplay 的速度口径一致。 */
	button_active = pump_message->run_flag; /* 抽吸和灌注泵仍按普通运行状态预绘按钮。 */
	if (pump_message->type == INJECTWATER)
	{
		button_active = (pump_message->timingDrainage_flag || pump_message->pedalDrainage_flag); /* 注水泵只有屏幕定时排空或脚踏轻排才预绘黄色按钮。 */
	}

	display_value[0] = (uint8_t)pump_message->type; /* Value[0] 传泵类型，UIPUMPADP/UIPUMPBDP 用它选择注水、灌注或抽吸图标。 */
	display_value[1] = (uint8_t)(display_speed >> 8); /* Value[1] 传显示速度高字节，保证 16 位流量值完整。 */
	display_value[2] = (uint8_t)(display_speed & 0xFFU); /* Value[2] 传显示速度低字节，与队列刷新协议保持一致。 */
	if(pump_area_id == UI_PUMPA_ID)
	{
		UIPUMPADP(pump_available, display_value[0], (uint16_t)((display_value[1] << 8) | display_value[2])); /* A 泵开机预绘直接写 VP，避免 page4 显示后再排队刷新。 */
	}
	else if(pump_area_id == UI_PUMPB_ID)
	{
		UIPUMPBDP(pump_available, display_value[0], (uint16_t)((display_value[1] << 8) | display_value[2])); /* B 泵同样直接预绘，解决 B 区按钮/流量区后加载的可见闪动。 */
	}

	display_value[1] = button_active ? 1U : 0U; /* 预绘参数使用排空业务状态，手柄冷却联动时保持白色。 */
	display_value[2] = 0U; /* 按钮刷新不使用速度低字节，清零避免复用上面的流量参数。 */
	if(button_area_id == UI_PUMPABUTTON_ID)
	{
		UIPUMPABUTTONDP(pump_available, display_value[0], display_value[1]); /* A 泵按钮在切页前同步到禁用、停止或运行态。 */
	}
	else if(button_area_id == UI_PUMPBBUTTON_ID)
	{
		UIPUMPBBUTTONDP(pump_available, display_value[0], display_value[1]); /* B 泵按钮在切页前同步到禁用、停止或运行态。 */
	}
}

/*
 * 函数功能：显示或隐藏触控/外部控制弹窗入口。
 * 输入参数：enable_flag 表示是否显示弹窗；run_flag 表示触控控制是否已经启动运行。
 * 返回参数：无。
 */
void UITOUCHDP(bool enable_flag,bool run_flag)
{
	if(enable_flag)
	{
		LCD_Show_Picture(UIDP_LCD_VP_TOUCH_WORK,70U);//触控激活时显示 0x1430 触控工作界面，保持主图标和工作区同步
		if(run_flag)
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,38U);//触控运行中高亮主运行页触控图标
		}
		else
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,37U);//触控已激活但未运行时显示可用态
		}
	}
	else
	{
		LCD_Disappear_Picture(UIDP_LCD_VP_TOUCH_WORK);//触控退出时隐藏 0x1430 工作区，避免旧触控界面残留
		LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,36U);//触控退出或外控释放时回到暗态，不再写旧弹窗 VP
	}
}
/*
 * 函数功能：周期消费屏幕刷新队列，并维护开机补刷及手柄拔出后的短时连接快照补发。
 * 输入参数：无。
 * 返回参数：无。
 */
void UIDISPLAYBehavior()
{
	if(UIDPMsgQueue != NULL)
	{
		UIDPMessage_t msg;
		uint8_t processed_count = 0U; /* 单次调度内批量消费少量 UI 消息，减少主运行页控件陆续出现。 */
		// 非阻塞方式接收消息
		while(processed_count < UIDP_DISPLAY_BATCH_LIMIT)
		{
			if(Kernel_QueueReceive(UIDPMsgQueue, &msg, 0) != pdTRUE)
			{
				break; /* 当前队列已空时立即让出任务周期，避免空转占用调度时间。 */
			}
			processed_count++; /* 记录本周期实际处理条数，用于限制 UART6 连续发送压力。 */
			switch (msg.areaId)
			{
					case UI_TOUCH_ID:
					UITOUCHDP(msg.enable_flag,msg.Value[0]);
					break;
					case UI_PUMPA_ID:
					UIPUMPADP(msg.enable_flag,msg.Value[0],msg.Value[1]<<8|msg.Value[2]);
					break;
					case UI_PUMPB_ID:
					UIPUMPBDP(msg.enable_flag,msg.Value[0],msg.Value[1]<<8|msg.Value[2]);
					break;
					case UI_CONTROL_ID:
					UICONTROLDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_DIR_ID:
					UIDIRDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_HANDLE_ID:
					UIHANDLEDP(msg.enable_flag,msg.Value[0],msg.Value[1],msg.Value[2]);
					break;
					case UI_TOOL_ID:
					UITOOLDP(msg.enable_flag,msg.Value[0]);
					break;
					case UI_TOOLSPEC_ID:
					UITOOLSPECDP(msg.enable_flag,msg.Value[0]<<8|msg.Value[1],msg.Value[2],msg.Value[3],msg.Value[4]);
					break;
					case UI_ORAL_ID:
					UIORALDP(msg.enable_flag);
					break;
					case UI_FREQ_ID:
					UIFREQDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_SPEED_ID:
					UISPEEDDP(msg.enable_flag,msg.Value[0]<<16|msg.Value[1]<<8|msg.Value[2],msg.Value[3],msg.Value[4]);
					break;
				    case UI_AIARM_ID:
					UIAIARMDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_MANUALBUTTON_ID:
					UIMANUALBUTTONDP(msg.enable_flag,msg.Value[0],msg.Value[1],msg.Value[2]);
					break;
					case UI_PUMPAGEAR_ID:
					break;
					case UI_PUMPBGEAR_ID:
					break;
					case UI_PUMPABUTTON_ID:
					UIPUMPABUTTONDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_PUMPBBUTTON_ID:
					UIPUMPBBUTTONDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_POWERINIT_ID:
					UIDP_DrawPumpDisplaySnapshot(UI_PUMPA_ID, UI_PUMPABUTTON_ID, &pumpMessageA);//A 泵按当前 pumpMessageA 直接预绘，切到 page4 后不再肉眼看到泵区二次加载
					UIDP_DrawPumpDisplaySnapshot(UI_PUMPB_ID, UI_PUMPBBUTTON_ID, &pumpMessageB);//B 泵同样按当前 pumpMessageB 直接预绘，保持左右泵区来源一致
					UICONTROLDP(0,0,0);
					LCD_Disappear_Picture(UIDP_LCD_VP_TOUCH_WORK);//开机预绘时只隐藏 0x1430 触控工作区，不改触控主按钮白色可选态
					UIDIRDP(0,0,0);
					UIHANDLEDP(0,0,1,0);//上电未插手柄时先画出 A 通道未连接状态，避免空白区域
					UIHANDLEDP(0,0,2,0);//上电未插手柄时先画出 B 通道未连接状态，后续插拔事件再覆盖
					UITOOLDP(0,0);
					UITOOLSPECDP(0,30,20,10,0);
					UIORALDP(0);
					UIFREQDP(0,0,0);
					UISPEEDDP(0,0,0,0);
					UIAIARMDP(0,0,0);
					UIMANUALBUTTONDP(0,0,0,0);
					LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);//主运行页 VP 全部预写完成后再切到 page4，减少控件逐个出现的可见过程
					SendKeyBeepMessage(1U); /* 主运行页已经切换完成，蜂鸣一次提示开机进入运行界面。 */
					s_uidp_startup_replay_ticks = UIDP_STARTUP_REPLAY_TOTAL_TICKS; /* 进入主运行页后开启短窗口补刷，等待 A/B 手柄识别和 RFID 首包完成。 */
					s_uidp_startup_replay_period = 0U; /* 首次调度允许立即检查当前状态，避免 B 手柄已经在线但仍显示未连接。 */
					break;
					default:
					break;
			}
		}
	}
	if (s_uidp_startup_replay_ticks > 0U)
	{
		--s_uidp_startup_replay_ticks; /* 每个 10ms 显示任务周期递减，3 秒后自动停止补刷，不影响后续正常 UI。 */
		if (s_uidp_startup_replay_period > 0U)
		{
			--s_uidp_startup_replay_period; /* 补刷间隔未到时只计时，不追加 UI 消息。 */
		}
		if ((s_uidp_startup_replay_period == 0U) &&
			(UIDPMsgQueue != NULL) &&
			(uxQueueMessagesWaiting(UIDPMsgQueue) == 0U))
		{
			s_uidp_startup_replay_period = UIDP_STARTUP_REPLAY_PERIOD_TICKS; /* 本次补刷后等待 200ms，再给 RFID 后续结果一次刷新机会。 */
			Pubinterface_RefreshRuntimeDisplaySnapshot(); /* 按当前全局状态重发 A/B 在线图标和当前通道参数区，不修改控制状态。 */
		}
	}

	if (s_uidp_handle_replay_remaining > 0U)
	{
		if (s_uidp_handle_replay_period > 0U)
		{
			--s_uidp_handle_replay_period; /* 补发间隔未到时只递减计数，不挤占当前屏幕消息队列。 */
		}
		if ((s_uidp_handle_replay_period == 0U) &&
			(UIDPMsgQueue != NULL) &&
			(uxQueueMessagesWaiting(UIDPMsgQueue) == 0U))
		{
			--s_uidp_handle_replay_remaining; /* 队列已空时消费一次补发额度，确保本轮快照不会夹在旧消息中间。 */
			s_uidp_handle_replay_period = UIDP_HANDLE_REPLAY_PERIOD_TICKS; /* 下一份快照再等待约 60ms，降低显示串口瞬时连续发送压力。 */
			Pubinterface_RefreshOnlineHandleDisplay(); /* 只读取当前 A/B 在线态和选中态，不重复触发插拔业务、报警或蜂鸣。 */
		}
	}
}

void UIDISPLAYBehaviorTask(uint32_t event)
{
	(void)event;

	UIDISPLAYBehavior();

}

void SscUIDisplayTask_Init(void)
{
	UIDPQueue_Init();
	Kernel_TaskCreate(&UIDISPLAYBehaviorHandle, UIDISPLAYBehaviorTask);
	Kernel_TaskStart(&UIDISPLAYBehaviorHandle, KERNEL_TASK_ALWAYS, 10);//10ms触发一次，更新及时
}
