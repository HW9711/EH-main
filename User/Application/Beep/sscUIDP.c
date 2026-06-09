#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscUIDP.h"
#include "lcd.h"
#include "Pubinterface.h"
#include "screen_address.h"

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
/* 泵档位正常运行态固定使用每档渐隐组的第 0 帧，取消动画后续如需播放再传入 1~4 帧。 */
#define UIDP_PUMP_GEAR_RUN_FRAME 0U
/* 每个显示任务周期最多连续处理 12 条 UI 消息，压缩上电主运行页控件逐个加载的可见时间。 */
#define UIDP_DISPLAY_BATCH_LIMIT 12U
/* 泵档位表每档包含 5 张渐隐资源，0 档没有渐隐时 5 个槽位都保持同一张图。 */
#define UIDP_PUMP_GEAR_FRAME_COUNT 5U
/* 泵档位最多显示 0~10 共 11 档，和屏幕工程 249/349 起始资源保持一致。 */
#define UIDP_PUMP_GEAR_COUNT 11U

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
    UIDPMsgQueue = Kernel_QueueCreate(20, sizeof(UIDPMessage_t), "UIDPMsgQueue");
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
		if(value==0U)gear_value=0U;
		else if(value<=5U)gear_value=1U;
		else if(value<=10U)gear_value=2U;
		else if(value<=15U)gear_value=3U;
		else if(value<=20U)gear_value=4U;
		else if(value<=25U)gear_value=5U;
		else if(value<=30U)gear_value=6U;
		else if(value<=40U)gear_value=7U;
		else if(value<=50U)gear_value=8U;
		else if(value<=60U)gear_value=9U;
		else gear_value=10U;
	}
	else if(pump_type==DRAWWATER)//抽吸
	{
		if(value==8U)gear_value=7U;
		else if(value==10U)gear_value=8U;
		else if(value==12U)gear_value=9U;
		else if(value>=15U)gear_value=10U;
		else gear_value=(uint8_t)value;
	}
	else if(pump_type==POURWATER)//灌注
	{
		if(value==0U)gear_value=0U;
		else if(value<=30U)gear_value=1U;
		else if(value<=60U)gear_value=2U;
		else if(value<=90U)gear_value=3U;
		else if(value<=120U)gear_value=4U;
		else if(value<=150U)gear_value=5U;
		else if(value<=180U)gear_value=6U;
		else if(value<=210U)gear_value=7U;
		else if(value<=240U)gear_value=8U;
		else if(value<=270U)gear_value=9U;
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
			return enable_flag ? 214U : 215U; /* 抽吸泵标题使用 214 黄色、215 灰色，和新屏图库保持一致。 */
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
	if(enable_flag == false)
	{
		run_flag = false; /* 禁用态永远显示停止底图，避免禁用泵误显示运行状态。 */
	}
	switch(button_type)
	{
		case DRAWWATER:
			return run_flag ? 201U : 200U; /* 抽吸泵启停按钮使用 200/201 一组。 */
		case POURWATER:
			return run_flag ? 203U : 202U; /* 灌注泵启停按钮使用 202/203 一组。 */
		case INJECTWATER:
			return run_flag ? 205U : 204U; /* 注水泵启停按钮使用 204/205 一组。 */
		default:
			return run_flag ? 201U : 200U; /* 未识别类型按抽吸泵按钮兜底。 */
	}
}

/*
 * 函数功能：向屏幕显示任务投递一个区域刷新消息。
 * 输入参数：areaId 为 UI 区域编号；enable_flag 为显示开关；Value 为最多 10 字节的区域参数，允许为空。
 * 返回参数：无。
 */
void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value)
{
	if(UIDPMsgQueue == NULL) return;
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
	if((s_uidp_last_valid != 0U) &&
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
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_PLUS, 223U);//A 泵加按钮最后重画，避免被 UIDP_LCD_VP_PUMP_A_GEAR_AREA 泵区底图覆盖
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_MINUS, 222U);//A 泵减按钮最后重画，避免刷新后方向被底图残留影响
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
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_PLUS, 225U);//A 泵加按钮最后重画，保持新屏加号方向
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_A_MINUS, 224U);//A 泵减按钮最后重画，保持新屏减号方向
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
		if(UIDP_IsPumpTypeKnown(pump_type))
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
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS, 498U);//B 泵加按钮使用屏幕导出绑定的 B 区暗态资源，避免资源绑定超出 UIDP_LCD_VP_PUMP_B_PLUS 控件范围后不显示
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS, 500U);//B 泵减按钮使用屏幕导出绑定的 B 区暗态资源，避免 222/224 只适用于 A 区控件
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
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_PLUS, 499U);//B 泵加按钮使用屏幕导出绑定的 B 区亮态资源，确保触控可用时按钮同步亮起
		LCD_Show_Picture(UIDP_LCD_VP_PUMP_B_MINUS, 501U);//B 泵减按钮使用屏幕导出绑定的 B 区亮态资源，确保触控可用时按钮同步亮起
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
           if(light_flag)
		   LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT,32U);
		   else
		   LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT,31U);
		}
		else if(control_type==2)//手控
		{
			if(light_flag)
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
			if(light_flag)
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
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE, 34U);//手控按钮默认白色可选态，未选中时不能显示黄色高亮
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH, 37U);//触控按钮默认白色可选态，按下进入触控后才显示黄色
			LCD_Disappear_Picture(UIDP_LCD_VP_CONTROL_EXTERNAL);//外部通信未接入时隐藏小电脑图标，避免误显示为在线
		}
		else if(control_type == 1U)
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_FOOT, 30U);
		}
		else if(control_type == 2U)
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_HANDLE, 33U);
		}
		else if(control_type == 3U)
		{
			LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH, 36U);
		}
		else if(control_type == 4U)
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
			if(light_flag == 1U)
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,22U);
			else if(light_flag == 2U)
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,20U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,21U);
			break;
			case 2://逆时针
			if(light_flag == 1U)
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,28U);
			else if(light_flag == 2U)
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,26U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,27U);
			break;
			case 3://往复
			if(light_flag == 1U)
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,25U);
			else if(light_flag == 2U)
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,23U);
			else
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,24U);
			break;
		}
	}
	else
	{
		if(dir_type == 0U)
		{
			LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD, 20U);
			LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC, 23U);
			LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE, 26U);
		}
		else
		{
			switch(dir_type)
			{
				case 1://顺时针
				LCD_Show_Picture(UIDP_LCD_VP_DIR_FORWARD,20U);
				break;
				case 2://逆时针
				LCD_Show_Picture(UIDP_LCD_VP_DIR_REVERSE,26U);
				break;
				case 3://往复
				LCD_Show_Picture(UIDP_LCD_VP_DIR_OSC,23U);
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
					case 1://耳膜
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,104):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,101);//耳膜连接
					break;
					case 2://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,105):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,102);//分体
					break;
					case 3://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,106):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,103);//一体
					break;
					case 4://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,108):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,107);//一体磨削
					break;
					case PXBA_ONLINES://空心钻 A 型手柄
					case PXBB_ONLINES://空心钻 B 型手柄，屏幕图标和 PXBA 使用同一组资源
					case LGZ_I_ONLINES://颅骨钻一型预留，先复用新增手柄图标资源
					case LGZ_II_ONLINES://颅骨钻二型预留，先复用新增手柄图标资源
					case KSZ_I_ONLINES://克氏针一型预留，先复用新增手柄图标资源
					case KSZ_II_ONLINES://克氏针二型预留，先复用新增手柄图标资源
					case KXZ_I_ONLINES://空心钻一型预留，沿用空心钻图标资源
					case KXZ_II_ONLINES://空心钻二型预留，沿用空心钻图标资源
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,110):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,109);//空心钻
					break;
					case COMMON_SOCKET_ONLINES:
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,105):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_A,102);//公共接头没有实体键，A 通道先用分体连接图标占位
					break;
				}

		}
		else if(handle_channel==2)//B通道
		{
				switch(handle_type)
				{
					case 1://耳膜
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,134):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,131);//耳膜连接
					break;
					case 2://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,135):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,132);//分体
					break;
					case 3://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,136):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,133);//一体
					break;
					case 4://
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,138):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,137);//一体磨削
					break;
					case PXBA_ONLINES://空心钻 A 型手柄
					case PXBB_ONLINES://空心钻 B 型手柄，屏幕图标和 PXBA 使用同一组资源
					case LGZ_I_ONLINES://颅骨钻一型预留，先复用新增手柄图标资源
					case LGZ_II_ONLINES://颅骨钻二型预留，先复用新增手柄图标资源
					case KSZ_I_ONLINES://克氏针一型预留，先复用新增手柄图标资源
					case KSZ_II_ONLINES://克氏针二型预留，先复用新增手柄图标资源
					case KXZ_I_ONLINES://空心钻一型预留，沿用空心钻图标资源
					case KXZ_II_ONLINES://空心钻二型预留，沿用空心钻图标资源
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,140):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,139);//空心钻
					break;
					case COMMON_SOCKET_ONLINES:
					light_flag?LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,135):LCD_Show_Picture(UIDP_LCD_VP_HANDLE_B,132);//公共接头没有实体键，B 通道先用分体连接图标占位
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
	if(enable_flag)
	{
        if(tool_type==1)//wanpao
		{
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刨刀选中时磨头按钮显示未选中态
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,56U);//刨刀按钮显示选中态
		}
		else
		{
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,54U);//磨头按钮显示选中态
			LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//磨头选中时刨刀按钮显示未选中态
		}
	}
	else
	{
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刀具区禁用时磨头回到暗态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//刀具区禁用时刨刀回到暗态
	}
}
//刀具规格 长度，直径、角度
void UITOOLSPECDP(bool enable_flag,uint16_t tool_length,uint8_t tool_Diameter,uint8_t tool_angle )
{
	if(enable_flag)
	{
		LCD_Show_2byte_Number(UIDP_LCD_SP_TOOL_SPEC_COLOR,34);
		LCD_IntegratedCutterData_Update(UIDP_LCD_VP_TOOL_SPEC_TEXT, tool_length, tool_Diameter, tool_angle); //刀具参数
	}
	else
	{
		//消失
		LCD_Show_2byte_Number(UIDP_LCD_SP_TOOL_SPEC_COLOR,0);
	}
}

/*
 * 函数功能：刷新手动刀具选择和自动识别按钮区域。
 * 输入参数：enable_flag 表示该区域是否显示；PAO_flag 为 true 表示刨刀选中、false 表示磨头选中。
 * 返回参数：无。
 */
void UIMANUALBUTTONDP(bool enable_flag,bool PAO_flag)
{
	if(enable_flag)
	{
      if(PAO_flag)
	  {
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_RESULT,62U);//刀具识别区域显示刨刀手动选择态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//刨刀选中时磨头按钮回未选中态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,56U);//刨刀按钮显示选中态
		LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,51U);//自动识别按钮保持待触发态
	  }
	  else
	  {
        LCD_Show_Picture(UIDP_LCD_VP_TOOL_RESULT,61U);//刀具识别区域显示磨头手动选择态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,54U);//磨头按钮显示选中态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//磨头选中时刨刀按钮回未选中态
		LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,51U);//自动识别按钮保持待触发态
	  }
	}
	else
	{
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_RESULT,60U);//刀具识别区域禁用态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BURR,53U);//磨头按钮禁用态
		LCD_Show_Picture(UIDP_LCD_VP_TOOL_BLADE,55U);//刨刀按钮禁用态
		LCD_Show_Picture(UIDP_LCD_VP_AUTO_RECOGNIZE,51U);//自动识别按钮未触发态
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
 * 函数功能：刷新频率显示区域。
 * 输入参数：enable_flag 表示频率区是否可见；freq_value 为当前频率；update_value 表示是否只刷新数值。
 * 返回参数：无。
 */
void UIFREQDP(bool enable_flag,uint8_t freq_value,bool update_value)
{
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
void UISPEEDDP(bool enable_flag,uint16_t speed_value,bool update_value,bool run_flag)
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
 * 函数功能：刷新报警提示图片。
 * 输入参数：enable_flag 表示是否显示报警；arm_value 为 WorkMessage 统一报警码。
 * 返回参数：无。
 */
void UIAIARMDP(bool enable_flag,uint8_t arm_value)
{
	if(enable_flag)
	{
      //根据报警值显示图片
	  /* 驱动板细分报警按 UI 绑定使用 3/4/5/8/11，部分编号与旧脚踏提示共用本地图片入口。 */
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
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,83U);//电机过载，请松开脚踏
		break;
		case WORK_ALARM_FOOT_VALUE_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,84U);//脚踏值错误，请联系售后
		break;
		case WORK_ALARM_MOTOR_OVERLOAD_ALT:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,83U);//电机过载复用同一张新屏报警图
		break;
		case WORK_ALARM_UID_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,85U);//UID错误
		break;
		case WORK_ALARM_MOTOR_COMM_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,86U);//电机通讯异常
		break;
		case WORK_ALARM_HALL_ERROR:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,87U);//HALL值错误
		break;
		case WORK_ALARM_HANDLE_MODEL_ERROR_A:
		case WORK_ALARM_HANDLE_MODEL_ERROR_B:
		case WORK_ALARM_HANDLE_MODEL_ERROR_AB:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,88U);//手柄型号错误
		break;
		case WORK_ALARM_MOTOR_DRIVER_BOARD:
		LCD_Show_Picture(UIDP_LCD_VP_ALARM_TIP,86U);//驱动板故障报警码 0x0B 沿用电机通讯异常图
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
 * 函数功能：显示或隐藏触控/外部控制弹窗入口。
 * 输入参数：enable_flag 表示是否显示弹窗；run_flag 表示触控控制是否已经启动运行。
 * 返回参数：无。
 */
void UITOUCHDP(bool enable_flag,bool run_flag)
{
	if(enable_flag)
	{
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
		LCD_Show_Picture(UIDP_LCD_VP_CONTROL_TOUCH,36U);//触控退出或外控释放时回到暗态，不再写旧弹窗 VP
	}
}
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
					UITOOLSPECDP(msg.enable_flag,msg.Value[0]<<8|msg.Value[1],msg.Value[2],msg.Value[3]);
					break;
					case UI_ORAL_ID:
					UIORALDP(msg.enable_flag);
					break;
					case UI_FREQ_ID:
					UIFREQDP(msg.enable_flag,msg.Value[0],msg.Value[1]);
					break;
					case UI_SPEED_ID:
					UISPEEDDP(msg.enable_flag,msg.Value[0]<<8|msg.Value[1],msg.Value[2],msg.Value[3]);
					break;
				    case UI_AIARM_ID:
					UIAIARMDP(msg.enable_flag,msg.Value[0]);
					break;
					case UI_MANUALBUTTON_ID:
					UIMANUALBUTTONDP(msg.enable_flag,msg.Value[0]);
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
					LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);//开机统一强制切到主运行页 page4，沿用老成功版 page0 启动、page4 运行的页分配
					Pubinterface_RefreshPumpADisplay();//A 泵按当前 pumpMessageA 重绘，已识别注水泵不会再被开机默认值覆盖成抽吸泵
					Pubinterface_RefreshPumpBDisplay();//B 泵同样按当前 pumpMessageB 重绘，保持左右泵区来源一致
					UICONTROLDP(0,0,0);
					UIDIRDP(0,0,0);
					UIHANDLEDP(0,0,1,0);//上电未插手柄时先画出 A 通道未连接状态，避免空白区域
					UIHANDLEDP(0,0,2,0);//上电未插手柄时先画出 B 通道未连接状态，后续插拔事件再覆盖
					UITOOLDP(0,0);
					UITOOLSPECDP(0,30,20,10);
					UIORALDP(0);
					UIFREQDP(0,0,0);
					UISPEEDDP(0,0,0,0);
					UIAIARMDP(0,0);
					UIMANUALBUTTONDP(0,0);
					break;
					default:
					break;
			}
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
