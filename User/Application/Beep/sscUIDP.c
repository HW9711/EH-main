#include "stm32f4xx_hal.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "sscUIDP.h"
#include "lcd.h"
#include "Pubinterface.h"

uint8_t DisPlayData[10] = {0};

QueueHandle_t UIDPMsgQueue = NULL;
kernel_task_t UIDISPLAYBehaviorHandle;

typedef struct 
{
	uint8_t areaId;//a泵区域，B泵区域，速度区域，等等
	
	bool enable_flag;//激活
	uint8_t Value[10];
}UIDPMessage_t ;

static void UIDPQueue_Init(void)  
{
    UIDPMsgQueue = Kernel_QueueCreate(20, sizeof(UIDPMessage_t), "UIDPMsgQueue");
}
void SendUIDSMessage(uint8_t areaId,bool enable_flag,uint8_t *Value)
{
	if(UIDPMsgQueue == NULL) return;
    UIDPMessage_t msg;
    msg.areaId = areaId;
	msg.enable_flag = enable_flag;
	memcpy(msg.Value, Value, 10);
    (void)Kernel_QueueSend(UIDPMsgQueue, &msg, 10);
}

void PumpGeardisplay(uint8_t pump_id ,uint8_t pump_type,uint16_t value)
{
	uint8_t Gear_values=0;
	if(pump_type==INJECTWATER)//注水
	{
		if(value==0)Gear_values=0;
		else if(value>0&&value<=5)Gear_values=1;
		else if(value>5&&value<=10)Gear_values=2;
		else if(value>10&&value<=15)Gear_values=3;
		else if(value>15&&value<=20)Gear_values=4;
		else if(value>20&&value<=25)Gear_values=5;
		else if(value>25&&value<=30)Gear_values=6;
		else if(value>30&&value<=40)Gear_values=7;
		else if(value>40&&value<=50)Gear_values=8;
		else if(value>50&&value<=60)Gear_values=9;
		else if(value>60&&value<=70)Gear_values=10;
	}
	else if(pump_type==DRAWWATER)//抽吸
	{
		if(value==8)Gear_values=7;
		else if(value==10)Gear_values=8;
		else if(value==12)Gear_values=9;
		else if(value==15)Gear_values=10;
		else
		Gear_values=value;
	}
	else if(pump_type==POURWATER)
	{
		if(value==0)Gear_values=0;
		else if(value>0&&value<=30)Gear_values=1;
		else	if(value>30&&value<=60)Gear_values=2;
		else	if(value>60&&value<=90)Gear_values=3;
		else	if(value>90&&value<=120)Gear_values=4;
		else	if(value>120&&value<=150)Gear_values=5;
		else	if(value>150&&value<=180)Gear_values=6;
		else	if(value>180&&value<=210)Gear_values=7;
		else	if(value>210&&value<=240)Gear_values=8;
		else	if(value>240&&value<=270)Gear_values=9;
		else	if(value>270&&value<=300)Gear_values=10;
	}
	if(pump_id==1)//A泵
			LCD_Show_Picture(0x1602,Gear_values+487);
			else if(pump_id==2)LCD_Show_Picture(0x1416,Gear_values+387);
}

//A泵区域显示，参数（是否激活，流量值，单位ml或者l）
void UIPUMPADP(bool enable_flag,uint8_t pump_type,uint16_t pump_value )
{

	if(!enable_flag)//A区域暗灭
	{
         //共同点档位暗灭，按钮默认启动暗灭
		LCD_Show_Picture(0x1601,482);//泵抬头
		LCD_Show_Picture(0x1603,498);//+
		LCD_Show_Picture(0x1604,500);//-
		LCD_Show_Picture(0x1602,487);//档位
		LCD_Show_Picture(0x1605,480);//ml
		LCD_Show_Picture(0x1606,484);//按钮
		LCD_Disappear_Number(0x9530);//流量值
	}
	else
	{
		LCD_Show_Picture(0x1603,499);
		LCD_Show_Picture(0x1604,501);
		LCD_Show_Number (0x9530, 0x3530);//显示
		LCD_Show_4byte_Number(0x3530,pump_value);
		switch(pump_type)
		{
			case DRAWWATER://抽吸
				LCD_Show_Picture(0x1601,483);//显示屏图片重新编号，包裹所以泵图片
				LCD_Show_Picture(0x1605,481);
				PumpGeardisplay(1,DRAWWATER,pump_value);
			case POURWATER://灌注
			    LCD_Show_Picture(0x1601,483);
				LCD_Show_Picture(0x1605,481);
				PumpGeardisplay(1,POURWATER,pump_value);
			break;
			case INJECTWATER://注水
				LCD_Show_Picture(0x1601,483);
				LCD_Show_Picture(0x1605,481);
				PumpGeardisplay(1,INJECTWATER,pump_value);
			//排空按钮黑色
			break;
		}
	}
}
//B泵区域显示，参数（是否激活，流量值，单位ml或者l）
void UIPUMPBDP(bool enable_flag,uint8_t pump_type,uint16_t pump_value )
{
if(!enable_flag)//A区域暗灭
	{
         //共同点档位暗灭，按钮默认启动暗灭
		LCD_Show_Picture(0x1415,382);
		LCD_Show_Picture(0x1417,498);
		LCD_Show_Picture(0x1418,500);
		LCD_Show_Picture(0x1419,480);
		LCD_Show_Picture(0x1420,384);
		LCD_Show_Picture(0x1416,387);
		LCD_Disappear_Number(0x9550);
	}
	else
	{
		
		LCD_Show_Picture(0x1417,499);
		LCD_Show_Picture(0x1418,501);
		
		LCD_Show_Number (0x9550, 0x3550);
		LCD_Show_4byte_Number(0x3550,pump_value);
		switch(pump_type)
		{
			case DRAWWATER://抽吸
				LCD_Show_Picture(0x1415,383);//显示屏图片重新编号，包裹所以泵图片
				LCD_Show_Picture(0x1419,481);
				PumpGeardisplay(2,DRAWWATER,pump_value);
			case POURWATER://灌注
			    LCD_Show_Picture(0x1601,483);
				LCD_Show_Picture(0x1605,481);
				
				PumpGeardisplay(2,POURWATER,pump_value);
			break;
			case INJECTWATER://注水
				LCD_Show_Picture(0x1601,483);
				LCD_Show_Picture(0x1605,481);
				PumpGeardisplay(2,INJECTWATER,pump_value);
			//排空按钮黑色
			break;
		}
	}
}
//控制模式显示，参数（是否激活，控制模式-脚控，手控，触控，外部控制）
void UICONTROLDP(bool enable_flag,uint8_t control_type, bool light_flag)
{
	if(enable_flag)
	{
		//传入控制模式，是否需要重写代码
		if(control_type==1)//脚踏
		{
           if(light_flag)
		   LCD_Show_Picture(0x1412,372);
		   else
		   LCD_Show_Picture(0x1412,371);
		}
		else if(control_type==2)//手控
		{
			if(light_flag)
			LCD_Show_Picture(0x1413,375);
			else
			LCD_Show_Picture(0x1413,374);
		}
		else if(control_type==3)//触控，记得宏定义
		{
			LCD_Show_Picture(0x1414,377);
		}
		else if(control_type==4)///外部控制
		{
			if(light_flag)
			LCD_Show_Picture(0x1450,551);
			else
			LCD_Show_Picture(0x1450,550);

		}
	}
	else
	{
		//暗灭
		if(control_type==1)
		LCD_Show_Picture(0x1412,370);
		else if(control_type==2)
		LCD_Show_Picture(0x1413,373);
		else if(control_type==3)
		LCD_Show_Picture(0x1414,376);
		else if(control_type==4)
		LCD_Disappear_Picture(0x1450);//这里HMI是否隐藏待考虑
	}
}
//方向显示，参数（是否激活，方向-顺nawo han时针，逆时针，往复）
void UIDIRDP(bool enable_flag,uint8_t dir_type, bool light_flag)
{
	if(enable_flag)
	{
		//传入方向，是否需要重写代码
		switch(dir_type)
		{
			case 1://顺时针
			if(light_flag)
			LCD_Show_Picture(0x1408,362);
			else
			LCD_Show_Picture(0x1408,361);
			break;
			case 2://逆时针
			if(light_flag)
				LCD_Show_Picture(0x1410,365);
			else
			LCD_Show_Picture(0x1410,364);
			break;
			case 3://往复
			if(light_flag)
			LCD_Show_Picture(0x1409,368);
			else
			LCD_Show_Picture(0x1409,367);
			break;
		}
	}
	else
	{
		switch(dir_type)
		{
			case 1://顺时针
			LCD_Show_Picture(0x1408,360);
			break;
			case 2://逆时针
			LCD_Show_Picture(0x1410,363);
			break;
			case 3://往复
			LCD_Show_Picture(0x1409,366);
			break;
		}
		//暗灭
	}
}
//手柄显示区域（启）
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
					light_flag?LCD_Show_Picture(0x1401,104):LCD_Show_Picture(0x1401,101);//耳膜连接
					break;
					case 2://
					light_flag?LCD_Show_Picture(0x1401,105):LCD_Show_Picture(0x1401,102);//分体
					break;
					case 3://
					light_flag?LCD_Show_Picture(0x1401,106):LCD_Show_Picture(0x1401,103);//一体
					break;
					case 4://
					light_flag?LCD_Show_Picture(0x1401,108):LCD_Show_Picture(0x1401,107);//一体磨削
					break;
					case PXBA_ONLINES://空心钻 A 型手柄
					case PXBB_ONLINES://空心钻 B 型手柄，屏幕图标和 PXBA 使用同一组资源
					light_flag?LCD_Show_Picture(0x1401,110):LCD_Show_Picture(0x1401,109);//空心钻
					break;
				}
			
		}
		else if(handle_channel==2)//B通道
		{
				switch(handle_type)
				{
					case 1://耳膜
					light_flag?LCD_Show_Picture(0x1402,204):LCD_Show_Picture(0x1402,201);//耳膜连接
					break;
					case 2://
					light_flag?LCD_Show_Picture(0x1402,205):LCD_Show_Picture(0x1402,202);//分体
					break;
					case 3://
					light_flag?LCD_Show_Picture(0x1402,206):LCD_Show_Picture(0x1402,203);//一体
					break;
					case 4://
					light_flag?LCD_Show_Picture(0x1402,208):LCD_Show_Picture(0x1402,207);//一体磨削
					break;
					case PXBA_ONLINES://空心钻 A 型手柄
					case PXBB_ONLINES://空心钻 B 型手柄，屏幕图标和 PXBA 使用同一组资源
					light_flag?LCD_Show_Picture(0x1402,210):LCD_Show_Picture(0x1402,209);//空心钻
					break;
				}
		}
	}
	else
	{
		//暗灭
		if(handle_channel==1)//A通道
		LCD_Show_Picture(0x1401,100);//无
		else if(handle_channel==2)//B通道
		LCD_Show_Picture(0x1402,200);//无手柄
	}
}
//刀具显示（是否显示，刀具类型直刨刀，弯刨刀）
void UITOOLDP(bool enable_flag,bool tool_type)
{
	if(enable_flag)
	{
        if(tool_type==1)//wanpao
		LCD_Show_Picture(0x1406,511);
		else
		LCD_Show_Picture(0x1406,512);//直刨
	}
	else
	{
		//消失
		LCD_Disappear_Picture(0x1406);
	}
}
//刀具规格 长度，直径、角度
void UITOOLSPECDP(bool enable_flag,uint16_t tool_length,uint8_t tool_Diameter,uint8_t tool_angle )
{
	if(enable_flag)
	{
		LCD_Show_2byte_Number(0x8008,34);
		LCD_IntegratedCutterData_Update(0x4200, tool_length, tool_Diameter, tool_angle); //刀具参数
	}
	else
	{
		//消失
		LCD_Show_2byte_Number(0x8008,0);
	}
}

//磨头还是刨刀（手动按钮）
void UIMANUALBUTTONDP(bool enable_flag,bool PAO_flag)
{
	if(enable_flag)
	{
      if(PAO_flag)
	  {
		LCD_Show_Picture(0x1404,420);//刨选中
		LCD_Show_Picture(0x1403,423);
	  }
	  else
	  {
        LCD_Show_Picture(0x1404,421);
		LCD_Show_Picture(0x1403,422);//磨选中
	  }
	}
	else
	{
		//消失
		LCD_Disappear_Picture(0x1403);//刨刀消失
		LCD_Disappear_Picture(0x1404);//磨刀消失
	}
}
//开口定位
void UIORALDP(bool enable_flag)
{
	enable_flag?LCD_Show_Picture(0x1405,400):LCD_Disappear_Picture(0x1405);
}

void UIFREQDP(bool enable_flag,uint8_t freq_value,bool update_value)
{
	if(enable_flag)
	{
       //显示频率值
	   if(!update_value)//只更新数据
	   {
		LCD_Show_Picture(0x1411,351);//显示频率
		LCD_Show_Number (0x9470, 0x3470);
	   }
	   LCD_Show_4byte_Number(0x3470,freq_value);
	}
	else
	{
	   //消失或者暗黑
	   LCD_Disappear_Picture(0x1411);//隐藏频率框
	   LCD_Disappear_Number(0x9470);///隐藏数字
	}
}

void UISPEEDDP(bool enable_flag,uint16_t speed_value,bool update_value,bool run_flag)
{
	if(enable_flag)
	{
       //显示速度值
	   if(!update_value)
	   {
			LCD_Show_Picture(0x1407,341);//速度激活
			LCD_Show_Number (0x9420, 0x3420);
	   }
	   else//运行中更新数据
	   {
		 if(run_flag)
		 {
			//黄色字体
			LCD_Show_2byte_Number(0x9423,0xffE0);
		 }
		 else
		 {
			//黑色字体
			LCD_Show_2byte_Number(0x9423,0xffFF);
		 }
	   }
	   LCD_Show_4byte_Number(0x3420,speed_value);
	}
	else
	{
	   //消失或者暗黑
	   	LCD_Show_Picture(0x1407,340);//速度暗黑
			LCD_Disappear_Number(0x9420);
	}
}
void UIAIARMDP(bool enable_flag,uint8_t arm_value)
{
	if(enable_flag)
	{
      //根据报警值显示图片
	  switch(arm_value)
	  {
		case WORK_ALARM_HANDLE_NOT_CONNECTED:
		LCD_Show_Picture(0x1421,520);//手柄未连接，请链接手柄
		break;
		case WORK_ALARM_MANUAL_SELECTED:
		LCD_Show_Picture(0x1421,521);//手控已选中，请用手控
		break;
		case WORK_ALARM_FOOT_SELECTED:
		LCD_Show_Picture(0x1421,525);//脚控已选中，请用脚控
		break;
		case WORK_ALARM_MOTOR_OVERLOAD:
		LCD_Show_Picture(0x1421,527);//电机过载，请松开脚踏
		break;
		case WORK_ALARM_FOOT_VALUE_ERROR:
		LCD_Show_Picture(0x1421,530);//脚踏值错误，请联系售后
		break;
		case WORK_ALARM_MOTOR_OVERLOAD_ALT:
		LCD_Show_Picture(0x1421,531);//电机过载，请松开脚踏
		break;
		case WORK_ALARM_UID_ERROR:
		LCD_Show_Picture(0x1421,533);//英文UID错误
		break;
		case WORK_ALARM_MOTOR_COMM_ERROR:
		LCD_Show_Picture(0x1421,534);//英文电机通讯异常
		break;
		case WORK_ALARM_HALL_ERROR:
		LCD_Show_Picture(0x1421,535);//英文HALL值错误
		break;
		case WORK_ALARM_HANDLE_MODEL_ERROR:
		LCD_Show_Picture(0x1421,536);//英文手柄型号错误
		break;
	  }
	  
	}
	else
	{
		//消失
		LCD_Disappear_Picture(0x1421);//无错隐藏
	}
}
void UIPUMPABUTTONDP(bool enable_flag,uint8_t button_type,bool run_flag)
{
	if(enable_flag)
	{
       //显示泵A按钮
	   if(button_type==1)//注水
	  run_flag? LCD_Show_Picture(0x1606,486):LCD_Show_Picture(0x1606,485);//按钮
	  else if(button_type==2||button_type==3)//灌注或者抽吸
	  run_flag? LCD_Show_Picture(0x1606,386):LCD_Show_Picture(0x1606,385);//按钮,重新排序
	}
	else
	{
		//消失
		LCD_Show_Picture(0x1606,484);//按钮
	}
}
//b泵显示按钮
void UIPUMPBBUTTONDP(bool enable_flag,uint8_t button_type,bool run_flag)//自动识别按钮，欠缺字体变黄，或者变黑
{
	if(enable_flag)
	{
       //显示泵A按钮
	   if(button_type==1)//注水
	  run_flag? LCD_Show_Picture(0x1420,486):LCD_Show_Picture(0x1420,485);//按钮
	  else if(button_type==2||button_type==3)//灌注或者抽吸
	  run_flag? LCD_Show_Picture(0x1420,386):LCD_Show_Picture(0x1420,385);//按钮,重新排序
	}
	else
	{
		//消失
		LCD_Show_Picture(0x1420,484);//按钮
	}
}
void UIDISPLAYBehavior()
{
	if(UIDPMsgQueue != NULL)
	{
		UIDPMessage_t msg;
		// 非阻塞方式接收消息
		if(Kernel_QueueReceive(UIDPMsgQueue, &msg, 0) == pdTRUE)
		{
			switch (msg.areaId)
			{
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
