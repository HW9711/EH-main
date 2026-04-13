
//screen.c
#include "stm32f4xx_hal.h"
#include "screen.h"
#include "uart6.h"
#include "common.h"
#include "data.h"
#include "lcd.h"
#include "pump.h"
#include "bsp_board.h" 
#include "datahand.h"
#include "kernel_scheduler.h"
#include "motor.h"

kernel_task_t KeyBehaviorHandle;
kernel_task_t PUMPBBehaviorHandle;
kernel_task_t  FootKeyHandle;
kernel_task_t  BeepHandle;

static void PUMPBehavior(void);
static void PumpScreenDebugPoint(uint16_t point, uint16_t value, uint8_t detail)
{
	(void)value;
	(void)detail;
	LCD_Show_2byte_Number(0x8008,34);
	LCD_Show_2byte_Number(0x4200, point);
}

uint32_t paoxueSpeciValue_A[4]={0};//直径，长度，角度,模式
uint32_t paoxueSpeciValue_B[4]={0};//直径，长度，角度，模式
uint8_t  paoxueSpeciValue_F[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//直径，长度，角度,[8]表示最大转速，[9]默认转速


volatile uint8_t KeyBeep_flag;
Memorytools Memorytools_S;
Defaultvalue Defaultvalue_s;
foot_memory_value Foot_memory_value_s;
Workvalue Workvalue_s;
ChannelValue ChannelValue_s;
FTValue FTValue_s;
UIdata UIdata_s;
UIStateControl UIStateControl_s;
UIShow_S UIShow_s;
void MotoPaoDaoSelectControl(uint8_t key_value);
 //初始值
void PoweronInit()
{
	ssc_Connectfootpedal(0);
	ssc_Connecthandel(0);
	ssc_Connecttouch(0);
	Ahandeldisplay(1);
	Bhandeldisplay(1);
	speeddisplay(0,0);
	freqdisplay(1,0);
	injectiondisplay(0,0);
	
	draindisplay(0);
	Irrndisplay(2);
	clockwisedisplay(0);
	anticlockwisedisplay(0);
	OSCdiplay(0);
	Tooldisplay(0);
	//OPENpostionDisplay(0);
	//PAOORMODisplay(0);
	
	ALARMdisplay();
	Workvalue_s.set_Freq=40;
	Workvalue_s.set_speed=0;
	Workvalue_s.Alarm_value=0;
	Workvalue_s.Achanell_online_flag=0;
	Workvalue_s.Bchanell_online_flag=0;
	Workvalue_s.FastGear_flag=0;
	Workvalue_s.footcontrol_online_flag=0;
	Workvalue_s.Footmemory_reads_signal=0;
	Workvalue_s.FootThrottletask_flag=0;
	Workvalue_s.Foot_Key_value=0;
	Workvalue_s.Foot_start_flag=0;
	Workvalue_s.Foot_type=0;
	Workvalue_s.HandControlStart_flag=0;
	Workvalue_s.hand_model=0;
	Workvalue_s.Injection_drain_flag=0;
	Workvalue_s.Irrigate_start_flag=0;
	Workvalue_s.set_Injection=30;
	#if water_uptake
	Workvalue_s.set_Irrigate=10;
	#else
	Workvalue_s.set_Irrigate=150;
	#endif
	Infusiondisplay(1,Workvalue_s.set_Irrigate);
	Workvalue_s.fenti_jiyi_flag=1;
	
 	 specidisplay(0,0,0,0);
    //AutomaticDisplay(1);
}
 
 
//显示脚控
void ssc_Connectfootpedal(uint8_t ui_data)
{
	if(ui_data==0)//
	{
		LCD_Show_Picture(0x1412,370);
	}
	else if(ui_data==1)
	{
		LCD_Show_Picture(0x1412,371);
	}
	else if(ui_data==2)
	{
		LCD_Show_Picture(0x1412,372);
	}
}
//显示手控
void ssc_Connecthandel(uint8_t ui_data)
{
	if(ui_data==0)
	{
		LCD_Show_Picture(0x1413,373);
	}
	else if(ui_data==1)
	{
		LCD_Show_Picture(0x1413,374);
	}
	else if(ui_data==2)
	{
		LCD_Show_Picture(0x1413,375);
	}
}
//显示触控
void ssc_Connecttouch(uint8_t ui_data)
{
	if(!ui_data)
	{
		LCD_Show_Picture(0x1414,376);
	}
	else
	{
		LCD_Show_Picture(0x1414,377);
	}
}

void Ahandeldisplay(uint8_t ui_data)
{
	switch(ui_data)
	{
		case 1:
			LCD_Show_Picture(0x1401,100);//无
		break;
		case 2:
			LCD_Show_Picture(0x1401,101);//耳膜连接
		break;
		case 3:
			LCD_Show_Picture(0x1401,102);//分体连接
		break;
		case 4:
			LCD_Show_Picture(0x1401,103);//一体连接
		break;
		case 5:
			LCD_Show_Picture(0x1401,104);//耳膜选中
		break;
		case 6:
			LCD_Show_Picture(0x1401,105);//分体选中
		break;
		case 7:
			LCD_Show_Picture(0x1401,106);//一体选中
		break;
		case 8:
			LCD_Show_Picture(0x1401,107);//一体磨削连接
		break;
		case 9:
			LCD_Show_Picture(0x1401,108);//一体磨削选中
		break;
	}
}
void Bhandeldisplay(uint8_t ui_data)
{
	switch(ui_data)
	{
		case 1:
			LCD_Show_Picture(0x1402,200);//无
		break;
		case 2:
			LCD_Show_Picture(0x1402,201);//耳膜连接
		break;
		case 3:
			LCD_Show_Picture(0x1402,202);//分体连接
		break;
		case 4:
			LCD_Show_Picture(0x1402,203);//一体连接
		break;
		case 5:
			LCD_Show_Picture(0x1402,204);//耳膜选中
		break;
		case 6:
			LCD_Show_Picture(0x1402,205);//分体选中
		break;
		case 7:
			LCD_Show_Picture(0x1402,206);//一体选中
		break;
		case 8:
			LCD_Show_Picture(0x1402,207);//一体磨削连接
		break;
		case 9:
			LCD_Show_Picture(0x1402,208);//一体磨削选中
		break;
	}
}

void speeddisplay(uint8_t ui_data,uint32_t speed_data)
{
	if(ui_data)
	{
		if(Workvalue_s.beep_Alarm_flag)return;
			LCD_Show_Picture(0x1407,341);//速度激活
			LCD_Show_Number (0x9420, 0x3420);
			LCD_Show_4byte_Number(0x3420,speed_data);
	}
	else
	{
			LCD_Show_Picture(0x1407,340);//速度暗黑
			LCD_Disappear_Number(0x9420);
	}
}

void  freqdisplay(uint8_t ui_hide_flag,uint8_t fre_data)
{
	if(ui_hide_flag)
	{
		LCD_Disappear_Picture(0x1411);//隐藏频率
		LCD_Disappear_Number(0x9470);
	}
	else
	{
		LCD_Show_Picture(0x1411,351);//显示频率
		LCD_Show_Number (0x9470, 0x3470);
		LCD_Show_4byte_Number(0x3470,fre_data);
	}
}

void injectiondisplay(uint8_t activation_flag,uint16_t inject_data)
{
	
	uint8_t Gear_values=0;
	if(activation_flag)
	{
		LCD_Show_Picture(0x1601,483);
	//	LCD_Show_Picture(0x1606,485);
		LCD_Show_Picture(0x1603,499);
		LCD_Show_Picture(0x1604,501);
		LCD_Show_Picture(0x1605,481);
		LCD_Show_Number (0x9530, 0x3530);
		LCD_Show_4byte_Number(0x3530,inject_data);
		
		if(inject_data==0)Gear_values=0;
		else if(inject_data>0&&inject_data<=5)Gear_values=1;
		else if(inject_data>5&&inject_data<=10)Gear_values=2;
		else if(inject_data>10&&inject_data<=15)Gear_values=3;
		else if(inject_data>15&&inject_data<=20)Gear_values=4;
		else if(inject_data>20&&inject_data<=25)Gear_values=5;
		else if(inject_data>25&&inject_data<=30)Gear_values=6;
		else if(inject_data>30&&inject_data<=40)Gear_values=7;
		else if(inject_data>40&&inject_data<=50)Gear_values=8;
		else if(inject_data>50&&inject_data<=60)Gear_values=9;
		else if(inject_data>60&&inject_data<=70)Gear_values=10;
		
		LCD_Show_Picture(0x1602,Gear_values+487);
	}
	else
	{
		LCD_Show_Picture(0x1601,482);
		
//		LCD_Show_Picture(0x1606,484);
		
		LCD_Show_Picture(0x1603,498);
		LCD_Show_Picture(0x1604,500);
		LCD_Show_Picture(0x1602,487);
		LCD_Show_Picture(0x1605,480);
		LCD_Disappear_Number(0x9530);
	}
}

//灌注
void Infusiondisplay(uint8_t activation_flag,uint16_t inject_data)
{

	uint8_t Gear_values=0;
	if(activation_flag)
	{
		LCD_Show_Picture(0x1415,383);
		LCD_Show_Picture(0x1417,499);
		LCD_Show_Picture(0x1418,501);
		#if water_uptake
		LCD_Show_Picture(0x1419,381);
		#else 
		LCD_Show_Picture(0x1419,481);
		#endif
	  //	LCD_Show_Picture(0x1420,385);//插入手柄就会有
	
		LCD_Show_Number (0x9550, 0x3550);
		LCD_Show_4byte_Number(0x3550,inject_data);
		
		#if water_uptake
		if(inject_data==8)Gear_values=7;
		else if(inject_data==10)Gear_values=8;
		else if(inject_data==12)Gear_values=9;
		else if(inject_data==15)Gear_values=10;
		else
		Gear_values=inject_data;
		#else
		
		if(inject_data==0)Gear_values=0;
		else if(inject_data>0&&inject_data<=30)Gear_values=1;
		else	if(inject_data>30&&inject_data<=60)Gear_values=2;
		else	if(inject_data>60&&inject_data<=90)Gear_values=3;
		else	if(inject_data>90&&inject_data<=120)Gear_values=4;
		else	if(inject_data>120&&inject_data<=150)Gear_values=5;
		else	if(inject_data>150&&inject_data<=180)Gear_values=6;
		else	if(inject_data>180&&inject_data<=210)Gear_values=7;
		else	if(inject_data>210&&inject_data<=240)Gear_values=8;
		else	if(inject_data>240&&inject_data<=270)Gear_values=9;
		else	if(inject_data>270&&inject_data<=300)Gear_values=10;
		#endif
		LCD_Show_Picture(0x1416,Gear_values+387);
		
	}
	else
	{
		LCD_Show_Picture(0x1415,382);
		LCD_Show_Picture(0x1417,498);
		LCD_Show_Picture(0x1418,500);
		#if water_uptake
		LCD_Show_Picture(0x1419,380);
		#else
			LCD_Show_Picture(0x1419,480);
		#endif
	
		LCD_Show_Picture(0x1420,384);
	
		LCD_Show_Picture(0x1416,387);;
		LCD_Disappear_Number(0x9550);
	}
}
void draindisplay(uint8_t  drain_flag)
{
	if(drain_flag==0)
	{
		LCD_Show_Picture(0x1606,484);
	}
	else  if(drain_flag==1)
	{
		LCD_Show_Picture(0x1606,485);
	}
	else
	{
		LCD_Show_Picture(0x1606,486);
	}
}


void Irrndisplay(uint8_t  Irrnd_flag)
{
	if(Irrnd_flag==0)
	{
		LCD_Show_Picture(0x1420,384);
	}
	else if(Irrnd_flag==1)
	{
		LCD_Show_Picture(0x1420,385);
	}
	else
	{
		LCD_Show_Picture(0x1420,386);
	}
}

//正
void clockwisedisplay(uint8_t clockwise_flag)
{
	if(clockwise_flag==0)
	{
		LCD_Show_Picture(0x1408,360);
	}
	else if(clockwise_flag==1)
	{
		LCD_Show_Picture(0x1408,361);
	}
	else
	{
		LCD_Show_Picture(0x1408,362);
	}
}
//反
void anticlockwisedisplay(uint8_t anticlockwise_flag)
{
	
	if(anticlockwise_flag==0)
	{
		LCD_Show_Picture(0x1410,363);
	}
	else if(anticlockwise_flag==1)
	{
		LCD_Show_Picture(0x1410,364);
	}
	else
	{
		LCD_Show_Picture(0x1410,365);
	}
}
//往复
void OSCdiplay(uint8_t OSC_flag)
{
	if(OSC_flag==0)
	{
		LCD_Show_Picture(0x1409,366);
	}
	else if(OSC_flag==1)
	{
		LCD_Show_Picture(0x1409,367);
	}
	else
	{
		LCD_Show_Picture(0x1409,368);
	}
}

void Tooldisplay(uint8_t tool_data)
{
	if(tool_data==0)//隐藏
	{
		LCD_Disappear_Picture(0x1406);
	}
	else if(tool_data==1)//刀具未识别
	{
		LCD_Show_Picture(0x1406,510);
	}
	else if(tool_data==2)//直刨刀
	{
		LCD_Show_Picture(0x1406,512);
	}
	else                 //弯刨刀
	{
		LCD_Show_Picture(0x1406,511);
	}
}


void OPENpostionDisplay(uint8_t openpostion_flag)
{
	if(openpostion_flag)
	{
		LCD_Show_Picture(0x1405,400);
	}
	else
	{
		LCD_Disappear_Picture(0x1405);
	}
}

void PAOORMODisplay(uint8_t PAOORMO_VALUE)
{
	if(PAOORMO_VALUE==0)
	{
		LCD_Disappear_Picture(0x1403);
		LCD_Disappear_Picture(0x1404);
		OPENpostionDisplay(0);
	}
	else if(PAOORMO_VALUE==1)//磨选中
	{
		OPENpostionDisplay(0);
		LCD_Show_Picture(0x1404,421);
		LCD_Show_Picture(0x1403,422);
	}
	else
	{
		OPENpostionDisplay(1);
		LCD_Show_Picture(0x1404,420);//刨选中
		LCD_Show_Picture(0x1403,423);
	}
}

//自动识别显示
void AutomaticDisplay(uint8_t activation_flag)
{
	KeyBeep_flag=1;
	if(activation_flag)
		{
			PAOORMODisplay(0);
			LCD_Show_Picture(0x1440,424);//
			Tooldisplay(0);//影藏刀具图片
			if(Workvalue_s.tool_model)
			{
				OPENpostionDisplay(1);
			}
			//specidisplay(1,paoxueSpeciValue_F[1]*5,paoxueSpeciValue_F[2],paoxueSpeciValue_F[3]);//显示刀具信息
		}
		else
		{
			PAOORMODisplay(2);
			LCD_Show_Picture(0x1440,425);
			Tooldisplay(3);//默认弯刨
			specidisplay(0,0,0,0);
			Workvalue_s.tool_model=0;
			MotoPaoDaoSelectControl(21);
		}
}
void AutomaticAxtion(uint8_t paodao_flag)
{
	if(paodao_flag)
	{
		Workvalue_s.set_Direction=Direction_osc;//往复转动
		OPENpostionDisplay(1);//打开开口定位
		clockwisedisplay(1);
		anticlockwisedisplay(1);
		OSCdiplay(2);
		Workvalue_s.set_speed=5000;//转速
		speeddisplay(1,Workvalue_s.set_speed);
		Workvalue_s.set_Freq=40;
		freqdisplay(0,Workvalue_s.set_Freq);
	}
	else
	{
		Workvalue_s.set_Direction=Direction_forward;
		OPENpostionDisplay(0);//打开开口定位
		clockwisedisplay(2);
		anticlockwisedisplay(1);
		OSCdiplay(0);
		if(paoxueSpeciValue_F[8]==0x8c)//7w刀具，平速
		{
		 	Workvalue_s.set_speed=50000;
		}
		else if(paoxueSpeciValue_F[8]==0x1c)//14w刀具，2倍增速
		{
			Workvalue_s.set_speed=100000;//转速
		}
		else
		{
			Workvalue_s.set_speed=10000;//转速//普通刀具，2倍减速
		}
		speeddisplay(1,Workvalue_s.set_speed);
		freqdisplay(1,Workvalue_s.set_Freq);
	}
}


//刀具规格
void specidisplay(uint8_t speci_flag,uint16_t Length, uint8_t Diameter, uint8_t Angle)
{
	if(speci_flag){
		LCD_Show_2byte_Number(0x8008,34);
		LCD_IntegratedCutterData_Update(0x4200, Length, Diameter, Angle); //刀具参数
	}
	else
	{
		//LCD_IntegratedCutterData_Update(0x4200, 0, 0, 0);
		LCD_Show_2byte_Number(0x8008,0);
	}
}

// 1 手柄未连接，请连接手柄
// 2 脚踏未连接，请连接脚踏
// 3 刀具未连接，请连接刀具
// 4 霍尔型号错误，请联系售后
// 5 点击通讯故障，请联系售后
// 6 电机过载，请松开脚踏后运行，或者检查刀具是否卡死
// 7 手柄未连接，请连接手柄后再启动脚踏
// 8 电机相位错误，请联系售后！
// 9 脚踏存储值错误，请联系售后！
// 10 手柄型号错误，请联系售后！
// 11 UID错误，请联系售后！
// 12 脚控已选中，请使用脚控启动手柄！
// 13 手控已选中，请使用手控启动手柄！
// 14 触控已选中，请使用手控启动手柄！
// 15右脚踏异常，请使用左脚踏并联系生产商！
// 16左脚踏异常，请使用右脚踏并联系生产商！

void ALARMdisplay()
{
	static uint8_t negation_data=0;

	if(Workvalue_s.beep_Alarm_flag==0)
	{
		if(negation_data){
			negation_data=0;
			LCD_Disappear_Picture(0x1421);//无错隐藏
			speeddisplay(1,Workvalue_s.set_speed);//速度数据显示 
			if(Workvalue_s.set_Direction==Direction_osc)
			{
				freqdisplay(0,Workvalue_s.set_Freq);//隐藏频率
			}
		}
	}
	else
	{
		if(negation_data==0){
		negation_data=1;
		LCD_Show_Picture(0x1421,519+Workvalue_s.Alarm_value);
		if(Workvalue_s.set_Direction==Direction_osc)
			{
			freqdisplay(1,0);//隐藏频率
			}
			speeddisplay(0,0);
		}
		//发送停止命令
	}
}
	



//脚踏值,这里分两种情况（一个是单脚3按键，一个是双脚踏3按键）
void FootControl_s()
{
	if(Workvalue_s.Foot_type==2)//当前模式是双踏板三按键，
		{
			if(Workvalue_s.set_Way==footcontrol){
				if(Workvalue_s.select_channel)
				//大于最低值，小于中间值
				if(Workvalue_s.Foot_L_value>Foot_memory_value_s.L_Lvalue)
				{
					Workvalue_s.Foot_start_flag=start_flag;
					if(Workvalue_s.Foot_L_value<Foot_memory_value_s.L_Hvalue&&Workvalue_s.Foot_L_value>Foot_memory_value_s.L_Mvalue)
						{
							Workvalue_s.Foot_start_flag=start_flag;
						}
						else
						{
								Workvalue_s.Foot_start_flag=stop_flag;
						}
				}
			}
			else
			{
				if(Workvalue_s.Foot_L_value>Foot_memory_value_s.L_Lvalue)
				{
					//报警（请选择）
				}
			}
		}
		else if(Workvalue_s.Foot_model==1)
		{
			
		}
}


//电机运行控制
void MotorControl()
{
	if(Workvalue_s.beep_Alarm_flag)
		{
			//报警存在的时候
		}
		else
		{
			if(Workvalue_s.Foot_start_flag)
				{
					
				}
				else if(Workvalue_s.HandControlStart_flag)
				{
					
				}
		}
}
//预计100ms触发一次，运行过程中有可能插拔手柄导致==falsh
void InjectionControl()
{
  if(Workvalue_s.Foot_start_flag)//该值在脚踏中赋值
		{
			if(Workvalue_s.Injection_drain_flag==start_flag){
				//白色图标
				Workvalue_s.Injection_drain_flag=stop_flag;
			}
			//Workvalue_s.set_Injection 发送该值;
		}
		else
		{
			if(Workvalue_s.Injection_drain_flag==stop_flag)//拔掉中把它置为fait
			{
				//停止泵转动命令
			}
		}
}
	



//刀具规格栏目信息展示
void SpeicInformUpdata(uint8_t updata_style)
{
	switch(updata_style)//切换手柄以及插拔入手柄
	{
		case 1: //TMBA,TMBB,EMBA ,EMBB,YIMOXUE
						//清楚规格显示，清除刀具图片
			break;
		
		case 2: //手动识别，磨头（清楚规格显示），显示刀具磨头图片
			      
			break;
		case 3: //手动识别，磨头（清楚规格显示），显示刀具刨刀图片
			break;
	}
}


void SpeedControl(uint8_t addsub_sign)
{
	static uint8_t speed_Gear_value=0;
//#define TMBB_ONLINE 1
//#define TMBA_ONLINE 2
//#define EMBA_ONLINE 3
//#define EMBB_ONLINE 4
//#define PXBA_ONLINE 5
//#define PXBB_ONLINE 6
//#define MX_YIM_ONLINE 8//磨削 一体磨
//#define MX_YIP_ONLINE 7//磨削 一体刨
//#define PX_YIM_ONLINE 9//刨削 一体磨
//#define PX_YIP_ONLINE 10//刨削 一体刨
	
	switch(Workvalue_s.hand_model)
	{
		case JMB_ONLINE:
			switch(addsub_sign)
					{
					 case 1:
						 if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=10000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=30000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed; 
								if(Workvalue_s.set_speed<10000)
								 Workvalue_s.set_speed=10000;
							}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=20000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-5000;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<TmbbMinSpeed)
								Workvalue_s.set_speed=TmbbMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>30000)
						 Workvalue_s.set_speed=30000;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbFastlyStepSpeed;
								 if(Workvalue_s.set_speed>30000)
									{
										Workvalue_s.set_speed=30000;
									}
							 }
							 break;
					}
				break;
			
		case TMBB_ONLINE://TMBB
				 switch(addsub_sign)
					{
					 case 1:
						 if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=50000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=60000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed; 
								if(Workvalue_s.set_speed<TmbbMinSpeed)
								 Workvalue_s.set_speed=TmbbMinSpeed;
							}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<TmbbMinSpeed)
								Workvalue_s.set_speed=TmbbMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>TmbbMaxSpeed)
						 Workvalue_s.set_speed=TmbbMaxSpeed;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbFastlyStepSpeed;
								 if(Workvalue_s.set_speed>TmbbMaxSpeed)
									{
										Workvalue_s.set_speed=TmbbMaxSpeed;
									}
							 }
							 break;
					}
				break;
		 case TMBA_ONLINE://TMBA
					  switch(addsub_sign)
						{
							case 1:
							if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=50000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=70000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
								{
									Workvalue_s.set_speed=	Workvalue_s.set_speed-TmbaSlowlyStepSpeed; 
									if(Workvalue_s.set_speed<TmbaMinSpeed)
										Workvalue_s.set_speed=TmbaMinSpeed;
									}
							 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbaFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbaSlowlyStepSpeed;
								if(Workvalue_s.set_speed<TmbaMinSpeed)
									Workvalue_s.set_speed=TmbaMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbaSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>TmbaMaxSpeed)
						 Workvalue_s.set_speed=TmbaMaxSpeed;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbaSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbaFastlyStepSpeed;
								 if(Workvalue_s.set_speed>TmbaMaxSpeed)
									{
										Workvalue_s.set_speed=TmbaMaxSpeed;
									}
							 }
							 break;
						}
				//TMBa的配置速度，是否储存标志（松开手，后计时器5秒后保存），未写
			 break;
				 case EMBA_ONLINE://EMBA
					  switch(addsub_sign)
						{
							case 1:
								if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=50000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=70000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else{
							Workvalue_s.set_speed=Workvalue_s.set_speed-EmbaSlowlyStepSpeed; 
						 if(Workvalue_s.set_speed<EmbaMinSpeed)
							 Workvalue_s.set_speed=EmbaMinSpeed;
						}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-EmbaFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-EmbaSlowlyStepSpeed;
								if(Workvalue_s.set_speed<EmbaMinSpeed)
								Workvalue_s.set_speed=EmbaMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+EmbaSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>EmbaMaxSpeed)
						 Workvalue_s.set_speed=EmbaMaxSpeed;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+EmbaSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+EmbaFastlyStepSpeed;
								 if(Workvalue_s.set_speed>EmbaMaxSpeed)
									{
										Workvalue_s.set_speed=EmbaMaxSpeed;
									}
							 }
							 break;
						}
						//TMBa的配置速度，是否储存标志（松开手，后计时器5秒后保存），未写
			 break;		
						case EMBB_ONLINE://EMBB
					  switch(addsub_sign)
						{
							case 1:
								if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=50000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=70000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else{
							Workvalue_s.set_speed=Workvalue_s.set_speed-EmbbSlowlyStepSpeed; 
						 if(Workvalue_s.set_speed<EmbbMinSpeed)
							 Workvalue_s.set_speed=EmbbMinSpeed;
					 }
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-EmbbFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-EmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<EmbbMinSpeed)
								Workvalue_s.set_speed=EmbbMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+EmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>EmbbMaxSpeed)
						 Workvalue_s.set_speed=EmbbMaxSpeed;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+EmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+EmbbFastlyStepSpeed;
									if(Workvalue_s.set_speed>EmbbMaxSpeed)
									{
										Workvalue_s.set_speed=EmbbMaxSpeed;
									}
							 }
							 break;
						}
						
						//TMBa的配置速度，是否储存标志（松开手，后计时器5秒后保存），未写
			 break;	
			case PXBA_ONLINE://PXBA
			case PXBB_ONLINE://PxBB
					  switch(addsub_sign)
						{
							case 1:
								if(Workvalue_s.tool_model==0)//刀具磨头形式
									{
										if(Workvalue_s.FastGear_flag)
											{
//												speed_Gear_value++;
//												if(speed_Gear_value>3)speed_Gear_value=0;
//												if(speed_Gear_value==1)
//												{
//													Workvalue_s.set_speed=3000;
//												}
//												else if(speed_Gear_value==2)
//												{
//													Workvalue_s.set_speed=10000;
//												}
//												else if(speed_Gear_value==3)
//												{
//													Workvalue_s.set_speed=13000;
//												}
//												Workvalue_s.FastGear_flag=0;
											}
											else
												{
													if(paoxueSpeciValue_F[8]==0x8c||paoxueSpeciValue_F[8]==0x1c)
													{
															Workvalue_s.set_speed=Workvalue_s.set_speed-2000; 
															if(Workvalue_s.set_speed<10000)
															Workvalue_s.set_speed=10000;
													}
													else
													{
															Workvalue_s.set_speed=Workvalue_s.set_speed-PXDXSlowlyStepSpeed; 
															if(Workvalue_s.set_speed<PXDXMinSpeed)
															Workvalue_s.set_speed=PXDXMinSpeed;
													}
												}
									 }
									else
									{
										if(Workvalue_s.FastGear_flag)
										{
											speed_Gear_value++;
											if(speed_Gear_value>3)speed_Gear_value=0;
											if(speed_Gear_value==1)
											{
												Workvalue_s.set_speed=1000;
											}
											else if(speed_Gear_value==2)
											{
												Workvalue_s.set_speed=4000;
											}
											else if(speed_Gear_value==3)
											{
												Workvalue_s.set_speed=6000;
											}
											Workvalue_s.FastGear_flag=0;
										}
										else{
										Workvalue_s.set_speed=Workvalue_s.set_speed-PXWFSlowlyStepSpeed; 
									 if(Workvalue_s.set_speed<PXDXMinSpeed_P)
										 Workvalue_s.set_speed=PXDXMinSpeed_P;
											}
									}
						 break;
					 case 2:
						 if(Workvalue_s.tool_model==0)
							 {
								 if(paoxueSpeciValue_F[8]==0x8c||paoxueSpeciValue_F[8]==0x1c)
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed-5000; 
										if(Workvalue_s.set_speed<10000)
										Workvalue_s.set_speed=10000;
								  }
								  else
									{
										
												Workvalue_s.set_speed=Workvalue_s.set_speed-PXDXFastlyStepSpeed;
												if(Workvalue_s.set_speed<PXDXMinSpeed)
												Workvalue_s.set_speed=PXDXMinSpeed;
											
									}
							}
						 else
							{
								
								if(Workvalue_s.set_speed>2000)
								{
									Workvalue_s.set_speed=Workvalue_s.set_speed-PXWFFastlyStepSpeed;
								}
								else
								{
									Workvalue_s.set_speed=Workvalue_s.set_speed-PXWFSlowlyStepSpeed;
									if(Workvalue_s.set_speed<PXDXMinSpeed_P)
									Workvalue_s.set_speed=PXDXMinSpeed_P;
								}
							}
							break;
						
					 case 3:
						 
					  if(Workvalue_s.tool_model==0)
							{
								if(paoxueSpeciValue_F[8]==0x8c)
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed+2000; 
										if(Workvalue_s.set_speed>70000)
										Workvalue_s.set_speed=70000;
								  }
									else if(paoxueSpeciValue_F[8]==0x1c)
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed+2000; 
										if(Workvalue_s.set_speed>140000)
										Workvalue_s.set_speed=140000;
									}
								else
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed+PXDXSlowlyStepSpeed;
										if(Workvalue_s.set_speed>PXDXMaxSpeed)
										Workvalue_s.set_speed=PXDXMaxSpeed;
									}
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed+PXWFSlowlyStepSpeed;
								if(Workvalue_s.set_speed>PXWFMaxSpeed)
								Workvalue_s.set_speed=PXWFMaxSpeed;
							}
					 break;
				  case 4:
						 if(Workvalue_s.tool_model==1)
							 {
								 if(Workvalue_s.set_speed<2000)
									Workvalue_s.set_speed=Workvalue_s.set_speed+PXWFSlowlyStepSpeed;
								 else
									 {
											Workvalue_s.set_speed=Workvalue_s.set_speed+PXWFFastlyStepSpeed;
											if(Workvalue_s.set_speed>PXDXMaxSpeed_P)
											{
												Workvalue_s.set_speed=PXDXMaxSpeed_P;
											}
									 }
								 }
							else
							{
								if(paoxueSpeciValue_F[8]==0x8c)
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed+5000; 
										if(Workvalue_s.set_speed>70000)
										Workvalue_s.set_speed=70000;
								  }
									else if(paoxueSpeciValue_F[8]==0x1c)
									{
										Workvalue_s.set_speed=Workvalue_s.set_speed+5000; 
										if(Workvalue_s.set_speed>140000)
										Workvalue_s.set_speed=140000;
									}
									else
										{
											
											Workvalue_s.set_speed=Workvalue_s.set_speed+PXDXFastlyStepSpeed;
											if(Workvalue_s.set_speed>PXDXMaxSpeed)
											{
												Workvalue_s.set_speed=PXDXMaxSpeed;
											}
												 
								    }
							 }
							 break;
					}
						//PX的配置速度，是否储存标志（松开手，后计时器5秒后保存），未写
			 break;	

					case PX_YIM_ONLINE://一体刨削(整体判断磨削刨，磨削磨，未区分写入)
            switch(addsub_sign)
						{		
								case  1:							
								if(Workvalue_s.FastGear_flag)
											{
												speed_Gear_value++;
												if(speed_Gear_value>3)speed_Gear_value=0;
												if(speed_Gear_value==1)
												{
													Workvalue_s.set_speed=3000;
												}
												else if(speed_Gear_value==2)
												{
													Workvalue_s.set_speed=10000;
												}
												else if(speed_Gear_value==3)
												{
													Workvalue_s.set_speed=13000;
												}
												Workvalue_s.FastGear_flag=0;
											}
											else{
												Workvalue_s.set_speed=Workvalue_s.set_speed-1000; 
												if(Workvalue_s.set_speed<3000)
												Workvalue_s.set_speed=3000;
											}
											break;
								case 2:
									if(Workvalue_s.set_speed>=5000)
										{
											Workvalue_s.set_speed=Workvalue_s.set_speed-2000;
											if(Workvalue_s.set_speed<3000)
											{
												Workvalue_s.set_speed=3000;
											}
										}
										else
										{
											Workvalue_s.set_speed=Workvalue_s.set_speed-1000;
											if(Workvalue_s.set_speed<3000)
											Workvalue_s.set_speed=3000;
										}
								break;
								case 3:
										Workvalue_s.set_speed=Workvalue_s.set_speed+1000;
										if(Workvalue_s.set_speed>13000)
										Workvalue_s.set_speed=13000;
								break;
										
								case 4:
								
											Workvalue_s.set_speed=Workvalue_s.set_speed+2000;
											if(Workvalue_s.set_speed>13000)
											{
												Workvalue_s.set_speed=13000;
											}
									 
									break;
							}
					break;
					case PX_YIP_ONLINE:
					  switch(addsub_sign)
						{
							case 1:
								
										if(Workvalue_s.FastGear_flag)
										{
											speed_Gear_value++;
											if(speed_Gear_value>3)speed_Gear_value=0;
											if(speed_Gear_value==1)
											{
												Workvalue_s.set_speed=1000;
											}
											else if(speed_Gear_value==2)
											{
												Workvalue_s.set_speed=5000;
											}
											else if(speed_Gear_value==3)
											{
												Workvalue_s.set_speed=6000;
											}
											Workvalue_s.FastGear_flag=0;
										}
										else{
											Workvalue_s.set_speed=Workvalue_s.set_speed-500; 
											if(Workvalue_s.set_speed<500)
											Workvalue_s.set_speed=500;
										}
								break;
					 case 2:
						
							 if(Workvalue_s.set_speed>=2000)
								{
									Workvalue_s.set_speed=Workvalue_s.set_speed-1000;
								}
								else
								{
									Workvalue_s.set_speed=Workvalue_s.set_speed-500;
									if(Workvalue_s.set_speed<500)
									Workvalue_s.set_speed=500;
								}
							
						 	break;
						
					 case 3:
						 
								Workvalue_s.set_speed=Workvalue_s.set_speed+500;
								if(Workvalue_s.set_speed>6000)
								Workvalue_s.set_speed=6000;
							
						
					 break;
				  case 4:
						
								 if(Workvalue_s.set_speed<6000)
									Workvalue_s.set_speed=Workvalue_s.set_speed+1000;
											if(Workvalue_s.set_speed>6000)
											{
												Workvalue_s.set_speed=6000;
											}
									 
							 break;
					}
						//一体刨的配置速度，是否储存标志（松开手，后计时器5秒后保存），未写
			 break;	
					case MX_YIM_ONLINE:
						//一体磨削，磨
					switch(addsub_sign){
					 case 1:
						 if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=80000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=120000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed; 
								if(Workvalue_s.set_speed<10000)
								 Workvalue_s.set_speed=10000;
							}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<10000)
								Workvalue_s.set_speed=10000;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>120000)
						 Workvalue_s.set_speed=120000;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbFastlyStepSpeed;
								 if(Workvalue_s.set_speed>120000)
									{
										Workvalue_s.set_speed=120000;
									}
							 }
							 break;
					}
					break;	
				case MX_YIM16_ONLINE:
						//一体磨削，磨
					switch(addsub_sign){
					 case 1:
						 if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=120000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=160000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed; 
								if(Workvalue_s.set_speed<10000)
								 Workvalue_s.set_speed=10000;
							}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<10000)
								Workvalue_s.set_speed=10000;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>160000)
						 Workvalue_s.set_speed=160000;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbFastlyStepSpeed;
								 if(Workvalue_s.set_speed>160000)
									{
										Workvalue_s.set_speed=160000;
									}
							 }
							 break;
					}
					break;						
					case MX_YIP_ONLINE:
							//一体磨削，刨
					 switch(addsub_sign)
					{
					 case 1:
						 if(Workvalue_s.FastGear_flag)
							{
								speed_Gear_value++;
								if(speed_Gear_value>3)speed_Gear_value=0;
								if(speed_Gear_value==1)
								{
									Workvalue_s.set_speed=20000;
								}
								else if(speed_Gear_value==2)
								{
									Workvalue_s.set_speed=50000;
								}
								else if(speed_Gear_value==3)
								{
									Workvalue_s.set_speed=60000;
								}
								Workvalue_s.FastGear_flag=0;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed; 
								if(Workvalue_s.set_speed<TmbbMinSpeed)
								 Workvalue_s.set_speed=TmbbMinSpeed;
							}
						 break;
					 case 2:
						 if(Workvalue_s.set_speed>=25000)
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbFastlyStepSpeed;
							}
							else
							{
								Workvalue_s.set_speed=Workvalue_s.set_speed-TmbbSlowlyStepSpeed;
								if(Workvalue_s.set_speed<TmbbMinSpeed)
								Workvalue_s.set_speed=TmbbMinSpeed;
							}
							break;
					 case 3:
						Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
					 if(Workvalue_s.set_speed>TmbbMaxSpeed)
						 Workvalue_s.set_speed=TmbbMaxSpeed;
					 break;
					 case 4:
						 if(Workvalue_s.set_speed<20000)
							Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbSlowlyStepSpeed;
						 else
							 {
									Workvalue_s.set_speed=Workvalue_s.set_speed+TmbbFastlyStepSpeed;
								 if(Workvalue_s.set_speed>TmbbMaxSpeed)
									{
										Workvalue_s.set_speed=TmbbMaxSpeed;
									}
							 }
							 break;
					}
					
				break;						
		}
	
			//界面更新速度(*这里需要写)
			if(Workvalue_s.hand_model>0){
			speeddisplay(1,Workvalue_s.set_speed);
			}
		if(Workvalue_s.select_channel==1)//如果A手柄选中
		{
			//ChannelValue_s.A.set_speed=Workvalue_s.set_speed;//切换记忆做准备
		}
		else
		{
			//ChannelValue_s.B.set_speed=Workvalue_s.set_speed;//切换记忆做准备
		}
}

//灌注
void IrrigateContril(uint8_t key_value)
{
		static uint8_t fast_gear_counts=0;
		if(key_value==5)
		{
			if(Workvalue_s.FastGear_flag)
			{
				 fast_gear_counts++;
			
				if(fast_gear_counts==1)
				{
		    	#if water_uptake
					
					Workvalue_s.set_Irrigate=2;
					#else
					Workvalue_s.set_Irrigate=50;
					#endif
					
				}
				else if(fast_gear_counts==2)
				{
					#if water_uptake
						Workvalue_s.set_Irrigate=4;
					#else
					Workvalue_s.set_Irrigate=100;
						#endif
				}
				else if(fast_gear_counts==3)
				{
					#if water_uptake
						Workvalue_s.set_Irrigate=6;
					#else
					Workvalue_s.set_Irrigate=150;
						#endif
				}
				else if(fast_gear_counts==4)
				{
					#if water_uptake
						Workvalue_s.set_Irrigate=10;
					#else
					Workvalue_s.set_Irrigate=200;
						#endif
				}
				else if(fast_gear_counts==5)
				{
					#if water_uptake
					Workvalue_s.set_Irrigate=15;
					#else
					Workvalue_s.set_Irrigate=300;
					#endif
				}
				else if(fast_gear_counts==6)
				{  
				
					Workvalue_s.set_Irrigate=0;
					
				}
				Workvalue_s.FastGear_flag=0;
				if(fast_gear_counts>=6)fast_gear_counts=0;
			}
			else{
					#if water_uptake
					if(Workvalue_s.set_Irrigate<6){Workvalue_s.set_Irrigate++;}
					else if(Workvalue_s.set_Irrigate>=6&&Workvalue_s.set_Irrigate<12)
					{
						Workvalue_s.set_Irrigate=Workvalue_s.set_Irrigate+2;
					}
					else if(Workvalue_s.set_Irrigate==12)
					{
						Workvalue_s.set_Irrigate=15;
					}
					#else
					Workvalue_s.set_Irrigate=Workvalue_s.set_Irrigate+IrrigateStep;
					if(Workvalue_s.set_Irrigate>IrrigateMax)
						Workvalue_s.set_Irrigate=IrrigateMax;
					#endif
					}
		}
		else if(key_value==6)
		{
			#if water_uptake
				if(Workvalue_s.set_Irrigate<=6&&Workvalue_s.set_Irrigate>=1)
					{
						Workvalue_s.set_Irrigate=Workvalue_s.set_Irrigate-1;
					}
					else if(Workvalue_s.set_Irrigate>6&&Workvalue_s.set_Irrigate<=12)
					{
						Workvalue_s.set_Irrigate=Workvalue_s.set_Irrigate-2;
					}
					else if(Workvalue_s.set_Irrigate==15)
					{
						Workvalue_s.set_Irrigate=12;
					}
			#else
			if(Workvalue_s.set_Irrigate>=IrrigateStep)//大于 等于步进值
			{
				Workvalue_s.set_Irrigate=Workvalue_s.set_Irrigate-IrrigateStep;
			}
			#endif
		}
	//	Workvalue_s.Irrigate_storage_flag=start_flag;
		
		Infusiondisplay(1,Workvalue_s.set_Irrigate);
		
		if(Workvalue_s.Irrigate_start_flag)//运行过程中改变流量参数 （）
		{
				PUMPBehavior();
		}
	//界面更新数据
}

void Irrigatesign()//是否需要一直发数据？？？
{	
			KeyBeep_flag=1;
			Workvalue_s.Irrigate_start_flag=Workvalue_s.Irrigate_start_flag?stop_flag:start_flag;//灌注信号
			if(Workvalue_s.Irrigate_start_flag)
			{
				PumpScreenDebugPoint(101U, Workvalue_s.set_Irrigate, Workvalue_s.Irrigate_start_flag);
				//界面黄色启动
				Irrndisplay(1);
				
			
			}
			else
			{
				PumpScreenDebugPoint(102U, Workvalue_s.set_Irrigate, Workvalue_s.Irrigate_start_flag);
//				Workvalue_s.set_Irrigate=0;
				Irrndisplay(2);
		
	
			}
			PUMPBehavior();
}


uint8_t  APump(uint8_t pump_data)
{
	uint8_t temp1;
	if(pump_data>70)pump_data=70;
//	if(pump_data>65&&pump_data<=70)
//				temp1=(uint32_t )pump_data*(pump_data*0.02+2.1);//注水
	 if(pump_data>60&&pump_data<=70)    
				temp1=(uint32_t )(pump_data-10)*(pump_data*0.02+1.6);
	 
	 	else if(pump_data>50&&pump_data<=60)
			temp1=(uint32_t )(pump_data-10)*(pump_data*0.02+1.8);
		
	else if(pump_data>=40&&pump_data<=50)
				temp1=(uint32_t )(pump_data-5)*(pump_data*0.02+1.9);
	
	else if(pump_data>=30&&pump_data<40)
				temp1=(uint32_t )(pump_data-3)*(pump_data*0.02+2.3);
	else 
				temp1 = (uint32_t )(pump_data*2.5);	
	return temp1;
}

//注水
void InjectionContril(uint8_t key_value)
{
	uint8_t temp1=0;
	static uint8_t fast_gear_counts=0;
	if(key_value==7)
		{
			
			if(!Workvalue_s.Injection_drain_flag)//排空未开启的情况下，+
			{
					KeyBeep_flag=1;
				if(Workvalue_s.FastGear_flag)
				{
					fast_gear_counts++;
				
					if(fast_gear_counts==1)
						{
							Workvalue_s.set_Injection=10;//一档10ml
						}
						else if(fast_gear_counts==2)
						{
							Workvalue_s.set_Injection=20;//二档20ml
						}
						else if(fast_gear_counts==3)
						{
							Workvalue_s.set_Injection=30;//三档20ml
						}
						else if(fast_gear_counts==4)
						{
							Workvalue_s.set_Injection=50;//三档20ml
						}
							else if(fast_gear_counts==5)
						{
							Workvalue_s.set_Injection=70;//三档20ml
						}
							else if(fast_gear_counts==6)
						{
							Workvalue_s.set_Injection=0;//三档20ml
						}
						Workvalue_s.FastGear_flag=0;
						if(fast_gear_counts>=6)fast_gear_counts=0;
				}
				else{
					if(Workvalue_s.set_Injection<30)Workvalue_s.set_Injection=Workvalue_s.set_Injection+5;
					else{
					Workvalue_s.set_Injection=Workvalue_s.set_Injection+InjectionStep;
					if(Workvalue_s.set_Injection>InjectionMax)Workvalue_s.set_Injection=InjectionMax;
					}
				}
				injectiondisplay(1,Workvalue_s.set_Injection);
				if(Workvalue_s.Foot_start_flag)
				{
					temp1=APump(Workvalue_s.set_Injection);
					Pump_SetSpeed_A(temp1);//泵运行流量
				}
				
			}
		}
		else if(key_value==8)
		{
			if(!Workvalue_s.Injection_drain_flag)//排空未开启的情况下，-
			{
					KeyBeep_flag=1;
				if(Workvalue_s.set_Injection>=5)
					{
						if(Workvalue_s.set_Injection<=30)
						{
							Workvalue_s.set_Injection=Workvalue_s.set_Injection-5;
						}
						else{
							Workvalue_s.set_Injection=Workvalue_s.set_Injection-InjectionStep;
						}
					}
					injectiondisplay(1,Workvalue_s.set_Injection);
					if(Workvalue_s.Foot_start_flag)
					{
						temp1=APump(Workvalue_s.set_Injection);
						Pump_SetSpeed_A(temp1);//泵运行流量
					}
			}
		}
	}

void	Injectionsign()//待修改
{
	uint32_t temp1=0;
	
	if(Workvalue_s.Achanell_online_flag||Workvalue_s.Bchanell_online_flag){
		if(!Workvalue_s.Foot_start_flag&&!Workvalue_s.beep_Alarm_flag&&!Workvalue_s.Handle_MOTORWorking_flag)//泵运行（非排空），状态下，启动或者停止排空按钮（这里需要思考下，如果启动，则停止排空）？？***
				{
						KeyBeep_flag=1;
					Workvalue_s.Injection_drain_flag=Workvalue_s.Injection_drain_flag?stop_flag:start_flag;//排空信号
					if(Workvalue_s.Injection_drain_flag)
						{
							temp1=70;
							//按钮更新图标（黄色）
							//InjectionMax 发送最大值，并且开始计时
							temp1=(uint32_t )temp1*(temp1*0.02+2.1);
							draindisplay(2);
								temp1=APump(temp1);
							Pump_SetSpeed_A(temp1);
							injectiondisplay(1,70);
						}
						else
						{
							Pump_SetSpeed_A(0);//停止排空
							injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
							//停止有两个信号来源，正常按键停止
							draindisplay(1);//按钮更新图标（白色）
						}
				}
				else
				{
					
				//	Workvalue_s.Injection_drain_flag=stop_flag;//当正常运行泵开始的时候，工作排空无效，这里需要在Workvalue_s.Injection_start_flag==ture的时候判断，如果在运行中则不管排空是否启动都立马变为停止，这句判断不应该在写在此处
					//按钮更新图标（白色）或许要写到报警机制里面去
				}
		}
	else//拔掉所有的手柄，或者运行中拔掉所有手柄,或许要写到报警机制里面去
	{
		//停止，灰色
			draindisplay(1);
			Workvalue_s.Injection_drain_flag=stop_flag;
			Pump_SetSpeed_A(0);//停止排空
	}
}


void FreqContril(uint8_t key_value)//当刀具处于刨刀状态下，这里需要判断？？？
{	
	
	if(Workvalue_s.set_Direction==Direction_osc){
			KeyBeep_flag=1;
		if(key_value==9)
		{
			Workvalue_s.set_Freq =Workvalue_s.set_Freq+FreqStep;
			if(Workvalue_s.set_Freq>FreqMax)
				Workvalue_s.set_Freq=FreqMax;
		}
		else
		{
			if(	Workvalue_s.set_Freq>FreqStep)
			{
					Workvalue_s.set_Freq=	Workvalue_s.set_Freq-FreqStep;
			}
		}
		freqdisplay(0,Workvalue_s.set_Freq);
	}
	
}

//运行方向控制
void DirectionControl(uint8_t key_value)
{
//	#define TMBB_ONLINE 1
//#define TMBA_ONLINE 2
//#define EMBA_ONLINE 3
//#define EMBB_ONLINE 4
//#define PXBA_ONLINE 5
//#define PXBB_ONLINE 6
//#define MX_YIM_ONLINE 7//磨削 一体磨
//#define MX_YIP_ONLINE 8//磨削 一体刨
//#define PX_YIM_ONLINE 9//刨削 一体磨
//#define PX_YIP_ONLINE 10//刨削 一体刨
	if(!Workvalue_s.MOTORWorking_flag&&Workvalue_s.Alarm_value==0){
		
	if(key_value==13) 
	{
	
		switch(Workvalue_s.hand_model)
		{
			case JMB_ONLINE://tmbb
			case TMBB_ONLINE://tmbb
			case TMBA_ONLINE://tmba
			case EMBA_ONLINE://emba
			case EMBB_ONLINE://embb
			case MX_YIM_ONLINE://磨削单向 最大10万转
			case MX_YIM16_ONLINE://磨削单向 最大10万转
			clockwisedisplay(2);//UI跟新
			anticlockwisedisplay(1);
			OSCdiplay(0);
				KeyBeep_flag=1;
			break;
			case PXBA_ONLINE://pxba 
			case PXBB_ONLINE://PXBB
					KeyBeep_flag=1;
			if(Workvalue_s.tool_model==0)//刀具模式是磨头
			{
					clockwisedisplay(2);//UI跟新
					anticlockwisedisplay(1);
					OSCdiplay(0);
			}
			else
			{
				freqdisplay(1,0);//隐藏频率
				clockwisedisplay(2);//UI跟新
				anticlockwisedisplay(1);
				OSCdiplay(1);
			}
			break;
			case PX_YIM_ONLINE://一体刨刀(磨头)
					KeyBeep_flag=1;
					clockwisedisplay(2);//UI跟新
					anticlockwisedisplay(1);
					OSCdiplay(0);
			break;
			case PX_YIP_ONLINE://一体刨（刨刀）
					KeyBeep_flag=1;
					clockwisedisplay(2);//UI跟新
					anticlockwisedisplay(1);
					OSCdiplay(1);
			freqdisplay(1,0);//隐藏频率
				//不执行
			break;
			
			case MX_YIP_ONLINE://一体磨削往复6万转，国外
				//什么都不执行，就没有单向模式
				return;
		}
		Workvalue_s.set_Direction=Direction_forward;//模式正转
	}
		if(key_value==14)
		{
			switch(Workvalue_s.hand_model)
			{
				case JMB_ONLINE://tmbb
				case TMBB_ONLINE://tmbb
				case TMBA_ONLINE:
				case EMBA_ONLINE:
				case EMBB_ONLINE:
				case MX_YIM_ONLINE:
					case MX_YIM16_ONLINE://磨削单向 最大10万转
					KeyBeep_flag=1;
				clockwisedisplay(1);//UI跟新
				anticlockwisedisplay(2);
				OSCdiplay(0);
					//反转图标
				break;
				
				case PXBA_ONLINE://pxba 
				case PXBB_ONLINE://PXBB
					KeyBeep_flag=1;
				if(Workvalue_s.tool_model==0)//刀具模式是磨头
				{
					clockwisedisplay(1);//UI跟新
					anticlockwisedisplay(2);
					OSCdiplay(0);
				}
				else
				{
					freqdisplay(1,0);//隐藏频率
					clockwisedisplay(1);//UI跟新
					anticlockwisedisplay(2);
					OSCdiplay(1);
				}
				break;
				case PX_YIM_ONLINE://一体刨刀
							clockwisedisplay(1);//UI跟新
							anticlockwisedisplay(2);
							OSCdiplay(0);
					KeyBeep_flag=1;
					break;
				case PX_YIP_ONLINE:
				//不执行操作
					KeyBeep_flag=1;
					clockwisedisplay(1);//UI跟新
					anticlockwisedisplay(2);
					OSCdiplay(1);
				freqdisplay(1,0);//隐藏频率
					break;
				case MX_YIP_ONLINE://一体磨削往复6万转，国外
				//什么都不执行，就没有单向模式
				return;
		}
		Workvalue_s.set_Direction=Direction_reverse;//模式反转
	}
	if(key_value==15)//往复按钮，只有刨刀下才有，所以直接判断模式
	{
		if(Workvalue_s.hand_model==PXBA_ONLINE||Workvalue_s.hand_model==PXBB_ONLINE||Workvalue_s.hand_model==PX_YIP_ONLINE){
			if(Workvalue_s.tool_model==1)//刀具模式等于刨刀
			{
					KeyBeep_flag=1;
						freqdisplay(0,Workvalue_s.set_Freq);//隐藏频率
						clockwisedisplay(1);//UI跟新
						anticlockwisedisplay(1);
						OSCdiplay(2);
						Workvalue_s.set_Direction=Direction_osc;
			}
		}
	}
	}
}



void WayContril(uint8_t key_value)//脚控在线还需要再插拔脚踏的过程中去识别判断（脚踏需要在中断里面判断是否成功，不会受到主函数延时，比如在运行过程或者循环报警中，脚踏掉线一切停止）
{
	if(!Workvalue_s.MOTORWorking_flag&&Workvalue_s.Alarm_value==0){
	switch(key_value)
	{
		case 16://脚控
	
			if(Workvalue_s.footcontrol_online_flag){
					KeyBeep_flag=1;
				Workvalue_s.set_Way=footcontrol;
				if(Workvalue_s.hand_model==PXBA_ONLINE)//如果等于EMBA的时候
				{
					//脚控ui,中间手控白色
					ssc_Connectfootpedal(2);
					ssc_Connecthandel(1);
					ssc_Connecttouch(1);
				}
				else 
				{
					ssc_Connectfootpedal(2);
					ssc_Connecthandel(0);
					ssc_Connecttouch(1);
					//中间手控黑色
				}
			}
			else
			{
				if(Workvalue_s.hand_model==PXBA_ONLINE)
				{
						Workvalue_s.set_Way=handelcontrol;
						ssc_Connectfootpedal(0);
						ssc_Connecthandel(2);
						ssc_Connecttouch(1);
				}
				else
				{
						ssc_Connectfootpedal(0);
						ssc_Connecthandel(0);
						ssc_Connecttouch(1);
					Workvalue_s.set_Way=noControl;
				}
			}
		break;
		case 17://手控
		//	Workvalue_s.beep_Alarm_flag=0;//测试用
			if(Workvalue_s.hand_model==PXBA_ONLINE)//如果工作模式是PXBA,???这个地方记忆需要整改
				{
					//跟新ui
					KeyBeep_flag=1;
					Workvalue_s.set_Way=handelcontrol;
					if(Workvalue_s.footcontrol_online_flag)//脚踏在线情况下
						{
							 //手控选中，脚控白色
							ssc_Connectfootpedal(1);
							ssc_Connecthandel(2);
							ssc_Connecttouch(1);
						}
						else
						{
						    //手控选中，脚控白色
								ssc_Connectfootpedal(0);
								ssc_Connecthandel(2);
								ssc_Connecttouch(1);
						}
					
				 }
			break;
		case 18://触控
			//暂时待定,弹出框
			KeyBeep_flag=1;
			Workvalue_s.set_Way=touchcontrol;
				LCD_Show_Picture(0x1430,401);//弹出框
			break;
			case 40://触控
			//暂时待定,弹出框
				KeyBeep_flag=1;
			if(Workvalue_s.footcontrol_online_flag)
				{
					WayContril(16);//递归函数
				}
				else
				{
					if(Workvalue_s.hand_model==PXBA_ONLINE)
					{
						WayContril(17);//递归函数
					}
					else
					{
						Workvalue_s.set_Way=noControl;//无控制
						
					}
				}
					LCD_Disappear_Picture(0x1430);//隐藏弹出框
	  }
  }
}

//
void MotoPaoDaoSelectControl(uint8_t key_value)
{
	if(!Workvalue_s.MOTORWorking_flag&&Workvalue_s.Alarm_value==0)
		{
			if(Workvalue_s.hand_model==5||Workvalue_s.hand_model==6)
			{
				KeyBeep_flag=1;
				if(key_value==20)//手动磨头
				{
					if(Workvalue_s.tool_model==1){
						PAOORMODisplay(1);
						if(Workvalue_s.set_Direction==Direction_osc)
						{
							//隐藏频率数据;
							freqdisplay(1,0);
						}
						Workvalue_s.tool_model=0;//磨头,1刨刀
						Workvalue_s.set_speed=50000;
						paoxueSpeciValue_F[8]=0x8c;
						Workvalue_s.set_Direction=Direction_forward;//默认正转
						clockwisedisplay(2);//UI跟新
						anticlockwisedisplay(1);
						OSCdiplay(0);
						speeddisplay(1,Workvalue_s.set_speed);
						Tooldisplay(2);
						//更新速度
					}
				}
				else//刨刀
				{
				
					if(Workvalue_s.tool_model==0)
					{
						PAOORMODisplay(2);
						Workvalue_s.tool_model=1;//磨头,1刨刀
						Workvalue_s.set_speed=5000;
						Workvalue_s.set_Freq=40;
						Workvalue_s.set_Direction=Direction_osc;//默认往复弹出频率
						freqdisplay(0,Workvalue_s.set_Freq);
						clockwisedisplay(1);//UI跟新
						anticlockwisedisplay(1);
						OSCdiplay(2);
						speeddisplay(1,Workvalue_s.set_speed);
						Tooldisplay(3);
					}
				}
			}
		}
}

			

void OpenPositionControl(uint8_t key_value)
{
	if(!Workvalue_s.MOTORWorking_flag&&Workvalue_s.Alarm_value==0)
		{
		 if(Workvalue_s.hand_model==5||Workvalue_s.hand_model==6)//PXBA pxBB模式下才有开口定位功能
			{
			
				if(Workvalue_s.tool_model==1)//刨刀模式
					{
							KeyBeep_flag=1;
						if(key_value==22)
						{
							//开口逆时针动作
							 BrushlessMotor_SetPosition(Workvalue_s.select_channel, 5, 1);
						}
						else
						{
							
							//开口顺时针动作开
							 BrushlessMotor_SetPosition(Workvalue_s.select_channel, 4, 1);
						}
				}
			}
		}
}



//界面抑制（当手柄A和B都不在线时候）
void InterfaceSuppression()
{
		OPENpostionDisplay(0);//隐藏开口定位
		PAOORMODisplay(0);//隐藏磨头
		Tooldisplay(0);//隐藏刀具信息
		freqdisplay(1,0);//隐藏频率
		Ahandeldisplay(0);//A手柄无连接
		Bhandeldisplay(0);//B手柄无连接
		speeddisplay(0,0);//速度暗黑
		injectiondisplay(0,0);//注水暗黑
		//Infusiondisplay(0,0);//灌注暗黑
		draindisplay(0);//排空注水暗黑
		//Irrndisplay(0);//灌注启动暗黑
		clockwisedisplay(0);
		anticlockwisedisplay(0);
		OSCdiplay(0);
		specidisplay(0,0,0,0);//隐藏刀具规格
}


//SSC 记忆
void MemorySwitch(uint8_t select_channel_flag)
{
	if(select_channel_flag==1)
	{
		Workvalue_s.set_Direction=ChannelValue_s.A.set_Direction;
			Workvalue_s.set_speed=ChannelValue_s.A.set_speed;
			Workvalue_s.set_Freq=ChannelValue_s.A.set_Freq;
			Workvalue_s.set_Way =ChannelValue_s.A.set_Way;
			Workvalue_s.tool_model=ChannelValue_s.A.tool_model;
			Workvalue_s.Handle_mutual_flag=ChannelValue_s.A.manual_flag;
	}
	else if(select_channel_flag==2)
	{
			Workvalue_s.set_Direction=ChannelValue_s.B.set_Direction;
			Workvalue_s.set_speed=ChannelValue_s.B.set_speed;
			Workvalue_s.set_Freq=ChannelValue_s.B.set_Freq;
			Workvalue_s.set_Way =ChannelValue_s.B.set_Way;
			Workvalue_s.tool_model=ChannelValue_s.B.tool_model;
			Workvalue_s.Handle_mutual_flag=ChannelValue_s.B.manual_flag;
	}
	if(Workvalue_s.set_Direction==Direction_osc)
	{
		
	}
	else if(Workvalue_s.set_Direction==Direction_forward)
	{
		
	}
	else if(Workvalue_s.set_Direction==Direction_reverse)
	{
		
	}
	if(Workvalue_s.hand_model==PXBB_ONLINE||Workvalue_s.hand_model==PXBB_ONLINE)
			{
				
				if(Workvalue_s.Handle_mutual_flag)
				{
						//自动识别图标开启
				}
				else
				{
					 //手动模式开启
					if(Workvalue_s.tool_model)
					{
						//打开定位开关
					}
					else
					{
						//关闭定位开关
					}
				}
			}
		
	}


void SwitchChanelassignment_s(uint8_t siga)
{
	KeyBeep_flag=1;
	Workvalue_s.Injection_drain_flag=0;
	Pump_SetSpeed_A(0);//停止排空
	injectiondisplay(1,Workvalue_s.set_Injection);//还原设置的流水值
	//停止有两个信号来源，正常按键停止
	
	draindisplay(1);//按钮更新图标（白色）
	Workvalue_s.fenti_switch_flag=0;
	switch(siga)
	{
		case 1://拔掉A手柄
		Ahandeldisplay(1);
		if(Workvalue_s.select_channel==1)
		{
			specidisplay(0,0,0,0);//显示刀具信息
			LCD_Disappear_Picture(0x1440);
			if(Workvalue_s.Bchanell_online_flag)
			{
				if(ChannelValue_s.B.hand_model==TMBA_ONLINE||ChannelValue_s.B.hand_model==TMBB_ONLINE\
					||ChannelValue_s.B.hand_model==EMBA_ONLINE||ChannelValue_s.B.hand_model==EMBB_ONLINE\
					||ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.B.hand_model==JMB_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE
					)
					{
						if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
						{
							Bhandeldisplay(9);//一体磨削选中
						}
						else{
							Bhandeldisplay(5);//耳膜图片选中
						}
							if(ChannelValue_s.B.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
							{
								clockwisedisplay(0);
								anticlockwisedisplay(0);
								OSCdiplay(2);	//显示默认选中正方向
							}
							else
							{
								
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//
							}
							PAOORMODisplay(0);//隐藏开口定位
							Tooldisplay(0);//隐藏刀具信息
							freqdisplay(1,0);//隐藏频率
							
							Workvalue_s.set_Direction=Direction_forward;//正转
							
							if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.B.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==TMBB_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
							{
						  	Workvalue_s.set_speed=120000;//
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							Workvalue_s.select_channel=2;//选择2通道
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;//赋值工作手柄
								
					}
					else if(ChannelValue_s.B.hand_model==PXBA_ONLINE||ChannelValue_s.B.hand_model==PXBB_ONLINE||ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
							{
									
									Bhandeldisplay(7);
									PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
									Tooldisplay(0);//默认弯刨
									specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
							}
							else
								{
									R200_K8_5ON();
									Workvalue_s.Handle_mutual_flag=1;
										Bhandeldisplay(6);//分体图标选中
//										PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//										Tooldisplay(3);//默认弯刨
									AutomaticDisplay(1);
								}
						
							clockwisedisplay(1);
							anticlockwisedisplay(1);
							OSCdiplay(2);	//显示默认选中往复方向
						
						
							freqdisplay(0,40);//隐藏频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;
							Workvalue_s.select_channel=2;//选择2通道
				
					}
					else 
					{
							specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
							Bhandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
					
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;
							Workvalue_s.select_channel=2;//选择2通道
					}
					
			}
			else
			{
				//全部黑暗处理，除了脚踏
				InterfaceSuppression();
				Workvalue_s.hand_model=0;
				Workvalue_s.select_channel=0;
			}
		}
		else if(Workvalue_s.select_channel==2)
		{
			//本来选择则不用
		}
		Workvalue_s.Achanell_online_flag=0;
		break;
		case 2://拔掉B手柄
		Bhandeldisplay(1);
		if(Workvalue_s.select_channel==2)
		{
			LCD_Disappear_Picture(0x1440);
			specidisplay(0,0,0,0);//显示刀具信息
			if(Workvalue_s.Achanell_online_flag)
			{
					if(ChannelValue_s.A.hand_model==TMBA_ONLINE||ChannelValue_s.A.hand_model==TMBB_ONLINE\
					||ChannelValue_s.A.hand_model==EMBA_ONLINE||ChannelValue_s.A.hand_model==EMBB_ONLINE\
					||ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.A.hand_model==JMB_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE
					)
					{
							if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
							{
								Ahandeldisplay(9);//耳膜图片选中
							}
							else
							{
								Ahandeldisplay(5);//耳膜图片选中
							}
							if(ChannelValue_s.A.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
							{
								clockwisedisplay(0);
								anticlockwisedisplay(0);
								OSCdiplay(2);	//显示默认选中正方向
							}
						
							else{
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//显示默认选中正方向
							}
							
								PAOORMODisplay(0);//隐藏开口定位
								Tooldisplay(0);//隐藏刀具信息
								freqdisplay(1,0);//隐藏频率
							
								Workvalue_s.set_Direction=Direction_forward;//正转
							
							if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.A.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==TMBB_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
							{
								Workvalue_s.set_speed=120000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							Workvalue_s.select_channel=1;//选择1通道
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;//赋值工作手柄
								
					}
					else if(ChannelValue_s.A.hand_model==PXBA_ONLINE||ChannelValue_s.A.hand_model==PXBB_ONLINE||ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
							{
									Ahandeldisplay(7);
									specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
								PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							}
							else{
								R200_K8_2ON();
								Workvalue_s.Handle_mutual_flag=1;
								AutomaticDisplay(1);
							Ahandeldisplay(6);//分体图标选中
//							PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//							Tooldisplay(3);//默认弯刨
							}
						
							clockwisedisplay(1);
							anticlockwisedisplay(1);
							OSCdiplay(2);	//显示默认选中往复方向
						
						
							freqdisplay(0,40);//隐藏频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;
							Workvalue_s.select_channel=1;//选择2通道
				
					}
					else 
					{
							specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
							Ahandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
					
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;
							Workvalue_s.select_channel=1;//选择2通道
					}
					
			}
			else
			{
				//全部黑暗处理，除了脚踏
				InterfaceSuppression();
				Workvalue_s.hand_model=0;
				Workvalue_s.select_channel=0;
			}
		}
		else if(Workvalue_s.select_channel==1)
		{
			//本来选择则不用
		}
			Workvalue_s.Bchanell_online_flag=0;
		break ;
		case 3://插入A手柄
				specidisplay(0,0,0,0);//显示刀具信息
			LCD_Disappear_Picture(0x1440);
			if(Workvalue_s.select_channel==2)//如果B区有手柄，先把B区图标改为连接
			{
				if(ChannelValue_s.B.hand_model==TMBA_ONLINE||ChannelValue_s.B.hand_model==TMBB_ONLINE\
				||ChannelValue_s.B.hand_model==EMBA_ONLINE||ChannelValue_s.B.hand_model==EMBB_ONLINE\
				||ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE\
				||ChannelValue_s.B.hand_model==JMB_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE
				)
				{
					if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
						{
							Bhandeldisplay(8);
						}
					else
						{
							Bhandeldisplay(2);
						}
				}
				else if(ChannelValue_s.B.hand_model==PXBA_ONLINE||ChannelValue_s.B.hand_model==PXBB_ONLINE)
				{
					 //b_Handel_F_连接;//连接图标
						Bhandeldisplay(3);
				}
				else 
				{
					 //b_Handel_y_连接;//连接图标
						Bhandeldisplay(4);
				}
			}
			if(ChannelValue_s.A.hand_model==TMBA_ONLINE||ChannelValue_s.A.hand_model==TMBB_ONLINE\
					||ChannelValue_s.A.hand_model==EMBA_ONLINE||ChannelValue_s.A.hand_model==EMBB_ONLINE\
					||ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.A.hand_model==JMB_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE
					)
					{
						if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
							{
									Ahandeldisplay(9);//耳膜图片选中
							}
							else
							{
									Ahandeldisplay(5);//耳膜图片选中
							}
							if(ChannelValue_s.A.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
								{
									clockwisedisplay(0);
									anticlockwisedisplay(0);
									OSCdiplay(2);	//显示默认选中正方向
								}
							else{
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//显示默认选中正方向
							}
							
								PAOORMODisplay(0);//隐藏开口定位
								Tooldisplay(0);//隐藏刀具信息
								freqdisplay(1,0);//隐藏频率
							
								Workvalue_s.set_Direction=Direction_forward;//正转
							
							if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.A.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==TMBB_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
							{
								Workvalue_s.set_speed=120000;//磨钻最高50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
					}
					else if(ChannelValue_s.A.hand_model==PXBA_ONLINE||ChannelValue_s.A.hand_model==PXBB_ONLINE||ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
							{
								Ahandeldisplay(7);
								//显示刀具规格信息
								specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
									PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
									Tooldisplay(0);//默认弯刨
							}
							else
							{
								R200_K8_2ON();
								Workvalue_s.Handle_mutual_flag=1;
								//specidisplay(0,0,0,0);//显示刀具信息
								Ahandeldisplay(6);//分体图标选中
								AutomaticDisplay(1);
//								PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//								Tooldisplay(3);//默认弯刨
							}
						
							clockwisedisplay(1);
							anticlockwisedisplay(1);
							OSCdiplay(2);	//显示默认选中往复方向
						
						
							freqdisplay(0,40);//打开频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//往复转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
					
				
					}
					else if(ChannelValue_s.A.hand_model==PX_YIM_ONLINE)
					{
							specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
							Ahandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
						}
					Workvalue_s.hand_model=ChannelValue_s.A.hand_model;
					Workvalue_s.Achanell_online_flag=start_flag;
					Workvalue_s.select_channel=1;
			
		break;
		case 4://插入B手柄
				LCD_Disappear_Picture(0x1440);
			if(Workvalue_s.select_channel==1)//如果B区有手柄，先把B区图标改为连接
			{
				specidisplay(0,0,0,0);//显示刀具信息
				if(ChannelValue_s.A.hand_model==TMBA_ONLINE||ChannelValue_s.A.hand_model==TMBB_ONLINE\
				||ChannelValue_s.A.hand_model==EMBA_ONLINE||ChannelValue_s.A.hand_model==EMBB_ONLINE\
				||ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE\
				||ChannelValue_s.A.hand_model==JMB_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE
				)
				{
					if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE){
						Ahandeldisplay(8);
					}
					else
						{
						Ahandeldisplay(2);
					}
				}
				else if(ChannelValue_s.A.hand_model==PXBA_ONLINE||ChannelValue_s.A.hand_model==PXBB_ONLINE)
				{
					 //b_Handel_F_连接;//连接图标
						Ahandeldisplay(3);
				}
				else 
				{
					 //b_Handel_y_连接;//连接图标
						Ahandeldisplay(4);
				}
			}
			if(ChannelValue_s.B.hand_model==TMBA_ONLINE||ChannelValue_s.B.hand_model==TMBB_ONLINE\
					||ChannelValue_s.B.hand_model==EMBA_ONLINE||ChannelValue_s.B.hand_model==EMBB_ONLINE\
					||ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.B.hand_model==JMB_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE
					)
					{
						if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
							{
									Bhandeldisplay(9);//耳膜图片选中
							}
							else
							{
								 Bhandeldisplay(5);//耳膜图片选中
							}
							if(ChannelValue_s.B.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
							{
								clockwisedisplay(0);
								anticlockwisedisplay(0);
								OSCdiplay(2);	//显示默认选中正方向
							}
						
							else{
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//显示默认选中正方向
							}
							
								PAOORMODisplay(0);//隐藏开口定位
								Tooldisplay(0);//隐藏刀具信息
								freqdisplay(1,0);//隐藏频率
							
								Workvalue_s.set_Direction=Direction_forward;//正转
							
							if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.B.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==TMBB_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
							{
								Workvalue_s.set_speed=120000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							
								
					}
					else if(ChannelValue_s.B.hand_model==PXBA_ONLINE||ChannelValue_s.B.hand_model==PXBB_ONLINE||ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
							{
									Bhandeldisplay(7);
									specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
								
									PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
									Tooldisplay(0);//默认弯刨
							}
							else{
								R200_K8_5ON();
								Workvalue_s.Handle_mutual_flag=1;
							Bhandeldisplay(6);//分体图标选中
									AutomaticDisplay(1);
//							PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//							Tooldisplay(3);//默认弯刨
							}
						
							clockwisedisplay(1);
							anticlockwisedisplay(1);
							OSCdiplay(2);	//显示默认选中往复方向
						
							freqdisplay(0,40);//隐藏频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
							
				
					}
					else 
					{
						
							Bhandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
						
					}
					Workvalue_s.hand_model=ChannelValue_s.B.hand_model;
					Workvalue_s.Bchanell_online_flag=start_flag;
					Workvalue_s.select_channel=2;
			break;
		case 5://按钮A手柄
			if(Workvalue_s.select_channel==1||Workvalue_s.select_channel==0||Workvalue_s.Achanell_online_flag==0)return;
			specidisplay(0,0,0,0);//显示刀具信息
			LCD_Disappear_Picture(0x1440);
			if(ChannelValue_s.B.hand_model==TMBA_ONLINE||ChannelValue_s.B.hand_model==TMBB_ONLINE\
				||ChannelValue_s.B.hand_model==EMBA_ONLINE||ChannelValue_s.B.hand_model==EMBB_ONLINE\
				||ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE\
				||ChannelValue_s.B.hand_model==JMB_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE
				)
				{
					if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
					{
						 Bhandeldisplay(8);
					}
					else
					{
					 Bhandeldisplay(2);
					}
				}
				else if(ChannelValue_s.B.hand_model==PXBA_ONLINE||ChannelValue_s.B.hand_model==PXBB_ONLINE)
				{
					 //b_Handel_F_连接;//连接图标
						Bhandeldisplay(3);
				}
				else 
				{
					 //b_Handel_y_连接;//连接图标
						Bhandeldisplay(4);
				}
			
			if(ChannelValue_s.A.hand_model==TMBA_ONLINE||ChannelValue_s.A.hand_model==TMBB_ONLINE\
					||ChannelValue_s.A.hand_model==EMBA_ONLINE||ChannelValue_s.A.hand_model==EMBB_ONLINE\
					||ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.A.hand_model==JMB_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE
					)
					{
							if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
								{
									Ahandeldisplay(9);
								}
								else
									{
									Ahandeldisplay(5);//耳膜图片选中
								}
								if(ChannelValue_s.A.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
							{
								clockwisedisplay(0);
								anticlockwisedisplay(0);
								OSCdiplay(2);	//显示默认选中正方向
							}
						
							else{
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//显示默认选中正方向
							}
							PAOORMODisplay(0);//隐藏开口定位
								Tooldisplay(0);//隐藏刀具信息
								freqdisplay(1,0);//隐藏频率
							
								Workvalue_s.set_Direction=Direction_forward;//正转
							
							if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.A.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==TMBB_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
							{
								Workvalue_s.set_speed=120000;//
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							Workvalue_s.select_channel=1;//选择1通道
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;//赋值工作手柄
								
					}
					else if(ChannelValue_s.A.hand_model==PXBA_ONLINE||ChannelValue_s.A.hand_model==PXBB_ONLINE||ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.A.hand_model==PX_YIP_ONLINE)
							{
								Ahandeldisplay(7);
								specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
								PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
								Tooldisplay(0);//默认弯刨
								clockwisedisplay(1);
								anticlockwisedisplay(1);
								OSCdiplay(2);	//显示默认选中往复方向
															freqdisplay(0,40);//隐藏频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
							
							}
							else{
								R200_K8_2ON();
								Workvalue_s.Handle_mutual_flag=1;
								Workvalue_s.fenti_switch_flag=1;
								
								Ahandeldisplay(6);//分体图标选中
//							PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//							Tooldisplay(3);//默认弯刨
								if(Workvalue_s.fenti_jiyi_flag)
								{
									Workvalue_s.tool_model=1;
									AutomaticAxtion(1);
								}
								else
								{
									Workvalue_s.tool_model=0;
									AutomaticAxtion(0);
								}
									AutomaticDisplay(1);
							}
							
//							clockwisedisplay(1);
//							anticlockwisedisplay(1);
//							OSCdiplay(2);	//显示默认选中往复方向
//						
//							
//							freqdisplay(0,40);//隐藏频率
//							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
//							Workvalue_s.set_speed=5000;//默认往复5000转
//							Workvalue_s.set_Direction=Direction_osc;//反转
//							speeddisplay(1,Workvalue_s.set_speed);
//							Workvalue_s.tool_model=1;
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;
							Workvalue_s.select_channel=1;//选择1通道
				
					}
					else 
					{
							specidisplay(1,paoxueSpeciValue_A[0]*5,paoxueSpeciValue_A[1],paoxueSpeciValue_A[2]);//显示刀具信息
							Ahandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
					
							Workvalue_s.hand_model=ChannelValue_s.A.hand_model;
							Workvalue_s.select_channel=1;//选择1通道
					}
			
		break;
		case 6://插入B手柄
			if(Workvalue_s.select_channel==2||Workvalue_s.select_channel==0||Workvalue_s.Bchanell_online_flag==0)return;
			specidisplay(0,0,0,0);//显示刀具信息
			LCD_Disappear_Picture(0x1440);
		if(ChannelValue_s.A.hand_model==TMBA_ONLINE||ChannelValue_s.A.hand_model==TMBB_ONLINE\
				||ChannelValue_s.A.hand_model==EMBA_ONLINE||ChannelValue_s.A.hand_model==EMBB_ONLINE\
				||ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE\
				||ChannelValue_s.A.hand_model==JMB_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE
				)
				{
					if(ChannelValue_s.A.hand_model==MX_YIM_ONLINE||ChannelValue_s.A.hand_model==MX_YIP_ONLINE||ChannelValue_s.A.hand_model==MX_YIM16_ONLINE)
					{
						Ahandeldisplay(8);
					}
					else
						{
					Ahandeldisplay(2);
					}
				}
				else if(ChannelValue_s.A.hand_model==PXBA_ONLINE||ChannelValue_s.A.hand_model==PXBB_ONLINE)
				{
					 //b_Handel_F_连接;//连接图标
						Ahandeldisplay(3);
				}
				else 
				{
					 //b_Handel_y_连接;//连接图标
						Ahandeldisplay(4);
				}
			
			if(ChannelValue_s.B.hand_model==TMBA_ONLINE||ChannelValue_s.B.hand_model==TMBB_ONLINE\
					||ChannelValue_s.B.hand_model==EMBA_ONLINE||ChannelValue_s.B.hand_model==EMBB_ONLINE\
					||ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE\
					||ChannelValue_s.B.hand_model==JMB_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE
					)
					{
							if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE||ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
								{
								Bhandeldisplay(9);//耳膜图片选中
							}
							else
							{
								Bhandeldisplay(5);//耳膜图片选中
							}
							if(ChannelValue_s.B.hand_model==MX_YIP_ONLINE)//当等于磨削一体刨的时候，只有中间往复方向
							{
								clockwisedisplay(0);
								anticlockwisedisplay(0);
								OSCdiplay(2);	//显示默认选中正方向
							}
						
							else{
								clockwisedisplay(2);
								anticlockwisedisplay(1);
								OSCdiplay(0);	//显示默认选中正方向
							}
							
								PAOORMODisplay(0);//隐藏开口定位
								Tooldisplay(0);//隐藏刀具信息
								freqdisplay(1,0);//隐藏频率
							
								Workvalue_s.set_Direction=Direction_forward;//正转
							
						if(ChannelValue_s.B.hand_model==MX_YIM_ONLINE)
							{
								Workvalue_s.set_speed=80000;//默认80000转，工作转速
								speeddisplay(1,Workvalue_s.set_speed);//但显示8W转
							}
							else if(ChannelValue_s.B.hand_model==JMB_ONLINE)
							{
									Workvalue_s.set_speed=20000;//磨钻通用50000
									speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==TMBB_ONLINE||ChannelValue_s.B.hand_model==MX_YIP_ONLINE)
							{
								Workvalue_s.set_speed=60000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else if(ChannelValue_s.B.hand_model==MX_YIM16_ONLINE)
							{
								Workvalue_s.set_speed=120000;//
								speeddisplay(1,Workvalue_s.set_speed);
							}
							else
							{
								Workvalue_s.set_speed=70000;//磨钻通用50000
								speeddisplay(1,Workvalue_s.set_speed);
							}
							Workvalue_s.select_channel=2;//选择2通道
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;//赋值工作手柄
					}
					else if(ChannelValue_s.B.hand_model==PXBA_ONLINE||ChannelValue_s.B.hand_model==PXBB_ONLINE||ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
					{
						 //b_Handel_F_连接;//连接图标
						if(ChannelValue_s.B.hand_model==PX_YIP_ONLINE)
							{
									Bhandeldisplay(7);
									specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
									PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
									Tooldisplay(0);//默认弯刨
									clockwisedisplay(1);
								anticlockwisedisplay(1);
								OSCdiplay(2);	//显示默认选中往复方向
															freqdisplay(0,40);//隐藏频率
							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
							Workvalue_s.set_speed=5000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_osc;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.tool_model=1;
								
								
							}
							else{
								R200_K8_5ON();
								Workvalue_s.fenti_switch_flag=1;
									Workvalue_s.Handle_mutual_flag=1;
								
									Bhandeldisplay(6);//分体图标选中
//									PAOORMODisplay(2);//打开刀具选择.默认刨刀按钮
//									Tooldisplay(3);//默认弯刨
										if(Workvalue_s.fenti_jiyi_flag)
										{
											Workvalue_s.tool_model=1;
											AutomaticAxtion(1);
										}
										else
										{
											Workvalue_s.tool_model=0;
											AutomaticAxtion(0);
										}
											AutomaticDisplay(1);
							}
					
						
//							clockwisedisplay(1);
//							anticlockwisedisplay(1);
//							OSCdiplay(2);	//显示默认选中往复方向
//						
//							
//							freqdisplay(0,40);//隐藏频率
//							Workvalue_s.set_Freq=40;//传送是4还是40，？？？记得跑一下
//							Workvalue_s.set_speed=5000;//默认往复5000转
//							Workvalue_s.set_Direction=Direction_osc;//反转
//							speeddisplay(1,Workvalue_s.set_speed);
//							Workvalue_s.tool_model=1;
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;
							Workvalue_s.select_channel=2;//选择2通道
				
					}
					else 
					{
						specidisplay(1,paoxueSpeciValue_B[0]*5,paoxueSpeciValue_B[1],paoxueSpeciValue_B[2]);//显示刀具信息
							Bhandeldisplay(7);//一体图标选中//还要跟新值（规格值）
							clockwisedisplay(2);
							anticlockwisedisplay(1);
							OSCdiplay(0);	//显示默认选中正方向
							PAOORMODisplay(0);//打开刀具选择.默认刨刀按钮
							Tooldisplay(0);//默认弯刨
							freqdisplay(1,0);//隐藏频率
							Workvalue_s.set_speed=10000;//默认往复5000转
							Workvalue_s.set_Direction=Direction_forward;//反转
							speeddisplay(1,Workvalue_s.set_speed);
							Workvalue_s.hand_model=ChannelValue_s.B.hand_model;
							Workvalue_s.select_channel=2;//选择2通道
					}
		break;
	}
	
	if(Workvalue_s.set_Way==touchcontrol)return;
	if(Workvalue_s.footcontrol_online_flag)
		{
			Workvalue_s.set_Way=footcontrol;
			if(Workvalue_s.hand_model==PXBA_ONLINE)
				{
					//显示
					ssc_Connectfootpedal(2);
					ssc_Connecthandel(1);
					ssc_Connecttouch(1);
				}
				else
				{
					ssc_Connectfootpedal(2);
					ssc_Connecthandel(0);
					ssc_Connecttouch(1);
				}
		}
		else
		{
			if(Workvalue_s.hand_model==PXBA_ONLINE)
			{
					Workvalue_s.set_Way=handelcontrol;
					ssc_Connectfootpedal(0);
					ssc_Connecthandel(2);
					ssc_Connecttouch(1);
			}
			else
			{
				ssc_Connectfootpedal(0);
				ssc_Connecthandel(0);
				ssc_Connecttouch(1);
				Workvalue_s.set_Way=noControl;
			}
		}
}



void SwitchChanell_A_B_Control(uint8_t key_value)//未运行状态下，可以切换（标志位，脚踏，手控均为启动的状况下）
{
	if(!Workvalue_s.MOTORWorking_flag&&Workvalue_s.Alarm_value==0){
	if(Workvalue_s.Achanell_online_flag&&Workvalue_s.Bchanell_online_flag)//同时在线时候，存在切换变更
	{
		if(key_value==24)//点击A手柄
				{
					SwitchChanelassignment_s(5);
				}
				else
				{
					SwitchChanelassignment_s(6);
				}
			}
	}
}

//拔出手柄控制
void ExtractHandControl(uint8_t key_value)
{
	switch(key_value)
	{
		case 26://拔掉A手柄
			SwitchChanelassignment_s(1);
		break;
			
		case 27://拔出b手柄
			SwitchChanelassignment_s(2);
			break;
	
 }
}
//插入手柄控制
void InsertHandControl(uint8_t key_value)
{
	if(key_value==28)//如果插入A手柄
	{
//		if(Workvalue_s.MOTORWorking_flag==1)
//						{
//							Workvalue_s.Alarm_value=13;
//							Workvalue_s.beep_Alarm_flag=1;
//						
//							return;
//						}          
		SwitchChanelassignment_s(3);
	}
	else 
	{
		SwitchChanelassignment_s(4);
	}
}
//插入或者拔出脚踏
void InsertExtractFootControl(uint8_t key_value)
{
	WayContril(16);
	
}



void KeyBehavior()
{
	if(Workvalue_s.HMI_Control_flag)
	{
		if(Workvalue_s.ScreenKey_data!=43&&Workvalue_s.ScreenKey_data!=1&&Workvalue_s.ScreenKey_data!=3&&Workvalue_s.ScreenKey_data!=24&&Workvalue_s.ScreenKey_data!=25&&Workvalue_s.ScreenKey_data!=11&&\
			Workvalue_s.ScreenKey_data!=12&&Workvalue_s.ScreenKey_data!=7&&Workvalue_s.ScreenKey_data!=5&&Workvalue_s.ScreenKey_data!=26&&Workvalue_s.ScreenKey_data!=27\
		&&Workvalue_s.ScreenKey_data!=28&&Workvalue_s.ScreenKey_data!=29)
			return;
	}
	if(Workvalue_s.set_Way==touchcontrol)
	{
		if(Workvalue_s.ScreenKey_data!=40&&Workvalue_s.ScreenKey_data!=18&&Workvalue_s.ScreenKey_data!=41&&Workvalue_s.ScreenKey_data!=42&&Workvalue_s.ScreenKey_data!=26&&Workvalue_s.ScreenKey_data!=27)
		{
			return;
		}
	}
	if(!Workvalue_s.Achanell_online_flag&&!Workvalue_s.Bchanell_online_flag)
	{
		if(Workvalue_s.Alarm_value!=13&& Workvalue_s.ScreenKey_data!=40&&Workvalue_s.ScreenKey_data!=5&&Workvalue_s.ScreenKey_data!=6&&Workvalue_s.ScreenKey_data!=11&&Workvalue_s.ScreenKey_data!=24&&Workvalue_s.ScreenKey_data!=25&&Workvalue_s.ScreenKey_data!=26&&Workvalue_s.ScreenKey_data!=27&&Workvalue_s.ScreenKey_data!=30&&Workvalue_s.ScreenKey_data!=31)
		return;
	}
	
	if(Workvalue_s.ScreenKey_data==0)return;

	switch(Workvalue_s.ScreenKey_data)
	{
		case 1://速度+（慢加d单点）
			SpeedControl(1);
			KeyBeep_flag=1;
			break;
		case 2://速度++(快加单点)
			SpeedControl(2);
			KeyBeep_flag=1;
			break;
		case 3://速度-（慢减单点）
			SpeedControl(3);
			KeyBeep_flag=1;
			break;
		case 4://速度--（快减单点）
			SpeedControl(4);
			KeyBeep_flag=1;
			break;
		case 5://灌注+
			IrrigateContril(5);
			KeyBeep_flag=1;
			break;
		case 6://灌注-
			IrrigateContril(6);
			KeyBeep_flag=1;
			break;
		case 7://注水+
			InjectionContril(7);
			break;
		case 8://注水-
			InjectionContril(8);
			break;
		case 9://频率+
			FreqContril(9);
			break;
		case 10://频率-
			FreqContril(10);
			break;
		case 11://灌注启动开关
			Irrigatesign();
	
			if(Workvalue_s.HMI_Injection_stop_flag)
			{
					Workvalue_s.HMI_Injection_stop_flag=0;
				if(Workvalue_s.Injection_drain_flag)
				{
					Workvalue_s.ScreenKey_data=12;
				}
			}
			else{
				break;
			}
		case 12://注水排空开关
			Injectionsign();
			break;
		case 13://正转按钮
			DirectionControl(13);
		
			break;
		case 14://反转按钮
			DirectionControl(14);
		 break;
		case 15://往复按钮
			DirectionControl(15);
			break;
		case 16://脚控按钮
			WayContril(16);
			break;
		case 17://手控按钮
			WayContril(17);
			break;
		case 18://触控按钮
				WayContril(18);
			Workvalue_s.TouchActivation_flag=start_flag;
			break;
		case 19://手动识别开关,海外机型不适用
			
			break;
		case 20://手动识别（磨头）
			MotoPaoDaoSelectControl(20);
			break;
		case 21://手动识别（刨刀）
			MotoPaoDaoSelectControl(21);
			break;
		case 22://开口位置（顺时针）
			OpenPositionControl(22);
		break;
		case 23://开口位置（逆时针）
		OpenPositionControl(23);
		break;
		case 24://A手柄切换按键
			SwitchChanell_A_B_Control(24);
		break;
		case 25://B手柄切换按键
			SwitchChanell_A_B_Control(25);
		break ;
		case 26://拔出A手柄
			ExtractHandControl(26);
				break ;
		case 27://拔出B手柄
			ExtractHandControl(27);
				break ;
		case 28://插入A手柄
		//	InsertHandControl(28);
				break ;
		case 29://插入B手柄
		//	InsertHandControl(29);
//			if(Workvalue_s.Handle_mutual_flag){
//				Workvalue_s.ScreenKey_data=28;
//				Workvalue_s.Handle_mutual_flag=0;
//			}
			
		break;
		case 30://脚踏插入
			InsertExtractFootControl(30);
			break;
		case 31://脚踏拔出
			InsertExtractFootControl(31);
			break;
		case 32://慢加长按
			break;
		case 33://块加长按
			break;
		case 34://慢减长按
			break;
		case 35://块减长按
			break;
		case 36://自动识别按钮开关
			//待写入
		if(Workvalue_s.MOTORWorking_flag)return;
		if(Workvalue_s.hand_model==PXBA_ONLINE||Workvalue_s.hand_model==PXBB_ONLINE)
			{
					Workvalue_s.Handle_mutual_flag?(Workvalue_s.Handle_mutual_flag=stop_flag):(Workvalue_s.Handle_mutual_flag=start_flag);
					AutomaticDisplay(Workvalue_s.Handle_mutual_flag);
			}
		break;
		case 40://退出紧急框
				WayContril(40);
		Workvalue_s.TouchActivation_flag=stop_flag;
			break;
		case 41://触控启动
			if(Workvalue_s.TouchActivation_flag){
		//	Workvalue_s.TouchActivation_flag=start_flag;
				Workvalue_s.MOTORWorking_flag=start_flag;
				KeyBeep_flag=1;
			}
			break;
		case 42://触控停止
			Workvalue_s.MOTORWorking_flag=stop_flag;
				KeyBeep_flag=1;
		if(Workvalue_s.Alarm_value==8||Workvalue_s.Alarm_value==9||Workvalue_s.Alarm_value==13)
		{
			Workvalue_s.Alarm_value=0;
			Workvalue_s.beep_Alarm_flag=0;
		}
		break;
		case 43:
			if(Workvalue_s.HMI_Control_flag)
				{
						KeyBeep_flag=1;
					LCD_Disappear_Picture(0x1450);
					Workvalue_s.HMI_Control_flag=0;
					Workvalue_s.HMI_Working_flag=0;
					if(Workvalue_s.Irrigate_start_flag)
					{
						Workvalue_s.ScreenKey_data=11;//失去之前停掉，灌注
					}
					if(Workvalue_s.beep_Alarm_flag)
					{
						Workvalue_s.beep_Alarm_flag=0;
						Workvalue_s.Alarm_value=0;
						Workvalue_s.HMI_Working_flag=0;
					}
				}
			break;
	}
}




//10ms,轮训一次
void BeepControl()
{
	static uint8_t times=0;
	static uint16_t Alarmtimes=0;
	if(KeyBeep_flag&&!Workvalue_s.beep_Alarm_flag)
	{
		 BEEP_ON();
		times++;
		//蜂鸣器响，
		if(times>beep_time)//考虑连续按只是相应一下
		{
			//停止蜂鸣器响
			times=0;
			KeyBeep_flag=0;
			BEEP_OFF();
		}
	}
	if(Workvalue_s.beep_Alarm_flag)
	{
		ALARMdisplay();
		Alarmtimes++;
		if(Alarmtimes<=10) BEEP_ON();
		
		else if(Alarmtimes>10&&Alarmtimes<=20)BEEP_OFF();
		
		else if(Alarmtimes>20&&Alarmtimes<=30) BEEP_ON();
		
		else if (Alarmtimes>30&&Alarmtimes<=40)BEEP_OFF();
		else if(Alarmtimes>40&&Alarmtimes<=50) BEEP_ON();
		
		else if (Alarmtimes>50&&Alarmtimes<=60)BEEP_OFF();
		
		if(Alarmtimes>80)Alarmtimes=0;
		
		
	}
	else 
	{
		if(!KeyBeep_flag)BEEP_OFF();
			Alarmtimes=0;
		//停止蜂鸣器响
	
			ALARMdisplay();
	}
		
	
}


void BeepControlTask(uint32_t event)
{
	(void)event;
	BeepControl();
}

void  BeepControlTask_Init(void)
{
	Kernel_TaskCreate(&BeepHandle, BeepControlTask);
	Kernel_TaskStart(&BeepHandle, KERNEL_TASK_ALWAYS, 10);
}

//3按键按下，触发相应的功能（这里需要区分，机型）,50ms线程,有限制（比如报警状态下无效，在无手柄接入的状态下无效）,放在脚踏连接里面？？？
void FootKeyControl_s()
{
//	Workvalue_s.Foot_Key_value=2;
	if(Workvalue_s.Alarm_value==0){
		
		if(Workvalue_s.Foot_Key_value==3)
			{
					Workvalue_s.FastGear_flag=1;//
					Workvalue_s.ScreenKey_data=5;
			}
			else if(Workvalue_s.Foot_Key_value==2)
			{
				Workvalue_s.ScreenKey_data=11;
			}
			else 
			{
				if(Workvalue_s.Achanell_online_flag||Workvalue_s.Bchanell_online_flag){
				if(Workvalue_s.Foot_Key_value==1)
				{
						Workvalue_s.FastGear_flag=1;
						Workvalue_s.ScreenKey_data=7;
				}
				else if(Workvalue_s.Foot_Key_value==5)
				{
							Workvalue_s.ScreenKey_data=22;//顺时针
				}
				else if(Workvalue_s.Foot_Key_value==6)
				{
					
					if(Workvalue_s.select_channel==2)
					{
						if(Workvalue_s.Foot_type==1)
							Workvalue_s.ScreenKey_data=24;
					}
					else if(Workvalue_s.select_channel==1)
					{
							if(Workvalue_s.Foot_type==1)
							Workvalue_s.ScreenKey_data=25;
					}
				}
			}
		}
//		if(Workvalue_s.Achanell_online_flag||Workvalue_s.Bchanell_online_flag){
//			switch(Workvalue_s.Foot_Key_value)
//			{
//				case 1://左键短按
//					Workvalue_s.FastGear_flag=1;
//						Workvalue_s.ScreenKey_data=7;
//				//	InjectionContril(7);//注水档位切换
//					break;
//				case 2://左键长按
//					break;
//				case 3://右键短按
//					Workvalue_s.FastGear_flag=1;//
//					Workvalue_s.ScreenKey_data=5;
////				//SpeedControl(3);//写入慢减流程
////				IrrigateContril(5);//灌注档位
//				
//				break;
//				case 4://右键长按
//					// Irrigatesign();//长按灌注
//				break;
//				case 5://中间短按
//					Workvalue_s.ScreenKey_data=22;//顺时针
//					break;
//				case 6://中间长按
//					if(Workvalue_s.select_channel==2)
//					{
//							Workvalue_s.ScreenKey_data=24;
//					
//					}
//					else if(Workvalue_s.select_channel==1)
//					{
//							Workvalue_s.ScreenKey_data=25;
//					
//					}
//					break;
//					
//			}
//			
//		}
	}
	Workvalue_s.Foot_Key_value=0;
}

static void PUMPBehavior()
{
	  uint32_t temp1 = 0;
			
		#if water_uptake
		temp1=(uint32_t )(Workvalue_s.set_Irrigate*42);//注水
		#else
		if(Workvalue_s.set_Irrigate>210)
		{
			temp1=(uint32_t )(Workvalue_s.set_Irrigate*0.62);//灌注
		}
		else
		temp1=(uint32_t )(Workvalue_s.set_Irrigate*0.55);//灌注
		#endif
		if(Workvalue_s.Irrigate_start_flag){
		PumpScreenDebugPoint(201U, Workvalue_s.set_Irrigate, (uint8_t)(temp1 > 255U ? 255U : temp1));
		Pump_SetSpeed_B(temp1); 
		}
		else
		{
				PumpScreenDebugPoint(202U, Workvalue_s.set_Irrigate, 0U);
				Pump_SetSpeed_B(0); 
		}
			
		
}


void FootKeyTask(uint32_t event)
{
	(void)event;
	if(Workvalue_s.HMI_Control_flag)
		return;
	FootKeyControl_s();
}
void FootKeyTask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	Kernel_TaskCreate(&FootKeyHandle, FootKeyTask);
	Kernel_TaskStart(&FootKeyHandle, KERNEL_TASK_ALWAYS, 30);
}


void KeyBehaviorTask(uint32_t event)
{
	(void)event;
	
	KeyBehavior();
	Workvalue_s.ScreenKey_data=0;
}
void ScreenKeyTask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	Kernel_TaskCreate(&KeyBehaviorHandle, KeyBehaviorTask);
	Kernel_TaskStart(&KeyBehaviorHandle, KERNEL_TASK_ALWAYS, 30);
}

void PUMPBBehaviorTask(uint32_t event)
{
	(void)event;
	
	PUMPBehavior();
	
}
void PUMPBTask_Init(void)
{
  /* definition and creation of HANDLEKEYTask */
	Kernel_TaskCreate(&PUMPBBehaviorHandle, PUMPBBehaviorTask);
	Kernel_TaskStart(&PUMPBBehaviorHandle, KERNEL_TASK_ALWAYS, 100);
}


//============================================================================
// 函数名称: NOHandelUI() 无手柄界面
// 功能描述: 密码输入显示
// 输　  入: 
// 输    出: 无
// 函数说明: SSC界面处理((无手柄处理界面)
//============================================================================
void NOHandelUI()
{
			LCD_Show_Picture(A_Handel_adrr,A_Handel_NO);
			LCD_Show_Picture(B_Handel_adrr,A_Handel_NO);
			LCD_Disappear_Number(Text_Spec_addr);//隐藏刀具规格信息
			LCD_Disappear_Number(Speed_data_addr);//隐藏速度数据
			LCD_Disappear_Number(WaterIrrigate_data_addr);//隐藏灌水数据
			LCD_Disappear_Number(WaterInjection_data_addr);//隐藏注水数据
			LCD_Disappear_Number(Frequency_data_addr);//隐藏频率数据
			LCD_Disappear_Picture(ManualRecognition_addr);
			LCD_Disappear_Picture(ModeSlect_M_P_addr);
			LCD_Disappear_Picture(OpeningPosition_addr);
			LCD_Disappear_Picture(Frequency_addr);
			LCD_Show_Picture(WaterIrrigate_addr,WaterIrrigate_close);//灌注暗黑
			LCD_Show_Picture(WaterIrrigate_drain_addr,WaterIrrigate_drain_close);//灌注按钮暗黑
			LCD_Show_Picture(WaterInjection_addr,WaterInjection_close);//注水暗黑
			LCD_Show_Picture(WaterInjection_drain_addr,WaterInjection_drain_close);//注水暗黑
			LCD_Show_Picture(Speed_addr,Speed_lazy);//速度区域暗黑
			LCD_Show_Picture(Sports_mode_addr,Sports_mode_NO);//运动模式区域暗黑
}


 




 
//============================================================================
// 函数名称: LCD_Show_Password_Input() 
// 功能描述: 密码输入显示
// 输　  入: Position：当前输入的密码位置
// 输    出: 无
// 函数说明: SSC界面处理(开机无图标处理，这里先初始化赋值)
//============================================================================
 
void SscDisplayInit()
{
	UIStateControl_s.A_Handel_mode=A_Handel_NO;                 //无图标
	UIStateControl_s.A_Handel_mode=B_Handel_NO;                 //无图标
	UIStateControl_s.control_Slect_mode=Control_model_NO;				//控制模式无
	UIStateControl_s.Handel_switch_flag=1;											//手柄切换flag,第一次置为1
	UIStateControl_s.manualRecognition_flag=0;									//不打开，默认自动识别
	UIStateControl_s.mode_M_P_Slect_mode=ModeSlect_M_select;		//默认是磨头
	UIStateControl_s.sports_Slect_mode=Sports_mode_NO;          //默认是无运动模式
	UIStateControl_s.waterInjection_flag=0;                     //灌注关
	UIStateControl_s.waterInjection_drain_flag=0;               //按钮暗黑
	UIStateControl_s.waterIrrigate_flag=0;                      //注水关
	UIStateControl_s.waterIrrigate_drain_flag=0;                //按钮暗黑
	UIStateControl_s.HandleselectionChannel=0;                  //0无手柄选中，1A通道选中，2B通道选中
}


//============================================================================
// 函数名称: UIGetKey()
// 功能描述: 数据显示管理 
// 输　  入: Position：insert_flag是否插入手柄，extract_flag是否拔出手柄，key_value触摸按键值
// 输    出: 无
// 函数说明: 1_管理员模式界面
//============================================================================
void UIGetKey( uint8_t insert_flag, uint8_t extract_flag,uint8_t key_value)
{
	if(insert_flag)
	{
		UIStateControl_s.Handel_switch_flag=1;
		insert_flag=0;
		if(UIControlState_s.HandleselectionChannel==HandleChannel_A)
		{
			if(UIControlState_s.B_ControlState.HandleChannel_state==Handle_select)
				{
					UIControlState_s.B_ControlState.HandleChannel_state=Handle_connect;
					if(UIControlState_s.B_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.B_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.B_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.B_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_M_connect;
					}
					else if(UIControlState_s.B_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==PXBB_ONLINE)
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_F_connect;
					}
					else
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_Y_connect;
					}
				}
				
				UIControlState_s.A_ControlState.HandleChannel_state=Handle_select;
				if(UIControlState_s.A_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.A_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.A_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.A_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_M_selected;
						switch(UIControlState_s.A_ControlState.OnlineModel){
							case TMBA_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.tmba_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.tmba_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
							case TMBB_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.tmbb_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.tmbb_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
							case EMBA_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.emba_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.emba_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case EMBB_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.embb_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.embb_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case MX_YIM_ONLINE:
									UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.mx_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.mx_ytm_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
								break;
								case MX_YIP_ONLINE:
									UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.mx_pump;
									UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.mx_ytp_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_reverse_select_M_W;//只有往复
							break;
								default:
									break;
						}
					}
					else if(UIControlState_s.A_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==PXBB_ONLINE)//刚插入默认往复，在改函数后面，进行持续采集刀具信息（rfid信息，芯片储存信息）
					{
					
						UIShow_s.A_Show.Handel_icon=A_Handel_F_selected;
						UIDataShow_s.A_DataShow.set_pump_data=30;
						UIDataShow_s.A_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.A_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.A_Show.manualRecognition_icon=ManualRecognition_close;
						UIShow_s.A_Show.mode_M_P_Slect_icon=ModeSlect_P_select;
						UIShow_s.A_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
					else
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_Y_selected;
						UIDataShow_s.A_DataShow.set_pump_data=30;
						UIDataShow_s.A_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.A_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.A_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
			}
			else
			{
				if(UIControlState_s.A_ControlState.HandleChannel_state==Handle_select)
				{
					UIControlState_s.A_ControlState.HandleChannel_state=Handle_connect;
					if(UIControlState_s.A_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.A_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.A_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.A_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_M_connect;
					}
					else if(UIControlState_s.A_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==PXBB_ONLINE)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_F_connect;
					}
					else
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_Y_connect;
					}
				}
				
				UIControlState_s.B_ControlState.HandleChannel_state=Handle_select;
				if(UIControlState_s.B_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.B_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.B_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.B_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_M_selected;
						switch(UIControlState_s.A_ControlState.OnlineModel){
							case TMBA_ONLINE:
								UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.tmba_pump;
								UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.tmba_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
							case TMBB_ONLINE:
								UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.tmbb_pump;
								UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.tmbb_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
							case EMBA_ONLINE:
								UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.emba_pump;
								UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.emba_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case EMBB_ONLINE:
								UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.embb_pump;
								UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.embb_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case MX_YIM_ONLINE:
									UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.mx_pump;
								UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.mx_ytm_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
								break;
								case MX_YIP_ONLINE:
									UIDataShow_s.B_DataShow.set_pump_data=UIDataShow_s.mx_pump;
									UIDataShow_s.B_DataShow.set_speed_data=UIDataShow_s.mx_ytp_speed;
								UIShow_s.B_Show.sports_icon=Sports_mode_reverse_select_M_W;//只有往复
							break;
								default:
									break;
						}
					}
					else if(UIControlState_s.B_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.B_ControlState.OnlineModel==PXBB_ONLINE)//刚插入默认往复，在改函数后面，进行持续采集刀具信息（rfid信息，芯片储存信息）
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_F_selected;
						UIDataShow_s.B_DataShow.set_pump_data=30;
						UIDataShow_s.B_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.B_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.B_Show.manualRecognition_icon=ManualRecognition_close;
						UIShow_s.B_Show.mode_M_P_Slect_icon=ModeSlect_P_select;
						UIShow_s.B_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
					else
					{
						UIShow_s.B_Show.Handel_icon=B_Handel_Y_selected;
						UIDataShow_s.B_DataShow.set_pump_data=30;
						UIDataShow_s.B_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.B_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.B_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
			}
		}
	if(extract_flag)//拔出手柄
	{
		UIStateControl_s.Handel_switch_flag=1;
		if(UIControlState_s.HandleselectionChannel==HandleChannel_A)//拔出之后，通道A选中
		{
			//b通道无手柄显示
			UIShow_s.B_Show.Handel_icon=B_Handel_NO;
			if(UIControlState_s.A_ControlState.HandleChannel_state==Handle_select)
			{
				//本来选择的数据不搜影响，本来就是选中的A手柄
			}
			else
			{
					//如果是A手柄已连接转化为选中
					if(UIControlState_s.A_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.A_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.A_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.A_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_M_selected;
					}
					else if(UIControlState_s.A_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==PXBB_ONLINE)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_F_selected;
					}
					else
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_Y_selected;
					}
			 }
		}
		else if(UIControlState_s.HandleselectionChannel==HandleChannel_B)
		{
			//b通道无手柄显示
				UIShow_s.A_Show.Handel_icon=A_Handel_NO;
			if(UIControlState_s.B_ControlState.HandleChannel_state==Handle_select)//本来选择的数据不搜影响
			{
				
			}
		}
		else if(UIControlState_s.HandleselectionChannel==HandleChannel_NO)
		{
			UIShow_s.B_Show.Handel_icon=B_Handel_NO;
			UIShow_s.A_Show.Handel_icon=A_Handel_NO;
		}
	}
	switch(key_value)
	{
		case 1://A通道按钮
			
			UIControlState_s.HandleselectionChannel=HandleChannel_A;
			if(UIControlState_s.A_ControlState.HandleChannel_state==Handle_connect)
			{
				UIControlState_s.A_ControlState.HandleChannel_state=Handle_select;
				if(UIControlState_s.A_ControlState.OnlineModel==TMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==TMBB_ONLINE\
						||UIControlState_s.A_ControlState.OnlineModel==EMBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==EMBB_ONLINE\
					||UIControlState_s.A_ControlState.OnlineModel==MX_YIM_ONLINE||UIControlState_s.A_ControlState.OnlineModel==MX_YIP_ONLINE
					)
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_M_selected;
						switch(UIControlState_s.A_ControlState.OnlineModel){
							case TMBA_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.A_DataShow.min_pump_data;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.A_DataShow.set_speed_data;
							 // UIControlState_s.A_ControlState.ControlModel
							
							break;
							case TMBB_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.tmbb_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.tmbb_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
							case EMBA_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.emba_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.emba_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case EMBB_ONLINE:
								UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.embb_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.embb_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
							break;
								case MX_YIM_ONLINE:
									UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.mx_pump;
								UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.mx_ytm_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_forward_select_p_M_M;//只有正，反
								break;
								case MX_YIP_ONLINE:
									UIDataShow_s.A_DataShow.set_pump_data=UIDataShow_s.mx_pump;
									UIDataShow_s.A_DataShow.set_speed_data=UIDataShow_s.mx_ytp_speed;
								UIShow_s.A_Show.sports_icon=Sports_mode_reverse_select_M_W;//只有往复
							break;
								default:
									break;
						}
					}
					else if(UIControlState_s.A_ControlState.OnlineModel==PXBA_ONLINE||UIControlState_s.A_ControlState.OnlineModel==PXBB_ONLINE)//刚插入默认往复，在改函数后面，进行持续采集刀具信息（rfid信息，芯片储存信息）
					{
					
						UIShow_s.A_Show.Handel_icon=A_Handel_F_selected;
						UIDataShow_s.A_DataShow.set_pump_data=30;
						UIDataShow_s.A_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.A_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.A_Show.manualRecognition_icon=ManualRecognition_close;
						UIShow_s.A_Show.mode_M_P_Slect_icon=ModeSlect_P_select;
						UIShow_s.A_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
					else
					{
						UIShow_s.A_Show.Handel_icon=A_Handel_Y_selected;
						UIDataShow_s.A_DataShow.set_pump_data=30;
						UIDataShow_s.A_DataShow.set_speed_data=5000;//速度和
						UIDataShow_s.A_DataShow.set_freq_data=4;//默认最大4hz
						UIShow_s.A_Show.sports_icon=Sports_mode_osc_select_p_P;
					}
			}
			case 5://灌注按钮（流量值在运行时候变黄，停止时候变白），先不管
			if(UIControlState_s.IrrigateSwitch_flag)
			{
				UIControlState_s.IrrigateSwitch_flag=0;
				UIShow_s.Irrigatepump_button_icon=WaterIrrigate_drain_stop;
			}
			else
			{
				UIControlState_s.IrrigateSwitch_flag=1;
				UIShow_s.Irrigatepump_button_icon=WaterIrrigate_drain_start;
			}
		break;
	}
	
	
	
//	switch(key_value)
//	{
//		case 1:
//		UIdata_s.SetDATA_A.waterInjection_data=UIDataShow_s.A_DataShow.set_pump_data;//
//		
//	}
}

 
//============================================================================
// 函数名称: SscDisplayManage()
// 功能描述: 显示管理 
// 输　  入: Position：
// 输    出: 无
// 函数说明: 1_管理员模式界面
//============================================================================
void SscDisplayManage()
{
	uint8_t ui_interOperation_sign=0;//内操作
	static uint8_t handChannel_compare=0;
	if(UIStateControl_s.Handel_switch_flag)//总的切换时候
	{
		UIStateControl_s.Handel_switch_flag=0;
		switch(UIStateControl_s.HandleselectionChannel)
			{
			case 0:
				
				NOHandelUI();
				handChannel_compare=0;
				break;
			case 1:
				if(!handChannel_compare)
				{
						LCD_Show_Picture(WaterIrrigate_addr,WaterIrrigate_start);//灌注激活
						LCD_Show_Picture(WaterIrrigate_drain_addr,WaterIrrigate_drain_stop);//按钮停止
						LCD_Show_Number(WaterIrrigate_data_addr,UIdata_s.SetDATA_A.waterIrrigate_data);
					
						LCD_Show_Picture(WaterInjection_addr,WaterInjection_start);//注水激活
						LCD_Show_Picture(WaterInjection_drain_addr,WaterInjection_drain_stop);//排空按钮停止
						LCD_Show_Number(WaterInjection_data_addr,UIdata_s.SetDATA_A.waterInjection_data);
					
						LCD_Show_Picture(Speed_addr,Speed_activation);//速度激活
						LCD_Show_Number(Speed_data_addr,UIdata_s.SetDATA_A.Speed_data);//速度数据初始化（设置）
				}
				LCD_Show_Picture(A_Handel_adrr,UIShow_s.A_Show.Handel_icon);
				LCD_Show_Picture(B_Handel_adrr,UIShow_s.B_Show.Handel_icon);
				
				if(UIShow_s.A_Show.Handel_icon==A_Handel_F_selected)//如果等于分体式选中
				{
					LCD_Show_Picture(A_Handel_adrr,UIShow_s.A_Show.manualRecognition_icon);
				
					if(UIShow_s.A_Show.manualRecognition_icon==ManualRecognition_start)//手动模式开启
					{
						if(UIShow_s.A_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
						{
							LCD_Show_Picture(ModeSlect_M_P_addr,ModeSlect_M_select);
							LCD_Disappear_Picture(OpeningPosition_addr);
						}
						else
						{
							LCD_Show_Picture(OpeningPosition_addr,OpeningPosition_icon);//
							LCD_Show_Picture(ModeSlect_M_P_addr,ModeSlect_P_select);
						}
					}
					else
					{
						LCD_Disappear_Picture(ModeSlect_M_P_addr);
						if(UIShow_s.A_Show.mode_M_P_Slect_icon==ModeSlect_M_select)//刀具读取？？？
								LCD_Disappear_Picture(OpeningPosition_addr);
						else
						LCD_Show_Picture(OpeningPosition_addr,OpeningPosition_icon);//后面识别？？？刀具具体判定
					}
				}
				else
				{
					LCD_Disappear_Picture(ModeSlect_M_P_addr);
				}
				
				LCD_Show_Picture(Sports_mode_addr,UIShow_s.A_Show.sports_icon);//A区运动模式
				if(UIShow_s.A_Show.sports_icon==Sports_mode_osc_select_p_P)
				{
					LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_A.Frequency_data);//显示频率数据(值来自于)
					LCD_Show_Picture(Frequency_addr,UIShow_s.A_Show.frequency_icon);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）
				}
				else
				{
					if(handChannel_compare)
						{
							LCD_Disappear_Number(Frequency_data_addr);//显示频率数据(值来自于)
							LCD_Disappear_Picture(Frequency_addr);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）
						}
				}
				handChannel_compare=1;
				break;
			case 2:
				if(!handChannel_compare)
				{
						LCD_Show_Picture(WaterIrrigate_addr,WaterIrrigate_start);//灌注激活
						LCD_Show_Picture(WaterIrrigate_drain_addr,WaterIrrigate_drain_stop);//按钮停止
						LCD_Show_Number(WaterIrrigate_data_addr,UIdata_s.SetDATA_B.waterInjection_data);
					
					
						LCD_Show_Picture(WaterInjection_addr,WaterInjection_start);//注水激活
						LCD_Show_Picture(WaterInjection_drain_addr,WaterInjection_drain_stop);//排空按钮停止
						LCD_Show_Number(WaterInjection_data_addr,UIdata_s.SetDATA_B.waterInjection_data);
					
						LCD_Show_Picture(Speed_addr,Speed_activation);//速度区域暗黑
						LCD_Show_Number(Speed_data_addr,UIdata_s.SetDATA_B.Speed_data);//速度数据初始化（设置）
				}
				LCD_Show_Picture(A_Handel_adrr,UIShow_s.A_Show.Handel_icon);
				LCD_Show_Picture(B_Handel_adrr,UIShow_s.B_Show.Handel_icon);
				if(UIShow_s.B_Show.Handel_icon==B_Handel_F_selected)//如果等于分体式选中
				{
					LCD_Show_Picture(B_Handel_adrr,UIShow_s.B_Show.manualRecognition_icon);
					if(UIShow_s.B_Show.manualRecognition_icon==ManualRecognition_start)
					{
						if(UIShow_s.B_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
						{
							LCD_Show_Picture(ModeSlect_M_P_addr,ModeSlect_M_select);
							LCD_Disappear_Picture(OpeningPosition_addr);
						}
						else
						{
							LCD_Show_Picture(OpeningPosition_addr,OpeningPosition_icon);//
							LCD_Show_Picture(ModeSlect_M_P_addr,ModeSlect_P_select);
						}
					}
					else
					{
						LCD_Disappear_Picture(ModeSlect_M_P_addr);
						if(UIShow_s.B_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
								LCD_Disappear_Picture(OpeningPosition_addr);
						else
						LCD_Show_Picture(OpeningPosition_addr,OpeningPosition_icon);
				
					}
				}
				else
				{
					LCD_Disappear_Picture(ModeSlect_M_P_addr);
				}
				LCD_Show_Picture(Sports_mode_addr,UIShow_s.B_Show.sports_icon);//B区运动模式
				if(UIShow_s.B_Show.sports_icon==Sports_mode_osc_select_p_P)
				{
					LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_B.Frequency_data);//显示频率数据(值来自于)
					LCD_Show_Picture(Frequency_addr,UIShow_s.B_Show.frequency_icon);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）
				}
				else
				{
					if(handChannel_compare)
						{
							LCD_Disappear_Number(Frequency_data_addr);//显示频率数据(值来自于)
							LCD_Disappear_Picture(Frequency_addr);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）  
				}
				handChannel_compare=2;
			break;
			default:
				break;
			}
  }
}
	else//2025.4.30任务当不切换手柄时候界面内调整？？？
	{
		//当不是切换手柄时，而是在当前选中模式界面操作时候
		switch(ui_interOperation_sign)
				{
					case 1:                           //排空按钮触发
						LCD_Show_Picture(WaterInjection_drain_addr,UIShow_s.Injectionpump_button_icon);//
						ui_interOperation_sign=0;
						break;
					case 2:                           //灌水开关触发
						LCD_Show_Picture(WaterIrrigate_drain_addr,UIShow_s.Irrigatepump_button_icon);
						ui_interOperation_sign=0;
						break;
					case 3:                           //控制区域按钮触发（脚控，手控，触控）
						UIStateControl_s.HandleselectionChannel==1?LCD_Show_Picture(Control_model_addr,UIShow_s.A_Show.control_model_icon):    \
						                                           LCD_Show_Picture(Control_model_addr,UIShow_s.B_Show.control_model_icon);		
						ui_interOperation_sign=0;					
					break;
					case 4:                           //运动方向区域按钮触发（正，反，往复）
						ui_interOperation_sign=0;
						if(UIStateControl_s.HandleselectionChannel==1)//A通道
						{
							LCD_Show_Picture(Sports_mode_addr,UIShow_s.A_Show.sports_icon);   
							if(UIShow_s.A_Show.sports_icon==Sports_mode_osc_select_p_P)//如果等于往复模式
							{
									LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_A.Frequency_data);//显示频率数据(值来自于)
									LCD_Show_Picture(Frequency_addr,UIShow_s.A_Show.frequency_icon);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）
							}
							else
							{
								LCD_Disappear_Number(Frequency_data_addr);//频率数据(值来自于)
								LCD_Disappear_Picture(Frequency_addr);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）  
							}
							LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_A.Speed_data);//速度值
						}	
						else ////B通道
						{
							LCD_Show_Picture(Sports_mode_addr,UIShow_s.B_Show.sports_icon);
							if(UIShow_s.B_Show.sports_icon==Sports_mode_osc_select_p_P)//如果等于往复模式
							{
									LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_B.Frequency_data);//显示频率数据(值来自于)
									LCD_Show_Picture(Frequency_addr,UIShow_s.B_Show.frequency_icon);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏）
							}
							else
							{
								LCD_Disappear_Number(Frequency_data_addr);//显示频率数据(值来自于)
								LCD_Disappear_Picture(Frequency_addr);//频率比较特殊（如果刨削功能往复模式，会弹出这个地方应该判断，直接隐藏） 
							}
							LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_B.Speed_data);//速度值								
						}
						break;                          
					case 5:                           //自动识别选择
						if(UIStateControl_s.HandleselectionChannel==1)
							{
								
								LCD_Show_Picture(ManualRecognition_addr,UIShow_s.A_Show.manualRecognition_icon);
								if(UIShow_s.A_Show.manualRecognition_icon==ManualRecognition_start)//如果手动模式开启
								{
									LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_A.Speed_data);//速度值	
									UIdata_s.RFID_flag=0;//关闭自动识别，关闭是否清除自动识别的内容（或者是在规格地方显示刀具图片，直刨图标，弯刨图标）？？？
									LCD_Show_Picture(ModeSlect_M_P_addr,UIShow_s.A_Show.mode_M_P_Slect_icon);//弹出磨头刨刀选择按钮
									if(UIShow_s.A_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
										{
												LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
												LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
												
										}
										else
										{
											if(UIShow_s.A_Show.sports_icon==Sports_mode_osc_select_p_P)
												{
													LCD_Show_Picture(Frequency_addr,UIShow_s.A_Show.frequency_icon);//***************如果是往复模式显示4月30号
													LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_A.Frequency_data);//显示频率数据(值来自于)
												}
												else
												{
													LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
													LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
												}
										}
								}
								else
								{
										LCD_Disappear_Picture(ModeSlect_M_P_addr);//
										UIdata_s.RFID_flag=1;//开启自动识别，可以从图片界面显示分离出去，单独开一个线程把自动识别功能添加
										LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_A.Speed_data);//速度值，记忆上一次的速度显示，当刀具发生改变的时候则跟新
										if(UIShow_s.A_Show.sports_icon==Sports_mode_osc_select_p_P)
										{
										  	LCD_Show_Picture(Frequency_addr,UIShow_s.A_Show.frequency_icon);//***************如果是往复模式显示4月30号
												LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_A.Frequency_data);//显示频率数据(值来自于)
										}
										else
										{
											LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
											LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
										}
								}
							
							}
						else
							{
								
								LCD_Show_Picture(ManualRecognition_addr,UIShow_s.B_Show.manualRecognition_icon);
								if(UIShow_s.B_Show.manualRecognition_icon==ManualRecognition_start)//如果手动模式开启
								{
									LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_B.Speed_data);//速度值	
									UIdata_s.RFID_flag=0;//关闭自动识别，关闭是否清除自动识别的内容（或者是在规格地方显示刀具图片，直刨图标，弯刨图标）？？？
									LCD_Show_Picture(ModeSlect_M_P_addr,UIShow_s.B_Show.mode_M_P_Slect_icon);//弹出磨头刨刀选择按钮
									if(UIShow_s.B_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
										{
												LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
												LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
												
										}
										else
										{
											if(UIShow_s.B_Show.sports_icon==Sports_mode_osc_select_p_P)
												{
													LCD_Show_Picture(Frequency_addr,UIShow_s.B_Show.frequency_icon);//***************如果是往复模式显示4月30号
													LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_B.Frequency_data);//显示频率数据(值来自于)
												}
												else
												{
													LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
													LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
												}
										}
								}
								else
								{
										LCD_Disappear_Picture(ModeSlect_M_P_addr);//
										UIdata_s.RFID_flag=1;//开启自动识别，可以从图片界面显示分离出去，单独开一个线程把自动识别功能添加
										LCD_Show_Picture(Speed_data_addr,UIdata_s.SetDATA_B.Speed_data);//速度值，记忆上一次的速度显示，当刀具发生改变的时候则跟新
										if(UIShow_s.B_Show.sports_icon==Sports_mode_osc_select_p_P)
										{
										  	LCD_Show_Picture(Frequency_addr,UIShow_s.B_Show.frequency_icon);//***************如果是往复模式显示4月30号
												LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_B.Frequency_data);//显示频率数据(值来自于)
											
										}
										else
										{
											LCD_Disappear_Number(Frequency_data_addr);//如果是单向，隐藏评率数字
											LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
										}
								}
							}
					break;
					case 6:														//模式选择
						if(UIStateControl_s.Handel_switch_flag==1)
						{
							LCD_Show_Picture(ModeSlect_M_P_addr,UIShow_s.A_Show.mode_M_P_Slect_icon);
							if(UIShow_s.A_Show.mode_M_P_Slect_icon==ModeSlect_M_select)
							{
								LCD_Disappear_Number(Frequency_data_addr);//如果是磨头，隐藏评率数字
								LCD_Disappear_Picture(Frequency_addr);//隐藏频率图标
								LCD_Disappear_Picture(OpeningPosition_addr);//隐藏开口定位
							}
							else
							{
								
								LCD_Show_Picture(Frequency_addr,UIShow_s.B_Show.frequency_icon);//***************如果是往复模式显示4月30号
								LCD_Show_Picture(Frequency_data_addr,UIdata_s.SetDATA_A.Frequency_data);//显示频率数据(值来自于)
								LCD_Show_Picture(OpeningPosition_addr,OpeningPosition_icon);
							}
						}
						else
						{
							LCD_Show_Picture(ModeSlect_M_P_addr,UIShow_s.B_Show.mode_M_P_Slect_icon);
							
						}
						break;
					default:
						break;
				}
			}

}










 //============================================================================
// 函数名称: LCD_Show_Password_Input()
// 功能描述: 密码输入显示
// 输　  入: Position：当前输入的密码位置
// 输    出: 无
// 函数说明: 1_管理员模式界面
//============================================================================
void Screen_Password_Input(uint8_t Position)
{
  switch (Position)
  {
	  case 0 :
	  {
	    LCD_Disappear_Picture(Addr_Pic_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_2);
	    LCD_Disappear_Picture(Addr_Pic_Page1_3);
	    LCD_Disappear_Picture(Addr_Pic_Page1_4);
	    LCD_Disappear_Picture(Addr_Pic_Page1_5);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	    LCD_Disappear_Picture(Addr_Pic_Page1_7);
	  }
	  break;
	  case 1 :
	  {
	    LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_2);
	    LCD_Disappear_Picture(Addr_Pic_Page1_3);
	    LCD_Disappear_Picture(Addr_Pic_Page1_4);
	    LCD_Disappear_Picture(Addr_Pic_Page1_5);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	  }
	  break;
	  case 2 :
	  {
	    LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_3);
	    LCD_Disappear_Picture(Addr_Pic_Page1_4);
	    LCD_Disappear_Picture(Addr_Pic_Page1_5);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	  }
	  break;
	  case 3 :
	  {
	    LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_3, Addr_ICL_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_4);
	    LCD_Disappear_Picture(Addr_Pic_Page1_5);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	  }
	  break;
	  case 4 :
	  {
      LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_3, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_4, Addr_ICL_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_5);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	  }
	  break;
	  case 5 :
	  {
      LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_3, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_4, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_5, Addr_ICL_Page1_1);
	    LCD_Disappear_Picture(Addr_Pic_Page1_6);
	  }
	  break;
	  case 6 :
	  {
	    LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_3, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_4, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_5, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_6, Addr_ICL_Page1_1);
	  }
	  break;
	  case 7 :
	  {
	    LCD_Show_Picture(Addr_Pic_Page1_1, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_2, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_3, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_4, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_5, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_6, Addr_ICL_Page1_1);
	    LCD_Show_Picture(Addr_Pic_Page1_7, Addr_ICL_Page1_2);
	  }
	  break;
	  default : break;
  }
}

//============================================================================
// 函数名称: Info_B()
// 功能描述: 信息栏图片更新
// 输　  入:  
//           Info1：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量2
 
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_B(uint8_t Info1)
{
	static uint8_t Info1Last = 0xff;
  //泵流量栏2
  if(Info1 != Info1Last)
  {
  	switch (Info1)
  	{
	    case 0 :
	    {
	      //4_
				pum_close_flag_B=0;
	      SysRunData.PumpModel_B = 0; //注水
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);
 	      LCD_Disappear_Number(0x9450);
	      LCD_Disappear_Picture(0x1506);
	    }
	    break;
	    case 1 ://注水模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 1; //注水
	      LCD_Show_Picture(0x1506, 301);
	      LCD_Show_Picture(0x1508, 323);
	      LCD_Show_Picture(0x1509, 324);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;
	    case 2 ://灌注模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 2; //灌注
  		  SysRunData.PumpPourIntoVelocityB = 150;
	      LCD_Show_Picture(0x1506, 300);
	      LCD_Show_Picture(0x1508, 321);
	      LCD_Show_Picture(0x1509, 320);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;
	    case 3 ://抽吸模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 3; //抽吸
	      LCD_Show_Picture(0x1506, 303);
	      LCD_Show_Picture(0x1508, 323);
	      LCD_Show_Picture(0x1509, 324);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;	
	    case 4 ://注水模式 开关
			{	
				SysRunData.PumpModel_B = 0; //注水
				if(SysRunData.PumpSteping5sNum_B==2||SysRunData.PumpDrain_B == 1||SysRunData.PumpPourIntoONOFF_B == 1)
				{pum_close_flag_B=1; }
				else{pum_close_flag_B=2; }
				LCD_Show_Picture(0x1509, 325);
	      LCD_Show_Picture(0x1506, 302);
	      LCD_Show_Picture(0x1508, 328);
	      
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);				
 	      LCD_Disappear_Number(0x9450);
			}	break;			
	    default :
	    {
		    SysRunData.PumpModel_B = 0; //注水
				if(SysRunData.PumpSteping5sNum_B==2||SysRunData.PumpDrain_B == 1||SysRunData.PumpPourIntoONOFF_B == 1)
				{pum_close_flag_B=1;LCD_Show_Picture(0x1508, 323);}
				else{pum_close_flag_B=2;LCD_Show_Picture(0x1508, 322);}
				LCD_Show_Picture(0x1509, 325);
	      LCD_Show_Picture(0x1506, 302);
//	      LCD_Show_Picture(0x1508, 328);
	      
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);				
 	      LCD_Disappear_Number(0x9450);
	    }
	    break;
	  }
		Info1Last = Info1;
  }
}

//============================================================================
// 函数名称: Info_A()
// 功能描述: 信息栏图片更新
// 输　  入:  
//           Info1：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量1
 
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_A(uint8_t Info1)
{
	static uint8_t Info1Last = 0xff;
  //泵流量栏1
  if(Info1 != Info1Last)
  {
  	switch (Info1)
  	{
	    case 0 :
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 0; //注水
	      LCD_Disappear_Number(0x9530);
//	      LCD_Disappear_Number(0x94D0);
	      LCD_Disappear_Number(0x9430);
	      LCD_Disappear_Picture(0x1502);
	    }
	    break;
	    case 1 : //注水模式
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 1; //注水
	      LCD_Show_Picture(0x1502, 301);
	      LCD_Show_Picture(0x1504, 323);
	      LCD_Show_Picture(0x1505, 324);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;
	    case 2 : //灌注模式
	    {
				pum_close_flag_A=0;
        SysRunData.PumpModel_A = 2; //灌注
  		  SysRunData.PumpPourIntoVelocityA = 130;				
	      LCD_Show_Picture(0x1502, 300);
	      LCD_Show_Picture(0x1504, 321);
	      LCD_Show_Picture(0x1505, 320);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;			
	    case 3 : //抽吸模式
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 3; //抽吸
	      LCD_Show_Picture(0x1502, 303);
	      LCD_Show_Picture(0x1504, 323);
	      LCD_Show_Picture(0x1505, 324);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;
	    case 4 : //注水模式 
	    {
		    SysRunData.PumpModel_A = 0; //注水
				if(SysRunData.PumpSteping5sNum_A==2||SysRunData.PumpDrain_A == 1||SysRunData.PumpPourIntoONOFF_A == 1)
				{pum_close_flag_A=1; }
				else
					{ pum_close_flag_A=2; }

	      LCD_Show_Picture(0x1502, 302);
	      LCD_Show_Picture(0x1504, 328);
	    	LCD_Show_Picture(0x1505, 325);		
			
				
	      LCD_Disappear_Number(0x9530);
	      LCD_Disappear_Number(0x9430);
	    }
	    break;
	    default :
	    {
 		    SysRunData.PumpModel_A = 0; //注水
				if(SysRunData.PumpSteping5sNum_A==2||SysRunData.PumpDrain_A == 1||SysRunData.PumpPourIntoONOFF_A == 1)
				{pum_close_flag_A=1; LCD_Show_Picture(0x1504, 323);}
				else
					{
						pum_close_flag_A=2;	
//						if(SysRunData.PumpModel_B == 1)
//						{
//							LCD_Show_Picture(0x1504, 328);	
//						}
//						else{
						LCD_Show_Picture(0x1504, 322);	
					//}
					}
	      LCD_Show_Picture(0x1502, 302);
//	      LCD_Show_Picture(0x1504, 328);
	    	LCD_Show_Picture(0x1505, 325);		
	      LCD_Disappear_Number(0x9530);
	      LCD_Disappear_Number(0x9430);					
	    }
	    break;
	  }
		Info1Last = Info1;
  }
}
 
//============================================================================
// 函数名称: Info_HZ()
// 功能描述: 信息栏图片更新
//           Info3：右下栏 0隐藏，1往复的频率信息（高亮），2往复的频率（灰色），
//                         3速度挡位信息（灰色），4Ⅰ挡位（高亮），5Ⅱ挡位（高亮），6Ⅲ挡位（高亮）
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Info_HZ(uint8_t Info3)
{
  static uint8_t Info3Last = 0xff;
  //单向的速度挡位、往复的频率栏
  if (Info3 != Info3Last)
  {
  	switch (Info3)
  	{
  	  case 0 :
  	  {
				
	  	  LCD_Disappear_Number(0x9460);
	  	  LCD_Disappear_Number(0x9470);
 
				LCD_Disappear_Picture(0x1601);
	    }
	    break;
	    case 1 :  //往复的频率栏高亮
	    {
		    LCD_Show_Picture(0x1601, 351);
				LCD_Show_Number (0x9460, 0x3460);
		    LCD_Show_Number (0x9470, 0x3470);
				
					LCD_Show_Picture(0x1601, 351);
				LCD_Show_Number (0x9460, 0x3460);
		    LCD_Show_Number (0x9470, 0x3470);

	    }
	    break;
	    case 2 :  //往复的频率栏灰色
	    {
	      LCD_Show_Picture(0x1601, 350);

	      LCD_Disappear_Number(0x9460);
	      LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 3 :  //速度挡位栏灰色
	    {
		    LCD_Show_Picture(0x1601, 355);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 4 : //I档位条的显示
	    {
	      LCD_Show_Picture(0x1601, 352);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 5 : //II档位条的显示
	    {
		    LCD_Show_Picture(0x1601, 353);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    default : //III档位条的显示
	    {
		    LCD_Show_Picture(0x1601, 354);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
  	}

	  Info3Last = Info3;
  }	
	
}
//============================================================================
// 函数名称: LCD_InformationBarImage_Update()
// 功能描述: 信息栏图片更新
// 输　  入: Info1：上方栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）转速
//           Info2：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量2
//           Info3：右下栏 0隐藏，1往复的频率信息（高亮），2往复的频率（灰色），
//                         3速度挡位信息（灰色），4Ⅰ挡位（高亮），5Ⅱ挡位（高亮），6Ⅲ挡位（高亮）
//           Info4：左下栏 0隐藏，1接如手柄（高亮），2未接入手柄（灰色）流量1
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_InformationBarImage_Update(uint8_t Info1, uint8_t Info2, uint8_t Info3, uint8_t Info4)
{
  static uint8_t Info1Last = 0xff, Info2Last = 0xff, Info3Last = 0xff, Info4Last = 0xff;

  //转速信息栏
  if (Info1 != Info1Last)
  {
	  switch (Info1)
	  {
	    case 0 :
	    {
		    LCD_Disappear_Number(0x9400);   //最大转速
		    LCD_Disappear_Number(0x9410);   //当前设置转速
		    LCD_Disappear_Number(0x9420);   //当前转速
				LCD_Disappear_Picture(0x1600);
	    }
	    break;
	    case 1 :
	    {
	      LCD_Show_Picture(0x1600, 341);  //当前转速栏高亮

	      LCD_Show_Number(0x9400, 0x3400);
	      LCD_Show_Number(0x9410, 0x3410);
	      LCD_Show_Number(0x9420, 0x3420);
	    }
	    break;
	    default :
	    {
	      LCD_Disappear_Number(0x9400);
	      LCD_Disappear_Number(0x9410);
	      LCD_Disappear_Number(0x9420);

	      LCD_Show_Picture(0x1600, 340); //当前转速栏灰色
	    }
	    break;
	  }

	  Info1Last = Info1;
  }

  //泵流量栏2
  if(Info2 != Info2Last)
  {
  	switch (Info2)
  	{
	    case 0 :
	    {
	      //4_
				pum_close_flag_B=0;
	      SysRunData.PumpModel_B = 0; //注水
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);
 	      LCD_Disappear_Number(0x9450);
	      LCD_Disappear_Picture(0x1506);
	    }
	    break;
	    case 1 ://注水模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 1; //注水
	      LCD_Show_Picture(0x1506, 301);
	      LCD_Show_Picture(0x1508, 323);
	      LCD_Show_Picture(0x1509, 324);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;
	    case 2 ://灌注模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 2; //灌注
				SysRunData.PumpPourIntoVelocityB = 150;
	      LCD_Show_Picture(0x1506, 300);
	      LCD_Show_Picture(0x1508, 321);
	      LCD_Show_Picture(0x1509, 320);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;
	    case 3 ://抽吸模式
	    {
				pum_close_flag_B=0;
				SysRunData.PumpModel_B = 3; //抽吸
	      LCD_Show_Picture(0x1506, 303);
	      LCD_Show_Picture(0x1508, 328);
	      LCD_Show_Picture(0x1509, 325);
//		    LCD_Show_Number (0x94A0, 0x34A0);
		    LCD_Show_Number (0x9550, 0x3550);				
//		    LCD_Show_Number (0x9450, 0x3450);
	    }
	    break;	
	    case 4 ://注水隐藏
	    {
				SysRunData.PumpModel_B = 0; //注水
				if(SysRunData.PumpSteping5sNum_B==2||SysRunData.PumpDrain_B == 1||SysRunData.PumpPourIntoONOFF_B == 1)
				{pum_close_flag_B=1; }
				else{pum_close_flag_B=2; }
				LCD_Show_Picture(0x1509, 325);
	      LCD_Show_Picture(0x1506, 302);
	      LCD_Show_Picture(0x1508, 328);
	      
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);				
 	      LCD_Disappear_Number(0x9450);				
	    }
	    break;		
	    default :
	    {
		    SysRunData.PumpModel_B = 0; //注水
				if(SysRunData.PumpSteping5sNum_B==2||SysRunData.PumpDrain_B == 1||SysRunData.PumpPourIntoONOFF_B == 1)
				{pum_close_flag_B=1; LCD_Show_Picture(0x1508, 323);}
				else{pum_close_flag_B=2; LCD_Show_Picture(0x1508, 322);}
				LCD_Show_Picture(0x1509, 325);
	      LCD_Show_Picture(0x1506, 302);
//	      LCD_Show_Picture(0x1508, 328);
	      //LCD_Show_Picture(0x1509, 324);
//	      LCD_Disappear_Number(0x94A0);
	      LCD_Disappear_Number(0x9550);				
 	      LCD_Disappear_Number(0x9450); 				
	    }
	    break;
	  }
		Info2Last = Info2;
  }

  //单向的速度挡位、往复的频率栏
  if (Info3 != Info3Last)
  {
  	switch (Info3)
  	{
  	  case 0 :
  	  {

	  	  LCD_Disappear_Number(0x9460);
	  	  LCD_Disappear_Number(0x9470);


	  	  LCD_Disappear_Picture(0x1601);
	    }
	    break;
	    case 1 :  //往复的频率栏高亮
	    {
		    LCD_Show_Picture(0x1601, 351);

		    LCD_Show_Number (0x9460, 0x3460);
		    LCD_Show_Number (0x9470, 0x3470);


	    }
	    break;
	    case 2 :  //往复的频率栏灰色
	    {
	      LCD_Show_Picture(0x1601, 350);

	      LCD_Disappear_Number(0x9460);
	      LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 3 :  //速度挡位栏灰色
	    {
		    LCD_Show_Picture(0x1601, 355);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 4 : //I档位条的显示
	    {
	      LCD_Show_Picture(0x1601, 352);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    case 5 : //II档位条的显示
	    {
		    LCD_Show_Picture(0x1601, 353);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
	    default : //III档位条的显示
	    {
		    LCD_Show_Picture(0x1601, 354);

		    LCD_Disappear_Number(0x9460);
		    LCD_Disappear_Number(0x9470);


	    }
	    break;
  	}

	  Info3Last = Info3;
  }
  //泵流量栏1
  if(Info4 != Info4Last)
  {
  	switch (Info4)
  	{
	    case 0 :
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 0; //注水
	      LCD_Disappear_Number(0x9530);
//	      LCD_Disappear_Number(0x94D0);
	      LCD_Disappear_Number(0x9430);
	      LCD_Disappear_Picture(0x1502);
	    }
	    break;
	    case 1 : //注水模式
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 1; //注水
	      LCD_Show_Picture(0x1502, 301);
	      LCD_Show_Picture(0x1504, 323);
	      LCD_Show_Picture(0x1505, 324);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;
	    case 2 : //灌注模式
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 2; //灌注
  		  SysRunData.PumpPourIntoVelocityA = 130;					
	      LCD_Show_Picture(0x1502, 300);
	      LCD_Show_Picture(0x1504, 321);
	      LCD_Show_Picture(0x1505, 320);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;			
	    case 3 : //抽吸模式
	    {
				pum_close_flag_A=0;
				SysRunData.PumpModel_A = 3; //抽吸
	      LCD_Show_Picture(0x1502, 303);
	      LCD_Show_Picture(0x1504, 323);
	      LCD_Show_Picture(0x1505, 324);
				
		    LCD_Show_Number (0x9530, 0x3530);
//		    LCD_Show_Number (0x94D0, 0x34D0);
//		    LCD_Show_Number (0x9430, 0x3430);
	    }
	    break;	
	    case 4 : //注水隐藏
	    {
		    SysRunData.PumpModel_A = 0; //注水
				if(SysRunData.PumpSteping5sNum_A==2||SysRunData.PumpDrain_A == 1||SysRunData.PumpPourIntoONOFF_A == 1)
				{pum_close_flag_A=1; }
				else{pum_close_flag_A=2; }

	      LCD_Show_Picture(0x1502, 302);
	      LCD_Show_Picture(0x1504, 328);
				LCD_Show_Picture(0x1505, 325);				
	      LCD_Disappear_Number(0x9530);
	      LCD_Disappear_Number(0x9430);
	    }
	    break;	
	    default :
	    {
		    SysRunData.PumpModel_A = 0; //注水
				if(SysRunData.PumpSteping5sNum_A==2||SysRunData.PumpDrain_A == 1||SysRunData.PumpPourIntoONOFF_A == 1)
				{pum_close_flag_A=1;  LCD_Show_Picture(0x1504, 323);}
				else{pum_close_flag_A=2;  LCD_Show_Picture(0x1504, 322);}

	      LCD_Show_Picture(0x1502, 302);
//	      LCD_Show_Picture(0x1504, 328);
				LCD_Show_Picture(0x1505, 325);				
	      LCD_Disappear_Number(0x9530);
	      LCD_Disappear_Number(0x9430); 				
	    }
	    break;
	  }
		Info4Last = Info4;
  }	
	
	
}

//============================================================================
// 函数名称: LCD_Show_IntegratedCutterPic_Update()
// 功能描述: 一体式刀具信息图片显示
// 输　  入: IntegratedCutter 1号刀具， 2 2号刀具，3 其它刀具
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面 10ms ~ 30ms
//============================================================================
void Screen_IntegratedCutterPic_Update(uint8_t IntegratedCutter)
{
  static uint8_t IntegratedCutterLast = 0xff;

  if (IntegratedCutter == IntegratedCutterLast)
	  return ;

  IntegratedCutterLast = IntegratedCutter;

  switch (IntegratedCutter)
  {
	  case 0 :
    {
	    //分离式刀具信息显示位置 
	    LCD_Disappear_Picture(0x1606);

	    //顶栏（一体式手柄，刀具信息）
	    LCD_Disappear_Picture(0x1604);
	    LCD_Disappear_Picture(0x1605);
 	    LCD_Disappear_Picture(0x4200);

	  }
	  break;
	  case 1 :  //1号刀具已连接
	  {
	    LCD_Show_Picture(0x1604, 380);
	    LCD_Show_Picture(0x1605, 386);
	  }
	  break;
	  case 2 :  //2号刀具已连接
	  {
	    LCD_Show_Picture(0x1604, 380);
	    LCD_Show_Picture(0x1605, 386);
	  }
	  break;
	  default :  //其它刀具已连接
	  {
	    LCD_Show_Picture(0x1604, 380);
	    LCD_Show_Picture(0x1605, 386);
	  }
    break;
  }
}

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
void Screen_SeparatingCutterPic_Update(uint8_t SeparatingCutter, uint8_t WFFlag)
{
  static uint8_t SeparatingCutterLast = 0xff, WFFlagLast = 0xff;

  if ((SeparatingCutter == SeparatingCutterLast) && (WFFlag == WFFlagLast))
	  return ;

  SeparatingCutterLast = SeparatingCutter;
  WFFlagLast = WFFlag;

  switch (SeparatingCutter)
  {
	  case 0 :  //顶栏
    {
	    //一体式刀具图片显示位置
	    LCD_Disappear_Picture(0x1604);
	    LCD_Disappear_Picture(0x1605);
 	    LCD_Disappear_Picture(0x4200);


 	    //分体式刀具图片显示位置
 	    LCD_Disappear_Picture(0x1605);
      LCD_Disappear_Picture(0x1606);
	  }
	  break;
	  case 1 :  //刀具未设别
	  {
	    if (WFFlag == 1)   //DX_WF_Flag
      {
		    LCD_Show_Picture(0x1606, 400);   //往复角度设置图片
	    }
  	  else
      {
		    LCD_Disappear_Picture(0x1606);
	    }
	    LCD_Show_Picture(0x1604, 381);   //刀具设别已开启
	    LCD_Show_Picture(0x1605, 383);   //刀具未设别
	  }
	  break;
	  case 2 :  //刀具已连接
	  {
	    if (WFFlag == 1)   //DX_WF_Flag
      {
		    LCD_Show_Picture(0x1606, 400);   //往复角度设置图片
	    }
  	  else
      {
		    LCD_Disappear_Picture(0x1606);
	    }
	    LCD_Show_Picture(0x1604, 381);   //刀具设别已开启
	    LCD_Show_Picture(0x1605, 384);   //刀具图片
	  }
	  break;
	  case 3 :  //刀具已连接
	  {
	    if (WFFlag == 1)   //DX_WF_Flag
      {
		    LCD_Show_Picture(0x1606, 400);   //往复角度设置图片
	    }
	    else
      {
		    LCD_Disappear_Picture(0x1606);
	    }
	    LCD_Show_Picture(0x1604, 381);   //刀具设别已开启
	    LCD_Show_Picture(0x1605, 385);   //刀具图片
	  }
	  break;
	  case 4 :  //刀具已连接
	  {
	    LCD_Show_Picture(0x1604, 382);   //刀具设别已开启
	    LCD_Show_Picture(0x1605, 388);  //刀具设别已关闭 选择刀具 选中“刨刀”
	    LCD_Show_Picture(0x1606, 400);  //往复角度设置图片
	  }
	  break;
	  default :  //刀具已连接
	  {
	    LCD_Show_Picture(0x1604, 382);   //刀具设别已开启
	    LCD_Disappear_Picture(0x1606);
	    LCD_Show_Picture(0x1605, 387);  //刀具设别已关闭 选择刀具 选中“磨头”
	  }
	  break;
  }
}

//============================================================================
//============================================================================
void Screen_PFNo5No2HandleDisPic_Update(uint16_t Addr, uint8_t HType, uint8_t Flay)
{
  if (Flay)
  {
	  //显
	  switch (HType)
	  {
	    case Handle_Type_22 :
	    case Handle_Type_23 :
	    {
		    //PXBA、PXBB
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 100);
        else   LCD_Show_Picture(Addr, 200);  
	    }
	    break;
	    case Handle_Type_2 :
	    {
		    //JMB无霍尔
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 104);
        else   LCD_Show_Picture(Addr, 204); 
	    }
	    break;
	    case Handle_Type_3 :
	    {
		    //TMBB磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 104);
        else   LCD_Show_Picture(Addr, 204); 
	    }
	    break;
	    case Handle_Type_4 :
			{
				//一体刨磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 104);
        else   LCD_Show_Picture(Addr, 204); 
			}
			 break;
	    case Handle_Type_5 :
	    {
		    //TMBC(未生产)、TMBA（EMBA、EMBB）磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 104);
        else   LCD_Show_Picture(Addr, 204); 
	    }
	    break;
	    default : break;
	  }
  }
  else
  {
	  //灰
	  switch (HType)
	  {
	    case Handle_Type_22 :
	    case Handle_Type_23 :
	    {
		    //PXBA、PXBB
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 101);
        else   LCD_Show_Picture(Addr, 201); 
	    }
	    break;
	    case Handle_Type_2 :
	    {
	      //JMB无霍尔
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 105);
        else   LCD_Show_Picture(Addr, 205); 
	    }
	    break;
	    case Handle_Type_3 :
	    {
		    //TMBB磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 105);
        else   LCD_Show_Picture(Addr, 205); 
	    }
	    break;
	    case Handle_Type_4 :
			{
				//一体刨磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 104);
        else   LCD_Show_Picture(Addr, 204); 
			}
			break;
	    case Handle_Type_5 :
	    {
		    //TMBC(未生产)、TMBA（EMBA、EMBB）磨钻手柄选中图
				if(Addr == 0x1500)
		    LCD_Show_Picture(Addr, 105);
        else   LCD_Show_Picture(Addr, 205); 
	    }
	    break;
	    default : break;
	  }
  }
}

//============================================================================
//============================================================================
void Screen_PANo1HandleDisPic_Update(uint16_t Addr, uint8_t HType, uint8_t Flay)
{
  if (Flay)
  {
	  switch (HType)
	  {
	    case Handle_Type_4 :
	    case Handle_Type_5 :
	    {
		    //TMBC(未生产)、TMBA磨钻手柄选中图 显
		    LCD_Show_Picture(Addr, 290);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 290);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_6 :
	    {
		    //EMBD磨钻手柄选中图 显
		    LCD_Show_Picture(Addr, 296);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 296);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_7 :
	    {
		    //EMBC磨钻手柄选中图 显
		    LCD_Show_Picture(Addr, 294);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 294);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_2 :
	    {
		    //JMB无霍尔
		    LCD_Show_Picture(Addr, 316);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 316);
//        delay_ms(5);
	    }
	    break;
	    default :
	    {
		    //TMBB磨钻手柄选中图 显
		    LCD_Show_Picture(Addr, 292);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 292);
//        delay_ms(5);
	    }
	    break;
	  }
  }
  else
  {
    switch (HType)
	  {
	    case Handle_Type_4 :
	    case Handle_Type_5 :
	    {
		    LCD_Show_Picture(Addr, 291);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 291);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_6 :
	    {
		    LCD_Show_Picture(Addr, 297);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 297);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_7 :
	    {
		    LCD_Show_Picture(Addr, 295);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 295);
//        delay_ms(5);
	    }
	    break;
	    case Handle_Type_2 :
	    {
		    LCD_Show_Picture(Addr, 317);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 317);
//        delay_ms(5);
	    }
	    break;
	    default :
	    {
		    LCD_Show_Picture(Addr, 293);
//		    delay_ms(5);
//		    LCD_Show_Picture(Addr, 293);
//        delay_ms(5);
	    }
	    break;
	  }
  }
}

//============================================================================
// 函数名称: LCD_Show_SeparatingCutterPic_Update()
// 功能描述: 手柄连接状态栏图片显示
// 输　  入: *HandleConn：HandleConn[0 ~ 3] 1 ~ 4号接口，1 接入未选中，2 接入且选中
//           *HandleType：HandleType[0 ~ 3] 1 ~ 4号接口手柄类型，1 ~ ... 对应手柄的类型
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面  显示位置 2[A] 3[B] 3[B] 1[C]
//============================================================================
uint8_t *Screen_HandleConnectState_Update(uint8_t *HandleConn, uint8_t *HandleType)
{
  //复位该区域
  //Background == 4 0x1307\0x1308\0x1309
  //Background == 5 0x130A\0x1307\0x1308\0x1309

  static uint8_t UIHandleOrder[5] = { 0 };  //[0 ~ 3]记录手柄显示位置，[4]记录选中的手柄的位置

  static uint8_t HandleConnLast[5] = { 0xff }, HandleTypeLast[5] = { 0xff };

//  if ((Common_CompareData(HandleConn, HandleConnLast, 5)) && (Common_CompareData(HandleType, HandleTypeLast, 5)))
//	  return UIHandleOrder;

  Common_CopyData(HandleConn, HandleConnLast, 5);
  Common_CopyData(HandleType, HandleTypeLast, 5);

  LCD_Disappear_Picture(0x1500);
  LCD_Disappear_Picture(0x1501);


  //未接 Background == 4
  if((HandleConn[4] == 0) && (HandleConn[1] == 0) && (HandleConn[2] == 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 0;
	  UIHandleOrder[1] = 0;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  UIHandleOrder[4] = 0;

		K1_OFF();
		K2_OFF();			
		LCD_Show_Picture(0x1500, 106);   //中"手柄未连接" 
	  LCD_Show_Picture(0x1501, 206);   //中"手柄未连接"

  }
  //5号手柄连接且选中 Background == 4
  else if((HandleConn[4] > 0) && (HandleConn[1] == 0) && (HandleConn[2] == 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 5;
	  UIHandleOrder[1] = 0;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

		K1_OFF();		
	  UIHandleOrder[4] = 1;  //当前选中的位置
    LCD_Show_Picture(0x1500, 106);   //中"手柄未连接" 
	  Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[4], 1);
  }
  //2号手柄连接且选中 Background == 4
  else if((HandleConn[4] == 0) && (HandleConn[1] > 0) && (HandleConn[2] == 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 2;
	  UIHandleOrder[1] = 0;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  UIHandleOrder[4] = 1;
		
		K2_OFF();			
    LCD_Show_Picture(0x1501, 206);   //中"手柄未连接" 
	  Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[1], 1);
  }
  //3号手柄连接且选中 Background == 4
  else if((HandleConn[4] == 0) && (HandleConn[1] == 0) && (HandleConn[2] > 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 3;
	  UIHandleOrder[1] = 0;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  UIHandleOrder[4] = 1;

		K2_OFF();	
	  LCD_Show_Picture(0x1500, 102);   //3号手柄选中图 显
	  LCD_Show_Picture(0x1501, 206);   //中"手柄未连接"
  }
  //4号手柄连接且选中 Background == 4
  else if((HandleConn[4] == 0) && (HandleConn[1] == 0) && (HandleConn[2] == 0) && (HandleConn[3] > 0))
  {
	  UIHandleOrder[0] = 4;
	  UIHandleOrder[1] = 0;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  UIHandleOrder[4] = 1;
		K1_OFF();		
	  LCD_Show_Picture(0x1500, 106);   //中"手柄未连接"
	  LCD_Show_Picture(0x1501, 202);   //4号手柄选中图 显
//					SysModelConfig.HandlePortA = 0;
//					SysModelConfig.HandlePortB = 1;
  }
  //2、5连接  Background == 5
  else if((HandleConn[4] > 0) && (HandleConn[1] > 0) && (HandleConn[2] == 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 2;
	  UIHandleOrder[1] = 5;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[4] > 1)  //5号手柄选中
	  {
	    //**************************2**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[1], 0);

	    //**************************5**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[4], 1);

	    UIHandleOrder[4] = 2;
	  }
	  else  //2号手柄选中
	  {
	    //**************************2**************************
      Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[1], 1);

	    //**************************5**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[4], 0);

	    UIHandleOrder[4] = 1;
	  }
  }
  //5、3号连接  Background == 5
  else if((HandleConn[4] > 0) && (HandleConn[1] == 0) && (HandleConn[2] > 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 3;//5;
	  UIHandleOrder[1] = 5;//3;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[4] > 1)
	  {
	    //**************************5**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[4], 1);

	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 103);
//	    delay_ms(5);
//	    LCD_Show_Picture(0x1500,103);
//      delay_ms(5);

	    UIHandleOrder[4] = 2;
	  }
	  else
	  {
	    //**************************5**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[4], 0);

	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 102);
//	    delay_ms(5);
//	    LCD_Show_Picture(0x1500,102);
//      delay_ms(5);

	    UIHandleOrder[4] = 1;
	  }
  }
  //5、4号连接  Background == 5
  else if((HandleConn[4] > 0) && (HandleConn[1] == 0) && (HandleConn[2] == 0) && (HandleConn[3] > 0))
  {
	  UIHandleOrder[0] = 5;
	  UIHandleOrder[1] = 4;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[4] > 1)
	  {
	    //**************************5**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[4], 1);

	    //**************************4**************************
	    LCD_Show_Picture(0x1501, 203);

	    UIHandleOrder[4] = 1;
    }
    else
    {
	    //**************************5**************************
      Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[4], 0);

	    //**************************4**************************
	    LCD_Show_Picture(0x1501, 202);
	    UIHandleOrder[4] = 2;
    }
  }
  //2、3号连接  Background == 5
  else if((HandleConn[4] == 0) && (HandleConn[1] > 0) && (HandleConn[2] > 0) && (HandleConn[3] == 0))
  {
	  UIHandleOrder[0] = 2;
	  UIHandleOrder[1] = 3;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[1] > 1)
	  {
	    //**************************2**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[1], 1);

	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 103);
	    UIHandleOrder[4] = 1;
	  }
	  else
	  {
	    //**************************2**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1501, HandleType[1], 0);

	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 102);
	    UIHandleOrder[4] = 2;
	  }
  }
  //2、4号连接  Background == 5
  else if((HandleConn[4] == 0) && (HandleConn[1] > 0) && (HandleConn[2] == 0) && (HandleConn[3] > 0))
  {
	  UIHandleOrder[0] = 2;
	  UIHandleOrder[1] = 4;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[1] > 1)
	  {
	    //**************************2**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[1], 1);

	    //**************************4**************************
     LCD_Show_Picture(0x1501, 203);
	    UIHandleOrder[4] = 1;
	  }				

	  else
	  {
	    //**************************2**************************
	    Screen_PFNo5No2HandleDisPic_Update(0x1500, HandleType[1], 0);

	    //**************************4**************************
	     LCD_Show_Picture(0x1501, 202);
	    UIHandleOrder[4] = 2;
	  }			
		
  }
  //3、4号连接  Background == 5
  else if((HandleConn[4] == 0) && (HandleConn[1] == 0) && (HandleConn[2] > 0) && (HandleConn[3] > 0))
  {
	  UIHandleOrder[0] = 3;
	  UIHandleOrder[1] = 4;
	  UIHandleOrder[2] = 0;
	  UIHandleOrder[3] = 0;

	  if (HandleConn[2] > 1)
	  {
	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 102);

	    //**************************4**************************
	    LCD_Show_Picture(0x1501, 203);

	    UIHandleOrder[4] = 1;
	  }
	  else
	  {
	    //**************************3**************************
	    LCD_Show_Picture(0x1500, 103);


	    //**************************4**************************
	    LCD_Show_Picture(0x1501, 202);


	    UIHandleOrder[4] = 2;
	  }
  }
  return UIHandleOrder;
}

//============================================================================
// 函数名称: LCD_FootPedalConnectState_Update()
// 功能描述: 脚踏连接状态图片显示
// 输　  入: Type:0 不显示脚控、手控图片，1 脚控未连接，2 脚控 已选中，
//                3 手控 已选中，4 脚控已连接 手控已选中，5 脚控已选中 手控已连接
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_FootPedalConnectState_Update(uint8_t Type)
{
  uint8_t TypeLast = 0xff;

  if (Type == TypeLast)
    return ;

  TypeLast = Type;

  switch (Type)
  {
	  case 0 :  //不显示脚控、手控图片
	  {
//	    LCD_Disappear_Picture(0x1312);
//	    LCD_Disappear_Picture(0x1313);
	    LCD_Disappear_Picture(0x1603);			
	  }
	  break;
	  case 1 :  //脚控未连接
	  {
//	    LCD_Disappear_Picture(0x1313);
	    LCD_Show_Picture(0x1603, 370);
	  }
	  break;
	  case 2 :  //脚控 已选中
	  {
//	    LCD_Disappear_Picture(0x1313);
	    LCD_Show_Picture(0x1603, 374);
	  }
	  break;
	  case 3 :  //手控 已选中
	  {
//	    LCD_Disappear_Picture(0x1313);
	    LCD_Show_Picture(0x1603, 377);
	  }
	  break;
	  case 4 : //脚控已连接 手控已选中
	  {
//	    LCD_Show_Picture(0x1312, 260);
//	    LCD_Show_Picture(0x1313, 263);
			  LCD_Show_Picture(0x1603, 372);
	  }
	  break;
	  case 5 : //脚控已选中 手控已连接
	  {
//	    LCD_Show_Picture(0x1312, 261);
//	    LCD_Show_Picture(0x1313, 262);
			LCD_Show_Picture(0x1603, 371);
	  }
	  break;
	  default : break;
  }
}
//============================================================================
// 函数名称: LCD_ElectricalMachineryDirectionState_Update()
// 功能描述: 电机转动方向图片显示
// 输　  入: Dir1 0 正向图片隐，1 正向 未选中 长，2 正向 选中 长，3 正向 未选中 短，4 正向 选中 短
//           Dir2 0 往复图片隐，1 正向隐 往复 未选中 长，2 正向隐 往复 选中 长，3 往复 未选中 长，4 往复 选中 长
//           Dir3 0 反转图片隐，1 反向 未选中 短，2 反向 选中 短
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_ElectricalMachineryDirectionState_Update2(uint8_t Dir1)
{
  static uint8_t Dir1Last = 0xff;
  if (Dir1 != Dir1Last)
  {
	  switch (Dir1)
	  {
	    case 0 : LCD_Show_Picture(0x1602,360); break; //3隐
	    case 1 : LCD_Show_Picture(0x1602,361); break; //3正显
	    case 2 : LCD_Show_Picture(0x1602,362); break; //3往复显
	    case 3 : LCD_Show_Picture(0x1602,363); break; //3反显
	    case 4 : LCD_Show_Picture(0x1602,364); break; //2正显			
	    case 5 : LCD_Show_Picture(0x1602,365); break; //2反显	
			case 6 : LCD_Show_Picture(0x1602,366); break; //往复单选						
	    default : LCD_Show_Picture(0x1602,360);break; //3隐
	  }
    Dir1Last = Dir1;
  }	
}
//============================================================================
// 函数名称: LCD_ElectricalMachineryDirectionState_Update()
// 功能描述: 电机转动方向图片显示
// 输　  入: Dir1 0 正向图片隐，1 正向 未选中 长，2 正向 选中 长，3 正向 未选中 短，4 正向 选中 短
//           Dir2 0 往复图片隐，1 正向隐 往复 未选中 长，2 正向隐 往复 选中 长，3 往复 未选中 长，4 往复 选中 长
//           Dir3 0 反转图片隐，1 反向 未选中 短，2 反向 选中 短
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_ElectricalMachineryDirectionState_Update(uint8_t Dir1, uint8_t Dir2, uint8_t Dir3)
{
//  static uint8_t Dir1Last = 0xff, Dir2Last = 0xff, Dir3Last = 0xff;

////	if ((Dir1 == 0) && (Dir2 == 0) && (Dir3 == 0))
////	{
////		LCD_Show_Picture(0x1602,360);  // 
////	}
////	else	if (Dir1 == 2)
////	{
////		LCD_Show_Picture(0x1602,364);  //正向 选中 长 
////	}
////	else	if (Dir1 == 4)
////	{
////		LCD_Show_Picture(0x1602,361);  //正向 选中 短
////	}	
////	else	if (Dir2 == 2 || Dir2 == 4)
////	{
////		LCD_Show_Picture(0x1602,362);  //正向 选中 短
////	}		
////	else	if (Dir3 == 2)
////	{
////		LCD_Show_Picture(0x1602,365);  //正向 选中 长 
////	}
////	else	if (Dir3 == 4)
////	{
////		LCD_Show_Picture(0x1602,363);  //正向 选中 短
////	}	
////	else
////		LCD_Show_Picture(0x1602,360);  // 	
//  //单向:0x1310  正向
//  if (Dir1 != Dir1Last)
//  {
//	  switch (Dir1)
//	  {
//	    case 0 : LCD_Show_Picture(0x1602,360); break;  //   LCD_Disappear_Picture(0x1310); break;  //隐
//	    case 1 :
//	    {
////		    LCD_Disappear_Picture(0x1311);  //往复 隐
//		    //LCD_Show_Picture(0x1602,365); break;  //LCD_Show_Picture(0x1310,254);  //正向 未选中 长
//	    }
//	    break;
//	    case 2 :
//	    {
////		    LCD_Disappear_Picture(0x1311);  //往复 隐
//		    LCD_Show_Picture(0x1602,361); break;  //LCD_Show_Picture(0x1310,255);  //正向 选中 长
//	    }
//	    break;
//	    case 3 : //LCD_Show_Picture(0x1602,360); break;  //LCD_Show_Picture(0x1310, 250); break;  //正向 未选中 短
//	    default : LCD_Show_Picture(0x1602,361); break;  //LCD_Show_Picture(0x1310, 251); break;  //正向 选中 短
//	  }

//    Dir1Last = Dir1;
//  }

//  //往复:0x1311
//  if (Dir2 != Dir2Last)
//  {
//	  switch (Dir2)
//	  {
//	    case 0 : LCD_Show_Picture(0x1602,360); break;  //LCD_Disappear_Picture(0x1311); break;  //隐
//      case 1 :
//	    {
////        LCD_Disappear_Picture(0x1310);  //隐
//          //LCow_Picture(0x1602,360); break;    //LCD_Show_Picture(0x1311, 256);  //往复 未选中 长
//	    }
//	    break;
//	    case 2 :
//	    {
////        LCD_Disappear_Picture(0x1310);  //隐
//          LCD_Show_Picture(0x1602,362); break;  //LCD_Show_Picture(0x1311, 257);  //往复 选中 长
//	    }
//	    break;
//	    case 3 : //LCD_Show_Picture(0x1602,360); break;    //LCD_Show_Picture(0x1311, 256); break;  //往复 未选中 长
//      case 4 : LCD_Show_Picture(0x1602,362); break;  //LCD_Show_Picture(0x1311, 257); break;  //往复 选中 长
//	    default : break;
//	  }

//	  Dir2Last = Dir2;
//  }

//  //单向:0x1316 反向
//  if (Dir3 != Dir3Last)
//  {
//	  switch (Dir3)
//	  {
//	    case 0 : LCD_Show_Picture(0x1602,360); break;  //LCD_Disappear_Picture(0x1316); break;  //隐
//	    case 1 : //LCD_Show_Picture(0x1602,360); break;    //LCD_Show_Picture(0x1316, 303); break;  //反向 未选中 短
//	    case 2 : LCD_Show_Picture(0x1602,363); break;    //LCD_Show_Picture(0x1316, 304); break;  //反向 选中 短
//	    default : break;
//    }

//	  Dir3Last = Dir3;
//  }
}

//============================================================================
// 函数名称: LCD_TipInfo_Update()
// 功能描述: 提示信息刷新
// 输　  入: TipID 提示信息ID
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_TipInfo_Update(uint8_t TipID)
{
  static uint8_t TipIDLast = 0xff;

  if (TipID == TipIDLast)
	  return ;

  TipIDLast = TipID;

  switch (TipID)
  {
	  case 0 : //隐
	  {
	    LCD_Disappear_Picture(0x1607);
 
    }
	  break;
	  case 1 : //1.手柄未连接，请连接手柄！
	  {
      LCD_Show_Picture(0x1607, 407);
 
	  }
    break;
	  case 2 : //3.刀具未连接，请连接刀具！
    {
      LCD_Show_Picture(0x1607, 403);
 
	  }
    break;
	  case 3 : //8.电机相位错误，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 408);
 
	  }
    break;
	  case 4 : //4.霍尔信号错误，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 404);
 
	  }
    break;
	  case 5 : //6.电机过载，请松开脚踏后再运行，或者检查刀具是否卡死。
	  {
      LCD_Show_Picture(0x1607, 406);
 
	  }
    break;
	  case 6 : //10.手柄型号错误，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 410);
 
	  }
    break;
	  case 7 : //5.电机通讯故障，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 405);
 
	  }
    break;
	  case 8 : //9.脚踏存储值读取错误，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 409);
 
	  }
    break;
	  case 9 : //11.UID错误，请联系售后！
	  {
      LCD_Show_Picture(0x1607, 411);
 
	  }
    break;
	  case 12 : //12.脚控已选中，请使用脚控启动电机！
	  {
      LCD_Show_Picture(0x1607, 412);
 
	  }
    break;
	  case 13 : //13.手控已选中，请使用手控启动电机！
	  {
      LCD_Show_Picture(0x1607, 413);
 
	  }
    break;
	  default : break;
  }
}

//============================================================================
// 函数名称: LCD_WindowSwitch_Update()
// 功能描述: 脚踏控制的窗口切换
// 输　  入: Num1 0 ~ 5 隐藏的框 Num2 0 ~ 5 显示的框
// 输    出: 无
// 函数说明: 4_运行界面，5_运行界面
//============================================================================
void Screen_WindowSwitch_Update(int8_t Num1, uint8_t Num2)
{
	switch (Num1)
  {
	  case 0 :
	  case 1 :
    case 2 :		
	  case 3 : 
	  case 4 : 
	  case 5 : 
		{
		 LCD_Disappear_Picture(0x1410);
		 LCD_Disappear_Picture(0x1411);	
		 LCD_Disappear_Picture(0x1412);	
		// LCD_Disappear_Picture(0x1413);	
		 LCD_Disappear_Picture(0x1414);
		 LCD_Disappear_Picture(0x1415);
		}
		break;

	  default : break;
  }
 
  switch (Num2) //闪烁
  {
	  case 0 :
	  {
	    LCD_Show_Picture(0x1410, 310);
	  }
	  break;
	  case 1 :
	  {
			if(SysModelConfig.HandlePortA == 1)
			{
				LCD_Show_Picture(0x1411, 312);			
			}			
			if(SysModelConfig.HandlePortB == 1)
			{
				LCD_Show_Picture(0x1412, 312);			
			} 
	  }
	  break;
	  case 2 :
	  {
  	  LCD_Show_Picture(0x1413, 313);
	  }
	  break;
	  case 3 :
	  {
	    LCD_Show_Picture(0x1415, 315); 
	  }
	  break;
 
	  default : break;
  }
}






















