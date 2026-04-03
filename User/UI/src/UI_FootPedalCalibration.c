//UI_FootPedalCalibration.c

#include "UI_FootPedalCalibration.h"
#include "lcd.h"
#include "delay.h"
#include "pedal.h"
#include "data.h"
#include "flash.h"
#include "iwdg.h"
#include "screenkey.h"
#include "screen.h"

//============================================================================
// 函数名称: UI_FootPedalCalibration_Fun()
// 功能描述: 1_脚踏定标
// 输　  入:
// 输    出: 0返回主界面 1上一页
// 函数说明:
//============================================================================
uint8_t UI_FootPedalCalibration_Fun(void)
{
	static uint8_t l_k_v=0;
	static uint8_t m_k_v=0;
	static uint8_t r_k_v=0;
	static uint16_t read_times=0;
	static uint8_t write_foot_key_time=0;
  uint8_t ReadHLValueFlag = 0;
  uint8_t temp = 0, TimeCnt = 0;
	uint8_t KEY1Flag = 0,KEY2Flag = 0,KEY3Flag = 0 ;
//  LCD_Show_4byte_Number(0x3700, 0);
//	LCD_Show_4byte_Number(0x3710, 0);
//	LCD_Show_4byte_Number(0x3720, 0);
//	LCD_Show_4byte_Number(0x3730, 0);
//	LCD_Show_4byte_Number(0x3740, 0);
//	LCD_Show_4byte_Number(0x3750, 0);
//	LCD_Show_4byte_Number(0x3760, 0);
//	LCD_Show_4byte_Number(0x3770, 0);
//	LCD_Show_4byte_Number(0x3780, 0);
//	LCD_Show_4byte_Number(0x3790, 0);
//	LCD_Show_4byte_Number(0x37A0, 0);
  while (1)
  {
    Delay_ms(2);

    //1.扫描按键
    //..................
    ScreenKey_Scan();

    //2.扫描脚踏
    //..................
    PedalRecv_Scan();
		

    Iwdg_Reset();  //喂狗

   
    switch (SysRunData.KeyValue)
    {
			case M_KEY_FOOT:
				    SysRunData.KeyValue = KEY_NONE;
				m_k_v>0?m_k_v--:m_k_v++;
				break;
			case L_KEY_FOOT:
				    SysRunData.KeyValue = KEY_NONE;
					l_k_v>0?l_k_v--:l_k_v++;
				break;
			case R_KEY_FOOT:
				    SysRunData.KeyValue = KEY_NONE;
					r_k_v>0?r_k_v--:r_k_v++;
				break;
      case KEY_STORAGEMIN :   //最低值
      {
        SysRunData.BeepTimeMS = 100;
		    SysRunData.KeyValue = KEY_NONE;
			  if (Workvalue_s.Foot_type == 2)
				{
					 Pedal_StorageLValue_Left();
				}
        else if(Workvalue_s.Foot_type == 1)
        {
					 Pedal_StorageLValue();				
				}

		    Delay_ms(5);
	    }
	    break;
	    case KEY_STORAGEMAX :  //最高值
	    {
		    SysRunData.BeepTimeMS = 100;
		    SysRunData.KeyValue = KEY_NONE;
				if (Workvalue_s.Foot_type == 2)
				{
					Pedal_StorageHValue_Left();
				}
				else
				{		
										
					Pedal_StorageHValue();
				}
				Delay_ms(5);
	    }
	    break;
	    case KEY_STORAMEDIAN :  //中间值
	    {
		    SysRunData.BeepTimeMS = 100;
		    SysRunData.KeyValue = KEY_NONE;
						if (Workvalue_s.Foot_type == 2){
							Pedal_StorageMValue_Left();
						}
						else
						{
							Pedal_StorageMValue();
						}
			  
		    Delay_ms(5);
	    }
	    break;			
	    case KEY_STORAMEDIAN2 :  //中间值2
	    {
			
					SysRunData.BeepTimeMS = 100;
					SysRunData.KeyValue = KEY_NONE;
					Pedal_StorageMValue();
				
					Delay_ms(5);
	    }
	    break;				
	    case KEY_STORAGEMIN2 :  //存储最小值2 右
	    {
				
					SysRunData.BeepTimeMS = 100;
					SysRunData.KeyValue = KEY_NONE;
					Pedal_StorageLValue();
					Delay_ms(5);				
				
	    }
	    break;
	    case KEY_STORAGEMAX2 :  //存储最大值2 右
	    {
						
					SysRunData.BeepTimeMS = 100;

					SysRunData.KeyValue = KEY_NONE;
					 
					Pedal_StorageHValue();
					Delay_ms(5);
				
	    }
	    break;			
		default : break;
	  }
		if(Workvalue_s.Foot_type==2)
		{
//				//刷新UI
			read_times++;
			if(read_times==30)
			{
				  Pedal_ReadLValue();
			}
			else if(read_times==60)
			{
					Pedal_ReadHValue();
			}
			else if(read_times==110)
			{
						Pedal_ReadMValue();
			}
			else if(read_times==140)
			{
							Pedal_ReadMValue_Left();
			}
				else if(read_times==170)
			{
					Pedal_ReadLValue_Left();
			}
				else if(read_times==200)
			{
							Pedal_ReadHValue_Left();
				
			}
			else if(read_times==230)
			{
				read_times=0;
				
				LCD_Show_4byte_Number(0x3740, SysFootPedalData.FootPedalMemoryLValue_Left);
				Delay_ms(2);
			
				LCD_Show_4byte_Number(0x3750, SysFootPedalData.FootPedalMemoryHValue_Left);
				Delay_ms(2);
				LCD_Show_4byte_Number(0x3790, SysFootPedalData.FootPedalMemoryMValue_Left);		
				Delay_ms(2);
				
				
				
				
				LCD_Show_4byte_Number(0x3770, SysFootPedalData.FootPedalMemoryLValue_Right);
				Delay_ms(2);
			
				LCD_Show_4byte_Number(0x3780, SysFootPedalData.FootPedalMemoryHValue_Right);
				Delay_ms(2);
				LCD_Show_4byte_Number(0x37A0, SysFootPedalData.FootPedalMemoryMValue_Right);		
				Delay_ms(2);
			}
				LCD_Show_4byte_Number(0x3730, SysFootPedalData.FootPedalADValue);
				Delay_ms(2);
				LCD_Show_4byte_Number(0x3760, SysFootPedalData.FootPedalADValue_Right);
				Delay_ms(2);
		}
		else
		{
		read_times++;
			if(read_times==30)
			{
				  Pedal_ReadLValue();
			}
			else if(read_times==60)
			{
					Pedal_ReadHValue();
			}
			else if(read_times==90)
			{
					Pedal_ReadMValue();
			}
			else if(read_times==120)
			{
				read_times=0;
				LCD_Show_4byte_Number(0x3740, SysFootPedalData.FootPedalMemoryLValue);
				Delay_ms(2);
			LCD_Show_4byte_Number(0x3750, SysFootPedalData.FootPedalMemoryHValue);
				Delay_ms(2);
				LCD_Show_4byte_Number(0x3790, SysFootPedalData.FootPedalMemoryMValue_Right);		
				Delay_ms(2);
			}
				LCD_Show_4byte_Number(0x3730, SysFootPedalData.FootPedalADValue);
				Delay_ms(2);
		}

		write_foot_key_time++;
		if(write_foot_key_time%30==0){
		  LCD_Show_4byte_Number(0x3700, l_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(0x3710, m_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(0x3720, r_k_v);		
		  Delay_ms(2);
		}
  }
}







