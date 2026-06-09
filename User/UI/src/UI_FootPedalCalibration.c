//UI_FootPedalCalibration.c

#include "UI_FootPedalCalibration.h"
#include "lcd.h"
#include "delay.h"
#include "pedal.h"
#include "flash.h"
#include "iwdg.h"
#include "screenkey.h"
#include "sscBEEP.h"
#include "screen_address.h"

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
	uint8_t key_value = KEY_NONE;
//  LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LOW_KEY_VALUE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_MID_KEY_VALUE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_HIGH_KEY_VALUE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_AD_VALUE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_LOW_STORE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_HIGH_STORE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_AD_VALUE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_LOW_STORE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_HIGH_STORE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_MID_STORE, 0);
//	LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_MID_STORE, 0);
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

    // 定标页只消费屏幕/脚踏桥接过来的一次性事件，避免继续把按键暂存在旧全局状态。
    key_value = ScreenKey_LegacyEventTake();
    switch (key_value)
    {
			case M_KEY_FOOT:
				m_k_v>0?m_k_v--:m_k_v++;
				break;
			case L_KEY_FOOT:
					l_k_v>0?l_k_v--:l_k_v++;
				break;
			case R_KEY_FOOT:
					r_k_v>0?r_k_v--:r_k_v++;
				break;
      case KEY_STORAGEMIN :   //最低值
      {
        SendKeyBeepMessage(1U);
			  if (PedalCalibrationData.FootPedalType == 1)
				{
					 Pedal_StorageLValue_Left();
				}
        else
        {
					 Pedal_StorageLValue();				
				}

		    Delay_ms(5);
	    }
	    break;
	    case KEY_STORAGEMAX :  //最高值
	    {
		    SendKeyBeepMessage(1U);
				if (PedalCalibrationData.FootPedalType == 1)
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
		    SendKeyBeepMessage(1U);
						if (PedalCalibrationData.FootPedalType == 1){
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
			
					SendKeyBeepMessage(1U);
					Pedal_StorageMValue();
				
					Delay_ms(5);
	    }
	    break;				
	    case KEY_STORAGEMIN2 :  //存储最小值2 右
	    {
				
					SendKeyBeepMessage(1U);
					Pedal_StorageLValue();
					Delay_ms(5);				
				
	    }
	    break;
	    case KEY_STORAGEMAX2 :  //存储最大值2 右
	    {
						
					SendKeyBeepMessage(1U);
					 
					Pedal_StorageHValue();
					Delay_ms(5);
				
	    }
	    break;			
		default : break;
	  }
		if(PedalCalibrationData.FootPedalType == 1)
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
				
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_LOW_STORE, PedalCalibrationData.FootPedalMemoryLValue_Left);
				Delay_ms(2);
			
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_HIGH_STORE, PedalCalibrationData.FootPedalMemoryHValue_Left);
				Delay_ms(2);
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_MID_STORE, PedalCalibrationData.FootPedalMemoryMValue_Left);
				Delay_ms(2);
				
				
				
				
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_LOW_STORE, PedalCalibrationData.FootPedalMemoryLValue_Right);
				Delay_ms(2);
			
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_HIGH_STORE, PedalCalibrationData.FootPedalMemoryHValue_Right);
				Delay_ms(2);
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_MID_STORE, PedalCalibrationData.FootPedalMemoryMValue_Right);
				Delay_ms(2);
			}
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_AD_VALUE, PedalCalibrationData.FootPedalADValue);
				Delay_ms(2);
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_RIGHT_AD_VALUE, PedalCalibrationData.FootPedalADValue_Right);
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
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_LOW_STORE, PedalCalibrationData.FootPedalMemoryLValue);
				Delay_ms(2);
			LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_HIGH_STORE, PedalCalibrationData.FootPedalMemoryHValue);
				Delay_ms(2);
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_MID_STORE, PedalCalibrationData.FootPedalMemoryMValue_Right);
				Delay_ms(2);
			}
				LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LEFT_AD_VALUE, PedalCalibrationData.FootPedalADValue);
				Delay_ms(2);
		}

		write_foot_key_time++;
		if(write_foot_key_time%30==0){
		  LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LOW_KEY_VALUE, l_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_MID_KEY_VALUE, m_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_HIGH_KEY_VALUE, r_k_v);
		  Delay_ms(2);
		}
  }
}






