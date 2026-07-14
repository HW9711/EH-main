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
/*
 * 函数功能：循环处理脚踏定标页按键，分时读取单双脚踏标定值并刷新屏幕。
 * 输入参数：无。
 * 返回参数：接口保留 uint8_t；当前实现持续停留在定标循环中，不主动返回。
 */
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
				m_k_v>0?m_k_v--:m_k_v++; /* 中键事件只翻转定标页测试指示值，后续周期写到屏幕确认按键链路。 */
				break;
			case L_KEY_FOOT:
					l_k_v>0?l_k_v--:l_k_v++; /* 左键事件翻转左侧测试指示值，不修改脚踏标定参数。 */
				break;
			case R_KEY_FOOT:
					r_k_v>0?r_k_v--:r_k_v++; /* 右键事件翻转右侧测试指示值，用于现场确认三键输入。 */
				break;
      case KEY_STORAGEMIN :   //最低值
      {
        SendKeyBeepMessage(1U);
			  if (PedalCalibrationData.FootPedalType == 1) /* 类型 1 为双踏板，主定标键保存左侧最低值。 */
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
				if (PedalCalibrationData.FootPedalType == 1) /* 双踏板主定标键保存左侧最高值，右侧由带 2 的按键单独保存。 */
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
						if (PedalCalibrationData.FootPedalType == 1){ /* 双踏板主定标键保存左侧中间值，保持左右定标入口分离。 */
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
		if(PedalCalibrationData.FootPedalType == 1) /* 双踏板需要依次读取左右各三组值，命令必须错开发送以免 UART4 应答重叠。 */
		{
//				//刷新UI
			read_times++;
			if(read_times==30) /* 第一时点读取默认/右侧最低值，给前一轮屏幕写入留出串口间隔。 */
			{
				  Pedal_ReadLValue();
			}
			else if(read_times==60) /* 第二时点读取默认/右侧最高值，避免连续查询挤占 UART4 接收。 */
			{
					Pedal_ReadHValue();
			}
			else if(read_times==110) /* 第三时点读取默认/右侧中间值，完成右侧三点标定快照。 */
			{
						Pedal_ReadMValue();
			}
			else if(read_times==140) /* 第四时点切到左侧中间值查询，左右应答保持分时。 */
			{
							Pedal_ReadMValue_Left();
			}
				else if(read_times==170) /* 第五时点读取左侧最低值，避免与中间值应答粘连。 */
			{
					Pedal_ReadLValue_Left();
			}
				else if(read_times==200) /* 第六时点读取左侧最高值，至此左右六个定标值均已请求。 */
			{
							Pedal_ReadHValue_Left();
				
			}
			else if(read_times==230) /* 等全部 UART4 应答写入缓存后统一刷新六个存储值，并开始下一轮。 */
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
		else /* 类型 0 为单踏板，只读取一套最低、最高和中间值。 */
		{
		read_times++;
			if(read_times==30) /* 单踏板第一时点读取最低值，分时发送避免应答冲突。 */
			{
				  Pedal_ReadLValue();
			}
			else if(read_times==60) /* 单踏板第二时点读取最高值。 */
			{
					Pedal_ReadHValue();
			}
			else if(read_times==90) /* 单踏板第三时点读取中间值，完成一轮三点查询。 */
			{
					Pedal_ReadMValue();
			}
			else if(read_times==120) /* 等三次应答完成后统一刷新屏幕存储值并重启计数。 */
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
		if(write_foot_key_time%30==0){ /* 每 30 个定标循环刷新一次三键测试值，降低 UART6 屏幕刷新占用。 */
		  LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_LOW_KEY_VALUE, l_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_MID_KEY_VALUE, m_k_v);
			Delay_ms(2);
			LCD_Show_4byte_Number(UIDP_LCD_VP_PEDAL_HIGH_KEY_VALUE, r_k_v);
		  Delay_ms(2);
		}
  }
}






