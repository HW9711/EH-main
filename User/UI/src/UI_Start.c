//UI_Start.c

#include "UI_Start.h"
#include "UI_Password.h"
#include "UI_FootPedalCalibration.h"
#include "UI_ModelConfiguration.h"
#include "iwdg.h"
#include "delay.h"
#include "data.h"
#include "screenkey.h"
#include "lcd.h"
#include "screen.h"

#include <stdint.h>

//============================================================================
// 函数名称: UI_Start_Fun()
// 功能描述: 起始页
// 输　  入:
// 输    出: 0返回主界面 1下一页
// 函数说明: 开机扫描”LOGO点击动作“ 进入厂家配置 4s等待...
//============================================================================
void UI_Start_Fun(void)
{
  uint8_t TimeCnt = 0;
  uint16_t DelayTime = 2000;

  while (DelayTime--)
  {
    Delay_ms(2);

    //按键扫描
	  //....................
    ScreenKey_Scan();

	  if(++TimeCnt < 20)
	    continue ;

	  TimeCnt = 0;

	  Iwdg_Reset();
 
 	  if (SysRunData.KeyValue == KEY_CONTINUOUSCLICK)
	  {
	    SysRunData.BeepTimeMS = 100;

			SysRunData.KeyValue = KEY_NONE;

			LCD_Show_Which_Map(3); 
			LCD_Show_Which_Map(0); 
			LCD_Show_Which_Map(3); 
		
      Delay_ms(5);

			UI_FootPedalCalibration_Fun();
			
		}
  }
}



//============================================================================
// 函数名称: UI_Show_init()
// 功能描述: UI主界面初始化
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void UI_Show_init(void)
{
  uint8_t temp[5] = { 0 };

  //先复位所有参数
  Screen_InformationBarImage_Update(2, 4, 2, 4);  //	LCD_Show_Info(2,2,2);
  Screen_IntegratedCutterPic_Update(0);  //	LCD_Show_Cutter(0);    //刀具连接图片  30ms
  Screen_HandleConnectState_Update(temp, temp);  //  LCD_Show_Handle_Connect(0,0,0);  50ms
  Screen_FootPedalConnectState_Update(1);  //  LCD_Show_FootPedal(1); //脚踏连接图片

  Screen_ElectricalMachineryDirectionState_Update2(0);// 运行模式切换 单向，往复
  Screen_TipInfo_Update(0);  // 	LCD_Show_Error(0);     //报警显示  5ms

//  LCD_Show_Which_Map(2); //运行界面
	LCD_Disappear_Picture(0x1410);
	LCD_Disappear_Picture(0x1411);	
	LCD_Disappear_Picture(0x1412);	
//	LCD_Disappear_Picture(0x1413);	
	LCD_Disappear_Picture(0x1414);
	LCD_Disappear_Picture(0x1415);	
	
}





